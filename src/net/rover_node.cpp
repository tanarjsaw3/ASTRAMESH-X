#include "astromesh/net/rover_node.hpp"

#include <cstring>
#include <iostream>

namespace astromesh::net {

RoverNode::RoverNode(const RoverNodeConfig& config)
    : m_config(config),
      m_dtn_buffer(config.dtn_config),
      m_impairment(config.impairment_config),
      m_running(false),
      m_routing_table(config.rover_id),
      m_multihop_router(config.rover_id, m_routing_table) {

    // Wire up MultihopRouter callbacks
    m_multihop_router.set_forward_callback([this](const MultihopEnvelope& env, uint16_t next_hop) -> bool {
        return forward_multihop_frame(env, next_hop, get_now_ns());
    });

    m_multihop_router.set_local_delivery_callback([this](const MultihopEnvelope& env) {
        handle_multihop_local_delivery(env, get_now_ns());
    });

    m_multihop_router.set_custody_ack_callback([this](uint16_t upstream, uint64_t message_id) {
        send_custody_ack(upstream, message_id, get_now_ns());
    });
}

RoverNode::~RoverNode() {
    stop();
}

uint64_t RoverNode::get_now_ns() const noexcept {
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

bool RoverNode::start() {
    if (m_running) return true;

    if (!m_socket.open()) {
        std::cerr << "[RoverNode " << m_config.rover_id << "] Failed to open UDP socket.\n";
        return false;
    }

    if (!m_socket.set_nonblocking(true)) {
        std::cerr << "[RoverNode " << m_config.rover_id << "] Failed to set non-blocking mode.\n";
        m_socket.close();
        return false;
    }

    if (!m_socket.bind(m_config.listen_port, m_config.listen_ip)) {
        std::cerr << "[RoverNode " << m_config.rover_id << "] Failed to bind to "
                  << m_config.listen_ip << ":" << m_config.listen_port << "\n";
        m_socket.close();
        return false;
    }

    // Register initial seed peers
    for (size_t i = 0; i < m_config.seed_peers.size(); ++i) {
        m_peers.register_peer(0, m_config.seed_peers[i]);
    }

    m_running = true;
    m_last_heartbeat_ns = 0;
    return true;
}

void RoverNode::stop() {
    if (m_running) {
        m_socket.close();
        m_running = false;
    }
}

int64_t RoverNode::transmit_raw_frame(const MessageFrame& frame, const Endpoint& dest, uint64_t now_ns) {
    if (!dest.is_valid()) return -1;
    auto bytes = frame.serialize();

    bool out_immediate = false;
    bool scheduled = m_impairment.process_outgoing(
        bytes.data(), bytes.size(),
        dest,
        frame.sender_rover_id,
        frame.target_rover_id,
        now_ns,
        out_immediate);

    if (!scheduled) {
        // Dropped by impairment (loss or blackout)
        return 0;
    }

    if (out_immediate) {
        int64_t sent = m_socket.send_to(bytes.data(), bytes.size(), dest);
        if (sent > 0) {
            m_stats.packets_sent++;
            m_stats.bytes_sent += sent;
        }
        return sent;
    }

    // Stored in impairment delay queue
    return static_cast<int64_t>(bytes.size());
}

void RoverNode::send_heartbeats(uint64_t now_ns) {
    MessageFrame hb;
    hb.schema_version = MessageFrame::CURRENT_SCHEMA_VERSION;
    hb.type = MessageType::Heartbeat;
    hb.sender_rover_id = m_config.rover_id;
    hb.target_rover_id = 0; // Broadcast
    hb.sequence = m_next_sequence++;
    hb.timestamp_ns = now_ns;

    // 1. Send to configured seed peers
    for (const auto& ep : m_config.seed_peers) {
        if (ep.is_valid()) {
            transmit_raw_frame(hb, ep, now_ns);
            m_stats.heartbeats_sent++;
        }
    }

    // 2. Send to dynamically discovered active peers
    const uint64_t timeout_ns = static_cast<uint64_t>(m_config.peer_timeout_ms) * 1'000'000ULL;
    auto active_ids = m_peers.get_active_peers(now_ns, timeout_ns);
    for (uint16_t peer_id : active_ids) {
        auto ep_opt = m_peers.get_peer_endpoint(peer_id);
        if (ep_opt.has_value()) {
            bool in_seed = false;
            for (const auto& sep : m_config.seed_peers) {
                if (sep == *ep_opt) { in_seed = true; break; }
            }
            if (!in_seed) {
                transmit_raw_frame(hb, *ep_opt, now_ns);
                m_stats.heartbeats_sent++;
            }
        }
    }
}

uint64_t RoverNode::enqueue_dtn_message(
    uint16_t target_rover_id,
    MessageType type,
    const std::vector<uint8_t>& payload,
    comms::PriorityLevel priority,
    uint64_t ttl_ns,
    uint64_t now_ns) {

    if (now_ns == 0) now_ns = get_now_ns();

    // Unique monotonic message ID across rovers
    const uint64_t msg_id = (static_cast<uint64_t>(m_config.rover_id) << 48) | (++m_next_dtn_msg_counter);

    // Pack 12-byte DTN envelope header: 4-byte MAGIC ('ADTN') + 8-byte msg_id followed by user payload
    constexpr uint32_t MAGIC_DTN = 0x4E544441;
    std::vector<uint8_t> envelope(12 + payload.size());
    std::memcpy(&envelope[0], &MAGIC_DTN, 4);
    for (int i = 0; i < 8; ++i) {
        envelope[4 + i] = static_cast<uint8_t>((msg_id >> (i * 8)) & 0xFF);
    }
    if (!payload.empty()) {
        std::memcpy(&envelope[12], payload.data(), payload.size());
    }

    comms::DtnMessage dtn_msg;
    dtn_msg.message_id = msg_id;
    dtn_msg.sender_rover_id = m_config.rover_id;
    dtn_msg.target_rover_id = target_rover_id;
    dtn_msg.message_type = static_cast<comms::DtnMessageType>(type);
    dtn_msg.priority = priority;
    dtn_msg.creation_time_ns = now_ns;
    dtn_msg.ttl_ns = ttl_ns;
    dtn_msg.payload = std::move(envelope);
    dtn_msg.sequence_number = m_next_sequence++;

    auto result = m_dtn_buffer.enqueue(dtn_msg, now_ns);
    if (result == comms::EnqueueResult::Success) {
        m_wire_stats.dtn_messages_enqueued++;
        return msg_id;
    }

    return 0;
}

void RoverNode::send_ack(uint16_t target_rover_id, uint64_t message_id, uint64_t now_ns) {
    comms::AckPayload ack;
    ack.protocol_version = comms::ACK_PROTOCOL_VERSION;
    ack.ack_sender_id = m_config.rover_id;
    ack.ack_target_id = target_rover_id;
    ack.acked_message_id = message_id;
    ack.ack_timestamp_ns = now_ns;

    auto ack_bytes = comms::serialize_ack_payload(ack);

    MessageFrame frame;
    frame.schema_version = MessageFrame::CURRENT_SCHEMA_VERSION;
    frame.type = MessageType::Ack;
    frame.sender_rover_id = m_config.rover_id;
    frame.target_rover_id = target_rover_id;
    frame.sequence = m_next_sequence++;
    frame.timestamp_ns = now_ns;
    frame.payload = std::move(ack_bytes);

    auto ep_opt = m_peers.get_peer_endpoint(target_rover_id);
    if (ep_opt.has_value()) {
        transmit_raw_frame(frame, *ep_opt, now_ns);

        const size_t ack_wire = frame.payload.size() + MessageFrame::HEADER_SIZE + m_config.transport_framing_bytes;
        m_wire_stats.acks_sent++;
        m_wire_stats.ack_wire_bytes_sent += ack_wire;
        m_wire_stats.total_wire_bytes_sent += ack_wire;
    }
}

void RoverNode::process_dtn_outgoing(uint64_t now_ns) {
    if (!m_config.enable_dtn) return;

    auto link_checker = [this, now_ns](uint16_t target) -> bool {
        if (target == 0) return true;
        if (m_impairment.is_link_connected(m_config.rover_id, target)) return true;
        if (m_config.enable_multihop) {
            auto next_hop = m_routing_table.lookup_next_hop(target, now_ns);
            if (next_hop.has_value()) {
                return m_impairment.is_link_connected(m_config.rover_id, *next_hop);
            }
        }
        return false;
    };

    while (auto msg_opt = m_dtn_buffer.dequeue_next_eligible(now_ns, link_checker)) {
        uint16_t target = msg_opt->target_rover_id;
        Endpoint dest_ep;
        uint16_t forward_target_id = target;

        auto direct_ep = m_peers.get_peer_endpoint(target);
        if (direct_ep.has_value() && m_impairment.is_link_connected(m_config.rover_id, target)) {
            dest_ep = *direct_ep;
        } else if (m_config.enable_multihop) {
            auto next_hop = m_routing_table.lookup_next_hop(target, now_ns);
            if (next_hop.has_value()) {
                forward_target_id = *next_hop;
                auto next_ep = m_peers.get_peer_endpoint(forward_target_id);
                if (next_ep.has_value()) {
                    dest_ep = *next_ep;
                }
            }
        }

        if (!dest_ep.is_valid() && !m_config.seed_peers.empty()) {
            dest_ep = m_config.seed_peers.front();
        }

        if (!dest_ep.is_valid()) {
            continue;
        }

        MessageFrame frame;
        frame.schema_version = MessageFrame::CURRENT_SCHEMA_VERSION;
        frame.sender_rover_id = m_config.rover_id;
        frame.timestamp_ns = now_ns;

        if (forward_target_id == target) {
            // Direct transmission
            frame.type = static_cast<MessageType>(msg_opt->message_type);
            frame.target_rover_id = target;
            frame.sequence = msg_opt->sequence_number;
            frame.payload = msg_opt->payload;
        } else {
            // Multi-hop encapsulation
            MessageFrame inner_frame;
            inner_frame.schema_version = MessageFrame::CURRENT_SCHEMA_VERSION;
            inner_frame.type = static_cast<MessageType>(msg_opt->message_type);
            inner_frame.sender_rover_id = m_config.rover_id;
            inner_frame.target_rover_id = target;
            inner_frame.sequence = msg_opt->sequence_number;
            inner_frame.timestamp_ns = now_ns;

            // Strip the 12-byte DTN transport header if present
            if (msg_opt->payload.size() >= 12 &&
                msg_opt->payload[0] == 'A' && msg_opt->payload[1] == 'D' &&
                msg_opt->payload[2] == 'T' && msg_opt->payload[3] == 'N') {
                inner_frame.payload.assign(msg_opt->payload.begin() + 12, msg_opt->payload.end());
            } else {
                inner_frame.payload = msg_opt->payload;
            }

            MultihopEnvelope env;
            env.origin_rover_id = m_config.rover_id;
            env.final_target_id = target;
            env.hop_count = 0;
            env.max_hops = m_config.default_max_hops;
            env.message_id = msg_opt->message_id;
            env.flags = MultihopFlags::CUSTODY_ACK_REQUESTED | MultihopFlags::E2E_ACK_REQUESTED;
            env.path_length = 1;
            env.visited_nodes.fill(0);
            env.visited_nodes[0] = m_config.rover_id;
            env.payload = inner_frame.serialize();

            frame.type = MessageType::MultihopForward;
            frame.target_rover_id = forward_target_id;
            frame.sequence = m_next_sequence++;
            frame.payload = env.serialize();
        }

        transmit_raw_frame(frame, dest_ep, now_ns);

        const size_t raw_payload_bytes = (msg_opt->payload.size() >= 12) ? (msg_opt->payload.size() - 12) : msg_opt->payload.size();
        const size_t data_wire = raw_payload_bytes + MessageFrame::HEADER_SIZE + m_config.transport_framing_bytes;

        m_wire_stats.payload_bytes_sent += raw_payload_bytes;
        m_wire_stats.header_bytes_sent += MessageFrame::HEADER_SIZE;
        m_wire_stats.framing_bytes_sent += m_config.transport_framing_bytes;
        m_wire_stats.data_wire_bytes_sent += data_wire;
        m_wire_stats.total_wire_bytes_sent += data_wire;

        if (msg_opt->attempt_count > 1) {
            m_wire_stats.dtn_retransmissions++;
        }
    }
}

size_t RoverNode::poll_events(uint64_t now_ns) {
    if (!m_running) return 0;
    if (now_ns == 0) now_ns = get_now_ns();

    // 1. Flush any delayed packets that are ready for transmission
    auto ready_delayed = m_impairment.release_due_packets(now_ns);
    for (const auto& pkt : ready_delayed) {
        int64_t sent = m_socket.send_to(pkt.data.data(), pkt.data.size(), pkt.destination);
        if (sent > 0) {
            m_stats.packets_sent++;
            m_stats.bytes_sent += sent;
        }
    }

    size_t app_messages_processed = 0;
    uint8_t recv_buf[4096];
    Endpoint sender;

    // 2. Drain incoming socket buffer up to 100 packets per poll
    for (size_t p = 0; p < 100; ++p) {
        int64_t bytes = m_socket.recv_from(recv_buf, sizeof(recv_buf), sender);
        if (bytes <= 0) break; // No more datagrams available

        auto frame_opt = MessageFrame::deserialize(recv_buf, static_cast<size_t>(bytes));
        if (!frame_opt.has_value()) {
            m_stats.corrupted_dropped++;
            continue;
        }

        const auto& frame = *frame_opt;

        // Ignore self-echo loopback
        if (frame.sender_rover_id == m_config.rover_id) {
            continue;
        }

        // Check unicast target address
        if (frame.target_rover_id != 0 && frame.target_rover_id != m_config.rover_id) {
            continue;
        }

        // Check if frame carries DTN envelope ('ADTN')
        constexpr uint32_t MAGIC_DTN = 0x4E544441;
        bool is_dtn_frame = false;
        uint64_t dtn_msg_id = 0;
        if (frame.payload.size() >= 12) {
            uint32_t magic = 0;
            std::memcpy(&magic, frame.payload.data(), 4);
            if (magic == MAGIC_DTN) {
                is_dtn_frame = true;
                for (int i = 0; i < 8; ++i) {
                    dtn_msg_id |= (static_cast<uint64_t>(frame.payload[4 + i]) << (8 * i));
                }
            }
        }

        // Check duplicate sequence
        if (m_peers.is_duplicate(frame.sender_rover_id, frame.sequence)) {
            m_stats.duplicates_dropped++;
            if (frame.type == MessageType::Heartbeat) {
                m_peers.record_packet(frame.sender_rover_id, frame.sequence, sender, now_ns);
            } else if (is_dtn_frame) {
                // If sequence is duplicate, peer may have missed our ACK. Resend ACK!
                m_wire_stats.duplicate_deliveries_suppressed++;
                send_ack(frame.sender_rover_id, dtn_msg_id, now_ns);
            }
            continue;
        }

        // Record valid packet
        m_peers.record_packet(frame.sender_rover_id, frame.sequence, sender, now_ns);
        m_routing_table.install_direct_route(frame.sender_rover_id, now_ns);
        m_stats.packets_received++;
        m_stats.bytes_received += bytes;

        if (frame.type == MessageType::Heartbeat) {
            m_stats.heartbeats_received++;
        } else if (frame.type == MessageType::RoutingAdvertisement) {
            m_routing_table.apply_advertisement_payload(
                frame.sender_rover_id,
                frame.payload.data(),
                frame.payload.size(),
                now_ns);
            m_multihop_router.drain_transit_buffer(now_ns);
        } else if (frame.type == MessageType::MultihopForward ||
                   (frame.payload.size() >= MultihopEnvelope::HEADER_SIZE &&
                    frame.payload[0] == 'M' && frame.payload[1] == 'H' &&
                    frame.payload[2] == 'T' && frame.payload[3] == 'N')) {
            bool ok = m_multihop_router.process_incoming_raw(
                frame.payload.data(),
                frame.payload.size(),
                frame.sender_rover_id,
                now_ns);
            if (ok) {
                app_messages_processed++;
            }
        } else if (frame.type == MessageType::Ack) {
            auto ack_opt = comms::deserialize_ack_payload(frame.payload);
            if (ack_opt.has_value()) {
                auto res = m_dtn_buffer.acknowledge(ack_opt->acked_message_id, now_ns);
                if (res == comms::AckResult::Acknowledged) {
                    m_wire_stats.acks_received++;
                    m_wire_stats.ack_wire_bytes_received += frame.payload.size() + MessageFrame::HEADER_SIZE + m_config.transport_framing_bytes;
                    m_wire_stats.dtn_messages_delivered++;
                }
            }
        } else if (is_dtn_frame) {
            if (m_dtn_buffer.is_duplicate(dtn_msg_id, now_ns)) {
                m_wire_stats.duplicate_deliveries_suppressed++;
                send_ack(frame.sender_rover_id, dtn_msg_id, now_ns);
            } else {
                m_dtn_buffer.record_delivered_dedup(dtn_msg_id, now_ns);
                app_messages_processed++;

                if (m_message_handler) {
                    MessageFrame unpacked = frame;
                    unpacked.payload.assign(frame.payload.begin() + 12, frame.payload.end());
                    m_message_handler(unpacked, sender);
                }

                // Send immediate explicit acknowledgment
                send_ack(frame.sender_rover_id, dtn_msg_id, now_ns);
            }
        } else {
            // Raw non-DTN application message
            app_messages_processed++;
            if (m_message_handler) {
                m_message_handler(frame, sender);
            }
        }
    }

    // 3. Process outgoing DTN buffer queue & retries
    process_dtn_outgoing(now_ns);

    // 4. Check heartbeat timer
    const uint64_t hb_interval_ns = static_cast<uint64_t>(m_config.heartbeat_interval_ms) * 1'000'000ULL;
    if (now_ns >= m_last_heartbeat_ns + hb_interval_ns) {
        send_heartbeats(now_ns);
        m_last_heartbeat_ns = now_ns;
    }

    // 5. Check routing advertisement timer (Milestone 7.11)
    if (m_config.enable_multihop) {
        const uint64_t routing_adv_interval_ns = static_cast<uint64_t>(m_config.routing_advertisement_interval_ms) * 1'000'000ULL;
        if (now_ns >= m_last_routing_adv_ns + routing_adv_interval_ns) {
            send_routing_advertisements(now_ns);
            m_last_routing_adv_ns = now_ns;
        }

        // Periodic route pruning and transit buffer draining (every 1.5s)
        if (now_ns >= m_last_route_prune_ns + 1'500'000'000ULL) {
            m_routing_table.prune_stale_routes(now_ns);
            m_multihop_router.drain_transit_buffer(now_ns);
            m_last_route_prune_ns = now_ns;
        }
    }

    return app_messages_processed;
}

int64_t RoverNode::broadcast_message(MessageType type, const std::vector<uint8_t>& payload, uint64_t now_ns) {
    if (!m_running) return -1;
    if (now_ns == 0) now_ns = get_now_ns();

    MessageFrame frame;
    frame.schema_version = MessageFrame::CURRENT_SCHEMA_VERSION;
    frame.type = type;
    frame.sender_rover_id = m_config.rover_id;
    frame.target_rover_id = 0; // Broadcast
    frame.sequence = m_next_sequence++;
    frame.timestamp_ns = now_ns;
    frame.payload = payload;

    int64_t total_sent = 0;
    const uint64_t timeout_ns = static_cast<uint64_t>(m_config.peer_timeout_ms) * 1'000'000ULL;
    auto active_ids = m_peers.get_active_peers(now_ns, timeout_ns);

    if (active_ids.empty()) {
        for (const auto& ep : m_config.seed_peers) {
            if (ep.is_valid()) {
                int64_t s = transmit_raw_frame(frame, ep, now_ns);
                if (s > 0) total_sent += s;
            }
        }
    } else {
        for (uint16_t peer_id : active_ids) {
            auto ep_opt = m_peers.get_peer_endpoint(peer_id);
            if (ep_opt.has_value()) {
                int64_t s = transmit_raw_frame(frame, *ep_opt, now_ns);
                if (s > 0) total_sent += s;
            }
        }
    }

    return total_sent;
}

int64_t RoverNode::send_message_to(uint16_t target_rover_id, MessageType type,
                                   const std::vector<uint8_t>& payload, uint64_t now_ns) {
    if (!m_running) return -1;
    if (now_ns == 0) now_ns = get_now_ns();

    auto ep_opt = m_peers.get_peer_endpoint(target_rover_id);
    if (ep_opt.has_value() && m_impairment.is_link_connected(m_config.rover_id, target_rover_id)) {
        MessageFrame frame;
        frame.schema_version = MessageFrame::CURRENT_SCHEMA_VERSION;
        frame.type = type;
        frame.sender_rover_id = m_config.rover_id;
        frame.target_rover_id = target_rover_id;
        frame.sequence = m_next_sequence++;
        frame.timestamp_ns = now_ns;
        frame.payload = payload;

        return transmit_raw_frame(frame, *ep_opt, now_ns);
    }

    if (m_config.enable_multihop) {
        uint64_t msg_id = (static_cast<uint64_t>(m_config.rover_id) << 48) | (++m_next_dtn_msg_counter);
        std::vector<uint8_t> wire_bytes;
        if (type != MessageType::DataPayload) {
            MessageFrame inner_frame;
            inner_frame.schema_version = MessageFrame::CURRENT_SCHEMA_VERSION;
            inner_frame.type = type;
            inner_frame.sender_rover_id = m_config.rover_id;
            inner_frame.target_rover_id = target_rover_id;
            inner_frame.sequence = m_next_sequence++;
            inner_frame.timestamp_ns = now_ns;
            inner_frame.payload = payload;
            wire_bytes = inner_frame.serialize();
        } else {
            wire_bytes = payload;
        }

        bool ok = m_multihop_router.originate_packet(
            target_rover_id,
            msg_id,
            wire_bytes,
            MultihopFlags::E2E_ACK_REQUESTED,
            m_config.default_max_hops,
            now_ns);
        return ok ? static_cast<int64_t>(payload.size()) : -1;
    }

    return -1;
}

bool RoverNode::originate_multihop(uint16_t final_target_id, MessageType type,
                                   const std::vector<uint8_t>& payload, uint64_t now_ns) {
    if (!m_running) return false;
    if (now_ns == 0) now_ns = get_now_ns();

    uint64_t msg_id = (static_cast<uint64_t>(m_config.rover_id) << 48) | (++m_next_dtn_msg_counter);
    return m_multihop_router.originate_packet(
        final_target_id,
        msg_id,
        payload,
        MultihopFlags::CUSTODY_ACK_REQUESTED | MultihopFlags::E2E_ACK_REQUESTED,
        m_config.default_max_hops,
        now_ns);
}

void RoverNode::send_routing_advertisements(uint64_t now_ns) {
    m_routing_table.increment_self_sequence_number();
    const uint64_t timeout_ns = static_cast<uint64_t>(m_config.peer_timeout_ms) * 1'000'000ULL;
    auto active_peers = m_peers.get_active_peers(now_ns, timeout_ns);

    for (uint16_t peer_id : active_peers) {
        auto ep_opt = m_peers.get_peer_endpoint(peer_id);
        if (!ep_opt.has_value()) continue;

        auto adv_payload = m_routing_table.generate_advertisement_payload(peer_id);
        if (adv_payload.empty()) continue;

        MessageFrame frame;
        frame.schema_version = MessageFrame::CURRENT_SCHEMA_VERSION;
        frame.type = MessageType::RoutingAdvertisement;
        frame.sender_rover_id = m_config.rover_id;
        frame.target_rover_id = peer_id;
        frame.sequence = m_next_sequence++;
        frame.timestamp_ns = now_ns;
        frame.payload = std::move(adv_payload);

        transmit_raw_frame(frame, *ep_opt, now_ns);
    }
}

bool RoverNode::forward_multihop_frame(const MultihopEnvelope& env, uint16_t next_hop, uint64_t now_ns) {
    auto bytes = env.serialize();
    MessageFrame frame;
    frame.schema_version = MessageFrame::CURRENT_SCHEMA_VERSION;
    frame.type = MessageType::MultihopForward;
    frame.sender_rover_id = m_config.rover_id;
    frame.target_rover_id = next_hop;
    frame.sequence = m_next_sequence++;
    frame.timestamp_ns = now_ns;
    frame.payload = std::move(bytes);

    if (next_hop == 0) {
        for (const auto& ep : m_config.seed_peers) {
            transmit_raw_frame(frame, ep, now_ns);
        }
        return true;
    }

    auto ep_opt = m_peers.get_peer_endpoint(next_hop);
    if (ep_opt.has_value()) {
        int64_t sent = transmit_raw_frame(frame, *ep_opt, now_ns);
        return sent > 0;
    }

    if (!m_config.seed_peers.empty()) {
        int64_t sent = transmit_raw_frame(frame, m_config.seed_peers.front(), now_ns);
        return sent > 0;
    }

    return false;
}

void RoverNode::handle_multihop_local_delivery(const MultihopEnvelope& env, uint64_t now_ns) {
    if (env.flags & MultihopFlags::IS_E2E_ACK) {
        auto ack_opt = comms::deserialize_ack_payload(env.payload);
        if (ack_opt.has_value()) {
            auto res = m_dtn_buffer.acknowledge(ack_opt->acked_message_id, now_ns);
            if (res == comms::AckResult::Acknowledged) {
                m_wire_stats.acks_received++;
                m_wire_stats.dtn_messages_delivered++;
            }
        }
        return;
    }

    auto it = m_delivered_message_ids.find(env.message_id);
    if (it != m_delivered_message_ids.end()) {
        m_wire_stats.duplicate_deliveries_suppressed++;
        if (env.flags & MultihopFlags::E2E_ACK_REQUESTED) {
            send_e2e_ack(env.origin_rover_id, env.message_id, now_ns);
        }
        return;
    }

    m_delivered_message_ids[env.message_id] = now_ns;

    if (m_delivered_message_ids.size() > 500) {
        for (auto dit = m_delivered_message_ids.begin(); dit != m_delivered_message_ids.end();) {
            if (now_ns >= dit->second && (now_ns - dit->second) > 60'000'000'000ULL) {
                dit = m_delivered_message_ids.erase(dit);
            } else {
                ++dit;
            }
        }
    }

    if (m_message_handler) {
        MessageFrame unpacked;
        unpacked.schema_version = MessageFrame::CURRENT_SCHEMA_VERSION;
        unpacked.sender_rover_id = env.origin_rover_id;
        unpacked.target_rover_id = env.final_target_id;
        unpacked.sequence = static_cast<uint32_t>(env.message_id & 0xFFFFFFFF);
        unpacked.timestamp_ns = now_ns;
        unpacked.payload = env.payload;
        unpacked.type = MessageType::DataPayload;

        if (env.payload.size() >= MessageFrame::HEADER_SIZE) {
            auto sub_opt = MessageFrame::deserialize(env.payload.data(), env.payload.size());
            if (sub_opt.has_value()) {
                unpacked = *sub_opt;
            }
        }

        Endpoint ep("127.0.0.1", 0);
        auto ep_opt = m_peers.get_peer_endpoint(env.origin_rover_id);
        if (ep_opt.has_value()) ep = *ep_opt;
        m_message_handler(unpacked, ep);
    }

    if (env.flags & MultihopFlags::E2E_ACK_REQUESTED) {
        send_e2e_ack(env.origin_rover_id, env.message_id, now_ns);
    }
}

void RoverNode::send_custody_ack(uint16_t upstream_rover_id, uint64_t message_id, uint64_t now_ns) {
    comms::AckPayload ack;
    ack.protocol_version = comms::ACK_PROTOCOL_VERSION;
    ack.ack_sender_id = m_config.rover_id;
    ack.ack_target_id = upstream_rover_id;
    ack.acked_message_id = message_id;
    ack.ack_timestamp_ns = now_ns;

    auto ack_bytes = comms::serialize_ack_payload(ack);

    MessageFrame frame;
    frame.schema_version = MessageFrame::CURRENT_SCHEMA_VERSION;
    frame.type = MessageType::Ack;
    frame.sender_rover_id = m_config.rover_id;
    frame.target_rover_id = upstream_rover_id;
    frame.sequence = m_next_sequence++;
    frame.timestamp_ns = now_ns;
    frame.payload = std::move(ack_bytes);

    auto ep_opt = m_peers.get_peer_endpoint(upstream_rover_id);
    if (ep_opt.has_value()) {
        transmit_raw_frame(frame, *ep_opt, now_ns);
        m_wire_stats.acks_sent++;
    }
}

void RoverNode::send_e2e_ack(uint16_t origin_rover_id, uint64_t message_id, uint64_t now_ns) {
    comms::AckPayload ack;
    ack.protocol_version = comms::ACK_PROTOCOL_VERSION;
    ack.ack_sender_id = m_config.rover_id;
    ack.ack_target_id = origin_rover_id;
    ack.acked_message_id = message_id;
    ack.ack_timestamp_ns = now_ns;

    auto ack_bytes = comms::serialize_ack_payload(ack);

    m_multihop_router.originate_packet(
        origin_rover_id,
        (static_cast<uint64_t>(m_config.rover_id) << 48) | (++m_next_dtn_msg_counter),
        ack_bytes,
        MultihopFlags::IS_E2E_ACK,
        m_config.default_max_hops,
        now_ns);
    m_wire_stats.acks_sent++;
}

} // namespace astromesh::net
