#include "astromesh/net/multihop_router.hpp"

#include <algorithm>

namespace astromesh::net {

MultihopRouter::MultihopRouter(uint16_t local_rover_id, RoutingTable& routing_table)
    : m_local_rover_id(local_rover_id),
      m_routing_table(routing_table) {}

bool MultihopRouter::originate_packet(
    uint16_t final_target_id,
    uint64_t message_id,
    std::vector<uint8_t> payload,
    uint8_t flags,
    uint8_t max_hops,
    uint64_t now_ns) {

    if (final_target_id == m_local_rover_id) {
        MultihopEnvelope local_env;
        local_env.version = MultihopEnvelope::CURRENT_VERSION;
        local_env.flags = flags;
        local_env.origin_rover_id = m_local_rover_id;
        local_env.final_target_id = m_local_rover_id;
        local_env.hop_count = 0;
        local_env.max_hops = 1;
        local_env.message_id = message_id;
        local_env.path_length = 1;
        local_env.visited_nodes = {m_local_rover_id, 0, 0, 0, 0, 0, 0, 0};
        local_env.payload = std::move(payload);

        m_stats.packets_received++;
        m_stats.packets_delivered_locally++;
        if (m_local_cb) {
            m_local_cb(local_env);
        }
        return true;
    }

    MultihopEnvelope env;
    env.version = MultihopEnvelope::CURRENT_VERSION;
    env.flags = flags;
    env.origin_rover_id = m_local_rover_id;
    env.final_target_id = final_target_id;
    env.hop_count = 0;
    env.max_hops = (max_hops == 0) ? 5 : std::min(max_hops, MultihopEnvelope::MAX_HOPS_LIMIT);
    env.message_id = message_id;
    env.path_length = 1;
    env.visited_nodes.fill(0);
    env.visited_nodes[0] = m_local_rover_id;
    env.payload = std::move(payload);

    return forward_or_buffer(env, now_ns);
}

bool MultihopRouter::process_incoming_raw(
    const uint8_t* data,
    size_t size,
    uint16_t immediate_sender_id,
    uint64_t now_ns) {

    auto env_opt = MultihopEnvelope::deserialize(data, size);
    if (!env_opt.has_value()) {
        m_stats.malformed_dropped++;
        return false;
    }

    return process_incoming_envelope(std::move(*env_opt), immediate_sender_id, now_ns);
}

bool MultihopRouter::process_incoming_envelope(
    MultihopEnvelope env,
    uint16_t immediate_sender_id,
    uint64_t now_ns) {

    m_stats.packets_received++;

    // 1. Loop Suppression: Has this rover already handled/visited this frame?
    if (env.is_loop_detected(m_local_rover_id)) {
        m_stats.routing_loops_suppressed++;
        return false;
    }

    // 2. Local Terminal Delivery
    const bool is_for_me = (env.final_target_id == m_local_rover_id);
    const bool is_broadcast = (env.final_target_id == 0);

    if (is_for_me || is_broadcast) {
        m_stats.packets_delivered_locally++;
        if (m_local_cb) {
            m_local_cb(env);
        }

        // Generate custody ACK if requested
        if ((env.flags & MultihopFlags::CUSTODY_ACK_REQUESTED) && immediate_sender_id > 0) {
            if (m_custody_ack_cb) {
                m_custody_ack_cb(immediate_sender_id, env.message_id);
            }
        }

        if (is_for_me) {
            return true; // Terminal unicast delivery complete
        }
    }

    // 3. TTL Expiration Boundary Check for Forwarding
    if (env.is_expired()) {
        m_stats.ttl_expired_dropped++;
        return false;
    }

    // 4. Update Hop Count and Visited Path for Transit Relay
    env.hop_count++;
    if (!env.append_visited(m_local_rover_id)) {
        m_stats.routing_loops_suppressed++;
        return false;
    }

    // Grant hop custody to upstream relay since we have safely accepted the transit message
    if ((env.flags & MultihopFlags::CUSTODY_ACK_REQUESTED) && immediate_sender_id > 0) {
        if (m_custody_ack_cb) {
            m_custody_ack_cb(immediate_sender_id, env.message_id);
        }
    }

    // 5. Forward or Buffer
    return forward_or_buffer(env, now_ns);
}

bool MultihopRouter::forward_or_buffer(MultihopEnvelope env, uint64_t now_ns) {
    if (env.final_target_id == 0) {
        // Broadcast forward to all direct reachable neighbors except where it came from
        if (m_forward_cb) {
            bool forwarded = m_forward_cb(env, 0); // 0 = broadcast next hop
            if (forwarded) {
                m_stats.packets_forwarded++;
                return true;
            }
        }
        m_stats.unroutable_dropped++;
        return false;
    }

    auto next_hop_opt = m_routing_table.lookup_next_hop(env.final_target_id, now_ns);
    if (next_hop_opt.has_value()) {
        uint16_t next_hop = *next_hop_opt;
        // Don't forward back to a node that was already visited in the path
        if (!env.is_loop_detected(next_hop)) {
            if (m_forward_cb && m_forward_cb(env, next_hop)) {
                m_stats.packets_forwarded++;
                return true;
            }
        }
    }

    // Next hop unavailable or forward failed: buffer for DTN store-and-forward transit
    if (m_transit_buffer.size() < MAX_TRANSIT_BUFFER_SIZE) {
        TransitPacket pkt;
        pkt.envelope = std::move(env);
        pkt.enqueue_timestamp_ns = now_ns;
        pkt.expiry_timestamp_ns = now_ns + DEFAULT_TRANSIT_TTL_NS;
        m_transit_buffer.push_back(std::move(pkt));
        m_stats.transit_buffered++;
        return true;
    }

    m_stats.unroutable_dropped++;
    return false;
}

size_t MultihopRouter::drain_transit_buffer(uint64_t now_ns) {
    size_t drained = 0;
    auto it = m_transit_buffer.begin();

    while (it != m_transit_buffer.end()) {
        // Prune expired transit packets
        if (now_ns > 0 && it->expiry_timestamp_ns > 0 && now_ns >= it->expiry_timestamp_ns) {
            m_stats.ttl_expired_dropped++;
            it = m_transit_buffer.erase(it);
            continue;
        }

        auto next_hop_opt = m_routing_table.lookup_next_hop(it->envelope.final_target_id, now_ns);
        if (next_hop_opt.has_value() && !it->envelope.is_loop_detected(*next_hop_opt)) {
            uint16_t next_hop = *next_hop_opt;
            if (m_forward_cb && m_forward_cb(it->envelope, next_hop)) {
                m_stats.packets_forwarded++;
                m_stats.transit_drained++;
                drained++;
                it = m_transit_buffer.erase(it);
                continue;
            }
        }

        ++it;
    }

    return drained;
}

} // namespace astromesh::net
