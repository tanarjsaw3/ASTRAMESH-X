#include "astromesh/orchestration/multi_rover_collective.hpp"

#include <algorithm>
#include <cmath>

namespace astromesh::orchestration {

// ==============================================================================
// SimulatedRover Implementation
// ==============================================================================

SimulatedRover::SimulatedRover(const SimulatedRoverConfig& config)
    : config_(config),
      state_machine_(config.rover_id, config.state_machine_config),
      keyframe_pruner_(config.pruner_config),
      budget_manager_(config.budget_config),
      dtn_buffer_(config.dtn_queue_config, &budget_manager_),
      iblt_(config.iblt_cell_count, config.iblt_probes, 1),
      cbf_(config.cbf_capacity, 0.05),
      boundary_filter_(config.filter_options),
      current_pose_(config.initial_pose),
      last_retained_pose_(config.initial_pose) {

    // Register initial anchor node (keyframe 0)
    graph::NodeId node0(config_.rover_id, 0);
    graph::KeyframeNode kf0(node0, graph::Pose2D(config_.initial_pose.x(), config_.initial_pose.y(), config_.initial_pose.theta()), 0);
    pose_graph_.add_node(kf0);

    // Initial partition setup
    partition_manager_.rebuild(pose_graph_);

    // Initial transition to Exploring
    state_machine_.transition_to(
        RoverState::Exploring,
        TransitionReason::InitializationComplete,
        0,
        "Rover initialization and anchor keyframe setup complete");
}

bool SimulatedRover::add_odometry_step(
    const geometry::SE2& delta_pose,
    uint64_t timestamp_ns,
    const std::vector<core::LandmarkKey>& landmarks) {

    geometry::SE2 next_pose = current_pose_ * delta_pose;
    current_pose_ = next_pose;

    // Check keyframe admission criteria
    graph::Pose2D cand_p(next_pose.x(), next_pose.y(), next_pose.theta());
    graph::Pose2D last_p(last_retained_pose_.x(), last_retained_pose_.y(), last_retained_pose_.theta());

    bool should_retain = keyframe_pruner_.should_retain_keyframe(
        cand_p, timestamp_ns, last_p, last_keyframe_time_ns_);

    if (!should_retain) {
        return false;
    }

    // Add keyframe node
    current_keyframe_index_++;
    graph::NodeId new_node_id(config_.rover_id, current_keyframe_index_);
    graph::NodeId prev_node_id(config_.rover_id, current_keyframe_index_ - 1);

    graph::KeyframeNode kf_node(
        new_node_id,
        graph::Pose2D(next_pose.x(), next_pose.y(), next_pose.theta()),
        timestamp_ns);
    kf_node.observed_landmarks = landmarks;

    if (!pose_graph_.add_node(kf_node)) {
        return false;
    }

    // Add sequential odometry edge
    graph::InformationMatrix2D info = graph::InformationMatrix2D::diagonal(100.0, 1000.0);
    graph::PoseGraphEdge odom_edge(
        0,
        prev_node_id,
        new_node_id,
        graph::EdgeType::Odometry,
        graph::Pose2D(delta_pose.x(), delta_pose.y(), delta_pose.theta()),
        info,
        graph::EdgeVerificationStatus::Verified);

    pose_graph_.add_edge(odom_edge);

    // Update landmark discovery data structures
    for (const auto& lm : landmarks) {
        cbf_.insert(lm);
        iblt_.insert(lm);
    }

    last_retained_pose_ = next_pose;
    last_keyframe_time_ns_ = timestamp_ns;

    // Synchronize DSU partition
    partition_manager_.rebuild(pose_graph_);

    // Memory bounding check
    prune_keyframes_if_needed();

    return true;
}

bool SimulatedRover::add_candidate_loop_closure(
    const graph::NodeId& source,
    const graph::NodeId& target,
    const geometry::SE2& relative_pose,
    const graph::InformationMatrix2D& info) {

    graph::EdgeType type = (source.rover_id == target.rover_id)
        ? graph::EdgeType::IntraRoverLoopClosure
        : graph::EdgeType::InterRoverLoopClosure;

    graph::PoseGraphEdge candidate_edge(
        0,
        source,
        target,
        type,
        graph::Pose2D(relative_pose.x(), relative_pose.y(), relative_pose.theta()),
        info,
        graph::EdgeVerificationStatus::Unverified);

    return pose_graph_.add_edge(candidate_edge);
}

graph::PruningResult SimulatedRover::prune_keyframes_if_needed() {
    if (pose_graph_.num_nodes() > config_.pruner_config.max_retained_keyframes) {
        auto result = keyframe_pruner_.enforce_memory_bound(pose_graph_, &partition_manager_);
        if (result.status == graph::PruningStatus::SuccessBoundMet) {
            keyframe_prunes_applied += result.nodes_pruned;
        } else {
            keyframe_prunes_rejected++;
        }
        return result;
    }
    return {};
}

// ==============================================================================
// MultiRoverCollective Implementation
// ==============================================================================

MultiRoverCollective::MultiRoverCollective(const CollectiveSimulationConfig& config)
    : config_(config),
      network_(config.network_config),
      rng_(config.random_seed) {

    for (const auto& rcfg : config.rover_configs) {
        rovers_[rcfg.rover_id] = std::make_unique<SimulatedRover>(rcfg);
    }
}

SimulatedRover* MultiRoverCollective::get_rover(uint16_t rover_id) noexcept {
    auto it = rovers_.find(rover_id);
    if (it != rovers_.end()) {
        return it->second.get();
    }
    return nullptr;
}

const SimulatedRover* MultiRoverCollective::get_rover(uint16_t rover_id) const noexcept {
    auto it = rovers_.find(rover_id);
    if (it != rovers_.end()) {
        return it->second.get();
    }
    return nullptr;
}

void MultiRoverCollective::set_link_state(uint16_t rover_a, uint16_t rover_b, bool connected) noexcept {
    if (rover_a == rover_b) return;
    auto pair = std::make_pair(std::min(rover_a, rover_b), std::max(rover_a, rover_b));
    if (connected) {
        disconnected_links_.erase(pair);
    } else {
        disconnected_links_.insert(pair);
    }
    network_.set_link_state(rover_a, rover_b, connected);
}

bool MultiRoverCollective::is_link_connected(uint16_t rover_a, uint16_t rover_b) const noexcept {
    if (rover_a == rover_b) return true;
    auto pair = std::make_pair(std::min(rover_a, rover_b), std::max(rover_a, rover_b));
    return disconnected_links_.find(pair) == disconnected_links_.end();
}

void MultiRoverCollective::set_static_route(uint16_t source, uint16_t destination, uint16_t next_hop) noexcept {
    static_routes_[std::make_pair(source, destination)] = next_hop;
}

std::optional<uint16_t> MultiRoverCollective::find_next_hop(uint16_t current_rover, uint16_t target_rover) const {
    if (current_rover == target_rover) {
        return current_rover;
    }

    // Direct link check
    if (is_link_connected(current_rover, target_rover)) {
        return target_rover;
    }

    // Static route check
    auto s_it = static_routes_.find(std::make_pair(current_rover, target_rover));
    if (s_it != static_routes_.end()) {
        uint16_t candidate = s_it->second;
        if (is_link_connected(current_rover, candidate)) {
            return candidate;
        }
    }

    // If multi-hop routing is not enabled, no path beyond direct link
    if (!config_.enable_multihop_routing) {
        return std::nullopt;
    }

    // Shortest-path BFS over currently connected links
    std::map<uint16_t, uint16_t> parent;
    std::set<uint16_t> visited;
    std::vector<uint16_t> queue;

    visited.insert(current_rover);
    queue.push_back(current_rover);

    bool found = false;
    size_t head = 0;

    while (head < queue.size()) {
        uint16_t curr = queue[head++];
        if (curr == target_rover) {
            found = true;
            break;
        }

        for (const auto& [peer_id, peer_rover] : rovers_) {
            if (peer_id == curr) continue;
            if (visited.find(peer_id) != visited.end()) continue;

            if (is_link_connected(curr, peer_id)) {
                visited.insert(peer_id);
                parent[peer_id] = curr;
                queue.push_back(peer_id);
            }
        }
    }

    if (!found) {
        return std::nullopt;
    }

    // Backtrack from target to find the first hop after current_rover
    uint16_t hop = target_rover;
    while (parent.find(hop) != parent.end() && parent[hop] != current_rover) {
        hop = parent[hop];
    }

    return hop;
}

comms::EnqueueResult MultiRoverCollective::enqueue_message(
    uint16_t sender_rover_id,
    uint16_t target_rover_id,
    comms::DtnMessageType message_type,
    comms::PriorityLevel priority,
    const std::vector<uint8_t>& payload,
    uint64_t ttl_ns,
    uint32_t sequence_number,
    uint8_t max_hops) {

    total_messages_generated_++;

    auto* sender = get_rover(sender_rover_id);
    if (!sender) {
        total_messages_rejected_++;
        return comms::EnqueueResult::InvalidMetadata;
    }

    comms::DtnMessage msg;
    // Sequential unique message ID across the collective simulation
    msg.message_id = next_message_seq_++;
    msg.sender_rover_id = sender_rover_id;
    msg.target_rover_id = target_rover_id;
    msg.origin_rover_id = sender_rover_id;
    msg.next_hop_rover_id = 0;
    msg.hop_count = 0;
    msg.max_hops = (max_hops > 0) ? max_hops : config_.default_max_hops;
    msg.message_type = message_type;
    msg.priority = priority;
    msg.creation_time_ns = current_time_ns_;
    msg.enqueue_time_ns = current_time_ns_;
    msg.ttl_ns = ttl_ns;
    msg.expiry_time_ns = current_time_ns_ + ttl_ns;
    msg.payload = payload;
    msg.sequence_number = sequence_number;
    msg.attempt_count = 0;
    msg.next_retry_time_ns = current_time_ns_;

    auto result = sender->dtn_buffer().enqueue(msg, current_time_ns_);

    if (result == comms::EnqueueResult::Success) {
        total_messages_admitted_++;
    } else {
        total_messages_rejected_++;
        if (result == comms::EnqueueResult::CapacityCountExceeded ||
            result == comms::EnqueueResult::CapacityBytesExceeded ||
            result == comms::EnqueueResult::QuotaExceeded) {
            sender->state_machine().transition_to(
                RoverState::ResourceConstrained,
                (result == comms::EnqueueResult::QuotaExceeded) ?
                    TransitionReason::ByteQuotaExhausted :
                    TransitionReason::QueueCapacityExceeded,
                current_time_ns_,
                "DTN queue capacity or quota reached");
            if (result == comms::EnqueueResult::QuotaExceeded) {
                sender->mark_quota_denied(current_time_ns_);
            }
        }
    }

    return result;
}

void MultiRoverCollective::dispatch_outgoing_dtn_messages() {
    for (auto& [rover_id, rover] : rovers_) {
        // Purge expired messages first
        size_t expired = rover->dtn_buffer().purge_expired(current_time_ns_);
        total_messages_expired_ += expired;

        // Link checker determines whether next hop towards target is currently connected
        auto link_checker = [this, rover_id](uint16_t target_rover_id) -> bool {
            auto hop = find_next_hop(rover_id, target_rover_id);
            return hop.has_value() && is_link_connected(rover_id, *hop);
        };

        while (true) {
            std::optional<comms::DtnMessage> opt_msg;
            if (config_.enable_multihop_routing) {
                // If direct link is disconnected and no multihop path exists,
                // but direct link was configured, we only drop if multi-hop is NOT enabled.
                // When multihop is enabled, rover RETAINS message until contact!
                opt_msg = rover->dtn_buffer().dequeue_next_eligible(current_time_ns_, link_checker);
            } else {
                opt_msg = rover->dtn_buffer().dequeue_next_eligible(current_time_ns_);
            }
            if (!opt_msg.has_value()) {
                break;
            }

            auto msg = *opt_msg;
            auto opt_next_hop = find_next_hop(rover_id, msg.target_rover_id);
            uint16_t next_hop = opt_next_hop.has_value() ? *opt_next_hop : msg.target_rover_id;
            msg.next_hop_rover_id = next_hop;

            total_messages_sent_++;

            const bool is_retry = (msg.attempt_count > 1);
            if (!is_retry) {
                total_initial_attempts_++;
            } else {
                total_messages_retried_++;
                if (config_.enable_explicit_acks) {
                    retransmissions_missing_ack_++;
                }
            }

            const bool is_ack = (msg.message_type == comms::DtnMessageType::Acknowledgment);
            if (is_ack) {
                ack_transmission_attempts_++;
            } else {
                hop_data_attempts_++;
            }

            // Account for bytes: payload + 24B DtnMessage metadata + transport framing
            const size_t payload_bytes = msg.payload_bytes();
            constexpr size_t protocol_bytes = 24;
            const size_t framing_bytes = config_.network_config.transport_header_bytes;
            const size_t total_wire = payload_bytes + protocol_bytes + framing_bytes;

            // Check byte quota on sender for next hop
            auto quota_res = rover->budget_manager().check_quota(
                next_hop, payload_bytes, current_time_ns_, protocol_bytes, framing_bytes);

            if (quota_res != comms::QuotaCheckResult::Approved) {
                // Denied by quota: enter ResourceConstrained
                rover->state_machine().transition_to(
                    RoverState::ResourceConstrained,
                    TransitionReason::ByteQuotaExhausted,
                    current_time_ns_,
                    "Transmission blocked by epoch byte budget");
                rover->mark_quota_denied(current_time_ns_);
                total_messages_rejected_++;
                rover->dtn_buffer().mark_transmission_attempt(msg.message_id, current_time_ns_, false);
                continue;
            }

            rover->budget_manager().record_sent(
                next_hop, msg.priority, payload_bytes, current_time_ns_,
                is_retry, protocol_bytes, framing_bytes);

            total_payload_bytes_sent_ += payload_bytes;
            total_protocol_bytes_sent_ += protocol_bytes;
            total_framing_bytes_sent_ += framing_bytes;
            total_wire_bytes_sent_ += total_wire;

            if (is_ack) {
                ack_wire_bytes_sent_ += total_wire;
            }

            // Check network link partition on hop link
            if (!is_link_connected(rover_id, next_hop)) {
                total_messages_dropped_++;
                if (is_ack) {
                    ack_losses_++;
                } else {
                    hop_data_losses_++;
                }
                if (config_.enable_dtn_retries && !is_ack) {
                    rover->dtn_buffer().mark_transmission_attempt(msg.message_id, current_time_ns_, false);
                } else {
                    rover->dtn_buffer().acknowledge(msg.message_id, current_time_ns_);
                }
                continue;
            }

            // Check stochastic packet drop
            double loss_rate = config_.network_config.packet_loss_rate;
            if (is_ack && config_.ack_packet_loss_rate >= 0.0) {
                loss_rate = config_.ack_packet_loss_rate;
            }

            if (loss_rate > 0.0 && dist_unit_(rng_) < loss_rate) {
                total_messages_dropped_++;
                if (is_ack) {
                    ack_losses_++;
                } else {
                    hop_data_losses_++;
                }
                if (config_.enable_dtn_retries && !is_ack) {
                    rover->dtn_buffer().mark_transmission_attempt(msg.message_id, current_time_ns_, false);
                } else {
                    rover->dtn_buffer().acknowledge(msg.message_id, current_time_ns_);
                }
                continue;
            }

            // If explicit ACKs are disabled, or if this message is itself an ACK,
            // or if this rover is an intermediate forwarding hop (not the original creator),
            // remove it from the sender queue upon departure.
            // Only the original source rover keeps the message waiting for the end-to-end ACK!
            const bool is_intermediate_hop = (rover_id != msg.get_origin_id());
            if (!config_.enable_dtn_retries || !config_.enable_explicit_acks || is_ack || is_intermediate_hop) {
                rover->dtn_buffer().acknowledge(msg.message_id, current_time_ns_);
            }

            // Schedule discrete-event delivery
            uint64_t delay_ns = config_.network_config.latency_ns;
            if (config_.network_config.jitter_ns > 0) {
                delay_ns += static_cast<uint64_t>(dist_unit_(rng_) * static_cast<double>(config_.network_config.jitter_ns));
            }

            InTransitDtnMessage transit_item;
            transit_item.delivery_time_ns = current_time_ns_ + delay_ns;
            transit_item.transit_id = next_transit_id_++;
            transit_item.hop_sender_id = rover_id;
            transit_item.hop_target_id = next_hop;
            transit_item.message = msg;
            in_transit_messages_.push(transit_item);
        }
    }
}

void MultiRoverCollective::deliver_inbound_dtn_messages() {
    while (!in_transit_messages_.empty()) {
        const auto& top = in_transit_messages_.top();
        if (top.delivery_time_ns > current_time_ns_) {
            break;
        }

        InTransitDtnMessage item = top;
        in_transit_messages_.pop();

        const bool is_ack = (item.message.message_type == comms::DtnMessageType::Acknowledgment);

        // Check if destination link is still connected upon arrival
        if (!is_link_connected(item.hop_sender_id, item.hop_target_id)) {
            total_messages_dropped_++;
            if (is_ack) {
                ack_losses_++;
            } else {
                hop_data_losses_++;
            }
            if (config_.enable_dtn_retries && !is_ack) {
                auto* sender = get_rover(item.message.sender_rover_id);
                if (sender) {
                    sender->dtn_buffer().mark_transmission_attempt(item.message.message_id, current_time_ns_, false);
                }
            }
            continue;
        }

        auto* dest = get_rover(item.hop_target_id);
        if (!dest) {
            total_messages_dropped_++;
            continue;
        }

        total_messages_delivered_++;
        const size_t wire_bytes = item.message.payload_bytes() + 24 + config_.network_config.transport_header_bytes;
        total_wire_bytes_delivered_ += wire_bytes;

        // -------------------------------------------------------------
        // Case 1: Inbound message is an explicit Acknowledgment
        // -------------------------------------------------------------
        if (is_ack) {
            ack_deliveries_++;
            ack_wire_bytes_delivered_ += wire_bytes;

            auto ack_opt = comms::deserialize_ack_payload(item.message.payload);
            if (ack_opt.has_value()) {
                const auto& ack = *ack_opt;
                if (dest->rover_id() == ack.ack_target_id) {
                    // ACK reached original source! Clear message from DTN buffer
                    dest->dtn_buffer().acknowledge(ack.acked_message_id, current_time_ns_);
                } else if (config_.enable_multihop_routing) {
                    // Forward ACK along towards ack_target_id
                    auto opt_next = find_next_hop(dest->rover_id(), ack.ack_target_id);
                    if (opt_next.has_value()) {
                        comms::DtnMessage fwd_ack = item.message;
                        fwd_ack.message_id = next_message_seq_++;
                        fwd_ack.sender_rover_id = dest->rover_id();
                        fwd_ack.target_rover_id = ack.ack_target_id;
                        fwd_ack.origin_rover_id = ack.ack_sender_id;
                        fwd_ack.next_hop_rover_id = *opt_next;
                        fwd_ack.enqueue_time_ns = current_time_ns_;
                        fwd_ack.attempt_count = 0;
                        fwd_ack.next_retry_time_ns = current_time_ns_;
                        dest->dtn_buffer().enqueue(fwd_ack, current_time_ns_);
                    }
                }
            }
            continue;
        }

        // -------------------------------------------------------------
        // Case 2: Inbound message is a DATA message
        // -------------------------------------------------------------
        hop_data_delivered_++;

        const uint16_t origin_id = item.message.get_origin_id();
        const uint16_t target_id = item.message.target_rover_id;

        // Check if message reached final destination
        if (dest->rover_id() == target_id) {
            // Check destination duplicate detection
            bool is_dup = dest->dtn_buffer().is_duplicate(item.message.message_id, current_time_ns_);

            if (is_dup) {
                total_duplicate_deliveries_++;
            } else {
                dest->dtn_buffer().record_delivered_dedup(item.message.message_id, current_time_ns_);
                total_unique_delivered_++;
                unique_messages_processed_++;
                process_received_message(*dest, item.message);
            }

            // Return explicit Acknowledgment back to origin_id
            if (config_.enable_explicit_acks) {
                comms::AckPayload ack_data;
                ack_data.protocol_version = comms::ACK_PROTOCOL_VERSION;
                ack_data.ack_sender_id = dest->rover_id();
                ack_data.ack_target_id = origin_id;
                ack_data.acked_message_id = item.message.message_id;
                ack_data.ack_timestamp_ns = current_time_ns_;

                std::vector<uint8_t> ack_payload = comms::serialize_ack_payload(ack_data);

                // Enqueue ACK with high priority and 5-second TTL
                enqueue_message(
                    dest->rover_id(),
                    origin_id,
                    comms::DtnMessageType::Acknowledgment,
                    comms::PriorityLevel::High,
                    ack_payload,
                    5'000'000'000ULL,
                    0,
                    item.message.max_hops);
            } else if (config_.enable_dtn_retries) {
                // Simulator shortcut fallback if explicit ACKs disabled
                auto* sender = get_rover(item.message.sender_rover_id);
                if (sender) {
                    sender->dtn_buffer().acknowledge(item.message.message_id, current_time_ns_);
                }
            }
        } else {
            // Intermediate Rover: Store-Carry-Forward
            if (!config_.enable_multihop_routing) {
                total_messages_dropped_++;
                continue;
            }

            // Hop limit check
            if (item.message.hop_count + 1 >= item.message.max_hops) {
                hop_limit_exhausted_++;
                total_messages_dropped_++;
                continue;
            }

            // Intermediate duplicate detection: don't forward cyclic duplicates
            if (dest->dtn_buffer().is_duplicate(item.message.message_id, current_time_ns_)) {
                total_duplicate_deliveries_++;
                continue;
            }

            // Forward message onward: retain original message_id, origin_id, target_id, and TTL
            comms::DtnMessage fwd_msg = item.message;
            fwd_msg.sender_rover_id = dest->rover_id();
            fwd_msg.hop_count = item.message.hop_count + 1;
            fwd_msg.enqueue_time_ns = current_time_ns_;
            fwd_msg.attempt_count = 0;
            fwd_msg.next_retry_time_ns = current_time_ns_;

            auto enq_res = dest->dtn_buffer().enqueue(fwd_msg, current_time_ns_);
            if (enq_res != comms::EnqueueResult::Success) {
                total_messages_dropped_++;
            } else {
                dest->dtn_buffer().record_delivered_dedup(item.message.message_id, current_time_ns_);
            }
        }
    }
}

void MultiRoverCollective::process_received_message(SimulatedRover& destination, const comms::DtnMessage& message) {
    destination.update_last_peer_contact_ns(current_time_ns_);
    destination.messages_processed++;

    // Evaluate state machine transitions upon inbound packet reception
    RoverState current_st = destination.current_state();

    if (current_st == RoverState::Disconnected) {
        destination.state_machine().transition_to(
            RoverState::Reconnecting,
            TransitionReason::PacketReceivedInDisconnected,
            current_time_ns_,
            "Inbound packet received while disconnected");

        destination.state_machine().transition_to(
            RoverState::Synchronizing,
            TransitionReason::SyncInitiated,
            current_time_ns_,
            "Automatic synchronization initiated upon reconnection");
    } else if (current_st == RoverState::CommunicationDegraded) {
        destination.state_machine().transition_to(
            RoverState::Exploring,
            TransitionReason::LinkQualityRestored,
            current_time_ns_,
            "Inbound traffic restored normal communication state");
    }

    // Process specific payload types
    if (message.message_type == comms::DtnMessageType::BoundaryState && message.payload.size() == 80) {
        // Validate boundary state packet
        auto vres = optimization::deserialize_boundary_packet(message.payload.data(), message.payload.size());
        if (vres.valid && vres.packet.has_value()) {
            if (config_.enable_boundary_filtering) {
                destination.boundary_filter().process_packet(*vres.packet, current_time_ns_);
                total_boundary_packets_filtered_++;
            }
        }
    }
}

bool MultiRoverCollective::perform_synchronization(
    uint16_t rover_a_id,
    uint16_t rover_b_id,
    const std::vector<verification::PointCorrespondence2D>& candidate_correspondences) {

    auto* rover_a = get_rover(rover_a_id);
    auto* rover_b = get_rover(rover_b_id);

    if (!rover_a || !rover_b) {
        return false;
    }

    rover_a->sync_attempts++;
    rover_b->sync_attempts++;

    rover_a->state_machine().transition_to(
        RoverState::Synchronizing,
        TransitionReason::SyncInitiated,
        current_time_ns_,
        "Reconciliation synchronization initiated");
    rover_b->state_machine().transition_to(
        RoverState::Synchronizing,
        TransitionReason::SyncInitiated,
        current_time_ns_,
        "Reconciliation synchronization initiated");

    // Check byte budget for synchronization exchange
    const size_t sync_payload_size = rover_a->iblt().num_cells() * sizeof(core::IBLTCell);
    if (sync_payload_size > config_.sync_byte_limit) {
        rover_a->sync_budget_denials++;
        rover_b->sync_budget_denials++;
        rover_a->state_machine().transition_to(
            RoverState::ResourceConstrained,
            TransitionReason::ByteQuotaExhausted,
            current_time_ns_,
            "Synchronization payload exceeded byte quota");
        return false;
    }

    // Perform IBLT set difference reconciliation: (A - B)
    auto diff = rover_a->iblt().subtract(rover_b->iblt());
    auto decode_res = diff.decode();

    if (!decode_res.is_success()) {
        rover_a->sync_incomplete_decodes++;
        rover_b->sync_incomplete_decodes++;
        // Safe failure: leave graph untouched, return to exploring or disconnected
        rover_a->state_machine().transition_to(
            RoverState::Exploring,
            TransitionReason::SyncFailedDecode,
            current_time_ns_,
            "IBLT peeling stalled; difference cardinality exceeded table capacity");
        rover_b->state_machine().transition_to(
            RoverState::Exploring,
            TransitionReason::SyncFailedDecode,
            current_time_ns_,
            "IBLT peeling stalled; difference cardinality exceeded table capacity");
        return false;
    }

    rover_a->sync_successes++;
    rover_b->sync_successes++;

    // Check geometric verification if correspondences provided
    if (!candidate_correspondences.empty()) {
        verification::RansacConfig2D ransac_cfg;
        ransac_cfg.min_correspondences = 4;
        ransac_cfg.min_inliers = 4;
        ransac_cfg.inlier_distance_threshold = 0.20;

        auto vres = verification::verify_relative_pose_2d(candidate_correspondences, ransac_cfg);

        if (vres.success) {
            // Promote inter-rover loop closure in graphs
            graph::NodeId node_a(rover_a_id, rover_a->current_keyframe_index());
            graph::NodeId node_b(rover_b_id, rover_b->current_keyframe_index());

            // Idempotency check: does inter-rover edge already exist between these nodes?
            bool edge_exists = false;
            for (const auto& [eid, edge] : rover_a->pose_graph().edges()) {
                if (edge.type == graph::EdgeType::InterRoverLoopClosure &&
                    ((edge.from_node == node_a && edge.to_node == node_b) ||
                     (edge.from_node == node_b && edge.to_node == node_a))) {
                    edge_exists = true;
                    break;
                }
            }

            if (edge_exists) {
                rover_a->state_machine().transition_to(
                    RoverState::Exploring,
                    TransitionReason::SyncCompletedSuccessfully,
                    current_time_ns_,
                    "Inter-rover loop closure already verified (idempotent)");
                rover_b->state_machine().transition_to(
                    RoverState::Exploring,
                    TransitionReason::SyncCompletedSuccessfully,
                    current_time_ns_,
                    "Inter-rover loop closure already verified (idempotent)");
                return true;
            }

            graph::InformationMatrix2D info = graph::InformationMatrix2D::diagonal(50.0, 500.0);

            geometry::SE2 rel_pose(vres.estimated_transform.x, vres.estimated_transform.y, vres.estimated_transform.theta);

            // Register endpoint nodes in graphs if not present
            if (!rover_a->pose_graph().has_node(node_b)) {
                geometry::SE2 pose_b = rover_a->current_pose() * rel_pose;
                graph::KeyframeNode kf_b(node_b, graph::Pose2D(pose_b.x(), pose_b.y(), pose_b.theta()), current_time_ns_);
                rover_a->pose_graph().add_node(kf_b);
            }
            if (!rover_b->pose_graph().has_node(node_a)) {
                geometry::SE2 pose_a = rover_b->current_pose() * rel_pose.inverse();
                graph::KeyframeNode kf_a(node_a, graph::Pose2D(pose_a.x(), pose_a.y(), pose_a.theta()), current_time_ns_);
                rover_b->pose_graph().add_node(kf_a);
            }

            graph::PoseGraphEdge loop_edge_a(
                0,
                node_a,
                node_b,
                graph::EdgeType::InterRoverLoopClosure,
                graph::Pose2D(vres.estimated_transform.x, vres.estimated_transform.y, vres.estimated_transform.theta),
                info,
                graph::EdgeVerificationStatus::Verified);

            graph::PoseGraphEdge loop_edge_b(
                0,
                node_a,
                node_b,
                graph::EdgeType::InterRoverLoopClosure,
                graph::Pose2D(vres.estimated_transform.x, vres.estimated_transform.y, vres.estimated_transform.theta),
                info,
                graph::EdgeVerificationStatus::Verified);

            rover_a->pose_graph().add_edge(loop_edge_a);
            rover_b->pose_graph().add_edge(loop_edge_b);

            // Synchronize DSU partitions
            rover_a->partition_manager().rebuild(rover_a->pose_graph());
            rover_b->partition_manager().rebuild(rover_b->pose_graph());
        } else {
            // Geometric verification rejected
            rover_a->state_machine().transition_to(
                RoverState::Exploring,
                TransitionReason::SyncCompletedSuccessfully,
                current_time_ns_,
                "Geometric verification rejected candidate loop closure");
            rover_b->state_machine().transition_to(
                RoverState::Exploring,
                TransitionReason::SyncCompletedSuccessfully,
                current_time_ns_,
                "Geometric verification rejected candidate loop closure");
            return false;
        }
    }

    // Return rovers to Exploring
    rover_a->state_machine().transition_to(
        RoverState::Exploring,
        TransitionReason::SyncCompletedSuccessfully,
        current_time_ns_,
        "Landmark key reconciliation complete");
    rover_b->state_machine().transition_to(
        RoverState::Exploring,
        TransitionReason::SyncCompletedSuccessfully,
        current_time_ns_,
        "Landmark key reconciliation complete");

    return true;
}

void MultiRoverCollective::step(uint64_t dt_ns) {
    last_step_dt_ns_ = dt_ns;
    current_time_ns_ += dt_ns;
    network_.advance_time_ns(dt_ns);

    // Evaluate state machine timeouts and resource constraints for each rover
    for (auto& [rover_id, rover] : rovers_) {
        bool is_queue_full = (rover->dtn_buffer().config().max_queue_count > 0 &&
            rover->dtn_buffer().message_count() >= rover->dtn_buffer().config().max_queue_count);

        bool is_quota_constrained = rover->is_quota_constrained(current_time_ns_);

        bool is_constrained = is_queue_full || is_quota_constrained;

        rover->state_machine().evaluate_timeouts(current_time_ns_, rover->last_peer_contact_ns(), is_constrained);

        // Track peak queue memory
        peak_queue_messages_ = std::max(peak_queue_messages_, rover->dtn_buffer().message_count());
        peak_queue_payload_bytes_ = std::max(peak_queue_payload_bytes_, rover->dtn_buffer().payload_bytes());
    }

    // Deliver inbound transit messages first so newly arrived packets can trigger ACKs/forwards
    deliver_inbound_dtn_messages();

    // Dispatch outgoing DTN messages (including any newly generated ACKs/forwards)
    dispatch_outgoing_dtn_messages();
}

void MultiRoverCollective::run(uint64_t duration_ns, uint64_t step_dt_ns) {
    const uint64_t end_time = current_time_ns_ + duration_ns;
    while (current_time_ns_ < end_time) {
        step(step_dt_ns);
    }
}

CollectiveMetrics MultiRoverCollective::collect_metrics() const {
    CollectiveMetrics m;
    m.scenario_name = config_.scenario_name;
    m.seed = config_.random_seed;
    m.rover_count = rovers_.size();
    m.duration_ms = static_cast<double>(current_time_ns_) / 1e6;
    m.step_ms = static_cast<double>(last_step_dt_ns_) / 1e6;
    m.latency_ms = static_cast<double>(config_.network_config.latency_ns) / 1e6;
    m.packet_loss_rate = config_.network_config.packet_loss_rate;
    m.partition_duration_ms = 0.0;

    m.messages_generated = total_messages_generated_;
    m.messages_admitted = total_messages_admitted_;
    m.messages_sent = total_messages_sent_;
    m.initial_transmission_attempts = total_initial_attempts_;
    m.messages_retried = total_messages_retried_;
    m.messages_delivered = total_messages_delivered_;
    m.unique_messages_delivered = total_unique_delivered_;
    m.duplicate_deliveries = total_duplicate_deliveries_;
    m.messages_dropped = total_messages_dropped_;
    m.messages_rejected = total_messages_rejected_;
    m.in_flight_messages = in_transit_messages_.size();
    m.boundary_packets_filtered = total_boundary_packets_filtered_;

    m.payload_bytes_sent = total_payload_bytes_sent_;
    m.protocol_bytes_sent = total_protocol_bytes_sent_;
    m.framing_bytes_sent = total_framing_bytes_sent_;
    m.total_wire_bytes_sent = total_wire_bytes_sent_;
    m.total_wire_bytes_delivered = total_wire_bytes_delivered_;

    m.queue_peak_messages = peak_queue_messages_;
    m.queue_peak_payload_bytes = peak_queue_payload_bytes_;

    // Phase 4.6 Metrics
    m.hop_data_attempts = hop_data_attempts_;
    m.hop_data_delivered = hop_data_delivered_;
    m.hop_data_losses = hop_data_losses_;
    m.ack_transmission_attempts = ack_transmission_attempts_;
    m.ack_deliveries = ack_deliveries_;
    m.ack_losses = ack_losses_;
    m.retransmissions_missing_ack = retransmissions_missing_ack_;
    m.unique_messages_processed = unique_messages_processed_;
    m.duplicate_processing_attempts = 0; // Exactly 0 by design (dedup prevents duplicate processing attempts)
    m.hop_limit_exhausted = hop_limit_exhausted_;
    m.ack_wire_bytes_sent = ack_wire_bytes_sent_;
    m.ack_wire_bytes_delivered = ack_wire_bytes_delivered_;

    // End-to-end completion ratio: unique_messages_processed / eligible source messages admitted
    if (m.messages_admitted > 0) {
        m.end_to_end_completion_ratio = static_cast<double>(m.unique_messages_processed) / static_cast<double>(m.messages_admitted);
    } else {
        m.end_to_end_completion_ratio = 1.0;
    }

    for (const auto& [id, rover] : rovers_) {
        m.intermediate_queued_messages += rover->dtn_buffer().message_count();
        m.messages_processed += rover->messages_processed;
        m.messages_expired += rover->dtn_buffer().statistics().messages_expired;
        m.retries_exhausted += rover->dtn_buffer().statistics().retries_exhausted;
        m.state_transitions_total += rover->state_machine().total_transitions();
        m.sync_attempts += rover->sync_attempts;
        m.sync_successes += rover->sync_successes;
        m.sync_incomplete_decodes += rover->sync_incomplete_decodes;
        m.sync_budget_denials += rover->sync_budget_denials;
        m.keyframe_prunes_applied += rover->keyframe_prunes_applied;
        m.keyframe_prunes_rejected += rover->keyframe_prunes_rejected;

        // Connected components across collective
        if (id == rovers_.begin()->first) {
            m.final_connected_components = rover->partition_manager().num_partitions();
        }

        // Count verified inter-rover loop closures
        for (const auto& [eid, edge] : rover->pose_graph().edges()) {
            if (edge.type == graph::EdgeType::InterRoverLoopClosure &&
                edge.status == graph::EdgeVerificationStatus::Verified) {
                m.verified_inter_rover_closures++;
            }
        }
    }

    return m;
}

} // namespace astromesh::orchestration
