#include "astromesh/simulation/pipeline_astromesh_x.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace astromesh::simulation {

AstroMeshXPipeline::RoverInternalState::RoverInternalState(uint16_t id, const AstroMeshXConfig& cfg)
    : rover_id(id),
      iblt(cfg.iblt_cells, cfg.iblt_probes, cfg.protocol_id),
      pose_graph(),
      partition_manager(graph::ConnectivityPolicy{}),
      dtn_buffer(cfg.dtn_queue_config),
      budget_manager(cfg.budget_config),
      pruner(cfg.pruner_config),
      filter() {
}

AstroMeshXPipeline::AstroMeshXPipeline(const AstroMeshXConfig& config)
    : config_(config),
      custom_pipeline_name_(config.custom_pipeline_name),
      rng_(config.seed) {
    if (!config_.is_valid()) {
        throw std::invalid_argument("AstroMeshXPipeline: invalid AstroMeshXConfig specified.");
    }
    reset();
}

void AstroMeshXPipeline::reset() {
    rovers_.clear();
    global_dsu_ = graph::DisjointSet();
    swarm_graph_ = graph::PoseGraph();
    loop_closure_history_.clear();
    accepted_closure_nodes_.clear();
    processed_timestamps_.clear();

    rng_.seed(config_.seed);

    metrics_ = SwarmPipelineMetrics{};
    metrics_.pipeline_name = custom_pipeline_name_.empty() ? "AstroMesh_X" : custom_pipeline_name_;
    metrics_.optimization_supported = config_.enable_cord;
    metrics_.optimization_mode = config_.enable_cord ? "SynchronousMultiComponentCord" : "None";
    metrics_.dtn_supported = config_.enable_dtn_retries;
    metrics_.keyframe_pruning_supported = config_.enable_keyframe_pruning;
    metrics_.explicit_acknowledgments_supported = config_.enable_explicit_acks;
    metrics_.multihop_routing_supported = config_.enable_multihop_routing;
}

void AstroMeshXPipeline::rebuild_global_dsu() {
    global_dsu_ = graph::DisjointSet();
    for (const auto& [nid, _] : swarm_graph_.nodes()) {
        global_dsu_.make_set(nid);
    }
    for (const auto& [eid, edge] : swarm_graph_.edges()) {
        if (edge.type == graph::EdgeType::Odometry ||
            edge.status == graph::EdgeVerificationStatus::Verified) {
            global_dsu_.unite(edge.from_node, edge.to_node);
        }
    }
}

bool AstroMeshXPipeline::ingest_step(const SimulationStepInput& step_input) {
    if (!step_input.is_valid()) {
        return false;
    }

    // Step idempotency: ignore duplicate processing of identical timestamp
    if (step_input.timestamp_ns > 0) {
        if (!processed_timestamps_.insert(step_input.timestamp_ns).second) {
            return true;
        }
    }

    // 1. Ingest odometry and observations per rover
    for (const auto& step_data : step_input.rover_steps) {
        const uint16_t rid = step_data.rover_id;

        auto it = rovers_.find(rid);
        if (it == rovers_.end()) {
            auto [inserted_it, ok] = rovers_.emplace(
                rid,
                RoverInternalState(rid, config_));
            it = inserted_it;

            // Register root node in graphs and DSU
            graph::NodeId root_node(rid, 0);
            global_dsu_.make_set(root_node);
            it->second.estimated_poses.push_back(geometry::SE2::identity());

            graph::KeyframeNode root_kf(root_node, graph::Pose2D(0.0, 0.0, 0.0), step_input.timestamp_ns);
            it->second.pose_graph.add_node(root_kf);
            swarm_graph_.add_node(root_kf);
        }

        RoverInternalState& rover = it->second;
        const size_t prev_kf = rover.keyframe_count;
        rover.keyframe_count++;
        const size_t curr_kf = rover.keyframe_count;

        // Compute open-loop dead reckoning pose
        const geometry::SE2 prev_pose = rover.estimated_poses.empty() ?
            geometry::SE2::identity() : rover.estimated_poses.back();

        geometry::SE2 noisy_step = step_data.odometry_step;
        if (config_.odometry_noise_trans > 0.0 || config_.odometry_noise_rot > 0.0) {
            const double nx = dist_odom_trans_(rng_) * config_.odometry_noise_trans;
            const double ny = dist_odom_trans_(rng_) * config_.odometry_noise_trans;
            const double ntheta = dist_odom_rot_(rng_) * config_.odometry_noise_rot;
            noisy_step = geometry::SE2(
                noisy_step.x() + nx,
                noisy_step.y() + ny,
                graph::normalize_angle(noisy_step.theta() + ntheta));
        }

        const geometry::SE2 next_pose = prev_pose.compose(noisy_step);
        rover.estimated_poses.push_back(next_pose);
        metrics_.total_odometry_steps_processed++;

        // Add node and odometry edge
        graph::NodeId prev_node(rid, static_cast<uint32_t>(prev_kf));
        graph::NodeId curr_node(rid, static_cast<uint32_t>(curr_kf));

        graph::KeyframeNode curr_kf_node(
            curr_node,
            graph::Pose2D(next_pose.x(), next_pose.y(), next_pose.theta()),
            step_input.timestamp_ns);

        rover.pose_graph.add_node(curr_kf_node);
        swarm_graph_.add_node(curr_kf_node);

        global_dsu_.make_set(curr_node);
        global_dsu_.unite(prev_node, curr_node);

        const uint64_t odom_edge_id = (static_cast<uint64_t>(rid) << 48) |
                                      (static_cast<uint64_t>(prev_kf) << 24) |
                                      curr_kf;

        graph::PoseGraphEdge odom_edge(
            odom_edge_id,
            prev_node,
            curr_node,
            graph::EdgeType::Odometry,
            graph::Pose2D(noisy_step.x(), noisy_step.y(), noisy_step.theta()),
            graph::InformationMatrix2D::diagonal(100.0, 1000.0),
            graph::EdgeVerificationStatus::Verified);

        rover.pose_graph.add_edge(odom_edge);
        swarm_graph_.add_edge(odom_edge);
        rover.partition_manager.add_eligible_edge(odom_edge);

        // Ingest landmark observations into local IBLT
        for (const auto& obs : step_data.observations) {
            if (rover.observations_by_landmark_id.find(obs.landmark_id) == rover.observations_by_landmark_id.end()) {
                rover.iblt.insert(obs.landmark_key);
                rover.observed_keys.push_back(obs.landmark_key);
            }
            rover.observations_by_landmark_id[obs.landmark_id] = StoredObservation{obs, static_cast<uint32_t>(curr_kf)};
            metrics_.total_observations_ingested++;
        }

        // 2. Safe Keyframe Pruning if enabled
        if (config_.enable_keyframe_pruning &&
            rover.pose_graph.num_nodes() > config_.pruner_config.max_retained_keyframes) {
            auto pr = rover.pruner.enforce_memory_bound(rover.pose_graph, &rover.partition_manager);
            if (pr.status == graph::PruningStatus::SuccessBoundMet) {
                metrics_.keyframe_prunes_applied += pr.nodes_pruned;
                for (const auto& nid : pr.pruned_node_ids) {
                    swarm_graph_.remove_node(nid);
                }
                rebuild_global_dsu();
            } else if (pr.status == graph::PruningStatus::LimitUnmetNoSafeCandidate) {
                metrics_.keyframe_prunes_rejected++;
            }
        }
    }

    // 3. Process communication and IBLT reconciliation across active rovers
    std::vector<uint16_t> active_rover_ids;
    active_rover_ids.reserve(rovers_.size());
    for (const auto& [rid, _] : rovers_) {
        active_rover_ids.push_back(rid);
    }
    std::sort(active_rover_ids.begin(), active_rover_ids.end());

    std::map<std::pair<uint16_t, uint16_t>, const ChannelLinkState*> link_map;
    for (const auto& link : step_input.channel_states) {
        const uint16_t u = std::min(link.rover_a, link.rover_b);
        const uint16_t v = std::max(link.rover_a, link.rover_b);
        link_map[{u, v}] = &link;
    }

    for (size_t i = 0; i < active_rover_ids.size(); ++i) {
        for (size_t j = i + 1; j < active_rover_ids.size(); ++j) {
            const uint16_t id_a = active_rover_ids[i];
            const uint16_t id_b = active_rover_ids[j];

            auto link_it = link_map.find({id_a, id_b});
            const ChannelLinkState* link_state = (link_it != link_map.end()) ? link_it->second : nullptr;

            process_pairwise_communication(
                rovers_.at(id_a),
                rovers_.at(id_b),
                link_state,
                step_input.timestamp_ns,
                link_map,
                active_rover_ids);
        }
    }

    // 4. Update topology and keyframe metrics
    metrics_.dsu_components = global_dsu_.num_components();
    metrics_.total_keyframes_retained = 0;
    for (const auto& [_, r] : rovers_) {
        metrics_.total_keyframes_retained += r.pose_graph.num_nodes();
    }

    // Compute ratios
    if (metrics_.logical_messages_sent > 0) {
        metrics_.application_completion_ratio =
            static_cast<double>(metrics_.unique_deliveries) /
            static_cast<double>(metrics_.logical_messages_sent);
    }
    if (metrics_.hop_transmissions_attempted > 0) {
        metrics_.per_hop_delivery_ratio =
            static_cast<double>(metrics_.hop_transmissions_successful) /
            static_cast<double>(metrics_.hop_transmissions_attempted);
    }

    return true;
}

void AstroMeshXPipeline::process_pairwise_communication(
    RoverInternalState& rover_a,
    RoverInternalState& rover_b,
    const ChannelLinkState* link_state,
    uint64_t timestamp_ns,
    const std::map<std::pair<uint16_t, uint16_t>, const ChannelLinkState*>& link_map,
    const std::vector<uint16_t>& active_rover_ids) {

    bool is_directly_connected = true;
    double loss_rate = config_.packet_loss_rate;

    if (link_state != nullptr) {
        if (!link_state->in_communication_range || link_state->is_partitioned) {
            is_directly_connected = false;
        }
        loss_rate = link_state->link_packet_loss_rate;
    } else {
        if (!rover_a.estimated_poses.empty() && !rover_b.estimated_poses.empty()) {
            const double dx = rover_a.estimated_poses.back().x() - rover_b.estimated_poses.back().x();
            const double dy = rover_a.estimated_poses.back().y() - rover_b.estimated_poses.back().y();
            if (std::hypot(dx, dy) > config_.communication_range) {
                is_directly_connected = false;
            }
        }
    }

    size_t route_hops = 1;
    if (!is_directly_connected) {
        if (!config_.enable_multihop_routing) {
            return; // Disconnected and multi-hop disabled
        }

        // Bounded multi-hop routing: check if an intermediate peer C can forward
        bool route_found = false;
        for (uint16_t cand_id : active_rover_ids) {
            if (cand_id == rover_a.rover_id || cand_id == rover_b.rover_id) continue;

            const auto key_ac = std::make_pair(std::min(rover_a.rover_id, cand_id), std::max(rover_a.rover_id, cand_id));
            const auto key_cb = std::make_pair(std::min(cand_id, rover_b.rover_id), std::max(cand_id, rover_b.rover_id));

            auto it_ac = link_map.find(key_ac);
            auto it_cb = link_map.find(key_cb);

            bool ac_ok = (it_ac != link_map.end()) ? (it_ac->second->in_communication_range && !it_ac->second->is_partitioned) : true;
            bool cb_ok = (it_cb != link_map.end()) ? (it_cb->second->in_communication_range && !it_cb->second->is_partitioned) : true;

            if (ac_ok && cb_ok) {
                route_found = true;
                route_hops = 2;
                break;
            }
        }

        if (!route_found) {
            return; // No connected multi-hop route
        }
    }

    // Check hop limit
    if (route_hops > config_.default_max_hops) {
        metrics_.hop_limit_exhausted++;
        metrics_.logical_messages_dropped++;
        return;
    }

    // Calculate wire bytes: IBLT payload (num_cells * 32 B) vs Raw observed keys (num_keys * 16 B)
    size_t payload_bytes = 0;
    if (config_.discovery_mechanism == AstroMeshXConfig::DiscoveryMechanism::RawExchange) {
        payload_bytes = rover_a.observed_keys.size() * sizeof(core::LandmarkKey);
    } else {
        payload_bytes = rover_a.iblt.num_cells() * sizeof(core::IBLTCell);
    }
    constexpr size_t PROTOCOL_HEADER_BYTES = 24;
    constexpr size_t FRAMING_BYTES = 16;
    const size_t wire_bytes_per_hop = payload_bytes + PROTOCOL_HEADER_BYTES + FRAMING_BYTES;
    const size_t total_data_wire_bytes = wire_bytes_per_hop * route_hops;

    const uint64_t one_way_delay_ns = (link_state != nullptr && link_state->one_way_delay_ns > 0)
        ? link_state->one_way_delay_ns
        : config_.default_one_way_delay_ns;

    metrics_.logical_messages_sent++;

    constexpr uint64_t retry_offsets_ns[5] = {0, 50'000'000ULL, 150'000'000ULL, 350'000'000ULL, 750'000'000ULL};

    if (config_.enable_dtn_retries) {
        size_t first_success_attempt = 5; // 5 = all 5 attempts failed
        for (size_t attempt = 0; attempt < 5; ++attempt) {
            if (loss_rate <= 0.0 || dist_unit_(rng_) >= loss_rate) {
                first_success_attempt = attempt;
                break;
            }
        }

        if (first_success_attempt == 5) {
            // All 5 attempts failed: retry exhaustion drop
            metrics_.retransmissions_performed += 4;
            metrics_.logical_messages_dropped++;
            metrics_.retry_exhausted_drops++;
            metrics_.hop_transmissions_attempted += 5 * route_hops;
            metrics_.payload_bytes_transmitted += 5 * route_hops * payload_bytes;
            metrics_.protocol_bytes_transmitted += 5 * route_hops * PROTOCOL_HEADER_BYTES;
            metrics_.framing_bytes_transmitted += 5 * route_hops * FRAMING_BYTES;
            metrics_.data_wire_bytes_transmitted += 5 * total_data_wire_bytes;
            metrics_.total_wire_bytes_transmitted += 5 * total_data_wire_bytes;
            return;
        }

        const size_t attempts_used = first_success_attempt + 1;
        const size_t loss_retries = first_success_attempt;
        metrics_.retransmissions_performed += loss_retries;
        metrics_.hop_transmissions_attempted += attempts_used * route_hops;
        metrics_.hop_transmissions_successful += route_hops;
        metrics_.payload_bytes_transmitted += attempts_used * route_hops * payload_bytes;
        metrics_.protocol_bytes_transmitted += attempts_used * route_hops * PROTOCOL_HEADER_BYTES;
        metrics_.framing_bytes_transmitted += attempts_used * route_hops * FRAMING_BYTES;
        metrics_.data_wire_bytes_transmitted += attempts_used * total_data_wire_bytes;
        metrics_.total_wire_bytes_transmitted += attempts_used * total_data_wire_bytes;

        // Model spurious retransmissions caused by propagation delay exceeding ACK retry timeout
        size_t spurious_retries = 0;
        if (config_.enable_explicit_acks && one_way_delay_ns > 0) {
            const uint64_t rtt_ns = 2ULL * one_way_delay_ns * route_hops;
            if (rtt_ns > 50'000'000ULL) spurious_retries++;
            if (rtt_ns > 150'000'000ULL) spurious_retries++;
            if (rtt_ns > 350'000'000ULL) spurious_retries++;
            if (rtt_ns > 750'000'000ULL) spurious_retries++;
            if (loss_retries + spurious_retries > 4) {
                spurious_retries = (4 > loss_retries) ? (4 - loss_retries) : 0;
            }
        }

        if (spurious_retries > 0) {
            metrics_.retransmissions_performed += spurious_retries;
            metrics_.duplicate_deliveries += spurious_retries;
            metrics_.hop_transmissions_attempted += spurious_retries * route_hops;
            metrics_.hop_transmissions_successful += spurious_retries * route_hops;
            metrics_.payload_bytes_transmitted += spurious_retries * route_hops * payload_bytes;
            metrics_.protocol_bytes_transmitted += spurious_retries * route_hops * PROTOCOL_HEADER_BYTES;
            metrics_.framing_bytes_transmitted += spurious_retries * route_hops * FRAMING_BYTES;
            metrics_.data_wire_bytes_transmitted += spurious_retries * total_data_wire_bytes;
            metrics_.total_wire_bytes_transmitted += spurious_retries * total_data_wire_bytes;
        }

        if (config_.enable_explicit_acks) {
            constexpr size_t ACK_PAYLOAD_BYTES = 24;
            constexpr size_t ACK_FRAMING_BYTES = 20;
            const size_t total_ack_pkts = 1 + spurious_retries;
            const size_t ack_wire_bytes = (ACK_PAYLOAD_BYTES + ACK_FRAMING_BYTES) * route_hops * total_ack_pkts;

            metrics_.ack_wire_bytes_transmitted += ack_wire_bytes;
            metrics_.total_wire_bytes_transmitted += ack_wire_bytes;
            metrics_.hop_transmissions_attempted += total_ack_pkts * route_hops;
            metrics_.hop_transmissions_successful += total_ack_pkts * route_hops;
        }

        const uint64_t trans_time_ns = timestamp_ns + retry_offsets_ns[first_success_attempt];
        const uint64_t delivery_time_ns = trans_time_ns + one_way_delay_ns * route_hops;

        if (delivery_time_ns > config_.simulation_duration_ns) {
            metrics_.in_flight_messages++;
            metrics_.in_flight_at_termination++;
            return;
        }

        metrics_.logical_messages_delivered++;
        metrics_.unique_deliveries++;
    } else {
        // DTN retries disabled
        metrics_.hop_transmissions_attempted += route_hops;
        metrics_.payload_bytes_transmitted += payload_bytes * route_hops;
        metrics_.protocol_bytes_transmitted += PROTOCOL_HEADER_BYTES * route_hops;
        metrics_.framing_bytes_transmitted += FRAMING_BYTES * route_hops;
        metrics_.data_wire_bytes_transmitted += total_data_wire_bytes;
        metrics_.total_wire_bytes_transmitted += total_data_wire_bytes;

        if (loss_rate > 0.0 && dist_unit_(rng_) < loss_rate) {
            metrics_.logical_messages_dropped++;
            return;
        }

        metrics_.hop_transmissions_successful += route_hops;

        if (config_.enable_explicit_acks) {
            constexpr size_t ACK_PAYLOAD_BYTES = 24;
            constexpr size_t ACK_FRAMING_BYTES = 20;
            const size_t ack_wire_bytes = (ACK_PAYLOAD_BYTES + ACK_FRAMING_BYTES) * route_hops;

            metrics_.ack_wire_bytes_transmitted += ack_wire_bytes;
            metrics_.total_wire_bytes_transmitted += ack_wire_bytes;
            metrics_.hop_transmissions_attempted += route_hops;
            metrics_.hop_transmissions_successful += route_hops;
        }

        const uint64_t delivery_time_ns = timestamp_ns + one_way_delay_ns * route_hops;
        if (delivery_time_ns > config_.simulation_duration_ns) {
            metrics_.in_flight_messages++;
            metrics_.in_flight_at_termination++;
            return;
        }

        metrics_.logical_messages_delivered++;
        metrics_.unique_deliveries++;
    }

    // ------------------------------------------------------------------
    // IBLT Set Reconciliation
    // ------------------------------------------------------------------
    attempt_iblt_reconciliation(rover_a, rover_b, timestamp_ns);
}

bool AstroMeshXPipeline::attempt_iblt_reconciliation(
    RoverInternalState& rover_a,
    RoverInternalState& rover_b,
    uint64_t timestamp_ns) {

    metrics_.sync_attempts++;

    std::vector<core::LandmarkKey> candidate_keys;
    std::set<uint32_t> seen_lm_ids;

    if (config_.discovery_mechanism == AstroMeshXConfig::DiscoveryMechanism::RawExchange) {
        // Direct LandmarkKey list intersection ablation (No IBLT)
        metrics_.sync_successes++;
        for (const auto& key_b : rover_b.observed_keys) {
            const uint32_t lm_id = key_b.local_index();
            if (rover_a.observations_by_landmark_id.find(lm_id) != rover_a.observations_by_landmark_id.end()) {
                if (seen_lm_ids.insert(lm_id).second) {
                    candidate_keys.push_back(key_b);
                }
            }
        }
    } else {
        // Compatibility pre-check: protocol ID and dimension alignment
        if (!rover_a.iblt.is_compatible_with(rover_b.iblt)) {
            metrics_.sync_incomplete_decodes++;
            return false;
        }

        // Subtract tables: T_diff = A - B with exception safety
        core::InvertibleBloomFilter diff(rover_a.iblt.num_cells(), rover_a.iblt.num_probes(), rover_a.iblt.protocol_id());
        try {
            diff = rover_a.iblt.subtract(rover_b.iblt);
        } catch (const std::exception&) {
            metrics_.sync_incomplete_decodes++;
            return false;
        }

        auto decode_res = diff.decode();

        if (!decode_res.is_success()) {
            metrics_.sync_incomplete_decodes++;
            // Failure atomicity: DO NOT mutate graph, DO NOT unite DSU
            return false;
        }

        metrics_.sync_successes++;

        // Candidate discovery: extract candidate keys
        // InvertibleBloomFilter peels symmetric difference A \ B (left_only) and B \ A (right_only).
        // Shared keys = observed_keys of rover_b that are NOT in right_only (or observed_keys of rover_a not in left_only)
        std::set<core::LandmarkKey> right_only_set(decode_res.right_only.begin(), decode_res.right_only.end());

        for (const auto& key_b : rover_b.observed_keys) {
            if (right_only_set.find(key_b) == right_only_set.end()) {
                const uint32_t lm_id = key_b.local_index();
                if (rover_a.observations_by_landmark_id.find(lm_id) != rover_a.observations_by_landmark_id.end()) {
                    if (seen_lm_ids.insert(lm_id).second) {
                        candidate_keys.push_back(key_b);
                    }
                }
            }
        }
    }

    metrics_.candidate_discovery_attempts++;
    metrics_.candidate_pairs_proposed += candidate_keys.size();

    LoopClosureCandidate closure_record;
    closure_record.rover_a = rover_a.rover_id;
    closure_record.rover_b = rover_b.rover_id;
    closure_record.timestamp_ns = timestamp_ns;
    closure_record.candidate_landmarks_matched = candidate_keys.size();

    if (candidate_keys.size() < config_.ransac_config.min_correspondences) {
        closure_record.attempted_verification = false;
        closure_record.verification_passed = false;
        closure_record.failure_reason = verification::VerificationFailureReason::InsufficientCorrespondences;
        loop_closure_history_.push_back(closure_record);
        return false;
    }

    // Build correspondences from measured coordinates
    std::vector<verification::PointCorrespondence2D> correspondences;
    uint32_t kf_a = static_cast<uint32_t>(rover_a.keyframe_count);
    uint32_t kf_b = static_cast<uint32_t>(rover_b.keyframe_count);

    for (const auto& key : candidate_keys) {
        const uint32_t lm_id = key.local_index();
        auto it_a = rover_a.observations_by_landmark_id.find(lm_id);
        auto it_b = rover_b.observations_by_landmark_id.find(lm_id);

        if (it_a != rover_a.observations_by_landmark_id.end() &&
            it_b != rover_b.observations_by_landmark_id.end()) {
            // Target is Rover A, Source is Rover B so that estimated_transform maps
            // points from B to A (i.e. T_AB = X_A^{-1} * X_B), matching PoseGraphEdge convention.
            correspondences.emplace_back(
                it_b->second.obs.measured_local_point,
                it_a->second.obs.measured_local_point,
                key);
            kf_a = it_a->second.keyframe_index;
            kf_b = it_b->second.keyframe_index;
        }
    }

    if (correspondences.size() < config_.ransac_config.min_correspondences) {
        closure_record.attempted_verification = false;
        closure_record.verification_passed = false;
        closure_record.failure_reason = verification::VerificationFailureReason::InsufficientCorrespondences;
        loop_closure_history_.push_back(closure_record);
        return false;
    }

    // ------------------------------------------------------------------
    // Geometric Verification & Gating
    // ------------------------------------------------------------------
    closure_record.attempted_verification = true;
    metrics_.verification_attempts++;

    auto vres = verification::verify_relative_pose_2d(correspondences, config_.ransac_config);

    closure_record.verification_passed = vres.success;
    closure_record.failure_reason = vres.failure_reason;
    closure_record.estimated_relative_transform = vres.estimated_transform;
    closure_record.inlier_count = vres.inlier_count;
    closure_record.inlier_ratio = vres.inlier_ratio;
    closure_record.inlier_rmse = vres.inlier_rmse;

    if (vres.success) {
        metrics_.loop_closures_verified++;

        // Idempotent edge insertion & DSU union
        const graph::NodeId node_a(rover_a.rover_id, kf_a);
        const graph::NodeId node_b(rover_b.rover_id, kf_b);

        const uint32_t na_hash = (static_cast<uint32_t>(node_a.rover_id) << 16) | node_a.local_index;
        const uint32_t nb_hash = (static_cast<uint32_t>(node_b.rover_id) << 16) | node_b.local_index;
        const auto pair_key = std::make_pair(std::min(na_hash, nb_hash), std::max(na_hash, nb_hash));

        if (accepted_closure_nodes_.insert(pair_key).second) {
            const uint64_t edge_id = (static_cast<uint64_t>(node_a.rover_id) << 48) |
                                     (static_cast<uint64_t>(node_b.rover_id) << 32) |
                                     (static_cast<uint64_t>(node_a.local_index) << 16) |
                                     node_b.local_index;

            graph::PoseGraphEdge inter_edge(
                edge_id,
                node_a,
                node_b,
                graph::EdgeType::InterRoverLoopClosure,
                vres.estimated_transform,
                graph::InformationMatrix2D::diagonal(50.0, 500.0),
                graph::EdgeVerificationStatus::Verified);

            rover_a.pose_graph.add_edge(inter_edge);
            rover_b.pose_graph.add_edge(inter_edge);
            swarm_graph_.add_edge(inter_edge);

            rover_a.partition_manager.add_eligible_edge(inter_edge);
            rover_b.partition_manager.add_eligible_edge(inter_edge);

            if (!global_dsu_.are_connected(node_a, node_b)) {
                global_dsu_.unite(node_a, node_b);
                metrics_.dsu_merge_events++;
            }
        }
    } else {
        metrics_.loop_closures_rejected++;
    }

    loop_closure_history_.push_back(closure_record);
    return vres.success;
}

bool AstroMeshXPipeline::optimize() {
    if (!config_.enable_cord) {
        return false;
    }

    // Collect initial poses across all active keyframes
    std::unordered_map<graph::NodeId, geometry::SE2> initial_poses;
    for (const auto& [rid, rover] : rovers_) {
        for (size_t k = 0; k < rover.estimated_poses.size(); ++k) {
            graph::NodeId nid(rid, static_cast<uint32_t>(k));
            if (rover.pose_graph.has_node(nid) && swarm_graph_.has_node(nid)) {
                initial_poses[nid] = rover.estimated_poses[k];
            }
        }
    }

    if (initial_poses.empty()) {
        return false;
    }

    // Execute CORD continuous Riemannian dynamics optimization (Synchronous Multi-Component mode)
    auto cord_res = optimization::integrate_cord(swarm_graph_, initial_poses, config_.cord_options);

    metrics_.optimization_steps_performed += cord_res.steps;
    metrics_.centralized_optimization_performed = false; // Synchronous Multi-Component CORD, not centralized LM
    metrics_.optimization_mode = "SynchronousMultiComponentCord";

    // Update estimated poses from relaxed states
    for (auto& [rid, rover] : rovers_) {
        for (size_t k = 0; k < rover.estimated_poses.size(); ++k) {
            graph::NodeId nid(rid, static_cast<uint32_t>(k));
            auto it = cord_res.final_states.find(nid);
            if (it != cord_res.final_states.end()) {
                rover.estimated_poses[k] = it->second.pose;
            }
        }
    }

    return cord_res.is_converged() || cord_res.status == optimization::CordStatus::MaxStepsReached;
}

SwarmPipelineMetrics AstroMeshXPipeline::collect_metrics() const {
    return metrics_;
}

std::map<uint16_t, std::vector<geometry::SE2>> AstroMeshXPipeline::get_estimated_trajectories() const {
    std::map<uint16_t, std::vector<geometry::SE2>> trajectories;
    for (const auto& [rid, rover] : rovers_) {
        trajectories[rid] = rover.estimated_poses;
    }
    return trajectories;
}

std::optional<geometry::SE2> AstroMeshXPipeline::get_current_pose(uint16_t rover_id) const {
    auto it = rovers_.find(rover_id);
    if (it == rovers_.end() || it->second.estimated_poses.empty()) {
        return std::nullopt;
    }
    return it->second.estimated_poses.back();
}

size_t AstroMeshXPipeline::num_connected_components() const {
    return global_dsu_.num_components();
}

bool AstroMeshXPipeline::are_rovers_connected(uint16_t rover_a, uint16_t rover_b) const {
    if (rover_a == rover_b) return true;

    auto it_a = rovers_.find(rover_a);
    auto it_b = rovers_.find(rover_b);
    if (it_a == rovers_.end() || it_b == rovers_.end()) {
        return false;
    }

    const graph::NodeId node_a(rover_a, static_cast<uint32_t>(it_a->second.keyframe_count));
    const graph::NodeId node_b(rover_b, static_cast<uint32_t>(it_b->second.keyframe_count));

    return global_dsu_.find_const(node_a) == global_dsu_.find_const(node_b);
}

const core::InvertibleBloomFilter* AstroMeshXPipeline::get_rover_iblt(uint16_t rover_id) const noexcept {
    auto it = rovers_.find(rover_id);
    return (it != rovers_.end()) ? &it->second.iblt : nullptr;
}

const graph::PoseGraph* AstroMeshXPipeline::get_rover_graph(uint16_t rover_id) const noexcept {
    auto it = rovers_.find(rover_id);
    return (it != rovers_.end()) ? &it->second.pose_graph : nullptr;
}

const comms::DtnBuffer* AstroMeshXPipeline::get_rover_dtn_buffer(uint16_t rover_id) const noexcept {
    auto it = rovers_.find(rover_id);
    return (it != rovers_.end()) ? &it->second.dtn_buffer : nullptr;
}

const graph::SubgraphPartitionManager* AstroMeshXPipeline::get_rover_partition_manager(uint16_t rover_id) const noexcept {
    auto it = rovers_.find(rover_id);
    return (it != rovers_.end()) ? &it->second.partition_manager : nullptr;
}

std::vector<LoopClosureCandidate> AstroMeshXPipeline::get_accepted_closures() const {
    std::vector<LoopClosureCandidate> accepted;
    for (const auto& c : loop_closure_history_) {
        if (c.verification_passed) accepted.push_back(c);
    }
    return accepted;
}

std::vector<LoopClosureCandidate> AstroMeshXPipeline::get_rejected_closures() const {
    std::vector<LoopClosureCandidate> rejected;
    for (const auto& c : loop_closure_history_) {
        if (!c.verification_passed) rejected.push_back(c);
    }
    return rejected;
}

} // namespace astromesh::simulation
