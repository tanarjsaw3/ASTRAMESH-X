#include "astromesh/simulation/baseline_astromesh_v1.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace astromesh::simulation {

BaselineAstroMeshV1::BaselineAstroMeshV1(const BaselineV1Config& config)
    : config_(config),
      rng_(config.seed) {
    if (!config_.is_valid()) {
        throw std::invalid_argument("BaselineAstroMeshV1: invalid BaselineV1Config specified.");
    }
    reset();
}

void BaselineAstroMeshV1::reset() {
    rovers_.clear();
    dsu_ = graph::DisjointSet();
    loop_closure_history_.clear();
    accepted_closure_nodes_.clear();
    processed_timestamps_.clear();

    rng_.seed(config_.seed);

    metrics_ = SwarmPipelineMetrics{};
    metrics_.pipeline_name = "AstroMesh_v1_Baseline";
    metrics_.optimization_supported = false;
    metrics_.dtn_supported = false;
    metrics_.keyframe_pruning_supported = false;
    metrics_.explicit_acknowledgments_supported = false;
    metrics_.multihop_routing_supported = false;
}

bool BaselineAstroMeshV1::ingest_step(const SimulationStepInput& step_input) {
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

        // Initialize rover state if first seen
        auto it = rovers_.find(rid);
        if (it == rovers_.end()) {
            auto [inserted_it, ok] = rovers_.emplace(
                rid,
                RoverInternalState(rid, config_.cbf_expected_items, config_.cbf_target_fpr));
            it = inserted_it;

            // Register root node in DSU
            graph::NodeId root_node(rid, 0);
            dsu_.make_set(root_node);
            it->second.estimated_poses.push_back(geometry::SE2::identity());
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

        // Add node to DSU and unite with previous sequential keyframe
        graph::NodeId prev_node(rid, static_cast<uint32_t>(prev_kf));
        graph::NodeId curr_node(rid, static_cast<uint32_t>(curr_kf));
        dsu_.make_set(curr_node);
        dsu_.unite(prev_node, curr_node);

        // Ingest landmark observations into local CBF and observation index
        for (const auto& obs : step_data.observations) {
            rover.cbf.insert(obs.landmark_key);
            if (rover.observations_by_landmark_id.find(obs.landmark_id) == rover.observations_by_landmark_id.end()) {
                rover.observed_keys.push_back(obs.landmark_key);
            }
            rover.observations_by_landmark_id[obs.landmark_id] = obs;
            metrics_.total_observations_ingested++;
        }
    }

    // 2. Perform pairwise inter-rover candidate discovery and geometric verification
    std::vector<uint16_t> active_rover_ids;
    active_rover_ids.reserve(rovers_.size());
    for (const auto& [rid, _] : rovers_) {
        active_rover_ids.push_back(rid);
    }
    std::sort(active_rover_ids.begin(), active_rover_ids.end());

    // Build link lookup map
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
                step_input.timestamp_ns);
        }
    }

    // Update graph topology metrics
    metrics_.dsu_components = dsu_.num_components();
    metrics_.total_keyframes_retained = 0;
    for (const auto& [_, r] : rovers_) {
        metrics_.total_keyframes_retained += r.estimated_poses.size();
    }

    // Compute communication ratios
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

void BaselineAstroMeshV1::process_pairwise_communication(
    RoverInternalState& rover_a,
    RoverInternalState& rover_b,
    const ChannelLinkState* link_state,
    uint64_t timestamp_ns) {

    // Evaluate physical range and partition
    if (link_state != nullptr) {
        if (!link_state->in_communication_range || link_state->is_partitioned) {
            return; // Out of range or severed by partition; no transmission occurs
        }
    } else {
        // Evaluate default communication range using estimated poses
        if (!rover_a.estimated_poses.empty() && !rover_b.estimated_poses.empty()) {
            const double dx = rover_a.estimated_poses.back().x() - rover_b.estimated_poses.back().x();
            const double dy = rover_a.estimated_poses.back().y() - rover_b.estimated_poses.back().y();
            if (std::hypot(dx, dy) > config_.communication_range) {
                return;
            }
        }
    }

    // Calculate wire bytes: CBF counter array + 24B protocol header + 16B transport framing
    const size_t cbf_payload_bytes = rover_a.cbf.num_counters(); // 8-bit counters = 1 byte each
    constexpr size_t PROTOCOL_HEADER_BYTES = 24;
    constexpr size_t FRAMING_BYTES = 16;
    const size_t wire_bytes = cbf_payload_bytes + PROTOCOL_HEADER_BYTES + FRAMING_BYTES;

    metrics_.logical_messages_sent++;
    metrics_.hop_transmissions_attempted++;
    metrics_.payload_bytes_transmitted += cbf_payload_bytes;
    metrics_.protocol_bytes_transmitted += PROTOCOL_HEADER_BYTES;
    metrics_.framing_bytes_transmitted += FRAMING_BYTES;
    metrics_.data_wire_bytes_transmitted += wire_bytes;
    metrics_.total_wire_bytes_transmitted += wire_bytes;

    // Evaluate stochastic packet loss
    const double loss_rate = (link_state != nullptr) ? link_state->link_packet_loss_rate : 0.0;
    if (loss_rate > 0.0 && dist_unit_(rng_) < loss_rate) {
        metrics_.logical_messages_dropped++;
        return; // Dropped in transit; no delivery, no candidate processing
    }

    const uint64_t one_way_delay_ns = (link_state != nullptr && link_state->one_way_delay_ns > 0)
        ? link_state->one_way_delay_ns
        : config_.default_one_way_delay_ns;

    const uint64_t delivery_time_ns = timestamp_ns + one_way_delay_ns;
    if (delivery_time_ns > config_.simulation_duration_ns) {
        metrics_.in_flight_messages++;
        metrics_.in_flight_at_termination++;
        return;
    }

    metrics_.logical_messages_delivered++;
    metrics_.unique_deliveries++;
    metrics_.hop_transmissions_successful++;

    // ------------------------------------------------------------------
    // CBF Candidate Discovery: Rover B queries its keys against Rover A's CBF
    // ------------------------------------------------------------------
    metrics_.candidate_discovery_attempts++;

    std::vector<core::LandmarkKey> candidate_keys;
    std::set<uint32_t> seen_lm_ids;
    for (const auto& key_b : rover_b.observed_keys) {
        if (rover_a.cbf.contains(key_b)) {
            const uint32_t lm_id = key_b.local_index();
            if (seen_lm_ids.insert(lm_id).second) {
                candidate_keys.push_back(key_b);
            }
        }
    }

    metrics_.candidate_pairs_proposed += candidate_keys.size();

    LoopClosureCandidate closure_record;
    closure_record.rover_a = rover_a.rover_id;
    closure_record.rover_b = rover_b.rover_id;
    closure_record.timestamp_ns = timestamp_ns;
    closure_record.candidate_landmarks_matched = candidate_keys.size();

    // Check minimum correspondences
    if (candidate_keys.size() < config_.ransac_config.min_correspondences) {
        closure_record.attempted_verification = false;
        closure_record.verification_passed = false;
        closure_record.failure_reason = verification::VerificationFailureReason::InsufficientCorrespondences;
        loop_closure_history_.push_back(closure_record);
        return;
    }

    // Extract measured point correspondences for matching keys
    // Target is Rover A, Source is Rover B so that estimated_transform maps
    // points from B to A (i.e. T_AB = X_A^{-1} * X_B), matching PoseGraphEdge convention.
    std::vector<verification::PointCorrespondence2D> correspondences;
    for (const auto& key : candidate_keys) {
        const uint32_t lm_id = key.local_index();
        auto it_a = rover_a.observations_by_landmark_id.find(lm_id);
        auto it_b = rover_b.observations_by_landmark_id.find(lm_id);

        if (it_a != rover_a.observations_by_landmark_id.end() &&
            it_b != rover_b.observations_by_landmark_id.end()) {
            correspondences.emplace_back(
                it_b->second.measured_local_point,
                it_a->second.measured_local_point,
                key);
        }
    }

    if (correspondences.size() < config_.ransac_config.min_correspondences) {
        closure_record.attempted_verification = false;
        closure_record.verification_passed = false;
        closure_record.failure_reason = verification::VerificationFailureReason::InsufficientCorrespondences;
        loop_closure_history_.push_back(closure_record);
        return;
    }

    // ------------------------------------------------------------------
    // Geometric Verification on Measured Correspondences
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

        // Direct DSU merge on accepted closure
        const graph::NodeId node_a(rover_a.rover_id, static_cast<uint32_t>(rover_a.keyframe_count));
        const graph::NodeId node_b(rover_b.rover_id, static_cast<uint32_t>(rover_b.keyframe_count));

        const uint32_t na_hash = (static_cast<uint32_t>(node_a.rover_id) << 16) | node_a.local_index;
        const uint32_t nb_hash = (static_cast<uint32_t>(node_b.rover_id) << 16) | node_b.local_index;
        const auto pair_key = std::make_pair(std::min(na_hash, nb_hash), std::max(na_hash, nb_hash));

        if (accepted_closure_nodes_.insert(pair_key).second) {
            if (!dsu_.are_connected(node_a, node_b)) {
                dsu_.unite(node_a, node_b);
                metrics_.dsu_merge_events++;
            }
        }
    } else {
        metrics_.loop_closures_rejected++;
    }

    loop_closure_history_.push_back(closure_record);
}

SwarmPipelineMetrics BaselineAstroMeshV1::collect_metrics() const {
    return metrics_;
}

std::map<uint16_t, std::vector<geometry::SE2>> BaselineAstroMeshV1::get_estimated_trajectories() const {
    std::map<uint16_t, std::vector<geometry::SE2>> trajectories;
    for (const auto& [rid, rover] : rovers_) {
        trajectories[rid] = rover.estimated_poses;
    }
    return trajectories;
}

std::optional<geometry::SE2> BaselineAstroMeshV1::get_current_pose(uint16_t rover_id) const {
    auto it = rovers_.find(rover_id);
    if (it == rovers_.end() || it->second.estimated_poses.empty()) {
        return std::nullopt;
    }
    return it->second.estimated_poses.back();
}

size_t BaselineAstroMeshV1::num_connected_components() const {
    return dsu_.num_components();
}

bool BaselineAstroMeshV1::are_rovers_connected(uint16_t rover_a, uint16_t rover_b) const {
    if (rover_a == rover_b) return true;

    auto it_a = rovers_.find(rover_a);
    auto it_b = rovers_.find(rover_b);
    if (it_a == rovers_.end() || it_b == rovers_.end()) {
        return false;
    }

    const graph::NodeId node_a(rover_a, static_cast<uint32_t>(it_a->second.keyframe_count));
    const graph::NodeId node_b(rover_b, static_cast<uint32_t>(it_b->second.keyframe_count));

    return dsu_.find_const(node_a) == dsu_.find_const(node_b);
}

const core::CountingBloomFilter* BaselineAstroMeshV1::get_rover_cbf(uint16_t rover_id) const noexcept {
    auto it = rovers_.find(rover_id);
    return (it != rovers_.end()) ? &it->second.cbf : nullptr;
}

} // namespace astromesh::simulation
