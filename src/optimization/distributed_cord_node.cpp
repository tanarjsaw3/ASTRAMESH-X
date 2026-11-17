#include "astromesh/optimization/distributed_cord_node.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>

namespace astromesh::optimization {

std::string neighbor_link_status_to_string(NeighborLinkStatus status) {
    switch (status) {
        case NeighborLinkStatus::Fresh: return "Fresh";
        case NeighborLinkStatus::StaleHeld: return "StaleHeld";
        case NeighborLinkStatus::Disconnected: return "Disconnected";
    }
    return "Unknown";
}

std::ostream& operator<<(std::ostream& os, NeighborLinkStatus status) {
    return os << neighbor_link_status_to_string(status);
}

std::string lifecycle_event_type_to_string(LifecycleEventType type) {
    switch (type) {
        case LifecycleEventType::LinkConnected: return "LinkConnected";
        case LifecycleEventType::LinkBecameStale: return "LinkBecameStale";
        case LifecycleEventType::LinkDisconnected: return "LinkDisconnected";
        case LifecycleEventType::LinkReconnected: return "LinkReconnected";
        case LifecycleEventType::PacketRejectedDuplicate: return "PacketRejectedDuplicate";
        case LifecycleEventType::PacketRejectedCorrupt: return "PacketRejectedCorrupt";
        case LifecycleEventType::PacketRejectedUnsupported: return "PacketRejectedUnsupported";
        case LifecycleEventType::PredictionGatedMotion: return "PredictionGatedMotion";
        case LifecycleEventType::PredictionGatedAge: return "PredictionGatedAge";
    }
    return "Unknown";
}

DistributedCordNode::DistributedCordNode(
    uint32_t rover_id,
    const graph::PoseGraph& local_graph,
    const std::unordered_map<graph::NodeId, NodeDynamicsState>& initial_local_states,
    const DistributedCordOptions& options,
    SimulatedMeshNetwork& network)
    : rover_id_(rover_id),
      local_graph_(local_graph),
      local_states_(initial_local_states),
      options_(options),
      network_(network),
      filter_(options.filter_options) {

    discover_boundary_topology();
}

void DistributedCordNode::discover_boundary_topology() {
    local_boundary_nodes_.clear();
    boundary_node_to_remote_rover_.clear();
    neighbor_states_.clear();
    inter_rover_edges_.clear();

    for (const auto& [edge_id, edge] : local_graph_.edges()) {
        if (!is_edge_eligible_for_objective(edge)) {
            continue;
        }

        if (edge.type != graph::EdgeType::InterRoverLoopClosure) {
            continue;
        }

        const bool from_local = (edge.from_node.rover_id == rover_id_);
        const bool to_local = (edge.to_node.rover_id == rover_id_);

        if (from_local && !to_local) {
            const graph::NodeId local_node = edge.from_node;
            const graph::NodeId remote_node = edge.to_node;
            const uint32_t remote_rover = remote_node.rover_id;

            inter_rover_edges_.push_back({edge_id, local_node, remote_node, remote_rover});

            if (std::find(local_boundary_nodes_.begin(), local_boundary_nodes_.end(), local_node) == local_boundary_nodes_.end()) {
                local_boundary_nodes_.push_back(local_node);
            }
            boundary_node_to_remote_rover_[local_node] = remote_rover;

            if (neighbor_states_.find(remote_node) == neighbor_states_.end()) {
                NeighborStateRecord rec;
                rec.node_id = remote_node;
                rec.remote_rover_id = remote_rover;
                rec.status = NeighborLinkStatus::Disconnected;
                neighbor_states_[remote_node] = rec;
            }
        } else if (!from_local && to_local) {
            const graph::NodeId local_node = edge.to_node;
            const graph::NodeId remote_node = edge.from_node;
            const uint32_t remote_rover = remote_node.rover_id;

            inter_rover_edges_.push_back({edge_id, local_node, remote_node, remote_rover});

            if (std::find(local_boundary_nodes_.begin(), local_boundary_nodes_.end(), local_node) == local_boundary_nodes_.end()) {
                local_boundary_nodes_.push_back(local_node);
            }
            boundary_node_to_remote_rover_[local_node] = remote_rover;

            if (neighbor_states_.find(remote_node) == neighbor_states_.end()) {
                NeighborStateRecord rec;
                rec.node_id = remote_node;
                rec.remote_rover_id = remote_rover;
                rec.status = NeighborLinkStatus::Disconnected;
                neighbor_states_[remote_node] = rec;
            }
        }
    }
}

size_t DistributedCordNode::active_inter_rover_edge_count() const noexcept {
    size_t count = 0;
    for (const auto& ir_edge : inter_rover_edges_) {
        auto it = neighbor_states_.find(ir_edge.remote_node);
        if (it != neighbor_states_.end() && it->second.status != NeighborLinkStatus::Disconnected) {
            count++;
        }
    }
    return count;
}

size_t DistributedCordNode::total_inter_rover_edge_count() const noexcept {
    return inter_rover_edges_.size();
}

void DistributedCordNode::inject_neighbor_state(
    const graph::NodeId& neighbor_id,
    uint32_t remote_rover_id,
    const geometry::SE2& pose,
    const geometry::Twist2D& velocity,
    uint64_t timestamp_ns) {

    NeighborStateRecord& rec = neighbor_states_[neighbor_id];
    rec.node_id = neighbor_id;
    rec.remote_rover_id = remote_rover_id;
    rec.received_pose = pose;
    rec.received_velocity = velocity;
    rec.current_estimate_pose = pose;
    rec.current_estimate_velocity = velocity;
    rec.packet_timestamp_ns = timestamp_ns;
    rec.reception_timestamp_ns = timestamp_ns;
    rec.has_received_packet = true;
    rec.status = NeighborLinkStatus::Fresh;
    rec.is_prediction_gated = false;

    if (options_.filter_options.enable_filtering) {
        BoundaryStatePacket syn_pkt;
        syn_pkt.protocol_version = BOUNDARY_PROTOCOL_VERSION;
        syn_pkt.sender_rover_id = static_cast<uint16_t>(remote_rover_id);
        syn_pkt.target_rover_id = static_cast<uint16_t>(rover_id_);
        syn_pkt.node_local_index = neighbor_id.local_index;
        syn_pkt.sequence_number = 1;
        syn_pkt.timestamp_ns = timestamp_ns;
        syn_pkt.x = pose.x();
        syn_pkt.y = pose.y();
        syn_pkt.theta = pose.theta();
        syn_pkt.vx = velocity.vx;
        syn_pkt.vy = velocity.vy;
        syn_pkt.omega = velocity.omega;
        std::vector<uint8_t> wire = serialize_boundary_packet(syn_pkt);
        PacketValidationResult res = deserialize_boundary_packet(wire.data(), wire.size());
        if (res.valid && res.packet.has_value()) {
            filter_.process_packet(*res.packet, timestamp_ns);
        }
    }
}

void DistributedCordNode::process_inbound_packets(uint64_t current_time_ns) {
    auto arriving_packets = network_.get().receive_packets(static_cast<uint16_t>(rover_id_));

    for (const auto& pkt : arriving_packets) {
        if (!pkt.is_valid()) {
            lifecycle_events_.push_back({
                current_time_ns,
                pkt.node_id(),
                LifecycleEventType::PacketRejectedCorrupt,
                "Packet is invalid or corrupt"
            });
            continue;
        }

        if (pkt.target_rover_id != rover_id_) {
            lifecycle_events_.push_back({
                current_time_ns,
                pkt.node_id(),
                LifecycleEventType::PacketRejectedUnsupported,
                "Packet target_rover_id mismatch"
            });
            continue;
        }

        const graph::NodeId nid = pkt.node_id();
        auto it_neighbor = neighbor_states_.find(nid);
        if (it_neighbor == neighbor_states_.end()) {
            // Unsolicited neighbor, ignore
            continue;
        }

        NeighborStateRecord& rec = it_neighbor->second;

        if (options_.filter_options.enable_filtering) {
            if (!filter_.process_packet(pkt, current_time_ns)) {
                lifecycle_events_.push_back({
                    current_time_ns,
                    nid,
                    LifecycleEventType::PacketRejectedDuplicate,
                    "Packet rejected by filter: sequence " + std::to_string(pkt.sequence_number)
                });
                continue;
            }
        } else {
            if (rec.has_received_packet) {
                if (!is_newer_sequence(pkt.sequence_number, rec.last_sequence)) {
                    lifecycle_events_.push_back({
                        current_time_ns,
                        nid,
                        LifecycleEventType::PacketRejectedDuplicate,
                        "Sequence " + std::to_string(pkt.sequence_number) + " rejected vs " + std::to_string(rec.last_sequence)
                    });
                    continue;
                }
            }
        }

        const bool was_disconnected = (rec.status == NeighborLinkStatus::Disconnected);
        rec.received_pose = pkt.pose();
        if (options_.filter_options.enable_filtering) {
            const auto* s = filter_.get_sample(nid);
            rec.received_velocity = s ? s->filtered_velocity : pkt.velocity();
        } else {
            rec.received_velocity = pkt.velocity();
        }
        rec.packet_timestamp_ns = pkt.timestamp_ns;
        rec.reception_timestamp_ns = current_time_ns;
        rec.last_sequence = pkt.sequence_number;
        rec.has_received_packet = true;
        rec.remote_rover_id = pkt.sender_rover_id;

        if (was_disconnected) {
            rec.status = NeighborLinkStatus::Fresh;
            lifecycle_events_.push_back({
                current_time_ns,
                nid,
                LifecycleEventType::LinkReconnected,
                "Link reconnected on seq " + std::to_string(pkt.sequence_number)
            });
        }
    }
}

void DistributedCordNode::update_neighbor_predictions(uint64_t current_time_ns) {
    for (auto& [nid, rec] : neighbor_states_) {
        if (!rec.has_received_packet) {
            rec.status = NeighborLinkStatus::Disconnected;
            rec.current_estimate_pose = rec.received_pose;
            rec.current_estimate_velocity = geometry::Twist2D(0.0, 0.0, 0.0);
            continue;
        }

        const uint64_t age_ns = (current_time_ns >= rec.packet_timestamp_ns)
            ? (current_time_ns - rec.packet_timestamp_ns)
            : 0;
        const double age_s = static_cast<double>(age_ns) * 1e-9;
        rec.last_prediction_dt_s = age_s;

        // 1. Stale edge timeout policy: sever link if age exceeds timeout
        if (age_ns > options_.stale_timeout_ns) {
            if (rec.status != NeighborLinkStatus::Disconnected) {
                rec.status = NeighborLinkStatus::Disconnected;
                lifecycle_events_.push_back({
                    current_time_ns,
                    nid,
                    LifecycleEventType::LinkDisconnected,
                    "Link disconnected: age " + std::to_string(age_s) + "s exceeds timeout"
                });
            }
            rec.current_estimate_pose = rec.received_pose;
            rec.current_estimate_velocity = geometry::Twist2D(0.0, 0.0, 0.0);
            rec.is_prediction_gated = true;
            continue;
        }

        if (options_.filter_options.enable_filtering) {
            geometry::SE2 est_pose;
            geometry::Twist2D est_vel;
            bool is_fallback = false;
            filter_.get_estimated_state(nid, current_time_ns, est_pose, est_vel, is_fallback);

            if (!options_.enable_velocity_prediction || is_fallback) {
                rec.current_estimate_pose = est_pose;
                rec.current_estimate_velocity = geometry::Twist2D(0.0, 0.0, 0.0);
                rec.is_prediction_gated = true;

                if (rec.status == NeighborLinkStatus::Fresh) {
                    rec.status = NeighborLinkStatus::StaleHeld;
                    lifecycle_events_.push_back({
                        current_time_ns,
                        nid,
                        LifecycleEventType::PredictionGatedAge,
                        "Prediction gated: horizon limit or ZOH fallback (age " + std::to_string(age_s) + "s)"
                    });
                }
            } else {
                rec.current_estimate_pose = est_pose;
                rec.current_estimate_velocity = est_vel;
                rec.status = NeighborLinkStatus::Fresh;
                rec.is_prediction_gated = false;
            }
        } else {
            // Baseline delay prediction gating checks
            const bool age_gated = (age_ns > options_.max_prediction_age_ns);

            const geometry::Twist2D delta_twist(
                age_s * rec.received_velocity.vx,
                age_s * rec.received_velocity.vy,
                age_s * rec.received_velocity.omega
            );
            const double trans_disp = std::sqrt(delta_twist.vx * delta_twist.vx + delta_twist.vy * delta_twist.vy);
            const double rot_disp = std::abs(delta_twist.omega);

            const bool motion_gated = (trans_disp > options_.max_prediction_translation_m) ||
                                      (rot_disp > options_.max_prediction_rotation_rad);

            if (!options_.enable_velocity_prediction || age_gated || motion_gated) {
                // Zero-Order Hold fallback: retain last received pose, freeze velocity
                rec.current_estimate_pose = rec.received_pose;
                rec.current_estimate_velocity = geometry::Twist2D(0.0, 0.0, 0.0);
                rec.is_prediction_gated = true;

                if (rec.status == NeighborLinkStatus::Fresh) {
                    rec.status = NeighborLinkStatus::StaleHeld;
                    if (age_gated) {
                        lifecycle_events_.push_back({
                            current_time_ns,
                            nid,
                            LifecycleEventType::PredictionGatedAge,
                            "Prediction gated: age " + std::to_string(age_s) + "s exceeds max prediction age"
                        });
                    } else if (motion_gated) {
                        lifecycle_events_.push_back({
                            current_time_ns,
                            nid,
                            LifecycleEventType::PredictionGatedMotion,
                            "Prediction gated: motion displacement exceeded threshold"
                        });
                    }
                }
            } else {
                // First-order constant-body-twist SE(2) extrapolation:
                // X_pred = X_recv * Exp((tau * xi_recv)^)
                rec.current_estimate_pose = rec.received_pose * geometry::SE2::exp(delta_twist);
                rec.current_estimate_velocity = rec.received_velocity;
                rec.status = NeighborLinkStatus::Fresh;
                rec.is_prediction_gated = false;
            }
        }
    }
}

double DistributedCordNode::evaluate_incident_edges_cost_for_node(
    const graph::NodeId& node_id,
    const geometry::SE2& test_pose,
    const std::unordered_map<graph::NodeId, geometry::SE2>& pose_context) const {

    double total_cost = 0.0;

    for (const auto& [edge_id, edge] : local_graph_.edges()) {
        if (!is_edge_eligible_for_objective(edge)) {
            continue;
        }

        const bool is_from = (edge.from_node == node_id);
        const bool is_to = (edge.to_node == node_id);
        if (!is_from && !is_to) {
            continue;
        }

        // Determine pose of other endpoint
        geometry::SE2 xi;
        geometry::SE2 xj;

        if (edge.type == graph::EdgeType::InterRoverLoopClosure) {
            const graph::NodeId other_node = is_from ? edge.to_node : edge.from_node;
            auto it_remote = neighbor_states_.find(other_node);
            if (it_remote == neighbor_states_.end() || it_remote->second.status == NeighborLinkStatus::Disconnected) {
                // Stale-edge policy: edge deactivated from active objective
                continue;
            }

            if (is_from) {
                xi = test_pose;
                xj = it_remote->second.current_estimate_pose;
            } else {
                xi = it_remote->second.current_estimate_pose;
                xj = test_pose;
            }
        } else {
            // Local edge
            const graph::NodeId other_node = is_from ? edge.to_node : edge.from_node;
            auto it_other = pose_context.find(other_node);
            if (it_other == pose_context.end()) {
                continue;
            }

            if (is_from) {
                xi = test_pose;
                xj = it_other->second;
            } else {
                xi = it_other->second;
                xj = test_pose;
            }
        }

        const geometry::SE2 zij(edge.relative_pose);
        EdgeResidualResult res = compute_edge_residual(xi, xj, zij, edge.information);
        if (res.valid) {
            total_cost += res.cost;
        }
    }

    return total_cost;
}

bool DistributedCordNode::compute_local_gradients(
    std::unordered_map<graph::NodeId, geometry::Twist2D>& out_gradients,
    double* out_max_norm) const {

    out_gradients.clear();

    // Prepare current local poses
    std::unordered_map<graph::NodeId, geometry::SE2> pose_context;
    for (const auto& [nid, state] : local_states_) {
        pose_context[nid] = state.pose;
    }

    const double eps = options_.gradient_finite_diff_step;
    double max_norm = 0.0;

    for (const auto& [nid, state] : local_states_) {
        // Static anchors have zero gradient
        const bool is_anchor = (std::find(options_.fixed_anchors.begin(), options_.fixed_anchors.end(), nid) != options_.fixed_anchors.end());
        if (is_anchor) {
            out_gradients[nid] = geometry::Twist2D(0.0, 0.0, 0.0);
            continue;
        }

        const geometry::SE2& X0 = state.pose;
        geometry::Twist2D grad;

        // Basis e_0 = [1, 0, 0]^T
        {
            geometry::SE2 X_plus = X0 * geometry::SE2::exp(geometry::Twist2D(+eps, 0.0, 0.0));
            geometry::SE2 X_minus = X0 * geometry::SE2::exp(geometry::Twist2D(-eps, 0.0, 0.0));
            double c_plus = evaluate_incident_edges_cost_for_node(nid, X_plus, pose_context);
            double c_minus = evaluate_incident_edges_cost_for_node(nid, X_minus, pose_context);
            grad.vx = (c_plus - c_minus) / (2.0 * eps);
        }

        // Basis e_1 = [0, 1, 0]^T
        {
            geometry::SE2 X_plus = X0 * geometry::SE2::exp(geometry::Twist2D(0.0, +eps, 0.0));
            geometry::SE2 X_minus = X0 * geometry::SE2::exp(geometry::Twist2D(0.0, -eps, 0.0));
            double c_plus = evaluate_incident_edges_cost_for_node(nid, X_plus, pose_context);
            double c_minus = evaluate_incident_edges_cost_for_node(nid, X_minus, pose_context);
            grad.vy = (c_plus - c_minus) / (2.0 * eps);
        }

        // Basis e_2 = [0, 0, 1]^T
        {
            geometry::SE2 X_plus = X0 * geometry::SE2::exp(geometry::Twist2D(0.0, 0.0, +eps));
            geometry::SE2 X_minus = X0 * geometry::SE2::exp(geometry::Twist2D(0.0, 0.0, -eps));
            double c_plus = evaluate_incident_edges_cost_for_node(nid, X_plus, pose_context);
            double c_minus = evaluate_incident_edges_cost_for_node(nid, X_minus, pose_context);
            grad.omega = (c_plus - c_minus) / (2.0 * eps);
        }

        if (!grad.is_valid()) {
            return false;
        }

        max_norm = std::max(max_norm, std::abs(grad.vx));
        max_norm = std::max(max_norm, std::abs(grad.vy));
        max_norm = std::max(max_norm, std::abs(grad.omega));

        out_gradients[nid] = grad;
    }

    if (out_max_norm != nullptr) {
        *out_max_norm = max_norm;
    }
    return true;
}

double DistributedCordNode::compute_local_active_cost() const {
    double total_cost = 0.0;

    std::unordered_map<graph::NodeId, geometry::SE2> pose_context;
    for (const auto& [nid, state] : local_states_) {
        pose_context[nid] = state.pose;
    }

    for (const auto& [edge_id, edge] : local_graph_.edges()) {
        if (!is_edge_eligible_for_objective(edge)) {
            continue;
        }

        geometry::SE2 xi;
        geometry::SE2 xj;

        if (edge.type == graph::EdgeType::InterRoverLoopClosure) {
            const bool from_local = (edge.from_node.rover_id == rover_id_);
            const bool to_local = (edge.to_node.rover_id == rover_id_);

            if (!from_local && !to_local) {
                continue;
            }

            const graph::NodeId local_node = from_local ? edge.from_node : edge.to_node;
            const graph::NodeId remote_node = from_local ? edge.to_node : edge.from_node;

            auto it_remote = neighbor_states_.find(remote_node);
            if (it_remote == neighbor_states_.end() || it_remote->second.status == NeighborLinkStatus::Disconnected) {
                continue; // Stale-edge policy: excluded
            }

            if (from_local) {
                xi = pose_context[local_node];
                xj = it_remote->second.current_estimate_pose;
            } else {
                xi = it_remote->second.current_estimate_pose;
                xj = pose_context[local_node];
            }
        } else {
            // Local edge
            auto it_from = pose_context.find(edge.from_node);
            auto it_to = pose_context.find(edge.to_node);
            if (it_from == pose_context.end() || it_to == pose_context.end()) {
                continue;
            }
            xi = it_from->second;
            xj = it_to->second;
        }

        const geometry::SE2 zij(edge.relative_pose);
        EdgeResidualResult res = compute_edge_residual(xi, xj, zij, edge.information);
        if (res.valid) {
            total_cost += res.cost;
        }
    }

    return total_cost;
}

void DistributedCordNode::transmit_boundary_states(uint64_t current_time_ns) {
    for (const auto& local_boundary_nid : local_boundary_nodes_) {
        auto it_state = local_states_.find(local_boundary_nid);
        if (it_state == local_states_.end()) {
            continue;
        }

        auto it_target = boundary_node_to_remote_rover_.find(local_boundary_nid);
        if (it_target == boundary_node_to_remote_rover_.end()) {
            continue;
        }
        const uint32_t target_rover = it_target->second;

        const uint32_t seq = ++local_outbound_seq_[local_boundary_nid];

        BoundaryStatePacket pkt;
        pkt.protocol_version = BOUNDARY_PROTOCOL_VERSION;
        pkt.sender_rover_id = static_cast<uint16_t>(rover_id_);
        pkt.target_rover_id = static_cast<uint16_t>(target_rover);
        pkt.node_local_index = local_boundary_nid.local_index;
        pkt.sequence_number = seq;
        pkt.timestamp_ns = current_time_ns;
        pkt.x = it_state->second.pose.x();
        pkt.y = it_state->second.pose.y();
        pkt.theta = it_state->second.pose.theta();
        pkt.vx = it_state->second.velocity.vx;
        pkt.vy = it_state->second.velocity.vy;
        pkt.omega = it_state->second.velocity.omega;

        network_.get().send_packet(pkt);
    }
}

bool DistributedCordNode::step(uint64_t current_time_ns) {
    if (!std::isfinite(options_.timestep) || options_.timestep <= 0.0 ||
        !options_.default_inertia.is_valid() || !options_.default_damping.is_valid()) {
        return false;
    }

    // 1. Ingest arriving packets from network
    process_inbound_packets(current_time_ns);

    // 2. Refresh neighbor status and update SE(2) velocity predictions
    update_neighbor_predictions(current_time_ns);

    // 3. Compute local Riemannian gradients
    std::unordered_map<graph::NodeId, geometry::Twist2D> gradients;
    if (!compute_local_gradients(gradients)) {
        return false;
    }

    // 4. Step semi-implicit CORD Lie integrator for local nodes
    const double h = options_.timestep;
    const NodeInertia& inertia = options_.default_inertia;

    NodeDamping damping = options_.default_damping;
    if (options_.filter_options.enable_adaptive_damping) {
        double max_delay_s = 0.0;
        for (const auto& [nid, rec] : neighbor_states_) {
            if (rec.has_received_packet && rec.status != NeighborLinkStatus::Disconnected) {
                const double age_s = (current_time_ns >= rec.packet_timestamp_ns)
                    ? static_cast<double>(current_time_ns - rec.packet_timestamp_ns) * 1e-9
                    : 0.0;
                max_delay_s = std::max(max_delay_s, age_s);
            }
        }
        const double mult = filter_.compute_damping_multiplier(max_delay_s);
        damping.damping_x *= mult;
        damping.damping_y *= mult;
        damping.damping_theta *= mult;
    }

    for (auto& [nid, state] : local_states_) {
        const bool is_anchor = (std::find(options_.fixed_anchors.begin(), options_.fixed_anchors.end(), nid) != options_.fixed_anchors.end());
        if (is_anchor) {
            state.velocity = geometry::Twist2D(0.0, 0.0, 0.0);
            continue;
        }

        const geometry::Twist2D& xi = state.velocity;
        const geometry::Twist2D& g = gradients[nid];

        // 1. Momentum: p = M * xi
        const geometry::Twist2D momentum(
            inertia.mass_x * xi.vx,
            inertia.mass_y * xi.vy,
            inertia.inertia_theta * xi.omega
        );

        // 2. Co-adjoint action: ad_{xi}^*(p)
        const geometry::Twist2D coadj = geometry::SE2::coadjoint(xi, momentum);

        // 3. Force: F = ad_{xi}^*(M * xi) - grad C(X)
        const double f_x = coadj.vx - g.vx;
        const double f_y = coadj.vy - g.vy;
        const double f_theta = coadj.omega - g.omega;

        // 4. Semi-implicit velocity update: (I + h * M^{-1} * D) * xi^{k+1} = xi^k + h * M^{-1} * F
        const double denom_x = 1.0 + h * (damping.damping_x / inertia.mass_x);
        const double denom_y = 1.0 + h * (damping.damping_y / inertia.mass_y);
        const double denom_theta = 1.0 + h * (damping.damping_theta / inertia.inertia_theta);

        const double next_vx = (xi.vx + h * (f_x / inertia.mass_x)) / denom_x;
        const double next_vy = (xi.vy + h * (f_y / inertia.mass_y)) / denom_y;
        const double next_omega = (xi.omega + h * (f_theta / inertia.inertia_theta)) / denom_theta;

        const geometry::Twist2D next_xi(next_vx, next_vy, next_omega);
        if (!next_xi.is_valid()) {
            return false;
        }

        // 5. Position retraction on SE(2): X_i^{k+1} = X_i^k * exp((h * xi^{k+1})^\wedge)
        const geometry::Twist2D delta_twist(h * next_vx, h * next_vy, h * next_omega);
        const geometry::SE2 next_pose = state.pose * geometry::SE2::exp(delta_twist);
        if (!next_pose.is_valid()) {
            return false;
        }

        state.velocity = next_xi;
        state.pose = next_pose;
    }

    // 5. Transmit updated boundary states to peer rovers
    transmit_boundary_states(current_time_ns);

    return true;
}

} // namespace astromesh::optimization
