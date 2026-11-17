#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/optimization/boundary_packet.hpp"
#include "astromesh/optimization/cord_integrator.hpp"
#include "astromesh/optimization/distributed_cord_node.hpp"
#include "astromesh/optimization/pose_graph_objective.hpp"
#include "astromesh/optimization/simulated_mesh_network.hpp"
#include "astromesh/optimization/trajectory_metrics.hpp"
#include "support/synthetic_pose_graph.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

using namespace astromesh;
using namespace astromesh::geometry;
using namespace astromesh::graph;
using namespace astromesh::optimization;
using namespace astromesh::tests::support;

struct FilteredCordNode {
    uint32_t rover_id;
    graph::PoseGraph graph;
    std::unordered_map<graph::NodeId, NodeDynamicsState> states;
    std::vector<graph::NodeId> fixed_anchors;
    double timestep{0.01};
    NodeInertia base_inertia{5.0, 5.0, 5.0};
    NodeDamping base_damping{20.0, 20.0, 20.0};

    // Filter options
    bool enable_filter{true};
    bool enable_pred{true};
    double smooth_alpha{0.5};
    double max_v{1.5};
    double max_w{1.0};
    double max_horizon_s{0.03}; // 30 ms horizon limit!
    bool enable_adaptive_damping{true};
    double damping_alpha{15.0}; // s^-1
    double max_damping_mult{5.0};

    struct NeighborRecord {
        graph::NodeId id;
        uint32_t rover;
        SE2 recv_pose;
        Twist2D recv_vel;
        Twist2D filtered_vel;
        uint64_t last_pkt_time{0};
        uint32_t last_seq{0};
        bool has_pkt{false};
        SE2 current_est_pose;
        Twist2D current_est_vel;
    };
    std::unordered_map<graph::NodeId, NeighborRecord> neighbors;
    std::vector<graph::NodeId> boundary_nodes;
    std::unordered_map<graph::NodeId, uint32_t> boundary_targets;
    std::unordered_map<graph::NodeId, uint32_t> outbound_seq;
};

void run_test_with_horizon(double max_h_s) {
    std::cout << "\n>>> Testing Filter + Pred + AdaptDamping with max_prediction_horizon = " << (max_h_s * 1000) << " ms <<<\n";

    for (double delay_ms : {0.0, 20.0, 50.0, 100.0, 200.0, 500.0}) {
        MultiRoverDataset ds = generate_multi_rover_circular_loop(2, 10, 5.0, 0.08, 0.04, 0.10, 0.05, 42);

        SimulatedNetworkConfig net_cfg;
        net_cfg.latency_ns = static_cast<uint64_t>(delay_ms * 1e6);
        net_cfg.packet_loss_rate = 0.0;
        SimulatedMeshNetwork network(net_cfg);

        std::map<uint32_t, FilteredCordNode> rovers;
        for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
            FilteredCordNode& node = rovers[r];
            node.rover_id = r;
            node.graph = ds.subgraphs.at(r);
            node.states = ds.initial_states_by_rover.at(r);
            node.fixed_anchors = ds.fixed_anchors;
            node.enable_filter = true;
            node.enable_pred = true;
            node.max_horizon_s = max_h_s;
            node.enable_adaptive_damping = true;
            node.damping_alpha = 15.0;

            for (const auto& [nid, p] : node.states) {
                for (const auto& [eid, e] : node.graph.edges()) {
                    if (e.type == EdgeType::InterRoverLoopClosure && (e.from_node == nid || e.to_node == nid)) {
                        node.boundary_nodes.push_back(nid);
                        NodeId remote = (e.from_node == nid) ? e.to_node : e.from_node;
                        node.boundary_targets[nid] = remote.rover_id;
                        node.neighbors[remote].id = remote;
                        node.neighbors[remote].rover = remote.rover_id;
                        node.neighbors[remote].recv_pose = ds.initial_poses.at(remote);
                        node.neighbors[remote].recv_vel = Twist2D(0, 0, 0);
                        node.neighbors[remote].filtered_vel = Twist2D(0, 0, 0);
                        node.neighbors[remote].current_est_pose = ds.initial_poses.at(remote);
                        node.neighbors[remote].current_est_vel = Twist2D(0, 0, 0);
                        node.neighbors[remote].has_pkt = true;
                    }
                }
            }
        }

        double init_cost = evaluate_pose_graph_objective(ds.full_graph, ds.initial_poses).total_cost;
        double max_w_seen = 0.0;

        for (size_t step = 0; step < 100; ++step) {
            uint64_t sim_time = step * 10'000'000ULL;
            network.set_current_time_ns(sim_time);

            for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
                FilteredCordNode& rn = rovers[r];
                auto pkts = network.receive_packets(static_cast<uint16_t>(r));
                for (const auto& pkt : pkts) {
                    if (!pkt.is_valid()) continue;
                    NodeId nid = pkt.node_id();
                    auto it = rn.neighbors.find(nid);
                    if (it == rn.neighbors.end()) continue;
                    auto& rec = it->second;

                    if (rec.has_pkt && !is_newer_sequence(pkt.sequence_number, rec.last_seq)) {
                        continue;
                    }

                    rec.recv_pose = pkt.pose();
                    Twist2D raw_vel = pkt.velocity();

                    double a = 0.5;
                    rec.filtered_vel = Twist2D(
                        (1.0 - a) * rec.filtered_vel.vx + a * raw_vel.vx,
                        (1.0 - a) * rec.filtered_vel.vy + a * raw_vel.vy,
                        (1.0 - a) * rec.filtered_vel.omega + a * raw_vel.omega
                    );
                    double v_norm = std::sqrt(rec.filtered_vel.vx * rec.filtered_vel.vx + rec.filtered_vel.vy * rec.filtered_vel.vy);
                    if (v_norm > rn.max_v) {
                        rec.filtered_vel.vx *= (rn.max_v / v_norm);
                        rec.filtered_vel.vy *= (rn.max_v / v_norm);
                    }
                    rec.filtered_vel.omega = std::clamp(rec.filtered_vel.omega, -rn.max_w, rn.max_w);

                    if (v_norm < 1e-3) { rec.filtered_vel.vx = 0; rec.filtered_vel.vy = 0; }
                    if (std::abs(rec.filtered_vel.omega) < 1e-3) { rec.filtered_vel.omega = 0; }

                    rec.recv_vel = raw_vel;
                    rec.last_pkt_time = pkt.timestamp_ns;
                    rec.last_seq = pkt.sequence_number;
                    rec.has_pkt = true;
                }
            }

            for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
                FilteredCordNode& rn = rovers[r];
                for (auto& [nid, rec] : rn.neighbors) {
                    if (!rec.has_pkt) continue;
                    uint64_t age_ns = (sim_time >= rec.last_pkt_time) ? (sim_time - rec.last_pkt_time) : 0;
                    double age_s = age_ns * 1e-9;

                    if (age_s > 2.0) {
                        rec.current_est_pose = rec.recv_pose;
                        rec.current_est_vel = Twist2D(0, 0, 0);
                        continue;
                    }

                    if (!rn.enable_pred || age_s > rn.max_horizon_s) {
                        // Fallback to ZOH when age exceeds permitted prediction horizon!
                        rec.current_est_pose = rec.recv_pose;
                        rec.current_est_vel = Twist2D(0, 0, 0);
                    } else {
                        double tau = age_s;
                        Twist2D dt(tau * rec.filtered_vel.vx, tau * rec.filtered_vel.vy, tau * rec.filtered_vel.omega);
                        rec.current_est_pose = rec.recv_pose * SE2::exp(dt);
                        rec.current_est_vel = rec.filtered_vel;
                    }
                }
            }

            for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
                FilteredCordNode& rn = rovers[r];

                double max_delay_s = 0.0;
                for (const auto& [nid, rec] : rn.neighbors) {
                    if (rec.has_pkt) {
                        double age_s = (sim_time >= rec.last_pkt_time) ? (sim_time - rec.last_pkt_time) * 1e-9 : 0.0;
                        max_delay_s = std::max(max_delay_s, age_s);
                    }
                }

                std::unordered_map<NodeId, SE2> pose_ctx;
                for (const auto& [nid, st] : rn.states) pose_ctx[nid] = st.pose;
                for (const auto& [nid, rec] : rn.neighbors) pose_ctx[nid] = rec.current_est_pose;

                std::unordered_map<NodeId, Twist2D> grads;
                for (const auto& [nid, st] : rn.states) {
                    const double eps = 1e-6;
                    Twist2D g(0, 0, 0);

                    for (int comp = 0; comp < 3; ++comp) {
                        Twist2D dp_plus(0, 0, 0), dp_minus(0, 0, 0);
                        if (comp == 0) { dp_plus.vx = eps; dp_minus.vx = -eps; }
                        else if (comp == 1) { dp_plus.vy = eps; dp_minus.vy = -eps; }
                        else { dp_plus.omega = eps; dp_minus.omega = -eps; }

                        SE2 test_plus = st.pose * SE2::exp(dp_plus);
                        SE2 test_minus = st.pose * SE2::exp(dp_minus);

                        double cost_plus = 0.0, cost_minus = 0.0;
                        for (const auto& [eid, e] : rn.graph.edges()) {
                            if (!is_edge_eligible_for_objective(e)) continue;
                            if (e.from_node != nid && e.to_node != nid) continue;

                            SE2 xi_p = (e.from_node == nid) ? test_plus : pose_ctx[e.from_node];
                            SE2 xj_p = (e.to_node == nid) ? test_plus : pose_ctx[e.to_node];
                            cost_plus += compute_edge_residual(xi_p, xj_p, SE2(e.relative_pose), e.information).cost;

                            SE2 xi_m = (e.from_node == nid) ? test_minus : pose_ctx[e.from_node];
                            SE2 xj_m = (e.to_node == nid) ? test_minus : pose_ctx[e.to_node];
                            cost_minus += compute_edge_residual(xi_m, xj_m, SE2(e.relative_pose), e.information).cost;
                        }

                        if (comp == 0) g.vx = (cost_plus - cost_minus) / (2.0 * eps);
                        else if (comp == 1) g.vy = (cost_plus - cost_minus) / (2.0 * eps);
                        else g.omega = (cost_plus - cost_minus) / (2.0 * eps);
                    }
                    grads[nid] = g;
                }

                double h = rn.timestep;
                for (auto& [nid, st] : rn.states) {
                    if (std::find(rn.fixed_anchors.begin(), rn.fixed_anchors.end(), nid) != rn.fixed_anchors.end()) {
                        st.velocity = Twist2D(0, 0, 0);
                        continue;
                    }

                    NodeDamping damp = rn.base_damping;
                    if (rn.enable_adaptive_damping) {
                        double mult = std::min(rn.max_damping_mult, 1.0 + rn.damping_alpha * max_delay_s);
                        damp.damping_x *= mult;
                        damp.damping_y *= mult;
                        damp.damping_theta *= mult;
                    }

                    Twist2D xi = st.velocity;
                    Twist2D g = grads[nid];
                    Twist2D p(rn.base_inertia.mass_x * xi.vx, rn.base_inertia.mass_y * xi.vy, rn.base_inertia.inertia_theta * xi.omega);
                    Twist2D coadj = SE2::coadjoint(xi, p);

                    double fx = coadj.vx - g.vx;
                    double fy = coadj.vy - g.vy;
                    double ftheta = coadj.omega - g.omega;

                    double den_x = 1.0 + h * (damp.damping_x / rn.base_inertia.mass_x);
                    double den_y = 1.0 + h * (damp.damping_y / rn.base_inertia.mass_y);
                    double den_th = 1.0 + h * (damp.damping_theta / rn.base_inertia.inertia_theta);

                    double n_vx = (xi.vx + h * (fx / rn.base_inertia.mass_x)) / den_x;
                    double n_vy = (xi.vy + h * (fy / rn.base_inertia.mass_y)) / den_y;
                    double n_w = (xi.omega + h * (ftheta / rn.base_inertia.inertia_theta)) / den_th;

                    st.velocity = Twist2D(n_vx, n_vy, n_w);
                    st.pose = st.pose * SE2::exp(Twist2D(h * n_vx, h * n_vy, h * n_w));
                    max_w_seen = std::max(max_w_seen, std::abs(n_w));
                }

                for (const auto& bn : rn.boundary_nodes) {
                    BoundaryStatePacket pkt;
                    pkt.protocol_version = BOUNDARY_PROTOCOL_VERSION;
                    pkt.sender_rover_id = static_cast<uint16_t>(r);
                    pkt.target_rover_id = static_cast<uint16_t>(rn.boundary_targets[bn]);
                    pkt.node_local_index = bn.local_index;
                    pkt.sequence_number = ++rn.outbound_seq[bn];
                    pkt.timestamp_ns = sim_time;
                    pkt.x = rn.states[bn].pose.x();
                    pkt.y = rn.states[bn].pose.y();
                    pkt.theta = rn.states[bn].pose.theta();
                    pkt.vx = rn.states[bn].velocity.vx;
                    pkt.vy = rn.states[bn].velocity.vy;
                    pkt.omega = rn.states[bn].velocity.omega;
                    network.send_packet(pkt);
                }
            }
        }

        std::unordered_map<NodeId, SE2> final_poses;
        for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
            for (const auto& [nid, st] : rovers[r].states) final_poses[nid] = st.pose;
        }

        double fin_cost = evaluate_pose_graph_objective(ds.full_graph, final_poses).total_cost;
        double red = 100.0 * (init_cost - fin_cost) / init_cost;
        TrajectoryEvaluation te = evaluate_trajectory(final_poses, ds.ground_truth_poses);

        std::cout << "Delay " << std::setw(3) << delay_ms << "ms | "
                  << "Final Cost=" << std::setw(8) << fin_cost 
                  << " (" << std::setw(6) << red << "%) | "
                  << "Aligned ATE=" << std::setw(6) << te.ate_aligned.translation.rmse << "m | "
                  << "Max |w|=" << std::setw(5) << max_w_seen << " rad/s\n";
    }
}

int main() {
    run_test_with_horizon(0.03); // 30 ms limit
    run_test_with_horizon(0.02); // 20 ms limit
    return 0;
}
