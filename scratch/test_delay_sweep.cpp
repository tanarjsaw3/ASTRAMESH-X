#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/optimization/boundary_packet.hpp"
#include "astromesh/optimization/cord_integrator.hpp"
#include "astromesh/optimization/distributed_cord_node.hpp"
#include "astromesh/optimization/pose_graph_objective.hpp"
#include "astromesh/optimization/simulated_mesh_network.hpp"
#include "astromesh/optimization/trajectory_metrics.hpp"
#include "support/synthetic_pose_graph.hpp"

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

struct TestResult {
    double delay_ms;
    std::string mode;
    double init_cost;
    double final_cost;
    double red_pct;
    double aligned_ate;
    double max_omega;
    bool stable;
};

TestResult run_test(
    double delay_ms,
    bool enable_pred,
    bool enable_filter,
    double smooth_alpha,
    double v_max,
    double omega_max,
    bool enable_adaptive_damping,
    double damping_alpha) {

    MultiRoverDataset ds = generate_multi_rover_circular_loop(2, 10, 5.0, 0.08, 0.04, 0.10, 0.05, 42);

    SimulatedNetworkConfig net_cfg;
    net_cfg.latency_ns = static_cast<uint64_t>(delay_ms * 1e6);
    net_cfg.packet_loss_rate = 0.0;
    SimulatedMeshNetwork network(net_cfg);

    DistributedCordOptions dist_opts;
    dist_opts.default_inertia = NodeInertia(5.0, 5.0, 5.0);
    dist_opts.default_damping = NodeDamping(20.0, 20.0, 20.0);
    dist_opts.enable_velocity_prediction = enable_pred;
    dist_opts.timestep = 0.01;
    dist_opts.fixed_anchors = ds.fixed_anchors;

    ObjectiveEvaluationResult init_obj = evaluate_pose_graph_objective(ds.full_graph, ds.initial_poses);
    double init_cost = init_obj.total_cost;

    std::map<uint32_t, std::unique_ptr<DistributedCordNode>> rovers;
    for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
        rovers[r] = std::make_unique<DistributedCordNode>(r, ds.subgraphs.at(r), ds.initial_states_by_rover.at(r), dist_opts, network);
    }

    for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
        for (const auto& nid : rovers[r]->local_boundary_nodes()) {
            for (uint32_t peer = 1; peer <= ds.num_rovers; ++peer) {
                if (peer == r) continue;
                rovers[peer]->inject_neighbor_state(nid, r, ds.initial_poses.at(nid), Twist2D(0, 0, 0), 0);
            }
        }
    }

    double max_omega_seen = 0.0;
    bool all_ok = true;

    for (size_t step = 0; step < 100; ++step) {
        uint64_t sim_time = step * 10'000'000ULL;
        network.set_current_time_ns(sim_time);

        for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
            bool ok = rovers[r]->step(sim_time);
            if (!ok) all_ok = false;
            for (const auto& [nid, st] : rovers[r]->local_states()) {
                max_omega_seen = std::max(max_omega_seen, std::abs(st.velocity.omega));
            }
        }
    }

    std::unordered_map<NodeId, SE2> final_poses;
    for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
        for (const auto& [nid, st] : rovers[r]->local_states()) {
            final_poses[nid] = st.pose;
        }
    }

    ObjectiveEvaluationResult fin_obj = evaluate_pose_graph_objective(ds.full_graph, final_poses);
    double fin_cost = fin_obj.valid ? fin_obj.total_cost : 999999.0;
    double red = 100.0 * (init_cost - fin_cost) / init_cost;

    TrajectoryEvaluation te = evaluate_trajectory(final_poses, ds.ground_truth_poses);

    TestResult tr;
    tr.delay_ms = delay_ms;
    tr.init_cost = init_cost;
    tr.final_cost = fin_cost;
    tr.red_pct = red;
    tr.aligned_ate = te.ate_aligned.translation.rmse;
    tr.max_omega = max_omega_seen;
    tr.stable = all_ok && std::isfinite(fin_cost) && fin_cost < init_cost && tr.aligned_ate < 0.50;
    return tr;
}

int main() {
    std::cout << "Testing baseline without modifications...\n";
    for (double d : {0.0, 20.0, 50.0, 100.0, 200.0, 500.0}) {
        auto r_pred = run_test(d, true, false, 1.0, 2.0, 1.5, false, 0.0);
        auto r_zoh  = run_test(d, false, false, 1.0, 2.0, 1.5, false, 0.0);
        std::cout << "Delay " << std::setw(3) << d << "ms | "
                  << "Pred: cost=" << std::setw(8) << r_pred.final_cost << " (" << std::setw(6) << r_pred.red_pct << "%), ATE=" << r_pred.aligned_ate << ", max_w=" << r_pred.max_omega << " | "
                  << "ZOH: cost=" << std::setw(8) << r_zoh.final_cost << " (" << std::setw(6) << r_zoh.red_pct << "%), ATE=" << r_zoh.aligned_ate << ", max_w=" << r_zoh.max_omega << "\n";
    }
    return 0;
}
