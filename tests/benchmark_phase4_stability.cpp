#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/optimization/boundary_filter.hpp"
#include "astromesh/optimization/boundary_packet.hpp"
#include "astromesh/optimization/cord_integrator.hpp"
#include "astromesh/optimization/distributed_cord_node.hpp"
#include "astromesh/optimization/pose_graph_objective.hpp"
#include "astromesh/optimization/simulated_mesh_network.hpp"
#include "astromesh/optimization/trajectory_metrics.hpp"
#include "support/synthetic_pose_graph.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace astromesh;
using namespace astromesh::geometry;
using namespace astromesh::graph;
using namespace astromesh::optimization;
using namespace astromesh::tests::support;

struct StabilityBenchmarkRecord {
    std::string experiment_group;
    std::string mode_label;
    std::string dataset_name;
    uint32_t num_rovers{2};
    uint32_t seed{42};
    double nominal_latency_ms{0.0};
    double packet_loss_rate_cfg{0.0};
    bool prediction_enabled{true};
    bool filter_enabled{false};
    bool adaptive_damping_enabled{false};
    double max_prediction_horizon_ms{30.0};
    double adaptive_damping_alpha{15.0};

    double initial_global_cost{0.0};
    double final_global_cost{0.0};
    double final_active_cost{0.0};
    double cost_reduction_pct{0.0};

    double init_ate_aligned_rmse{0.0};
    double final_ate_aligned_rmse{0.0};
    double max_linear_velocity{0.0};
    double max_angular_velocity{0.0};

    double mean_boundary_pred_error_m{0.0};
    size_t fallback_count{0};
    size_t packets_sent{0};
    size_t packets_delivered{0};
    size_t wire_bytes_sent{0};

    double runtime_wall_ms{0.0};
    std::string status{"Completed"};
};

struct StabilitySimContext {
    SimulatedNetworkConfig net_cfg;
    DistributedCordOptions dist_opts;
    size_t total_steps{100};
    uint64_t step_dt_ns{10'000'000}; // 10 ms
    bool partition_test{false};
    size_t partition_start_step{30};
    size_t partition_end_step{70};

    StabilitySimContext() {
        dist_opts.default_inertia = NodeInertia(5.0, 5.0, 5.0);
        dist_opts.default_damping = NodeDamping(20.0, 20.0, 20.0);
        dist_opts.timestep = 0.01;
    }
};

StabilityBenchmarkRecord run_stability_trial(
    const MultiRoverDataset& ds,
    const StabilitySimContext& ctx,
    const std::string& group_name,
    const std::string& mode_name) {

    StabilityBenchmarkRecord rec;
    rec.experiment_group = group_name;
    rec.mode_label = mode_name;
    rec.dataset_name = ds.name;
    rec.num_rovers = ds.num_rovers;
    rec.seed = ds.seed;
    rec.nominal_latency_ms = static_cast<double>(ctx.net_cfg.latency_ns) * 1e-6;
    rec.packet_loss_rate_cfg = ctx.net_cfg.packet_loss_rate;
    rec.prediction_enabled = ctx.dist_opts.enable_velocity_prediction;
    rec.filter_enabled = ctx.dist_opts.filter_options.enable_filtering;
    rec.adaptive_damping_enabled = ctx.dist_opts.filter_options.enable_adaptive_damping;
    rec.max_prediction_horizon_ms = ctx.dist_opts.filter_options.max_prediction_horizon_s * 1000.0;
    rec.adaptive_damping_alpha = ctx.dist_opts.filter_options.adaptive_damping_alpha;

    // Initial metrics
    TrajectoryEvaluation init_ate = evaluate_trajectory(ds.initial_poses, ds.ground_truth_poses);
    rec.init_ate_aligned_rmse = init_ate.ate_aligned.translation.rmse;

    ObjectiveEvaluationResult init_obj = evaluate_pose_graph_objective(ds.full_graph, ds.initial_poses);
    rec.initial_global_cost = init_obj.valid ? init_obj.total_cost : 0.0;

    SimulatedMeshNetwork network(ctx.net_cfg);

    std::map<uint32_t, std::unique_ptr<DistributedCordNode>> rovers;
    for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
        DistributedCordOptions opts = ctx.dist_opts;
        opts.timestep = static_cast<double>(ctx.step_dt_ns) * 1e-9;
        opts.fixed_anchors = ds.fixed_anchors;

        rovers[r] = std::make_unique<DistributedCordNode>(r, ds.subgraphs.at(r), ds.initial_states_by_rover.at(r), opts, network);
    }

    // Initial boundary injection
    for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
        for (const auto& nid : rovers[r]->local_boundary_nodes()) {
            for (uint32_t peer = 1; peer <= ds.num_rovers; ++peer) {
                if (peer == r) continue;
                rovers[peer]->inject_neighbor_state(nid, r, ds.initial_poses.at(nid), Twist2D(0.0, 0.0, 0.0), 0);
            }
        }
    }

    double total_pred_err = 0.0;
    size_t pred_samples = 0;
    double max_lin_vel = 0.0;
    double max_ang_vel = 0.0;

    const auto t_start = std::chrono::high_resolution_clock::now();

    for (size_t step = 0; step < ctx.total_steps; ++step) {
        uint64_t sim_time = step * ctx.step_dt_ns;
        network.set_current_time_ns(sim_time);

        if (ctx.partition_test) {
            if (step == ctx.partition_start_step) {
                for (uint32_t r1 = 1; r1 <= ds.num_rovers; ++r1) {
                    for (uint32_t r2 = r1 + 1; r2 <= ds.num_rovers; ++r2) {
                        network.set_link_state(static_cast<uint16_t>(r1), static_cast<uint16_t>(r2), false);
                    }
                }
            } else if (step == ctx.partition_end_step) {
                for (uint32_t r1 = 1; r1 <= ds.num_rovers; ++r1) {
                    for (uint32_t r2 = r1 + 1; r2 <= ds.num_rovers; ++r2) {
                        network.set_link_state(static_cast<uint16_t>(r1), static_cast<uint16_t>(r2), true);
                    }
                }
            }
        }

        for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
            bool ok = rovers[r]->step(sim_time);
            if (!ok) {
                rec.status = "NumericalFailure";
            }
            for (const auto& [nid, st] : rovers[r]->local_states()) {
                if (!std::isfinite(st.pose.x()) || !std::isfinite(st.pose.y()) || !std::isfinite(st.pose.theta()) ||
                    !std::isfinite(st.velocity.vx) || !std::isfinite(st.velocity.vy) || !std::isfinite(st.velocity.omega)) {
                    rec.status = "NonFiniteState";
                }
                double v_norm = std::sqrt(st.velocity.vx * st.velocity.vx + st.velocity.vy * st.velocity.vy);
                max_lin_vel = std::max(max_lin_vel, v_norm);
                max_ang_vel = std::max(max_ang_vel, std::abs(st.velocity.omega));
            }
        }

        // Measure prediction error against ground truth
        for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
            for (const auto& [nid, rec_n] : rovers[r]->neighbor_states()) {
                if (rec_n.status == NeighborLinkStatus::Fresh) {
                    uint32_t rem_r = nid.rover_id;
                    if (rovers.find(rem_r) != rovers.end() && rovers[rem_r]->local_states().find(nid) != rovers[rem_r]->local_states().end()) {
                        SE2 true_p = rovers[rem_r]->local_states().at(nid).pose;
                        double dx = rec_n.current_estimate_pose.x() - true_p.x();
                        double dy = rec_n.current_estimate_pose.y() - true_p.y();
                        total_pred_err += std::sqrt(dx * dx + dy * dy);
                        pred_samples++;
                    }
                }
            }
        }
    }

    const auto t_end = std::chrono::high_resolution_clock::now();
    rec.runtime_wall_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    std::unordered_map<NodeId, SE2> final_poses;
    rec.final_active_cost = 0.0;
    for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
        rec.final_active_cost += rovers[r]->compute_local_active_cost();
        for (const auto& [nid, st] : rovers[r]->local_states()) {
            final_poses[nid] = st.pose;
        }
        rec.fallback_count += rovers[r]->filter().fallback_count();
    }

    ObjectiveEvaluationResult fin_obj = evaluate_pose_graph_objective(ds.full_graph, final_poses);
    rec.final_global_cost = fin_obj.valid ? fin_obj.total_cost : 999999.0;
    if (rec.initial_global_cost > 1e-9) {
        rec.cost_reduction_pct = 100.0 * (rec.initial_global_cost - rec.final_global_cost) / rec.initial_global_cost;
    }

    TrajectoryEvaluation fin_ate = evaluate_trajectory(final_poses, ds.ground_truth_poses);
    rec.final_ate_aligned_rmse = fin_ate.ate_aligned.translation.rmse;
    rec.max_linear_velocity = max_lin_vel;
    rec.max_angular_velocity = max_ang_vel;

    if (pred_samples > 0) {
        rec.mean_boundary_pred_error_m = total_pred_err / static_cast<double>(pred_samples);
    }

    rec.packets_sent = network.statistics().packets_sent;
    rec.packets_delivered = network.statistics().packets_delivered;
    rec.wire_bytes_sent = network.statistics().transport_bytes_sent;

    if (rec.status == "Completed") {
        if (rec.cost_reduction_pct < 0.0 || max_ang_vel > 20.0) {
            rec.status = "DivergedOrIncreased";
        }
    }

    return rec;
}

void write_header(std::ostream& os) {
    os << "experiment_group,mode_label,dataset_name,num_rovers,seed,nominal_latency_ms,packet_loss_rate_cfg,"
       << "prediction_enabled,filter_enabled,adaptive_damping_enabled,max_prediction_horizon_ms,adaptive_damping_alpha,"
       << "initial_global_cost,final_global_cost,final_active_cost,cost_reduction_pct,init_ate_aligned_rmse,final_ate_aligned_rmse,"
       << "max_linear_velocity,max_angular_velocity,mean_boundary_pred_error_m,fallback_count,packets_sent,packets_delivered,"
       << "wire_bytes_sent,runtime_wall_ms,status\n";
}

void write_row(std::ostream& os, const StabilityBenchmarkRecord& r) {
    os << r.experiment_group << ","
       << r.mode_label << ","
       << r.dataset_name << ","
       << r.num_rovers << ","
       << r.seed << ","
       << std::fixed << std::setprecision(1) << r.nominal_latency_ms << ","
       << std::setprecision(3) << r.packet_loss_rate_cfg << ","
       << (r.prediction_enabled ? "true" : "false") << ","
       << (r.filter_enabled ? "true" : "false") << ","
       << (r.adaptive_damping_enabled ? "true" : "false") << ","
       << std::setprecision(1) << r.max_prediction_horizon_ms << ","
       << std::setprecision(1) << r.adaptive_damping_alpha << ","
       << std::setprecision(4) << r.initial_global_cost << ","
       << std::setprecision(4) << r.final_global_cost << ","
       << std::setprecision(4) << r.final_active_cost << ","
       << std::setprecision(2) << r.cost_reduction_pct << ","
       << std::setprecision(4) << r.init_ate_aligned_rmse << ","
       << std::setprecision(4) << r.final_ate_aligned_rmse << ","
       << std::setprecision(4) << r.max_linear_velocity << ","
       << std::setprecision(4) << r.max_angular_velocity << ","
       << std::setprecision(4) << r.mean_boundary_pred_error_m << ","
       << r.fallback_count << ","
       << r.packets_sent << ","
       << r.packets_delivered << ","
       << r.wire_bytes_sent << ","
       << std::setprecision(2) << r.runtime_wall_ms << ","
       << r.status << "\n";
}

int main(int argc, char** argv) {
    std::cout << "==============================================================================\n";
    std::cout << " AstroMesh-X Phase 4.1: Delay-Tolerant Boundary Filter & Stability Benchmark\n";
    std::cout << "==============================================================================\n";

    std::string csv_file = "benchmark_phase4_stability_results.csv";
    if (argc > 1) {
        csv_file = argv[1];
    }

    std::vector<StabilityBenchmarkRecord> records;

    // ---------------------------------------------------------------------------
    // 1. LATENCY ABLATION MATRIX (Delays: 0, 20, 50, 100, 200, 500 ms)
    // Modes compared:
    //   A. Baseline_Pred_Unmitigated: Pred ON, Filter OFF, AdaptDamp OFF
    //   B. Baseline_ZOH_Unmitigated:  Pred OFF, Filter OFF, AdaptDamp OFF
    //   C. Filter_Only:               Pred ON, Filter ON, AdaptDamp OFF
    //   D. Mitigated_Full:            Pred ON, Filter ON, AdaptDamp ON (alpha=15)
    //   E. ZOH_AdaptiveDamping:       Pred OFF, Filter OFF, AdaptDamp ON (alpha=15)
    // ---------------------------------------------------------------------------
    std::cout << "\n>>> Suite 1: Full Latency Ablation Sweep (5 modes x 6 delays = 30 trials)...\n";
    MultiRoverDataset ds_loop = generate_multi_rover_circular_loop(2, 10, 5.0, 0.08, 0.04, 0.10, 0.05, 42);
    const std::vector<uint64_t> delays_ns = {0, 20'000'000, 50'000'000, 100'000'000, 200'000'000, 500'000'000};

    struct ModeConfig {
        std::string label;
        bool pred;
        bool filter;
        bool adapt_damp;
    };

    const std::vector<ModeConfig> modes = {
        {"Baseline_Pred_Unmitigated", true,  false, false},
        {"Baseline_ZOH_Unmitigated",  false, false, false},
        {"Filter_Only",               true,  true,  false},
        {"Mitigated_Full",            true,  true,  true},
        {"ZOH_AdaptiveDamping",       false, false, true}
    };

    for (uint64_t d_ns : delays_ns) {
        for (const auto& m : modes) {
            StabilitySimContext ctx;
            ctx.net_cfg.latency_ns = d_ns;
            ctx.net_cfg.packet_loss_rate = 0.0;
            ctx.dist_opts.enable_velocity_prediction = m.pred;
            ctx.dist_opts.filter_options.enable_filtering = m.filter;
            ctx.dist_opts.filter_options.max_prediction_horizon_s = 0.030; // 30 ms
            ctx.dist_opts.filter_options.enable_adaptive_damping = m.adapt_damp;
            ctx.dist_opts.filter_options.adaptive_damping_alpha = 15.0;

            StabilityBenchmarkRecord rec = run_stability_trial(ds_loop, ctx, "Latency_Ablation", m.label);
            records.push_back(rec);
        }
    }

    // ---------------------------------------------------------------------------
    // 2. PACKET LOSS RESILIENCE (Loss: 0%, 10%, 25%, 50% at 50ms delay)
    // ---------------------------------------------------------------------------
    std::cout << ">>> Suite 2: Packet Loss Sweep under 50ms Delay (4 loss rates)...\n";
    MultiRoverDataset ds_grid = generate_multi_rover_grid_manhattan(2, 4, 3, 2.0, 0.08, 0.04, 0.10, 0.05, 42);
    for (double loss : {0.0, 0.10, 0.25, 0.50}) {
        StabilitySimContext ctx;
        ctx.net_cfg.latency_ns = 50'000'000; // 50ms
        ctx.net_cfg.packet_loss_rate = loss;
        ctx.dist_opts.enable_velocity_prediction = true;
        ctx.dist_opts.filter_options.enable_filtering = true;
        ctx.dist_opts.filter_options.max_prediction_horizon_s = 0.030;
        ctx.dist_opts.filter_options.enable_adaptive_damping = true;
        ctx.dist_opts.filter_options.adaptive_damping_alpha = 15.0;

        StabilityBenchmarkRecord rec = run_stability_trial(ds_grid, ctx, "PacketLoss_Sweep", "Mitigated_Full");
        records.push_back(rec);
    }

    // ---------------------------------------------------------------------------
    // 3. MULTI-ROVER TOPOLOGY SCALING (3 Rovers: Ring, Grid, Chain at 50ms delay)
    // ---------------------------------------------------------------------------
    std::cout << ">>> Suite 3: Multi-Rover Topology Scaling (3 topologies)...\n";
    std::vector<MultiRoverDataset> topo_datasets = {
        generate_multi_rover_circular_loop(3, 10, 5.0, 0.08, 0.04, 0.10, 0.05, 42),
        generate_multi_rover_grid_manhattan(3, 4, 3, 2.0, 0.08, 0.04, 0.10, 0.05, 42),
        generate_multi_rover_straight_chain(3, 10, 1.0, 0.08, 0.04, 0.10, 0.05, 42)
    };

    for (const auto& ds : topo_datasets) {
        StabilitySimContext ctx;
        ctx.net_cfg.latency_ns = 50'000'000; // 50ms
        ctx.dist_opts.enable_velocity_prediction = true;
        ctx.dist_opts.filter_options.enable_filtering = true;
        ctx.dist_opts.filter_options.max_prediction_horizon_s = 0.030;
        ctx.dist_opts.filter_options.enable_adaptive_damping = true;
        ctx.dist_opts.filter_options.adaptive_damping_alpha = 15.0;

        StabilityBenchmarkRecord rec = run_stability_trial(ds, ctx, "Topology_Scaling", "Mitigated_Full");
        records.push_back(rec);
    }

    // ---------------------------------------------------------------------------
    // 4. NETWORK PARTITION RECOVERY (400ms link severance)
    // ---------------------------------------------------------------------------
    std::cout << ">>> Suite 4: Network Partition Recovery (400ms partition)...\n";
    {
        StabilitySimContext ctx;
        ctx.net_cfg.latency_ns = 50'000'000;
        ctx.partition_test = true;
        ctx.partition_start_step = 30;
        ctx.partition_end_step = 70;
        ctx.dist_opts.enable_velocity_prediction = true;
        ctx.dist_opts.filter_options.enable_filtering = true;
        ctx.dist_opts.filter_options.max_prediction_horizon_s = 0.030;
        ctx.dist_opts.filter_options.enable_adaptive_damping = true;
        ctx.dist_opts.filter_options.adaptive_damping_alpha = 15.0;

        StabilityBenchmarkRecord rec = run_stability_trial(ds_loop, ctx, "Partition_Recovery", "Mitigated_Full");
        records.push_back(rec);
    }

    // ---------------------------------------------------------------------------
    // 5. MULTI-SEED REPEATABILITY (Seeds: 42, 101, 777 at 50ms delay)
    // ---------------------------------------------------------------------------
    std::cout << ">>> Suite 5: Multi-Seed Robustness (Seeds 42, 101, 777)...\n";
    for (uint32_t s : {42u, 101u, 777u}) {
        MultiRoverDataset ds_s = generate_multi_rover_circular_loop(2, 10, 5.0, 0.08, 0.04, 0.10, 0.05, s);
        StabilitySimContext ctx;
        ctx.net_cfg.latency_ns = 50'000'000;
        ctx.net_cfg.random_seed = s;
        ctx.dist_opts.enable_velocity_prediction = true;
        ctx.dist_opts.filter_options.enable_filtering = true;
        ctx.dist_opts.filter_options.max_prediction_horizon_s = 0.030;
        ctx.dist_opts.filter_options.enable_adaptive_damping = true;
        ctx.dist_opts.filter_options.adaptive_damping_alpha = 15.0;

        StabilityBenchmarkRecord rec = run_stability_trial(ds_s, ctx, "MultiSeed_Sweep", "Mitigated_Full");
        records.push_back(rec);
    }

    // Export to CSV
    std::ofstream ofs(csv_file);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open CSV output file: " << csv_file << "\n";
        return 1;
    }
    write_header(ofs);
    for (const auto& r : records) {
        write_row(ofs, r);
    }
    ofs.close();

    std::cout << "\n>>> Successfully exported " << records.size() << " records to " << csv_file << " <<<\n\n";

    // Print summary table
    std::cout << "========================================================================================================================================\n";
    std::cout << " PHASE 4.1 LATENCY ABLATION COMPARISON (MultiRoverLoop_2R_10N)\n";
    std::cout << "========================================================================================================================================\n";
    std::cout << std::left
              << std::setw(28) << "Mode"
              << std::setw(8)  << "Delay"
              << std::setw(12) << "Init Cost"
              << std::setw(12) << "Final Cost"
              << std::setw(9)  << "Reduct%"
              << std::setw(12) << "ATE(m)"
              << std::setw(12) << "Max |w|"
              << std::setw(10) << "Status"
              << "\n";
    std::cout << "----------------------------------------------------------------------------------------------------------------------------------------\n";
    for (const auto& r : records) {
        if (r.experiment_group == "Latency_Ablation") {
            std::cout << std::left
                      << std::setw(28) << r.mode_label
                      << std::setw(8)  << (std::to_string(static_cast<int>(r.nominal_latency_ms)) + "ms")
                      << std::setw(12) << std::fixed << std::setprecision(2) << r.initial_global_cost
                      << std::setw(12) << std::fixed << std::setprecision(2) << r.final_global_cost
                      << std::setw(9)  << std::fixed << std::setprecision(1) << r.cost_reduction_pct
                      << std::setw(12) << std::fixed << std::setprecision(4) << r.final_ate_aligned_rmse
                      << std::setw(12) << std::fixed << std::setprecision(2) << r.max_angular_velocity
                      << std::setw(10) << r.status
                      << "\n";
        }
    }
    std::cout << "========================================================================================================================================\n";

    return 0;
}
