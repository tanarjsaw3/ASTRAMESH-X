#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/optimization/boundary_packet.hpp"
#include "astromesh/optimization/cord_integrator.hpp"
#include "astromesh/optimization/centralized_optimizer.hpp"
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

struct DistributedBenchmarkRecord {
    std::string experiment_group;
    std::string dataset_name;
    uint32_t num_rovers{2};
    size_t num_nodes{0};
    size_t num_edges{0};
    uint32_t seed{42};
    double nominal_latency_ms{0.0};
    double packet_loss_rate_cfg{0.0};
    double packet_loss_rate_obs{0.0};
    bool velocity_prediction_enabled{true};
    size_t simulation_steps{100};
    double step_dt_ms{10.0};

    double initial_global_cost{0.0};
    double final_global_cost{0.0};
    double final_active_cost{0.0};
    double cost_reduction_pct{0.0};

    double init_ate_raw_rmse{0.0};
    double final_ate_raw_rmse{0.0};
    double init_ate_aligned_rmse{0.0};
    double final_ate_aligned_rmse{0.0};

    double mean_boundary_pred_error_m{0.0};
    size_t gated_fallback_count{0};
    size_t disconnected_events_count{0};
    size_t reconnected_events_count{0};

    size_t packets_sent{0};
    size_t packets_delivered{0};
    size_t packets_dropped{0};
    size_t payload_bytes_sent{0};
    size_t wire_bytes_sent{0};
    double peak_wire_bytes_per_step{0.0};
    double avg_wire_bytes_per_step{0.0};

    bool reached_ate_threshold{false};
    size_t steps_to_threshold{0};
    double runtime_wall_ms{0.0};
    std::string status{"Completed"};
};

struct SimulationContext {
    SimulatedNetworkConfig net_cfg;
    DistributedCordOptions dist_opts;
    size_t total_steps{100};
    uint64_t step_dt_ns{10'000'000}; // 10 ms
    double ate_threshold_m{0.50};
    bool partition_test{false};
    size_t partition_start_step{30};
    size_t partition_end_step{70};

    SimulationContext() {
        dist_opts.default_inertia = NodeInertia(5.0, 5.0, 5.0);
        dist_opts.default_damping = NodeDamping(20.0, 20.0, 20.0);
    }
};

DistributedBenchmarkRecord run_distributed_cord_simulation(
    const MultiRoverDataset& ds,
    const SimulationContext& ctx,
    const std::string& group_name) {

    DistributedBenchmarkRecord rec;
    rec.experiment_group = group_name;
    rec.dataset_name = ds.name;
    rec.num_rovers = ds.num_rovers;
    rec.num_nodes = ds.ground_truth_poses.size();
    rec.num_edges = ds.num_odometry_edges + ds.num_verified_loop_closures;
    rec.seed = ds.seed;
    rec.nominal_latency_ms = static_cast<double>(ctx.net_cfg.latency_ns) * 1e-6;
    rec.packet_loss_rate_cfg = ctx.net_cfg.packet_loss_rate;
    rec.velocity_prediction_enabled = ctx.dist_opts.enable_velocity_prediction;
    rec.simulation_steps = ctx.total_steps;
    rec.step_dt_ms = static_cast<double>(ctx.step_dt_ns) * 1e-6;

    // Initial global metrics
    TrajectoryEvaluation init_ate = evaluate_trajectory(ds.initial_poses, ds.ground_truth_poses);
    rec.init_ate_raw_rmse = init_ate.ate_raw.translation.rmse;
    rec.init_ate_aligned_rmse = init_ate.ate_aligned.translation.rmse;

    ObjectiveEvaluationResult init_obj = evaluate_pose_graph_objective(ds.full_graph, ds.initial_poses);
    rec.initial_global_cost = init_obj.valid ? init_obj.total_cost : 0.0;

    // Initialize Network and Rover Nodes
    SimulatedMeshNetwork network(ctx.net_cfg);

    std::map<uint32_t, std::unique_ptr<DistributedCordNode>> rovers;
    for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
        DistributedCordOptions opts = ctx.dist_opts;
        opts.timestep = rec.step_dt_ms * 1e-3;
        opts.fixed_anchors = ds.fixed_anchors;

        const auto& sub_g = ds.subgraphs.at(r);
        const auto& init_st = ds.initial_states_by_rover.at(r);
        rovers[r] = std::make_unique<DistributedCordNode>(r, sub_g, init_st, opts, network);
    }

    // Seed initial boundary state discovery for step 0
    for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
        for (const auto& nid : rovers[r]->local_boundary_nodes()) {
            // Find remote peer rovers for this boundary node
            for (uint32_t peer = 1; peer <= ds.num_rovers; ++peer) {
                if (peer == r) continue;
                rovers[peer]->inject_neighbor_state(
                    nid, r,
                    ds.initial_poses.at(nid),
                    Twist2D(0.0, 0.0, 0.0),
                    0
                );
            }
        }
    }

    double total_pred_error_sum = 0.0;
    size_t pred_error_sample_count = 0;
    size_t peak_wire_bytes = 0;
    size_t total_wire_bytes = 0;

    const auto t_wall_start = std::chrono::high_resolution_clock::now();

    for (size_t step = 0; step < ctx.total_steps; ++step) {
        uint64_t sim_time = step * ctx.step_dt_ns;
        network.set_current_time_ns(sim_time);

        // Network partition handling if enabled
        if (ctx.partition_test) {
            if (step == ctx.partition_start_step) {
                // Sever all inter-rover links
                for (uint32_t r1 = 1; r1 <= ds.num_rovers; ++r1) {
                    for (uint32_t r2 = r1 + 1; r2 <= ds.num_rovers; ++r2) {
                        network.set_link_state(static_cast<uint16_t>(r1), static_cast<uint16_t>(r2), false);
                    }
                }
            } else if (step == ctx.partition_end_step) {
                // Restore all inter-rover links
                for (uint32_t r1 = 1; r1 <= ds.num_rovers; ++r1) {
                    for (uint32_t r2 = r1 + 1; r2 <= ds.num_rovers; ++r2) {
                        network.set_link_state(static_cast<uint16_t>(r1), static_cast<uint16_t>(r2), true);
                    }
                }
            }
        }

        const size_t prev_wire_bytes = network.statistics().transport_bytes_sent;

        // Advance rovers
        for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
            bool ok = rovers[r]->step(sim_time);
            if (!ok) {
                rec.status = "NumericalFailure";
            }
        }

        const size_t step_wire_bytes = network.statistics().transport_bytes_sent - prev_wire_bytes;
        peak_wire_bytes = std::max(peak_wire_bytes, step_wire_bytes);
        total_wire_bytes += step_wire_bytes;

        // Measure boundary prediction error against true current poses
        for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
            for (const auto& [neighbor_id, rec_n] : rovers[r]->neighbor_states()) {
                if (rec_n.status == NeighborLinkStatus::Fresh) {
                    const uint32_t remote_r = neighbor_id.rover_id;
                    if (rovers.find(remote_r) != rovers.end() && rovers[remote_r]->local_states().find(neighbor_id) != rovers[remote_r]->local_states().end()) {
                        const SE2 true_pose = rovers[remote_r]->local_states().at(neighbor_id).pose;
                        const double dx = rec_n.current_estimate_pose.x() - true_pose.x();
                        const double dy = rec_n.current_estimate_pose.y() - true_pose.y();
                        total_pred_error_sum += std::sqrt(dx * dx + dy * dy);
                        pred_error_sample_count++;
                    }
                }
            }
        }

        // Check threshold convergence
        if (!rec.reached_ate_threshold) {
            std::unordered_map<NodeId, SE2> current_poses;
            for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
                for (const auto& [nid, state] : rovers[r]->local_states()) {
                    current_poses[nid] = state.pose;
                }
            }
            TrajectoryEvaluation cur_ate = evaluate_trajectory(current_poses, ds.ground_truth_poses);
            if (cur_ate.ate_aligned.translation.rmse <= ctx.ate_threshold_m) {
                rec.reached_ate_threshold = true;
                rec.steps_to_threshold = step;
            }
        }
    }

    const auto t_wall_end = std::chrono::high_resolution_clock::now();
    rec.runtime_wall_ms = std::chrono::duration<double, std::milli>(t_wall_end - t_wall_start).count();

    // Aggregate final states
    std::unordered_map<NodeId, SE2> final_poses;
    rec.final_active_cost = 0.0;
    for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
        rec.final_active_cost += rovers[r]->compute_local_active_cost();
        for (const auto& [nid, state] : rovers[r]->local_states()) {
            final_poses[nid] = state.pose;
        }

        // Tally lifecycle events
        for (const auto& ev : rovers[r]->lifecycle_events()) {
            if (ev.event_type == LifecycleEventType::PredictionGatedAge ||
                ev.event_type == LifecycleEventType::PredictionGatedMotion) {
                rec.gated_fallback_count++;
            } else if (ev.event_type == LifecycleEventType::LinkDisconnected) {
                rec.disconnected_events_count++;
            } else if (ev.event_type == LifecycleEventType::LinkReconnected) {
                rec.reconnected_events_count++;
            }
        }
    }

    // Final global metrics
    ObjectiveEvaluationResult final_obj = evaluate_pose_graph_objective(ds.full_graph, final_poses);
    rec.final_global_cost = final_obj.valid ? final_obj.total_cost : 0.0;

    if (rec.initial_global_cost > 1e-9) {
        rec.cost_reduction_pct = 100.0 * (rec.initial_global_cost - rec.final_global_cost) / rec.initial_global_cost;
    }

    TrajectoryEvaluation final_ate = evaluate_trajectory(final_poses, ds.ground_truth_poses);
    rec.final_ate_raw_rmse = final_ate.ate_raw.translation.rmse;
    rec.final_ate_aligned_rmse = final_ate.ate_aligned.translation.rmse;

    if (pred_error_sample_count > 0) {
        rec.mean_boundary_pred_error_m = total_pred_error_sum / static_cast<double>(pred_error_sample_count);
    }

    // Network stats
    const auto& net_stats = network.statistics();
    rec.packets_sent = net_stats.packets_sent;
    rec.packets_delivered = net_stats.packets_delivered;
    rec.packets_dropped = net_stats.packets_dropped;
    rec.payload_bytes_sent = net_stats.raw_payload_bytes_sent;
    rec.wire_bytes_sent = net_stats.transport_bytes_sent;
    rec.peak_wire_bytes_per_step = static_cast<double>(peak_wire_bytes);
    rec.avg_wire_bytes_per_step = (ctx.total_steps > 0) ? (static_cast<double>(total_wire_bytes) / static_cast<double>(ctx.total_steps)) : 0.0;

    if (rec.packets_sent > 0) {
        rec.packet_loss_rate_obs = static_cast<double>(rec.packets_dropped) / static_cast<double>(rec.packets_sent);
    }

    return rec;
}

void write_csv_header(std::ostream& os) {
    os << "experiment_group,"
       << "dataset_name,"
       << "num_rovers,"
       << "num_nodes,"
       << "num_edges,"
       << "seed,"
       << "nominal_latency_ms,"
       << "packet_loss_rate_cfg,"
       << "packet_loss_rate_obs,"
       << "velocity_prediction_enabled,"
       << "simulation_steps,"
       << "step_dt_ms,"
       << "initial_global_cost,"
       << "final_global_cost,"
       << "final_active_cost,"
       << "cost_reduction_pct,"
       << "init_ate_raw_rmse,"
       << "final_ate_raw_rmse,"
       << "init_ate_aligned_rmse,"
       << "final_ate_aligned_rmse,"
       << "mean_boundary_pred_error_m,"
       << "gated_fallback_count,"
       << "disconnected_events_count,"
       << "reconnected_events_count,"
       << "packets_sent,"
       << "packets_delivered,"
       << "packets_dropped,"
       << "payload_bytes_sent,"
       << "wire_bytes_sent,"
       << "peak_wire_bytes_per_step,"
       << "avg_wire_bytes_per_step,"
       << "reached_ate_threshold,"
       << "steps_to_threshold,"
       << "runtime_wall_ms,"
       << "status\n";
}

void write_csv_record(std::ostream& os, const DistributedBenchmarkRecord& r) {
    os << r.experiment_group << ","
       << r.dataset_name << ","
       << r.num_rovers << ","
       << r.num_nodes << ","
       << r.num_edges << ","
       << r.seed << ","
       << std::fixed << std::setprecision(1) << r.nominal_latency_ms << ","
       << std::setprecision(3) << r.packet_loss_rate_cfg << ","
       << std::setprecision(3) << r.packet_loss_rate_obs << ","
       << (r.velocity_prediction_enabled ? "true" : "false") << ","
       << r.simulation_steps << ","
       << std::setprecision(1) << r.step_dt_ms << ","
       << std::setprecision(4) << r.initial_global_cost << ","
       << std::setprecision(4) << r.final_global_cost << ","
       << std::setprecision(4) << r.final_active_cost << ","
       << std::setprecision(2) << r.cost_reduction_pct << ","
       << std::setprecision(4) << r.init_ate_raw_rmse << ","
       << std::setprecision(4) << r.final_ate_raw_rmse << ","
       << std::setprecision(4) << r.init_ate_aligned_rmse << ","
       << std::setprecision(4) << r.final_ate_aligned_rmse << ","
       << std::setprecision(4) << r.mean_boundary_pred_error_m << ","
       << r.gated_fallback_count << ","
       << r.disconnected_events_count << ","
       << r.reconnected_events_count << ","
       << r.packets_sent << ","
       << r.packets_delivered << ","
       << r.packets_dropped << ","
       << r.payload_bytes_sent << ","
       << r.wire_bytes_sent << ","
       << std::setprecision(1) << r.peak_wire_bytes_per_step << ","
       << std::setprecision(1) << r.avg_wire_bytes_per_step << ","
       << (r.reached_ate_threshold ? "true" : "false") << ","
       << r.steps_to_threshold << ","
       << std::setprecision(2) << r.runtime_wall_ms << ","
       << r.status << "\n";
}

void print_summary_table(const std::vector<DistributedBenchmarkRecord>& records) {
    std::cout << "\n================================================================================================================================================\n";
    std::cout << " ASTRAMESH-X PHASE 3 (M3.6): DISTRIBUTED CORD BENCHMARK RESULTS & ABLATION PROFILING\n";
    std::cout << "================================================================================================================================================\n";
    std::cout << std::left
              << std::setw(18) << "Group"
              << std::setw(22) << "Dataset"
              << std::setw(8)  << "Delay"
              << std::setw(8)  << "Loss%"
              << std::setw(6)  << "Pred"
              << std::setw(12) << "Init Cost"
              << std::setw(12) << "Final Cost"
              << std::setw(9)  << "Reduct%"
              << std::setw(12) << "Final ATE(m)"
              << std::setw(12) << "PredErr(m)"
              << std::setw(10) << "Pkts(D/S)"
              << std::setw(10) << "WireBytes"
              << std::setw(10) << "Status"
              << "\n";
    std::cout << "------------------------------------------------------------------------------------------------------------------------------------------------\n";

    for (const auto& r : records) {
        std::string pkts = std::to_string(r.packets_delivered) + "/" + std::to_string(r.packets_sent);
        std::cout << std::left
                  << std::setw(18) << r.experiment_group
                  << std::setw(22) << r.dataset_name
                  << std::setw(8)  << (std::to_string(static_cast<int>(r.nominal_latency_ms)) + "ms")
                  << std::setw(8)  << (std::to_string(static_cast<int>(r.packet_loss_rate_cfg * 100)) + "%")
                  << std::setw(6)  << (r.velocity_prediction_enabled ? "YES" : "NO")
                  << std::setw(12) << std::fixed << std::setprecision(2) << r.initial_global_cost
                  << std::setw(12) << std::fixed << std::setprecision(2) << r.final_global_cost
                  << std::setw(9)  << std::fixed << std::setprecision(1) << r.cost_reduction_pct
                  << std::setw(12) << std::fixed << std::setprecision(4) << r.final_ate_aligned_rmse
                  << std::setw(12) << std::fixed << std::setprecision(4) << r.mean_boundary_pred_error_m
                  << std::setw(10) << pkts
                  << std::setw(10) << r.wire_bytes_sent
                  << std::setw(10) << r.status
                  << "\n";
    }
    std::cout << "================================================================================================================================================\n\n";
}

int main(int argc, char** argv) {
    std::cout << "==============================================================================\n";
    std::cout << " AstroMesh-X Phase 3 Milestone 3.6: Distributed CORD Benchmark & Ablation Suite\n";
    std::cout << "==============================================================================\n";

    std::string csv_filename = "benchmark_distributed_cord_results.csv";
    if (argc > 1) {
        csv_filename = argv[1];
    }

    std::vector<DistributedBenchmarkRecord> all_records;

    // ---------------------------------------------------------------------------
    // 1. LATENCY ABLATION STUDY: Delay in {0, 20, 50, 100, 200, 500} ms
    //    Paired comparison: Velocity Prediction Enabled vs Disabled
    // ---------------------------------------------------------------------------
    std::cout << "\n>>> Running Experiment Suite 1: Latency Ablations (0 to 500ms)...\n";
    {
        MultiRoverDataset ds_loop = generate_multi_rover_circular_loop(2, 10, 5.0, 0.08, 0.04, 0.10, 0.05, 42);
        const std::vector<uint64_t> delays_ns = {
            0,            // 0ms (instantaneous delivery)
            20'000'000,   // 20ms (2 steps)
            50'000'000,   // 50ms (5 steps)
            100'000'000,  // 100ms (10 steps)
            200'000'000,  // 200ms (20 steps)
            500'000'000   // 500ms (50 steps)
        };

        for (uint64_t delay : delays_ns) {
            for (bool pred_enabled : {true, false}) {
                SimulationContext ctx;
                ctx.net_cfg.latency_ns = delay;
                ctx.net_cfg.packet_loss_rate = 0.0;
                ctx.dist_opts.enable_velocity_prediction = pred_enabled;
                ctx.total_steps = 100;

                std::string grp = "Latency_Ablation";
                DistributedBenchmarkRecord rec = run_distributed_cord_simulation(ds_loop, ctx, grp);
                all_records.push_back(rec);
            }
        }
    }

    // ---------------------------------------------------------------------------
    // 2. PACKET-LOSS ABLATION STUDY: Loss in {0%, 10%, 25%, 50%}
    // ---------------------------------------------------------------------------
    std::cout << ">>> Running Experiment Suite 2: Packet-Loss Ablations (0% to 50%)...\n";
    {
        MultiRoverDataset ds_grid = generate_multi_rover_grid_manhattan(2, 4, 3, 2.0, 0.08, 0.04, 0.10, 0.05, 42);
        const std::vector<double> loss_rates = {0.0, 0.10, 0.25, 0.50};

        for (double loss : loss_rates) {
            SimulationContext ctx;
            ctx.net_cfg.latency_ns = 20'000'000; // 20ms base delay
            ctx.net_cfg.packet_loss_rate = loss;
            ctx.dist_opts.enable_velocity_prediction = true;
            ctx.total_steps = 100;

            std::string grp = "PacketLoss_Ablation";
            DistributedBenchmarkRecord rec = run_distributed_cord_simulation(ds_grid, ctx, grp);
            all_records.push_back(rec);
        }
    }

    // ---------------------------------------------------------------------------
    // 3. TOPOLOGY SCALING STUDY: Straight Chain, Grid Manhattan & Multi-Rover Ring (3 Rovers)
    // ---------------------------------------------------------------------------
    std::cout << ">>> Running Experiment Suite 3: Topology & Rover Scaling (2 and 3 Rovers)...\n";
    {
        // 3-Rover Straight Chain (30 nodes)
        MultiRoverDataset ds_chain3 = generate_multi_rover_straight_chain(3, 10, 1.0, 0.08, 0.04, 0.10, 0.05, 42);
        SimulationContext ctx_chain;
        ctx_chain.net_cfg.latency_ns = 20'000'000;
        ctx_chain.total_steps = 100;
        all_records.push_back(run_distributed_cord_simulation(ds_chain3, ctx_chain, "Topology_Scaling"));

        // 3-Rover Manhattan Grid (36 nodes)
        MultiRoverDataset ds_grid3 = generate_multi_rover_grid_manhattan(3, 4, 3, 2.0, 0.08, 0.04, 0.10, 0.05, 42);
        SimulationContext ctx_grid;
        ctx_grid.net_cfg.latency_ns = 20'000'000;
        ctx_grid.total_steps = 100;
        all_records.push_back(run_distributed_cord_simulation(ds_grid3, ctx_grid, "Topology_Scaling"));

        // 3-Rover Circular Loop with cross-closures (30 nodes)
        MultiRoverDataset ds_loop3 = generate_multi_rover_circular_loop(3, 10, 6.0, 0.08, 0.04, 0.10, 0.05, 42);
        SimulationContext ctx_loop;
        ctx_loop.net_cfg.latency_ns = 20'000'000;
        ctx_loop.total_steps = 100;
        all_records.push_back(run_distributed_cord_simulation(ds_loop3, ctx_loop, "Topology_Scaling"));
    }

    // ---------------------------------------------------------------------------
    // 4. TEMPORARY NETWORK PARTITION & RECONNECTION RECOVERY STUDY
    // ---------------------------------------------------------------------------
    std::cout << ">>> Running Experiment Suite 4: Network Partition & Dynamic Reconnection...\n";
    {
        MultiRoverDataset ds_loop = generate_multi_rover_circular_loop(2, 10, 5.0, 0.08, 0.04, 0.10, 0.05, 42);
        SimulationContext ctx_part;
        ctx_part.net_cfg.latency_ns = 10'000'000;
        ctx_part.partition_test = true;
        ctx_part.partition_start_step = 25;
        ctx_part.partition_end_step = 65; // Disconnected for 40 steps (400 ms)
        ctx_part.total_steps = 120;

        DistributedBenchmarkRecord rec = run_distributed_cord_simulation(ds_loop, ctx_part, "Partition_Recovery");
        all_records.push_back(rec);
    }

    // ---------------------------------------------------------------------------
    // 5. MULTI-SEED REPEATABILITY & AGGREGATE STATS (Seeds 42, 101, 777)
    // ---------------------------------------------------------------------------
    std::cout << ">>> Running Experiment Suite 5: Multi-Seed Robustness & Repeatability...\n";
    {
        for (uint32_t s : {42U, 101U, 777U}) {
            MultiRoverDataset ds_seed = generate_multi_rover_circular_loop(2, 10, 5.0, 0.08, 0.04, 0.10, 0.05, s);
            SimulationContext ctx_s;
            ctx_s.net_cfg.latency_ns = 20'000'000;
            ctx_s.net_cfg.packet_loss_rate = 0.10;
            ctx_s.total_steps = 80;

            DistributedBenchmarkRecord rec = run_distributed_cord_simulation(ds_seed, ctx_s, "Seed_Robustness");
            all_records.push_back(rec);
        }
    }

    // Print Formatted Summary Table to stdout
    print_summary_table(all_records);

    // Write Machine-Readable CSV
    std::ofstream ofs(csv_filename);
    if (ofs.is_open()) {
        write_csv_header(ofs);
        for (const auto& r : all_records) {
            write_csv_record(ofs, r);
        }
        std::cout << ">>> Successfully generated benchmark CSV: " << csv_filename << " (" << all_records.size() << " records)\n\n";
    } else {
        std::cerr << "ERROR: Failed to open output CSV file: " << csv_filename << "\n";
        return 1;
    }

    return 0;
}
