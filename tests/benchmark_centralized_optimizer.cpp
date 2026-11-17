#include "astromesh/optimization/centralized_optimizer.hpp"
#include "astromesh/optimization/trajectory_metrics.hpp"
#include "support/synthetic_pose_graph.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace astromesh;
using namespace astromesh::optimization;
using namespace astromesh::tests::support;

struct BenchmarkRunConfig {
    std::string dataset_type{"all"};
    size_t graph_size{20};
    uint32_t seed{42};
    double odom_noise_t{0.02};
    double odom_noise_r{0.01};
    double init_noise_t{0.05};
    double init_noise_r{0.02};
    std::string output_csv{"benchmark_centralized_results.csv"};
    size_t max_iterations{100};
    double cost_tolerance{1e-6};
    double step_tolerance{1e-6};
};

struct BenchmarkRecord {
    std::string dataset_name;
    size_t num_nodes{0};
    size_t num_eligible_edges{0};
    size_t num_excluded_edges{0};
    uint32_t seed{0};
    double odom_noise_trans{0.0};
    double odom_noise_rot{0.0};
    double init_noise_trans{0.0};
    double init_noise_rot{0.0};
    std::string solver_status;
    bool converged{false};
    size_t iterations{0};
    size_t successful_steps{0};
    size_t rejected_steps{0};
    double initial_cost{0.0};
    double final_cost{0.0};
    double cost_reduction_pct{0.0};
    double init_ate_raw_rmse{0.0};
    double final_ate_raw_rmse{0.0};
    double init_ate_aligned_rmse{0.0};
    double final_ate_aligned_rmse{0.0};
    double final_rpe_trans_rmse{0.0};
    double runtime_ms{0.0};
};

BenchmarkRecord run_single_experiment(const SyntheticDataset& ds, const OptimizerOptions& opt) {
    BenchmarkRecord rec;
    rec.dataset_name = ds.name;
    rec.num_nodes = ds.ground_truth_poses.size();
    rec.num_eligible_edges = ds.num_odometry_edges + ds.num_verified_loop_closures;
    rec.num_excluded_edges = ds.num_unverified_loop_closures + ds.num_rejected_loop_closures;
    rec.seed = ds.seed;

    // Evaluate initial metrics
    TrajectoryEvaluation init_eval = evaluate_trajectory(ds.initial_poses, ds.ground_truth_poses);
    rec.init_ate_raw_rmse = init_eval.ate_raw.translation.rmse;
    rec.init_ate_aligned_rmse = init_eval.ate_aligned.translation.rmse;

    // Measure wall-clock optimization runtime
    const auto t_start = std::chrono::high_resolution_clock::now();
    OptimizerResult result = optimize_pose_graph(ds.graph, ds.initial_poses, opt);
    const auto t_end = std::chrono::high_resolution_clock::now();

    rec.runtime_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    rec.solver_status = optimizer_status_to_string(result.status);
    rec.converged = result.is_converged();
    rec.iterations = result.iterations;
    rec.successful_steps = result.successful_steps;
    rec.rejected_steps = result.rejected_steps;
    rec.initial_cost = result.initial_cost;
    rec.final_cost = result.final_cost;

    if (rec.initial_cost > 1e-9) {
        rec.cost_reduction_pct = 100.0 * (rec.initial_cost - rec.final_cost) / rec.initial_cost;
    }

    // Evaluate final metrics
    TrajectoryEvaluation final_eval = evaluate_trajectory(result.optimized_poses, ds.ground_truth_poses);
    rec.final_ate_raw_rmse = final_eval.ate_raw.translation.rmse;
    rec.final_ate_aligned_rmse = final_eval.ate_aligned.translation.rmse;
    rec.final_rpe_trans_rmse = final_eval.rpe.translation.rmse;

    return rec;
}

void print_table(const std::vector<BenchmarkRecord>& records) {
    std::cout << "\n=========================================================================================================\n";
    std::cout << " ASTRAMESH-X CENTRALIZED SE(2) REFERENCE OPTIMIZER BENCHMARK RESULTS (SYNTHETIC BASELINE)\n";
    std::cout << "=========================================================================================================\n";
    std::cout << std::left
              << std::setw(25) << "Dataset"
              << std::setw(8)  << "Nodes"
              << std::setw(8)  << "Edges"
              << std::setw(18) << "Status"
              << std::setw(8)  << "Iters"
              << std::setw(14) << "Init Cost"
              << std::setw(14) << "Final Cost"
              << std::setw(12) << "ATE_Raw(m)"
              << std::setw(12) << "ATE_Aln(m)"
              << std::setw(10) << "Time(ms)"
              << "\n---------------------------------------------------------------------------------------------------------\n";

    for (const auto& r : records) {
        std::cout << std::left
                  << std::setw(25) << r.dataset_name
                  << std::setw(8)  << r.num_nodes
                  << std::setw(8)  << r.num_eligible_edges
                  << std::setw(18) << r.solver_status
                  << std::setw(8)  << r.iterations
                  << std::setw(14) << std::fixed << std::setprecision(4) << r.initial_cost
                  << std::setw(14) << std::fixed << std::setprecision(4) << r.final_cost
                  << std::setw(12) << std::fixed << std::setprecision(4) << r.final_ate_raw_rmse
                  << std::setw(12) << std::fixed << std::setprecision(4) << r.final_ate_aligned_rmse
                  << std::setw(10) << std::fixed << std::setprecision(2) << r.runtime_ms
                  << "\n";
    }
    std::cout << "=========================================================================================================\n\n";
}

bool write_csv(const std::string& filepath, const std::vector<BenchmarkRecord>& records) {
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open CSV output file: " << filepath << "\n";
        return false;
    }

    ofs << "dataset_name,num_nodes,num_eligible_edges,num_excluded_edges,seed,"
        << "odom_noise_trans,odom_noise_rot,init_noise_trans,init_noise_rot,"
        << "solver_status,converged,iterations,successful_steps,rejected_steps,"
        << "initial_cost,final_cost,cost_reduction_pct,"
        << "init_ate_raw_rmse,final_ate_raw_rmse,init_ate_aligned_rmse,final_ate_aligned_rmse,"
        << "final_rpe_trans_rmse,runtime_ms\n";

    for (const auto& r : records) {
        ofs << r.dataset_name << ","
            << r.num_nodes << ","
            << r.num_eligible_edges << ","
            << r.num_excluded_edges << ","
            << r.seed << ","
            << r.odom_noise_trans << ","
            << r.odom_noise_rot << ","
            << r.init_noise_trans << ","
            << r.init_noise_rot << ","
            << r.solver_status << ","
            << (r.converged ? "true" : "false") << ","
            << r.iterations << ","
            << r.successful_steps << ","
            << r.rejected_steps << ","
            << r.initial_cost << ","
            << r.final_cost << ","
            << r.cost_reduction_pct << ","
            << r.init_ate_raw_rmse << ","
            << r.final_ate_raw_rmse << ","
            << r.init_ate_aligned_rmse << ","
            << r.final_ate_aligned_rmse << ","
            << r.final_rpe_trans_rmse << ","
            << r.runtime_ms << "\n";
    }

    std::cout << "Benchmark CSV recorded successfully to: " << filepath << "\n";
    return true;
}

int main(int argc, char** argv) {
    BenchmarkRunConfig cfg;

    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--dataset" && i + 1 < argc) {
            cfg.dataset_type = argv[++i];
        } else if (arg == "--size" && i + 1 < argc) {
            cfg.graph_size = static_cast<size_t>(std::stoul(argv[++i]));
        } else if (arg == "--seed" && i + 1 < argc) {
            cfg.seed = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--output" && i + 1 < argc) {
            cfg.output_csv = argv[++i];
        }
    }

    OptimizerOptions opt;
    opt.max_iterations = cfg.max_iterations;
    opt.cost_tolerance = cfg.cost_tolerance;
    opt.step_tolerance = cfg.step_tolerance;

    std::vector<SyntheticDataset> suite;

    if (cfg.dataset_type == "straight" || cfg.dataset_type == "all") {
        suite.push_back(generate_straight_chain(cfg.graph_size, 1.0, cfg.odom_noise_t, cfg.odom_noise_r,
                                                cfg.init_noise_t, cfg.init_noise_r, cfg.seed));
    }
    if (cfg.dataset_type == "loop" || cfg.dataset_type == "all") {
        suite.push_back(generate_closed_loop(cfg.graph_size, 5.0, cfg.odom_noise_t, cfg.odom_noise_r,
                                             cfg.init_noise_t, cfg.init_noise_r, true, cfg.seed));
    }
    if (cfg.dataset_type == "grid" || cfg.dataset_type == "all") {
        suite.push_back(generate_grid_manhattan(4, 4, 2.0, cfg.odom_noise_t, cfg.odom_noise_r,
                                                cfg.init_noise_t, cfg.init_noise_r, 0.5, cfg.seed));
    }
    if (cfg.dataset_type == "multirover" || cfg.dataset_type == "all") {
        suite.push_back(generate_multi_component_graph(2, 10, cfg.seed));
    }
    if (cfg.dataset_type == "gating" || cfg.dataset_type == "all") {
        suite.push_back(generate_graph_with_unverified_and_rejected_closures(15, cfg.seed));
    }

    std::vector<BenchmarkRecord> records;
    records.reserve(suite.size());

    for (auto& ds : suite) {
        BenchmarkRecord rec = run_single_experiment(ds, opt);
        rec.odom_noise_trans = cfg.odom_noise_t;
        rec.odom_noise_rot = cfg.odom_noise_r;
        rec.init_noise_trans = cfg.init_noise_t;
        rec.init_noise_rot = cfg.init_noise_r;
        records.push_back(rec);
    }

    print_table(records);
    write_csv(cfg.output_csv, records);

    return 0;
}
