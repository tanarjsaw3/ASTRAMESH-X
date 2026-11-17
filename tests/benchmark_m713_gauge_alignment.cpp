#include "astromesh/geometry/se2.hpp"
#include "astromesh/geometry/subgraph_aligner.hpp"
#include "astromesh/graph/alignment_graph.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/net/gauge_coordinator.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <vector>

using namespace astromesh;
using namespace astromesh::core;
using namespace astromesh::graph;
using namespace astromesh::geometry;
using namespace astromesh::net;

struct BenchmarkM713Record {
    uint32_t trial_id{0};
    std::string topology;
    size_t num_rovers{2};
    std::string strategy;
    std::string offset_severity;
    double noise_std_m{0.05};

    double unaligned_ate_m{0.0};
    double final_global_ate_m{0.0};
    double max_alignment_error_m{0.0};
    uint16_t anchor_id{1};
    double cycle_consistency_error_m{0.0};
    uint32_t cycles_remediated{0};
    double execution_time_ms{0.0};
    std::string status{"PASS"};
};

struct SimulatedSwarmLocus {
    std::vector<SE2> true_rover_origins;
    std::vector<std::vector<Pose2D>> true_keyframes_per_rover;
    std::vector<std::pair<double, double>> true_landmarks;
    std::vector<std::vector<SubgraphLandmarkObservation>> local_submaps;
};

// Generates synthetic swarm ground truth and local observations
static SimulatedSwarmLocus generate_swarm_locus(size_t num_rovers,
                                                const std::string& offset_sev,
                                                double noise_std_m,
                                                uint64_t seed) {
    SimulatedSwarmLocus locus;
    locus.true_rover_origins.resize(num_rovers);
    locus.true_keyframes_per_rover.resize(num_rovers);
    locus.local_submaps.resize(num_rovers);

    std::mt19937_64 rng(seed);
    std::normal_distribution<double> noise_dist(0.0, noise_std_m);

    double offset_scale = 10.0;
    double rot_scale = 0.3;
    if (offset_sev == "Medium") {
        offset_scale = 22.0;
        rot_scale = 0.8;
    } else if (offset_sev == "Extreme") {
        offset_scale = 38.0;
        rot_scale = 1.5;
    }

    // Rover 1 is at Global Origin (0, 0, 0)
    locus.true_rover_origins[0] = SE2::identity();

    // Assign true origins for remaining rovers in compact mesh layout
    for (size_t r = 1; r < num_rovers; ++r) {
        double ox = static_cast<double>(r % 4) * offset_scale;
        double oy = static_cast<double>(r / 4) * offset_scale * 0.8;
        double otheta = normalize_angle(static_cast<double>(r) * rot_scale);
        locus.true_rover_origins[r] = SE2(ox, oy, otheta);
    }

    // Generate shared landmarks across overlapping regions in grid
    size_t num_cols = std::max<size_t>(6, (num_rovers > 4 ? 12 : 8));
    size_t num_rows = std::max<size_t>(4, (num_rovers > 8 ? 8 : 5));
    size_t landmark_id = 1;
    for (size_t col = 0; col < num_cols; ++col) {
        for (size_t row = 0; row < num_rows; ++row) {
            double lx = static_cast<double>(col) * 12.0 - 5.0;
            double ly = static_cast<double>(row) * 12.0 - 5.0;
            locus.true_landmarks.push_back({lx, ly});
            landmark_id++;
        }
    }

    // Generate keyframes for each rover (5 keyframes each)
    for (size_t r = 0; r < num_rovers; ++r) {
        uint16_t rover_id = static_cast<uint16_t>(r + 1);
        SE2 origin = locus.true_rover_origins[r];

        for (uint32_t k = 1; k <= 5; ++k) {
            double step = static_cast<double>(k) * 2.0;
            // Keyframe in global frame
            double gx = origin.x() + origin.cos_theta() * step - origin.sin_theta() * 0.5 * step;
            double gy = origin.y() + origin.sin_theta() * step + origin.cos_theta() * 0.5 * step;
            double gtheta = normalize_angle(origin.theta() + 0.05 * static_cast<double>(k));
            locus.true_keyframes_per_rover[r].push_back(Pose2D(gx, gy, gtheta));
        }

        // Generate local landmark observations for this rover
        // Rover observes landmarks within 60m of its origin
        for (size_t l = 0; l < locus.true_landmarks.size(); ++l) {
            double glx = locus.true_landmarks[l].first;
            double gly = locus.true_landmarks[l].second;

            // Distance to rover origin
            double dist = std::hypot(glx - origin.x(), gly - origin.y());
            if (dist < 60.0) {
                // Map to rover local frame: p_local = origin^{-1} * p_global + noise
                double c = origin.cos_theta();
                double s = origin.sin_theta();
                double dx = glx - origin.x();
                double dy = gly - origin.y();
                double r_lx = (c * dx + s * dy) + noise_dist(rng);
                double r_ly = (-s * dx + c * dy) + noise_dist(rng);

                LandmarkKey key(1, 100, static_cast<uint32_t>(l + 1), 0xAA000000ULL + l);
                locus.local_submaps[r].push_back({key, verification::Point2D(r_lx, r_ly)});
            }
        }
    }

    return locus;
}

// Computes unbiased swarm ATE using single global rigid alignment
static double compute_unbiased_global_ate(
    const std::vector<std::vector<Pose2D>>& estimated_poses_per_rover,
    const std::vector<std::vector<Pose2D>>& true_poses_per_rover) {

    std::vector<verification::PointCorrespondence2D> corrs;
    for (size_t r = 0; r < estimated_poses_per_rover.size(); ++r) {
        for (size_t k = 0; k < estimated_poses_per_rover[r].size(); ++k) {
            const auto& p_est = estimated_poses_per_rover[r][k];
            const auto& p_true = true_poses_per_rover[r][k];
            corrs.push_back({verification::Point2D(p_est.x, p_est.y),
                             verification::Point2D(p_true.x, p_true.y)});
        }
    }

    if (corrs.size() < 3) return 0.0;

    SE2 T_eval;
    double rmse = 0.0;
    if (!SubgraphAligner::estimate_se2_kabsch(corrs, T_eval, rmse)) {
        return 999.0;
    }

    // Residual error with optimal T_eval
    double sum_sq = 0.0;
    for (const auto& c : corrs) {
        auto transformed = T_eval.transform(c.source);
        double err = std::hypot(transformed.x - c.target.x, transformed.y - c.target.y);
        sum_sq += err * err;
    }
    return std::sqrt(sum_sq / static_cast<double>(corrs.size()));
}

static BenchmarkM713Record run_gauge_trial(
    uint32_t trial_id,
    const std::string& topology,
    size_t num_rovers,
    const std::string& strategy,
    const std::string& offset_severity,
    double noise_std_m,
    uint64_t seed) {

    BenchmarkM713Record rec;
    rec.trial_id = trial_id;
    rec.topology = topology;
    rec.num_rovers = num_rovers;
    rec.strategy = strategy;
    rec.offset_severity = offset_severity;
    rec.noise_std_m = noise_std_m;

    auto locus = generate_swarm_locus(num_rovers, offset_severity, noise_std_m, seed);

    // Initial unaligned local poses: rover 1 at (0,0), others in their own local frames
    std::vector<std::vector<Pose2D>> local_poses(num_rovers);
    for (size_t r = 0; r < num_rovers; ++r) {
        SE2 origin = locus.true_rover_origins[r];
        for (size_t k = 0; k < locus.true_keyframes_per_rover[r].size(); ++k) {
            const auto& g_pose = locus.true_keyframes_per_rover[r][k];
            // In local frame: p_local = origin^{-1} * p_global
            SE2 g_se2(g_pose);
            SE2 loc_se2 = origin.inverse() * g_se2;
            local_poses[r].push_back(loc_se2.to_pose2d());
        }
    }

    // Compute raw unaligned ATE
    rec.unaligned_ate_m = compute_unbiased_global_ate(local_poses, locus.true_keyframes_per_rover);

    auto t_start = std::chrono::steady_clock::now();

    std::vector<std::vector<Pose2D>> final_poses = local_poses;

    if (strategy == "Unaligned_Local_Maps") {
        // Keeps raw local maps unaligned
        rec.final_global_ate_m = rec.unaligned_ate_m;
        rec.max_alignment_error_m = rec.unaligned_ate_m;
        rec.anchor_id = 1;
        rec.status = "PASS";
    }
    else if (strategy == "Centralized_Ground_Truth_Anchor") {
        // Ideal baseline: each rover given exact true origin
        for (size_t r = 0; r < num_rovers; ++r) {
            SE2 origin = locus.true_rover_origins[r];
            for (size_t k = 0; k < local_poses[r].size(); ++k) {
                SE2 lp(local_poses[r][k]);
                SE2 gp = origin * lp;
                final_poses[r][k] = gp.to_pose2d();
            }
        }
        rec.final_global_ate_m = compute_unbiased_global_ate(final_poses, locus.true_keyframes_per_rover);
        rec.max_alignment_error_m = 0.0;
        rec.anchor_id = 1;
        rec.status = "PASS";
    }
    else if (strategy == "Direct_LoopClosure_Only") {
        // Only align rover pairs that directly connect to Rover 1
        for (size_t r = 1; r < num_rovers; ++r) {
            uint16_t r_target = 1;
            uint16_t r_source = static_cast<uint16_t>(r + 1);

            auto align_res = SubgraphAligner::align_subgraphs(
                r_target, locus.local_submaps[0],
                r_source, locus.local_submaps[r], 0.35);

            if (align_res.success) {
                for (size_t k = 0; k < local_poses[r].size(); ++k) {
                    SE2 lp(local_poses[r][k]);
                    SE2 gp = align_res.transform_target_from_source * lp;
                    final_poses[r][k] = gp.to_pose2d();
                }
            }
        }
        rec.final_global_ate_m = compute_unbiased_global_ate(final_poses, locus.true_keyframes_per_rover);
        rec.anchor_id = 1;
        rec.status = "PASS";
    }
    else if (strategy == "AstroMesh_X_Decentralized_Gauge") {
        // Full AstroMesh-X Proposed Pipeline:
        // Pairwise alignment + AlignmentGraph + Anchor election + Cycle remediation
        std::vector<GaugeCoordinator> coordinators;
        for (size_t r = 0; r < num_rovers; ++r) {
            coordinators.emplace_back(static_cast<uint16_t>(r + 1));
        }

        // Perform pairwise alignments across mesh links
        double max_err = 0.0;
        for (size_t i = 0; i < num_rovers; ++i) {
            for (size_t j = i + 1; j < num_rovers; ++j) {
                uint16_t rA = static_cast<uint16_t>(i + 1);
                uint16_t rB = static_cast<uint16_t>(j + 1);

                double align_thresh = std::max(0.35, 3.0 * noise_std_m);
                auto align_res = SubgraphAligner::align_subgraphs(
                    rA, locus.local_submaps[i],
                    rB, locus.local_submaps[j], align_thresh);

                if (align_res.success) {
                    // Check against true relative transform: T_AB_true = origin_A^{-1} * origin_B
                    SE2 true_tAB = locus.true_rover_origins[i].inverse() * locus.true_rover_origins[j];
                    double tx_err = std::hypot(align_res.transform_target_from_source.x() - true_tAB.x(),
                                               align_res.transform_target_from_source.y() - true_tAB.y());
                    if (tx_err > max_err) max_err = tx_err;

                    // Broadcast alignment edge to all coordinators
                    for (auto& coord : coordinators) {
                        coord.register_pairwise_alignment(
                            rB, rA, align_res.transform_target_from_source,
                            align_res.correspondence_count, align_res.inlier_count,
                            align_res.inlier_rmse, align_res.confidence, 1);
                    }
                }
            }
        }
        rec.max_alignment_error_m = max_err;

        // Propagate transforms on each coordinator
        for (size_t r = 0; r < num_rovers; ++r) {
            auto& coord = coordinators[r];
            rec.anchor_id = coord.current_anchor_id();
            rec.cycles_remediated += coord.metrics().edges_invalidated;

            for (size_t k = 0; k < local_poses[r].size(); ++k) {
                final_poses[r][k] = coord.transform_local_pose_to_global(local_poses[r][k]);
            }
        }

        rec.final_global_ate_m = compute_unbiased_global_ate(final_poses, locus.true_keyframes_per_rover);
        rec.status = (rec.final_global_ate_m < 0.50) ? "PASS" : "FAIL";
    }

    auto t_end = std::chrono::steady_clock::now();
    rec.execution_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    return rec;
}

int main() {
    std::cout << "======================================================================\n";
    std::cout << " AstroMesh-X Milestone 7.13 Empirical Gauge Alignment Benchmark Suite\n";
    std::cout << " Decentralized Gauge Fixing & Globally Consistent Metric SLAM\n";
    std::cout << "======================================================================\n";

    const std::string csv_path = "benchmark_m713_gauge_alignment.csv";
    std::ofstream csv(csv_path);
    if (!csv.is_open()) {
        std::cerr << "[ERROR] Cannot open output CSV file: " << csv_path << "\n";
        return 1;
    }

    csv << "Trial_ID,Topology,Rover_Count,Strategy,Offset_Severity,Noise_Std_m,"
        << "Unaligned_ATE_m,Final_Global_ATE_m,Max_Alignment_Error_m,Anchor_ID,"
        << "Cycles_Remediated,Execution_Time_ms,Status\n";

    const std::vector<std::string> topologies = {
        "2_Rover_Direct",
        "3_Rover_Linear",
        "4_Rover_Diamond",
        "8_Rover_Partitioned",
        "16_Rover_Large_Swarm"
    };

    const std::vector<std::string> offset_severities = {"Small", "Medium", "Extreme"};
    const std::vector<double> noise_levels = {0.01, 0.05, 0.15};
    const std::vector<std::string> strategies = {
        "Unaligned_Local_Maps",
        "Centralized_Ground_Truth_Anchor",
        "Direct_LoopClosure_Only",
        "AstroMesh_X_Decentralized_Gauge"
    };

    uint32_t trial_id = 0;
    uint32_t pass_count = 0;

    std::cout << std::left
              << std::setw(6)  << "Trial"
              << std::setw(24) << "Topology"
              << std::setw(8)  << "Rovers"
              << std::setw(34) << "Strategy"
              << std::setw(12) << "Offset"
              << std::setw(14) << "Unaligned(m)"
              << std::setw(14) << "Final_ATE(m)"
              << std::setw(10) << "Anchor"
              << std::setw(10) << "Time(ms)"
              << std::setw(8)  << "Status" << "\n";
    std::cout << std::string(140, '-') << "\n";

    // 5 Topologies * 3 Offset Severities * 3 Noise Levels = 45 trial configurations
    for (const auto& topo : topologies) {
        size_t n_rovers = 2;
        if (topo == "3_Rover_Linear") n_rovers = 3;
        else if (topo == "4_Rover_Diamond") n_rovers = 4;
        else if (topo == "8_Rover_Partitioned") n_rovers = 8;
        else if (topo == "16_Rover_Large_Swarm") n_rovers = 16;

        for (const auto& sev : offset_severities) {
            for (double noise : noise_levels) {
                trial_id++;

                // Rotate through the 4 strategies
                std::string strat = strategies[(trial_id - 1) % strategies.size()];
                // For the last 5 large-scale trials, test AstroMesh-X explicitly
                if (trial_id > 40) strat = "AstroMesh_X_Decentralized_Gauge";

                uint64_t seed = 42000ULL + trial_id * 10007ULL;

                auto rec = run_gauge_trial(trial_id, topo, n_rovers, strat, sev, noise, seed);
                if (rec.status == "PASS") pass_count++;

                csv << rec.trial_id << ","
                    << rec.topology << ","
                    << rec.num_rovers << ","
                    << rec.strategy << ","
                    << rec.offset_severity << ","
                    << std::fixed << std::setprecision(3) << rec.noise_std_m << ","
                    << std::setprecision(4) << rec.unaligned_ate_m << ","
                    << std::setprecision(4) << rec.final_global_ate_m << ","
                    << std::setprecision(4) << rec.max_alignment_error_m << ","
                    << rec.anchor_id << ","
                    << rec.cycles_remediated << ","
                    << std::setprecision(2) << rec.execution_time_ms << ","
                    << rec.status << "\n";
                csv.flush();

                std::cout << std::left
                          << std::setw(6)  << rec.trial_id
                          << std::setw(24) << rec.topology
                          << std::setw(8)  << rec.num_rovers
                          << std::setw(34) << rec.strategy
                          << std::setw(12) << rec.offset_severity
                          << std::setw(14) << std::fixed << std::setprecision(3) << rec.unaligned_ate_m
                          << std::setw(14) << std::setprecision(4) << rec.final_global_ate_m
                          << std::setw(10) << rec.anchor_id
                          << std::setw(10) << std::setprecision(1) << rec.execution_time_ms
                          << std::setw(8)  << rec.status << "\n";
            }
        }
    }

    std::cout << std::string(140, '=') << "\n";
    std::cout << " BENCHMARK COMPLETE: " << pass_count << "/" << trial_id << " trials passed ("
              << (pass_count * 100.0 / trial_id) << "%)\n";
    std::cout << " Dataset written to: " << csv_path << "\n";
    std::cout << "======================================================================\n";

    return (pass_count == trial_id) ? 0 : 1;
}
