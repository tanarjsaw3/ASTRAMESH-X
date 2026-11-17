#include "astromesh/geometry/se2.hpp"
#include "astromesh/optimization/trajectory_metrics.hpp"
#include "astromesh/simulation/baseline_astromesh_v1.hpp"
#include "astromesh/simulation/observation_model.hpp"
#include "astromesh/simulation/pipeline_astromesh_x.hpp"
#include "astromesh/simulation/pipeline_interface.hpp"
#include "astromesh/simulation/swarm_trajectories.hpp"
#include "astromesh/simulation/synthetic_world.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace astromesh;
using namespace astromesh::simulation;

// ==============================================================================
// 1. Scenario Configuration Definitions
// ==============================================================================

struct BenchmarkScenarioConfig {
    std::string id;
    std::string description;
    size_t rover_count{2};
    std::string trajectory_type{"ConcentricLoops"};
    size_t num_landmarks{100};
    double sensor_range{18.0};
    double arena_extent{30.0};
    uint64_t duration_ns{8'000'000'000ULL}; // 8.0 seconds
    uint64_t step_dt_ns{200'000'000ULL};    // 200 ms (40 steps)
    double speed{2.0};
    double packet_loss_rate{0.0};
    uint64_t partition_start_step{0};
    uint64_t partition_end_step{0};
    double comm_range{25.0};
    size_t epoch_quota{0};
    bool degenerate_geometry{false};
    double odom_noise_trans{0.02};
    double odom_noise_rot{0.005};
    double obs_noise_range{0.03};
    double obs_noise_bearing{0.01};
};

// ==============================================================================
// 2. Metrics Recording Structures
// ==============================================================================

struct PipelineTrialRecord {
    std::string scenario_id;
    uint64_t seed{0};
    std::string variant;
    size_t rover_count{0};
    double duration_s{0.0};
    size_t steps{0};
    double packet_loss_rate{0.0};
    bool partition_active{false};
    bool budget_constrained{false};
    bool degenerate_geometry{false};
    std::string trajectory_type;
    uint64_t runtime_us{0};

    size_t graph_nodes{0};
    size_t graph_edges{0};
    std::string optimization_mode{"None"};
    size_t optimizer_calls{0};
    size_t optimizer_iterations{0};
    size_t optimizer_failures{0};

    size_t keyframes_retained{0};
    size_t keyframe_prunes_applied{0};
    size_t keyframe_prunes_rejected{0};

    size_t candidates_generated{0};
    size_t closures_verified{0};
    size_t closures_rejected{0};
    size_t closures_true_accept{0};
    size_t closures_false_accept{0};
    double precision{0.0};
    bool precision_valid{false};
    double recall{0.0};
    bool recall_valid{false};

    size_t initial_dsu_components{0};
    size_t final_dsu_components{0};
    int64_t first_merge_step{-1};
    bool merge_persisted{false};

    double ate_raw_rmse_m{0.0};
    double ate_aligned_swarm_rmse_m{0.0};
    double ate_aligned_swarm_mean_m{0.0};
    double ate_aligned_swarm_median_m{0.0};
    double ate_aligned_swarm_max_m{0.0};
    bool ate_aligned_valid{false};
    double ate_aligned_per_rover_mean_m{0.0};
    double rpe_rmse_m{0.0};
    bool rpe_valid{false};

    size_t logical_messages_sent{0};
    size_t logical_messages_delivered{0};
    size_t logical_messages_dropped{0};
    size_t unique_deliveries{0};
    size_t hop_transmissions_attempted{0};
    size_t hop_transmissions_successful{0};
    size_t hop_transmissions_dropped{0};
    size_t retransmissions_performed{0};
    size_t payload_bytes{0};
    size_t protocol_bytes{0};
    size_t framing_bytes{0};
    size_t data_wire_bytes{0};
    size_t ack_wire_bytes{0};
    size_t total_wire_bytes{0};

    bool data_byte_conservation_valid{false};
    bool total_byte_conservation_valid{false};
    double app_completion_ratio{0.0};
    bool app_completion_ratio_valid{false};
    double per_hop_delivery_ratio{0.0};
    bool per_hop_delivery_ratio_valid{false};

    std::string trial_status{"Success"};
    std::string failure_reason;
};

struct TrialFailureRecord {
    std::string scenario_id;
    uint64_t seed{0};
    std::string variant;
    std::string failure_stage;
    std::string error_message;
};

// ==============================================================================
// 3. Statistical Summaries with Bonferroni Correction
// ==============================================================================

struct MetricSummary {
    double mean{0.0};
    double median{0.0};
    double std_dev{0.0};
    double p25{0.0};
    double p75{0.0};
};

MetricSummary compute_summary(std::vector<double> vals) {
    MetricSummary s;
    if (vals.empty()) return s;
    std::sort(vals.begin(), vals.end());
    const size_t n = vals.size();
    double sum = std::accumulate(vals.begin(), vals.end(), 0.0);
    s.mean = sum / static_cast<double>(n);
    s.median = (n % 2 == 1) ? vals[n / 2] : 0.5 * (vals[n / 2 - 1] + vals[n / 2]);
    s.p25 = vals[n / 4];
    s.p75 = vals[(3 * n) / 4];
    if (n > 1) {
        double sq_sum = 0.0;
        for (double v : vals) {
            sq_sum += (v - s.mean) * (v - s.mean);
        }
        s.std_dev = std::sqrt(sq_sum / static_cast<double>(n - 1));
    }
    return s;
}

struct PairedAblationComparison {
    std::string scenario_id;
    std::string variant_a; // Reference (AstroMesh_X_Full)
    std::string variant_b; // Ablated variant or Baseline
    std::string metric_name;
    std::string unit;
    bool lower_is_better{true};
    MetricSummary ref_summary;
    MetricSummary abl_summary;
    double paired_diff_mean{0.0};
    double paired_diff_median{0.0};
    double paired_diff_std{0.0};
    double ci95_low{0.0};
    double ci95_high{0.0};
    double ci95_bonferroni_low{0.0};
    double ci95_bonferroni_high{0.0};
    std::string conclusion;
};

PairedAblationComparison analyze_ablation_pair(
    const std::string& sc_id,
    const std::string& var_a,
    const std::string& var_b,
    const std::string& name,
    const std::string& unit,
    bool lower_is_better,
    const std::vector<double>& a_vals,
    const std::vector<double>& b_vals,
    size_t num_comparisons) {

    PairedAblationComparison comp;
    comp.scenario_id = sc_id;
    comp.variant_a = var_a;
    comp.variant_b = var_b;
    comp.metric_name = name;
    comp.unit = unit;
    comp.lower_is_better = lower_is_better;

    comp.ref_summary = compute_summary(a_vals);
    comp.abl_summary = compute_summary(b_vals);

    const size_t n = std::min(a_vals.size(), b_vals.size());
    if (n == 0) return comp;

    std::vector<double> diffs(n, 0.0); // diff = b (ablated) - a (reference)
    for (size_t i = 0; i < n; ++i) {
        diffs[i] = b_vals[i] - a_vals[i];
    }
    MetricSummary diff_sum = compute_summary(diffs);
    comp.paired_diff_mean = diff_sum.mean;
    comp.paired_diff_median = diff_sum.median;
    comp.paired_diff_std = diff_sum.std_dev;

    const double se = (n > 1) ? (diff_sum.std_dev / std::sqrt(static_cast<double>(n))) : 0.0;
    
    // Critical t values:
    // Standard alpha=0.05 two-tailed:
    // N=1 -> 0, N=2 -> 12.706, N=30 -> 2.045
    const double t_crit_unadj = (n <= 1) ? 0.0 : (n <= 2 ? 12.7062 : 2.04523);
    comp.ci95_low = diff_sum.mean - t_crit_unadj * se;
    comp.ci95_high = diff_sum.mean + t_crit_unadj * se;

    // Bonferroni-adjusted alpha = 0.05 / num_comparisons:
    // For 5 pairwise ablation comparisons against Full, alpha_adj = 0.01 -> t_crit(nu=29) approx 2.756
    const double t_crit_bonf = (n <= 1) ? 0.0 : (n <= 2 ? 63.656 : (num_comparisons > 1 ? 2.75639 : 2.04523));
    comp.ci95_bonferroni_low = diff_sum.mean - t_crit_bonf * se;
    comp.ci95_bonferroni_high = diff_sum.mean + t_crit_bonf * se;

    if (comp.ci95_bonferroni_low > 0.0) {
        comp.conclusion = lower_is_better ? (var_a + " Superior (p_adj < 0.05)") : (var_b + " Superior (p_adj < 0.05)");
    } else if (comp.ci95_bonferroni_high < 0.0) {
        comp.conclusion = lower_is_better ? (var_b + " Superior (p_adj < 0.05)") : (var_a + " Superior (p_adj < 0.05)");
    } else {
        comp.conclusion = "Statistically Indistinguishable";
    }

    return comp;
}

// ==============================================================================
// 4. Trial Execution
// ==============================================================================

PipelineTrialRecord evaluate_pipeline_instance(
    IRoverSwarmPipeline& pipeline,
    const BenchmarkScenarioConfig& sc,
    uint64_t seed,
    const std::vector<SimulationStepInput>& step_inputs,
    const std::vector<RoverTrajectory>& trajectories,
    const std::unordered_map<graph::NodeId, geometry::SE2>& gt_pose_map,
    size_t ground_truth_loop_opportunities,
    bool run_optimization,
    std::vector<TrialFailureRecord>& failures) {

    PipelineTrialRecord rec;
    rec.scenario_id = sc.id;
    rec.seed = seed;
    rec.variant = pipeline.pipeline_name();
    rec.rover_count = sc.rover_count;
    rec.duration_s = static_cast<double>(sc.duration_ns) * 1e-9;
    rec.steps = step_inputs.size();
    rec.packet_loss_rate = sc.packet_loss_rate;
    rec.partition_active = (sc.partition_end_step > sc.partition_start_step);
    rec.budget_constrained = (sc.epoch_quota > 0);
    rec.degenerate_geometry = sc.degenerate_geometry;
    rec.trajectory_type = sc.trajectory_type;
    rec.initial_dsu_components = sc.rover_count;

    const auto start_time = std::chrono::high_resolution_clock::now();

    int64_t merge_step = -1;
    for (size_t s = 0; s < step_inputs.size(); ++s) {
        if (!pipeline.ingest_step(step_inputs[s])) {
            rec.trial_status = "Failed";
            rec.failure_reason = "IngestStepRejected";
            failures.push_back({sc.id, seed, rec.variant, "Ingest", "Step rejected"});
            return rec;
        }

        if (merge_step < 0 && pipeline.num_connected_components() < sc.rover_count) {
            merge_step = static_cast<int64_t>(s);
        }
    }

    if (run_optimization) {
        rec.optimizer_calls = 1;
        const bool opt_ok = pipeline.optimize();
        if (!opt_ok) {
            rec.optimizer_failures++;
        }
    }

    const auto end_time = std::chrono::high_resolution_clock::now();
    rec.runtime_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count());

    rec.first_merge_step = merge_step;
    rec.final_dsu_components = pipeline.num_connected_components();
    rec.merge_persisted = (merge_step >= 0 && rec.final_dsu_components < sc.rover_count);

    const auto m = pipeline.collect_metrics();
    rec.optimization_mode = m.optimization_mode;
    rec.optimizer_iterations = m.optimization_steps_performed;
    rec.keyframes_retained = m.total_keyframes_retained;
    rec.keyframe_prunes_applied = m.keyframe_prunes_applied;
    rec.keyframe_prunes_rejected = m.keyframe_prunes_rejected;
    rec.candidates_generated = m.candidate_pairs_proposed;
    rec.closures_verified = m.loop_closures_verified;
    rec.closures_rejected = m.loop_closures_rejected;

    auto* astromesh_ptr = dynamic_cast<AstroMeshXPipeline*>(&pipeline);
    if (astromesh_ptr) {
        rec.graph_nodes = astromesh_ptr->swarm_graph().num_nodes();
        rec.graph_edges = astromesh_ptr->swarm_graph().num_edges();
    } else {
        rec.graph_nodes = sc.rover_count * step_inputs.size();
        rec.graph_edges = sc.rover_count * (step_inputs.size() - 1);
    }

    // Evaluate Loop Closure True/False Acceptances
    const auto history = pipeline.get_loop_closure_history();
    for (const auto& cand : history) {
        if (cand.verification_passed) {
            const size_t s_idx = std::min(step_inputs.size() - 1, static_cast<size_t>(cand.timestamp_ns / sc.step_dt_ns));
            const auto& p_gt_a = trajectories[cand.rover_a - 1].waypoints[s_idx].pose;
            const auto& p_gt_b = trajectories[cand.rover_b - 1].waypoints[s_idx].pose;
            const auto t_gt_ab = p_gt_a.inverse().compose(p_gt_b);

            const auto& est_ab = cand.estimated_relative_transform;
            const double dx = est_ab.x - t_gt_ab.x();
            const double dy = est_ab.y - t_gt_ab.y();
            const double dtheta = std::abs(graph::normalize_angle(est_ab.theta - t_gt_ab.theta()));
            const double trans_err = std::hypot(dx, dy);

            if (trans_err <= 1.0 && dtheta <= 0.35) {
                rec.closures_true_accept++;
            } else {
                rec.closures_false_accept++;
            }
        }
    }

    if (rec.closures_true_accept + rec.closures_false_accept > 0) {
        rec.precision = static_cast<double>(rec.closures_true_accept) /
                        static_cast<double>(rec.closures_true_accept + rec.closures_false_accept);
        rec.precision_valid = true;
    } else {
        rec.precision = -1.0;
        rec.precision_valid = false;
    }

    if (ground_truth_loop_opportunities > 0) {
        rec.recall = static_cast<double>(rec.closures_true_accept) /
                     static_cast<double>(ground_truth_loop_opportunities);
        rec.recall_valid = true;
    } else {
        rec.recall = -1.0;
        rec.recall_valid = false;
    }

    rec.logical_messages_sent = m.logical_messages_sent;
    rec.logical_messages_delivered = m.logical_messages_delivered;
    rec.logical_messages_dropped = m.logical_messages_dropped;
    rec.unique_deliveries = m.unique_deliveries;
    rec.hop_transmissions_attempted = m.hop_transmissions_attempted;
    rec.hop_transmissions_successful = m.hop_transmissions_successful;
    rec.hop_transmissions_dropped = m.logical_messages_dropped;
    rec.retransmissions_performed = m.retransmissions_performed;
    rec.payload_bytes = m.payload_bytes_transmitted;
    rec.protocol_bytes = m.protocol_bytes_transmitted;
    rec.framing_bytes = m.framing_bytes_transmitted;
    rec.data_wire_bytes = m.data_wire_bytes_transmitted;
    rec.ack_wire_bytes = m.ack_wire_bytes_transmitted;
    rec.total_wire_bytes = m.total_wire_bytes_transmitted;

    rec.data_byte_conservation_valid = (m.data_wire_bytes_transmitted ==
        (m.payload_bytes_transmitted + m.protocol_bytes_transmitted + m.framing_bytes_transmitted));
    rec.total_byte_conservation_valid = (m.total_wire_bytes_transmitted ==
        (m.data_wire_bytes_transmitted + m.ack_wire_bytes_transmitted));

    rec.app_completion_ratio = m.application_completion_ratio;
    rec.app_completion_ratio_valid = (m.logical_messages_sent > 0);
    rec.per_hop_delivery_ratio = m.per_hop_delivery_ratio;
    rec.per_hop_delivery_ratio_valid = (m.hop_transmissions_attempted > 0);

    // Trajectory metrics
    const auto est_trajs = pipeline.get_estimated_trajectories();
    std::unordered_map<graph::NodeId, geometry::SE2> est_pose_map;
    std::vector<double> per_rover_aligned_ates;

    for (const auto& [rid, poses] : est_trajs) {
        std::unordered_map<graph::NodeId, geometry::SE2> rover_est;
        std::unordered_map<graph::NodeId, geometry::SE2> rover_gt;

        for (size_t k = 0; k < poses.size(); ++k) {
            const graph::NodeId nid(rid, static_cast<uint32_t>(k));
            est_pose_map[nid] = poses[k];
            rover_est[nid] = poses[k];
            if (gt_pose_map.find(nid) != gt_pose_map.end()) {
                rover_gt[nid] = gt_pose_map.at(nid);
            }
        }

        auto rover_ate = optimization::compute_ate(rover_est, rover_gt, true);
        if (rover_ate.valid) {
            per_rover_aligned_ates.push_back(rover_ate.translation.rmse);
        }
    }

    if (!per_rover_aligned_ates.empty()) {
        rec.ate_aligned_per_rover_mean_m = std::accumulate(
            per_rover_aligned_ates.begin(), per_rover_aligned_ates.end(), 0.0) /
            static_cast<double>(per_rover_aligned_ates.size());
    }

    auto ate_raw = optimization::compute_ate(est_pose_map, gt_pose_map, false);
    auto ate_aligned = optimization::compute_ate(est_pose_map, gt_pose_map, true);
    auto rpe = optimization::compute_rpe(est_pose_map, gt_pose_map, 1);

    if (ate_raw.valid) rec.ate_raw_rmse_m = ate_raw.translation.rmse;
    if (ate_aligned.valid) {
        rec.ate_aligned_swarm_rmse_m = ate_aligned.translation.rmse;
        rec.ate_aligned_swarm_mean_m = ate_aligned.translation.mean;
        rec.ate_aligned_swarm_median_m = ate_aligned.translation.median;
        rec.ate_aligned_swarm_max_m = ate_aligned.translation.max_error;
        rec.ate_aligned_valid = true;
    }
    if (rpe.valid) {
        rec.rpe_rmse_m = rpe.translation.rmse;
        rec.rpe_valid = true;
    }

    return rec;
}

// ==============================================================================
// 5. CSV Exporters
// ==============================================================================

void write_trial_row(std::ostream& out, const PipelineTrialRecord& r) {
    out << r.scenario_id << ","
        << r.seed << ","
        << r.variant << ","
        << r.rover_count << ","
        << std::fixed << std::setprecision(2) << r.duration_s << ","
        << r.steps << ","
        << std::fixed << std::setprecision(4) << r.packet_loss_rate << ","
        << (r.partition_active ? "true" : "false") << ","
        << (r.budget_constrained ? "true" : "false") << ","
        << (r.degenerate_geometry ? "true" : "false") << ","
        << r.trajectory_type << ","
        << r.runtime_us << ","
        << r.graph_nodes << ","
        << r.graph_edges << ","
        << r.optimization_mode << ","
        << r.optimizer_calls << ","
        << r.optimizer_iterations << ","
        << r.optimizer_failures << ","
        << r.keyframes_retained << ","
        << r.keyframe_prunes_applied << ","
        << r.keyframe_prunes_rejected << ","
        << r.candidates_generated << ","
        << r.closures_verified << ","
        << r.closures_rejected << ","
        << r.closures_true_accept << ","
        << r.closures_false_accept << ","
        << std::fixed << std::setprecision(6) << r.precision << ","
        << (r.precision_valid ? "true" : "false") << ","
        << std::fixed << std::setprecision(6) << r.recall << ","
        << (r.recall_valid ? "true" : "false") << ","
        << r.initial_dsu_components << ","
        << r.final_dsu_components << ","
        << r.first_merge_step << ","
        << (r.merge_persisted ? "true" : "false") << ","
        << std::fixed << std::setprecision(6) << r.ate_raw_rmse_m << ","
        << std::fixed << std::setprecision(6) << r.ate_aligned_swarm_rmse_m << ","
        << std::fixed << std::setprecision(6) << r.ate_aligned_swarm_mean_m << ","
        << std::fixed << std::setprecision(6) << r.ate_aligned_swarm_median_m << ","
        << std::fixed << std::setprecision(6) << r.ate_aligned_swarm_max_m << ","
        << (r.ate_aligned_valid ? "true" : "false") << ","
        << std::fixed << std::setprecision(6) << r.ate_aligned_per_rover_mean_m << ","
        << std::fixed << std::setprecision(6) << r.rpe_rmse_m << ","
        << (r.rpe_valid ? "true" : "false") << ","
        << r.logical_messages_sent << ","
        << r.logical_messages_delivered << ","
        << r.logical_messages_dropped << ","
        << r.unique_deliveries << ","
        << r.hop_transmissions_attempted << ","
        << r.hop_transmissions_successful << ","
        << r.hop_transmissions_dropped << ","
        << r.retransmissions_performed << ","
        << r.payload_bytes << ","
        << r.protocol_bytes << ","
        << r.framing_bytes << ","
        << r.data_wire_bytes << ","
        << r.ack_wire_bytes << ","
        << r.total_wire_bytes << ","
        << (r.data_byte_conservation_valid ? "true" : "false") << ","
        << (r.total_byte_conservation_valid ? "true" : "false") << ","
        << std::fixed << std::setprecision(6) << r.app_completion_ratio << ","
        << (r.app_completion_ratio_valid ? "true" : "false") << ","
        << std::fixed << std::setprecision(6) << r.per_hop_delivery_ratio << ","
        << (r.per_hop_delivery_ratio_valid ? "true" : "false") << ","
        << r.trial_status << ","
        << "\"" << r.failure_reason << "\"\n";
    out.flush();
}

void write_summary_csv(
    const std::string& filepath,
    const std::vector<PairedAblationComparison>& comparisons,
    size_t num_seeds) {

    std::ofstream out(filepath);
    if (!out.is_open()) return;

    out << "scenario_id,num_paired_seeds,variant_a,variant_b,metric_name,unit,lower_is_better,"
        << "ref_mean,ref_median,ref_std,abl_mean,abl_median,abl_std,"
        << "paired_diff_mean,paired_diff_median,paired_diff_std,"
        << "ci95_low,ci95_high,ci95_bonf_low,ci95_bonf_high,conclusion\n";

    for (const auto& c : comparisons) {
        out << c.scenario_id << ","
            << num_seeds << ","
            << c.variant_a << ","
            << c.variant_b << ","
            << c.metric_name << ","
            << c.unit << ","
            << (c.lower_is_better ? "true" : "false") << ","
            << std::fixed << std::setprecision(6) << c.ref_summary.mean << ","
            << std::fixed << std::setprecision(6) << c.ref_summary.median << ","
            << std::fixed << std::setprecision(6) << c.ref_summary.std_dev << ","
            << std::fixed << std::setprecision(6) << c.abl_summary.mean << ","
            << std::fixed << std::setprecision(6) << c.abl_summary.median << ","
            << std::fixed << std::setprecision(6) << c.abl_summary.std_dev << ","
            << std::fixed << std::setprecision(6) << c.paired_diff_mean << ","
            << std::fixed << std::setprecision(6) << c.paired_diff_median << ","
            << std::fixed << std::setprecision(6) << c.paired_diff_std << ","
            << std::fixed << std::setprecision(6) << c.ci95_low << ","
            << std::fixed << std::setprecision(6) << c.ci95_high << ","
            << std::fixed << std::setprecision(6) << c.ci95_bonferroni_low << ","
            << std::fixed << std::setprecision(6) << c.ci95_bonferroni_high << ","
            << "\"" << c.conclusion << "\"\n";
    }
    out.flush();
}

void write_failures_csv(const std::string& filepath, const std::vector<TrialFailureRecord>& failures) {
    std::ofstream out(filepath);
    if (!out.is_open()) return;
    out << "scenario_id,seed,variant,failure_stage,error_message\n";
    for (const auto& f : failures) {
        out << f.scenario_id << ","
            << f.seed << ","
            << f.variant << ","
            << f.failure_stage << ","
            << "\"" << f.error_message << "\"\n";
    }
    out.flush();
}

// ==============================================================================
// 6. Main Benchmark Runner
// ==============================================================================

int main(int argc, char* argv[]) {
    std::cout << "==============================================================================\n";
    std::cout << " AstroMesh-X Phase 6.1: Controlled Communication Payload Ablation Study\n";
    std::cout << "==============================================================================\n";
    std::cout.flush();

    std::string mode = "pilot";
    size_t num_seeds = 2;
    std::string prefix = "benchmark_phase61";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--smoke") {
            mode = "smoke";
            num_seeds = 1;
            prefix = "benchmark_phase61_smoke";
        } else if (arg == "--pilot") {
            mode = "pilot";
            num_seeds = 2;
            prefix = "benchmark_phase61_pilot";
        } else if (arg == "--full") {
            mode = "full";
            num_seeds = 30;
            prefix = "benchmark_phase61";
        } else if (arg == "--seeds" && i + 1 < argc) {
            num_seeds = static_cast<size_t>(std::stoul(argv[++i]));
            if (num_seeds == 1) {
                mode = "smoke";
                prefix = "benchmark_phase61_smoke";
            } else if (num_seeds == 2) {
                mode = "pilot";
                prefix = "benchmark_phase61_pilot";
            } else {
                mode = "full";
                prefix = "benchmark_phase61";
            }
        }
    }

    std::vector<BenchmarkScenarioConfig> scenarios = {
        {"SC1_Nominal_2R", "2-rover concentric loops, 0% loss, lossless channel", 2, "ConcentricLoops", 100, 18.0, 30.0, 8'000'000'000ULL, 200'000'000ULL, 2.0, 0.0, 0, 0, 25.0, 0, false},
        {"SC2_Loss10_2R", "2-rover concentric loops under 10% packet loss", 2, "ConcentricLoops", 100, 18.0, 30.0, 8'000'000'000ULL, 200'000'000ULL, 2.0, 0.10, 0, 0, 25.0, 0, false},
        {"SC3_Loss30_2R", "2-rover concentric loops under 30% packet loss", 2, "ConcentricLoops", 100, 18.0, 30.0, 8'000'000'000ULL, 200'000'000ULL, 2.0, 0.30, 0, 0, 25.0, 0, false},
        {"SC4_Loss50_2R", "2-rover concentric loops under 50% severe packet loss", 2, "ConcentricLoops", 100, 18.0, 30.0, 8'000'000'000ULL, 200'000'000ULL, 2.0, 0.50, 0, 0, 25.0, 0, false},
        {"SC5_Partition_Recon_2R", "2-rover concentric loops with partition (steps 15-28) & DTN recon", 2, "ConcentricLoops", 100, 18.0, 30.0, 8'000'000'000ULL, 200'000'000ULL, 2.0, 0.0, 15, 28, 25.0, 0, false},
        {"SC6_ConstrainedBudget_2R", "2-rover concentric loops under strict byte budget (1500 B)", 2, "ConcentricLoops", 100, 18.0, 30.0, 8'000'000'000ULL, 200'000'000ULL, 2.0, 0.10, 0, 0, 25.0, 1500, false},
        {"SC7_DegenerateGeometry_2R", "2-rover collinear degenerate geometry testing false closure rejection", 2, "ConcentricLoops", 50, 18.0, 30.0, 8'000'000'000ULL, 200'000'000ULL, 2.0, 0.0, 0, 0, 25.0, 0, true},
        {"SC8_LowOverlap_2R", "2-rover parallel search with minimal trajectory overlap", 2, "ParallelSearch", 50, 15.0, 40.0, 8'000'000'000ULL, 200'000'000ULL, 2.0, 0.0, 0, 0, 20.0, 0, false},
        {"SC9_Swarm_4R", "4-rover orthogonal crossing grid search, 10% loss", 4, "OverlappingGrid", 150, 18.0, 30.0, 8'000'000'000ULL, 200'000'000ULL, 2.0, 0.10, 0, 0, 25.0, 0, false},
        {"SC10_Swarm_8R", "8-rover multi-agent collective grid, 20% loss, multi-hop routing", 8, "OverlappingGrid", 200, 18.0, 30.0, 8'000'000'000ULL, 200'000'000ULL, 2.0, 0.20, 0, 0, 20.0, 0, false}
    };

    if (mode == "smoke") {
        scenarios.resize(1); // SC1 only
    }

    std::vector<uint64_t> seeds;
    seeds.reserve(num_seeds);
    for (size_t s = 1; s <= num_seeds; ++s) {
        seeds.push_back(1042ULL * s);
    }

    // List of Ablation Variants to evaluate per seed
    const std::vector<std::string> variant_names = {
        "AstroMesh_X_Full",         // Variant A: Full proposed pipeline
        "AstroMesh_X_RawExchange",   // Variant B: Direct raw LandmarkKey exchange (No IBLT)
        "AstroMesh_X_NoACKs",        // Variant C: Explicit ACKs disabled
        "AstroMesh_X_NoRetries",     // Variant D: DTN retransmissions disabled
        "AstroMesh_X_NoMultiHop",    // Variant E: Multi-hop routing disabled
        "AstroMesh_X_Minimal",       // Variant F: No ACKs, No Retries, No Multi-Hop
        "AstroMesh_v1_Baseline"      // Reference Baseline
    };

    const std::string trials_csv = prefix + "_trials.csv";
    const std::string summary_csv = prefix + "_summary.csv";
    const std::string failures_csv = prefix + "_failures.csv";

    std::ofstream trials_out(trials_csv);
    if (!trials_out.is_open()) {
        std::cerr << "CRITICAL ERROR: Failed to open " << trials_csv << " for writing!\n";
        return 1;
    }

    trials_out << "scenario_id,seed,variant,rover_count,duration_s,steps,packet_loss_rate,"
               << "partition_active,budget_constrained,degenerate_geometry,trajectory_type,runtime_us,"
               << "graph_nodes,graph_edges,optimization_mode,optimizer_calls,optimizer_iterations,"
               << "optimizer_failures,keyframes_retained,keyframe_prunes_applied,keyframe_prunes_rejected,"
               << "candidates_generated,closures_verified,closures_rejected,closures_true_accept,"
               << "closures_false_accept,precision,precision_valid,recall,recall_valid,"
               << "initial_dsu_components,final_dsu_components,first_merge_step,merge_persisted,"
               << "ate_raw_rmse_m,ate_aligned_swarm_rmse_m,ate_aligned_swarm_mean_m,"
               << "ate_aligned_swarm_median_m,ate_aligned_swarm_max_m,ate_aligned_valid,"
               << "ate_aligned_per_rover_mean_m,rpe_rmse_m,rpe_valid,logical_messages_sent,"
               << "logical_messages_delivered,logical_messages_dropped,unique_deliveries,"
               << "hop_transmissions_attempted,hop_transmissions_successful,hop_transmissions_dropped,"
               << "retransmissions_performed,payload_bytes,protocol_bytes,framing_bytes,data_wire_bytes,"
               << "ack_wire_bytes,total_wire_bytes,data_byte_conservation_valid,total_byte_conservation_valid,"
               << "app_completion_ratio,app_completion_ratio_valid,per_hop_delivery_ratio,"
               << "per_hop_delivery_ratio_valid,trial_status,failure_reason\n";
    trials_out.flush();

    const size_t total_trials = scenarios.size() * num_seeds * variant_names.size();

    std::cout << "[INFO] Commencing Phase 6.1 Ablation Study (" << mode << " mode): "
              << scenarios.size() << " Scenarios x " << num_seeds << " Seeds x "
              << variant_names.size() << " Variants = " << total_trials << " Pipeline Runs\n";
    std::cout << "[INFO] Streaming trial outputs to: " << trials_csv << "\n\n";
    std::cout.flush();

    std::vector<TrialFailureRecord> failures;
    std::vector<PairedAblationComparison> all_comparisons;

    const auto total_start = std::chrono::high_resolution_clock::now();
    size_t completed_runs = 0;

    for (const auto& sc : scenarios) {
        std::cout << ">>> Running Scenario [" << sc.id << "]: " << sc.description << " (" << sc.rover_count << " Rovers)..." << std::endl;
        const auto sc_start = std::chrono::high_resolution_clock::now();

        // Map: variant_name -> metric_name -> vector<double>
        std::map<std::string, std::map<std::string, std::vector<double>>> sc_metrics;

        for (uint64_t seed : seeds) {
            // 1. Setup deterministic world
            WorldConfig wcfg;
            wcfg.num_landmarks = sc.num_landmarks;
            wcfg.sensor_range = sc.sensor_range;
            wcfg.seed = seed ^ 0x9e3779b97f4a7c15ULL;
            if (sc.degenerate_geometry) {
                wcfg.min_x = -sc.arena_extent;
                wcfg.max_x = sc.arena_extent;
                wcfg.min_y = -0.5;
                wcfg.max_y = 0.5;
            } else {
                wcfg.min_x = -sc.arena_extent;
                wcfg.max_x = sc.arena_extent;
                wcfg.min_y = -sc.arena_extent;
                wcfg.max_y = sc.arena_extent;
            }
            SyntheticWorld2D world(wcfg);

            // 2. Setup ground truth trajectories
            TrajectoryConfig tcfg;
            tcfg.num_rovers = sc.rover_count;
            tcfg.duration_ns = sc.duration_ns;
            tcfg.step_dt_ns = sc.step_dt_ns;
            tcfg.speed = sc.speed;
            tcfg.arena_extent = sc.arena_extent;
            tcfg.seed = seed ^ 0x517cc1b727220a95ULL;

            std::vector<RoverTrajectory> trajectories;
            if (sc.trajectory_type == "ConcentricLoops") {
                trajectories = SwarmTrajectoryGenerator::generate_concentric_loops(tcfg);
            } else if (sc.trajectory_type == "OverlappingGrid") {
                trajectories = SwarmTrajectoryGenerator::generate_overlapping_grid(tcfg);
            } else {
                trajectories = SwarmTrajectoryGenerator::generate_parallel_search(tcfg);
            }

            const size_t num_steps = static_cast<size_t>(sc.duration_ns / sc.step_dt_ns) + 1;

            // 3. Observation model
            ObservationNoiseConfig obs_noise;
            obs_noise.range_std_dev = sc.obs_noise_range;
            obs_noise.bearing_std_dev = sc.obs_noise_bearing;
            obs_noise.seed = seed ^ 0x3141592653589793ULL;
            ObservationModel2D obs_model(obs_noise);

            // 4. Precompute identical step inputs
            std::vector<SimulationStepInput> step_inputs;
            step_inputs.reserve(num_steps);
            size_t ground_truth_loop_opportunities = 0;

            for (size_t s = 0; s < num_steps; ++s) {
                SimulationStepInput step;
                step.timestamp_ns = s * sc.step_dt_ns;

                std::map<uint16_t, std::vector<LandmarkObservation>> step_obs;
                for (size_t r = 0; r < sc.rover_count; ++r) {
                    RoverStepData rdata;
                    rdata.rover_id = static_cast<uint16_t>(r + 1);
                    rdata.timestamp_ns = step.timestamp_ns;
                    if (s == 0) {
                        rdata.odometry_step = geometry::SE2::identity();
                    } else {
                        const auto& prev_pose = trajectories[r].waypoints[s - 1].pose;
                        const auto& curr_pose = trajectories[r].waypoints[s].pose;
                        rdata.odometry_step = prev_pose.inverse().compose(curr_pose);
                    }
                    rdata.observations = obs_model.observe(world, trajectories[r].waypoints[s].pose, rdata.rover_id);
                    step_obs[rdata.rover_id] = rdata.observations;
                    step.rover_steps.push_back(rdata);
                }

                for (size_t i = 0; i < sc.rover_count; ++i) {
                    for (size_t j = i + 1; j < sc.rover_count; ++j) {
                        const uint16_t id_a = static_cast<uint16_t>(i + 1);
                        const uint16_t id_b = static_cast<uint16_t>(j + 1);
                        size_t shared_count = 0;
                        for (const auto& oa : step_obs[id_a]) {
                            for (const auto& ob : step_obs[id_b]) {
                                if (oa.landmark_id == ob.landmark_id) {
                                    shared_count++;
                                    break;
                                }
                            }
                        }
                        if (shared_count >= 4) {
                            ground_truth_loop_opportunities++;
                        }
                    }
                }

                const bool is_partition = (sc.partition_end_step > sc.partition_start_step) &&
                                          (s >= sc.partition_start_step && s < sc.partition_end_step);

                for (size_t i = 0; i < sc.rover_count; ++i) {
                    for (size_t j = i + 1; j < sc.rover_count; ++j) {
                        const auto& pose_a = trajectories[i].waypoints[s].pose;
                        const auto& pose_b = trajectories[j].waypoints[s].pose;
                        const double dist = std::hypot(pose_a.x() - pose_b.x(), pose_a.y() - pose_b.y());

                        ChannelLinkState link;
                        link.rover_a = static_cast<uint16_t>(i + 1);
                        link.rover_b = static_cast<uint16_t>(j + 1);
                        link.in_communication_range = (dist <= sc.comm_range);
                        link.is_partitioned = is_partition;
                        link.link_packet_loss_rate = sc.packet_loss_rate;
                        step.channel_states.push_back(link);
                    }
                }

                step_inputs.push_back(step);
            }

            // 5. Build ground truth pose map
            std::unordered_map<graph::NodeId, geometry::SE2> gt_pose_map;
            for (size_t r = 0; r < sc.rover_count; ++r) {
                const uint16_t rid = static_cast<uint16_t>(r + 1);
                for (size_t s = 0; s < num_steps; ++s) {
                    gt_pose_map[graph::NodeId(rid, static_cast<uint32_t>(s))] = trajectories[r].waypoints[s].pose;
                }
            }

            // 6. Base configuration factory for AstroMesh-X
            auto make_base_x_config = [&]() {
                AstroMeshXConfig cfg;
                cfg.seed = seed;
                cfg.communication_range = sc.comm_range;
                cfg.odometry_noise_trans = sc.odom_noise_trans;
                cfg.odometry_noise_rot = sc.odom_noise_rot;
                cfg.packet_loss_rate = sc.packet_loss_rate;
                cfg.enable_cord = true;
                cfg.cord_options.max_steps = 40;
                cfg.cord_options.timestep = 0.02;
                cfg.enable_dtn_retries = true;
                cfg.enable_explicit_acks = true;
                cfg.enable_multihop_routing = true;
                cfg.enable_keyframe_pruning = true;
                cfg.pruner_config.max_retained_keyframes = 100;
                cfg.pruner_config.protect_articulation_points = false;
                cfg.pruner_config.protect_gauge_anchors = true;
                if (sc.epoch_quota > 0) {
                    cfg.budget_config.max_bytes_per_window = sc.epoch_quota;
                }
                return cfg;
            };

            // Evaluate all variants under identical input stream:
            for (const auto& var_name : variant_names) {
                PipelineTrialRecord rec;

                if (var_name == "AstroMesh_X_Full") {
                    auto cfg = make_base_x_config();
                    cfg.custom_pipeline_name = "AstroMesh_X_Full";
                    AstroMeshXPipeline p(cfg);
                    rec = evaluate_pipeline_instance(p, sc, seed, step_inputs, trajectories, gt_pose_map, ground_truth_loop_opportunities, true, failures);
                } else if (var_name == "AstroMesh_X_RawExchange") {
                    auto cfg = make_base_x_config();
                    cfg.custom_pipeline_name = "AstroMesh_X_RawExchange";
                    cfg.discovery_mechanism = AstroMeshXConfig::DiscoveryMechanism::RawExchange; // No IBLT
                    AstroMeshXPipeline p(cfg);
                    rec = evaluate_pipeline_instance(p, sc, seed, step_inputs, trajectories, gt_pose_map, ground_truth_loop_opportunities, true, failures);
                } else if (var_name == "AstroMesh_X_NoACKs") {
                    auto cfg = make_base_x_config();
                    cfg.custom_pipeline_name = "AstroMesh_X_NoACKs";
                    cfg.enable_explicit_acks = false; // No ACKs
                    AstroMeshXPipeline p(cfg);
                    rec = evaluate_pipeline_instance(p, sc, seed, step_inputs, trajectories, gt_pose_map, ground_truth_loop_opportunities, true, failures);
                } else if (var_name == "AstroMesh_X_NoRetries") {
                    auto cfg = make_base_x_config();
                    cfg.custom_pipeline_name = "AstroMesh_X_NoRetries";
                    cfg.enable_dtn_retries = false; // No Retries
                    AstroMeshXPipeline p(cfg);
                    rec = evaluate_pipeline_instance(p, sc, seed, step_inputs, trajectories, gt_pose_map, ground_truth_loop_opportunities, true, failures);
                } else if (var_name == "AstroMesh_X_NoMultiHop") {
                    auto cfg = make_base_x_config();
                    cfg.custom_pipeline_name = "AstroMesh_X_NoMultiHop";
                    cfg.enable_multihop_routing = false; // No Multi-Hop
                    AstroMeshXPipeline p(cfg);
                    rec = evaluate_pipeline_instance(p, sc, seed, step_inputs, trajectories, gt_pose_map, ground_truth_loop_opportunities, true, failures);
                } else if (var_name == "AstroMesh_X_Minimal") {
                    auto cfg = make_base_x_config();
                    cfg.custom_pipeline_name = "AstroMesh_X_Minimal";
                    cfg.enable_explicit_acks = false;
                    cfg.enable_dtn_retries = false;
                    cfg.enable_multihop_routing = false;
                    AstroMeshXPipeline p(cfg);
                    rec = evaluate_pipeline_instance(p, sc, seed, step_inputs, trajectories, gt_pose_map, ground_truth_loop_opportunities, true, failures);
                } else if (var_name == "AstroMesh_v1_Baseline") {
                    BaselineV1Config bcfg;
                    bcfg.seed = seed;
                    bcfg.communication_range = sc.comm_range;
                    bcfg.odometry_noise_trans = sc.odom_noise_trans;
                    bcfg.odometry_noise_rot = sc.odom_noise_rot;
                    BaselineAstroMeshV1 p(bcfg);
                    rec = evaluate_pipeline_instance(p, sc, seed, step_inputs, trajectories, gt_pose_map, ground_truth_loop_opportunities, false, failures);
                }

                write_trial_row(trials_out, rec);
                completed_runs++;

                sc_metrics[var_name]["total_wire_bytes"].push_back(static_cast<double>(rec.total_wire_bytes));
                sc_metrics[var_name]["data_wire_bytes"].push_back(static_cast<double>(rec.data_wire_bytes));
                sc_metrics[var_name]["ack_wire_bytes"].push_back(static_cast<double>(rec.ack_wire_bytes));
                sc_metrics[var_name]["app_completion_ratio"].push_back(rec.app_completion_ratio);
                sc_metrics[var_name]["per_hop_delivery_ratio"].push_back(rec.per_hop_delivery_ratio);
                sc_metrics[var_name]["closures_verified"].push_back(static_cast<double>(rec.closures_verified));
                sc_metrics[var_name]["candidates_generated"].push_back(static_cast<double>(rec.candidates_generated));
                sc_metrics[var_name]["final_dsu_components"].push_back(static_cast<double>(rec.final_dsu_components));
                sc_metrics[var_name]["ate_aligned_swarm_rmse_m"].push_back(rec.ate_aligned_swarm_rmse_m);
                sc_metrics[var_name]["runtime_us"].push_back(static_cast<double>(rec.runtime_us));
            }
        }

        const auto sc_end = std::chrono::high_resolution_clock::now();
        const auto sc_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(sc_end - sc_start).count();

        // Perform statistical comparisons against AstroMesh_X_Full reference
        const std::string ref_var = "AstroMesh_X_Full";
        const size_t num_ablations = variant_names.size() - 1; // 6 comparisons

        for (const auto& comp_var : variant_names) {
            if (comp_var == ref_var) continue;

            all_comparisons.push_back(analyze_ablation_pair(sc.id, ref_var, comp_var, "Total_Transmitted_Wire_Bytes", "bytes", true,
                sc_metrics[ref_var]["total_wire_bytes"], sc_metrics[comp_var]["total_wire_bytes"], num_ablations));
            all_comparisons.push_back(analyze_ablation_pair(sc.id, ref_var, comp_var, "Data_Wire_Bytes", "bytes", true,
                sc_metrics[ref_var]["data_wire_bytes"], sc_metrics[comp_var]["data_wire_bytes"], num_ablations));
            all_comparisons.push_back(analyze_ablation_pair(sc.id, ref_var, comp_var, "ACK_Wire_Bytes", "bytes", true,
                sc_metrics[ref_var]["ack_wire_bytes"], sc_metrics[comp_var]["ack_wire_bytes"], num_ablations));
            all_comparisons.push_back(analyze_ablation_pair(sc.id, ref_var, comp_var, "Application_Completion_Ratio", "ratio", false,
                sc_metrics[ref_var]["app_completion_ratio"], sc_metrics[comp_var]["app_completion_ratio"], num_ablations));
            all_comparisons.push_back(analyze_ablation_pair(sc.id, ref_var, comp_var, "Verified_Loop_Closures", "count", false,
                sc_metrics[ref_var]["closures_verified"], sc_metrics[comp_var]["closures_verified"], num_ablations));
            all_comparisons.push_back(analyze_ablation_pair(sc.id, ref_var, comp_var, "Candidates_Generated", "count", true,
                sc_metrics[ref_var]["candidates_generated"], sc_metrics[comp_var]["candidates_generated"], num_ablations));
            all_comparisons.push_back(analyze_ablation_pair(sc.id, ref_var, comp_var, "Final_DSU_Components", "count", true,
                sc_metrics[ref_var]["final_dsu_components"], sc_metrics[comp_var]["final_dsu_components"], num_ablations));
            all_comparisons.push_back(analyze_ablation_pair(sc.id, ref_var, comp_var, "ATE_Aligned_Swarm_RMSE", "meters", true,
                sc_metrics[ref_var]["ate_aligned_swarm_rmse_m"], sc_metrics[comp_var]["ate_aligned_swarm_rmse_m"], num_ablations));
        }

        std::cout << "    [SCENARIO DONE] " << sc_elapsed_ms << " ms | Wire Bytes: "
                  << "Full=" << compute_summary(sc_metrics["AstroMesh_X_Full"]["total_wire_bytes"]).mean << " B, "
                  << "RawEx=" << compute_summary(sc_metrics["AstroMesh_X_RawExchange"]["total_wire_bytes"]).mean << " B, "
                  << "NoACKs=" << compute_summary(sc_metrics["AstroMesh_X_NoACKs"]["total_wire_bytes"]).mean << " B, "
                  << "Base=" << compute_summary(sc_metrics["AstroMesh_v1_Baseline"]["total_wire_bytes"]).mean << " B\n" << std::endl;
    }

    const auto total_end = std::chrono::high_resolution_clock::now();
    const auto total_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start).count();

    std::cout << "==============================================================================\n";
    std::cout << " [" << mode << " COMPLETED] " << completed_runs << " Runs Executed in " << total_elapsed_ms
              << " ms (" << (completed_runs > 0 ? (total_elapsed_ms / static_cast<double>(completed_runs)) : 0.0)
              << " ms per pipeline run)\n";
    std::cout << "==============================================================================\n\n";

    std::cout << "Writing summary to: " << summary_csv << " ...\n";
    write_summary_csv(summary_csv, all_comparisons, num_seeds);

    std::cout << "Writing failures to: " << failures_csv << " ...\n";
    write_failures_csv(failures_csv, failures);

    std::cout << "Recorded failures: " << failures.size() << "\n";
    std::cout << "==============================================================================\n";

    return 0;
}

