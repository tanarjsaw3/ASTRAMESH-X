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
    std::string trajectory_type{"ConcentricLoops"}; // "ConcentricLoops", "OverlappingGrid", "ParallelSearch"
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
    size_t epoch_quota{0};                  // 0 = unconstrained byte quota
    bool degenerate_geometry{false};        // Collinear landmarks along y ~ 0
    double odom_noise_trans{0.02};          // 2 cm translation noise std dev
    double odom_noise_rot{0.005};           // ~0.28 deg orientation noise std dev
    double obs_noise_range{0.03};           // 3 cm observation range noise std dev
    double obs_noise_bearing{0.01};         // ~0.57 deg observation bearing noise std dev
};

// ==============================================================================
// 2. Metrics Recording Structures
// ==============================================================================

struct PipelineTrialRecord {
    std::string scenario_id;
    uint64_t seed{0};
    std::string pipeline_name;
    size_t rover_count{0};
    double duration_s{0.0};
    size_t steps{0};
    double packet_loss_rate{0.0};
    bool partition_active{false};
    bool budget_constrained{false};
    bool degenerate_geometry{false};
    std::string trajectory_type;
    uint64_t runtime_us{0};

    // Graph Complexity & Optimizer Telemetry
    size_t graph_nodes{0};
    size_t graph_edges{0};
    std::string optimization_mode{"None"};
    size_t optimizer_calls{0};
    size_t optimizer_iterations{0};
    size_t optimizer_failures{0};

    // Keyframe & Topology Retention
    size_t keyframes_retained{0};
    size_t keyframe_prunes_applied{0};
    size_t keyframe_prunes_rejected{0};

    // Loop Closure and Verification
    size_t candidates_generated{0};
    size_t closures_verified{0};
    size_t closures_rejected{0};
    size_t closures_true_accept{0};
    size_t closures_false_accept{0};
    double precision{0.0};
    bool precision_valid{false};
    double recall{0.0};
    bool recall_valid{false};

    // Topology & Connectivity Recovery
    size_t initial_dsu_components{0};
    size_t final_dsu_components{0};
    int64_t first_merge_step{-1};
    bool merge_persisted{false};

    // Trajectory Accuracy (ATE & RPE)
    double ate_raw_rmse_m{0.0};
    double ate_aligned_swarm_rmse_m{0.0};
    double ate_aligned_swarm_mean_m{0.0};
    double ate_aligned_swarm_median_m{0.0};
    double ate_aligned_swarm_max_m{0.0};
    bool ate_aligned_valid{false};
    double ate_aligned_per_rover_mean_m{0.0};
    double rpe_rmse_m{0.0};
    bool rpe_valid{false};

    // Communication & Byte Accounting
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
    size_t wire_bytes_received{0};

    // Invariant Validity Flags
    bool data_byte_conservation_valid{false};
    bool total_byte_conservation_valid{false};
    double app_completion_ratio{0.0};
    bool app_completion_ratio_valid{false};
    double per_hop_delivery_ratio{0.0};
    bool per_hop_delivery_ratio_valid{false};

    std::string trial_status{"Success"};
    std::string failure_reason;
};

struct PairedTrialRecord {
    std::string scenario_id;
    uint64_t seed{0};
    PipelineTrialRecord baseline;
    PipelineTrialRecord proposed;
};

struct TrialFailureRecord {
    std::string scenario_id;
    uint64_t seed{0};
    std::string pipeline_name;
    std::string failure_stage;
    std::string error_message;
};

// ==============================================================================
// 3. Statistical Aggregations (Means, Medians, CI 95%)
// ==============================================================================

struct MetricSummary {
    double mean{0.0};
    double median{0.0};
    double std_dev{0.0};
    double p25{0.0};
    double p75{0.0};
    double p95{0.0};
};

MetricSummary compute_summary(std::vector<double> vals) {
    MetricSummary s;
    if (vals.empty()) return s;

    std::sort(vals.begin(), vals.end());
    const size_t n = vals.size();

    double sum = std::accumulate(vals.begin(), vals.end(), 0.0);
    s.mean = sum / static_cast<double>(n);

    if (n % 2 == 1) {
        s.median = vals[n / 2];
    } else {
        s.median = 0.5 * (vals[n / 2 - 1] + vals[n / 2]);
    }

    s.p25 = vals[n / 4];
    s.p75 = vals[(3 * n) / 4];
    s.p95 = vals[std::min(n - 1, (95 * n) / 100)];

    if (n > 1) {
        double sq_sum = 0.0;
        for (double v : vals) {
            sq_sum += (v - s.mean) * (v - s.mean);
        }
        s.std_dev = std::sqrt(sq_sum / static_cast<double>(n - 1));
    }
    return s;
}

struct PairedMetricComparison {
    std::string metric_name;
    std::string unit;
    bool lower_is_better{true};
    MetricSummary baseline_summary;
    MetricSummary proposed_summary;
    double paired_diff_mean{0.0};
    double paired_diff_median{0.0};
    double paired_diff_std{0.0};
    double ci95_low{0.0};
    double ci95_high{0.0};
    std::string conclusion;
};

PairedMetricComparison analyze_paired_metric(
    const std::string& name,
    const std::string& unit,
    bool lower_is_better,
    const std::vector<double>& base_vals,
    const std::vector<double>& prop_vals) {

    PairedMetricComparison comp;
    comp.metric_name = name;
    comp.unit = unit;
    comp.lower_is_better = lower_is_better;

    comp.baseline_summary = compute_summary(base_vals);
    comp.proposed_summary = compute_summary(prop_vals);

    const size_t n = std::min(base_vals.size(), prop_vals.size());
    if (n == 0) return comp;

    std::vector<double> diffs(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        diffs[i] = prop_vals[i] - base_vals[i];
    }
    MetricSummary diff_sum = compute_summary(diffs);
    comp.paired_diff_mean = diff_sum.mean;
    comp.paired_diff_median = diff_sum.median;
    comp.paired_diff_std = diff_sum.std_dev;

    // Student's t critical values: nu=1 (N=2) -> 12.706; nu=29 (N=30) -> 2.045
    const double t_crit = (n <= 2) ? 12.7062 : 2.04523;
    const double se = (n > 1) ? (diff_sum.std_dev / std::sqrt(static_cast<double>(n))) : 0.0;
    comp.ci95_low = diff_sum.mean - t_crit * se;
    comp.ci95_high = diff_sum.mean + t_crit * se;

    if (lower_is_better) {
        if (comp.ci95_high < 0.0) {
            comp.conclusion = "AstroMesh-X Superior (p < 0.05)";
        } else if (comp.ci95_low > 0.0) {
            comp.conclusion = "Baseline Superior (p < 0.05)";
        } else {
            comp.conclusion = "Statistically Indistinguishable";
        }
    } else {
        if (comp.ci95_low > 0.0) {
            comp.conclusion = "AstroMesh-X Superior (p < 0.05)";
        } else if (comp.ci95_high < 0.0) {
            comp.conclusion = "Baseline Superior (p < 0.05)";
        } else {
            comp.conclusion = "Statistically Indistinguishable";
        }
    }

    return comp;
}

// ==============================================================================
// 4. Paired Trial Execution Function
// ==============================================================================

PairedTrialRecord run_single_paired_trial(
    const BenchmarkScenarioConfig& sc,
    uint64_t seed,
    std::vector<TrialFailureRecord>& failures) {

    PairedTrialRecord paired;
    paired.scenario_id = sc.id;
    paired.seed = seed;

    // 1. Setup Deterministic Synthetic World
    WorldConfig wcfg;
    wcfg.num_landmarks = sc.num_landmarks;
    wcfg.sensor_range = sc.sensor_range;
    wcfg.seed = seed ^ 0x9e3779b97f4a7c15ULL; // Seed separation for world

    if (sc.degenerate_geometry) {
        // Collinear landmarks along y in [-0.5, 0.5]
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

    std::unique_ptr<SyntheticWorld2D> world;
    try {
        world = std::make_unique<SyntheticWorld2D>(wcfg);
    } catch (const std::exception& e) {
        failures.push_back({sc.id, seed, "Shared", "WorldCreation", e.what()});
        return paired;
    }

    // 2. Setup Ground-Truth Trajectories
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

    // 3. Setup Observation Model
    ObservationNoiseConfig obs_noise;
    obs_noise.range_std_dev = sc.obs_noise_range;
    obs_noise.bearing_std_dev = sc.obs_noise_bearing;
    obs_noise.seed = seed ^ 0x3141592653589793ULL;
    ObservationModel2D obs_model(obs_noise);

    // 4. Precompute Simulation Steps (Identical Stream for Both Pipelines)
    std::vector<SimulationStepInput> step_inputs;
    step_inputs.reserve(num_steps);

    // Track ground truth loop opportunities (|Obs_A ∩ Obs_B| >= 4)
    size_t ground_truth_loop_opportunities = 0;

    for (size_t s = 0; s < num_steps; ++s) {
        SimulationStepInput step;
        step.timestamp_ns = s * sc.step_dt_ns;

        // Rover steps
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

            rdata.observations = obs_model.observe(*world, trajectories[r].waypoints[s].pose, rdata.rover_id);
            step_obs[rdata.rover_id] = rdata.observations;
            step.rover_steps.push_back(rdata);
        }

        // Count ground-truth co-observations
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

        // Pairwise channel states
        const bool is_partition_step = (sc.partition_end_step > sc.partition_start_step) &&
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
                link.is_partitioned = is_partition_step;
                link.link_packet_loss_rate = sc.packet_loss_rate;

                step.channel_states.push_back(link);
            }
        }

        step_inputs.push_back(step);
    }

    // 5. Build Ground-Truth Pose Map
    std::unordered_map<graph::NodeId, geometry::SE2> gt_pose_map;
    for (size_t r = 0; r < sc.rover_count; ++r) {
        const uint16_t rid = static_cast<uint16_t>(r + 1);
        for (size_t s = 0; s < num_steps; ++s) {
            gt_pose_map[graph::NodeId(rid, static_cast<uint32_t>(s))] = trajectories[r].waypoints[s].pose;
        }
    }

    // Helper: evaluate a pipeline
    auto evaluate_pipeline = [&](IRoverSwarmPipeline& pipeline, bool is_proposed) -> PipelineTrialRecord {
        PipelineTrialRecord rec;
        rec.scenario_id = sc.id;
        rec.seed = seed;
        rec.pipeline_name = pipeline.pipeline_name();
        rec.rover_count = sc.rover_count;
        rec.duration_s = static_cast<double>(sc.duration_ns) * 1e-9;
        rec.steps = num_steps;
        rec.packet_loss_rate = sc.packet_loss_rate;
        rec.partition_active = (sc.partition_end_step > sc.partition_start_step);
        rec.budget_constrained = (sc.epoch_quota > 0);
        rec.degenerate_geometry = sc.degenerate_geometry;
        rec.trajectory_type = sc.trajectory_type;
        rec.initial_dsu_components = sc.rover_count;

        const auto start_time = std::chrono::high_resolution_clock::now();

        // Feed step inputs
        int64_t merge_step = -1;
        for (size_t s = 0; s < step_inputs.size(); ++s) {
            if (!pipeline.ingest_step(step_inputs[s])) {
                rec.trial_status = "Failed";
                rec.failure_reason = "IngestStepRejected";
                failures.push_back({sc.id, seed, rec.pipeline_name, "Ingest", "Step rejected"});
                return rec;
            }

            if (merge_step < 0 && pipeline.num_connected_components() < sc.rover_count) {
                merge_step = static_cast<int64_t>(s);
            }
        }

        // Run optimization if supported (at checkpoint: mission end)
        if (is_proposed) {
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

        // Collect metrics snapshot
        const auto m = pipeline.collect_metrics();
        rec.optimization_mode = m.optimization_mode;
        rec.optimizer_iterations = m.optimization_steps_performed;
        rec.keyframes_retained = m.total_keyframes_retained;
        rec.keyframe_prunes_applied = m.keyframe_prunes_applied;
        rec.keyframe_prunes_rejected = m.keyframe_prunes_rejected;
        rec.candidates_generated = m.candidate_pairs_proposed;
        rec.closures_verified = m.loop_closures_verified;
        rec.closures_rejected = m.loop_closures_rejected;

        // Graph size tracking
        if (is_proposed) {
            auto* astromesh_ptr = dynamic_cast<AstroMeshXPipeline*>(&pipeline);
            if (astromesh_ptr) {
                rec.graph_nodes = astromesh_ptr->swarm_graph().num_nodes();
                rec.graph_edges = astromesh_ptr->swarm_graph().num_edges();
            }
        } else {
            rec.graph_nodes = sc.rover_count * num_steps;
            rec.graph_edges = sc.rover_count * (num_steps - 1);
        }

        // Evaluate Loop Closure True/False Acceptances
        const auto history = pipeline.get_loop_closure_history();
        for (const auto& cand : history) {
            if (cand.verification_passed) {
                // Find ground truth relative transform between rovers at candidate timestamp
                const size_t s_idx = std::min(num_steps - 1, static_cast<size_t>(cand.timestamp_ns / sc.step_dt_ns));
                const auto& p_gt_a = trajectories[cand.rover_a - 1].waypoints[s_idx].pose;
                const auto& p_gt_b = trajectories[cand.rover_b - 1].waypoints[s_idx].pose;
                const auto t_gt_ab = p_gt_a.inverse().compose(p_gt_b);

                const auto& est_ab = cand.estimated_relative_transform;
                const double dx = est_ab.x - t_gt_ab.x();
                const double dy = est_ab.y - t_gt_ab.y();
                const double dtheta = std::abs(graph::normalize_angle(est_ab.theta - t_gt_ab.theta()));
                const double trans_err = std::hypot(dx, dy);

                // Documented verification rule: trans error <= 1.0 m, rot error <= 0.35 rad (~20 deg)
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

        // Communication metrics & wire byte accounting
        rec.logical_messages_sent = m.logical_messages_sent;
        rec.logical_messages_delivered = m.logical_messages_delivered;
        rec.logical_messages_dropped = m.logical_messages_dropped;
        rec.unique_deliveries = m.unique_deliveries;
        rec.hop_transmissions_attempted = m.hop_transmissions_attempted;
        rec.hop_transmissions_successful = m.hop_transmissions_successful;
        rec.hop_transmissions_dropped = m.logical_messages_dropped; // mapped dropped attempts
        rec.retransmissions_performed = m.retransmissions_performed;
        rec.payload_bytes = m.payload_bytes_transmitted;
        rec.protocol_bytes = m.protocol_bytes_transmitted;
        rec.framing_bytes = m.framing_bytes_transmitted;
        rec.data_wire_bytes = m.data_wire_bytes_transmitted;
        rec.ack_wire_bytes = m.ack_wire_bytes_transmitted;
        rec.total_wire_bytes = m.total_wire_bytes_transmitted;

        // Byte conservation assertions
        rec.data_byte_conservation_valid = (m.data_wire_bytes_transmitted ==
            (m.payload_bytes_transmitted + m.protocol_bytes_transmitted + m.framing_bytes_transmitted));
        rec.total_byte_conservation_valid = (m.total_wire_bytes_transmitted ==
            (m.data_wire_bytes_transmitted + m.ack_wire_bytes_transmitted));

        rec.app_completion_ratio = m.application_completion_ratio;
        rec.app_completion_ratio_valid = (m.logical_messages_sent > 0);

        rec.per_hop_delivery_ratio = m.per_hop_delivery_ratio;
        rec.per_hop_delivery_ratio_valid = (m.hop_transmissions_attempted > 0);

        // Trajectory evaluation (ATE & RPE)
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

        if (ate_raw.valid) {
            rec.ate_raw_rmse_m = ate_raw.translation.rmse;
        }
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
    };

    // 6. Run Baseline Pipeline
    BaselineV1Config bcfg;
    bcfg.seed = seed;
    bcfg.communication_range = sc.comm_range;
    bcfg.odometry_noise_trans = sc.odom_noise_trans;
    bcfg.odometry_noise_rot = sc.odom_noise_rot;
    BaselineAstroMeshV1 baseline(bcfg);

    paired.baseline = evaluate_pipeline(baseline, false);

    // 7. Run AstroMesh-X Proposed Pipeline
    AstroMeshXConfig xcfg;
    xcfg.seed = seed;
    xcfg.communication_range = sc.comm_range;
    xcfg.odometry_noise_trans = sc.odom_noise_trans;
    xcfg.odometry_noise_rot = sc.odom_noise_rot;
    xcfg.packet_loss_rate = sc.packet_loss_rate;
    xcfg.enable_cord = true;
    xcfg.cord_options.max_steps = 40;  // Bounded CORD relaxation (partial Riemannian relaxation, not asymptotic convergence)
    xcfg.cord_options.timestep = 0.02; // Stable dissipative step size
    xcfg.enable_dtn_retries = true;
    xcfg.enable_explicit_acks = true;
    xcfg.enable_multihop_routing = true;
    xcfg.enable_keyframe_pruning = true;
    xcfg.pruner_config.max_retained_keyframes = 100;
    xcfg.pruner_config.protect_articulation_points = false; // Allow safe pruning of redundant intermediate chain
    xcfg.pruner_config.protect_gauge_anchors = true;

    if (sc.epoch_quota > 0) {
        xcfg.budget_config.max_bytes_per_window = sc.epoch_quota;
    }

    AstroMeshXPipeline proposed(xcfg);

    paired.proposed = evaluate_pipeline(proposed, true);

    return paired;
}

// ==============================================================================
// 5. Incremental & Summary CSV Exporters
// ==============================================================================

void write_trial_row_to_stream(std::ostream& out, const PipelineTrialRecord& r) {
    out << r.scenario_id << ","
        << r.seed << ","
        << r.pipeline_name << ","
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

void write_failures_csv(const std::string& filepath, const std::vector<TrialFailureRecord>& failures) {
    std::ofstream out(filepath);
    if (!out.is_open()) return;

    out << "scenario_id,seed,pipeline_name,failure_stage,error_message\n";
    for (const auto& f : failures) {
        out << f.scenario_id << ","
            << f.seed << ","
            << f.pipeline_name << ","
            << f.failure_stage << ","
            << "\"" << f.error_message << "\"\n";
    }
    out.flush();
}

void write_summary_csv(
    const std::string& filepath,
    const std::map<std::string, std::vector<PairedMetricComparison>>& scenario_summaries,
    size_t num_seeds) {

    std::ofstream out(filepath);
    if (!out.is_open()) return;

    out << "scenario_id,num_paired_seeds,metric_name,unit,lower_is_better,"
        << "baseline_mean,baseline_median,baseline_std,proposed_mean,proposed_median,proposed_std,"
        << "paired_diff_mean,paired_diff_median,paired_diff_std,ci95_low,ci95_high,conclusion\n";

    for (const auto& [sc_id, comparisons] : scenario_summaries) {
        for (const auto& c : comparisons) {
            out << sc_id << ","
                << num_seeds << ","
                << c.metric_name << ","
                << c.unit << ","
                << (c.lower_is_better ? "true" : "false") << ","
                << std::fixed << std::setprecision(6) << c.baseline_summary.mean << ","
                << std::fixed << std::setprecision(6) << c.baseline_summary.median << ","
                << std::fixed << std::setprecision(6) << c.baseline_summary.std_dev << ","
                << std::fixed << std::setprecision(6) << c.proposed_summary.mean << ","
                << std::fixed << std::setprecision(6) << c.proposed_summary.median << ","
                << std::fixed << std::setprecision(6) << c.proposed_summary.std_dev << ","
                << std::fixed << std::setprecision(6) << c.paired_diff_mean << ","
                << std::fixed << std::setprecision(6) << c.paired_diff_median << ","
                << std::fixed << std::setprecision(6) << c.paired_diff_std << ","
                << std::fixed << std::setprecision(6) << c.ci95_low << ","
                << std::fixed << std::setprecision(6) << c.ci95_high << ","
                << "\"" << c.conclusion << "\"\n";
        }
    }
    out.flush();
}

// ==============================================================================
// 6. Main Benchmark Runner
// ==============================================================================

int main(int argc, char* argv[]) {
    std::cout << "==============================================================================\n";
    std::cout << " AstroMesh-X Phase 5.4: Bounded Monte Carlo Benchmark & Evaluation\n";
    std::cout << "==============================================================================\n";
    std::cout.flush();

    // Parse CLI arguments: --smoke, --pilot, --full, --seeds <N>
    std::string mode = "pilot"; // "smoke", "pilot", "full"
    size_t num_seeds = 2;
    std::string prefix = "benchmark_phase54_pilot";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--smoke") {
            mode = "smoke";
            num_seeds = 1;
            prefix = "benchmark_phase54_smoke";
        } else if (arg == "--pilot") {
            mode = "pilot";
            num_seeds = 2;
            prefix = "benchmark_phase54_pilot";
        } else if (arg == "--full") {
            mode = "full";
            num_seeds = 30;
            prefix = "benchmark_phase54";
        } else if (arg == "--seeds" && i + 1 < argc) {
            num_seeds = static_cast<size_t>(std::stoul(argv[++i]));
            if (num_seeds == 1) {
                mode = "smoke";
                prefix = "benchmark_phase54_smoke";
            } else if (num_seeds == 2) {
                mode = "pilot";
                prefix = "benchmark_phase54_pilot";
            } else {
                mode = "full";
                prefix = "benchmark_phase54";
            }
        }
    }

    // 10 Balanced Deterministic Benchmark Scenarios
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
        scenarios.resize(1); // Only run SC1 for smoke test
    }

    std::vector<uint64_t> seeds;
    seeds.reserve(num_seeds);
    for (size_t s = 1; s <= num_seeds; ++s) {
        seeds.push_back(1042ULL * s);
    }

    std::vector<TrialFailureRecord> failures;
    std::vector<PairedTrialRecord> all_trials;
    all_trials.reserve(scenarios.size() * num_seeds);

    const std::string trials_csv = prefix + "_trials.csv";
    const std::string summary_csv = prefix + "_summary.csv";
    const std::string failures_csv = prefix + "_failures.csv";

    // Open trials CSV and write header immediately
    std::ofstream trials_out(trials_csv);
    if (!trials_out.is_open()) {
        std::cerr << "CRITICAL ERROR: Failed to open " << trials_csv << " for writing!\n";
        return 1;
    }

    trials_out << "scenario_id,seed,pipeline,rover_count,duration_s,steps,packet_loss_rate,"
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

    std::cout << "[INFO] Commencing Phase 5.4 Benchmark (" << mode << " mode): " << scenarios.size()
              << " Scenarios x " << num_seeds << " Seeds = " << (scenarios.size() * num_seeds)
              << " Paired Trials (" << (scenarios.size() * num_seeds * 2) << " Pipeline Runs)\n";
    std::cout << "[INFO] Bounded CORD: max_steps = 40 (explicit bounded relaxation, not asymptotic convergence)\n";
    std::cout << "[INFO] Streaming trial results to: " << trials_csv << "\n\n";
    std::cout.flush();

    const auto total_bench_start = std::chrono::high_resolution_clock::now();

    std::map<std::string, std::vector<PairedMetricComparison>> scenario_summaries;
    std::map<std::string, double> scenario_runtimes_ms;

    for (const auto& sc : scenarios) {
        std::cout << ">>> Running Scenario [" << sc.id << "]: " << sc.description << " (" << sc.rover_count << " Rovers)..." << std::endl;
        const auto sc_start = std::chrono::high_resolution_clock::now();

        std::vector<double> base_ate, prop_ate;
        std::vector<double> base_rover_ate, prop_rover_ate;
        std::vector<double> base_closures, prop_closures;
        std::vector<double> base_wire_bytes, prop_wire_bytes;
        std::vector<double> base_app_completion, prop_app_completion;
        std::vector<double> base_final_dsu, prop_final_dsu;
        std::vector<double> base_runtime, prop_runtime;

        for (uint64_t seed : seeds) {
            auto paired = run_single_paired_trial(sc, seed, failures);
            all_trials.push_back(paired);

            // Stream trial results to disk immediately
            write_trial_row_to_stream(trials_out, paired.baseline);
            write_trial_row_to_stream(trials_out, paired.proposed);

            base_ate.push_back(paired.baseline.ate_aligned_swarm_rmse_m);
            prop_ate.push_back(paired.proposed.ate_aligned_swarm_rmse_m);

            base_rover_ate.push_back(paired.baseline.ate_aligned_per_rover_mean_m);
            prop_rover_ate.push_back(paired.proposed.ate_aligned_per_rover_mean_m);

            base_closures.push_back(static_cast<double>(paired.baseline.closures_verified));
            prop_closures.push_back(static_cast<double>(paired.proposed.closures_verified));

            base_wire_bytes.push_back(static_cast<double>(paired.baseline.total_wire_bytes));
            prop_wire_bytes.push_back(static_cast<double>(paired.proposed.total_wire_bytes));

            base_app_completion.push_back(paired.baseline.app_completion_ratio);
            prop_app_completion.push_back(paired.proposed.app_completion_ratio);

            base_final_dsu.push_back(static_cast<double>(paired.baseline.final_dsu_components));
            prop_final_dsu.push_back(static_cast<double>(paired.proposed.final_dsu_components));

            base_runtime.push_back(static_cast<double>(paired.baseline.runtime_us));
            prop_runtime.push_back(static_cast<double>(paired.proposed.runtime_us));
        }

        const auto sc_end = std::chrono::high_resolution_clock::now();
        const auto sc_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(sc_end - sc_start).count();
        scenario_runtimes_ms[sc.id] = static_cast<double>(sc_elapsed_ms);

        // Perform statistical comparisons
        std::vector<PairedMetricComparison> comps;
        comps.push_back(analyze_paired_metric("ATE_Aligned_Swarm_RMSE", "meters", true, base_ate, prop_ate));
        comps.push_back(analyze_paired_metric("ATE_Aligned_PerRover_Mean", "meters", true, base_rover_ate, prop_rover_ate));
        comps.push_back(analyze_paired_metric("Verified_Loop_Closures", "count", false, base_closures, prop_closures));
        comps.push_back(analyze_paired_metric("Total_Transmitted_Wire_Bytes", "bytes", true, base_wire_bytes, prop_wire_bytes));
        comps.push_back(analyze_paired_metric("Application_Completion_Ratio", "ratio", false, base_app_completion, prop_app_completion));
        comps.push_back(analyze_paired_metric("Final_DSU_Components", "count", true, base_final_dsu, prop_final_dsu));
        comps.push_back(analyze_paired_metric("Runtime", "microseconds", true, base_runtime, prop_runtime));

        scenario_summaries[sc.id] = comps;

        std::cout << "    [SCENARIO DONE] " << sc_elapsed_ms << " ms | Swarm ATE: "
                  << std::fixed << std::setprecision(4) << comps[0].baseline_summary.mean << " m (Base) vs "
                  << comps[0].proposed_summary.mean << " m (Astro-X) [Diff: "
                  << comps[0].paired_diff_mean << " m, CI95: [" << comps[0].ci95_low << ", " << comps[0].ci95_high << "]]" << std::endl;
        std::cout << "    Closures: "
                  << std::fixed << std::setprecision(1) << comps[2].baseline_summary.mean << " (Base) vs "
                  << comps[2].proposed_summary.mean << " (Astro-X) | DSU Components: "
                  << comps[5].baseline_summary.mean << " (Base) vs " << comps[5].proposed_summary.mean << " (Astro-X)\n" << std::endl;
    }

    const auto total_bench_end = std::chrono::high_resolution_clock::now();
    const auto total_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(total_bench_end - total_bench_start).count();
    const size_t total_pipeline_runs = all_trials.size() * 2;

    std::cout << "==============================================================================\n";
    std::cout << " [" << mode << " COMPLETED] " << all_trials.size() << " Paired Trials (" << total_pipeline_runs
              << " Pipeline Runs) Executed in " << total_elapsed_ms << " ms ("
              << (total_pipeline_runs > 0 ? (total_elapsed_ms / static_cast<double>(total_pipeline_runs)) : 0.0)
              << " ms per pipeline run)\n";
    std::cout << "==============================================================================\n\n";

    std::cout << "Writing summary to: " << summary_csv << " ...\n";
    write_summary_csv(summary_csv, scenario_summaries, num_seeds);

    std::cout << "Writing failures to: " << failures_csv << " ...\n";
    write_failures_csv(failures_csv, failures);

    // 8. Invariant Validation Across All Trials
    size_t invalid_data_bytes = 0;
    size_t invalid_total_bytes = 0;
    size_t false_acceptances_recorded = 0;

    for (const auto& t : all_trials) {
        if (!t.proposed.data_byte_conservation_valid) invalid_data_bytes++;
        if (!t.baseline.data_byte_conservation_valid) invalid_data_bytes++;
        if (!t.proposed.total_byte_conservation_valid) invalid_total_bytes++;
        if (!t.baseline.total_byte_conservation_valid) invalid_total_bytes++;
        if (t.proposed.closures_false_accept > 0) false_acceptances_recorded += t.proposed.closures_false_accept;
        if (t.baseline.closures_false_accept > 0) false_acceptances_recorded += t.baseline.closures_false_accept;
    }

    std::cout << "\n--- Invariant Verification Checks ---\n";
    std::cout << "  Total Paired Trials: " << all_trials.size() << "\n";
    std::cout << "  Recorded Simulation Failures: " << failures.size() << "\n";
    std::cout << "  Data Wire Byte Invariant Violations: " << invalid_data_bytes << "\n";
    std::cout << "  Total Wire Byte Invariant Violations: " << invalid_total_bytes << "\n";
    std::cout << "  Total False Accepted Closures: " << false_acceptances_recorded << "\n";

    if (mode != "full") {
        std::cout << "\n--- Per-Scenario Runtime Breakdown & Projections ---\n";
        double projected_30seed_total_ms = 0.0;
        for (const auto& [sc_id, ms] : scenario_runtimes_ms) {
            const double projected_ms = (ms / static_cast<double>(num_seeds)) * 30.0;
            projected_30seed_total_ms += projected_ms;
            std::cout << "  " << std::left << std::setw(26) << sc_id
                      << ": Observed (" << num_seeds << " seeds) = " << std::right << std::setw(6) << static_cast<int>(ms)
                      << " ms | Projected (30 seeds) = " << std::setw(8) << std::fixed << std::setprecision(1)
                      << (projected_ms / 1000.0) << " s\n";
        }
        std::cout << "  Projected 300-Trial Full Run Time: "
                  << std::fixed << std::setprecision(1) << (projected_30seed_total_ms / 1000.0) << " seconds ("
                  << (projected_30seed_total_ms / 60000.0) << " minutes)\n";
    }

    std::cout << "==============================================================================\n";

    if (invalid_data_bytes > 0 || invalid_total_bytes > 0) {
        std::cerr << "  CRITICAL ERROR: Wire-byte conservation invariant violated!\n";
        return 1;
    }

    std::cout << "  All invariants held strictly across all " << total_pipeline_runs << " pipeline executions.\n";
    std::cout << "==============================================================================\n";

    return 0;
}
