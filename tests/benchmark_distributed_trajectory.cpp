#include "astromesh/core/iblt.hpp"
#include "astromesh/core/landmark_key.hpp"
#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/keyframe_pruner.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/net/distributed_reconciliation.hpp"
#include "astromesh/net/rover_node.hpp"
#include "astromesh/optimization/cord_integrator.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <thread>
#include <vector>

using namespace astromesh::net;
using namespace astromesh::core;
using namespace astromesh::graph;
using namespace astromesh::geometry;
using namespace astromesh::optimization;
using namespace astromesh::verification;

struct TrajectoryBenchmarkTrialResult {
    uint32_t trial_id{0};
    uint64_t seed{42};
    size_t rover_count{2};
    size_t keyframes_per_rover{5};
    double packet_loss_rate{0.0};
    double delay_ms{0.0};
    double elapsed_compute_ms{0.0};

    // Communication metrics
    uint64_t sync_packets_sent{0};
    uint64_t sync_packets_received{0};
    uint64_t decodes_succeeded{0};
    uint64_t decodes_failed{0};
    uint64_t genuine_proposals{0};
    uint64_t imposter_proposals{0};
    uint64_t proposals_sent{0};
    uint64_t proposals_received{0};
    uint64_t accepted_proposals{0};
    uint64_t rejected_proposals{0};
    uint64_t loop_closures_verified{0};
    uint64_t loop_closures_rejected{0};
    uint64_t dsu_unions_performed{0};
    size_t final_dsu_components{0};
    bool is_swarm_unified{false};

    // Scientific Precision & Recall metrics
    uint64_t true_positives{0};
    uint64_t false_positives{0};
    uint64_t true_negatives{0};
    uint64_t false_negatives{0};
    double precision{0.0};
    double recall{0.0};
    double specificity{0.0};
    double fpr{0.0};
    double fnr{0.0};
    double f1_score{0.0};

    // Wire byte metrics
    uint64_t payload_bytes_sent{0};
    uint64_t header_bytes_sent{0};
    uint64_t framing_bytes_sent{0};
    uint64_t data_wire_bytes{0};
    uint64_t ack_wire_bytes{0};
    uint64_t total_wire_bytes{0};
    uint64_t retransmissions{0};
    uint64_t duplicate_suppressed{0};

    // SLAM & Distributed CORD metrics
    double initial_ate_rmse{0.0};
    double final_cord_ate_rmse{0.0};
    double ate_absolute_improvement{0.0};
    double ate_reduction_pct{0.0};
    uint64_t cord_optimizations_run{0};
    uint64_t keyframes_pruned{0};
};

static TrajectoryBenchmarkTrialResult run_trajectory_trial(
    uint32_t trial_id,
    size_t rover_count,
    size_t keyframes_per_rover,
    double loss_rate,
    double delay_ms,
    uint64_t seed,
    uint16_t base_port) {

    TrajectoryBenchmarkTrialResult res;
    res.trial_id = trial_id;
    res.seed = seed;
    res.rover_count = rover_count;
    res.keyframes_per_rover = keyframes_per_rover;
    res.packet_loss_rate = loss_rate;
    res.delay_ms = delay_ms;

    std::mt19937_64 rng(seed);
    std::normal_distribution<double> dist_odom_noise(0.0, 0.05); // 5cm odometry noise
    std::normal_distribution<double> dist_lm_noise(0.0, 0.02);   // 2cm landmark noise

    const auto t_start = std::chrono::steady_clock::now();

    // 1. Initialize rover nodes
    std::vector<std::unique_ptr<RoverNode>> nodes;
    std::vector<std::unique_ptr<DistributedReconciliationAgent>> agents;

    for (size_t i = 1; i <= rover_count; ++i) {
        RoverNodeConfig cfg;
        cfg.rover_id = static_cast<uint16_t>(i);
        cfg.listen_port = base_port + static_cast<uint16_t>(i);
        cfg.listen_ip = "127.0.0.1";
        cfg.heartbeat_interval_ms = 40;
        cfg.dtn_config.initial_retry_delay_ns = 35'000'000ULL;
        cfg.impairment_config.packet_loss_rate = loss_rate;
        cfg.impairment_config.one_way_delay_ns = static_cast<uint64_t>(delay_ms * 1'000'000.0);
        cfg.impairment_config.seed = seed + i * 100ULL;

        for (size_t j = 1; j <= rover_count; ++j) {
            if (i != j) {
                cfg.seed_peers.emplace_back("127.0.0.1", base_port + static_cast<uint16_t>(j));
            }
        }

        auto node = std::make_unique<RoverNode>(cfg);
        if (!node->start()) {
            std::cerr << "[Trial " << trial_id << "] Failed to bind Rover " << i
                      << " on port " << cfg.listen_port << "\n";
            return res;
        }

        auto agent = std::make_unique<DistributedReconciliationAgent>(
            static_cast<uint16_t>(i), *node, 32, 3);
        agent->setup_callbacks();
        agent->set_auto_optimize_on_boundary(true);

        // Rover base position: spaced by 10.0m along X axis
        const double base_x = static_cast<double>(i - 1) * 10.0;
        double current_x = base_x;
        double current_y = 0.0;

        // Generate multi-keyframe trajectory with accumulated Brownian odometry drift
        for (uint32_t k = 0; k < keyframes_per_rover; ++k) {
            if (k > 0) {
                // Nominal 2.0m forward step along Y with odometry perturbation
                current_x += dist_odom_noise(rng);
                current_y += 2.0 + dist_odom_noise(rng);
            }

            const SE2 kf_pose(current_x, current_y, 0.0);
            std::vector<LandmarkObservationRecord> obs;

            // If at the frontier keyframe, observe shared landmarks with neighbor
            if (k == keyframes_per_rover - 1) {
                // Shared landmarks with right neighbor
                if (i < rover_count) {
                    const uint32_t base_shared = static_cast<uint32_t>(i * 100);
                    for (uint32_t l = 1; l <= 4; ++l) {
                        obs.push_back({
                            LandmarkKey(1, static_cast<uint16_t>(i), base_shared + l, 0x1000ULL + base_shared + l),
                            2.0 * l + dist_lm_noise(rng), 1.0 * l + dist_lm_noise(rng), k, true
                        });
                    }
                    // Adversarial outlier imposter landmark
                    obs.push_back({
                        LandmarkKey(1, static_cast<uint16_t>(i), base_shared + 99, 0x9999ULL),
                        1.0, 1.0, k, false
                    });
                }

                // Shared landmarks with left neighbor
                if (i > 1) {
                    const uint32_t base_shared = static_cast<uint32_t>((i - 1) * 100);
                    for (uint32_t l = 1; l <= 4; ++l) {
                        // In Rover i frame, relative x is offset by -10.0m
                        obs.push_back({
                            LandmarkKey(1, static_cast<uint16_t>(i - 1), base_shared + l, 0x1000ULL + base_shared + l),
                            2.0 * l - 10.0 + dist_lm_noise(rng), 1.0 * l + dist_lm_noise(rng), k, true
                        });
                    }
                    // Adversarial outlier imposter: huge spatial contradiction (-30, 25)
                    obs.push_back({
                        LandmarkKey(1, static_cast<uint16_t>(i - 1), base_shared + 99, 0x9999ULL),
                        -30.0, 25.0, k, false
                    });
                }
            }

            // Local unique landmark
            const uint32_t u_id = static_cast<uint32_t>(1000 * i + k);
            obs.push_back({
                LandmarkKey(1, static_cast<uint16_t>(i), u_id, 0xE000ULL + u_id),
                0.5 + dist_lm_noise(rng), 1.5 + dist_lm_noise(rng), k, true
            });

            agent->append_trajectory_keyframe(k, kf_pose, obs, k * 1'000'000'000ULL);
        }

        nodes.push_back(std::move(node));
        agents.push_back(std::move(agent));
    }

    // Ground truth poses map for authentic node-based ATE evaluation
    std::unordered_map<NodeId, SE2> ground_truth_poses;
    for (size_t i = 1; i <= rover_count; ++i) {
        for (uint32_t k = 0; k < keyframes_per_rover; ++k) {
            ground_truth_poses[NodeId(static_cast<uint16_t>(i), k)] =
                SE2(static_cast<double>(i - 1) * 10.0, 2.0 * static_cast<double>(k), 0.0);
        }
    }

    // 2. Peer discovery phase
    for (int t = 0; t < 5; ++t) {
        for (auto& n : nodes) n->poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // 3. Initiate IBLT sync from each rover to its adjacent neighbor (generates genuine proposals)
    for (size_t i = 1; i < rover_count; ++i) {
        agents[i - 1]->initiate_sync_with_peer(static_cast<uint16_t>(i + 1));
    }

    // Deterministic, reproducible imposter proposal injection:
    // Rover i sends an adversarial proposal with scrambled non-rigid geometry to rover i + 1
    uint64_t trial_imposters_injected = 0;
    for (size_t i = 1; i < rover_count; ++i) {
        CandidateProposalPayload imposter;
        imposter.sender_rover_id = static_cast<uint16_t>(i);
        imposter.target_rover_id = static_cast<uint16_t>(i + 1);

        const uint32_t base_shared = static_cast<uint32_t>(i * 100);
        // Matching keys with scrambled non-rigid coordinates: forces RANSAC geometric rejection
        imposter.observations.push_back({
            LandmarkKey(1, static_cast<uint16_t>(i), base_shared + 1, 0x1000ULL + base_shared + 1),
            -50.0, 75.0, 0, false
        });
        imposter.observations.push_back({
            LandmarkKey(1, static_cast<uint16_t>(i), base_shared + 2, 0x1000ULL + base_shared + 2),
            80.0, -30.0, 0, false
        });
        imposter.observations.push_back({
            LandmarkKey(1, static_cast<uint16_t>(i), base_shared + 3, 0x1000ULL + base_shared + 3),
            12.0, 90.0, 0, false
        });
        imposter.observations.push_back({
            LandmarkKey(1, static_cast<uint16_t>(i), base_shared + 4, 0x1000ULL + base_shared + 4),
            -40.0, -15.0, 0, false
        });

        auto bytes = imposter.serialize();
        nodes[i - 1]->enqueue_dtn_message(static_cast<uint16_t>(i + 1), MessageType::CandidateProposal, bytes);
        trial_imposters_injected++;
    }

    // 4. Execution loop (expanded to 80 ticks for DTN retry completion under harsh impairment)
    for (int step = 0; step < 80; ++step) {
        for (auto& n : nodes) {
            n->poll_events(0);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 5. Evaluate Initial ATE RMSE on rover local pose graph nodes against ground truth
    double init_sq_err = 0.0;
    size_t init_nodes_evaluated = 0;

    for (size_t i = 0; i < rover_count; ++i) {
        const auto& g = agents[i]->get_pose_graph();
        for (const auto& [nid, node] : g.nodes()) {
            auto it_gt = ground_truth_poses.find(nid);
            if (it_gt != ground_truth_poses.end()) {
                const double dx = node.pose.x - it_gt->second.x();
                const double dy = node.pose.y - it_gt->second.y();
                init_sq_err += (dx * dx + dy * dy);
                init_nodes_evaluated++;
            }
        }
    }

    res.initial_ate_rmse = init_nodes_evaluated > 0 ? std::sqrt(init_sq_err / static_cast<double>(init_nodes_evaluated)) : 0.0;

    // 6. Execute Distributed CORD Pose Graph Optimization on each rover
    CordIntegratorOptions cord_opts;
    cord_opts.timestep = 0.01;
    cord_opts.max_steps = 150;
    cord_opts.gradient_tolerance = 1e-4;

    for (size_t i = 0; i < rover_count; ++i) {
        agents[i]->optimize_local_graph_cord(cord_opts);
    }

    // 7. Evaluate Final CORD ATE RMSE on rover local pose graph nodes against ground truth
    double final_sq_err = 0.0;
    size_t final_nodes_evaluated = 0;

    for (size_t i = 0; i < rover_count; ++i) {
        const auto& g = agents[i]->get_pose_graph();
        for (const auto& [nid, node] : g.nodes()) {
            auto it_gt = ground_truth_poses.find(nid);
            if (it_gt != ground_truth_poses.end()) {
                const double dx = node.pose.x - it_gt->second.x();
                const double dy = node.pose.y - it_gt->second.y();
                final_sq_err += (dx * dx + dy * dy);
                final_nodes_evaluated++;
            }
        }
    }

    res.final_cord_ate_rmse = final_nodes_evaluated > 0 ? std::sqrt(final_sq_err / static_cast<double>(final_nodes_evaluated)) : 0.0;
    res.ate_absolute_improvement = res.initial_ate_rmse - res.final_cord_ate_rmse;
    res.ate_reduction_pct = (res.initial_ate_rmse > 1e-6)
        ? (res.ate_absolute_improvement / res.initial_ate_rmse * 100.0)
        : 0.0;

    // 8. Aggregate telemetry, wire bytes, and precision/recall
    uint64_t total_tp = 0;
    uint64_t total_fp = 0;
    uint64_t total_tn = 0;
    uint64_t total_fn = 0;

    for (size_t i = 0; i < rover_count; ++i) {
        const auto s = agents[i]->get_stats();
        const auto w = nodes[i]->get_wire_stats();

        res.sync_packets_sent += s.sync_sent;
        res.sync_packets_received += s.sync_received;
        res.decodes_succeeded += s.sync_decodes_succeeded;
        res.decodes_failed += s.sync_decodes_failed;
        res.proposals_sent += s.proposals_sent;
        res.proposals_received += s.proposals_received;
        res.loop_closures_verified += s.loop_closures_verified;
        res.loop_closures_rejected += s.loop_closures_rejected;
        res.dsu_unions_performed += s.dsu_unions_performed;
        res.cord_optimizations_run += s.cord_optimizations_run;
        res.keyframes_pruned += s.keyframes_pruned;

        total_tp += s.true_positives;
        total_fp += s.false_positives;
        total_tn += s.true_negatives;
        total_fn += s.false_negatives;

        res.payload_bytes_sent += w.payload_bytes_sent;
        res.header_bytes_sent += w.header_bytes_sent;
        res.framing_bytes_sent += w.framing_bytes_sent;
        res.data_wire_bytes += w.data_wire_bytes_sent;
        res.ack_wire_bytes += w.ack_wire_bytes_sent;
        res.total_wire_bytes += w.total_wire_bytes_sent;
        res.retransmissions += w.dtn_retransmissions;
        res.duplicate_suppressed += w.duplicate_deliveries_suppressed;
    }

    res.genuine_proposals = res.proposals_sent;
    res.imposter_proposals = trial_imposters_injected;
    res.proposals_sent += trial_imposters_injected;
    res.accepted_proposals = total_tp + total_fp;
    res.rejected_proposals = total_tn + total_fn;

    res.true_positives = total_tp;
    res.false_positives = total_fp;
    res.true_negatives = total_tn;
    res.false_negatives = total_fn;

    const uint64_t pred_pos = total_tp + total_fp;
    res.precision = (pred_pos > 0) ? (static_cast<double>(total_tp) / static_cast<double>(pred_pos)) : 0.0;

    const uint64_t act_pos = total_tp + total_fn;
    res.recall = (act_pos > 0) ? (static_cast<double>(total_tp) / static_cast<double>(act_pos)) : 0.0;

    const uint64_t act_neg = total_tn + total_fp;
    res.specificity = (act_neg > 0) ? (static_cast<double>(total_tn) / static_cast<double>(act_neg)) : 0.0;

    res.fpr = (act_neg > 0) ? (static_cast<double>(total_fp) / static_cast<double>(act_neg)) : 0.0;
    res.fnr = (act_pos > 0) ? (static_cast<double>(total_fn) / static_cast<double>(act_pos)) : 0.0;

    if (res.precision + res.recall > 1e-9) {
        res.f1_score = 2.0 * (res.precision * res.recall) / (res.precision + res.recall);
    } else {
        res.f1_score = 0.0;
    }

    std::vector<const astromesh::graph::PoseGraph*> graph_ptrs;
    for (size_t i = 0; i < rover_count; ++i) {
        graph_ptrs.push_back(&agents[i]->get_pose_graph());
    }
    auto swarm_eval = GlobalSwarmEvaluator::evaluate_graphs(graph_ptrs);
    res.final_dsu_components = swarm_eval.global_connected_components;
    res.is_swarm_unified = (swarm_eval.global_connected_components == 1);

    const auto t_end = std::chrono::steady_clock::now();
    res.elapsed_compute_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    // Stop nodes cleanly
    for (auto& n : nodes) n->stop();

    return res;
}

int main() {
    std::cout << "=================================================================\n";
    std::cout << "  ASTRAMESH-X M7.10 DISTRIBUTED MULTI-KEYFRAME TRAJECTORY BENCHMARK\n";
    std::cout << "=================================================================\n";

    const std::string csv_filename = "benchmark_distributed_trajectory_validated.csv";
    std::ofstream csv(csv_filename);
    if (!csv.is_open()) {
        std::cerr << "Failed to open " << csv_filename << " for writing\n";
        return 1;
    }

    // CSV Header
    csv << "trial_id,seed,rover_count,keyframes_per_rover,packet_loss_rate,delay_ms,"
        << "sync_packets_sent,sync_packets_received,decodes_succeeded,decodes_failed,"
        << "genuine_proposals,imposter_proposals,proposals_sent,proposals_received,"
        << "accepted_proposals,rejected_proposals,"
        << "true_positives,false_positives,true_negatives,false_negatives,"
        << "precision,recall,specificity,fpr,fnr,f1_score,"
        << "dsu_unions_performed,final_dsu_components,is_swarm_unified,"
        << "payload_bytes_sent,header_bytes_sent,framing_bytes_sent,data_wire_bytes,ack_wire_bytes,total_wire_bytes,"
        << "retransmissions,duplicate_suppressed,"
        << "initial_ate_rmse,final_cord_ate_rmse,ate_absolute_improvement,ate_reduction_pct,"
        << "cord_optimizations_run,keyframes_pruned,elapsed_compute_ms\n";

    // Factorial sweep: 3 swarm sizes x 3 keyframe depths x 3 network impairments x 3 seeds = 81 trials
    const std::vector<size_t> rovers = {2, 3, 4};
    const std::vector<size_t> keyframe_counts = {3, 5, 8};
    struct Impairment { double loss; double delay; };
    const std::vector<Impairment> impairments = {
        {0.00, 0.0},   // Nominal
        {0.20, 20.0},  // Moderate
        {0.40, 50.0}   // Harsh
    };
    const std::vector<uint64_t> seeds = {42, 101, 2026};

    uint32_t trial_id = 1;
    const uint16_t base_port = 31000;

    std::vector<TrajectoryBenchmarkTrialResult> results;

    for (size_t r : rovers) {
        for (size_t k : keyframe_counts) {
            for (const auto& imp : impairments) {
                for (uint64_t s : seeds) {
                    std::cout << "[Trial " << std::setw(2) << trial_id << "/81] "
                              << "Rovers=" << r << ", Keyframes=" << k
                              << ", Loss=" << std::fixed << std::setprecision(0) << (imp.loss * 100.0) << "%"
                              << ", Delay=" << std::setprecision(0) << imp.delay << "ms"
                              << ", Seed=" << s << " ... " << std::flush;

                    auto res = run_trajectory_trial(
                        trial_id, r, k, imp.loss, imp.delay, s,
                        base_port + static_cast<uint16_t>((trial_id * 10) % 5000));

                    std::cout << "ATE: " << std::fixed << std::setprecision(4)
                              << res.initial_ate_rmse << "m -> " << res.final_cord_ate_rmse << "m"
                              << " (-" << std::setprecision(2) << res.ate_reduction_pct << "%)"
                              << " (Prec=" << std::setprecision(1) << (res.precision * 100.0) << "%"
                              << ", Spec=" << std::setprecision(1) << (res.specificity * 100.0) << "%"
                              << ", DSU=" << res.final_dsu_components << ") in "
                              << std::setprecision(0) << res.elapsed_compute_ms << "ms\n";

                    // Invariant checks
                    assert(res.total_wire_bytes == res.data_wire_bytes + res.ack_wire_bytes);
                    assert(res.false_positives == 0); // Safety invariant: Zero false acceptances

                    // Write to CSV
                    csv << res.trial_id << ","
                        << res.seed << ","
                        << res.rover_count << ","
                        << res.keyframes_per_rover << ","
                        << std::fixed << std::setprecision(3) << res.packet_loss_rate << ","
                        << std::setprecision(1) << res.delay_ms << ","
                        << res.sync_packets_sent << ","
                        << res.sync_packets_received << ","
                        << res.decodes_succeeded << ","
                        << res.decodes_failed << ","
                        << res.genuine_proposals << ","
                        << res.imposter_proposals << ","
                        << res.proposals_sent << ","
                        << res.proposals_received << ","
                        << res.accepted_proposals << ","
                        << res.rejected_proposals << ","
                        << res.true_positives << ","
                        << res.false_positives << ","
                        << res.true_negatives << ","
                        << res.false_negatives << ","
                        << std::setprecision(4) << res.precision << ","
                        << res.recall << ","
                        << res.specificity << ","
                        << res.fpr << ","
                        << res.fnr << ","
                        << res.f1_score << ","
                        << res.dsu_unions_performed << ","
                        << res.final_dsu_components << ","
                        << (res.is_swarm_unified ? 1 : 0) << ","
                        << res.payload_bytes_sent << ","
                        << res.header_bytes_sent << ","
                        << res.framing_bytes_sent << ","
                        << res.data_wire_bytes << ","
                        << res.ack_wire_bytes << ","
                        << res.total_wire_bytes << ","
                        << res.retransmissions << ","
                        << res.duplicate_suppressed << ","
                        << std::setprecision(6) << res.initial_ate_rmse << ","
                        << res.final_cord_ate_rmse << ","
                        << res.ate_absolute_improvement << ","
                        << std::setprecision(2) << res.ate_reduction_pct << ","
                        << res.cord_optimizations_run << ","
                        << res.keyframes_pruned << ","
                        << std::setprecision(2) << res.elapsed_compute_ms << "\n";
                    csv.flush();

                    results.push_back(res);
                    trial_id++;
                }
            }
        }
    }

    csv.close();
    std::cout << "\n=================================================================\n";
    std::cout << "  BENCHMARK COMPLETE: " << results.size() << " TRIALS EXECUTED & EXPORTED\n";
    std::cout << "  Target file: " << csv_filename << "\n";
    std::cout << "=================================================================\n";

    return 0;
}

