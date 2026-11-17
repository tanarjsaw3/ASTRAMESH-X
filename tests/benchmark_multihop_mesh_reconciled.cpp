#include "astromesh/core/iblt.hpp"
#include "astromesh/core/landmark_key.hpp"
#include "astromesh/geometry/se2.hpp"
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
#include <string>
#include <thread>
#include <vector>

using namespace astromesh::net;
using namespace astromesh::core;
using namespace astromesh::graph;
using namespace astromesh::geometry;
using namespace astromesh::optimization;
using namespace astromesh::verification;

struct MultihopBenchmarkRecord {
    uint32_t trial_id{0};
    std::string topology;
    uint64_t seed{42};
    size_t num_rovers{2};
    double packet_loss_rate{0.0};
    double delay_ms{0.0};
    double elapsed_ms{0.0};

    // Protocol & Sync
    uint64_t sync_packets_sent{0};
    uint64_t sync_packets_received{0};
    uint64_t proposals_sent{0};
    uint64_t proposals_received{0};
    uint64_t genuine_proposals{0};
    uint64_t imposter_proposals{0};
    uint64_t loop_closures_verified{0};
    uint64_t loop_closures_rejected{0};

    // Classification matrix
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

    // Multi-Hop Routing Metrics
    uint64_t multihop_forward_count{0};
    uint64_t routing_loops_suppressed{0};
    uint64_t ttl_expired_dropped{0};
    uint64_t custody_acks_sent{0};
    uint64_t e2e_acks_sent{0};
    uint64_t transit_buffer_evictions{0};

    // Swarm Connectivity
    size_t global_connected_components{0};
    bool is_fully_unified{false};

    // ATE RMSE
    double initial_mean_ate{0.0};
    double optimized_mean_ate{0.0};
    double ate_improvement_pct{0.0};
};

static MultihopBenchmarkRecord run_multihop_trial(
    uint32_t trial_id,
    const std::string& topology,
    size_t num_rovers,
    double loss_rate,
    double delay_ms,
    uint64_t seed,
    uint16_t base_port) {

    MultihopBenchmarkRecord rec;
    rec.trial_id = trial_id;
    rec.topology = topology;
    rec.num_rovers = num_rovers;
    rec.packet_loss_rate = loss_rate;
    rec.delay_ms = delay_ms;
    rec.seed = seed;

    const auto t_start = std::chrono::steady_clock::now();
    std::mt19937_64 rng(seed);
    std::normal_distribution<double> dist_noise(0.0, 0.03);

    // 1. Initialize Rover Nodes
    std::vector<std::unique_ptr<RoverNode>> nodes;
    std::vector<std::unique_ptr<DistributedReconciliationAgent>> agents;

    for (size_t i = 1; i <= num_rovers; ++i) {
        RoverNodeConfig cfg;
        cfg.rover_id = static_cast<uint16_t>(i);
        cfg.listen_port = base_port + static_cast<uint16_t>(i);
        cfg.listen_ip = "127.0.0.1";
        cfg.heartbeat_interval_ms = 30;
        cfg.routing_advertisement_interval_ms = 30;
        cfg.enable_multihop = true;
        cfg.default_max_hops = 6;
        cfg.impairment_config.packet_loss_rate = loss_rate;
        cfg.impairment_config.one_way_delay_ns = static_cast<uint64_t>(delay_ms * 1'000'000.0);
        cfg.dtn_config.initial_retry_delay_ns = 35'000'000ULL;

        // Seed with all potential local peers so endpoints are registered
        for (size_t j = 1; j <= num_rovers; ++j) {
            if (i != j) {
                cfg.seed_peers.emplace_back("127.0.0.1", base_port + static_cast<uint16_t>(j));
            }
        }

        nodes.push_back(std::make_unique<RoverNode>(cfg));
    }

    // 2. Configure Link States based on Topology
    auto set_bidirectional_link = [&](size_t r_a, size_t r_b, bool state) {
        nodes[r_a - 1]->get_channel_impairment().set_link_state(
            static_cast<uint16_t>(r_a), static_cast<uint16_t>(r_b), state);
        nodes[r_b - 1]->get_channel_impairment().set_link_state(
            static_cast<uint16_t>(r_b), static_cast<uint16_t>(r_a), state);
    };

    // Default: isolate all links
    for (size_t i = 1; i <= num_rovers; ++i) {
        for (size_t j = 1; j <= num_rovers; ++j) {
            if (i != j) {
                nodes[i - 1]->get_channel_impairment().set_link_state(
                    static_cast<uint16_t>(i), static_cast<uint16_t>(j), false);
            }
        }
    }

    if (topology == "2_Rover_Direct") {
        set_bidirectional_link(1, 2, true);
    } else if (topology == "3_Rover_Linear") {
        set_bidirectional_link(1, 2, true);
        set_bidirectional_link(2, 3, true);
    } else if (topology == "4_Rover_Linear") {
        set_bidirectional_link(1, 2, true);
        set_bidirectional_link(2, 3, true);
        set_bidirectional_link(3, 4, true);
    } else if (topology == "4_Rover_Diamond" || topology == "4_Rover_Broken_Failover") {
        set_bidirectional_link(1, 2, true);
        set_bidirectional_link(1, 3, true);
        set_bidirectional_link(2, 4, true);
        set_bidirectional_link(3, 4, true);
    }

    // Start all nodes
    for (auto& n : nodes) {
        bool ok = n->start();
        assert(ok);
    }

    // Create Agents
    for (size_t i = 0; i < num_rovers; ++i) {
        agents.push_back(std::make_unique<DistributedReconciliationAgent>(
            static_cast<uint16_t>(i + 1), *nodes[i], 32, 3));
        agents[i]->setup_callbacks();
    }

    // 3. Ground truth poses and landmark observations
    std::unordered_map<NodeId, SE2> gt_poses;
    const double spacing = 10.0;
    for (size_t i = 1; i <= num_rovers; ++i) {
        const double true_x = static_cast<double>(i - 1) * spacing;
        const double true_y = 0.0;
        const double true_th = 0.0;
        gt_poses[NodeId(static_cast<uint16_t>(i), 0)] = SE2(true_x, true_y, true_th);

        // Perturbed initial odometry pose
        const double noisy_x = true_x + dist_noise(rng);
        const double noisy_y = true_y + dist_noise(rng);
        const double noisy_th = true_th + dist_noise(rng) * 0.1;

        // Reconciled: each adjacent pair shares 3 landmarks (lm_base increment 2 instead of 3)
        // so {1,2,3,4,5} and {3,4,5,6,7} share {3,4,5} (>= 3 correspondences for Kabsch RANSAC)
        std::vector<LandmarkObservationRecord> obs;
        const uint32_t lm_base = static_cast<uint32_t>((i - 1) * 2);
        for (uint32_t k = 1; k <= 5; ++k) {
            uint32_t lm_id = lm_base + k;
            double global_lm_x = static_cast<double>(lm_id) * 4.0;
            double global_lm_y = ((lm_id % 2 == 0) ? 5.0 : -5.0);

            // Observation in local rover frame
            double lx = global_lm_x - true_x + dist_noise(rng);
            double ly = global_lm_y - true_y + dist_noise(rng);

            obs.push_back({
                LandmarkKey(1, 1, lm_id, 0x10000ULL + lm_id),
                lx, ly, 0, true
            });
        }

        agents[i - 1]->add_local_keyframe(0, SE2(noisy_x, noisy_y, noisy_th), obs);
    }

    // Initial route discovery phase
    for (int step = 0; step < 35; ++step) {
        for (auto& n : nodes) {
            n->poll_events(0);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // 4. Initiate Multi-Hop Reconciliation Traffic
    // Rover 1 initiates sync to destination rover (num_rovers) across multi-hop relay
    agents[0]->initiate_sync_with_peer(static_cast<uint16_t>(num_rovers));

    // Also initiate genuine sync between neighbors
    for (size_t i = 1; i < num_rovers; ++i) {
        agents[i - 1]->initiate_sync_with_peer(static_cast<uint16_t>(i + 1));
    }

    // Deterministic imposter injection across multi-hop path
    // Rover 1 sends imposter proposal directly addressed to num_rovers
    CandidateProposalPayload imposter;
    imposter.sender_rover_id = 1;
    imposter.target_rover_id = static_cast<uint16_t>(num_rovers);
    imposter.observations.push_back({ LandmarkKey(1, 1, 9991, 0x9991ULL), -60.0,  80.0, 0, false });
    imposter.observations.push_back({ LandmarkKey(1, 1, 9992, 0x9992ULL),  70.0, -40.0, 0, false });
    imposter.observations.push_back({ LandmarkKey(1, 1, 9993, 0x9993ULL),  15.0,  95.0, 0, false });
    imposter.observations.push_back({ LandmarkKey(1, 1, 9994, 0x9994ULL), -35.0, -20.0, 0, false });
    auto imposter_bytes = imposter.serialize();
    nodes[0]->enqueue_dtn_message(static_cast<uint16_t>(num_rovers), MessageType::CandidateProposal, imposter_bytes);
    uint64_t imposters_injected = 1;

    // Execution Loop Phase 1
    for (int step = 0; step < 40; ++step) {
        for (auto& n : nodes) {
            n->poll_events(0);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // If Broken Failover topology: sever R2-R4 link halfway, force dynamic failover through R3!
    if (topology == "4_Rover_Broken_Failover") {
        set_bidirectional_link(2, 4, false);
        nodes[1]->get_routing_table().invalidate_neighbor_routes(4, 0);
        nodes[3]->get_routing_table().invalidate_neighbor_routes(2, 0);

        // Rover 1 sends another message to Rover 4 that must reroute through R3
        nodes[0]->send_message_to(4, MessageType::DataPayload, {'F', 'A', 'I', 'L', 'O', 'V', 'E', 'R'});
    }

    // Execution Loop Phase 2
    for (int step = 0; step < 40; ++step) {
        for (auto& n : nodes) {
            n->poll_events(0);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // 5. Evaluate Initial ATE
    double init_sq_err = 0.0;
    size_t init_count = 0;
    for (size_t i = 0; i < num_rovers; ++i) {
        const auto& g = agents[i]->get_pose_graph();
        for (const auto& [nid, node] : g.nodes()) {
            auto it = gt_poses.find(nid);
            if (it != gt_poses.end()) {
                double dx = node.pose.x - it->second.x();
                double dy = node.pose.y - it->second.y();
                init_sq_err += (dx * dx + dy * dy);
                init_count++;
            }
        }
    }
    rec.initial_mean_ate = (init_count > 0) ? std::sqrt(init_sq_err / static_cast<double>(init_count)) : 0.0;

    // Run CORD optimization on each agent
    for (auto& agent : agents) {
        CordIntegratorOptions opts;
        opts.max_steps = 60;
        opts.timestep = 0.05;
        opts.fixed_anchor = NodeId(1, 0);
        agent->optimize_local_graph_cord(opts);
    }

    // Evaluate Final CORD ATE
    double final_sq_err = 0.0;
    size_t final_count = 0;
    for (size_t i = 0; i < num_rovers; ++i) {
        const auto& g = agents[i]->get_pose_graph();
        for (const auto& [nid, node] : g.nodes()) {
            auto it = gt_poses.find(nid);
            if (it != gt_poses.end()) {
                double dx = node.pose.x - it->second.x();
                double dy = node.pose.y - it->second.y();
                final_sq_err += (dx * dx + dy * dy);
                final_count++;
            }
        }
    }
    rec.optimized_mean_ate = (final_count > 0) ? std::sqrt(final_sq_err / static_cast<double>(final_count)) : 0.0;
    double ate_abs_gain = rec.initial_mean_ate - rec.optimized_mean_ate;
    rec.ate_improvement_pct = (rec.initial_mean_ate > 1e-6) ? (ate_abs_gain / rec.initial_mean_ate * 100.0) : 0.0;

    // 6. Aggregate Swarm Connectivity & Router Metrics
    auto swarm_metrics = GlobalSwarmEvaluator::evaluate(agents);
    rec.global_connected_components = swarm_metrics.global_connected_components;
    rec.is_fully_unified = swarm_metrics.is_fully_unified;

    uint64_t total_tp = 0;
    uint64_t total_fp = 0;
    uint64_t total_tn = 0;
    uint64_t total_fn = 0;

    for (size_t i = 0; i < num_rovers; ++i) {
        const auto s = agents[i]->get_stats();
        const auto w = nodes[i]->get_wire_stats();
        const auto r = nodes[i]->get_multihop_router().get_stats();

        rec.sync_packets_sent += s.sync_sent;
        rec.sync_packets_received += s.sync_received;
        rec.proposals_sent += s.proposals_sent;
        rec.proposals_received += s.proposals_received;
        rec.loop_closures_verified += s.loop_closures_verified;
        rec.loop_closures_rejected += s.loop_closures_rejected;

        total_tp += s.true_positives;
        total_fp += s.false_positives;
        total_tn += s.true_negatives;
        total_fn += s.false_negatives;

        rec.multihop_forward_count += r.packets_forwarded;
        rec.routing_loops_suppressed += r.routing_loops_suppressed;
        rec.ttl_expired_dropped += r.ttl_expired_dropped;
        rec.custody_acks_sent += w.acks_sent;
        rec.e2e_acks_sent += w.acks_received;
        rec.transit_buffer_evictions += r.transit_buffered;
    }

    rec.genuine_proposals = rec.proposals_sent;
    rec.imposter_proposals = imposters_injected;
    rec.proposals_sent += imposters_injected;

    rec.true_positives = total_tp;
    rec.false_positives = total_fp;
    rec.true_negatives = total_tn;
    rec.false_negatives = total_fn;

    const uint64_t pred_pos = total_tp + total_fp;
    rec.precision = (pred_pos > 0) ? (static_cast<double>(total_tp) / static_cast<double>(pred_pos)) : 0.0;

    const uint64_t act_pos = total_tp + total_fn;
    rec.recall = (act_pos > 0) ? (static_cast<double>(total_tp) / static_cast<double>(act_pos)) : 0.0;

    const uint64_t act_neg = total_tn + total_fp;
    rec.specificity = (act_neg > 0) ? (static_cast<double>(total_tn) / static_cast<double>(act_neg)) : 0.0;

    rec.fpr = (act_neg > 0) ? (static_cast<double>(total_fp) / static_cast<double>(act_neg)) : 0.0;
    rec.fnr = (act_pos > 0) ? (static_cast<double>(total_fn) / static_cast<double>(act_pos)) : 0.0;

    if (rec.precision + rec.recall > 1e-9) {
        rec.f1_score = 2.0 * (rec.precision * rec.recall) / (rec.precision + rec.recall);
    } else {
        rec.f1_score = 0.0;
    }

    // Stop all nodes
    for (auto& n : nodes) {
        n->stop();
    }

    const auto t_end = std::chrono::steady_clock::now();
    rec.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    return rec;
}

int main() {
    std::cout << "=================================================================\n";
    std::cout << "  ASTRAMESH-X M7.11 MULTI-HOP MESH FACTORIAL BENCHMARK SUITE    \n";
    std::cout << "=================================================================\n";

    const std::string csv_path = "benchmark_multihop_trials_reconciled.csv";
    std::ofstream csv(csv_path, std::ios::out | std::ios::trunc);
    if (!csv.is_open()) {
        std::cerr << "Failed to open CSV file for writing: " << csv_path << "\n";
        return 1;
    }

    // Write CSV Header
    csv << "trial_id,topology,seed,num_rovers,packet_loss_rate,delay_ms,elapsed_ms,"
        << "sync_packets_sent,sync_packets_received,proposals_sent,proposals_received,"
        << "genuine_proposals,imposter_proposals,loop_closures_verified,loop_closures_rejected,"
        << "true_positives,false_positives,true_negatives,false_negatives,"
        << "precision,recall,specificity,fpr,fnr,f1_score,"
        << "multihop_forward_count,routing_loops_suppressed,ttl_expired_dropped,"
        << "custody_acks_sent,e2e_acks_sent,transit_buffer_evictions,"
        << "global_connected_components,is_fully_unified,"
        << "initial_mean_ate,optimized_mean_ate,ate_improvement_pct\n";

    struct TopologyConfig {
        std::string name;
        size_t rovers;
    };

    std::vector<TopologyConfig> topologies = {
        { "2_Rover_Direct", 2 },
        { "3_Rover_Linear", 3 },
        { "4_Rover_Linear", 4 },
        { "4_Rover_Diamond", 4 },
        { "4_Rover_Broken_Failover", 4 }
    };

    struct ImpairmentConfig {
        double loss_rate;
        double delay_ms;
    };

    std::vector<ImpairmentConfig> impairments = {
        { 0.00,  0.0 },
        { 0.05,  5.0 },
        { 0.10, 10.0 }
    };

    std::vector<uint64_t> seeds = { 42, 101, 2024 };

    std::vector<MultihopBenchmarkRecord> all_records;
    uint32_t trial_counter = 1;
    uint16_t base_port = 23000;

    const size_t total_trials = topologies.size() * impairments.size() * seeds.size();
    std::cout << "Executing " << total_trials << " Factorial Multi-Hop Trials...\n\n";

    for (const auto& top : topologies) {
        for (const auto& imp : impairments) {
            for (uint64_t seed : seeds) {
                std::cout << "Running Trial [" << std::setw(2) << trial_counter << "/" << total_trials << "] "
                          << std::left << std::setw(24) << top.name
                          << " Loss=" << std::fixed << std::setprecision(2) << imp.loss_rate
                          << " Delay=" << std::setprecision(0) << imp.delay_ms << "ms"
                          << " Seed=" << seed << " ... " << std::flush;

                auto rec = run_multihop_trial(
                    trial_counter,
                    top.name,
                    top.rovers,
                    imp.loss_rate,
                    imp.delay_ms,
                    seed,
                    base_port);

                base_port += 20;
                all_records.push_back(rec);

                // Write row to CSV
                csv << rec.trial_id << ","
                    << rec.topology << ","
                    << rec.seed << ","
                    << rec.num_rovers << ","
                    << std::fixed << std::setprecision(4) << rec.packet_loss_rate << ","
                    << std::fixed << std::setprecision(2) << rec.delay_ms << ","
                    << std::fixed << std::setprecision(2) << rec.elapsed_ms << ","
                    << rec.sync_packets_sent << ","
                    << rec.sync_packets_received << ","
                    << rec.proposals_sent << ","
                    << rec.proposals_received << ","
                    << rec.genuine_proposals << ","
                    << rec.imposter_proposals << ","
                    << rec.loop_closures_verified << ","
                    << rec.loop_closures_rejected << ","
                    << rec.true_positives << ","
                    << rec.false_positives << ","
                    << rec.true_negatives << ","
                    << rec.false_negatives << ","
                    << std::fixed << std::setprecision(4) << rec.precision << ","
                    << std::fixed << std::setprecision(4) << rec.recall << ","
                    << std::fixed << std::setprecision(4) << rec.specificity << ","
                    << std::fixed << std::setprecision(4) << rec.fpr << ","
                    << std::fixed << std::setprecision(4) << rec.fnr << ","
                    << std::fixed << std::setprecision(4) << rec.f1_score << ","
                    << rec.multihop_forward_count << ","
                    << rec.routing_loops_suppressed << ","
                    << rec.ttl_expired_dropped << ","
                    << rec.custody_acks_sent << ","
                    << rec.e2e_acks_sent << ","
                    << rec.transit_buffer_evictions << ","
                    << rec.global_connected_components << ","
                    << (rec.is_fully_unified ? 1 : 0) << ","
                    << std::fixed << std::setprecision(4) << rec.initial_mean_ate << ","
                    << std::fixed << std::setprecision(4) << rec.optimized_mean_ate << ","
                    << std::fixed << std::setprecision(2) << rec.ate_improvement_pct << "\n";
                csv.flush();

                std::cout << "Done (" << std::setprecision(0) << rec.elapsed_ms << "ms) | "
                          << "Fwd=" << rec.multihop_forward_count << " | "
                          << "TP=" << rec.true_positives << " FP=" << rec.false_positives << " | "
                          << "Unified=" << (rec.is_fully_unified ? "YES" : "NO") << "\n";

                trial_counter++;
            }
        }
    }

    csv.close();
    std::cout << "\n=================================================================\n";
    std::cout << "  MULTI-HOP BENCHMARK COMPLETED: " << all_records.size() << " TRIALS RECORDED\n";
    std::cout << "  Artifact: " << csv_path << "\n";
    std::cout << "=================================================================\n";
    return 0;
}
