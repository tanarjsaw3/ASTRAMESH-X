#include "astromesh/core/iblt.hpp"
#include "astromesh/core/landmark_key.hpp"
#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/net/distributed_reconciliation.hpp"
#include "astromesh/net/rover_node.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

using namespace astromesh::net;
using namespace astromesh::core;
using namespace astromesh::graph;
using namespace astromesh::geometry;

struct BenchmarkTrialResult {
    uint32_t trial_id{0};
    uint64_t seed{42};
    size_t rover_count{2};
    double packet_loss_rate{0.0};
    double delay_ms{0.0};
    double elapsed_compute_ms{0.0};

    // Communication metrics
    uint64_t sync_packets_sent{0};
    uint64_t sync_packets_received{0};
    uint64_t decodes_succeeded{0};
    uint64_t decodes_failed{0};
    uint64_t proposals_sent{0};
    uint64_t proposals_received{0};
    uint64_t loop_closures_verified{0};
    uint64_t loop_closures_rejected{0};
    uint64_t dsu_unions_performed{0};
    size_t final_dsu_components{0};

    // Wire byte metrics
    uint64_t payload_bytes_sent{0};
    uint64_t header_bytes_sent{0};
    uint64_t framing_bytes_sent{0};
    uint64_t data_wire_bytes{0};
    uint64_t ack_wire_bytes{0};
    uint64_t total_wire_bytes{0};
    uint64_t retransmissions{0};
    uint64_t duplicate_suppressed{0};

    // Trajectory error
    double swarm_ate_rmse{0.0};
};

static BenchmarkTrialResult run_trial(
    uint32_t trial_id,
    size_t rover_count,
    double loss_rate,
    double delay_ms,
    uint64_t seed,
    uint16_t base_port) {

    BenchmarkTrialResult res;
    res.trial_id = trial_id;
    res.seed = seed;
    res.rover_count = rover_count;
    res.packet_loss_rate = loss_rate;
    res.delay_ms = delay_ms;

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
        cfg.dtn_config.initial_retry_delay_ns = 35'000'000ULL; // 35 ms retry
        cfg.impairment_config.packet_loss_rate = loss_rate;
        cfg.impairment_config.one_way_delay_ns = static_cast<uint64_t>(delay_ms * 1'000'000.0);
        cfg.impairment_config.seed = seed + i * 100ULL;

        // Register immediate neighbor as seed peer for discovery
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

        // Setup local keyframe and landmark observations
        // Rover i is positioned along X axis at x = (i-1) * 10.0
        const double rx = static_cast<double>(i - 1) * 10.0;
        const SE2 pose(rx, 0.0, 0.0);

        // Landmarks: 5 per rover. Rovers i and i+1 share landmarks (i*10 + 1, i*10 + 2, i*10 + 3)
        std::vector<LandmarkObservationRecord> obs;
        // Shared landmarks with right neighbor (if exists)
        if (i < rover_count) {
            const uint32_t base_shared = static_cast<uint32_t>(i * 10);
            obs.push_back({ LandmarkKey(1, static_cast<uint16_t>(i), base_shared + 1, 0x1000ULL + base_shared + 1),
                            2.0, 3.0, 0 });
            obs.push_back({ LandmarkKey(1, static_cast<uint16_t>(i), base_shared + 2, 0x1000ULL + base_shared + 2),
                            4.0, 1.0, 0 });
            obs.push_back({ LandmarkKey(1, static_cast<uint16_t>(i), base_shared + 3, 0x1000ULL + base_shared + 3),
                            3.0, -2.0, 0 });
        }
        // Shared landmarks with left neighbor (if exists)
        if (i > 1) {
            const uint32_t base_shared = static_cast<uint32_t>((i - 1) * 10);
            // In Rover i frame, relative x is (original_x - 10.0)
            obs.push_back({ LandmarkKey(1, static_cast<uint16_t>(i - 1), base_shared + 1, 0x1000ULL + base_shared + 1),
                            2.0 - 10.0, 3.0, 0 });
            obs.push_back({ LandmarkKey(1, static_cast<uint16_t>(i - 1), base_shared + 2, 0x1000ULL + base_shared + 2),
                            4.0 - 10.0, 1.0, 0 });
            obs.push_back({ LandmarkKey(1, static_cast<uint16_t>(i - 1), base_shared + 3, 0x1000ULL + base_shared + 3),
                            3.0 - 10.0, -2.0, 0 });
        }
        // Unique landmark
        const uint32_t unique_id = static_cast<uint32_t>(100 + i);
        obs.push_back({ LandmarkKey(1, static_cast<uint16_t>(i), unique_id, 0x9000ULL + unique_id),
                        1.0, 5.0, 0 });

        agent->add_local_keyframe(0, pose, obs);

        nodes.push_back(std::move(node));
        agents.push_back(std::move(agent));
    }

    // 2. Mutual peer discovery phase (5 ticks)
    for (int t = 0; t < 5; ++t) {
        for (auto& n : nodes) n->poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 3. Initiate IBLT sync from odd rovers to even rovers
    for (size_t i = 1; i < rover_count; ++i) {
        agents[i - 1]->initiate_sync_with_peer(static_cast<uint16_t>(i + 1));
    }

    // 4. Distributed execution event loop (up to 30 ticks of 15ms)
    for (int step = 0; step < 30; ++step) {
        for (auto& n : nodes) {
            n->poll_events(0);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }

    // 5. Aggregate metrics across rovers
    double total_sq_err = 0.0;
    size_t closures_evaluated = 0;

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

        res.payload_bytes_sent += w.payload_bytes_sent;
        res.header_bytes_sent += w.header_bytes_sent;
        res.framing_bytes_sent += w.framing_bytes_sent;
        res.data_wire_bytes += w.data_wire_bytes_sent;
        res.ack_wire_bytes += w.ack_wire_bytes_sent;
        res.total_wire_bytes += w.total_wire_bytes_sent;
        res.retransmissions += w.dtn_retransmissions;
        res.duplicate_suppressed += w.duplicate_deliveries_suppressed;

        // Evaluate ATE on verified inter-rover edges
        const auto& g = agents[i]->get_pose_graph();
        for (const auto& [eid, edge] : g.edges()) {
            if (edge.type == astromesh::graph::EdgeType::InterRoverLoopClosure &&
                edge.status == astromesh::graph::EdgeVerificationStatus::Verified) {
                // Ground truth relative distance between adjacent rovers is exactly 10.0m
                const double est_dist = std::hypot(edge.relative_pose.x, edge.relative_pose.y);
                const double err = std::abs(est_dist - 10.0);
                total_sq_err += err * err;
                closures_evaluated++;
            }
        }
    }

    res.final_dsu_components = agents[0]->get_dsu_component_count();
    res.swarm_ate_rmse = closures_evaluated > 0 ? std::sqrt(total_sq_err / static_cast<double>(closures_evaluated)) : 0.0;

    const auto t_end = std::chrono::steady_clock::now();
    res.elapsed_compute_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    // 6. Stop all nodes
    for (auto& n : nodes) {
        n->stop();
    }

    return res;
}

int main(int argc, char* argv[]) {
    std::cout << "=================================================================\n"
              << "  ASTRAMESH-X DISTRIBUTED MULTI-ROVER SCIENTIFIC BENCHMARK SUITE \n"
              << "=================================================================\n";

    const std::string csv_path = "benchmark_distributed_trials.csv";
    std::ofstream csv(csv_path, std::ios::trunc);
    if (!csv.is_open()) {
        std::cerr << "Failed to open " << csv_path << " for writing!\n";
        return 1;
    }

    // CSV Header
    csv << "trial_id,seed,rover_count,loss_rate,delay_ms,elapsed_ms,"
        << "sync_sent,sync_recv,decodes_succ,decodes_fail,proposals_sent,proposals_recv,"
        << "closures_verified,closures_rejected,dsu_unions,final_dsu_components,"
        << "payload_bytes,header_bytes,framing_bytes,data_wire_bytes,ack_wire_bytes,total_wire_bytes,"
        << "retransmissions,duplicate_suppressed,swarm_ate_rmse\n";

    // Factorial experimental design:
    // Rover counts: 2, 3, 4
    // Loss rates: 0.0, 0.20, 0.50
    // Latencies: 0.0, 50.0 ms, 100.0 ms
    // Seeds: 3 distinct random seeds
    // Total runs = 3 x 3 x 3 x 3 = 81 trials
    const std::vector<size_t> rover_counts = {2, 3, 4};
    const std::vector<double> loss_rates = {0.0, 0.20, 0.50};
    const std::vector<double> delays_ms = {0.0, 50.0, 100.0};
    const std::vector<uint64_t> seeds = {1001, 2002, 3003};

    const size_t total_trials = rover_counts.size() * loss_rates.size() * delays_ms.size() * seeds.size();
    std::cout << "Running " << total_trials << " distributed trials across UDP loopback sockets...\n"
              << "Sweep grid: rovers={2,3,4}, loss={0%,20%,50%}, delay={0ms,50ms,100ms}, seeds=3\n\n";

    uint32_t trial_id = 0;
    uint16_t port_counter = 22000;

    for (size_t rc : rover_counts) {
        for (double loss : loss_rates) {
            for (double delay : delays_ms) {
                for (uint64_t seed : seeds) {
                    trial_id++;
                    port_counter += 10;
                    if (port_counter > 30000) port_counter = 22000;

                    auto r = run_trial(trial_id, rc, loss, delay, seed, port_counter);

                    // Strict wire-byte accounting assertion
                    assert(r.data_wire_bytes == r.payload_bytes_sent + r.header_bytes_sent + r.framing_bytes_sent);
                    assert(r.total_wire_bytes == r.data_wire_bytes + r.ack_wire_bytes);

                    csv << r.trial_id << ","
                        << r.seed << ","
                        << r.rover_count << ","
                        << std::fixed << std::setprecision(2) << r.packet_loss_rate << ","
                        << r.delay_ms << ","
                        << std::setprecision(1) << r.elapsed_compute_ms << ","
                        << r.sync_packets_sent << ","
                        << r.sync_packets_received << ","
                        << r.decodes_succeeded << ","
                        << r.decodes_failed << ","
                        << r.proposals_sent << ","
                        << r.proposals_received << ","
                        << r.loop_closures_verified << ","
                        << r.loop_closures_rejected << ","
                        << r.dsu_unions_performed << ","
                        << r.final_dsu_components << ","
                        << r.payload_bytes_sent << ","
                        << r.header_bytes_sent << ","
                        << r.framing_bytes_sent << ","
                        << r.data_wire_bytes << ","
                        << r.ack_wire_bytes << ","
                        << r.total_wire_bytes << ","
                        << r.retransmissions << ","
                        << r.duplicate_suppressed << ","
                        << std::setprecision(4) << r.swarm_ate_rmse << "\n";
                    csv.flush();

                    if (trial_id % 9 == 0 || trial_id == total_trials) {
                        std::cout << "  [" << std::setw(2) << trial_id << "/" << total_trials << "]"
                                  << " Rovers: " << rc
                                  << " | Loss: " << std::setw(3) << static_cast<int>(loss * 100) << "%"
                                  << " | Delay: " << std::setw(3) << static_cast<int>(delay) << "ms"
                                  << " | Wire: " << std::setw(7) << r.total_wire_bytes << " B"
                                  << " | Closures: " << r.loop_closures_verified
                                  << " | DSU: " << r.final_dsu_components
                                  << " | ATE: " << std::fixed << std::setprecision(4) << r.swarm_ate_rmse << "m\n";
                    }
                }
            }
        }
    }

    csv.close();
    std::cout << "\n=================================================================\n"
              << "  DISTRIBUTED BENCHMARK COMPLETE! Results saved to:\n"
              << "  -> " << csv_path << " (" << total_trials << " trials recorded)\n"
              << "=================================================================\n";
    return 0;
}
