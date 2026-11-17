#include "astromesh/core/iblt.hpp"
#include "astromesh/core/landmark_key.hpp"
#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/net/distributed_reconciliation.hpp"
#include "astromesh/net/rover_node.hpp"
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

struct NoisyBenchmarkTrialResult {
    uint32_t trial_id{0};
    uint64_t seed{42};
    size_t rover_count{2};
    double sigma_landmark{0.01};
    double sigma_odom{0.02};
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

    // Trajectory and SLAM accuracy metrics
    double swarm_ate_rmse{0.0};
    double relative_pose_error{0.0};
    double mean_residual_error{0.0};
};

static NoisyBenchmarkTrialResult run_noisy_trial(
    uint32_t trial_id,
    size_t rover_count,
    double sigma_lm,
    double sigma_odom,
    double loss_rate,
    double delay_ms,
    uint64_t seed,
    uint16_t base_port) {

    NoisyBenchmarkTrialResult res;
    res.trial_id = trial_id;
    res.seed = seed;
    res.rover_count = rover_count;
    res.sigma_landmark = sigma_lm;
    res.sigma_odom = sigma_odom;
    res.packet_loss_rate = loss_rate;
    res.delay_ms = delay_ms;

    std::mt19937_64 rng(seed);
    std::normal_distribution<double> dist_noise(0.0, sigma_lm);
    std::normal_distribution<double> dist_odom(0.0, sigma_odom);

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

        // Local rover pose with optional odometry drift perturbation
        const double odom_dx = (i > 1) ? dist_odom(rng) : 0.0;
        const double odom_dy = (i > 1) ? dist_odom(rng) : 0.0;
        const double rx = static_cast<double>(i - 1) * 10.0 + odom_dx;
        const double ry = odom_dy;
        const SE2 pose(rx, ry, 0.0);

        // Generate landmark observations with Gaussian noise
        std::vector<LandmarkObservationRecord> obs;

        // Shared landmarks with right neighbor
        if (i < rover_count) {
            const uint32_t base_shared = static_cast<uint32_t>(i * 10);
            // 4 consistent landmarks + 1 outlier
            obs.push_back({ LandmarkKey(1, static_cast<uint16_t>(i), base_shared + 1, 0x1000ULL + base_shared + 1),
                            2.0 + dist_noise(rng), 3.0 + dist_noise(rng), 0 });
            obs.push_back({ LandmarkKey(1, static_cast<uint16_t>(i), base_shared + 2, 0x1000ULL + base_shared + 2),
                            4.0 + dist_noise(rng), 1.0 + dist_noise(rng), 0 });
            obs.push_back({ LandmarkKey(1, static_cast<uint16_t>(i), base_shared + 3, 0x1000ULL + base_shared + 3),
                            3.0 + dist_noise(rng), -2.0 + dist_noise(rng), 0 });
            obs.push_back({ LandmarkKey(1, static_cast<uint16_t>(i), base_shared + 4, 0x1000ULL + base_shared + 4),
                            5.0 + dist_noise(rng), 2.0 + dist_noise(rng), 0 });
            // Outlier landmark (spurious match to verify RANSAC outlier filtering)
            obs.push_back({ LandmarkKey(1, static_cast<uint16_t>(i), base_shared + 5, 0x1000ULL + base_shared + 5),
                            1.0, 4.0, 0 });
        }

        // Shared landmarks with left neighbor
        if (i > 1) {
            const uint32_t base_shared = static_cast<uint32_t>((i - 1) * 10);
            // True relative x in Rover i frame is (original_x - 10.0)
            obs.push_back({ LandmarkKey(1, static_cast<uint16_t>(i - 1), base_shared + 1, 0x1000ULL + base_shared + 1),
                            2.0 - 10.0 + dist_noise(rng), 3.0 + dist_noise(rng), 0 });
            obs.push_back({ LandmarkKey(1, static_cast<uint16_t>(i - 1), base_shared + 2, 0x1000ULL + base_shared + 2),
                            4.0 - 10.0 + dist_noise(rng), 1.0 + dist_noise(rng), 0 });
            obs.push_back({ LandmarkKey(1, static_cast<uint16_t>(i - 1), base_shared + 3, 0x1000ULL + base_shared + 3),
                            3.0 - 10.0 + dist_noise(rng), -2.0 + dist_noise(rng), 0 });
            obs.push_back({ LandmarkKey(1, static_cast<uint16_t>(i - 1), base_shared + 4, 0x1000ULL + base_shared + 4),
                            5.0 - 10.0 + dist_noise(rng), 2.0 + dist_noise(rng), 0 });
            // Outlier landmark: huge offset in neighbor frame to test outlier rejection
            obs.push_back({ LandmarkKey(1, static_cast<uint16_t>(i - 1), base_shared + 5, 0x1000ULL + base_shared + 5),
                            -25.0, 18.0, 0 });
        }

        // Unique landmark
        const uint32_t unique_id = static_cast<uint32_t>(100 + i);
        obs.push_back({ LandmarkKey(1, static_cast<uint16_t>(i), unique_id, 0x9000ULL + unique_id),
                        1.0 + dist_noise(rng), 5.0 + dist_noise(rng), 0 });

        agent->add_local_keyframe(0, pose, obs);

        nodes.push_back(std::move(node));
        agents.push_back(std::move(agent));
    }

    // 2. Peer discovery phase (5 ticks)
    for (int t = 0; t < 5; ++t) {
        for (auto& n : nodes) n->poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // 3. Initiate IBLT sync from rover i to rover i+1
    for (size_t i = 1; i < rover_count; ++i) {
        agents[i - 1]->initiate_sync_with_peer(static_cast<uint16_t>(i + 1));
    }

    // 4. Execution loop (30 ticks)
    for (int step = 0; step < 30; ++step) {
        for (auto& n : nodes) {
            n->poll_events(0);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 5. Aggregate metrics
    double total_sq_err = 0.0;
    double total_abs_err = 0.0;
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

        // Evaluate ATE on verified inter-rover loop closures
        const auto& g = agents[i]->get_pose_graph();
        for (const auto& [eid, edge] : g.edges()) {
            if (edge.type == astromesh::graph::EdgeType::InterRoverLoopClosure &&
                edge.status == astromesh::graph::EdgeVerificationStatus::Verified) {
                // Ground truth relative baseline between adjacent rovers is 10.0m along X axis
                const double est_dist = std::hypot(edge.relative_pose.x, edge.relative_pose.y);
                const double dist_err = std::abs(est_dist - 10.0);
                total_sq_err += dist_err * dist_err;
                total_abs_err += dist_err;
                closures_evaluated++;
            }
        }
    }

    res.final_dsu_components = agents[0]->get_dsu_component_count();
    res.swarm_ate_rmse = closures_evaluated > 0 ? std::sqrt(total_sq_err / static_cast<double>(closures_evaluated)) : 0.0;
    res.relative_pose_error = closures_evaluated > 0 ? (total_abs_err / static_cast<double>(closures_evaluated)) : 0.0;

    const auto t_end = std::chrono::steady_clock::now();
    res.elapsed_compute_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    // 6. Stop nodes
    for (auto& n : nodes) {
        n->stop();
    }

    return res;
}

int main(int argc, char* argv[]) {
    std::cout << "=================================================================\n"
              << "  ASTRAMESH-X SCIENTIFIC NOISY BENCHMARK (M7.8 VALIDATION)       \n"
              << "=================================================================\n";

    const std::string csv_path = "benchmark_distributed_noisy_trials.csv";
    std::ofstream csv(csv_path, std::ios::trunc);
    if (!csv.is_open()) {
        std::cerr << "Failed to open " << csv_path << " for writing!\n";
        return 1;
    }

    // CSV Header
    csv << "trial_id,seed,rover_count,sigma_landmark,sigma_odom,loss_rate,delay_ms,elapsed_ms,"
        << "sync_sent,sync_recv,decodes_succ,decodes_fail,proposals_sent,proposals_recv,"
        << "closures_verified,closures_rejected,dsu_unions,final_dsu_components,"
        << "payload_bytes,header_bytes,framing_bytes,data_wire_bytes,ack_wire_bytes,total_wire_bytes,"
        << "retransmissions,duplicate_suppressed,swarm_ate_rmse,relative_pose_error\n";

    // Factorial Design across:
    // Rover counts: 2, 3, 4
    // Landmark observation noise sigma: 0.01m, 0.05m, 0.10m
    // Network conditions:
    //   - Clear: (loss = 0.0, delay = 0.0 ms)
    //   - Moderate: (loss = 0.20, delay = 50.0 ms)
    //   - Severe: (loss = 0.50, delay = 100.0 ms)
    // Seeds: 3 distinct random seeds (1001, 2002, 3003)
    // Total runs = 3 x 3 x 3 x 3 = 81 trials
    const std::vector<size_t> rover_counts = {2, 3, 4};
    const std::vector<double> sigma_levels = {0.01, 0.05, 0.10};
    struct NetCondition { double loss; double delay_ms; };
    const std::vector<NetCondition> net_conditions = {
        {0.00, 0.0},
        {0.20, 50.0},
        {0.50, 100.0}
    };
    const std::vector<uint64_t> seeds = {1001, 2002, 3003};
    const double sigma_odom = 0.02;

    const size_t total_trials = rover_counts.size() * sigma_levels.size() * net_conditions.size() * seeds.size();
    std::cout << "Executing " << total_trials << " noisy trials across UDP loopback sockets...\n"
              << "Grid: rovers={2,3,4}, sigma_lm={0.01m, 0.05m, 0.10m}, sigma_odom=0.02m, net={Clear, Moderate, Severe}, seeds=3\n\n";

    uint32_t trial_id = 0;
    uint16_t port_counter = 24000;

    for (size_t rc : rover_counts) {
        for (double sigma_lm : sigma_levels) {
            for (const auto& net : net_conditions) {
                for (uint64_t seed : seeds) {
                    trial_id++;
                    port_counter += 10;
                    if (port_counter > 31000) port_counter = 24000;

                    auto r = run_noisy_trial(trial_id, rc, sigma_lm, sigma_odom, net.loss, net.delay_ms, seed, port_counter);

                    // Wire-byte accounting invariants
                    assert(r.data_wire_bytes == r.payload_bytes_sent + r.header_bytes_sent + r.framing_bytes_sent);
                    assert(r.total_wire_bytes == r.data_wire_bytes + r.ack_wire_bytes);

                    csv << r.trial_id << ","
                        << r.seed << ","
                        << r.rover_count << ","
                        << std::fixed << std::setprecision(3) << r.sigma_landmark << ","
                        << std::setprecision(3) << r.sigma_odom << ","
                        << std::setprecision(2) << r.packet_loss_rate << ","
                        << std::setprecision(1) << r.delay_ms << ","
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
                        << std::setprecision(4) << r.swarm_ate_rmse << ","
                        << std::setprecision(4) << r.relative_pose_error << "\n";
                    csv.flush();

                    if (trial_id % 9 == 0 || trial_id == total_trials) {
                        std::cout << "  [" << std::setw(2) << trial_id << "/" << total_trials << "]"
                                  << " Rovers: " << rc
                                  << " | Sigma: " << std::fixed << std::setprecision(2) << sigma_lm << "m"
                                  << " | Loss: " << std::setw(3) << static_cast<int>(net.loss * 100) << "%"
                                  << " | Delay: " << std::setw(3) << static_cast<int>(net.delay_ms) << "ms"
                                  << " | Closures: " << r.loop_closures_verified
                                  << " | DSU: " << r.final_dsu_components
                                  << " | ATE RMSE: " << std::fixed << std::setprecision(4) << r.swarm_ate_rmse << "m\n";
                    }
                }
            }
        }
    }

    csv.close();
    std::cout << "\n=================================================================\n"
              << "  SCIENTIFIC NOISY BENCHMARK COMPLETE! Results saved to:\n"
              << "  -> " << csv_path << " (" << total_trials << " trials recorded)\n"
              << "=================================================================\n";
    return 0;
}

