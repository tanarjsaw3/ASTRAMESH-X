#include "astromesh/orchestration/multi_rover_collective.hpp"
#include "astromesh/optimization/boundary_packet.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace astromesh;
using namespace astromesh::orchestration;

struct BenchmarkScenarioRunner {
    std::string name;
    size_t rover_count{2};
    uint64_t duration_ms{500};
    uint64_t step_ms{10};
    uint64_t latency_ms{10};
    double packet_loss_rate{0.0};
    bool enable_retries{true};
    bool enable_filtering{true};
    bool enable_pruning{true};
    size_t epoch_quota{0}; // 0 = unlimited
    uint64_t partition_start_ms{0};
    uint64_t partition_duration_ms{0};
    bool inject_shared_landmarks{false};
    bool inject_good_geometry{false};
    bool inject_bad_geometry{false};
    uint64_t seed{42};
};

static CollectiveMetrics run_scenario(const BenchmarkScenarioRunner& sc) {
    CollectiveSimulationConfig cfg;
    cfg.scenario_name = sc.name;
    cfg.random_seed = sc.seed;
    cfg.network_config.latency_ns = sc.latency_ms * 1'000'000;
    cfg.network_config.packet_loss_rate = sc.packet_loss_rate;
    cfg.enable_dtn_retries = sc.enable_retries;
    cfg.enable_boundary_filtering = sc.enable_filtering;
    cfg.enable_keyframe_pruning = sc.enable_pruning;

    for (size_t i = 1; i <= sc.rover_count; ++i) {
        SimulatedRoverConfig rcfg;
        rcfg.rover_id = static_cast<uint16_t>(i);
        rcfg.initial_pose = geometry::SE2(static_cast<double>(i * 5), 0.0, 0.0);
        if (sc.epoch_quota > 0) {
            rcfg.budget_config.max_bytes_per_window = sc.epoch_quota;
        }
        cfg.rover_configs.push_back(rcfg);
    }

    MultiRoverCollective collective(cfg);

    // If shared landmarks requested
    if (sc.inject_shared_landmarks && sc.rover_count >= 2) {
        auto* r1 = collective.get_rover(1);
        auto* r2 = collective.get_rover(2);
        if (r1 && r2) {
            for (uint32_t k = 1; k <= 5; ++k) {
                core::LandmarkKey shared_lm(1, 1, k, k * 0xABCD);
                r1->iblt().insert(shared_lm);
                r2->iblt().insert(shared_lm);
            }
        }
    }

    const uint64_t duration_ns = sc.duration_ms * 1'000'000;
    const uint64_t step_ns = sc.step_ms * 1'000'000;
    const uint64_t partition_start_ns = sc.partition_start_ms * 1'000'000;
    const uint64_t partition_end_ns = partition_start_ns + (sc.partition_duration_ms * 1'000'000);

    uint64_t elapsed_ns = 0;
    uint32_t msg_seq = 1;

    while (elapsed_ns < duration_ns) {
        // Partition management
        if (sc.partition_duration_ms > 0) {
            if (elapsed_ns >= partition_start_ns && elapsed_ns < partition_end_ns) {
                collective.set_link_state(1, 2, false);
            } else {
                collective.set_link_state(1, 2, true);
            }
        }

        // Periodic message generation every 20 ms
        if (elapsed_ns % 20'000'000 == 0) {
            for (size_t s = 1; s <= sc.rover_count; ++s) {
                size_t t = (s % sc.rover_count) + 1;
                std::vector<uint8_t> payload = {
                    static_cast<uint8_t>(s),
                    static_cast<uint8_t>(t),
                    static_cast<uint8_t>(msg_seq & 0xFF),
                    static_cast<uint8_t>((msg_seq >> 8) & 0xFF)
                };
                uint64_t ttl_ns = (sc.partition_duration_ms > 500) ? 200'000'000 : 1'000'000'000;
                collective.enqueue_message(
                    static_cast<uint16_t>(s),
                    static_cast<uint16_t>(t),
                    comms::DtnMessageType::GenericData,
                    comms::PriorityLevel::Normal,
                    payload,
                    ttl_ns,
                    msg_seq++);
            }
        }

        // Periodic boundary packet exchange every 50 ms to evaluate prediction and filtering
        if (elapsed_ns % 50'000'000 == 0 && sc.rover_count >= 2) {
            optimization::BoundaryStatePacket bp;
            bp.protocol_version = optimization::BOUNDARY_PROTOCOL_VERSION;
            bp.sender_rover_id = 1;
            bp.target_rover_id = 2;
            bp.node_local_index = static_cast<uint32_t>((elapsed_ns / 50'000'000) + 1);
            bp.sequence_number = static_cast<uint32_t>((elapsed_ns / 50'000'000) + 1);
            bp.timestamp_ns = elapsed_ns;
            bp.x = 1.0; bp.y = 0.5; bp.theta = 0.1;
            bp.vx = 0.1; bp.vy = 0.0; bp.omega = 0.01;
            std::vector<uint8_t> bp_payload = optimization::serialize_boundary_packet(bp);
            collective.enqueue_message(
                1, 2, comms::DtnMessageType::BoundaryState,
                comms::PriorityLevel::High, bp_payload, 1'000'000'000, bp.sequence_number);
        }

        // Odometry steps every 50 ms
        if (elapsed_ns % 50'000'000 == 0) {
            for (size_t s = 1; s <= sc.rover_count; ++s) {
                auto* rover = collective.get_rover(static_cast<uint16_t>(s));
                if (rover) {
                    rover->add_odometry_step(geometry::SE2(0.5, 0.0, 0.05), elapsed_ns);
                }
            }
        }

        collective.step(step_ns);
        elapsed_ns += step_ns;
    }

    // Run geometric verification synchronization if configured
    if (sc.inject_good_geometry && sc.rover_count >= 2) {
        std::vector<verification::PointCorrespondence2D> good_corrs = {
            {{0.0, 0.0}, {2.0, 0.0}},
            {{1.0, 0.0}, {3.0, 0.0}},
            {{0.0, 1.0}, {2.0, 1.0}},
            {{1.0, 1.0}, {3.0, 1.0}}
        };
        collective.perform_synchronization(1, 2, good_corrs);
    } else if (sc.inject_bad_geometry && sc.rover_count >= 2) {
        std::vector<verification::PointCorrespondence2D> bad_corrs = {
            {{0.0, 0.0}, {10.0, 0.0}},
            {{1.0, 0.0}, {0.0, 10.0}},
            {{0.0, 1.0}, {5.0, 5.0}},
            {{1.0, 1.0}, {-2.0, 8.0}}
        };
        collective.perform_synchronization(1, 2, bad_corrs);
    }

    CollectiveMetrics m = collective.collect_metrics();
    m.scenario_name = sc.name;
    m.partition_duration_ms = static_cast<double>(sc.partition_duration_ms);
    return m;
}

int main(int argc, char** argv) {
    std::string output_path = "benchmark_multi_rover_results.csv";
    if (argc > 1) {
        output_path = argv[1];
    }

    std::cout << "======================================================================" << std::endl;
    std::cout << " AstroMesh-X Phase 4.4 Multi-Rover Collective Benchmark" << std::endl;
    std::cout << " Output: " << output_path << std::endl;
    std::cout << "======================================================================" << std::endl;

    std::vector<BenchmarkScenarioRunner> scenarios = {
        {"Reliable_2Rovers_Baseline", 2, 500, 10, 10, 0.0, true, true, true, 0, 0, 0, false, false, false, 42},
        {"Reliable_3Rovers_Mesh", 3, 500, 10, 15, 0.0, true, true, true, 0, 0, 0, false, false, false, 42},
        {"Temporary_Partition_200ms", 2, 500, 10, 10, 0.0, true, true, true, 0, 100, 200, false, false, false, 42},
        {"Long_Partition_1000ms_TTLExpiry", 2, 1200, 10, 10, 0.0, true, true, true, 0, 100, 1000, false, false, false, 42},
        {"High_Loss_30pct_NoRetry", 2, 500, 10, 10, 0.30, false, true, true, 0, 0, 0, false, false, false, 42},
        {"High_Loss_30pct_DTNRetry", 2, 500, 10, 10, 0.30, true, true, true, 0, 0, 0, false, false, false, 42},
        {"High_Loss_50pct_DTNRetry", 2, 500, 10, 10, 0.50, true, true, true, 0, 0, 0, false, false, false, 42},
        {"High_Latency_50ms_NoPred", 2, 500, 10, 50, 0.0, true, false, true, 0, 0, 0, false, false, false, 42},
        {"High_Latency_50ms_FilteredPred", 2, 500, 10, 50, 0.0, true, true, true, 0, 0, 0, false, false, false, 42},
        {"ByteBudget_Strict_2000Bps", 2, 500, 10, 10, 0.0, true, true, true, 200, 0, 0, false, false, false, 42},
        {"ByteBudget_Generous_10000Bps", 2, 500, 10, 10, 0.0, true, true, true, 2000, 0, 0, false, false, false, 42},
        {"Reconciliation_Verified_Closure", 2, 500, 10, 10, 0.0, true, true, true, 0, 0, 0, true, true, false, 42},
        {"Reconciliation_Aliased_Rejected", 2, 500, 10, 10, 0.0, true, true, true, 0, 0, 0, true, false, true, 42},
        {"KeyframePruning_BoundEnforced", 2, 500, 10, 10, 0.0, true, true, true, 0, 0, 0, false, false, false, 42},
        {"MultiRover_Partition_2Subswarms", 4, 500, 10, 15, 0.0, true, true, true, 0, 100, 300, false, false, false, 42}
    };

    std::ofstream csv(output_path);
    if (!csv.is_open()) {
        std::cerr << "Failed to open output CSV: " << output_path << std::endl;
        return 1;
    }

    // Write CSV Header with audited metrics
    csv << "scenario_name,seed,rover_count,duration_ms,step_ms,latency_ms,packet_loss_rate,"
        << "partition_duration_ms,messages_generated,messages_admitted,messages_sent,"
        << "initial_transmission_attempts,messages_retried,messages_delivered,"
        << "unique_messages_delivered,duplicate_deliveries,messages_expired,messages_dropped,"
        << "messages_rejected,retries_exhausted,in_flight_messages,messages_processed,"
        << "payload_bytes_sent,protocol_bytes_sent,framing_bytes_sent,total_wire_bytes_sent,"
        << "total_wire_bytes_delivered,boundary_packets_filtered,state_transitions_total,"
        << "sync_attempts,sync_successes,sync_incomplete_decodes,sync_budget_denials,"
        << "queue_peak_messages,queue_peak_payload_bytes,final_connected_components,"
        << "verified_inter_rover_closures\n";

    for (const auto& sc : scenarios) {
        std::cout << "Running scenario: " << sc.name << "..." << std::flush;
        auto m = run_scenario(sc);
        std::cout << " [Done] Delivered: " << m.messages_delivered
                  << " (Unique: " << m.unique_messages_delivered << ")"
                  << ", Sent: " << m.messages_sent
                  << " (Init: " << m.initial_transmission_attempts
                  << ", Retries: " << m.messages_retried << ")"
                  << ", Dropped: " << m.messages_dropped
                  << ", Filtered: " << m.boundary_packets_filtered << std::endl;

        csv << m.scenario_name << ","
            << m.seed << ","
            << m.rover_count << ","
            << std::fixed << std::setprecision(1) << m.duration_ms << ","
            << m.step_ms << ","
            << m.latency_ms << ","
            << std::setprecision(2) << m.packet_loss_rate << ","
            << std::setprecision(1) << m.partition_duration_ms << ","
            << m.messages_generated << ","
            << m.messages_admitted << ","
            << m.messages_sent << ","
            << m.initial_transmission_attempts << ","
            << m.messages_retried << ","
            << m.messages_delivered << ","
            << m.unique_messages_delivered << ","
            << m.duplicate_deliveries << ","
            << m.messages_expired << ","
            << m.messages_dropped << ","
            << m.messages_rejected << ","
            << m.retries_exhausted << ","
            << m.in_flight_messages << ","
            << m.messages_processed << ","
            << m.payload_bytes_sent << ","
            << m.protocol_bytes_sent << ","
            << m.framing_bytes_sent << ","
            << m.total_wire_bytes_sent << ","
            << m.total_wire_bytes_delivered << ","
            << m.boundary_packets_filtered << ","
            << m.state_transitions_total << ","
            << m.sync_attempts << ","
            << m.sync_successes << ","
            << m.sync_incomplete_decodes << ","
            << m.sync_budget_denials << ","
            << m.queue_peak_messages << ","
            << m.queue_peak_payload_bytes << ","
            << m.final_connected_components << ","
            << m.verified_inter_rover_closures << "\n";
    }

    csv.close();
    std::cout << "======================================================================" << std::endl;
    std::cout << " SUCCESS: Wrote " << scenarios.size() << " records to " << output_path << std::endl;
    std::cout << "======================================================================" << std::endl;

    return 0;
}

