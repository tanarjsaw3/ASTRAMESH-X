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

struct Phase46ScenarioConfig {
    std::string name;
    std::string routing_mode{"MultiHop"};
    std::string ack_mode{"ExplicitACK"};
    size_t rover_count{2};
    uint64_t duration_ms{500};
    uint64_t step_ms{10};
    uint64_t latency_ms{10};
    double packet_loss_rate{0.0};
    double ack_loss_rate{-1.0}; // < 0 means same as packet_loss_rate
    bool enable_explicit_acks{true};
    bool enable_multihop_routing{true};
    bool enable_dtn_retries{true};
    size_t epoch_quota{0}; // 0 = unlimited
    uint8_t message_max_hops{5};
    uint64_t partition_start_ms{0};
    uint64_t partition_duration_ms{0};
    uint64_t message_ttl_ms{2000};
    bool cyclic_topology{false};
    bool line_topology_sever_direct{false};
    uint64_t seed{42};
};

static CollectiveMetrics run_phase46_scenario(const Phase46ScenarioConfig& sc) {
    CollectiveSimulationConfig cfg;
    cfg.scenario_name = sc.name;
    cfg.random_seed = sc.seed;
    cfg.network_config.latency_ns = sc.latency_ms * 1'000'000;
    cfg.network_config.packet_loss_rate = sc.packet_loss_rate;
    cfg.enable_explicit_acks = sc.enable_explicit_acks;
    cfg.enable_multihop_routing = sc.enable_multihop_routing;
    cfg.enable_dtn_retries = sc.enable_dtn_retries;
    cfg.default_max_hops = sc.message_max_hops;
    if (sc.ack_loss_rate >= 0.0) {
        cfg.ack_packet_loss_rate = sc.ack_loss_rate;
    }

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

    // Setup topology
    if (sc.line_topology_sever_direct && sc.rover_count >= 3) {
        // Line topology: 1 <-> 2 and 2 <-> 3 connected, 1 <-> 3 severed
        collective.set_link_state(1, 3, false);
    }

    const uint64_t duration_ns = sc.duration_ms * 1'000'000;
    const uint64_t step_ns = sc.step_ms * 1'000'000;
    const uint64_t partition_start_ns = sc.partition_start_ms * 1'000'000;
    const uint64_t partition_end_ns = partition_start_ns + (sc.partition_duration_ms * 1'000'000);

    uint64_t elapsed_ns = 0;
    uint32_t msg_seq = 1;

    while (elapsed_ns < duration_ns) {
        // Dynamic partition management
        if (sc.partition_duration_ms > 0) {
            if (elapsed_ns >= partition_start_ns && elapsed_ns < partition_end_ns) {
                if (sc.line_topology_sever_direct) {
                    collective.set_link_state(2, 3, false);
                } else {
                    collective.set_link_state(1, 2, false);
                }
            } else {
                if (sc.line_topology_sever_direct) {
                    collective.set_link_state(2, 3, true);
                } else {
                    collective.set_link_state(1, 2, true);
                }
            }
        }

        // Periodic offered message traffic every 25 ms
        if (elapsed_ns % 25'000'000 == 0) {
            if (sc.line_topology_sever_direct) {
                // Sender 1 sends to destination 3 via relay 2
                std::vector<uint8_t> payload = {
                    1, 3,
                    static_cast<uint8_t>(msg_seq & 0xFF),
                    static_cast<uint8_t>((msg_seq >> 8) & 0xFF)
                };
                collective.enqueue_message(
                    1, 3,
                    comms::DtnMessageType::GenericData,
                    comms::PriorityLevel::Normal,
                    payload,
                    sc.message_ttl_ms * 1'000'000,
                    msg_seq++,
                    sc.message_max_hops);
            } else {
                // Sender 1 sends to Receiver 2
                std::vector<uint8_t> payload = {
                    1, 2,
                    static_cast<uint8_t>(msg_seq & 0xFF),
                    static_cast<uint8_t>((msg_seq >> 8) & 0xFF)
                };
                collective.enqueue_message(
                    1, 2,
                    comms::DtnMessageType::GenericData,
                    comms::PriorityLevel::Normal,
                    payload,
                    sc.message_ttl_ms * 1'000'000,
                    msg_seq++,
                    sc.message_max_hops);
            }
        }

        collective.step(step_ns);
        elapsed_ns += step_ns;
    }

    CollectiveMetrics m = collective.collect_metrics();
    m.scenario_name = sc.name;
    m.partition_duration_ms = static_cast<double>(sc.partition_duration_ms);
    return m;
}

int main(int argc, char** argv) {
    std::string output_path = "benchmark_phase46_results.csv";
    if (argc > 1) {
        output_path = argv[1];
    }

    std::cout << "======================================================================" << std::endl;
    std::cout << " AstroMesh-X Phase 4.6 Reliable ACK & Multi-Hop DTN Benchmark" << std::endl;
    std::cout << " Output: " << output_path << std::endl;
    std::cout << "======================================================================" << std::endl;

    // 10 Scenarios covering direct reliable, lossy, ACK loss, store-carry-forward, partitions,
    // TTL expiry, byte budgets, buffer pressure, duplicate loops, and hop limits.
    std::vector<Phase46ScenarioConfig> scenarios = {
        // 1. Direct Link, Reliable Channel
        {"Direct_Reliable_Channel", "Direct", "ExplicitACK", 2, 500, 10, 10, 0.0, 0.0, true, false, true, 0, 5, 0, 0, 2000, false, false, 42},

        // 2. Direct Link, Lossy Channel (30% Data Drop with DTN Retries)
        {"Direct_Lossy_Channel_30pct", "Direct", "ExplicitACK", 2, 500, 10, 10, 0.30, 0.0, true, false, true, 0, 5, 0, 0, 2000, false, false, 42},

        // 3. Direct Link with ACK Loss (0% Data Loss, 50% Return ACK Loss)
        {"Direct_ACK_Loss_50pct", "Direct", "ExplicitACK", 2, 500, 10, 10, 0.0, 0.50, true, false, true, 0, 5, 0, 0, 2000, false, false, 42},

        // 4. Three-Rover Store-Carry-Forward (1 -> 2 -> 3 with Temporary 2-3 Partition)
        {"Three_Rover_Store_Carry_Forward", "MultiHop", "ExplicitACK", 3, 600, 10, 10, 0.0, 0.0, true, true, true, 0, 5, 50, 200, 2000, false, true, 42},

        // 5. Temporary Partition Followed by Reconnection (1 <-> 2 partitioned 100ms - 300ms)
        {"Temporary_Partition_Reconnection", "Direct", "ExplicitACK", 2, 500, 10, 10, 0.0, 0.0, true, false, true, 0, 5, 100, 200, 2000, false, false, 42},

        // 6. Long Partition Causing TTL Expiry (1 <-> 2 partitioned 100ms - 500ms with 150ms TTL)
        {"Long_Partition_TTL_Expiry", "Direct", "ExplicitACK", 2, 600, 10, 10, 0.0, 0.0, true, false, true, 0, 5, 100, 400, 150, false, false, 42},

        // 7. Strict Byte Budget with ACK Overhead (200B/window quota under explicit ACKs)
        {"Strict_Byte_Budget_With_ACK", "Direct", "ExplicitACK", 2, 500, 10, 10, 0.0, 0.0, true, false, true, 200, 5, 0, 0, 2000, false, false, 42},

        // 8. Intermediate Buffer Pressure (3 rovers, high offered load, 200B quota)
        {"Intermediate_Buffer_Pressure", "MultiHop", "ExplicitACK", 3, 500, 10, 15, 0.0, 0.0, true, true, true, 250, 5, 0, 0, 2000, false, true, 42},

        // 9. Duplicate Forwarding Pressure (Cyclic Triangle, Anti-Looping Suppression)
        {"Duplicate_Forwarding_Pressure", "MultiHop", "ExplicitACK", 3, 500, 10, 10, 0.0, 0.0, true, true, true, 0, 5, 0, 0, 2000, true, false, 42},

        // 10. Hop-Limit Exhaustion (1 -> 2 -> 3 with max_hops = 1, dropping at intermediate relay)
        {"Hop_Limit_Exhaustion", "MultiHop", "ExplicitACK", 3, 500, 10, 10, 0.0, 0.0, true, true, true, 0, 1, 0, 0, 2000, false, true, 42}
    };

    std::ofstream csv(output_path);
    if (!csv.is_open()) {
        std::cerr << "Failed to open output CSV: " << output_path << std::endl;
        return 1;
    }

    // Write audited CSV Header with clear, explicit metric definitions
    csv << "scenario_name,seed,routing_mode,ack_mode,rover_count,duration_ms,step_ms,latency_ms,"
        << "packet_loss_rate,ack_loss_rate,partition_duration_ms,messages_generated,messages_admitted,"
        << "unique_messages_processed,end_to_end_completion_ratio,hop_data_attempts,hop_data_delivered,"
        << "hop_data_losses,ack_transmission_attempts,ack_deliveries,ack_losses,"
        << "retransmissions_missing_ack,duplicate_deliveries,duplicate_processing_attempts,"
        << "messages_expired,messages_dropped,hop_limit_exhausted,in_flight_messages,"
        << "intermediate_queued_messages,queue_peak_messages,payload_bytes_sent,protocol_bytes_sent,"
        << "framing_bytes_sent,ack_wire_bytes_sent,ack_wire_bytes_delivered,total_wire_bytes_sent,"
        << "total_wire_bytes_delivered\n";

    for (const auto& sc : scenarios) {
        std::cout << "Running scenario: " << std::left << std::setw(35) << sc.name << "..." << std::flush;
        auto m = run_phase46_scenario(sc);
        std::cout << " [Done] Unique: " << m.unique_messages_processed
                  << "/" << m.messages_admitted
                  << " (Ratio: " << std::fixed << std::setprecision(2) << (m.end_to_end_completion_ratio * 100.0) << "%)"
                  << ", Data Hops: " << m.hop_data_attempts
                  << ", ACKs Sent: " << m.ack_transmission_attempts
                  << ", ACK Losses: " << m.ack_losses
                  << ", Retries: " << m.retransmissions_missing_ack
                  << ", Wire Bytes: " << m.total_wire_bytes_sent << " B" << std::endl;

        csv << sc.name << ","
            << sc.seed << ","
            << sc.routing_mode << ","
            << sc.ack_mode << ","
            << sc.rover_count << ","
            << sc.duration_ms << ","
            << sc.step_ms << ","
            << sc.latency_ms << ","
            << std::fixed << std::setprecision(2) << sc.packet_loss_rate << ","
            << std::setprecision(2) << sc.ack_loss_rate << ","
            << std::setprecision(1) << m.partition_duration_ms << ","
            << m.messages_generated << ","
            << m.messages_admitted << ","
            << m.unique_messages_processed << ","
            << std::setprecision(4) << m.end_to_end_completion_ratio << ","
            << m.hop_data_attempts << ","
            << m.hop_data_delivered << ","
            << m.hop_data_losses << ","
            << m.ack_transmission_attempts << ","
            << m.ack_deliveries << ","
            << m.ack_losses << ","
            << m.retransmissions_missing_ack << ","
            << m.duplicate_deliveries << ","
            << m.duplicate_processing_attempts << ","
            << m.messages_expired << ","
            << m.messages_dropped << ","
            << m.hop_limit_exhausted << ","
            << m.in_flight_messages << ","
            << m.intermediate_queued_messages << ","
            << m.queue_peak_messages << ","
            << m.payload_bytes_sent << ","
            << m.protocol_bytes_sent << ","
            << m.framing_bytes_sent << ","
            << m.ack_wire_bytes_sent << ","
            << m.ack_wire_bytes_delivered << ","
            << m.total_wire_bytes_sent << ","
            << m.total_wire_bytes_delivered << "\n";
    }

    csv.close();
    std::cout << "======================================================================" << std::endl;
    std::cout << " SUCCESS: Wrote " << scenarios.size() << " records to " << output_path << std::endl;
    std::cout << "======================================================================" << std::endl;

    return 0;
}
