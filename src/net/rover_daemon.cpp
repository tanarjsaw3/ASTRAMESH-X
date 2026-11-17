#include "astromesh/core/landmark_key.hpp"
#include "astromesh/geometry/se2.hpp"
#include "astromesh/net/distributed_reconciliation.hpp"
#include "astromesh/net/rover_node.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace astromesh::net;
using namespace astromesh::core;
using namespace astromesh::geometry;

static void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "Options:\n"
              << "  --rover-id <id>          Rover identifier (1..255, default: 1)\n"
              << "  --port <port>            UDP listening port (default: 19001)\n"
              << "  --peer <ip:port>         Seed peer endpoint (can be repeated)\n"
              << "  --duration-s <sec>       Process execution duration (default: 5.0, 0 = infinite)\n"
              << "  --heartbeat-ms <ms>      Heartbeat interval in ms (default: 100)\n"
              << "  --test-msg <text>        Broadcast test string payload during run\n"
              << "  --scenario-slam          Enable IBLT reconciliation & distributed topological SLAM\n"
              << "  --sync-target <id>       Initiate IBLT sync toward target rover ID\n"
              << "  --multihop-target <id>   Unicast multi-hop test-msg toward target rover ID\n"
              << "  --help                   Display this help message\n";
}

int main(int argc, char* argv[]) {
    RoverNodeConfig config;
    config.rover_id = 1;
    config.listen_port = 19001;
    config.heartbeat_interval_ms = 100;
    config.peer_timeout_ms = 800;

    double duration_s = 5.0;
    double send_delay_s = 0.5;
    std::string test_msg;
    bool scenario_slam = false;
    uint16_t sync_target = 0;
    uint16_t multihop_target = 0;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        } else if (arg == "--rover-id" && i + 1 < argc) {
            config.rover_id = static_cast<uint16_t>(std::stoul(argv[++i]));
        } else if (arg == "--port" && i + 1 < argc) {
            config.listen_port = static_cast<uint16_t>(std::stoul(argv[++i]));
        } else if (arg == "--peer" && i + 1 < argc) {
            std::string pstr = argv[++i];
            auto colon = pstr.find(':');
            if (colon != std::string::npos) {
                Endpoint ep;
                ep.ip = pstr.substr(0, colon);
                ep.port = static_cast<uint16_t>(std::stoul(pstr.substr(colon + 1)));
                config.seed_peers.push_back(ep);
            }
        } else if (arg == "--duration-s" && i + 1 < argc) {
            duration_s = std::stod(argv[++i]);
        } else if (arg == "--send-delay-s" && i + 1 < argc) {
            send_delay_s = std::stod(argv[++i]);
        } else if (arg == "--heartbeat-ms" && i + 1 < argc) {
            config.heartbeat_interval_ms = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--peer-timeout-ms" && i + 1 < argc) {
            config.peer_timeout_ms = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--test-msg" && i + 1 < argc) {
            test_msg = argv[++i];
        } else if (arg == "--scenario-slam") {
            scenario_slam = true;
        } else if (arg == "--sync-target" && i + 1 < argc) {
            sync_target = static_cast<uint16_t>(std::stoul(argv[++i]));
        } else if (arg == "--multihop-target" && i + 1 < argc) {
            multihop_target = static_cast<uint16_t>(std::stoul(argv[++i]));
        }
    }

    std::cout << "=== ASTRAMESH-X STANDALONE ROVER DAEMON ===\n"
              << "Rover ID:        " << config.rover_id << "\n"
              << "Listen Port:     " << config.listen_port << "\n"
              << "Seed Peers:      " << config.seed_peers.size() << "\n"
              << "Heartbeat:       " << config.heartbeat_interval_ms << " ms\n"
              << "Duration:        " << duration_s << " s\n"
              << "Scenario SLAM:   " << (scenario_slam ? "ENABLED" : "DISABLED") << "\n"
              << "-------------------------------------------\n";

    RoverNode node(config);
    if (!node.start()) {
        std::cerr << "Failed to start RoverNode on port " << config.listen_port << "\n";
        return 1;
    }

    std::cout << "[Rover " << config.rover_id << "] Node started successfully. Listening...\n";

    std::unique_ptr<DistributedReconciliationAgent> agent;
    if (scenario_slam) {
        agent = std::make_unique<DistributedReconciliationAgent>(config.rover_id, node, 32, 3);
        agent->setup_callbacks();

        // Populate local topological map and observations
        if (config.rover_id == 1) {
            std::vector<LandmarkObservationRecord> obs1 = {
                { astromesh::core::LandmarkKey(1, 1, 1, 0x11111111ULL),  5.0,  0.0, 0 },
                { astromesh::core::LandmarkKey(1, 1, 2, 0x22222222ULL),  6.0,  1.0, 0 },
                { astromesh::core::LandmarkKey(1, 1, 3, 0x33333333ULL),  2.0,  3.0, 0 },
                { astromesh::core::LandmarkKey(1, 1, 4, 0x44444444ULL),  4.0,  1.0, 0 },
                { astromesh::core::LandmarkKey(1, 1, 5, 0x55555555ULL),  3.0, -2.0, 0 }
            };
            agent->add_local_keyframe(0, astromesh::geometry::SE2(0.0, 0.0, 0.0), obs1);
        } else if (config.rover_id == 2) {
            std::vector<LandmarkObservationRecord> obs2 = {
                { astromesh::core::LandmarkKey(1, 1, 3, 0x33333333ULL), -8.0,  3.0, 0 },
                { astromesh::core::LandmarkKey(1, 1, 4, 0x44444444ULL), -6.0,  1.0, 0 },
                { astromesh::core::LandmarkKey(1, 1, 5, 0x55555555ULL), -7.0, -2.0, 0 },
                { astromesh::core::LandmarkKey(1, 2, 6, 0x66666666ULL),  1.0,  2.0, 0 },
                { astromesh::core::LandmarkKey(1, 2, 7, 0x77777777ULL),  2.0, -1.0, 0 }
            };
            agent->add_local_keyframe(0, astromesh::geometry::SE2(10.0, 0.0, 0.0), obs2);
        } else if (config.rover_id == 3) {
            std::vector<LandmarkObservationRecord> obs3 = {
                { astromesh::core::LandmarkKey(1, 1, 3, 0x33333333ULL), -8.0,  3.0, 0 },
                { astromesh::core::LandmarkKey(1, 1, 4, 0x44444444ULL), -6.0,  1.0, 0 },
                { astromesh::core::LandmarkKey(1, 1, 5, 0x55555555ULL), -7.0, -2.0, 0 },
                { astromesh::core::LandmarkKey(1, 3, 6, 0x66666666ULL),  1.0,  2.0, 0 },
                { astromesh::core::LandmarkKey(1, 3, 7, 0x77777777ULL),  2.0, -1.0, 0 }
            };
            agent->add_local_keyframe(0, astromesh::geometry::SE2(10.0, 0.0, 0.0), obs3);
        }
    } else {
        node.set_message_handler([&](const MessageFrame& frame, const Endpoint& sender) {
            std::cout << "[Rover " << config.rover_id << "] RECV " << to_string(frame.type)
                      << " from Rover " << frame.sender_rover_id
                      << " (" << sender.to_string() << "), seq=" << frame.sequence
                      << ", bytes=" << frame.payload.size() << "\n";
            if (frame.type == MessageType::DataPayload && !frame.payload.empty()) {
                std::string text(frame.payload.begin(), frame.payload.end());
                std::cout << "   -> Payload text: \"" << text << "\"\n";
            }
        });
    }

    const auto start_time = std::chrono::steady_clock::now();
    uint64_t last_msg_send = 0;
    bool sync_initiated = false;

    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - start_time).count();
        if (duration_s > 0 && elapsed >= duration_s) {
            break;
        }

        node.poll_events();

        if (scenario_slam && sync_target > 0 && !sync_initiated && elapsed >= send_delay_s) {
            std::cout << "[Rover " << config.rover_id << "] Initiating IBLT synchronization with Rover " << sync_target << "...\n";
            agent->initiate_sync_with_peer(sync_target);
            sync_initiated = true;
        }

        if (!test_msg.empty() && elapsed >= send_delay_s) {
            uint64_t now_ms = static_cast<uint64_t>(elapsed * 1000.0);
            if (now_ms >= last_msg_send + 500) {
                std::vector<uint8_t> payload(test_msg.begin(), test_msg.end());
                if (multihop_target > 0) {
                    node.send_message_to(multihop_target, MessageType::DataPayload, payload);
                } else {
                    node.broadcast_message(MessageType::DataPayload, payload);
                }
                last_msg_send = now_ms;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "\n=== ROVER " << config.rover_id << " SHUTDOWN TELEMETRY ===\n";
    auto stats = node.get_stats();
    std::cout << "Packets Sent:          " << stats.packets_sent << " (" << stats.bytes_sent << " B)\n"
              << "Packets Received:      " << stats.packets_received << " (" << stats.bytes_received << " B)\n"
              << "Heartbeats Sent:       " << stats.heartbeats_sent << "\n"
              << "Heartbeats Received:   " << stats.heartbeats_received << "\n"
              << "Duplicates Dropped:    " << stats.duplicates_dropped << "\n"
              << "Corrupted Dropped:     " << stats.corrupted_dropped << "\n"
              << "Active Peers Seen:     " << node.get_peer_tracker().total_peers() << "\n";

    auto mh_stats = node.get_multihop_router().get_stats();
    std::cout << "--- MULTI-HOP MESH STATS ---\n"
              << "Packets Forwarded:     " << mh_stats.packets_forwarded << "\n"
              << "Routing Loops Supp:    " << mh_stats.routing_loops_suppressed << "\n"
              << "Transit Buffered:      " << mh_stats.transit_buffered << "\n";

    std::cout << "--- ROUTING TABLE ---\n";
    for (const auto& entry : node.get_routing_table().get_all_routes()) {
        std::cout << "Route Dest " << entry.destination_rover_id
                  << " -> NextHop " << entry.next_hop_rover_id
                  << " metric=" << static_cast<int>(entry.metric)
                  << " valid=" << (entry.is_valid() ? "YES" : "NO") << "\n";
    }

    if (scenario_slam && agent) {
        auto r_stats = agent->get_stats();
        std::cout << "--- SLAM RECONCILIATION SUMMARY ---\n"
                  << "Sync Packets Sent:     " << r_stats.sync_sent << "\n"
                  << "Sync Packets Received: " << r_stats.sync_received << "\n"
                  << "Decodes Succeeded:     " << r_stats.sync_decodes_succeeded << "\n"
                  << "Proposals Sent:        " << r_stats.proposals_sent << "\n"
                  << "Proposals Received:    " << r_stats.proposals_received << "\n"
                  << "Loop Closures Verified:" << " " << r_stats.loop_closures_verified << "\n"
                  << "DSU Unions Performed:  " << r_stats.dsu_unions_performed << "\n"
                  << "DSU Components:        " << agent->get_dsu_component_count() << "\n"
                  << "Boundary States Sent:  " << r_stats.boundary_states_sent << "\n"
                  << "Boundary States Recv:  " << r_stats.boundary_states_received << "\n";
    }
    std::cout << "===============================================\n";

    node.stop();
    return 0;
}
