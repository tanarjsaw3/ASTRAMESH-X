#include "astromesh/net/distributed_reconciliation.hpp"
#include "astromesh/net/rover_node.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/graph/disjoint_set.hpp"
#include "astromesh/geometry/se2.hpp"

#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

using namespace astromesh;
using namespace astromesh::net;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "Assertion failed: " << msg << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
            std::exit(1); \
        } \
    } while (0)

// 1. Two-Hop UDP Route Discovery (R1 <-> R2 <-> R3)
static void test_two_hop_route_discovery() {
    std::cout << "[TEST 1] Two-Hop UDP Route Discovery (R1 <-> R2 <-> R3)..." << std::endl;

    RoverNodeConfig c1;
    c1.rover_id = 1;
    c1.listen_port = 21101;
    c1.listen_ip = "127.0.0.1";
    c1.seed_peers = { Endpoint("127.0.0.1", 21102) }; // Only knows R2
    c1.heartbeat_interval_ms = 50;
    c1.routing_advertisement_interval_ms = 50;

    RoverNodeConfig c2;
    c2.rover_id = 2;
    c2.listen_port = 21102;
    c2.listen_ip = "127.0.0.1";
    c2.seed_peers = { Endpoint("127.0.0.1", 21101), Endpoint("127.0.0.1", 21103) }; // Knows R1 and R3
    c2.heartbeat_interval_ms = 50;
    c2.routing_advertisement_interval_ms = 50;

    RoverNodeConfig c3;
    c3.rover_id = 3;
    c3.listen_port = 21103;
    c3.listen_ip = "127.0.0.1";
    c3.seed_peers = { Endpoint("127.0.0.1", 21102) }; // Only knows R2
    c3.heartbeat_interval_ms = 50;
    c3.routing_advertisement_interval_ms = 50;

    RoverNode r1(c1);
    RoverNode r2(c2);
    RoverNode r3(c3);

    // Strictly sever direct link between R1 and R3
    r1.get_channel_impairment().set_link_state(1, 3, false);
    r3.get_channel_impairment().set_link_state(3, 1, false);

    TEST_ASSERT(r1.start(), "R1 start");
    TEST_ASSERT(r2.start(), "R2 start");
    TEST_ASSERT(r3.start(), "R3 start");

    // Exchange heartbeats and advertisements for 400 ms
    for (int step = 0; step < 40; ++step) {
        r1.poll_events(0);
        r2.poll_events(0);
        r3.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Verify distance-vector convergence:
    // R1 must have route to R3 via next_hop = 2
    auto r1_to_r3 = r1.get_routing_table().lookup_next_hop(3, 0);
    TEST_ASSERT(r1_to_r3.has_value() && *r1_to_r3 == 2, "R1 must route to R3 via next_hop 2");

    // R3 must have route to R1 via next_hop = 2
    auto r3_to_r1 = r3.get_routing_table().lookup_next_hop(1, 0);
    TEST_ASSERT(r3_to_r1.has_value() && *r3_to_r1 == 2, "R3 must route to R1 via next_hop 2");

    // R2 must have direct routes to both R1 and R3
    auto r2_to_r1 = r2.get_routing_table().lookup_next_hop(1, 0);
    auto r2_to_r3 = r2.get_routing_table().lookup_next_hop(3, 0);
    TEST_ASSERT(r2_to_r1.has_value() && *r2_to_r1 == 1, "R2 direct to R1");
    TEST_ASSERT(r2_to_r3.has_value() && *r2_to_r3 == 3, "R2 direct to R3");

    r1.stop();
    r2.stop();
    r3.stop();
    std::cout << "  -> PASSED" << std::endl;
}

// 2. Two-Hop Application Message Delivery through Relay
static void test_two_hop_application_delivery() {
    std::cout << "[TEST 2] Two-Hop Application Message Delivery through Relay..." << std::endl;

    RoverNodeConfig c1;
    c1.rover_id = 1;
    c1.listen_port = 21111;
    c1.listen_ip = "127.0.0.1";
    c1.seed_peers = { Endpoint("127.0.0.1", 21112) };
    c1.heartbeat_interval_ms = 40;
    c1.routing_advertisement_interval_ms = 40;

    RoverNodeConfig c2;
    c2.rover_id = 2;
    c2.listen_port = 21112;
    c2.listen_ip = "127.0.0.1";
    c2.seed_peers = { Endpoint("127.0.0.1", 21111), Endpoint("127.0.0.1", 21113) };
    c2.heartbeat_interval_ms = 40;
    c2.routing_advertisement_interval_ms = 40;

    RoverNodeConfig c3;
    c3.rover_id = 3;
    c3.listen_port = 21113;
    c3.listen_ip = "127.0.0.1";
    c3.seed_peers = { Endpoint("127.0.0.1", 21112) };
    c3.heartbeat_interval_ms = 40;
    c3.routing_advertisement_interval_ms = 40;

    RoverNode r1(c1);
    RoverNode r2(c2);
    RoverNode r3(c3);

    r1.get_channel_impairment().set_link_state(1, 3, false);
    r3.get_channel_impairment().set_link_state(3, 1, false);

    TEST_ASSERT(r1.start(), "R1 start");
    TEST_ASSERT(r2.start(), "R2 start");
    TEST_ASSERT(r3.start(), "R3 start");

    std::vector<MessageFrame> r3_received;
    r3.set_message_handler([&](const MessageFrame& frame, const Endpoint&) {
        r3_received.push_back(frame);
    });

    std::vector<MessageFrame> r2_received;
    r2.set_message_handler([&](const MessageFrame& frame, const Endpoint&) {
        r2_received.push_back(frame);
    });

    // Exchange discovery frames for 300 ms
    for (int i = 0; i < 30; ++i) {
        r1.poll_events(0);
        r2.poll_events(0);
        r3.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // R1 originates multi-hop message to R3
    const std::string payload_str = "MULTIHOP_PAYLOAD_LUNAR_MAP_2026";
    std::vector<uint8_t> payload_bytes(payload_str.begin(), payload_str.end());

    int64_t sent = r1.send_message_to(3, MessageType::DataPayload, payload_bytes);
    TEST_ASSERT(sent > 0, "R1 must originate multihop message to R3");

    // Poll to relay and deliver
    for (int i = 0; i < 40; ++i) {
        r1.poll_events(0);
        r2.poll_events(0);
        r3.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (!r3_received.empty()) break;
    }

    // Verify R3 received the message
    TEST_ASSERT(r3_received.size() == 1, "R3 must receive 1 message");
    const auto& msg = r3_received[0];
    TEST_ASSERT(msg.sender_rover_id == 1, "Original sender is Rover 1");
    TEST_ASSERT(msg.target_rover_id == 3, "Final target is Rover 3");
    std::string rec_str(msg.payload.begin(), msg.payload.end());
    TEST_ASSERT(rec_str == payload_str, "Payload content preserved exactly");

    // Crucial check: Relay R2 must NOT deliver transit payload to its own application handler!
    TEST_ASSERT(r2_received.empty(), "Relay R2 must NOT deliver transit payload locally");

    r1.stop();
    r2.stop();
    r3.stop();
    std::cout << "  -> PASSED" << std::endl;
}

// 3. Multi-Hop Distributed SLAM Reconciliation (IBLT -> Proposal -> RANSAC -> Graph Merge)
static void test_multihop_distributed_slam_reconciliation() {
    std::cout << "[TEST 3] Multi-Hop Distributed SLAM Reconciliation..." << std::endl;

    RoverNodeConfig c1;
    c1.rover_id = 1;
    c1.listen_port = 21121;
    c1.listen_ip = "127.0.0.1";
    c1.seed_peers = { Endpoint("127.0.0.1", 21122) };
    c1.heartbeat_interval_ms = 40;
    c1.routing_advertisement_interval_ms = 40;

    RoverNodeConfig c2;
    c2.rover_id = 2;
    c2.listen_port = 21122;
    c2.listen_ip = "127.0.0.1";
    c2.seed_peers = { Endpoint("127.0.0.1", 21121), Endpoint("127.0.0.1", 21123) };
    c2.heartbeat_interval_ms = 40;
    c2.routing_advertisement_interval_ms = 40;

    RoverNodeConfig c3;
    c3.rover_id = 3;
    c3.listen_port = 21123;
    c3.listen_ip = "127.0.0.1";
    c3.seed_peers = { Endpoint("127.0.0.1", 21122) };
    c3.heartbeat_interval_ms = 40;
    c3.routing_advertisement_interval_ms = 40;

    RoverNode r1(c1);
    RoverNode r2(c2);
    RoverNode r3(c3);

    // Impairment: R1 and R3 can only communicate via R2
    r1.get_channel_impairment().set_link_state(1, 3, false);
    r3.get_channel_impairment().set_link_state(3, 1, false);

    TEST_ASSERT(r1.start(), "R1 start");
    TEST_ASSERT(r2.start(), "R2 start");
    TEST_ASSERT(r3.start(), "R3 start");

    // Initialize reconciliation agents
    DistributedReconciliationAgent agent1(1, r1);
    DistributedReconciliationAgent agent2(2, r2);
    DistributedReconciliationAgent agent3(3, r3);

    agent1.setup_callbacks();
    agent2.setup_callbacks();
    agent3.setup_callbacks();

    // Populate landmarks: R1 and R3 share 5 common landmarks with a known transform:
    // Relative transform from R1 (0,0) to R3 (10,0) is dx = 10.0, dy = 0.0, dtheta = 0.0.
    // In R1 frame: (x, y); In R3 frame: (x - 10, y).
    std::vector<LandmarkObservationRecord> obs1;
    std::vector<LandmarkObservationRecord> obs3;
    for (uint32_t lm = 1; lm <= 5; ++lm) {
        core::LandmarkKey k(1, 1, lm, 0x1000 + lm);
        double r1_x = 10.0 + lm * 2.0;
        double r1_y = 5.0 + lm * 1.5;

        obs1.push_back({ k, r1_x, r1_y, 0, true });
        obs3.push_back({ k, r1_x - 10.0, r1_y, 0, true });
    }

    agent1.add_local_keyframe(0, geometry::SE2(0.0, 0.0, 0.0), obs1);
    agent2.add_local_keyframe(0, geometry::SE2(5.0, 0.0, 0.0), {});
    agent3.add_local_keyframe(0, geometry::SE2(10.0, 0.0, 0.0), obs3);

    // Exchange discovery and routing advertisements
    for (int i = 0; i < 35; ++i) {
        r1.poll_events(0);
        r2.poll_events(0);
        r3.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Verify distance-vector routes converged
    TEST_ASSERT(r1.get_routing_table().lookup_next_hop(3, 0).has_value(), "R1 knows route to R3");
    TEST_ASSERT(r3.get_routing_table().lookup_next_hop(1, 0).has_value(), "R3 knows route to R1");

    // Initiate multi-hop IBLT sync from R1 to R3 across R2
    uint64_t sync_id = agent1.initiate_sync_with_peer(3, 0);
    TEST_ASSERT(sync_id > 0, "R1 initiated sync with R3");

    // Poll multi-hop event loop: R1 -> R2 -> R3 (sync), R3 -> R2 -> R1 (proposal), R1 -> R2 -> R3 (response)
    for (int step = 0; step < 80; ++step) {
        r1.poll_events(0);
        r2.poll_events(0);
        r3.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Check if loop closure verified on both R1 and R3
        if (agent1.get_stats().loop_closures_verified >= 1 &&
            agent3.get_stats().loop_closures_verified >= 1) {
            break;
        }
    }

    // Verify multi-hop SLAM success:
    std::cout << "  Agent1 sync_sent: " << agent1.get_stats().sync_sent << "\n"
              << "  Agent3 sync_received: " << agent3.get_stats().sync_received << "\n"
              << "  Agent3 proposals_sent: " << agent3.get_stats().proposals_sent << "\n"
              << "  Agent1 proposals_recv: " << agent1.get_stats().proposals_received << "\n"
              << "  Agent1 verified closures: " << agent1.get_stats().loop_closures_verified << std::endl;
    std::cout << "  Agent3 verified closures: " << agent3.get_stats().loop_closures_verified << std::endl;
    std::cout << "  Agent2 verified closures: " << agent2.get_stats().loop_closures_verified << std::endl;

    TEST_ASSERT(agent1.get_stats().loop_closures_verified >= 1, "Agent 1 must verify loop closure");
    TEST_ASSERT(agent3.get_stats().loop_closures_verified >= 1, "Agent 3 must verify loop closure");
    TEST_ASSERT(agent2.get_stats().loop_closures_verified == 0, "Relay Agent 2 must NOT falsely claim loop closure");

    // Check graph edges added on R1 and R3
    TEST_ASSERT(agent1.get_pose_graph().edges().size() >= 1, "PoseGraph 1 has verified inter-rover edge");
    TEST_ASSERT(agent3.get_pose_graph().edges().size() >= 1, "PoseGraph 3 has verified inter-rover edge");

    // Check DSU component unification
    TEST_ASSERT(agent1.get_partition_manager().are_connected(graph::NodeId(1, 0), graph::NodeId(3, 0)),
                "DSU 1 unified R1 and R3 components");

    r1.stop();
    r2.stop();
    r3.stop();
    std::cout << "  -> PASSED" << std::endl;
}

// 4. Broken Link Transit Buffering & Route Recovery
static void test_broken_link_transit_buffering_and_recovery() {
    std::cout << "[TEST 4] Broken Link Transit Buffering & Route Recovery..." << std::endl;

    RoverNodeConfig c1;
    c1.rover_id = 1;
    c1.listen_port = 21131;
    c1.listen_ip = "127.0.0.1";
    c1.seed_peers = { Endpoint("127.0.0.1", 21132) };
    c1.heartbeat_interval_ms = 40;
    c1.routing_advertisement_interval_ms = 40;

    RoverNodeConfig c2;
    c2.rover_id = 2;
    c2.listen_port = 21132;
    c2.listen_ip = "127.0.0.1";
    c2.seed_peers = { Endpoint("127.0.0.1", 21131), Endpoint("127.0.0.1", 21133) };
    c2.heartbeat_interval_ms = 40;
    c2.routing_advertisement_interval_ms = 40;

    RoverNodeConfig c3;
    c3.rover_id = 3;
    c3.listen_port = 21133;
    c3.listen_ip = "127.0.0.1";
    c3.seed_peers = { Endpoint("127.0.0.1", 21132) };
    c3.heartbeat_interval_ms = 40;
    c3.routing_advertisement_interval_ms = 40;

    RoverNode r1(c1);
    RoverNode r2(c2);
    RoverNode r3(c3);

    // No direct R1 <-> R3 link
    r1.get_channel_impairment().set_link_state(1, 3, false);
    r3.get_channel_impairment().set_link_state(3, 1, false);

    TEST_ASSERT(r1.start(), "R1 start");
    TEST_ASSERT(r2.start(), "R2 start");
    TEST_ASSERT(r3.start(), "R3 start");

    std::vector<std::string> r3_messages;
    r3.set_message_handler([&](const MessageFrame& frame, const Endpoint&) {
        r3_messages.push_back(std::string(frame.payload.begin(), frame.payload.end()));
    });

    // Establish initial convergence
    for (int i = 0; i < 30; ++i) {
        r1.poll_events(0);
        r2.poll_events(0);
        r3.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 1. Induce failure on link between R2 and R3!
    r2.get_channel_impairment().set_link_state(2, 3, false);
    r3.get_channel_impairment().set_link_state(3, 2, false);

    // Invalidate neighbor route on R2 to simulate link failure detection
    r2.get_routing_table().invalidate_neighbor_routes(3, 0);

    // 2. R1 sends message to R3 while R2-R3 link is broken!
    r1.send_message_to(3, MessageType::DataPayload, {'D', 'T', 'N', '_', 'Q', 'U', 'E', 'U', 'E'});

    for (int i = 0; i < 20; ++i) {
        r1.poll_events(0);
        r2.poll_events(0);
        r3.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // R3 has not received anything yet
    TEST_ASSERT(r3_messages.empty(), "R3 must not receive message during link break");

    // R2 has buffered transit packet
    std::cout << "  R2 transit buffer size during break: " << r2.get_multihop_router().transit_buffer_size() << std::endl;
    TEST_ASSERT(r2.get_multihop_router().transit_buffer_size() >= 1 ||
                r1.get_dtn_buffer().message_count() >= 1, "Message buffered during link failure");

    // 3. Restore R2-R3 link!
    r2.get_channel_impairment().set_link_state(2, 3, true);
    r3.get_channel_impairment().set_link_state(3, 2, true);
    r2.get_routing_table().install_direct_route(3, 0);

    // Poll to drain buffered traffic
    for (int i = 0; i < 40; ++i) {
        r1.poll_events(0);
        r2.poll_events(0);
        r3.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (!r3_messages.empty()) break;
    }

    // Verify recovery and delivery!
    TEST_ASSERT(!r3_messages.empty(), "R3 must receive buffered message after link restoration");
    TEST_ASSERT(r3_messages[0] == "DTN_QUEUE", "Buffered message content matches");

    r1.stop();
    r2.stop();
    r3.stop();
    std::cout << "  -> PASSED" << std::endl;
}

int main() {
    std::cout << "=================================================================" << std::endl;
    std::cout << "  ASTRAMESH-X M7.11 MULTI-HOP SLAM INTEGRATION TEST SUITE        " << std::endl;
    std::cout << "=================================================================" << std::endl;

    test_two_hop_route_discovery();
    test_two_hop_application_delivery();
    test_multihop_distributed_slam_reconciliation();
    test_broken_link_transit_buffering_and_recovery();

    std::cout << "=================================================================" << std::endl;
    std::cout << "  ALL 4 MULTI-HOP SLAM INTEGRATION TESTS PASSED (100%)           " << std::endl;
    std::cout << "=================================================================" << std::endl;
    return 0;
}

