#include "astromesh/net/multihop_router.hpp"

#include <cassert>
#include <iostream>
#include <vector>

using namespace astromesh::net;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "Assertion failed: " << msg << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
            std::exit(1); \
        } \
    } while (0)

// 1. Direct Local Delivery
static void test_direct_local_delivery() {
    std::cout << "[TEST 1] Direct Local Delivery..." << std::endl;

    RoutingTable table(1);
    MultihopRouter router(1, table);

    std::vector<MultihopEnvelope> delivered;
    router.set_local_delivery_callback([&](const MultihopEnvelope& env) {
        delivered.push_back(env);
    });

    bool ok = router.originate_packet(1, 101, {'L', 'O', 'C', 'A', 'L'});
    TEST_ASSERT(ok, "Local originate should succeed");
    TEST_ASSERT(delivered.size() == 1, "Delivered locally");
    TEST_ASSERT(delivered[0].message_id == 101, "Message ID match");
    TEST_ASSERT(router.stats().packets_delivered_locally == 1, "Stats local count");

    std::cout << "  -> PASSED" << std::endl;
}

// 2. Two-Hop Route Relay (R1 -> R2 -> R3)
static void test_two_hop_route_relay() {
    std::cout << "[TEST 2] Two-Hop Route Relay (R1 -> R2 -> R3)..." << std::endl;

    RoutingTable table_r1(1);
    RoutingTable table_r2(2);
    RoutingTable table_r3(3);

    // Setup routes:
    // R2: direct route to 3 via next_hop=3, metric=1
    table_r2.install_direct_route(3, 1000);
    // R1: route to 3 via next_hop=2, metric=2 (R2 reports metric 1 to R1)
    table_r1.update_route_from_advertisement(2, 3, 1, 10, 1000);

    MultihopRouter r1(1, table_r1);
    MultihopRouter r2(2, table_r2);
    MultihopRouter r3(3, table_r3);

    std::vector<MultihopEnvelope> r3_received;
    r3.set_local_delivery_callback([&](const MultihopEnvelope& env) {
        r3_received.push_back(env);
    });

    // Wire linkage callbacks:
    // R1 forward sends to R2
    r1.set_forward_callback([&](const MultihopEnvelope& env, uint16_t next_hop) {
        TEST_ASSERT(next_hop == 2, "R1 must forward to next_hop 2");
        return r2.process_incoming_envelope(env, 1, 1000);
    });

    // R2 forward sends to R3
    r2.set_forward_callback([&](const MultihopEnvelope& env, uint16_t next_hop) {
        TEST_ASSERT(next_hop == 3, "R2 must forward to next_hop 3");
        return r3.process_incoming_envelope(env, 2, 1000);
    });

    bool sent = r1.originate_packet(3, 5001, {'D', 'A', 'T', 'A'});
    TEST_ASSERT(sent, "Originate must succeed");

    // Verify delivery at R3
    TEST_ASSERT(r3_received.size() == 1, "R3 must receive message");
    const auto& res = r3_received[0];
    TEST_ASSERT(res.origin_rover_id == 1, "Origin is R1");
    TEST_ASSERT(res.final_target_id == 3, "Target is R3");
    TEST_ASSERT(res.message_id == 5001, "MessageID preserved");
    TEST_ASSERT(res.hop_count == 1, "Relay R2 incremented hop_count from 0 to 1");
    TEST_ASSERT(res.path_length == 2, "Path contains R1, R2");
    TEST_ASSERT(res.visited_nodes[0] == 1 && res.visited_nodes[1] == 2, "Visited trail: 1 -> 2");

    // Verify stats
    TEST_ASSERT(r1.stats().packets_forwarded == 1, "R1 forwarded 1 packet");
    TEST_ASSERT(r2.stats().packets_forwarded == 1, "R2 forwarded 1 packet");
    TEST_ASSERT(r3.stats().packets_delivered_locally == 1, "R3 delivered locally");

    std::cout << "  -> PASSED" << std::endl;
}

// 3. Five-Hop Route (R1 -> R2 -> R3 -> R4 -> R5 -> R6)
static void test_five_hop_route() {
    std::cout << "[TEST 3] Five-Hop Route Relay (R1 -> ... -> R6)..." << std::endl;

    std::vector<RoutingTable> tables;
    std::vector<MultihopRouter> routers;
    for (uint16_t i = 1; i <= 6; ++i) {
        tables.emplace_back(i);
    }
    for (uint16_t i = 1; i <= 6; ++i) {
        routers.emplace_back(i, tables[i - 1]);
    }

    // Configure linear routes: to rover 6 via next rover (i + 1)
    tables[4].install_direct_route(6, 1000);
    for (int i = 3; i >= 0; --i) {
        uint8_t reported_metric = 5 - (i + 1);
        tables[i].update_route_from_advertisement(i + 2, 6, reported_metric, 100, 1000);
    }

    std::vector<MultihopEnvelope> r6_received;
    routers[5].set_local_delivery_callback([&](const MultihopEnvelope& env) {
        r6_received.push_back(env);
    });

    // Link forward callbacks
    for (size_t i = 0; i < 5; ++i) {
        routers[i].set_forward_callback([&routers, i](const MultihopEnvelope& env, uint16_t next_hop) {
            TEST_ASSERT(next_hop == i + 2, "Must route to next linear neighbor");
            return routers[i + 1].process_incoming_envelope(env, i + 1, 1000);
        });
    }

    bool sent = routers[0].originate_packet(6, 7777, {'F', 'I', 'V', 'E'}, MultihopFlags::NONE, 6);
    TEST_ASSERT(sent, "Originate should succeed");
    TEST_ASSERT(r6_received.size() == 1, "R6 must receive message");

    const auto& res = r6_received[0];
    TEST_ASSERT(res.origin_rover_id == 1, "Origin R1");
    TEST_ASSERT(res.final_target_id == 6, "Target R6");
    TEST_ASSERT(res.hop_count == 4, "Hop count 4 (relays R2, R3, R4, R5)");
    TEST_ASSERT(res.path_length == 5, "Visited R1, R2, R3, R4, R5");
    for (uint16_t i = 0; i < 5; ++i) {
        TEST_ASSERT(res.visited_nodes[i] == i + 1, "Visited path sequence match");
    }

    std::cout << "  -> PASSED" << std::endl;
}

// 4. Maximum Supported Hop Route (8 Hops)
static void test_maximum_hop_route() {
    std::cout << "[TEST 4] Maximum Hop Route (8 Hops)..." << std::endl;

    std::vector<RoutingTable> tables;
    std::vector<MultihopRouter> routers;
    for (uint16_t i = 1; i <= 9; ++i) {
        tables.emplace_back(i);
    }
    for (uint16_t i = 1; i <= 9; ++i) {
        routers.emplace_back(i, tables[i - 1]);
    }

    // Set routes to rover 9 via next linear neighbor
    tables[7].install_direct_route(9, 1000);
    for (int i = 6; i >= 0; --i) {
        uint8_t reported_metric = 8 - (i + 1);
        tables[i].update_route_from_advertisement(i + 2, 9, reported_metric, 100, 1000);
    }

    std::vector<MultihopEnvelope> r9_received;
    routers[8].set_local_delivery_callback([&](const MultihopEnvelope& env) {
        r9_received.push_back(env);
    });

    for (size_t i = 0; i < 8; ++i) {
        routers[i].set_forward_callback([&routers, i](const MultihopEnvelope& env, uint16_t next_hop) {
            return routers[i + 1].process_incoming_envelope(env, i + 1, 1000);
        });
    }

    bool sent = routers[0].originate_packet(9, 8888, {'M', 'A', 'X'}, MultihopFlags::NONE, 8);
    TEST_ASSERT(sent, "Originate max hops");
    TEST_ASSERT(r9_received.size() == 1, "R9 must receive 8-hop packet");
    TEST_ASSERT(r9_received[0].hop_count == 7, "Traversed 7 relays before terminal R9");
    TEST_ASSERT(r9_received[0].path_length == 8, "Path has exactly 8 nodes [1..8]");

    std::cout << "  -> PASSED" << std::endl;
}

// 5. TTL Expiration at Relay
static void test_ttl_expiration_at_relay() {
    std::cout << "[TEST 5] TTL Expiration at Relay..." << std::endl;

    RoutingTable table_r1(1);
    RoutingTable table_r2(2);
    RoutingTable table_r3(3);

    table_r2.install_direct_route(3, 1000);
    table_r1.update_route_from_advertisement(2, 3, 1, 10, 1000);

    MultihopRouter r1(1, table_r1);
    MultihopRouter r2(2, table_r2);
    MultihopRouter r3(3, table_r3);

    r1.set_forward_callback([&](const MultihopEnvelope& env, uint16_t) {
        return r2.process_incoming_envelope(env, 1, 1000);
    });

    bool r3_got_packet = false;
    r3.set_local_delivery_callback([&](const MultihopEnvelope&) {
        r3_got_packet = true;
    });
    r2.set_forward_callback([&](const MultihopEnvelope& env, uint16_t) {
        return r3.process_incoming_envelope(env, 2, 1000);
    });

    // Originate with max_hops = 1. R1 sends with hop_count=0.
    // When R2 receives it, R2 checks if hop_count(0) >= max_hops(1) -> not expired.
    // Then R2 increments hop_count to 1.
    // If R2 was asked to forward with max_hops=1, now hop_count is 1!
    // But what if max_hops = 1 was intended to only reach R2?
    // Let's test packet with hop_count = 1 and max_hops = 1 arriving at R2!
    MultihopEnvelope env;
    env.origin_rover_id = 1;
    env.final_target_id = 3;
    env.hop_count = 1; // Already 1 hop taken
    env.max_hops = 1;  // Max hops is 1
    env.message_id = 999;
    env.path_length = 2;
    env.visited_nodes = {1, 9, 0, 0, 0, 0, 0, 0};

    bool processed = r2.process_incoming_envelope(env, 9, 1000);
    TEST_ASSERT(!processed, "R2 must drop TTL-expired packet");
    TEST_ASSERT(r2.stats().ttl_expired_dropped == 1, "R2 incremented ttl_expired_dropped");
    TEST_ASSERT(!r3_got_packet, "R3 must not receive expired packet");

    std::cout << "  -> PASSED" << std::endl;
}

// 6. Cyclic Route & Loop Suppression
static void test_cyclic_route_loop_suppression() {
    std::cout << "[TEST 6] Cyclic Route & Loop Suppression..." << std::endl;

    RoutingTable table_r1(1);
    MultihopRouter r1(1, table_r1);

    // Create a packet that already has Rover 1 in its visited path: [2, 1, 3]
    MultihopEnvelope env;
    env.origin_rover_id = 2;
    env.final_target_id = 4;
    env.hop_count = 2;
    env.max_hops = 5;
    env.message_id = 1234;
    env.path_length = 3;
    env.visited_nodes = {2, 1, 3, 0, 0, 0, 0, 0};

    bool processed = r1.process_incoming_envelope(env, 3, 1000);
    TEST_ASSERT(!processed, "R1 must drop packet with loop detected");
    TEST_ASSERT(r1.stats().routing_loops_suppressed == 1, "R1 incremented routing_loops_suppressed");

    std::cout << "  -> PASSED" << std::endl;
}

// 7. Unknown Destination & Transit Buffering
static void test_unknown_destination_and_transit_buffering() {
    std::cout << "[TEST 7] Unknown Destination & Transit Buffering..." << std::endl;

    RoutingTable table_r1(1); // No route to 5 initially
    MultihopRouter r1(1, table_r1);

    std::vector<uint16_t> forwarded_to;
    r1.set_forward_callback([&](const MultihopEnvelope&, uint16_t next_hop) {
        forwarded_to.push_back(next_hop);
        return true;
    });

    // Originate packet to 5 (no route known yet)
    bool ok = r1.originate_packet(5, 5555, {'W', 'A', 'I', 'T'}, MultihopFlags::NONE, 5, 1000);
    TEST_ASSERT(ok, "Packet should be stored in transit buffer");
    TEST_ASSERT(r1.transit_buffer_size() == 1, "Transit buffer holds 1 packet");
    TEST_ASSERT(r1.stats().transit_buffered == 1, "Stats transit_buffered count");
    TEST_ASSERT(forwarded_to.empty(), "Nothing forwarded yet");

    // Now route to 5 is discovered!
    table_r1.update_route_from_advertisement(2, 5, 1, 1, 2000);

    // Drain transit buffer
    size_t drained = r1.drain_transit_buffer(2000);
    TEST_ASSERT(drained == 1, "Drained 1 packet");
    TEST_ASSERT(r1.transit_buffer_size() == 0, "Transit buffer empty");
    TEST_ASSERT(forwarded_to.size() == 1 && forwarded_to[0] == 2, "Forwarded to next hop 2");
    TEST_ASSERT(r1.stats().transit_drained == 1, "Stats transit_drained count");

    std::cout << "  -> PASSED" << std::endl;
}

// 8. Custody ACK Generation
static void test_custody_ack_generation() {
    std::cout << "[TEST 8] Custody ACK Generation..." << std::endl;

    RoutingTable table_r2(2);
    table_r2.install_direct_route(3, 1000);
    MultihopRouter r2(2, table_r2);

    r2.set_forward_callback([](const MultihopEnvelope&, uint16_t) { return true; });

    uint16_t ack_upstream = 0;
    uint64_t ack_msg_id = 0;
    r2.set_custody_ack_callback([&](uint16_t upstream, uint64_t msg_id) {
        ack_upstream = upstream;
        ack_msg_id = msg_id;
    });

    MultihopEnvelope env;
    env.origin_rover_id = 1;
    env.final_target_id = 3;
    env.hop_count = 0;
    env.max_hops = 4;
    env.message_id = 0xABCDEF;
    env.flags = MultihopFlags::CUSTODY_ACK_REQUESTED;
    env.path_length = 1;
    env.visited_nodes = {1, 0, 0, 0, 0, 0, 0, 0};

    bool processed = r2.process_incoming_envelope(env, 1, 1000);
    TEST_ASSERT(processed, "R2 must accept and forward");
    TEST_ASSERT(ack_upstream == 1, "Custody ACK addressed upstream to R1");
    TEST_ASSERT(ack_msg_id == 0xABCDEF, "Custody ACK matches MessageID");

    std::cout << "  -> PASSED" << std::endl;
}

// 9. Arbitrary Rover IDs
static void test_arbitrary_rover_ids() {
    std::cout << "[TEST 9] Arbitrary Rover IDs..." << std::endl;

    RoutingTable table_1337(1337);
    table_1337.install_direct_route(42000, 1000);
    MultihopRouter router(1337, table_1337);

    uint16_t delivered_target = 0;
    router.set_forward_callback([&](const MultihopEnvelope& env, uint16_t next_hop) {
        delivered_target = next_hop;
        return true;
    });

    bool sent = router.originate_packet(42000, 99999, {'X'});
    TEST_ASSERT(sent, "Originate arbitrary ID");
    TEST_ASSERT(delivered_target == 42000, "Forwarded to 42000");

    std::cout << "  -> PASSED" << std::endl;
}

int main() {
    std::cout << "=================================================================" << std::endl;
    std::cout << "  ASTRAMESH-X M7.11 MULTIHOP ROUTER UNIT TEST SUITE              " << std::endl;
    std::cout << "=================================================================" << std::endl;

    test_direct_local_delivery();
    test_two_hop_route_relay();
    test_five_hop_route();
    test_maximum_hop_route();
    test_ttl_expiration_at_relay();
    test_cyclic_route_loop_suppression();
    test_unknown_destination_and_transit_buffering();
    test_custody_ack_generation();
    test_arbitrary_rover_ids();

    std::cout << "=================================================================" << std::endl;
    std::cout << "  ALL 9 MULTIHOP ROUTER TESTS PASSED (100%)                      " << std::endl;
    std::cout << "=================================================================" << std::endl;
    return 0;
}
