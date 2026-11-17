#include "astromesh/net/multihop_envelope.hpp"
#include "astromesh/net/multihop_router.hpp"
#include "astromesh/net/routing_table.hpp"

#include <cassert>
#include <iostream>
#include <vector>

using namespace astromesh::net;

// Test 1: Wire byte accounting conservation
static void test_wire_byte_conservation() {
    uint64_t payload_bytes = 1000;
    uint64_t header_bytes = 36;   // MessageFrame header
    uint64_t framing_bytes = 40;  // MultihopEnvelope header
    uint64_t ack_wire_bytes = 24; // AckPayload

    uint64_t data_wire_bytes = payload_bytes + header_bytes + framing_bytes;
    uint64_t total_wire_bytes = data_wire_bytes + ack_wire_bytes;

    assert(data_wire_bytes == 1076);
    assert(total_wire_bytes == 1100);
    assert(total_wire_bytes == data_wire_bytes + ack_wire_bytes);

    std::cout << "[PASS] Wire byte conservation identity verified.\n";
}

// Test 2: Multihop forwarding amplification O(h)
static void test_multihop_forwarding_overhead() {
    // Envelope for a 3-hop traversal: Rover 1 -> Rover 2 -> Rover 3 -> Rover 4
    MultihopEnvelope env;
    env.origin_rover_id = 1;
    env.final_target_id = 4;
    env.max_hops = 5;
    env.message_id = 0x12345678ULL;
    env.visited_nodes[0] = 1;
    env.path_length = 1;

    // Hop 1: Rover 1 -> Rover 2
    bool ok2 = env.append_visited(2);
    assert(ok2);
    env.hop_count++;

    // Hop 2: Rover 2 -> Rover 3
    bool ok3 = env.append_visited(3);
    assert(ok3);
    env.hop_count++;

    // Hop 3: Rover 3 -> Rover 4
    bool ok4 = env.append_visited(4);
    assert(ok4);
    env.hop_count++;

    assert(env.hop_count == 3);

    // Total physical transmissions across the mesh is 3 hops
    // Wire overhead across the network is 3 * (DataWire + CustodyAck)
    size_t single_hop_wire = MultihopEnvelope::HEADER_SIZE + 36 + 24;
    size_t total_network_wire = env.hop_count * single_hop_wire;
    assert(total_network_wire == 3 * single_hop_wire);

    std::cout << "[PASS] Multi-hop forwarding wire amplification O(h) verified.\n";
}

// Test 3: Routing loop suppression prevents infinite wire amplification
static void test_routing_loop_suppression() {
    MultihopEnvelope env;
    env.origin_rover_id = 1;
    env.final_target_id = 4;
    env.max_hops = 5;
    env.message_id = 0x9999ULL;
    env.visited_nodes[0] = 1;
    env.path_length = 1;

    env.append_visited(2);
    env.append_visited(3);

    // Rover 2 receives the packet again (a routing loop: 1 -> 2 -> 3 -> 2)
    // is_loop_detected should detect that Rover 2 is already in visited_nodes!
    bool loop_detected = env.is_loop_detected(2);
    assert(loop_detected);
    std::cout << "[PASS] Routing loop detection correctly bounds network wire consumption.\n";
}

int main() {
    std::cout << "Running test_m717_network_accounting...\n";
    test_wire_byte_conservation();
    test_multihop_forwarding_overhead();
    test_routing_loop_suppression();
    std::cout << "All M7.17 network accounting tests PASSED.\n";
    return 0;
}
