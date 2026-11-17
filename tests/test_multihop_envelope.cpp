#include "astromesh/net/multihop_envelope.hpp"

#include <cassert>
#include <cstring>
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

// 1. Exact Serialization & Deserialization Round-trip
static void test_exact_serialization_roundtrip() {
    std::cout << "[TEST 1] Exact Serialization & Deserialization Round-trip..." << std::endl;

    MultihopEnvelope env;
    env.version = 1;
    env.flags = MultihopFlags::CUSTODY_ACK_REQUESTED | MultihopFlags::E2E_ACK_REQUESTED;
    env.origin_rover_id = 1;
    env.final_target_id = 3;
    env.hop_count = 1;
    env.max_hops = 5;
    env.message_id = 0x1122334455667788ULL;
    env.path_length = 2;
    env.visited_nodes = {1, 2, 0, 0, 0, 0, 0, 0};
    env.payload = {'H', 'E', 'L', 'L', 'O', '_', 'M', 'E', 'S', 'H'};

    auto bytes = env.serialize();
    TEST_ASSERT(bytes.size() == 40 + env.payload.size(), "Total serialized size must be 40 + payload");

    // Byte offset verification
    // Offset 0..3: MAGIC 'MHTN'
    TEST_ASSERT(bytes[0] == 'M' && bytes[1] == 'H' && bytes[2] == 'T' && bytes[3] == 'N', "Magic bytes match");
    // Offset 4: Version
    TEST_ASSERT(bytes[4] == 1, "Version byte");
    // Offset 5: Flags
    TEST_ASSERT(bytes[5] == 0x03, "Flags byte");
    // Offset 6..7: Origin 1
    TEST_ASSERT(bytes[6] == 1 && bytes[7] == 0, "Origin rover ID");
    // Offset 8..9: Target 3
    TEST_ASSERT(bytes[8] == 3 && bytes[9] == 0, "Target rover ID");
    // Offset 10: Hop Count
    TEST_ASSERT(bytes[10] == 1, "Hop count");
    // Offset 11: Max Hops
    TEST_ASSERT(bytes[11] == 5, "Max hops");
    // Offset 12..19: Message ID
    uint64_t decoded_id = 0;
    std::memcpy(&decoded_id, bytes.data() + 12, 8);
    TEST_ASSERT(decoded_id == 0x1122334455667788ULL, "Message ID");
    // Offset 20: Path length
    TEST_ASSERT(bytes[20] == 2, "Path length");
    // Offset 24..27: Visited nodes 0 and 1
    TEST_ASSERT(bytes[24] == 1 && bytes[25] == 0, "Visited node 0");
    TEST_ASSERT(bytes[26] == 2 && bytes[27] == 0, "Visited node 1");

    // Deserialization
    auto decoded_opt = MultihopEnvelope::deserialize(bytes.data(), bytes.size());
    TEST_ASSERT(decoded_opt.has_value(), "Deserialization must succeed");
    const auto& dec = *decoded_opt;
    TEST_ASSERT(dec.version == env.version, "Version preserved");
    TEST_ASSERT(dec.flags == env.flags, "Flags preserved");
    TEST_ASSERT(dec.origin_rover_id == env.origin_rover_id, "Origin preserved");
    TEST_ASSERT(dec.final_target_id == env.final_target_id, "Target preserved");
    TEST_ASSERT(dec.hop_count == env.hop_count, "Hop count preserved");
    TEST_ASSERT(dec.max_hops == env.max_hops, "Max hops preserved");
    TEST_ASSERT(dec.message_id == env.message_id, "Message ID preserved");
    TEST_ASSERT(dec.path_length == env.path_length, "Path length preserved");
    TEST_ASSERT(dec.visited_nodes == env.visited_nodes, "Visited nodes preserved");
    TEST_ASSERT(dec.payload == env.payload, "Payload preserved");

    std::cout << "  -> PASSED" << std::endl;
}

// 2. Invalid Magic Rejection
static void test_invalid_magic_rejection() {
    std::cout << "[TEST 2] Invalid Magic Rejection..." << std::endl;

    MultihopEnvelope env;
    env.origin_rover_id = 1;
    env.final_target_id = 2;
    env.hop_count = 0;
    env.max_hops = 5;
    env.message_id = 100;
    env.path_length = 1;
    env.visited_nodes = {1, 0, 0, 0, 0, 0, 0, 0};

    auto bytes = env.serialize();
    bytes[0] = 'X'; // Corrupt magic

    auto dec = MultihopEnvelope::deserialize(bytes.data(), bytes.size());
    TEST_ASSERT(!dec.has_value(), "Must reject corrupted magic");

    std::cout << "  -> PASSED" << std::endl;
}

// 3. Unsupported Version & Unknown Flags Rejection
static void test_invalid_version_and_flags() {
    std::cout << "[TEST 3] Unsupported Version & Unknown Flags..." << std::endl;

    MultihopEnvelope env;
    env.origin_rover_id = 1;
    env.final_target_id = 2;
    env.hop_count = 0;
    env.max_hops = 5;
    env.message_id = 100;
    env.path_length = 1;
    env.visited_nodes = {1, 0, 0, 0, 0, 0, 0, 0};

    auto bytes = env.serialize();
    // Test invalid version 2
    bytes[4] = 2;
    TEST_ASSERT(!MultihopEnvelope::deserialize(bytes.data(), bytes.size()).has_value(), "Must reject version 2");

    // Test unknown flags
    bytes[4] = 1;
    bytes[5] = 0x80; // Undefined high flag bit
    TEST_ASSERT(!MultihopEnvelope::deserialize(bytes.data(), bytes.size()).has_value(), "Must reject unknown flag bits");

    std::cout << "  -> PASSED" << std::endl;
}

// 4. Truncated Payload Rejection
static void test_truncated_payload_rejection() {
    std::cout << "[TEST 4] Truncated Payload Rejection..." << std::endl;

    MultihopEnvelope env;
    env.origin_rover_id = 1;
    env.final_target_id = 2;
    env.path_length = 1;
    env.visited_nodes = {1, 0, 0, 0, 0, 0, 0, 0};

    auto bytes = env.serialize();
    // Less than HEADER_SIZE (40 bytes)
    for (size_t len = 0; len < 40; ++len) {
        TEST_ASSERT(!MultihopEnvelope::deserialize(bytes.data(), len).has_value(), "Must reject truncated header");
    }

    std::cout << "  -> PASSED" << std::endl;
}

// 5. Hop Limit & Boundary Semantics
static void test_hop_boundaries_and_ttl() {
    std::cout << "[TEST 5] Hop Boundaries & TTL Semantics..." << std::endl;

    MultihopEnvelope env;
    env.origin_rover_id = 1;
    env.final_target_id = 2;
    env.max_hops = 8;
    env.path_length = 1;
    env.visited_nodes = {1, 0, 0, 0, 0, 0, 0, 0};

    // Valid initial state
    env.hop_count = 0;
    TEST_ASSERT(!env.is_expired(), "Hop 0 is not expired");

    env.hop_count = 7;
    TEST_ASSERT(!env.is_expired(), "Hop 7 is not expired for max_hops 8");

    env.hop_count = 8;
    TEST_ASSERT(env.is_expired(), "Hop 8 is expired for max_hops 8");

    // Enforce max_hops <= 8 limit
    env.max_hops = 9;
    TEST_ASSERT(!env.validate(), "Must reject max_hops > 8");

    env.max_hops = 0;
    TEST_ASSERT(!env.validate(), "Must reject max_hops == 0");

    std::cout << "  -> PASSED" << std::endl;
}

// 6. Path Loop Detection & Duplicate Path Rejection
static void test_loop_detection_and_duplicate_path() {
    std::cout << "[TEST 6] Loop Detection & Duplicate Path Rejection..." << std::endl;

    MultihopEnvelope env;
    env.origin_rover_id = 1;
    env.final_target_id = 4;
    env.hop_count = 2;
    env.max_hops = 5;
    env.message_id = 500;
    env.path_length = 3;
    env.visited_nodes = {1, 2, 3, 0, 0, 0, 0, 0};

    TEST_ASSERT(env.validate(), "Valid 3-node path");
    TEST_ASSERT(env.is_loop_detected(1), "Rover 1 already visited");
    TEST_ASSERT(env.is_loop_detected(2), "Rover 2 already visited");
    TEST_ASSERT(env.is_loop_detected(3), "Rover 3 already visited");
    TEST_ASSERT(!env.is_loop_detected(4), "Rover 4 not yet visited");

    // Test append
    TEST_ASSERT(env.append_visited(4), "Should append rover 4");
    TEST_ASSERT(env.path_length == 4, "Path length incremented to 4");
    TEST_ASSERT(env.is_loop_detected(4), "Rover 4 now visited");

    // Cannot append rover 2 again (duplicate / loop)
    TEST_ASSERT(!env.append_visited(2), "Must reject re-appending rover 2");

    // Deserialization with duplicate in path must be rejected
    auto bytes = env.serialize();
    bytes[24 + 2 * 3] = 2; // Overwrite 4th node with 2 -> path is [1, 2, 3, 2]
    TEST_ASSERT(!MultihopEnvelope::deserialize(bytes.data(), bytes.size()).has_value(), "Must reject duplicate in wire path");

    std::cout << "  -> PASSED" << std::endl;
}

// 7. Maximum Path Capacity (8 Hops / 8 Visited Nodes)
static void test_maximum_path_capacity() {
    std::cout << "[TEST 7] Maximum Path Capacity (8 Nodes)..." << std::endl;

    MultihopEnvelope env;
    env.origin_rover_id = 1;
    env.final_target_id = 9;
    env.hop_count = 7;
    env.max_hops = 8;
    env.message_id = 999;
    env.path_length = 8;
    env.visited_nodes = {1, 2, 3, 4, 5, 6, 7, 8};

    TEST_ASSERT(env.validate(), "8-node path must be valid");
    TEST_ASSERT(!env.append_visited(9), "Cannot append beyond 8 nodes");

    auto bytes = env.serialize();
    auto dec = MultihopEnvelope::deserialize(bytes.data(), bytes.size());
    TEST_ASSERT(dec.has_value(), "8-node path must deserialize cleanly");
    TEST_ASSERT(dec->path_length == 8, "Path length must be 8");

    std::cout << "  -> PASSED" << std::endl;
}

// 8. Origin Node Must Match First Visited Node
static void test_origin_first_node_invariant() {
    std::cout << "[TEST 8] Origin Must Match First Visited Node..." << std::endl;

    MultihopEnvelope env;
    env.origin_rover_id = 1;
    env.final_target_id = 3;
    env.hop_count = 0;
    env.max_hops = 5;
    env.message_id = 10;
    env.path_length = 1;
    env.visited_nodes = {2, 0, 0, 0, 0, 0, 0, 0}; // Mismatch: origin 1, but visited[0] is 2

    TEST_ASSERT(!env.validate(), "Must fail if visited[0] != origin_rover_id");

    auto bytes = env.serialize();
    TEST_ASSERT(!MultihopEnvelope::deserialize(bytes.data(), bytes.size()).has_value(), "Must reject mismatch on wire");

    std::cout << "  -> PASSED" << std::endl;
}

// 9. Broadcast Target Support (0)
static void test_broadcast_target_support() {
    std::cout << "[TEST 9] Broadcast Target Support..." << std::endl;

    MultihopEnvelope env;
    env.origin_rover_id = 1;
    env.final_target_id = 0; // Broadcast
    env.hop_count = 0;
    env.max_hops = 3;
    env.message_id = 12345;
    env.path_length = 1;
    env.visited_nodes = {1, 0, 0, 0, 0, 0, 0, 0};

    TEST_ASSERT(env.validate(), "Broadcast target 0 must be valid");
    auto bytes = env.serialize();
    auto dec = MultihopEnvelope::deserialize(bytes.data(), bytes.size());
    TEST_ASSERT(dec.has_value(), "Broadcast must deserialize cleanly");
    TEST_ASSERT(dec->final_target_id == 0, "Target 0 preserved");

    std::cout << "  -> PASSED" << std::endl;
}

int main() {
    std::cout << "=================================================================" << std::endl;
    std::cout << "  ASTRAMESH-X M7.11 MULTIHOP ENVELOPE UNIT TEST SUITE            " << std::endl;
    std::cout << "=================================================================" << std::endl;

    test_exact_serialization_roundtrip();
    test_invalid_magic_rejection();
    test_invalid_version_and_flags();
    test_truncated_payload_rejection();
    test_hop_boundaries_and_ttl();
    test_loop_detection_and_duplicate_path();
    test_maximum_path_capacity();
    test_origin_first_node_invariant();
    test_broadcast_target_support();

    std::cout << "=================================================================" << std::endl;
    std::cout << "  ALL 9 MULTIHOP ENVELOPE TESTS PASSED (100%)                    " << std::endl;
    std::cout << "=================================================================" << std::endl;
    return 0;
}
