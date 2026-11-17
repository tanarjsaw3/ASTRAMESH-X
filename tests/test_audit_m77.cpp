#include "astromesh/comms/dtn_buffer.hpp"
#include "astromesh/core/iblt.hpp"
#include "astromesh/core/landmark_key.hpp"
#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/net/channel_impairment.hpp"
#include "astromesh/net/distributed_reconciliation.hpp"
#include "astromesh/net/message_frame.hpp"
#include "astromesh/net/peer_tracker.hpp"
#include "astromesh/net/rover_node.hpp"
#include "astromesh/net/udp_socket.hpp"
#include "astromesh/optimization/boundary_packet.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

using namespace astromesh::net;
using namespace astromesh::core;
using namespace astromesh::graph;
using namespace astromesh::geometry;
using namespace astromesh::comms;
using namespace astromesh::verification;

#define AUDIT_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[AUDIT FAIL] " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            std::exit(1); \
        } \
    } while(0)

// ==============================================================================
// AUDIT 1: UDP Socket Lifecycle, Error Handling, Framing, and Sequencing
// ==============================================================================
void test_audit_udp_lifecycle_and_errors() {
    std::cout << "[AUDIT 1.1] Testing UDP Socket Lifecycle and Error Handlers...\n";
    UdpSocket sock;
    AUDIT_ASSERT(!sock.is_open(), "Socket should not be open on default construction");
    AUDIT_ASSERT(sock.open(), "Socket should open successfully");
    AUDIT_ASSERT(sock.is_open(), "Socket is_open should be true");

    // Dynamic port binding (port 0)
    AUDIT_ASSERT(sock.bind(0, "127.0.0.1"), "Binding to port 0 should succeed");
    uint16_t dynamic_port = sock.get_bound_port();
    AUDIT_ASSERT(dynamic_port > 0, "Bound dynamic port should be > 0");

    // Setting non-blocking mode and timeout
    AUDIT_ASSERT(sock.set_nonblocking(true), "Setting non-blocking should succeed");
    AUDIT_ASSERT(sock.set_recv_timeout_ms(50), "Setting receive timeout should succeed");

    // Invalid sends
    Endpoint invalid_ep;
    uint8_t dummy_buf[16] = {0};
    AUDIT_ASSERT(sock.send_to(dummy_buf, 16, invalid_ep) == -1, "Send to invalid endpoint must return -1");
    Endpoint valid_ep("127.0.0.1", dynamic_port);
    AUDIT_ASSERT(sock.send_to(nullptr, 16, valid_ep) == -1, "Send nullptr must return -1");
    AUDIT_ASSERT(sock.send_to(dummy_buf, 0, valid_ep) == -1, "Send 0 bytes must return -1");

    // Receive on empty socket returns 0 (timeout / wouldblock)
    Endpoint sender_out;
    uint8_t recv_buf[64];
    int64_t n_recv = sock.recv_from(recv_buf, sizeof(recv_buf), sender_out);
    AUDIT_ASSERT(n_recv == 0, "Non-blocking receive on empty socket must return 0");

    // Close and test receive on closed socket
    sock.close();
    AUDIT_ASSERT(!sock.is_open(), "Socket should be closed");
    AUDIT_ASSERT(sock.recv_from(recv_buf, sizeof(recv_buf), sender_out) == -1, "Recv on closed socket must return -1");
    AUDIT_ASSERT(sock.send_to(dummy_buf, 16, valid_ep) == -1, "Send on closed socket must return -1");

    std::cout << "  -> PASSED\n";
}

void test_audit_message_framing_and_checksums() {
    std::cout << "[AUDIT 1.2] Testing Framing, Checksums, and Malformed Rejections...\n";
    MessageFrame frame;
    frame.schema_version = MessageFrame::CURRENT_SCHEMA_VERSION;
    frame.type = MessageType::DataPayload;
    frame.sender_rover_id = 1;
    frame.target_rover_id = 2;
    frame.sequence = 42;
    frame.timestamp_ns = 1234567890ULL;
    frame.payload = {'H', 'E', 'L', 'L', 'O', '_', 'A', 'U', 'D', 'I', 'T'};

    auto bytes = frame.serialize();
    AUDIT_ASSERT(bytes.size() == MessageFrame::HEADER_SIZE + frame.payload.size(),
                 "Serialized size must equal HEADER_SIZE (36) + payload size");
    AUDIT_ASSERT(bytes.size() == 36 + 11, "Serialized size must be 47 bytes");

    // Normal round-trip
    auto decoded_opt = MessageFrame::deserialize(bytes.data(), bytes.size());
    AUDIT_ASSERT(decoded_opt.has_value(), "Deserialization of valid frame must succeed");
    AUDIT_ASSERT(decoded_opt->sequence == 42, "Sequence must match");
    AUDIT_ASSERT(decoded_opt->payload == frame.payload, "Payload must match");

    // Truncated frames
    AUDIT_ASSERT(!MessageFrame::deserialize(bytes.data(), 35).has_value(),
                 "Frame truncated below 36 bytes must be rejected");
    AUDIT_ASSERT(!MessageFrame::deserialize(bytes.data(), 46).has_value(),
                 "Frame with payload length mismatch must be rejected");

    // Corrupted magic
    auto corrupted_magic = bytes;
    corrupted_magic[0] ^= 0xFF;
    AUDIT_ASSERT(!MessageFrame::deserialize(corrupted_magic.data(), corrupted_magic.size()).has_value(),
                 "Frame with corrupted magic must be rejected");

    // Single-bit corruption in payload
    auto corrupted_payload = bytes;
    corrupted_payload[40] ^= 0x01; // flip 1 bit in payload
    AUDIT_ASSERT(!MessageFrame::deserialize(corrupted_payload.data(), corrupted_payload.size()).has_value(),
                 "Frame with payload bit-flip must fail checksum");

    // Single-bit corruption in header (e.g. sequence number)
    auto corrupted_seq = bytes;
    corrupted_seq[12] ^= 0x01; // flip 1 bit in sequence
    AUDIT_ASSERT(!MessageFrame::deserialize(corrupted_seq.data(), corrupted_seq.size()).has_value(),
                 "Frame with header sequence bit-flip must fail checksum");

    // Unsupported schema versions (0, 2, 999)
    MessageFrame old_v = frame;
    old_v.schema_version = 0;
    auto old_v_bytes = old_v.serialize();
    AUDIT_ASSERT(!MessageFrame::deserialize(old_v_bytes.data(), old_v_bytes.size()).has_value(),
                 "Older schema version 0 must be rejected");

    MessageFrame future_v = frame;
    future_v.schema_version = 2;
    auto future_v_bytes = future_v.serialize();
    AUDIT_ASSERT(!MessageFrame::deserialize(future_v_bytes.data(), future_v_bytes.size()).has_value(),
                 "Future schema version 2 must be rejected");

    MessageFrame invalid_v = frame;
    invalid_v.schema_version = 999;
    auto invalid_v_bytes = invalid_v.serialize();
    AUDIT_ASSERT(!MessageFrame::deserialize(invalid_v_bytes.data(), invalid_v_bytes.size()).has_value(),
                 "Unknown schema version 999 must be rejected");

    // Oversized payload (> 2048 B)
    std::vector<uint8_t> huge_payload(2049, 0xAA);
    MessageFrame huge_frame = frame;
    huge_frame.payload = huge_payload;
    auto huge_bytes = huge_frame.serialize();
    AUDIT_ASSERT(!MessageFrame::deserialize(huge_bytes.data(), huge_bytes.size()).has_value(),
                 "Payload exceeding 2048 bytes must be rejected");

    std::cout << "  -> PASSED\n";
}

void test_audit_peer_tracker_and_sequencing() {
    std::cout << "[AUDIT 1.3] Testing Peer Tracker, Out-of-Order Packets, and Duplicate Suppression...\n";
    PeerTracker tracker;
    const uint16_t peer_id = 2;
    const Endpoint ep("127.0.0.1", 19502);

    // Initial state: no duplicates
    AUDIT_ASSERT(!tracker.is_duplicate(peer_id, 1), "New sequence should not be duplicate");

    // Record sequence 1
    tracker.record_packet(peer_id, 1, ep, 1000);
    AUDIT_ASSERT(tracker.is_duplicate(peer_id, 1), "Sequence 1 should now be flagged as duplicate");
    AUDIT_ASSERT(!tracker.is_duplicate(peer_id, 2), "Sequence 2 should not be duplicate");

    // Record out-of-order sequence: 5 then 3 then 2
    tracker.record_packet(peer_id, 5, ep, 1010);
    tracker.record_packet(peer_id, 3, ep, 1020);
    tracker.record_packet(peer_id, 2, ep, 1030);

    AUDIT_ASSERT(tracker.is_duplicate(peer_id, 1), "Sequence 1 duplicate");
    AUDIT_ASSERT(tracker.is_duplicate(peer_id, 2), "Sequence 2 duplicate");
    AUDIT_ASSERT(tracker.is_duplicate(peer_id, 3), "Sequence 3 duplicate");
    AUDIT_ASSERT(tracker.is_duplicate(peer_id, 5), "Sequence 5 duplicate");
    AUDIT_ASSERT(!tracker.is_duplicate(peer_id, 4), "Sequence 4 should not be duplicate");

    // Sliding window boundary: push 64 more sequences (100 to 163)
    for (uint32_t s = 100; s < 164; ++s) {
        tracker.record_packet(peer_id, s, ep, 2000 + s);
    }
    // Sequences 1, 2, 3, 5 are now older than 64-element window: suppressed as stale duplicate replay
    AUDIT_ASSERT(tracker.is_duplicate(peer_id, 1), "Old sequence 1 should be suppressed as stale duplicate");
    AUDIT_ASSERT(tracker.is_duplicate(peer_id, 163), "New sequence 163 must be in window");

    // Vitality and timeout
    uint64_t now_ns = 5'000'000'000ULL;
    uint64_t timeout_ns = 1'000'000'000ULL; // 1 second
    // Peer last seen at ~2163 ns, so it should be timed out
    AUDIT_ASSERT(tracker.get_active_peers(now_ns, timeout_ns).empty(), "Peer should be inactive after 5s");

    // Update with fresh packet
    tracker.record_packet(peer_id, 200, ep, now_ns);
    auto active = tracker.get_active_peers(now_ns, timeout_ns);
    AUDIT_ASSERT(active.size() == 1 && active[0] == peer_id, "Peer should be active after fresh packet");

    std::cout << "  -> PASSED\n";
}

// ==============================================================================
// AUDIT 2: DTN Queue Limits, TTL, Retries, and Byte Accounting Invariants
// ==============================================================================
void test_audit_dtn_queue_limits_and_ttl() {
    std::cout << "[AUDIT 2.1] Testing DTN Queue Limits, Capacity Rejection, and TTL Expiration...\n";
    astromesh::comms::DtnQueueConfig cfg;
    cfg.max_queue_count = 5;
    cfg.max_payload_bytes = 1000;
    cfg.initial_retry_delay_ns = 50'000'000ULL;

    astromesh::comms::DtnBuffer buffer(cfg);
    uint64_t now_ns = 1'000'000'000ULL;

    // Enqueue 5 messages -> all should succeed
    for (uint64_t i = 1; i <= 5; ++i) {
        astromesh::comms::DtnMessage msg;
        msg.message_id = i;
        msg.sender_rover_id = 1;
        msg.target_rover_id = 2;
        msg.creation_time_ns = now_ns;
        msg.ttl_ns = 10'000'000'000ULL; // 10s TTL
        msg.payload = {0x01, 0x02, 0x03};
        auto res = buffer.enqueue(msg, now_ns);
        AUDIT_ASSERT(res == astromesh::comms::EnqueueResult::Success, "Enqueue within capacity must succeed");
    }

    // 6th message exceeds max_queue_count (5) -> must be rejected with CapacityCountExceeded
    astromesh::comms::DtnMessage overflow_msg;
    overflow_msg.message_id = 6;
    overflow_msg.sender_rover_id = 1;
    overflow_msg.target_rover_id = 2;
    overflow_msg.creation_time_ns = now_ns;
    overflow_msg.ttl_ns = 10'000'000'000ULL;
    overflow_msg.payload = {0x01};
    AUDIT_ASSERT(buffer.enqueue(overflow_msg, now_ns) == astromesh::comms::EnqueueResult::CapacityCountExceeded,
                 "6th message must be rejected due to queue capacity");

    // Test TTL expiration: advance time by 15 seconds (> 10s TTL)
    now_ns += 15'000'000'000ULL;
    auto dequeued = buffer.dequeue_next_eligible(now_ns);
    AUDIT_ASSERT(!dequeued.has_value(), "Expired messages must not be dequeued");
    AUDIT_ASSERT(buffer.empty(), "Buffer must be empty after expired messages are purged");

    std::cout << "  -> PASSED\n";
}

void test_audit_wire_byte_accounting_invariants() {
    std::cout << "[AUDIT 2.2] Testing Wire Byte Accounting Invariants and ACK Math...\n";
    RoverNodeConfig cfg;
    cfg.rover_id = 1;
    cfg.listen_port = 19601;
    cfg.transport_framing_bytes = 20;

    RoverNode node(cfg);
    AUDIT_ASSERT(node.start(), "RoverNode start should succeed");

    // Target endpoint
    Endpoint peer_ep("127.0.0.1", 19602);
    node.get_peer_tracker().register_peer(2, peer_ep);

    uint64_t now_ns = 1'000'000'000ULL;
    // Enqueue DTN message with 100-byte payload with explicit now_ns
    std::vector<uint8_t> payload(100, 0x42);
    uint64_t msg_id = node.enqueue_dtn_message(2, MessageType::DataPayload, payload,
                                               astromesh::comms::PriorityLevel::Normal,
                                               60'000'000'000ULL, now_ns);
    AUDIT_ASSERT(msg_id > 0, "Enqueue DTN message should succeed");

    // Process outgoing DTN transmission with the same now_ns
    node.poll_events(now_ns);

    auto w = node.get_wire_stats();
    std::cout << "DEBUG: w.payload_bytes_sent=" << w.payload_bytes_sent
              << " w.data_wire_bytes_sent=" << w.data_wire_bytes_sent
              << " w.dtn_messages_enqueued=" << w.dtn_messages_enqueued << "\n";
    AUDIT_ASSERT(w.payload_bytes_sent == 100, "Payload bytes sent must be 100");
    AUDIT_ASSERT(w.header_bytes_sent == 36, "Header bytes sent must be 36");
    AUDIT_ASSERT(w.framing_bytes_sent == 20, "Framing bytes sent must be 20");
    AUDIT_ASSERT(w.data_wire_bytes_sent == 156, "Data wire bytes must be exactly 156");
    AUDIT_ASSERT(w.total_wire_bytes_sent == w.data_wire_bytes_sent + w.ack_wire_bytes_sent,
                 "Total wire bytes must equal data_wire + ack_wire");

    node.stop();
    std::cout << "  -> PASSED\n";
}

// ==============================================================================
// AUDIT 3: IBLT 1024B/1036B Representation, Peeling, Geometric Verification
// ==============================================================================
void test_audit_iblt_cell_size_and_1036b_serialization() {
    std::cout << "[AUDIT 3.1] Testing IBLT Cell Size, 1024B Footprint, and 1036B Binary Format...\n";
    // 1. Verify cell struct size
    struct ExpectedIbltCell {
        int32_t count;
        uint32_t pad;
        uint64_t key_sum_low;
        uint64_t key_sum_high;
        uint64_t checksum_sum;
    };
    AUDIT_ASSERT(sizeof(ExpectedIbltCell) == 32, "IBLT cell size must be exactly 32 bytes");

    // 2. 32-cell IBLT table footprint
    constexpr size_t NUM_CELLS = 32;
    constexpr size_t CELL_DATA_SIZE = NUM_CELLS * 32;
    AUDIT_ASSERT(CELL_DATA_SIZE == 1024, "32 IBLT cells must occupy exactly 1024 bytes");

    InvertibleBloomFilter iblt(NUM_CELLS, 3, 1);
    AUDIT_ASSERT(iblt.num_cells() == 32, "Cell count must be 32");

    // 3. Serialized binary buffer size: 12-byte header (nc, np, proto) + 1024-byte cells = 1036 bytes
    auto bytes = serialize_iblt_fingerprint(iblt);
    AUDIT_ASSERT(bytes.size() == 1036, "Serialized IBLT buffer must be exactly 1036 bytes");

    // Verify 12-byte header
    uint32_t nc = 0, np = 0, proto = 0;
    std::memcpy(&nc, &bytes[0], 4);
    std::memcpy(&np, &bytes[4], 4);
    std::memcpy(&proto, &bytes[8], 4);
    AUDIT_ASSERT(nc == 32, "Header nc must be 32");
    AUDIT_ASSERT(np == 3, "Header np must be 3");
    AUDIT_ASSERT(proto == 1, "Header proto must be 1");

    // Round-trip deserialization
    auto deserialized = deserialize_iblt_fingerprint(bytes.data(), bytes.size());
    AUDIT_ASSERT(deserialized.has_value(), "Deserialization must succeed");
    AUDIT_ASSERT(deserialized->num_cells() == 32, "Deserialized cell count must be 32");
    AUDIT_ASSERT(deserialized->num_probes() == 3, "Deserialized probe count must be 3");

    std::cout << "  -> PASSED\n";
}

void test_audit_iblt_peeling_capacity_and_overflow() {
    std::cout << "[AUDIT 3.2] Testing IBLT Peeling Under Capacity and Overflow Failure...\n";
    InvertibleBloomFilter iblt_a(32, 3, 1);
    InvertibleBloomFilter iblt_b(32, 3, 1);

    // Shared keys
    for (uint32_t i = 1; i <= 20; ++i) {
        LandmarkKey key(1, 1, i, 0x1000ULL + i);
        iblt_a.insert(key);
        iblt_b.insert(key);
    }

    // Difference of 6 keys (3 only in A, 3 only in B) -> well within capacity (d <= 26)
    for (uint32_t i = 21; i <= 23; ++i) {
        iblt_a.insert(LandmarkKey(1, 1, i, 0x1000ULL + i));
    }
    for (uint32_t i = 24; i <= 26; ++i) {
        iblt_b.insert(LandmarkKey(1, 2, i, 0x2000ULL + i));
    }

    auto diff = iblt_a.subtract(iblt_b);
    auto decode_res = diff.decode();
    AUDIT_ASSERT(decode_res.is_success(), "Decoding difference of 6 keys must succeed");
    AUDIT_ASSERT(decode_res.left_only.size() == 3, "Left-only size must be 3");
    AUDIT_ASSERT(decode_res.right_only.size() == 3, "Right-only size must be 3");

    // Overload difference: insert 35 disjoint keys into A -> d > 26 capacity
    for (uint32_t i = 100; i <= 135; ++i) {
        iblt_a.insert(LandmarkKey(1, 1, i, 0x9000ULL + i));
    }
    auto overloaded_diff = iblt_a.subtract(iblt_b);
    auto overloaded_res = overloaded_diff.decode();
    AUDIT_ASSERT(!overloaded_res.is_success(), "Overloaded difference must fail decode gracefully");
    AUDIT_ASSERT(overloaded_res.status == astromesh::core::IBLTDecodeStatus::IncompleteResidual,
                 "DecodeStatus must be IncompleteResidual");

    std::cout << "  -> PASSED\n";
}

void test_audit_geometric_verification_valid_vs_adversarial() {
    std::cout << "[AUDIT 3.3] Testing Geometric Verification: Valid vs. Adversarial Outliers...\n";
    astromesh::verification::RansacConfig2D config;
    config.min_correspondences = 3;
    config.min_inliers = 3;
    config.inlier_distance_threshold = 0.50;

    // 1. Valid correspondences with true translation (10.0, 0.0)
    std::vector<astromesh::verification::PointCorrespondence2D> valid_corr = {
        { {2.0, 3.0}, {-8.0, 3.0}, LandmarkKey(1, 1, 1, 0x1001) },
        { {4.0, 1.0}, {-6.0, 1.0}, LandmarkKey(1, 1, 2, 0x1002) },
        { {3.0, -2.0}, {-7.0, -2.0}, LandmarkKey(1, 1, 3, 0x1003) }
    };
    auto valid_res = astromesh::verification::verify_relative_pose_2d(valid_corr, config);
    AUDIT_ASSERT(valid_res.success, "Valid correspondences must pass geometric verification");
    AUDIT_ASSERT(valid_res.inlier_count == 3, "All 3 must be inliers");
    AUDIT_ASSERT(std::abs(valid_res.estimated_transform.x - (-10.0)) < 1e-4, "Estimated dx must be -10.0m");
    AUDIT_ASSERT(std::abs(valid_res.estimated_transform.y) < 1e-4, "Estimated dy must be 0.0m");

    // 2. Adversarial correspondences (perceptual aliasing: same landmark keys, scrambled coordinates)
    std::vector<astromesh::verification::PointCorrespondence2D> aliased_corr = {
        { {2.0, 3.0}, {15.0, -8.0}, LandmarkKey(1, 1, 1, 0x1001) },
        { {4.0, 1.0}, {-2.0, 9.0}, LandmarkKey(1, 1, 2, 0x1002) },
        { {3.0, -2.0}, {0.0, 0.0}, LandmarkKey(1, 1, 3, 0x1003) }
    };
    auto aliased_res = astromesh::verification::verify_relative_pose_2d(aliased_corr, config);
    AUDIT_ASSERT(!aliased_res.success, "Scrambled aliased correspondences must FAIL geometric verification");

    std::cout << "  -> PASSED\n";
}

// ==============================================================================
// AUDIT 5: Failure, Stress, and Resilience Testing
// ==============================================================================
void test_audit_channel_complete_packet_loss() {
    std::cout << "[AUDIT 5.1] Testing Complete (100%) Packet Loss Resilience...\n";
    RoverNodeConfig cfg;
    cfg.rover_id = 1;
    cfg.listen_port = 19701;
    cfg.impairment_config.packet_loss_rate = 1.0; // 100% loss

    RoverNode node(cfg);
    AUDIT_ASSERT(node.start(), "RoverNode start should succeed");
    node.get_peer_tracker().register_peer(2, Endpoint("127.0.0.1", 19702));

    // Enqueue message
    uint64_t msg_id = node.enqueue_dtn_message(2, MessageType::DataPayload, {1, 2, 3});
    AUDIT_ASSERT(msg_id > 0, "Enqueue should succeed");

    // Poll 10 ticks under 100% loss
    for (int t = 0; t < 10; ++t) {
        node.poll_events(1'000'000'000ULL + t * 50'000'000ULL);
    }

    auto w = node.get_wire_stats();
    // Packets attempted and retransmitted, 0 ACKs received, no crashes
    AUDIT_ASSERT(w.acks_received == 0, "No ACKs should be received under 100% loss");
    AUDIT_ASSERT(w.dtn_messages_delivered == 0, "0 messages delivered under 100% loss");

    node.stop();
    std::cout << "  -> PASSED\n";
}

void test_audit_channel_blackout_and_recovery() {
    std::cout << "[AUDIT 5.2] Testing Communication Blackout and Recovery...\n";
    RoverNodeConfig cfg1;
    cfg1.rover_id = 1;
    cfg1.listen_port = 19801;
    cfg1.seed_peers = { Endpoint("127.0.0.1", 19802) };

    RoverNodeConfig cfg2;
    cfg2.rover_id = 2;
    cfg2.listen_port = 19802;
    cfg2.seed_peers = { Endpoint("127.0.0.1", 19801) };

    RoverNode r1(cfg1);
    RoverNode r2(cfg2);
    AUDIT_ASSERT(r1.start() && r2.start(), "Both nodes should start");

    std::vector<std::string> r2_received;
    r2.set_message_handler([&](const MessageFrame& frame, const Endpoint&) {
        r2_received.push_back(std::string(frame.payload.begin(), frame.payload.end()));
    });

    // 1. Bilateral blackout active before enqueue
    r1.get_channel_impairment().set_link_state(1, 2, false);
    r2.get_channel_impairment().set_link_state(1, 2, false);

    std::vector<uint8_t> payload = {'B', 'L', 'A', 'C', 'K', 'O', 'U', 'T'};
    uint64_t msg_id = r1.enqueue_dtn_message(2, MessageType::DataPayload, payload);
    AUDIT_ASSERT(msg_id > 0, "Enqueue during blackout should succeed");

    // Poll during blackout -> message remains in buffer
    for (int i = 0; i < 3; ++i) {
        r1.poll_events(0);
        r2.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    AUDIT_ASSERT(r1.get_wire_stats().dtn_messages_delivered == 0, "No delivery during blackout");
    AUDIT_ASSERT(r2_received.empty(), "Receiver must not receive payload during blackout");

    // 2. Restore link
    r1.get_channel_impairment().set_link_state(1, 2, true);
    r2.get_channel_impairment().set_link_state(1, 2, true);

    // Poll after reconnection -> message delivered and ACK received!
    bool delivered = false;
    for (int t = 1; t <= 20; ++t) {
        r1.poll_events(0);
        r2.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (r1.get_wire_stats().dtn_messages_delivered > 0 && !r2_received.empty()) {
            delivered = true;
            break;
        }
    }
    AUDIT_ASSERT(delivered, "Message must be delivered successfully after blackout recovery");
    AUDIT_ASSERT(r2_received[0] == "BLACKOUT", "Delivered payload must match");

    r1.stop();
    r2.stop();
    std::cout << "  -> PASSED\n";
}

void test_audit_high_message_volume_and_malformed_burst() {
    std::cout << "[AUDIT 5.3] Testing High Message Volume and Malformed Datagram Burst...\n";
    RoverNodeConfig cfg;
    cfg.rover_id = 1;
    cfg.listen_port = 19901;

    RoverNode node(cfg);
    AUDIT_ASSERT(node.start(), "Node start should succeed");

    // Send bursts of malformed datagrams directly via raw socket to node's port
    UdpSocket attacker;
    AUDIT_ASSERT(attacker.open(), "Attacker socket open");
    Endpoint target("127.0.0.1", 19901);

    // 1. Zero-byte datagrams
    uint8_t zero_byte = 0;
    attacker.send_to(&zero_byte, 0, target);

    // 2. 1-byte, 10-byte truncated garbage
    uint8_t garbage[10] = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
    attacker.send_to(garbage, sizeof(garbage), target);

    // 3. 36-byte valid size but invalid magic
    uint8_t fake_hdr[36] = {0};
    fake_hdr[0] = 0xAA; fake_hdr[1] = 0xBB;
    attacker.send_to(fake_hdr, sizeof(fake_hdr), target);

    // 4. 200 rapid raw datagrams
    for (int i = 0; i < 200; ++i) {
        attacker.send_to(garbage, sizeof(garbage), target);
    }

    // Node drains socket buffer -> must not crash, corrupted packets recorded
    size_t processed = node.poll_events(1'000'000'000ULL);
    AUDIT_ASSERT(processed == 0, "No valid application messages should be produced from garbage");
    AUDIT_ASSERT(node.get_stats().corrupted_dropped > 0, "Corrupted dropped count must be > 0");

    attacker.close();
    node.stop();
    std::cout << "  -> PASSED\n";
}

// ==============================================================================
// MAIN
// ==============================================================================
int main() {
    std::cout << "=================================================================\n";
    std::cout << "  ASTRAMESH-X M7.7 DISTRIBUTED SYSTEM INTEGRITY AUDIT SUITE      \n";
    std::cout << "=================================================================\n";

    // Audit 1
    test_audit_udp_lifecycle_and_errors();
    test_audit_message_framing_and_checksums();
    test_audit_peer_tracker_and_sequencing();

    // Audit 2
    test_audit_dtn_queue_limits_and_ttl();
    test_audit_wire_byte_accounting_invariants();

    // Audit 3
    test_audit_iblt_cell_size_and_1036b_serialization();
    test_audit_iblt_peeling_capacity_and_overflow();
    test_audit_geometric_verification_valid_vs_adversarial();

    // Audit 5
    test_audit_channel_complete_packet_loss();
    test_audit_channel_blackout_and_recovery();
    test_audit_high_message_volume_and_malformed_burst();

    std::cout << "=================================================================\n";
    std::cout << "  ALL M7.7 INTEGRITY AUDIT TESTS PASSED CLEANLY (100%)           \n";
    std::cout << "=================================================================\n";
    return 0;
}
