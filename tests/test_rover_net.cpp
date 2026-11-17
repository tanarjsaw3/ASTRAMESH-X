#include "astromesh/net/message_frame.hpp"
#include "astromesh/net/peer_tracker.hpp"
#include "astromesh/net/rover_node.hpp"
#include "astromesh/net/udp_socket.hpp"

#include <cassert>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace astromesh::net;

static void test_message_framing_roundtrip() {
    std::cout << "[TEST 1] Message framing serialization & deserialization round-trip...\n";

    MessageFrame frame;
    frame.schema_version = MessageFrame::CURRENT_SCHEMA_VERSION;
    frame.type = MessageType::LandmarkSync;
    frame.sender_rover_id = 42;
    frame.target_rover_id = 100;
    frame.sequence = 1337;
    frame.timestamp_ns = 1700000000000ULL;
    frame.payload = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04};

    auto bytes = frame.serialize();
    assert(bytes.size() == MessageFrame::HEADER_SIZE + frame.payload.size());

    auto decoded = MessageFrame::deserialize(bytes.data(), bytes.size());
    assert(decoded.has_value());
    assert(decoded->schema_version == frame.schema_version);
    assert(decoded->type == frame.type);
    assert(decoded->sender_rover_id == frame.sender_rover_id);
    assert(decoded->target_rover_id == frame.target_rover_id);
    assert(decoded->sequence == frame.sequence);
    assert(decoded->timestamp_ns == frame.timestamp_ns);
    assert(decoded->payload == frame.payload);
    assert(decoded->verify_checksum());

    std::cout << "  -> PASSED\n";
}

static void test_tampered_frame_rejection() {
    std::cout << "[TEST 2] Corrupted and tampered frame rejection...\n";

    MessageFrame frame;
    frame.type = MessageType::DataPayload;
    frame.sender_rover_id = 1;
    frame.sequence = 10;
    frame.payload = {1, 2, 3, 4, 5};

    auto bytes = frame.serialize();

    // 1. Corrupt magic byte
    bytes[0] ^= 0xFF;
    assert(!MessageFrame::deserialize(bytes.data(), bytes.size()).has_value());
    bytes[0] ^= 0xFF; // restore

    // 2. Corrupt payload bit
    bytes[MessageFrame::HEADER_SIZE + 2] ^= 0x01;
    assert(!MessageFrame::deserialize(bytes.data(), bytes.size()).has_value());
    bytes[MessageFrame::HEADER_SIZE + 2] ^= 0x01; // restore

    // 3. Truncate header
    assert(!MessageFrame::deserialize(bytes.data(), 20).has_value());

    // 4. Truncate payload
    assert(!MessageFrame::deserialize(bytes.data(), bytes.size() - 1).has_value());

    // 5. Older unsupported schema version (0)
    MessageFrame old_frame = frame;
    old_frame.schema_version = 0;
    auto old_bytes = old_frame.serialize();
    assert(!MessageFrame::deserialize(old_bytes.data(), old_bytes.size()).has_value());

    // 6. Future unsupported schema version (2)
    MessageFrame future_frame = frame;
    future_frame.schema_version = 2;
    auto future_bytes = future_frame.serialize();
    assert(!MessageFrame::deserialize(future_bytes.data(), future_bytes.size()).has_value());

    // 7. Unknown large future version (999)
    MessageFrame unknown_frame = frame;
    unknown_frame.schema_version = 999;
    auto unknown_bytes = unknown_frame.serialize();
    assert(!MessageFrame::deserialize(unknown_bytes.data(), unknown_bytes.size()).has_value());

    // 8. Corrupted version field in serialized buffer (e.g. byte 4 altered)
    auto corrupted_version = bytes;
    corrupted_version[4] = 0xFF;
    assert(!MessageFrame::deserialize(corrupted_version.data(), corrupted_version.size()).has_value());

    std::cout << "  -> PASSED\n";
}

static void test_sequence_tracking_and_duplicate_suppression() {
    std::cout << "[TEST 3] Sequence tracking and duplicate packet suppression...\n";

    PeerTracker tracker;
    Endpoint ep{"127.0.0.1", 19001};

    assert(!tracker.is_duplicate(1, 100));
    tracker.record_packet(1, 100, ep, 1000);
    assert(tracker.is_duplicate(1, 100));
    assert(!tracker.is_duplicate(1, 101));

    // Record 64 packets (sliding window fill)
    for (uint32_t s = 200; s < 264; ++s) {
        tracker.record_packet(1, s, ep, 1000 + s);
        assert(tracker.is_duplicate(1, s));
    }

    // Sequence 100 has fallen out of the 64-item window (suppressed as stale duplicate replay)
    assert(tracker.is_duplicate(1, 100));

    // Peer 2 should have independent sequence tracking
    assert(!tracker.is_duplicate(2, 200));

    std::cout << "  -> PASSED\n";
}

static void test_peer_tracker_vitality_and_timeout() {
    std::cout << "[TEST 4] Peer tracker vitality and heartbeat timeout...\n";

    PeerTracker tracker;
    Endpoint ep{"127.0.0.1", 19001};

    uint64_t t0 = 1'000'000'000ULL;
    tracker.record_packet(1, 1, ep, t0);

    const uint64_t timeout_ns = 500'000'000ULL; // 500 ms
    assert(tracker.active_peer_count(t0 + 200'000'000ULL, timeout_ns) == 1);

    auto active = tracker.get_active_peers(t0 + 200'000'000ULL, timeout_ns);
    assert(active.size() == 1 && active[0] == 1);

    // After 600 ms, peer should be considered timed out
    assert(tracker.active_peer_count(t0 + 600'000'000ULL, timeout_ns) == 0);
    assert(tracker.get_active_peers(t0 + 600'000'000ULL, timeout_ns).empty());

    std::cout << "  -> PASSED\n";
}

static void test_udp_socket_loopback_io() {
    std::cout << "[TEST 5] Raw UDP socket loopback I/O...\n";

    UdpSocket s1;
    UdpSocket s2;

    assert(s1.bind(19201, "127.0.0.1"));
    assert(s2.bind(19202, "127.0.0.1"));
    assert(s2.set_nonblocking(true));

    const std::string msg = "AstroMesh-X UDP Loopback Test";
    int64_t sent = s1.send_to(reinterpret_cast<const uint8_t*>(msg.data()), msg.size(), {"127.0.0.1", 19202});
    assert(sent == static_cast<int64_t>(msg.size()));

    // Small delay for OS packet delivery
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    uint8_t buf[256];
    Endpoint sender;
    int64_t recvd = s2.recv_from(buf, sizeof(buf), sender);
    assert(recvd == static_cast<int64_t>(msg.size()));
    assert(sender.port == 19201);
    assert(std::string(reinterpret_cast<char*>(buf), recvd) == msg);

    s1.close();
    s2.close();
    std::cout << "  -> PASSED\n";
}

static void test_rover_node_peer_discovery_and_messaging() {
    std::cout << "[TEST 6] Autonomous peer discovery and bidirectional messaging between RoverNodes...\n";

    RoverNodeConfig c1;
    c1.rover_id = 1;
    c1.listen_port = 19301;
    c1.seed_peers = {{"127.0.0.1", 19302}};
    c1.heartbeat_interval_ms = 50;
    c1.peer_timeout_ms = 400;

    RoverNodeConfig c2;
    c2.rover_id = 2;
    c2.listen_port = 19302;
    c2.seed_peers = {{"127.0.0.1", 19301}};
    c2.heartbeat_interval_ms = 50;
    c2.peer_timeout_ms = 400;

    RoverNode r1(c1);
    RoverNode r2(c2);

    assert(r1.start());
    assert(r2.start());

    std::vector<MessageFrame> r2_received;
    r2.set_message_handler([&](const MessageFrame& frame, const Endpoint& /*sender*/) {
        r2_received.push_back(frame);
    });

    // Exchange heartbeats for 250 ms
    for (int step = 0; step < 25; ++step) {
        r1.poll_events();
        r2.poll_events();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Verify mutual discovery
    auto p1_active = r1.get_peer_tracker().get_active_peers(0, 500'000'000ULL);
    auto p2_active = r2.get_peer_tracker().get_active_peers(0, 500'000'000ULL);
    assert(!p1_active.empty() && p1_active[0] == 2);
    assert(!p2_active.empty() && p2_active[0] == 1);

    // Send application message from Rover 1 to Rover 2
    const std::vector<uint8_t> payload = {'S', 'L', 'A', 'M', '_', 'S', 'Y', 'N', 'C'};
    int64_t sent = r1.send_message_to(2, MessageType::LandmarkSync, payload);
    assert(sent > 0);

    // Poll to receive
    for (int step = 0; step < 10; ++step) {
        r1.poll_events();
        r2.poll_events();
        if (!r2_received.empty()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    assert(r2_received.size() == 1);
    assert(r2_received[0].sender_rover_id == 1);
    assert(r2_received[0].type == MessageType::LandmarkSync);
    assert(r2_received[0].payload == payload);

    r1.stop();
    r2.stop();
    std::cout << "  -> PASSED\n";
}

int main() {
    std::cout << "=================================================================\n";
    std::cout << "     ASTRAMESH-X STANDALONE ROVER NETWORKING TEST SUITE (M1)     \n";
    std::cout << "=================================================================\n";

    test_message_framing_roundtrip();
    test_tampered_frame_rejection();
    test_sequence_tracking_and_duplicate_suppression();
    test_peer_tracker_vitality_and_timeout();
    test_udp_socket_loopback_io();
    test_rover_node_peer_discovery_and_messaging();

    std::cout << "=================================================================\n";
    std::cout << "      ALL 6 ROVER NETWORKING TESTS PASSED SUCCESSFULLY!          \n";
    std::cout << "=================================================================\n";
    return 0;
}

