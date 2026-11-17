#include "astromesh/comms/dtn_buffer.hpp"
#include "astromesh/net/channel_impairment.hpp"
#include "astromesh/net/message_frame.hpp"
#include "astromesh/net/rover_node.hpp"

#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

using namespace astromesh::net;
using namespace astromesh::comms;

void test_channel_impairment_loss() {
    std::cout << "[TEST] Running test_channel_impairment_loss..." << std::endl;

    ChannelImpairmentConfig cfg;
    cfg.packet_loss_rate = 0.30; // 30% loss
    cfg.seed = 98765;
    ChannelImpairment impairment(cfg);

    const size_t TOTAL_PACKETS = 2000;
    Endpoint dummy_ep("127.0.0.1", 19999);
    uint8_t payload[64] = {0};

    uint64_t now_ns = 1'000'000'000ULL;
    for (size_t i = 0; i < TOTAL_PACKETS; ++i) {
        bool immediate = false;
        impairment.process_outgoing(payload, sizeof(payload), dummy_ep, 1, 2, now_ns, immediate);
    }

    auto stats = impairment.get_stats();
    assert(stats.packets_submitted == TOTAL_PACKETS);
    assert(stats.packets_dropped_loss + stats.packets_passed == TOTAL_PACKETS);

    const double empirical_loss = static_cast<double>(stats.packets_dropped_loss) / static_cast<double>(TOTAL_PACKETS);
    std::cout << "  Submitted: " << stats.packets_submitted
              << " Dropped: " << stats.packets_dropped_loss
              << " Passed: " << stats.packets_passed
              << " Empirical loss: " << (empirical_loss * 100.0) << "%" << std::endl;

    // Within +/- 4% of target 30%
    assert(std::abs(empirical_loss - 0.30) < 0.04);
    std::cout << "  -> Passed." << std::endl;
}

void test_channel_impairment_delay() {
    std::cout << "[TEST] Running test_channel_impairment_delay..." << std::endl;

    ChannelImpairmentConfig cfg;
    cfg.one_way_delay_ns = 50'000'000ULL; // 50 ms
    ChannelImpairment impairment(cfg);

    Endpoint dummy_ep("127.0.0.1", 19999);
    uint8_t payload[32] = {1, 2, 3, 4};

    const uint64_t t0 = 100'000'000ULL;
    bool immediate = false;
    bool scheduled = impairment.process_outgoing(payload, sizeof(payload), dummy_ep, 1, 2, t0, immediate);

    assert(scheduled == true);
    assert(immediate == false); // Must be delayed
    assert(impairment.pending_delayed_count() == 1);

    // At t0 + 20 ms, packet must still be delayed
    auto ready1 = impairment.release_due_packets(t0 + 20'000'000ULL);
    assert(ready1.empty());
    assert(impairment.pending_delayed_count() == 1);

    // At t0 + 55 ms, packet must be released
    auto ready2 = impairment.release_due_packets(t0 + 55'000'000ULL);
    assert(ready2.size() == 1);
    assert(ready2[0].data.size() == sizeof(payload));
    assert(ready2[0].data[0] == 1 && ready2[0].data[3] == 4);
    assert(impairment.pending_delayed_count() == 0);

    std::cout << "  -> Passed." << std::endl;
}

void test_channel_impairment_blackout() {
    std::cout << "[TEST] Running test_channel_impairment_blackout..." << std::endl;

    ChannelImpairment impairment;
    Endpoint ep2("127.0.0.1", 19992);
    Endpoint ep3("127.0.0.1", 19993);
    uint8_t payload[16] = {0};
    uint64_t t = 1000ULL;

    // Normal state: connected
    bool imm = false;
    assert(impairment.process_outgoing(payload, sizeof(payload), ep2, 1, 2, t, imm));
    assert(imm == true);

    // Sever bilateral link between 1 and 2
    impairment.set_link_state(1, 2, false);
    assert(!impairment.is_link_connected(1, 2));
    assert(!impairment.is_link_connected(2, 1)); // Symmetric check

    // Rover 1 -> Rover 2 must be dropped
    assert(!impairment.process_outgoing(payload, sizeof(payload), ep2, 1, 2, t, imm));

    // Rover 1 -> Rover 3 must still pass
    assert(impairment.process_outgoing(payload, sizeof(payload), ep3, 1, 3, t, imm));

    // Restore link
    impairment.set_link_state(1, 2, true);
    assert(impairment.process_outgoing(payload, sizeof(payload), ep2, 1, 2, t, imm));

    // Global blackout
    impairment.set_global_blackout(true);
    assert(!impairment.process_outgoing(payload, sizeof(payload), ep3, 1, 3, t, imm));
    impairment.set_global_blackout(false);
    assert(impairment.process_outgoing(payload, sizeof(payload), ep3, 1, 3, t, imm));

    std::cout << "  -> Passed." << std::endl;
}

void test_dtn_end_to_end_reliable_delivery() {
    std::cout << "[TEST] Running test_dtn_end_to_end_reliable_delivery..." << std::endl;

    RoverNodeConfig c1;
    c1.rover_id = 1;
    c1.listen_port = 20501;
    c1.listen_ip = "127.0.0.1";
    c1.seed_peers = { Endpoint("127.0.0.1", 20502) };
    c1.heartbeat_interval_ms = 50;

    RoverNodeConfig c2;
    c2.rover_id = 2;
    c2.listen_port = 20502;
    c2.listen_ip = "127.0.0.1";
    c2.seed_peers = { Endpoint("127.0.0.1", 20501) };
    c2.heartbeat_interval_ms = 50;

    RoverNode r1(c1);
    RoverNode r2(c2);

    assert(r1.start());
    assert(r2.start());

    std::vector<std::vector<uint8_t>> r2_received_payloads;
    r2.set_message_handler([&](const MessageFrame& frame, const Endpoint&) {
        r2_received_payloads.push_back(frame.payload);
    });

    // Exchange heartbeats to establish mutual peer discovery
    for (int i = 0; i < 5; ++i) {
        r1.poll_events(0);
        r2.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Verify mutual peer discovery
    auto p1_active = r1.get_peer_tracker().get_active_peers(0, 2'000'000'000ULL);
    auto p2_active = r2.get_peer_tracker().get_active_peers(0, 2'000'000'000ULL);
    assert(!p1_active.empty() && p1_active[0] == 2);
    assert(!p2_active.empty() && p2_active[0] == 1);

    // Enqueue DTN message from Rover 1 to Rover 2
    std::string test_payload = "LUNAR_MAP_KEYFRAME_ALPHA";
    std::vector<uint8_t> payload_bytes(test_payload.begin(), test_payload.end());

    uint64_t msg_id = r1.enqueue_dtn_message(2, MessageType::DataPayload, payload_bytes, PriorityLevel::High);
    assert(msg_id > 0);
    assert(r1.get_dtn_buffer().message_count() == 1);

    // Poll events: Rover 1 transmits DTN frame, Rover 2 receives and ACKs, Rover 1 receives ACK
    for (int step = 0; step < 10; ++step) {
        r1.poll_events(0);
        r2.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (r1.get_dtn_buffer().message_count() == 0 && !r2_received_payloads.empty()) {
            break;
        }
    }

    // Verify delivery
    assert(!r2_received_payloads.empty());
    std::string received_str(r2_received_payloads[0].begin(), r2_received_payloads[0].end());
    assert(received_str == test_payload);

    // Verify message removed from Rover 1 queue upon ACK receipt
    assert(r1.get_dtn_buffer().message_count() == 0);
    assert(r1.get_wire_stats().acks_received >= 1);
    assert(r1.get_wire_stats().dtn_messages_delivered >= 1);

    // Verify wire byte accounting invariants
    auto w1 = r1.get_wire_stats();
    assert(w1.payload_bytes_sent == payload_bytes.size());
    assert(w1.header_bytes_sent == MessageFrame::HEADER_SIZE);
    assert(w1.framing_bytes_sent == c1.transport_framing_bytes);
    assert(w1.data_wire_bytes_sent == w1.payload_bytes_sent + w1.header_bytes_sent + w1.framing_bytes_sent);
    assert(w1.total_wire_bytes_sent == w1.data_wire_bytes_sent + w1.ack_wire_bytes_sent);

    auto w2 = r2.get_wire_stats();
    assert(w2.acks_sent >= 1);
    assert(w2.ack_wire_bytes_sent == (24 + MessageFrame::HEADER_SIZE + c2.transport_framing_bytes));
    assert(w2.total_wire_bytes_sent == w2.data_wire_bytes_sent + w2.ack_wire_bytes_sent);

    r1.stop();
    r2.stop();
    std::cout << "  -> Passed." << std::endl;
}

void test_dtn_store_and_forward_blackout_recovery() {
    std::cout << "[TEST] Running test_dtn_store_and_forward_blackout_recovery..." << std::endl;

    RoverNodeConfig c1;
    c1.rover_id = 1;
    c1.listen_port = 20511;
    c1.listen_ip = "127.0.0.1";
    c1.seed_peers = { Endpoint("127.0.0.1", 20512) };

    RoverNodeConfig c2;
    c2.rover_id = 2;
    c2.listen_port = 20512;
    c2.listen_ip = "127.0.0.1";
    c2.seed_peers = { Endpoint("127.0.0.1", 20511) };

    RoverNode r1(c1);
    RoverNode r2(c2);

    assert(r1.start());
    assert(r2.start());

    std::vector<std::string> r2_messages;
    r2.set_message_handler([&](const MessageFrame& frame, const Endpoint&) {
        r2_messages.push_back(std::string(frame.payload.begin(), frame.payload.end()));
    });

    // 1. Induce communication blackout before enqueuing messages
    r1.get_channel_impairment().set_link_state(1, 2, false);

    // 2. Enqueue 3 store-and-forward messages during blackout
    uint64_t id1 = r1.enqueue_dtn_message(2, MessageType::DataPayload, {'M', 'S', 'G', '_', '1'});
    uint64_t id2 = r1.enqueue_dtn_message(2, MessageType::DataPayload, {'M', 'S', 'G', '_', '2'});
    uint64_t id3 = r1.enqueue_dtn_message(2, MessageType::DataPayload, {'M', 'S', 'G', '_', '3'});
    assert(id1 > 0 && id2 > 0 && id3 > 0);

    // Buffer holds all 3 messages
    assert(r1.get_dtn_buffer().message_count() == 3);

    // Poll while disconnected: messages must NOT be sent or dropped
    for (int i = 0; i < 3; ++i) {
        r1.poll_events(0);
        r2.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    assert(r1.get_dtn_buffer().message_count() == 3);
    assert(r2_messages.empty()); // Nothing received yet

    // 3. Restore communication link! Store-and-forward drain should occur.
    r1.get_channel_impairment().set_link_state(1, 2, true);

    for (int i = 0; i < 20; ++i) {
        r1.poll_events(0);
        r2.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
        if (r2_messages.size() == 3 && r1.get_dtn_buffer().message_count() == 0) {
            break;
        }
    }

    // Verify all 3 messages delivered successfully in order
    assert(r2_messages.size() == 3);
    assert(r2_messages[0] == "MSG_1");
    assert(r2_messages[1] == "MSG_2");
    assert(r2_messages[2] == "MSG_3");

    // All acknowledged and freed
    assert(r1.get_dtn_buffer().message_count() == 0);
    assert(r1.get_wire_stats().dtn_messages_delivered == 3);

    r1.stop();
    r2.stop();
    std::cout << "  -> Passed." << std::endl;
}

void test_dtn_retransmission_and_deduplication() {
    std::cout << "[TEST] Running test_dtn_retransmission_and_deduplication..." << std::endl;

    RoverNodeConfig c1;
    c1.rover_id = 1;
    c1.listen_port = 20521;
    c1.listen_ip = "127.0.0.1";
    c1.seed_peers = { Endpoint("127.0.0.1", 20522) };
    c1.dtn_config.initial_retry_delay_ns = 30'000'000ULL; // 30 ms retry timer

    RoverNodeConfig c2;
    c2.rover_id = 2;
    c2.listen_port = 20522;
    c2.listen_ip = "127.0.0.1";
    c2.seed_peers = { Endpoint("127.0.0.1", 20521) };

    RoverNode r1(c1);
    RoverNode r2(c2);

    assert(r1.start());
    assert(r2.start());

    int r2_delivery_count = 0;
    r2.set_message_handler([&](const MessageFrame&, const Endpoint&) {
        r2_delivery_count++;
    });

    // Induce temporary 100% packet loss to drop initial transmission
    r1.get_channel_impairment().set_loss_rate(1.0);

    uint64_t msg_id = r1.enqueue_dtn_message(2, MessageType::DataPayload, {'R', 'E', 'T', 'R', 'Y', '_', '1'});
    assert(msg_id > 0);

    // First attempt transmitted by DTN but dropped by channel loss
    r1.poll_events(0);
    assert(r2_delivery_count == 0);
    assert(r1.get_dtn_buffer().message_count() == 1);

    // Wait past retry timeout (40 ms) and restore channel
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    r1.get_channel_impairment().set_loss_rate(0.0);

    // Retransmission should now succeed
    for (int i = 0; i < 15; ++i) {
        r1.poll_events(0);
        r2.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (r2_delivery_count == 1 && r1.get_dtn_buffer().message_count() == 0) {
            break;
        }
    }

    assert(r2_delivery_count == 1);
    assert(r1.get_dtn_buffer().message_count() == 0);
    assert(r1.get_wire_stats().dtn_retransmissions >= 1);

    r1.stop();
    r2.stop();
    std::cout << "  -> Passed." << std::endl;
}

int main() {
    std::cout << "==================================================" << std::endl;
    std::cout << "  AstroMesh-X: Delay-Tolerant Network & UDP Tests" << std::endl;
    std::cout << "==================================================" << std::endl;

    test_channel_impairment_loss();
    test_channel_impairment_delay();
    test_channel_impairment_blackout();
    test_dtn_end_to_end_reliable_delivery();
    test_dtn_store_and_forward_blackout_recovery();
    test_dtn_retransmission_and_deduplication();

    std::cout << "==================================================" << std::endl;
    std::cout << "  All DTN & UDP Network Tests PASSED successfully!" << std::endl;
    std::cout << "==================================================" << std::endl;
    return 0;
}
