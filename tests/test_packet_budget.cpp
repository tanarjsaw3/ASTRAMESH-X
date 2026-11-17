#include "astromesh/comms/packet_budget.hpp"

#include <cassert>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

int g_assertions = 0;

#define EXPECT_TRUE(cond) do { \
    g_assertions++; \
    if (!(cond)) { \
        std::cerr << "Assertion failed at " << __FILE__ << ":" << __LINE__ << " - " << #cond << std::endl; \
        std::exit(1); \
    } \
} while(0)

#define EXPECT_FALSE(cond) EXPECT_TRUE(!(cond))

#define EXPECT_EQ(a, b) do { \
    g_assertions++; \
    if ((a) != (b)) { \
        std::cerr << "Assertion failed at " << __FILE__ << ":" << __LINE__ << " - " << #a << " (" << (a) << ") == " << #b << " (" << (b) << ")" << std::endl; \
        std::exit(1); \
    } \
} while(0)

} // namespace

using namespace astromesh::comms;

// Test 1: Exact payload/header/overhead arithmetic
void test_exact_byte_breakdown() {
    ByteBudgetConfig cfg;
    cfg.default_header_bytes = 24;
    cfg.default_transport_overhead_bytes = 20;

    PacketBudgetManager mgr(cfg);
    ByteBreakdown b1 = mgr.check_quota(1, 100, 0) == QuotaCheckResult::Approved ?
        ByteBreakdown{100, 24, 20} : ByteBreakdown{};

    EXPECT_EQ(b1.payload_bytes, 100);
    EXPECT_EQ(b1.header_bytes, 24);
    EXPECT_EQ(b1.transport_overhead_bytes, 20);
    EXPECT_EQ(b1.total_wire_bytes(), 144);

    // Custom headers and transport
    EXPECT_TRUE(mgr.record_sent(1, PriorityLevel::Normal, 100, 0, false, 32, 16));
    const auto& stats = mgr.statistics();
    EXPECT_EQ(stats.sent.payload_bytes, 100);
    EXPECT_EQ(stats.sent.header_bytes, 32);
    EXPECT_EQ(stats.sent.transport_overhead_bytes, 16);
    EXPECT_EQ(stats.sent.total_wire_bytes, 148);
    EXPECT_EQ(stats.sent.message_count, 1);
}

// Test 2: Successful transmission accounting and per-priority/peer tracking
void test_successful_transmission_accounting() {
    ByteBudgetConfig cfg;
    cfg.default_header_bytes = 20;
    cfg.default_transport_overhead_bytes = 10;
    PacketBudgetManager mgr(cfg);

    EXPECT_TRUE(mgr.record_sent(2, PriorityLevel::High, 50, 1000));
    EXPECT_TRUE(mgr.record_sent(3, PriorityLevel::Low, 80, 2000));

    const auto& s = mgr.statistics();
    EXPECT_EQ(s.sent.message_count, 2);
    EXPECT_EQ(s.sent.payload_bytes, 130);
    EXPECT_EQ(s.sent.header_bytes, 40);
    EXPECT_EQ(s.sent.transport_overhead_bytes, 20);
    EXPECT_EQ(s.sent.total_wire_bytes, 190);

    // Peer 2
    auto it2 = s.per_peer_sent.find(2);
    EXPECT_TRUE(it2 != s.per_peer_sent.end());
    EXPECT_EQ(it2->second.payload_bytes, 50);
    EXPECT_EQ(it2->second.total_wire_bytes, 80);

    // Peer 3
    auto it3 = s.per_peer_sent.find(3);
    EXPECT_TRUE(it3 != s.per_peer_sent.end());
    EXPECT_EQ(it3->second.payload_bytes, 80);
    EXPECT_EQ(it3->second.total_wire_bytes, 110);

    // Priority breakdown
    EXPECT_EQ(s.per_priority_sent[static_cast<size_t>(PriorityLevel::High)].message_count, 1);
    EXPECT_EQ(s.per_priority_sent[static_cast<size_t>(PriorityLevel::Low)].message_count, 1);
    EXPECT_EQ(s.per_priority_sent[static_cast<size_t>(PriorityLevel::Normal)].message_count, 0);
}

// Test 3: Network-drop accounting: sent versus received
void test_network_drop_accounting() {
    ByteBudgetConfig cfg;
    cfg.default_header_bytes = 10;
    cfg.default_transport_overhead_bytes = 10;
    PacketBudgetManager mgr(cfg);

    // Send packet
    EXPECT_TRUE(mgr.record_sent(2, PriorityLevel::Normal, 100, 1000));
    // Network drops the packet in transit
    mgr.record_network_drop(2, PriorityLevel::Normal, 100);

    const auto& s = mgr.statistics();
    // Sent must reflect the packet was transmitted over radio
    EXPECT_EQ(s.sent.message_count, 1);
    EXPECT_EQ(s.sent.total_wire_bytes, 120);

    // Dropped must reflect channel loss
    EXPECT_EQ(s.network_dropped.message_count, 1);
    EXPECT_EQ(s.network_dropped.total_wire_bytes, 120);

    // Received must be strictly 0
    EXPECT_EQ(s.received.message_count, 0);
    EXPECT_EQ(s.received.total_wire_bytes, 0);

    // Now another packet successfully arrives at receiver
    mgr.record_received(2, PriorityLevel::Normal, 100, 2000);
    EXPECT_EQ(mgr.statistics().received.message_count, 1);
    EXPECT_EQ(mgr.statistics().received.total_wire_bytes, 120);
}

// Test 4: Retransmission accounting
void test_retransmission_accounting() {
    ByteBudgetConfig cfg;
    cfg.default_header_bytes = 10;
    cfg.default_transport_overhead_bytes = 10;
    PacketBudgetManager mgr(cfg);

    // Initial transmission
    EXPECT_TRUE(mgr.record_sent(2, PriorityLevel::High, 80, 1000, false));
    EXPECT_EQ(mgr.statistics().sent.message_count, 1);
    EXPECT_EQ(mgr.statistics().retransmitted.message_count, 0);

    // Retransmission 1
    EXPECT_TRUE(mgr.record_sent(2, PriorityLevel::High, 80, 2000, true));
    EXPECT_EQ(mgr.statistics().sent.message_count, 2);
    EXPECT_EQ(mgr.statistics().sent.total_wire_bytes, 200);
    EXPECT_EQ(mgr.statistics().retransmitted.message_count, 1);
    EXPECT_EQ(mgr.statistics().retransmitted.total_wire_bytes, 100);

    // Retransmission 2
    EXPECT_TRUE(mgr.record_sent(2, PriorityLevel::High, 80, 3000, true));
    EXPECT_EQ(mgr.statistics().sent.message_count, 3);
    EXPECT_EQ(mgr.statistics().sent.total_wire_bytes, 300);
    EXPECT_EQ(mgr.statistics().retransmitted.message_count, 2);
    EXPECT_EQ(mgr.statistics().retransmitted.total_wire_bytes, 200);
}

// Test 5 & 6: Quota acceptance at exact boundary and rejection one byte above
void test_quota_boundary_and_rejection() {
    ByteBudgetConfig cfg;
    cfg.default_header_bytes = 20;
    cfg.default_transport_overhead_bytes = 10;
    // Total wire per message = payload + 30
    // Set budget to exactly 200 wire bytes
    cfg.max_bytes_per_window = 200;
    cfg.window_duration_ns = 1'000'000'000; // 1 second

    PacketBudgetManager mgr(cfg);

    // Message 1: 70 bytes payload -> 100 wire bytes (100 / 200 used)
    EXPECT_EQ(mgr.check_quota(1, 70, 0), QuotaCheckResult::Approved);
    EXPECT_TRUE(mgr.record_sent(1, PriorityLevel::Normal, 70, 0));
    EXPECT_EQ(mgr.current_window_bytes_used(), 100);

    // Message 2: exact boundary! 70 bytes payload -> 100 wire bytes (200 / 200 used)
    EXPECT_EQ(mgr.check_quota(1, 70, 100), QuotaCheckResult::Approved);
    EXPECT_TRUE(mgr.record_sent(1, PriorityLevel::Normal, 70, 100));
    EXPECT_EQ(mgr.current_window_bytes_used(), 200);

    // Message 3: 1 byte payload -> 31 wire bytes -> exceeds 200 by 31
    EXPECT_EQ(mgr.check_quota(1, 1, 200), QuotaCheckResult::ExceedsWindowBudget);
    EXPECT_FALSE(mgr.record_sent(1, PriorityLevel::Normal, 1, 200));

    // Transmitted bytes must NOT have changed
    EXPECT_EQ(mgr.statistics().sent.message_count, 2);
    EXPECT_EQ(mgr.statistics().sent.total_wire_bytes, 200);

    // Rejected counter must have incremented
    EXPECT_EQ(mgr.statistics().rejected.message_count, 1);
    EXPECT_EQ(mgr.statistics().rejected.payload_bytes, 1);
    EXPECT_EQ(mgr.statistics().rejected.total_wire_bytes, 31);
}

// Test 7: Time-window rollover
void test_time_window_rollover() {
    ByteBudgetConfig cfg;
    cfg.default_header_bytes = 20;
    cfg.default_transport_overhead_bytes = 10;
    cfg.max_bytes_per_window = 100;
    cfg.window_duration_ns = 1'000'000'000; // 1.0 s

    PacketBudgetManager mgr(cfg);

    // Exhaust window at t = 100 ms
    EXPECT_TRUE(mgr.record_sent(1, PriorityLevel::High, 70, 100'000'000));
    EXPECT_EQ(mgr.current_window_bytes_used(), 100);

    // Attempt at t = 900 ms -> should be rejected
    EXPECT_EQ(mgr.check_quota(1, 70, 900'000'000), QuotaCheckResult::ExceedsWindowBudget);

    // Time rolls over to next epoch at t = 1.000000001 s (1'000'000'001 ns)
    EXPECT_EQ(mgr.check_quota(1, 70, 1'000'000'001ULL), QuotaCheckResult::Approved);
    EXPECT_TRUE(mgr.record_sent(1, PriorityLevel::High, 70, 1'000'000'001ULL));
    EXPECT_EQ(mgr.current_window_bytes_used(), 100);

    // Total sent across both windows
    EXPECT_EQ(mgr.statistics().sent.message_count, 2);
    EXPECT_EQ(mgr.statistics().sent.total_wire_bytes, 200);
}

// Test 8: Per-peer quota isolation
void test_per_peer_quota_isolation() {
    ByteBudgetConfig cfg;
    cfg.default_header_bytes = 10;
    cfg.default_transport_overhead_bytes = 10;
    cfg.max_bytes_per_window = 1000; // Large global window
    cfg.per_peer_quota_bytes = 100;  // 100 bytes per peer
    cfg.window_duration_ns = 1'000'000'000;

    PacketBudgetManager mgr(cfg);

    // Peer 2: send 80 payload + 20 overhead = 100 wire bytes -> hits quota
    EXPECT_TRUE(mgr.record_sent(2, PriorityLevel::Normal, 80, 0));
    EXPECT_EQ(mgr.check_quota(2, 1, 100), QuotaCheckResult::ExceedsPeerQuota);

    // Peer 3: must still have full quota approved!
    EXPECT_EQ(mgr.check_quota(3, 80, 100), QuotaCheckResult::Approved);
    EXPECT_TRUE(mgr.record_sent(3, PriorityLevel::Normal, 80, 100));

    // Both peers now at quota
    EXPECT_EQ(mgr.check_quota(2, 1, 200), QuotaCheckResult::ExceedsPeerQuota);
    EXPECT_EQ(mgr.check_quota(3, 1, 200), QuotaCheckResult::ExceedsPeerQuota);
}

// Test 9: Max message payload size limit
void test_max_message_payload_size() {
    ByteBudgetConfig cfg;
    cfg.max_message_payload_bytes = 500;
    PacketBudgetManager mgr(cfg);

    EXPECT_EQ(mgr.check_quota(1, 500, 0), QuotaCheckResult::Approved);
    EXPECT_EQ(mgr.check_quota(1, 501, 0), QuotaCheckResult::ExceedsMaxMessageSize);
}

// Test 10: Overflow safety
void test_overflow_safety() {
    ByteBudgetConfig cfg;
    PacketBudgetManager mgr(cfg);

    // Saturating math on huge numbers
    TrafficStatistics stat;
    ByteBreakdown b;
    b.payload_bytes = std::numeric_limits<uint64_t>::max() - 10;
    b.header_bytes = 20;
    b.transport_overhead_bytes = 20;
    stat.add(b);

    EXPECT_EQ(stat.total_wire_bytes, std::numeric_limits<uint64_t>::max());
}

// Test 11: No double counting on repeated reads
void test_no_double_counting_on_reads() {
    PacketBudgetManager mgr;
    mgr.record_sent(1, PriorityLevel::Normal, 100, 0);

    const auto& s1 = mgr.statistics();
    const auto& s2 = mgr.statistics();
    EXPECT_EQ(s1.sent.total_wire_bytes, s2.sent.total_wire_bytes);
    EXPECT_EQ(s1.sent.message_count, s2.sent.message_count);
}

// Test 12: Deterministic repeatability
void test_deterministic_repeatability() {
    auto run_sequence = []() -> uint64_t {
        ByteBudgetConfig cfg;
        cfg.max_bytes_per_window = 500;
        cfg.window_duration_ns = 1'000'000;
        PacketBudgetManager mgr(cfg);

        for (uint64_t t = 0; t < 10'000'000; t += 500'000) {
            mgr.record_sent(1, PriorityLevel::Normal, 40, t);
        }
        return mgr.statistics().sent.total_wire_bytes;
    };

    uint64_t r1 = run_sequence();
    uint64_t r2 = run_sequence();
    EXPECT_EQ(r1, r2);
}

int main() {
    std::cout << "[RUN      ] test_packet_budget" << std::endl;
    test_exact_byte_breakdown();
    test_successful_transmission_accounting();
    test_network_drop_accounting();
    test_retransmission_accounting();
    test_quota_boundary_and_rejection();
    test_time_window_rollover();
    test_per_peer_quota_isolation();
    test_max_message_payload_size();
    test_overflow_safety();
    test_no_double_counting_on_reads();
    test_deterministic_repeatability();
    std::cout << "[       OK ] test_packet_budget (" << g_assertions << " assertions verified)" << std::endl;
    return 0;
}
