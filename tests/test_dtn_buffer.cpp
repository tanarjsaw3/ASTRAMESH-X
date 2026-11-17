#include "astromesh/comms/dtn_buffer.hpp"

#include <cassert>
#include <iostream>
#include <string>
#include <vector>

using namespace astromesh::comms;

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

DtnMessage make_msg(uint64_t id, PriorityLevel prio, uint64_t ttl_ns, size_t payload_len = 10, uint16_t sender = 1, uint16_t target = 2) {
    DtnMessage m;
    m.message_id = id;
    m.sender_rover_id = sender;
    m.target_rover_id = target;
    m.priority = prio;
    m.ttl_ns = ttl_ns;
    m.payload.assign(payload_len, static_cast<uint8_t>(id & 0xFF));
    return m;
}

} // namespace

// Test 1: FIFO order within equal priority
void test_fifo_within_equal_priority() {
    DtnBuffer buf;
    uint64_t now = 1000;

    EXPECT_EQ(buf.enqueue(make_msg(1, PriorityLevel::Normal, 1'000'000), now), EnqueueResult::Success);
    EXPECT_EQ(buf.enqueue(make_msg(2, PriorityLevel::Normal, 1'000'000), now), EnqueueResult::Success);
    EXPECT_EQ(buf.enqueue(make_msg(3, PriorityLevel::Normal, 1'000'000), now), EnqueueResult::Success);

    auto m1 = buf.dequeue_next_eligible(now);
    EXPECT_TRUE(m1.has_value());
    EXPECT_EQ(m1->message_id, 1ULL);

    auto m2 = buf.dequeue_next_eligible(now);
    EXPECT_TRUE(m2.has_value());
    EXPECT_EQ(m2->message_id, 2ULL);

    auto m3 = buf.dequeue_next_eligible(now);
    EXPECT_TRUE(m3.has_value());
    EXPECT_EQ(m3->message_id, 3ULL);
}

// Test 2: High/Normal/Low priority selection
void test_priority_selection() {
    DtnQueueConfig cfg;
    cfg.fairness_policy = FairnessPolicy::StrictPriority;
    DtnBuffer buf(cfg);

    uint64_t now = 0;
    EXPECT_EQ(buf.enqueue(make_msg(10, PriorityLevel::Low, 1'000'000), now), EnqueueResult::Success);
    EXPECT_EQ(buf.enqueue(make_msg(20, PriorityLevel::High, 1'000'000), now), EnqueueResult::Success);
    EXPECT_EQ(buf.enqueue(make_msg(30, PriorityLevel::Normal, 1'000'000), now), EnqueueResult::Success);

    // High first
    auto m1 = buf.dequeue_next_eligible(now);
    EXPECT_TRUE(m1.has_value());
    EXPECT_EQ(m1->message_id, 20ULL);

    // Normal second
    auto m2 = buf.dequeue_next_eligible(now);
    EXPECT_TRUE(m2.has_value());
    EXPECT_EQ(m2->message_id, 30ULL);

    // Low third
    auto m3 = buf.dequeue_next_eligible(now);
    EXPECT_TRUE(m3.has_value());
    EXPECT_EQ(m3->message_id, 10ULL);
}

// Test 3: Low-priority fairness under sustained high-priority arrivals (PriorityAging)
void test_priority_aging_fairness() {
    DtnQueueConfig cfg;
    cfg.fairness_policy = FairnessPolicy::PriorityAging;
    cfg.aging_interval_ns = 100'000'000; // 100 ms promotion interval
    DtnBuffer buf(cfg);

    uint64_t t = 0;
    // Low priority message enqueued at t = 0
    EXPECT_EQ(buf.enqueue(make_msg(1, PriorityLevel::Low, 1'000'000'000), t), EnqueueResult::Success);

    // Continuous stream of high priority messages arriving
    for (uint64_t i = 10; i < 15; ++i) {
        EXPECT_EQ(buf.enqueue(make_msg(i, PriorityLevel::High, 1'000'000'000), t), EnqueueResult::Success);
    }

    // At t = 0, High priority is picked over Low
    auto m_t0 = buf.dequeue_next_eligible(t);
    EXPECT_TRUE(m_t0.has_value());
    EXPECT_EQ(m_t0->priority, PriorityLevel::High);

    // Advance time past 2 aging intervals: t = 250 ms
    // Message 1 (Low) promotes Low -> Normal (after 100ms) -> High (after 200ms)
    // Since Message 1 is now effectively High and was enqueued earliest (t = 0), it must be served next!
    t = 250'000'000;
    auto m_t250 = buf.dequeue_next_eligible(t);
    EXPECT_TRUE(m_t250.has_value());
    EXPECT_EQ(m_t250->message_id, 1ULL); // Anti-starvation verified!
}

// Test 4: Exact TTL expiration boundary
void test_exact_ttl_expiration_boundary() {
    DtnBuffer buf;
    uint64_t enqueue_t = 1000;
    uint64_t ttl = 500; // Expiry = 1500

    EXPECT_EQ(buf.enqueue(make_msg(1, PriorityLevel::Normal, ttl), enqueue_t), EnqueueResult::Success);

    // 1 ns before expiry: must NOT be expired
    auto m1 = buf.peek_next_eligible(1499);
    EXPECT_TRUE(m1.has_value());
    EXPECT_EQ(m1->message_id, 1ULL);

    // Exact expiry boundary: now >= expiry_time_ns (1500 >= 1500) -> expired!
    auto m2 = buf.peek_next_eligible(1500);
    EXPECT_FALSE(m2.has_value());

    EXPECT_EQ(buf.purge_expired(1500), 1ULL);
    EXPECT_EQ(buf.statistics().messages_expired, 1ULL);
    EXPECT_EQ(buf.message_count(), 0ULL);
}

// Test 5: Already-expired enqueue
void test_already_expired_enqueue() {
    DtnBuffer buf;
    DtnMessage m = make_msg(1, PriorityLevel::Normal, 0); // TTL = 0
    EXPECT_EQ(buf.enqueue(m, 1000), EnqueueResult::AlreadyExpired);
    EXPECT_EQ(buf.message_count(), 0ULL);
}

// Test 6: Expiration freeing queue capacity
void test_expiration_frees_capacity() {
    DtnQueueConfig cfg;
    cfg.max_queue_count = 1;
    DtnBuffer buf(cfg);

    // Enqueue message expiring in 100 ns
    EXPECT_EQ(buf.enqueue(make_msg(1, PriorityLevel::Normal, 100), 0), EnqueueResult::Success);
    EXPECT_EQ(buf.message_count(), 1ULL);

    // Enqueue second message at t = 50 ns -> rejected (buffer full)
    EXPECT_EQ(buf.enqueue(make_msg(2, PriorityLevel::Normal, 100), 50), EnqueueResult::CapacityCountExceeded);

    // Enqueue second message at t = 100 ns -> message 1 purged, message 2 admitted!
    EXPECT_EQ(buf.enqueue(make_msg(2, PriorityLevel::Normal, 100), 100), EnqueueResult::Success);
    EXPECT_EQ(buf.message_count(), 1ULL);
    EXPECT_EQ(buf.statistics().messages_expired, 1ULL);
}

// Test 7 & 8: Message count & payload byte capacity limits
void test_capacity_limits() {
    DtnQueueConfig cfg;
    cfg.max_queue_count = 2;
    cfg.max_payload_bytes = 50;
    DtnBuffer buf(cfg);

    // Message 1: 30 bytes
    EXPECT_EQ(buf.enqueue(make_msg(1, PriorityLevel::Normal, 10'000, 30), 0), EnqueueResult::Success);
    // Message 2: 30 bytes -> exceeds 50 bytes!
    EXPECT_EQ(buf.enqueue(make_msg(2, PriorityLevel::Normal, 10'000, 30), 0), EnqueueResult::CapacityBytesExceeded);
    // Message 2: 20 bytes -> fits exactly (50/50 bytes)
    EXPECT_EQ(buf.enqueue(make_msg(2, PriorityLevel::Normal, 10'000, 20), 0), EnqueueResult::Success);

    // Message 3: 0 bytes -> exceeds count (2/2 messages)
    EXPECT_EQ(buf.enqueue(make_msg(3, PriorityLevel::Normal, 10'000, 0), 0), EnqueueResult::CapacityCountExceeded);
}

// Test 9: Oversized message rejection
void test_oversized_message_rejection() {
    DtnQueueConfig cfg;
    cfg.max_message_payload_bytes = 100;
    DtnBuffer buf(cfg);

    EXPECT_EQ(buf.enqueue(make_msg(1, PriorityLevel::Normal, 10'000, 101), 0), EnqueueResult::PayloadTooLarge);
    EXPECT_EQ(buf.enqueue(make_msg(1, PriorityLevel::Normal, 10'000, 100), 0), EnqueueResult::Success);
}

// Test 10 & 11: Retry timing, exponential backoff, and cap
void test_retry_timing_and_backoff() {
    DtnQueueConfig cfg;
    cfg.initial_retry_delay_ns = 100'000'000; // 100 ms
    cfg.backoff_multiplier = 2.0;
    cfg.max_retry_delay_ns = 500'000'000;     // 500 ms cap
    cfg.max_attempts = 10;
    DtnBuffer buf(cfg);

    uint64_t t = 0;
    EXPECT_EQ(buf.enqueue(make_msg(1, PriorityLevel::Normal, 10'000'000'000ULL), t), EnqueueResult::Success);

    // Attempt 1 at t = 0
    auto m1 = buf.dequeue_next_eligible(t);
    EXPECT_TRUE(m1.has_value());
    EXPECT_EQ(m1->attempt_count, 1ULL);

    // Immediately after dequeue, message is waiting for backoff (100 ms)
    EXPECT_FALSE(buf.peek_next_eligible(t).has_value());
    EXPECT_FALSE(buf.peek_next_eligible(50'000'000).has_value());

    // Fails at t = 100 ms -> next retry at 100 ms + backoff (100ms * 2.0 = 200 ms) -> t = 300 ms
    t = 100'000'000;
    buf.mark_transmission_attempt(1, t, false);
    EXPECT_FALSE(buf.peek_next_eligible(299'000'000).has_value());
    EXPECT_TRUE(buf.peek_next_eligible(300'000'000).has_value());

    // Attempt 2 dequeued at t = 300 ms
    t = 300'000'000;
    auto m2 = buf.dequeue_next_eligible(t);
    EXPECT_TRUE(m2.has_value());
    EXPECT_EQ(m2->attempt_count, 2ULL);

    // Fails again at t = 300 ms -> backoff multiplier 2.0^2 * 100ms = 400 ms -> t = 700 ms
    buf.mark_transmission_attempt(1, t, false);
    EXPECT_FALSE(buf.peek_next_eligible(699'000'000).has_value());
    EXPECT_TRUE(buf.peek_next_eligible(700'000'000).has_value());

    // Attempt 3 dequeued at t = 700 ms -> backoff 800 ms capped to max 500 ms!
    t = 700'000'000;
    auto m3 = buf.dequeue_next_eligible(t);
    EXPECT_TRUE(m3.has_value());
    EXPECT_EQ(m3->attempt_count, 3ULL);

    buf.mark_transmission_attempt(1, t, false);
    // Next eligible at 700 ms + 500 ms (cap) = 1200 ms
    EXPECT_FALSE(buf.peek_next_eligible(1'199'000'000).has_value());
    EXPECT_TRUE(buf.peek_next_eligible(1'200'000'000).has_value());
}

// Test 12: Max retry attempts
void test_max_retry_attempts() {
    DtnQueueConfig cfg;
    cfg.initial_retry_delay_ns = 10;
    cfg.max_attempts = 2;
    DtnBuffer buf(cfg);

    uint64_t t = 0;
    EXPECT_EQ(buf.enqueue(make_msg(1, PriorityLevel::Normal, 1'000'000), t), EnqueueResult::Success);

    // Attempt 1
    auto m1 = buf.dequeue_next_eligible(t);
    EXPECT_TRUE(m1.has_value());
    EXPECT_EQ(m1->attempt_count, 1ULL);
    buf.mark_transmission_attempt(1, t, false);

    // Attempt 2
    t = 50;
    auto m2 = buf.dequeue_next_eligible(t);
    EXPECT_TRUE(m2.has_value());
    EXPECT_EQ(m2->attempt_count, 2ULL);

    // Fails on attempt 2 (max_attempts = 2) -> dropped, exhausted!
    buf.mark_transmission_attempt(1, t, false);
    EXPECT_EQ(buf.message_count(), 0ULL);
    EXPECT_EQ(buf.statistics().retries_exhausted, 1ULL);
}

// Test 13: Retry suppression after TTL expiration
void test_retry_suppression_after_ttl_expiration() {
    DtnQueueConfig cfg;
    cfg.initial_retry_delay_ns = 200;
    cfg.max_attempts = 5;
    DtnBuffer buf(cfg);

    // TTL = 150 ns
    EXPECT_EQ(buf.enqueue(make_msg(1, PriorityLevel::Normal, 150), 0), EnqueueResult::Success);

    auto m = buf.dequeue_next_eligible(0);
    EXPECT_TRUE(m.has_value());

    // Failure scheduled next retry for 0 + 200 = 200 ns, but TTL is 150 ns!
    buf.mark_transmission_attempt(1, 0, false);
    EXPECT_EQ(buf.message_count(), 0ULL); // Dropped immediately as expired
    EXPECT_EQ(buf.statistics().messages_expired, 1ULL);
}

// Test 14 & 15: Partition and reconnect behavior
void test_partition_and_reconnect() {
    DtnBuffer buf;
    bool link_up = false;

    auto link_checker = [&](uint16_t target) -> bool {
        if (target == 2) return link_up;
        return true;
    };

    uint64_t t = 0;
    // Enqueue message for Rover 2 with TTL = 1000 ns
    EXPECT_EQ(buf.enqueue(make_msg(1, PriorityLevel::Normal, 1000, 10, 1, 2), t), EnqueueResult::Success);

    // Link is down: dequeue must skip Rover 2
    EXPECT_FALSE(buf.dequeue_next_eligible(t, link_checker).has_value());
    EXPECT_EQ(buf.message_count(), 1ULL); // Retained in buffer

    // Advance time during partition (t = 500 ns)
    t = 500;
    EXPECT_FALSE(buf.dequeue_next_eligible(t, link_checker).has_value());
    EXPECT_EQ(buf.message_count(), 1ULL);

    // Reconnection! link_up = true
    link_up = true;
    auto m = buf.dequeue_next_eligible(t, link_checker);
    EXPECT_TRUE(m.has_value());
    EXPECT_EQ(m->message_id, 1ULL);

    // Test long partition where message expires during partition
    link_up = false;
    EXPECT_EQ(buf.enqueue(make_msg(2, PriorityLevel::Normal, 200, 10, 1, 2), t), EnqueueResult::Success);
    t += 300; // Now t = 800 ns, message 2 has expired (500 + 200 = 700)
    link_up = true; // Reconnected after expiry
    EXPECT_FALSE(buf.dequeue_next_eligible(t, link_checker).has_value());
    EXPECT_EQ(buf.statistics().messages_expired, 1ULL);
}

// Test 16 & 17: Duplicate suppression and bounded cache
void test_duplicate_suppression_and_bounded_cache() {
    DtnQueueConfig cfg;
    cfg.dedup_cache_capacity = 2; // Cache holds max 2 IDs
    cfg.dedup_cache_ttl_ns = 500;
    DtnBuffer buf(cfg);

    uint64_t t = 0;
    EXPECT_EQ(buf.enqueue(make_msg(101, PriorityLevel::Normal, 1000), t), EnqueueResult::Success);

    // Immediate duplicate
    EXPECT_EQ(buf.enqueue(make_msg(101, PriorityLevel::Normal, 1000), t), EnqueueResult::DuplicateIgnored);
    EXPECT_EQ(buf.statistics().duplicates_suppressed, 1ULL);

    // Insert message 102 and 103 -> pushes 101 out of cache (capacity = 2)
    EXPECT_EQ(buf.enqueue(make_msg(102, PriorityLevel::Normal, 1000), t), EnqueueResult::Success);
    EXPECT_EQ(buf.enqueue(make_msg(103, PriorityLevel::Normal, 1000), t), EnqueueResult::Success);

    // Message 101 was evicted from dedup cache -> can be enqueued again
    EXPECT_EQ(buf.enqueue(make_msg(101, PriorityLevel::Normal, 1000), t), EnqueueResult::Success);

    // Duplicate cache TTL expiration
    t = 600; // Past 500 ns cache TTL
    EXPECT_EQ(buf.enqueue(make_msg(103, PriorityLevel::Normal, 1000), t), EnqueueResult::Success);
}

// Test 18: Acknowledgement handling
void test_acknowledgement() {
    DtnBuffer buf;
    uint64_t t = 100;

    EXPECT_EQ(buf.enqueue(make_msg(5, PriorityLevel::Normal, 10'000), t), EnqueueResult::Success);
    EXPECT_EQ(buf.message_count(), 1ULL);

    // Acknowledge unknown ID
    EXPECT_EQ(buf.acknowledge(999, t), AckResult::NotFound);

    // Acknowledge valid ID
    EXPECT_EQ(buf.acknowledge(5, t), AckResult::Acknowledged);
    EXPECT_EQ(buf.message_count(), 0ULL);
    EXPECT_EQ(buf.statistics().messages_delivered, 1ULL);
}

// Test 19: Priority eviction under capacity
void test_priority_eviction() {
    DtnQueueConfig cfg;
    cfg.max_queue_count = 2;
    cfg.enable_priority_eviction = true;
    DtnBuffer buf(cfg);

    uint64_t t = 0;
    EXPECT_EQ(buf.enqueue(make_msg(1, PriorityLevel::Low, 10'000), t), EnqueueResult::Success);
    EXPECT_EQ(buf.enqueue(make_msg(2, PriorityLevel::Normal, 10'000), t), EnqueueResult::Success);

    // Queue full (2/2). High priority arrival evicts Low priority (Message 1)
    EXPECT_EQ(buf.enqueue(make_msg(3, PriorityLevel::High, 10'000), t), EnqueueResult::Success);
    EXPECT_EQ(buf.statistics().messages_evicted, 1ULL);
    EXPECT_EQ(buf.message_count(), 2ULL);

    // Low priority arrival cannot evict High or Normal -> rejected!
    EXPECT_EQ(buf.enqueue(make_msg(4, PriorityLevel::Low, 10'000), t), EnqueueResult::CapacityCountExceeded);
}

// Test 20: Deterministic results across runs
void test_deterministic_repeatability() {
    auto run_sim = []() -> uint64_t {
        DtnBuffer buf;
        for (uint64_t i = 1; i <= 50; ++i) {
            buf.enqueue(make_msg(i, static_cast<PriorityLevel>(i % 3), 1000 + i * 10), i * 100);
            if (i % 2 == 0) {
                buf.dequeue_next_eligible(i * 100);
            }
        }
        return buf.statistics().messages_enqueued + buf.statistics().max_residence_time_ns;
    };

    uint64_t r1 = run_sim();
    uint64_t r2 = run_sim();
    EXPECT_EQ(r1, r2);
}

int main() {
    std::cout << "[RUN      ] test_dtn_buffer" << std::endl;
    test_fifo_within_equal_priority();
    test_priority_selection();
    test_priority_aging_fairness();
    test_exact_ttl_expiration_boundary();
    test_already_expired_enqueue();
    test_expiration_frees_capacity();
    test_capacity_limits();
    test_oversized_message_rejection();
    test_retry_timing_and_backoff();
    test_max_retry_attempts();
    test_retry_suppression_after_ttl_expiration();
    test_partition_and_reconnect();
    test_duplicate_suppression_and_bounded_cache();
    test_acknowledgement();
    test_priority_eviction();
    test_deterministic_repeatability();
    std::cout << "[       OK ] test_dtn_buffer (" << g_assertions << " assertions verified)" << std::endl;
    return 0;
}
