#include "astromesh/comms/dtn_buffer.hpp"
#include "astromesh/comms/packet_budget.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace astromesh::comms;

struct BenchmarkScenario {
    std::string name;
    uint64_t seed{42};
    size_t queue_capacity_count{100};
    size_t queue_capacity_bytes{1024 * 1024};
    size_t payload_size_bytes{80}; // Canonical boundary packet size
    FairnessPolicy fairness_policy{FairnessPolicy::StrictPriority};
    uint64_t partition_duration_ms{0};
    uint64_t ttl_ms{1000};
    uint32_t max_attempts{5};
    double packet_loss_rate{0.0};
    uint64_t window_byte_quota{0}; // 0 = unlimited
    size_t total_messages_to_generate{100};
    double high_prio_ratio{0.2};
    double normal_prio_ratio{0.5};
};

struct BenchmarkResult {
    std::string name;
    uint64_t seed{0};
    size_t queue_capacity_count{0};
    size_t queue_capacity_bytes{0};
    size_t payload_size_bytes{0};
    std::string fairness_policy;
    uint64_t partition_duration_ms{0};
    uint64_t ttl_ms{0};
    uint32_t max_attempts{0};
    double packet_loss_rate{0.0};
    uint64_t window_byte_quota{0};

    uint64_t messages_enqueued{0};
    uint64_t messages_delivered{0};
    uint64_t messages_expired{0};
    uint64_t messages_rejected_capacity{0};
    uint64_t messages_rejected_quota{0};
    uint64_t messages_evicted{0};
    uint64_t duplicates_suppressed{0};
    uint64_t retry_attempts_scheduled{0};
    uint64_t retries_exhausted{0};

    uint64_t payload_bytes_sent{0};
    uint64_t total_simulated_bytes_sent{0};
    uint64_t payload_bytes_received{0};
    uint64_t total_simulated_bytes_received{0};

    double delivery_ratio{0.0};
    double expiry_ratio{0.0};
    size_t queue_high_watermark_count{0};
    size_t queue_high_watermark_bytes{0};
    double max_residence_time_ms{0.0};
};

BenchmarkResult run_simulation(const BenchmarkScenario& sc) {
    std::mt19937_64 rng(sc.seed);
    std::uniform_real_distribution<double> dist_unit(0.0, 1.0);

    ByteBudgetConfig budget_cfg;
    budget_cfg.default_header_bytes = 24;
    budget_cfg.default_transport_overhead_bytes = 20;
    budget_cfg.window_duration_ns = 1'000'000'000ULL; // 1 s
    budget_cfg.max_bytes_per_window = sc.window_byte_quota;

    PacketBudgetManager budget(budget_cfg);

    DtnQueueConfig queue_cfg;
    queue_cfg.max_queue_count = sc.queue_capacity_count;
    queue_cfg.max_payload_bytes = sc.queue_capacity_bytes;
    queue_cfg.fairness_policy = sc.fairness_policy;
    queue_cfg.max_attempts = sc.max_attempts;
    queue_cfg.initial_retry_delay_ns = 20'000'000ULL; // 20 ms
    queue_cfg.backoff_multiplier = 2.0;
    queue_cfg.max_retry_delay_ns = 500'000'000ULL;    // 500 ms

    DtnBuffer buffer(queue_cfg, &budget);

    // Simulation parameters: run for 2.0 seconds in 1 ms steps
    const uint64_t sim_duration_ns = 2'000'000'000ULL;
    const uint64_t step_ns = 1'000'000ULL; // 1 ms
    const uint64_t partition_end_ns = sc.partition_duration_ms * 1'000'000ULL;

    size_t msg_gen_count = 0;
    const uint64_t msg_interval_ns = sim_duration_ns / (sc.total_messages_to_generate + 1);

    for (uint64_t now_ns = 0; now_ns < sim_duration_ns; now_ns += step_ns) {
        // Periodic message generation
        if (msg_gen_count < sc.total_messages_to_generate && (now_ns >= msg_gen_count * msg_interval_ns)) {
            msg_gen_count++;
            DtnMessage msg;
            msg.message_id = msg_gen_count;
            msg.sender_rover_id = 1;
            msg.target_rover_id = 2;
            msg.ttl_ns = sc.ttl_ms * 1'000'000ULL;
            msg.payload.assign(sc.payload_size_bytes, static_cast<uint8_t>(msg_gen_count & 0xFF));

            const double p = dist_unit(rng);
            if (p < sc.high_prio_ratio) {
                msg.priority = PriorityLevel::High;
            } else if (p < sc.high_prio_ratio + sc.normal_prio_ratio) {
                msg.priority = PriorityLevel::Normal;
            } else {
                msg.priority = PriorityLevel::Low;
            }

            buffer.enqueue(msg, now_ns);
        }

        // Link connectivity status (partition check)
        const bool link_connected = (now_ns >= partition_end_ns);

        auto link_checker = [&](uint16_t target) -> bool {
            if (target == 2) return link_connected;
            return true;
        };

        // Service DTN queue
        while (true) {
            auto candidate = buffer.dequeue_next_eligible(now_ns, link_checker);
            if (!candidate.has_value()) {
                break;
            }

            // Attempt transmission over simulated physical channel
            const bool budget_ok = budget.record_sent(
                candidate->target_rover_id,
                candidate->priority,
                candidate->payload_bytes(),
                now_ns,
                candidate->attempt_count > 1);

            if (!budget_ok) {
                // Rejected by byte quota
                buffer.mark_transmission_attempt(candidate->message_id, now_ns, false);
                continue;
            }

            // Channel packet loss simulation
            const bool dropped = (sc.packet_loss_rate > 0.0 && dist_unit(rng) < sc.packet_loss_rate);

            if (dropped) {
                budget.record_network_drop(
                    candidate->target_rover_id,
                    candidate->priority,
                    candidate->payload_bytes());
                // Transmission failed; advance backoff schedule
                buffer.mark_transmission_attempt(candidate->message_id, now_ns, false);
            } else {
                // Successful delivery!
                budget.record_received(
                    candidate->sender_rover_id,
                    candidate->priority,
                    candidate->payload_bytes(),
                    now_ns);
                buffer.acknowledge(candidate->message_id, now_ns);
            }
        }
    }

    // Final purge of expired messages at end of simulation
    buffer.purge_expired(sim_duration_ns);

    BenchmarkResult res;
    res.name = sc.name;
    res.seed = sc.seed;
    res.queue_capacity_count = sc.queue_capacity_count;
    res.queue_capacity_bytes = sc.queue_capacity_bytes;
    res.payload_size_bytes = sc.payload_size_bytes;
    res.fairness_policy = to_string(sc.fairness_policy);
    res.partition_duration_ms = sc.partition_duration_ms;
    res.ttl_ms = sc.ttl_ms;
    res.max_attempts = sc.max_attempts;
    res.packet_loss_rate = sc.packet_loss_rate;
    res.window_byte_quota = sc.window_byte_quota;

    const auto& qs = buffer.statistics();
    const auto& bs = budget.statistics();

    res.messages_enqueued = qs.messages_enqueued;
    res.messages_delivered = qs.messages_delivered;
    res.messages_expired = qs.messages_expired;
    res.messages_rejected_capacity = qs.messages_rejected_capacity;
    res.messages_rejected_quota = qs.messages_rejected_quota;
    res.messages_evicted = qs.messages_evicted;
    res.duplicates_suppressed = qs.duplicates_suppressed;
    res.retry_attempts_scheduled = qs.retry_attempts_scheduled;
    res.retries_exhausted = qs.retries_exhausted;

    res.payload_bytes_sent = bs.sent.payload_bytes;
    res.total_simulated_bytes_sent = bs.sent.total_wire_bytes;
    res.payload_bytes_received = bs.received.payload_bytes;
    res.total_simulated_bytes_received = bs.received.total_wire_bytes;

    res.delivery_ratio = (res.messages_enqueued > 0) ?
        static_cast<double>(res.messages_delivered) / static_cast<double>(res.messages_enqueued) : 0.0;
    res.expiry_ratio = (res.messages_enqueued > 0) ?
        static_cast<double>(res.messages_expired) / static_cast<double>(res.messages_enqueued) : 0.0;

    res.queue_high_watermark_count = qs.max_queue_count_watermark;
    res.queue_high_watermark_bytes = qs.max_payload_bytes_watermark;
    res.max_residence_time_ms = static_cast<double>(qs.max_residence_time_ns) / 1'000'000.0;

    return res;
}

void write_csv_header(std::ostream& os) {
    os << "scenario_name,seed,capacity_count,capacity_bytes,payload_bytes,fairness_policy,"
       << "partition_duration_ms,ttl_ms,max_attempts,packet_loss_rate,window_byte_quota,"
       << "messages_enqueued,messages_delivered,messages_expired,messages_rejected_capacity,"
       << "messages_rejected_quota,messages_evicted,duplicates_suppressed,retries_scheduled,"
       << "retries_exhausted,payload_bytes_sent,total_sim_bytes_sent,payload_bytes_recv,"
       << "total_sim_bytes_recv,delivery_ratio,expiry_ratio,high_watermark_count,"
       << "high_watermark_bytes,max_residence_time_ms\n";
}

void write_csv_row(std::ostream& os, const BenchmarkResult& r) {
    os << r.name << ","
       << r.seed << ","
       << r.queue_capacity_count << ","
       << r.queue_capacity_bytes << ","
       << r.payload_size_bytes << ","
       << r.fairness_policy << ","
       << r.partition_duration_ms << ","
       << r.ttl_ms << ","
       << r.max_attempts << ","
       << std::fixed << std::setprecision(4) << r.packet_loss_rate << ","
       << r.window_byte_quota << ","
       << r.messages_enqueued << ","
       << r.messages_delivered << ","
       << r.messages_expired << ","
       << r.messages_rejected_capacity << ","
       << r.messages_rejected_quota << ","
       << r.messages_evicted << ","
       << r.duplicates_suppressed << ","
       << r.retry_attempts_scheduled << ","
       << r.retries_exhausted << ","
       << r.payload_bytes_sent << ","
       << r.total_simulated_bytes_sent << ","
       << r.payload_bytes_received << ","
       << r.total_simulated_bytes_received << ","
       << std::fixed << std::setprecision(4) << r.delivery_ratio << ","
       << std::fixed << std::setprecision(4) << r.expiry_ratio << ","
       << r.queue_high_watermark_count << ","
       << r.queue_high_watermark_bytes << ","
       << std::fixed << std::setprecision(3) << r.max_residence_time_ms << "\n";
}

int main(int argc, char* argv[]) {
    std::string output_csv = "benchmark_dtn_buffer_results.csv";
    if (argc > 1) {
        output_csv = argv[1];
    }

    std::cout << "Running AstroMesh-X Phase 4.2 DTN & Byte Quota Benchmark Suite..." << std::endl;
    std::cout << "Output CSV: " << output_csv << std::endl;

    std::vector<BenchmarkScenario> scenarios;

    // 1. Baseline FIFO / No Retries / No Quota
    {
        BenchmarkScenario s;
        s.name = "Baseline_FIFO_NoRetry_NoQuota";
        s.max_attempts = 1;
        s.fairness_policy = FairnessPolicy::StrictPriority;
        scenarios.push_back(s);
    }

    // 2. Queue Capacity Contention Sweep (burst 150 messages into capacity 10, 25, 50, 100)
    for (size_t cap : {10ULL, 25ULL, 50ULL, 100ULL}) {
        BenchmarkScenario s;
        s.name = "Capacity_Constraint_Cap" + std::to_string(cap);
        s.queue_capacity_count = cap;
        s.total_messages_to_generate = 150;
        scenarios.push_back(s);
    }

    // 3. Fairness Policy Comparison under Priority Contention
    for (auto pol : {FairnessPolicy::StrictPriority, FairnessPolicy::WeightedInterleaving, FairnessPolicy::PriorityAging}) {
        BenchmarkScenario s;
        s.name = std::string("Fairness_") + to_string(pol);
        s.fairness_policy = pol;
        s.high_prio_ratio = 0.3;
        s.normal_prio_ratio = 0.4;
        scenarios.push_back(s);
    }

    // 4. Partition Duration Sweep (0, 100, 300, 500, 1000, 1500 ms with TTL = 500 ms)
    for (uint64_t part_ms : {0ULL, 100ULL, 300ULL, 500ULL, 1000ULL, 1500ULL}) {
        BenchmarkScenario s;
        s.name = "Partition_" + std::to_string(part_ms) + "ms";
        s.partition_duration_ms = part_ms;
        s.ttl_ms = 500;
        scenarios.push_back(s);
    }

    // 5. Retry Policy under Packet Loss (Loss 0%, 10%, 25%, 50% with NoRetry vs Exponential Backoff)
    for (double loss : {0.0, 0.10, 0.25, 0.50}) {
        // No Retry
        BenchmarkScenario s_no;
        s_no.name = "Loss_" + std::to_string(static_cast<int>(loss * 100)) + "pct_NoRetry";
        s_no.packet_loss_rate = loss;
        s_no.max_attempts = 1;
        scenarios.push_back(s_no);

        // Exponential Backoff Retry (max 5)
        BenchmarkScenario s_re;
        s_re.name = "Loss_" + std::to_string(static_cast<int>(loss * 100)) + "pct_ExpRetry";
        s_re.packet_loss_rate = loss;
        s_re.max_attempts = 5;
        scenarios.push_back(s_re);
    }

    // 6. Byte Quota Accounting Sweep (Window 1s, Quotas: 2000, 5000, 10000, Unlimited)
    for (uint64_t q : {2000ULL, 5000ULL, 10000ULL, 0ULL}) {
        BenchmarkScenario s;
        s.name = (q == 0) ? "Quota_Unlimited" : ("Quota_" + std::to_string(q) + "Bps");
        s.window_byte_quota = q;
        scenarios.push_back(s);
    }

    // 7. Multi-Seed Stability Sweep (Seeds 42, 101, 777, 2026 under 20% loss + 200ms partition)
    for (uint64_t seed : {42ULL, 101ULL, 777ULL, 2026ULL}) {
        BenchmarkScenario s;
        s.name = "MultiSeed_S" + std::to_string(seed) + "_Loss20_Part200";
        s.seed = seed;
        s.packet_loss_rate = 0.20;
        s.partition_duration_ms = 200;
        scenarios.push_back(s);
    }

    std::ofstream csv(output_csv);
    write_csv_header(csv);

    size_t count = 0;
    for (const auto& sc : scenarios) {
        count++;
        BenchmarkResult r = run_simulation(sc);
        write_csv_row(csv, r);
        std::cout << "[" << std::setw(2) << count << "/" << scenarios.size() << "] "
                  << std::setw(38) << std::left << r.name
                  << " Deliv=" << std::setw(3) << r.messages_delivered
                  << " Exp=" << std::setw(3) << r.messages_expired
                  << " RejCap=" << std::setw(3) << r.messages_rejected_capacity
                  << " Retries=" << std::setw(3) << r.retry_attempts_scheduled
                  << " Sent=" << std::setw(6) << r.total_simulated_bytes_sent << " B"
                  << " Ratio=" << std::fixed << std::setprecision(1) << (r.delivery_ratio * 100.0) << "%"
                  << std::endl;
    }

    csv.close();
    std::cout << "\nBenchmark complete. " << scenarios.size() << " trials written to " << output_csv << std::endl;
    return 0;
}
