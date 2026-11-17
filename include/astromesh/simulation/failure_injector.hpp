#pragma once

#include <cstdint>
#include <map>
#include <random>
#include <utility>
#include <vector>

namespace astromesh::simulation {

struct GilbertElliottParams {
    double p_g_loss{0.01};     ///< Packet loss rate in Good state
    double p_b_loss{0.70};     ///< Packet loss rate in Bad state
    double p_gb{0.05};         ///< Probability of transitioning Good -> Bad
    double p_bg{0.30};         ///< Probability of transitioning Bad -> Good
};

struct PartitionSchedule {
    uint16_t rover_a{0};
    uint16_t rover_b{0};
    uint64_t start_time_ms{0};
    uint64_t end_time_ms{0};
};

struct NodeCrashSchedule {
    uint16_t rover_id{0};
    uint64_t crash_time_ms{0};
    uint64_t restart_time_ms{0};
};

enum class ChannelState : uint8_t {
    Good = 0,
    Bad = 1
};

/**
 * @brief Multi-Rover Swarm Failure and Partition Injection Engine.
 * Supports deterministic Gilbert-Elliott burst loss, scheduled link partitions,
 * and node crash/restart cycles for rigorous network evaluation.
 */
class FailureInjector {
public:
    explicit FailureInjector(uint64_t seed = 42);

    void set_gilbert_elliott_params(GilbertElliottParams params);

    void schedule_partition(uint16_t rover_a, uint16_t rover_b, uint64_t start_ms, uint64_t end_ms);
    void schedule_node_crash(uint16_t rover_id, uint64_t crash_ms, uint64_t restart_ms);

    bool is_link_partitioned(uint16_t rover_a, uint16_t rover_b, uint64_t current_time_ms) const noexcept;
    bool is_node_crashed(uint16_t rover_id, uint64_t current_time_ms) const noexcept;

    /**
     * @brief Evaluates whether a packet between from_rover and to_rover should be dropped
     * due to partitions, crash state, or Gilbert-Elliott burst loss.
     */
    bool should_drop_packet(uint16_t from_rover, uint16_t to_rover, uint64_t current_time_ms);

    /**
     * @brief Computes transmission delay including Gaussian jitter (ms).
     */
    double sample_latency_ms(double mean_ms, double stddev_ms);

    void reset();

private:
    std::pair<uint16_t, uint16_t> canonical_link(uint16_t a, uint16_t b) const noexcept;

    uint64_t m_seed;
    std::mt19937_64 m_rng;
    GilbertElliottParams m_ge_params;

    std::vector<PartitionSchedule> m_partitions;
    std::vector<NodeCrashSchedule> m_crashes;

    // Link (min, max) -> Current Gilbert-Elliott channel state
    std::map<std::pair<uint16_t, uint16_t>, ChannelState> m_link_states;
};

} // namespace astromesh::simulation

