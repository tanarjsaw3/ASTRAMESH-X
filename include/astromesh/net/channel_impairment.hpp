#pragma once

#include "astromesh/net/message_frame.hpp"
#include "astromesh/net/udp_socket.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <utility>
#include <vector>

namespace astromesh::net {

/**
 * @brief Configuration parameters for network impairment simulation.
 */
struct ChannelImpairmentConfig {
    double packet_loss_rate{0.0};       ///< Probability in [0.0, 1.0] of dropping a packet.
    uint64_t one_way_delay_ns{0};       ///< Fixed propagation latency in nanoseconds (0 to 500 ms).
    uint64_t seed{42};                  ///< Deterministic seed for reproducible loss modeling.
    bool global_blackout{false};        ///< If true, all link transmissions are dropped.
};

/**
 * @brief Real-time metrics tracking dropped, delayed, and transmitted packets.
 */
struct ChannelImpairmentStats {
    uint64_t packets_submitted{0};
    uint64_t packets_dropped_loss{0};
    uint64_t packets_dropped_blackout{0};
    uint64_t packets_delayed{0};
    uint64_t packets_passed{0};
    uint64_t bytes_submitted{0};
    uint64_t bytes_passed{0};
};

/**
 * @brief Simulates real-world radio channel impairments (loss, latency, blackouts) on UDP packets.
 */
class ChannelImpairment {
public:
    struct DelayedPacket {
        std::vector<uint8_t> data;
        Endpoint destination;
        uint64_t release_time_ns{0};
    };

    explicit ChannelImpairment(const ChannelImpairmentConfig& config = {});

    void set_config(const ChannelImpairmentConfig& config);
    const ChannelImpairmentConfig& get_config() const;

    void set_loss_rate(double loss_rate);
    void set_delay_ns(uint64_t delay_ns);
    void set_global_blackout(bool blackout);

    /**
     * @brief Sets bilateral link state between two rovers.
     */
    void set_link_state(uint16_t rover_a, uint16_t rover_b, bool connected);

    /**
     * @brief Checks whether the bilateral link between two rovers is currently connected.
     */
    bool is_link_connected(uint16_t rover_a, uint16_t rover_b) const;

    /**
     * @brief Evaluates an outgoing packet against blackouts, loss rates, and delay queues.
     * @param data Serialized bytes to send.
     * @param dest Target network endpoint.
     * @param from_rover Sending rover ID.
     * @param to_rover Destination rover ID (0 if broadcast/unknown).
     * @param now_ns Current monotonic timestamp.
     * @param out_immediate Set to true if packet should be transmitted immediately; false if dropped or delayed.
     * @return true if packet is either scheduled for immediate send or buffered in delay queue;
     *         false if dropped due to loss or blackout.
     */
    bool process_outgoing(
        const uint8_t* data, size_t size,
        const Endpoint& dest,
        uint16_t from_rover, uint16_t to_rover,
        uint64_t now_ns,
        bool& out_immediate);

    /**
     * @brief Retrieves and removes all delayed packets whose release_time_ns <= now_ns.
     */
    std::vector<DelayedPacket> release_due_packets(uint64_t now_ns);

    /**
     * @brief Returns number of packets currently waiting in delay queue.
     */
    size_t pending_delayed_count() const;

    /**
     * @brief Resets counters and drains the delay queue.
     */
    void reset(uint64_t seed = 42);

    ChannelImpairmentStats get_stats() const;

private:
    uint64_t next_random();
    double next_uniform();

    mutable std::mutex m_mutex;
    ChannelImpairmentConfig m_config;
    ChannelImpairmentStats m_stats;
    uint64_t m_rng_state{42};

    // Bilateral link states: key = (min(r1, r2) << 16) | max(r1, r2)
    std::map<uint32_t, bool> m_bilateral_links;

    // Transit queue for delayed packets
    std::deque<DelayedPacket> m_delayed_queue;
};

} // namespace astromesh::net

