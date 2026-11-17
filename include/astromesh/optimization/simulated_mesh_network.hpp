#pragma once

#include "astromesh/optimization/boundary_packet.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <queue>
#include <random>
#include <set>
#include <utility>
#include <vector>

namespace astromesh::optimization {

/**
 * @brief Configuration parameters for discrete-event simulated mesh network.
 */
struct SimulatedNetworkConfig {
    uint64_t latency_ns{0};              ///< Base deterministic transmission latency in nanoseconds.
    uint64_t jitter_ns{0};               ///< Stochastic delay jitter [0, jitter_ns].
    double packet_loss_rate{0.0};        ///< Fraction of packets randomly dropped [0.0, 1.0].
    double duplicate_rate{0.0};          ///< Fraction of packets duplicated [0.0, 1.0].
    size_t transport_header_bytes{20};   ///< Simulated framing/transport layer overhead per packet (bytes).
    uint64_t random_seed{42};            ///< PRNG seed for deterministic repeatability.
};

/**
 * @brief Real-time communication statistics tracked by the simulated network.
 */
struct NetworkStatistics {
    size_t packets_sent{0};
    size_t packets_delivered{0};
    size_t packets_dropped{0};
    size_t packets_duplicated{0};
    size_t raw_payload_bytes_sent{0};
    size_t transport_bytes_sent{0};
    size_t raw_payload_bytes_delivered{0};
    size_t transport_bytes_delivered{0};

    void reset() noexcept {
        *this = NetworkStatistics();
    }
};

/**
 * @brief Discrete-event simulated mesh network with deterministic delay, packet drop,
 *        link disconnections, and communication byte accounting.
 */
class SimulatedMeshNetwork {
public:
    explicit SimulatedMeshNetwork(const SimulatedNetworkConfig& config = {}) noexcept;

    void reset(uint64_t initial_time_ns = 0);

    uint64_t current_time_ns() const noexcept { return current_time_ns_; }
    void advance_time_ns(uint64_t delta_ns) noexcept { current_time_ns_ += delta_ns; }
    void set_current_time_ns(uint64_t time_ns) noexcept { current_time_ns_ = time_ns; }

    const SimulatedNetworkConfig& config() const noexcept { return config_; }
    const NetworkStatistics& statistics() const noexcept { return stats_; }

    /**
     * @brief Configures bilateral link state between two rovers (connected or disconnected).
     */
    void set_link_state(uint16_t rover_a, uint16_t rover_b, bool connected) noexcept;
    bool is_link_connected(uint16_t rover_a, uint16_t rover_b) const noexcept;

    /**
     * @brief Enqueues a boundary state packet for transmission.
     *        Applies serialization, transport byte accounting, link connectivity checks,
     *        simulated packet loss, and latency scheduling.
     * @return true if packet was accepted into transit, false if dropped or disconnected.
     */
    bool send_packet(const BoundaryStatePacket& packet);

    /**
     * @brief Delivers all packets scheduled for delivery to target_rover_id up to current_time_ns().
     */
    std::vector<BoundaryStatePacket> receive_packets(uint16_t target_rover_id);

    /**
     * @brief Returns total number of packets currently in transit.
     */
    size_t in_flight_count() const noexcept { return in_flight_queue_.size(); }

private:
    struct QueuedMessage {
        uint64_t delivery_time_ns{0};
        uint64_t packet_id{0};
        uint16_t target_rover_id{0};
        BoundaryStatePacket packet;

        bool operator>(const QueuedMessage& other) const noexcept {
            if (delivery_time_ns != other.delivery_time_ns) {
                return delivery_time_ns > other.delivery_time_ns;
            }
            return packet_id > other.packet_id;
        }
    };

    SimulatedNetworkConfig config_;
    uint64_t current_time_ns_{0};
    uint64_t next_packet_id_{1};
    NetworkStatistics stats_;

    std::mt19937_64 rng_;
    std::uniform_real_distribution<double> dist_unit_{0.0, 1.0};

    // Disconnected peer pairs: stored as normalized pair (min(a,b), max(a,b))
    std::set<std::pair<uint16_t, uint16_t>> disconnected_links_;

    // Priority queue of in-flight packets ordered by delivery time
    std::priority_queue<QueuedMessage, std::vector<QueuedMessage>, std::greater<QueuedMessage>> in_flight_queue_;
};

} // namespace astromesh::optimization
