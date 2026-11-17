#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <ostream>
#include <string>

namespace astromesh::comms {

/**
 * @brief Categorization of message priority for communication budgeting.
 */
enum class PriorityLevel : uint8_t {
    Low = 0,
    Normal = 1,
    High = 2
};

/**
 * @brief Result of evaluating an intended transmission against active quotas.
 */
enum class QuotaCheckResult : uint8_t {
    Approved = 0,
    ExceedsWindowBudget = 1,
    ExceedsPeerQuota = 2,
    ExceedsMaxMessageSize = 3,
    InvalidMessageSize = 4
};

const char* to_string(QuotaCheckResult result) noexcept;
const char* to_string(PriorityLevel priority) noexcept;

inline std::ostream& operator<<(std::ostream& os, QuotaCheckResult res) {
    return os << to_string(res);
}

inline std::ostream& operator<<(std::ostream& os, PriorityLevel prio) {
    return os << to_string(prio);
}

/**
 * @brief Explicit breakdown of physical and protocol byte components.
 */
struct ByteBreakdown {
    size_t payload_bytes{0};            ///< Application data (keyframes, IBLT, poses, etc.)
    size_t header_bytes{0};             ///< Protocol framing / message envelope header
    size_t transport_overhead_bytes{0}; ///< Simulated lower-layer transport (IP/UDP/MAC)

    size_t total_wire_bytes() const noexcept {
        size_t total = payload_bytes;
        if (std::numeric_limits<size_t>::max() - total < header_bytes) {
            return std::numeric_limits<size_t>::max();
        }
        total += header_bytes;
        if (std::numeric_limits<size_t>::max() - total < transport_overhead_bytes) {
            return std::numeric_limits<size_t>::max();
        }
        return total + transport_overhead_bytes;
    }
};

/**
 * @brief Configuration parameters for communication byte quotas.
 */
struct ByteBudgetConfig {
    uint64_t window_duration_ns{1'000'000'000}; ///< Epoch window for sliding quota (default 1.0 s; 0 = unlimited).
    uint64_t max_bytes_per_window{0};           ///< Maximum total wire bytes permitted per window (0 = unlimited).
    uint64_t per_peer_quota_bytes{0};          ///< Maximum wire bytes permitted per peer within window (0 = unlimited).
    size_t max_message_payload_bytes{0};        ///< Maximum allowable application payload size (0 = unlimited).
    size_t default_header_bytes{24};            ///< Default DTN protocol header overhead.
    size_t default_transport_overhead_bytes{20};///< Simulated framing/transport layer overhead.
};

/**
 * @brief Byte and packet metrics tracked for a single peer or collective aggregate.
 */
struct TrafficStatistics {
    uint64_t payload_bytes{0};
    uint64_t header_bytes{0};
    uint64_t transport_overhead_bytes{0};
    uint64_t total_wire_bytes{0};
    uint64_t message_count{0};

    void add(const ByteBreakdown& bytes) noexcept;
    void reset() noexcept;
};

/**
 * @brief Comprehensive communication accounting counters.
 */
struct BudgetStatistics {
    TrafficStatistics sent;
    TrafficStatistics received;
    TrafficStatistics retransmitted;
    TrafficStatistics rejected;
    TrafficStatistics network_dropped;

    // Per-peer sent traffic
    std::map<uint16_t, TrafficStatistics> per_peer_sent;

    // Per-priority sent traffic
    std::array<TrafficStatistics, 3> per_priority_sent;

    void reset() noexcept;
};

/**
 * @brief Bounded communication byte quota and traffic accounting engine.
 *
 * Implements deterministic epoch-window byte metering, strict separation
 * between application payload and simulated transport overhead, and quota
 * gating with zero double-counting.
 */
class PacketBudgetManager {
public:
    explicit PacketBudgetManager(const ByteBudgetConfig& config = {}) noexcept;

    void reset(uint64_t initial_time_ns = 0) noexcept;

    const ByteBudgetConfig& config() const noexcept { return config_; }
    void set_config(const ByteBudgetConfig& config) noexcept { config_ = config; }

    const BudgetStatistics& statistics() const noexcept { return stats_; }
    uint64_t current_window_bytes_used() const noexcept { return current_window_wire_bytes_; }
    uint64_t current_window_start_ns() const noexcept { return window_start_ns_; }

    /**
     * @brief Evaluates whether an intended transmission of given payload size is permitted under quotas.
     *        Does NOT mutate statistics or consume budget.
     */
    QuotaCheckResult check_quota(
        uint16_t peer_id,
        size_t payload_bytes,
        uint64_t now_ns,
        size_t custom_header_bytes = 0,
        size_t custom_transport_bytes = 0) noexcept;

    /**
     * @brief Records a successfully dispatched message into transmission statistics and consumes quota.
     * @return true if transmission was permitted and recorded, false if rejected by quota.
     */
    bool record_sent(
        uint16_t peer_id,
        PriorityLevel priority,
        size_t payload_bytes,
        uint64_t now_ns,
        bool is_retransmission = false,
        size_t custom_header_bytes = 0,
        size_t custom_transport_bytes = 0) noexcept;

    /**
     * @brief Records an incoming message received at the destination node.
     */
    void record_received(
        uint16_t sender_id,
        PriorityLevel priority,
        size_t payload_bytes,
        uint64_t now_ns,
        size_t custom_header_bytes = 0,
        size_t custom_transport_bytes = 0) noexcept;

    /**
     * @brief Records a message rejected prior to transmission (does NOT increment transmitted bytes).
     */
    void record_rejected(
        uint16_t peer_id,
        PriorityLevel priority,
        size_t payload_bytes,
        QuotaCheckResult reason,
        size_t custom_header_bytes = 0,
        size_t custom_transport_bytes = 0) noexcept;

    /**
     * @brief Records a message transmitted into the network but subsequently dropped by channel loss/partition.
     *        Sent bytes were already recorded; this updates network drop counters.
     */
    void record_network_drop(
        uint16_t peer_id,
        PriorityLevel priority,
        size_t payload_bytes,
        size_t custom_header_bytes = 0,
        size_t custom_transport_bytes = 0) noexcept;

    /**
     * @brief Advances window epoch if now_ns has crossed current window duration.
     */
    void update_window(uint64_t now_ns) noexcept;

private:
    ByteBreakdown resolve_breakdown(size_t payload, size_t custom_header, size_t custom_transport) const noexcept;

    ByteBudgetConfig config_;
    BudgetStatistics stats_;

    uint64_t window_start_ns_{0};
    uint64_t current_window_wire_bytes_{0};
    std::map<uint16_t, uint64_t> current_window_peer_bytes_;
};

} // namespace astromesh::comms
