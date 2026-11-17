#pragma once

#include "astromesh/comms/packet_budget.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace astromesh::comms {

/**
 * @brief Categorization of payloads transported through the DTN buffer.
 */
enum class DtnMessageType : uint8_t {
    GenericData = 0,
    BoundaryState = 1,
    IBLTReconciliation = 2,
    LoopClosureProposal = 3,
    Acknowledgment = 4
};

const char* to_string(DtnMessageType type) noexcept;

constexpr uint16_t ACK_PROTOCOL_VERSION = 1;

/**
 * @brief Explicit network-transported acknowledgment control payload (24 bytes).
 */
struct AckPayload {
    uint16_t protocol_version{ACK_PROTOCOL_VERSION};
    uint16_t ack_sender_id{0};       ///< Destination rover generating this ACK.
    uint16_t ack_target_id{0};       ///< Origin rover receiving this ACK.
    uint16_t reserved{0};
    uint64_t acked_message_id{0};    ///< Logical message ID being acknowledged.
    uint64_t ack_timestamp_ns{0};    ///< Monotonic simulated time when ACK was generated.
};

std::vector<uint8_t> serialize_ack_payload(const AckPayload& ack);
std::optional<AckPayload> deserialize_ack_payload(const uint8_t* data, size_t length);
std::optional<AckPayload> deserialize_ack_payload(const std::vector<uint8_t>& payload);

/**
 * @brief Outcome of an enqueue operation.
 */
enum class EnqueueResult : uint8_t {
    Success = 0,
    DuplicateIgnored = 1,
    CapacityCountExceeded = 2,
    CapacityBytesExceeded = 3,
    AlreadyExpired = 4,
    InvalidMetadata = 5,
    PayloadTooLarge = 6,
    QuotaExceeded = 7
};

const char* to_string(EnqueueResult result) noexcept;

/**
 * @brief Outcome of acknowledging a message.
 */
enum class AckResult : uint8_t {
    Acknowledged = 0,
    NotFound = 1
};

const char* to_string(AckResult result) noexcept;

/**
 * @brief Fairness policy governing dequeue ordering among priority tiers.
 */
enum class FairnessPolicy : uint8_t {
    StrictPriority = 0,       ///< Always deliver High before Normal, Normal before Low.
    WeightedInterleaving = 1, ///< Weighted round-robin deficit counter (e.g., 4 High : 2 Normal : 1 Low).
    PriorityAging = 2         ///< Promotes waiting messages up priority tiers based on elapsed wait time.
};

const char* to_string(FairnessPolicy policy) noexcept;

inline std::ostream& operator<<(std::ostream& os, DtnMessageType type) {
    return os << to_string(type);
}

inline std::ostream& operator<<(std::ostream& os, EnqueueResult res) {
    return os << to_string(res);
}

inline std::ostream& operator<<(std::ostream& os, AckResult res) {
    return os << to_string(res);
}

inline std::ostream& operator<<(std::ostream& os, FairnessPolicy pol) {
    return os << to_string(pol);
}

/**
 * @brief Complete record of a buffered store-and-forward message.
 */
struct DtnMessage {
    uint64_t message_id{0};             ///< Stable unique message identifier (> 0).
    uint16_t sender_rover_id{0};        ///< Origin rover or hop transmitter identifier.
    uint16_t target_rover_id{0};        ///< Final destination rover identifier (0 = broadcast/collective).
    uint16_t origin_rover_id{0};        ///< Original logical source rover identifier (0 = sender_rover_id).
    uint16_t next_hop_rover_id{0};      ///< Immediate hop receiver (0 = target_rover_id).
    uint8_t hop_count{0};               ///< Number of hops traversed so far.
    uint8_t max_hops{5};                ///< Maximum allowed hops before drop.
    DtnMessageType message_type{DtnMessageType::GenericData};
    PriorityLevel priority{PriorityLevel::Normal};
    uint64_t creation_time_ns{0};       ///< Message generation timestamp.
    uint64_t enqueue_time_ns{0};        ///< Arrival timestamp in the DTN buffer.
    uint64_t ttl_ns{0};                 ///< Maximum lifetime in nanoseconds.
    uint64_t expiry_time_ns{0};         ///< Absolute expiration timestamp = enqueue_time_ns + ttl_ns.
    std::vector<uint8_t> payload;       ///< Serialized application bytes.
    uint32_t sequence_number{0};        ///< Optional protocol sequence/version number.
    uint32_t attempt_count{0};          ///< Number of transmission attempts made on current hop.
    uint64_t next_retry_time_ns{0};     ///< Earliest time when next transmission attempt is permitted.
    uint64_t last_transmit_time_ns{0};  ///< Timestamp of the most recent physical transmission attempt.

    size_t payload_bytes() const noexcept { return payload.size(); }

    uint16_t get_origin_id() const noexcept {
        return (origin_rover_id != 0) ? origin_rover_id : sender_rover_id;
    }

    uint16_t get_next_hop_id() const noexcept {
        return (next_hop_rover_id != 0) ? next_hop_rover_id : target_rover_id;
    }

    bool is_expired(uint64_t now_ns) const noexcept {
        return now_ns >= expiry_time_ns;
    }

    bool is_eligible(uint64_t now_ns) const noexcept {
        return !is_expired(now_ns) && (now_ns >= next_retry_time_ns);
    }
};

/**
 * @brief Configuration parameters for Jacobson/Karn RTT and RTO estimation (RFC 6298).
 */
struct RttEstimatorConfig {
    uint64_t initial_rto_ns{50'000'000ULL};    ///< Initial RTO before any measurement (50 ms).
    uint64_t min_rto_ns{35'000'000ULL};        ///< Minimum RTO clamp floor (35 ms).
    uint64_t max_rto_ns{2'000'000'000ULL};     ///< Maximum RTO clamp ceiling (2.0 s).
    double alpha{0.125};                        ///< SRTT smoothing factor (1/8).
    double beta{0.25};                          ///< RTTVAR smoothing factor (1/4).
    double k{4.0};                              ///< Variance multiplier for RTO calculation.
};

/**
 * @brief Jacobson/Karn Round-Trip Time and Retransmission Timeout estimator (RFC 6298).
 *        Maintains smoothed RTT (SRTT), RTT variation (RTTVAR), and adaptive RTO.
 *        Enforces Karn's algorithm: ignores RTT samples from retransmitted messages.
 */
class RttEstimator {
public:
    explicit RttEstimator(const RttEstimatorConfig& config = {}) noexcept
        : config_(config), current_rto_ns_(config.initial_rto_ns) {}

    const RttEstimatorConfig& config() const noexcept { return config_; }
    void set_config(const RttEstimatorConfig& config) noexcept {
        config_ = config;
        if (!has_measurement_) {
            current_rto_ns_ = config_.initial_rto_ns;
        }
    }

    bool has_measurement() const noexcept { return has_measurement_; }
    uint64_t srtt_ns() const noexcept { return srtt_ns_; }
    uint64_t rttvar_ns() const noexcept { return rttvar_ns_; }
    uint64_t current_rto_ns() const noexcept { return current_rto_ns_; }

    /**
     * @brief Updates RTT estimate upon receiving an acknowledgment.
     *        Applies Karn's rule: if attempt_count > 1, the sample is ignored.
     * @param measured_rtt_ns Elapsed duration between transmission and ACK.
     * @param attempt_count Transmission attempts made for this message (1 = first attempt).
     * @return true if SRTT/RTTVAR/RTO were updated; false if Karn's rule rejected the sample.
     */
    bool update_rtt(uint64_t measured_rtt_ns, uint32_t attempt_count) noexcept {
        if (attempt_count > 1) {
            // Karn's algorithm: Do NOT update SRTT/RTTVAR for retransmitted segments
            return false;
        }

        const double m = static_cast<double>(measured_rtt_ns);

        if (!has_measurement_) {
            // First RTT measurement (RFC 6298 Section 2.2)
            srtt_ns_ = measured_rtt_ns;
            rttvar_ns_ = measured_rtt_ns / 2;
            has_measurement_ = true;
        } else {
            // Subsequent RTT measurements (RFC 6298 Section 2.3)
            const double diff = std::abs(static_cast<double>(srtt_ns_) - m);
            const double new_rttvar = (1.0 - config_.beta) * static_cast<double>(rttvar_ns_) + config_.beta * diff;
            const double new_srtt = (1.0 - config_.alpha) * static_cast<double>(srtt_ns_) + config_.alpha * m;

            rttvar_ns_ = static_cast<uint64_t>(std::round(new_rttvar));
            srtt_ns_ = static_cast<uint64_t>(std::round(new_srtt));
        }

        // RTO = SRTT + K * RTTVAR
        const double raw_rto = static_cast<double>(srtt_ns_) + config_.k * static_cast<double>(rttvar_ns_);
        uint64_t computed_rto = static_cast<uint64_t>(std::round(raw_rto));

        if (computed_rto < config_.min_rto_ns) {
            computed_rto = config_.min_rto_ns;
        } else if (computed_rto > config_.max_rto_ns) {
            computed_rto = config_.max_rto_ns;
        }

        current_rto_ns_ = computed_rto;
        return true;
    }

    /**
     * @brief Performs exponential backoff on timeout/retransmission (Karn's algorithm).
     */
    void backoff(double multiplier = 2.0) noexcept {
        const double backed_off = static_cast<double>(current_rto_ns_) * std::max(1.0, multiplier);
        if (backed_off >= static_cast<double>(config_.max_rto_ns)) {
            current_rto_ns_ = config_.max_rto_ns;
        } else {
            current_rto_ns_ = static_cast<uint64_t>(std::round(backed_off));
        }
    }

    void reset() noexcept {
        srtt_ns_ = 0;
        rttvar_ns_ = 0;
        current_rto_ns_ = config_.initial_rto_ns;
        has_measurement_ = false;
    }

private:
    RttEstimatorConfig config_;
    uint64_t srtt_ns_{0};
    uint64_t rttvar_ns_{0};
    uint64_t current_rto_ns_{50'000'000ULL};
    bool has_measurement_{false};
};

/**
 * @brief Configuration parameters for the store-and-forward buffer.
 */
struct DtnQueueConfig {
    size_t max_queue_count{1000};                   ///< Maximum number of queued messages (0 = unlimited).
    size_t max_payload_bytes{1024 * 1024};          ///< Maximum aggregate buffered payload bytes (0 = unlimited).
    size_t max_message_payload_bytes{64 * 1024};    ///< Maximum single-message payload bytes (0 = unlimited).
    bool enable_priority_eviction{false};           ///< If true, allows higher-priority arrivals to evict lowest-priority messages.
    FairnessPolicy fairness_policy{FairnessPolicy::PriorityAging};
    uint64_t aging_interval_ns{500'000'000};        ///< Time interval (0.5 s) to promote low/normal priority messages.
    uint32_t high_priority_weight{4};               ///< High priority weight for WeightedInterleaving.
    uint32_t normal_priority_weight{2};             ///< Normal priority weight for WeightedInterleaving.
    uint32_t low_priority_weight{1};                ///< Low priority weight for WeightedInterleaving.
    uint64_t initial_retry_delay_ns{50'000'000};    ///< Initial retry delay (50 ms).
    double backoff_multiplier{2.0};                 ///< Exponential backoff multiplier.
    uint64_t max_retry_delay_ns{2'000'000'000};     ///< Maximum retry delay cap (2.0 s).
    uint32_t max_attempts{5};                       ///< Maximum transmission attempts before dropping.
    size_t dedup_cache_capacity{5000};              ///< Maximum entries in deduplication cache.
    uint64_t dedup_cache_ttl_ns{60'000'000'000ULL}; ///< TTL for deduplication cache entries (60 s).
    RttEstimatorConfig rtt_config;                  ///< Adaptive RTT / RTO configuration.
};

/**
 * @brief Counters and performance metrics for the DTN buffer.
 */
struct DtnQueueStatistics {
    uint64_t messages_enqueued{0};
    uint64_t payload_bytes_enqueued{0};
    uint64_t messages_delivered{0};
    uint64_t payload_bytes_delivered{0};
    uint64_t messages_expired{0};
    uint64_t payload_bytes_expired{0};
    uint64_t messages_evicted{0};
    uint64_t payload_bytes_evicted{0};
    uint64_t messages_rejected_capacity{0};
    uint64_t payload_bytes_rejected_capacity{0};
    uint64_t messages_rejected_quota{0};
    uint64_t payload_bytes_rejected_quota{0};
    uint64_t duplicates_suppressed{0};
    uint64_t retry_attempts_scheduled{0};
    uint64_t retries_exhausted{0};
    size_t max_queue_count_watermark{0};
    size_t max_payload_bytes_watermark{0};
    uint64_t max_residence_time_ns{0};

    void reset() noexcept;
};

/**
 * @brief Bounded, deterministic Delay-Tolerant Network store-and-forward queue.
 */
class DtnBuffer {
public:
    explicit DtnBuffer(
        const DtnQueueConfig& config = {},
        PacketBudgetManager* budget_manager = nullptr) noexcept;

    void reset(uint64_t initial_time_ns = 0) noexcept;

    const DtnQueueConfig& config() const noexcept { return config_; }
    void set_config(const DtnQueueConfig& config) noexcept {
        config_ = config;
        rtt_estimator_.set_config(config.rtt_config);
    }

    const RttEstimator& rtt_estimator() const noexcept { return rtt_estimator_; }
    RttEstimator& rtt_estimator() noexcept { return rtt_estimator_; }

    const DtnQueueStatistics& statistics() const noexcept { return stats_; }
    PacketBudgetManager* budget_manager() const noexcept { return budget_manager_; }
    void set_budget_manager(PacketBudgetManager* mgr) noexcept { budget_manager_ = mgr; }

    size_t message_count() const noexcept { return queued_messages_.size(); }
    size_t payload_bytes() const noexcept { return current_payload_bytes_; }
    bool empty() const noexcept { return queued_messages_.empty(); }

    /**
     * @brief Enqueues a message into the store-and-forward buffer.
     *        Enforces deduplication, TTL validation, capacity boundaries, and optional byte quotas.
     */
    EnqueueResult enqueue(DtnMessage message, uint64_t now_ns);

    /**
     * @brief Returns and removes the next eligible message for transmission according to priority & fairness.
     * @param now_ns Current simulated timestamp.
     * @param link_checker Optional callback returning true if bilateral link to destination rover is connected.
     * @return Next eligible message, or std::nullopt if none is currently eligible or reachable.
     */
    std::optional<DtnMessage> dequeue_next_eligible(
        uint64_t now_ns,
        std::function<bool(uint16_t target)> link_checker = nullptr);

    /**
     * @brief Peeks at the next eligible message without removing it from the buffer.
     */
    std::optional<DtnMessage> peek_next_eligible(
        uint64_t now_ns,
        std::function<bool(uint16_t target)> link_checker = nullptr) const;

    /**
     * @brief Marks a transmission attempt for a message currently in-flight or buffered.
     * @param message_id Message identifier.
     * @param now_ns Current simulated timestamp.
     * @param success If true, marks delivery complete (acknowledges and removes message).
     *                If false, advances retry schedule or drops if retries exhausted/expired.
     */
    bool mark_transmission_attempt(uint64_t message_id, uint64_t now_ns, bool success);

    /**
     * @brief Explicitly acknowledges a delivered message by ID, removing it from the buffer.
     */
    AckResult acknowledge(uint64_t message_id, uint64_t now_ns);

    /**
     * @brief Purges all messages that have expired at now_ns (now_ns >= expiry_time_ns).
     * @return Number of expired messages purged.
     */
    size_t purge_expired(uint64_t now_ns);

    /**
     * @brief Checks if a message ID is currently held in the deduplication cache.
     */
    bool is_duplicate(uint64_t message_id, uint64_t now_ns) const;

    /**
     * @brief Records a delivered message ID in the deduplication cache with configured TTL.
     */
    void record_delivered_dedup(uint64_t message_id, uint64_t now_ns);

private:
    struct DedupRecord {
        uint64_t message_id{0};
        uint64_t expiry_time_ns{0};
    };

    void purge_dedup_cache(uint64_t now_ns);
    PriorityLevel compute_effective_priority(const DtnMessage& msg, uint64_t now_ns) const noexcept;
    bool evict_for_capacity(PriorityLevel incoming_prio, size_t needed_bytes);
    uint64_t get_base_rto_ns() const noexcept;

    DtnQueueConfig config_;
    PacketBudgetManager* budget_manager_{nullptr};
    DtnQueueStatistics stats_;
    RttEstimator rtt_estimator_;

    size_t current_payload_bytes_{0};

    // Queued messages: stored in a deque for stable traversal and FIFO ordering within priorities
    std::deque<DtnMessage> queued_messages_;

    // Deduplication cache: message_id -> expiry_time_ns
    std::deque<DedupRecord> dedup_cache_fifo_;
    std::map<uint64_t, uint64_t> dedup_cache_map_;

    // Weighted round-robin deficit counters for WeightedInterleaving
    uint32_t wrr_deficit_high_{0};
    uint32_t wrr_deficit_normal_{0};
    uint32_t wrr_deficit_low_{0};
};

} // namespace astromesh::comms
