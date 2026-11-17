#include "astromesh/comms/dtn_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace astromesh::comms {

const char* to_string(DtnMessageType type) noexcept {
    switch (type) {
        case DtnMessageType::GenericData: return "GenericData";
        case DtnMessageType::BoundaryState: return "BoundaryState";
        case DtnMessageType::IBLTReconciliation: return "IBLTReconciliation";
        case DtnMessageType::LoopClosureProposal: return "LoopClosureProposal";
        case DtnMessageType::Acknowledgment: return "Acknowledgment";
    }
    return "Unknown";
}

std::vector<uint8_t> serialize_ack_payload(const AckPayload& ack) {
    std::vector<uint8_t> buf(24);
    buf[0] = static_cast<uint8_t>(ack.protocol_version & 0xFF);
    buf[1] = static_cast<uint8_t>((ack.protocol_version >> 8) & 0xFF);
    buf[2] = static_cast<uint8_t>(ack.ack_sender_id & 0xFF);
    buf[3] = static_cast<uint8_t>((ack.ack_sender_id >> 8) & 0xFF);
    buf[4] = static_cast<uint8_t>(ack.ack_target_id & 0xFF);
    buf[5] = static_cast<uint8_t>((ack.ack_target_id >> 8) & 0xFF);
    buf[6] = static_cast<uint8_t>(ack.reserved & 0xFF);
    buf[7] = static_cast<uint8_t>((ack.reserved >> 8) & 0xFF);
    for (int i = 0; i < 8; ++i) {
        buf[8 + i] = static_cast<uint8_t>((ack.acked_message_id >> (8 * i)) & 0xFF);
        buf[16 + i] = static_cast<uint8_t>((ack.ack_timestamp_ns >> (8 * i)) & 0xFF);
    }
    return buf;
}

std::optional<AckPayload> deserialize_ack_payload(const uint8_t* data, size_t length) {
    if (!data || length < 24) return std::nullopt;
    AckPayload ack;
    ack.protocol_version = static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
    ack.ack_sender_id = static_cast<uint16_t>(data[2]) | (static_cast<uint16_t>(data[3]) << 8);
    ack.ack_target_id = static_cast<uint16_t>(data[4]) | (static_cast<uint16_t>(data[5]) << 8);
    ack.reserved = static_cast<uint16_t>(data[6]) | (static_cast<uint16_t>(data[7]) << 8);
    ack.acked_message_id = 0;
    ack.ack_timestamp_ns = 0;
    for (int i = 0; i < 8; ++i) {
        ack.acked_message_id |= (static_cast<uint64_t>(data[8 + i]) << (8 * i));
        ack.ack_timestamp_ns |= (static_cast<uint64_t>(data[16 + i]) << (8 * i));
    }
    if (ack.protocol_version != 1) return std::nullopt;
    return ack;
}

std::optional<AckPayload> deserialize_ack_payload(const std::vector<uint8_t>& payload) {
    return deserialize_ack_payload(payload.data(), payload.size());
}

const char* to_string(EnqueueResult result) noexcept {
    switch (result) {
        case EnqueueResult::Success: return "Success";
        case EnqueueResult::DuplicateIgnored: return "DuplicateIgnored";
        case EnqueueResult::CapacityCountExceeded: return "CapacityCountExceeded";
        case EnqueueResult::CapacityBytesExceeded: return "CapacityBytesExceeded";
        case EnqueueResult::AlreadyExpired: return "AlreadyExpired";
        case EnqueueResult::InvalidMetadata: return "InvalidMetadata";
        case EnqueueResult::PayloadTooLarge: return "PayloadTooLarge";
        case EnqueueResult::QuotaExceeded: return "QuotaExceeded";
    }
    return "Unknown";
}

const char* to_string(AckResult result) noexcept {
    switch (result) {
        case AckResult::Acknowledged: return "Acknowledged";
        case AckResult::NotFound: return "NotFound";
    }
    return "Unknown";
}

const char* to_string(FairnessPolicy policy) noexcept {
    switch (policy) {
        case FairnessPolicy::StrictPriority: return "StrictPriority";
        case FairnessPolicy::WeightedInterleaving: return "WeightedInterleaving";
        case FairnessPolicy::PriorityAging: return "PriorityAging";
    }
    return "Unknown";
}

namespace {

inline uint64_t saturating_add(uint64_t a, uint64_t b) noexcept {
    if (std::numeric_limits<uint64_t>::max() - a < b) {
        return std::numeric_limits<uint64_t>::max();
    }
    return a + b;
}

uint64_t compute_backoff_delay(
    uint32_t attempt,
    uint64_t initial_delay_ns,
    double multiplier,
    uint64_t max_delay_ns) noexcept {

    if (attempt <= 1) {
        return std::min(initial_delay_ns, max_delay_ns);
    }

    const double mult_factor = std::pow(std::max(1.0, multiplier), static_cast<double>(attempt - 1));
    const double raw_delay = static_cast<double>(initial_delay_ns) * mult_factor;

    if (raw_delay >= static_cast<double>(max_delay_ns) || raw_delay >= static_cast<double>(std::numeric_limits<uint64_t>::max())) {
        return max_delay_ns;
    }

    return std::min(static_cast<uint64_t>(raw_delay), max_delay_ns);
}

} // namespace

void DtnQueueStatistics::reset() noexcept {
    *this = DtnQueueStatistics();
}

DtnBuffer::DtnBuffer(
    const DtnQueueConfig& config,
    PacketBudgetManager* budget_manager) noexcept
    : config_(config), budget_manager_(budget_manager), rtt_estimator_(config.rtt_config) {
    if (config_.initial_retry_delay_ns != 50'000'000ULL && config_.rtt_config.initial_rto_ns == 50'000'000ULL) {
        auto rcfg = config_.rtt_config;
        rcfg.initial_rto_ns = config_.initial_retry_delay_ns;
        rcfg.max_rto_ns = config_.max_retry_delay_ns;
        rtt_estimator_.set_config(rcfg);
    }
    wrr_deficit_high_ = config_.high_priority_weight;
    wrr_deficit_normal_ = config_.normal_priority_weight;
    wrr_deficit_low_ = config_.low_priority_weight;
}

void DtnBuffer::reset(uint64_t initial_time_ns) noexcept {
    queued_messages_.clear();
    current_payload_bytes_ = 0;
    dedup_cache_fifo_.clear();
    dedup_cache_map_.clear();
    stats_.reset();
    rtt_estimator_.reset();

    wrr_deficit_high_ = config_.high_priority_weight;
    wrr_deficit_normal_ = config_.normal_priority_weight;
    wrr_deficit_low_ = config_.low_priority_weight;

    if (budget_manager_) {
        budget_manager_->reset(initial_time_ns);
    }
}

void DtnBuffer::purge_dedup_cache(uint64_t now_ns) {
    while (!dedup_cache_fifo_.empty()) {
        const auto& front = dedup_cache_fifo_.front();
        if (now_ns >= front.expiry_time_ns) {
            auto it = dedup_cache_map_.find(front.message_id);
            if (it != dedup_cache_map_.end() && it->second == front.expiry_time_ns) {
                dedup_cache_map_.erase(it);
            }
            dedup_cache_fifo_.pop_front();
        } else {
            break;
        }
    }
}

bool DtnBuffer::is_duplicate(uint64_t message_id, uint64_t now_ns) const {
    auto it = dedup_cache_map_.find(message_id);
    if (it != dedup_cache_map_.end()) {
        return now_ns < it->second;
    }
    return false;
}

void DtnBuffer::record_delivered_dedup(uint64_t message_id, uint64_t now_ns) {
    if (config_.dedup_cache_capacity == 0) return;
    purge_dedup_cache(now_ns);
    const uint64_t dedup_exp = saturating_add(now_ns, config_.dedup_cache_ttl_ns);
    dedup_cache_fifo_.push_back({message_id, dedup_exp});
    dedup_cache_map_[message_id] = dedup_exp;

    while (dedup_cache_fifo_.size() > config_.dedup_cache_capacity) {
        const auto front = dedup_cache_fifo_.front();
        dedup_cache_fifo_.pop_front();
        auto mit = dedup_cache_map_.find(front.message_id);
        if (mit != dedup_cache_map_.end() && mit->second == front.expiry_time_ns) {
            dedup_cache_map_.erase(mit);
        }
    }
}

size_t DtnBuffer::purge_expired(uint64_t now_ns) {
    size_t count = 0;
    auto it = queued_messages_.begin();
    while (it != queued_messages_.end()) {
        if (it->is_expired(now_ns)) {
            stats_.messages_expired++;
            stats_.payload_bytes_expired += it->payload_bytes();
            current_payload_bytes_ -= it->payload_bytes();
            it = queued_messages_.erase(it);
            count++;
        } else {
            ++it;
        }
    }
    return count;
}

PriorityLevel DtnBuffer::compute_effective_priority(const DtnMessage& msg, uint64_t now_ns) const noexcept {
    if (config_.fairness_policy != FairnessPolicy::PriorityAging || config_.aging_interval_ns == 0) {
        return msg.priority;
    }

    const uint64_t elapsed = (now_ns >= msg.enqueue_time_ns) ? (now_ns - msg.enqueue_time_ns) : 0;
    const uint64_t levels_promoted = elapsed / config_.aging_interval_ns;

    if (msg.priority == PriorityLevel::Low) {
        if (levels_promoted >= 2) return PriorityLevel::High;
        if (levels_promoted >= 1) return PriorityLevel::Normal;
        return PriorityLevel::Low;
    } else if (msg.priority == PriorityLevel::Normal) {
        if (levels_promoted >= 1) return PriorityLevel::High;
        return PriorityLevel::Normal;
    }

    return PriorityLevel::High;
}

bool DtnBuffer::evict_for_capacity(PriorityLevel incoming_prio, size_t needed_bytes) {
    if (!config_.enable_priority_eviction) {
        return false;
    }

    while (true) {
        const bool count_ok = (config_.max_queue_count == 0 || queued_messages_.size() < config_.max_queue_count);
        const bool bytes_ok = (config_.max_payload_bytes == 0 || current_payload_bytes_ + needed_bytes <= config_.max_payload_bytes);
        if (count_ok && bytes_ok) {
            return true;
        }

        // Find oldest message with priority strictly lower than incoming_prio
        auto victim_it = queued_messages_.end();
        PriorityLevel lowest_prio = incoming_prio;

        for (auto it = queued_messages_.begin(); it != queued_messages_.end(); ++it) {
            if (static_cast<uint8_t>(it->priority) < static_cast<uint8_t>(lowest_prio)) {
                lowest_prio = it->priority;
                victim_it = it;
            }
        }

        if (victim_it == queued_messages_.end()) {
            // No lower priority victim found
            return false;
        }

        // Evict victim
        stats_.messages_evicted++;
        stats_.payload_bytes_evicted += victim_it->payload_bytes();
        current_payload_bytes_ -= victim_it->payload_bytes();
        queued_messages_.erase(victim_it);
    }
}

EnqueueResult DtnBuffer::enqueue(DtnMessage message, uint64_t now_ns) {
    // 1. Validate metadata
    if (message.message_id == 0 || message.sender_rover_id == 0) {
        return EnqueueResult::InvalidMetadata;
    }

    if (config_.max_message_payload_bytes > 0 && message.payload_bytes() > config_.max_message_payload_bytes) {
        return EnqueueResult::PayloadTooLarge;
    }

    if (message.ttl_ns == 0) {
        return EnqueueResult::AlreadyExpired;
    }

    // 2. Purge expired queued messages and stale deduplication entries
    purge_expired(now_ns);
    purge_dedup_cache(now_ns);

    // 3. Duplicate suppression
    if (is_duplicate(message.message_id, now_ns)) {
        stats_.duplicates_suppressed++;
        return EnqueueResult::DuplicateIgnored;
    }

    // 4. Setup timestamps
    message.enqueue_time_ns = now_ns;
    message.expiry_time_ns = saturating_add(now_ns, message.ttl_ns);
    message.next_retry_time_ns = now_ns;

    if (message.is_expired(now_ns)) {
        return EnqueueResult::AlreadyExpired;
    }

    // 5. Quota check
    if (budget_manager_) {
        const QuotaCheckResult qres = budget_manager_->check_quota(
            message.target_rover_id, message.payload_bytes(), now_ns);
        if (qres != QuotaCheckResult::Approved) {
            stats_.messages_rejected_quota++;
            stats_.payload_bytes_rejected_quota += message.payload_bytes();
            return EnqueueResult::QuotaExceeded;
        }
    }

    // 6. Capacity check with optional eviction
    bool count_ok = (config_.max_queue_count == 0 || queued_messages_.size() < config_.max_queue_count);
    bool bytes_ok = (config_.max_payload_bytes == 0 || current_payload_bytes_ + message.payload_bytes() <= config_.max_payload_bytes);

    if (!count_ok || !bytes_ok) {
        if (config_.enable_priority_eviction) {
            evict_for_capacity(message.priority, message.payload_bytes());
            count_ok = (config_.max_queue_count == 0 || queued_messages_.size() < config_.max_queue_count);
            bytes_ok = (config_.max_payload_bytes == 0 || current_payload_bytes_ + message.payload_bytes() <= config_.max_payload_bytes);
        }

        if (!count_ok) {
            stats_.messages_rejected_capacity++;
            stats_.payload_bytes_rejected_capacity += message.payload_bytes();
            return EnqueueResult::CapacityCountExceeded;
        }
        if (!bytes_ok) {
            stats_.messages_rejected_capacity++;
            stats_.payload_bytes_rejected_capacity += message.payload_bytes();
            return EnqueueResult::CapacityBytesExceeded;
        }
    }

    // 7. Enqueue message
    current_payload_bytes_ += message.payload_bytes();
    queued_messages_.push_back(message);

    stats_.messages_enqueued++;
    stats_.payload_bytes_enqueued += message.payload_bytes();
    stats_.max_queue_count_watermark = std::max(stats_.max_queue_count_watermark, queued_messages_.size());
    stats_.max_payload_bytes_watermark = std::max(stats_.max_payload_bytes_watermark, current_payload_bytes_);

    // 8. Record in deduplication cache
    if (config_.dedup_cache_capacity > 0) {
        const uint64_t dedup_exp = saturating_add(now_ns, config_.dedup_cache_ttl_ns);
        dedup_cache_fifo_.push_back({message.message_id, dedup_exp});
        dedup_cache_map_[message.message_id] = dedup_exp;

        while (dedup_cache_fifo_.size() > config_.dedup_cache_capacity) {
            const auto front = dedup_cache_fifo_.front();
            dedup_cache_fifo_.pop_front();
            auto mit = dedup_cache_map_.find(front.message_id);
            if (mit != dedup_cache_map_.end() && mit->second == front.expiry_time_ns) {
                dedup_cache_map_.erase(mit);
            }
        }
    }

    return EnqueueResult::Success;
}

std::optional<DtnMessage> DtnBuffer::peek_next_eligible(
    uint64_t now_ns,
    std::function<bool(uint16_t target)> link_checker) const {

    if (queued_messages_.empty()) {
        return std::nullopt;
    }

    auto is_candidate_eligible = [&](const DtnMessage& msg) -> bool {
        if (!msg.is_eligible(now_ns)) return false;
        if (link_checker && msg.target_rover_id != 0) {
            if (!link_checker(msg.target_rover_id)) return false;
        }
        return true;
    };

    if (config_.fairness_policy == FairnessPolicy::StrictPriority) {
        // High -> Normal -> Low (FIFO within tier)
        for (int p = 2; p >= 0; --p) {
            const PriorityLevel target_prio = static_cast<PriorityLevel>(p);
            for (const auto& msg : queued_messages_) {
                if (msg.priority == target_prio && is_candidate_eligible(msg)) {
                    return msg;
                }
            }
        }
        return std::nullopt;
    }

    if (config_.fairness_policy == FairnessPolicy::PriorityAging) {
        const DtnMessage* best_msg = nullptr;
        PriorityLevel best_eff_prio = PriorityLevel::Low;

        for (const auto& msg : queued_messages_) {
            if (!is_candidate_eligible(msg)) continue;
            const PriorityLevel eff_prio = compute_effective_priority(msg, now_ns);
            if (best_msg == nullptr || static_cast<uint8_t>(eff_prio) > static_cast<uint8_t>(best_eff_prio)) {
                best_msg = &msg;
                best_eff_prio = eff_prio;
            }
        }
        if (best_msg) return *best_msg;
        return std::nullopt;
    }

    // WeightedInterleaving peek: mirrors selection order
    bool has_high = false, has_normal = false, has_low = false;
    for (const auto& msg : queued_messages_) {
        if (!is_candidate_eligible(msg)) continue;
        if (msg.priority == PriorityLevel::High) has_high = true;
        else if (msg.priority == PriorityLevel::Normal) has_normal = true;
        else if (msg.priority == PriorityLevel::Low) has_low = true;
    }

    if (!has_high && !has_normal && !has_low) {
        return std::nullopt;
    }

    // Check deficits
    auto pick_tier = [&](PriorityLevel prio) -> const DtnMessage* {
        for (const auto& msg : queued_messages_) {
            if (msg.priority == prio && is_candidate_eligible(msg)) return &msg;
        }
        return nullptr;
    };

    if (has_high && wrr_deficit_high_ > 0) return *pick_tier(PriorityLevel::High);
    if (has_normal && wrr_deficit_normal_ > 0) return *pick_tier(PriorityLevel::Normal);
    if (has_low && wrr_deficit_low_ > 0) return *pick_tier(PriorityLevel::Low);

    // If deficits depleted, reset cycle check
    if (has_high) return *pick_tier(PriorityLevel::High);
    if (has_normal) return *pick_tier(PriorityLevel::Normal);
    if (has_low) return *pick_tier(PriorityLevel::Low);

    return std::nullopt;
}

uint64_t DtnBuffer::get_base_rto_ns() const noexcept {
    if (rtt_estimator_.has_measurement()) {
        const double raw_rto = static_cast<double>(rtt_estimator_.srtt_ns()) + config_.rtt_config.k * static_cast<double>(rtt_estimator_.rttvar_ns());
        const uint64_t rto = static_cast<uint64_t>(std::round(raw_rto));
        return std::clamp(rto, config_.rtt_config.min_rto_ns, config_.rtt_config.max_rto_ns);
    }
    return config_.initial_retry_delay_ns;
}

std::optional<DtnMessage> DtnBuffer::dequeue_next_eligible(
    uint64_t now_ns,
    std::function<bool(uint16_t target)> link_checker) {

    purge_expired(now_ns);

    if (queued_messages_.empty()) {
        return std::nullopt;
    }

    auto is_candidate_eligible = [&](const DtnMessage& msg) -> bool {
        if (!msg.is_eligible(now_ns)) return false;
        if (link_checker && msg.target_rover_id != 0) {
            if (!link_checker(msg.target_rover_id)) return false;
        }
        return true;
    };

    auto it_selected = queued_messages_.end();

    if (config_.fairness_policy == FairnessPolicy::StrictPriority) {
        for (int p = 2; p >= 0 && it_selected == queued_messages_.end(); --p) {
            const PriorityLevel target_prio = static_cast<PriorityLevel>(p);
            for (auto it = queued_messages_.begin(); it != queued_messages_.end(); ++it) {
                if (it->priority == target_prio && is_candidate_eligible(*it)) {
                    it_selected = it;
                    break;
                }
            }
        }
    } else if (config_.fairness_policy == FairnessPolicy::PriorityAging) {
        PriorityLevel best_eff_prio = PriorityLevel::Low;
        for (auto it = queued_messages_.begin(); it != queued_messages_.end(); ++it) {
            if (!is_candidate_eligible(*it)) continue;
            const PriorityLevel eff_prio = compute_effective_priority(*it, now_ns);
            if (it_selected == queued_messages_.end() || static_cast<uint8_t>(eff_prio) > static_cast<uint8_t>(best_eff_prio)) {
                it_selected = it;
                best_eff_prio = eff_prio;
            }
        }
    } else { // WeightedInterleaving
        bool has_high = false, has_normal = false, has_low = false;
        for (const auto& msg : queued_messages_) {
            if (!is_candidate_eligible(msg)) continue;
            if (msg.priority == PriorityLevel::High) has_high = true;
            else if (msg.priority == PriorityLevel::Normal) has_normal = true;
            else if (msg.priority == PriorityLevel::Low) has_low = true;
        }

        if (!has_high && !has_normal && !has_low) {
            return std::nullopt;
        }

        if (wrr_deficit_high_ == 0 && wrr_deficit_normal_ == 0 && wrr_deficit_low_ == 0) {
            wrr_deficit_high_ = config_.high_priority_weight;
            wrr_deficit_normal_ = config_.normal_priority_weight;
            wrr_deficit_low_ = config_.low_priority_weight;
        }

        PriorityLevel target_prio = PriorityLevel::Low;
        if (has_high && wrr_deficit_high_ > 0) {
            target_prio = PriorityLevel::High;
            --wrr_deficit_high_;
        } else if (has_normal && wrr_deficit_normal_ > 0) {
            target_prio = PriorityLevel::Normal;
            --wrr_deficit_normal_;
        } else if (has_low && wrr_deficit_low_ > 0) {
            target_prio = PriorityLevel::Low;
            --wrr_deficit_low_;
        } else {
            // Recharge and reselect
            wrr_deficit_high_ = config_.high_priority_weight;
            wrr_deficit_normal_ = config_.normal_priority_weight;
            wrr_deficit_low_ = config_.low_priority_weight;

            if (has_high && wrr_deficit_high_ > 0) {
                target_prio = PriorityLevel::High;
                --wrr_deficit_high_;
            } else if (has_normal && wrr_deficit_normal_ > 0) {
                target_prio = PriorityLevel::Normal;
                --wrr_deficit_normal_;
            } else {
                target_prio = PriorityLevel::Low;
                --wrr_deficit_low_;
            }
        }

        for (auto it = queued_messages_.begin(); it != queued_messages_.end(); ++it) {
            if (it->priority == target_prio && is_candidate_eligible(*it)) {
                it_selected = it;
                break;
            }
        }
    }

    if (it_selected == queued_messages_.end()) {
        return std::nullopt;
    }

    // Mark attempt and prepare transmission
    DtnMessage selected = *it_selected;
    it_selected->attempt_count++;
    it_selected->last_transmit_time_ns = now_ns;
    selected.attempt_count = it_selected->attempt_count;
    selected.last_transmit_time_ns = now_ns;

    // Adaptive Jacobson/Karn RTO calculation with backoff
    const uint64_t base_delay = get_base_rto_ns();
    const uint64_t backoff = compute_backoff_delay(
        it_selected->attempt_count,
        base_delay,
        config_.backoff_multiplier,
        config_.max_retry_delay_ns);
    it_selected->next_retry_time_ns = saturating_add(now_ns, backoff);

    const uint64_t residence_time = (now_ns >= selected.enqueue_time_ns) ? (now_ns - selected.enqueue_time_ns) : 0;
    stats_.max_residence_time_ns = std::max(stats_.max_residence_time_ns, residence_time);

    return selected;
}

bool DtnBuffer::mark_transmission_attempt(uint64_t message_id, uint64_t now_ns, bool success) {
    auto it = std::find_if(queued_messages_.begin(), queued_messages_.end(),
        [message_id](const DtnMessage& m) { return m.message_id == message_id; });

    if (it == queued_messages_.end()) {
        return false;
    }

    if (success) {
        if (it->last_transmit_time_ns > 0 && now_ns >= it->last_transmit_time_ns) {
            const uint64_t measured_rtt = now_ns - it->last_transmit_time_ns;
            rtt_estimator_.update_rtt(measured_rtt, it->attempt_count);
        }
        stats_.messages_delivered++;
        stats_.payload_bytes_delivered += it->payload_bytes();
        current_payload_bytes_ -= it->payload_bytes();
        queued_messages_.erase(it);
        return true;
    }

    // Failure: backoff RTO and check if retries exhausted or expired
    rtt_estimator_.backoff(config_.backoff_multiplier);

    if (it->attempt_count >= config_.max_attempts) {
        stats_.retries_exhausted++;
        current_payload_bytes_ -= it->payload_bytes();
        queued_messages_.erase(it);
        return false;
    }

    const uint64_t base_delay = get_base_rto_ns();
    const uint64_t backoff = compute_backoff_delay(
        it->attempt_count + 1,
        base_delay,
        config_.backoff_multiplier,
        config_.max_retry_delay_ns);

    it->next_retry_time_ns = saturating_add(now_ns, backoff);

    if (it->is_expired(it->next_retry_time_ns)) {
        // Will expire before next retry
        stats_.messages_expired++;
        stats_.payload_bytes_expired += it->payload_bytes();
        current_payload_bytes_ -= it->payload_bytes();
        queued_messages_.erase(it);
        return false;
    }

    stats_.retry_attempts_scheduled++;
    return false;
}

AckResult DtnBuffer::acknowledge(uint64_t message_id, [[maybe_unused]] uint64_t now_ns) {
    auto it = std::find_if(queued_messages_.begin(), queued_messages_.end(),
        [message_id](const DtnMessage& m) { return m.message_id == message_id; });

    if (it == queued_messages_.end()) {
        return AckResult::NotFound;
    }

    if (it->last_transmit_time_ns > 0 && now_ns >= it->last_transmit_time_ns) {
        const uint64_t measured_rtt = now_ns - it->last_transmit_time_ns;
        rtt_estimator_.update_rtt(measured_rtt, it->attempt_count);
    }

    stats_.messages_delivered++;
    stats_.payload_bytes_delivered += it->payload_bytes();
    current_payload_bytes_ -= it->payload_bytes();
    queued_messages_.erase(it);
    return AckResult::Acknowledged;
}

} // namespace astromesh::comms
