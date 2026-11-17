#include "astromesh/comms/packet_budget.hpp"

#include <algorithm>
#include <limits>

namespace astromesh::comms {

const char* to_string(QuotaCheckResult result) noexcept {
    switch (result) {
        case QuotaCheckResult::Approved: return "Approved";
        case QuotaCheckResult::ExceedsWindowBudget: return "ExceedsWindowBudget";
        case QuotaCheckResult::ExceedsPeerQuota: return "ExceedsPeerQuota";
        case QuotaCheckResult::ExceedsMaxMessageSize: return "ExceedsMaxMessageSize";
        case QuotaCheckResult::InvalidMessageSize: return "InvalidMessageSize";
    }
    return "Unknown";
}

const char* to_string(PriorityLevel priority) noexcept {
    switch (priority) {
        case PriorityLevel::Low: return "Low";
        case PriorityLevel::Normal: return "Normal";
        case PriorityLevel::High: return "High";
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

} // namespace

void TrafficStatistics::add(const ByteBreakdown& bytes) noexcept {
    payload_bytes = saturating_add(payload_bytes, bytes.payload_bytes);
    header_bytes = saturating_add(header_bytes, bytes.header_bytes);
    transport_overhead_bytes = saturating_add(transport_overhead_bytes, bytes.transport_overhead_bytes);
    total_wire_bytes = saturating_add(total_wire_bytes, bytes.total_wire_bytes());
    message_count = saturating_add(message_count, 1);
}

void TrafficStatistics::reset() noexcept {
    *this = TrafficStatistics();
}

void BudgetStatistics::reset() noexcept {
    sent.reset();
    received.reset();
    retransmitted.reset();
    rejected.reset();
    network_dropped.reset();
    per_peer_sent.clear();
    for (auto& prio_stat : per_priority_sent) {
        prio_stat.reset();
    }
}

PacketBudgetManager::PacketBudgetManager(const ByteBudgetConfig& config) noexcept
    : config_(config) {}

void PacketBudgetManager::reset(uint64_t initial_time_ns) noexcept {
    stats_.reset();
    window_start_ns_ = initial_time_ns;
    current_window_wire_bytes_ = 0;
    current_window_peer_bytes_.clear();
}

ByteBreakdown PacketBudgetManager::resolve_breakdown(
    size_t payload,
    size_t custom_header,
    size_t custom_transport) const noexcept {
    ByteBreakdown b;
    b.payload_bytes = payload;
    b.header_bytes = (custom_header > 0) ? custom_header : config_.default_header_bytes;
    b.transport_overhead_bytes = (custom_transport > 0) ? custom_transport : config_.default_transport_overhead_bytes;
    return b;
}

void PacketBudgetManager::update_window(uint64_t now_ns) noexcept {
    if (config_.window_duration_ns == 0) {
        return; // Unlimited window
    }

    if (now_ns < window_start_ns_) {
        // Monotonic clock reset or jump backwards: anchor to now_ns
        window_start_ns_ = now_ns;
        current_window_wire_bytes_ = 0;
        current_window_peer_bytes_.clear();
        return;
    }

    const uint64_t elapsed = now_ns - window_start_ns_;
    if (elapsed >= config_.window_duration_ns) {
        const uint64_t windows_passed = elapsed / config_.window_duration_ns;
        window_start_ns_ = saturating_add(window_start_ns_, windows_passed * config_.window_duration_ns);
        current_window_wire_bytes_ = 0;
        current_window_peer_bytes_.clear();
    }
}

QuotaCheckResult PacketBudgetManager::check_quota(
    uint16_t peer_id,
    size_t payload_bytes,
    uint64_t now_ns,
    size_t custom_header_bytes,
    size_t custom_transport_bytes) noexcept {

    if (config_.max_message_payload_bytes > 0 && payload_bytes > config_.max_message_payload_bytes) {
        return QuotaCheckResult::ExceedsMaxMessageSize;
    }

    update_window(now_ns);

    const ByteBreakdown breakdown = resolve_breakdown(payload_bytes, custom_header_bytes, custom_transport_bytes);
    const uint64_t msg_wire_bytes = static_cast<uint64_t>(breakdown.total_wire_bytes());

    if (config_.max_bytes_per_window > 0) {
        if (saturating_add(current_window_wire_bytes_, msg_wire_bytes) > config_.max_bytes_per_window) {
            return QuotaCheckResult::ExceedsWindowBudget;
        }
    }

    if (config_.per_peer_quota_bytes > 0) {
        auto it = current_window_peer_bytes_.find(peer_id);
        const uint64_t peer_current = (it != current_window_peer_bytes_.end()) ? it->second : 0;
        if (saturating_add(peer_current, msg_wire_bytes) > config_.per_peer_quota_bytes) {
            return QuotaCheckResult::ExceedsPeerQuota;
        }
    }

    return QuotaCheckResult::Approved;
}

bool PacketBudgetManager::record_sent(
    uint16_t peer_id,
    PriorityLevel priority,
    size_t payload_bytes,
    uint64_t now_ns,
    bool is_retransmission,
    size_t custom_header_bytes,
    size_t custom_transport_bytes) noexcept {

    const QuotaCheckResult check = check_quota(
        peer_id, payload_bytes, now_ns, custom_header_bytes, custom_transport_bytes);

    if (check != QuotaCheckResult::Approved) {
        record_rejected(peer_id, priority, payload_bytes, check, custom_header_bytes, custom_transport_bytes);
        return false;
    }

    const ByteBreakdown breakdown = resolve_breakdown(payload_bytes, custom_header_bytes, custom_transport_bytes);
    const uint64_t msg_wire_bytes = static_cast<uint64_t>(breakdown.total_wire_bytes());

    stats_.sent.add(breakdown);
    if (is_retransmission) {
        stats_.retransmitted.add(breakdown);
    }

    stats_.per_peer_sent[peer_id].add(breakdown);

    const size_t prio_idx = std::min(static_cast<size_t>(priority), size_t{2});
    stats_.per_priority_sent[prio_idx].add(breakdown);

    current_window_wire_bytes_ = saturating_add(current_window_wire_bytes_, msg_wire_bytes);
    current_window_peer_bytes_[peer_id] = saturating_add(current_window_peer_bytes_[peer_id], msg_wire_bytes);

    return true;
}

void PacketBudgetManager::record_received(
    [[maybe_unused]] uint16_t sender_id,
    [[maybe_unused]] PriorityLevel priority,
    size_t payload_bytes,
    [[maybe_unused]] uint64_t now_ns,
    size_t custom_header_bytes,
    size_t custom_transport_bytes) noexcept {

    const ByteBreakdown breakdown = resolve_breakdown(payload_bytes, custom_header_bytes, custom_transport_bytes);
    stats_.received.add(breakdown);
}

void PacketBudgetManager::record_rejected(
    [[maybe_unused]] uint16_t peer_id,
    [[maybe_unused]] PriorityLevel priority,
    size_t payload_bytes,
    [[maybe_unused]] QuotaCheckResult reason,
    size_t custom_header_bytes,
    size_t custom_transport_bytes) noexcept {

    const ByteBreakdown breakdown = resolve_breakdown(payload_bytes, custom_header_bytes, custom_transport_bytes);
    stats_.rejected.add(breakdown);
}

void PacketBudgetManager::record_network_drop(
    [[maybe_unused]] uint16_t peer_id,
    [[maybe_unused]] PriorityLevel priority,
    size_t payload_bytes,
    size_t custom_header_bytes,
    size_t custom_transport_bytes) noexcept {

    const ByteBreakdown breakdown = resolve_breakdown(payload_bytes, custom_header_bytes, custom_transport_bytes);
    stats_.network_dropped.add(breakdown);
}

} // namespace astromesh::comms
