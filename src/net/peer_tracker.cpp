#include "astromesh/net/peer_tracker.hpp"

#include <algorithm>
#include <chrono>

namespace astromesh::net {

static inline uint64_t get_steady_now_ns() noexcept {
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

bool PeerTracker::is_duplicate(uint16_t rover_id, uint32_t sequence) const {
    auto it = m_peers.find(rover_id);
    if (it == m_peers.end() || !it->second.initialized) {
        return false;
    }
    const auto& peer = it->second;
    const int32_t delta = static_cast<int32_t>(sequence - peer.max_sequence);
    if (delta > 0) {
        // Sequence is newer than current window head
        return false;
    }
    if (delta == 0) {
        // Exactly matches max_sequence (already observed)
        return true;
    }
    // delta < 0: sequence is older than max_sequence
    const int32_t dist = -delta;
    if (dist >= 64) {
        // Beyond the 64-packet window: suppressed as stale duplicate replay
        return true;
    }
    // Within 64-packet window: check bit dist
    return (peer.sequence_bitmap & (1ULL << dist)) != 0;
}

void PeerTracker::record_packet(uint16_t rover_id, uint32_t sequence, const Endpoint& ep, uint64_t timestamp_ns) {
    if (timestamp_ns == 0) timestamp_ns = get_steady_now_ns();

    auto& peer = m_peers[rover_id];
    peer.rover_id = rover_id;
    peer.endpoint = ep;
    peer.last_seen_ns = timestamp_ns;
    peer.packets_received++;

    if (!peer.initialized) {
        peer.max_sequence = sequence;
        peer.last_sequence = sequence;
        peer.sequence_bitmap = 1ULL;
        peer.initialized = true;
        return;
    }

    const int32_t delta = static_cast<int32_t>(sequence - peer.max_sequence);
    if (delta > 0) {
        if (delta < 64) {
            peer.sequence_bitmap = (peer.sequence_bitmap << delta) | 1ULL;
        } else {
            peer.sequence_bitmap = 1ULL;
        }
        peer.max_sequence = sequence;
        peer.last_sequence = sequence;
    } else if (delta == 0) {
        peer.duplicates_dropped++;
    } else {
        const int32_t dist = -delta;
        if (dist < 64) {
            const uint64_t mask = (1ULL << dist);
            if ((peer.sequence_bitmap & mask) != 0) {
                peer.duplicates_dropped++;
            } else {
                peer.sequence_bitmap |= mask;
            }
        } else {
            peer.duplicates_dropped++;
        }
    }
}

void PeerTracker::reset_peer(uint16_t rover_id) {
    auto it = m_peers.find(rover_id);
    if (it != m_peers.end()) {
        it->second.initialized = false;
        it->second.max_sequence = 0;
        it->second.last_sequence = 0;
        it->second.sequence_bitmap = 0;
        it->second.packets_received = 0;
        it->second.duplicates_dropped = 0;
    }
}

void PeerTracker::register_peer(uint16_t rover_id, const Endpoint& ep) {
    auto& peer = m_peers[rover_id];
    peer.rover_id = rover_id;
    peer.endpoint = ep;
}

std::vector<uint16_t> PeerTracker::get_active_peers(uint64_t now_ns, uint64_t timeout_ns) const {
    if (now_ns == 0) now_ns = get_steady_now_ns();

    std::vector<uint16_t> active;
    for (const auto& [id, state] : m_peers) {
        if (id > 0 && state.is_alive(now_ns, timeout_ns)) {
            active.push_back(id);
        }
    }
    std::sort(active.begin(), active.end());
    return active;
}

std::optional<Endpoint> PeerTracker::get_peer_endpoint(uint16_t rover_id) const {
    auto it = m_peers.find(rover_id);
    if (it != m_peers.end() && it->second.endpoint.is_valid()) {
        return it->second.endpoint;
    }
    return std::nullopt;
}

const PeerState* PeerTracker::get_peer_state(uint16_t rover_id) const {
    auto it = m_peers.find(rover_id);
    return (it != m_peers.end()) ? &(it->second) : nullptr;
}

size_t PeerTracker::active_peer_count(uint64_t now_ns, uint64_t timeout_ns) const {
    if (now_ns == 0) now_ns = get_steady_now_ns();

    size_t count = 0;
    for (const auto& [id, state] : m_peers) {
        if (id > 0 && state.is_alive(now_ns, timeout_ns)) {
            count++;
        }
    }
    return count;
}

} // namespace astromesh::net
