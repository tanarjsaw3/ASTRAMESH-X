#pragma once

#include "astromesh/net/udp_socket.hpp"

#include <cstdint>
#include <deque>
#include <optional>
#include <unordered_map>
#include <vector>

namespace astromesh::net {

/**
 * @brief Live state and reception statistics for an active peer rover.
 */
struct PeerState {
    uint16_t rover_id{0};
    Endpoint endpoint;
    uint64_t last_seen_ns{0};
    uint32_t max_sequence{0};
    uint32_t last_sequence{0};       ///< Maintained for backwards compatibility (mirrors max_sequence)
    uint64_t sequence_bitmap{0};     ///< 64-bit sliding window: bit 0 = max_sequence, bit k = (max_sequence - k)
    bool initialized{false};
    uint32_t packets_received{0};
    uint32_t duplicates_dropped{0};

    static constexpr size_t WINDOW_SIZE = 64;

    bool is_alive(uint64_t now_ns, uint64_t timeout_ns) const noexcept {
        return (now_ns >= last_seen_ns) && ((now_ns - last_seen_ns) <= timeout_ns);
    }
};

/**
 * @brief Manages peer discovery, heartbeat vitality, endpoint mapping, and duplicate suppression.
 */
class PeerTracker {
public:
    PeerTracker() = default;

    /**
     * @brief Checks if a received sequence number is a duplicate from the given rover.
     */
    bool is_duplicate(uint16_t rover_id, uint32_t sequence) const;

    /**
     * @brief Records a valid incoming packet, updating peer state, endpoint, and sequence window.
     */
    void record_packet(uint16_t rover_id, uint32_t sequence, const Endpoint& ep, uint64_t timestamp_ns);

    /**
     * @brief Registers or manually configures a peer endpoint.
     */
    void register_peer(uint16_t rover_id, const Endpoint& ep);

    /**
     * @brief Returns list of rover IDs that have sent heartbeats within the timeout window.
     */
    std::vector<uint16_t> get_active_peers(uint64_t now_ns, uint64_t timeout_ns) const;

    /**
     * @brief Returns the most recently seen endpoint for a peer rover.
     */
    std::optional<Endpoint> get_peer_endpoint(uint16_t rover_id) const;

    /**
     * @brief Retrieves detailed state pointer for a peer rover.
     */
    const PeerState* get_peer_state(uint16_t rover_id) const;

    size_t total_peers() const noexcept { return m_peers.size(); }
    size_t active_peer_count(uint64_t now_ns, uint64_t timeout_ns) const;

    void clear() { m_peers.clear(); }

    /**
     * @brief Resets sequence tracking and vitality for a specific peer rover.
     */
    void reset_peer(uint16_t rover_id);

private:
    std::unordered_map<uint16_t, PeerState> m_peers;
};

} // namespace astromesh::net

