#pragma once

#include "astromesh/net/multihop_envelope.hpp"
#include "astromesh/net/routing_table.hpp"

#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <vector>

namespace astromesh::net {

/**
 * @brief Diagnostic telemetry counters for the multi-hop router.
 */
struct MultihopRouterStats {
    uint64_t packets_received{0};
    uint64_t packets_delivered_locally{0};
    uint64_t packets_forwarded{0};
    uint64_t routing_loops_suppressed{0};
    uint64_t ttl_expired_dropped{0};
    uint64_t unroutable_dropped{0};
    uint64_t malformed_dropped{0};
    uint64_t transit_buffered{0};
    uint64_t transit_drained{0};
};

/**
 * @brief Pending transit packet stored when next hop is temporarily unavailable.
 */
struct TransitPacket {
    MultihopEnvelope envelope;
    uint64_t enqueue_timestamp_ns{0};
    uint64_t expiry_timestamp_ns{0};
};

/**
 * @brief Bounded Multi-Hop Mesh Router.
 * 
 * Inspects incoming MultihopEnvelopes, performs strict path-trail loop suppression,
 * enforces MaxHops TTL boundaries, determines next hops via RoutingTable,
 * delivers locally addressed messages to application handlers, and forwards
 * transit messages with updated hop counts and visited node trails.
 */
class MultihopRouter {
public:
    using ForwardCallback = std::function<bool(const MultihopEnvelope& env, uint16_t next_hop_rover_id)>;
    using LocalDeliveryCallback = std::function<void(const MultihopEnvelope& env)>;
    using CustodyAckCallback = std::function<void(uint16_t upstream_rover_id, uint64_t message_id)>;

    static constexpr size_t MAX_TRANSIT_BUFFER_SIZE = 100;
    static constexpr uint64_t DEFAULT_TRANSIT_TTL_NS = 30'000'000'000ULL; // 30s

    explicit MultihopRouter(uint16_t local_rover_id, RoutingTable& routing_table);

    uint16_t local_rover_id() const noexcept { return m_local_rover_id; }
    const MultihopRouterStats& stats() const noexcept { return m_stats; }
    const MultihopRouterStats& get_stats() const noexcept { return m_stats; }
    void reset_stats() noexcept { m_stats = MultihopRouterStats{}; }

    void set_forward_callback(ForwardCallback cb) { m_forward_cb = std::move(cb); }
    void set_local_delivery_callback(LocalDeliveryCallback cb) { m_local_cb = std::move(cb); }
    void set_custody_ack_callback(CustodyAckCallback cb) { m_custody_ack_cb = std::move(cb); }

    /**
     * @brief Originates a new multi-hop packet from the local rover.
     * Initializes visited_nodes with local_rover_id, path_length=1, hop_count=0.
     * Resolves next hop and forwards.
     */
    bool originate_packet(
        uint16_t final_target_id,
        uint64_t message_id,
        std::vector<uint8_t> payload,
        uint8_t flags = MultihopFlags::NONE,
        uint8_t max_hops = 5,
        uint64_t now_ns = 0);

    /**
     * @brief Ingests an incoming raw buffer containing a MultihopEnvelope.
     */
    bool process_incoming_raw(const uint8_t* data, size_t size, uint16_t immediate_sender_id, uint64_t now_ns = 0);

    /**
     * @brief Ingests an already deserialized MultihopEnvelope.
     */
    bool process_incoming_envelope(MultihopEnvelope env, uint16_t immediate_sender_id, uint64_t now_ns = 0);

    /**
     * @brief Re-evaluates transit queue against updated routing table.
     * Flushes eligible buffered packets whose next hops have become reachable.
     */
    size_t drain_transit_buffer(uint64_t now_ns = 0);

    size_t transit_buffer_size() const noexcept { return m_transit_buffer.size(); }

private:
    bool forward_or_buffer(MultihopEnvelope env, uint64_t now_ns);

    uint16_t m_local_rover_id;
    RoutingTable& m_routing_table;
    MultihopRouterStats m_stats;

    ForwardCallback m_forward_cb;
    LocalDeliveryCallback m_local_cb;
    CustodyAckCallback m_custody_ack_cb;

    std::deque<TransitPacket> m_transit_buffer;
};

} // namespace astromesh::net
