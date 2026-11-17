#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

namespace astromesh::net {

/**
 * @brief Canonical metric indicating an unreachable route (infinity).
 */
constexpr uint8_t ROUTING_INFINITY_METRIC = 16;

/**
 * @brief Maximum permitted distance-vector hop limit (clamped).
 */
constexpr uint8_t ROUTING_MAX_METRIC = 15;

/**
 * @brief Default duration before an unrefreshed route transitions to unreachable.
 */
constexpr uint64_t DEFAULT_ROUTE_TIMEOUT_NS = 1'500'000'000ULL; // 1.5 seconds

/**
 * @brief Distance-vector routing table entry for a single destination.
 */
struct RouteEntry {
    uint16_t destination_rover_id{0};
    uint16_t next_hop_rover_id{0};
    uint8_t metric{ROUTING_INFINITY_METRIC};
    uint32_t destination_sequence_number{0}; ///< Monotonic sequence originated strictly by destination
    uint64_t last_refreshed_ns{0};

    /**
     * @brief Checks if route is currently reachable and not expired.
     */
    bool is_valid(uint64_t now_ns = 0, uint64_t timeout_ns = DEFAULT_ROUTE_TIMEOUT_NS) const noexcept {
        if (metric >= ROUTING_INFINITY_METRIC) return false;
        if (next_hop_rover_id == 0) return false;
        if (destination_rover_id == next_hop_rover_id && metric == 0) return true; // Local self-route
        if (now_ns == 0) return true; // 0 indicates unconstrained / time-agnostic check
        return (now_ns >= last_refreshed_ns) && ((now_ns - last_refreshed_ns) <= timeout_ns);
    }

    bool is_reachable() const noexcept {
        return metric < ROUTING_INFINITY_METRIC;
    }
};

/**
 * @brief Wire-serialized entry inside a distance-vector routing advertisement payload (8 bytes).
 */
#pragma pack(push, 1)
struct RouteAdvertisementRecord {
    uint16_t destination_rover_id{0};
    uint8_t metric{ROUTING_INFINITY_METRIC};
    uint8_t reserved{0};
    uint32_t destination_sequence_number{0};
};
#pragma pack(pop)

static_assert(sizeof(RouteAdvertisementRecord) == 8, "RouteAdvertisementRecord must be exactly 8 bytes");

/**
 * @brief Distance-Vector Routing Table (AstroMesh-DVR).
 *
 * Implements:
 * - Per-destination origin sequence tracking for loop-free freshness.
 * - Split Horizon with Poison Reverse to prevent 2-node counting-to-infinity.
 * - Route expiration and neighbor failure link poisoning.
 * - Deterministic serialization and deserialization of routing advertisements.
 */
class RoutingTable {
public:
    explicit RoutingTable(uint16_t local_rover_id);

    uint16_t get_local_rover_id() const noexcept { return m_local_rover_id; }
    uint32_t get_self_sequence_number() const noexcept { return m_self_sequence_number; }

    /**
     * @brief Increments local sequence number (e.g. periodically or on local topology event).
     */
    void increment_self_sequence_number() noexcept {
        m_self_sequence_number += 2; // Increments by 2 (even numbers denote normal stable state)
        if (auto it = m_routes.find(m_local_rover_id); it != m_routes.end()) {
            it->second.destination_sequence_number = m_self_sequence_number;
        }
    }

    /**
     * @brief Installs or refreshes a direct 1-hop link to an active neighbor.
     * @return true if route was newly added or modified.
     */
    bool install_direct_route(uint16_t neighbor_id, uint64_t now_ns);

    /**
     * @brief Updates or installs a route from a received distance-vector advertisement.
     *
     * Stale route rejection policy:
     * - If dest_seq < existing.destination_sequence_number: REJECT as stale.
     * - If dest_seq > existing.destination_sequence_number: ADOPT fresher sequence.
     * - If dest_seq == existing.destination_sequence_number: ADOPT if candidate metric is strictly lower.
     */
    bool update_route_from_advertisement(
        uint16_t reporting_neighbor_id,
        uint16_t destination_id,
        uint8_t reported_metric,
        uint32_t dest_seq,
        uint64_t now_ns);

    /**
     * @brief Resolves the next-hop rover ID towards destination.
     * @return Next-hop rover ID if route is reachable and valid; nullopt otherwise.
     */
    std::optional<uint16_t> lookup_next_hop(
        uint16_t destination_id,
        uint64_t now_ns,
        uint64_t timeout_ns = DEFAULT_ROUTE_TIMEOUT_NS) const;

    /**
     * @brief Retrieves a read-only copy of a specific route entry.
     */
    std::optional<RouteEntry> get_route(uint16_t destination_id) const;

    /**
     * @brief Marks all routes using the failed neighbor as next-hop as unreachable (metric = 16).
     */
    void invalidate_neighbor_routes(uint16_t failed_neighbor_id, uint64_t now_ns);

    /**
     * @brief Prunes or marks expired routes whose freshness exceeds timeout.
     * @return Number of routes newly poisoned/invalidated.
     */
    size_t prune_stale_routes(uint64_t now_ns, uint64_t timeout_ns = DEFAULT_ROUTE_TIMEOUT_NS);

    /**
     * @brief Generates a binary advertisement payload intended for target_neighbor_id.
     *        Applies Split Horizon with Poison Reverse: routes whose next_hop is target_neighbor_id
     *        are advertised with metric = ROUTING_INFINITY_METRIC (16).
     */
    std::vector<uint8_t> generate_advertisement_payload(uint16_t target_neighbor_id) const;

    /**
     * @brief Ingests and processes a received binary advertisement payload.
     * @return Number of route entries accepted or updated.
     */
    size_t apply_advertisement_payload(
        uint16_t reporting_neighbor_id,
        const uint8_t* data,
        size_t length,
        uint64_t now_ns);

    size_t route_count() const noexcept { return m_routes.size(); }
    std::vector<RouteEntry> get_all_routes() const;

    void clear();

private:
    uint16_t m_local_rover_id{0};
    uint32_t m_self_sequence_number{2};
    std::map<uint16_t, RouteEntry> m_routes;
};

} // namespace astromesh::net

