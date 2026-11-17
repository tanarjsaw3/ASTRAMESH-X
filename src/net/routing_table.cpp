#include "astromesh/net/routing_table.hpp"

#include <algorithm>
#include <cstring>

namespace astromesh::net {

RoutingTable::RoutingTable(uint16_t local_rover_id)
    : m_local_rover_id(local_rover_id) {
    // Install permanent self-route (metric = 0, next_hop = self)
    RouteEntry self_route;
    self_route.destination_rover_id = m_local_rover_id;
    self_route.next_hop_rover_id = m_local_rover_id;
    self_route.metric = 0;
    self_route.destination_sequence_number = m_self_sequence_number;
    self_route.last_refreshed_ns = 0;
    m_routes[m_local_rover_id] = self_route;
}

bool RoutingTable::install_direct_route(uint16_t neighbor_id, uint64_t now_ns) {
    if (neighbor_id == 0 || neighbor_id == m_local_rover_id) {
        return false;
    }

    auto it = m_routes.find(neighbor_id);
    if (it != m_routes.end()) {
        bool changed = (it->second.next_hop_rover_id != neighbor_id) || (it->second.metric != 1);
        it->second.next_hop_rover_id = neighbor_id;
        it->second.metric = 1;
        it->second.last_refreshed_ns = now_ns;
        return changed;
    }

    RouteEntry entry;
    entry.destination_rover_id = neighbor_id;
    entry.next_hop_rover_id = neighbor_id;
    entry.metric = 1;
    entry.destination_sequence_number = 0;
    entry.last_refreshed_ns = now_ns;
    m_routes[neighbor_id] = entry;
    return true;
}

bool RoutingTable::update_route_from_advertisement(
    uint16_t reporting_neighbor_id,
    uint16_t destination_id,
    uint8_t reported_metric,
    uint32_t dest_seq,
    uint64_t now_ns) {

    // Cannot route to 0 or self via external advertisements
    if (destination_id == 0 || destination_id == m_local_rover_id) {
        return false;
    }
    // Cannot receive advertisements from invalid reporting neighbor
    if (reporting_neighbor_id == 0 || reporting_neighbor_id == m_local_rover_id) {
        return false;
    }

    // Compute candidate metric with infinity clamping
    uint8_t candidate_metric = ROUTING_INFINITY_METRIC;
    if (reported_metric < ROUTING_INFINITY_METRIC) {
        candidate_metric = static_cast<uint8_t>(std::min<int>(ROUTING_INFINITY_METRIC, reported_metric + 1));
    }

    auto it = m_routes.find(destination_id);
    if (it == m_routes.end()) {
        // Only install new route if reachable
        if (candidate_metric >= ROUTING_INFINITY_METRIC) {
            return false;
        }

        RouteEntry entry;
        entry.destination_rover_id = destination_id;
        entry.next_hop_rover_id = reporting_neighbor_id;
        entry.metric = candidate_metric;
        entry.destination_sequence_number = dest_seq;
        entry.last_refreshed_ns = now_ns;
        m_routes[destination_id] = entry;
        return true;
    }

    RouteEntry& existing = it->second;

    // Modular sequence comparison (RFC 1982 signed arithmetic)
    const int32_t seq_diff = static_cast<int32_t>(dest_seq - existing.destination_sequence_number);

    if (seq_diff < 0) {
        // Stale sequence number originated by destination: REJECT
        return false;
    }

    if (seq_diff > 0) {
        // Fresher route epoch originated by destination: ADOPT
        existing.next_hop_rover_id = reporting_neighbor_id;
        existing.metric = candidate_metric;
        existing.destination_sequence_number = dest_seq;
        existing.last_refreshed_ns = now_ns;
        return true;
    }

    // Same sequence number: apply standard distance-vector metric rules
    if (candidate_metric < existing.metric) {
        // Better metric: adopt route
        existing.next_hop_rover_id = reporting_neighbor_id;
        existing.metric = candidate_metric;
        existing.last_refreshed_ns = now_ns;
        return true;
    }

    if (reporting_neighbor_id == existing.next_hop_rover_id) {
        // Existing next hop reports metric change or refreshes vitality
        bool changed = (existing.metric != candidate_metric);
        existing.metric = candidate_metric;
        existing.last_refreshed_ns = now_ns;
        return changed;
    }

    // Equal or worse alternate path from different neighbor: ignore
    return false;
}

std::optional<uint16_t> RoutingTable::lookup_next_hop(
    uint16_t destination_id,
    uint64_t now_ns,
    uint64_t timeout_ns) const {

    if (destination_id == 0) return std::nullopt;
    if (destination_id == m_local_rover_id) return m_local_rover_id;

    auto it = m_routes.find(destination_id);
    if (it != m_routes.end() && it->second.is_valid(now_ns, timeout_ns)) {
        return it->second.next_hop_rover_id;
    }

    return std::nullopt;
}

std::optional<RouteEntry> RoutingTable::get_route(uint16_t destination_id) const {
    auto it = m_routes.find(destination_id);
    if (it != m_routes.end()) {
        return it->second;
    }
    return std::nullopt;
}

void RoutingTable::invalidate_neighbor_routes(uint16_t failed_neighbor_id, uint64_t now_ns) {
    if (failed_neighbor_id == 0 || failed_neighbor_id == m_local_rover_id) return;

    for (auto& [dest_id, entry] : m_routes) {
        if (dest_id == m_local_rover_id) continue;
        if (entry.next_hop_rover_id == failed_neighbor_id) {
            entry.metric = ROUTING_INFINITY_METRIC;
            entry.last_refreshed_ns = now_ns;
        }
    }
}

size_t RoutingTable::prune_stale_routes(uint64_t now_ns, uint64_t timeout_ns) {
    size_t poisoned = 0;
    for (auto& [dest_id, entry] : m_routes) {
        if (dest_id == m_local_rover_id) continue;
        if (entry.metric < ROUTING_INFINITY_METRIC) {
            if (now_ns < entry.last_refreshed_ns || (now_ns - entry.last_refreshed_ns > timeout_ns)) {
                entry.metric = ROUTING_INFINITY_METRIC;
                poisoned++;
            }
        }
    }
    return poisoned;
}

std::vector<uint8_t> RoutingTable::generate_advertisement_payload(uint16_t target_neighbor_id) const {
    std::vector<uint8_t> buf;
    buf.reserve(m_routes.size() * sizeof(RouteAdvertisementRecord));

    for (const auto& [dest_id, entry] : m_routes) {
        RouteAdvertisementRecord rec;
        rec.destination_rover_id = entry.destination_rover_id;
        rec.reserved = 0;
        rec.destination_sequence_number = entry.destination_sequence_number;

        // Split Horizon with Poison Reverse
        if (entry.next_hop_rover_id == target_neighbor_id && entry.destination_rover_id != target_neighbor_id) {
            rec.metric = ROUTING_INFINITY_METRIC; // Poison reverse
        } else {
            rec.metric = entry.metric;
        }

        const uint8_t* rec_bytes = reinterpret_cast<const uint8_t*>(&rec);
        buf.insert(buf.end(), rec_bytes, rec_bytes + sizeof(RouteAdvertisementRecord));
    }

    return buf;
}

size_t RoutingTable::apply_advertisement_payload(
    uint16_t reporting_neighbor_id,
    const uint8_t* data,
    size_t length,
    uint64_t now_ns) {

    if (!data || length < sizeof(RouteAdvertisementRecord)) return 0;

    const size_t record_count = length / sizeof(RouteAdvertisementRecord);
    size_t updated = 0;

    for (size_t i = 0; i < record_count; ++i) {
        RouteAdvertisementRecord rec;
        std::memcpy(&rec, data + i * sizeof(RouteAdvertisementRecord), sizeof(RouteAdvertisementRecord));

        if (update_route_from_advertisement(
                reporting_neighbor_id,
                rec.destination_rover_id,
                rec.metric,
                rec.destination_sequence_number,
                now_ns)) {
            updated++;
        }
    }

    return updated;
}

std::vector<RouteEntry> RoutingTable::get_all_routes() const {
    std::vector<RouteEntry> result;
    result.reserve(m_routes.size());
    for (const auto& [id, entry] : m_routes) {
        result.push_back(entry);
    }
    return result;
}

void RoutingTable::clear() {
    m_routes.clear();
    RouteEntry self_route;
    self_route.destination_rover_id = m_local_rover_id;
    self_route.next_hop_rover_id = m_local_rover_id;
    self_route.metric = 0;
    self_route.destination_sequence_number = m_self_sequence_number;
    self_route.last_refreshed_ns = 0;
    m_routes[m_local_rover_id] = self_route;
}

} // namespace astromesh::net

