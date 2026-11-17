#pragma once

#include "astromesh/geometry/se2.hpp"

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace astromesh::graph {

struct AlignmentEdge {
    uint64_t edge_id{0};
    uint16_t from_rover{0}; // Source rover B
    uint16_t to_rover{0};   // Target rover A
    geometry::SE2 transform_to_from; // T_AB mapping points from B into A (p_A = T_AB * p_B)
    size_t correspondence_count{0};
    size_t inlier_count{0};
    double inlier_rmse{0.0};
    double confidence{0.0};
    uint64_t epoch{0};
    bool is_active{true};
};

struct CycleAuditResult {
    bool has_inconsistent_cycles{false};
    double max_cycle_error_m{0.0};
    size_t evaluated_cycle_count{0};
    std::vector<std::vector<uint16_t>> inconsistent_cycles;
    std::vector<uint64_t> suggested_invalidations; // Edge IDs recommended for removal
};

/**
 * @brief Inter-subgraph Alignment Graph for multi-rover decentralized metric SLAM.
 * Tracks pairwise relative SE(2) transforms between rovers, checks cycle consistency,
 * rejects geometric contradictions, and extracts Maximum Confidence Spanning Trees.
 */
class AlignmentGraph {
public:
    static constexpr double DEFAULT_CYCLE_TOLERANCE_M = 0.35; // 35 cm

    AlignmentGraph() = default;

    /**
     * @brief Adds or updates a directed alignment edge between rovers.
     * @return true if added/updated; false if rejected as stale.
     */
    bool add_or_update_edge(AlignmentEdge edge);

    /**
     * @brief Invalidates an existing alignment edge.
     */
    bool invalidate_edge(uint64_t edge_id);

    bool has_edge(uint16_t from_rover, uint16_t to_rover) const noexcept;
    std::optional<AlignmentEdge> get_edge(uint16_t from_rover, uint16_t to_rover) const;

    /**
     * @brief Returns all unique rovers represented as vertices in the alignment graph.
     */
    std::vector<uint16_t> get_rovers() const;

    /**
     * @brief Evaluates cycle consistency across all simple cycles up to length max_cycle_len.
     */
    CycleAuditResult audit_cycle_consistency(double tolerance_m = DEFAULT_CYCLE_TOLERANCE_M, size_t max_cycle_len = 4) const;

    /**
     * @brief Resolves optimal path transform from root_anchor to target_rover.
     * Returns std::nullopt if rovers are in disconnected partitions.
     */
    std::optional<geometry::SE2> resolve_transform_from_anchor(uint16_t root_anchor, uint16_t target_rover) const;

    /**
     * @brief Finds connected components of rovers in the alignment graph.
     */
    std::vector<std::vector<uint16_t>> get_connected_components() const;

    size_t num_edges() const noexcept { return m_edges.size(); }
    const std::unordered_map<uint64_t, AlignmentEdge>& edges() const noexcept { return m_edges; }

private:
    uint64_t m_next_edge_id{1};
    std::unordered_map<uint64_t, AlignmentEdge> m_edges;

    // Ordered pair (from, to) -> Edge ID
    struct RoverPairHash {
        size_t operator()(const std::pair<uint16_t, uint16_t>& p) const noexcept {
            return (static_cast<size_t>(p.first) << 16) | p.second;
        }
    };
    std::unordered_map<std::pair<uint16_t, uint16_t>, uint64_t, RoverPairHash> m_pair_to_edge;

    // Adjacency index: from_rover -> list of outgoing edge IDs
    std::unordered_map<uint16_t, std::vector<uint64_t>> m_adjacency;
};

} // namespace astromesh::graph

