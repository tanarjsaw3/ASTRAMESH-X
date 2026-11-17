#pragma once

#include "astromesh/core/version_vector.hpp"
#include "astromesh/graph/disjoint_set.hpp"
#include "astromesh/graph/map_update.hpp"
#include "astromesh/graph/pose_graph.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace astromesh::graph {

/**
 * @brief Provenance and quality metadata associated with an edge in the distributed graph.
 */
struct EdgeProvenance {
    uint64_t update_id{0};
    uint16_t origin_rover_id{0};
    uint64_t sequence{0};
    uint32_t inlier_count{0};
    double inlier_rmse{0.0};
    core::VersionVector causal_context;
    bool is_active{true};
};

/**
 * @brief Conflict resolution metrics tracked by the merger.
 */
struct MergerMetrics {
    uint32_t keyframes_added{0};
    uint32_t edges_verified{0};
    uint32_t edges_invalidated{0};
    uint32_t duplicate_edges_suppressed{0};
    uint32_t conflicts_resolved{0};
    uint32_t dsu_rebuilds_executed{0};
};

/**
 * @brief Conflict-Aware Distributed Graph Merger.
 * Implements deterministic tie-breaking for concurrent conflicting loop closure proposals,
 * provenance tracking, and dynamic DSU rebuilding upon edge invalidation.
 */
class ConflictAwareGraphMerger {
public:
    ConflictAwareGraphMerger();
    explicit ConflictAwareGraphMerger(PoseGraph graph);

    /**
     * @brief Applies a causally ordered MapUpdate to the local pose-graph and DSU.
     * @return true if update mutated the active graph; false if rejected, stale, or duplicate.
     */
    bool apply_update(const MapUpdate& update);

    /**
     * @brief Evaluates whether an edge candidate dominates an existing edge under deterministic tie-breaking.
     * Dominance rules:
     * 1. Higher inlier count.
     * 2. Lower inlier RMSE (if inlier counts equal).
     * 3. Lower origin rover ID (if RMSE equal).
     */
    static bool candidate_dominates(
        uint32_t cand_inliers, double cand_rmse, uint16_t cand_origin,
        uint32_t exist_inliers, double exist_rmse, uint16_t exist_origin) noexcept;

    /**
     * @brief Rebuilds the DisjointSet from all active nodes and verified/odometry edges.
     */
    void rebuild_dsu();

    /**
     * @brief Checks if a verified edge exists between node u and node v.
     */
    bool is_reciprocal_verified(const NodeId& u, const NodeId& v) const;

    const PoseGraph& graph() const noexcept { return m_graph; }
    PoseGraph& graph() noexcept { return m_graph; }

    const DisjointSet& dsu() const noexcept { return m_dsu; }
    DisjointSet& dsu() noexcept { return m_dsu; }

    const MergerMetrics& metrics() const noexcept { return m_metrics; }

    std::optional<EdgeProvenance> get_edge_provenance(uint64_t edge_id) const;

private:
    PoseGraph m_graph;
    DisjointSet m_dsu;
    MergerMetrics m_metrics;

    // Edge ID -> Provenance
    std::unordered_map<uint64_t, EdgeProvenance> m_provenance;

    // Ordered pair (min(u,v), max(u,v)) -> Edge ID for duplicate/conflict detection
    struct NodePairHash {
        size_t operator()(const std::pair<NodeId, NodeId>& p) const noexcept {
            std::hash<NodeId> h;
            return h(p.first) ^ (h(p.second) << 1);
        }
    };
    std::unordered_map<std::pair<NodeId, NodeId>, uint64_t, NodePairHash> m_active_pair_to_edge;
};

} // namespace astromesh::graph

