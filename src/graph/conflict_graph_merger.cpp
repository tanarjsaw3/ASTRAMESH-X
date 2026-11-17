#include "astromesh/graph/conflict_graph_merger.hpp"

#include <cmath>
#include <iostream>

namespace astromesh::graph {

ConflictAwareGraphMerger::ConflictAwareGraphMerger() = default;

ConflictAwareGraphMerger::ConflictAwareGraphMerger(PoseGraph graph)
    : m_graph(std::move(graph)) {
    rebuild_dsu();
}

bool ConflictAwareGraphMerger::candidate_dominates(
    uint32_t cand_inliers, double cand_rmse, uint16_t cand_origin,
    uint32_t exist_inliers, double exist_rmse, uint16_t exist_origin) noexcept {

    // Rule 1: Higher inlier count strictly dominates
    if (cand_inliers > exist_inliers) return true;
    if (cand_inliers < exist_inliers) return false;

    // Rule 2: Lower RMSE strictly dominates when inliers are equal
    constexpr double EPS = 1e-6;
    if (cand_rmse < exist_rmse - EPS) return true;
    if (cand_rmse > exist_rmse + EPS) return false;

    // Rule 3: Deterministic tie-breaking by lowest origin rover ID
    return cand_origin < exist_origin;
}

static std::pair<NodeId, NodeId> canonical_pair(const NodeId& u, const NodeId& v) {
    if (u < v) {
        return {u, v};
    }
    return {v, u};
}

bool ConflictAwareGraphMerger::apply_update(const MapUpdate& update) {
    if (update.type == MapUpdateType::KeyframeAdded) {
        if (!m_graph.has_node(update.node_id)) {
            KeyframeNode node(update.node_id, Pose2D(update.transform.x(), update.transform.y(), update.transform.theta()));
            m_graph.add_node(node);
            m_dsu.make_set(update.node_id);
            m_metrics.keyframes_added++;
            return true;
        }
        return false;
    }

    if (update.type == MapUpdateType::EdgeVerified) {
        // Ensure both endpoints exist in graph and DSU
        if (!m_graph.has_node(update.node_id)) {
            m_graph.add_node(KeyframeNode(update.node_id, Pose2D(0.0, 0.0, 0.0)));
            m_dsu.make_set(update.node_id);
            m_metrics.keyframes_added++;
        }
        if (!m_graph.has_node(update.target_node_id)) {
            m_graph.add_node(KeyframeNode(update.target_node_id, Pose2D(0.0, 0.0, 0.0)));
            m_dsu.make_set(update.target_node_id);
            m_metrics.keyframes_added++;
        }

        auto pair_key = canonical_pair(update.node_id, update.target_node_id);
        auto it = m_active_pair_to_edge.find(pair_key);

        if (it != m_active_pair_to_edge.end()) {
            uint64_t exist_edge_id = it->second;
            auto prov_it = m_provenance.find(exist_edge_id);
            if (prov_it != m_provenance.end()) {
                const auto& exist_prov = prov_it->second;

                // Check conflict dominance
                bool dominates = candidate_dominates(
                    update.inlier_count, update.inlier_rmse, update.origin_rover_id,
                    exist_prov.inlier_count, exist_prov.inlier_rmse, exist_prov.origin_rover_id);

                if (!dominates) {
                    m_metrics.duplicate_edges_suppressed++;
                    return false;
                }

                // Candidate dominates: resolve conflict and replace existing edge
                m_metrics.conflicts_resolved++;
                m_graph.remove_edge(exist_edge_id);
                m_provenance.erase(prov_it);
                m_active_pair_to_edge.erase(it);
            }
        }

        // Add new winning edge
        PoseGraphEdge edge(0, update.node_id, update.target_node_id, update.edge_type,
                           Pose2D(update.transform.x(), update.transform.y(), update.transform.theta()),
                           InformationMatrix2D::diagonal(100.0, 1000.0),
                           EdgeVerificationStatus::Verified);

        m_graph.add_edge_detailed(edge);

        auto edges = m_graph.edges_between(update.node_id, update.target_node_id);
        if (!edges.empty()) {
            uint64_t assigned_id = edges.back().id;
            m_active_pair_to_edge[pair_key] = assigned_id;
            m_provenance[assigned_id] = EdgeProvenance{
                update.update_id,
                update.origin_rover_id,
                update.sequence,
                update.inlier_count,
                update.inlier_rmse,
                update.causal_context,
                true
            };
        }

        m_dsu.unite(update.node_id, update.target_node_id);
        m_metrics.edges_verified++;
        return true;
    }

    if (update.type == MapUpdateType::EdgeInvalidated) {
        auto pair_key = canonical_pair(update.node_id, update.target_node_id);
        auto it = m_active_pair_to_edge.find(pair_key);
        if (it != m_active_pair_to_edge.end()) {
            uint64_t edge_id = it->second;
            m_active_pair_to_edge.erase(it);
            m_provenance.erase(edge_id);
            m_graph.remove_edge(edge_id);

            // Invalidation splits connectivity: Rebuild DSU from ground up
            rebuild_dsu();
            m_metrics.edges_invalidated++;
            return true;
        }
        return false;
    }

    return false;
}

void ConflictAwareGraphMerger::rebuild_dsu() {
    m_dsu.clear();

    // 1. Re-register all nodes
    for (const auto& [node_id, node] : m_graph.nodes()) {
        m_dsu.make_set(node_id);
    }

    // 2. Re-unite all verified or active odometric edges
    for (const auto& [edge_id, edge] : m_graph.edges()) {
        if (edge.status == EdgeVerificationStatus::Verified || edge.type == EdgeType::Odometry) {
            m_dsu.unite(edge.from_node, edge.to_node);
        }
    }

    m_metrics.dsu_rebuilds_executed++;
}

bool ConflictAwareGraphMerger::is_reciprocal_verified(const NodeId& u, const NodeId& v) const {
    auto pair_key = canonical_pair(u, v);
    auto it = m_active_pair_to_edge.find(pair_key);
    if (it == m_active_pair_to_edge.end()) {
        return false;
    }
    const auto* edge = m_graph.find_edge(it->second);
    return edge != nullptr && edge->status == EdgeVerificationStatus::Verified;
}

std::optional<EdgeProvenance> ConflictAwareGraphMerger::get_edge_provenance(uint64_t edge_id) const {
    auto it = m_provenance.find(edge_id);
    if (it != m_provenance.end()) {
        return it->second;
    }
    return std::nullopt;
}

} // namespace astromesh::graph

