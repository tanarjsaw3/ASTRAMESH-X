#include "astromesh/memory/map_compactor.hpp"

#include <algorithm>
#include <cmath>
#include <map>

namespace astromesh::memory {

std::set<graph::NodeId> MapCompactor::identify_protected_nodes(
    const graph::PoseGraph& graph,
    const std::set<graph::NodeId>& external_protected) const {

    std::set<graph::NodeId> protected_nodes = external_protected;

    // Track min and max keyframe index per rover (origin and exploration frontier)
    std::map<uint16_t, uint32_t> min_indices;
    std::map<uint16_t, uint32_t> max_indices;

    for (const auto& [nid, kf] : graph.nodes()) {
        uint16_t rid = nid.rover_id;
        uint32_t idx = nid.local_index;

        if (min_indices.find(rid) == min_indices.end() || idx < min_indices[rid]) {
            min_indices[rid] = idx;
        }
        if (max_indices.find(rid) == max_indices.end() || idx > max_indices[rid]) {
            max_indices[rid] = idx;
        }
    }

    // Protect origins and exploration frontiers
    for (const auto& [rid, min_idx] : min_indices) {
        protected_nodes.insert(graph::NodeId(rid, min_idx));
    }
    for (const auto& [rid, max_idx] : max_indices) {
        protected_nodes.insert(graph::NodeId(rid, max_idx));
    }

    // Protect loop closure endpoints and non-odometry constraints
    for (const auto& [eid, edge] : graph.edges()) {
        if (edge.type != graph::EdgeType::Odometry ||
            edge.status == graph::EdgeVerificationStatus::Verified) {

            // Check if this edge is a loop closure
            if (edge.type == graph::EdgeType::IntraRoverLoopClosure ||
                edge.type == graph::EdgeType::InterRoverLoopClosure ||
                std::abs(static_cast<int64_t>(edge.to_node.local_index) -
                         static_cast<int64_t>(edge.from_node.local_index)) > 1) {
                protected_nodes.insert(edge.from_node);
                protected_nodes.insert(edge.to_node);
            }
        }
    }

    // Protect any node with degree != 2 (branch points or dead ends)
    for (const auto& [nid, kf] : graph.nodes()) {
        auto incident_eids = graph.incident_edges(nid);
        if (incident_eids.size() != 2) {
            protected_nodes.insert(nid);
        }
    }

    return protected_nodes;
}

bool MapCompactor::is_node_compactible(
    const graph::PoseGraph& graph,
    const graph::NodeId& nid,
    const std::set<graph::NodeId>& protected_nodes) const {

    if (!graph.has_node(nid)) return false;
    if (protected_nodes.find(nid) != protected_nodes.end()) return false;

    auto incident_eids = graph.incident_edges(nid);
    if (incident_eids.size() != 2) return false;

    const auto& e1 = graph.get_edge(incident_eids[0]);
    const auto& e2 = graph.get_edge(incident_eids[1]);

    // Must be sequential odometry edges
    if (e1.type != graph::EdgeType::Odometry || e2.type != graph::EdgeType::Odometry) {
        return false;
    }

    // Identify predecessor and successor
    graph::NodeId pred, succ;
    if (e1.to_node == nid && e2.from_node == nid) {
        pred = e1.from_node;
        succ = e2.to_node;
    } else if (e2.to_node == nid && e1.from_node == nid) {
        pred = e2.from_node;
        succ = e1.to_node;
    } else {
        return false; // Not a linear directed chain
    }

    if (pred == succ || pred == nid || succ == nid) return false;
    if (pred.rover_id != nid.rover_id || succ.rover_id != nid.rover_id) return false;

    return true;
}

CompactionPassSummary MapCompactor::compact_graph(
    graph::PoseGraph& graph,
    const std::set<graph::NodeId>& external_protected_nodes) {

    CompactionPassSummary summary;
    auto protected_nodes = identify_protected_nodes(graph, external_protected_nodes);

    bool modified = true;
    while (modified && (m_config.target_max_keyframes == 0 || graph.num_nodes() > m_config.target_max_keyframes)) {
        modified = false;

        // Collect all currently compactible nodes
        std::vector<graph::NodeId> candidates;
        for (const auto& [nid, kf] : graph.nodes()) {
            if (is_node_compactible(graph, nid, protected_nodes)) {
                candidates.push_back(nid);
            }
        }

        for (const auto& nid : candidates) {
            if (m_config.target_max_keyframes > 0 && graph.num_nodes() <= m_config.target_max_keyframes) {
                break;
            }

            if (!is_node_compactible(graph, nid, protected_nodes)) {
                continue;
            }

            auto incident_eids = graph.incident_edges(nid);
            if (incident_eids.size() != 2) continue;

            const auto& eA = graph.get_edge(incident_eids[0]);
            const auto& eB = graph.get_edge(incident_eids[1]);

            graph::PoseGraphEdge in_edge, out_edge;
            if (eA.to_node == nid && eB.from_node == nid) {
                in_edge = eA;
                out_edge = eB;
            } else if (eB.to_node == nid && eA.from_node == nid) {
                in_edge = eB;
                out_edge = eA;
            } else {
                continue;
            }

            graph::NodeId pred = in_edge.from_node;
            graph::NodeId succ = out_edge.to_node;

            // Compound relative transform: T_pred_succ = T_pred_nid * T_nid_succ
            geometry::SE2 T1(in_edge.relative_pose);
            geometry::SE2 T2(out_edge.relative_pose);
            geometry::SE2 T_comp = T1 * T2;

            double span_dist = std::hypot(T_comp.x(), T_comp.y());
            if (span_dist > m_config.max_compounded_distance_m) {
                continue; // Do not create excessively long single edges
            }

            // Composed information matrix: 1 / (1/w1 + 1/w2)
            double w_trans1 = in_edge.information.xx;
            double w_trans2 = out_edge.information.xx;
            double w_rot1 = in_edge.information.thetatheta;
            double w_rot2 = out_edge.information.thetatheta;

            double new_trans = (w_trans1 * w_trans2) / (w_trans1 + w_trans2 + 1e-9);
            double new_rot = (w_rot1 * w_rot2) / (w_rot1 + w_rot2 + 1e-9);
            graph::InformationMatrix2D comp_info = graph::InformationMatrix2D::diagonal(new_trans, new_rot);

            // Reassociate landmark observations from nid to pred
            if (m_config.preserve_landmarks) {
                const auto* node_ptr = graph.find_node(nid);
                auto* pred_ptr = graph.find_node_mut(pred);
                if (node_ptr && pred_ptr) {
                    for (const auto& lk : node_ptr->observed_landmarks) {
                        if (std::find(pred_ptr->observed_landmarks.begin(),
                                      pred_ptr->observed_landmarks.end(),
                                      lk) == pred_ptr->observed_landmarks.end()) {
                            pred_ptr->observed_landmarks.push_back(lk);
                            summary.observations_reassociated++;
                        }
                    }
                }
            }

            // Construct new compounded edge
            uint64_t eid = m_next_compounded_edge_id++;
            graph::PoseGraphEdge comp_edge(
                eid, pred, succ, graph::EdgeType::Odometry,
                T_comp.to_pose2d(), comp_info,
                graph::EdgeVerificationStatus::Verified);

            // Atomically remove intermediate node and insert compounded edge
            graph.remove_node(nid);
            graph.add_edge(comp_edge);

            summary.nodes_pruned++;
            summary.edges_compounded++;
            summary.pruned_nodes.push_back(nid);
            summary.new_edge_ids.push_back(eid);

            modified = true;
            // Update protected nodes set if needed
            break; // Restart candidate search to preserve graph consistency
        }
    }

    summary.remaining_nodes = graph.num_nodes();
    summary.remaining_edges = graph.num_edges();
    return summary;
}

} // namespace astromesh::memory
