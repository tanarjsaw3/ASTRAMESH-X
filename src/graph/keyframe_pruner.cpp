#include "astromesh/graph/keyframe_pruner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace astromesh::graph {

const char* to_string(PruningStatus status) noexcept {
    switch (status) {
        case PruningStatus::SuccessBoundMet: return "SuccessBoundMet";
        case PruningStatus::NoPruningNeeded: return "NoPruningNeeded";
        case PruningStatus::LimitUnmetNoSafeCandidate: return "LimitUnmetNoSafeCandidate";
        case PruningStatus::InvalidGraphState: return "InvalidGraphState";
    }
    return "Unknown";
}

KeyframePruner::KeyframePruner(const PrunerConfig& config) noexcept
    : config_(config) {}

bool KeyframePruner::should_retain_keyframe(
    const Pose2D& candidate_pose,
    uint64_t timestamp_ns,
    const Pose2D& last_keyframe_pose,
    uint64_t last_timestamp_ns) const noexcept {

    if (!candidate_pose.is_valid() || !last_keyframe_pose.is_valid()) {
        return false;
    }

    const double dx = candidate_pose.x - last_keyframe_pose.x;
    const double dy = candidate_pose.y - last_keyframe_pose.y;
    const double trans_dist = std::sqrt(dx * dx + dy * dy);

    const double dtheta = std::abs(normalize_angle(candidate_pose.theta - last_keyframe_pose.theta));

    if (trans_dist >= config_.selection.min_translation_m) {
        return true;
    }

    if (dtheta >= config_.selection.min_rotation_rad) {
        return true;
    }

    if (config_.selection.min_time_delta_ns > 0) {
        const uint64_t dt = (timestamp_ns >= last_timestamp_ns) ? (timestamp_ns - last_timestamp_ns) : 0;
        if (dt >= config_.selection.min_time_delta_ns) {
            return true;
        }
    }

    return false;
}

std::unordered_set<NodeId> KeyframePruner::find_articulation_points(const PoseGraph& graph) const {
    std::unordered_set<NodeId> articulation_points;
    if (graph.num_nodes() < 3) {
        return articulation_points; // Graphs with < 3 nodes cannot have cut-vertices
    }

    // Build undirected adjacency for eligible edges under connectivity policy
    std::unordered_map<NodeId, std::vector<NodeId>> adj;
    for (const auto& [nid, _] : graph.nodes()) {
        adj[nid] = {};
    }

    for (const auto& [_, edge] : graph.edges()) {
        if (!config_.connectivity_policy.is_eligible(edge)) {
            continue;
        }
        if (graph.has_node(edge.from_node) && graph.has_node(edge.to_node)) {
            adj[edge.from_node].push_back(edge.to_node);
            adj[edge.to_node].push_back(edge.from_node);
        }
    }

    // Tarjan's cut-vertex algorithm
    std::unordered_map<NodeId, int> disc;
    std::unordered_map<NodeId, int> low;
    std::unordered_map<NodeId, NodeId> parent;
    std::unordered_set<NodeId> visited;
    int timer = 0;

    auto dfs = [&](auto& self, const NodeId& u) -> void {
        visited.insert(u);
        disc[u] = low[u] = ++timer;
        int children = 0;

        for (const auto& v : adj[u]) {
            if (visited.find(v) == visited.end()) {
                children++;
                parent[v] = u;
                self(self, v);

                low[u] = std::min(low[u], low[v]);

                // Case 1: Root of DFS tree with >= 2 children is an articulation point
                if (parent.find(u) == parent.end() && children > 1) {
                    articulation_points.insert(u);
                }

                // Case 2: Non-root node with low[v] >= disc[u] is an articulation point
                if (parent.find(u) != parent.end() && low[v] >= disc[u]) {
                    articulation_points.insert(u);
                }
            } else if (parent.find(u) == parent.end() || !(v == parent[u])) {
                low[u] = std::min(low[u], disc[v]);
            }
        }
    };

    // Ensure all connected components are traversed
    for (const auto& [nid, _] : graph.nodes()) {
        if (visited.find(nid) == visited.end()) {
            dfs(dfs, nid);
        }
    }

    return articulation_points;
}

std::unordered_set<NodeId> KeyframePruner::identify_protected_nodes(
    const PoseGraph& graph,
    const std::vector<NodeId>& custom_protected_nodes) const {

    std::unordered_set<NodeId> protected_set(custom_protected_nodes.begin(), custom_protected_nodes.end());

    // 1. Protect articulation points
    if (config_.protect_articulation_points) {
        const auto cuts = find_articulation_points(graph);
        protected_set.insert(cuts.begin(), cuts.end());
    }

    // 2. Protect newest keyframe per rover (active exploration frontier)
    if (config_.protect_newest_keyframe) {
        std::unordered_map<uint16_t, NodeId> newest_node_per_rover;
        for (const auto& [nid, _] : graph.nodes()) {
            auto it = newest_node_per_rover.find(nid.rover_id);
            if (it == newest_node_per_rover.end() || nid.local_index > it->second.local_index) {
                newest_node_per_rover[nid.rover_id] = nid;
            }
        }
        for (const auto& [_, newest_id] : newest_node_per_rover) {
            protected_set.insert(newest_id);
        }
    }

    // 3. Protect gauge anchors (e.g. NodeId(r, 0))
    if (config_.protect_gauge_anchors) {
        for (uint16_t rover_id : graph.rovers()) {
            NodeId anchor_candidate(rover_id, 0);
            if (graph.has_node(anchor_candidate)) {
                protected_set.insert(anchor_candidate);
            }
        }
    }

    // 4. Protect verified loop closure endpoints & inter-rover constraint endpoints
    for (const auto& [_, edge] : graph.edges()) {
        if (edge.status == EdgeVerificationStatus::Rejected) {
            continue;
        }

        const bool is_verified_loop = (edge.status == EdgeVerificationStatus::Verified) &&
            (edge.type == EdgeType::IntraRoverLoopClosure || edge.type == EdgeType::InterRoverLoopClosure);

        if (config_.protect_verified_loop_endpoints && is_verified_loop) {
            protected_set.insert(edge.from_node);
            protected_set.insert(edge.to_node);
        }

        if (config_.protect_inter_rover_endpoints && edge.type == EdgeType::InterRoverLoopClosure) {
            protected_set.insert(edge.from_node);
            protected_set.insert(edge.to_node);
        }
    }

    return protected_set;
}

GraphMemoryMetrics KeyframePruner::compute_memory_metrics(
    const PoseGraph& graph,
    const std::vector<NodeId>& custom_protected_nodes) const {

    GraphMemoryMetrics m;
    m.node_count = graph.num_nodes();
    m.edge_count = graph.num_edges();
    m.odometry_edge_count = graph.num_edges_of_type(EdgeType::Odometry);

    for (const auto& [_, edge] : graph.edges()) {
        if (edge.type == EdgeType::IntraRoverLoopClosure || edge.type == EdgeType::InterRoverLoopClosure) {
            if (edge.status == EdgeVerificationStatus::Verified) {
                m.verified_loop_closure_count++;
            } else if (edge.status == EdgeVerificationStatus::Unverified) {
                m.unverified_loop_closure_count++;
            }
        }
    }

    for (const auto& [_, node] : graph.nodes()) {
        m.landmark_reference_count += node.observed_landmarks.size();
    }

    m.protected_node_count = identify_protected_nodes(graph, custom_protected_nodes).size();

    // Transparent calculation of struct memory footprint
    m.estimated_struct_bytes = (m.node_count * sizeof(KeyframeNode)) +
                               (m.edge_count * sizeof(PoseGraphEdge)) +
                               (m.landmark_reference_count * sizeof(core::LandmarkKey));

    // Serialized canonical wire bytes:
    // Node: 4B rover + 4B local_index + 24B pose + 8B timestamp + 4B count + (landmarks * 24B) = 44B + landmarks*24B
    // Edge: 8B eid + 8B from + 8B to + 1B type + 1B status + 24B rel_pose + 48B info = 98B
    m.estimated_serialized_bytes = (m.node_count * 44) +
                                  (m.landmark_reference_count * 24) +
                                  (m.edge_count * 98);

    return m;
}

std::vector<NodeId> KeyframePruner::rank_pruning_candidates(
    const PoseGraph& graph,
    const std::unordered_set<NodeId>& protected_nodes) const {

    struct CandidateScore {
        NodeId id;
        double min_neighbor_distance{std::numeric_limits<double>::infinity()};

        bool operator<(const CandidateScore& other) const noexcept {
            if (std::abs(min_neighbor_distance - other.min_neighbor_distance) > 1e-6) {
                return min_neighbor_distance < other.min_neighbor_distance; // Smaller distance = more redundant
            }
            return id < other.id; // Deterministic tie-breaking
        }
    };

    std::vector<CandidateScore> scored;

    for (const auto& [nid, node] : graph.nodes()) {
        if (protected_nodes.find(nid) != protected_nodes.end()) {
            continue; // Node is protected
        }

        CandidateScore cs;
        cs.id = nid;

        // Compute distance to closest adjacent neighbor in graph
        const auto incident = graph.incident_edges(nid);
        double min_d = std::numeric_limits<double>::infinity();

        for (uint64_t eid : incident) {
            const auto* edge = graph.find_edge(eid);
            if (!edge) continue;
            const NodeId neighbor_id = (edge->from_node == nid) ? edge->to_node : edge->from_node;
            const auto* neighbor_node = graph.find_node(neighbor_id);
            if (!neighbor_node) continue;

            const double dx = node.pose.x - neighbor_node->pose.x;
            const double dy = node.pose.y - neighbor_node->pose.y;
            const double d = std::sqrt(dx * dx + dy * dy);
            min_d = std::min(min_d, d);
        }

        cs.min_neighbor_distance = min_d;
        scored.push_back(cs);
    }

    std::sort(scored.begin(), scored.end());

    std::vector<NodeId> result;
    result.reserve(scored.size());
    for (const auto& cs : scored) {
        result.push_back(cs.id);
    }
    return result;
}

PruningResult KeyframePruner::enforce_memory_bound(
    PoseGraph& graph,
    SubgraphPartitionManager* partition_manager,
    const std::vector<NodeId>& custom_protected_nodes) {

    PruningResult res;
    res.memory_before = compute_memory_metrics(graph, custom_protected_nodes);
    res.initial_nodes = graph.num_nodes();
    res.final_nodes = graph.num_nodes();

    if (graph.empty()) {
        res.status = PruningStatus::NoPruningNeeded;
        res.memory_after = res.memory_before;
        return res;
    }

    if (config_.max_retained_keyframes == 0 || graph.num_nodes() <= config_.max_retained_keyframes) {
        res.status = PruningStatus::NoPruningNeeded;
        res.memory_after = res.memory_before;
        return res;
    }

    // Iteratively prune candidates until bound is met or no safe candidate remains
    while (graph.num_nodes() > config_.max_retained_keyframes) {
        const auto protected_nodes = identify_protected_nodes(graph, custom_protected_nodes);
        res.protected_nodes = protected_nodes.size();

        const auto ranked_candidates = rank_pruning_candidates(graph, protected_nodes);

        if (ranked_candidates.empty()) {
            // No safe, un-protected candidate remains!
            res.status = PruningStatus::LimitUnmetNoSafeCandidate;
            break;
        }

        // Pick top candidate
        const NodeId victim = ranked_candidates.front();
        const size_t edges_before = graph.num_edges();

        const bool removed = graph.remove_node(victim);
        if (!removed) {
            res.status = PruningStatus::InvalidGraphState;
            break;
        }

        res.nodes_pruned++;
        res.edges_pruned += (edges_before - graph.num_edges());
        res.pruned_node_ids.push_back(victim);
    }

    res.final_nodes = graph.num_nodes();

    if (res.final_nodes <= config_.max_retained_keyframes && res.status != PruningStatus::LimitUnmetNoSafeCandidate) {
        res.status = PruningStatus::SuccessBoundMet;
    }

    // Rebuild partition manager if graph was mutated
    if (partition_manager && res.nodes_pruned > 0) {
        partition_manager->rebuild(graph);
    }

    res.memory_after = compute_memory_metrics(graph, custom_protected_nodes);
    return res;
}

} // namespace astromesh::graph
