#include "astromesh/graph/pose_graph.hpp"

#include <iostream>
#include <algorithm>
#include <set>

namespace astromesh::graph {

std::string NodeId::to_string() const {
    return "N" + std::to_string(rover_id) + ":" + std::to_string(local_index);
}

std::ostream& operator<<(std::ostream& os, const NodeId& id) {
    os << id.to_string();
    return os;
}

// ==============================================================================
// Node Operations
// ==============================================================================

GraphStatus PoseGraph::add_node_detailed(const KeyframeNode& node) {
    if (!node.is_valid()) {
        return GraphStatus::InvalidNode;
    }
    if (nodes_.find(node.id) != nodes_.end()) {
        return GraphStatus::DuplicateNode;
    }

    nodes_.emplace(node.id, node);
    adjacency_[node.id] = {}; // Initialize empty incident edge list
    return GraphStatus::Success;
}

bool PoseGraph::add_node(const KeyframeNode& node) {
    return add_node_detailed(node) == GraphStatus::Success;
}

bool PoseGraph::update_node_pose(const NodeId& id, const Pose2D& new_pose) {
    if (!new_pose.is_valid()) {
        return false;
    }
    auto it = nodes_.find(id);
    if (it == nodes_.end()) {
        return false;
    }
    it->second.pose = new_pose;
    return true;
}

bool PoseGraph::remove_node(const NodeId& id) {
    auto it = nodes_.find(id);
    if (it == nodes_.end()) {
        return false;
    }

    // Remove all incident edges
    auto adj_it = adjacency_.find(id);
    if (adj_it != adjacency_.end()) {
        std::vector<uint64_t> incident = adj_it->second;
        for (uint64_t eid : incident) {
            remove_edge(eid);
        }
        adjacency_.erase(adj_it);
    }

    nodes_.erase(it);
    return true;
}

bool PoseGraph::has_node(const NodeId& id) const noexcept {
    return nodes_.find(id) != nodes_.end();
}

const KeyframeNode* PoseGraph::find_node(const NodeId& id) const noexcept {
    auto it = nodes_.find(id);
    if (it != nodes_.end()) {
        return &(it->second);
    }
    return nullptr;
}

KeyframeNode* PoseGraph::find_node_mut(const NodeId& id) noexcept {
    auto it = nodes_.find(id);
    if (it != nodes_.end()) {
        return &(it->second);
    }
    return nullptr;
}

const KeyframeNode& PoseGraph::get_node(const NodeId& id) const {
    auto it = nodes_.find(id);
    if (it == nodes_.end()) {
        throw std::out_of_range("KeyframeNode not found in PoseGraph: " + id.to_string());
    }
    return it->second;
}

std::vector<uint16_t> PoseGraph::rovers() const {
    std::set<uint16_t> unique_rovers;
    for (const auto& [nid, _] : nodes_) {
        unique_rovers.insert(nid.rover_id);
    }
    return std::vector<uint16_t>(unique_rovers.begin(), unique_rovers.end());
}

std::vector<NodeId> PoseGraph::nodes_for_rover(uint16_t rover_id) const {
    std::vector<NodeId> result;
    for (const auto& [nid, _] : nodes_) {
        if (nid.rover_id == rover_id) {
            result.push_back(nid);
        }
    }
    std::sort(result.begin(), result.end());
    return result;
}

// ==============================================================================
// Edge Operations
// ==============================================================================

GraphStatus PoseGraph::add_edge_detailed(PoseGraphEdge edge) {
    // 1. Validate self-edge prohibition
    if (edge.from_node == edge.to_node) {
        return GraphStatus::SelfEdgeProhibited;
    }

    // 2. Validate intrinsic edge parameters (finite values, positive-definite information)
    if (!edge.is_valid()) {
        return GraphStatus::InvalidEdge;
    }

    // 3. Validate endpoint node existence
    if (!has_node(edge.from_node) || !has_node(edge.to_node)) {
        return GraphStatus::MissingEndpointNode;
    }

    // 4. Validate provenance semantic consistency with edge type
    bool is_same_rover = (edge.from_node.rover_id == edge.to_node.rover_id);
    if (edge.type == EdgeType::InterRoverLoopClosure && is_same_rover) {
        return GraphStatus::InvalidEdge; // Cross-rover closure cannot have identical rover IDs
    }
    if ((edge.type == EdgeType::Odometry || edge.type == EdgeType::IntraRoverLoopClosure) && !is_same_rover) {
        return GraphStatus::InvalidEdge; // Odometry or intra-rover loop closure must have identical rover IDs
    }

    // 5. Check duplicate edge ID (if caller assigned explicit ID)
    if (edge.id != 0 && has_edge(edge.id)) {
        return GraphStatus::DuplicateEdge;
    }

    // 6. Check duplicate topological constraint (same endpoints and same type)
    auto incident = incident_edges(edge.from_node);
    for (uint64_t eid : incident) {
        const auto& existing = edges_.at(eid);
        if (existing.type == edge.type &&
            ((existing.from_node == edge.from_node && existing.to_node == edge.to_node) ||
             (existing.from_node == edge.to_node && existing.to_node == edge.from_node))) {
            return GraphStatus::DuplicateEdge;
        }
    }

    // 7. Auto-assign ID if caller provided 0
    if (edge.id == 0) {
        while (edges_.find(next_auto_edge_id_) != edges_.end()) {
            ++next_auto_edge_id_;
        }
        edge.id = next_auto_edge_id_++;
    }

    // 8. Insert into container and update adjacency index
    uint64_t edge_id = edge.id;
    edges_.emplace(edge_id, edge);
    adjacency_[edge.from_node].push_back(edge_id);
    adjacency_[edge.to_node].push_back(edge_id);

    return GraphStatus::Success;
}

bool PoseGraph::add_edge(PoseGraphEdge edge) {
    return add_edge_detailed(edge) == GraphStatus::Success;
}

bool PoseGraph::remove_edge(uint64_t edge_id) {
    auto it = edges_.find(edge_id);
    if (it == edges_.end()) {
        return false;
    }

    const auto& edge = it->second;

    // Remove from adjacency index
    auto remove_from_adj = [edge_id](std::vector<uint64_t>& vec) {
        auto vit = std::remove(vec.begin(), vec.end(), edge_id);
        vec.erase(vit, vec.end());
    };

    if (adjacency_.find(edge.from_node) != adjacency_.end()) {
        remove_from_adj(adjacency_[edge.from_node]);
    }
    if (adjacency_.find(edge.to_node) != adjacency_.end()) {
        remove_from_adj(adjacency_[edge.to_node]);
    }

    edges_.erase(it);
    return true;
}

bool PoseGraph::update_edge_verification(uint64_t edge_id, EdgeVerificationStatus new_status,
                                          const std::optional<Pose2D>& verified_relative_pose) {
    auto it = edges_.find(edge_id);
    if (it == edges_.end()) {
        return false;
    }
    if (verified_relative_pose.has_value() && !verified_relative_pose->is_valid()) {
        return false;
    }
    it->second.status = new_status;
    if (verified_relative_pose.has_value()) {
        it->second.relative_pose = *verified_relative_pose;
    }
    return true;
}

bool PoseGraph::has_edge(uint64_t edge_id) const noexcept {

    return edges_.find(edge_id) != edges_.end();
}

const PoseGraphEdge* PoseGraph::find_edge(uint64_t edge_id) const noexcept {
    auto it = edges_.find(edge_id);
    if (it != edges_.end()) {
        return &(it->second);
    }
    return nullptr;
}

const PoseGraphEdge& PoseGraph::get_edge(uint64_t edge_id) const {
    auto it = edges_.find(edge_id);
    if (it == edges_.end()) {
        throw std::out_of_range("PoseGraphEdge not found in PoseGraph: " + std::to_string(edge_id));
    }
    return it->second;
}

size_t PoseGraph::num_edges_of_type(EdgeType type) const noexcept {
    size_t count = 0;
    for (const auto& [_, edge] : edges_) {
        if (edge.type == type) {
            ++count;
        }
    }
    return count;
}

std::vector<uint64_t> PoseGraph::incident_edges(const NodeId& id) const {
    auto it = adjacency_.find(id);
    if (it != adjacency_.end()) {
        return it->second;
    }
    return {};
}

std::vector<PoseGraphEdge> PoseGraph::edges_between(const NodeId& u, const NodeId& v) const {
    std::vector<PoseGraphEdge> result;
    auto u_edges = incident_edges(u);
    for (uint64_t eid : u_edges) {
        const auto& edge = edges_.at(eid);
        if ((edge.from_node == u && edge.to_node == v) ||
            (edge.from_node == v && edge.to_node == u)) {
            result.push_back(edge);
        }
    }
    return result;
}

void PoseGraph::clear() noexcept {
    nodes_.clear();
    edges_.clear();
    adjacency_.clear();
    next_auto_edge_id_ = 1;
}

} // namespace astromesh::graph
