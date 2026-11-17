#include "astromesh/graph/disjoint_set.hpp"

#include <algorithm>

namespace astromesh::graph {

// ==============================================================================
// DisjointSet Implementation
// ==============================================================================

bool DisjointSet::make_set(const NodeId& node) {
    if (!node.is_valid()) {
        return false;
    }
    if (elements_.find(node) != elements_.end()) {
        return false; // Already registered
    }

    elements_[node] = Element{node, 0, 1};
    ++num_components_;
    return true;
}

bool DisjointSet::has_node(const NodeId& node) const noexcept {
    return elements_.find(node) != elements_.end();
}

std::optional<NodeId> DisjointSet::find(const NodeId& node) {
    auto it = elements_.find(node);
    if (it == elements_.end()) {
        return std::nullopt;
    }

    // Step 1: Find the root representative
    NodeId curr = node;
    while (elements_[curr].parent != curr) {
        curr = elements_[curr].parent;
    }
    NodeId root = curr;

    // Step 2: Path compression (flatten all intermediate nodes along the path)
    curr = node;
    while (elements_[curr].parent != root) {
        NodeId next = elements_[curr].parent;
        elements_[curr].parent = root;
        curr = next;
    }

    return root;
}

std::optional<NodeId> DisjointSet::find_const(const NodeId& node) const noexcept {
    auto it = elements_.find(node);
    if (it == elements_.end()) {
        return std::nullopt;
    }

    NodeId curr = node;
    while (true) {
        auto parent_it = elements_.find(curr);
        if (parent_it == elements_.end()) {
            return std::nullopt;
        }
        if (parent_it->second.parent == curr) {
            return curr;
        }
        curr = parent_it->second.parent;
    }
}

bool DisjointSet::unite(const NodeId& u, const NodeId& v) {
    auto root_u_opt = find(u);
    auto root_v_opt = find(v);

    if (!root_u_opt || !root_v_opt) {
        return false; // Either u or v is unregistered
    }

    NodeId root_u = *root_u_opt;
    NodeId root_v = *root_v_opt;

    if (root_u == root_v) {
        return false; // Already in the same set
    }

    auto& elem_u = elements_[root_u];
    auto& elem_v = elements_[root_v];

    // Union by rank with size accumulation
    if (elem_u.rank < elem_v.rank) {
        elem_u.parent = root_v;
        elem_v.size += elem_u.size;
    } else if (elem_u.rank > elem_v.rank) {
        elem_v.parent = root_u;
        elem_u.size += elem_v.size;
    } else {
        // Equal rank: attach v under u and increment u's rank
        elem_v.parent = root_u;
        elem_u.size += elem_v.size;
        elem_u.rank += 1;
    }

    --num_components_;
    return true;
}

bool DisjointSet::are_connected(const NodeId& u, const NodeId& v) {
    auto root_u = find(u);
    auto root_v = find(v);
    if (!root_u || !root_v) {
        return false;
    }
    return *root_u == *root_v;
}

size_t DisjointSet::component_size(const NodeId& node) {
    auto root_opt = find(node);
    if (!root_opt) {
        return 0;
    }
    return elements_[*root_opt].size;
}

void DisjointSet::clear() noexcept {
    elements_.clear();
    num_components_ = 0;
}

std::vector<NodeId> DisjointSet::all_nodes() const {
    std::vector<NodeId> nodes;
    nodes.reserve(elements_.size());
    for (const auto& kv : elements_) {
        nodes.push_back(kv.first);
    }
    std::sort(nodes.begin(), nodes.end());
    return nodes;
}

// ==============================================================================
// SubgraphPartitionManager Implementation
// ==============================================================================

void SubgraphPartitionManager::rebuild(const PoseGraph& graph) {
    dsu_.clear();

    // Step 1: Register all graph nodes
    for (const auto& kv : graph.nodes()) {
        dsu_.make_set(kv.first);
    }

    // Step 2: Unite components through all policy-eligible edges
    for (const auto& kv : graph.edges()) {
        const auto& edge = kv.second;
        if (policy_.is_eligible(edge)) {
            dsu_.unite(edge.from_node, edge.to_node);
        }
    }
}

bool SubgraphPartitionManager::add_eligible_edge(const PoseGraphEdge& edge) {
    if (!policy_.is_eligible(edge)) {
        return false;
    }

    // Ensure endpoints are registered
    dsu_.make_set(edge.from_node);
    dsu_.make_set(edge.to_node);

    return dsu_.unite(edge.from_node, edge.to_node);
}

std::vector<NodeId> SubgraphPartitionManager::get_partition_members(const NodeId& node) {
    auto root_opt = dsu_.find(node);
    if (!root_opt) {
        return {};
    }

    NodeId target_root = *root_opt;
    std::vector<NodeId> members;

    for (const auto& n : dsu_.all_nodes()) {
        if (dsu_.find(n) == target_root) {
            members.push_back(n);
        }
    }

    std::sort(members.begin(), members.end());
    return members;
}

std::map<NodeId, std::vector<NodeId>> SubgraphPartitionManager::get_all_partitions() {
    std::map<NodeId, std::vector<NodeId>> partitions;

    for (const auto& n : dsu_.all_nodes()) {
        auto root_opt = dsu_.find(n);
        if (root_opt) {
            partitions[*root_opt].push_back(n);
        }
    }

    for (auto& kv : partitions) {
        std::sort(kv.second.begin(), kv.second.end());
    }

    return partitions;
}

std::vector<std::vector<NodeId>> SubgraphPartitionManager::get_partitions_list() {
    auto partitions_map = get_all_partitions();
    std::vector<std::vector<NodeId>> result;
    result.reserve(partitions_map.size());

    for (auto& kv : partitions_map) {
        result.push_back(std::move(kv.second));
    }

    // Deterministic ordering: sort component lists by their smallest NodeId
    std::sort(result.begin(), result.end(),
              [](const std::vector<NodeId>& a, const std::vector<NodeId>& b) {
                  if (a.empty()) return true;
                  if (b.empty()) return false;
                  return a.front() < b.front();
              });

    return result;
}

bool SubgraphPartitionManager::are_rovers_connected(uint16_t rover_a, uint16_t rover_b,
                                                    const PoseGraph& graph) {
    if (rover_a == 0 || rover_b == 0) {
        return false;
    }

    auto nodes_a = graph.nodes_for_rover(rover_a);
    auto nodes_b = graph.nodes_for_rover(rover_b);

    if (nodes_a.empty() || nodes_b.empty()) {
        return false;
    }

    if (rover_a == rover_b) {
        // Test if all nodes of this rover form a single connected component
        NodeId first_node = nodes_a.front();
        for (size_t i = 1; i < nodes_a.size(); ++i) {
            if (!are_connected(first_node, nodes_a[i])) {
                return false;
            }
        }
        return true;
    }

    // Cross-rover test: test if any node in A connects to any node in B
    for (const auto& na : nodes_a) {
        for (const auto& nb : nodes_b) {
            if (are_connected(na, nb)) {
                return true;
            }
        }
    }

    return false;
}

} // namespace astromesh::graph
