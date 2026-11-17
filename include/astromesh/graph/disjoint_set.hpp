#pragma once

#include "astromesh/graph/pose_graph.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <map>
#include <optional>
#include <algorithm>

namespace astromesh::graph {

/**
 * @brief Disjoint Set Union (DSU) data structure with path compression and union by rank.
 *
 * Provides near-constant amortized time complexity O(alpha(n)) per operation, where alpha
 * is the extremely slow-growing inverse Ackermann function (alpha(n) < 5 for all practical n).
 *
 * Key characteristics:
 * - Operates directly on the project's stable NodeId identifiers.
 * - Supports safe idempotent make_set() and safe handling of unregistered nodes.
 * - Tracks total disjoint component count and per-component member counts.
 * - Strictly append/merge-only: does not support arbitrary edge deletions directly.
 *   Subgraphs requiring component splits must use rebuild() via SubgraphPartitionManager.
 */
class DisjointSet {
public:
    DisjointSet() = default;
    ~DisjointSet() = default;

    /**
     * @brief Registers a node as its own independent set of size 1.
     * @param node The node identifier to register.
     * @return true if newly registered; false if invalid or already registered.
     */
    bool make_set(const NodeId& node);

    /**
     * @brief Checks whether a node is currently registered in the disjoint set.
     */
    bool has_node(const NodeId& node) const noexcept;

    /**
     * @brief Finds the representative root of the set containing the given node,
     *        applying two-pass path compression.
     * @param node The node to look up.
     * @return The representative root NodeId, or std::nullopt if the node is not registered.
     */
    std::optional<NodeId> find(const NodeId& node);

    /**
     * @brief Non-mutating find query without path compression.
     * @param node The node to look up.
     * @return The representative root NodeId, or std::nullopt if the node is not registered.
     */
    std::optional<NodeId> find_const(const NodeId& node) const noexcept;

    /**
     * @brief Unites the sets containing nodes u and v using union by rank.
     * @param u First node identifier.
     * @param v Second node identifier.
     * @return true if two distinct sets were merged; false if they were already in the same set
     *         or if either node is unregistered.
     */
    bool unite(const NodeId& u, const NodeId& v);

    /**
     * @brief Tests whether two nodes belong to the same connected component.
     */
    bool are_connected(const NodeId& u, const NodeId& v);

    /**
     * @brief Returns the total number of nodes in the component containing the given node.
     * @return Component size, or 0 if node is not registered.
     */
    size_t component_size(const NodeId& node);

    /**
     * @brief Returns the total number of disjoint connected components.
     */
    size_t num_components() const noexcept { return num_components_; }

    /**
     * @brief Returns the total number of registered nodes.
     */
    size_t num_nodes() const noexcept { return elements_.size(); }

    /**
     * @brief Clears all registered elements and resets component count to zero.
     */
    void clear() noexcept;

    /**
     * @brief Returns all registered nodes in deterministic sorted order.
     */
    std::vector<NodeId> all_nodes() const;

private:
    struct Element {
        NodeId parent;
        size_t rank{0};
        size_t size{1};
    };

    std::unordered_map<NodeId, Element> elements_;
    size_t num_components_{0};
};

/**
 * @brief Policy governing which pose-graph edges are eligible to merge components in DSU.
 */
struct ConnectivityPolicy {
    /**
     * @brief Whether Odometry edges require explicit EdgeVerificationStatus::Verified.
     *        If false (default), Odometry edges are considered physical kinematic links
     *        and merge components unless explicitly Rejected.
     *        If true, Odometry edges must also be marked Verified.
     */
    bool require_verified_odometry{false};

    /**
     * @brief Whether IntraRoverLoopClosure edges require EdgeVerificationStatus::Verified.
     *        Default true (unverified candidate closures never merge components).
     */
    bool require_verified_intra_rover_loop{true};

    /**
     * @brief Whether InterRoverLoopClosure edges require EdgeVerificationStatus::Verified.
     *        Default true (strict gating against premature inter-rover merging).
     */
    bool require_verified_inter_rover_loop{true};

    /**
     * @brief Evaluates whether an edge is eligible to connect components.
     */
    bool is_eligible(const PoseGraphEdge& edge) const noexcept {
        if (edge.status == EdgeVerificationStatus::Rejected) {
            return false;
        }
        switch (edge.type) {
            case EdgeType::Odometry:
                return require_verified_odometry
                    ? (edge.status == EdgeVerificationStatus::Verified)
                    : true;
            case EdgeType::IntraRoverLoopClosure:
                return require_verified_intra_rover_loop
                    ? (edge.status == EdgeVerificationStatus::Verified)
                    : true;
            case EdgeType::InterRoverLoopClosure:
                return require_verified_inter_rover_loop
                    ? (edge.status == EdgeVerificationStatus::Verified)
                    : true;
        }
        return false;
    }
};

/**
 * @brief Manages topological subgraphs and connected partitions for a PoseGraph.
 *
 * Characteristics:
 * - Maintains an internal DisjointSet representing graph connectivity under ConnectivityPolicy.
 * - Ensures candidate loop closures (IBLT/CBF proposals) remain unmerged until verified.
 * - Handles edge/node deletions safely via an explicit rebuild() strategy, avoiding
 *   the known theoretical limitation where DSU cannot split components without rebuild.
 * - Provides deterministic, sorted member queries for reproducible multi-rover logging.
 */
class SubgraphPartitionManager {
public:
    explicit SubgraphPartitionManager(ConnectivityPolicy policy = ConnectivityPolicy{})
        : policy_(policy) {}

    explicit SubgraphPartitionManager(const PoseGraph& graph,
                                     ConnectivityPolicy policy = ConnectivityPolicy{})
        : policy_(policy) {
        rebuild(graph);
    }

    /**
     * @brief Rebuilds all disjoint set components from scratch using eligible edges in the given graph.
     *
     * Complexity: O(V + E * alpha(V)).
     * Safely handles edge/node deletions, edge status rejections, and isolated nodes.
     *
     * @param graph The source pose-graph.
     */
    void rebuild(const PoseGraph& graph);

    /**
     * @brief Incrementally unites components using an added or updated edge if it satisfies policy.
     * @param edge The edge to process.
     * @return true if the edge was eligible and merged two previously disjoint components.
     */
    bool add_eligible_edge(const PoseGraphEdge& edge);

    /**
     * @brief Retrieves the canonical root representative of the partition containing node.
     */
    std::optional<NodeId> find_partition(const NodeId& node) {
        return dsu_.find(node);
    }

    /**
     * @brief Tests whether two nodes are in the same connected partition.
     */
    bool are_connected(const NodeId& u, const NodeId& v) {
        return dsu_.are_connected(u, v);
    }

    /**
     * @brief Returns the size of the partition containing node.
     */
    size_t partition_size(const NodeId& node) {
        return dsu_.component_size(node);
    }

    /**
     * @brief Returns the total number of connected partitions.
     */
    size_t num_partitions() const noexcept {
        return dsu_.num_components();
    }

    /**
     * @brief Returns the total number of registered nodes in the partition manager.
     */
    size_t num_nodes() const noexcept {
        return dsu_.num_nodes();
    }

    /**
     * @brief Returns all members belonging to the same partition as node, sorted ascending.
     * @return Sorted vector of NodeIds, or empty vector if node is not found.
     */
    std::vector<NodeId> get_partition_members(const NodeId& node);

    /**
     * @brief Returns all partitions mapped by canonical representative root NodeId.
     *        Member vectors are sorted in deterministic ascending order.
     */
    std::map<NodeId, std::vector<NodeId>> get_all_partitions();

    /**
     * @brief Returns all partitions as a list of sorted member lists, ordered deterministically
     *        by each partition's smallest NodeId.
     */
    std::vector<std::vector<NodeId>> get_partitions_list();

    /**
     * @brief Tests whether any node belonging to rover_a is connected to rover_b.
     */
    bool are_rovers_connected(uint16_t rover_a, uint16_t rover_b, const PoseGraph& graph);

    /**
     * @brief Resets the partition manager to empty state.
     */
    void clear() noexcept {
        dsu_.clear();
    }

    const ConnectivityPolicy& policy() const noexcept { return policy_; }
    void set_policy(const ConnectivityPolicy& policy) noexcept { policy_ = policy; }

    const DisjointSet& dsu() const noexcept { return dsu_; }

private:
    DisjointSet dsu_;
    ConnectivityPolicy policy_;
};

} // namespace astromesh::graph
