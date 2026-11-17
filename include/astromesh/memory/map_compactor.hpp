#ifndef ASTROMESH_MAP_COMPACTOR_HPP
#define ASTROMESH_MAP_COMPACTOR_HPP

#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"

#include <cstddef>
#include <cstdint>
#include <set>
#include <vector>

namespace astromesh::memory {

/**
 * @brief Configuration parameters for dynamic map compaction.
 */
struct CompactionConfig {
    size_t target_max_keyframes{50};            ///< Desired upper bound on keyframes
    double max_compounded_distance_m{30.0};     ///< Maximum spatial span of a single compounded edge
    bool preserve_landmarks{true};              ///< Whether to reassociate landmark references to neighbors
};

/**
 * @brief Detailed report of a compaction pass.
 */
struct CompactionPassSummary {
    size_t nodes_pruned{0};
    size_t edges_compounded{0};
    size_t observations_reassociated{0};
    size_t remaining_nodes{0};
    size_t remaining_edges{0};
    std::vector<graph::NodeId> pruned_nodes;
    std::vector<uint64_t> new_edge_ids;
};

/**
 * @brief Conservative topological map compactor for PoseGraph.
 * Compunds linear non-branching odometry keyframe chains into equivalent SE(2) relative edges.
 * Strictly preserves loop closure endpoints, exploration frontiers, gauge anchors, and graph connectivity.
 */
class MapCompactor {
public:
    explicit MapCompactor(const CompactionConfig& config = {}) noexcept
        : m_config(config) {}

    /**
     * @brief Executes a conservative compaction pass on the provided PoseGraph.
     * @param graph The pose graph to compact.
     * @param external_protected_nodes Additional node IDs that must not be removed (e.g. gauge anchors).
     * @return Summary of the compaction pass.
     */
    CompactionPassSummary compact_graph(
        graph::PoseGraph& graph,
        const std::set<graph::NodeId>& external_protected_nodes = {});

    /**
     * @brief Evaluates whether an individual node can be safely removed and compounded.
     */
    bool is_node_compactible(
        const graph::PoseGraph& graph,
        const graph::NodeId& nid,
        const std::set<graph::NodeId>& protected_nodes) const;

    const CompactionConfig& config() const noexcept { return m_config; }
    void set_config(const CompactionConfig& config) noexcept { m_config = config; }

private:
    CompactionConfig m_config;
    uint64_t m_next_compounded_edge_id{9000000ULL};

    std::set<graph::NodeId> identify_protected_nodes(
        const graph::PoseGraph& graph,
        const std::set<graph::NodeId>& external_protected) const;
};

} // namespace astromesh::memory

#endif // ASTROMESH_MAP_COMPACTOR_HPP

