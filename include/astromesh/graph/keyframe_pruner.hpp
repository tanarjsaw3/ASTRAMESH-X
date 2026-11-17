#pragma once

#include "astromesh/graph/disjoint_set.hpp"
#include "astromesh/graph/pose_graph.hpp"

#include <cstddef>
#include <cstdint>
#include <ostream>
#include <string>
#include <unordered_set>
#include <vector>

namespace astromesh::graph {

/**
 * @brief Status outcome of a keyframe pruning or bounded-memory operation.
 */
enum class PruningStatus : uint8_t {
    SuccessBoundMet = 0,         ///< Graph is at or below the configured maximum keyframe bound.
    NoPruningNeeded = 1,         ///< Graph was already within limits; no mutations performed.
    LimitUnmetNoSafeCandidate = 2,///< Bound could not be met because all candidates are protected.
    InvalidGraphState = 3        ///< Graph was empty or possessed non-finite/corrupt elements.
};

const char* to_string(PruningStatus status) noexcept;

inline std::ostream& operator<<(std::ostream& os, PruningStatus s) {
    return os << to_string(s);
}

/**
 * @brief Criteria governing whether an observed pose should be instantiated as a new keyframe.
 */
struct KeyframeSelectionCriteria {
    double min_translation_m{0.50};      ///< Minimum spatial translation since last keyframe (metres).
    double min_rotation_rad{0.20};       ///< Minimum angular change since last keyframe (radians, ~11.5 deg).
    uint64_t min_time_delta_ns{0};       ///< Optional minimum elapsed time (nanoseconds; 0 = ignore time).

    bool is_valid() const noexcept {
        return min_translation_m >= 0.0 && min_rotation_rad >= 0.0;
    }
};

/**
 * @brief Configuration parameters for bounded graph memory and safe keyframe pruning.
 */
struct PrunerConfig {
    KeyframeSelectionCriteria selection; ///< Geometric/temporal keyframe selection thresholds.
    size_t max_retained_keyframes{100};  ///< Maximum keyframe capacity (0 = unlimited).
    bool protect_newest_keyframe{true};  ///< Always protect highest local_index node per rover (trajectory frontier).
    bool protect_gauge_anchors{true};    ///< Protect designated fixed coordinate gauge anchors (e.g. NodeId(r, 0)).
    bool protect_verified_loop_endpoints{true}; ///< Protect endpoints of active verified loop closures.
    bool protect_inter_rover_endpoints{true};   ///< Protect endpoints of cross-rover constraint edges.
    bool protect_articulation_points{true};     ///< Protect articulation points whose removal would split components.
    ConnectivityPolicy connectivity_policy{};   ///< Policy defining edge eligibility for topological connectivity.
};

/**
 * @brief Explicit, transparent accounting of pose-graph memory metrics.
 */
struct GraphMemoryMetrics {
    size_t node_count{0};                    ///< Number of KeyframeNode entities in graph.
    size_t edge_count{0};                    ///< Number of PoseGraphEdge entities in graph.
    size_t odometry_edge_count{0};           ///< Number of sequential odometry edges.
    size_t verified_loop_closure_count{0};   ///< Number of verified intra/inter-rover loop closures.
    size_t unverified_loop_closure_count{0}; ///< Number of unverified loop closure candidates.
    size_t landmark_reference_count{0};      ///< Total landmark observations tracked across all nodes.
    size_t estimated_struct_bytes{0};        ///< Plain C++ struct memory footprint (excluding allocator overhead).
    size_t estimated_serialized_bytes{0};    ///< Estimated payload bytes if serialized to disk/network.
    size_t protected_node_count{0};          ///< Number of nodes currently classified as protected from deletion.
};

/**
 * @brief Detailed result of a pruning evaluation or execution pass.
 */
struct PruningResult {
    PruningStatus status{PruningStatus::NoPruningNeeded};
    size_t initial_nodes{0};
    size_t final_nodes{0};
    size_t nodes_pruned{0};
    size_t edges_pruned{0};
    size_t protected_nodes{0};
    std::vector<NodeId> pruned_node_ids;
    GraphMemoryMetrics memory_before;
    GraphMemoryMetrics memory_after;
};

/**
 * @brief Safe, conservative keyframe selection and pose-graph pruning subsystem.
 *
 * Core Principles:
 * 1. Topological Invariant Protection: Uses Tarjan's DFS to identify articulation points
 *    in the active connectivity graph. Articulation nodes are strictly protected from removal.
 * 2. Constraint Safety: Endpoints of verified loop closures and cross-rover boundaries are protected.
 * 3. Frontier Safety: The latest keyframe of each rover is protected to prevent truncating exploration.
 * 4. Honest Accounting: Accurately differentiates struct bytes from serialized wire bytes without guessing.
 * 5. Safe Fallback: If the graph bound cannot be satisfied without violating protection rules,
 *    the operation halts atomically and returns LimitUnmetNoSafeCandidate without mutating the graph.
 */
class KeyframePruner {
public:
    explicit KeyframePruner(const PrunerConfig& config = {}) noexcept;

    const PrunerConfig& config() const noexcept { return config_; }
    void set_config(const PrunerConfig& config) noexcept { config_ = config; }

    /**
     * @brief Evaluates whether a new candidate pose warrants creating a new keyframe.
     * @param candidate_pose The proposed rover pose.
     * @param timestamp_ns Timestamp of proposed observation.
     * @param last_keyframe_pose The pose of the previous retained keyframe.
     * @param last_timestamp_ns Timestamp of previous retained keyframe.
     * @return true if spatial or temporal threshold is crossed; false otherwise or if pose is invalid.
     */
    bool should_retain_keyframe(
        const Pose2D& candidate_pose,
        uint64_t timestamp_ns,
        const Pose2D& last_keyframe_pose,
        uint64_t last_timestamp_ns) const noexcept;

    /**
     * @brief Computes all articulation points in the graph under the active connectivity policy.
     *        Uses Tarjan's bridge/cut-vertex DFS algorithm.
     */
    std::unordered_set<NodeId> find_articulation_points(const PoseGraph& graph) const;

    /**
     * @brief Enumerates all nodes in the graph that are currently protected from removal.
     */
    std::unordered_set<NodeId> identify_protected_nodes(
        const PoseGraph& graph,
        const std::vector<NodeId>& custom_protected_nodes = {}) const;

    /**
     * @brief Computes explicit graph memory and size metrics.
     */
    GraphMemoryMetrics compute_memory_metrics(
        const PoseGraph& graph,
        const std::vector<NodeId>& custom_protected_nodes = {}) const;

    /**
     * @brief Evaluates pruning candidates and ranks removable nodes deterministically.
     *        Removable nodes are sorted primarily by spatial redundancy (proximity to neighbors)
     *        with deterministic NodeId tie-breaking.
     */
    std::vector<NodeId> rank_pruning_candidates(
        const PoseGraph& graph,
        const std::unordered_set<NodeId>& protected_nodes) const;

    /**
     * @brief Executes bounded-memory pruning on the provided pose-graph.
     *        If partition_manager is provided, it is rebuilt atomically after valid mutations.
     * @param graph The pose graph to prune.
     * @param partition_manager Optional topological partition manager to synchronize.
     * @param custom_protected_nodes Optional explicit caller-specified protected nodes.
     * @return Detailed PruningResult. If no safe candidate exists, returns LimitUnmetNoSafeCandidate.
     */
    PruningResult enforce_memory_bound(
        PoseGraph& graph,
        SubgraphPartitionManager* partition_manager = nullptr,
        const std::vector<NodeId>& custom_protected_nodes = {});

private:
    PrunerConfig config_;
};

} // namespace astromesh::graph
