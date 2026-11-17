#pragma once

#include "astromesh/core/iblt.hpp"
#include "astromesh/core/landmark_key.hpp"
#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/disjoint_set.hpp"
#include "astromesh/graph/keyframe_pruner.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/net/message_frame.hpp"
#include "astromesh/net/rover_node.hpp"
#include "astromesh/optimization/boundary_packet.hpp"
#include "astromesh/optimization/cord_integrator.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <vector>

namespace astromesh::net {

/**
 * @brief Landmark observation record carrying 2D local sensor measurement.
 */
struct LandmarkObservationRecord {
    core::LandmarkKey key;
    double local_x{0.0};
    double local_y{0.0};
    uint32_t keyframe_index{0};
    bool is_ground_truth_inlier{true};
};

/**
 * @brief Inter-rover candidate proposal carrying matched landmark coordinates.
 */
struct CandidateProposalPayload {
    uint16_t sender_rover_id{0};
    uint16_t target_rover_id{0};
    std::vector<LandmarkObservationRecord> observations;

    std::vector<uint8_t> serialize() const;
    static std::optional<CandidateProposalPayload> deserialize(const uint8_t* data, size_t size);
};

/**
 * @brief Response to candidate proposal confirming geometric verification and estimated transform.
 */
struct CandidateResponsePayload {
    uint16_t sender_rover_id{0};
    uint16_t target_rover_id{0};
    bool verified{false};
    uint32_t inlier_count{0};
    double inlier_rmse{0.0};
    geometry::SE2 relative_transform;
    uint32_t sender_keyframe{0};
    uint32_t target_keyframe{0};

    std::vector<uint8_t> serialize() const;
    static std::optional<CandidateResponsePayload> deserialize(const uint8_t* data, size_t size);
};

// IBLT Binary Serialization
std::vector<uint8_t> serialize_iblt_fingerprint(const core::InvertibleBloomFilter& iblt);
std::optional<core::InvertibleBloomFilter> deserialize_iblt_fingerprint(const uint8_t* data, size_t size);

/**
 * @brief Telemetry counters for the distributed reconciliation agent.
 */
struct ReconciliationStats {
    uint64_t sync_sent{0};
    uint64_t sync_received{0};
    uint64_t sync_decodes_succeeded{0};
    uint64_t sync_decodes_failed{0};
    uint64_t proposals_sent{0};
    uint64_t proposals_received{0};
    uint64_t loop_closures_verified{0};
    uint64_t loop_closures_rejected{0};
    uint64_t dsu_unions_performed{0};
    uint64_t boundary_states_sent{0};
    uint64_t boundary_states_received{0};
    uint64_t true_positives{0};
    uint64_t false_positives{0};
    uint64_t true_negatives{0};
    uint64_t false_negatives{0};
    uint64_t cord_optimizations_run{0};
    uint64_t keyframes_pruned{0};
};

/**
 * @brief Autonomous distributed reconciliation agent orchestrating IBLT synchronization,
 *        Kabsch RANSAC geometric verification, DSU partition merging, and boundary state exchange.
 */
class DistributedReconciliationAgent {
public:
    DistributedReconciliationAgent(
        uint16_t rover_id,
        RoverNode& net_node,
        size_t iblt_cells = 32,
        size_t iblt_probes = 3);

    void setup_callbacks();

    // Local SLAM & Map Management
    void add_local_keyframe(
        uint32_t keyframe_index,
        const geometry::SE2& pose,
        const std::vector<LandmarkObservationRecord>& observations,
        uint64_t timestamp_ns = 0);

    // Multi-keyframe trajectory extension with odometry and landmark ingestion
    void append_trajectory_keyframe(
        uint32_t keyframe_index,
        const geometry::SE2& pose,
        const std::vector<LandmarkObservationRecord>& observations,
        uint64_t timestamp_ns = 0);

    // CORD Pose Graph Optimization on local and boundary nodes
    optimization::CordIntegratorResult optimize_local_graph_cord(
        const optimization::CordIntegratorOptions& options = optimization::CordIntegratorOptions{});

    // Safe Keyframe Pruning
    graph::PruningResult prune_redundant_keyframes(
        const graph::KeyframeSelectionCriteria& criteria = graph::KeyframeSelectionCriteria{},
        const graph::PrunerConfig& config = graph::PrunerConfig{});

    // Statistical queries for loop closure verification accuracy
    double get_loop_closure_precision() const noexcept;
    double get_loop_closure_recall() const noexcept;
    double get_loop_closure_f1() const noexcept;

    // Custom RANSAC configuration
    void set_ransac_config(const verification::RansacConfig2D& config) noexcept {
        m_ransac_config = config;
    }
    const verification::RansacConfig2D& get_ransac_config() const noexcept {
        return m_ransac_config;
    }

    // Auto-optimization on receiving boundary state
    void set_auto_optimize_on_boundary(bool enable, uint32_t max_rounds = 5) noexcept {
        m_auto_optimize_on_boundary = enable;
        m_max_boundary_cord_rounds = max_rounds;
    }
    bool is_auto_optimize_on_boundary() const noexcept {
        return m_auto_optimize_on_boundary;
    }
    uint32_t get_boundary_cord_rounds_executed() const noexcept {
        return m_boundary_cord_rounds_executed;
    }

    // Landmark Observation History Query
    std::vector<LandmarkObservationRecord> get_landmark_observation_history(uint32_t landmark_id) const;

    // Protocol Initiation
    uint64_t initiate_sync_with_peer(uint16_t target_rover_id, uint64_t now_ns = 0);
    uint64_t broadcast_boundary_state(uint16_t target_rover_id, uint32_t keyframe_index, uint64_t now_ns = 0);

    // Incoming Message Handlers
    void handle_message(const MessageFrame& frame, const Endpoint& sender_ep, uint64_t now_ns = 0);

    // State Inspection
    uint16_t get_rover_id() const noexcept { return m_rover_id; }
    const graph::PoseGraph& get_pose_graph() const noexcept { return m_graph; }
    graph::PoseGraph& get_pose_graph() noexcept { return m_graph; }
    const graph::SubgraphPartitionManager& get_partition_manager() const noexcept { return m_partition_mgr; }
    graph::SubgraphPartitionManager& get_partition_manager() noexcept { return m_partition_mgr; }
    const core::InvertibleBloomFilter& get_iblt() const noexcept { return m_iblt; }
    const ReconciliationStats& get_stats() const noexcept { return m_stats; }
    size_t get_dsu_component_count() const noexcept { return m_partition_mgr.num_partitions(); }

private:
    void process_landmark_sync(const MessageFrame& frame, uint64_t now_ns);
    void process_candidate_proposal(const MessageFrame& frame, uint64_t now_ns);
    void process_candidate_response(const MessageFrame& frame, uint64_t now_ns);
    void process_boundary_state(const MessageFrame& frame, uint64_t now_ns);

    uint16_t m_rover_id;
    RoverNode& m_net_node;
    graph::PoseGraph m_graph;
    graph::SubgraphPartitionManager m_partition_mgr;
    core::InvertibleBloomFilter m_iblt;
    verification::RansacConfig2D m_ransac_config;

    std::map<uint32_t, LandmarkObservationRecord> m_local_observations; // landmark_id -> latest record
    std::map<uint32_t, std::vector<LandmarkObservationRecord>> m_landmark_history; // landmark_id -> all observations
    std::map<graph::NodeId, geometry::SE2> m_boundary_neighbor_poses;

    ReconciliationStats m_stats;
    uint32_t m_boundary_sequence{1};
    bool m_auto_optimize_on_boundary{true};
    uint32_t m_max_boundary_cord_rounds{5};
    uint32_t m_boundary_cord_rounds_executed{0};
};

/**
 * @brief Swarm-wide connectivity metrics evaluated globally across all rovers.
 */
struct SwarmConnectivityMetrics {
    size_t total_swarm_nodes{0};
    size_t total_swarm_edges{0};
    size_t total_odometry_edges{0};
    size_t verified_inter_rover_closures{0};
    size_t global_connected_components{0};
    bool is_fully_unified{false};
};

/**
 * @brief Independent evaluator aggregating verified connectivity across all rover graphs.
 */
class GlobalSwarmEvaluator {
public:
    static SwarmConnectivityMetrics evaluate(
        const std::vector<std::unique_ptr<DistributedReconciliationAgent>>& agents);

    static SwarmConnectivityMetrics evaluate_graphs(
        const std::vector<const graph::PoseGraph*>& graphs);
};

} // namespace astromesh::net
