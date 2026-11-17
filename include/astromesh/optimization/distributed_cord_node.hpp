#pragma once

#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/optimization/boundary_filter.hpp"
#include "astromesh/optimization/boundary_packet.hpp"
#include "astromesh/optimization/cord_integrator.hpp"
#include "astromesh/optimization/pose_graph_objective.hpp"
#include "astromesh/optimization/simulated_mesh_network.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <iosfwd>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace astromesh::optimization {

/**
 * @brief Tracking status of an inter-rover boundary neighbor link.
 */
enum class NeighborLinkStatus {
    Fresh,          ///< Within prediction age and motion bounds; extrapolating twist.
    StaleHeld,      ///< Exceeded prediction age/motion but <= timeout; Zero-Order Hold fallback.
    Disconnected    ///< Exceeded stale timeout; link severed, edge excluded from active optimization.
};

std::string neighbor_link_status_to_string(NeighborLinkStatus status);
std::ostream& operator<<(std::ostream& os, NeighborLinkStatus status);

/**
 * @brief Lifecycle and diagnostics event types for distributed boundary nodes.
 */
enum class LifecycleEventType {
    LinkConnected,
    LinkBecameStale,
    LinkDisconnected,
    LinkReconnected,
    PacketRejectedDuplicate,
    PacketRejectedCorrupt,
    PacketRejectedUnsupported,
    PredictionGatedMotion,
    PredictionGatedAge
};

std::string lifecycle_event_type_to_string(LifecycleEventType type);

/**
 * @brief Telemetry record of a lifecycle event.
 */
struct NodeLifecycleEvent {
    uint64_t timestamp_ns{0};
    graph::NodeId neighbor_id;
    LifecycleEventType event_type{LifecycleEventType::LinkConnected};
    std::string details;
};

/**
 * @brief Tracking record for a remote boundary neighbor node.
 */
struct NeighborStateRecord {
    graph::NodeId node_id;
    uint32_t remote_rover_id{0};
    geometry::SE2 received_pose;
    geometry::Twist2D received_velocity;
    uint64_t packet_timestamp_ns{0};
    uint64_t reception_timestamp_ns{0};
    uint32_t last_sequence{0};
    bool has_received_packet{false};

    geometry::SE2 current_estimate_pose;
    geometry::Twist2D current_estimate_velocity;
    NeighborLinkStatus status{NeighborLinkStatus::Disconnected};

    double last_prediction_dt_s{0.0};
    bool is_prediction_gated{false};
};

/**
 * @brief Configuration parameters for a distributed CORD node.
 */
struct DistributedCordOptions {
    double timestep{0.01};                   ///< CORD integration step h in seconds.
    NodeInertia default_inertia{1.0, 1.0, 1.0};
    NodeDamping default_damping{2.0, 2.0, 2.0};

    bool enable_velocity_prediction{true};   ///< Toggle SE(2) body-twist extrapolation.
    uint64_t max_prediction_age_ns{500'000'000}; ///< 500 ms: maximum delay for velocity extrapolation.
    uint64_t stale_timeout_ns{2'000'000'000};    ///< 2000 ms: timeout to declare link disconnected.
    double max_prediction_translation_m{0.5};   ///< Maximum translation displacement in extrapolation (m).
    double max_prediction_rotation_rad{0.5};    ///< Maximum rotation displacement in extrapolation (rad).

    std::vector<graph::NodeId> fixed_anchors; ///< Anchor nodes held static.
    double gradient_finite_diff_step{1e-6};  ///< Step size for central difference gradient.

    BoundaryFilterOptions filter_options;    ///< Boundary-state filtering and delay mitigation options.
};

/**
 * @brief Distributed CORD optimizer agent running on a single rover.
 *
 * Owns only its local sub-pose-graph and local dynamic states.
 * Communicates with peer rovers strictly via BoundaryStatePacket exchange over SimulatedMeshNetwork.
 */
class DistributedCordNode {
public:
    DistributedCordNode(
        uint32_t rover_id,
        const graph::PoseGraph& local_graph,
        const std::unordered_map<graph::NodeId, NodeDynamicsState>& initial_local_states,
        const DistributedCordOptions& options,
        SimulatedMeshNetwork& network);

    uint32_t rover_id() const noexcept { return rover_id_; }
    const DistributedCordOptions& options() const noexcept { return options_; }

    const std::unordered_map<graph::NodeId, NodeDynamicsState>& local_states() const noexcept {
        return local_states_;
    }

    const std::unordered_map<graph::NodeId, NeighborStateRecord>& neighbor_states() const noexcept {
        return neighbor_states_;
    }

    const std::vector<graph::NodeId>& local_boundary_nodes() const noexcept {
        return local_boundary_nodes_;
    }

    const std::vector<NodeLifecycleEvent>& lifecycle_events() const noexcept {
        return lifecycle_events_;
    }

    size_t active_inter_rover_edge_count() const noexcept;
    size_t total_inter_rover_edge_count() const noexcept;

    /**
     * @brief Execute one distributed simulation step at simulation time current_time_ns.
     *
     * 1. Ingest arriving packets from network.
     * 2. Update neighbor freshness and extrapolate poses if active.
     * 3. Evaluate local Riemannian gradient for local nodes.
     * 4. Step semi-implicit CORD Lie integrator for local nodes.
     * 5. Broadcast outbound BoundaryStatePacket for boundary nodes.
     *
     * @param current_time_ns Current simulation timestamp.
     * @return true if step succeeded, false if numerical breakdown or invalid state.
     */
    bool step(uint64_t current_time_ns);

    /**
     * @brief Calculate local active objective cost C_{active}.
     */
    double compute_local_active_cost() const;

    /**
     * @brief Compute local Riemannian gradient for all local nodes under current states.
     */
    bool compute_local_gradients(
        std::unordered_map<graph::NodeId, geometry::Twist2D>& out_gradients,
        double* out_max_norm = nullptr) const;

    const BoundaryStateFilter& filter() const noexcept { return filter_; }
    BoundaryStateFilter& filter() noexcept { return filter_; }

    /**
     * @brief Manually inject a neighbor state (useful for zero-delay unit testing).
     */
    void inject_neighbor_state(
        const graph::NodeId& neighbor_id,
        uint32_t remote_rover_id,
        const geometry::SE2& pose,
        const geometry::Twist2D& velocity,
        uint64_t timestamp_ns);

private:
    void discover_boundary_topology();
    void process_inbound_packets(uint64_t current_time_ns);
    void update_neighbor_predictions(uint64_t current_time_ns);
    void transmit_boundary_states(uint64_t current_time_ns);

    double evaluate_incident_edges_cost_for_node(
        const graph::NodeId& node_id,
        const geometry::SE2& test_pose,
        const std::unordered_map<graph::NodeId, geometry::SE2>& pose_context) const;

    uint32_t rover_id_{0};
    graph::PoseGraph local_graph_;
    std::unordered_map<graph::NodeId, NodeDynamicsState> local_states_;
    DistributedCordOptions options_;
    std::reference_wrapper<SimulatedMeshNetwork> network_;
    BoundaryStateFilter filter_;

    std::vector<graph::NodeId> local_boundary_nodes_;
    std::unordered_map<graph::NodeId, uint32_t> boundary_node_to_remote_rover_;
    std::unordered_map<graph::NodeId, NeighborStateRecord> neighbor_states_;

    // Mapping from edge_id to inter-rover edge metadata
    struct InterRoverEdgeInfo {
        uint64_t edge_id{0};
        graph::NodeId local_node;
        graph::NodeId remote_node;
        uint32_t remote_rover_id{0};
    };
    std::vector<InterRoverEdgeInfo> inter_rover_edges_;

    std::unordered_map<graph::NodeId, uint32_t> local_outbound_seq_;
    std::vector<NodeLifecycleEvent> lifecycle_events_;
};

} // namespace astromesh::optimization
