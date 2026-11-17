#pragma once

#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/simulation/observation_model.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace astromesh::simulation {

/**
 * @brief Observation payload ingested from a single rover at a specific simulation timestep.
 */
struct RoverStepData {
    uint16_t rover_id{0};
    uint64_t timestamp_ns{0};
    geometry::SE2 odometry_step{0.0, 0.0, 0.0}; // Relative delta displacement since previous step
    std::vector<LandmarkObservation> observations;

    bool is_valid() const noexcept {
        return rover_id > 0 && odometry_step.is_valid();
    }
};

/**
 * @brief Bilateral channel state between a pair of rovers during a simulation step.
 */
struct ChannelLinkState {
    uint16_t rover_a{0};
    uint16_t rover_b{0};
    bool in_communication_range{true}; // Physical distance <= R_comm
    bool is_partitioned{false};        // Environmental / terrain obstacle partition
    double link_packet_loss_rate{0.0};  // Channel drop rate for this step [0.0, 1.0]
    uint64_t one_way_delay_ns{0};       // One-way propagation/transmission delay in nanoseconds
};

/**
 * @brief Complete simulation input for one discrete timestep across all active rovers.
 */
struct SimulationStepInput {
    uint64_t timestamp_ns{0};
    std::vector<RoverStepData> rover_steps;
    std::vector<ChannelLinkState> channel_states;

    bool is_valid() const noexcept {
        for (const auto& r : rover_steps) {
            if (!r.is_valid()) return false;
        }
        return true;
    }
};

/**
 * @brief Record of a candidate loop closure evaluated between two rovers.
 */
struct LoopClosureCandidate {
    uint16_t rover_a{0};
    uint16_t rover_b{0};
    uint64_t timestamp_ns{0};
    size_t candidate_landmarks_matched{0};
    bool attempted_verification{false};
    bool verification_passed{false};
    verification::VerificationFailureReason failure_reason{verification::VerificationFailureReason::None};
    graph::Pose2D estimated_relative_transform;
    size_t inlier_count{0};
    double inlier_ratio{0.0};
    double inlier_rmse{0.0};
};

/**
 * @brief Comprehensive snapshot of swarm pipeline telemetry and resource metrics.
 */
struct SwarmPipelineMetrics {
    std::string pipeline_name;

    // Ingestion and Processing
    size_t total_observations_ingested{0};
    size_t total_odometry_steps_processed{0};

    // Candidate Discovery & Verification
    size_t candidate_discovery_attempts{0};
    size_t candidate_pairs_proposed{0};
    size_t verification_attempts{0};
    size_t loop_closures_verified{0};
    size_t loop_closures_rejected{0};

    // Graph and Topology
    size_t dsu_components{0};
    size_t dsu_merge_events{0};
    size_t total_keyframes_retained{0};

    // Communication & Network
    size_t logical_messages_sent{0};
    size_t logical_messages_delivered{0};
    size_t logical_messages_dropped{0};
    size_t payload_bytes_transmitted{0};
    size_t protocol_bytes_transmitted{0};
    size_t framing_bytes_transmitted{0};
    size_t total_wire_bytes_transmitted{0};

    // Optimization telemetry
    size_t optimization_steps_performed{0};
    bool centralized_optimization_performed{false};
    std::string optimization_mode{"None"};

    // Synchronization telemetry
    size_t sync_attempts{0};
    size_t sync_successes{0};
    size_t sync_incomplete_decodes{0};

    // Pruning telemetry
    size_t keyframe_prunes_applied{0};
    size_t keyframe_prunes_rejected{0};

    // Network & Routing Telemetry
    size_t hop_transmissions_attempted{0};
    size_t hop_transmissions_successful{0};
    size_t retransmissions_performed{0};
    size_t unique_deliveries{0};
    size_t duplicate_deliveries{0};
    size_t hop_limit_exhausted{0};
    size_t retry_exhausted_drops{0};
    size_t in_flight_messages{0};
    size_t in_flight_at_termination{0};
    size_t ack_wire_bytes_transmitted{0};
    size_t data_wire_bytes_transmitted{0};
    double application_completion_ratio{0.0};
    double per_hop_delivery_ratio{0.0};

    // Explicit Capability Flags (distinguishing supported vs unsupported features)
    bool optimization_supported{false};         // false for AstroMesh v1; true for AstroMesh-X
    bool dtn_supported{false};                  // false for AstroMesh v1; true for AstroMesh-X
    bool keyframe_pruning_supported{false};     // false for AstroMesh v1; true for AstroMesh-X
    bool explicit_acknowledgments_supported{false};
    bool multihop_routing_supported{false};
};

/**
 * @brief Shared abstract pipeline interface implemented by both AstroMesh v1 (baseline)
 *        and AstroMesh-X (proposed system) for standardized comparative benchmarking.
 */
class IRoverSwarmPipeline {
public:
    virtual ~IRoverSwarmPipeline() = default;

    /**
     * @brief Human-readable name of the pipeline implementation.
     */
    virtual std::string pipeline_name() const = 0;

    /**
     * @brief Resets all internal states, filters, graphs, and metrics to initial baseline.
     */
    virtual void reset() = 0;

    /**
     * @brief Ingests one discrete simulation timestep containing observations, odometry,
     *        and channel link states.
     * @return true if step was ingested successfully; false on invalid input.
     */
    virtual bool ingest_step(const SimulationStepInput& step_input) = 0;

    /**
     * @brief Collects a consistent snapshot of metrics across all rovers.
     */
    virtual SwarmPipelineMetrics collect_metrics() const = 0;

    /**
     * @brief Returns chronological history of all attempted loop closures (both accepted and rejected).
     */
    virtual std::vector<LoopClosureCandidate> get_loop_closure_history() const = 0;

    /**
     * @brief Returns estimated trajectories (ordered poses) per rover ID.
     */
    virtual std::map<uint16_t, std::vector<geometry::SE2>> get_estimated_trajectories() const = 0;

    /**
     * @brief Returns the most recent estimated pose for the specified rover.
     */
    virtual std::optional<geometry::SE2> get_current_pose(uint16_t rover_id) const = 0;

    /**
     * @brief Returns the number of disconnected components in the global swarm topology.
     */
    virtual size_t num_connected_components() const = 0;

    /**
     * @brief Checks whether two rovers are in the same topological partition.
     */
    virtual bool are_rovers_connected(uint16_t rover_a, uint16_t rover_b) const = 0;

    /**
     * @brief Executes distributed graph optimization (e.g. CORD) if supported.
     * @return true if optimization was performed successfully; false if unsupported or failed.
     */
    virtual bool optimize() { return false; }
};

} // namespace astromesh::simulation
