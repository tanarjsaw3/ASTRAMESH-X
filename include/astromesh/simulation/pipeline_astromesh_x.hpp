#pragma once

#include "astromesh/comms/dtn_buffer.hpp"
#include "astromesh/comms/packet_budget.hpp"
#include "astromesh/core/iblt.hpp"
#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/disjoint_set.hpp"
#include "astromesh/graph/keyframe_pruner.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/optimization/boundary_filter.hpp"
#include "astromesh/optimization/cord_integrator.hpp"
#include "astromesh/simulation/pipeline_interface.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <vector>

namespace astromesh::simulation {

/**
 * @brief Configuration parameters for the AstroMesh-X pipeline.
 */
struct AstroMeshXConfig {
    // IBLT set reconciliation configuration
    enum class DiscoveryMechanism {
        IBLT,        // Fixed 32-cell (1024 B) IBLT difference reconciliation (default)
        RawExchange  // Direct transmission of observed LandmarkKey list (16 B per key)
    };
    DiscoveryMechanism discovery_mechanism{DiscoveryMechanism::IBLT};
    size_t iblt_cells{32};             // 32 cells * 32 B = 1024 B strict envelope
    size_t iblt_probes{3};
    uint32_t protocol_id{1};

    // Custom pipeline display name for ablation tagging
    std::string custom_pipeline_name{"AstroMesh_X"};

    // Geometric verification configuration
    verification::RansacConfig2D ransac_config{};

    // Optimization (CORD) configuration
    bool enable_cord{true};
    optimization::CordIntegratorOptions cord_options{};

    // Communication (DTN, ACKs, multi-hop) configuration
    double communication_range{25.0};  // Meters
    bool enable_dtn_retries{true};
    bool enable_explicit_acks{true};
    bool enable_multihop_routing{true};
    uint8_t default_max_hops{5};
    double packet_loss_rate{0.0};
    uint64_t default_one_way_delay_ns{0};
    uint64_t simulation_duration_ns{8'000'000'000ULL};
    comms::DtnQueueConfig dtn_queue_config{};
    comms::ByteBudgetConfig budget_config{};

    // Safe Keyframe Pruning configuration
    bool enable_keyframe_pruning{true};
    graph::PrunerConfig pruner_config{};

    // Simulation & Noise configuration
    double odometry_noise_trans{0.0};
    double odometry_noise_rot{0.0};
    uint64_t seed{42};

    bool is_valid() const noexcept {
        return iblt_cells > 0 &&
               iblt_probes > 0 &&
               ransac_config.is_valid() &&
               std::isfinite(communication_range) && communication_range > 0.0 &&
               std::isfinite(odometry_noise_trans) && odometry_noise_trans >= 0.0 &&
               std::isfinite(odometry_noise_rot) && odometry_noise_rot >= 0.0;
    }
};

/**
 * @brief Complete AstroMesh-X Swarm Pipeline Implementation.
 *
 * Implements the full AstroMesh-X decentralized architecture:
 * 1. Candidate Discovery: Invertible Bloom Lookup Table (IBLT) symmetric difference reconciliation.
 * 2. Geometric Verification: Closed-form Kabsch/Umeyama RANSAC gating on measured correspondences.
 * 3. Topological Management: PoseGraph with SubgraphPartitionManager and DisjointSet tracking.
 * 4. Trajectory Optimization: Continuous Riemannian Dynamics (CORD) on Lie group SE(2).
 * 5. Communication Layer: DTN store-carry-forward buffer with packet budget quotas, explicit ACKs,
 *    and bounded multi-hop routing.
 * 6. Memory Management: Safe KeyframePruner protecting gauge anchors, newest keyframes, articulation
 *    points, and verified loop closure endpoints.
 */
class AstroMeshXPipeline : public IRoverSwarmPipeline {
public:
    explicit AstroMeshXPipeline(const AstroMeshXConfig& config);
    ~AstroMeshXPipeline() override = default;

    std::string pipeline_name() const override {
        return custom_pipeline_name_.empty() ? "AstroMesh_X" : custom_pipeline_name_;
    }

    void reset() override;
    bool ingest_step(const SimulationStepInput& step_input) override;
    SwarmPipelineMetrics collect_metrics() const override;

    std::vector<LoopClosureCandidate> get_loop_closure_history() const override {
        return loop_closure_history_;
    }

    std::map<uint16_t, std::vector<geometry::SE2>> get_estimated_trajectories() const override;
    std::optional<geometry::SE2> get_current_pose(uint16_t rover_id) const override;

    size_t num_connected_components() const override;
    bool are_rovers_connected(uint16_t rover_a, uint16_t rover_b) const override;

    bool optimize() override;

    // Direct accessors for white-box integration verification
    const AstroMeshXConfig& config() const noexcept { return config_; }
    const core::InvertibleBloomFilter* get_rover_iblt(uint16_t rover_id) const noexcept;
    const graph::PoseGraph* get_rover_graph(uint16_t rover_id) const noexcept;
    const graph::PoseGraph& swarm_graph() const noexcept { return swarm_graph_; }
    const comms::DtnBuffer* get_rover_dtn_buffer(uint16_t rover_id) const noexcept;
    const graph::SubgraphPartitionManager* get_rover_partition_manager(uint16_t rover_id) const noexcept;

    std::vector<LoopClosureCandidate> get_accepted_closures() const;
    std::vector<LoopClosureCandidate> get_rejected_closures() const;

private:
    struct StoredObservation {
        LandmarkObservation obs;
        uint32_t keyframe_index{0};
    };

    struct RoverInternalState {
        uint16_t rover_id{0};
        core::InvertibleBloomFilter iblt;
        graph::PoseGraph pose_graph;
        graph::SubgraphPartitionManager partition_manager;
        comms::DtnBuffer dtn_buffer;
        comms::PacketBudgetManager budget_manager;
        graph::KeyframePruner pruner;
        optimization::BoundaryStateFilter filter;

        std::vector<geometry::SE2> estimated_poses;
        std::vector<core::LandmarkKey> observed_keys;
        std::map<uint32_t, StoredObservation> observations_by_landmark_id;
        size_t keyframe_count{0};

        RoverInternalState(uint16_t id, const AstroMeshXConfig& cfg);
    };

    void rebuild_global_dsu();

    void process_pairwise_communication(
        RoverInternalState& rover_a,
        RoverInternalState& rover_b,
        const ChannelLinkState* link_state,
        uint64_t timestamp_ns,
        const std::map<std::pair<uint16_t, uint16_t>, const ChannelLinkState*>& link_map,
        const std::vector<uint16_t>& active_rover_ids);

    bool attempt_iblt_reconciliation(
        RoverInternalState& rover_a,
        RoverInternalState& rover_b,
        uint64_t timestamp_ns);

    AstroMeshXConfig config_;
    std::string custom_pipeline_name_;
    std::mt19937_64 rng_;
    std::uniform_real_distribution<double> dist_unit_{0.0, 1.0};
    std::normal_distribution<double> dist_odom_trans_{0.0, 1.0};
    std::normal_distribution<double> dist_odom_rot_{0.0, 1.0};

    std::map<uint16_t, RoverInternalState> rovers_;
    graph::DisjointSet global_dsu_;
    std::vector<LoopClosureCandidate> loop_closure_history_;
    std::set<std::pair<uint32_t, uint32_t>> accepted_closure_nodes_;
    std::set<uint64_t> processed_timestamps_;

    // Combined multi-rover graph for global CORD optimization
    graph::PoseGraph swarm_graph_;

    SwarmPipelineMetrics metrics_;
};

} // namespace astromesh::simulation
