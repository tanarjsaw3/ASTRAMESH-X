#pragma once

#include "astromesh/core/counting_bloom_filter.hpp"
#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/disjoint_set.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/simulation/pipeline_interface.hpp"
#include "astromesh/simulation/voronoi_assignment.hpp"
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
 * @brief Configuration parameters for the AstroMesh v1 reference baseline pipeline.
 */
struct BaselineV1Config {
    size_t cbf_expected_items{200};
    double cbf_target_fpr{0.05};
    verification::RansacConfig2D ransac_config{};
    double communication_range{25.0}; // Meters
    uint64_t default_one_way_delay_ns{0};
    uint64_t simulation_duration_ns{8'000'000'000ULL};
    double odometry_noise_trans{0.0}; // Linear odometry standard deviation in meters
    double odometry_noise_rot{0.0};   // Heading odometry standard deviation in radians
    uint64_t seed{42};
    std::optional<NearestSiteVoronoiAssignment> voronoi_assignment{std::nullopt};

    bool is_valid() const noexcept {
        return cbf_expected_items > 0 &&
               cbf_target_fpr > 0.0 && cbf_target_fpr < 1.0 &&
               ransac_config.is_valid() &&
               std::isfinite(communication_range) && communication_range > 0.0 &&
               std::isfinite(odometry_noise_trans) && odometry_noise_trans >= 0.0 &&
               std::isfinite(odometry_noise_rot) && odometry_noise_rot >= 0.0;
    }
};

/**
 * @brief AstroMesh v1 Baseline Pipeline.
 *
 * Implements the documented reference baseline architecture:
 * 1. Candidate Discovery: Counting Bloom Filter (CBF) per rover with saturated 8-bit counters.
 * 2. Geometric Verification: Closed-form Kabsch/Umeyama RANSAC gating on measured correspondences.
 * 3. Topological Merging: Direct Disjoint Set Union (DSU) union on verified loop closures.
 * 4. Trajectory Estimation: Open-loop dead-reckoning integration (no CORD or centralized optimization).
 * 5. Communication Layer: One-hop best-effort message exchange (no DTN buffer, no retries, no ACKs).
 * 6. Spatial Assignment: Nearest-site Voronoi spatial assignment with deterministic tie-breaking.
 */
class BaselineAstroMeshV1 : public IRoverSwarmPipeline {
public:
    explicit BaselineAstroMeshV1(const BaselineV1Config& config);
    ~BaselineAstroMeshV1() override = default;

    std::string pipeline_name() const override { return "AstroMesh_v1_Baseline"; }

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
    bool optimize() override { return false; }

    const BaselineV1Config& config() const noexcept { return config_; }

    /**
     * @brief Access the local Counting Bloom Filter for a specific rover.
     */
    const core::CountingBloomFilter* get_rover_cbf(uint16_t rover_id) const noexcept;

    /**
     * @brief Access the nearest-site Voronoi spatial assigner if configured.
     */
    const NearestSiteVoronoiAssignment* voronoi_assignment() const noexcept {
        return config_.voronoi_assignment.has_value() ? &config_.voronoi_assignment.value() : nullptr;
    }

private:
    struct RoverInternalState {
        uint16_t rover_id{0};
        core::CountingBloomFilter cbf;
        std::vector<core::LandmarkKey> observed_keys; // Deterministic insertion history
        std::vector<geometry::SE2> estimated_poses;
        std::map<uint32_t, LandmarkObservation> observations_by_landmark_id;
        size_t keyframe_count{0};

        RoverInternalState(uint16_t id, size_t items, double fpr)
            : rover_id(id), cbf(items, fpr) {}
    };

    void process_pairwise_communication(
        RoverInternalState& rover_a,
        RoverInternalState& rover_b,
        const ChannelLinkState* link_state,
        uint64_t timestamp_ns);

    BaselineV1Config config_;
    std::map<uint16_t, RoverInternalState> rovers_;
    graph::DisjointSet dsu_;
    std::vector<LoopClosureCandidate> loop_closure_history_;
    std::set<std::pair<uint32_t, uint32_t>> accepted_closure_nodes_;
    std::set<uint64_t> processed_timestamps_;

    SwarmPipelineMetrics metrics_;

    mutable std::mt19937_64 rng_;
    mutable std::uniform_real_distribution<double> dist_unit_{0.0, 1.0};
    mutable std::normal_distribution<double> dist_odom_trans_{0.0, 1.0};
    mutable std::normal_distribution<double> dist_odom_rot_{0.0, 1.0};
};

} // namespace astromesh::simulation
