#pragma once

#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace astromesh::exploration {

/**
 * @brief Lifecycle status of a topological exploration frontier.
 */
enum class FrontierStatus : uint8_t {
    Candidate = 0,
    Active = 1,
    Claimed = 2,
    Completed = 3,
    Expired = 4,
    Unreachable = 5
};

std::string to_string(FrontierStatus status);

/**
 * @brief Represents an exploratory candidate destination on the topological boundary of observed space.
 */
struct Frontier {
    uint64_t frontier_id{0};
    verification::Point2D position{0.0, 0.0};
    uint16_t origin_rover_id{0};
    uint64_t detected_step{0};
    uint64_t ttl_steps{60};
    double expected_information_gain{0.0};
    double expected_dispersion{0.0};
    double spatial_coverage_radius{5.0};
    FrontierStatus status{FrontierStatus::Candidate};
    uint16_t claimed_by_rover{0};
    uint64_t claim_lease_expiration{0};
    double utility_score{0.0};

    bool is_valid() const noexcept {
        return frontier_id > 0 && position.is_valid() && origin_rover_id > 0;
    }

    bool is_available(uint64_t current_step) const noexcept {
        if (status == FrontierStatus::Active) return true;
        if (status == FrontierStatus::Claimed && current_step >= claim_lease_expiration) return true;
        return false;
    }
};

/**
 * @brief Configuration parameters for topological frontier detection.
 */
struct FrontierDetectorConfig {
    double min_x{-50.0};
    double max_x{50.0};
    double min_y{-50.0};
    double max_y{50.0};
    double sensor_range{10.0};
    double frontier_cluster_radius{3.0};
    double projection_distance{8.0};
    uint32_t num_angular_sectors{8};
    uint64_t default_ttl_steps{60};
    double completion_radius{3.0};
};

/**
 * @brief Detects, filters, deduplicates, and manages the lifecycle of topological frontiers.
 *
 * In AstroMesh-X's landmark graph representation, frontiers are detected from peripheral keyframes
 * and landmark clusters without requiring dense 2D occupancy grid rasters.
 */
class TopologicalFrontierDetector {
public:
    explicit TopologicalFrontierDetector(FrontierDetectorConfig config = {});

    /**
     * @brief Generates new candidate frontiers by radially projecting from peripheral nodes into unobserved angular sectors.
     */
    std::vector<Frontier> detect_frontiers(
        const graph::PoseGraph& graph,
        const std::vector<verification::Point2D>& landmarks,
        uint16_t rover_id,
        uint64_t current_step);

    /**
     * @brief Dynamically updates active frontiers against new keyframes and landmarks, marking covered frontiers as Completed.
     */
    void update_frontiers(
        const graph::PoseGraph& graph,
        const std::vector<verification::Point2D>& new_landmarks,
        uint64_t current_step);

    /**
     * @brief Marks a specific frontier as Completed upon rover arrival or visual coverage.
     */
    bool mark_completed(uint64_t frontier_id);

    /**
     * @brief Prunes expired frontiers whose TTL has elapsed.
     */
    size_t prune_expired(uint64_t current_step);

    /**
     * @brief Claims a frontier for a specific rover with lease TTL.
     */
    bool claim_frontier(uint64_t frontier_id, uint16_t rover_id, uint64_t lease_duration, uint64_t current_step);

    /**
     * @brief Releases a claim, returning frontier to Active status.
     */
    bool release_claim(uint64_t frontier_id, uint16_t rover_id);

    const std::vector<Frontier>& active_frontiers() const noexcept { return m_frontiers; }
    std::vector<Frontier>& active_frontiers_mut() noexcept { return m_frontiers; }

    const FrontierDetectorConfig& config() const noexcept { return m_config; }

    const Frontier* find_frontier(uint64_t frontier_id) const noexcept;
    Frontier* find_frontier_mut(uint64_t frontier_id) noexcept;

    void clear() noexcept;

private:
    FrontierDetectorConfig m_config;
    std::vector<Frontier> m_frontiers;
    uint64_t m_next_seq{1};

    bool is_within_bounds(const verification::Point2D& pt) const noexcept;
    bool is_covered_by_existing_data(
        const verification::Point2D& pt,
        const graph::PoseGraph& graph,
        const std::vector<verification::Point2D>& landmarks) const noexcept;

    uint64_t generate_frontier_id(uint16_t rover_id, uint64_t seq, const verification::Point2D& pt) const noexcept;
};

} // namespace astromesh::exploration

