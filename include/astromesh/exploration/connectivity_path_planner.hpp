#pragma once

#include "astromesh/exploration/exploration_utility.hpp"
#include "astromesh/exploration/frontier.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace astromesh::exploration {

/**
 * @brief Operational status resulting from a path planning query.
 */
enum class PlannerStatus : uint8_t {
    Success = 0,
    DegradedConnectivity = 1,
    ReturnToSwarm = 2,
    StationKeepingFallback = 3
};

std::string to_string(PlannerStatus status);

/**
 * @brief Detailed outcome of connectivity-aware path planning.
 */
struct PathPlanResult {
    PlannerStatus status{PlannerStatus::StationKeepingFallback};
    uint64_t target_frontier_id{0};
    verification::Point2D target_position{0.0, 0.0};
    double travel_distance{0.0};
    double utility_score{0.0};
    UtilityBreakdown breakdown;
    std::vector<verification::Point2D> waypoints;
    bool is_fallback{false};
    std::string reason;

    bool is_valid() const noexcept {
        return target_position.is_valid();
    }
};

/**
 * @brief Configuration parameters for connectivity-aware path planning.
 */
struct ConnectivityPlannerConfig {
    double comm_range_meters{15.0};
    double max_disconnected_travel_meters{25.0};
    double switching_hysteresis_bonus{0.15}; // Delta required to switch active target
    double waypoint_step_size_meters{2.0};
    double max_travel_meters{30.0};
};

/**
 * @brief Plans collision-free paths toward frontiers while bounding communication risk and memory growth.
 */
class ConnectivityPathPlanner {
public:
    explicit ConnectivityPathPlanner(
        uint16_t local_rover_id,
        ConnectivityPlannerConfig config = {},
        ExplorationUtilityWeights utility_weights = {});

    /**
     * @brief Computes the optimal exploration or return-to-connectivity destination for the rover.
     */
    PathPlanResult plan_destination(
        const ExplorationContext& context,
        const std::vector<Frontier>& available_frontiers,
        const std::vector<verification::Point2D>& local_landmarks,
        uint64_t current_step);

    /**
     * @brief Clears active target commitment.
     */
    void reset_commitment() noexcept;

    uint16_t rover_id() const noexcept { return m_local_rover_id; }
    uint64_t committed_frontier_id() const noexcept { return m_committed_frontier_id; }
    double committed_utility() const noexcept { return m_committed_utility; }

    const ConnectivityPlannerConfig& config() const noexcept { return m_config; }
    const ExplorationUtilityModel& utility_model() const noexcept { return m_utility_model; }

private:
    uint16_t m_local_rover_id{0};
    ConnectivityPlannerConfig m_config;
    ExplorationUtilityModel m_utility_model;

    uint64_t m_committed_frontier_id{0};
    verification::Point2D m_committed_target{0.0, 0.0};
    double m_committed_utility{0.0};

    std::vector<verification::Point2D> generate_linear_waypoints(
        const verification::Point2D& from,
        const verification::Point2D& to,
        double step_size) const;
};

} // namespace astromesh::exploration

