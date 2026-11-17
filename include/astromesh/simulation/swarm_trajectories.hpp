#pragma once

#include "astromesh/geometry/se2.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace astromesh::simulation {

/**
 * @brief Represents a time-indexed SE(2) robot pose and velocity.
 */
struct TimedPose2D {
    uint64_t timestamp_ns{0};
    geometry::SE2 pose{0.0, 0.0, 0.0};
    geometry::Twist2D velocity{0.0, 0.0, 0.0};

    bool is_valid() const noexcept {
        return pose.is_valid() && velocity.is_valid();
    }
};

/**
 * @brief Discrete-time ground-truth trajectory for a single rover.
 */
struct RoverTrajectory {
    uint16_t rover_id{0};
    std::vector<TimedPose2D> waypoints;

    bool empty() const noexcept { return waypoints.empty(); }
    size_t size() const noexcept { return waypoints.size(); }
};

/**
 * @brief Configuration parameters for swarm trajectory generation.
 */
struct TrajectoryConfig {
    size_t num_rovers{2};
    uint64_t duration_ns{10'000'000'000ULL}; // 10.0 seconds
    uint64_t step_dt_ns{100'000'000ULL};     // 100 ms steps
    double speed{1.0};                        // Linear speed in m/s
    double arena_extent{40.0};               // Spatial bounding limit in meters
    uint64_t seed{42};

    bool is_valid() const noexcept {
        if (num_rovers == 0 || step_dt_ns == 0 || duration_ns < step_dt_ns) {
            return false;
        }
        if (!std::isfinite(speed) || speed <= 0.0) {
            return false;
        }
        if (!std::isfinite(arena_extent) || arena_extent <= 0.0) {
            return false;
        }
        return true;
    }
};

/**
 * @brief Generator for deterministic ground-truth multi-rover trajectories.
 *
 * All trajectories enforce:
 * - Deterministic repeatability from an explicit seed.
 * - Monotonically increasing timestamps starting at t = 0.
 * - Bounded motion and strictly finite SE(2) poses.
 */
class SwarmTrajectoryGenerator {
public:
    /**
     * @brief Generates parallel search paths where rovers sweep along parallel lanes.
     */
    static std::vector<RoverTrajectory> generate_parallel_search(const TrajectoryConfig& config);

    /**
     * @brief Generates intersecting grid trajectories designed to create spatial
     *        crossing points and co-observation opportunities between rovers.
     */
    static std::vector<RoverTrajectory> generate_overlapping_grid(const TrajectoryConfig& config);

    /**
     * @brief Generates concentric or overlapping circular closed-loop trajectories.
     */
    static std::vector<RoverTrajectory> generate_concentric_loops(const TrajectoryConfig& config);
};

} // namespace astromesh::simulation
