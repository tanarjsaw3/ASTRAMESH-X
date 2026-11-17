#include "astromesh/simulation/swarm_trajectories.hpp"

#include <cmath>
#include <random>
#include <stdexcept>

namespace astromesh::simulation {

std::vector<RoverTrajectory> SwarmTrajectoryGenerator::generate_parallel_search(const TrajectoryConfig& config) {
    if (!config.is_valid()) {
        throw std::invalid_argument("SwarmTrajectoryGenerator: invalid TrajectoryConfig specified.");
    }

    std::vector<RoverTrajectory> trajectories;
    trajectories.reserve(config.num_rovers);

    const size_t num_steps = static_cast<size_t>(config.duration_ns / config.step_dt_ns) + 1;
    const double dt = static_cast<double>(config.step_dt_ns) * 1e-9;
    const double lane_spacing = (2.0 * config.arena_extent) / static_cast<double>(config.num_rovers + 1);

    for (size_t r = 0; r < config.num_rovers; ++r) {
        RoverTrajectory traj;
        traj.rover_id = static_cast<uint16_t>(r + 1);
        traj.waypoints.reserve(num_steps);

        const double y_lane = -config.arena_extent + static_cast<double>(r + 1) * lane_spacing;
        double current_x = -config.arena_extent * 0.8;
        double current_y = y_lane;
        double current_theta = 0.0; // Heading along +X

        for (size_t s = 0; s < num_steps; ++s) {
            TimedPose2D wp;
            wp.timestamp_ns = s * config.step_dt_ns;
            wp.pose = geometry::SE2(current_x, current_y, current_theta);
            wp.velocity = geometry::Twist2D(config.speed, 0.0, 0.0);

            traj.waypoints.push_back(wp);

            current_x += config.speed * dt;
        }

        trajectories.push_back(traj);
    }

    return trajectories;
}

std::vector<RoverTrajectory> SwarmTrajectoryGenerator::generate_overlapping_grid(const TrajectoryConfig& config) {
    if (!config.is_valid()) {
        throw std::invalid_argument("SwarmTrajectoryGenerator: invalid TrajectoryConfig specified.");
    }

    std::vector<RoverTrajectory> trajectories;
    trajectories.reserve(config.num_rovers);

    const size_t num_steps = static_cast<size_t>(config.duration_ns / config.step_dt_ns) + 1;
    const double dt = static_cast<double>(config.step_dt_ns) * 1e-9;
    const double half_extent = config.arena_extent * 0.7;

    for (size_t r = 0; r < config.num_rovers; ++r) {
        RoverTrajectory traj;
        traj.rover_id = static_cast<uint16_t>(r + 1);
        traj.waypoints.reserve(num_steps);

        // Assign alternating horizontal and vertical tracks that cross near the origin
        const bool is_horizontal = (r % 2 == 0);
        const double offset = (static_cast<double>(r / 2) - static_cast<double>(config.num_rovers / 4)) * 8.0;

        double x = is_horizontal ? -half_extent : offset;
        double y = is_horizontal ? offset : -half_extent;
        double theta = is_horizontal ? 0.0 : (M_PI * 0.5);
        const double vx = is_horizontal ? config.speed : 0.0;
        const double vy = is_horizontal ? 0.0 : config.speed;

        for (size_t s = 0; s < num_steps; ++s) {
            TimedPose2D wp;
            wp.timestamp_ns = s * config.step_dt_ns;
            wp.pose = geometry::SE2(x, y, theta);
            wp.velocity = geometry::Twist2D(config.speed, 0.0, 0.0); // Body forward speed

            traj.waypoints.push_back(wp);

            x += vx * dt;
            y += vy * dt;
        }

        trajectories.push_back(traj);
    }

    return trajectories;
}

std::vector<RoverTrajectory> SwarmTrajectoryGenerator::generate_concentric_loops(const TrajectoryConfig& config) {
    if (!config.is_valid()) {
        throw std::invalid_argument("SwarmTrajectoryGenerator: invalid TrajectoryConfig specified.");
    }

    std::vector<RoverTrajectory> trajectories;
    trajectories.reserve(config.num_rovers);

    const size_t num_steps = static_cast<size_t>(config.duration_ns / config.step_dt_ns) + 1;
    const double dt = static_cast<double>(config.step_dt_ns) * 1e-9;
    const double base_radius = std::max(5.0, config.arena_extent * 0.3);

    for (size_t r = 0; r < config.num_rovers; ++r) {
        RoverTrajectory traj;
        traj.rover_id = static_cast<uint16_t>(r + 1);
        traj.waypoints.reserve(num_steps);

        const double radius = base_radius + static_cast<double>(r) * 3.0; // Overlapping radii
        const double angular_speed = config.speed / radius;
        // Stagger initial phases deterministically
        const double initial_phase = (2.0 * M_PI * static_cast<double>(r)) / static_cast<double>(config.num_rovers);

        for (size_t s = 0; s < num_steps; ++s) {
            const double t = static_cast<double>(s) * dt;
            const double phi = initial_phase + angular_speed * t;

            const double x = radius * std::cos(phi);
            const double y = radius * std::sin(phi);
            const double theta = graph::normalize_angle(phi + (M_PI * 0.5)); // Tangent to circle

            TimedPose2D wp;
            wp.timestamp_ns = s * config.step_dt_ns;
            wp.pose = geometry::SE2(x, y, theta);
            wp.velocity = geometry::Twist2D(config.speed, 0.0, angular_speed);

            traj.waypoints.push_back(wp);
        }

        trajectories.push_back(traj);
    }

    return trajectories;
}

} // namespace astromesh::simulation
