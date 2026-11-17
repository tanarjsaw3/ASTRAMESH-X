#pragma once

#include "astromesh/core/landmark_key.hpp"
#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace astromesh::simulation {

/**
 * @brief Configuration parameters for a synthetic 2D planar simulation world.
 *
 * Coordinate Frame Convention:
 * - World frame: Standard right-handed Cartesian coordinate system.
 *   +X points right (east), +Y points up (north).
 *   Angles theta are measured counter-clockwise from the +X axis in radians, normalized to [-pi, pi).
 */
struct WorldConfig {
    double min_x{-50.0};
    double max_x{50.0};
    double min_y{-50.0};
    double max_y{50.0};
    size_t num_landmarks{100};
    uint64_t seed{42};
    double sensor_range{10.0};             // Radial perception range in meters
    double field_of_view_rad{2.0 * M_PI};  // Angular perception cone in radians (default: 2*pi omnidirectional)

    /**
     * @brief Validates parameter bounds and finiteness.
     */
    bool is_valid() const noexcept;
};

/**
 * @brief Represents a static landmark in the 2D world.
 */
struct WorldLandmark {
    uint32_t id{0};
    verification::Point2D position;  // Coordinates in the global world frame (meters)
    core::LandmarkKey key;           // Canonical provenance & descriptor key
};

/**
 * @brief Deterministic 2D planar environment generating static landmarks and
 *        answering geometric visibility queries from rover poses.
 */
class SyntheticWorld2D {
public:
    /**
     * @brief Constructs a synthetic world with the specified configuration.
     * @throws std::invalid_argument if the configuration is invalid.
     */
    explicit SyntheticWorld2D(const WorldConfig& config);

    const WorldConfig& config() const noexcept { return config_; }
    const std::vector<WorldLandmark>& landmarks() const noexcept { return landmarks_; }
    size_t num_landmarks() const noexcept { return landmarks_.size(); }

    /**
     * @brief Queries landmarks visible from a given rover pose in the world frame.
     *
     * Visibility Criteria:
     * 1. Distance d = ||p_l - p_r||_2 <= sensor_range.
     * 2. Bearing beta = normalize_angle(atan2(dy, dx) - theta_r) satisfies |beta| <= field_of_view / 2.
     *
     * @param rover_pose Rover pose in the global world frame (x, y, theta).
     * @return List of visible landmarks sorted deterministically by landmark ID.
     */
    std::vector<WorldLandmark> query_visible_landmarks(const geometry::SE2& rover_pose) const;

    /**
     * @brief Finds a landmark by its global ID. Returns nullptr if not found.
     */
    const WorldLandmark* find_landmark(uint32_t landmark_id) const noexcept;

private:
    void generate_landmarks();

    WorldConfig config_;
    std::vector<WorldLandmark> landmarks_;
};

} // namespace astromesh::simulation
