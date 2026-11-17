#pragma once

#include "astromesh/simulation/synthetic_world.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

namespace astromesh::simulation {

/**
 * @brief Configuration for range-bearing sensor noise.
 */
struct ObservationNoiseConfig {
    double range_std_dev{0.0};   // Range standard deviation in meters (0 = exact ground truth)
    double bearing_std_dev{0.0}; // Bearing standard deviation in radians (0 = exact ground truth)
    uint64_t seed{42};           // Deterministic PRNG seed

    bool is_valid() const noexcept {
        return std::isfinite(range_std_dev) && range_std_dev >= 0.0 &&
               std::isfinite(bearing_std_dev) && bearing_std_dev >= 0.0;
    }
};

/**
 * @brief Paired ground-truth and measured observation of a static landmark.
 *
 * Coordinate Convention:
 * - World frame: Global Cartesian coordinates (x, y) in meters.
 * - Local body frame: Rover-fixed Cartesian coordinates (+X forward, +Y left).
 * - Bearing: Polar angle beta in radians measured counter-clockwise from the rover forward axis (+X).
 */
struct LandmarkObservation {
    uint32_t landmark_id{0};
    core::LandmarkKey landmark_key;

    // Ground-truth quantities
    verification::Point2D ground_truth_world_point;
    verification::Point2D ground_truth_local_point;
    double ground_truth_range{0.0};
    double ground_truth_bearing{0.0};

    // Noisy sensor measurements in the rover body frame
    double measured_range{0.0};
    double measured_bearing{0.0};
    verification::Point2D measured_local_point;
};

/**
 * @brief Observation model extracting visible landmarks from a rover pose with
 *        configurable deterministic range and bearing measurement noise.
 */
class ObservationModel2D {
public:
    /**
     * @brief Constructs an ObservationModel2D with the specified noise parameters.
     * @throws std::invalid_argument if noise configuration is invalid.
     */
    explicit ObservationModel2D(const ObservationNoiseConfig& config);

    const ObservationNoiseConfig& config() const noexcept { return config_; }

    /**
     * @brief Observes landmarks visible from the given rover pose in the synthetic world.
     *
     * @param world Synthetic world containing static landmarks.
     * @param rover_pose Current rover pose in the global world frame.
     * @param observer_rover_id ID of the observing rover (stamped onto local LandmarkKey provenance).
     * @return Deterministically ordered list of landmark observations.
     */
    std::vector<LandmarkObservation> observe(
        const SyntheticWorld2D& world,
        const geometry::SE2& rover_pose,
        uint16_t observer_rover_id = 1);

    /**
     * @brief Extracts geometric point correspondences for co-observed landmarks between
     *        two sets of observations (e.g. from two different rovers or keyframes).
     *
     * Matches observations by identical landmark ID:
     * - source point: coordinates in the source rover/keyframe local frame.
     * - target point: coordinates in the target rover/keyframe local frame.
     *
     * @param obs_source Observations taken from source keyframe.
     * @param obs_target Observations taken from target keyframe.
     * @param use_ground_truth If true, uses ground-truth local points instead of noisy measurements.
     * @return List of 2D point correspondences compatible with geometric verifier.
     */
    static std::vector<verification::PointCorrespondence2D> extract_correspondences(
        const std::vector<LandmarkObservation>& obs_source,
        const std::vector<LandmarkObservation>& obs_target,
        bool use_ground_truth = false);

private:
    ObservationNoiseConfig config_;
    mutable std::mt19937_64 rng_;
    mutable std::normal_distribution<double> dist_range_;
    mutable std::normal_distribution<double> dist_bearing_;
};

} // namespace astromesh::simulation
