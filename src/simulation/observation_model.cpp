#include "astromesh/simulation/observation_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace astromesh::simulation {

ObservationModel2D::ObservationModel2D(const ObservationNoiseConfig& config)
    : config_(config),
      rng_(config.seed),
      dist_range_(0.0, config.range_std_dev > 0.0 ? config.range_std_dev : 1.0),
      dist_bearing_(0.0, config.bearing_std_dev > 0.0 ? config.bearing_std_dev : 1.0) {
    if (!config_.is_valid()) {
        throw std::invalid_argument("ObservationModel2D: invalid ObservationNoiseConfig specified.");
    }
}

std::vector<LandmarkObservation> ObservationModel2D::observe(
    const SyntheticWorld2D& world,
    const geometry::SE2& rover_pose,
    uint16_t observer_rover_id) {

    if (!rover_pose.is_valid()) {
        return {};
    }

    const auto visible_landmarks = world.query_visible_landmarks(rover_pose);
    std::vector<LandmarkObservation> observations;
    observations.reserve(visible_landmarks.size());

    for (const auto& lm : visible_landmarks) {
        LandmarkObservation obs;
        obs.landmark_id = lm.id;

        // Retain canonical landmark key from the synthetic world
        obs.landmark_key = lm.key;

        obs.ground_truth_world_point = lm.position;

        // Transform world coordinates into rover-local frame (+X forward, +Y left)
        obs.ground_truth_local_point = rover_pose.inverse_transform(lm.position);

        const double lx = obs.ground_truth_local_point.x;
        const double ly = obs.ground_truth_local_point.y;
        obs.ground_truth_range = std::hypot(lx, ly);
        obs.ground_truth_bearing = std::atan2(ly, lx);

        // Apply noise if standard deviation is non-zero
        if (config_.range_std_dev > 0.0) {
            const double noise_r = dist_range_(rng_);
            obs.measured_range = std::max(0.001, obs.ground_truth_range + noise_r);
        } else {
            obs.measured_range = obs.ground_truth_range;
        }

        if (config_.bearing_std_dev > 0.0) {
            const double noise_b = dist_bearing_(rng_);
            obs.measured_bearing = graph::normalize_angle(obs.ground_truth_bearing + noise_b);
        } else {
            obs.measured_bearing = obs.ground_truth_bearing;
        }

        // Reconstruct measured local Cartesian coordinates from noisy polar measurements
        obs.measured_local_point.x = obs.measured_range * std::cos(obs.measured_bearing);
        obs.measured_local_point.y = obs.measured_range * std::sin(obs.measured_bearing);

        observations.push_back(obs);
    }

    return observations;
}

std::vector<verification::PointCorrespondence2D> ObservationModel2D::extract_correspondences(
    const std::vector<LandmarkObservation>& obs_source,
    const std::vector<LandmarkObservation>& obs_target,
    bool use_ground_truth) {

    std::unordered_map<uint32_t, const LandmarkObservation*> target_map;
    target_map.reserve(obs_target.size());
    for (const auto& tgt : obs_target) {
        target_map[tgt.landmark_id] = &tgt;
    }

    std::vector<verification::PointCorrespondence2D> correspondences;

    for (const auto& src : obs_source) {
        auto it = target_map.find(src.landmark_id);
        if (it != target_map.end()) {
            const auto* tgt = it->second;

            const verification::Point2D p_src = use_ground_truth ?
                src.ground_truth_local_point : src.measured_local_point;
            const verification::Point2D p_tgt = use_ground_truth ?
                tgt->ground_truth_local_point : tgt->measured_local_point;

            correspondences.emplace_back(p_src, p_tgt, src.landmark_key);
        }
    }

    // Sort deterministically by landmark ID via landmark_key local_index
    std::sort(correspondences.begin(), correspondences.end(),
        [](const verification::PointCorrespondence2D& a, const verification::PointCorrespondence2D& b) {
            const uint32_t id_a = a.landmark_key ? a.landmark_key->local_index() : 0;
            const uint32_t id_b = b.landmark_key ? b.landmark_key->local_index() : 0;
            return id_a < id_b;
        });

    return correspondences;
}

} // namespace astromesh::simulation
