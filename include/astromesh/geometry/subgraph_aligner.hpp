#pragma once

#include "astromesh/core/landmark_key.hpp"
#include "astromesh/geometry/se2.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace astromesh::geometry {

struct SubgraphLandmarkObservation {
    core::LandmarkKey key;
    verification::Point2D local_pos;
};

struct SubgraphAlignmentResult {
    bool success{false};
    uint16_t source_rover_id{0}; // Rover B
    uint16_t target_rover_id{0}; // Rover A
    SE2 transform_target_from_source; // T_AB mapping points from B into A: p_A = T_AB * p_B
    size_t correspondence_count{0};
    size_t inlier_count{0};
    double inlier_rmse{0.0};
    double confidence{0.0};
    std::string failure_reason;
};

class SubgraphAligner {
public:
    static constexpr size_t MIN_CORRESPONDENCES = 3;
    static constexpr double MIN_SPATIAL_VARIANCE = 0.05; // 5 cm^2
    static constexpr double MAX_ACCEPTABLE_RMSE = 0.40;   // 40 cm

    SubgraphAligner() = default;

    /**
     * @brief Computes optimal SE(2) transform T_AB aligning Rover B's frame into Rover A's frame
     * using verified shared landmark observations.
     */
    static SubgraphAlignmentResult align_subgraphs(
        uint16_t target_rover_id, // Rover A
        const std::vector<SubgraphLandmarkObservation>& target_landmarks,
        uint16_t source_rover_id, // Rover B
        const std::vector<SubgraphLandmarkObservation>& source_landmarks,
        double inlier_threshold = 0.25);

    /**
     * @brief Estimates rigid SE(2) transform directly from paired point correspondences.
     */
    static bool estimate_se2_kabsch(
        const std::vector<verification::PointCorrespondence2D>& correspondences,
        SE2& out_transform,
        double& out_rmse) noexcept;
};

} // namespace astromesh::geometry

