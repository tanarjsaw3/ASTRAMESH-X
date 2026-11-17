#pragma once

#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"

#include <cstddef>
#include <string>
#include <vector>
#include <unordered_map>

namespace astromesh::optimization {

/**
 * @brief Summary statistics for a 1D scalar error distribution (e.g. translation or rotation errors).
 */
struct ErrorStatistics {
    double rmse{0.0};
    double mean{0.0};
    double median{0.0};
    double std_dev{0.0};
    double min_error{0.0};
    double max_error{0.0};
};

/**
 * @brief Absolute Trajectory Error (ATE) evaluation outcome comparing estimated poses against ground truth.
 *
 * Direct metric (align == false):
 *   Evaluates error directly in the anchor/world frame:
 *     e_trans_k = || t_est,k - t_gt,k ||_2
 *     e_rot_k = | normalize_angle(theta_est,k - theta_gt,k) |
 *
 * Aligned metric (align == true):
 *   Computes the optimal rigid SE(2) transformation S = [R(phi), t_align] aligning estimated positions
 *   to ground-truth positions via 2D Horn/Umeyama closed-form orthogonal Procrustes analysis:
 *     t_aligned = R(phi) * t_est + t_align
 *     theta_aligned = theta_est + phi
 *   This isolates shape reconstruction error from arbitrary global coordinate frame alignment.
 */
struct AbsoluteTrajectoryError {
    bool valid{false};
    size_t num_poses{0};
    bool alignment_applied{false};
    geometry::SE2 aligning_transform{geometry::SE2::identity()};
    ErrorStatistics translation; ///< Translation error statistics in meters.
    ErrorStatistics rotation;    ///< Rotation error statistics in radians.
    std::string error_message;
};

/**
 * @brief Relative Pose Error (RPE) evaluation outcome over consecutive keyframe intervals.
 *
 * For keyframe pair (k, k+delta):
 *   Ground truth delta: T_gt = X_gt,k^{-1} * X_gt,k+delta
 *   Estimated delta:    T_est = X_est,k^{-1} * X_est,k+delta
 *   Delta error:        E = T_gt^{-1} * T_est
 *   e_trans = || t_E ||_2,  e_rot = | theta_E |
 */
struct RelativePoseError {
    bool valid{false};
    size_t num_pairs{0};
    size_t delta{1};
    ErrorStatistics translation; ///< Relative translation error statistics in meters.
    ErrorStatistics rotation;    ///< Relative rotation error statistics in radians.
    std::string error_message;
};

/**
 * @brief Combined trajectory evaluation metrics summary.
 */
struct TrajectoryEvaluation {
    bool valid{false};
    size_t num_poses{0};
    AbsoluteTrajectoryError ate_raw;     ///< ATE without rigid alignment.
    AbsoluteTrajectoryError ate_aligned; ///< ATE with optimal rigid SE(2) Umeyama alignment.
    RelativePoseError rpe;               ///< RPE with step interval delta.
    std::string error_message;
};

/**
 * @brief Computes 2D rigid Horn/Umeyama alignment transformation S in SE(2) that minimizes
 *        sum_k || S * t_est,k - t_gt,k ||^2.
 */
geometry::SE2 compute_umeyama_alignment_2d(
    const std::vector<verification::Point2D>& est_points,
    const std::vector<verification::Point2D>& gt_points) noexcept;

/**
 * @brief Computes Absolute Trajectory Error (ATE) between estimated and ground-truth poses.
 *
 * @param estimated Estimated pose map.
 * @param ground_truth Ground-truth pose map.
 * @param align If true, applies 2D Umeyama rigid alignment before computing errors.
 * @return AbsoluteTrajectoryError Detailed statistics or valid=false on empty/mismatched input.
 */
AbsoluteTrajectoryError compute_ate(
    const std::unordered_map<graph::NodeId, geometry::SE2>& estimated,
    const std::unordered_map<graph::NodeId, geometry::SE2>& ground_truth,
    bool align = false);

/**
 * @brief Computes Relative Pose Error (RPE) between consecutive keyframes.
 *
 * @param estimated Estimated pose map.
 * @param ground_truth Ground-truth pose map.
 * @param delta Keyframe index step separation (default: 1).
 * @return RelativePoseError Detailed statistics or valid=false on empty/mismatched input.
 */
RelativePoseError compute_rpe(
    const std::unordered_map<graph::NodeId, geometry::SE2>& estimated,
    const std::unordered_map<graph::NodeId, geometry::SE2>& ground_truth,
    size_t delta = 1);

/**
 * @brief Evaluates full trajectory metric suite (raw ATE, aligned ATE, and RPE).
 */
TrajectoryEvaluation evaluate_trajectory(
    const std::unordered_map<graph::NodeId, geometry::SE2>& estimated,
    const std::unordered_map<graph::NodeId, geometry::SE2>& ground_truth,
    size_t rpe_delta = 1);

} // namespace astromesh::optimization
