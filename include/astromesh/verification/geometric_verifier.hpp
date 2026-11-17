#pragma once

#include "astromesh/core/landmark_key.hpp"
#include "astromesh/graph/pose_graph.hpp"

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <vector>
#include <string>
#include <optional>

namespace astromesh::verification {

/**
 * @brief Represents a 2D Cartesian point in meters in a local keyframe reference frame.
 */
struct Point2D {
    double x{0.0};
    double y{0.0};

    constexpr Point2D() noexcept = default;
    constexpr Point2D(double px, double py) noexcept : x(px), y(py) {}

    bool is_valid() const noexcept {
        return std::isfinite(x) && std::isfinite(y);
    }

    bool operator==(const Point2D& other) const noexcept {
        constexpr double EPS = 1e-9;
        return std::abs(x - other.x) < EPS && std::abs(y - other.y) < EPS;
    }

    bool operator!=(const Point2D& other) const noexcept {
        return !(*this == other);
    }
};

/**
 * @brief Represents a paired 2D spatial landmark observation between two keyframes:
 *        - source: Landmark coordinates (meters) in Source keyframe frame A.
 *        - target: Landmark coordinates (meters) in Target keyframe frame B.
 *        - landmark_key: Optional Phase 1 landmark key proving provenance/descriptor identity.
 *
 * Coordinate Frame Convention:
 *   Right-handed 2D body/local coordinate frame (+X forward, +Y left, yaw counter-clockwise).
 *   The rigid transformation T_{A -> B} = (tx, ty, theta) maps source points to target points:
 *     p_target = R(theta) * p_source + t
 *     [ x_tgt ]   [ cos(theta) -sin(theta) ] [ x_src ]   [ tx ]
 *     [ y_tgt ] = [ sin(theta)  cos(theta) ] [ y_src ] + [ ty ]
 */
struct PointCorrespondence2D {
    Point2D source;
    Point2D target;
    std::optional<astromesh::core::LandmarkKey> landmark_key;

    constexpr PointCorrespondence2D() = default;
    PointCorrespondence2D(const Point2D& src, const Point2D& tgt,
                          const std::optional<astromesh::core::LandmarkKey>& key = std::nullopt)
        : source(src), target(tgt), landmark_key(key) {}

    bool is_valid() const noexcept {
        return source.is_valid() && target.is_valid();
    }
};

#include <limits>

/**
 * @brief Configurable parameters for the RANSAC 2D rigid transform estimation pipeline.
 */
struct RansacConfig2D {
    double inlier_distance_threshold{0.20}; ///< Inlier distance threshold in meters (default: 0.20 m = 20 cm).
    size_t min_correspondences{4};          ///< Minimum total correspondences required to attempt verification.
    size_t min_inliers{4};                  ///< Minimum consensus inliers required for acceptance.
    double min_inlier_ratio{0.50};          ///< Minimum inlier ratio (inliers / total) (default: 0.50 = 50%).
    size_t max_iterations{100};             ///< Maximum RANSAC hypothesis iterations.
    uint64_t random_seed{42};               ///< Deterministic seed for reproducible RANSAC hypothesis sampling.

    /// Optional candidate prior pose for geometric consistency gating
    std::optional<astromesh::graph::Pose2D> candidate_prior{std::nullopt};
    double max_prior_translation_diff{std::numeric_limits<double>::infinity()}; ///< Max allowed translation deviation from prior (meters).
    double max_prior_rotation_diff{std::numeric_limits<double>::infinity()};    ///< Max allowed yaw deviation from prior (radians).

    bool is_valid() const noexcept {
        return std::isfinite(inlier_distance_threshold) && inlier_distance_threshold > 0.0 &&
               min_correspondences >= 2 &&
               min_inliers >= 2 &&
               std::isfinite(min_inlier_ratio) && min_inlier_ratio > 0.0 && min_inlier_ratio <= 1.0 &&
               max_iterations >= 1 &&
               (!candidate_prior.has_value() || candidate_prior->is_valid()) &&
               (!std::isfinite(max_prior_translation_diff) || max_prior_translation_diff >= 0.0) &&
               (!std::isfinite(max_prior_rotation_diff) || max_prior_rotation_diff >= 0.0);
    }
};

/**
 * @brief Failure diagnosis codes for geometric verification.
 */
enum class VerificationFailureReason {
    None,                           ///< Verification succeeded.
    InvalidConfiguration,           ///< RansacConfig2D parameters are invalid/out-of-bounds.
    NonFiniteInputCoordinates,      ///< Source or target coordinates contain NaN or infinity.
    InsufficientCorrespondences,    ///< Total correspondences < min_correspondences.
    DegeneratePointConfiguration,   ///< Points are collinear, coincident, or orientation is ambiguous.
    InsufficientInliers,            ///< Inlier count < min_inliers.
    LowInlierRatio,                 ///< Inlier ratio < min_inlier_ratio.
    InconsistentWithCandidatePrior  ///< Estimated transform conflicts with candidate prior pose beyond tolerance.
};

/**
 * @brief Structured outcome returned by the geometric verification pipeline.
 */
struct GeometricVerificationResult {
    bool success{false};
    VerificationFailureReason failure_reason{VerificationFailureReason::None};
    std::string error_message;

    astromesh::graph::Pose2D estimated_transform; ///< Refined rigid SE(2) relative pose (tx, ty, theta).
    size_t total_correspondences{0};              ///< Total input correspondence count.
    size_t inlier_count{0};                       ///< Number of consensus inliers.
    double inlier_ratio{0.0};                     ///< inlier_count / total_correspondences.
    double inlier_rmse{0.0};                      ///< Root-mean-square residual error over consensus inliers (meters).
    std::vector<bool> inlier_mask;                ///< Per-correspondence inlier flag (size == total_correspondences).
};

/**
 * @brief Closed-form least-squares rigid 2D transform estimator (Kabsch / Umeyama without scale).
 *
 * Given N >= 2 correspondences, computes optimal rotation angle theta and translation (tx, ty)
 * minimizing the sum of squared Euclidean residuals:
 *   E(R, t) = sum_{i=1}^N || p_{tgt, i} - (R(theta) * p_{src, i} + t) ||^2
 *
 * @param correspondences Input correspondence vector.
 * @param indices Subset of indices to use for estimation.
 * @param out_transform Populated with estimated Pose2D on success.
 * @return true if estimation succeeded; false if points are degenerate/coincident.
 */
bool estimate_rigid_transform_2d(const std::vector<PointCorrespondence2D>& correspondences,
                                const std::vector<size_t>& indices,
                                astromesh::graph::Pose2D& out_transform) noexcept;

/**
 * @brief Deterministic RANSAC-style geometric verification pipeline.
 *
 * Identifies consensus inliers and estimates a refined planar SE(2) relative transform.
 * Rejects perceptual aliasing outliers and degenerate geometries.
 *
 * @param correspondences Paired 2D landmark observations.
 * @param config Configurable thresholds and iteration parameters.
 * @return Detailed GeometricVerificationResult.
 */
GeometricVerificationResult verify_relative_pose_2d(
    const std::vector<PointCorrespondence2D>& correspondences,
    const RansacConfig2D& config = RansacConfig2D{}) noexcept;

/**
 * @brief Safely gates and promotes a PoseGraph loop-closure edge based on geometric verification.
 *
 * Scientific Contract:
 * - If verification succeeds with consensus inliers, the edge status is updated to `Verified`
 *   and its relative pose is updated with the refined transform estimate.
 * - If verification fails (outliers, insufficient data, or geometric inconsistency), the edge
 *   status is updated to `Rejected` (or remains `Unverified` if edge was non-existent).
 *
 * @param graph The mutable PoseGraph.
 * @param edge_id The identifier of the edge to gate.
 * @param correspondences Point correspondences associated with this candidate edge.
 * @param config RANSAC parameters.
 * @return GeometricVerificationResult detailing the outcome.
 */
GeometricVerificationResult gate_and_promote_loop_closure(
    astromesh::graph::PoseGraph& graph,
    uint64_t edge_id,
    const std::vector<PointCorrespondence2D>& correspondences,
    const RansacConfig2D& config = RansacConfig2D{});

} // namespace astromesh::verification
