#include "astromesh/verification/geometric_verifier.hpp"

#include <cmath>
#include <random>
#include <algorithm>
#include <sstream>

namespace astromesh::verification {

bool estimate_rigid_transform_2d(const std::vector<PointCorrespondence2D>& correspondences,
                                const std::vector<size_t>& indices,
                                astromesh::graph::Pose2D& out_transform) noexcept {
    size_t k = indices.size();
    if (k < 2) {
        return false;
    }

    // 1. Calculate centroids of source and target points
    double sum_src_x = 0.0, sum_src_y = 0.0;
    double sum_tgt_x = 0.0, sum_tgt_y = 0.0;

    for (size_t idx : indices) {
        if (idx >= correspondences.size()) {
            return false;
        }
        const auto& corr = correspondences[idx];
        if (!corr.is_valid()) {
            return false;
        }
        sum_src_x += corr.source.x;
        sum_src_y += corr.source.y;
        sum_tgt_x += corr.target.x;
        sum_tgt_y += corr.target.y;
    }

    double inv_k = 1.0 / static_cast<double>(k);
    double mean_src_x = sum_src_x * inv_k;
    double mean_src_y = sum_src_y * inv_k;
    double mean_tgt_x = sum_tgt_x * inv_k;
    double mean_tgt_y = sum_tgt_y * inv_k;

    // 2. Compute cross-covariance components
    double c1 = 0.0; // sum(u'*x' + v'*y')
    double c2 = 0.0; // sum(v'*x' - u'*y')
    double var_src = 0.0;
    double var_tgt = 0.0;

    for (size_t idx : indices) {
        const auto& corr = correspondences[idx];
        double x_prime = corr.source.x - mean_src_x;
        double y_prime = corr.source.y - mean_src_y;
        double u_prime = corr.target.x - mean_tgt_x;
        double v_prime = corr.target.y - mean_tgt_y;

        c1 += (u_prime * x_prime + v_prime * y_prime);
        c2 += (v_prime * x_prime - u_prime * y_prime);

        var_src += (x_prime * x_prime + y_prime * y_prime);
        var_tgt += (u_prime * u_prime + v_prime * v_prime);
    }

    // 3. Degeneracy checks
    constexpr double EPS_VARIANCE = 1e-10;
    if (var_src < EPS_VARIANCE || var_tgt < EPS_VARIANCE) {
        return false; // Points are coincident; orientation mathematically undefined
    }

    double norm_c = std::hypot(c1, c2);
    if (norm_c < EPS_VARIANCE) {
        return false; // Ambiguous orientation
    }

    // 4. Closed-form optimal orientation maximizing trace(R * H)
    double theta = std::atan2(c2, c1);
    theta = astromesh::graph::normalize_angle(theta);

    // 5. Optimal translation: t = mean_tgt - R(theta) * mean_src
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);
    double tx = mean_tgt_x - (cos_t * mean_src_x - sin_t * mean_src_y);
    double ty = mean_tgt_y - (sin_t * mean_src_x + cos_t * mean_src_y);

    out_transform = astromesh::graph::Pose2D(tx, ty, theta);
    return out_transform.is_valid();
}

GeometricVerificationResult verify_relative_pose_2d(
    const std::vector<PointCorrespondence2D>& correspondences,
    const RansacConfig2D& config) noexcept {

    GeometricVerificationResult result{};
    result.total_correspondences = correspondences.size();
    result.inlier_mask.resize(correspondences.size(), false);

    // 1. Validate configuration parameters
    if (!config.is_valid()) {
        result.success = false;
        result.failure_reason = VerificationFailureReason::InvalidConfiguration;
        result.error_message = "RansacConfig2D parameters are invalid or out of bounds.";
        return result;
    }

    // 2. Validate input coordinate finiteness
    for (const auto& corr : correspondences) {
        if (!corr.is_valid()) {
            result.success = false;
            result.failure_reason = VerificationFailureReason::NonFiniteInputCoordinates;
            result.error_message = "Input correspondence contains non-finite coordinates (NaN or Inf).";
            return result;
        }
    }

    // 3. Check correspondence count threshold
    size_t n = correspondences.size();
    if (n < config.min_correspondences) {
        result.success = false;
        result.failure_reason = VerificationFailureReason::InsufficientCorrespondences;
        std::ostringstream ss;
        ss << "Correspondence count (" << n << ") < min_correspondences (" << config.min_correspondences << ").";
        result.error_message = ss.str();
        return result;
    }

    // 4. Check global spatial variance for degeneracy (e.g. coincident points)
    double sum_src_x = 0.0, sum_src_y = 0.0;
    double sum_tgt_x = 0.0, sum_tgt_y = 0.0;
    for (const auto& c : correspondences) {
        sum_src_x += c.source.x;
        sum_src_y += c.source.y;
        sum_tgt_x += c.target.x;
        sum_tgt_y += c.target.y;
    }
    double inv_n = 1.0 / static_cast<double>(n);
    double m_src_x = sum_src_x * inv_n, m_src_y = sum_src_y * inv_n;
    double m_tgt_x = sum_tgt_x * inv_n, m_tgt_y = sum_tgt_y * inv_n;
    double var_src = 0.0, var_tgt = 0.0;
    for (const auto& c : correspondences) {
        double dx = c.source.x - m_src_x;
        double dy = c.source.y - m_src_y;
        var_src += dx * dx + dy * dy;
        double dtx = c.target.x - m_tgt_x;
        double dty = c.target.y - m_tgt_y;
        var_tgt += dtx * dtx + dty * dty;
    }
    constexpr double EPS_GLOBAL_VAR = 1e-10;
    if (var_src < EPS_GLOBAL_VAR || var_tgt < EPS_GLOBAL_VAR) {
        result.success = false;
        result.failure_reason = VerificationFailureReason::DegeneratePointConfiguration;
        result.error_message = "Input correspondence point set has zero spatial variance (coincident points).";
        return result;
    }

    // 5. RANSAC hypothesis sampling loop
    std::mt19937_64 rng(config.random_seed);
    std::uniform_int_distribution<size_t> dist(0, n - 1);

    std::vector<size_t> best_inliers;
    best_inliers.reserve(n);

    double threshold_sq = config.inlier_distance_threshold * config.inlier_distance_threshold;

    for (size_t iter = 0; iter < config.max_iterations; ++iter) {
        // Draw 2 distinct sample indices
        size_t i1 = dist(rng);
        size_t i2 = dist(rng);
        if (i1 == i2) {
            continue;
        }

        // Geometric rigidity pre-filter on sample pair
        const auto& c1 = correspondences[i1];
        const auto& c2 = correspondences[i2];

        double d_src = std::hypot(c1.source.x - c2.source.x, c1.source.y - c2.source.y);
        double d_tgt = std::hypot(c1.target.x - c2.target.x, c1.target.y - c2.target.y);

        if (d_src < 1e-4 || d_tgt < 1e-4) {
            continue; // Points too close to define stable orientation
        }

        // In a rigid transform, pair distance must be approximately preserved
        if (std::abs(d_src - d_tgt) > 2.0 * config.inlier_distance_threshold) {
            continue; // Pair is mutually inconsistent
        }

        // Estimate 2-point minimal transform
        astromesh::graph::Pose2D candidate_pose;
        std::vector<size_t> pair_indices = {i1, i2};
        if (!estimate_rigid_transform_2d(correspondences, pair_indices, candidate_pose)) {
            continue;
        }

        // Count inliers for this hypothesis
        double cos_c = std::cos(candidate_pose.theta);
        double sin_c = std::sin(candidate_pose.theta);
        std::vector<size_t> current_inliers;
        current_inliers.reserve(n);

        for (size_t j = 0; j < n; ++j) {
            const auto& pt = correspondences[j];
            double pred_x = cos_c * pt.source.x - sin_c * pt.source.y + candidate_pose.x;
            double pred_y = sin_c * pt.source.x + cos_c * pt.source.y + candidate_pose.y;

            double dx = pt.target.x - pred_x;
            double dy = pt.target.y - pred_y;
            double dist_sq = dx * dx + dy * dy;

            if (dist_sq <= threshold_sq) {
                current_inliers.push_back(j);
            }
        }

        if (current_inliers.size() > best_inliers.size()) {
            best_inliers = std::move(current_inliers);
            // Early exit if all correspondences are inliers
            if (best_inliers.size() == n) {
                break;
            }
        }
    }

    // 6. Evaluate consensus inliers
    if (best_inliers.size() < config.min_inliers) {
        result.success = false;
        result.failure_reason = VerificationFailureReason::InsufficientInliers;
        result.inlier_count = best_inliers.size();
        result.inlier_ratio = static_cast<double>(best_inliers.size()) / static_cast<double>(n);
        std::ostringstream ss;
        ss << "Inlier count (" << best_inliers.size() << ") < min_inliers (" << config.min_inliers << ").";
        result.error_message = ss.str();
        return result;
    }

    double inlier_ratio = static_cast<double>(best_inliers.size()) / static_cast<double>(n);
    if (inlier_ratio < config.min_inlier_ratio) {
        result.success = false;
        result.failure_reason = VerificationFailureReason::LowInlierRatio;
        result.inlier_count = best_inliers.size();
        result.inlier_ratio = inlier_ratio;
        std::ostringstream ss;
        ss << "Inlier ratio (" << (inlier_ratio * 100.0) << "%) < min_inlier_ratio ("
           << (config.min_inlier_ratio * 100.0) << "%).";
        result.error_message = ss.str();
        return result;
    }

    // 7. Refine transform on all consensus inliers using least-squares
    astromesh::graph::Pose2D refined_pose;
    if (!estimate_rigid_transform_2d(correspondences, best_inliers, refined_pose)) {
        result.success = false;
        result.failure_reason = VerificationFailureReason::DegeneratePointConfiguration;
        result.error_message = "Consensus inlier points form a degenerate geometric configuration.";
        return result;
    }

    // 8. Recompute consensus inlier mask and residual RMSE with refined transform
    double refined_cos = std::cos(refined_pose.theta);
    double refined_sin = std::sin(refined_pose.theta);
    double sum_sq_residuals = 0.0;
    size_t confirmed_inliers = 0;

    for (size_t j = 0; j < n; ++j) {
        const auto& pt = correspondences[j];
        double pred_x = refined_cos * pt.source.x - refined_sin * pt.source.y + refined_pose.x;
        double pred_y = refined_sin * pt.source.x + refined_cos * pt.source.y + refined_pose.y;

        double dx = pt.target.x - pred_x;
        double dy = pt.target.y - pred_y;
        double dist_sq = dx * dx + dy * dy;

        if (dist_sq <= threshold_sq) {
            result.inlier_mask[j] = true;
            sum_sq_residuals += dist_sq;
            ++confirmed_inliers;
        } else {
            result.inlier_mask[j] = false;
        }
    }

    if (confirmed_inliers < config.min_inliers) {
        result.success = false;
        result.failure_reason = VerificationFailureReason::InsufficientInliers;
        result.inlier_count = confirmed_inliers;
        result.inlier_ratio = static_cast<double>(confirmed_inliers) / static_cast<double>(n);
        result.error_message = "Confirmed inliers after refinement fell below min_inliers threshold.";
        return result;
    }

    // 9. Consistency check against candidate prior pose (if specified)
    if (config.candidate_prior.has_value()) {
        const auto& prior = *config.candidate_prior;
        double dx = refined_pose.x - prior.x;
        double dy = refined_pose.y - prior.y;
        double trans_diff = std::hypot(dx, dy);
        double rot_diff = std::abs(astromesh::graph::normalize_angle(refined_pose.theta - prior.theta));

        if (trans_diff > config.max_prior_translation_diff || rot_diff > config.max_prior_rotation_diff) {
            result.success = false;
            result.failure_reason = VerificationFailureReason::InconsistentWithCandidatePrior;
            std::ostringstream ss;
            ss << "Estimated transform (dx=" << trans_diff << "m, dtheta=" << rot_diff
               << "rad) is inconsistent with candidate prior (max_trans="
               << config.max_prior_translation_diff << "m, max_rot="
               << config.max_prior_rotation_diff << "rad).";
            result.error_message = ss.str();
            result.estimated_transform = refined_pose;
            result.inlier_count = confirmed_inliers;
            result.inlier_ratio = static_cast<double>(confirmed_inliers) / static_cast<double>(n);
            result.inlier_rmse = std::sqrt(sum_sq_residuals / static_cast<double>(confirmed_inliers));
            return result;
        }
    }

    result.success = true;
    result.failure_reason = VerificationFailureReason::None;
    result.error_message.clear();
    result.estimated_transform = refined_pose;
    result.inlier_count = confirmed_inliers;
    result.inlier_ratio = static_cast<double>(confirmed_inliers) / static_cast<double>(n);
    result.inlier_rmse = std::sqrt(sum_sq_residuals / static_cast<double>(confirmed_inliers));

    return result;
}

GeometricVerificationResult gate_and_promote_loop_closure(
    astromesh::graph::PoseGraph& graph,
    uint64_t edge_id,
    const std::vector<PointCorrespondence2D>& correspondences,
    const RansacConfig2D& config) {

    // 1. Verify that edge exists in the graph
    const auto* edge_ptr = graph.find_edge(edge_id);
    if (!edge_ptr) {
        GeometricVerificationResult r{};
        r.success = false;
        r.failure_reason = VerificationFailureReason::InvalidConfiguration;
        r.error_message = "Edge ID does not exist in graph.";
        return r;
    }

    // 2. Odometry edges cannot be gated as loop closures
    if (edge_ptr->type == astromesh::graph::EdgeType::Odometry) {
        GeometricVerificationResult r{};
        r.success = false;
        r.failure_reason = VerificationFailureReason::InvalidConfiguration;
        r.error_message = "Cannot gate Odometry edge as a loop closure.";
        return r;
    }

    // 3. Only an edge currently in Unverified candidate state can be gated
    if (edge_ptr->status != astromesh::graph::EdgeVerificationStatus::Unverified) {
        GeometricVerificationResult r{};
        r.success = false;
        r.failure_reason = VerificationFailureReason::InvalidConfiguration;
        r.error_message = "Edge is not in Unverified candidate state.";
        return r;
    }

    // 4. Verify endpoints exist in graph
    if (!graph.has_node(edge_ptr->from_node) || !graph.has_node(edge_ptr->to_node)) {
        GeometricVerificationResult r{};
        r.success = false;
        r.failure_reason = VerificationFailureReason::InvalidConfiguration;
        r.error_message = "Edge endpoint nodes do not exist in graph.";
        return r;
    }

    // 5. Setup effective configuration (populate candidate_prior from edge if prior tolerances are active)
    RansacConfig2D effective_config = config;
    if (!effective_config.candidate_prior.has_value() &&
        (std::isfinite(effective_config.max_prior_translation_diff) ||
         std::isfinite(effective_config.max_prior_rotation_diff))) {
        effective_config.candidate_prior = edge_ptr->relative_pose;
    }

    auto result = verify_relative_pose_2d(correspondences, effective_config);

    // 6. Update edge status atomically
    if (result.success) {
        graph.update_edge_verification(
            edge_id,
            astromesh::graph::EdgeVerificationStatus::Verified,
            result.estimated_transform);
    } else if (result.failure_reason != VerificationFailureReason::InvalidConfiguration) {
        // Only mark Rejected if verification actually ran and failed (not if config was invalid)
        graph.update_edge_verification(
            edge_id,
            astromesh::graph::EdgeVerificationStatus::Rejected);
    }

    return result;
}

} // namespace astromesh::verification
