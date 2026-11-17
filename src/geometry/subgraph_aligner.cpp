#include "astromesh/geometry/subgraph_aligner.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>

namespace astromesh::geometry {

bool SubgraphAligner::estimate_se2_kabsch(
    const std::vector<verification::PointCorrespondence2D>& correspondences,
    SE2& out_transform,
    double& out_rmse) noexcept {

    size_t n = correspondences.size();
    if (n < 2) {
        return false;
    }

    // 1. Centroids
    double cx_src = 0.0, cy_src = 0.0;
    double cx_tgt = 0.0, cy_tgt = 0.0;

    for (const auto& c : correspondences) {
        cx_src += c.source.x;
        cy_src += c.source.y;
        cx_tgt += c.target.x;
        cy_tgt += c.target.y;
    }

    cx_src /= static_cast<double>(n);
    cy_src /= static_cast<double>(n);
    cx_tgt /= static_cast<double>(n);
    cy_tgt /= static_cast<double>(n);

    // 2. Spatial Variance Check
    double var_src = 0.0, var_tgt = 0.0;
    for (const auto& c : correspondences) {
        double dx_s = c.source.x - cx_src;
        double dy_s = c.source.y - cy_src;
        double dx_t = c.target.x - cx_tgt;
        double dy_t = c.target.y - cy_tgt;
        var_src += dx_s * dx_s + dy_s * dy_s;
        var_tgt += dx_t * dx_t + dy_t * dy_t;
    }
    var_src /= static_cast<double>(n);
    var_tgt /= static_cast<double>(n);

    if (var_src < 1e-4 || var_tgt < 1e-4) {
        return false; // Degenerate / coincident points
    }

    // 3. Cross-Covariance
    double s_xx = 0.0, s_xy = 0.0;
    double s_yx = 0.0, s_yy = 0.0;

    for (const auto& c : correspondences) {
        double q_src_x = c.source.x - cx_src;
        double q_src_y = c.source.y - cy_src;
        double q_tgt_x = c.target.x - cx_tgt;
        double q_tgt_y = c.target.y - cy_tgt;

        s_xx += q_tgt_x * q_src_x;
        s_xy += q_tgt_x * q_src_y;
        s_yx += q_tgt_y * q_src_x;
        s_yy += q_tgt_y * q_src_y;
    }

    // 4. Planar Rotation & Translation
    double theta = std::atan2(s_yx - s_xy, s_xx + s_yy);
    double cos_th = std::cos(theta);
    double sin_th = std::sin(theta);

    double tx = cx_tgt - (cos_th * cx_src - sin_th * cy_src);
    double ty = cy_tgt - (sin_th * cx_src + cos_th * cy_src);

    out_transform = SE2(tx, ty, theta);

    // 5. Residual RMSE
    double sq_err_sum = 0.0;
    for (const auto& c : correspondences) {
        double px_est = cos_th * c.source.x - sin_th * c.source.y + tx;
        double py_est = sin_th * c.source.x + cos_th * c.source.y + ty;
        double err_x = c.target.x - px_est;
        double err_y = c.target.y - py_est;
        sq_err_sum += err_x * err_x + err_y * err_y;
    }

    out_rmse = std::sqrt(sq_err_sum / static_cast<double>(n));
    return true;
}

SubgraphAlignmentResult SubgraphAligner::align_subgraphs(
    uint16_t target_rover_id,
    const std::vector<SubgraphLandmarkObservation>& target_landmarks,
    uint16_t source_rover_id,
    const std::vector<SubgraphLandmarkObservation>& source_landmarks,
    double inlier_threshold) {

    SubgraphAlignmentResult res;
    res.target_rover_id = target_rover_id;
    res.source_rover_id = source_rover_id;

    // 1. Associate mutual landmarks by LandmarkKey
    std::unordered_map<core::LandmarkKey, verification::Point2D> target_map;
    for (const auto& obs : target_landmarks) {
        if (obs.key.is_valid()) {
            target_map[obs.key] = obs.local_pos;
        }
    }

    std::vector<verification::PointCorrespondence2D> correspondences;
    for (const auto& obs : source_landmarks) {
        auto it = target_map.find(obs.key);
        if (it != target_map.end()) {
            correspondences.emplace_back(obs.local_pos, it->second, obs.key);
        }
    }

    res.correspondence_count = correspondences.size();
    if (correspondences.size() < MIN_CORRESPONDENCES) {
        res.success = false;
        res.failure_reason = "Insufficient mutual landmark correspondences (< 3).";
        return res;
    }

    // 2. Initial Kabsch estimate
    SE2 initial_tx;
    double initial_rmse = 0.0;
    if (!estimate_se2_kabsch(correspondences, initial_tx, initial_rmse)) {
        res.success = false;
        res.failure_reason = "Geometric degeneracy detected in correspondence layout.";
        return res;
    }

    // 3. RANSAC consensus inlier filtering
    std::mt19937_64 rng(42);
    size_t best_inliers_count = 0;
    SE2 best_transform = initial_tx;
    double best_rmse = initial_rmse;
    std::vector<verification::PointCorrespondence2D> best_inlier_set;

    constexpr size_t RANSAC_ITERATIONS = 50;
    for (size_t iter = 0; iter < RANSAC_ITERATIONS; ++iter) {
        // Sample 2 random distinct correspondences
        size_t idx1 = rng() % correspondences.size();
        size_t idx2 = rng() % correspondences.size();
        if (idx1 == idx2) continue;

        std::vector<verification::PointCorrespondence2D> sample = {
            correspondences[idx1], correspondences[idx2]
        };

        SE2 hyp_tx;
        double hyp_rmse = 0.0;
        if (!estimate_se2_kabsch(sample, hyp_tx, hyp_rmse)) {
            continue;
        }

        // Count inliers
        std::vector<verification::PointCorrespondence2D> inliers;
        for (const auto& c : correspondences) {
            auto p_est = hyp_tx.transform(c.source);
            double dist = std::hypot(c.target.x - p_est.x, c.target.y - p_est.y);
            if (dist <= inlier_threshold) {
                inliers.push_back(c);
            }
        }

        if (inliers.size() > best_inliers_count && inliers.size() >= MIN_CORRESPONDENCES) {
            SE2 refined_tx;
            double refined_rmse = 0.0;
            if (estimate_se2_kabsch(inliers, refined_tx, refined_rmse)) {
                best_inliers_count = inliers.size();
                best_transform = refined_tx;
                best_rmse = refined_rmse;
                best_inlier_set = std::move(inliers);
            }
        }
    }

    if (best_inliers_count < MIN_CORRESPONDENCES || best_rmse > MAX_ACCEPTABLE_RMSE) {
        // Fallback to initial Kabsch if valid
        if (initial_rmse <= MAX_ACCEPTABLE_RMSE) {
            res.success = true;
            res.transform_target_from_source = initial_tx;
            res.inlier_count = correspondences.size();
            res.inlier_rmse = initial_rmse;
            res.confidence = static_cast<double>(res.inlier_count) / (1.0 + 10.0 * initial_rmse);
            return res;
        }
        res.success = false;
        res.failure_reason = "Inlier RMSE exceeded tolerance or insufficient inliers found.";
        return res;
    }

    res.success = true;
    res.transform_target_from_source = best_transform;
    res.inlier_count = best_inliers_count;
    res.inlier_rmse = best_rmse;
    res.confidence = static_cast<double>(best_inliers_count) / (1.0 + 10.0 * best_rmse);
    return res;
}

} // namespace astromesh::geometry

