#include "astromesh/optimization/trajectory_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>

namespace astromesh::optimization {

namespace {

ErrorStatistics compute_statistics(std::vector<double>& errors) {
    ErrorStatistics stats;
    if (errors.empty()) {
        return stats;
    }

    const size_t n = errors.size();
    std::sort(errors.begin(), errors.end());

    double sum = 0.0;
    double sum_sq = 0.0;
    for (double e : errors) {
        sum += e;
        sum_sq += e * e;
    }

    stats.mean = sum / static_cast<double>(n);
    stats.rmse = std::sqrt(sum_sq / static_cast<double>(n));
    stats.min_error = errors.front();
    stats.max_error = errors.back();

    if (n % 2 == 1) {
        stats.median = errors[n / 2];
    } else {
        stats.median = 0.5 * (errors[n / 2 - 1] + errors[n / 2]);
    }

    double variance = 0.0;
    for (double e : errors) {
        const double diff = e - stats.mean;
        variance += diff * diff;
    }
    stats.std_dev = std::sqrt(variance / static_cast<double>(n));

    return stats;
}

} // namespace

geometry::SE2 compute_umeyama_alignment_2d(
    const std::vector<verification::Point2D>& est_points,
    const std::vector<verification::Point2D>& gt_points) noexcept {

    if (est_points.size() != gt_points.size() || est_points.empty()) {
        return geometry::SE2::identity();
    }

    const size_t n = est_points.size();
    if (n == 1) {
        // Single point: pure translation
        return geometry::SE2(gt_points[0].x - est_points[0].x,
                             gt_points[0].y - est_points[0].y,
                             0.0);
    }

    // Centroids
    double mx_est = 0.0;
    double my_est = 0.0;
    double mx_gt = 0.0;
    double my_gt = 0.0;

    for (size_t i = 0; i < n; ++i) {
        mx_est += est_points[i].x;
        my_est += est_points[i].y;
        mx_gt += gt_points[i].x;
        my_gt += gt_points[i].y;
    }
    mx_est /= static_cast<double>(n);
    my_est /= static_cast<double>(n);
    mx_gt /= static_cast<double>(n);
    my_gt /= static_cast<double>(n);

    // Cross-covariance terms
    double s_xx = 0.0;
    double s_xy = 0.0;
    double s_yx = 0.0;
    double s_yy = 0.0;

    for (size_t i = 0; i < n; ++i) {
        const double px = est_points[i].x - mx_est;
        const double py = est_points[i].y - my_est;
        const double qx = gt_points[i].x - mx_gt;
        const double qy = gt_points[i].y - my_gt;

        s_xx += px * qx;
        s_xy += px * qy;
        s_yx += py * qx;
        s_yy += py * qy;
    }

    // Optimal 2D rotation angle: maximizes cos(phi)*(s_xx + s_yy) + sin(phi)*(s_xy - s_yx)
    const double phi = std::atan2(s_xy - s_yx, s_xx + s_yy);

    // Optimal translation: t_align = m_gt - R(phi) * m_est
    const double cos_phi = std::cos(phi);
    const double sin_phi = std::sin(phi);

    const double tx = mx_gt - (cos_phi * mx_est - sin_phi * my_est);
    const double ty = my_gt - (sin_phi * mx_est + cos_phi * my_est);

    return geometry::SE2(tx, ty, phi);
}

AbsoluteTrajectoryError compute_ate(
    const std::unordered_map<graph::NodeId, geometry::SE2>& estimated,
    const std::unordered_map<graph::NodeId, geometry::SE2>& ground_truth,
    bool align) {

    AbsoluteTrajectoryError result;
    if (estimated.empty()) {
        result.valid = false;
        result.error_message = "Estimated poses map is empty";
        return result;
    }
    if (ground_truth.empty()) {
        result.valid = false;
        result.error_message = "Ground truth poses map is empty";
        return result;
    }

    // Find intersection of NodeIds
    std::vector<graph::NodeId> common_nodes;
    common_nodes.reserve(estimated.size());
    for (const auto& [nid, _] : estimated) {
        if (ground_truth.find(nid) != ground_truth.end()) {
            common_nodes.push_back(nid);
        }
    }

    if (common_nodes.empty()) {
        result.valid = false;
        result.error_message = "No common nodes between estimated poses and ground truth";
        return result;
    }

    // Sort nodes deterministically
    std::sort(common_nodes.begin(), common_nodes.end(), [](const graph::NodeId& a, const graph::NodeId& b) {
        if (a.rover_id != b.rover_id) return a.rover_id < b.rover_id;
        return a.local_index < b.local_index;
    });

    geometry::SE2 aligning_transform = geometry::SE2::identity();
    if (align) {
        std::vector<verification::Point2D> est_pts;
        std::vector<verification::Point2D> gt_pts;
        est_pts.reserve(common_nodes.size());
        gt_pts.reserve(common_nodes.size());

        for (const auto& nid : common_nodes) {
            const auto& p_est = estimated.at(nid);
            const auto& p_gt = ground_truth.at(nid);
            est_pts.emplace_back(p_est.x(), p_est.y());
            gt_pts.emplace_back(p_gt.x(), p_gt.y());
        }

        aligning_transform = compute_umeyama_alignment_2d(est_pts, gt_pts);
    }

    std::vector<double> trans_errors;
    std::vector<double> rot_errors;
    trans_errors.reserve(common_nodes.size());
    rot_errors.reserve(common_nodes.size());

    for (const auto& nid : common_nodes) {
        geometry::SE2 p_est = estimated.at(nid);
        if (align) {
            // Apply rigid transformation: S * X_est
            p_est = aligning_transform * p_est;
        }
        const geometry::SE2& p_gt = ground_truth.at(nid);

        const double dx = p_est.x() - p_gt.x();
        const double dy = p_est.y() - p_gt.y();
        const double trans_err = std::sqrt(dx * dx + dy * dy);

        const double rot_err = std::abs(graph::normalize_angle(p_est.theta() - p_gt.theta()));

        trans_errors.push_back(trans_err);
        rot_errors.push_back(rot_err);
    }

    result.valid = true;
    result.num_poses = common_nodes.size();
    result.alignment_applied = align;
    result.aligning_transform = aligning_transform;
    result.translation = compute_statistics(trans_errors);
    result.rotation = compute_statistics(rot_errors);

    return result;
}

RelativePoseError compute_rpe(
    const std::unordered_map<graph::NodeId, geometry::SE2>& estimated,
    const std::unordered_map<graph::NodeId, geometry::SE2>& ground_truth,
    size_t delta) {

    RelativePoseError result;
    result.delta = delta;

    if (delta == 0) {
        result.valid = false;
        result.error_message = "RPE step delta must be greater than zero";
        return result;
    }
    if (estimated.size() <= delta || ground_truth.size() <= delta) {
        result.valid = false;
        result.error_message = "Insufficient poses to evaluate RPE with requested delta";
        return result;
    }

    // Group common nodes by rover_id to preserve trajectory sequence
    std::unordered_map<uint16_t, std::vector<graph::NodeId>> rover_trajectories;
    for (const auto& [nid, _] : estimated) {
        if (ground_truth.find(nid) != ground_truth.end()) {
            rover_trajectories[nid.rover_id].push_back(nid);
        }
    }

    std::vector<double> trans_errors;
    std::vector<double> rot_errors;

    for (auto& [rid, nodes] : rover_trajectories) {
        std::sort(nodes.begin(), nodes.end(), [](const graph::NodeId& a, const graph::NodeId& b) {
            return a.local_index < b.local_index;
        });

        if (nodes.size() <= delta) {
            continue;
        }

        for (size_t i = 0; i + delta < nodes.size(); ++i) {
            const auto& nid_a = nodes[i];
            const auto& nid_b = nodes[i + delta];

            const geometry::SE2& est_a = estimated.at(nid_a);
            const geometry::SE2& est_b = estimated.at(nid_b);
            const geometry::SE2& gt_a = ground_truth.at(nid_a);
            const geometry::SE2& gt_b = ground_truth.at(nid_b);

            // Relative transforms: T = X_a^{-1} * X_b
            const geometry::SE2 t_est = est_a.inverse() * est_b;
            const geometry::SE2 t_gt = gt_a.inverse() * gt_b;

            // Delta error: E = T_gt^{-1} * T_est
            const geometry::SE2 e = t_gt.inverse() * t_est;

            const double trans_err = std::sqrt(e.x() * e.x() + e.y() * e.y());
            const double rot_err = std::abs(graph::normalize_angle(e.theta()));

            trans_errors.push_back(trans_err);
            rot_errors.push_back(rot_err);
        }
    }

    if (trans_errors.empty()) {
        result.valid = false;
        result.error_message = "No consecutive node pairs found for RPE evaluation";
        return result;
    }

    result.valid = true;
    result.num_pairs = trans_errors.size();
    result.translation = compute_statistics(trans_errors);
    result.rotation = compute_statistics(rot_errors);

    return result;
}

TrajectoryEvaluation evaluate_trajectory(
    const std::unordered_map<graph::NodeId, geometry::SE2>& estimated,
    const std::unordered_map<graph::NodeId, geometry::SE2>& ground_truth,
    size_t rpe_delta) {

    TrajectoryEvaluation eval;
    eval.ate_raw = compute_ate(estimated, ground_truth, false);
    eval.ate_aligned = compute_ate(estimated, ground_truth, true);
    eval.rpe = compute_rpe(estimated, ground_truth, rpe_delta);

    eval.valid = eval.ate_raw.valid && eval.ate_aligned.valid && eval.rpe.valid;
    if (!eval.valid) {
        std::ostringstream oss;
        if (!eval.ate_raw.valid) oss << "[ATE raw: " << eval.ate_raw.error_message << "] ";
        if (!eval.ate_aligned.valid) oss << "[ATE aligned: " << eval.ate_aligned.error_message << "] ";
        if (!eval.rpe.valid) oss << "[RPE: " << eval.rpe.error_message << "] ";
        eval.error_message = oss.str();
    } else {
        eval.num_poses = eval.ate_raw.num_poses;
    }

    return eval;
}

} // namespace astromesh::optimization
