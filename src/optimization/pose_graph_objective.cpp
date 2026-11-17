#include "astromesh/optimization/pose_graph_objective.hpp"

#include <cmath>
#include <sstream>

namespace astromesh::optimization {

EdgeResidualResult compute_edge_residual(
    const geometry::SE2& xi,
    const geometry::SE2& xj,
    const geometry::SE2& zij,
    const graph::InformationMatrix2D& omega) noexcept {

    EdgeResidualResult result;

    if (!xi.is_valid()) {
        result.valid = false;
        result.error_message = "Source pose Xi is invalid or non-finite";
        return result;
    }
    if (!xj.is_valid()) {
        result.valid = false;
        result.error_message = "Target pose Xj is invalid or non-finite";
        return result;
    }
    if (!zij.is_valid()) {
        result.valid = false;
        result.error_message = "Measurement pose Zij is invalid or non-finite";
        return result;
    }
    if (!omega.is_valid()) {
        result.valid = false;
        result.error_message = "Information matrix Omega is indefinite or non-finite";
        return result;
    }

    // Predicted relative pose of frame j in frame i: X_hat = X_i^{-1} * X_j
    const geometry::SE2 x_pred = xi.inverse() * xj;

    // Multiplicative error in measurement frame: E_ij = Z_ij^{-1} * X_hat = Z_ij^{-1} * X_i^{-1} * X_j
    const geometry::SE2 error_elem = zij.inverse() * x_pred;

    // Tangent-space geodesic residual: r_ij = Log(E_ij)^\vee
    const geometry::Twist2D r = error_elem.log();
    if (!r.is_valid()) {
        result.valid = false;
        result.error_message = "Failed to evaluate logarithm map of relative error";
        return result;
    }

    // Weighted least-squares cost: 0.5 * r^T * Omega * r
    const double v1 = omega.xx * r.vx + omega.xy * r.vy + omega.xtheta * r.omega;
    const double v2 = omega.xy * r.vx + omega.yy * r.vy + omega.ytheta * r.omega;
    const double v3 = omega.xtheta * r.vx + omega.ytheta * r.vy + omega.thetatheta * r.omega;

    const double quadratic_form = r.vx * v1 + r.vy * v2 + r.omega * v3;
    if (!std::isfinite(quadratic_form) || quadratic_form < -1e-9) {
        result.valid = false;
        result.error_message = "Residual quadratic form evaluated to non-finite or negative value";
        return result;
    }

    result.valid = true;
    result.residual = r;
    result.cost = 0.5 * std::max(0.0, quadratic_form);
    return result;
}

bool is_edge_eligible_for_objective(const graph::PoseGraphEdge& edge) noexcept {
    if (edge.type == graph::EdgeType::Odometry) {
        // Odometry is eligible unless explicitly rejected as an outlier
        return edge.status != graph::EdgeVerificationStatus::Rejected;
    }
    if (edge.type == graph::EdgeType::IntraRoverLoopClosure ||
        edge.type == graph::EdgeType::InterRoverLoopClosure) {
        // Loop closures MUST be geometrically verified to be admitted
        return edge.status == graph::EdgeVerificationStatus::Verified;
    }
    return false;
}

ObjectiveEvaluationResult evaluate_pose_graph_objective(
    const graph::PoseGraph& graph,
    const std::unordered_map<graph::NodeId, geometry::SE2>& poses) {

    ObjectiveEvaluationResult result;
    result.valid = true;

    const auto& edges = graph.edges();
    for (const auto& [edge_id, edge] : edges) {
        if (!is_edge_eligible_for_objective(edge)) {
            if (edge.status == graph::EdgeVerificationStatus::Rejected) {
                result.num_skipped_rejected++;
            } else {
                result.num_skipped_unverified++;
            }
            continue;
        }

        // Endpoint existence checks
        auto it_from = poses.find(edge.from_node);
        if (it_from == poses.end()) {
            result.valid = false;
            std::ostringstream oss;
            oss << "Eligible edge " << edge.id << " references missing from_node " << edge.from_node;
            result.error_message = oss.str();
            return result;
        }

        auto it_to = poses.find(edge.to_node);
        if (it_to == poses.end()) {
            result.valid = false;
            std::ostringstream oss;
            oss << "Eligible edge " << edge.id << " references missing to_node " << edge.to_node;
            result.error_message = oss.str();
            return result;
        }

        const geometry::SE2& xi = it_from->second;
        const geometry::SE2& xj = it_to->second;
        const geometry::SE2 zij(edge.relative_pose);

        EdgeResidualResult edge_eval = compute_edge_residual(xi, xj, zij, edge.information);
        if (!edge_eval.valid) {
            result.valid = false;
            std::ostringstream oss;
            oss << "Evaluation of eligible edge " << edge.id << " failed: " << edge_eval.error_message;
            result.error_message = oss.str();
            return result;
        }

        result.total_cost += edge_eval.cost;
        result.num_eligible_edges++;
        if (edge.type == graph::EdgeType::Odometry) {
            result.num_odometry_edges++;
        } else {
            result.num_verified_loop_closures++;
        }
        result.evaluated_edge_ids.push_back(edge.id);
    }

    return result;
}

ObjectiveEvaluationResult evaluate_pose_graph_objective(
    const graph::PoseGraph& graph) {

    std::unordered_map<graph::NodeId, geometry::SE2> poses;
    for (const auto& [nid, node] : graph.nodes()) {
        poses[nid] = geometry::SE2(node.pose);
    }
    return evaluate_pose_graph_objective(graph, poses);
}

GradientEvaluationResult evaluate_objective_numerical_gradient(
    const graph::PoseGraph& graph,
    const std::unordered_map<graph::NodeId, geometry::SE2>& poses,
    double step_size) {

    GradientEvaluationResult grad_result;
    if (!std::isfinite(step_size) || step_size <= 0.0) {
        grad_result.valid = false;
        grad_result.error_message = "Step size for numerical gradient must be positive and finite";
        return grad_result;
    }

    // Verify baseline objective evaluation
    ObjectiveEvaluationResult base_eval = evaluate_pose_graph_objective(graph, poses);
    if (!base_eval.valid) {
        grad_result.valid = false;
        grad_result.error_message = "Baseline objective evaluation failed: " + base_eval.error_message;
        return grad_result;
    }

    grad_result.valid = true;
    std::unordered_map<graph::NodeId, geometry::SE2> perturbed_poses = poses;

    // Evaluate central finite difference for each node
    for (const auto& [nid, node_pose] : poses) {
        double g[3] = {0.0, 0.0, 0.0};

        for (int k = 0; k < 3; ++k) {
            geometry::Twist2D twist_plus(
                k == 0 ? step_size : 0.0,
                k == 1 ? step_size : 0.0,
                k == 2 ? step_size : 0.0
            );
            geometry::Twist2D twist_minus(
                k == 0 ? -step_size : 0.0,
                k == 1 ? -step_size : 0.0,
                k == 2 ? -step_size : 0.0
            );

            // Right perturbation on SE(2): X * exp(+/- step * e_k^wedge)
            perturbed_poses[nid] = node_pose * geometry::SE2::exp(twist_plus);
            ObjectiveEvaluationResult eval_plus = evaluate_pose_graph_objective(graph, perturbed_poses);
            if (!eval_plus.valid) {
                grad_result.valid = false;
                grad_result.error_message = "Perturbed (+k) objective evaluation failed: " + eval_plus.error_message;
                return grad_result;
            }

            perturbed_poses[nid] = node_pose * geometry::SE2::exp(twist_minus);
            ObjectiveEvaluationResult eval_minus = evaluate_pose_graph_objective(graph, perturbed_poses);
            if (!eval_minus.valid) {
                grad_result.valid = false;
                grad_result.error_message = "Perturbed (-k) objective evaluation failed: " + eval_minus.error_message;
                return grad_result;
            }

            // Reset node pose in perturbed map
            perturbed_poses[nid] = node_pose;

            // Central difference derivative
            g[k] = (eval_plus.total_cost - eval_minus.total_cost) / (2.0 * step_size);
        }

        grad_result.gradients[nid] = geometry::Twist2D(g[0], g[1], g[2]);
    }

    return grad_result;
}

} // namespace astromesh::optimization
