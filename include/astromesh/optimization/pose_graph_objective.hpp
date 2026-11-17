#pragma once

#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"

#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>

namespace astromesh::optimization {

/**
 * @brief Structured outcome of single-edge residual and cost evaluation.
 */
struct EdgeResidualResult {
    bool valid{false};
    geometry::Twist2D residual; ///< Geodesic residual r_ij = Log(Z_ij^{-1} * X_i^{-1} * X_j)^\vee.
    double cost{0.0};           ///< Weighted least-squares cost: 0.5 * r^T * Omega * r.
    std::string error_message;
};

/**
 * @brief Evaluates the tangent-space geodesic residual and weighted least-squares cost for a single constraint.
 *
 * @param xi Estimated pose of source node i (X_i \in SE(2)).
 * @param xj Estimated pose of target node j (X_j \in SE(2)).
 * @param zij Measured relative transform from i to j (Z_ij \in SE(2)).
 * @param omega Symmetric positive-definite information matrix (Omega_ij \in R^{3x3}).
 * @return EdgeResidualResult Valid result if all inputs are finite and Omega is positive-definite,
 *                            or valid=false with descriptive error message.
 */
EdgeResidualResult compute_edge_residual(
    const geometry::SE2& xi,
    const geometry::SE2& xj,
    const geometry::SE2& zij,
    const graph::InformationMatrix2D& omega) noexcept;

/**
 * @brief Evaluates edge eligibility according to AstroMesh-X graph policy:
 *        - Odometry: Eligible unless status == Rejected.
 *        - Intra-rover & Inter-rover loop closures: Eligible IF AND ONLY IF status == Verified.
 *        - Unverified candidates and Rejected outliers are NEVER eligible.
 */
bool is_edge_eligible_for_objective(const graph::PoseGraphEdge& edge) noexcept;

/**
 * @brief Structured outcome of full pose-graph objective cost evaluation.
 */
struct ObjectiveEvaluationResult {
    bool valid{false};
    double total_cost{0.0};
    size_t num_eligible_edges{0};
    size_t num_odometry_edges{0};
    size_t num_verified_loop_closures{0};
    size_t num_skipped_unverified{0};
    size_t num_skipped_rejected{0};
    std::vector<uint64_t> evaluated_edge_ids;
    std::string error_message;
};

/**
 * @brief Evaluates the total pose-graph objective cost over all eligible edges given explicit pose estimates:
 *        C(X) = 0.5 * sum_{(i,j) \in E_eligible} r_ij^T * Omega_ij * r_ij.
 *
 * Rejects evaluation with valid=false if:
 *   - Any eligible edge references a node missing in `poses`.
 *   - Any pose or measurement contains non-finite values.
 *   - Any information matrix is indefinite or non-finite.
 */
ObjectiveEvaluationResult evaluate_pose_graph_objective(
    const graph::PoseGraph& graph,
    const std::unordered_map<graph::NodeId, geometry::SE2>& poses);

/**
 * @brief Overload evaluating objective cost using poses currently stored in the PoseGraph nodes.
 */
ObjectiveEvaluationResult evaluate_pose_graph_objective(
    const graph::PoseGraph& graph);

/**
 * @brief Structured outcome of numerical Riemannian gradient evaluation.
 */
struct GradientEvaluationResult {
    bool valid{false};
    std::unordered_map<graph::NodeId, geometry::Twist2D> gradients; ///< Body-frame Riemannian gradient per node.
    std::string error_message;
};

/**
 * @brief Reference numerical Riemannian gradient evaluation using central finite differences.
 *
 * NOTE: This is explicitly a reference implementation intended for verification and testing,
 *       NOT the final production optimizer interface.
 *
 * Perturbation convention:
 *   Right-trivialized (body-frame) Lie group perturbation on SE(2):
 *     X_i^{(+k)} = X_i * exp(+step_size * e_k^wedge)
 *     X_i^{(-k)} = X_i * exp(-step_size * e_k^wedge)
 *   Gradient component:
 *     [grad_{X_i} C]_k = (C(X_i^{(+k)}) - C(X_i^{(-k)})) / (2 * step_size)
 *
 * @param graph Topological pose graph containing constraints.
 * @param poses Current estimate configurations for all nodes.
 * @param step_size Perturbation step epsilon (default: 1e-6).
 * @return GradientEvaluationResult Mapping each NodeId to [g_vx, g_vy, g_omega]^T.
 */
GradientEvaluationResult evaluate_objective_numerical_gradient(
    const graph::PoseGraph& graph,
    const std::unordered_map<graph::NodeId, geometry::SE2>& poses,
    double step_size = 1e-6);

} // namespace astromesh::optimization
