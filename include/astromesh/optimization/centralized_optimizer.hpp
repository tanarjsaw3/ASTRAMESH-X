#pragma once

#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/optimization/pose_graph_objective.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>
#include <optional>
#include <iosfwd>

namespace astromesh::optimization {

/**
 * @brief Discrete termination status for the centralized SE(2) pose-graph optimizer.
 */
enum class OptimizerStatus {
    Success,                        ///< Successfully converged to an optimal configuration.
    ConvergedCost,                  ///< Terminated: relative cost change below cost_tolerance.
    ConvergedStep,                  ///< Terminated: maximum Lie algebra step norm below step_tolerance.
    ConvergedGradient,              ///< Terminated: maximum gradient infinity norm below gradient_tolerance.
    MaxIterationsReached,           ///< Failed to converge within max_iterations.
    SingularSystem,                 ///< Linear system is singular or poorly conditioned.
    DampingLimitReached,            ///< Damping parameter exceeded max_lambda.
    InvalidInput,                   ///< Invalid initial poses, measurements, or malformed constraints.
    NoEligibleEdges,                ///< No eligible constraints found in the graph.
    MultipleDisconnectedComponents, ///< Multiple disconnected components present when allow_disconnected is false.
    NumericalFailure                ///< Non-finite cost, residual, or numerical breakdown during iterations.
};

std::string optimizer_status_to_string(OptimizerStatus status);
std::ostream& operator<<(std::ostream& os, OptimizerStatus status);

/**
 * @brief Algorithmic options and numerical safeguards for Levenberg-Marquardt optimization.
 */
struct OptimizerOptions {
    size_t max_iterations{100};       ///< Maximum allowable optimization iterations.
    double cost_tolerance{1e-6};      ///< Relative cost reduction threshold for convergence.
    double step_tolerance{1e-6};      ///< Max Lie algebra step norm ||Delta xi||_inf threshold.
    double gradient_tolerance{1e-6};  ///< Max gradient infinity norm ||g||_inf threshold.

    double initial_lambda{1e-3};      ///< Initial Levenberg-Marquardt damping parameter.
    double lambda_factor{10.0};       ///< Multiplicative damping scaling factor upon rejection.
    double min_lambda{1e-7};          ///< Minimum allowable damping floor.
    double max_lambda{1e7};           ///< Maximum allowable damping ceiling before aborting.

    bool allow_disconnected{true};    ///< If true, anchors the minimum NodeId in each disconnected component.
    std::optional<graph::NodeId> fixed_anchor{std::nullopt}; ///< Optional user-specified anchor node.
    bool verbose{false};              ///< Debug logging flag.
};

/**
 * @brief Comprehensive optimization outcome and diagnostic result.
 */
struct OptimizerResult {
    OptimizerStatus status{OptimizerStatus::InvalidInput};
    size_t iterations{0};
    size_t successful_steps{0};
    size_t rejected_steps{0};
    double initial_cost{0.0};
    double final_cost{0.0};
    double final_gradient_norm{0.0};
    double final_step_norm{0.0};

    std::unordered_map<graph::NodeId, geometry::SE2> optimized_poses;
    std::vector<graph::NodeId> fixed_anchor_nodes;

    size_t num_eligible_edges{0};
    size_t num_excluded_edges{0};
    std::string message;

    bool is_converged() const noexcept {
        return status == OptimizerStatus::Success ||
               status == OptimizerStatus::ConvergedCost ||
               status == OptimizerStatus::ConvergedStep ||
               status == OptimizerStatus::ConvergedGradient;
    }
};

/**
 * @brief Optimizes poses on SE(2) via centralized Levenberg-Marquardt pose-graph optimization.
 *
 * Mathematical formulation:
 *   Minimizes:
 *     C(X) = 0.5 * sum_{(i,j) in E_eligible} r_ij^T * Omega_ij * r_ij
 *   where:
 *     r_ij = Log(Z_ij^{-1} * X_i^{-1} * X_j)^\vee in R^3
 *
 * Retraction and update rule:
 *   Right-trivialized Lie algebra update:
 *     X_k^{(t+1)} = X_k^{(t)} * exp(Delta xi_k^\wedge)
 *
 * Normal equations:
 *     (H + lambda * I) * Delta xi = -b
 *   with exact gauge fixing applied to designated anchor nodes (Delta xi_anchor == 0).
 *
 * Constraint eligibility policy:
 *   - Odometry: Eligible unless status == Rejected.
 *   - Loop closures: Eligible IF AND ONLY IF status == Verified.
 *   - Unverified and Rejected loop closures NEVER enter the objective or Hessian.
 *
 * @param graph Input topological pose graph (remains strictly const/unmodified).
 * @param initial_poses Initial pose estimates for all nodes.
 * @param options Optimization parameters and safeguards.
 * @return OptimizerResult Detailed convergence metrics, status, and optimized pose configuration.
 */
OptimizerResult optimize_pose_graph(
    const graph::PoseGraph& graph,
    const std::unordered_map<graph::NodeId, geometry::SE2>& initial_poses,
    const OptimizerOptions& options = {});

/**
 * @brief Convenience overload extracting initial poses directly from graph nodes.
 */
OptimizerResult optimize_pose_graph(
    const graph::PoseGraph& graph,
    const OptimizerOptions& options = {});

} // namespace astromesh::optimization
