#include "astromesh/optimization/centralized_optimizer.hpp"
#include "astromesh/graph/disjoint_set.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>

namespace astromesh::optimization {

std::string optimizer_status_to_string(OptimizerStatus status) {
    switch (status) {
        case OptimizerStatus::Success: return "Success";
        case OptimizerStatus::ConvergedCost: return "ConvergedCost";
        case OptimizerStatus::ConvergedStep: return "ConvergedStep";
        case OptimizerStatus::ConvergedGradient: return "ConvergedGradient";
        case OptimizerStatus::MaxIterationsReached: return "MaxIterationsReached";
        case OptimizerStatus::SingularSystem: return "SingularSystem";
        case OptimizerStatus::DampingLimitReached: return "DampingLimitReached";
        case OptimizerStatus::InvalidInput: return "InvalidInput";
        case OptimizerStatus::NoEligibleEdges: return "NoEligibleEdges";
        case OptimizerStatus::MultipleDisconnectedComponents: return "MultipleDisconnectedComponents";
        case OptimizerStatus::NumericalFailure: return "NumericalFailure";
    }
    return "Unknown";
}

std::ostream& operator<<(std::ostream& os, OptimizerStatus status) {
    return os << optimizer_status_to_string(status);
}

namespace {

/**
 * @brief Solves dense linear system A * x = b via Gaussian elimination with partial pivoting.
 *        A is given in row-major order: A[r * n + c].
 * @return true on success, false if matrix is singular or non-finite.
 */
bool solve_linear_system(size_t n, std::vector<double> A, std::vector<double> b, std::vector<double>& x) {
    x.assign(n, 0.0);
    if (n == 0) {
        return true;
    }

    constexpr double PIVOT_EPS = 1e-12;

    for (size_t col = 0; col < n; ++col) {
        // Find best pivot row
        size_t max_row = col;
        double max_val = std::abs(A[col * n + col]);

        for (size_t row = col + 1; row < n; ++row) {
            const double val = std::abs(A[row * n + col]);
            if (val > max_val) {
                max_val = val;
                max_row = row;
            }
        }

        if (!std::isfinite(max_val) || max_val < PIVOT_EPS) {
            return false; // Singular or ill-conditioned system
        }

        // Swap pivot row if needed
        if (max_row != col) {
            for (size_t k = col; k < n; ++k) {
                std::swap(A[col * n + k], A[max_row * n + k]);
            }
            std::swap(b[col], b[max_row]);
        }

        // Eliminate column entries below pivot
        const double pivot = A[col * n + col];
        for (size_t row = col + 1; row < n; ++row) {
            const double factor = A[row * n + col] / pivot;
            if (std::abs(factor) > 0.0) {
                for (size_t k = col + 1; k < n; ++k) {
                    A[row * n + k] -= factor * A[col * n + k];
                }
                b[row] -= factor * b[col];
            }
            A[row * n + col] = 0.0;
        }
    }

    // Back-substitution
    for (int row = static_cast<int>(n) - 1; row >= 0; --row) {
        double sum = b[static_cast<size_t>(row)];
        for (size_t col = static_cast<size_t>(row) + 1; col < n; ++col) {
            sum -= A[static_cast<size_t>(row) * n + col] * x[col];
        }
        const double diag = A[static_cast<size_t>(row) * n + static_cast<size_t>(row)];
        if (std::abs(diag) < PIVOT_EPS) {
            return false;
        }
        x[static_cast<size_t>(row)] = sum / diag;
        if (!std::isfinite(x[static_cast<size_t>(row)])) {
            return false;
        }
    }

    return true;
}

/**
 * @brief Computes 3x3 Jacobians Ji = d(r_ij)/d(xi_i) and Jj = d(r_ij)/d(xi_j) via central differences.
 */
bool compute_edge_jacobians(
    const geometry::SE2& xi,
    const geometry::SE2& xj,
    const geometry::SE2& zij,
    const graph::InformationMatrix2D& omega,
    double Ji[3][3],
    double Jj[3][3],
    double step_h = 1e-7) {

    for (int c = 0; c < 3; ++c) {
        geometry::Twist2D dp(c == 0 ? step_h : 0.0, c == 1 ? step_h : 0.0, c == 2 ? step_h : 0.0);
        geometry::Twist2D dm(c == 0 ? -step_h : 0.0, c == 1 ? -step_h : 0.0, c == 2 ? -step_h : 0.0);

        // Perturb Xi: Xi * exp(+/- step * e_c)
        const geometry::SE2 xi_plus = xi * geometry::SE2::exp(dp);
        const geometry::SE2 xi_minus = xi * geometry::SE2::exp(dm);

        EdgeResidualResult res_i_plus = compute_edge_residual(xi_plus, xj, zij, omega);
        EdgeResidualResult res_i_minus = compute_edge_residual(xi_minus, xj, zij, omega);

        if (!res_i_plus.valid || !res_i_minus.valid) {
            return false;
        }

        Ji[0][c] = (res_i_plus.residual.vx - res_i_minus.residual.vx) / (2.0 * step_h);
        Ji[1][c] = (res_i_plus.residual.vy - res_i_minus.residual.vy) / (2.0 * step_h);
        Ji[2][c] = (res_i_plus.residual.omega - res_i_minus.residual.omega) / (2.0 * step_h);

        // Perturb Xj: Xj * exp(+/- step * e_c)
        const geometry::SE2 xj_plus = xj * geometry::SE2::exp(dp);
        const geometry::SE2 xj_minus = xj * geometry::SE2::exp(dm);

        EdgeResidualResult res_j_plus = compute_edge_residual(xi, xj_plus, zij, omega);
        EdgeResidualResult res_j_minus = compute_edge_residual(xi, xj_minus, zij, omega);

        if (!res_j_plus.valid || !res_j_minus.valid) {
            return false;
        }

        Jj[0][c] = (res_j_plus.residual.vx - res_j_minus.residual.vx) / (2.0 * step_h);
        Jj[1][c] = (res_j_plus.residual.vy - res_j_minus.residual.vy) / (2.0 * step_h);
        Jj[2][c] = (res_j_plus.residual.omega - res_j_minus.residual.omega) / (2.0 * step_h);
    }

    return true;
}

} // namespace

OptimizerResult optimize_pose_graph(
    const graph::PoseGraph& graph,
    const std::unordered_map<graph::NodeId, geometry::SE2>& initial_poses,
    const OptimizerOptions& options) {

    OptimizerResult result;
    result.optimized_poses = initial_poses;

    // Validate initial poses
    if (initial_poses.empty()) {
        result.status = OptimizerStatus::InvalidInput;
        result.message = "Initial poses map is empty";
        return result;
    }

    for (const auto& [nid, pose] : initial_poses) {
        if (!nid.is_valid() || !pose.is_valid()) {
            result.status = OptimizerStatus::InvalidInput;
            result.message = "Initial pose configuration contains invalid or non-finite poses";
            return result;
        }
    }

    // Filter eligible edges
    std::vector<const graph::PoseGraphEdge*> eligible_edges;
    size_t excluded_count = 0;

    for (const auto& [edge_id, edge] : graph.edges()) {
        if (is_edge_eligible_for_objective(edge)) {
            eligible_edges.push_back(&edge);
        } else {
            excluded_count++;
        }
    }

    result.num_eligible_edges = eligible_edges.size();
    result.num_excluded_edges = excluded_count;

    if (eligible_edges.empty()) {
        result.status = OptimizerStatus::NoEligibleEdges;
        result.message = "Pose graph has no eligible edges for optimization";
        return result;
    }

    // Verify all eligible edge endpoints exist in initial_poses and information matrices are valid
    for (const auto* edge : eligible_edges) {
        if (initial_poses.find(edge->from_node) == initial_poses.end() ||
            initial_poses.find(edge->to_node) == initial_poses.end()) {
            result.status = OptimizerStatus::InvalidInput;
            result.message = "Eligible edge references node missing from initial poses";
            return result;
        }
        if (!edge->information.is_valid()) {
            result.status = OptimizerStatus::InvalidInput;
            result.message = "Eligible edge has indefinite or non-finite information matrix";
            return result;
        }
        if (!geometry::SE2(edge->relative_pose).is_valid()) {
            result.status = OptimizerStatus::InvalidInput;
            result.message = "Eligible edge has invalid relative measurement pose";
            return result;
        }
    }

    // Connected component analysis on eligible graph
    graph::DisjointSet dsu;
    for (const auto& [nid, _] : initial_poses) {
        dsu.make_set(nid);
    }
    for (const auto* edge : eligible_edges) {
        dsu.unite(edge->from_node, edge->to_node);
    }

    // Group nodes by component
    std::map<graph::NodeId, std::vector<graph::NodeId>> components;
    for (const auto& [nid, _] : initial_poses) {
        auto root = dsu.find(nid);
        if (root.has_value()) {
            components[*root].push_back(nid);
        }
    }

    if (components.size() > 1 && !options.allow_disconnected) {
        result.status = OptimizerStatus::MultipleDisconnectedComponents;
        std::ostringstream oss;
        oss << "Graph contains " << components.size()
            << " disconnected components, but allow_disconnected is false";
        result.message = oss.str();
        return result;
    }

    // Designate anchors: one anchor per connected component
    std::vector<graph::NodeId> anchor_nodes;
    for (auto& [root, members] : components) {
        std::sort(members.begin(), members.end(), [](const graph::NodeId& a, const graph::NodeId& b) {
            if (a.rover_id != b.rover_id) return a.rover_id < b.rover_id;
            return a.local_index < b.local_index;
        });

        graph::NodeId chosen_anchor = members.front();
        if (options.fixed_anchor.has_value()) {
            if (std::find(members.begin(), members.end(), *options.fixed_anchor) != members.end()) {
                chosen_anchor = *options.fixed_anchor;
            }
        }
        anchor_nodes.push_back(chosen_anchor);
    }

    result.fixed_anchor_nodes = anchor_nodes;

    // Create stable node index mapping
    std::vector<graph::NodeId> all_nodes;
    all_nodes.reserve(initial_poses.size());
    for (const auto& [nid, _] : initial_poses) {
        all_nodes.push_back(nid);
    }
    std::sort(all_nodes.begin(), all_nodes.end(), [](const graph::NodeId& a, const graph::NodeId& b) {
        if (a.rover_id != b.rover_id) return a.rover_id < b.rover_id;
        return a.local_index < b.local_index;
    });

    std::unordered_map<graph::NodeId, size_t> node_to_idx;
    for (size_t i = 0; i < all_nodes.size(); ++i) {
        node_to_idx[all_nodes[i]] = i;
    }

    const size_t num_nodes = all_nodes.size();
    const size_t dim = 3 * num_nodes;

    // Initial objective evaluation
    ObjectiveEvaluationResult init_eval = evaluate_pose_graph_objective(graph, result.optimized_poses);
    if (!init_eval.valid) {
        result.status = OptimizerStatus::NumericalFailure;
        result.message = "Failed to evaluate initial objective: " + init_eval.error_message;
        return result;
    }

    result.initial_cost = init_eval.total_cost;
    result.final_cost = init_eval.total_cost;

    double current_cost = init_eval.total_cost;
    double lambda = std::max(options.initial_lambda, options.min_lambda);

    // Iteration loop
    for (size_t iter = 0; iter < options.max_iterations; ++iter) {
        result.iterations = iter + 1;

        // Allocate Normal Equations: H * Delta_xi = -b
        std::vector<double> H(dim * dim, 0.0);
        std::vector<double> b(dim, 0.0);

        bool jacobian_ok = true;

        for (const auto* edge : eligible_edges) {
            const size_t idx_i = node_to_idx[edge->from_node];
            const size_t idx_j = node_to_idx[edge->to_node];

            const geometry::SE2& xi = result.optimized_poses[edge->from_node];
            const geometry::SE2& xj = result.optimized_poses[edge->to_node];
            const geometry::SE2 zij(edge->relative_pose);

            EdgeResidualResult res = compute_edge_residual(xi, xj, zij, edge->information);
            if (!res.valid) {
                jacobian_ok = false;
                break;
            }

            double Ji[3][3];
            double Jj[3][3];
            if (!compute_edge_jacobians(xi, xj, zij, edge->information, Ji, Jj)) {
                jacobian_ok = false;
                break;
            }

            // Weighted residual: Omega * r
            const auto& om = edge->information;
            const auto& r = res.residual;
            const double om_r[3] = {
                om.xx * r.vx + om.xy * r.vy + om.xtheta * r.omega,
                om.xy * r.vx + om.yy * r.vy + om.ytheta * r.omega,
                om.xtheta * r.vx + om.ytheta * r.vy + om.thetatheta * r.omega
            };

            // Accumulate RHS gradient: b += J^T * Omega * r
            for (int r_idx = 0; r_idx < 3; ++r_idx) {
                double bi_val = 0.0;
                double bj_val = 0.0;
                for (int c_idx = 0; c_idx < 3; ++c_idx) {
                    bi_val += Ji[c_idx][r_idx] * om_r[c_idx];
                    bj_val += Jj[c_idx][r_idx] * om_r[c_idx];
                }
                b[3 * idx_i + static_cast<size_t>(r_idx)] += bi_val;
                b[3 * idx_j + static_cast<size_t>(r_idx)] += bj_val;
            }

            // Compute Wi = Omega * Ji, Wj = Omega * Jj (3x3 each)
            double Wi[3][3];
            double Wj[3][3];
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    const double om_row[3] = {
                        row == 0 ? om.xx : (row == 1 ? om.xy : om.xtheta),
                        row == 0 ? om.xy : (row == 1 ? om.yy : om.ytheta),
                        row == 0 ? om.xtheta : (row == 1 ? om.ytheta : om.thetatheta)
                    };
                    Wi[row][col] = om_row[0] * Ji[0][col] + om_row[1] * Ji[1][col] + om_row[2] * Ji[2][col];
                    Wj[row][col] = om_row[0] * Jj[0][col] + om_row[1] * Jj[1][col] + om_row[2] * Jj[2][col];
                }
            }

            // Accumulate blocks H_ii, H_ij, H_ji, H_jj
            for (int r_idx = 0; r_idx < 3; ++r_idx) {
                for (int c_idx = 0; c_idx < 3; ++c_idx) {
                    double hii = 0.0;
                    double hij = 0.0;
                    double hji = 0.0;
                    double hjj = 0.0;
                    for (int k = 0; k < 3; ++k) {
                        hii += Ji[k][r_idx] * Wi[k][c_idx];
                        hij += Ji[k][r_idx] * Wj[k][c_idx];
                        hji += Jj[k][r_idx] * Wi[k][c_idx];
                        hjj += Jj[k][r_idx] * Wj[k][c_idx];
                    }
                    H[(3 * idx_i + static_cast<size_t>(r_idx)) * dim + (3 * idx_i + static_cast<size_t>(c_idx))] += hii;
                    H[(3 * idx_i + static_cast<size_t>(r_idx)) * dim + (3 * idx_j + static_cast<size_t>(c_idx))] += hij;
                    H[(3 * idx_j + static_cast<size_t>(r_idx)) * dim + (3 * idx_i + static_cast<size_t>(c_idx))] += hji;
                    H[(3 * idx_j + static_cast<size_t>(r_idx)) * dim + (3 * idx_j + static_cast<size_t>(c_idx))] += hjj;
                }
            }
        }

        if (!jacobian_ok) {
            result.status = OptimizerStatus::NumericalFailure;
            result.message = "Jacobian calculation failed during normal equation assembly";
            return result;
        }

        // Gradient convergence check
        double max_grad = 0.0;
        for (double g_val : b) {
            max_grad = std::max(max_grad, std::abs(g_val));
        }
        result.final_gradient_norm = max_grad;

        if (max_grad < options.gradient_tolerance) {
            result.status = OptimizerStatus::ConvergedGradient;
            result.final_cost = current_cost;
            result.message = "Converged: gradient infinity norm below tolerance";
            return result;
        }

        // Levenberg-Marquardt inner trial loop
        bool step_accepted = false;

        while (!step_accepted) {
            std::vector<double> A = H;
            std::vector<double> rhs = b;

            // Negate rhs for solving A * Delta_xi = -b
            for (double& val : rhs) {
                val = -val;
            }

            // Add Levenberg-Marquardt damping
            for (size_t d = 0; d < dim; ++d) {
                A[d * dim + d] += lambda;
            }

            // Gauge Anchoring: enforce Delta_xi_anchor == 0
            for (const auto& anchor_nid : anchor_nodes) {
                const size_t a_idx = node_to_idx[anchor_nid];
                for (size_t c = 0; c < 3; ++c) {
                    const size_t row = 3 * a_idx + c;
                    for (size_t col = 0; col < dim; ++col) {
                        A[row * dim + col] = 0.0;
                        A[col * dim + row] = 0.0;
                    }
                    A[row * dim + row] = 1.0;
                    rhs[row] = 0.0;
                }
            }

            // Solve linear system
            std::vector<double> delta_xi(dim, 0.0);
            if (!solve_linear_system(dim, A, rhs, delta_xi)) {
                result.status = OptimizerStatus::SingularSystem;
                result.message = "Linear system is singular or ill-conditioned";
                return result;
            }

            // Step norm check
            double max_step = 0.0;
            for (double s_val : delta_xi) {
                max_step = std::max(max_step, std::abs(s_val));
            }
            result.final_step_norm = max_step;

            // Retract candidate poses
            std::unordered_map<graph::NodeId, geometry::SE2> candidate_poses = result.optimized_poses;
            for (size_t i = 0; i < num_nodes; ++i) {
                const auto& nid = all_nodes[i];
                geometry::Twist2D step_twist(delta_xi[3 * i], delta_xi[3 * i + 1], delta_xi[3 * i + 2]);
                candidate_poses[nid] = result.optimized_poses[nid] * geometry::SE2::exp(step_twist);
            }

            // Evaluate candidate cost
            ObjectiveEvaluationResult cand_eval = evaluate_pose_graph_objective(graph, candidate_poses);

            if (cand_eval.valid && cand_eval.total_cost <= current_cost + 1e-12) {
                // Step accepted
                step_accepted = true;
                result.successful_steps++;

                const double cost_reduction = current_cost - cand_eval.total_cost;
                const double rel_reduction = cost_reduction / std::max(1.0, current_cost);

                current_cost = cand_eval.total_cost;
                result.optimized_poses = candidate_poses;
                result.final_cost = current_cost;

                // Decrease damping
                lambda = std::max(lambda / options.lambda_factor, options.min_lambda);

                // Convergence check on step or cost
                if (max_step < options.step_tolerance) {
                    result.status = OptimizerStatus::ConvergedStep;
                    result.message = "Converged: step norm below tolerance";
                    return result;
                }
                if (rel_reduction < options.cost_tolerance) {
                    result.status = OptimizerStatus::ConvergedCost;
                    result.message = "Converged: relative cost reduction below tolerance";
                    return result;
                }
            } else {
                // Step rejected (cost increased or invalid)
                result.rejected_steps++;
                lambda *= options.lambda_factor;

                if (lambda > options.max_lambda) {
                    result.status = OptimizerStatus::DampingLimitReached;
                    result.final_cost = current_cost;
                    result.message = "Optimizer reached maximum damping without progress";
                    return result;
                }
            }
        }
    }

    result.status = OptimizerStatus::MaxIterationsReached;
    result.final_cost = current_cost;
    result.message = "Reached maximum iterations without meeting convergence criteria";
    return result;
}

OptimizerResult optimize_pose_graph(
    const graph::PoseGraph& graph,
    const OptimizerOptions& options) {

    std::unordered_map<graph::NodeId, geometry::SE2> poses;
    for (const auto& [nid, node] : graph.nodes()) {
        poses[nid] = geometry::SE2(node.pose);
    }
    return optimize_pose_graph(graph, poses, options);
}

} // namespace astromesh::optimization
