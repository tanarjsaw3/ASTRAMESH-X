#pragma once

#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/optimization/pose_graph_objective.hpp"

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace astromesh::optimization {

/**
 * @brief Discrete termination status for the CORD SE(2) continuous dynamics Lie integrator.
 */
enum class CordStatus {
    Success,              ///< Converged: gradient norm and kinetic energy below tolerances.
    MaxStepsReached,      ///< Reached maximum integration steps without meeting convergence criteria.
    InvalidInput,         ///< Invalid initial poses, measurements, non-positive mass, or malformed parameters.
    NoEligibleEdges,      ///< No eligible constraints found in the pose graph.
    MultipleDisconnectedComponents, ///< Multiple disconnected components present when allow_disconnected is false.
    NumericalFailure      ///< Encountered non-finite state, cost, or numerical breakdown during integration.
};

std::string cord_status_to_string(CordStatus status);
std::ostream& operator<<(std::ostream& os, CordStatus status);

/**
 * @brief Positive-definite diagonal mass/inertia tensor M_i on se(2):
 *        M_i = diag(mass_x, mass_y, inertia_theta)
 */
struct NodeInertia {
    double mass_x{1.0};       ///< Linear virtual mass along body x-axis (kg).
    double mass_y{1.0};       ///< Linear virtual mass along body y-axis (kg).
    double inertia_theta{1.0};///< Rotational virtual inertia around yaw axis (kg*m^2).

    constexpr NodeInertia() noexcept = default;
    constexpr NodeInertia(double mx, double my, double itheta) noexcept
        : mass_x(mx), mass_y(my), inertia_theta(itheta) {}

    bool is_valid() const noexcept {
        return std::isfinite(mass_x) && mass_x > 0.0 &&
               std::isfinite(mass_y) && mass_y > 0.0 &&
               std::isfinite(inertia_theta) && inertia_theta > 0.0;
    }
};

/**
 * @brief Positive-semidefinite diagonal Rayleigh damping tensor D_i on se(2):
 *        D_i = diag(damping_x, damping_y, damping_theta)
 */
struct NodeDamping {
    double damping_x{2.0};       ///< Linear viscous damping coefficient along body x-axis.
    double damping_y{2.0};       ///< Linear viscous damping coefficient along body y-axis.
    double damping_theta{2.0};   ///< Rotational viscous damping coefficient around yaw axis.

    constexpr NodeDamping() noexcept = default;
    constexpr NodeDamping(double dx, double dy, double dtheta) noexcept
        : damping_x(dx), damping_y(dy), damping_theta(dtheta) {}

    bool is_valid() const noexcept {
        return std::isfinite(damping_x) && damping_x >= 0.0 &&
               std::isfinite(damping_y) && damping_y >= 0.0 &&
               std::isfinite(damping_theta) && damping_theta >= 0.0;
    }
};

/**
 * @brief Complete dynamic state of a pose node (configuration X_i and body twist xi_i).
 */
struct NodeDynamicsState {
    geometry::SE2 pose;          ///< Configuration pose X_i in SE(2).
    geometry::Twist2D velocity;  ///< Body-fixed twist xi_i = [vx, vy, omega]^T in se(2).

    NodeDynamicsState() = default;
    NodeDynamicsState(const geometry::SE2& p, const geometry::Twist2D& v = geometry::Twist2D())
        : pose(p), velocity(v) {}

    bool is_valid() const noexcept {
        return pose.is_valid() && velocity.is_valid();
    }
};

/**
 * @brief Configuration parameters for the CORD geometric Lie integrator.
 */
struct CordIntegratorOptions {
    double timestep{0.01};              ///< Integration step size h in seconds (h > 0).
    size_t max_steps{1000};             ///< Maximum number of discrete integration steps.
    double gradient_tolerance{1e-4};    ///< Gradient infinity norm threshold for stationarity.
    double kinetic_energy_tolerance{1e-6}; ///< Kinetic energy threshold for stationarity.

    NodeInertia default_inertia{1.0, 1.0, 1.0}; ///< Default diagonal inertia tensor.
    NodeDamping default_damping{2.0, 2.0, 2.0}; ///< Default diagonal damping tensor.

    bool allow_disconnected{true};      ///< If true, anchors the minimum NodeId in each disconnected component.
    std::optional<graph::NodeId> fixed_anchor{std::nullopt}; ///< Optional user-specified anchor node.
    bool verbose{false};                ///< Debug logging flag.
};

/**
 * @brief Structured outcome of CORD continuous dynamics integration.
 */
struct CordIntegratorResult {
    CordStatus status{CordStatus::InvalidInput};
    size_t steps{0};
    double initial_cost{0.0};
    double final_cost{0.0};
    double initial_kinetic_energy{0.0};
    double final_kinetic_energy{0.0};
    double initial_hamiltonian{0.0};
    double final_hamiltonian{0.0};
    double final_gradient_norm{0.0};
    double final_velocity_norm{0.0};

    std::unordered_map<graph::NodeId, NodeDynamicsState> final_states;
    std::vector<graph::NodeId> fixed_anchor_nodes;
    size_t num_eligible_edges{0};
    std::string message;

    bool is_converged() const noexcept {
        return status == CordStatus::Success;
    }
};

/**
 * @brief Computes total kinetic energy K(xi) = 0.5 * sum_i xi_i^T * M_i * xi_i across all nodes.
 */
double compute_kinetic_energy(
    const std::unordered_map<graph::NodeId, geometry::Twist2D>& velocities,
    const NodeInertia& inertia) noexcept;

/**
 * @brief Computes total Hamiltonian energy H(X, xi) = C(X) + K(xi).
 */
inline double compute_hamiltonian(double potential_cost, double kinetic_energy) noexcept {
    return potential_cost + kinetic_energy;
}

/**
 * @brief Advances the CORD dynamics state by one discrete semi-implicit Lie group timestep h.
 *
 * Update scheme (Tian et al., CORD):
 *   1. Velocity Step (linearized semi-implicit with respect to damping):
 *      xi_i^{k+1} = Lambda_i^{-1} * [ xi_i^k + h * M_i^{-1} * ( ad_{xi_i^k}^*(M_i * xi_i^k) - grad_{X_i} C(X^k) ) ]
 *      where Lambda_i = I + h * M_i^{-1} * D_i is strictly diagonal.
 *   2. Position Retraction Step:
 *      X_i^{k+1} = X_i^k * exp( (h * xi_i^{k+1})^\wedge )
 *
 * Anchor nodes have velocity clamped to zero and pose held strictly fixed:
 *   xi_anchor^{k+1} = [0, 0, 0]^T, X_anchor^{k+1} = X_anchor^k.
 *
 * @param graph Topological pose graph containing constraints.
 * @param current_states Current configurations and body twists.
 * @param anchor_nodes Nodes held static as gauge anchors.
 * @param h Integration timestep in seconds.
 * @param inertia Diagonal mass/inertia tensor.
 * @param damping Diagonal viscous damping tensor.
 * @param next_states Output updated states.
 * @param out_grad_norm Optional output for maximum gradient infinity norm.
 * @return true on valid update, false on non-finite failure.
 */
bool step_cord_integrator(
    const graph::PoseGraph& graph,
    const std::unordered_map<graph::NodeId, NodeDynamicsState>& current_states,
    const std::vector<graph::NodeId>& anchor_nodes,
    double h,
    const NodeInertia& inertia,
    const NodeDamping& damping,
    std::unordered_map<graph::NodeId, NodeDynamicsState>& next_states,
    double* out_grad_norm = nullptr);

/**
 * @brief Integrates CORD continuous dynamics on SE(2) from an initial state until convergence or max_steps.
 *
 * @param graph Input topological pose graph (remains strictly const/unmodified).
 * @param initial_states Initial pose and velocity state per node.
 * @param options Timestep, damping, inertia, and termination criteria.
 * @return CordIntegratorResult Trajectory convergence diagnostics and final states.
 */
CordIntegratorResult integrate_cord(
    const graph::PoseGraph& graph,
    const std::unordered_map<graph::NodeId, NodeDynamicsState>& initial_states,
    const CordIntegratorOptions& options = {});

/**
 * @brief Convenience overload initializing velocities to zero from initial poses.
 */
CordIntegratorResult integrate_cord(
    const graph::PoseGraph& graph,
    const std::unordered_map<graph::NodeId, geometry::SE2>& initial_poses,
    const CordIntegratorOptions& options = {});

} // namespace astromesh::optimization
