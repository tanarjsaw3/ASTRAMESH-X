#include "astromesh/optimization/cord_integrator.hpp"
#include "astromesh/graph/disjoint_set.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <sstream>

namespace astromesh::optimization {

std::string cord_status_to_string(CordStatus status) {
    switch (status) {
        case CordStatus::Success: return "Success";
        case CordStatus::MaxStepsReached: return "MaxStepsReached";
        case CordStatus::InvalidInput: return "InvalidInput";
        case CordStatus::NoEligibleEdges: return "NoEligibleEdges";
        case CordStatus::MultipleDisconnectedComponents: return "MultipleDisconnectedComponents";
        case CordStatus::NumericalFailure: return "NumericalFailure";
    }
    return "Unknown";
}

std::ostream& operator<<(std::ostream& os, CordStatus status) {
    return os << cord_status_to_string(status);
}

double compute_kinetic_energy(
    const std::unordered_map<graph::NodeId, geometry::Twist2D>& velocities,
    const NodeInertia& inertia) noexcept {

    double total_ke = 0.0;
    for (const auto& [_, vel] : velocities) {
        total_ke += 0.5 * (inertia.mass_x * vel.vx * vel.vx +
                           inertia.mass_y * vel.vy * vel.vy +
                           inertia.inertia_theta * vel.omega * vel.omega);
    }
    return total_ke;
}

bool step_cord_integrator(
    const graph::PoseGraph& graph,
    const std::unordered_map<graph::NodeId, NodeDynamicsState>& current_states,
    const std::vector<graph::NodeId>& anchor_nodes,
    double h,
    const NodeInertia& inertia,
    const NodeDamping& damping,
    std::unordered_map<graph::NodeId, NodeDynamicsState>& next_states,
    double* out_grad_norm) {

    if (!std::isfinite(h) || h <= 0.0 || !inertia.is_valid() || !damping.is_valid()) {
        return false;
    }

    // Extract current poses map for objective and gradient evaluation
    std::unordered_map<graph::NodeId, geometry::SE2> current_poses;
    current_poses.reserve(current_states.size());
    for (const auto& [nid, state] : current_states) {
        if (!state.is_valid()) {
            return false;
        }
        current_poses[nid] = state.pose;
    }

    // Evaluate Riemannian gradient of the objective: grad_{X_i} C(X)
    GradientEvaluationResult grad_eval = evaluate_objective_numerical_gradient(graph, current_poses);
    if (!grad_eval.valid) {
        return false;
    }

    double max_g_norm = 0.0;
    for (const auto& [nid, g] : grad_eval.gradients) {
        max_g_norm = std::max(max_g_norm, std::abs(g.vx));
        max_g_norm = std::max(max_g_norm, std::abs(g.vy));
        max_g_norm = std::max(max_g_norm, std::abs(g.omega));
    }
    if (out_grad_norm != nullptr) {
        *out_grad_norm = max_g_norm;
    }

    next_states = current_states;

    for (auto& [nid, state] : next_states) {
        // Check if node is a static gauge anchor
        const bool is_anchor = (std::find(anchor_nodes.begin(), anchor_nodes.end(), nid) != anchor_nodes.end());

        if (is_anchor) {
            // Anchor node: clamp velocity to 0 and keep pose static
            state.velocity = geometry::Twist2D(0.0, 0.0, 0.0);
            continue;
        }

        const auto& xi = current_states.at(nid).velocity;
        auto it_g = grad_eval.gradients.find(nid);
        const geometry::Twist2D g = (it_g != grad_eval.gradients.end()) ? it_g->second : geometry::Twist2D();

        // 1. Momentum: p = M * xi
        const geometry::Twist2D momentum(
            inertia.mass_x * xi.vx,
            inertia.mass_y * xi.vy,
            inertia.inertia_theta * xi.omega
        );

        // 2. Co-adjoint action: ad_{xi}^*(p)
        const geometry::Twist2D coadj = geometry::SE2::coadjoint(xi, momentum);

        // 3. Conservative and gyroscopic forces: F = ad_{xi}^*(M * xi) - grad C(X)
        const double f_x = coadj.vx - g.vx;
        const double f_y = coadj.vy - g.vy;
        const double f_theta = coadj.omega - g.omega;

        // 4. Semi-implicit velocity update:
        //    (I + h * M^{-1} * D) * xi^{k+1} = xi^k + h * M^{-1} * F
        const double denom_x = 1.0 + h * (damping.damping_x / inertia.mass_x);
        const double denom_y = 1.0 + h * (damping.damping_y / inertia.mass_y);
        const double denom_theta = 1.0 + h * (damping.damping_theta / inertia.inertia_theta);

        const double next_vx = (xi.vx + h * (f_x / inertia.mass_x)) / denom_x;
        const double next_vy = (xi.vy + h * (f_y / inertia.mass_y)) / denom_y;
        const double next_omega = (xi.omega + h * (f_theta / inertia.inertia_theta)) / denom_theta;

        const geometry::Twist2D next_xi(next_vx, next_vy, next_omega);
        if (!next_xi.is_valid()) {
            return false;
        }

        // 5. Position retraction on SE(2):
        //    X_i^{k+1} = X_i^k * exp( (h * xi^{k+1})^\wedge )
        const geometry::Twist2D delta_twist(h * next_vx, h * next_vy, h * next_omega);
        const geometry::SE2 next_pose = current_states.at(nid).pose * geometry::SE2::exp(delta_twist);

        if (!next_pose.is_valid()) {
            return false;
        }

        state.velocity = next_xi;
        state.pose = next_pose;
    }

    return true;
}

CordIntegratorResult integrate_cord(
    const graph::PoseGraph& graph,
    const std::unordered_map<graph::NodeId, NodeDynamicsState>& initial_states,
    const CordIntegratorOptions& options) {

    CordIntegratorResult result;
    result.final_states = initial_states;

    // Validate options
    if (!std::isfinite(options.timestep) || options.timestep <= 0.0 ||
        options.max_steps == 0 ||
        !options.default_inertia.is_valid() ||
        !options.default_damping.is_valid()) {
        result.status = CordStatus::InvalidInput;
        result.message = "Invalid integrator options (non-positive timestep, zero max_steps, or invalid inertia/damping)";
        return result;
    }

    // Validate initial states
    if (initial_states.empty()) {
        result.status = CordStatus::InvalidInput;
        result.message = "Initial dynamics states map is empty";
        return result;
    }

    for (const auto& [nid, state] : initial_states) {
        if (!nid.is_valid() || !state.is_valid()) {
            result.status = CordStatus::InvalidInput;
            result.message = "Initial state contains invalid NodeId or non-finite pose/velocity";
            return result;
        }
    }

    // Filter eligible edges
    size_t eligible_count = 0;
    for (const auto& [_, edge] : graph.edges()) {
        if (is_edge_eligible_for_objective(edge)) {
            eligible_count++;
            if (initial_states.find(edge.from_node) == initial_states.end() ||
                initial_states.find(edge.to_node) == initial_states.end()) {
                result.status = CordStatus::InvalidInput;
                result.message = "Eligible edge references node missing from initial states";
                return result;
            }
        }
    }

    result.num_eligible_edges = eligible_count;
    if (eligible_count == 0) {
        result.status = CordStatus::NoEligibleEdges;
        result.message = "Pose graph has no eligible edges for dynamics integration";
        return result;
    }

    // Connected component analysis on eligible graph
    graph::DisjointSet dsu;
    for (const auto& [nid, _] : initial_states) {
        dsu.make_set(nid);
    }
    for (const auto& [_, edge] : graph.edges()) {
        if (is_edge_eligible_for_objective(edge)) {
            dsu.unite(edge.from_node, edge.to_node);
        }
    }

    std::map<graph::NodeId, std::vector<graph::NodeId>> components;
    for (const auto& [nid, _] : initial_states) {
        auto root = dsu.find(nid);
        if (root.has_value()) {
            components[*root].push_back(nid);
        }
    }

    if (components.size() > 1 && !options.allow_disconnected) {
        result.status = CordStatus::MultipleDisconnectedComponents;
        std::ostringstream oss;
        oss << "Graph contains " << components.size()
            << " disconnected components, but allow_disconnected is false";
        result.message = oss.str();
        return result;
    }

    // Designate anchor nodes per component
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

    // Initial energy evaluations
    std::unordered_map<graph::NodeId, geometry::SE2> poses;
    std::unordered_map<graph::NodeId, geometry::Twist2D> vels;
    poses.reserve(initial_states.size());
    vels.reserve(initial_states.size());

    for (const auto& [nid, state] : initial_states) {
        poses[nid] = state.pose;
        vels[nid] = state.velocity;
    }

    ObjectiveEvaluationResult init_cost_eval = evaluate_pose_graph_objective(graph, poses);
    if (!init_cost_eval.valid) {
        result.status = CordStatus::NumericalFailure;
        result.message = "Failed to evaluate initial objective cost: " + init_cost_eval.error_message;
        return result;
    }

    result.initial_cost = init_cost_eval.total_cost;
    result.initial_kinetic_energy = compute_kinetic_energy(vels, options.default_inertia);
    result.initial_hamiltonian = compute_hamiltonian(result.initial_cost, result.initial_kinetic_energy);

    result.final_cost = result.initial_cost;
    result.final_kinetic_energy = result.initial_kinetic_energy;
    result.final_hamiltonian = result.initial_hamiltonian;

    std::unordered_map<graph::NodeId, NodeDynamicsState> current_states = initial_states;
    std::unordered_map<graph::NodeId, NodeDynamicsState> next_states;

    double grad_norm = 0.0;

    // Time-stepping integration loop
    for (size_t step = 0; step < options.max_steps; ++step) {
        result.steps = step + 1;

        if (!step_cord_integrator(graph, current_states, anchor_nodes, options.timestep,
                                  options.default_inertia, options.default_damping,
                                  next_states, &grad_norm)) {
            result.status = CordStatus::NumericalFailure;
            result.message = "Numerical failure encountered during discrete integration step";
            return result;
        }

        current_states = next_states;

        // Extract velocities and poses
        poses.clear();
        vels.clear();
        double max_v_norm = 0.0;
        for (const auto& [nid, state] : current_states) {
            poses[nid] = state.pose;
            vels[nid] = state.velocity;
            max_v_norm = std::max(max_v_norm, std::sqrt(state.velocity.vx * state.velocity.vx +
                                                        state.velocity.vy * state.velocity.vy +
                                                        state.velocity.omega * state.velocity.omega));
        }

        const double current_ke = compute_kinetic_energy(vels, options.default_inertia);
        result.final_gradient_norm = grad_norm;
        result.final_velocity_norm = max_v_norm;
        result.final_kinetic_energy = current_ke;

        // Stationarity convergence check
        if (grad_norm < options.gradient_tolerance && current_ke < options.kinetic_energy_tolerance) {
            result.status = CordStatus::Success;
            result.message = "Converged: gradient norm and kinetic energy below tolerances";
            break;
        }
    }

    result.final_states = current_states;

    // Final cost and Hamiltonian
    ObjectiveEvaluationResult final_cost_eval = evaluate_pose_graph_objective(graph, poses);
    if (final_cost_eval.valid) {
        result.final_cost = final_cost_eval.total_cost;
    }
    result.final_hamiltonian = compute_hamiltonian(result.final_cost, result.final_kinetic_energy);

    if (result.status != CordStatus::Success) {
        result.status = CordStatus::MaxStepsReached;
        result.message = "Reached maximum integration steps without meeting convergence criteria";
    }

    return result;
}

CordIntegratorResult integrate_cord(
    const graph::PoseGraph& graph,
    const std::unordered_map<graph::NodeId, geometry::SE2>& initial_poses,
    const CordIntegratorOptions& options) {

    std::unordered_map<graph::NodeId, NodeDynamicsState> states;
    states.reserve(initial_poses.size());
    for (const auto& [nid, pose] : initial_poses) {
        states[nid] = NodeDynamicsState(pose, geometry::Twist2D(0.0, 0.0, 0.0));
    }
    return integrate_cord(graph, states, options);
}

} // namespace astromesh::optimization
