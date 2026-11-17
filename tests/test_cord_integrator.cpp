#include "astromesh/optimization/cord_integrator.hpp"
#include "support/synthetic_pose_graph.hpp"

#include <cmath>
#include <iostream>
#include <vector>

using namespace astromesh;
using namespace astromesh::geometry;
using namespace astromesh::graph;
using namespace astromesh::optimization;
using namespace astromesh::tests::support;

static size_t g_assertions = 0;
#define EXPECT_TRUE(cond) do { \
    g_assertions++; \
    if (!(cond)) { \
        std::cerr << "Assertion FAILED at line " << __LINE__ << ": " << #cond << "\n"; \
        std::exit(1); \
    } \
} while(0)

#define EXPECT_FALSE(cond) EXPECT_TRUE(!(cond))
#define EXPECT_NEAR(a, b, eps) EXPECT_TRUE(std::abs((a) - (b)) <= (eps))

// ---------------------------------------------------------------------------
// 1. Stationary State Invariant: zero velocity and zero gradient -> no motion
// ---------------------------------------------------------------------------
void test_stationary_state() {
    std::cout << "[Test 1] Zero velocity & zero gradient stationary state...\n";

    PoseGraph graph;
    NodeId n1(1, 1);
    NodeId n2(1, 2);

    add_node_helper(graph, n1, Pose2D(0, 0, 0), 100);
    add_node_helper(graph, n2, Pose2D(1, 0, 0), 200);

    const auto info = InformationMatrix2D::diagonal(100.0, 100.0);
    add_odometry_helper(graph, n1, n2, Pose2D(1, 0, 0), info);

    std::unordered_map<NodeId, NodeDynamicsState> states = {
        {n1, NodeDynamicsState(SE2(0, 0, 0), Twist2D(0, 0, 0))},
        {n2, NodeDynamicsState(SE2(1, 0, 0), Twist2D(0, 0, 0))}
    };

    std::unordered_map<NodeId, NodeDynamicsState> next_states;
    double grad_norm = 0.0;
    bool ok = step_cord_integrator(graph, states, {n1}, 0.01, NodeInertia(1, 1, 1), NodeDamping(2, 2, 2), next_states, &grad_norm);

    EXPECT_TRUE(ok);
    EXPECT_NEAR(grad_norm, 0.0, 1e-12);

    // Node 2 must remain exactly at (1, 0, 0) with zero velocity
    EXPECT_NEAR(next_states[n2].pose.x(), 1.0, 1e-12);
    EXPECT_NEAR(next_states[n2].pose.y(), 0.0, 1e-12);
    EXPECT_NEAR(next_states[n2].pose.theta(), 0.0, 1e-12);
    EXPECT_NEAR(next_states[n2].velocity.vx, 0.0, 1e-12);
    EXPECT_NEAR(next_states[n2].velocity.vy, 0.0, 1e-12);
    EXPECT_NEAR(next_states[n2].velocity.omega, 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// 2. Zero Damping Acceleration Match: xi^{k+1} = xi^k + h * M^{-1} * (-grad C)
// ---------------------------------------------------------------------------
void test_zero_damping_acceleration() {
    std::cout << "[Test 2] Zero damping acceleration matches -M^{-1} * grad(C)...\n";

    PoseGraph graph;
    NodeId n1(1, 1);
    NodeId n2(1, 2);

    add_node_helper(graph, n1, Pose2D(0, 0, 0), 100);
    add_node_helper(graph, n2, Pose2D(1.2, 0, 0), 200); // 0.2m displacement error

    const auto info = InformationMatrix2D::diagonal(50.0, 50.0);
    add_odometry_helper(graph, n1, n2, Pose2D(1.0, 0, 0), info);

    std::unordered_map<NodeId, NodeDynamicsState> states = {
        {n1, NodeDynamicsState(SE2(0, 0, 0), Twist2D(0, 0, 0))},
        {n2, NodeDynamicsState(SE2(1.2, 0, 0), Twist2D(0, 0, 0))}
    };

    const double h = 0.001; // small dt
    const NodeInertia inertia(2.0, 2.0, 2.0); // m = 2.0 kg
    const NodeDamping zero_damping(0.0, 0.0, 0.0); // D = 0

    std::unordered_map<NodeId, NodeDynamicsState> next_states;
    double grad_norm = 0.0;
    bool ok = step_cord_integrator(graph, states, {n1}, h, inertia, zero_damping, next_states, &grad_norm);

    EXPECT_TRUE(ok);
    // Since r_x = 0.2, Omega_xx = 50, grad_x = Omega_xx * r_x = 10.0 N
    // Acceleration a_x = -grad_x / m = -10.0 / 2.0 = -5.0 m/s^2
    // Next velocity vx = a_x * h = -5.0 * 0.001 = -0.005 m/s
    EXPECT_NEAR(next_states[n2].velocity.vx, -0.005, 1e-5);
    EXPECT_NEAR(next_states[n2].velocity.vy, 0.0, 1e-5);
    EXPECT_NEAR(next_states[n2].velocity.omega, 0.0, 1e-5);
}

// ---------------------------------------------------------------------------
// 3. Positive Damping Energy Dissipation
// ---------------------------------------------------------------------------
void test_positive_damping_dissipation() {
    std::cout << "[Test 3] Positive damping dissipates total energy over trajectory...\n";

    // Closed loop with realistic measurement noise (trans: 0.1m, rot: 0.05 rad -> Omega ~ 100, 400)
    SyntheticDataset ds = generate_closed_loop(8, 3.0, 0.1, 0.05, 0.05, 0.02, true, 42);

    CordIntegratorOptions opt;
    opt.timestep = 0.005;
    opt.max_steps = 400;
    opt.default_damping = NodeDamping(3.0, 3.0, 3.0);

    CordIntegratorResult res = integrate_cord(ds.graph, ds.initial_poses, opt);

    // Initial Hamiltonian vs Final Hamiltonian
    EXPECT_TRUE(res.final_hamiltonian < res.initial_hamiltonian);
    EXPECT_TRUE(res.final_cost < res.initial_cost);
    EXPECT_TRUE(res.final_kinetic_energy < 0.1); // Kinetic energy settles down
}

// ---------------------------------------------------------------------------
// 4. Gradient Consistency with Right Perturbation
// ---------------------------------------------------------------------------
void test_gradient_perturbation_consistency() {
    std::cout << "[Test 4] Gradient direction matches numerical right perturbation...\n";

    PoseGraph graph;
    NodeId n1(1, 1);
    NodeId n2(1, 2);

    add_node_helper(graph, n1, Pose2D(0, 0, 0), 100);
    add_node_helper(graph, n2, Pose2D(2.0, 0.5, 0.1), 200);

    const auto info = InformationMatrix2D::diagonal(20.0, 10.0);
    add_odometry_helper(graph, n1, n2, Pose2D(1.8, 0.4, 0.0), info);

    std::unordered_map<NodeId, SE2> poses = {
        {n1, SE2(0, 0, 0)},
        {n2, SE2(2.0, 0.5, 0.1)}
    };

    auto grad_eval = evaluate_objective_numerical_gradient(graph, poses);
    EXPECT_TRUE(grad_eval.valid);

    // Forward finite difference check on node 2 along vx
    const double eps = 1e-6;
    auto poses_plus = poses;
    poses_plus[n2] = poses[n2] * SE2::exp(Twist2D(eps, 0, 0));
    auto poses_minus = poses;
    poses_minus[n2] = poses[n2] * SE2::exp(Twist2D(-eps, 0, 0));

    double c_plus = evaluate_pose_graph_objective(graph, poses_plus).total_cost;
    double c_minus = evaluate_pose_graph_objective(graph, poses_minus).total_cost;
    double expected_g_vx = (c_plus - c_minus) / (2.0 * eps);

    EXPECT_NEAR(grad_eval.gradients[n2].vx, expected_g_vx, 1e-6);
}

// ---------------------------------------------------------------------------
// 5. Gauge Anchor Strict Invariance
// ---------------------------------------------------------------------------
void test_anchor_invariance() {
    std::cout << "[Test 5] Gauge anchor velocity zero and pose fixed...\n";

    SyntheticDataset ds = generate_straight_chain(6, 1.0, 0.02, 0.01, 0.06, 0.02, 101);
    NodeId anchor_nid(1, 0);

    CordIntegratorOptions opt;
    opt.timestep = 0.01;
    opt.max_steps = 50;
    opt.fixed_anchor = anchor_nid;

    SE2 initial_anchor_pose = ds.initial_poses[anchor_nid];
    CordIntegratorResult res = integrate_cord(ds.graph, ds.initial_poses, opt);

    EXPECT_TRUE(res.final_states.find(anchor_nid) != res.final_states.end());
    const auto& anchor_state = res.final_states.at(anchor_nid);

    EXPECT_NEAR(anchor_state.velocity.vx, 0.0, 1e-12);
    EXPECT_NEAR(anchor_state.velocity.vy, 0.0, 1e-12);
    EXPECT_NEAR(anchor_state.velocity.omega, 0.0, 1e-12);

    EXPECT_NEAR(anchor_state.pose.x(), initial_anchor_pose.x(), 1e-12);
    EXPECT_NEAR(anchor_state.pose.y(), initial_anchor_pose.y(), 1e-12);
    EXPECT_NEAR(anchor_state.pose.theta(), initial_anchor_pose.theta(), 1e-12);
}

// ---------------------------------------------------------------------------
// 6. Invalid Input Validation & Non-Finite Safeguards
// ---------------------------------------------------------------------------
void test_invalid_input_safeguards() {
    std::cout << "[Test 6] Invalid inputs and parameter safeguards...\n";

    PoseGraph graph;
    NodeId n1(1, 1);
    NodeId n2(1, 2);
    add_node_helper(graph, n1, Pose2D(0, 0, 0), 100);
    add_node_helper(graph, n2, Pose2D(1, 0, 0), 200);
    add_odometry_helper(graph, n1, n2, Pose2D(1, 0, 0), InformationMatrix2D::diagonal(1.0, 1.0));

    std::unordered_map<NodeId, NodeDynamicsState> states = {
        {n1, NodeDynamicsState(SE2(0, 0, 0))},
        {n2, NodeDynamicsState(SE2(1, 0, 0))}
    };

    // 1. Non-positive timestep
    CordIntegratorOptions opt_bad_h;
    opt_bad_h.timestep = -0.01;
    EXPECT_TRUE(integrate_cord(graph, states, opt_bad_h).status == CordStatus::InvalidInput);

    // 2. Zero max_steps
    CordIntegratorOptions opt_bad_steps;
    opt_bad_steps.max_steps = 0;
    EXPECT_TRUE(integrate_cord(graph, states, opt_bad_steps).status == CordStatus::InvalidInput);

    // 3. Negative inertia mass
    CordIntegratorOptions opt_bad_m;
    opt_bad_m.default_inertia = NodeInertia(-1.0, 1.0, 1.0);
    EXPECT_TRUE(integrate_cord(graph, states, opt_bad_m).status == CordStatus::InvalidInput);

    // 4. Negative damping
    CordIntegratorOptions opt_bad_d;
    opt_bad_d.default_damping = NodeDamping(-2.0, 1.0, 1.0);
    EXPECT_TRUE(integrate_cord(graph, states, opt_bad_d).status == CordStatus::InvalidInput);

    // 5. Non-finite pose in initial state
    auto bad_states = states;
    bad_states[n2].pose = SE2(std::numeric_limits<double>::quiet_NaN(), 0, 0);
    EXPECT_TRUE(integrate_cord(graph, bad_states).status == CordStatus::InvalidInput);
}

// ---------------------------------------------------------------------------
// 7. Disconnected Components Policy
// ---------------------------------------------------------------------------
void test_disconnected_components() {
    std::cout << "[Test 7] Disconnected components policy...\n";

    SyntheticDataset ds = generate_multi_component_graph(2, 5, 202);

    // Case A: allow_disconnected = false -> MultipleDisconnectedComponents
    CordIntegratorOptions opt_disallow;
    opt_disallow.allow_disconnected = false;
    CordIntegratorResult res_disallow = integrate_cord(ds.graph, ds.initial_poses, opt_disallow);
    EXPECT_TRUE(res_disallow.status == CordStatus::MultipleDisconnectedComponents);

    // Case B: allow_disconnected = true -> Successful multi-anchor integration
    CordIntegratorOptions opt_allow;
    opt_allow.allow_disconnected = true;
    opt_allow.max_steps = 20;
    CordIntegratorResult res_allow = integrate_cord(ds.graph, ds.initial_poses, opt_allow);
    EXPECT_TRUE(res_allow.fixed_anchor_nodes.size() == 2);
    EXPECT_TRUE(res_allow.status == CordStatus::MaxStepsReached || res_allow.status == CordStatus::Success);
}

// ---------------------------------------------------------------------------
// 8. Deterministic Repeatability
// ---------------------------------------------------------------------------
void test_deterministic_repeatability() {
    std::cout << "[Test 8] Deterministic repeatability across identical runs...\n";

    SyntheticDataset ds = generate_closed_loop(8, 3.0, 0.05, 0.02, 0.1, 0.05, true, 303);

    CordIntegratorOptions opt;
    opt.timestep = 0.002;
    opt.max_steps = 30;

    CordIntegratorResult run1 = integrate_cord(ds.graph, ds.initial_poses, opt);
    CordIntegratorResult run2 = integrate_cord(ds.graph, ds.initial_poses, opt);

    EXPECT_TRUE(run1.status == CordStatus::MaxStepsReached || run1.status == CordStatus::Success);
    EXPECT_TRUE(run2.status == CordStatus::MaxStepsReached || run2.status == CordStatus::Success);
    EXPECT_TRUE(std::isfinite(run1.final_kinetic_energy));
    EXPECT_TRUE(std::isfinite(run2.final_kinetic_energy));

    std::cout << "  run1 ke: " << run1.final_kinetic_energy << ", run2 ke: " << run2.final_kinetic_energy
              << ", diff: " << std::abs(run1.final_kinetic_energy - run2.final_kinetic_energy) << "\n";
    EXPECT_NEAR(run1.final_cost, run2.final_cost, 1e-9);
    EXPECT_NEAR(run1.final_kinetic_energy, run2.final_kinetic_energy, 1e-9);
    EXPECT_NEAR(run1.final_hamiltonian, run2.final_hamiltonian, 1e-9);

    for (const auto& [nid, s1] : run1.final_states) {
        const auto& s2 = run2.final_states.at(nid);
        EXPECT_NEAR(s1.pose.x(), s2.pose.x(), 1e-9);
        EXPECT_NEAR(s1.pose.y(), s2.pose.y(), 1e-9);
        EXPECT_NEAR(s1.pose.theta(), s2.pose.theta(), 1e-9);
        EXPECT_NEAR(s1.velocity.vx, s2.velocity.vx, 1e-9);
    }
}

// ---------------------------------------------------------------------------
// 9. Timestep Sensitivity and Discrete Energy Drift
// ---------------------------------------------------------------------------
void test_timestep_sensitivity_and_energy_drift() {
    std::cout << "[Test 9] Timestep sensitivity experiment over multiple step sizes...\n";

    // 2-node graph with zero damping to observe purely symplectic energy behavior
    PoseGraph graph;
    NodeId n1(1, 1);
    NodeId n2(1, 2);
    add_node_helper(graph, n1, Pose2D(0, 0, 0), 100);
    add_node_helper(graph, n2, Pose2D(1.1, 0, 0), 200);
    add_odometry_helper(graph, n1, n2, Pose2D(1.0, 0, 0), InformationMatrix2D::diagonal(100.0, 100.0));

    std::unordered_map<NodeId, NodeDynamicsState> init_states = {
        {n1, NodeDynamicsState(SE2(0, 0, 0))},
        {n2, NodeDynamicsState(SE2(1.1, 0, 0))}
    };

    const double timesteps[] = {0.02, 0.01, 0.005, 0.001};
    const double total_time = 0.05; // 50 ms

    std::vector<double> recorded_drifts;

    for (double h : timesteps) {
        const size_t num_steps = static_cast<size_t>(total_time / h);
        auto states = init_states;
        std::unordered_map<NodeId, NodeDynamicsState> next_states;

        double initial_H = evaluate_pose_graph_objective(graph, {{n1, SE2(0,0,0)}, {n2, SE2(1.1,0,0)}}).total_cost;

        for (size_t s = 0; s < num_steps; ++s) {
            bool ok = step_cord_integrator(graph, states, {n1}, h, NodeInertia(1, 1, 1), NodeDamping(0, 0, 0), next_states);
            EXPECT_TRUE(ok);
            states = next_states;
        }

        double final_V = evaluate_pose_graph_objective(graph, {{n1, states[n1].pose}, {n2, states[n2].pose}}).total_cost;
        double final_K = compute_kinetic_energy({{n1, states[n1].velocity}, {n2, states[n2].velocity}}, NodeInertia(1, 1, 1));
        double final_H = final_V + final_K;

        double energy_drift = std::abs(final_H - initial_H);
        std::cout << "   h = " << h << " s -> discrete energy drift |Delta H| = " << energy_drift << " J\n";
        recorded_drifts.push_back(energy_drift);
    }

    // Verify smaller timestep yields strictly smaller energy drift: drift(0.001) < drift(0.02)
    EXPECT_TRUE(recorded_drifts.back() < recorded_drifts.front());
    EXPECT_TRUE(recorded_drifts.back() < 0.01); // Sub-0.01 J drift at 1 ms step size
}

// ---------------------------------------------------------------------------
// 10. Continuous Energy Derivative Identity dH/dt == -sum xi^T D xi <= 0
// ---------------------------------------------------------------------------
void test_continuous_energy_derivative_identity() {
    std::cout << "[Test 10] Continuous energy derivative identity dH/dt == -sum xi^T D xi...\n";

    PoseGraph graph;
    NodeId n1(1, 1);
    NodeId n2(1, 2);
    add_node_helper(graph, n1, Pose2D(0, 0, 0), 100);
    add_node_helper(graph, n2, Pose2D(1.2, 0.3, 0.1), 200);

    const auto info = InformationMatrix2D::diagonal(100.0, 50.0);
    add_odometry_helper(graph, n1, n2, Pose2D(1.0, 0.0, 0.0), info);

    const Twist2D xi2(0.8, -0.6, 1.2);
    std::unordered_map<NodeId, NodeDynamicsState> states0 = {
        {n1, NodeDynamicsState(SE2(0, 0, 0), Twist2D(0, 0, 0))},
        {n2, NodeDynamicsState(SE2(1.2, 0.3, 0.1), xi2)}
    };

    struct TestConfig {
        std::string name;
        NodeInertia inertia;
        NodeDamping damping;
    };

    std::vector<TestConfig> configs = {
        {"Isotropic mass, anisotropic damping",
         NodeInertia(1.0, 1.0, 1.0),
         NodeDamping(2.5, 1.8, 3.2)},
        {"Strongly anisotropic mass, anisotropic damping",
         NodeInertia(3.0, 1.5, 4.0),
         NodeDamping(1.2, 2.8, 1.5)},
        {"Conservative undamped system (D = 0, anisotropic mass)",
         NodeInertia(3.0, 1.5, 4.0),
         NodeDamping(0.0, 0.0, 0.0)}
    };

    const double h = 1e-6; // 1 microsecond numerical derivative step

    for (const auto& cfg : configs) {
        // Compute initial Hamiltonian H0 = C0 + K0
        double V0 = evaluate_pose_graph_objective(graph, {{n1, states0[n1].pose}, {n2, states0[n2].pose}}).total_cost;
        double K0 = compute_kinetic_energy({{n1, states0[n1].velocity}, {n2, states0[n2].velocity}}, cfg.inertia);
        double H0 = V0 + K0;

        // Step forward by h
        std::unordered_map<NodeId, NodeDynamicsState> states1;
        bool ok = step_cord_integrator(graph, states0, {n1}, h, cfg.inertia, cfg.damping, states1);
        EXPECT_TRUE(ok);

        double V1 = evaluate_pose_graph_objective(graph, {{n1, states1[n1].pose}, {n2, states1[n2].pose}}).total_cost;
        double K1 = compute_kinetic_energy({{n1, states1[n1].velocity}, {n2, states1[n2].velocity}}, cfg.inertia);
        double H1 = V1 + K1;

        double dH_dt_num = (H1 - H0) / h;

        // Theoretical continuous power dissipation: - sum_{i in dynamic} xi_i^T * D_i * xi_i
        double dH_dt_theo = -(cfg.damping.damping_x * xi2.vx * xi2.vx +
                              cfg.damping.damping_y * xi2.vy * xi2.vy +
                              cfg.damping.damping_theta * xi2.omega * xi2.omega);

        std::cout << "   " << cfg.name << ":\n"
                  << "      dH/dt numerical   = " << dH_dt_num << " W\n"
                  << "      dH/dt theoretical = " << dH_dt_theo << " W\n";

        // Must match to within 1e-3 W (0.001 W tolerance accounts for O(h) Euler discretization error)
        EXPECT_NEAR(dH_dt_num, dH_dt_theo, 1e-3);
        EXPECT_TRUE(dH_dt_num <= 1e-5); // Strictly non-positive (dissipative or zero)
    }
}

// ---------------------------------------------------------------------------
// 11. Nontrivial Co-Adjoint Acceleration & Angular Momentum Coupling
// ---------------------------------------------------------------------------
void test_nontrivial_coadjoint_acceleration() {
    std::cout << "[Test 11] Nontrivial co-adjoint acceleration under anisotropic mass...\n";

    PoseGraph graph;
    NodeId n1(1, 1);
    NodeId n2(1, 2);
    add_node_helper(graph, n1, Pose2D(0, 0, 0), 100);
    // Pose exactly matching measurement Z12 = (1, 0, 0) so grad C(X) == 0
    add_node_helper(graph, n2, Pose2D(1.0, 0.0, 0.0), 200);

    const auto info = InformationMatrix2D::diagonal(100.0, 100.0);
    add_odometry_helper(graph, n1, n2, Pose2D(1.0, 0.0, 0.0), info);

    // Non-zero velocity on node 2: vx=1.2, vy=-3.4, omega=2.5
    const Twist2D xi(1.2, -3.4, 2.5);
    const NodeInertia inertia(1.0, 4.0, 2.0); // mx=1, my=4, I=2
    const NodeDamping zero_damping(0.0, 0.0, 0.0);

    std::unordered_map<NodeId, NodeDynamicsState> states = {
        {n1, NodeDynamicsState(SE2(0, 0, 0), Twist2D(0, 0, 0))},
        {n2, NodeDynamicsState(SE2(1.0, 0.0, 0.0), xi)}
    };

    const double h = 1e-5;
    std::unordered_map<NodeId, NodeDynamicsState> next_states;
    bool ok = step_cord_integrator(graph, states, {n1}, h, inertia, zero_damping, next_states);
    EXPECT_TRUE(ok);

    // Since grad C == 0 and D == 0:
    // a = M^{-1} * ad_xi^*(M * xi)
    // p = [1.0*1.2, 4.0*(-3.4), 2.0*2.5] = [1.2, -13.6, 5.0]
    // ad_xi^*(p) = [ omega * py,  -omega * px,  vy * px - vx * py ]
    //            = [ 2.5 * (-13.6), -2.5 * 1.2, (-3.4)*1.2 - 1.2*(-13.6) ]
    //            = [ -34.0, -3.0, -4.08 + 16.32 = +12.24 ]
    // a = [ -34.0/1.0, -3.0/4.0, 12.24/2.0 ] = [ -34.0, -0.75, +6.12 ]
    //
    // Note: If the flawed expression [omega py, -omega px, vx py - vy px] were used:
    // third component of ad would be vx*py - vy*px = -12.24
    // producing flawed angular acceleration a_theta_flawed = -12.24/2.0 = -6.12 rad/s^2!
    // That would be a huge sign error of 12.24 rad/s^2!

    const double expected_ax = -34.0;
    const double expected_ay = -0.75;
    const double expected_atheta = +6.12;

    const double computed_ax = (next_states[n2].velocity.vx - xi.vx) / h;
    const double computed_ay = (next_states[n2].velocity.vy - xi.vy) / h;
    const double computed_atheta = (next_states[n2].velocity.omega - xi.omega) / h;

    EXPECT_NEAR(computed_ax, expected_ax, 1e-3);
    EXPECT_NEAR(computed_ay, expected_ay, 1e-3);
    EXPECT_NEAR(computed_atheta, expected_atheta, 1e-3);

    // Verify gyroscopic power is zero:
    double power = xi.vx * (inertia.mass_x * computed_ax) +
                   xi.vy * (inertia.mass_y * computed_ay) +
                   xi.omega * (inertia.inertia_theta * computed_atheta);
    EXPECT_NEAR(power, 0.0, 1e-3);
}

int main() {
    std::cout << "=======================================================\n";
    std::cout << " RUNNING CORD DYNAMICS & SE(2) INTEGRATOR TEST SUITE\n";
    std::cout << "=======================================================\n";

    test_stationary_state();
    test_zero_damping_acceleration();
    test_positive_damping_dissipation();
    test_gradient_perturbation_consistency();
    test_anchor_invariance();
    test_invalid_input_safeguards();
    test_disconnected_components();
    test_deterministic_repeatability();
    test_timestep_sensitivity_and_energy_drift();
    test_continuous_energy_derivative_identity();
    test_nontrivial_coadjoint_acceleration();

    std::cout << "\n>> All CORD Integrator tests PASSED ("
              << g_assertions << " assertions verified)\n";
    return 0;
}
