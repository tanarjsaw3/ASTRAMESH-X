#include "astromesh/optimization/centralized_optimizer.hpp"
#include "astromesh/optimization/trajectory_metrics.hpp"
#include "support/synthetic_pose_graph.hpp"

#include <cassert>
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
// 1. Metric Hand Calculations
// ---------------------------------------------------------------------------
void test_metric_hand_calculations() {
    std::cout << "[Test 1] Trajectory metrics hand-calculated verification...\n";

    NodeId n1(1, 1);
    NodeId n2(1, 2);

    // Exact identical poses -> ATE translation and rotation == 0
    std::unordered_map<NodeId, SE2> est1 = {
        {n1, SE2(0.0, 0.0, 0.0)},
        {n2, SE2(1.0, 0.0, 0.0)}
    };
    std::unordered_map<NodeId, SE2> gt1 = est1;

    AbsoluteTrajectoryError ate_zero = compute_ate(est1, gt1, false);
    EXPECT_TRUE(ate_zero.valid);
    EXPECT_NEAR(ate_zero.translation.rmse, 0.0, 1e-12);
    EXPECT_NEAR(ate_zero.rotation.rmse, 0.0, 1e-12);

    // Known translation offset: est = gt + (3, 4) -> error is sqrt(3^2 + 4^2) = 5.0 m
    std::unordered_map<NodeId, SE2> est_shift = {
        {n1, SE2(3.0, 4.0, 0.0)},
        {n2, SE2(4.0, 4.0, 0.0)}
    };
    AbsoluteTrajectoryError ate_raw = compute_ate(est_shift, gt1, false);
    EXPECT_TRUE(ate_raw.valid);
    EXPECT_NEAR(ate_raw.translation.rmse, 5.0, 1e-9);
    EXPECT_NEAR(ate_raw.translation.mean, 5.0, 1e-9);

    // With Umeyama rigid alignment: pure translation shift is eliminated -> aligned RMSE == 0
    AbsoluteTrajectoryError ate_aligned = compute_ate(est_shift, gt1, true);
    EXPECT_TRUE(ate_aligned.valid);
    EXPECT_TRUE(ate_aligned.alignment_applied);
    EXPECT_NEAR(ate_aligned.translation.rmse, 0.0, 1e-9);

    // RPE hand calculation: step delta = 1
    // In gt1: T_gt = (0,0,0)^{-1} * (1,0,0) = (1,0,0)
    // In est_shift: T_est = (3,4,0)^{-1} * (4,4,0) = (1,0,0) -> relative motion is identical!
    RelativePoseError rpe = compute_rpe(est_shift, gt1, 1);
    EXPECT_TRUE(rpe.valid);
    EXPECT_NEAR(rpe.translation.rmse, 0.0, 1e-12);
    EXPECT_NEAR(rpe.rotation.rmse, 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// 2. Consistent Zero-Residual Graph
// ---------------------------------------------------------------------------
void test_consistent_zero_residual_graph() {
    std::cout << "[Test 2] Consistent zero-residual graph optimization...\n";

    PoseGraph graph;
    NodeId n1(1, 1);
    NodeId n2(1, 2);
    NodeId n3(1, 3);

    std::unordered_map<NodeId, SE2> true_poses = {
        {n1, SE2(0.0, 0.0, 0.0)},
        {n2, SE2(1.0, 0.0, 0.0)},
        {n3, SE2(2.0, 0.0, 0.0)}
    };

    add_node_helper(graph, n1, true_poses[n1].to_pose2d(), 100);
    add_node_helper(graph, n2, true_poses[n2].to_pose2d(), 200);
    add_node_helper(graph, n3, true_poses[n3].to_pose2d(), 300);

    const auto info = InformationMatrix2D::diagonal(100.0, 100.0);
    add_odometry_helper(graph, n1, n2, Pose2D(1.0, 0.0, 0.0), info);
    add_odometry_helper(graph, n2, n3, Pose2D(1.0, 0.0, 0.0), info);

    OptimizerOptions opt;
    opt.gradient_tolerance = 1e-6;

    OptimizerResult res = optimize_pose_graph(graph, true_poses, opt);
    EXPECT_TRUE(res.is_converged());
    EXPECT_NEAR(res.initial_cost, 0.0, 1e-12);
    EXPECT_NEAR(res.final_cost, 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// 3. Perturbed Pose Graph Optimization
// ---------------------------------------------------------------------------
void test_perturbed_graph_reduction() {
    std::cout << "[Test 3] Perturbed graph optimization reduces cost...\n";

    // Synthetic closed loop with 10 nodes and noisy initial estimates
    SyntheticDataset ds = generate_closed_loop(10, 4.0, 0.01, 0.005, 0.08, 0.03, true, 101);

    OptimizerOptions opt;
    opt.max_iterations = 50;

    OptimizerResult res = optimize_pose_graph(ds.graph, ds.initial_poses, opt);
    EXPECT_TRUE(res.is_converged());
    EXPECT_TRUE(res.successful_steps > 0);
    EXPECT_TRUE(res.final_cost < res.initial_cost);

    // Verify significant cost reduction (> 90%)
    double cost_drop_pct = (res.initial_cost - res.final_cost) / res.initial_cost;
    EXPECT_TRUE(cost_drop_pct > 0.90);

    // Verify ATE reduction
    auto ate_init = compute_ate(ds.initial_poses, ds.ground_truth_poses, false);
    auto ate_opt = compute_ate(res.optimized_poses, ds.ground_truth_poses, false);
    EXPECT_TRUE(ate_opt.translation.rmse < ate_init.translation.rmse);
}

// ---------------------------------------------------------------------------
// 4. Fixed Gauge Anchor Preservation
// ---------------------------------------------------------------------------
void test_gauge_anchor_preservation() {
    std::cout << "[Test 4] Gauge anchor preservation...\n";

    SyntheticDataset ds = generate_straight_chain(5, 1.0, 0.02, 0.01, 0.05, 0.02, 202);
    NodeId anchor_nid(1, 0);

    OptimizerOptions opt;
    opt.fixed_anchor = anchor_nid;

    const SE2 initial_anchor_pose = ds.initial_poses[anchor_nid];
    OptimizerResult res = optimize_pose_graph(ds.graph, ds.initial_poses, opt);

    EXPECT_TRUE(res.is_converged());
    const SE2 final_anchor_pose = res.optimized_poses[anchor_nid];

    // The anchor pose must remain exactly identical down to floating point noise
    EXPECT_NEAR(final_anchor_pose.x(), initial_anchor_pose.x(), 1e-12);
    EXPECT_NEAR(final_anchor_pose.y(), initial_anchor_pose.y(), 1e-12);
    EXPECT_NEAR(final_anchor_pose.theta(), initial_anchor_pose.theta(), 1e-12);
}

// ---------------------------------------------------------------------------
// 5. Loop Closure Contribution & Unverified/Rejected Rejection
// ---------------------------------------------------------------------------
void test_edge_admission_and_rejection() {
    std::cout << "[Test 5] Edge admission and unverified/rejected exclusion...\n";

    // Base clean graph with only odometry and verified loop closures
    SyntheticDataset ds_clean = generate_closed_loop(12, 3.0, 0.02, 0.01, 0.06, 0.02, true, 303);

    // Gated graph: exactly identical base, plus bogus Unverified and Rejected loop closures
    SyntheticDataset ds_gated = ds_clean;
    const auto info = InformationMatrix2D::diagonal(100.0, 100.0);

    // Bogus unverified edge
    add_loop_closure_helper(ds_gated.graph, NodeId(1, 1), NodeId(1, 8),
                            Pose2D(999.0, -999.0, 2.5), info,
                            EdgeVerificationStatus::Unverified);
    // Bogus rejected edge
    add_loop_closure_helper(ds_gated.graph, NodeId(1, 3), NodeId(1, 10),
                            Pose2D(-888.0, 777.0, -1.8), info,
                            EdgeVerificationStatus::Rejected);

    OptimizerOptions opt;
    opt.max_iterations = 40;

    OptimizerResult res_clean = optimize_pose_graph(ds_clean.graph, ds_clean.initial_poses, opt);
    OptimizerResult res_gated = optimize_pose_graph(ds_gated.graph, ds_gated.initial_poses, opt);

    EXPECT_TRUE(res_clean.is_converged());
    EXPECT_TRUE(res_gated.is_converged());

    // Both initial and final costs must be identical
    EXPECT_NEAR(res_clean.initial_cost, res_gated.initial_cost, 1e-9);
    EXPECT_NEAR(res_clean.final_cost, res_gated.final_cost, 1e-9);

    // Optimized poses must be identical
    for (const auto& [nid, pose_clean] : res_clean.optimized_poses) {
        const auto& pose_gated = res_gated.optimized_poses[nid];
        EXPECT_NEAR(pose_clean.x(), pose_gated.x(), 1e-9);
        EXPECT_NEAR(pose_clean.y(), pose_gated.y(), 1e-9);
        EXPECT_NEAR(pose_clean.theta(), pose_gated.theta(), 1e-9);
    }

    // Excluded edges count must be 2 in gated
    EXPECT_TRUE(res_gated.num_excluded_edges == 2);
}

// ---------------------------------------------------------------------------
// 6. Disconnected Components Handling
// ---------------------------------------------------------------------------
void test_disconnected_components() {
    std::cout << "[Test 6] Disconnected components and gauge handling...\n";

    SyntheticDataset ds = generate_multi_component_graph(2, 6, 404);

    // Case A: allow_disconnected = false -> should return MultipleDisconnectedComponents
    OptimizerOptions opt_disallow;
    opt_disallow.allow_disconnected = false;

    OptimizerResult res_disallow = optimize_pose_graph(ds.graph, ds.initial_poses, opt_disallow);
    EXPECT_FALSE(res_disallow.is_converged());
    EXPECT_TRUE(res_disallow.status == OptimizerStatus::MultipleDisconnectedComponents);

    // Case B: allow_disconnected = true -> should succeed, anchoring 1 node per component
    OptimizerOptions opt_allow;
    opt_allow.allow_disconnected = true;

    OptimizerResult res_allow = optimize_pose_graph(ds.graph, ds.initial_poses, opt_allow);
    EXPECT_TRUE(res_allow.is_converged());
    EXPECT_TRUE(res_allow.fixed_anchor_nodes.size() == 2);
}

// ---------------------------------------------------------------------------
// 7. Invalid Input Handling
// ---------------------------------------------------------------------------
void test_invalid_input_handling() {
    std::cout << "[Test 7] Invalid inputs and malformed constraints...\n";

    PoseGraph graph;
    NodeId n1(1, 1);
    NodeId n2(1, 2);

    add_node_helper(graph, n1, Pose2D(0, 0, 0), 100);
    add_node_helper(graph, n2, Pose2D(1, 0, 0), 200);

    const auto info_ok = InformationMatrix2D::diagonal(1.0, 1.0);
    add_odometry_helper(graph, n1, n2, Pose2D(1, 0, 0), info_ok);

    // 1. Non-finite initial pose
    std::unordered_map<NodeId, SE2> invalid_poses = {
        {n1, SE2(0.0, 0.0, 0.0)},
        {n2, SE2(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)}
    };
    OptimizerResult res1 = optimize_pose_graph(graph, invalid_poses);
    EXPECT_TRUE(res1.status == OptimizerStatus::InvalidInput);

    // 2. Indefinite information matrix rejection
    PoseGraph bad_graph;
    add_node_helper(bad_graph, n1, Pose2D(0, 0, 0), 100);
    add_node_helper(bad_graph, n2, Pose2D(1, 0, 0), 200);
    InformationMatrix2D bad_info(-1.0, 0.0, 0.0, -1.0, 0.0, -1.0); // Negative definite
    bool edge_added = add_odometry_helper(bad_graph, n1, n2, Pose2D(1, 0, 0), bad_info);
    EXPECT_FALSE(edge_added); // PoseGraph rejects indefinite info matrix at insertion

    // 3. Missing endpoint node in initial_poses
    std::unordered_map<NodeId, SE2> partial_poses = {
        {n1, SE2(0.0, 0.0, 0.0)} // Missing n2
    };
    OptimizerResult res2 = optimize_pose_graph(graph, partial_poses);
    EXPECT_TRUE(res2.status == OptimizerStatus::InvalidInput);

    // 4. Graph with 0 eligible edges
    PoseGraph empty_graph;
    add_node_helper(empty_graph, n1, Pose2D(0, 0, 0), 100);
    OptimizerResult res3 = optimize_pose_graph(empty_graph, {{n1, SE2(0, 0, 0)}});
    EXPECT_TRUE(res3.status == OptimizerStatus::NoEligibleEdges);
}

// ---------------------------------------------------------------------------
// 8. Monotonic Accepted Steps and Repeatability
// ---------------------------------------------------------------------------
void test_monotonicity_and_repeatability() {
    std::cout << "[Test 8] Monotonic cost behavior and repeatability...\n";

    SyntheticDataset ds = generate_closed_loop(16, 4.0, 0.02, 0.01, 0.05, 0.02, true, 505);

    OptimizerOptions opt;
    opt.max_iterations = 30;

    OptimizerResult run1 = optimize_pose_graph(ds.graph, ds.initial_poses, opt);
    OptimizerResult run2 = optimize_pose_graph(ds.graph, ds.initial_poses, opt);

    EXPECT_TRUE(run1.is_converged());
    EXPECT_TRUE(run2.is_converged());

    // Deterministic repeatability: exact same metrics and final poses
    EXPECT_NEAR(run1.final_cost, run2.final_cost, 1e-12);
    EXPECT_TRUE(run1.iterations == run2.iterations);
    EXPECT_TRUE(run1.successful_steps == run2.successful_steps);

    for (const auto& [nid, pose1] : run1.optimized_poses) {
        const auto& pose2 = run2.optimized_poses[nid];
        EXPECT_NEAR(pose1.x(), pose2.x(), 1e-12);
        EXPECT_NEAR(pose1.y(), pose2.y(), 1e-12);
        EXPECT_NEAR(pose1.theta(), pose2.theta(), 1e-12);
    }
}

// ---------------------------------------------------------------------------
// 9. Difficult Initialization and Max Iterations
// ---------------------------------------------------------------------------
void test_difficult_initialization() {
    std::cout << "[Test 9] Difficult initialization reporting non-convergence...\n";

    SyntheticDataset ds = generate_closed_loop(8, 3.0, 0.02, 0.01, 0.05, 0.02, true, 606);

    // Intentionally corrupt initial poses with massive perturbations
    std::unordered_map<NodeId, SE2> wild_poses = ds.initial_poses;
    wild_poses[NodeId(1, 3)] = SE2(100.0, -80.0, 3.14);
    wild_poses[NodeId(1, 5)] = SE2(-90.0, 110.0, -2.5);

    OptimizerOptions opt;
    opt.max_iterations = 2; // Strict low limit so it cannot converge

    OptimizerResult res = optimize_pose_graph(ds.graph, wild_poses, opt);
    EXPECT_FALSE(res.is_converged());
    EXPECT_TRUE(res.status == OptimizerStatus::MaxIterationsReached);
}

int main() {
    std::cout << "=======================================================\n";
    std::cout << " RUNNING CENTRALIZED REFERENCE OPTIMIZER TEST SUITE\n";
    std::cout << "=======================================================\n";

    test_metric_hand_calculations();
    test_consistent_zero_residual_graph();
    test_perturbed_graph_reduction();
    test_gauge_anchor_preservation();
    test_edge_admission_and_rejection();
    test_disconnected_components();
    test_invalid_input_handling();
    test_monotonicity_and_repeatability();
    test_difficult_initialization();

    std::cout << "\n>> All Centralized Optimizer tests PASSED ("
              << g_assertions << " assertions verified)\n";
    return 0;
}
