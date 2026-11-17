#include "astromesh/optimization/pose_graph_objective.hpp"
#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"

#include <iostream>
#include <cmath>
#include <limits>
#include <vector>

using namespace astromesh;
using geometry::SE2;
using geometry::Twist2D;
using graph::NodeId;
using graph::Pose2D;
using graph::KeyframeNode;
using graph::PoseGraphEdge;
using graph::InformationMatrix2D;
using graph::EdgeType;
using graph::EdgeVerificationStatus;
using graph::PoseGraph;
using optimization::compute_edge_residual;
using optimization::is_edge_eligible_for_objective;
using optimization::evaluate_pose_graph_objective;
using optimization::evaluate_objective_numerical_gradient;

static int g_total_assertions = 0;
static int g_failed_assertions = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        ++g_total_assertions; \
        if (!(cond)) { \
            ++g_failed_assertions; \
            std::cerr << "  FAILED: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
        } \
    } while (0)

constexpr double PI = 3.14159265358979323846;
constexpr double EPS_TIGHT = 1e-9;

void test_zero_residual_consistent_measurement() {
    std::cout << "[TEST 1] Zero Residual & Zero Cost on Consistent Measurement...\n";

    SE2 X1(10.0, 20.0, PI / 6.0);
    SE2 X2(12.0, 25.0, PI / 3.0);

    // Exact measurement matching relative pose: Z_12 = X1^{-1} * X2
    SE2 Z12 = X1.inverse() * X2;
    InformationMatrix2D omega = InformationMatrix2D::diagonal(100.0, 500.0);

    auto res = compute_edge_residual(X1, X2, Z12, omega);
    TEST_ASSERT(res.valid, "Evaluation must succeed");
    TEST_ASSERT(std::abs(res.residual.vx) < EPS_TIGHT, "Residual vx == 0");
    TEST_ASSERT(std::abs(res.residual.vy) < EPS_TIGHT, "Residual vy == 0");
    TEST_ASSERT(std::abs(res.residual.omega) < EPS_TIGHT, "Residual omega == 0");
    TEST_ASSERT(std::abs(res.cost) < EPS_TIGHT, "Cost == 0.0");
}

void test_hand_calculated_worked_example() {
    std::cout << "[TEST 2] Verification of M3.1 Hand-Calculated Worked Example...\n";

    // Values from docs/CORD_MATHEMATICAL_FORMULATION.md (Section 8)
    SE2 X1(0.0, 0.0, 0.0);
    SE2 X2(1.20, 0.10, 0.05);
    SE2 Z12(1.0, 0.0, 0.0);
    InformationMatrix2D omega(100.0, 0.0, 0.0, 100.0, 0.0, 400.0);

    auto res = compute_edge_residual(X1, X2, Z12, omega);
    TEST_ASSERT(res.valid, "Worked example evaluation succeeds");

    // In M3.1 Section 8.2:
    // r_12 = [0.202458, 0.094979, 0.050000]^T
    TEST_ASSERT(std::abs(res.residual.vx - 0.202458) < 1e-4, "r.vx matches hand-calculated 0.202458");
    TEST_ASSERT(std::abs(res.residual.vy - 0.094979) < 1e-4, "r.vy matches hand-calculated 0.094979");
    TEST_ASSERT(std::abs(res.residual.omega - 0.050000) < 1e-4, "r.omega matches hand-calculated 0.050000");

    // Cost C = 3.0005
    TEST_ASSERT(std::abs(res.cost - 3.0005) < 1e-3, "Cost matches hand-calculated 3.0005");
}

void test_global_frame_gauge_invariance() {
    std::cout << "[TEST 3] Global Frame Rigid Transformation Gauge Invariance...\n";

    PoseGraph graph;
    NodeId n1(1, 1), n2(1, 2), n3(1, 3);
    graph.add_node(KeyframeNode(n1, Pose2D(0.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(n2, Pose2D(5.0, 1.0, 0.2)));
    graph.add_node(KeyframeNode(n3, Pose2D(10.0, 0.5, -0.1)));

    // Non-zero residual constraints
    graph.add_edge(PoseGraphEdge(1, n1, n2, EdgeType::Odometry, Pose2D(4.8, 1.2, 0.15)));
    graph.add_edge(PoseGraphEdge(2, n2, n3, EdgeType::Odometry, Pose2D(5.1, -0.4, -0.25)));

    // 1. Base evaluation
    auto base_eval = evaluate_pose_graph_objective(graph);
    TEST_ASSERT(base_eval.valid, "Base objective evaluation valid");
    TEST_ASSERT(base_eval.total_cost > 0.0, "Cost is non-zero");

    // 2. Apply arbitrary global rigid transformation: g = (25.0, -14.0, 1.2 rad)
    SE2 g(25.0, -14.0, 1.2);
    std::unordered_map<NodeId, SE2> transformed_poses;
    transformed_poses[n1] = g * SE2(0.0, 0.0, 0.0);
    transformed_poses[n2] = g * SE2(5.0, 1.0, 0.2);
    transformed_poses[n3] = g * SE2(10.0, 0.5, -0.1);

    auto trans_eval = evaluate_pose_graph_objective(graph, transformed_poses);
    TEST_ASSERT(trans_eval.valid, "Transformed objective evaluation valid");
    TEST_ASSERT(std::abs(trans_eval.total_cost - base_eval.total_cost) < 1e-9,
                "Objective is invariant under arbitrary global frame transformation g * X");
}

void test_edge_eligibility_policy() {
    std::cout << "[TEST 4] Edge Eligibility and Verification Status Gating...\n";

    PoseGraph graph;
    NodeId r1_1(1, 1), r1_2(1, 2), r1_3(1, 3);
    NodeId r2_1(2, 1);
    graph.add_node(KeyframeNode(r1_1, Pose2D(0.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(r1_2, Pose2D(1.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(r1_3, Pose2D(2.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(r2_1, Pose2D(0.0, 5.0, 0.0)));

    // 1. Odometry (Unverified) -> Eligible
    PoseGraphEdge e_odo_unver(1, r1_1, r1_2, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0),
                              InformationMatrix2D::diagonal(10.0, 10.0), EdgeVerificationStatus::Unverified);
    TEST_ASSERT(is_edge_eligible_for_objective(e_odo_unver), "Unverified odometry is eligible");

    // 2. Odometry (Verified) -> Eligible
    PoseGraphEdge e_odo_ver(2, r1_2, r1_3, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0),
                            InformationMatrix2D::diagonal(10.0, 10.0), EdgeVerificationStatus::Verified);
    TEST_ASSERT(is_edge_eligible_for_objective(e_odo_ver), "Verified odometry is eligible");

    // 3. Odometry (Rejected) -> Ineligible
    PoseGraphEdge e_odo_rej(3, r1_1, r1_3, EdgeType::Odometry, Pose2D(2.0, 0.0, 0.0),
                            InformationMatrix2D::diagonal(10.0, 10.0), EdgeVerificationStatus::Rejected);
    TEST_ASSERT(!is_edge_eligible_for_objective(e_odo_rej), "Rejected odometry is INELIGIBLE");

    // 4. Intra-Rover Loop Closure (Unverified) -> Ineligible
    PoseGraphEdge e_intra_unver(4, r1_1, r1_3, EdgeType::IntraRoverLoopClosure, Pose2D(2.0, 0.0, 0.0),
                                InformationMatrix2D::diagonal(10.0, 10.0), EdgeVerificationStatus::Unverified);
    TEST_ASSERT(!is_edge_eligible_for_objective(e_intra_unver), "Unverified intra-rover closure is INELIGIBLE");

    // 5. Intra-Rover Loop Closure (Verified) -> Eligible
    PoseGraphEdge e_intra_ver(5, r1_1, r1_3, EdgeType::IntraRoverLoopClosure, Pose2D(2.0, 0.0, 0.0),
                              InformationMatrix2D::diagonal(10.0, 10.0), EdgeVerificationStatus::Verified);
    TEST_ASSERT(is_edge_eligible_for_objective(e_intra_ver), "Verified intra-rover closure is ELIGIBLE");

    // 6. Intra-Rover Loop Closure (Rejected) -> Ineligible
    PoseGraphEdge e_intra_rej(6, r1_1, r1_3, EdgeType::IntraRoverLoopClosure, Pose2D(2.0, 0.0, 0.0),
                              InformationMatrix2D::diagonal(10.0, 10.0), EdgeVerificationStatus::Rejected);
    TEST_ASSERT(!is_edge_eligible_for_objective(e_intra_rej), "Rejected intra-rover closure is INELIGIBLE");

    // 7. Inter-Rover Loop Closure (Unverified) -> Ineligible
    PoseGraphEdge e_inter_unver(7, r1_1, r2_1, EdgeType::InterRoverLoopClosure, Pose2D(0.0, 5.0, 0.0),
                                InformationMatrix2D::diagonal(10.0, 10.0), EdgeVerificationStatus::Unverified);
    TEST_ASSERT(!is_edge_eligible_for_objective(e_inter_unver), "Unverified inter-rover closure is INELIGIBLE");

    // 8. Inter-Rover Loop Closure (Verified) -> Eligible
    PoseGraphEdge e_inter_ver(8, r1_1, r2_1, EdgeType::InterRoverLoopClosure, Pose2D(0.0, 5.0, 0.0),
                              InformationMatrix2D::diagonal(10.0, 10.0), EdgeVerificationStatus::Verified);
    TEST_ASSERT(is_edge_eligible_for_objective(e_inter_ver), "Verified inter-rover closure is ELIGIBLE");

    // 9. Inter-Rover Loop Closure (Rejected) -> Ineligible
    PoseGraphEdge e_inter_rej(9, r1_1, r2_1, EdgeType::InterRoverLoopClosure, Pose2D(0.0, 5.0, 0.0),
                              InformationMatrix2D::diagonal(10.0, 10.0), EdgeVerificationStatus::Rejected);
    TEST_ASSERT(!is_edge_eligible_for_objective(e_inter_rej), "Rejected inter-rover closure is INELIGIBLE");

    // Build graph with 2 eligible and 2 ineligible edges
    graph.add_edge(e_odo_unver);   // ID 1 (Eligible)
    graph.add_edge(e_odo_ver);     // ID 2 (Eligible)
    graph.add_edge(e_intra_unver); // ID 4 (Ineligible)
    graph.add_edge(e_inter_rej);   // ID 9 (Ineligible)

    auto eval = evaluate_pose_graph_objective(graph);
    TEST_ASSERT(eval.valid, "Evaluation must succeed");
    TEST_ASSERT(eval.num_eligible_edges == 2, "Exactly 2 eligible edges evaluated");
    TEST_ASSERT(eval.num_odometry_edges == 2, "Both eligible edges are odometry");
    TEST_ASSERT(eval.num_verified_loop_closures == 0, "No verified loop closures");
    TEST_ASSERT(eval.num_skipped_unverified == 1, "1 unverified skipped");
    TEST_ASSERT(eval.num_skipped_rejected == 1, "1 rejected skipped");
}

void test_invalid_graph_data_rejection() {
    std::cout << "[TEST 5] Rejection of Invalid Data, Missing Nodes, and Indefinite Info Matrices...\n";

    double nan_val = std::numeric_limits<double>::quiet_NaN();

    // 1. Missing node in poses map
    PoseGraph graph;
    NodeId n1(1, 1), n2(1, 2);
    graph.add_node(KeyframeNode(n1, Pose2D(0.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(n2, Pose2D(1.0, 0.0, 0.0)));
    graph.add_edge(PoseGraphEdge(1, n1, n2, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0)));

    std::unordered_map<NodeId, SE2> partial_poses;
    partial_poses[n1] = SE2(0.0, 0.0, 0.0);
    // n2 is missing!

    auto eval_missing = evaluate_pose_graph_objective(graph, partial_poses);
    TEST_ASSERT(!eval_missing.valid, "Objective must reject eligible edge referencing missing node in poses");

    // 2. NaN in node pose
    std::unordered_map<NodeId, SE2> nan_poses;
    nan_poses[n1] = SE2(0.0, 0.0, 0.0);
    nan_poses[n2] = SE2(nan_val, 0.0, 0.0);

    auto eval_nan = evaluate_pose_graph_objective(graph, nan_poses);
    TEST_ASSERT(!eval_nan.valid, "Objective must reject NaN node pose");

    // 3. Indefinite information matrix
    InformationMatrix2D not_pd(1.0, 5.0, 0.0, 1.0, 0.0, 1.0); // det(submatrix) = 1 - 25 = -24 < 0
    auto edge_res = compute_edge_residual(SE2::identity(), SE2(1.0, 0.0, 0.0), SE2(1.0, 0.0, 0.0), not_pd);
    TEST_ASSERT(!edge_res.valid, "compute_edge_residual must reject indefinite information matrix");
}

void test_numerical_gradient_reference() {
    std::cout << "[TEST 6] Reference Numerical Riemannian Gradient Validation...\n";

    PoseGraph graph;
    NodeId n1(1, 1), n2(1, 2);
    graph.add_node(KeyframeNode(n1, Pose2D(0.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(n2, Pose2D(1.20, 0.10, 0.05)));

    // Edge constraint from M3.1 worked example:
    // Z_12 = (1.0, 0, 0), Omega = diag(100, 100, 400)
    InformationMatrix2D omega(100.0, 0.0, 0.0, 100.0, 0.0, 400.0);
    graph.add_edge(PoseGraphEdge(1, n1, n2, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0), omega, EdgeVerificationStatus::Verified));

    std::unordered_map<NodeId, SE2> poses;
    poses[n1] = SE2(0.0, 0.0, 0.0);
    poses[n2] = SE2(1.20, 0.10, 0.05);

    auto grad_res = evaluate_objective_numerical_gradient(graph, poses, 1e-6);
    TEST_ASSERT(grad_res.valid, "Numerical gradient evaluation valid");

    // The exact numerical Riemannian gradient on X2 accounts for the exact right Jacobian:
    // grad_{X2} C = [20.479063, 8.989792, 20.020839]^T
    // Whereas the zero-th order right Jacobian approximation (J_r \approx I) evaluated in M3.1 gave [20.2458, 9.4979, 20.0000]^T.
    const Twist2D& g2 = grad_res.gradients[n2];
    TEST_ASSERT(std::abs(g2.vx - 20.479063) < 1e-3, "Exact numerical Riemannian gradient on X2 vx == 20.479063");
    TEST_ASSERT(std::abs(g2.vy - 8.989792) < 1e-3, "Exact numerical Riemannian gradient on X2 vy == 8.989792");
    TEST_ASSERT(std::abs(g2.omega - 20.020839) < 1e-3, "Exact numerical Riemannian gradient on X2 omega == 20.020839");

    // Asymptotic agreement: For small residual displacements, exact gradient asymptotically converges to Omega * r
    poses[n2] = SE2(1.002, 0.001, 0.0005);
    auto grad_small = evaluate_objective_numerical_gradient(graph, poses, 1e-6);
    TEST_ASSERT(grad_small.valid, "Small-displacement numerical gradient valid");
    // For small displacement: r ~= [0.002, 0.001, 0.0005]^T, Omega * r = [0.2, 0.1, 0.2]^T
    TEST_ASSERT(std::abs(grad_small.gradients[n2].vx - 0.200) < 1e-2, "Small-displacement gradient vx matches Omega * r");
    TEST_ASSERT(std::abs(grad_small.gradients[n2].vy - 0.100) < 1e-2, "Small-displacement gradient vy matches Omega * r");
    TEST_ASSERT(std::abs(grad_small.gradients[n2].omega - 0.200) < 1e-2, "Small-displacement gradient omega matches Omega * r");

    // At exact measurement pose X2 = (1.0, 0, 0), gradient on X2 must be identically zero
    poses[n2] = SE2(1.0, 0.0, 0.0);
    auto grad_zero = evaluate_objective_numerical_gradient(graph, poses, 1e-6);
    TEST_ASSERT(grad_zero.valid, "Numerical gradient at minimum valid");
    TEST_ASSERT(std::abs(grad_zero.gradients[n2].vx) < 1e-5, "Gradient vx at minimum is zero");
    TEST_ASSERT(std::abs(grad_zero.gradients[n2].vy) < 1e-5, "Gradient vy at minimum is zero");
    TEST_ASSERT(std::abs(grad_zero.gradients[n2].omega) < 1e-5, "Gradient omega at minimum is zero");
}

int main() {
    std::cout << "===============================================================\n";
    std::cout << " AstroMesh-X: Pose-Graph Objective & Gradient Reference Suite\n";
    std::cout << "===============================================================\n";

    test_zero_residual_consistent_measurement();
    test_hand_calculated_worked_example();
    test_global_frame_gauge_invariance();
    test_edge_eligibility_policy();
    test_invalid_graph_data_rejection();
    test_numerical_gradient_reference();

    std::cout << "===============================================================\n";
    std::cout << " Total assertions: " << g_total_assertions << "\n";
    std::cout << " Failed assertions: " << g_failed_assertions << "\n";
    std::cout << "===============================================================\n";

    return (g_failed_assertions == 0) ? 0 : 1;
}
