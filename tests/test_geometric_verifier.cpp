#include "astromesh/verification/geometric_verifier.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/core/landmark_key.hpp"

#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <limits>
#include <cassert>

using namespace astromesh::verification;
using namespace astromesh::graph;
using astromesh::core::LandmarkKey;

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

// Helper to generate transformed points: p_tgt = R(theta) * p_src + t
static std::vector<PointCorrespondence2D> generate_synthetic_correspondences(
    const std::vector<Point2D>& source_points,
    double tx, double ty, double theta,
    double noise_sigma = 0.0,
    uint64_t seed = 42) {

    std::mt19937_64 rng(seed);
    std::normal_distribution<double> dist(0.0, noise_sigma > 0.0 ? noise_sigma : 1.0);

    double c = std::cos(theta);
    double s = std::sin(theta);

    std::vector<PointCorrespondence2D> correspondences;
    correspondences.reserve(source_points.size());

    for (size_t i = 0; i < source_points.size(); ++i) {
        const auto& p_src = source_points[i];
        double noise_x = (noise_sigma > 0.0) ? dist(rng) : 0.0;
        double noise_y = (noise_sigma > 0.0) ? dist(rng) : 0.0;
        double x_tgt = c * p_src.x - s * p_src.y + tx + noise_x;
        double y_tgt = s * p_src.x + c * p_src.y + ty + noise_y;

        LandmarkKey key(1, 1, static_cast<uint32_t>(i + 1), 0xABCD1234 + i);
        correspondences.emplace_back(p_src, Point2D(x_tgt, y_tgt), key);
    }

    return correspondences;
}

void test_exact_known_rigid_transforms() {
    std::cout << "[TEST 1] Exact Known Rigid Transforms (Noise-Free)...\n";

    std::vector<Point2D> src_pts = {
        {0.0, 0.0}, {2.0, 0.0}, {2.0, 3.0}, {0.0, 3.0},
        {1.0, 1.5}, {-1.0, 2.0}, {3.0, -1.0}, {4.0, 2.0}
    };

    // Case 1: Pure Translation
    {
        double true_tx = 4.5, true_ty = -3.2, true_theta = 0.0;
        auto corrs = generate_synthetic_correspondences(src_pts, true_tx, true_ty, true_theta);
        auto res = verify_relative_pose_2d(corrs);

        TEST_ASSERT(res.success, "Pure translation verification must succeed");
        TEST_ASSERT(std::abs(res.estimated_transform.x - true_tx) < 1e-6, "tx matches");
        TEST_ASSERT(std::abs(res.estimated_transform.y - true_ty) < 1e-6, "ty matches");
        TEST_ASSERT(std::abs(res.estimated_transform.theta - true_theta) < 1e-6, "theta matches");
        TEST_ASSERT(res.inlier_count == src_pts.size(), "All points are inliers");
        TEST_ASSERT(res.inlier_rmse < 1e-9, "Residual RMSE zero for noise-free data");
    }

    // Case 2: Pure Rotation (pi/4)
    {
        double true_tx = 0.0, true_ty = 0.0, true_theta = 0.7853981633974483;
        auto corrs = generate_synthetic_correspondences(src_pts, true_tx, true_ty, true_theta);
        auto res = verify_relative_pose_2d(corrs);

        TEST_ASSERT(res.success, "Pure rotation verification must succeed");
        TEST_ASSERT(std::abs(res.estimated_transform.x - true_tx) < 1e-6, "tx matches");
        TEST_ASSERT(std::abs(res.estimated_transform.y - true_ty) < 1e-6, "ty matches");
        TEST_ASSERT(std::abs(res.estimated_transform.theta - true_theta) < 1e-6, "theta matches");
    }

    // Case 3: Combined Translation and Rotation
    {
        double true_tx = -7.8, true_ty = 12.4, true_theta = -1.25;
        auto corrs = generate_synthetic_correspondences(src_pts, true_tx, true_ty, true_theta);
        auto res = verify_relative_pose_2d(corrs);

        TEST_ASSERT(res.success, "Combined transform verification must succeed");
        TEST_ASSERT(std::abs(res.estimated_transform.x - true_tx) < 1e-6, "tx matches");
        TEST_ASSERT(std::abs(res.estimated_transform.y - true_ty) < 1e-6, "ty matches");
        TEST_ASSERT(std::abs(res.estimated_transform.theta - true_theta) < 1e-6, "theta matches");
        TEST_ASSERT(res.inlier_ratio == 1.0, "100% inlier ratio");
    }
}

void test_noisy_inliers() {
    std::cout << "[TEST 2] Small Measurement Noise Handling...\n";

    std::vector<Point2D> src_pts = {
        {1.0, 1.0}, {5.0, 2.0}, {8.0, 7.0}, {3.0, 9.0},
        {6.0, 4.0}, {2.0, 6.0}, {9.0, 1.0}, {4.0, 5.0}
    };

    double true_tx = 3.0, true_ty = -2.0, true_theta = 0.50;
    double noise_sigma = 0.02; // 2 cm noise
    auto corrs = generate_synthetic_correspondences(src_pts, true_tx, true_ty, true_theta, noise_sigma, 999);

    RansacConfig2D cfg;
    cfg.inlier_distance_threshold = 0.10; // 10 cm inlier threshold
    auto res = verify_relative_pose_2d(corrs, cfg);

    TEST_ASSERT(res.success, "Noisy inliers verification must succeed");
    TEST_ASSERT(res.inlier_count == src_pts.size(), "All points retained as inliers");
    TEST_ASSERT(std::abs(res.estimated_transform.x - true_tx) < 0.05, "tx within noise tolerance");
    TEST_ASSERT(std::abs(res.estimated_transform.y - true_ty) < 0.05, "ty within noise tolerance");
    TEST_ASSERT(std::abs(res.estimated_transform.theta - true_theta) < 0.02, "theta within noise tolerance");
    TEST_ASSERT(res.inlier_rmse > 0.0 && res.inlier_rmse < 0.05, "RMSE accurately reflects noise level");
}

void test_outliers_mixed_with_inliers() {
    std::cout << "[TEST 3] Outliers Mixed with Inliers (RANSAC Robustness)...\n";

    // 10 true inliers
    std::vector<Point2D> inlier_pts = {
        {0.0, 0.0}, {3.0, 1.0}, {6.0, 0.0}, {8.0, 4.0}, {5.0, 6.0},
        {1.0, 5.0}, {4.0, 3.0}, {7.0, 2.0}, {2.0, 7.0}, {9.0, 5.0}
    };

    double true_tx = 5.0, true_ty = -3.0, true_theta = 0.80;
    auto corrs = generate_synthetic_correspondences(inlier_pts, true_tx, true_ty, true_theta, 0.01, 123);

    // Append 5 gross outliers (uncorrelated positions)
    corrs.emplace_back(Point2D(10.0, 10.0), Point2D(-20.0, 5.0));
    corrs.emplace_back(Point2D(-5.0, 2.0), Point2D(30.0, -10.0));
    corrs.emplace_back(Point2D(0.0, 8.0), Point2D(15.0, 25.0));
    corrs.emplace_back(Point2D(12.0, -3.0), Point2D(-10.0, -15.0));
    corrs.emplace_back(Point2D(7.0, 11.0), Point2D(0.0, -25.0));

    TEST_ASSERT(corrs.size() == 15, "Total 15 correspondences (10 inliers + 5 outliers)");

    RansacConfig2D cfg;
    cfg.inlier_distance_threshold = 0.20;
    cfg.min_correspondences = 4;
    cfg.min_inliers = 5;
    cfg.min_inlier_ratio = 0.50; // 10/15 = 66.7% > 50%
    cfg.max_iterations = 200;
    cfg.random_seed = 42;

    auto res = verify_relative_pose_2d(corrs, cfg);

    TEST_ASSERT(res.success, "RANSAC must succeed with 66% inlier ratio");
    TEST_ASSERT(res.inlier_count == 10, "Exactly 10 inliers identified");
    TEST_ASSERT(std::abs(res.inlier_ratio - (10.0 / 15.0)) < 1e-6, "Inlier ratio matches 10/15");

    // Check that all 10 inliers are masked true and all 5 outliers are false
    for (size_t i = 0; i < 10; ++i) {
        TEST_ASSERT(res.inlier_mask[i] == true, "True inlier must be masked true");
    }
    for (size_t i = 10; i < 15; ++i) {
        TEST_ASSERT(res.inlier_mask[i] == false, "Outlier must be masked false");
    }

    TEST_ASSERT(std::abs(res.estimated_transform.x - true_tx) < 0.05, "tx accurate despite outliers");
    TEST_ASSERT(std::abs(res.estimated_transform.y - true_ty) < 0.05, "ty accurate despite outliers");
    TEST_ASSERT(std::abs(res.estimated_transform.theta - true_theta) < 0.02, "theta accurate despite outliers");
}

void test_insufficient_correspondences() {
    std::cout << "[TEST 4] Insufficient Correspondences Gating...\n";

    std::vector<PointCorrespondence2D> few_corrs = {
        {Point2D(0.0, 0.0), Point2D(1.0, 1.0)},
        {Point2D(1.0, 0.0), Point2D(2.0, 1.0)},
        {Point2D(0.0, 1.0), Point2D(1.0, 2.0)}
    };

    RansacConfig2D cfg;
    cfg.min_correspondences = 4;

    auto res = verify_relative_pose_2d(few_corrs, cfg);
    TEST_ASSERT(!res.success, "Verification must fail with < min_correspondences");
    TEST_ASSERT(res.failure_reason == VerificationFailureReason::InsufficientCorrespondences,
                "Reason must be InsufficientCorrespondences");
    TEST_ASSERT(res.inlier_count == 0, "Inlier count 0");
}

void test_all_outliers_rejection() {
    std::cout << "[TEST 5] All-Outlier Inconsistent Data Rejection...\n";

    // Random independent noise where no rigid motion exists
    std::mt19937_64 rng(777);
    std::uniform_real_distribution<double> dist(-50.0, 50.0);

    std::vector<PointCorrespondence2D> random_corrs;
    for (size_t i = 0; i < 12; ++i) {
        random_corrs.emplace_back(Point2D(dist(rng), dist(rng)), Point2D(dist(rng), dist(rng)));
    }

    RansacConfig2D cfg;
    cfg.min_inliers = 4;
    cfg.inlier_distance_threshold = 0.20;

    auto res = verify_relative_pose_2d(random_corrs, cfg);
    TEST_ASSERT(!res.success, "All-outlier correspondence set must be rejected");
    TEST_ASSERT(res.failure_reason == VerificationFailureReason::InsufficientInliers ||
                res.failure_reason == VerificationFailureReason::LowInlierRatio,
                "Failure reason must indicate insufficient inliers");
}

void test_low_inlier_ratio_rejection() {
    std::cout << "[TEST 6] Low Inlier Ratio Rejection...\n";

    // 4 true inliers + 16 outliers (ratio = 4/20 = 20% < 50% min threshold)
    std::vector<Point2D> inliers = {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}};
    auto corrs = generate_synthetic_correspondences(inliers, 1.0, 1.0, 0.0);

    std::mt19937_64 rng(555);
    std::uniform_real_distribution<double> dist(-100.0, 100.0);
    for (size_t i = 0; i < 16; ++i) {
        corrs.emplace_back(Point2D(dist(rng), dist(rng)), Point2D(dist(rng), dist(rng)));
    }

    RansacConfig2D cfg;
    cfg.min_inliers = 4;
    cfg.min_inlier_ratio = 0.50; // Requires 50% inliers

    auto res = verify_relative_pose_2d(corrs, cfg);
    TEST_ASSERT(!res.success, "Verification must fail when inlier ratio is below threshold");
    TEST_ASSERT(res.failure_reason == VerificationFailureReason::LowInlierRatio,
                "Failure reason must be LowInlierRatio");
}

void test_degenerate_configurations() {
    std::cout << "[TEST 7] Degenerate Point Configurations...\n";

    // Coincident points (all points at (1.0, 1.0))
    std::vector<PointCorrespondence2D> coincident_corrs = {
        {Point2D(1.0, 1.0), Point2D(5.0, 5.0)},
        {Point2D(1.0, 1.0), Point2D(5.0, 5.0)},
        {Point2D(1.0, 1.0), Point2D(5.0, 5.0)},
        {Point2D(1.0, 1.0), Point2D(5.0, 5.0)}
    };

    auto res = verify_relative_pose_2d(coincident_corrs);
    TEST_ASSERT(!res.success, "Coincident points must fail verification");
    TEST_ASSERT(res.failure_reason == VerificationFailureReason::DegeneratePointConfiguration ||
                res.failure_reason == VerificationFailureReason::InsufficientInliers,
                "Degenerate points flagged");
}

void test_non_finite_inputs() {
    std::cout << "[TEST 8] Non-Finite Coordinate Inputs (NaN / Inf)...\n";

    double nan_val = std::numeric_limits<double>::quiet_NaN();
    double inf_val = std::numeric_limits<double>::infinity();

    std::vector<PointCorrespondence2D> nan_corrs = {
        {Point2D(0.0, 0.0), Point2D(1.0, 1.0)},
        {Point2D(nan_val, 0.0), Point2D(2.0, 1.0)},
        {Point2D(2.0, 0.0), Point2D(3.0, 1.0)},
        {Point2D(3.0, 0.0), Point2D(4.0, 1.0)}
    };

    auto res = verify_relative_pose_2d(nan_corrs);
    TEST_ASSERT(!res.success, "NaN input must be rejected");
    TEST_ASSERT(res.failure_reason == VerificationFailureReason::NonFiniteInputCoordinates,
                "Reason must be NonFiniteInputCoordinates");

    std::vector<PointCorrespondence2D> inf_corrs = {
        {Point2D(0.0, 0.0), Point2D(1.0, 1.0)},
        {Point2D(1.0, 0.0), Point2D(2.0, inf_val)},
        {Point2D(2.0, 0.0), Point2D(3.0, 1.0)},
        {Point2D(3.0, 0.0), Point2D(4.0, 1.0)}
    };

    res = verify_relative_pose_2d(inf_corrs);
    TEST_ASSERT(!res.success, "Inf input must be rejected");
    TEST_ASSERT(res.failure_reason == VerificationFailureReason::NonFiniteInputCoordinates,
                "Reason must be NonFiniteInputCoordinates");
}

void test_configuration_validation() {
    std::cout << "[TEST 9] RansacConfig2D Parameter Bounds Validation...\n";

    std::vector<Point2D> pts = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
    auto corrs = generate_synthetic_correspondences(pts, 0.0, 0.0, 0.0);

    // Negative threshold
    RansacConfig2D bad_cfg1;
    bad_cfg1.inlier_distance_threshold = -0.5;
    auto res = verify_relative_pose_2d(corrs, bad_cfg1);
    TEST_ASSERT(!res.success && res.failure_reason == VerificationFailureReason::InvalidConfiguration,
                "Negative distance threshold rejected");

    // Zero max iterations
    RansacConfig2D bad_cfg2;
    bad_cfg2.max_iterations = 0;
    res = verify_relative_pose_2d(corrs, bad_cfg2);
    TEST_ASSERT(!res.success && res.failure_reason == VerificationFailureReason::InvalidConfiguration,
                "Zero max iterations rejected");

    // Invalid inlier ratio (> 1.0)
    RansacConfig2D bad_cfg3;
    bad_cfg3.min_inlier_ratio = 1.5;
    res = verify_relative_pose_2d(corrs, bad_cfg3);
    TEST_ASSERT(!res.success && res.failure_reason == VerificationFailureReason::InvalidConfiguration,
                "Ratio > 1.0 rejected");
}

void test_determinism_across_runs() {
    std::cout << "[TEST 10] Determinism with Fixed Random Seed...\n";

    std::vector<Point2D> inliers = {
        {0.0, 0.0}, {3.0, 1.0}, {6.0, 0.0}, {8.0, 4.0}, {5.0, 6.0},
        {1.0, 5.0}, {4.0, 3.0}, {7.0, 2.0}, {2.0, 7.0}, {9.0, 5.0}
    };
    auto corrs = generate_synthetic_correspondences(inliers, 2.5, -4.0, 0.60, 0.02, 111);
    // Add outliers
    corrs.emplace_back(Point2D(10.0, 10.0), Point2D(0.0, 0.0));
    corrs.emplace_back(Point2D(-5.0, -5.0), Point2D(20.0, 20.0));

    RansacConfig2D cfg;
    cfg.random_seed = 12345;

    auto res1 = verify_relative_pose_2d(corrs, cfg);
    auto res2 = verify_relative_pose_2d(corrs, cfg);

    TEST_ASSERT(res1.success && res2.success, "Both runs succeed");
    TEST_ASSERT(res1.estimated_transform == res2.estimated_transform, "Transforms match bit-for-bit");
    TEST_ASSERT(res1.inlier_count == res2.inlier_count, "Inlier counts match");
    TEST_ASSERT(res1.inlier_mask == res2.inlier_mask, "Inlier masks match");
    TEST_ASSERT(res1.inlier_rmse == res2.inlier_rmse, "Inlier RMSE matches");
}

void test_angle_normalization_in_estimation() {
    std::cout << "[TEST 11] Angle Normalization in Estimation...\n";

    std::vector<Point2D> pts = {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}};
    // Rotate by 3*pi/2 (= 270 deg) -> normalized to -pi/2
    constexpr double PI = 3.14159265358979323846;
    auto corrs = generate_synthetic_correspondences(pts, 0.0, 0.0, 1.5 * PI);

    auto res = verify_relative_pose_2d(corrs);
    TEST_ASSERT(res.success, "3*pi/2 rotation succeeds");
    TEST_ASSERT(std::abs(res.estimated_transform.theta - (-0.5 * PI)) < 1e-6,
                "Angle correctly normalized to -pi/2");
}

void test_pose_graph_integration_and_gating() {
    std::cout << "[TEST 12] PoseGraph Edge Promotion and Rejection Gating...\n";

    PoseGraph graph;
    NodeId r1(1, 10);
    NodeId r2(2, 20);
    graph.add_node(KeyframeNode(r1, Pose2D(0.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(r2, Pose2D(10.0, 5.0, 0.0)));

    // 1. Create a candidate loop-closure edge initialized as Unverified
    uint64_t edge_id = 100;
    Pose2D coarse_initial_pose(9.5, 4.8, 0.1);
    PoseGraphEdge candidate_edge(edge_id, r1, r2, EdgeType::InterRoverLoopClosure,
                                coarse_initial_pose,
                                InformationMatrix2D::diagonal(20.0, 200.0),
                                EdgeVerificationStatus::Unverified);
    TEST_ASSERT(graph.add_edge(candidate_edge), "Candidate edge added to graph");
    TEST_ASSERT(graph.get_edge(edge_id).status == EdgeVerificationStatus::Unverified,
                "Candidate begins Unverified");

    // 2. Case A: Successful geometric verification promotes edge to Verified with refined pose
    std::vector<Point2D> landmark_pts = {
        {1.0, 2.0}, {4.0, 1.0}, {5.0, 6.0}, {2.0, 7.0}, {8.0, 3.0}
    };
    double true_tx = 10.0, true_ty = 5.0, true_theta = 0.25;
    auto valid_corrs = generate_synthetic_correspondences(landmark_pts, true_tx, true_ty, true_theta, 0.01, 789);

    auto res_success = gate_and_promote_loop_closure(graph, edge_id, valid_corrs);

    TEST_ASSERT(res_success.success, "Geometric verification succeeds on consistent landmarks");
    TEST_ASSERT(graph.get_edge(edge_id).status == EdgeVerificationStatus::Verified,
                "Edge status promoted to Verified");
    TEST_ASSERT(std::abs(graph.get_edge(edge_id).relative_pose.x - true_tx) < 0.05,
                "Edge relative_pose updated with refined tx");
    TEST_ASSERT(std::abs(graph.get_edge(edge_id).relative_pose.y - true_ty) < 0.05,
                "Edge relative_pose updated with refined ty");
    TEST_ASSERT(std::abs(graph.get_edge(edge_id).relative_pose.theta - true_theta) < 0.02,
                "Edge relative_pose updated with refined theta");

    // 3. Case B: Failed geometric verification rejects candidate edge
    uint64_t bad_edge_id = 200;
    NodeId r1_b(1, 11);
    NodeId r2_b(2, 21);
    graph.add_node(KeyframeNode(r1_b, Pose2D(1.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(r2_b, Pose2D(20.0, 5.0, 0.0)));

    PoseGraphEdge bad_candidate(bad_edge_id, r1_b, r2_b, EdgeType::InterRoverLoopClosure,
                               Pose2D(0.0, 0.0, 0.0),
                               InformationMatrix2D::diagonal(20.0, 200.0),
                               EdgeVerificationStatus::Unverified);
    graph.add_edge(bad_candidate);
    TEST_ASSERT(graph.get_edge(bad_edge_id).status == EdgeVerificationStatus::Unverified,
                "Second candidate begins Unverified");

    // Feed inconsistent / all-outlier correspondences (perceptual aliasing)
    std::vector<PointCorrespondence2D> outlier_corrs = {
        {Point2D(0.0, 0.0), Point2D(100.0, 20.0)},
        {Point2D(5.0, 5.0), Point2D(-30.0, 80.0)},
        {Point2D(10.0, 0.0), Point2D(10.0, -90.0)},
        {Point2D(2.0, 8.0), Point2D(-50.0, -10.0)}
    };

    auto res_fail = gate_and_promote_loop_closure(graph, bad_edge_id, outlier_corrs);

    TEST_ASSERT(!res_fail.success, "Verification fails on inconsistent landmarks");
    TEST_ASSERT(graph.get_edge(bad_edge_id).status == EdgeVerificationStatus::Rejected,
                "Failed candidate is marked Rejected, NEVER Verified");

    // 4. Verify that verified edge remains Verified
    TEST_ASSERT(graph.get_edge(edge_id).status == EdgeVerificationStatus::Verified,
                "Original verified edge remains intact");
}

int main() {
    std::cout << "======================================================================\n";
    std::cout << " AstroMesh-X Phase 2 M2.2: Geometric Verification Unit Tests\n";
    std::cout << "======================================================================\n";

    test_exact_known_rigid_transforms();
    test_noisy_inliers();
    test_outliers_mixed_with_inliers();
    test_insufficient_correspondences();
    test_all_outliers_rejection();
    test_low_inlier_ratio_rejection();
    test_degenerate_configurations();
    test_non_finite_inputs();
    test_configuration_validation();
    test_determinism_across_runs();
    test_angle_normalization_in_estimation();
    test_pose_graph_integration_and_gating();

    std::cout << "======================================================================\n";
    std::cout << " Assertions checked : " << g_total_assertions << "\n";
    std::cout << " Assertions failed  : " << g_failed_assertions << "\n";
    std::cout << "======================================================================\n";

    if (g_failed_assertions > 0) {
        std::cerr << "FAILED: Geometric verification tests encountered " << g_failed_assertions << " failures.\n";
        return 1;
    }

    std::cout << "SUCCESS: All Geometric Verification tests PASSED.\n";
    return 0;
}
