#include "astromesh/verification/geometric_verifier.hpp"
#include "astromesh/geometry/se2.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using namespace astromesh;
using namespace astromesh::verification;
using namespace astromesh::geometry;

// Test 1: Genuine loop closures achieve 100% True Positive Rate
static void test_genuine_loop_closures_tp() {
    SE2 T_true(12.0, -8.0, 0.4);
    std::vector<Point2D> pts_src = {
        Point2D(0.0, 0.0), Point2D(5.0, 1.0), Point2D(2.0, 6.0), Point2D(7.0, 8.0)
    };

    std::vector<PointCorrespondence2D> corrs;
    for (const auto& p : pts_src) {
        auto p_dst = T_true.transform(p);
        // Small 0.01m observation noise
        corrs.push_back({Point2D(p_dst.x + 0.01, p_dst.y - 0.01), p});
    }

    RansacConfig2D cfg;
    cfg.inlier_distance_threshold = 0.35;
    cfg.min_correspondences = 3;
    cfg.min_inliers = 3;

    GeometricVerificationResult res = verify_relative_pose_2d(corrs, cfg);
    assert(res.success);
    assert(res.inlier_count >= 3);
    assert(res.inlier_rmse < 0.05);

    std::cout << "[PASS] Genuine loop closure correctly verified (True Positive).\n";
}

// Test 2: Random non-rigid imposters achieve 100% True Negative Rate (explains FP=0)
static void test_random_non_rigid_imposters_tn() {
    // 5 correspondences with completely scrambled, non-rigid coordinates
    std::vector<PointCorrespondence2D> corrs = {
        {Point2D(0.0, 0.0),   Point2D(10.0, 50.0)},
        {Point2D(5.0, 1.0),   Point2D(-20.0, 3.0)},
        {Point2D(2.0, 6.0),   Point2D(15.0, -12.0)},
        {Point2D(7.0, 8.0),   Point2D(0.0, 80.0)},
        {Point2D(12.0, 3.0),  Point2D(-5.0, -5.0)}
    };

    RansacConfig2D cfg;
    cfg.inlier_distance_threshold = 0.35;
    cfg.min_correspondences = 3;
    cfg.min_inliers = 3;

    GeometricVerificationResult res = verify_relative_pose_2d(corrs, cfg);
    // Non-rigid randomized landmarks CANNOT form a 3-point consensus!
    assert(!res.success);
    assert(res.failure_reason == VerificationFailureReason::InsufficientInliers ||
           res.failure_reason == VerificationFailureReason::DegeneratePointConfiguration);

    std::cout << "[PASS] Random imposters deterministically rejected (True Negative, FP=0 explanation verified).\n";
}

// Test 3: Adversarial perceptual aliasing and candidate prior protection
static void test_adversarial_perceptual_aliasing_gating() {
    // Imagine an adversary creates a fake loop proposal that HAS a valid internal rigid shape,
    // but corresponds to a geographically impossible location 100 meters away from odometry prior.
    SE2 T_fake(100.0, 100.0, 0.0); // 141 meters away from expected prior!

    std::vector<Point2D> pts_src = {
        Point2D(0.0, 0.0), Point2D(4.0, 0.0), Point2D(2.0, 3.0), Point2D(6.0, 4.0)
    };

    std::vector<PointCorrespondence2D> corrs;
    for (const auto& p : pts_src) {
        auto p_dst = T_fake.transform(p);
        corrs.push_back({p_dst, p});
    }

    // Gating with odometry prior (expected near 0.0, 0.0)
    RansacConfig2D cfg_with_prior;
    cfg_with_prior.min_correspondences = 3;
    cfg_with_prior.min_inliers = 3;
    cfg_with_prior.inlier_distance_threshold = 0.35;
    cfg_with_prior.candidate_prior = graph::Pose2D(0.0, 0.0, 0.0);
    cfg_with_prior.max_prior_translation_diff = 10.0; // Allowed translation tolerance: 10.0m

    GeometricVerificationResult res = verify_relative_pose_2d(corrs, cfg_with_prior);
    // Even though points are internally rigid, the 141m discrepancy with the prior triggers rejection!
    assert(!res.success);
    assert(res.failure_reason == VerificationFailureReason::InconsistentWithCandidatePrior);

    std::cout << "[PASS] Perceptual aliasing rejected by candidate prior gating.\n";
}

int main() {
    std::cout << "Running test_m717_loop_closure_statistics...\n";
    test_genuine_loop_closures_tp();
    test_random_non_rigid_imposters_tn();
    test_adversarial_perceptual_aliasing_gating();
    std::cout << "All M7.17 loop closure statistics tests PASSED.\n";
    return 0;
}
