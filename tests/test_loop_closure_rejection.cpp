#include "astromesh/verification/geometric_verifier.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/graph/disjoint_set.hpp"
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

// Helper to create synthetic points transformed by T = (tx, ty, theta)
static std::vector<PointCorrespondence2D> make_transformed_correspondences(
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

        LandmarkKey key(1, 1, static_cast<uint32_t>(i + 1), 0xAA000000 + i);
        correspondences.emplace_back(p_src, Point2D(x_tgt, y_tgt), key);
    }

    return correspondences;
}

// -----------------------------------------------------------------------------
// Scenario 1: Perceptual Aliasing
// LandmarkKey identifiers match, but spatial coordinates are physically scrambled.
// Proves matching LandmarkKey alone does NOT grant geometric verification.
// -----------------------------------------------------------------------------
void test_perceptual_aliasing_rejection() {
    std::cout << "[SCENARIO 1] Perceptual Aliasing Rejection...\n";

    std::vector<PointCorrespondence2D> correspondences;
    // 10 pairs with identical LandmarkKeys, but scrambled coordinates that do not fit any SE(2) transform
    for (size_t i = 0; i < 10; ++i) {
        Point2D src(static_cast<double>(i) * 2.0, static_cast<double>(i % 3) * 1.5);
        // Scramble target point completely
        Point2D tgt(static_cast<double>((i * 7 + 3) % 13) * 3.0, static_cast<double>((i * 11 + 5) % 17) * 2.0);
        LandmarkKey key(1, 1, static_cast<uint32_t>(i + 1), 0xCAFE0000 + i);
        correspondences.emplace_back(src, tgt, key);
    }

    RansacConfig2D config;
    config.inlier_distance_threshold = 0.20;
    config.min_correspondences = 4;
    config.min_inliers = 4;
    config.min_inlier_ratio = 0.50;
    config.max_iterations = 100;
    config.random_seed = 12345;

    auto result = verify_relative_pose_2d(correspondences, config);
    TEST_ASSERT(!result.success, "Perceptual aliasing must fail geometric verification");
    TEST_ASSERT(result.failure_reason == VerificationFailureReason::InsufficientInliers ||
                result.failure_reason == VerificationFailureReason::LowInlierRatio,
                "Perceptual aliasing failure reason must indicate inlier deficiency");
    TEST_ASSERT(result.inlier_count < config.min_inliers || result.inlier_ratio < config.min_inlier_ratio,
                "Inliers must not satisfy consensus threshold");

    // Test integration with PoseGraph gating
    PoseGraph graph;
    NodeId r1_n1(1, 1);
    NodeId r2_n1(2, 1);
    graph.add_node(KeyframeNode(r1_n1, Pose2D(0.0, 0.0, 0.0), 100.0));
    graph.add_node(KeyframeNode(r2_n1, Pose2D(10.0, 0.0, 0.0), 101.0));

    PoseGraphEdge candidate(1, r1_n1, r2_n1, EdgeType::InterRoverLoopClosure,
                           Pose2D(10.0, 0.0, 0.0), InformationMatrix2D(),
                           EdgeVerificationStatus::Unverified);
    graph.add_edge(candidate);

    auto gate_res = gate_and_promote_loop_closure(graph, 1, correspondences, config);
    TEST_ASSERT(!gate_res.success, "Perceptually aliased edge must fail gating");
    const auto* edge_after = graph.find_edge(1);
    TEST_ASSERT(edge_after != nullptr, "Edge must exist in graph");
    TEST_ASSERT(edge_after->status == EdgeVerificationStatus::Rejected,
                "Perceptually aliased edge must be transitioned to Rejected status");
}

// -----------------------------------------------------------------------------
// Scenario 2: Random False Correspondences
// Completely uncorrelated uniform random points in source and target frames.
// -----------------------------------------------------------------------------
void test_random_false_correspondences_rejection() {
    std::cout << "[SCENARIO 2] Random False Correspondences Rejection...\n";

    std::mt19937_64 rng(9999);
    std::uniform_real_distribution<double> dist(-50.0, 50.0);

    std::vector<PointCorrespondence2D> correspondences;
    for (size_t i = 0; i < 20; ++i) {
        Point2D src(dist(rng), dist(rng));
        Point2D tgt(dist(rng), dist(rng));
        correspondences.emplace_back(src, tgt);
    }

    RansacConfig2D config;
    config.inlier_distance_threshold = 0.20;
    config.min_correspondences = 4;
    config.min_inliers = 4;
    config.min_inlier_ratio = 0.40;
    config.max_iterations = 100;

    auto result = verify_relative_pose_2d(correspondences, config);
    TEST_ASSERT(!result.success, "Random false correspondences must fail verification");
    TEST_ASSERT(result.failure_reason == VerificationFailureReason::InsufficientInliers ||
                result.failure_reason == VerificationFailureReason::LowInlierRatio,
                "Random correspondences must fail inlier test");
}

// -----------------------------------------------------------------------------
// Scenario 3: Geometrically Inconsistent Loop Closure
// Point measurements are mutually consistent with transform T_actual, but candidate
// prior pose in graph conflicts with T_actual beyond tolerance.
// -----------------------------------------------------------------------------
void test_candidate_prior_inconsistency_rejection() {
    std::cout << "[SCENARIO 3] Candidate Prior Inconsistency Rejection...\n";

    std::vector<Point2D> src_pts = {
        {0.0, 0.0}, {2.0, 0.0}, {4.0, 1.0}, {1.0, 3.0}, {3.0, 4.0}, {5.0, 2.0}
    };
    // True transform: dx = 5.0, dy = -3.0, theta = 0.3 rad
    double true_tx = 5.0;
    double true_ty = -3.0;
    double true_theta = 0.3;
    auto correspondences = make_transformed_correspondences(src_pts, true_tx, true_ty, true_theta);

    // Case 3a: Prior translation conflict (claimed tx=0, ty=0; tolerance 1.0 m)
    RansacConfig2D config_trans_conflict;
    config_trans_conflict.candidate_prior = Pose2D(0.0, 0.0, 0.3);
    config_trans_conflict.max_prior_translation_diff = 1.0; // 1.0 m max allowed, but true is ~5.83 m
    config_trans_conflict.max_prior_rotation_diff = 0.1;

    auto res_trans = verify_relative_pose_2d(correspondences, config_trans_conflict);
    TEST_ASSERT(!res_trans.success, "Prior translation conflict must be rejected");
    TEST_ASSERT(res_trans.failure_reason == VerificationFailureReason::InconsistentWithCandidatePrior,
                "Must report InconsistentWithCandidatePrior failure reason");

    // Case 3b: Prior rotation conflict (claimed theta = -1.0 rad; tolerance 0.1 rad)
    RansacConfig2D config_rot_conflict;
    config_rot_conflict.candidate_prior = Pose2D(5.0, -3.0, -1.0);
    config_rot_conflict.max_prior_translation_diff = 1.0;
    config_rot_conflict.max_prior_rotation_diff = 0.2; // 0.2 rad max, but true diff is 1.3 rad

    auto res_rot = verify_relative_pose_2d(correspondences, config_rot_conflict);
    TEST_ASSERT(!res_rot.success, "Prior rotation conflict must be rejected");
    TEST_ASSERT(res_rot.failure_reason == VerificationFailureReason::InconsistentWithCandidatePrior,
                "Must report InconsistentWithCandidatePrior failure reason");

    // Case 3c: Gating integration with PoseGraph prior
    PoseGraph graph;
    NodeId n1(1, 1), n2(2, 1);
    graph.add_node(KeyframeNode(n1, Pose2D(0.0, 0.0, 0.0), 100.0));
    graph.add_node(KeyframeNode(n2, Pose2D(0.0, 0.0, 0.0), 101.0));

    // Candidate edge claims relative pose (0, 0, 0)
    PoseGraphEdge edge(42, n1, n2, EdgeType::InterRoverLoopClosure,
                       Pose2D(0.0, 0.0, 0.0), InformationMatrix2D(),
                       EdgeVerificationStatus::Unverified);
    graph.add_edge(edge);

    RansacConfig2D gate_config;
    gate_config.max_prior_translation_diff = 2.0; // edge relative_pose (0,0) used as candidate prior
    gate_config.max_prior_rotation_diff = 0.2;

    auto gate_res = gate_and_promote_loop_closure(graph, 42, correspondences, gate_config);
    TEST_ASSERT(!gate_res.success, "Inconsistent candidate edge must fail gating");
    TEST_ASSERT(gate_res.failure_reason == VerificationFailureReason::InconsistentWithCandidatePrior,
                "Failure reason must be InconsistentWithCandidatePrior");
    TEST_ASSERT(graph.get_edge(42).status == EdgeVerificationStatus::Rejected,
                "Edge must be marked Rejected in graph");
}

// -----------------------------------------------------------------------------
// Scenario 4: Insufficient Correspondences
// Fewer than the configured minimum correspondences.
// -----------------------------------------------------------------------------
void test_insufficient_correspondences_rejection() {
    std::cout << "[SCENARIO 4] Insufficient Correspondences Rejection...\n";

    RansacConfig2D config;
    config.min_correspondences = 4;

    // 0 correspondences
    std::vector<PointCorrespondence2D> empty_corr;
    auto r0 = verify_relative_pose_2d(empty_corr, config);
    TEST_ASSERT(!r0.success, "0 correspondences must fail");
    TEST_ASSERT(r0.failure_reason == VerificationFailureReason::InsufficientCorrespondences,
                "Must report InsufficientCorrespondences for 0");

    // 1 correspondence
    std::vector<PointCorrespondence2D> c1 = {
        PointCorrespondence2D(Point2D(0.0, 0.0), Point2D(1.0, 1.0))
    };
    auto r1 = verify_relative_pose_2d(c1, config);
    TEST_ASSERT(!r1.success, "1 correspondence must fail");
    TEST_ASSERT(r1.failure_reason == VerificationFailureReason::InsufficientCorrespondences,
                "Must report InsufficientCorrespondences for 1");

    // 3 correspondences when min is 4
    std::vector<PointCorrespondence2D> c3 = {
        PointCorrespondence2D(Point2D(0.0, 0.0), Point2D(1.0, 1.0)),
        PointCorrespondence2D(Point2D(1.0, 0.0), Point2D(2.0, 1.0)),
        PointCorrespondence2D(Point2D(0.0, 1.0), Point2D(1.0, 2.0))
    };
    auto r3 = verify_relative_pose_2d(c3, config);
    TEST_ASSERT(!r3.success, "3 correspondences must fail when min is 4");
    TEST_ASSERT(r3.failure_reason == VerificationFailureReason::InsufficientCorrespondences,
                "Must report InsufficientCorrespondences for 3");
}

// -----------------------------------------------------------------------------
// Scenario 5: Degenerate Configurations
// Coincident points, zero spatial variance, and ambiguous samples.
// -----------------------------------------------------------------------------
void test_degenerate_configurations_rejection() {
    std::cout << "[SCENARIO 5] Degenerate Configurations Rejection...\n";

    // 5a: Coincident points in source (all points at the exact same location)
    std::vector<PointCorrespondence2D> coincident_src;
    for (size_t i = 0; i < 5; ++i) {
        coincident_src.emplace_back(Point2D(3.0, 3.0), Point2D(static_cast<double>(i), static_cast<double>(i)));
    }
    RansacConfig2D config;
    auto res_coincident_src = verify_relative_pose_2d(coincident_src, config);
    TEST_ASSERT(!res_coincident_src.success, "Coincident source points must fail");
    TEST_ASSERT(res_coincident_src.failure_reason == VerificationFailureReason::DegeneratePointConfiguration,
                "Coincident source points must report DegeneratePointConfiguration");

    // 5b: Coincident points in target
    std::vector<PointCorrespondence2D> coincident_tgt;
    for (size_t i = 0; i < 5; ++i) {
        coincident_tgt.emplace_back(Point2D(static_cast<double>(i), static_cast<double>(i)), Point2D(4.0, -2.0));
    }
    auto res_coincident_tgt = verify_relative_pose_2d(coincident_tgt, config);
    TEST_ASSERT(!res_coincident_tgt.success, "Coincident target points must fail");
    TEST_ASSERT(res_coincident_tgt.failure_reason == VerificationFailureReason::DegeneratePointConfiguration,
                "Coincident target points must report DegeneratePointConfiguration");

    // 5c: Closed-form estimator on coincident points directly
    std::vector<PointCorrespondence2D> direct_coincident = {
        PointCorrespondence2D(Point2D(1.0, 1.0), Point2D(5.0, 5.0)),
        PointCorrespondence2D(Point2D(1.0, 1.0), Point2D(5.0, 5.0))
    };
    Pose2D out_pose;
    bool est_ok = estimate_rigid_transform_2d(direct_coincident, {0, 1}, out_pose);
    TEST_ASSERT(!est_ok, "Direct estimator must reject coincident point pair");

    // 5d: Ambiguous orientation (zero cross-covariance norm)
    std::vector<PointCorrespondence2D> ambiguous = {
        PointCorrespondence2D(Point2D(1.0, 0.0), Point2D(0.0, 1.0)),
        PointCorrespondence2D(Point2D(-1.0, 0.0), Point2D(0.0, 1.0))
    };
    bool amb_ok = estimate_rigid_transform_2d(ambiguous, {0, 1}, out_pose);
    TEST_ASSERT(!amb_ok, "Estimator must reject degenerate ambiguous target pairing");
}

// -----------------------------------------------------------------------------
// Scenario 6: Outlier-Heavy Input (e.g. 85% gross outliers)
// -----------------------------------------------------------------------------
void test_outlier_heavy_input_rejection() {
    std::cout << "[SCENARIO 6] Outlier-Heavy Input Rejection...\n";

    // 3 true inliers (dx=2, dy=1, theta=0)
    std::vector<Point2D> true_src = {
        {0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}
    };
    auto inliers = make_transformed_correspondences(true_src, 2.0, 1.0, 0.0);

    // 17 gross outliers (85% outliers)
    std::mt19937_64 rng(777);
    std::uniform_real_distribution<double> dist(-30.0, 30.0);
    std::vector<PointCorrespondence2D> mixture = inliers;
    for (size_t i = 0; i < 17; ++i) {
        mixture.emplace_back(Point2D(dist(rng), dist(rng)), Point2D(dist(rng), dist(rng)));
    }
    TEST_ASSERT(mixture.size() == 20, "Mixture should have 20 correspondences");

    // With min_inliers = 4 and min_inlier_ratio = 0.50, mixture must be rejected
    RansacConfig2D strict_config;
    strict_config.min_correspondences = 4;
    strict_config.min_inliers = 4;
    strict_config.min_inlier_ratio = 0.50;
    strict_config.max_iterations = 100;

    auto result = verify_relative_pose_2d(mixture, strict_config);
    TEST_ASSERT(!result.success, "85% outlier mixture must fail under standard gating");
    TEST_ASSERT(result.failure_reason == VerificationFailureReason::InsufficientInliers ||
                result.failure_reason == VerificationFailureReason::LowInlierRatio,
                "Must fail due to inlier deficiency or low inlier ratio");
}

// -----------------------------------------------------------------------------
// Scenario 7: Non-Finite and Invalid Data
// NaN, infinity, invalid thresholds, and invalid configuration parameters.
// -----------------------------------------------------------------------------
void test_non_finite_and_invalid_data_rejection() {
    std::cout << "[SCENARIO 7] Non-Finite and Invalid Data Rejection...\n";

    double nan_val = std::numeric_limits<double>::quiet_NaN();
    double inf_val = std::numeric_limits<double>::infinity();

    // 7a: NaN in source coordinate
    std::vector<PointCorrespondence2D> nan_src = {
        PointCorrespondence2D(Point2D(nan_val, 0.0), Point2D(0.0, 0.0)),
        PointCorrespondence2D(Point2D(1.0, 0.0), Point2D(1.0, 0.0)),
        PointCorrespondence2D(Point2D(0.0, 1.0), Point2D(0.0, 1.0)),
        PointCorrespondence2D(Point2D(1.0, 1.0), Point2D(1.0, 1.0))
    };
    RansacConfig2D valid_config;
    auto r_nan = verify_relative_pose_2d(nan_src, valid_config);
    TEST_ASSERT(!r_nan.success, "NaN coordinate must fail verification");
    TEST_ASSERT(r_nan.failure_reason == VerificationFailureReason::NonFiniteInputCoordinates,
                "Must report NonFiniteInputCoordinates");

    // 7b: Infinity in target coordinate
    std::vector<PointCorrespondence2D> inf_tgt = {
        PointCorrespondence2D(Point2D(0.0, 0.0), Point2D(inf_val, 0.0)),
        PointCorrespondence2D(Point2D(1.0, 0.0), Point2D(1.0, 0.0)),
        PointCorrespondence2D(Point2D(0.0, 1.0), Point2D(0.0, 1.0)),
        PointCorrespondence2D(Point2D(1.0, 1.0), Point2D(1.0, 1.0))
    };
    auto r_inf = verify_relative_pose_2d(inf_tgt, valid_config);
    TEST_ASSERT(!r_inf.success, "Infinity coordinate must fail verification");
    TEST_ASSERT(r_inf.failure_reason == VerificationFailureReason::NonFiniteInputCoordinates,
                "Must report NonFiniteInputCoordinates");

    // 7c: Invalid configurations
    std::vector<PointCorrespondence2D> valid_corr = {
        PointCorrespondence2D(Point2D(0.0, 0.0), Point2D(1.0, 0.0)),
        PointCorrespondence2D(Point2D(1.0, 0.0), Point2D(2.0, 0.0)),
        PointCorrespondence2D(Point2D(0.0, 1.0), Point2D(1.0, 1.0)),
        PointCorrespondence2D(Point2D(1.0, 1.0), Point2D(2.0, 1.0))
    };

    // Negative threshold
    RansacConfig2D cfg_neg_thresh;
    cfg_neg_thresh.inlier_distance_threshold = -0.1;
    TEST_ASSERT(!cfg_neg_thresh.is_valid(), "Negative threshold is invalid config");
    auto r_neg = verify_relative_pose_2d(valid_corr, cfg_neg_thresh);
    TEST_ASSERT(!r_neg.success && r_neg.failure_reason == VerificationFailureReason::InvalidConfiguration,
                "Negative threshold must fail with InvalidConfiguration");

    // Zero max_iterations
    RansacConfig2D cfg_zero_iter;
    cfg_zero_iter.max_iterations = 0;
    TEST_ASSERT(!cfg_zero_iter.is_valid(), "Zero iterations is invalid config");
    auto r_iter = verify_relative_pose_2d(valid_corr, cfg_zero_iter);
    TEST_ASSERT(!r_iter.success && r_iter.failure_reason == VerificationFailureReason::InvalidConfiguration,
                "Zero iterations must fail with InvalidConfiguration");

    // min_correspondences < 2
    RansacConfig2D cfg_min_corr;
    cfg_min_corr.min_correspondences = 1;
    TEST_ASSERT(!cfg_min_corr.is_valid(), "min_correspondences < 2 is invalid config");
    auto r_mcorr = verify_relative_pose_2d(valid_corr, cfg_min_corr);
    TEST_ASSERT(!r_mcorr.success && r_mcorr.failure_reason == VerificationFailureReason::InvalidConfiguration,
                "min_correspondences < 2 must fail with InvalidConfiguration");

    // min_inlier_ratio > 1.0
    RansacConfig2D cfg_ratio_high;
    cfg_ratio_high.min_inlier_ratio = 1.5;
    TEST_ASSERT(!cfg_ratio_high.is_valid(), "min_inlier_ratio > 1.0 is invalid config");
    auto r_rh = verify_relative_pose_2d(valid_corr, cfg_ratio_high);
    TEST_ASSERT(!r_rh.success && r_rh.failure_reason == VerificationFailureReason::InvalidConfiguration,
                "min_inlier_ratio > 1.0 must fail with InvalidConfiguration");

    // Candidate prior with NaN
    RansacConfig2D cfg_prior_nan;
    cfg_prior_nan.candidate_prior = Pose2D(nan_val, 0.0, 0.0);
    TEST_ASSERT(!cfg_prior_nan.is_valid(), "Candidate prior with NaN is invalid config");
    auto r_pnan = verify_relative_pose_2d(valid_corr, cfg_prior_nan);
    TEST_ASSERT(!r_pnan.success && r_pnan.failure_reason == VerificationFailureReason::InvalidConfiguration,
                "Candidate prior with NaN must fail with InvalidConfiguration");

    // Negative prior tolerance
    RansacConfig2D cfg_neg_tol;
    cfg_neg_tol.max_prior_translation_diff = -1.0;
    TEST_ASSERT(!cfg_neg_tol.is_valid(), "Negative prior tolerance is invalid config");
    auto r_ntol = verify_relative_pose_2d(valid_corr, cfg_neg_tol);
    TEST_ASSERT(!r_ntol.success && r_ntol.failure_reason == VerificationFailureReason::InvalidConfiguration,
                "Negative prior tolerance must fail with InvalidConfiguration");
}

// -----------------------------------------------------------------------------
// Scenario 8: Transform-Direction Safety
// Reversed source/target frames must not silently pass as intended transform.
// -----------------------------------------------------------------------------
void test_transform_direction_safety() {
    std::cout << "[SCENARIO 8] Transform-Direction Safety...\n";

    // Forward transform: A -> B with tx = 10.0, ty = 4.0, theta = 0.5 rad
    std::vector<Point2D> pts_a = {
        {0.0, 0.0}, {3.0, 0.0}, {0.0, 3.0}, {3.0, 3.0}, {1.5, 4.0}
    };
    double fwd_tx = 10.0, fwd_ty = 4.0, fwd_theta = 0.5;
    auto forward_corr = make_transformed_correspondences(pts_a, fwd_tx, fwd_ty, fwd_theta);

    // Reversed correspondences: target points passed as source, source passed as target
    std::vector<PointCorrespondence2D> reversed_corr;
    reversed_corr.reserve(forward_corr.size());
    for (const auto& c : forward_corr) {
        reversed_corr.emplace_back(c.target, c.source, c.landmark_key);
    }

    // Expected forward candidate prior: (10.0, 4.0, 0.5)
    RansacConfig2D config_expected_forward;
    config_expected_forward.candidate_prior = Pose2D(fwd_tx, fwd_ty, fwd_theta);
    config_expected_forward.max_prior_translation_diff = 1.0;
    config_expected_forward.max_prior_rotation_diff = 0.1;

    // Estimating forward correspondences against forward prior: SUCCEEDS
    auto r_fwd = verify_relative_pose_2d(forward_corr, config_expected_forward);
    TEST_ASSERT(r_fwd.success, "Forward correspondences must pass against forward prior");

    // Estimating reversed correspondences against forward prior: MUST FAIL
    auto r_rev = verify_relative_pose_2d(reversed_corr, config_expected_forward);
    TEST_ASSERT(!r_rev.success, "Reversed correspondences must FAIL against forward prior");
    TEST_ASSERT(r_rev.failure_reason == VerificationFailureReason::InconsistentWithCandidatePrior,
                "Reversed transform must fail with InconsistentWithCandidatePrior");
}

// -----------------------------------------------------------------------------
// Scenario 9: Boundary Cases
// Exact inlier distance threshold, exact min_inliers, exact min_inlier_ratio.
// -----------------------------------------------------------------------------
void test_boundary_cases() {
    std::cout << "[SCENARIO 9] Boundary Cases...\n";

    double threshold = 0.20;

    // 9a: Point residual exactly equal to threshold
    // 4 exact identity correspondences + 1 point exactly at boundary + 1 point slightly beyond boundary
    std::vector<PointCorrespondence2D> corr = {
        PointCorrespondence2D(Point2D(0.0, 0.0), Point2D(0.0, 0.0)),
        PointCorrespondence2D(Point2D(2.0, 0.0), Point2D(2.0, 0.0)),
        PointCorrespondence2D(Point2D(0.0, 2.0), Point2D(0.0, 2.0)),
        PointCorrespondence2D(Point2D(2.0, 2.0), Point2D(2.0, 2.0)),
        PointCorrespondence2D(Point2D(1.0, 1.0), Point2D(1.0 + threshold, 1.0)),          // Exactly at boundary
        PointCorrespondence2D(Point2D(1.0, 0.0), Point2D(1.0, threshold + 0.005))         // Beyond boundary (0.205 m)
    };

    RansacConfig2D config;
    config.inlier_distance_threshold = threshold;
    config.min_correspondences = 4;
    config.min_inliers = 4;
    config.min_inlier_ratio = 0.50;
    config.max_iterations = 50;

    auto res = verify_relative_pose_2d(corr, config);
    TEST_ASSERT(res.success, "Verification must succeed");
    TEST_ASSERT(res.inlier_count == 5, "Exactly-at-boundary point must be counted as inlier (5/6 total inliers)");
    TEST_ASSERT(res.inlier_mask[4] == true, "Point at exact threshold must have inlier_mask true");
    TEST_ASSERT(res.inlier_mask[5] == false, "Point beyond threshold must have inlier_mask false");

    // 9b: Inlier count exactly equal to min_inliers
    RansacConfig2D config_exact_inliers = config;
    config_exact_inliers.min_inliers = 5; // exactly 5 inliers present
    auto res_exact = verify_relative_pose_2d(corr, config_exact_inliers);
    TEST_ASSERT(res_exact.success, "Inliers == min_inliers must pass");

    // 9c: Inlier count = min_inliers - 1 (min_inliers = 6, but only 5 inliers present)
    RansacConfig2D config_one_less = config;
    config_one_less.min_inliers = 6;
    auto res_fail = verify_relative_pose_2d(corr, config_one_less);
    TEST_ASSERT(!res_fail.success, "Inliers == min_inliers - 1 must fail");
    TEST_ASSERT(res_fail.failure_reason == VerificationFailureReason::InsufficientInliers,
                "Must report InsufficientInliers");

    // 9d: Inlier ratio boundary: 5 inliers out of 6 total = 5/6 = 0.8333...
    RansacConfig2D config_ratio_pass = config;
    config_ratio_pass.min_inlier_ratio = 5.0 / 6.0; // exactly 5/6
    auto res_ratio_pass = verify_relative_pose_2d(corr, config_ratio_pass);
    TEST_ASSERT(res_ratio_pass.success, "Inlier ratio == min_inlier_ratio must pass");

    RansacConfig2D config_ratio_fail = config;
    config_ratio_fail.min_inlier_ratio = (5.0 / 6.0) + 0.01; // slightly higher than 5/6
    auto res_ratio_fail = verify_relative_pose_2d(corr, config_ratio_fail);
    TEST_ASSERT(!res_ratio_fail.success, "Inlier ratio < min_inlier_ratio must fail");
    TEST_ASSERT(res_ratio_fail.failure_reason == VerificationFailureReason::LowInlierRatio,
                "Must report LowInlierRatio");
}

// -----------------------------------------------------------------------------
// Scenario 10: Determinism
// Identical input and random seed produce identical outcomes.
// -----------------------------------------------------------------------------
void test_determinism() {
    std::cout << "[SCENARIO 10] Determinism and Reproducibility...\n";

    std::vector<Point2D> pts = {
        {0.0, 0.0}, {2.0, 0.0}, {4.0, 1.0}, {1.0, 3.0}, {3.0, 4.0}, {5.0, 2.0}
    };
    auto corr = make_transformed_correspondences(pts, 1.5, -2.5, 0.45, 0.03, 12345);

    RansacConfig2D config;
    config.random_seed = 987654321ULL;

    auto res1 = verify_relative_pose_2d(corr, config);
    auto res2 = verify_relative_pose_2d(corr, config);

    TEST_ASSERT(res1.success == res2.success, "Determinism: success flags must match");
    TEST_ASSERT(res1.failure_reason == res2.failure_reason, "Determinism: failure reasons must match");
    TEST_ASSERT(res1.inlier_count == res2.inlier_count, "Determinism: inlier counts must match");
    TEST_ASSERT(res1.inlier_ratio == res2.inlier_ratio, "Determinism: inlier ratios must match");
    TEST_ASSERT(res1.inlier_rmse == res2.inlier_rmse, "Determinism: RMSE must match");
    TEST_ASSERT(res1.inlier_mask == res2.inlier_mask, "Determinism: inlier masks must match");
    TEST_ASSERT(res1.estimated_transform.x == res2.estimated_transform.x, "Determinism: estimated tx must match");
    TEST_ASSERT(res1.estimated_transform.y == res2.estimated_transform.y, "Determinism: estimated ty must match");
    TEST_ASSERT(res1.estimated_transform.theta == res2.estimated_transform.theta, "Determinism: estimated theta must match");
}

// -----------------------------------------------------------------------------
// Scenario 11: Loop-Closure Gating Safety & Lifecycle
// Non-existent edge, Odometry edge, missing endpoints, repeated verification.
// -----------------------------------------------------------------------------
void test_gating_safety_and_edge_lifecycle() {
    std::cout << "[SCENARIO 11] Gating Safety & Edge Lifecycle...\n";

    PoseGraph graph;
    NodeId r1_n1(1, 1), r1_n2(1, 2);
    NodeId r2_n1(2, 1);
    graph.add_node(KeyframeNode(r1_n1, Pose2D(0.0, 0.0, 0.0), 10.0));
    graph.add_node(KeyframeNode(r1_n2, Pose2D(1.0, 0.0, 0.0), 11.0));
    graph.add_node(KeyframeNode(r2_n1, Pose2D(5.0, 5.0, 0.0), 12.0));

    // Odometry edge: 1 -> 2
    PoseGraphEdge odom_edge(1, r1_n1, r1_n2, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0));
    graph.add_edge(odom_edge);

    // Inter-rover candidate: 2 -> 3
    PoseGraphEdge candidate_edge(2, r1_n2, r2_n1, EdgeType::InterRoverLoopClosure,
                                 Pose2D(4.0, 5.0, 0.0), InformationMatrix2D(),
                                 EdgeVerificationStatus::Unverified);
    graph.add_edge(candidate_edge);

    std::vector<Point2D> pts = {
        {0.0, 0.0}, {2.0, 0.0}, {0.0, 2.0}, {2.0, 2.0}
    };
    auto valid_corr = make_transformed_correspondences(pts, 4.0, 5.0, 0.0);

    // 11a: Attempt to gate non-existent edge
    auto r_missing = gate_and_promote_loop_closure(graph, 9999, valid_corr);
    TEST_ASSERT(!r_missing.success, "Gating non-existent edge must fail");
    TEST_ASSERT(r_missing.failure_reason == VerificationFailureReason::InvalidConfiguration,
                "Must report InvalidConfiguration for non-existent edge");

    // 11b: Attempt to gate Odometry edge
    auto r_odom = gate_and_promote_loop_closure(graph, 1, valid_corr);
    TEST_ASSERT(!r_odom.success, "Gating Odometry edge must fail");
    TEST_ASSERT(r_odom.failure_reason == VerificationFailureReason::InvalidConfiguration,
                "Must report InvalidConfiguration when trying to gate Odometry edge");
    // Odometry edge must remain untouched
    TEST_ASSERT(graph.get_edge(1).type == EdgeType::Odometry, "Odometry edge type must remain unchanged");
    TEST_ASSERT(graph.get_edge(1).status == EdgeVerificationStatus::Unverified, "Odometry edge status unchanged");

    // 11c: Successful gating promotes Unverified candidate to Verified
    auto r_promote = gate_and_promote_loop_closure(graph, 2, valid_corr);
    TEST_ASSERT(r_promote.success, "Valid candidate must be promoted");
    TEST_ASSERT(graph.get_edge(2).status == EdgeVerificationStatus::Verified,
                "Promoted edge status must be Verified");

    // 11d: Repeated verification attempt on already-Verified edge
    // Gating requires candidate in Unverified state
    auto r_repeat = gate_and_promote_loop_closure(graph, 2, valid_corr);
    TEST_ASSERT(!r_repeat.success, "Repeated gating on already-Verified edge must fail safely");
    TEST_ASSERT(r_repeat.failure_reason == VerificationFailureReason::InvalidConfiguration,
                "Must report InvalidConfiguration on already-Verified edge");
    TEST_ASSERT(graph.get_edge(2).status == EdgeVerificationStatus::Verified,
                "Edge status must remain Verified");

    // 11e: Failed verification on candidate edge transitions to Rejected, and cannot be re-gated
    PoseGraphEdge candidate_fail(3, r1_n1, r2_n1, EdgeType::InterRoverLoopClosure,
                                 Pose2D(5.0, 5.0, 0.0), InformationMatrix2D(),
                                 EdgeVerificationStatus::Unverified);
    graph.add_edge(candidate_fail);

    std::vector<PointCorrespondence2D> invalid_corr = {
        PointCorrespondence2D(Point2D(0.0, 0.0), Point2D(100.0, 100.0)),
        PointCorrespondence2D(Point2D(1.0, 0.0), Point2D(-100.0, 50.0)),
        PointCorrespondence2D(Point2D(0.0, 1.0), Point2D(0.0, -200.0)),
        PointCorrespondence2D(Point2D(1.0, 1.0), Point2D(50.0, 0.0))
    };
    auto r_fail = gate_and_promote_loop_closure(graph, 3, invalid_corr);
    TEST_ASSERT(!r_fail.success, "Invalid correspondences must fail gating");
    TEST_ASSERT(graph.get_edge(3).status == EdgeVerificationStatus::Rejected,
                "Edge must be marked Rejected");

    // Attempting to re-gate an already Rejected edge fails safely without corrupting status
    auto r_repeat_fail = gate_and_promote_loop_closure(graph, 3, valid_corr);
    TEST_ASSERT(!r_repeat_fail.success, "Cannot re-gate an already Rejected edge");
    TEST_ASSERT(graph.get_edge(3).status == EdgeVerificationStatus::Rejected,
                "Edge must remain Rejected");
}

// -----------------------------------------------------------------------------
// Scenario 12: DSU Subgraph Partition Safety
// Unverified/rejected candidates do not merge components; edge removal rebuilds.
// -----------------------------------------------------------------------------
void test_dsu_partition_safety() {
    std::cout << "[SCENARIO 12] DSU Subgraph Partition Safety...\n";

    PoseGraph graph;
    // Rover 1 has keyframes 101, 102 connected by odometry
    NodeId r1_k1(1, 101), r1_k2(1, 102);
    graph.add_node(KeyframeNode(r1_k1, Pose2D(0.0, 0.0, 0.0), 10.0));
    graph.add_node(KeyframeNode(r1_k2, Pose2D(1.0, 0.0, 0.0), 11.0));
    graph.add_edge(PoseGraphEdge(1, r1_k1, r1_k2, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0)));

    // Rover 2 has keyframes 201, 202 connected by odometry
    NodeId r2_k1(2, 201), r2_k2(2, 202);
    graph.add_node(KeyframeNode(r2_k1, Pose2D(10.0, 0.0, 0.0), 12.0));
    graph.add_node(KeyframeNode(r2_k2, Pose2D(11.0, 0.0, 0.0), 13.0));
    graph.add_edge(PoseGraphEdge(2, r2_k1, r2_k2, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0)));

    // Initialize partition manager with default policy (requires verified loop closures)
    SubgraphPartitionManager manager;
    manager.rebuild(graph);

    TEST_ASSERT(manager.num_partitions() == 2, "Initially rovers must be in 2 separate partitions");
    TEST_ASSERT(!manager.are_connected(r1_k1, r2_k1), "Rovers 1 and 2 must not be connected");

    // 12a: Add UNVERIFIED inter-rover candidate edge (102 -> 201)
    PoseGraphEdge candidate_edge(10, r1_k2, r2_k1, EdgeType::InterRoverLoopClosure,
                                 Pose2D(9.0, 0.0, 0.0), InformationMatrix2D(),
                                 EdgeVerificationStatus::Unverified);
    graph.add_edge(candidate_edge);

    // Attempting to add an unverified edge to partition manager does NOT merge components
    bool added_unverified = manager.add_eligible_edge(candidate_edge);
    TEST_ASSERT(!added_unverified, "Unverified edge must NOT be merged by partition manager");
    TEST_ASSERT(manager.num_partitions() == 2, "Partitions must remain 2 after unverified candidate added");
    TEST_ASSERT(!manager.are_connected(r1_k1, r2_k1), "Rovers must remain disconnected");

    // Rebuilding partition manager from graph also ignores unverified candidate
    manager.rebuild(graph);
    TEST_ASSERT(manager.num_partitions() == 2, "Rebuild must ignore unverified candidate");

    // 12b: Gate candidate edge with perceptually aliased / invalid correspondences -> REJECTED
    std::vector<PointCorrespondence2D> bad_corr = {
        PointCorrespondence2D(Point2D(0.0, 0.0), Point2D(50.0, 50.0)),
        PointCorrespondence2D(Point2D(1.0, 0.0), Point2D(-30.0, 20.0)),
        PointCorrespondence2D(Point2D(0.0, 1.0), Point2D(10.0, -80.0)),
        PointCorrespondence2D(Point2D(1.0, 1.0), Point2D(-5.0, 5.0))
    };
    auto gate_bad = gate_and_promote_loop_closure(graph, 10, bad_corr);
    TEST_ASSERT(!gate_bad.success, "Bad correspondences must be rejected");
    TEST_ASSERT(graph.get_edge(10).status == EdgeVerificationStatus::Rejected,
                "Candidate edge must be marked Rejected");

    // Adding rejected edge to partition manager does NOT connect
    bool added_rejected = manager.add_eligible_edge(graph.get_edge(10));
    TEST_ASSERT(!added_rejected, "Rejected edge must NOT be merged by partition manager");
    TEST_ASSERT(manager.num_partitions() == 2, "Partitions must remain 2 after rejection");
    TEST_ASSERT(!manager.are_connected(r1_k1, r2_k1), "Rovers must remain disconnected after rejection");

    manager.rebuild(graph);
    TEST_ASSERT(manager.num_partitions() == 2, "Rebuild from graph must ignore rejected edge");
    TEST_ASSERT(!manager.are_connected(r1_k2, r2_k1), "Rovers must remain disconnected after rebuild");

    // 12c: Remove rejected edge and add a new candidate edge with geometrically valid correspondences -> VERIFIED
    graph.remove_edge(10);
    PoseGraphEdge candidate_good(20, r1_k2, r2_k1, EdgeType::InterRoverLoopClosure,
                                 Pose2D(9.0, 0.0, 0.0), InformationMatrix2D(),
                                 EdgeVerificationStatus::Unverified);
    bool added_cand = graph.add_edge(candidate_good);
    TEST_ASSERT(added_cand, "Candidate edge 20 must be added to graph");

    std::vector<Point2D> pts = {
        {0.0, 0.0}, {2.0, 0.0}, {0.0, 2.0}, {2.0, 2.0}, {1.0, 1.0}
    };
    auto good_corr = make_transformed_correspondences(pts, 9.0, 0.0, 0.0);

    auto gate_good = gate_and_promote_loop_closure(graph, 20, good_corr);
    TEST_ASSERT(gate_good.success, "Good correspondences must be verified");
    TEST_ASSERT(graph.get_edge(20).status == EdgeVerificationStatus::Verified,
                "Candidate edge 20 must be marked Verified");

    // Incorporating verified edge into partition manager merges partitions!
    bool added_verified = manager.add_eligible_edge(graph.get_edge(20));
    TEST_ASSERT(added_verified, "Verified edge must be accepted by partition manager");
    TEST_ASSERT(manager.num_partitions() == 1, "Partitions must merge into 1 after verified loop closure");
    TEST_ASSERT(manager.are_connected(r1_k1, r2_k2), "Rovers 1 and 2 must now be connected");

    // 12d: Removing the verified edge followed by partition rebuild splits components back
    graph.remove_edge(20);
    manager.rebuild(graph);
    TEST_ASSERT(manager.num_partitions() == 2, "After removing verified edge and rebuilding, partitions must be 2");
    TEST_ASSERT(!manager.are_connected(r1_k1, r2_k2), "Rovers must be disconnected after edge removal and rebuild");
}

int main() {
    std::cout << "============================================================\n";
    std::cout << " AstroMesh-X Milestone 2.4: Perceptual Aliasing & Rejection\n";
    std::cout << "============================================================\n";

    test_perceptual_aliasing_rejection();
    test_random_false_correspondences_rejection();
    test_candidate_prior_inconsistency_rejection();
    test_insufficient_correspondences_rejection();
    test_degenerate_configurations_rejection();
    test_outlier_heavy_input_rejection();
    test_non_finite_and_invalid_data_rejection();
    test_transform_direction_safety();
    test_boundary_cases();
    test_determinism();
    test_gating_safety_and_edge_lifecycle();
    test_dsu_partition_safety();

    std::cout << "\n============================================================\n";
    std::cout << " Milestone 2.4 Test Summary: "
              << g_total_assertions << " assertions, "
              << g_failed_assertions << " failures.\n";
    std::cout << "============================================================\n";

    return (g_failed_assertions == 0) ? 0 : 1;
}
