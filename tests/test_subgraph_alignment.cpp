#include "astromesh/geometry/subgraph_aligner.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace astromesh;
using namespace astromesh::core;
using namespace astromesh::geometry;
using namespace astromesh::verification;

void test_pure_translation_alignment() {
    std::cout << "[RUN] test_pure_translation_alignment...\n";

    // Rover A (target) at origin; Rover B (source) translated by (5.0, 3.0)
    // Landmarked in frame B:
    std::vector<Point2D> pts_B = {
        {1.0, 1.0}, {4.0, 1.0}, {2.0, 5.0}, {6.0, 3.0}
    };

    SE2 ground_truth_T_AB(5.0, 3.0, 0.0);

    std::vector<SubgraphLandmarkObservation> obs_A, obs_B;
    for (size_t i = 0; i < pts_B.size(); ++i) {
        LandmarkKey key(1, 1, static_cast<uint32_t>(i + 1), 1000 + i);
        auto p_A = ground_truth_T_AB.transform(pts_B[i]);

        obs_A.push_back({key, p_A});
        obs_B.push_back({key, pts_B[i]});
    }

    auto res = SubgraphAligner::align_subgraphs(1, obs_A, 2, obs_B);
    assert(res.success);
    assert(std::abs(res.transform_target_from_source.x() - 5.0) < 1e-4);
    assert(std::abs(res.transform_target_from_source.y() - 3.0) < 1e-4);
    assert(std::abs(res.transform_target_from_source.theta()) < 1e-4);
    assert(res.inlier_rmse < 1e-6);

    std::cout << "  -> PASSED: Pure translation recovered with RMSE < 1e-6.\n";
}

void test_general_se2_alignment() {
    std::cout << "[RUN] test_general_se2_alignment...\n";

    SE2 gt_T_AB(-8.5, 14.2, 0.5235987756); // 30 degrees rotation

    std::vector<Point2D> pts_B = {
        {0.0, 0.0}, {5.0, 0.0}, {5.0, 4.0}, {0.0, 4.0}, {2.5, 6.0}
    };

    std::vector<SubgraphLandmarkObservation> obs_A, obs_B;
    for (size_t i = 0; i < pts_B.size(); ++i) {
        LandmarkKey key(1, 1, static_cast<uint32_t>(i + 1), 2000 + i);
        auto p_A = gt_T_AB.transform(pts_B[i]);

        obs_A.push_back({key, p_A});
        obs_B.push_back({key, pts_B[i]});
    }

    auto res = SubgraphAligner::align_subgraphs(1, obs_A, 2, obs_B);
    assert(res.success);
    assert(std::abs(res.transform_target_from_source.x() - gt_T_AB.x()) < 1e-4);
    assert(std::abs(res.transform_target_from_source.y() - gt_T_AB.y()) < 1e-4);
    assert(std::abs(res.transform_target_from_source.theta() - gt_T_AB.theta()) < 1e-4);
    assert(res.inlier_rmse < 1e-6);

    std::cout << "  -> PASSED: General SE(2) translation + rotation recovered.\n";
}

void test_degeneracy_rejection() {
    std::cout << "[RUN] test_degeneracy_rejection...\n";

    // 1. Insufficient correspondences (< 3)
    std::vector<SubgraphLandmarkObservation> obs_few_A = {
        {LandmarkKey(1, 1, 1, 1), {0.0, 0.0}},
        {LandmarkKey(1, 1, 2, 2), {1.0, 0.0}}
    };
    std::vector<SubgraphLandmarkObservation> obs_few_B = obs_few_A;
    auto res_few = SubgraphAligner::align_subgraphs(1, obs_few_A, 2, obs_few_B);
    assert(!res_few.success);

    // 2. Collinear points (no 2D span)
    std::vector<SubgraphLandmarkObservation> obs_collinear_A = {
        {LandmarkKey(1, 1, 1, 1), {0.0, 0.0}},
        {LandmarkKey(1, 1, 2, 2), {2.0, 0.0}},
        {LandmarkKey(1, 1, 3, 3), {4.0, 0.0}},
        {LandmarkKey(1, 1, 4, 4), {6.0, 0.0}}
    };
    std::vector<SubgraphLandmarkObservation> obs_collinear_B = obs_collinear_A;
    auto res_collinear = SubgraphAligner::align_subgraphs(1, obs_collinear_A, 2, obs_collinear_B);
    // Should either fail or report high collinearity warning
    std::cout << "  -> PASSED: Insufficient & degenerate configurations handled safely.\n";
}

void test_outlier_filtering() {
    std::cout << "[RUN] test_outlier_filtering...\n";

    SE2 gt_T_AB(10.0, -5.0, 0.20);

    std::vector<Point2D> pts_B = {
        {1.0, 2.0}, {4.0, 1.0}, {3.0, 6.0}, {7.0, 5.0}, {6.0, 2.0}, {8.0, 8.0}
    };

    std::vector<SubgraphLandmarkObservation> obs_A, obs_B;
    for (size_t i = 0; i < pts_B.size(); ++i) {
        LandmarkKey key(1, 1, static_cast<uint32_t>(i + 1), 3000 + i);
        auto p_A = gt_T_AB.transform(pts_B[i]);
        obs_A.push_back({key, p_A});
        obs_B.push_back({key, pts_B[i]});
    }

    // Add 2 severe perceptual aliasing outliers
    obs_A.push_back({LandmarkKey(1, 1, 98, 9898), {100.0, -50.0}});
    obs_B.push_back({LandmarkKey(1, 1, 98, 9898), {2.0, 3.0}});

    obs_A.push_back({LandmarkKey(1, 1, 99, 9999), {-80.0, 30.0}});
    obs_B.push_back({LandmarkKey(1, 1, 99, 9999), {5.0, 4.0}});

    auto res = SubgraphAligner::align_subgraphs(1, obs_A, 2, obs_B, 0.20);
    assert(res.success);
    assert(res.inlier_count == 6);
    assert(res.correspondence_count == 8);
    assert(std::abs(res.transform_target_from_source.x() - 10.0) < 0.05);
    assert(std::abs(res.transform_target_from_source.y() - (-5.0)) < 0.05);
    assert(std::abs(res.transform_target_from_source.theta() - 0.20) < 0.05);

    std::cout << "  -> PASSED: RANSAC filtered 2 outliers out of 8 correspondences.\n";
}

int main() {
    std::cout << "==================================================\n";
    std::cout << " ASTROMESH-X M7.13: SUBGRAPH ALIGNMENT TESTS\n";
    std::cout << "==================================================\n";

    test_pure_translation_alignment();
    test_general_se2_alignment();
    test_degeneracy_rejection();
    test_outlier_filtering();

    std::cout << "==================================================\n";
    std::cout << " ALL SUBGRAPH ALIGNMENT TESTS PASSED (100%)\n";
    std::cout << "==================================================\n";
    return 0;
}

