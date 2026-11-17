#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/optimization/cord_integrator.hpp"
#include "astromesh/verification/geometric_verifier.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace astromesh;

int main() {
    std::cout << "=== TRANSFORM INVERSION & CORD RESIDUAL VERIFICATION ===\n";

    // Ground truth poses:
    // Rover 1 at X1 = (10.0, 0.0, 0.0)
    // Rover 2 at X2 = (20.0, 0.0, 0.0) (relative offset = +10.0 along X)
    geometry::SE2 X1(10.0, 0.0, 0.0);
    geometry::SE2 X2(20.0, 0.0, 0.0);

    // Landmark in world frame at Pw = (25.0, 5.0)
    // In Rover 1 frame: p1 = X1^{-1} * Pw = (25-10, 5) = (15.0, 5.0)
    // In Rover 2 frame: p2 = X2^{-1} * Pw = (25-20, 5) = (5.0, 5.0)
    // Notice: p1 = p2 + (10.0, 0.0)
    // So T_{1 <- 2} has translation (+10.0, 0.0)
    // T_{2 <- 1} has translation (-10.0, 0.0)

    std::vector<verification::PointCorrespondence2D> corr_2_to_1;
    // Source = Rover 2 (p2), Target = Rover 1 (p1)
    for (int l = 1; l <= 4; ++l) {
        corr_2_to_1.emplace_back(
            verification::Point2D(5.0 * l, 2.0 * l),       // Source: Rover 2
            verification::Point2D(5.0 * l + 10.0, 2.0 * l),// Target: Rover 1
            core::LandmarkKey(1, 1, 100 + l, 0x1000ULL + l)
        );
    }

    verification::RansacConfig2D cfg;
    auto res_2_to_1 = verification::verify_relative_pose_2d(corr_2_to_1, cfg);
    std::cout << "Direction 2 -> 1: estimated transform (local_from_remote) = ("
              << res_2_to_1.estimated_transform.x << ", " << res_2_to_1.estimated_transform.y << ")\n";

    // Now test PoseGraph edge definition:
    // Edge from Node 2 to Node 1:
    // Required Zij = X2^{-1} * X1 = (20, 0)^{-1} * (10, 0) = (-10.0, 0.0)
    // Since res_2_to_1.estimated_transform is (+10.0, 0.0), its inverse is (-10.0, 0.0)!
    geometry::SE2 inv_2_to_1(res_2_to_1.estimated_transform.x, res_2_to_1.estimated_transform.y, 0.0);
    geometry::SE2 edge_pose_2_to_1 = inv_2_to_1.inverse();
    std::cout << "Edge 2 -> 1 pose: (" << edge_pose_2_to_1.x() << ", " << edge_pose_2_to_1.y() << ")\n";

    auto residual_2_to_1 = optimization::compute_edge_residual(
        X2, X1, edge_pose_2_to_1, graph::InformationMatrix2D::diagonal(100.0, 1000.0)
    );
    std::cout << "Residual for Edge 2 -> 1: valid=" << residual_2_to_1.valid
              << " cost=" << residual_2_to_1.cost
              << " residual=(" << residual_2_to_1.residual.vx << ", " << residual_2_to_1.residual.vy << ")\n";
    assert(std::abs(residual_2_to_1.cost) < 1e-9);

    // Now test Reverse Direction:
    // Source = Rover 1 (p1), Target = Rover 2 (p2)
    std::vector<verification::PointCorrespondence2D> corr_1_to_2;
    for (int l = 1; l <= 4; ++l) {
        corr_1_to_2.emplace_back(
            verification::Point2D(5.0 * l + 10.0, 2.0 * l),// Source: Rover 1
            verification::Point2D(5.0 * l, 2.0 * l),       // Target: Rover 2
            core::LandmarkKey(1, 1, 100 + l, 0x1000ULL + l)
        );
    }
    auto res_1_to_2 = verification::verify_relative_pose_2d(corr_1_to_2, cfg);
    std::cout << "\nDirection 1 -> 2: estimated transform (local_from_remote) = ("
              << res_1_to_2.estimated_transform.x << ", " << res_1_to_2.estimated_transform.y << ")\n";

    // Edge from Node 1 to Node 2:
    // Required Zij = X1^{-1} * X2 = (10, 0)^{-1} * (20, 0) = (+10.0, 0.0)
    // res_1_to_2.estimated_transform is (-10.0, 0.0). Its inverse is (+10.0, 0.0)!
    geometry::SE2 inv_1_to_2(res_1_to_2.estimated_transform.x, res_1_to_2.estimated_transform.y, 0.0);
    geometry::SE2 edge_pose_1_to_2 = inv_1_to_2.inverse();
    std::cout << "Edge 1 -> 2 pose: (" << edge_pose_1_to_2.x() << ", " << edge_pose_1_to_2.y() << ")\n";

    auto residual_1_to_2 = optimization::compute_edge_residual(
        X1, X2, edge_pose_1_to_2, graph::InformationMatrix2D::diagonal(100.0, 1000.0)
    );
    std::cout << "Residual for Edge 1 -> 2: valid=" << residual_1_to_2.valid
              << " cost=" << residual_1_to_2.cost
              << " residual=(" << residual_1_to_2.residual.vx << ", " << residual_1_to_2.residual.vy << ")\n";
    assert(std::abs(residual_1_to_2.cost) < 1e-9);

    std::cout << "\nALL TRANSFORM INVERSIONS ARE RIGOROUSLY CONSISTENT (Cost = 0.0)!\n";
    return 0;
}

