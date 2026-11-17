#include <iostream>
#include <cmath>
#include "astromesh/geometry/se2.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

using namespace astromesh;

int main() {
    // Rover A at (10, 5, 0.2 rad)
    geometry::SE2 T_WA(10.0, 5.0, 0.2);
    // Rover B at (14.0, 8.0, -0.1 rad)
    geometry::SE2 T_WB(14.0, 8.0, -0.1);

    // Landmarks in world
    std::vector<verification::PointCorrespondence2D> corrs;
    for (int i = 0; i < 5; ++i) {
        double wx = 12.0 + i * 2.0;
        double wy = 7.0 + i * 1.5;
        // Local point in Rover A
        geometry::SE2 P_W(wx, wy, 0.0);
        geometry::SE2 P_A = T_WA.inverse() * P_W;
        geometry::SE2 P_B = T_WB.inverse() * P_W;

        verification::Point2D pt_a(P_A.x(), P_A.y());
        verification::Point2D pt_b(P_B.x(), P_B.y());
        corrs.emplace_back(pt_b, pt_a);
    }

    verification::RansacConfig2D cfg;
    auto vres = verification::verify_relative_pose_2d(corrs, cfg);

    std::cout << "vres success: " << vres.success << "\n";
    std::cout << "vres estimated: (" << vres.estimated_transform.x << ", " 
              << vres.estimated_transform.y << ", " << vres.estimated_transform.theta << ")\n";

    // What is T_WA.inverse() * T_WB?
    geometry::SE2 T_AB = T_WA.inverse() * T_WB;
    std::cout << "T_AB (T_WA^-1 * T_WB): (" << T_AB.x() << ", " << T_AB.y() << ", " << T_AB.theta() << ")\n";

    // What is T_WB.inverse() * T_WA?
    geometry::SE2 T_BA = T_WB.inverse() * T_WA;
    std::cout << "T_BA (T_WB^-1 * T_WA): (" << T_BA.x() << ", " << T_BA.y() << ", " << T_BA.theta() << ")\n";

    return 0;
}
