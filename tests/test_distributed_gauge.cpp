#include "astromesh/net/gauge_coordinator.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using namespace astromesh;
using namespace astromesh::net;
using namespace astromesh::graph;
using namespace astromesh::geometry;

static bool approx_equal(double a, double b, double eps = 1e-4) {
    return std::abs(a - b) <= eps;
}

static bool pose_approx_equal(const SE2& a, const SE2& b, double eps = 1e-3) {
    return approx_equal(a.x(), b.x(), eps) &&
           approx_equal(a.y(), b.y(), eps) &&
           approx_equal(a.theta(), b.theta(), eps);
}

void test_initial_standalone_gauge() {
    std::cout << "[RUN] test_initial_standalone_gauge..." << std::endl;
    GaugeCoordinator coord(42);

    assert(coord.local_rover_id() == 42);
    assert(coord.current_anchor_id() == 42);
    assert(coord.is_anchor());
    assert(coord.is_aligned());
    assert(pose_approx_equal(coord.transform_from_anchor(), SE2::identity()));

    // Local pose transformed to global should equal local pose
    Pose2D local(10.0, 5.0, 0.3);
    Pose2D global = coord.transform_local_pose_to_global(local);
    assert(approx_equal(global.x, local.x));
    assert(approx_equal(global.y, local.y));
    assert(approx_equal(global.theta, local.theta));

    std::cout << "[PASS] test_initial_standalone_gauge" << std::endl;
}

void test_tworover_anchor_election_and_alignment() {
    std::cout << "[RUN] test_tworover_anchor_election_and_alignment..." << std::endl;

    GaugeCoordinator r1(1);
    GaugeCoordinator r2(2);

    // Rover 2 is offset from Rover 1: p1 = T_12 * p2
    // T_12: translation (5, 2), rotation 0.25 rad
    SE2 t12(5.0, 2.0, 0.25);

    // Rover 2 was matched against Rover 1
    // Edge: from_rover=2, to_rover=1, transform=t12
    bool ok1 = r1.register_pairwise_alignment(2, 1, t12, 15, 14, 0.03, 0.95, 1);
    bool ok2 = r2.register_pairwise_alignment(2, 1, t12, 15, 14, 0.03, 0.95, 1);

    assert(ok1);
    assert(ok2);

    // Anchor on both should be min(1, 2) = 1
    assert(r1.current_anchor_id() == 1);
    assert(r1.is_anchor());
    assert(pose_approx_equal(r1.transform_from_anchor(), SE2::identity()));

    assert(r2.current_anchor_id() == 1);
    assert(!r2.is_anchor());
    assert(r2.is_aligned());
    assert(pose_approx_equal(r2.transform_from_anchor(), t12));

    // A pose at origin in Rover 2's frame (0, 0, 0) should be at (5, 2, 0.25) in Global (Anchor 1) frame
    Pose2D p2_local(0.0, 0.0, 0.0);
    Pose2D p2_global = r2.transform_local_pose_to_global(p2_local);
    assert(approx_equal(p2_global.x, 5.0));
    assert(approx_equal(p2_global.y, 2.0));
    assert(approx_equal(p2_global.theta, 0.25));

    // Transform landmark
    auto [lx_glob, ly_glob] = r2.transform_landmark_to_global(0.0, 0.0);
    assert(approx_equal(lx_glob, 5.0));
    assert(approx_equal(ly_glob, 2.0));

    std::cout << "[PASS] test_tworover_anchor_election_and_alignment" << std::endl;
}

void test_four_rover_diamond_and_cycle_remediation() {
    std::cout << "[RUN] test_four_rover_diamond_and_cycle_remediation..." << std::endl;

    // 4 rovers in diamond topology:
    // Rover 1 (origin)
    // Rover 2 at (10, 0, 0) relative to 1 -> T_12 = (10, 0, 0)
    // Rover 3 at (0, 10, 0) relative to 1 -> T_13 = (0, 10, 0)
    // Rover 4 at (0, 10, 0) relative to 2 -> T_24 = (0, 10, 0)
    // Rover 4 at (10, 0, 0) relative to 3 -> T_34 = (10, 0, 0)
    // Note: p1 = T_12 * p2 = (10, 0, 0) + (0, 10, 0) = (10, 10, 0) for p4
    // p1 = T_13 * p3 = (0, 10, 0) + (10, 0, 0) = (10, 10, 0) for p4
    SE2 t12(10.0, 0.0, 0.0);
    SE2 t13(0.0, 10.0, 0.0);
    SE2 t24(0.0, 10.0, 0.0);
    SE2 t34(10.0, 0.0, 0.0);

    GaugeCoordinator r4(4);
    assert(r4.register_pairwise_alignment(2, 1, t12, 20, 20, 0.02, 1.0, 1));
    assert(r4.register_pairwise_alignment(3, 1, t13, 20, 20, 0.02, 1.0, 1));
    assert(r4.register_pairwise_alignment(4, 2, t24, 20, 20, 0.02, 1.0, 1));
    assert(r4.register_pairwise_alignment(4, 3, t34, 20, 20, 0.02, 1.0, 1));

    assert(r4.current_anchor_id() == 1);
    assert(r4.is_aligned());

    // In Anchor 1's frame, Rover 4 origin is at (10, 10, 0)
    SE2 t_anchor_r4 = r4.transform_from_anchor();
    assert(approx_equal(t_anchor_r4.x(), 10.0));
    assert(approx_equal(t_anchor_r4.y(), 10.0));
    assert(approx_equal(t_anchor_r4.theta(), 0.0));

    // Now inject a conflicting cycle on Rover 1
    // Form triangle 1-2-3 where edge 3->2 is corrupted
    // T_12 = (10, 0, 0), T_23_true = (-10, 10, 0), T_31 = (0, -10, 0)
    // Corrupt T_23 with an error of 5.0 m
    GaugeCoordinator r1(1);
    r1.register_pairwise_alignment(2, 1, t12, 20, 20, 0.01, 1.0, 1);
    r1.register_pairwise_alignment(1, 3, t13.inverse(), 20, 20, 0.01, 1.0, 1);

    SE2 t23_corrupt(-10.0 + 5.0, 10.0, 0.5); // Large error
    r1.register_pairwise_alignment(3, 2, t23_corrupt, 20, 8, 0.85, 0.6, 1);

    // Verify that the corrupt cycle was audited and remediated automatically
    assert(r1.metrics().inconsistent_cycles_detected >= 1);
    assert(r1.metrics().edges_invalidated >= 1);

    std::cout << "[PASS] test_four_rover_diamond_and_cycle_remediation" << std::endl;
}

void test_wire_record_serialization() {
    std::cout << "[RUN] test_wire_record_serialization..." << std::endl;

    AlignmentWireRecord rec;
    rec.from_rover = 3;
    rec.to_rover = 1;
    rec.tx = 12.345;
    rec.ty = -67.89;
    rec.theta = 1.234;
    rec.confidence = 0.987;
    rec.inlier_rmse = 0.025;
    rec.inlier_count = 42;
    rec.epoch = 1005;

    std::vector<uint8_t> buffer;
    rec.serialize(buffer);
    assert(buffer.size() == sizeof(AlignmentWireRecord));

    AlignmentWireRecord decoded;
    bool ok = AlignmentWireRecord::deserialize(buffer.data(), buffer.size(), decoded);
    assert(ok);
    assert(decoded.from_rover == 3);
    assert(decoded.to_rover == 1);
    assert(approx_equal(decoded.tx, 12.345));
    assert(approx_equal(decoded.ty, -67.89));
    assert(approx_equal(decoded.theta, 1.234));
    assert(approx_equal(decoded.confidence, 0.987));
    assert(approx_equal(decoded.inlier_rmse, 0.025));
    assert(decoded.inlier_count == 42);
    assert(decoded.epoch == 1005);

    // Truncated buffer should fail deserialization
    assert(!AlignmentWireRecord::deserialize(buffer.data(), 10, decoded));
    assert(!AlignmentWireRecord::deserialize(nullptr, sizeof(AlignmentWireRecord), decoded));

    std::cout << "[PASS] test_wire_record_serialization" << std::endl;
}

void test_pose_graph_batch_transformation() {
    std::cout << "[RUN] test_pose_graph_batch_transformation..." << std::endl;

    GaugeCoordinator coord(2);
    // Align with Anchor 1 where T_12 = (10, 20, pi/2)
    SE2 t12(10.0, 20.0, 0.5 * 3.14159265);
    coord.register_pairwise_alignment(2, 1, t12, 10, 10, 0.01, 1.0, 1);

    PoseGraph pg;
    pg.add_node(KeyframeNode(NodeId(2, 1), Pose2D(0.0, 0.0, 0.0)));
    pg.add_node(KeyframeNode(NodeId(2, 2), Pose2D(5.0, 0.0, 0.0)));

    PoseGraph transformed = coord.transform_pose_graph(pg);
    assert(transformed.num_nodes() == 2);

    // Node 1 was at (0, 0, 0) in R2 frame -> in global frame: T_12 * (0,0,0) = (10, 20, pi/2)
    const auto* kf1 = transformed.find_node(NodeId(2, 1));
    assert(kf1 != nullptr);
    assert(approx_equal(kf1->pose.x, 10.0));
    assert(approx_equal(kf1->pose.y, 20.0));
    assert(approx_equal(kf1->pose.theta, 0.5 * 3.14159265));

    // Node 2 was at (5, 0, 0) in R2 frame.
    // Under rotation pi/2: (5, 0) rotates to (0, 5).
    // Plus translation (10, 20) -> (10, 25, pi/2)
    const auto* kf2 = transformed.find_node(NodeId(2, 2));
    assert(kf2 != nullptr);
    assert(approx_equal(kf2->pose.x, 10.0));
    assert(approx_equal(kf2->pose.y, 25.0));
    assert(approx_equal(kf2->pose.theta, 0.5 * 3.14159265));

    std::cout << "[PASS] test_pose_graph_batch_transformation" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << " AstroMesh-X: Distributed Gauge Coordinator Tests" << std::endl;
    std::cout << "========================================" << std::endl;

    test_initial_standalone_gauge();
    test_tworover_anchor_election_and_alignment();
    test_four_rover_diamond_and_cycle_remediation();
    test_wire_record_serialization();
    test_pose_graph_batch_transformation();

    std::cout << "All Distributed Gauge Coordinator tests PASSED successfully." << std::endl;
    return 0;
}

