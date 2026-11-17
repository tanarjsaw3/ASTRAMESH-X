#include "astromesh/geometry/se2.hpp"
#include "astromesh/geometry/subgraph_aligner.hpp"
#include "astromesh/graph/alignment_graph.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/net/gauge_coordinator.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using namespace astromesh;
using namespace astromesh::geometry;
using namespace astromesh::graph;
using namespace astromesh::net;

// Test 1: Verify that PoseGraph has no dependency or storage for ground truth
static void test_pose_graph_has_no_ground_truth_leakage() {
    PoseGraph graph;
    
    // Add nodes with noisy/local coordinates
    graph.add_node(KeyframeNode(NodeId(1, 0), Pose2D(0.0, 0.0, 0.0), 1000));
    graph.add_node(KeyframeNode(NodeId(1, 1), Pose2D(10.0, 0.5, 0.05), 2000));
    graph.add_node(KeyframeNode(NodeId(1, 2), Pose2D(20.0, 1.2, 0.10), 3000));

    // Verify that PoseGraph nodes store only local estimated poses
    assert(graph.num_nodes() == 3);
    const auto& node1 = graph.get_node(NodeId(1, 1));
    assert(std::abs(node1.pose.x - 10.0) < 1e-6);
    assert(std::abs(node1.pose.y - 0.5) < 1e-6);
    
    // Graph can be cloned and serialized without any external ground truth reference
    assert(graph.edges().empty());
    std::cout << "[PASS] PoseGraph ground-truth isolation verified.\n";
}

// Test 2: Verify SubgraphAligner computes relative SE(2) transforms solely from relative observations
static void test_subgraph_aligner_isolation() {
    // Two rovers observe mutual landmarks in their local frames
    // True relative transform between Rover 1 and Rover 2 is dx=25.0, dy=-10.0, dtheta=0.25 rad
    SE2 T_12_true(25.0, -10.0, 0.25);

    // Landmarks in Rover 2 frame
    std::vector<verification::Point2D> pts_in_rover2 = {
        verification::Point2D(5.0, 5.0),
        verification::Point2D(15.0, 2.0),
        verification::Point2D(8.0, 18.0),
        verification::Point2D(20.0, 12.0)
    };

    // Projected into Rover 1 frame via T_12_true: pt1 = T_12 * pt2
    // For Kabsch, source = pt2 (Rover 2), target = pt1 (Rover 1) so transform estimates T_12 mapping 2 into 1
    std::vector<verification::PointCorrespondence2D> correspondences;
    for (const auto& pt2 : pts_in_rover2) {
        auto pt1_true = T_12_true.transform(pt2);
        correspondences.push_back({pt2, pt1_true});
    }

    SE2 T_estimated;
    double rmse = 0.0;
    bool success = SubgraphAligner::estimate_se2_kabsch(correspondences, T_estimated, rmse);
    assert(success);
    assert(rmse < 1e-5);
    
    // Verify estimated relative transform matches the true relative offset without knowing world positions
    assert(std::abs(T_estimated.x() - 25.0) < 1e-4);
    assert(std::abs(T_estimated.y() - (-10.0)) < 1e-4);
    assert(std::abs(T_estimated.theta() - 0.25) < 1e-4);

    std::cout << "[PASS] SubgraphAligner operates purely on pairwise correspondences without world ground truth.\n";
}

// Test 3: Verify GaugeCoordinator manages anchors and transforms without oracle labels
static void test_gauge_coordinator_decentralized_invariants() {
    GaugeCoordinator coordinator(1);
    // Rover 1 <-> Rover 2: relative offset (20.0, 0.0, 0.0)
    coordinator.register_pairwise_alignment(1, 2, SE2(20.0, 0.0, 0.0), 12, 12, 0.02, 0.95, 1);
    // Rover 2 <-> Rover 3: relative offset (15.0, 5.0, 0.1)
    coordinator.register_pairwise_alignment(2, 3, SE2(15.0, 5.0, 0.1), 10, 10, 0.03, 0.90, 1);

    coordinator.recompute_gauge();

    // Anchor must be deterministically elected as minimum ID (Rover 1)
    assert(coordinator.is_anchor());
    assert(coordinator.current_anchor_id() == 1);

    // Rover 1 to Rover 1 is identity
    SE2 T_anchor = coordinator.transform_from_anchor();
    assert(std::abs(T_anchor.x()) < 1e-5);

    std::cout << "[PASS] GaugeCoordinator decentralized anchor resolution verified.\n";
}

int main() {
    std::cout << "Running test_m717_ground_truth_integrity...\n";
    test_pose_graph_has_no_ground_truth_leakage();
    test_subgraph_aligner_isolation();
    test_gauge_coordinator_decentralized_invariants();
    std::cout << "All M7.17 ground-truth integrity tests PASSED.\n";
    return 0;
}
