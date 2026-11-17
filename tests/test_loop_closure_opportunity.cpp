#include "astromesh/coordination/loop_closure_opportunity.hpp"

#include <cassert>
#include <iostream>
#include <vector>

using namespace astromesh;
using namespace astromesh::core;
using namespace astromesh::graph;
using namespace astromesh::coordination;

void test_opportunity_addition_and_deduplication() {
    std::cout << "[RUN] test_opportunity_addition_and_deduplication..." << std::endl;
    LoopClosureOpportunityManager mgr(1);

    // Reject self or zero
    LoopClosureOpportunity inv1;
    inv1.rover_a = 0; inv1.rover_b = 2;
    assert(!mgr.add_opportunity(inv1));

    LoopClosureOpportunity inv2;
    inv2.rover_a = 1; inv2.rover_b = 1;
    assert(!mgr.add_opportunity(inv2));

    // Valid opportunity
    LoopClosureOpportunity o1;
    o1.rover_a = 1;
    o1.rover_b = 2;
    o1.keyframe_a = NodeId(1, 5);
    o1.keyframe_b = NodeId(2, 8);
    o1.confidence = 0.85;
    assert(mgr.add_opportunity(o1));
    assert(mgr.size() == 1);

    auto retrieved = mgr.get_opportunity(1);
    assert(retrieved.has_value());
    assert(retrieved->rover_a == 1);
    assert(retrieved->rover_b == 2);

    // Duplicate rejection
    LoopClosureOpportunity o1_dup = o1;
    assert(!mgr.add_opportunity(o1_dup));
    assert(mgr.size() == 1);

    // Inverted duplicate rejection (2,8) <-> (1,5)
    LoopClosureOpportunity o1_inv;
    o1_inv.rover_a = 2;
    o1_inv.rover_b = 1;
    o1_inv.keyframe_a = NodeId(2, 8);
    o1_inv.keyframe_b = NodeId(1, 5);
    assert(!mgr.add_opportunity(o1_inv));
    assert(mgr.size() == 1);

    std::cout << "[PASS] test_opportunity_addition_and_deduplication" << std::endl;
}

void test_expiration_pruning() {
    std::cout << "[RUN] test_expiration_pruning..." << std::endl;
    LoopClosureOpportunityManager mgr(1);

    LoopClosureOpportunity o1;
    o1.rover_a = 1; o1.rover_b = 2;
    o1.keyframe_a = NodeId(1, 1);
    o1.keyframe_b = NodeId(2, 1);
    o1.created_timestamp_ms = 1000;
    o1.expiration_timestamp_ms = 2000;

    LoopClosureOpportunity o2;
    o2.rover_a = 1; o2.rover_b = 2;
    o2.keyframe_a = NodeId(1, 2);
    o2.keyframe_b = NodeId(2, 2);
    o2.created_timestamp_ms = 1000;
    o2.expiration_timestamp_ms = 5000;

    assert(mgr.add_opportunity(o1));
    assert(mgr.add_opportunity(o2));
    assert(mgr.size() == 2);

    // At t = 1500, neither is expired
    size_t pruned = mgr.prune_expired(1500);
    assert(pruned == 0);
    assert(mgr.size() == 2);

    // At t = 2500, o1 is expired, o2 remains
    pruned = mgr.prune_expired(2500);
    assert(pruned == 1);
    assert(mgr.size() == 1);
    assert(mgr.opportunities()[0].keyframe_a == NodeId(1, 2));

    std::cout << "[PASS] test_expiration_pruning" << std::endl;
}

void test_spatial_and_overlap_detection() {
    std::cout << "[RUN] test_spatial_and_overlap_detection..." << std::endl;
    LoopClosureOpportunityManager mgr(1);

    PoseGraph local_graph;
    // Local keyframes for Rover 1
    KeyframeNode kf1(NodeId(1, 1), Pose2D(0.0, 0.0, 0.0));
    kf1.observed_landmarks = {
        LandmarkKey(1, 1, 1, 0x1111),
        LandmarkKey(1, 1, 2, 0x2222),
        LandmarkKey(1, 1, 3, 0x3333),
        LandmarkKey(1, 1, 4, 0x4444)
    };
    local_graph.add_node(kf1);

    KeyframeNode kf2(NodeId(1, 2), Pose2D(10.0, 0.0, 0.0));
    local_graph.add_node(kf2);

    // Remote keyframes for Rover 2
    std::vector<KeyframeNode> remote_kfs;
    KeyframeNode rem1(NodeId(2, 1), Pose2D(2.0, 1.0, 0.0)); // Close to kf1 (dist ~ 2.23m)
    rem1.observed_landmarks = {
        LandmarkKey(1, 1, 1, 0x1111),
        LandmarkKey(1, 1, 2, 0x2222),
        LandmarkKey(1, 1, 3, 0x3333)
    };
    remote_kfs.push_back(rem1);

    KeyframeNode rem2(NodeId(2, 2), Pose2D(100.0, 100.0, 0.0)); // Far away
    remote_kfs.push_back(rem2);

    // Detect spatial candidates within 5m
    size_t created = mgr.detect_spatial_opportunities(local_graph, 2, remote_kfs, 5.0);
    assert(created == 1); // Only (kf1, rem1) is within 5m (dist ~ 2.23m vs 8.06m for kf2)
    assert(mgr.size() == 1);

    const auto& opp = mgr.opportunities()[0];
    assert(opp.keyframe_a == NodeId(1, 1));
    assert(opp.keyframe_b == NodeId(2, 1));
    assert(opp.estimated_landmark_overlap == 3);

    // Detect subgraph overlap candidates
    std::vector<core::LandmarkKey> remote_keys = {
        LandmarkKey(1, 1, 1, 0x1111),
        LandmarkKey(1, 1, 2, 0x2222),
        LandmarkKey(1, 1, 3, 0x3333),
        LandmarkKey(1, 1, 4, 0x4444)
    };
    // Clean and detect overlap
    mgr.clear();
    size_t overlap_created = mgr.detect_subgraph_overlap_opportunities(
        2, NodeId(2, 5), remote_keys, local_graph);
    assert(overlap_created == 1);
    assert(mgr.opportunities()[0].estimated_landmark_overlap == 4);

    std::cout << "[PASS] test_spatial_and_overlap_detection" << std::endl;
}

void test_redundant_opportunity_filtering() {
    std::cout << "[RUN] test_redundant_opportunity_filtering..." << std::endl;
    LoopClosureOpportunityManager mgr(1);

    // Opp 1: (1, 10) <-> (2, 20) with 8 correspondences
    LoopClosureOpportunity o1;
    o1.rover_a = 1; o1.rover_b = 2;
    o1.keyframe_a = NodeId(1, 10);
    o1.keyframe_b = NodeId(2, 20);
    o1.expected_correspondences = 8;
    mgr.add_opportunity(o1);

    // Opp 2: (1, 10) <-> (2, 21) consecutive with only 3 correspondences
    LoopClosureOpportunity o2;
    o2.rover_a = 1; o2.rover_b = 2;
    o2.keyframe_a = NodeId(1, 10);
    o2.keyframe_b = NodeId(2, 21);
    o2.expected_correspondences = 3;
    mgr.add_opportunity(o2);

    // Opp 3: (1, 15) <-> (2, 30) distant
    LoopClosureOpportunity o3;
    o3.rover_a = 1; o3.rover_b = 2;
    o3.keyframe_a = NodeId(1, 15);
    o3.keyframe_b = NodeId(2, 30);
    o3.expected_correspondences = 5;
    mgr.add_opportunity(o3);

    assert(mgr.size() == 3);
    size_t filtered = mgr.filter_redundant_opportunities();
    assert(filtered == 1);
    assert(mgr.size() == 2);

    // Opp 1 should be retained, Opp 2 pruned
    bool found_o1 = false;
    for (const auto& opp : mgr.opportunities()) {
        if (opp.keyframe_a == NodeId(1, 10) && opp.keyframe_b == NodeId(2, 20)) {
            found_o1 = true;
        }
    }
    assert(found_o1);

    std::cout << "[PASS] test_redundant_opportunity_filtering" << std::endl;
}

int main() {
    std::cout << "=================================================" << std::endl;
    std::cout << " AstroMesh-X: Loop Closure Opportunity Unit Tests" << std::endl;
    std::cout << "=================================================" << std::endl;

    test_opportunity_addition_and_deduplication();
    test_expiration_pruning();
    test_spatial_and_overlap_detection();
    test_redundant_opportunity_filtering();

    std::cout << "All Loop Closure Opportunity tests PASSED." << std::endl;
    return 0;
}
