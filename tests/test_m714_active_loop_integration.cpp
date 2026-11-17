#include "astromesh/coordination/active_loop_scheduler.hpp"
#include "astromesh/coordination/loop_closure_opportunity.hpp"
#include "astromesh/coordination/rendezvous_protocol.hpp"
#include "astromesh/graph/alignment_graph.hpp"
#include "astromesh/graph/conflict_graph_merger.hpp"
#include "astromesh/net/gauge_coordinator.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>

using namespace astromesh;
using namespace astromesh::core;
using namespace astromesh::graph;
using namespace astromesh::geometry;
using namespace astromesh::verification;
using namespace astromesh::coordination;
using namespace astromesh::net;

static bool approx_equal(double a, double b, double eps = 1e-3) {
    return std::abs(a - b) <= eps;
}

void test_two_rover_active_loop_closure() {
    std::cout << "[RUN] test_two_rover_active_loop_closure...\n";

    LoopClosureOpportunityManager opp_mgr(1);
    ActiveLoopClosureScheduler scheduler(1);
    RendezvousProtocolAgent agent1(1);
    RendezvousProtocolAgent agent2(2);
    GaugeCoordinator gauge1(1), gauge2(2);
    PoseGraph pg1, pg2;

    // Ground truth: Rover 2 is offset by T_12_true = (15.0, 10.0, 0.3 rad)
    SE2 T_12_true(15.0, 10.0, 0.3);

    // Rover 1 has Keyframe (1, 5) at (10, 5, 0)
    KeyframeNode kf1(NodeId(1, 5), Pose2D(10.0, 5.0, 0.0));
    // Rover 2 has Keyframe (2, 8) at (0, 0, 0) in local frame
    KeyframeNode kf2(NodeId(2, 8), Pose2D(0.0, 0.0, 0.0));

    pg1.add_node(kf1);
    pg2.add_node(kf2);

    // 6 mutual landmarks in global frame
    std::map<LandmarkKey, Point2D> db1, db2;
    std::vector<LandmarkKey> candidate_keys;
    std::vector<Point2D> opp_lms;

    for (uint32_t i = 1; i <= 6; ++i) {
        LandmarkKey key(1, 100, i, 0x5000 + i);
        candidate_keys.push_back(key);

        double gx = 12.0 + static_cast<double>(i) * 2.0;
        double gy = (i % 2 == 0) ? (8.0 + static_cast<double>(i) * 2.0) : (4.0 - static_cast<double>(i) * 1.0);

        // Rover 1 local: identical to global
        db1[key] = Point2D(gx, gy);
        opp_lms.push_back(Point2D(gx, gy));

        // Rover 2 local: p_r2 = T_12_true^{-1} * p_global
        double c = T_12_true.cos_theta();
        double s = T_12_true.sin_theta();
        double dx = gx - T_12_true.x();
        double dy = gy - T_12_true.y();
        db2[key] = Point2D(c * dx + s * dy, -s * dx + c * dy);
    }

    // Phase 1: Opportunity detection
    LoopClosureOpportunity opp;
    opp.opportunity_id = 1;
    opp.rover_a = 1;
    opp.rover_b = 2;
    opp.keyframe_a = NodeId(1, 5);
    opp.keyframe_b = NodeId(2, 8);
    opp.type = OpportunityType::SubgraphOverlap;
    opp.expected_correspondences = 6;
    opp.expected_wire_cost_bytes = 144;
    assert(opp_mgr.add_opportunity(opp));

    // Phase 2: Scheduling
    std::map<uint64_t, std::vector<Point2D>> opp_landmarks;
    opp_landmarks[opp.opportunity_id] = opp_lms;

    auto scheduled = scheduler.schedule_batch(opp_mgr, pg1, opp_landmarks, 500, 2);
    assert(scheduled.size() == 1);
    assert(scheduled[0].priority_score > 0.0);

    // Phase 3: Rendezvous exchange
    auto inq = agent1.create_inquiry(scheduled[0].opportunity_id, 2, NodeId(1, 5), NodeId(2, 8), candidate_keys);
    auto resp = agent2.handle_inquiry(inq, pg2, db2);
    assert(resp.status == 0);

    // Phase 4: Local geometric verification
    auto prop_opt = agent1.process_response(resp, db1);
    assert(prop_opt.has_value());
    assert(prop_opt->inlier_count >= 5);
    assert(prop_opt->inlier_rmse < 0.05);

    // Check that relative pose matches ground truth T_12_true
    assert(approx_equal(prop_opt->tx, T_12_true.x(), 0.05));
    assert(approx_equal(prop_opt->ty, T_12_true.y(), 0.05));
    assert(approx_equal(prop_opt->theta, T_12_true.theta(), 0.02));

    // Phase 5: Register alignment with GaugeCoordinator
    SE2 verified_tx(prop_opt->tx, prop_opt->ty, prop_opt->theta);
    gauge1.register_pairwise_alignment(2, 1, verified_tx, 6, prop_opt->inlier_count,
                                       prop_opt->inlier_rmse, prop_opt->confidence, 1);
    gauge2.register_pairwise_alignment(2, 1, verified_tx, 6, prop_opt->inlier_count,
                                       prop_opt->inlier_rmse, prop_opt->confidence, 1);

    // Both should elect min(1, 2) = 1 as Anchor
    assert(gauge1.current_anchor_id() == 1);
    assert(gauge2.current_anchor_id() == 1);
    assert(!gauge2.is_anchor());

    // Local kf2 (0,0) on Rover 2 transformed to global frame matches (15, 10, 0.3)
    Pose2D kf2_global = gauge2.transform_local_pose_to_global(kf2.pose);
    assert(approx_equal(kf2_global.x, 15.0, 0.05));
    assert(approx_equal(kf2_global.y, 10.0, 0.05));
    assert(approx_equal(kf2_global.theta, 0.3, 0.02));

    std::cout << "[PASS] test_two_rover_active_loop_closure\n";
}

void test_adversarial_aliasing_rejection() {
    std::cout << "[RUN] test_adversarial_aliasing_rejection...\n";

    RendezvousProtocolAgent agent1(1);
    RendezvousProtocolAgent agent2(2);
    PoseGraph pg2;
    pg2.add_node(KeyframeNode(NodeId(2, 1), Pose2D(0.0, 0.0, 0.0)));

    // Create perceptually aliased landmarks: keys match, but coordinates are scrambled/inconsistent
    std::map<LandmarkKey, Point2D> db1, db2;
    std::vector<LandmarkKey> candidate_keys;

    for (uint32_t i = 1; i <= 6; ++i) {
        LandmarkKey key(1, 100, i, 0xDEAD0000 + i);
        candidate_keys.push_back(key);

        // Rover 1 has points along a grid
        db1[key] = Point2D(static_cast<double>(i) * 5.0, static_cast<double>(i) * 3.0);

        // Rover 2 has scrambled/contradictory points that violate rigid transformation
        if (i % 2 == 0) {
            db2[key] = Point2D(-50.0 + static_cast<double>(i), 80.0 - static_cast<double>(i));
        } else {
            db2[key] = Point2D(100.0 - static_cast<double>(i), -40.0 + static_cast<double>(i));
        }
    }

    auto inq = agent1.create_inquiry(77, 2, NodeId(1, 1), NodeId(2, 1), candidate_keys);
    auto resp = agent2.handle_inquiry(inq, pg2, db2);
    assert(resp.status == 0); // Responder found the keys

    // Initiator runs geometric verification: RANSAC should FAIL to find consensus
    auto prop_opt = agent1.process_response(resp, db1, 0.25);
    assert(!prop_opt.has_value()); // Strictly rejected, zero false acceptances

    std::cout << "[PASS] test_adversarial_aliasing_rejection\n";
}

void test_four_rover_prioritization_under_budget() {
    std::cout << "[RUN] test_four_rover_prioritization_under_budget...\n";

    LoopClosureOpportunityManager opp_mgr(1);
    ActiveLoopClosureScheduler scheduler(1);
    PoseGraph graph;

    // Set peer link conditions: Rover 2 has good link, Rover 3 has high loss
    scheduler.set_peer_link_state(2, {0.02, 15.0, 0.95});
    scheduler.set_peer_link_state(3, {0.50, 400.0, 0.40}); // Bad link

    std::vector<Point2D> lms = {{-2, -2}, {2, -2}, {2, 2}, {-2, 2}};
    std::map<uint64_t, std::vector<Point2D>> opp_landmarks;

    // Opp 1 with Rover 2 (good link)
    LoopClosureOpportunity o1;
    o1.opportunity_id = 1;
    o1.rover_a = 1; o1.rover_b = 2;
    o1.keyframe_a = NodeId(1, 1); o1.keyframe_b = NodeId(2, 1);
    o1.expected_correspondences = 6;
    o1.expected_wire_cost_bytes = 150;
    opp_mgr.add_opportunity(o1);
    opp_landmarks[1] = lms;

    // Opp 2 with Rover 3 (bad link)
    LoopClosureOpportunity o2;
    o2.opportunity_id = 2;
    o2.rover_a = 1; o2.rover_b = 3;
    o2.keyframe_a = NodeId(1, 2); o2.keyframe_b = NodeId(3, 1);
    o2.expected_correspondences = 6;
    o2.expected_wire_cost_bytes = 150;
    opp_mgr.add_opportunity(o2);
    opp_landmarks[2] = lms;

    // Opp 3 with Rover 4 (nominal)
    LoopClosureOpportunity o3;
    o3.opportunity_id = 3;
    o3.rover_a = 1; o3.rover_b = 4;
    o3.keyframe_a = NodeId(1, 3); o3.keyframe_b = NodeId(4, 1);
    o3.expected_correspondences = 8;
    o3.expected_wire_cost_bytes = 180;
    opp_mgr.add_opportunity(o3);
    opp_landmarks[3] = lms;

    // Schedule batch with budget = 350 bytes and max 2
    auto batch = scheduler.schedule_batch(opp_mgr, graph, opp_landmarks, 350, 2);
    assert(batch.size() == 2);

    // Opp with Rover 3 (bad link) should NOT be in the top scheduled batch
    bool scheduled_r3 = false;
    for (const auto& b : batch) {
        if (b.rover_b == 3) scheduled_r3 = true;
    }
    assert(!scheduled_r3);

    std::cout << "[PASS] test_four_rover_prioritization_under_budget\n";
}

int main() {
    std::cout << "===========================================================\n";
    std::cout << " AstroMesh-X: M7.14 Active Loop Integration Tests\n";
    std::cout << "===========================================================\n";

    test_two_rover_active_loop_closure();
    test_adversarial_aliasing_rejection();
    test_four_rover_prioritization_under_budget();

    std::cout << "All M7.14 Active Loop Integration tests PASSED successfully.\n";
    return 0;
}
