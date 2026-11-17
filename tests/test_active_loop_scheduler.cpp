#include "astromesh/coordination/active_loop_scheduler.hpp"

#include <cassert>
#include <iostream>
#include <vector>

using namespace astromesh;
using namespace astromesh::graph;
using namespace astromesh::verification;
using namespace astromesh::coordination;

void test_scheduler_scoring_and_link_sensitivity() {
    std::cout << "[RUN] test_scheduler_scoring_and_link_sensitivity..." << std::endl;

    ActiveLoopClosureScheduler scheduler(1);
    PoseGraph graph;
    std::vector<Point2D> lms = {{-3, -3}, {3, -3}, {3, 3}, {-3, 3}};

    LoopClosureOpportunity opp;
    opp.opportunity_id = 1;
    opp.rover_a = 1;
    opp.rover_b = 2;
    opp.keyframe_a = NodeId(1, 5);
    opp.keyframe_b = NodeId(2, 5);
    opp.expected_correspondences = 8;
    opp.expected_wire_cost_bytes = 160;

    // Nominal link: 0 loss, 10ms RTT
    scheduler.set_peer_link_state(2, {0.0, 10.0, 0.95});
    double score_nominal = scheduler.score_opportunity(opp, graph, lms);
    assert(score_nominal > 0.0);

    // Impaired link: 40% loss, 300ms RTT
    scheduler.set_peer_link_state(2, {0.40, 300.0, 0.60});
    double score_impaired = scheduler.score_opportunity(opp, graph, lms);
    assert(score_impaired < score_nominal);

    std::cout << "[PASS] test_scheduler_scoring_and_link_sensitivity" << std::endl;
}

void test_budget_constrained_batch_scheduling() {
    std::cout << "[RUN] test_budget_constrained_batch_scheduling..." << std::endl;

    ActiveLoopClosureScheduler scheduler(1);
    LoopClosureOpportunityManager mgr(1);
    PoseGraph graph;

    std::vector<Point2D> lms = {{-2, -2}, {2, -2}, {2, 2}, {-2, 2}};
    std::map<uint64_t, std::vector<Point2D>> opp_landmarks;

    // Create 4 opportunities each costing 200 bytes
    for (uint32_t i = 1; i <= 4; ++i) {
        LoopClosureOpportunity opp;
        opp.opportunity_id = i;
        opp.rover_a = 1;
        opp.rover_b = static_cast<uint16_t>(i + 1);
        opp.keyframe_a = NodeId(1, i);
        opp.keyframe_b = NodeId(static_cast<uint16_t>(i + 1), i);
        opp.expected_correspondences = i * 2;
        opp.expected_wire_cost_bytes = 200;
        mgr.add_opportunity(opp);
        opp_landmarks[i] = lms;
    }

    // Schedule with budget = 500 bytes (can only fit 2 opportunities = 400 bytes)
    auto batch = scheduler.schedule_batch(mgr, graph, opp_landmarks, 500, 10);
    assert(batch.size() == 2);
    assert(scheduler.metrics().scheduled_wire_bytes == 400);

    // The 2 scheduled opportunities should be the highest-scoring ones (opp 4 and opp 3)
    assert(batch[0].opportunity_id == 4);
    assert(batch[1].opportunity_id == 3);

    std::cout << "[PASS] test_budget_constrained_batch_scheduling" << std::endl;
}

void test_deterministic_tie_breaking() {
    std::cout << "[RUN] test_deterministic_tie_breaking..." << std::endl;

    ActiveLoopClosureScheduler scheduler(1);
    LoopClosureOpportunityManager mgr(1);
    PoseGraph graph;

    std::vector<Point2D> lms = {{-2, -2}, {2, -2}, {2, 2}, {-2, 2}};
    std::map<uint64_t, std::vector<Point2D>> opp_landmarks;

    // Two identical opportunities except rover_b is 2 vs 3
    LoopClosureOpportunity o1;
    o1.opportunity_id = 1;
    o1.rover_a = 1; o1.rover_b = 3;
    o1.keyframe_a = NodeId(1, 1); o1.keyframe_b = NodeId(3, 1);
    o1.expected_correspondences = 5;
    o1.expected_wire_cost_bytes = 100;
    mgr.add_opportunity(o1);
    opp_landmarks[1] = lms;

    LoopClosureOpportunity o2;
    o2.opportunity_id = 2;
    o2.rover_a = 1; o2.rover_b = 2; // Lower rover ID
    o2.keyframe_a = NodeId(1, 1); o2.keyframe_b = NodeId(2, 1);
    o2.expected_correspondences = 5;
    o2.expected_wire_cost_bytes = 100;
    mgr.add_opportunity(o2);
    opp_landmarks[2] = lms;

    // Schedule 1
    auto batch = scheduler.schedule_batch(mgr, graph, opp_landmarks, 1000, 1);
    assert(batch.size() == 1);
    // Lower rover_b (2 < 3) should win tie-break
    assert(batch[0].rover_b == 2);

    std::cout << "[PASS] test_deterministic_tie_breaking" << std::endl;
}

int main() {
    std::cout << "===============================================" << std::endl;
    std::cout << " AstroMesh-X: Active Loop Scheduler Unit Tests" << std::endl;
    std::cout << "===============================================" << std::endl;

    test_scheduler_scoring_and_link_sensitivity();
    test_budget_constrained_batch_scheduling();
    test_deterministic_tie_breaking();

    std::cout << "All Active Loop Scheduler tests PASSED." << std::endl;
    return 0;
}

