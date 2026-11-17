#include "astromesh/exploration/integrated_exploration_agent.hpp"
#include "astromesh/graph/pose_graph.hpp"

#include <cassert>
#include <iostream>

using namespace astromesh;
using namespace astromesh::graph;
using namespace astromesh::verification;
using namespace astromesh::exploration;

void test_partition_local_autonomy() {
    std::cout << "[RUN] test_partition_local_autonomy...\n";

    IntegratedExplorationAgent agent1(1);
    IntegratedExplorationAgent agent2(2);

    PoseGraph pg1, pg2;
    pg1.add_node(KeyframeNode(NodeId(1, 1), Pose2D(0.0, 0.0, 0.0)));
    pg2.add_node(KeyframeNode(NodeId(2, 1), Pose2D(40.0, 40.0, 0.0)));

    // Rover 1 is at (0, 0), partitioned from Rover 2 (distance = 56m > 15m comm range)
    Point2D pos1(0.0, 0.0);
    Point2D pos2(40.0, 40.0);

    // Both rovers step autonomously in their disconnected partitions
    auto dec1 = agent1.step(1, pos1, 0.0, pg1, {}, {}, pos1);
    auto dec2 = agent2.step(1, pos2, 0.0, pg2, {}, {}, pos2);

    // Neither rover should stall or deadlock
    assert(dec1.status == PlannerStatus::Success || dec1.status == PlannerStatus::DegradedConnectivity);
    assert(dec2.status == PlannerStatus::Success || dec2.status == PlannerStatus::DegradedConnectivity);
    assert(dec1.target_position.is_valid());
    assert(dec2.target_position.is_valid());

    // Both rovers generated independent valid claims
    assert(dec1.has_claim);
    assert(dec2.has_claim);
    assert(dec1.claim.rover_id == 1);
    assert(dec2.claim.rover_id == 2);

    std::cout << "  -> PASSED: Autonomous exploration under network partitions verified.\n";
}

void test_partition_reconnection_and_reconciliation() {
    std::cout << "[RUN] test_partition_reconnection_and_reconciliation...\n";

    IntegratedExplorationAgent agent1(1);
    IntegratedExplorationAgent agent2(2);

    // Rover 1 has claim on (15, 15) with utility 0.85
    FrontierClaim claim1;
    claim1.frontier_id = 1001;
    claim1.rover_id = 1;
    claim1.target_x = 15.0;
    claim1.target_y = 15.0;
    claim1.utility_score = 0.85;
    claim1.lease_ttl_steps = 30;
    claim1.epoch = 1;

    // Rover 2 had an overlapping claim on (16, 15) with lower utility 0.65
    FrontierClaim claim2;
    claim2.frontier_id = 2001;
    claim2.rover_id = 2;
    claim2.target_x = 16.0;
    claim2.target_y = 15.0;
    claim2.utility_score = 0.65;
    claim2.lease_ttl_steps = 30;
    claim2.epoch = 2;

    agent2.coordinator().create_local_claim(2001, Point2D(16.0, 15.0), 0.65, 10, 30);
    assert(agent2.coordinator().is_local_claim_active(10));

    // Reconnection occurs: Agent 2 receives dominant claim from Agent 1
    bool accepted = agent2.handle_remote_claim(claim1, 15);
    assert(accepted);

    // Agent 2 detects conflict with Agent 1's dominant claim
    uint16_t peer = 0;
    bool conflict = agent2.coordinator().has_peer_conflict(Point2D(16.0, 15.0), 0.65, 15, &peer);
    assert(conflict);
    assert(peer == 1);

    std::cout << "  -> PASSED: Claim reconciliation upon reconnection verified.\n";
}

int main() {
    std::cout << "===========================================================\n";
    std::cout << " ASTROMESH-X M7.16: PARTITION AUTONOMOUS EXPLORATION TESTS\n";
    std::cout << "===========================================================\n";

    test_partition_local_autonomy();
    test_partition_reconnection_and_reconciliation();

    std::cout << "===========================================================\n";
    std::cout << " ALL PARTITION AUTONOMOUS EXPLORATION TESTS PASSED (100%)\n";
    std::cout << "===========================================================\n";
    return 0;
}

