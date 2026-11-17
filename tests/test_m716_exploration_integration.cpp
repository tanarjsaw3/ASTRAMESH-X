#include "astromesh/exploration/integrated_exploration_agent.hpp"
#include "astromesh/graph/pose_graph.hpp"

#include <cassert>
#include <iostream>

using namespace astromesh;
using namespace astromesh::graph;
using namespace astromesh::verification;
using namespace astromesh::exploration;

void test_two_rover_autonomous_exploration_loop() {
    std::cout << "[RUN] test_two_rover_autonomous_exploration_loop...\n";

    IntegratedExplorationAgent agent1(1);
    IntegratedExplorationAgent agent2(2);

    PoseGraph pg1, pg2;
    Point2D pos1(-5.0, 0.0);
    Point2D pos2(5.0, 0.0);
    double head1 = 0.0, head2 = M_PI;

    // Initial keyframes
    pg1.add_node(KeyframeNode(NodeId(1, 1), Pose2D(pos1.x, pos1.y, head1)));
    pg2.add_node(KeyframeNode(NodeId(2, 1), Pose2D(pos2.x, pos2.y, head2)));

    // Run 10 exploration steps
    for (uint64_t step = 1; step <= 10; ++step) {
        // 1. Agent 1 planning
        auto dec1 = agent1.step(step, pos1, head1, pg1, {}, {pos2}, Point2D(0, 0));
        assert(dec1.target_position.is_valid());

        // 2. Agent 2 planning
        auto dec2 = agent2.step(step, pos2, head2, pg2, {}, {pos1}, Point2D(0, 0));
        assert(dec2.target_position.is_valid());

        // 3. Exchange claims between agents
        if (dec1.has_claim) {
            agent2.handle_remote_claim(dec1.claim, step);
        }
        if (dec2.has_claim) {
            agent1.handle_remote_claim(dec2.claim, step);
        }

        // 4. Move rovers toward next waypoint (speed: 1.0 m/s)
        double dx1 = dec1.next_waypoint.x - pos1.x;
        double dy1 = dec1.next_waypoint.y - pos1.y;
        double dist1 = std::hypot(dx1, dy1);
        if (dist1 > 1.0) {
            pos1.x += (dx1 / dist1) * 1.0;
            pos1.y += (dy1 / dist1) * 1.0;
        } else {
            pos1 = dec1.next_waypoint;
        }

        double dx2 = dec2.next_waypoint.x - pos2.x;
        double dy2 = dec2.next_waypoint.y - pos2.y;
        double dist2 = std::hypot(dx2, dy2);
        if (dist2 > 1.0) {
            pos2.x += (dx2 / dist2) * 1.0;
            pos2.y += (dy2 / dist2) * 1.0;
        } else {
            pos2 = dec2.next_waypoint;
        }

        // 5. Append keyframes periodically
        if (step % 2 == 0) {
            NodeId k1(1, static_cast<uint32_t>(step / 2 + 1));
            pg1.add_node(KeyframeNode(k1, Pose2D(pos1.x, pos1.y, head1)));

            NodeId k2(2, static_cast<uint32_t>(step / 2 + 1));
            pg2.add_node(KeyframeNode(k2, Pose2D(pos2.x, pos2.y, head2)));
        }
    }

    assert(pg1.num_nodes() >= 5);
    assert(pg2.num_nodes() >= 5);
    assert(agent1.detector().active_frontiers().size() > 0);
    assert(agent2.detector().active_frontiers().size() > 0);

    std::cout << "  -> PASSED: 10-step multi-rover autonomous exploration loop verified.\n";
}

int main() {
    std::cout << "===========================================================\n";
    std::cout << " ASTROMESH-X M7.16: INTEGRATED EXPLORATION TESTS\n";
    std::cout << "===========================================================\n";

    test_two_rover_autonomous_exploration_loop();

    std::cout << "===========================================================\n";
    std::cout << " ALL INTEGRATED EXPLORATION TESTS PASSED (100%)\n";
    std::cout << "===========================================================\n";
    return 0;
}

