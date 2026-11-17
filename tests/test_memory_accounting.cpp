#include "astromesh/memory/memory_tracker.hpp"

#include <cassert>
#include <iostream>

using namespace astromesh;
using namespace astromesh::core;
using namespace astromesh::graph;
using namespace astromesh::memory;
using namespace astromesh::verification;

void test_accounting_pose_graph_and_landmarks() {
    std::cout << "[RUN] test_accounting_pose_graph_and_landmarks..." << std::endl;

    PoseGraph graph;
    // Add 5 nodes and 4 odometry edges
    for (uint32_t i = 1; i <= 5; ++i) {
        graph.add_node(KeyframeNode(NodeId(1, i), Pose2D(i * 2.0, 0, 0)));
    }
    for (uint32_t i = 1; i <= 4; ++i) {
        graph.add_edge(PoseGraphEdge(i, NodeId(1, i), NodeId(1, i + 1),
                                     EdgeType::Odometry, Pose2D(2.0, 0, 0)));
    }

    auto pg_rec = MemoryTracker::measure_pose_graph(graph);
    assert(pg_rec.subsystem_name == "PoseGraph");
    assert(pg_rec.item_count == 9); // 5 nodes + 4 edges
    assert(pg_rec.logical_bytes > 0);
    assert(pg_rec.capacity_bytes >= pg_rec.logical_bytes);
    assert(pg_rec.serialized_wire_bytes > 0);

    // Landmarks
    std::map<LandmarkKey, Point2D> lms;
    for (uint32_t l = 1; l <= 10; ++l) {
        LandmarkKey k(1, 10, l, 0x12345678ULL + l);
        lms[k] = Point2D(l * 1.5, l * 2.5);
    }

    auto lm_rec = MemoryTracker::measure_landmarks(lms);
    assert(lm_rec.subsystem_name == "Landmarks");
    assert(lm_rec.item_count == 10);
    assert(lm_rec.logical_bytes == 10 * (sizeof(LandmarkKey) + sizeof(Point2D)));
    assert(lm_rec.capacity_bytes > lm_rec.logical_bytes);
    assert(lm_rec.serialized_wire_bytes == 10 * 32);

    std::cout << "[PASS] test_accounting_pose_graph_and_landmarks" << std::endl;
}

void test_accounting_swarm_and_rss() {
    std::cout << "[RUN] test_accounting_swarm_and_rss..." << std::endl;

    PoseGraph g1, g2;
    g1.add_node(KeyframeNode(NodeId(1, 1), Pose2D(0, 0, 0)));
    g2.add_node(KeyframeNode(NodeId(2, 1), Pose2D(10, 0, 0)));

    std::map<LandmarkKey, Point2D> l1, l2;
    AlignmentGraph ag1, ag2;
    coordination::LoopClosureOpportunityManager opp1(1), opp2(2);
    core::VersionVector vv1, vv2;
    comms::DtnBuffer dtn1, dtn2;
    DisjointSet dsu1, dsu2;

    auto r1 = MemoryTracker::generate_rover_report(1, g1, l1, ag1, opp1, vv1, 0, dtn1, dsu1);
    auto r2 = MemoryTracker::generate_rover_report(2, g2, l2, ag2, opp2, vv2, 0, dtn2, dsu2);

    SwarmMemorySnapshot swarm;
    swarm.rovers[1] = r1;
    swarm.rovers[2] = r2;
    swarm.process_rss_bytes = get_process_rss_bytes();

    assert(swarm.total_swarm_logical_bytes() == r1.total_logical_bytes() + r2.total_logical_bytes());
    assert(swarm.total_swarm_capacity_bytes() == r1.total_capacity_bytes() + r2.total_capacity_bytes());

#ifdef _WIN32
    // On Windows, working set size should be strictly positive for running process
    assert(swarm.process_rss_bytes > 0);
    std::cout << "  -> Measured Process RSS: " << (swarm.process_rss_bytes / 1024) << " KB" << std::endl;
#endif

    std::cout << "[PASS] test_accounting_swarm_and_rss" << std::endl;
}

int main() {
    std::cout << "===========================================" << std::endl;
    std::cout << " AstroMesh-X: Memory Accounting Unit Tests" << std::endl;
    std::cout << "===========================================" << std::endl;

    test_accounting_pose_graph_and_landmarks();
    test_accounting_swarm_and_rss();

    std::cout << "All Memory Accounting tests PASSED." << std::endl;
    return 0;
}
