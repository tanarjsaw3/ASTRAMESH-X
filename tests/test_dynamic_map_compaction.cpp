#include "astromesh/memory/map_compactor.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace astromesh;
using namespace astromesh::graph;
using namespace astromesh::memory;

void test_conservative_linear_chain_compaction() {
    std::cout << "[RUN] test_conservative_linear_chain_compaction..." << std::endl;

    PoseGraph graph;
    // 5 nodes in a line with step dx = 2.0m
    for (uint32_t i = 1; i <= 5; ++i) {
        graph.add_node(KeyframeNode(NodeId(1, i), Pose2D((i - 1) * 2.0, 0, 0)));
    }
    for (uint32_t i = 1; i <= 4; ++i) {
        graph.add_edge(PoseGraphEdge(i, NodeId(1, i), NodeId(1, i + 1),
                                     EdgeType::Odometry, Pose2D(2.0, 0, 0)));
    }

    assert(graph.num_nodes() == 5);
    assert(graph.num_edges() == 4);

    CompactionConfig cfg;
    cfg.target_max_keyframes = 3;
    cfg.max_compounded_distance_m = 20.0;
    MapCompactor compactor(cfg);

    auto summary = compactor.compact_graph(graph);

    // Should have pruned 2 nodes (nodes 2 and 3 or 4) to bring node count down to 3
    assert(summary.nodes_pruned == 2);
    assert(graph.num_nodes() == 3);

    // Frontier node (1, 5) and origin node (1, 1) must be preserved!
    assert(graph.has_node(NodeId(1, 1)));
    assert(graph.has_node(NodeId(1, 5)));

    // Total distance from (1, 1) to (1, 5) across remaining edges must equal 8.0m
    double total_dx = 0.0;
    for (const auto& [_, edge] : graph.edges()) {
        total_dx += edge.relative_pose.x;
    }
    assert(std::abs(total_dx - 8.0) < 1e-4);

    std::cout << "[PASS] test_conservative_linear_chain_compaction" << std::endl;
}

void test_protected_loop_closure_endpoints_preserved() {
    std::cout << "[RUN] test_protected_loop_closure_endpoints_preserved..." << std::endl;

    PoseGraph graph;
    for (uint32_t i = 1; i <= 6; ++i) {
        graph.add_node(KeyframeNode(NodeId(1, i), Pose2D((i - 1) * 2.0, 0, 0)));
    }
    for (uint32_t i = 1; i <= 5; ++i) {
        graph.add_edge(PoseGraphEdge(i, NodeId(1, i), NodeId(1, i + 1),
                                     EdgeType::Odometry, Pose2D(2.0, 0, 0)));
    }

    // Add a verified loop closure between Node(1, 2) and Node(1, 4)
    PoseGraphEdge lc_edge(100, NodeId(1, 2), NodeId(1, 4),
                          EdgeType::IntraRoverLoopClosure, Pose2D(4.0, 0, 0),
                          InformationMatrix2D::diagonal(200.0, 2000.0),
                          EdgeVerificationStatus::Verified);
    graph.add_edge(lc_edge);

    CompactionConfig cfg;
    cfg.target_max_keyframes = 3;
    MapCompactor compactor(cfg);

    auto summary = compactor.compact_graph(graph);

    // Node 2 and Node 4 are protected by the loop closure!
    // Node 1 is origin, Node 6 is frontier.
    // Therefore, only Node 3 or Node 5 could be compacted!
    assert(graph.has_node(NodeId(1, 2)));
    assert(graph.has_node(NodeId(1, 4)));
    assert(graph.has_edge(100)); // Verified loop closure edge intact

    std::cout << "[PASS] test_protected_loop_closure_endpoints_preserved" << std::endl;
}

int main() {
    std::cout << "=============================================" << std::endl;
    std::cout << " AstroMesh-X: Dynamic Map Compactor Unit Tests" << std::endl;
    std::cout << "=============================================" << std::endl;

    test_conservative_linear_chain_compaction();
    test_protected_loop_closure_endpoints_preserved();

    std::cout << "All Dynamic Map Compactor tests PASSED." << std::endl;
    return 0;
}
