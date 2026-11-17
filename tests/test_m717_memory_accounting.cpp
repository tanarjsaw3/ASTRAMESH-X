#include "astromesh/memory/memory_tracker.hpp"
#include "astromesh/memory/map_compactor.hpp"
#include "astromesh/graph/pose_graph.hpp"

#include <cassert>
#include <iostream>

using namespace astromesh;
using namespace astromesh::memory;
using namespace astromesh::graph;

// Test 1: Verify distinction between logical size, container capacity, and process RSS
static void test_memory_hierarchy_distinction() {
    // Add nodes to pose graph
    PoseGraph graph;
    for (uint32_t i = 0; i < 50; ++i) {
        graph.add_node(KeyframeNode(NodeId(1, i), Pose2D(static_cast<double>(i), 0.0, 0.0), 1000 + i));
    }
    for (uint32_t i = 0; i < 49; ++i) {
        graph.add_edge(PoseGraphEdge(100 + i, NodeId(1, i), NodeId(1, i + 1),
                                     EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0)));
    }

    auto record = MemoryTracker::measure_pose_graph(graph);
    size_t rss_bytes = get_process_rss_bytes();

    // Logical bytes > 0
    assert(record.logical_bytes > 0);
    // Container capacity bytes >= logical bytes
    assert(record.capacity_bytes >= record.logical_bytes);
    // Process RSS > 0
    assert(rss_bytes > 0);
    // Working set RSS covers the process and is larger than single graph
    assert(rss_bytes > record.logical_bytes);

    std::cout << "Logical Bytes:   " << record.logical_bytes << " B\n";
    std::cout << "Capacity Bytes:  " << record.capacity_bytes << " B\n";
    std::cout << "Process RSS:     " << rss_bytes << " B\n";
    std::cout << "[PASS] Memory accounting hierarchy rigorously distinguished.\n";
}

// Test 2: Verify conservative map compaction reduces logical graph size
static void test_compaction_reduces_logical_memory() {
    PoseGraph graph;
    // Linear chain of 10 nodes (0 -> 1 -> 2 -> ... -> 9)
    for (uint32_t i = 0; i < 10; ++i) {
        graph.add_node(KeyframeNode(NodeId(1, i), Pose2D(static_cast<double>(i), 0.0, 0.0), 1000 + i));
    }
    for (uint32_t i = 0; i < 9; ++i) {
        graph.add_edge(PoseGraphEdge(100 + i, NodeId(1, i), NodeId(1, i + 1),
                                     EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0)));
    }

    assert(graph.num_nodes() == 10);
    assert(graph.num_edges() == 9);

    CompactionConfig cfg;
    cfg.target_max_keyframes = 5;
    MapCompactor compactor(cfg);

    CompactionPassSummary res = compactor.compact_graph(graph);
    assert(res.nodes_pruned > 0);

    // Graph node count must have strictly decreased
    assert(graph.num_nodes() < 10);
    assert(graph.num_edges() < 9);

    std::cout << "Compaction pruned " << res.nodes_pruned << " nodes, net edges: "
              << graph.num_edges() << " (was 9).\n";
    std::cout << "[PASS] Topological map compaction reduces graph memory footprint.\n";
}

int main() {
    std::cout << "Running test_m717_memory_accounting...\n";
    test_memory_hierarchy_distinction();
    test_compaction_reduces_logical_memory();
    std::cout << "All M7.17 memory accounting tests PASSED.\n";
    return 0;
}
