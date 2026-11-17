#include "astromesh/geometry/se2.hpp"
#include "astromesh/geometry/subgraph_aligner.hpp"
#include "astromesh/graph/alignment_graph.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/memory/landmark_retention.hpp"
#include "astromesh/memory/map_compactor.hpp"
#include "astromesh/memory/memory_budget_controller.hpp"
#include "astromesh/memory/memory_tracker.hpp"
#include "astromesh/net/gauge_coordinator.hpp"

#include <cassert>
#include <iostream>

using namespace astromesh;
using namespace astromesh::core;
using namespace astromesh::graph;
using namespace astromesh::geometry;
using namespace astromesh::memory;
using namespace astromesh::net;
using namespace astromesh::verification;

void test_memory_bounded_slam_integration() {
    std::cout << "[RUN] test_memory_bounded_slam_integration..." << std::endl;

    PoseGraph graph;
    std::map<LandmarkKey, Point2D> landmarks;
    LandmarkLifecycleManager lm_mgr;

    // Simulate 20 keyframes along a trajectory
    for (uint32_t i = 1; i <= 20; ++i) {
        graph.add_node(KeyframeNode(NodeId(1, i), Pose2D((i - 1) * 2.0, 0, 0)));
        if (i > 1) {
            graph.add_edge(PoseGraphEdge(i - 1, NodeId(1, i - 1), NodeId(1, i),
                                         EdgeType::Odometry, Pose2D(2.0, 0, 0)));
        }

        // Add 5 landmarks per keyframe
        for (uint32_t l = 1; l <= 5; ++l) {
            uint32_t gid = (i - 1) * 3 + l;
            LandmarkKey key(1, 10, gid, 0x5000ULL + gid);
            Point2D pt((i - 1) * 2.0 + l * 0.5, l * 0.5);
            landmarks[key] = pt;
            lm_mgr.add_or_update_observation(key, NodeId(1, i), pt, 0.9);
        }
    }

    // Add a verified loop closure between (1, 5) and (1, 15)
    PoseGraphEdge lc(500, NodeId(1, 5), NodeId(1, 15),
                     EdgeType::IntraRoverLoopClosure, Pose2D(20.0, 0, 0),
                     InformationMatrix2D::diagonal(200.0, 2000.0),
                     EdgeVerificationStatus::Verified);
    graph.add_edge(lc);

    // Initial memory tracking
    auto pg_mem_initial = MemoryTracker::measure_pose_graph(graph);
    auto lm_mem_initial = MemoryTracker::measure_landmarks(landmarks);
    size_t total_initial = pg_mem_initial.capacity_bytes + lm_mem_initial.capacity_bytes;

    std::cout << "  -> Initial Nodes: " << graph.num_nodes()
              << ", Landmarks: " << landmarks.size()
              << ", Memory: " << total_initial << " B" << std::endl;

    // Configure budget controller with tight limit = 60% of initial
    MemoryBudgetConfig bcfg;
    bcfg.per_rover_budget_bytes = static_cast<size_t>(total_initial * 0.60);
    MemoryBudgetController controller(bcfg);

    // Pressure evaluation
    auto pressure = controller.evaluate_pressure(total_initial, 500);
    assert(pressure == MemoryPressureLevel::Critical);

    // Execute Map Compaction & Landmark Pruning
    CompactionConfig ccfg;
    ccfg.target_max_keyframes = 10;
    MapCompactor compactor(ccfg);

    auto comp_summary = compactor.compact_graph(graph);
    auto lm_summary = lm_mgr.prune_to_budget(30);

    auto compacted_lms = lm_mgr.export_active_landmarks();
    auto pg_mem_final = MemoryTracker::measure_pose_graph(graph);
    auto lm_mem_final = MemoryTracker::measure_landmarks(compacted_lms);
    size_t total_final = pg_mem_final.capacity_bytes + lm_mem_final.capacity_bytes;

    std::cout << "  -> Final Nodes: " << graph.num_nodes()
              << " (Pruned: " << comp_summary.nodes_pruned << ")"
              << ", Landmarks: " << compacted_lms.size()
              << " (Evicted: " << lm_summary.landmarks_evicted << ")"
              << ", Memory: " << total_final << " B" << std::endl;

    // Must be bounded below original
    assert(graph.num_nodes() <= 12);
    assert(total_final < total_initial);

    // Verified loop closure endpoints (1, 5) and (1, 15) must be preserved!
    assert(graph.has_node(NodeId(1, 5)));
    assert(graph.has_node(NodeId(1, 15)));
    assert(graph.has_edge(500));

    // Post-compaction pressure evaluation
    auto post_pressure = controller.evaluate_pressure(total_final, 500);
    assert(post_pressure != MemoryPressureLevel::Critical);

    std::cout << "[PASS] test_memory_bounded_slam_integration" << std::endl;
}

int main() {
    std::cout << "==========================================================" << std::endl;
    std::cout << " AstroMesh-X: Memory-Bounded SLAM Integration Test        " << std::endl;
    std::cout << "==========================================================" << std::endl;

    test_memory_bounded_slam_integration();

    std::cout << "All Memory-Bounded SLAM Integration tests PASSED." << std::endl;
    return 0;
}
