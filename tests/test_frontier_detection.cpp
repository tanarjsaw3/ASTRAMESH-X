#include "astromesh/exploration/frontier.hpp"
#include "astromesh/graph/pose_graph.hpp"

#include <cassert>
#include <iostream>

using namespace astromesh;
using namespace astromesh::graph;
using namespace astromesh::verification;
using namespace astromesh::exploration;

void test_frontier_detection_lifecycle() {
    std::cout << "[RUN] test_frontier_detection_lifecycle...\n";

    FrontierDetectorConfig cfg;
    cfg.min_x = -20.0; cfg.max_x = 20.0;
    cfg.min_y = -20.0; cfg.max_y = 20.0;
    cfg.projection_distance = 5.0;
    cfg.num_angular_sectors = 4;
    cfg.default_ttl_steps = 10;

    TopologicalFrontierDetector detector(cfg);
    PoseGraph pg;
    std::vector<Point2D> landmarks;

    // Add initial rover keyframe at origin
    NodeId kf1(1, 1);
    pg.add_node(KeyframeNode(kf1, Pose2D(0.0, 0.0, 0.0)));

    auto detected = detector.detect_frontiers(pg, landmarks, 1, 0);
    assert(!detected.empty());
    assert(detector.active_frontiers().size() == detected.size());

    uint64_t fid = detected[0].frontier_id;
    assert(detector.find_frontier(fid) != nullptr);

    // Verify claim lifecycle
    assert(detector.claim_frontier(fid, 1, 5, 0));
    const auto* f_claimed = detector.find_frontier(fid);
    assert(f_claimed->status == FrontierStatus::Claimed);
    assert(f_claimed->claimed_by_rover == 1);

    // Release claim
    assert(detector.release_claim(fid, 1));
    const auto* f_released = detector.find_frontier(fid);
    assert(f_released->status == FrontierStatus::Active);

    // Mark completed
    assert(detector.mark_completed(fid));
    const auto* f_comp = detector.find_frontier(fid);
    assert(f_comp->status == FrontierStatus::Completed);

    std::cout << "  -> PASSED: Frontier detection and lifecycle verified.\n";
}

void test_bounds_and_duplicate_suppression() {
    std::cout << "[RUN] test_bounds_and_duplicate_suppression...\n";

    FrontierDetectorConfig cfg;
    cfg.min_x = -10.0; cfg.max_x = 10.0;
    cfg.min_y = -10.0; cfg.max_y = 10.0;
    cfg.projection_distance = 6.0;
    cfg.frontier_cluster_radius = 4.0;
    cfg.num_angular_sectors = 8;

    TopologicalFrontierDetector detector(cfg);
    PoseGraph pg;
    std::vector<Point2D> landmarks;

    // Node near boundary (8, 8) -> projections beyond (10, 10) must be rejected
    NodeId kf1(1, 1);
    pg.add_node(KeyframeNode(kf1, Pose2D(8.0, 8.0, 0.0)));

    auto detected = detector.detect_frontiers(pg, landmarks, 1, 0);
    for (const auto& f : detected) {
        assert(f.position.x >= -10.0 && f.position.x <= 10.0);
        assert(f.position.y >= -10.0 && f.position.y <= 10.0);
    }

    // Call detect again: duplicate clustering must suppress already tracked frontiers
    auto second_pass = detector.detect_frontiers(pg, landmarks, 1, 1);
    assert(second_pass.empty()); // All candidate projections within 4.0m of existing

    std::cout << "  -> PASSED: Bounds checking and duplicate suppression verified.\n";
}

void test_dynamic_completion_and_expiration() {
    std::cout << "[RUN] test_dynamic_completion_and_expiration...\n";

    FrontierDetectorConfig cfg;
    cfg.completion_radius = 3.0;
    cfg.default_ttl_steps = 15;

    TopologicalFrontierDetector detector(cfg);
    PoseGraph pg;
    std::vector<Point2D> landmarks;

    NodeId kf1(1, 1);
    pg.add_node(KeyframeNode(kf1, Pose2D(0.0, 0.0, 0.0)));

    auto detected = detector.detect_frontiers(pg, landmarks, 1, 0);
    assert(!detected.empty());
    Point2D target_pos = detected[0].position;

    // Add new keyframe directly at the frontier position
    NodeId kf2(1, 2);
    pg.add_node(KeyframeNode(kf2, Pose2D(target_pos.x, target_pos.y, 0.0)));

    // Update detector: frontier should dynamically transition to Completed
    detector.update_frontiers(pg, landmarks, 1);
    const auto* updated_f = detector.find_frontier(detected[0].frontier_id);
    assert(updated_f->status == FrontierStatus::Completed);

    // Verify expiration for unvisited frontiers
    size_t expired_count = detector.prune_expired(100); // 100 > 0 + 15
    assert(expired_count > 0);

    std::cout << "  -> PASSED: Dynamic completion and TTL expiration verified.\n";
}

int main() {
    std::cout << "==================================================\n";
    std::cout << " ASTROMESH-X M7.16: TOPOLOGICAL FRONTIER TESTS\n";
    std::cout << "==================================================\n";

    test_frontier_detection_lifecycle();
    test_bounds_and_duplicate_suppression();
    test_dynamic_completion_and_expiration();

    std::cout << "==================================================\n";
    std::cout << " ALL TOPOLOGICAL FRONTIER TESTS PASSED (100%)\n";
    std::cout << "==================================================\n";
    return 0;
}

