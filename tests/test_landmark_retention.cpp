#include "astromesh/memory/landmark_retention.hpp"

#include <cassert>
#include <iostream>

using namespace astromesh;
using namespace astromesh::core;
using namespace astromesh::graph;
using namespace astromesh::memory;
using namespace astromesh::verification;

void test_landmark_observation_bounding_and_protection() {
    std::cout << "[RUN] test_landmark_observation_bounding_and_protection..." << std::endl;

    LandmarkRetentionConfig cfg;
    cfg.max_landmarks = 5;
    cfg.max_observations_per_landmark = 3;
    LandmarkLifecycleManager mgr(cfg);

    // Create 10 landmarks
    for (uint32_t i = 1; i <= 10; ++i) {
        LandmarkKey key(1, 10, i, 0x1000ULL + i);

        // Add 5 observations to each landmark (exceeding max 3)
        for (uint32_t k = 1; k <= 5; ++k) {
            mgr.add_or_update_observation(key, NodeId(1, k), Point2D(i * 1.0, k * 1.0), 0.5 + 0.1 * k);
        }
    }

    assert(mgr.landmark_count() == 10);
    assert(mgr.total_observations() == 50);

    // Protect landmark 1 and landmark 2 as loop closure support
    LandmarkKey k1(1, 10, 1, 0x1000ULL + 1);
    LandmarkKey k2(1, 10, 2, 0x1000ULL + 2);
    mgr.mark_as_loop_closure_support(k1, true);
    mgr.mark_as_loop_closure_support(k2, true);

    // Prune to budget of 5 landmarks
    auto summary = mgr.prune_to_budget(5);

    assert(mgr.landmark_count() == 5);
    assert(summary.landmarks_evicted == 5);
    assert(summary.protected_landmarks_retained >= 2);

    // Protected landmarks MUST still exist!
    assert(mgr.has_landmark(k1));
    assert(mgr.has_landmark(k2));

    // Per-landmark observations must be bounded <= 3
    const auto* r1 = mgr.get_record(k1);
    assert(r1 != nullptr);
    assert(r1->observations.size() <= 3);

    std::cout << "[PASS] test_landmark_observation_bounding_and_protection" << std::endl;
}

int main() {
    std::cout << "=============================================" << std::endl;
    std::cout << " AstroMesh-X: Landmark Retention Unit Tests  " << std::endl;
    std::cout << "=============================================" << std::endl;

    test_landmark_observation_bounding_and_protection();

    std::cout << "All Landmark Retention tests PASSED." << std::endl;
    return 0;
}

