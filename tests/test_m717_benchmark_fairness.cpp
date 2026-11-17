#include "astromesh/simulation/synthetic_world.hpp"
#include "astromesh/core/landmark_key.hpp"
#include "astromesh/geometry/se2.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using namespace astromesh;
using namespace astromesh::simulation;
using namespace astromesh::geometry;

// Test 1: Verify deterministic seed reproducibility of synthetic world generation
static void test_synthetic_world_seed_determinism() {
    WorldConfig cfg;
    cfg.min_x = -50.0;
    cfg.max_x = 50.0;
    cfg.min_y = -50.0;
    cfg.max_y = 50.0;
    cfg.num_landmarks = 80;
    cfg.seed = 20261114ULL;

    SyntheticWorld2D world1(cfg);
    SyntheticWorld2D world2(cfg);

    // Verify exact landmark identity across runs with same seed
    const auto& lm1 = world1.landmarks();
    const auto& lm2 = world2.landmarks();
    assert(lm1.size() == lm2.size());

    for (size_t i = 0; i < lm1.size(); ++i) {
        assert(lm1[i].key == lm2[i].key);
        assert(std::abs(lm1[i].position.x - lm2[i].position.x) < 1e-9);
        assert(std::abs(lm1[i].position.y - lm2[i].position.y) < 1e-9);
    }

    // Query visibility from a test pose
    SE2 query_pose(10.0, 5.0, 0.2);
    auto vis1 = world1.query_visible_landmarks(query_pose);
    auto vis2 = world2.query_visible_landmarks(query_pose);

    assert(vis1.size() == vis2.size());
    for (size_t i = 0; i < vis1.size(); ++i) {
        assert(vis1[i].id == vis2[i].id);
        assert(vis1[i].key == vis2[i].key);
    }

    std::cout << "[PASS] SyntheticWorld2D seed determinism verified.\n";
}

// Test 2: Verify fairness contract - different seeds generate distinct challenges
static void test_seed_diversity() {
    WorldConfig cfg1;
    cfg1.seed = 1001ULL;
    SyntheticWorld2D world1(cfg1);

    WorldConfig cfg2;
    cfg2.seed = 1002ULL;
    SyntheticWorld2D world2(cfg2);

    const auto& lm1 = world1.landmarks();
    const auto& lm2 = world2.landmarks();
    
    // Different seeds must generate different landmark layouts
    bool all_identical = true;
    for (size_t i = 0; i < std::min(lm1.size(), lm2.size()); ++i) {
        if (std::abs(lm1[i].position.x - lm2[i].position.x) > 0.1 ||
            std::abs(lm1[i].position.y - lm2[i].position.y) > 0.1) {
            all_identical = false;
            break;
        }
    }
    assert(!all_identical);
    std::cout << "[PASS] Seed diversity verified: different seeds produce independent trials.\n";
}

int main() {
    std::cout << "Running test_m717_benchmark_fairness...\n";
    test_synthetic_world_seed_determinism();
    test_seed_diversity();
    std::cout << "All M7.17 benchmark fairness tests PASSED.\n";
    return 0;
}
