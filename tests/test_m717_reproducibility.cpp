#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/simulation/synthetic_world.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using namespace astromesh;
using namespace astromesh::geometry;
using namespace astromesh::graph;
using namespace astromesh::simulation;

// Helper to simulate a multi-rover run deterministically
static std::vector<WorldLandmark> run_deterministic_trial(uint64_t seed) {
    WorldConfig cfg;
    cfg.seed = seed;
    cfg.num_landmarks = 40;
    cfg.min_x = -40.0;
    cfg.max_x = 40.0;
    cfg.min_y = -40.0;
    cfg.max_y = 40.0;

    SyntheticWorld2D world(cfg);
    return world.landmarks();
}

// Test 1: Bit-for-bit exact reproducibility across identical seeds
static void test_exact_seed_reproducibility() {
    uint64_t test_seed = 42ULL;

    auto run_A = run_deterministic_trial(test_seed);
    auto run_B = run_deterministic_trial(test_seed);

    assert(run_A.size() == run_B.size());
    assert(!run_A.empty());

    for (size_t i = 0; i < run_A.size(); ++i) {
        // Bit-level equivalence (must be identical)
        assert(run_A[i].id == run_B[i].id);
        assert(run_A[i].key == run_B[i].key);
        assert(run_A[i].position.x == run_B[i].position.x);
        assert(run_A[i].position.y == run_B[i].position.y);
    }

    std::cout << "[PASS] Exact bit-for-bit numerical reproducibility verified across identical seeds.\n";
}

// Test 2: Different seeds produce distinct landmark locations
static void test_seed_independence() {
    auto run_A = run_deterministic_trial(42ULL);
    auto run_C = run_deterministic_trial(1337ULL);

    assert(run_A.size() == run_C.size());
    bool has_difference = false;
    for (size_t i = 0; i < run_A.size(); ++i) {
        if (std::abs(run_A[i].position.x - run_C[i].position.x) > 1e-4 ||
            std::abs(run_A[i].position.y - run_C[i].position.y) > 1e-4) {
            has_difference = true;
            break;
        }
    }
    assert(has_difference);
    std::cout << "[PASS] Seed independence verified across different seeds.\n";
}

int main() {
    std::cout << "Running test_m717_reproducibility...\n";
    test_exact_seed_reproducibility();
    test_seed_independence();
    std::cout << "All M7.17 reproducibility tests PASSED.\n";
    return 0;
}
