#include "astromesh/exploration/exploration_utility.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace astromesh;
using namespace astromesh::verification;
using namespace astromesh::exploration;
using namespace astromesh::memory;

void test_dispersion_computation() {
    std::cout << "[RUN] test_dispersion_computation...\n";

    // Collinear points -> det should be ~0
    std::vector<Point2D> collinear = {
        Point2D(0.0, 0.0), Point2D(1.0, 1.0), Point2D(2.0, 2.0), Point2D(3.0, 3.0)
    };
    double det_col = ExplorationUtilityModel::compute_dispersion(collinear);
    assert(det_col < 1e-4);

    // Well-dispersed 2D cluster -> det should be clearly positive
    std::vector<Point2D> dispersed = {
        Point2D(0.0, 0.0), Point2D(5.0, 0.0), Point2D(0.0, 5.0), Point2D(5.0, 5.0)
    };
    double det_disp = ExplorationUtilityModel::compute_dispersion(dispersed);
    assert(det_disp > 1.0);

    std::cout << "  -> PASSED: 2D dispersion computation verified.\n";
}

void test_connectivity_and_memory_penalties() {
    std::cout << "[RUN] test_connectivity_and_memory_penalties...\n";

    ExplorationUtilityModel model;
    Frontier f;
    f.frontier_id = 1;
    f.position = Point2D(10.0, 0.0);
    f.spatial_coverage_radius = 5.0;

    ExplorationContext ctx_normal;
    ctx_normal.rover_position = Point2D(0.0, 0.0);
    ctx_normal.comm_range_meters = 15.0;
    ctx_normal.active_peers = { Point2D(5.0, 0.0) }; // Close peer (distance to f is 5m <= 15m)
    ctx_normal.memory_pressure = MemoryPressureLevel::Normal;

    auto res_normal = model.evaluate_frontier(f, ctx_normal, {});
    assert(res_normal.connectivity_risk < 1.0);
    assert(res_normal.memory_penalty == 0.0);
    assert(res_normal.net_utility > 0.5);

    // Context with severed link: peer at (-20, 0), distance to f is 30m > 15m
    ExplorationContext ctx_severed = ctx_normal;
    ctx_severed.active_peers = { Point2D(-20.0, 0.0) };
    auto res_severed = model.evaluate_frontier(f, ctx_severed, {});
    assert(res_severed.connectivity_risk > 1.0); // Super-linear breach penalty
    assert(res_severed.net_utility < res_normal.net_utility);

    // Context with Critical memory pressure
    ExplorationContext ctx_critical = ctx_normal;
    ctx_critical.memory_pressure = MemoryPressureLevel::Critical;
    auto res_critical = model.evaluate_frontier(f, ctx_critical, {});
    assert(res_critical.memory_penalty >= 5.0);
    assert(res_critical.net_utility < res_normal.net_utility * 0.5);

    std::cout << "  -> PASSED: Connectivity and memory penalty escalations verified.\n";
}

void test_peer_redundancy_suppression() {
    std::cout << "[RUN] test_peer_redundancy_suppression...\n";

    ExplorationUtilityModel model;
    Frontier f;
    f.frontier_id = 1;
    f.position = Point2D(12.0, 8.0);
    f.spatial_coverage_radius = 5.0;

    ExplorationContext ctx;
    ctx.rover_position = Point2D(0.0, 0.0);
    ctx.comm_range_meters = 20.0;
    ctx.active_peers = { Point2D(0.0, 0.0) };

    // Case 1: No peer claims
    auto ub1 = model.evaluate_frontier(f, ctx, {});
    assert(ub1.redundancy_penalty == 0.0);

    // Case 2: Peer claimed exact same target (12, 8)
    ctx.peer_claimed_targets = { Point2D(12.0, 8.0) };
    auto ub2 = model.evaluate_frontier(f, ctx, {});
    assert(ub2.redundancy_penalty > 0.8);
    assert(ub2.net_utility < ub1.net_utility);

    std::cout << "  -> PASSED: Peer redundancy penalty verified.\n";
}

int main() {
    std::cout << "==================================================\n";
    std::cout << " ASTROMESH-X M7.16: EXPLORATION UTILITY TESTS\n";
    std::cout << "==================================================\n";

    test_dispersion_computation();
    test_connectivity_and_memory_penalties();
    test_peer_redundancy_suppression();

    std::cout << "==================================================\n";
    std::cout << " ALL EXPLORATION UTILITY TESTS PASSED (100%)\n";
    std::cout << "==================================================\n";
    return 0;
}

