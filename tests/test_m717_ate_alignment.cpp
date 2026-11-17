#include "astromesh/geometry/se2.hpp"
#include "astromesh/geometry/subgraph_aligner.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/optimization/trajectory_metrics.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <vector>

using namespace astromesh;
using namespace astromesh::geometry;
using namespace astromesh::graph;
using namespace astromesh::optimization;

// Helper to compute independent per-rover ATE (the flawed/fraudulent method)
static double compute_fraudulent_per_rover_ate(
    const std::unordered_map<NodeId, SE2>& estimated,
    const std::unordered_map<NodeId, SE2>& ground_truth) {

    // Group nodes by rover
    std::unordered_map<uint16_t, std::unordered_map<NodeId, SE2>> est_by_rover;
    std::unordered_map<uint16_t, std::unordered_map<NodeId, SE2>> gt_by_rover;

    for (const auto& [nid, p] : estimated) {
        est_by_rover[nid.rover_id][nid] = p;
    }
    for (const auto& [nid, p] : ground_truth) {
        gt_by_rover[nid.rover_id][nid] = p;
    }

    double sum_sq_error = 0.0;
    size_t total_poses = 0;

    for (const auto& [rover_id, est_nodes] : est_by_rover) {
        auto it = gt_by_rover.find(rover_id);
        if (it == gt_by_rover.end()) continue;

        // Fraudulent step: Align EACH rover's subgraph independently!
        auto ate_rover = compute_ate(est_nodes, it->second, true /* align per rover */);
        assert(ate_rover.valid);
        sum_sq_error += ate_rover.translation.rmse * ate_rover.translation.rmse * ate_rover.num_poses;
        total_poses += ate_rover.num_poses;
    }

    return total_poses > 0 ? std::sqrt(sum_sq_error / static_cast<double>(total_poses)) : 0.0;
}

// Test 1: Adversarial verification proving per-rover alignment masks coordinate divergence
static void test_per_rover_vs_global_alignment_audit() {
    // Scenario: Rover 1 and Rover 2 each have a straight trajectory of 5 poses.
    // Ground truth:
    // Rover 1 is at x in [0, 4], y = 0
    // Rover 2 is at x in [10, 14], y = 0
    std::unordered_map<NodeId, SE2> ground_truth;
    for (uint32_t k = 0; k < 5; ++k) {
        ground_truth[NodeId(1, k)] = SE2(static_cast<double>(k), 0.0, 0.0);
        ground_truth[NodeId(2, k)] = SE2(10.0 + static_cast<double>(k), 0.0, 0.0);
    }

    // Now, create an ESTIMATE where:
    // Rover 1 has near-perfect local shape (minor 0.01m jitter).
    // Rover 2 has near-perfect local shape (minor 0.01m jitter).
    // BUT Rover 2's coordinate frame is offset by +15.0 meters in Y relative to Rover 1!
    // (A catastrophic inter-rover gauge failure: Rover 2 thinks it is 15 meters north!)
    std::unordered_map<NodeId, SE2> estimated;
    for (uint32_t k = 0; k < 5; ++k) {
        estimated[NodeId(1, k)] = SE2(static_cast<double>(k) + 0.01, 0.01, 0.0);
        // Rover 2 has a 15.0m internal gauge misalignment!
        estimated[NodeId(2, k)] = SE2(10.0 + static_cast<double>(k) + 0.01, 15.01, 0.0);
    }

    // 1. Evaluate with the fraudulent per-rover alignment:
    double per_rover_ate = compute_fraudulent_per_rover_ate(estimated, ground_truth);

    // 2. Evaluate with the mandatory single global rigid SE(2) alignment:
    auto global_ate_result = compute_ate(estimated, ground_truth, true /* single global rigid alignment */);
    assert(global_ate_result.valid);
    double global_ate = global_ate_result.translation.rmse;

    std::cout << "Fraudulent Per-Rover ATE: " << per_rover_ate << " m" << std::endl;
    std::cout << "Rigorous Global ATE:     " << global_ate << " m" << std::endl;

    // Crucial Invariant Checks:
    // Fraudulent per-rover alignment absorbs the 15m shift independently per rover,
    // falsely reporting a tiny error (< 0.05 m):
    assert(per_rover_ate < 0.05);

    // Single global rigid alignment CANNOT absorb both frames at once;
    // it correctly exposes the massive inter-rover divergence (> 4.0 m RMSE):
    assert(global_ate > 4.0);

    std::cout << "[PASS] Proved that single global rigid SE(2) alignment is necessary and prevents fraud.\n";
}

// Test 2: Frame Indifference Invariant (Global ATE is invariant under global coordinate shifts)
static void test_global_ate_frame_indifference() {
    std::unordered_map<NodeId, SE2> ground_truth;
    std::unordered_map<NodeId, SE2> estimated;

    for (uint32_t k = 0; k < 10; ++k) {
        ground_truth[NodeId(1, k)] = SE2(static_cast<double>(k), 0.5 * static_cast<double>(k), 0.0);
        estimated[NodeId(1, k)] = SE2(static_cast<double>(k) + 0.05, 0.5 * static_cast<double>(k) - 0.03, 0.01);
    }

    auto base_ate = compute_ate(estimated, ground_truth, true);
    assert(base_ate.valid);

    // Apply an arbitrary rigid shift G to the entire estimated map:
    SE2 G(100.0, -250.0, 1.25); // Large translation and rotation
    std::unordered_map<NodeId, SE2> shifted_estimated;
    for (const auto& [nid, p] : estimated) {
        shifted_estimated[nid] = G * p;
    }

    auto shifted_ate = compute_ate(shifted_estimated, ground_truth, true);
    assert(shifted_ate.valid);

    // Global rigid ATE must be identical down to floating point precision:
    assert(std::abs(base_ate.translation.rmse - shifted_ate.translation.rmse) < 1e-6);
    std::cout << "[PASS] Global rigid ATE frame indifference verified.\n";
}

int main() {
    std::cout << "Running test_m717_ate_alignment...\n";
    test_per_rover_vs_global_alignment_audit();
    test_global_ate_frame_indifference();
    std::cout << "All M7.17 ATE alignment tests PASSED.\n";
    return 0;
}
