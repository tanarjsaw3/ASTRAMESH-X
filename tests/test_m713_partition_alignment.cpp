#include "astromesh/geometry/subgraph_aligner.hpp"
#include "astromesh/graph/conflict_graph_merger.hpp"
#include "astromesh/net/gauge_coordinator.hpp"
#include "astromesh/net/partition_reconciler.hpp"
#include "astromesh/verification/global_consistency_auditor.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>

using namespace astromesh;
using namespace astromesh::core;
using namespace astromesh::graph;
using namespace astromesh::geometry;
using namespace astromesh::net;
using namespace astromesh::verification;

static bool approx_equal(double a, double b, double eps = 1e-3) {
    return std::abs(a - b) <= eps;
}

static MapUpdate make_kf_update(uint16_t rover_id, uint32_t kf_idx, double x, double y, double theta = 0.0) {
    MapUpdate u;
    u.origin_rover_id = rover_id;
    u.type = MapUpdateType::KeyframeAdded;
    u.node_id = NodeId(rover_id, kf_idx);
    u.transform = SE2(x, y, theta);
    u.landmark_key = LandmarkKey(1, rover_id, kf_idx, static_cast<uint64_t>(kf_idx * 10007ULL + rover_id * 31ULL));
    return u;
}

static MapUpdate make_edge_update(uint16_t origin, uint64_t seq, NodeId from, NodeId to, EdgeType type, double x, double y) {
    MapUpdate u;
    u.origin_rover_id = origin;
    u.sequence = seq;
    u.update_id = (static_cast<uint64_t>(origin) << 48) | (seq & 0xFFFFFFFFFFFFULL);
    u.type = MapUpdateType::EdgeVerified;
    u.node_id = from;
    u.target_node_id = to;
    u.edge_type = type;
    u.transform = SE2(x, y, 0.0);
    u.inlier_count = 25;
    u.inlier_rmse = 0.02;
    return u;
}

void test_two_rover_partition_gauge_alignment() {
    std::cout << "[RUN] test_two_rover_partition_gauge_alignment...\n";

    PartitionAwareReconciler r1(1), r2(2);
    ConflictAwareGraphMerger m1, m2;
    GaugeCoordinator g1(1), g2(2);

    // Rover 1 is at Global Origin (0, 0, 0)
    // Rover 2 has a local coordinate frame offset from Rover 1 by T_12_true = (15.0, 8.0, 0.40 rad)
    SE2 T_12_true(15.0, 8.0, 0.40);

    // 8 Landmarks in global frame
    std::vector<std::pair<double, double>> global_landmarks = {
        {10.0, 5.0}, {12.0, 8.0}, {14.0, 4.0}, {16.0, 10.0},
        {18.0, 6.0}, {20.0, 9.0}, {11.0, 12.0}, {15.0, 15.0}
    };

    // Both rovers build local keyframes
    for (uint32_t i = 1; i <= 8; ++i) {
        // Rover 1 local coordinates match global frame
        auto u1 = make_kf_update(1, i, static_cast<double>(i) * 2.0, 2.0);
        r1.record_local_update(u1);
        m1.apply_update(u1);

        // Rover 2 local coordinates are in Rover 2's shifted local frame
        // p_r2 = T_12_true^{-1} * p_global
        SE2 p_global_r2(15.0 + static_cast<double>(i), 8.0 + static_cast<double>(i) * 0.5, 0.40);
        SE2 p_local_r2 = T_12_true.inverse() * p_global_r2;

        auto u2 = make_kf_update(2, i, p_local_r2.x(), p_local_r2.y(), p_local_r2.theta());
        r2.record_local_update(u2);
        m2.apply_update(u2);
    }

    // Intra-rover odometry
    for (uint32_t i = 1; i < 8; ++i) {
        auto e1 = make_edge_update(1, i + 10, NodeId(1, i), NodeId(1, i + 1), EdgeType::Odometry, 2.0, 0.0);
        r1.record_local_update(e1);
        m1.apply_update(e1);

        auto e2 = make_edge_update(2, i + 10, NodeId(2, i), NodeId(2, i + 1), EdgeType::Odometry, 1.0, 0.5);
        r2.record_local_update(e2);
        m2.apply_update(e2);
    }

    // Both observe the 8 landmarks in their respective local frames
    std::vector<SubgraphLandmarkObservation> submap1, submap2;
    for (size_t k = 0; k < global_landmarks.size(); ++k) {
        LandmarkKey key(1, 100, static_cast<uint32_t>(k + 1), 0xABCD0000ULL + k);

        // Rover 1 local observation
        submap1.push_back({key, verification::Point2D(global_landmarks[k].first, global_landmarks[k].second)});

        // Rover 2 local observation: p_r2 = T_12_true^{-1} * p_global
        double c = T_12_true.cos_theta();
        double s = T_12_true.sin_theta();
        double dx = global_landmarks[k].first - T_12_true.x();
        double dy = global_landmarks[k].second - T_12_true.y();
        double r2_lx = c * dx + s * dy;
        double r2_ly = -s * dx + c * dy;
        submap2.push_back({key, verification::Point2D(r2_lx, r2_ly)});
    }

    // Phase 1: Ledgers reconcile across partition
    ReconcilerMetrics rec_metrics;
    auto rec_status = r1.execute_multi_round_recovery(r2, rec_metrics);
    assert(rec_status == ReconciliationStatus::Completed);
    for (const auto& u : r1.ledger()) {
        m1.apply_update(u);
        m2.apply_update(u);
    }

    // Phase 2: Estimate relative gauge transform using SubgraphAligner
    // Aligns submap2 into submap1 frame: returns T_12 such that p1 = T_12 * p2
    auto align_res = SubgraphAligner::align_subgraphs(1, submap1, 2, submap2, 0.15);
    assert(align_res.success);
    assert(align_res.inlier_count >= 6);
    assert(align_res.inlier_rmse < 0.05);

    // Verify estimated SE(2) transform matches ground truth T_12_true within millimeters
    assert(approx_equal(align_res.transform_target_from_source.x(), T_12_true.x(), 0.05));
    assert(approx_equal(align_res.transform_target_from_source.y(), T_12_true.y(), 0.05));
    assert(approx_equal(align_res.transform_target_from_source.theta(), T_12_true.theta(), 0.02));

    // Phase 3: Register alignment with GaugeCoordinators
    g1.register_pairwise_alignment(2, 1, align_res.transform_target_from_source,
                                   align_res.correspondence_count,
                                   align_res.inlier_count,
                                   align_res.inlier_rmse,
                                   align_res.confidence,
                                   1);
    g2.register_pairwise_alignment(2, 1, align_res.transform_target_from_source,
                                   align_res.correspondence_count,
                                   align_res.inlier_count,
                                   align_res.inlier_rmse,
                                   align_res.confidence,
                                   1);

    // Check anchor election: min(1, 2) = 1
    assert(g1.current_anchor_id() == 1);
    assert(g1.is_anchor());
    assert(g2.current_anchor_id() == 1);
    assert(!g2.is_anchor());
    assert(g2.is_aligned());

    // Phase 4: Transform Rover 2's local landmark observations to global frame
    for (size_t k = 0; k < global_landmarks.size(); ++k) {
        double r2_lx = submap2[k].local_pos.x;
        double r2_ly = submap2[k].local_pos.y;
        auto [g_lx, g_ly] = g2.transform_landmark_to_global(r2_lx, r2_ly);

        // Verify that global landmark coordinates match Rover 1 / ground truth
        assert(approx_equal(g_lx, global_landmarks[k].first, 0.05));
        assert(approx_equal(g_ly, global_landmarks[k].second, 0.05));
    }

    std::cout << "[PASS] test_two_rover_partition_gauge_alignment\n";
}

void test_three_rover_linear_relay_gauge_alignment() {
    std::cout << "[RUN] test_three_rover_linear_relay_gauge_alignment...\n";

    // 3 rovers: Rover 1 (Anchor) <-> Rover 2 (Relay) <-> Rover 3
    // Rover 1 is at origin.
    // Rover 2 has offset T_12 = (10.0, 5.0, 0.2 rad)
    // Rover 3 has offset T_23 = (15.0, -3.0, -0.15 rad) relative to Rover 2
    // Rover 3 offset relative to Rover 1: T_13 = T_12 * T_23
    SE2 t12(10.0, 5.0, 0.2);
    SE2 t23(15.0, -3.0, -0.15);
    SE2 expected_t13 = t12 * t23;

    GaugeCoordinator g1(1), g2(2), g3(3);

    // Register T_12 on all rovers
    g1.register_pairwise_alignment(2, 1, t12, 10, 10, 0.01, 1.0, 1);
    g2.register_pairwise_alignment(2, 1, t12, 10, 10, 0.01, 1.0, 1);
    g3.register_pairwise_alignment(2, 1, t12, 10, 10, 0.01, 1.0, 1);

    // Register T_23 on all rovers (propagated across relay)
    g1.register_pairwise_alignment(3, 2, t23, 10, 10, 0.01, 1.0, 1);
    g2.register_pairwise_alignment(3, 2, t23, 10, 10, 0.01, 1.0, 1);
    g3.register_pairwise_alignment(3, 2, t23, 10, 10, 0.01, 1.0, 1);

    // All rovers must elect min(1, 2, 3) = 1 as Anchor
    assert(g1.current_anchor_id() == 1);
    assert(g2.current_anchor_id() == 1);
    assert(g3.current_anchor_id() == 1);

    // Rover 3 transform from anchor must equal T_12 * T_23
    SE2 g3_tx = g3.transform_from_anchor();
    assert(approx_equal(g3_tx.x(), expected_t13.x(), 1e-3));
    assert(approx_equal(g3_tx.y(), expected_t13.y(), 1e-3));
    assert(approx_equal(g3_tx.theta(), expected_t13.theta(), 1e-3));

    // Rover 3 origin (0, 0, 0) transformed to global frame matches expected_t13
    Pose2D r3_origin(0.0, 0.0, 0.0);
    Pose2D r3_global = g3.transform_local_pose_to_global(r3_origin);
    assert(approx_equal(r3_global.x, expected_t13.x(), 1e-3));
    assert(approx_equal(r3_global.y, expected_t13.y(), 1e-3));
    assert(approx_equal(r3_global.theta, expected_t13.theta(), 1e-3));

    std::cout << "[PASS] test_three_rover_linear_relay_gauge_alignment\n";
}

void test_four_rover_diamond_and_dynamic_anchor_failover() {
    std::cout << "[RUN] test_four_rover_diamond_and_dynamic_anchor_failover...\n";

    // Diamond topology: 1 <-> 2, 1 <-> 3, 2 <-> 4, 3 <-> 4
    SE2 t12(10.0, 0.0, 0.0);
    SE2 t13(0.0, 10.0, 0.0);
    SE2 t24(0.0, 10.0, 0.0);
    SE2 t34(10.0, 0.0, 0.0);

    GaugeCoordinator g2(2), g3(3), g4(4);

    // Broadcast all diamond edges
    for (auto* g : {&g2, &g3, &g4}) {
        g->register_pairwise_alignment(2, 1, t12, 15, 15, 0.01, 1.0, 1);
        g->register_pairwise_alignment(3, 1, t13, 15, 15, 0.01, 1.0, 1);
        g->register_pairwise_alignment(4, 2, t24, 15, 15, 0.01, 1.0, 1);
        g->register_pairwise_alignment(4, 3, t34, 15, 15, 0.01, 1.0, 1);
    }

    // Initially Anchor is 1 for all rovers
    assert(g2.current_anchor_id() == 1);
    assert(g3.current_anchor_id() == 1);
    assert(g4.current_anchor_id() == 1);

    // Now simulate Anchor 1 dropping offline / network partition isolating Anchor 1:
    // Invalidate all edges involving Rover 1 on remaining rovers {2, 3, 4}
    for (auto* g : {&g2, &g3, &g4}) {
        auto e21 = g->alignment_graph().get_edge(2, 1);
        if (e21) g->alignment_graph().invalidate_edge(e21->edge_id);

        auto e31 = g->alignment_graph().get_edge(3, 1);
        if (e31) g->alignment_graph().invalidate_edge(e31->edge_id);

        g->recompute_gauge();
    }

    // In remaining sub-swarm {2, 3, 4}, minimum ID is Rover 2!
    // Dynamic Anchor Failover must elect Rover 2 as the new Anchor
    assert(g2.current_anchor_id() == 2);
    assert(g2.is_anchor());
    assert(g3.current_anchor_id() == 2);
    assert(!g3.is_anchor());
    assert(g4.current_anchor_id() == 2);
    assert(!g4.is_anchor());

    // Rover 4 relative to new Anchor 2 should be T_24 = (0, 10, 0)
    SE2 g4_tx = g4.transform_from_anchor();
    assert(approx_equal(g4_tx.x(), 0.0, 1e-3));
    assert(approx_equal(g4_tx.y(), 10.0, 1e-3));
    assert(approx_equal(g4_tx.theta(), 0.0, 1e-3));

    std::cout << "[PASS] test_four_rover_diamond_and_dynamic_anchor_failover\n";
}

int main() {
    std::cout << "=========================================================\n";
    std::cout << " AstroMesh-X: M7.13 End-to-End Metric Alignment Tests\n";
    std::cout << "=========================================================\n";

    test_two_rover_partition_gauge_alignment();
    test_three_rover_linear_relay_gauge_alignment();
    test_four_rover_diamond_and_dynamic_anchor_failover();

    std::cout << "All M7.13 End-to-End Metric Alignment tests PASSED successfully.\n";
    return 0;
}
