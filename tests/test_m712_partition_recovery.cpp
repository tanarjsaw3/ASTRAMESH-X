#include "astromesh/graph/conflict_graph_merger.hpp"
#include "astromesh/net/partition_reconciler.hpp"
#include "astromesh/simulation/failure_injector.hpp"
#include "astromesh/verification/global_consistency_auditor.hpp"

#include <iostream>
#include <cassert>
#include <map>

using namespace astromesh;
using namespace astromesh::core;
using namespace astromesh::graph;
using namespace astromesh::net;
using namespace astromesh::simulation;
using namespace astromesh::verification;

static MapUpdate create_kf_update(uint16_t rover_id, uint32_t kf_idx, double x, double y) {
    MapUpdate u;
    u.origin_rover_id = rover_id;
    u.type = MapUpdateType::KeyframeAdded;
    u.node_id = NodeId(rover_id, kf_idx);
    u.transform = geometry::SE2(x, y, 0.0);
    u.landmark_key = LandmarkKey(1, rover_id, kf_idx, static_cast<uint64_t>(kf_idx * 10007ULL + rover_id * 31ULL));
    return u;
}

static MapUpdate create_edge_update(uint16_t origin, uint64_t seq, NodeId from, NodeId to, EdgeType type, double x, double y, uint32_t inliers, double rmse) {
    MapUpdate u;
    u.origin_rover_id = origin;
    u.sequence = seq;
    u.update_id = (static_cast<uint64_t>(origin) << 48) | (seq & 0xFFFFFFFFFFFFULL);
    u.type = MapUpdateType::EdgeVerified;
    u.node_id = from;
    u.target_node_id = to;
    u.edge_type = type;
    u.transform = geometry::SE2(x, y, 0.0);
    u.inlier_count = inliers;
    u.inlier_rmse = rmse;
    return u;
}

void test_two_rover_partition_recovery() {
    std::cout << "[RUN] test_two_rover_partition_recovery...\n";

    PartitionAwareReconciler r1(1), r2(2);
    ConflictAwareGraphMerger m1, m2;

    // Phase 1: Disconnected exploration (10 keyframes each)
    for (uint32_t i = 1; i <= 10; ++i) {
        auto u1 = create_kf_update(1, i, static_cast<double>(i), 0.0);
        r1.record_local_update(u1);
        m1.apply_update(u1);

        auto u2 = create_kf_update(2, i, 0.0, static_cast<double>(i));
        r2.record_local_update(u2);
        m2.apply_update(u2);
    }

    // Add intra-rover odometry edges
    for (uint32_t i = 1; i < 10; ++i) {
        auto e1 = create_edge_update(1, i + 10, NodeId(1, i), NodeId(1, i + 1), EdgeType::Odometry, 1.0, 0.0, 30, 0.01);
        r1.record_local_update(e1);
        m1.apply_update(e1);

        auto e2 = create_edge_update(2, i + 10, NodeId(2, i), NodeId(2, i + 1), EdgeType::Odometry, 0.0, 1.0, 30, 0.01);
        r2.record_local_update(e2);
        m2.apply_update(e2);
    }

    // Before reconnection: Swarm has 2 separate components
    std::map<uint16_t, const PartitionAwareReconciler*> rec_map = {{1, &r1}, {2, &r2}};
    std::map<uint16_t, const ConflictAwareGraphMerger*> mer_map = {{1, &m1}, {2, &m2}};

    auto audit_before = GlobalConsistencyAuditor::audit(rec_map, mer_map, 1);
    assert(!audit_before.is_converged);
    assert(audit_before.dsu_components == 2);
    assert(audit_before.pending_sync_items > 0);

    // Phase 2: Channel reopens -> Execute multi-round reconciliation
    ReconcilerMetrics metrics;
    auto status = r1.execute_multi_round_recovery(r2, metrics);
    assert(status == ReconciliationStatus::Completed);

    // Apply synchronized updates to graph mergers
    for (const auto& u : r1.ledger()) {
        m1.apply_update(u);
        m2.apply_update(u);
    }

    // Establish inter-rover loop closure between Node(1, 5) and Node(2, 5) originated by rover 1
    auto bridge = create_edge_update(1, 0, NodeId(1, 5), NodeId(2, 5), EdgeType::InterRoverLoopClosure, 1.0, 1.0, 20, 0.05);
    r1.record_local_update(bridge);
    const auto& recorded_bridge = r1.ledger().back();
    r2.apply_remote_update(recorded_bridge);
    m1.apply_update(recorded_bridge);
    m2.apply_update(recorded_bridge);

    // Audit after recovery
    auto audit_after = GlobalConsistencyAuditor::audit(rec_map, mer_map, 1);
    assert(audit_after.is_converged);
    assert(audit_after.dsu_components == 1);
    assert(audit_after.pending_sync_items == 0);
    assert(audit_after.reciprocal_edge_fraction >= 0.999);

    std::cout << "  -> PASSED: " << audit_after.diagnostic_summary << "\n";
}

void test_three_rover_relay_reconciliation() {
    std::cout << "[RUN] test_three_rover_relay_reconciliation...\n";

    PartitionAwareReconciler r1(1), r2(2), r3(3);
    ConflictAwareGraphMerger m1, m2, m3;

    // R1, R2, R3 generate keyframes
    for (uint32_t i = 1; i <= 5; ++i) {
        auto u1 = create_kf_update(1, i, static_cast<double>(i), 0.0);
        r1.record_local_update(u1); m1.apply_update(u1);

        auto u2 = create_kf_update(2, i, 5.0 + i, 0.0);
        r2.record_local_update(u2); m2.apply_update(u2);

        auto u3 = create_kf_update(3, i, 10.0 + i, 0.0);
        r3.record_local_update(u3); m3.apply_update(u3);
    }

    // Intra-rover odometry edges
    for (uint32_t i = 1; i < 5; ++i) {
        auto o1 = create_edge_update(1, 0, NodeId(1, i), NodeId(1, i + 1), EdgeType::Odometry, 1.0, 0.0, 30, 0.01);
        r1.record_local_update(o1); m1.apply_update(o1);
        auto o2 = create_edge_update(2, 0, NodeId(2, i), NodeId(2, i + 1), EdgeType::Odometry, 1.0, 0.0, 30, 0.01);
        r2.record_local_update(o2); m2.apply_update(o2);
        auto o3 = create_edge_update(3, 0, NodeId(3, i), NodeId(3, i + 1), EdgeType::Odometry, 1.0, 0.0, 30, 0.01);
        r3.record_local_update(o3); m3.apply_update(o3);
    }

    // Connect R1-R2 and R2-R3
    auto e12 = create_edge_update(1, 20, NodeId(1, 5), NodeId(2, 1), EdgeType::InterRoverLoopClosure, 1.0, 0.0, 15, 0.05);
    auto e23 = create_edge_update(2, 21, NodeId(2, 5), NodeId(3, 1), EdgeType::InterRoverLoopClosure, 1.0, 0.0, 18, 0.04);
    r1.record_local_update(e12);
    r2.record_local_update(e23);
    m1.apply_update(e12);
    m2.apply_update(e23);

    // Relay hops: R1 <-> R2, then R2 <-> R3, then R1 <-> R2 again
    ReconcilerMetrics met1, met2, met3;
    r1.execute_multi_round_recovery(r2, met1);
    r2.execute_multi_round_recovery(r3, met2);
    r1.execute_multi_round_recovery(r2, met3);

    // Apply all ledger items to all mergers
    for (const auto& u : r1.ledger()) { m1.apply_update(u); m2.apply_update(u); m3.apply_update(u); }
    for (const auto& u : r2.ledger()) { m1.apply_update(u); m2.apply_update(u); m3.apply_update(u); }
    for (const auto& u : r3.ledger()) { m1.apply_update(u); m2.apply_update(u); m3.apply_update(u); }

    std::map<uint16_t, const PartitionAwareReconciler*> rec_map = {{1, &r1}, {2, &r2}, {3, &r3}};
    std::map<uint16_t, const ConflictAwareGraphMerger*> mer_map = {{1, &m1}, {2, &m2}, {3, &m3}};

    auto audit = GlobalConsistencyAuditor::audit(rec_map, mer_map, 1);
    assert(audit.is_converged);
    assert(audit.dsu_components == 1);
    assert(audit.pending_sync_items == 0);

    std::cout << "  -> PASSED: 3-Rover relay convergence verified. " << audit.diagnostic_summary << "\n";
}

void test_four_rover_island_partition_healing() {
    std::cout << "[RUN] test_four_rover_island_partition_healing...\n";

    PartitionAwareReconciler r1(1), r2(2), r3(3), r4(4);
    ConflictAwareGraphMerger m1, m2, m3, m4;

    // Island A: Rovers 1 and 2
    for (uint32_t i = 1; i <= 6; ++i) {
        auto u1 = create_kf_update(1, i, i, 0); r1.record_local_update(u1); m1.apply_update(u1);
        auto u2 = create_kf_update(2, i, i, 1); r2.record_local_update(u2); m2.apply_update(u2);
    }
    for (uint32_t i = 1; i < 6; ++i) {
        auto o1 = create_edge_update(1, 0, NodeId(1, i), NodeId(1, i + 1), EdgeType::Odometry, 1.0, 0.0, 30, 0.01);
        r1.record_local_update(o1); m1.apply_update(o1);
        auto o2 = create_edge_update(2, 0, NodeId(2, i), NodeId(2, i + 1), EdgeType::Odometry, 1.0, 0.0, 30, 0.01);
        r2.record_local_update(o2); m2.apply_update(o2);
    }
    auto e12 = create_edge_update(1, 30, NodeId(1, 3), NodeId(2, 3), EdgeType::InterRoverLoopClosure, 0, 1, 20, 0.02);
    r1.record_local_update(e12);
    m1.apply_update(e12);

    ReconcilerMetrics met_island_a;
    r1.execute_multi_round_recovery(r2, met_island_a);
    for (const auto& u : r1.ledger()) { m1.apply_update(u); m2.apply_update(u); }
    for (const auto& u : r2.ledger()) { m1.apply_update(u); m2.apply_update(u); }

    // Island B: Rovers 3 and 4
    for (uint32_t i = 1; i <= 6; ++i) {
        auto u3 = create_kf_update(3, i, i, 10); r3.record_local_update(u3); m3.apply_update(u3);
        auto u4 = create_kf_update(4, i, i, 11); r4.record_local_update(u4); m4.apply_update(u4);
    }
    for (uint32_t i = 1; i < 6; ++i) {
        auto o3 = create_edge_update(3, 0, NodeId(3, i), NodeId(3, i + 1), EdgeType::Odometry, 1.0, 0.0, 30, 0.01);
        r3.record_local_update(o3); m3.apply_update(o3);
        auto o4 = create_edge_update(4, 0, NodeId(4, i), NodeId(4, i + 1), EdgeType::Odometry, 1.0, 0.0, 30, 0.01);
        r4.record_local_update(o4); m4.apply_update(o4);
    }
    auto e34 = create_edge_update(3, 30, NodeId(3, 3), NodeId(4, 3), EdgeType::InterRoverLoopClosure, 0, 1, 22, 0.02);
    r3.record_local_update(e34);
    m3.apply_update(e34);

    ReconcilerMetrics met_island_b;
    r3.execute_multi_round_recovery(r4, met_island_b);
    for (const auto& u : r3.ledger()) { m3.apply_update(u); m4.apply_update(u); }
    for (const auto& u : r4.ledger()) { m3.apply_update(u); m4.apply_update(u); }

    // Check pre-healing audit: Swarm has 2 disconnected islands
    std::map<uint16_t, const PartitionAwareReconciler*> rec_map = {{1, &r1}, {2, &r2}, {3, &r3}, {4, &r4}};
    std::map<uint16_t, const ConflictAwareGraphMerger*> mer_map = {{1, &m1}, {2, &m2}, {3, &m3}, {4, &m4}};

    auto audit_split = GlobalConsistencyAuditor::audit(rec_map, mer_map, 2);
    assert(audit_split.dsu_components == 2);

    // Island Bridge connects Rover 2 and Rover 3
    auto e_bridge = create_edge_update(2, 50, NodeId(2, 6), NodeId(3, 1), EdgeType::InterRoverLoopClosure, 0, 9, 16, 0.03);
    r2.record_local_update(e_bridge);
    m2.apply_update(e_bridge);

    // Reconcile bridge across islands
    ReconcilerMetrics met_bridge, met_a2, met_b2;
    r2.execute_multi_round_recovery(r3, met_bridge);
    r1.execute_multi_round_recovery(r2, met_a2);
    r3.execute_multi_round_recovery(r4, met_b2);

    // Propagate all ledger updates across all rovers
    for (auto* r : {&r1, &r2, &r3, &r4}) {
        for (const auto& u : r->ledger()) {
            m1.apply_update(u); m2.apply_update(u); m3.apply_update(u); m4.apply_update(u);
        }
    }

    auto audit_final = GlobalConsistencyAuditor::audit(rec_map, mer_map, 1);
    assert(audit_final.is_converged);
    assert(audit_final.dsu_components == 1);
    assert(audit_final.pending_sync_items == 0);

    std::cout << "  -> PASSED: 4-Rover island partition healing verified. " << audit_final.diagnostic_summary << "\n";
}

int main() {
    std::cout << "==================================================\n";
    std::cout << " ASTROMESH-X M7.12: PARTITION RECOVERY INTEGRATION\n";
    std::cout << "==================================================\n";

    test_two_rover_partition_recovery();
    test_three_rover_relay_reconciliation();
    test_four_rover_island_partition_healing();

    std::cout << "==================================================\n";
    std::cout << " ALL PARTITION RECOVERY INTEGRATION TESTS PASSED\n";
    std::cout << "==================================================\n";
    return 0;
}
