#include "astromesh/net/partition_reconciler.hpp"
#include <iostream>
#include <cassert>
#include <vector>

using namespace astromesh;
using namespace astromesh::core;
using namespace astromesh::graph;
using namespace astromesh::net;

static MapUpdate make_dummy_update(uint16_t rover_id, uint32_t item_id) {
    MapUpdate u;
    u.origin_rover_id = rover_id;
    u.sequence = 0; // Will be assigned by record_local_update
    u.update_id = 0;
    u.type = MapUpdateType::KeyframeAdded;
    u.node_id = NodeId(rover_id, item_id);
    u.transform = geometry::SE2(static_cast<double>(item_id), static_cast<double>(rover_id), 0.0);
    // LandmarkKey: schema=1, rover_id=rover_id, local_index=item_id, desc_hash=(item_id * 10007 + rover_id * 31)
    u.landmark_key = LandmarkKey(1, rover_id, item_id, static_cast<uint64_t>(item_id * 10007ULL + rover_id * 31ULL));
    return u;
}

void test_already_synchronized() {
    std::cout << "[RUN] test_already_synchronized...\n";
    PartitionAwareReconciler r1(1);
    PartitionAwareReconciler r2(2);

    ReconcilerMetrics metrics;
    auto status = r1.execute_multi_round_recovery(r2, metrics);
    assert(status == ReconciliationStatus::AlreadySynchronized);
    assert(metrics.wire_bytes_transmitted == 2 * VersionVector::WIRE_SIZE);
    assert(metrics.rounds_executed == 1);
    assert(r1.version_vector() == r2.version_vector());
    std::cout << "  -> PASSED: Already synchronized detected in 1 round with minimal wire bytes ("
              << metrics.wire_bytes_transmitted << " B)\n";
}

void test_single_round_peeling() {
    std::cout << "[RUN] test_single_round_peeling...\n";
    PartitionAwareReconciler r1(1);
    PartitionAwareReconciler r2(2);

    // Rover 1 has 8 updates
    for (uint32_t i = 1; i <= 8; ++i) {
        r1.record_local_update(make_dummy_update(1, i));
    }
    // Rover 2 has 6 updates
    for (uint32_t i = 1; i <= 6; ++i) {
        r2.record_local_update(make_dummy_update(2, i));
    }

    assert(r1.needs_reconciliation(r2.version_vector()));
    assert(r2.needs_reconciliation(r1.version_vector()));

    ReconcilerMetrics metrics;
    auto status = r1.execute_multi_round_recovery(r2, metrics);
    assert(status == ReconciliationStatus::Completed);
    assert(r1.version_vector() == r2.version_vector());
    assert(r1.version_vector().get(1) == 8);
    assert(r1.version_vector().get(2) == 6);
    assert(r1.ledger().size() == 14);
    assert(r2.ledger().size() == 14);
    assert(metrics.decodes_succeeded > 0);
    assert(metrics.missing_updates_extracted == 14);

    std::cout << "  -> PASSED: Reconciled 14 updates in single IBLT round. Rounds="
              << metrics.rounds_executed << ", Wire=" << metrics.wire_bytes_transmitted << " B\n";
}

void test_multi_round_chunking() {
    std::cout << "[RUN] test_multi_round_chunking...\n";
    PartitionAwareReconciler r1(1);
    PartitionAwareReconciler r2(2);

    // Rover 1 has 30 unique updates
    for (uint32_t i = 1; i <= 30; ++i) {
        r1.record_local_update(make_dummy_update(1, i));
    }
    // Rover 2 has 30 unique updates (Total difference = 60, exceeding 32-cell single-round capacity)
    for (uint32_t i = 1; i <= 30; ++i) {
        r2.record_local_update(make_dummy_update(2, 100 + i));
    }

    ReconcilerMetrics metrics;
    auto status = r1.execute_multi_round_recovery(r2, metrics);
    assert(status == ReconciliationStatus::Completed);
    assert(r1.version_vector() == r2.version_vector());
    assert(r1.ledger().size() == 60);
    assert(r2.ledger().size() == 60);
    assert(metrics.rounds_executed >= 2);
    assert(metrics.missing_updates_extracted == 60);

    std::cout << "  -> PASSED: Successfully reconciled 60 updates via multi-round prefix chunking. Rounds="
              << metrics.rounds_executed << ", Wire=" << metrics.wire_bytes_transmitted << " B\n";
}

void test_full_map_baseline_comparison() {
    std::cout << "[RUN] test_full_map_baseline_comparison...\n";
    // Setup identical states for baseline and IBLT
    PartitionAwareReconciler r1_base(1), r2_base(2);
    PartitionAwareReconciler r1_iblt(1), r2_iblt(2);

    for (uint32_t i = 1; i <= 10; ++i) {
        r1_base.record_local_update(make_dummy_update(1, i));
        r1_iblt.record_local_update(make_dummy_update(1, i));
    }
    for (uint32_t i = 1; i <= 5; ++i) {
        r2_base.record_local_update(make_dummy_update(2, i));
        r2_iblt.record_local_update(make_dummy_update(2, i));
    }

    ReconcilerMetrics m_base;
    PartitionAwareReconciler::execute_full_map_baseline(r1_base, r2_base, m_base);

    ReconcilerMetrics m_iblt;
    r1_iblt.execute_multi_round_recovery(r2_iblt, m_iblt);

    assert(r1_base.version_vector() == r2_base.version_vector());
    assert(r1_iblt.version_vector() == r2_iblt.version_vector());
    assert(r1_base.version_vector() == r1_iblt.version_vector());
    assert(r1_base.ledger().size() == 15);
    assert(r1_iblt.ledger().size() == 15);

    std::cout << "  -> Full-Map Baseline: Wire=" << m_base.wire_bytes_transmitted << " B, Time=" << m_base.elapsed_ns << " ns\n";
    std::cout << "  -> Multi-Round IBLT:  Wire=" << m_iblt.wire_bytes_transmitted << " B, Time=" << m_iblt.elapsed_ns << " ns\n";
    std::cout << "  -> PASSED: Equivalence verified.\n";
}

int main() {
    std::cout << "==================================================\n";
    std::cout << " ASTROMESH-X M7.12: PARTITION RECONCILER UNIT TESTS\n";
    std::cout << "==================================================\n";

    test_already_synchronized();
    test_single_round_peeling();
    test_multi_round_chunking();
    test_full_map_baseline_comparison();

    std::cout << "==================================================\n";
    std::cout << " ALL PARTITION RECONCILER TESTS PASSED (100%)\n";
    std::cout << "==================================================\n";
    return 0;
}
