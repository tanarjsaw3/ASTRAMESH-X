#include "astromesh/graph/alignment_graph.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using namespace astromesh;
using namespace astromesh::graph;
using namespace astromesh::geometry;

static bool approx_equal(double a, double b, double eps = 1e-4) {
    return std::abs(a - b) <= eps;
}

static bool pose_approx_equal(const SE2& a, const SE2& b, double eps = 1e-3) {
    return approx_equal(a.x(), b.x(), eps) &&
           approx_equal(a.y(), b.y(), eps) &&
           approx_equal(a.theta(), b.theta(), eps);
}

void test_edge_addition_and_stale_rejection() {
    std::cout << "[RUN] test_edge_addition_and_stale_rejection..." << std::endl;
    AlignmentGraph graph;

    // Reject invalid rovers
    AlignmentEdge invalid1{0, 0, 1, SE2::identity(), 5, 5, 0.01, 0.9, 1, true};
    assert(!graph.add_or_update_edge(invalid1));
    AlignmentEdge invalid2{0, 1, 1, SE2::identity(), 5, 5, 0.01, 0.9, 1, true};
    assert(!graph.add_or_update_edge(invalid2));

    // Add valid edge R2 -> R1
    SE2 t12(2.0, 3.0, 0.1);
    AlignmentEdge e1{0, 2, 1, t12, 10, 9, 0.05, 0.95, 10, true};
    assert(graph.add_or_update_edge(e1));
    assert(graph.has_edge(2, 1));
    assert(!graph.has_edge(1, 2)); // Directed

    auto retrieved = graph.get_edge(2, 1);
    assert(retrieved.has_value());
    assert(retrieved->epoch == 10);
    assert(pose_approx_equal(retrieved->transform_to_from, t12));

    // Try stale update (epoch 5 < 10)
    SE2 t_stale(9.0, 9.0, 0.0);
    AlignmentEdge e_stale{0, 2, 1, t_stale, 10, 9, 0.05, 0.95, 5, true};
    assert(!graph.add_or_update_edge(e_stale));
    assert(pose_approx_equal(graph.get_edge(2, 1)->transform_to_from, t12));

    // Valid update with higher epoch (epoch 15 > 10)
    SE2 t_newer(2.5, 3.2, 0.12);
    AlignmentEdge e_newer{0, 2, 1, t_newer, 12, 11, 0.03, 0.98, 15, true};
    assert(graph.add_or_update_edge(e_newer));
    assert(graph.get_edge(2, 1)->epoch == 15);
    assert(pose_approx_equal(graph.get_edge(2, 1)->transform_to_from, t_newer));

    // Invalidation
    uint64_t eid = graph.get_edge(2, 1)->edge_id;
    assert(graph.invalidate_edge(eid));
    assert(!graph.has_edge(2, 1));
    assert(!graph.get_edge(2, 1).has_value());

    std::cout << "[PASS] test_edge_addition_and_stale_rejection" << std::endl;
}

void test_connected_components() {
    std::cout << "[RUN] test_connected_components..." << std::endl;
    AlignmentGraph graph;

    // Component 1: rovers {1, 2, 3}
    graph.add_or_update_edge({0, 2, 1, SE2(1, 0, 0), 5, 5, 0.01, 1.0, 1, true});
    graph.add_or_update_edge({0, 3, 2, SE2(0, 1, 0), 5, 5, 0.01, 1.0, 1, true});

    // Component 2: rovers {4, 5}
    graph.add_or_update_edge({0, 5, 4, SE2(2, 2, 0), 5, 5, 0.01, 1.0, 1, true});

    auto comps = graph.get_connected_components();
    assert(comps.size() == 2);

    // Verify component elements
    bool found_c1 = false;
    bool found_c2 = false;
    for (const auto& comp : comps) {
        if (comp.size() == 3 && comp[0] == 1 && comp[1] == 2 && comp[2] == 3) {
            found_c1 = true;
        }
        if (comp.size() == 2 && comp[0] == 4 && comp[1] == 5) {
            found_c2 = true;
        }
    }
    assert(found_c1);
    assert(found_c2);

    std::cout << "[PASS] test_connected_components" << std::endl;
}

void test_path_transform_accumulation() {
    std::cout << "[RUN] test_path_transform_accumulation..." << std::endl;
    AlignmentGraph graph;

    // Self transform is identity
    auto self_tx = graph.resolve_transform_from_anchor(1, 1);
    assert(self_tx.has_value());
    assert(pose_approx_equal(*self_tx, SE2::identity()));

    // Linear chain: 1 <-(T_12)- 2 <-(T_23)- 3
    // Edge (2 -> 1): points in 2 mapped to 1: p1 = T_12 * p2
    // Edge (3 -> 2): points in 3 mapped to 2: p2 = T_23 * p3
    // Combined: p1 = T_12 * T_23 * p3
    SE2 t12(5.0, 0.0, 0.0);             // Rover 2 is at x=5 relative to 1
    SE2 t23(0.0, 3.0, 0.5 * 3.14159265); // Rover 3 is at (0, 3, pi/2) relative to 2

    graph.add_or_update_edge({0, 2, 1, t12, 10, 10, 0.01, 1.0, 1, true});
    graph.add_or_update_edge({0, 3, 2, t23, 10, 10, 0.01, 1.0, 1, true});

    auto tx_1_to_2 = graph.resolve_transform_from_anchor(1, 2);
    assert(tx_1_to_2.has_value());
    assert(pose_approx_equal(*tx_1_to_2, t12));

    auto tx_1_to_3 = graph.resolve_transform_from_anchor(1, 3);
    assert(tx_1_to_3.has_value());
    SE2 expected_13 = t12 * t23;
    assert(pose_approx_equal(*tx_1_to_3, expected_13));

    // Reverse path resolution: Anchor is 3, target is 1
    auto tx_3_to_1 = graph.resolve_transform_from_anchor(3, 1);
    assert(tx_3_to_1.has_value());
    assert(pose_approx_equal(*tx_3_to_1, expected_13.inverse()));

    // Disconnected query
    auto tx_disconnected = graph.resolve_transform_from_anchor(1, 99);
    assert(!tx_disconnected.has_value());

    std::cout << "[PASS] test_path_transform_accumulation" << std::endl;
}

void test_cycle_consistency_audit() {
    std::cout << "[RUN] test_cycle_consistency_audit..." << std::endl;
    AlignmentGraph graph;

    // Create consistent 3-cycle:
    // T_12: 2 -> 1
    // T_23: 3 -> 2
    // T_31: 1 -> 3
    // In a consistent cycle: T_12 * T_23 * T_31 = I
    SE2 t12(3.0, 1.0, 0.2);
    SE2 t23(-1.0, 2.0, -0.5);
    SE2 t13 = t12 * t23;
    SE2 t31 = t13.inverse(); // T_31 maps 1 into 3

    graph.add_or_update_edge({0, 2, 1, t12, 10, 10, 0.02, 1.0, 1, true});
    graph.add_or_update_edge({0, 3, 2, t23, 10, 10, 0.03, 1.0, 1, true});
    graph.add_or_update_edge({0, 1, 3, t31, 10, 10, 0.02, 1.0, 1, true});

    auto audit_clean = graph.audit_cycle_consistency(0.2);
    assert(audit_clean.evaluated_cycle_count == 1);
    assert(!audit_clean.has_inconsistent_cycles);
    assert(audit_clean.max_cycle_error_m < 1e-3);
    assert(audit_clean.suggested_invalidations.empty());

    // Corrupt one edge (T_31)
    SE2 t31_corrupt = t31 * SE2(2.0, -1.5, 0.8);
    AlignmentEdge corrupt_edge{0, 1, 3, t31_corrupt, 10, 5, 0.45, 0.5, 2, true};
    graph.add_or_update_edge(corrupt_edge);

    auto audit_dirty = graph.audit_cycle_consistency(0.2);
    assert(audit_dirty.evaluated_cycle_count == 1);
    assert(audit_dirty.has_inconsistent_cycles);
    assert(audit_dirty.max_cycle_error_m > 1.0);
    assert(audit_dirty.suggested_invalidations.size() == 1);

    // Verify that the suggested invalidation is indeed the corrupted edge (worst RMSE = 0.45)
    uint64_t bad_eid = graph.get_edge(1, 3)->edge_id;
    assert(audit_dirty.suggested_invalidations[0] == bad_eid);

    // Invalidate the bad edge and re-audit
    graph.invalidate_edge(bad_eid);
    auto audit_remediated = graph.audit_cycle_consistency(0.2);
    assert(audit_remediated.evaluated_cycle_count == 0); // No full 3-cycles left
    assert(!audit_remediated.has_inconsistent_cycles);

    std::cout << "[PASS] test_cycle_consistency_audit" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << " AstroMesh-X: Alignment Graph Unit Tests" << std::endl;
    std::cout << "========================================" << std::endl;

    test_edge_addition_and_stale_rejection();
    test_connected_components();
    test_path_transform_accumulation();
    test_cycle_consistency_audit();

    std::cout << "All Alignment Graph tests PASSED successfully." << std::endl;
    return 0;
}

