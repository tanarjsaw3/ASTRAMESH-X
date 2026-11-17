#include "astromesh/graph/conflict_graph_merger.hpp"
#include <iostream>
#include <cassert>

using namespace astromesh;
using namespace astromesh::core;
using namespace astromesh::graph;

void test_dominance_rules() {
    std::cout << "[RUN] test_dominance_rules...\n";

    // Rule 1: Higher inliers dominate
    assert(ConflictAwareGraphMerger::candidate_dominates(10, 0.5, 2, 8, 0.1, 1));
    assert(!ConflictAwareGraphMerger::candidate_dominates(8, 0.1, 1, 10, 0.5, 2));

    // Rule 2: Equal inliers, lower RMSE dominates
    assert(ConflictAwareGraphMerger::candidate_dominates(10, 0.2, 2, 10, 0.4, 1));
    assert(!ConflictAwareGraphMerger::candidate_dominates(10, 0.4, 1, 10, 0.2, 2));

    // Rule 3: Equal inliers and RMSE, lower origin ID dominates
    assert(ConflictAwareGraphMerger::candidate_dominates(10, 0.2, 1, 10, 0.2, 2));
    assert(!ConflictAwareGraphMerger::candidate_dominates(10, 0.2, 2, 10, 0.2, 1));

    std::cout << "  -> PASSED: All 3 dominance tie-breaking rules verified.\n";
}

void test_keyframe_and_edge_addition() {
    std::cout << "[RUN] test_keyframe_and_edge_addition...\n";
    ConflictAwareGraphMerger merger;

    NodeId kf1(1, 0);
    NodeId kf2(1, 1);
    NodeId kf3(2, 0);

    MapUpdate u1 = MapUpdate::make_keyframe(1, 1, 0, geometry::SE2(0, 0, 0), VersionVector{});
    MapUpdate u2 = MapUpdate::make_keyframe(1, 2, 1, geometry::SE2(1, 0, 0), VersionVector{});
    MapUpdate u3 = MapUpdate::make_keyframe(2, 1, 0, geometry::SE2(5, 5, 0), VersionVector{});

    assert(merger.apply_update(u1));
    assert(merger.apply_update(u2));
    assert(merger.apply_update(u3));

    assert(merger.graph().num_nodes() == 3);
    assert(merger.dsu().num_nodes() == 3);
    assert(merger.dsu().num_components() == 3);

    // Add odometry edge between kf1 and kf2
    MapUpdate edge_odom;
    edge_odom.type = MapUpdateType::EdgeVerified;
    edge_odom.origin_rover_id = 1;
    edge_odom.sequence = 3;
    edge_odom.update_id = 3;
    edge_odom.node_id = kf1;
    edge_odom.target_node_id = kf2;
    edge_odom.edge_type = EdgeType::Odometry;
    edge_odom.transform = geometry::SE2(1, 0, 0);
    edge_odom.inlier_count = 25;
    edge_odom.inlier_rmse = 0.05;

    assert(merger.apply_update(edge_odom));
    assert(merger.graph().num_edges() == 1);
    assert(merger.dsu().num_components() == 2);
    assert(merger.dsu().are_connected(kf1, kf2));
    assert(!merger.dsu().are_connected(kf1, kf3));

    std::cout << "  -> PASSED: Keyframe and edge addition verified.\n";
}

void test_conflict_resolution_and_duplicate_suppression() {
    std::cout << "[RUN] test_conflict_resolution_and_duplicate_suppression...\n" << std::flush;
    ConflictAwareGraphMerger merger;

    NodeId u(1, 1);
    NodeId v(2, 1);

    // Initial loop closure proposal with 10 inliers, RMSE 0.30, proposed by rover 2
    MapUpdate cand1;
    cand1.type = MapUpdateType::EdgeVerified;
    cand1.origin_rover_id = 2;
    cand1.sequence = 5;
    cand1.update_id = 502;
    cand1.node_id = u;
    cand1.target_node_id = v;
    cand1.edge_type = EdgeType::InterRoverLoopClosure;
    cand1.transform = geometry::SE2(2.0, 0.0, 0.0);
    cand1.inlier_count = 10;
    cand1.inlier_rmse = 0.30;

    std::cout << "Applying cand1..." << std::endl;
    bool r1 = merger.apply_update(cand1);
    std::cout << "cand1 result: " << r1 << ", num_edges: " << merger.graph().num_edges() << std::endl;
    assert(r1);
    assert(merger.graph().num_edges() == 1);

    // Weaker conflicting proposal with 6 inliers, proposed by rover 1
    MapUpdate cand2 = cand1;
    cand2.origin_rover_id = 1;
    cand2.sequence = 6;
    cand2.update_id = 601;
    cand2.inlier_count = 6;
    cand2.inlier_rmse = 0.40;

    std::cout << "Applying cand2..." << std::endl;
    bool r2 = merger.apply_update(cand2);
    std::cout << "cand2 result: " << r2 << ", num_edges: " << merger.graph().num_edges() << std::endl;
    assert(!r2);
    assert(merger.metrics().duplicate_edges_suppressed == 1);
    assert(merger.graph().num_edges() == 1);

    // Stronger conflicting proposal with 15 inliers, proposed by rover 1
    MapUpdate cand3 = cand1;
    cand3.origin_rover_id = 1;
    cand3.sequence = 7;
    cand3.update_id = 701;
    cand3.inlier_count = 15;
    cand3.inlier_rmse = 0.15;
    cand3.transform = geometry::SE2(2.1, 0.0, 0.0);

    std::cout << "Applying cand3..." << std::endl;
    bool r3 = merger.apply_update(cand3);
    std::cout << "cand3 result: " << r3 << ", num_edges: " << merger.graph().num_edges() << std::endl;
    assert(r3);
    std::cout << "Cand3 applied!" << std::endl;
    std::cout << "Checking metrics..." << std::endl;
    std::cout << "conflicts_resolved: " << merger.metrics().conflicts_resolved << std::endl;
    assert(merger.metrics().conflicts_resolved == 1);
    std::cout << "Checking num_edges: " << merger.graph().num_edges() << std::endl;
    assert(merger.graph().num_edges() == 1);

    std::cout << "Calling edges_between..." << std::endl;
    auto edges = merger.graph().edges_between(u, v);
    std::cout << "edges_between returned " << edges.size() << " edges" << std::endl;
    assert(edges.size() == 1);
    std::cout << "Calling get_edge_provenance..." << std::endl;
    auto prov = merger.get_edge_provenance(edges[0].id);
    std::cout << "get_edge_provenance returned" << std::endl;
    assert(prov.has_value());
    assert(prov->inlier_count == 15);
    assert(prov->update_id == 701);

    std::cout << "  -> PASSED: Duplicate suppressed and conflict resolved deterministically.\n";
}

void test_edge_invalidation_and_dsu_rebuild() {
    std::cout << "[RUN] test_edge_invalidation_and_dsu_rebuild...\n";
    ConflictAwareGraphMerger merger;

    NodeId n1(1, 0), n2(1, 1), n3(2, 0), n4(2, 1);

    // Rover 1 intra-edge (n1-n2)
    MapUpdate e1;
    e1.type = MapUpdateType::EdgeVerified;
    e1.origin_rover_id = 1; e1.sequence = 1; e1.update_id = 1;
    e1.node_id = n1; e1.target_node_id = n2;
    e1.edge_type = EdgeType::Odometry;
    e1.inlier_count = 20; e1.inlier_rmse = 0.01;
    merger.apply_update(e1);

    // Rover 2 intra-edge (n3-n4)
    MapUpdate e2;
    e2.type = MapUpdateType::EdgeVerified;
    e2.origin_rover_id = 2; e2.sequence = 1; e2.update_id = 2;
    e2.node_id = n3; e2.target_node_id = n4;
    e2.edge_type = EdgeType::Odometry;
    e2.inlier_count = 20; e2.inlier_rmse = 0.01;
    merger.apply_update(e2);

    assert(merger.dsu().num_components() == 2);

    // Cross-rover bridge edge (n2-n3) unites the swarm into 1 component
    MapUpdate e_bridge;
    e_bridge.type = MapUpdateType::EdgeVerified;
    e_bridge.origin_rover_id = 1; e_bridge.sequence = 2; e_bridge.update_id = 3;
    e_bridge.node_id = n2; e_bridge.target_node_id = n3;
    e_bridge.edge_type = EdgeType::InterRoverLoopClosure;
    e_bridge.inlier_count = 10; e_bridge.inlier_rmse = 0.10;
    merger.apply_update(e_bridge);

    assert(merger.dsu().num_components() == 1);
    assert(merger.dsu().are_connected(n1, n4));
    assert(merger.is_reciprocal_verified(n2, n3));

    // Now invalidate the bridge edge!
    MapUpdate e_inval;
    e_inval.type = MapUpdateType::EdgeInvalidated;
    e_inval.origin_rover_id = 1; e_inval.sequence = 3; e_inval.update_id = 4;
    e_inval.node_id = n2; e_inval.target_node_id = n3;

    assert(merger.apply_update(e_inval));
    assert(merger.metrics().edges_invalidated == 1);
    assert(merger.metrics().dsu_rebuilds_executed >= 1);

    // DSU must dynamically split back to 2 disconnected components!
    assert(merger.dsu().num_components() == 2);
    assert(merger.dsu().are_connected(n1, n2));
    assert(merger.dsu().are_connected(n3, n4));
    assert(!merger.dsu().are_connected(n1, n4));
    assert(!merger.is_reciprocal_verified(n2, n3));

    std::cout << "  -> PASSED: Invalidation dynamically split DSU from 1 back to 2 components.\n";
}

int main() {
    std::cout << "==================================================\n";
    std::cout << " ASTROMESH-X M7.12: CONFLICT GRAPH MERGER TESTS\n";
    std::cout << "==================================================\n";

    test_dominance_rules();
    test_keyframe_and_edge_addition();
    test_conflict_resolution_and_duplicate_suppression();
    test_edge_invalidation_and_dsu_rebuild();

    std::cout << "==================================================\n";
    std::cout << " ALL CONFLICT GRAPH MERGER TESTS PASSED (100%)\n";
    std::cout << "==================================================\n";
    return 0;
}

