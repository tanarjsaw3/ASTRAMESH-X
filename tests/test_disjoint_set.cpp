#include "astromesh/graph/disjoint_set.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <iostream>
#include <vector>
#include <string>
#include <cassert>
#include <algorithm>

using namespace astromesh::graph;
using namespace astromesh::verification;

static int g_total_assertions = 0;
static int g_failed_assertions = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        ++g_total_assertions; \
        if (!(cond)) { \
            ++g_failed_assertions; \
            std::cerr << "  FAILED: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
        } \
    } while (0)

// -----------------------------------------------------------------------------
// Test 1: Basic DSU Operations (make_set, find, unite, are_connected)
// -----------------------------------------------------------------------------
void test_basic_dsu_operations() {
    std::cout << "[TEST 1] Basic DSU Operations...\n";

    DisjointSet dsu;
    TEST_ASSERT(dsu.num_nodes() == 0, "Initial nodes 0");
    TEST_ASSERT(dsu.num_components() == 0, "Initial components 0");

    NodeId n1(1, 1);
    NodeId n2(1, 2);
    NodeId n3(1, 3);

    TEST_ASSERT(dsu.make_set(n1), "make_set(n1) succeeds");
    TEST_ASSERT(dsu.make_set(n2), "make_set(n2) succeeds");
    TEST_ASSERT(dsu.make_set(n3), "make_set(n3) succeeds");

    TEST_ASSERT(dsu.num_nodes() == 3, "Total nodes 3");
    TEST_ASSERT(dsu.num_components() == 3, "Total components 3");

    TEST_ASSERT(dsu.has_node(n1), "has_node(n1) true");
    TEST_ASSERT(dsu.has_node(n2), "has_node(n2) true");
    TEST_ASSERT(dsu.has_node(n3), "has_node(n3) true");

    auto r1 = dsu.find(n1);
    auto r2 = dsu.find(n2);
    auto r3 = dsu.find(n3);

    TEST_ASSERT(r1.has_value() && *r1 == n1, "Initial root of n1 is n1");
    TEST_ASSERT(r2.has_value() && *r2 == n2, "Initial root of n2 is n2");
    TEST_ASSERT(r3.has_value() && *r3 == n3, "Initial root of n3 is n3");

    TEST_ASSERT(!dsu.are_connected(n1, n2), "n1 and n2 initially disconnected");
    TEST_ASSERT(!dsu.are_connected(n2, n3), "n2 and n3 initially disconnected");

    // Unite n1 and n2
    TEST_ASSERT(dsu.unite(n1, n2), "unite(n1, n2) succeeds");
    TEST_ASSERT(dsu.num_components() == 2, "Components decremented to 2");
    TEST_ASSERT(dsu.are_connected(n1, n2), "n1 and n2 are now connected");
    TEST_ASSERT(!dsu.are_connected(n1, n3), "n1 and n3 still disconnected");

    // Re-uniting already connected nodes returns false (no-op)
    TEST_ASSERT(!dsu.unite(n1, n2), "unite(n1, n2) again returns false");
    TEST_ASSERT(dsu.num_components() == 2, "Components remain 2");
}

// -----------------------------------------------------------------------------
// Test 2: Path Compression & Union by Rank / Size
// -----------------------------------------------------------------------------
void test_path_compression_and_union_by_rank() {
    std::cout << "[TEST 2] Path Compression & Union by Rank/Size...\n";

    DisjointSet dsu;
    const size_t N = 10;
    std::vector<NodeId> nodes;
    for (size_t i = 1; i <= N; ++i) {
        NodeId n(1, static_cast<uint32_t>(i));
        nodes.push_back(n);
        dsu.make_set(n);
    }

    // Sequentially unite in a line: 1-2, 2-3, 3-4, ...
    for (size_t i = 0; i < N - 1; ++i) {
        TEST_ASSERT(dsu.unite(nodes[i], nodes[i + 1]), "Unite adjacent line node");
    }

    TEST_ASSERT(dsu.num_components() == 1, "All united into 1 component");
    TEST_ASSERT(dsu.component_size(nodes[0]) == N, "Component size equals N");

    // Query find() on the tail to trigger path compression
    auto tail_root = dsu.find(nodes[N - 1]);
    auto head_root = dsu.find(nodes[0]);
    TEST_ASSERT(tail_root.has_value() && head_root.has_value(), "Roots exist");
    TEST_ASSERT(*tail_root == *head_root, "All share the exact same root");

    // All intermediate nodes should now point to root
    for (size_t i = 0; i < N; ++i) {
        auto r = dsu.find(nodes[i]);
        TEST_ASSERT(r.has_value() && *r == *head_root, "Every node points to root");
        TEST_ASSERT(dsu.component_size(nodes[i]) == N, "Component size is N for every member");
    }
}

// -----------------------------------------------------------------------------
// Test 3: Duplicate Registration & Unknown Node Handling
// -----------------------------------------------------------------------------
void test_duplicate_registration_and_unknown_nodes() {
    std::cout << "[TEST 3] Duplicate Registration & Unknown Node Handling...\n";

    DisjointSet dsu;
    NodeId valid_node(1, 100);
    NodeId unknown_node(1, 999);
    NodeId invalid_sentinel(0, 0);

    TEST_ASSERT(!dsu.make_set(invalid_sentinel), "Cannot register invalid NodeId(0, 0)");
    TEST_ASSERT(dsu.make_set(valid_node), "make_set valid node succeeds");
    TEST_ASSERT(!dsu.make_set(valid_node), "Duplicate make_set must return false");
    TEST_ASSERT(dsu.num_nodes() == 1, "Total nodes remains 1");

    // Unknown node queries
    TEST_ASSERT(!dsu.has_node(unknown_node), "has_node unknown false");
    TEST_ASSERT(!dsu.find(unknown_node).has_value(), "find unknown returns nullopt");
    TEST_ASSERT(!dsu.find_const(unknown_node).has_value(), "find_const unknown returns nullopt");
    TEST_ASSERT(dsu.component_size(unknown_node) == 0, "component_size unknown returns 0");

    // Unite with unknown node
    TEST_ASSERT(!dsu.unite(valid_node, unknown_node), "unite with unknown node must fail");
    TEST_ASSERT(!dsu.unite(unknown_node, valid_node), "unite with unknown node must fail");
    TEST_ASSERT(!dsu.are_connected(valid_node, unknown_node), "are_connected unknown returns false");
    TEST_ASSERT(dsu.num_components() == 1, "Components unmodified");
}

// -----------------------------------------------------------------------------
// Test 4: Multiple Disconnected Components & Transitive Connectivity
// -----------------------------------------------------------------------------
void test_multiple_disconnected_components() {
    std::cout << "[TEST 4] Multiple Disconnected Components & Transitivity...\n";

    DisjointSet dsu;

    // Component A: { (1, 1), (1, 2), (1, 3) }
    NodeId a1(1, 1), a2(1, 2), a3(1, 3);
    dsu.make_set(a1); dsu.make_set(a2); dsu.make_set(a3);
    dsu.unite(a1, a2);
    dsu.unite(a2, a3);

    // Component B: { (2, 1), (2, 2) }
    NodeId b1(2, 1), b2(2, 2);
    dsu.make_set(b1); dsu.make_set(b2);
    dsu.unite(b1, b2);

    // Component C: { (3, 1) } (Isolated node)
    NodeId c1(3, 1);
    dsu.make_set(c1);

    TEST_ASSERT(dsu.num_nodes() == 6, "Total 6 nodes");
    TEST_ASSERT(dsu.num_components() == 3, "Total 3 distinct components");

    TEST_ASSERT(dsu.component_size(a1) == 3, "Component A size 3");
    TEST_ASSERT(dsu.component_size(b1) == 2, "Component B size 2");
    TEST_ASSERT(dsu.component_size(c1) == 1, "Component C size 1");

    // Transitive check inside A
    TEST_ASSERT(dsu.are_connected(a1, a3), "Transitive: a1 and a3 connected");

    // Cross-component checks
    TEST_ASSERT(!dsu.are_connected(a1, b1), "A and B disconnected");
    TEST_ASSERT(!dsu.are_connected(b2, c1), "B and C disconnected");
    TEST_ASSERT(!dsu.are_connected(a3, c1), "A and C disconnected");

    // Merge A and B
    TEST_ASSERT(dsu.unite(a3, b1), "Unite bridge between a3 and b1");
    TEST_ASSERT(dsu.num_components() == 2, "Now 2 components: {A U B} and {C}");
    TEST_ASSERT(dsu.are_connected(a1, b2), "Transitive: a1 connected to b2");
    TEST_ASSERT(dsu.component_size(a1) == 5, "New component size 5");
    TEST_ASSERT(!dsu.are_connected(a1, c1), "C still disconnected");
}

// -----------------------------------------------------------------------------
// Test 5: SubgraphPartitionManager Construction & Odometry Connectivity
// -----------------------------------------------------------------------------
void test_odometry_partitioning() {
    std::cout << "[TEST 5] Odometry Connectivity Policy...\n";

    PoseGraph graph;
    // Rover 1 has 3 sequential keyframes
    NodeId r1_1(1, 1), r1_2(1, 2), r1_3(1, 3);
    graph.add_node(KeyframeNode(r1_1, Pose2D(0.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(r1_2, Pose2D(1.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(r1_3, Pose2D(2.0, 0.0, 0.0)));

    // Rover 2 has 2 sequential keyframes
    NodeId r2_1(2, 1), r2_2(2, 2);
    graph.add_node(KeyframeNode(r2_1, Pose2D(10.0, 10.0, 0.0)));
    graph.add_node(KeyframeNode(r2_2, Pose2D(11.0, 10.0, 0.0)));

    // Odometry edges within Rover 1
    graph.add_edge(PoseGraphEdge(1, r1_1, r1_2, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0)));
    graph.add_edge(PoseGraphEdge(2, r1_2, r1_3, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0)));

    // Odometry edge within Rover 2
    graph.add_edge(PoseGraphEdge(3, r2_1, r2_2, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0)));

    SubgraphPartitionManager manager(graph);

    TEST_ASSERT(manager.num_nodes() == 5, "5 total nodes registered");
    TEST_ASSERT(manager.num_partitions() == 2, "2 disjoint rover partitions");

    TEST_ASSERT(manager.are_connected(r1_1, r1_3), "Rover 1 trajectory connected");
    TEST_ASSERT(manager.are_connected(r2_1, r2_2), "Rover 2 trajectory connected");
    TEST_ASSERT(!manager.are_connected(r1_1, r2_1), "Rover 1 and Rover 2 disconnected");

    TEST_ASSERT(manager.partition_size(r1_1) == 3, "Rover 1 partition size 3");
    TEST_ASSERT(manager.partition_size(r2_1) == 2, "Rover 2 partition size 2");

    TEST_ASSERT(manager.are_rovers_connected(1, 1, graph), "Rover 1 is internally connected");
    TEST_ASSERT(manager.are_rovers_connected(2, 2, graph), "Rover 2 is internally connected");
    TEST_ASSERT(!manager.are_rovers_connected(1, 2, graph), "Rover 1 and Rover 2 are not connected");
}

// -----------------------------------------------------------------------------
// Test 6: Unverified and Rejected Loop Closures Gating Policy
// -----------------------------------------------------------------------------
void test_unverified_and_rejected_loop_closures_gating() {
    std::cout << "[TEST 6] Unverified and Rejected Loop Closures Gating...\n";

    PoseGraph graph;
    NodeId r1(1, 1);
    NodeId r2(2, 1);
    graph.add_node(KeyframeNode(r1, Pose2D(0.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(r2, Pose2D(10.0, 0.0, 0.0)));

    // 1. Add an UNVERIFIED inter-rover loop closure candidate
    PoseGraphEdge unverified_edge(10, r1, r2, EdgeType::InterRoverLoopClosure,
                                  Pose2D(10.0, 0.0, 0.0),
                                  InformationMatrix2D::diagonal(100.0, 1000.0),
                                  EdgeVerificationStatus::Unverified);
    graph.add_edge(unverified_edge);

    SubgraphPartitionManager manager(graph);

    TEST_ASSERT(manager.num_partitions() == 2,
                "Unverified inter-rover closure MUST NOT merge components (remains 2 partitions)");
    TEST_ASSERT(!manager.are_connected(r1, r2),
                "r1 and r2 remain disconnected under unverified candidate");

    // 2. Reject the edge
    graph.update_edge_verification(10, EdgeVerificationStatus::Rejected);
    manager.rebuild(graph);

    TEST_ASSERT(manager.num_partitions() == 2,
                "Rejected loop closure MUST NOT merge components (remains 2 partitions)");
    TEST_ASSERT(!manager.are_connected(r1, r2),
                "r1 and r2 remain disconnected under rejected candidate");

    // 3. Intra-rover candidate closure gating test
    NodeId r1_2(1, 2);
    graph.add_node(KeyframeNode(r1_2, Pose2D(5.0, 0.0, 0.0)));
    // Notice: no odometry between r1 and r1_2, only an unverified loop closure
    PoseGraphEdge intra_candidate(11, r1, r1_2, EdgeType::IntraRoverLoopClosure,
                                  Pose2D(5.0, 0.0, 0.0),
                                  InformationMatrix2D::diagonal(100.0, 1000.0),
                                  EdgeVerificationStatus::Unverified);
    graph.add_edge(intra_candidate);
    manager.rebuild(graph);

    TEST_ASSERT(!manager.are_connected(r1, r1_2),
                "Unverified intra-rover loop closure MUST NOT merge nodes");
}

// -----------------------------------------------------------------------------
// Test 7: Verification Promotion Merging Partitions
// -----------------------------------------------------------------------------
void test_verification_promotion_merging() {
    std::cout << "[TEST 7] Verification Promotion Merging Partitions...\n";

    PoseGraph graph;
    NodeId r1(1, 1);
    NodeId r2(2, 1);
    graph.add_node(KeyframeNode(r1, Pose2D(0.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(r2, Pose2D(10.0, 0.0, 0.0)));

    PoseGraphEdge candidate_edge(10, r1, r2, EdgeType::InterRoverLoopClosure,
                                 Pose2D(10.0, 0.0, 0.0),
                                 InformationMatrix2D::diagonal(100.0, 1000.0),
                                 EdgeVerificationStatus::Unverified);
    graph.add_edge(candidate_edge);

    SubgraphPartitionManager manager(graph);
    TEST_ASSERT(manager.num_partitions() == 2, "Initially 2 partitions");

    // Promote candidate edge to Verified with refined pose
    Pose2D verified_pose(10.02, -0.01, 0.005);
    bool update_ok = graph.update_edge_verification(10, EdgeVerificationStatus::Verified, verified_pose);
    TEST_ASSERT(update_ok, "update_edge_verification succeeds");

    // Rebuild partition manager from updated graph
    manager.rebuild(graph);

    TEST_ASSERT(manager.num_partitions() == 1,
                "Verified loop closure successfully MERGES rovers into 1 partition");
    TEST_ASSERT(manager.are_connected(r1, r2), "r1 and r2 are now connected");
    TEST_ASSERT(manager.partition_size(r1) == 2, "Partition size is now 2");
    TEST_ASSERT(manager.are_rovers_connected(1, 2, graph), "Rover 1 and Rover 2 are connected");
}

// -----------------------------------------------------------------------------
// Test 8: Edge and Node Removal Rebuild Strategy
// -----------------------------------------------------------------------------
void test_edge_and_node_removal_rebuilding() {
    std::cout << "[TEST 8] Edge and Node Removal Rebuilding Strategy...\n";

    PoseGraph graph;
    NodeId n1(1, 1), n2(1, 2), n3(1, 3);
    graph.add_node(KeyframeNode(n1, Pose2D(0.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(n2, Pose2D(1.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(n3, Pose2D(2.0, 0.0, 0.0)));

    // Line: n1 -[e1]-> n2 -[e2]-> n3
    graph.add_edge(PoseGraphEdge(1, n1, n2, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0)));
    graph.add_edge(PoseGraphEdge(2, n2, n3, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0)));

    SubgraphPartitionManager manager(graph);
    TEST_ASSERT(manager.num_partitions() == 1, "Initially 1 component of size 3");
    TEST_ASSERT(manager.are_connected(n1, n3), "n1 and n3 connected");

    // 1. Remove edge 2 (n2 - n3): should split graph into {n1, n2} and {n3}
    TEST_ASSERT(graph.remove_edge(2), "remove_edge(2) succeeds");
    manager.rebuild(graph);

    TEST_ASSERT(manager.num_partitions() == 2, "Now 2 partitions after edge removal");
    TEST_ASSERT(manager.are_connected(n1, n2), "n1 and n2 still connected");
    TEST_ASSERT(!manager.are_connected(n1, n3), "n1 and n3 now disconnected");
    TEST_ASSERT(!manager.are_connected(n2, n3), "n2 and n3 now disconnected");
    TEST_ASSERT(manager.partition_size(n1) == 2, "{n1, n2} size 2");
    TEST_ASSERT(manager.partition_size(n3) == 1, "{n3} size 1");

    // 2. Remove node n2: cascades edge 1, leaving isolated node n1 and isolated node n3
    TEST_ASSERT(graph.remove_node(n2), "remove_node(n2) succeeds");
    manager.rebuild(graph);

    TEST_ASSERT(manager.num_nodes() == 2, "2 nodes remain in graph");
    TEST_ASSERT(manager.num_partitions() == 2, "2 isolated partitions");
    TEST_ASSERT(!manager.are_connected(n1, n3), "n1 and n3 disconnected");
    TEST_ASSERT(manager.partition_size(n1) == 1, "n1 size 1");
    TEST_ASSERT(manager.partition_size(n3) == 1, "n3 size 1");
}

// -----------------------------------------------------------------------------
// Test 9: Subgraph and Partition Member Queries (Deterministic Ordering)
// -----------------------------------------------------------------------------
void test_partition_member_queries() {
    std::cout << "[TEST 9] Subgraph Member Queries and Determinism...\n";

    PoseGraph graph;
    // Component 1: Rover 1 (indices 3, 1, 2 added in arbitrary order)
    NodeId r1_3(1, 3), r1_1(1, 1), r1_2(1, 2);
    graph.add_node(KeyframeNode(r1_3, Pose2D(2.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(r1_1, Pose2D(0.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(r1_2, Pose2D(1.0, 0.0, 0.0)));

    graph.add_edge(PoseGraphEdge(1, r1_1, r1_2, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0)));
    graph.add_edge(PoseGraphEdge(2, r1_2, r1_3, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0)));

    // Component 2: Rover 2 (indices 2, 1)
    NodeId r2_2(2, 2), r2_1(2, 1);
    graph.add_node(KeyframeNode(r2_2, Pose2D(5.0, 5.0, 0.0)));
    graph.add_node(KeyframeNode(r2_1, Pose2D(4.0, 5.0, 0.0)));
    graph.add_edge(PoseGraphEdge(3, r2_1, r2_2, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0)));

    SubgraphPartitionManager manager(graph);

    auto members_r1 = manager.get_partition_members(r1_2);
    TEST_ASSERT(members_r1.size() == 3, "Rover 1 partition has 3 members");
    TEST_ASSERT(members_r1[0] == r1_1, "First member is r1_1 (sorted)");
    TEST_ASSERT(members_r1[1] == r1_2, "Second member is r1_2 (sorted)");
    TEST_ASSERT(members_r1[2] == r1_3, "Third member is r1_3 (sorted)");

    auto partitions_list = manager.get_partitions_list();
    TEST_ASSERT(partitions_list.size() == 2, "Total 2 partitions in list");
    TEST_ASSERT(partitions_list[0].size() == 3, "First partition is Rover 1");
    TEST_ASSERT(partitions_list[1].size() == 2, "Second partition is Rover 2");
    TEST_ASSERT(partitions_list[1][0] == r2_1, "Rover 2 first node is r2_1");
    TEST_ASSERT(partitions_list[1][1] == r2_2, "Rover 2 second node is r2_2");

    // Rebuild multiple times and assert identical output
    for (int rep = 0; rep < 5; ++rep) {
        manager.rebuild(graph);
        auto repeated_list = manager.get_partitions_list();
        TEST_ASSERT(repeated_list == partitions_list, "Rebuild is strictly idempotent and deterministic");
    }
}

// -----------------------------------------------------------------------------
// Test 10: Empty Graph & Isolated Nodes Edge Cases
// -----------------------------------------------------------------------------
void test_empty_graph_and_isolated_nodes() {
    std::cout << "[TEST 10] Empty Graph & Isolated Nodes...\n";

    PoseGraph empty_graph;
    SubgraphPartitionManager manager(empty_graph);

    TEST_ASSERT(manager.num_nodes() == 0, "Empty graph nodes 0");
    TEST_ASSERT(manager.num_partitions() == 0, "Empty graph partitions 0");
    TEST_ASSERT(manager.get_partitions_list().empty(), "Empty graph partitions list empty");
    TEST_ASSERT(manager.get_partition_members(NodeId(1, 1)).empty(), "Members of unknown node empty");

    // Add 3 isolated nodes without any edges
    NodeId iso1(1, 1), iso2(2, 1), iso3(3, 1);
    empty_graph.add_node(KeyframeNode(iso1, Pose2D(0.0, 0.0, 0.0)));
    empty_graph.add_node(KeyframeNode(iso2, Pose2D(10.0, 0.0, 0.0)));
    empty_graph.add_node(KeyframeNode(iso3, Pose2D(20.0, 0.0, 0.0)));

    manager.rebuild(empty_graph);
    TEST_ASSERT(manager.num_nodes() == 3, "3 isolated nodes");
    TEST_ASSERT(manager.num_partitions() == 3, "3 distinct partitions");
    TEST_ASSERT(manager.partition_size(iso1) == 1, "iso1 partition size 1");
    TEST_ASSERT(manager.partition_size(iso2) == 1, "iso2 partition size 1");
    TEST_ASSERT(manager.partition_size(iso3) == 1, "iso3 partition size 1");
    TEST_ASSERT(!manager.are_connected(iso1, iso2), "iso1 and iso2 disconnected");
}

// -----------------------------------------------------------------------------
// Test 11: End-to-End Pipeline Integration (Verification + DSU Gating)
// -----------------------------------------------------------------------------
void test_end_to_end_verification_and_partition_merging() {
    std::cout << "[TEST 11] End-to-End Pipeline: Geometric Gating + DSU Merge...\n";

    PoseGraph graph;
    // Swarm: Rover 1 and Rover 2
    NodeId r1_node(1, 10);
    NodeId r2_node(2, 20);

    graph.add_node(KeyframeNode(r1_node, Pose2D(0.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(r2_node, Pose2D(5.0, 5.0, 0.0)));

    // Candidate edge proposed by set reconciliation
    uint64_t loop_edge_id = 99;
    PoseGraphEdge candidate_edge(loop_edge_id, r1_node, r2_node,
                                 EdgeType::InterRoverLoopClosure,
                                 Pose2D(0.0, 0.0, 0.0), // rough unverified estimate
                                 InformationMatrix2D::diagonal(100.0, 1000.0),
                                 EdgeVerificationStatus::Unverified);
    graph.add_edge(candidate_edge);

    SubgraphPartitionManager manager(graph);
    TEST_ASSERT(manager.num_partitions() == 2, "Candidate loop closure is UNVERIFIED; partitions remain 2");
    TEST_ASSERT(!manager.are_connected(r1_node, r2_node), "Rovers are not connected");

    // Case A: Geometric verification FAILS (e.g. perceptual aliasing / all-outliers)
    std::vector<PointCorrespondence2D> fake_corrs = {
        {Point2D(0.0, 0.0), Point2D(100.0, -50.0)},
        {Point2D(1.0, 0.0), Point2D(-30.0, 80.0)},
        {Point2D(0.0, 1.0), Point2D(40.0, 200.0)},
        {Point2D(2.0, 2.0), Point2D(-90.0, -10.0)}
    };

    auto gate_fail_res = gate_and_promote_loop_closure(graph, loop_edge_id, fake_corrs);
    TEST_ASSERT(!gate_fail_res.success, "Geometric verification fails for inconsistent outlier correspondences");
    TEST_ASSERT(graph.get_edge(loop_edge_id).status == EdgeVerificationStatus::Rejected,
                "Edge status marked Rejected");

    manager.rebuild(graph);
    TEST_ASSERT(manager.num_partitions() == 2, "Partitions remain 2 after rejected candidate");
    TEST_ASSERT(!manager.are_connected(r1_node, r2_node), "Rovers remain disconnected");

    // Case B: Geometric verification SUCCEEDS (consistent observations)
    // Remove the rejected edge 99 first to avoid duplicate constraint rejection
    TEST_ASSERT(graph.remove_edge(loop_edge_id), "remove rejected edge 99 succeeds");

    uint64_t true_loop_id = 100;
    PoseGraphEdge true_candidate(true_loop_id, r1_node, r2_node,
                                 EdgeType::InterRoverLoopClosure,
                                 Pose2D(0.0, 0.0, 0.0),
                                 InformationMatrix2D::diagonal(100.0, 1000.0),
                                 EdgeVerificationStatus::Unverified);
    TEST_ASSERT(graph.add_edge(true_candidate), "add_edge(true_candidate) succeeds");

    // True relative transform: tx=4.0, ty=3.0, theta=0.5
    double true_tx = 4.0, true_ty = 3.0, true_theta = 0.5;
    double c = std::cos(true_theta), s = std::sin(true_theta);
    std::vector<Point2D> src_pts = {
        {0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}, {0.0, 4.0}, {1.5, 2.0}, {-1.0, 1.0}
    };
    std::vector<PointCorrespondence2D> true_corrs;
    for (size_t i = 0; i < src_pts.size(); ++i) {
        double xt = c * src_pts[i].x - s * src_pts[i].y + true_tx;
        double yt = s * src_pts[i].x + c * src_pts[i].y + true_ty;
        true_corrs.emplace_back(src_pts[i], Point2D(xt, yt),
                                astromesh::core::LandmarkKey(1, 1, static_cast<uint32_t>(i + 1), 0x1234));
    }

    auto gate_success_res = gate_and_promote_loop_closure(graph, true_loop_id, true_corrs);
    TEST_ASSERT(gate_success_res.success, "Geometric verification succeeds for genuine landmark correspondences");
    TEST_ASSERT(graph.get_edge(true_loop_id).status == EdgeVerificationStatus::Verified,
                "Edge promoted to Verified");

    // Update partition manager with verified edge
    manager.rebuild(graph);
    TEST_ASSERT(manager.num_partitions() == 1,
                "Verified loop closure successfully MERGES multi-rover swarm into 1 partition");
    TEST_ASSERT(manager.are_connected(r1_node, r2_node), "r1_node and r2_node are now connected");
    TEST_ASSERT(manager.are_rovers_connected(1, 2, graph), "Rover 1 and Rover 2 are officially merged");
}

int main() {
    std::cout << "======================================================================\n";
    std::cout << " AstroMesh-X Phase 2 M2.3: Disjoint Set & Partition Management Tests  \n";
    std::cout << "======================================================================\n";

    test_basic_dsu_operations();
    test_path_compression_and_union_by_rank();
    test_duplicate_registration_and_unknown_nodes();
    test_multiple_disconnected_components();
    test_odometry_partitioning();
    test_unverified_and_rejected_loop_closures_gating();
    test_verification_promotion_merging();
    test_edge_and_node_removal_rebuilding();
    test_partition_member_queries();
    test_empty_graph_and_isolated_nodes();
    test_end_to_end_verification_and_partition_merging();

    std::cout << "======================================================================\n";
    std::cout << " Assertions checked : " << g_total_assertions << "\n";
    std::cout << " Assertions failed  : " << g_failed_assertions << "\n";
    std::cout << "======================================================================\n";

    if (g_failed_assertions > 0) {
        std::cerr << "FAILURE: " << g_failed_assertions << " assertion(s) failed!\n";
        return 1;
    }

    std::cout << "SUCCESS: All DSU and Partition Management tests PASSED.\n";
    return 0;
}
