#include "astromesh/graph/keyframe_pruner.hpp"
#include "astromesh/graph/pose_graph.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace {

int g_assertions = 0;

#define EXPECT_TRUE(cond) do { \
    g_assertions++; \
    if (!(cond)) { \
        std::cerr << "Assertion failed at " << __FILE__ << ":" << __LINE__ << " - " << #cond << std::endl; \
        std::exit(1); \
    } \
} while(0)

#define EXPECT_FALSE(cond) EXPECT_TRUE(!(cond))

#define EXPECT_EQ(a, b) do { \
    g_assertions++; \
    if ((a) != (b)) { \
        std::cerr << "Assertion failed at " << __FILE__ << ":" << __LINE__ << " - " << #a << " (" << (a) << ") == " << #b << " (" << (b) << ")" << std::endl; \
        std::exit(1); \
    } \
} while(0)

using namespace astromesh;
using namespace astromesh::graph;

KeyframeNode make_kf(uint16_t rover, uint32_t idx, double x, double y, double th, uint64_t ts = 0) {
    KeyframeNode kf;
    kf.id = NodeId(rover, idx);
    kf.pose = Pose2D(x, y, th);
    kf.timestamp_ns = ts;
    return kf;
}

PoseGraphEdge make_edge(uint64_t eid, NodeId from, NodeId to, EdgeType type, EdgeVerificationStatus status) {
    PoseGraphEdge e;
    e.id = eid;
    e.from_node = from;
    e.to_node = to;
    e.type = type;
    e.status = status;
    e.relative_pose = Pose2D(1.0, 0.0, 0.0);
    e.information = InformationMatrix2D::diagonal(100.0, 1000.0);
    return e;
}

} // namespace

// Test 1: Keyframe retained when translation threshold is crossed
void test_selection_translation_threshold() {
    PrunerConfig cfg;
    cfg.selection.min_translation_m = 0.5;
    cfg.selection.min_rotation_rad = 0.5;
    KeyframePruner pruner(cfg);

    Pose2D prev(0.0, 0.0, 0.0);
    Pose2D cand(0.6, 0.0, 0.0); // 0.6 m >= 0.5 m

    EXPECT_TRUE(pruner.should_retain_keyframe(cand, 100, prev, 0));
}

// Test 2: Keyframe retained when rotation threshold is crossed
void test_selection_rotation_threshold() {
    PrunerConfig cfg;
    cfg.selection.min_translation_m = 1.0;
    cfg.selection.min_rotation_rad = 0.3;
    KeyframePruner pruner(cfg);

    Pose2D prev(0.0, 0.0, 0.0);
    Pose2D cand(0.1, 0.0, 0.4); // 0.4 rad >= 0.3 rad

    EXPECT_TRUE(pruner.should_retain_keyframe(cand, 100, prev, 0));
}

// Test 3: Keyframe rejected when neither threshold is met
void test_selection_rejection() {
    PrunerConfig cfg;
    cfg.selection.min_translation_m = 0.5;
    cfg.selection.min_rotation_rad = 0.5;
    KeyframePruner pruner(cfg);

    Pose2D prev(0.0, 0.0, 0.0);
    Pose2D cand(0.1, 0.1, 0.1); // dist ~0.141m < 0.5m, rot 0.1 < 0.5 rad

    EXPECT_FALSE(pruner.should_retain_keyframe(cand, 100, prev, 0));
}

// Test 4: Invalid/non-finite input handling
void test_selection_nonfinite_inputs() {
    KeyframePruner pruner;
    Pose2D valid(0.0, 0.0, 0.0);
    Pose2D nan_cand(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    Pose2D inf_cand(0.0, std::numeric_limits<double>::infinity(), 0.0);

    EXPECT_FALSE(pruner.should_retain_keyframe(nan_cand, 100, valid, 0));
    EXPECT_FALSE(pruner.should_retain_keyframe(inf_cand, 100, valid, 0));
    EXPECT_FALSE(pruner.should_retain_keyframe(valid, 100, nan_cand, 0));
}

// Test 5: Articulation-point detection on a chain
void test_articulation_points_chain() {
    // Chain: 0 -- 1 -- 2 -- 3
    // Nodes 1 and 2 are articulation points. 0 and 3 are leaves (endpoints).
    PoseGraph g;
    for (uint32_t i = 0; i < 4; ++i) {
        g.add_node(make_kf(1, i, static_cast<double>(i), 0.0, 0.0));
    }
    g.add_edge(make_edge(1, NodeId(1, 0), NodeId(1, 1), EdgeType::Odometry, EdgeVerificationStatus::Verified));
    g.add_edge(make_edge(2, NodeId(1, 1), NodeId(1, 2), EdgeType::Odometry, EdgeVerificationStatus::Verified));
    g.add_edge(make_edge(3, NodeId(1, 2), NodeId(1, 3), EdgeType::Odometry, EdgeVerificationStatus::Verified));

    KeyframePruner pruner;
    auto cuts = pruner.find_articulation_points(g);

    EXPECT_EQ(cuts.size(), 2ULL);
    EXPECT_TRUE(cuts.find(NodeId(1, 1)) != cuts.end());
    EXPECT_TRUE(cuts.find(NodeId(1, 2)) != cuts.end());
    EXPECT_FALSE(cuts.find(NodeId(1, 0)) != cuts.end());
    EXPECT_FALSE(cuts.find(NodeId(1, 3)) != cuts.end());
}

// Test 6: Articulation-point detection on a cycle
void test_articulation_points_cycle() {
    // Cycle: 0 -- 1 -- 2 -- 3 -- 0
    // A 4-node 2-connected cycle has NO articulation points! Any single node can be removed.
    PoseGraph g;
    for (uint32_t i = 0; i < 4; ++i) {
        g.add_node(make_kf(1, i, static_cast<double>(i), 0.0, 0.0));
    }
    g.add_edge(make_edge(1, NodeId(1, 0), NodeId(1, 1), EdgeType::Odometry, EdgeVerificationStatus::Verified));
    g.add_edge(make_edge(2, NodeId(1, 1), NodeId(1, 2), EdgeType::Odometry, EdgeVerificationStatus::Verified));
    g.add_edge(make_edge(3, NodeId(1, 2), NodeId(1, 3), EdgeType::Odometry, EdgeVerificationStatus::Verified));
    g.add_edge(make_edge(4, NodeId(1, 3), NodeId(1, 0), EdgeType::IntraRoverLoopClosure, EdgeVerificationStatus::Verified));

    KeyframePruner pruner;
    auto cuts = pruner.find_articulation_points(g);

    EXPECT_EQ(cuts.size(), 0ULL);
}

// Test 7: Protected-node rules: Gauge anchor, newest keyframe, verified closure
void test_protected_nodes_rules() {
    PoseGraph g;
    // 0: anchor, 1: cycle node, 2: cycle node, 3: loop closure endpoint, 4: frontier
    for (uint32_t i = 0; i <= 4; ++i) {
        g.add_node(make_kf(1, i, static_cast<double>(i), 0.0, 0.0));
    }
    // Edges
    g.add_edge(make_edge(1, NodeId(1, 0), NodeId(1, 1), EdgeType::Odometry, EdgeVerificationStatus::Verified));
    g.add_edge(make_edge(2, NodeId(1, 1), NodeId(1, 2), EdgeType::Odometry, EdgeVerificationStatus::Verified));
    g.add_edge(make_edge(3, NodeId(1, 2), NodeId(1, 3), EdgeType::Odometry, EdgeVerificationStatus::Verified));
    g.add_edge(make_edge(4, NodeId(1, 3), NodeId(1, 1), EdgeType::IntraRoverLoopClosure, EdgeVerificationStatus::Verified));
    g.add_edge(make_edge(5, NodeId(1, 3), NodeId(1, 4), EdgeType::Odometry, EdgeVerificationStatus::Verified));

    KeyframePruner pruner;
    auto protected_nodes = pruner.identify_protected_nodes(g);

    // Node 0: gauge anchor protected
    EXPECT_TRUE(protected_nodes.find(NodeId(1, 0)) != protected_nodes.end());
    // Node 4: newest keyframe protected
    EXPECT_TRUE(protected_nodes.find(NodeId(1, 4)) != protected_nodes.end());
    // Node 1 and Node 3: endpoints of verified loop closure protected
    EXPECT_TRUE(protected_nodes.find(NodeId(1, 1)) != protected_nodes.end());
    EXPECT_TRUE(protected_nodes.find(NodeId(1, 3)) != protected_nodes.end());

    // Node 2 is inside the cycle (1-2-3-1), not an articulation point, not an anchor, not a closure endpoint -> Removable!
    EXPECT_FALSE(protected_nodes.find(NodeId(1, 2)) != protected_nodes.end());
}

// Test 8: Inter-rover constraint endpoint protection
void test_inter_rover_endpoint_protection() {
    PoseGraph g;
    g.add_node(make_kf(1, 0, 0.0, 0.0, 0.0));
    g.add_node(make_kf(1, 1, 1.0, 0.0, 0.0));
    g.add_node(make_kf(2, 0, 5.0, 0.0, 0.0));
    g.add_node(make_kf(2, 1, 6.0, 0.0, 0.0));

    // Inter-rover closure between (1,1) and (2,0)
    g.add_edge(make_edge(1, NodeId(1, 0), NodeId(1, 1), EdgeType::Odometry, EdgeVerificationStatus::Verified));
    g.add_edge(make_edge(2, NodeId(2, 0), NodeId(2, 1), EdgeType::Odometry, EdgeVerificationStatus::Verified));
    g.add_edge(make_edge(3, NodeId(1, 1), NodeId(2, 0), EdgeType::InterRoverLoopClosure, EdgeVerificationStatus::Verified));

    KeyframePruner pruner;
    auto protected_nodes = pruner.identify_protected_nodes(g);

    EXPECT_TRUE(protected_nodes.find(NodeId(1, 1)) != protected_nodes.end());
    EXPECT_TRUE(protected_nodes.find(NodeId(2, 0)) != protected_nodes.end());
}

// Test 9: Count bound exceeded when all candidates are protected -> LimitUnmetNoSafeCandidate
void test_bound_exceeded_all_protected() {
    // Chain of 3 nodes: 0 -- 1 -- 2
    // 0 = anchor, 1 = articulation point, 2 = newest keyframe
    // All 3 nodes are protected! Max bound = 2
    PoseGraph g;
    for (uint32_t i = 0; i < 3; ++i) {
        g.add_node(make_kf(1, i, static_cast<double>(i), 0.0, 0.0));
    }
    g.add_edge(make_edge(1, NodeId(1, 0), NodeId(1, 1), EdgeType::Odometry, EdgeVerificationStatus::Verified));
    g.add_edge(make_edge(2, NodeId(1, 1), NodeId(1, 2), EdgeType::Odometry, EdgeVerificationStatus::Verified));

    PrunerConfig cfg;
    cfg.max_retained_keyframes = 2; // Bound requires removing 1 node
    KeyframePruner pruner(cfg);

    SubgraphPartitionManager dsu(g);
    PruningResult res = pruner.enforce_memory_bound(g, &dsu);

    EXPECT_EQ(res.status, PruningStatus::LimitUnmetNoSafeCandidate);
    EXPECT_EQ(res.nodes_pruned, 0ULL);
    EXPECT_EQ(g.num_nodes(), 3ULL); // Graph remained intact!
}

// Test 10: Count bound reached with safe redundant candidates available
void test_safe_pruning_execution() {
    // Topology with redundant cycle nodes:
    // 0 (anchor) -- 1 (closure endpoint) -- 2 (redundant) -- 3 (redundant) -- 4 (closure endpoint) -- 5 (frontier)
    // Plus loop closure between 1 and 4.
    PoseGraph g;
    for (uint32_t i = 0; i <= 5; ++i) {
        g.add_node(make_kf(1, i, static_cast<double>(i), 0.0, 0.0));
    }
    for (uint32_t i = 0; i < 5; ++i) {
        g.add_edge(make_edge(i + 1, NodeId(1, i), NodeId(1, i + 1), EdgeType::Odometry, EdgeVerificationStatus::Verified));
    }
    g.add_edge(make_edge(10, NodeId(1, 1), NodeId(1, 4), EdgeType::IntraRoverLoopClosure, EdgeVerificationStatus::Verified));

    EXPECT_EQ(g.num_nodes(), 6ULL);

    PrunerConfig cfg;
    cfg.max_retained_keyframes = 4; // Wants to remove 2 nodes (nodes 2 and 3 can be safely removed!)
    KeyframePruner pruner(cfg);

    SubgraphPartitionManager dsu(g);
    PruningResult res = pruner.enforce_memory_bound(g, &dsu);

    EXPECT_EQ(res.status, PruningStatus::SuccessBoundMet);
    EXPECT_EQ(res.nodes_pruned, 2ULL);
    EXPECT_EQ(g.num_nodes(), 4ULL);

    // Verify pruned nodes were exactly 2 and 3
    EXPECT_FALSE(g.has_node(NodeId(1, 2)));
    EXPECT_FALSE(g.has_node(NodeId(1, 3)));

    // Verify protected nodes (0, 1, 4, 5) remain present!
    EXPECT_TRUE(g.has_node(NodeId(1, 0)));
    EXPECT_TRUE(g.has_node(NodeId(1, 1)));
    EXPECT_TRUE(g.has_node(NodeId(1, 4)));
    EXPECT_TRUE(g.has_node(NodeId(1, 5)));

    // Verify partition manager rebuilt cleanly and all remaining nodes are connected
    EXPECT_TRUE(dsu.are_connected(NodeId(1, 0), NodeId(1, 5)));
}

// Test 11: No dangling edges after removal
void test_no_dangling_edges() {
    PoseGraph g;
    g.add_node(make_kf(1, 0, 0.0, 0.0, 0.0));
    g.add_node(make_kf(1, 1, 1.0, 0.0, 0.0));
    g.add_edge(make_edge(1, NodeId(1, 0), NodeId(1, 1), EdgeType::Odometry, EdgeVerificationStatus::Verified));

    g.remove_node(NodeId(1, 1));

    EXPECT_EQ(g.num_nodes(), 1ULL);
    EXPECT_EQ(g.num_edges(), 0ULL); // Edge 1 removed
    EXPECT_EQ(g.incident_edges(NodeId(1, 0)).size(), 0ULL);
}

// Test 12: Memory metrics calculations
void test_memory_metrics() {
    PoseGraph g;
    auto kf0 = make_kf(1, 0, 0.0, 0.0, 0.0);
    kf0.observed_landmarks.push_back(core::LandmarkKey(1, 1, 0, 0x1234));
    kf0.observed_landmarks.push_back(core::LandmarkKey(1, 1, 1, 0x5678));
    g.add_node(kf0);

    auto kf1 = make_kf(1, 1, 1.0, 0.0, 0.0);
    kf1.observed_landmarks.push_back(core::LandmarkKey(1, 1, 2, 0x9ABC));
    g.add_node(kf1);

    g.add_edge(make_edge(1, NodeId(1, 0), NodeId(1, 1), EdgeType::Odometry, EdgeVerificationStatus::Verified));

    KeyframePruner pruner;
    auto m = pruner.compute_memory_metrics(g);

    EXPECT_EQ(m.node_count, 2ULL);
    EXPECT_EQ(m.edge_count, 1ULL);
    EXPECT_EQ(m.odometry_edge_count, 1ULL);
    EXPECT_EQ(m.landmark_reference_count, 3ULL);
    EXPECT_TRUE(m.estimated_struct_bytes > 0);
    EXPECT_TRUE(m.estimated_serialized_bytes > 0);
}

// Test 13: Deterministic candidate ranking
void test_deterministic_candidate_ranking() {
    PoseGraph g;
    // Anchor 0, then 1 (dist 0.1 to 0), 2 (dist 1.0 to 0)
    g.add_node(make_kf(1, 0, 0.0, 0.0, 0.0));
    g.add_node(make_kf(1, 1, 0.1, 0.0, 0.0));
    g.add_node(make_kf(1, 2, 1.0, 0.0, 0.0));
    g.add_edge(make_edge(1, NodeId(1, 0), NodeId(1, 1), EdgeType::Odometry, EdgeVerificationStatus::Verified));
    g.add_edge(make_edge(2, NodeId(1, 0), NodeId(1, 2), EdgeType::Odometry, EdgeVerificationStatus::Verified));

    KeyframePruner pruner;
    std::unordered_set<NodeId> protected_nodes = {NodeId(1, 0)};

    auto ranked = pruner.rank_pruning_candidates(g, protected_nodes);
    EXPECT_EQ(ranked.size(), 2ULL);
    // Node 1 is at dist 0.1, Node 2 is at dist 1.0. Node 1 is ranked first (more redundant)
    EXPECT_EQ(ranked[0], NodeId(1, 1));
    EXPECT_EQ(ranked[1], NodeId(1, 2));
}

int main() {
    std::cout << "[RUN      ] test_keyframe_pruner" << std::endl;
    test_selection_translation_threshold();
    test_selection_rotation_threshold();
    test_selection_rejection();
    test_selection_nonfinite_inputs();
    test_articulation_points_chain();
    test_articulation_points_cycle();
    test_protected_nodes_rules();
    test_inter_rover_endpoint_protection();
    test_bound_exceeded_all_protected();
    test_safe_pruning_execution();
    test_no_dangling_edges();
    test_memory_metrics();
    test_deterministic_candidate_ranking();
    std::cout << "[       OK ] test_keyframe_pruner (" << g_assertions << " assertions verified)" << std::endl;
    return 0;
}
