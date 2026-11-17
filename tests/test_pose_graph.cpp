#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/core/landmark_key.hpp"

#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>
#include <limits>

using astromesh::graph::PoseGraph;
using astromesh::graph::KeyframeNode;
using astromesh::graph::PoseGraphEdge;
using astromesh::graph::NodeId;
using astromesh::graph::Pose2D;
using astromesh::graph::InformationMatrix2D;
using astromesh::graph::EdgeType;
using astromesh::graph::EdgeVerificationStatus;
using astromesh::graph::GraphStatus;
using astromesh::graph::normalize_angle;
using astromesh::core::LandmarkKey;

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

void test_empty_graph() {
    std::cout << "[TEST 1] Empty Graph Behavior...\n";
    PoseGraph graph;
    TEST_ASSERT(graph.empty(), "Initial graph must be empty");
    TEST_ASSERT(graph.num_nodes() == 0, "Initial node count must be 0");
    TEST_ASSERT(graph.num_edges() == 0, "Initial edge count must be 0");
    TEST_ASSERT(graph.num_edges_of_type(EdgeType::Odometry) == 0, "Odometry edge count 0");
    TEST_ASSERT(!graph.has_node(NodeId(1, 1)), "Empty graph has no node");
    TEST_ASSERT(graph.find_node(NodeId(1, 1)) == nullptr, "find_node on empty graph returns nullptr");

    bool caught = false;
    try {
        graph.get_node(NodeId(1, 1));
    } catch (const std::out_of_range&) {
        caught = true;
    }
    TEST_ASSERT(caught, "get_node on non-existent node throws std::out_of_range");

    // clear on empty is safe
    graph.clear();
    TEST_ASSERT(graph.empty(), "clear on empty graph remains empty");
}

void test_node_addition_and_retrieval() {
    std::cout << "[TEST 2] Node Addition, Retrieval, and Landmark Referencing...\n";
    PoseGraph graph;
    NodeId nid(1, 42);
    Pose2D pose(10.5, -20.25, 0.785398); // ~pi/4
    KeyframeNode node(nid, pose, 123456789ULL);

    // Attach observed landmark references
    LandmarkKey lmk1(1, 1, 10, 0xABCDEF);
    LandmarkKey lmk2(1, 1, 11, 0x123456);
    node.observed_landmarks.push_back(lmk1);
    node.observed_landmarks.push_back(lmk2);

    TEST_ASSERT(graph.add_node(node), "add_node must succeed");
    TEST_ASSERT(graph.num_nodes() == 1, "Graph has 1 node");
    TEST_ASSERT(graph.has_node(nid), "has_node returns true for added node");

    const KeyframeNode* found = graph.find_node(nid);
    TEST_ASSERT(found != nullptr, "find_node returns non-null");
    TEST_ASSERT(found->id == nid, "NodeId matches");
    TEST_ASSERT(found->pose == pose, "Pose2D matches");
    TEST_ASSERT(found->timestamp_ns == 123456789ULL, "Timestamp matches");
    TEST_ASSERT(found->observed_landmarks.size() == 2, "Observed landmarks size 2");
    TEST_ASSERT(found->observed_landmarks[0] == lmk1, "LandmarkKey 1 matches");
    TEST_ASSERT(found->observed_landmarks[1] == lmk2, "LandmarkKey 2 matches");

    const KeyframeNode& ref = graph.get_node(nid);
    TEST_ASSERT(ref.id == nid, "get_node reference matches");
}

void test_multiple_nodes_and_rovers() {
    std::cout << "[TEST 3] Multi-Rover Node Management...\n";
    PoseGraph graph;

    // Rover 1 has nodes (1, 1), (1, 2)
    graph.add_node(KeyframeNode(NodeId(1, 1), Pose2D(0.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(NodeId(1, 2), Pose2D(1.0, 0.0, 0.0)));

    // Rover 2 has nodes (2, 1), (2, 2), (2, 3)
    graph.add_node(KeyframeNode(NodeId(2, 1), Pose2D(10.0, 5.0, 1.57)));
    graph.add_node(KeyframeNode(NodeId(2, 2), Pose2D(11.0, 5.0, 1.57)));
    graph.add_node(KeyframeNode(NodeId(2, 3), Pose2D(12.0, 5.0, 1.57)));

    TEST_ASSERT(graph.num_nodes() == 5, "Total nodes must be 5");

    auto rovers = graph.rovers();
    TEST_ASSERT(rovers.size() == 2, "Must detect 2 distinct rovers");
    TEST_ASSERT(rovers[0] == 1 && rovers[1] == 2, "Rovers list must be [1, 2]");

    auto r1_nodes = graph.nodes_for_rover(1);
    TEST_ASSERT(r1_nodes.size() == 2, "Rover 1 has 2 nodes");
    TEST_ASSERT(r1_nodes[0] == NodeId(1, 1) && r1_nodes[1] == NodeId(1, 2), "Rover 1 nodes match");

    auto r2_nodes = graph.nodes_for_rover(2);
    TEST_ASSERT(r2_nodes.size() == 3, "Rover 2 has 3 nodes");

    auto r3_nodes = graph.nodes_for_rover(3);
    TEST_ASSERT(r3_nodes.empty(), "Rover 3 has 0 nodes");
}

void test_node_validation_and_duplicate_rejection() {
    std::cout << "[TEST 4] Node Validation & Duplicate Rejection...\n";
    PoseGraph graph;

    // 1. Invalid sentinel rover_id == 0 rejected
    KeyframeNode invalid_rover(NodeId(0, 1), Pose2D(0.0, 0.0, 0.0));
    TEST_ASSERT(!graph.add_node(invalid_rover), "rover_id=0 node must be rejected");
    TEST_ASSERT(graph.add_node_detailed(invalid_rover) == GraphStatus::InvalidNode, "Status InvalidNode");

    // 2. Non-finite pose rejected
    double nan_val = std::numeric_limits<double>::quiet_NaN();
    double inf_val = std::numeric_limits<double>::infinity();

    KeyframeNode nan_node(NodeId(1, 1), Pose2D(nan_val, 0.0, 0.0));
    TEST_ASSERT(!graph.add_node(nan_node), "NaN pose node must be rejected");

    KeyframeNode inf_node(NodeId(1, 1), Pose2D(0.0, inf_val, 0.0));
    TEST_ASSERT(!graph.add_node(inf_node), "Inf pose node must be rejected");

    KeyframeNode inf_theta(NodeId(1, 1), Pose2D(0.0, 0.0, inf_val));
    TEST_ASSERT(!graph.add_node(inf_theta), "Inf theta node must be rejected");

    // 3. Valid node addition
    KeyframeNode valid_node(NodeId(1, 1), Pose2D(5.0, 5.0, 0.0));
    TEST_ASSERT(graph.add_node(valid_node), "Valid node addition succeeds");

    // 4. Duplicate node rejection
    KeyframeNode dup_node(NodeId(1, 1), Pose2D(10.0, 10.0, 1.0));
    TEST_ASSERT(!graph.add_node(dup_node), "Duplicate NodeId must be rejected");
    TEST_ASSERT(graph.add_node_detailed(dup_node) == GraphStatus::DuplicateNode, "Status DuplicateNode");
    TEST_ASSERT(graph.num_nodes() == 1, "Graph remains size 1");
    TEST_ASSERT(graph.get_node(NodeId(1, 1)).pose == Pose2D(5.0, 5.0, 0.0), "Original node unmodified");
}

void test_edge_addition_and_endpoint_validation() {
    std::cout << "[TEST 5] Edge Addition, Edge Types, and Endpoint Validation...\n";
    PoseGraph graph;

    NodeId n1(1, 1);
    NodeId n2(1, 2);
    NodeId n3(1, 3);
    NodeId n2_1(2, 1);

    graph.add_node(KeyframeNode(n1, Pose2D(0.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(n2, Pose2D(1.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(n3, Pose2D(2.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(n2_1, Pose2D(0.5, 0.5, 0.0)));

    // 1. Missing endpoint rejection
    NodeId missing(1, 999);
    PoseGraphEdge bad_edge(1, n1, missing, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0));
    TEST_ASSERT(!graph.add_edge(bad_edge), "Edge with missing target node must fail");
    TEST_ASSERT(graph.add_edge_detailed(bad_edge) == GraphStatus::MissingEndpointNode, "Status MissingEndpointNode");

    // 2. Self-edge rejection
    PoseGraphEdge self_edge(2, n1, n1, EdgeType::IntraRoverLoopClosure, Pose2D(0.0, 0.0, 0.0));
    TEST_ASSERT(!graph.add_edge(self_edge), "Self-edge must be rejected");
    TEST_ASSERT(graph.add_edge_detailed(self_edge) == GraphStatus::SelfEdgeProhibited, "Status SelfEdgeProhibited");

    // 3. Valid Odometry edge
    PoseGraphEdge odom_edge(10, n1, n2, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0));
    TEST_ASSERT(graph.add_edge(odom_edge), "Valid odometry edge must succeed");
    TEST_ASSERT(graph.num_edges() == 1, "Total edges 1");
    TEST_ASSERT(graph.num_edges_of_type(EdgeType::Odometry) == 1, "Odometry edges 1");

    // 4. Valid Intra-rover loop closure edge
    PoseGraphEdge intra_loop(20, n1, n3, EdgeType::IntraRoverLoopClosure, Pose2D(2.0, 0.0, 0.0),
                             InformationMatrix2D::diagonal(50.0, 500.0), EdgeVerificationStatus::Verified);
    TEST_ASSERT(graph.add_edge(intra_loop), "Valid intra-rover loop closure succeeds");
    TEST_ASSERT(graph.num_edges_of_type(EdgeType::IntraRoverLoopClosure) == 1, "Intra-rover loop closures 1");

    // 5. Valid Inter-rover loop closure edge (between rover 1 and rover 2)
    PoseGraphEdge inter_loop(30, n1, n2_1, EdgeType::InterRoverLoopClosure, Pose2D(0.5, 0.5, 0.0),
                             InformationMatrix2D::diagonal(20.0, 200.0), EdgeVerificationStatus::Unverified);
    TEST_ASSERT(graph.add_edge(inter_loop), "Valid inter-rover loop closure succeeds");
    TEST_ASSERT(graph.num_edges_of_type(EdgeType::InterRoverLoopClosure) == 1, "Inter-rover loop closures 1");
    TEST_ASSERT(graph.num_edges() == 3, "Total edges 3");

    // 6. Verification status inspection
    TEST_ASSERT(graph.get_edge(30).status == EdgeVerificationStatus::Unverified, "Candidate starts Unverified");
    TEST_ASSERT(graph.get_edge(20).status == EdgeVerificationStatus::Verified, "Gated loop starts Verified");
}

void test_edge_provenance_and_type_consistency() {
    std::cout << "[TEST 6] Edge Provenance & Semantic Consistency Enforcement...\n";
    PoseGraph graph;
    NodeId r1_1(1, 1);
    NodeId r1_2(1, 2);
    NodeId r2_1(2, 1);

    graph.add_node(KeyframeNode(r1_1, Pose2D(0.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(r1_2, Pose2D(1.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(r2_1, Pose2D(5.0, 5.0, 0.0)));

    // Rejection 1: InterRoverLoopClosure with identical rover IDs
    PoseGraphEdge bad_inter(1, r1_1, r1_2, EdgeType::InterRoverLoopClosure, Pose2D(1.0, 0.0, 0.0));
    TEST_ASSERT(!graph.add_edge(bad_inter), "InterRoverLoopClosure on same rover must fail");
    TEST_ASSERT(graph.add_edge_detailed(bad_inter) == GraphStatus::InvalidEdge, "Status InvalidEdge");

    // Rejection 2: Odometry between different rovers
    PoseGraphEdge bad_odom(2, r1_1, r2_1, EdgeType::Odometry, Pose2D(5.0, 5.0, 0.0));
    TEST_ASSERT(!graph.add_edge(bad_odom), "Odometry across rovers must fail");

    // Rejection 3: IntraRoverLoopClosure between different rovers
    PoseGraphEdge bad_intra(3, r1_1, r2_1, EdgeType::IntraRoverLoopClosure, Pose2D(5.0, 5.0, 0.0));
    TEST_ASSERT(!graph.add_edge(bad_intra), "IntraRoverLoopClosure across rovers must fail");
}

void test_edge_duplicate_policy() {
    std::cout << "[TEST 7] Duplicate Edge Rejection...\n";
    PoseGraph graph;
    NodeId n1(1, 1);
    NodeId n2(1, 2);
    graph.add_node(KeyframeNode(n1, Pose2D(0.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(n2, Pose2D(1.0, 0.0, 0.0)));

    PoseGraphEdge e1(10, n1, n2, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0));
    TEST_ASSERT(graph.add_edge(e1), "First edge succeeds");

    // 1. Duplicate edge ID rejected
    PoseGraphEdge e_dup_id(10, n2, n1, EdgeType::IntraRoverLoopClosure, Pose2D(-1.0, 0.0, 0.0));
    TEST_ASSERT(!graph.add_edge(e_dup_id), "Duplicate edge ID must be rejected");
    TEST_ASSERT(graph.add_edge_detailed(e_dup_id) == GraphStatus::DuplicateEdge, "Status DuplicateEdge");

    // 2. Duplicate topological constraint (same endpoints and same type) rejected
    PoseGraphEdge e_dup_constraint(20, n1, n2, EdgeType::Odometry, Pose2D(1.1, 0.0, 0.0));
    TEST_ASSERT(!graph.add_edge(e_dup_constraint), "Duplicate odometry constraint between same nodes rejected");
    TEST_ASSERT(graph.add_edge_detailed(e_dup_constraint) == GraphStatus::DuplicateEdge, "Status DuplicateEdge");

    TEST_ASSERT(graph.num_edges() == 1, "Only 1 edge remains");
}

void test_information_matrix_validation() {
    std::cout << "[TEST 8] Information Matrix & Positive-Definiteness Validation...\n";
    // 1. Valid diagonal
    InformationMatrix2D diag = InformationMatrix2D::diagonal(100.0, 1000.0);
    TEST_ASSERT(diag.is_valid(), "Standard diagonal information matrix must be valid");

    // 2. Non-positive diagonal entries
    InformationMatrix2D bad_diag1(-1.0, 0.0, 0.0, 1.0, 0.0, 1.0);
    TEST_ASSERT(!bad_diag1.is_valid(), "Negative xx must be invalid");

    InformationMatrix2D bad_diag2(1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    TEST_ASSERT(!bad_diag2.is_valid(), "Zero yy must be invalid");

    // 3. Not positive-definite (determinant <= 0)
    // Submatrix [[1, 2], [2, 1]] has det = 1 - 4 = -3 <= 0
    InformationMatrix2D not_pd(1.0, 2.0, 0.0, 1.0, 0.0, 1.0);
    TEST_ASSERT(!not_pd.is_valid(), "Indefinite information matrix must be invalid");

    // 4. Non-finite values
    double nan_val = std::numeric_limits<double>::quiet_NaN();
    InformationMatrix2D nan_mat(1.0, nan_val, 0.0, 1.0, 0.0, 1.0);
    TEST_ASSERT(!nan_mat.is_valid(), "NaN in information matrix must be invalid");

    // 5. Graph rejects edge with invalid information matrix
    PoseGraph graph;
    NodeId n1(1, 1), n2(1, 2);
    graph.add_node(KeyframeNode(n1, Pose2D(0.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(n2, Pose2D(1.0, 0.0, 0.0)));

    PoseGraphEdge bad_info_edge(1, n1, n2, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0), not_pd);
    TEST_ASSERT(!graph.add_edge(bad_info_edge), "Edge with indefinite information matrix must be rejected");
}

void test_angle_normalization() {
    std::cout << "[TEST 9] Angle Normalization [-pi, pi)...\n";
    constexpr double PI = 3.14159265358979323846;
    constexpr double EPS = 1e-9;

    TEST_ASSERT(std::abs(normalize_angle(0.0)) < EPS, "normalize(0) == 0");
    TEST_ASSERT(std::abs(normalize_angle(PI / 2.0) - PI / 2.0) < EPS, "normalize(pi/2) == pi/2");
    TEST_ASSERT(std::abs(normalize_angle(-PI / 2.0) - (-PI / 2.0)) < EPS, "normalize(-pi/2) == -pi/2");
    TEST_ASSERT(std::abs(normalize_angle(3.0 * PI) - (-PI)) < EPS, "normalize(3*pi) wraps to -pi");
    TEST_ASSERT(std::abs(normalize_angle(-3.0 * PI) - (-PI)) < EPS, "normalize(-3*pi) wraps to -pi");

    // Pose2D automatic normalization
    Pose2D p(0.0, 0.0, 3.0 * PI);
    TEST_ASSERT(std::abs(p.theta - (-PI)) < EPS, "Pose2D constructor normalizes angle");
}

void test_node_and_edge_removal() {
    std::cout << "[TEST 10] Node & Incident Edge Removal...\n";
    PoseGraph graph;
    NodeId n1(1, 1), n2(1, 2), n3(1, 3);
    graph.add_node(KeyframeNode(n1, Pose2D(0.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(n2, Pose2D(1.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(n3, Pose2D(2.0, 0.0, 0.0)));

    graph.add_edge(PoseGraphEdge(1, n1, n2, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0)));
    graph.add_edge(PoseGraphEdge(2, n2, n3, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0)));
    graph.add_edge(PoseGraphEdge(3, n1, n3, EdgeType::IntraRoverLoopClosure, Pose2D(2.0, 0.0, 0.0)));

    TEST_ASSERT(graph.num_nodes() == 3, "Initial 3 nodes");
    TEST_ASSERT(graph.num_edges() == 3, "Initial 3 edges");

    // Remove edge 1
    TEST_ASSERT(graph.remove_edge(1), "remove_edge 1 must succeed");
    TEST_ASSERT(!graph.has_edge(1), "Edge 1 no longer in graph");
    TEST_ASSERT(graph.num_edges() == 2, "2 edges remaining");
    TEST_ASSERT(!graph.remove_edge(1), "Second remove_edge 1 returns false");

    // Remove node n2: incident edge 2 should be cascade-removed
    TEST_ASSERT(graph.remove_node(n2), "remove_node n2 succeeds");
    TEST_ASSERT(!graph.has_node(n2), "Node n2 no longer in graph");
    TEST_ASSERT(!graph.has_edge(2), "Incident edge 2 automatically removed");
    TEST_ASSERT(graph.num_nodes() == 2, "2 nodes remaining (n1, n3)");
    TEST_ASSERT(graph.num_edges() == 1, "1 edge remaining (edge 3)");

    auto between = graph.edges_between(n1, n3);
    TEST_ASSERT(between.size() == 1, "edges_between(n1, n3) returns 1 edge");
    TEST_ASSERT(between[0].id == 3, "Remaining edge is ID 3");
}

void test_boundary_identifiers() {
    std::cout << "[TEST 11] Boundary Identifier Values...\n";
    PoseGraph graph;
    NodeId max_id(std::numeric_limits<uint16_t>::max(), std::numeric_limits<uint32_t>::max());
    TEST_ASSERT(max_id.is_valid(), "Max integer NodeId must be valid");

    graph.add_node(KeyframeNode(max_id, Pose2D(100.0, 100.0, 0.0)));
    TEST_ASSERT(graph.has_node(max_id), "Max NodeId stored and retrieved successfully");

    NodeId node2(std::numeric_limits<uint16_t>::max(), std::numeric_limits<uint32_t>::max() - 1);
    graph.add_node(KeyframeNode(node2, Pose2D(101.0, 100.0, 0.0)));

    uint64_t max_edge_id = std::numeric_limits<uint64_t>::max();
    PoseGraphEdge max_edge(max_edge_id, node2, max_id, EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0));
    TEST_ASSERT(graph.add_edge(max_edge), "Max edge ID stored and retrieved successfully");
    TEST_ASSERT(graph.has_edge(max_edge_id), "has_edge for max uint64");
}

void test_deterministic_multi_rover_swarm_construction() {
    std::cout << "[TEST 12] Deterministic Multi-Rover Swarm Graph Construction...\n";
    PoseGraph graph;
    const size_t NUM_KEYFRAMES = 50;

    // Build trajectories for Rover 1 and Rover 2
    for (size_t i = 1; i <= NUM_KEYFRAMES; ++i) {
        double x1 = static_cast<double>(i) * 1.0;
        double y1 = std::sin(static_cast<double>(i) * 0.1) * 2.0;
        graph.add_node(KeyframeNode(NodeId(1, static_cast<uint32_t>(i)), Pose2D(x1, y1, 0.0)));

        double x2 = static_cast<double>(i) * 1.0 + 5.0;
        double y2 = std::cos(static_cast<double>(i) * 0.1) * 2.0 + 10.0;
        graph.add_node(KeyframeNode(NodeId(2, static_cast<uint32_t>(i)), Pose2D(x2, y2, 0.0)));
    }

    TEST_ASSERT(graph.num_nodes() == 100, "100 total keyframe nodes added");

    // Add sequential odometry edges
    for (size_t i = 1; i < NUM_KEYFRAMES; ++i) {
        // Rover 1 odometry
        graph.add_edge(PoseGraphEdge(0, NodeId(1, static_cast<uint32_t>(i)),
                                     NodeId(1, static_cast<uint32_t>(i + 1)),
                                     EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0)));
        // Rover 2 odometry
        graph.add_edge(PoseGraphEdge(0, NodeId(2, static_cast<uint32_t>(i)),
                                     NodeId(2, static_cast<uint32_t>(i + 1)),
                                     EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0)));
    }

    TEST_ASSERT(graph.num_edges_of_type(EdgeType::Odometry) == 98, "98 total odometry edges");

    // Add 5 intra-rover loop closures each
    for (size_t k = 0; k < 5; ++k) {
        uint32_t from_idx = static_cast<uint32_t>(5 + k * 8);
        uint32_t to_idx = from_idx + 6;
        graph.add_edge(PoseGraphEdge(0, NodeId(1, from_idx), NodeId(1, to_idx),
                                     EdgeType::IntraRoverLoopClosure, Pose2D(6.0, 0.0, 0.0)));
        graph.add_edge(PoseGraphEdge(0, NodeId(2, from_idx), NodeId(2, to_idx),
                                     EdgeType::IntraRoverLoopClosure, Pose2D(6.0, 0.0, 0.0)));
    }

    TEST_ASSERT(graph.num_edges_of_type(EdgeType::IntraRoverLoopClosure) == 10, "10 intra-rover loop closures");

    // Add 4 inter-rover loop closures connecting rover 1 and rover 2
    for (size_t k = 0; k < 4; ++k) {
        uint32_t r1_idx = static_cast<uint32_t>(10 + k * 10);
        uint32_t r2_idx = static_cast<uint32_t>(12 + k * 10);
        graph.add_edge(PoseGraphEdge(0, NodeId(1, r1_idx), NodeId(2, r2_idx),
                                     EdgeType::InterRoverLoopClosure, Pose2D(5.0, 10.0, 0.0),
                                     InformationMatrix2D::diagonal(20.0, 200.0),
                                     EdgeVerificationStatus::Unverified));
    }

    TEST_ASSERT(graph.num_edges_of_type(EdgeType::InterRoverLoopClosure) == 4, "4 inter-rover loop closures");
    TEST_ASSERT(graph.num_edges() == 112, "Total edges 112 (98 odom + 10 intra + 4 inter)");

    // Test clear behavior
    graph.clear();
    TEST_ASSERT(graph.empty(), "Graph empty after clear");
    TEST_ASSERT(graph.num_nodes() == 0, "0 nodes after clear");
    TEST_ASSERT(graph.num_edges() == 0, "0 edges after clear");
}

int main() {
    std::cout << "======================================================================\n";
    std::cout << " AstroMesh-X Phase 2 M2.1: Topological Pose-Graph Unit Tests\n";
    std::cout << "======================================================================\n";

    test_empty_graph();
    test_node_addition_and_retrieval();
    test_multiple_nodes_and_rovers();
    test_node_validation_and_duplicate_rejection();
    test_edge_addition_and_endpoint_validation();
    test_edge_provenance_and_type_consistency();
    test_edge_duplicate_policy();
    test_information_matrix_validation();
    test_angle_normalization();
    test_node_and_edge_removal();
    test_boundary_identifiers();
    test_deterministic_multi_rover_swarm_construction();

    std::cout << "======================================================================\n";
    std::cout << " Assertions checked : " << g_total_assertions << "\n";
    std::cout << " Assertions failed  : " << g_failed_assertions << "\n";
    std::cout << "======================================================================\n";

    if (g_failed_assertions > 0) {
        std::cerr << "FAILED: Pose-Graph tests encountered " << g_failed_assertions << " failures.\n";
        return 1;
    }

    std::cout << "SUCCESS: All Topological Pose-Graph tests PASSED.\n";
    return 0;
}
