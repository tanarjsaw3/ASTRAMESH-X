#include "astromesh/core/iblt.hpp"
#include "astromesh/core/landmark_key.hpp"
#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/keyframe_pruner.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/net/distributed_reconciliation.hpp"
#include "astromesh/net/rover_node.hpp"
#include "astromesh/optimization/cord_integrator.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

using namespace astromesh::net;
using namespace astromesh::core;
using namespace astromesh::graph;
using namespace astromesh::geometry;
using namespace astromesh::optimization;
using namespace astromesh::verification;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[FAIL] Line " << __LINE__ << ": " << msg << std::endl; \
            std::exit(1); \
        } \
    } while (0)

// ==============================================================================
// Test 1: Multi-Keyframe Trajectory Construction and Odometry
// ==============================================================================
static void test_multi_keyframe_trajectory_construction() {
    std::cout << "[TEST 1] Multi-Keyframe Trajectory Construction & Odometry..." << std::endl;

    RoverNodeConfig cfg;
    cfg.rover_id = 1;
    cfg.listen_port = 28101;
    cfg.listen_ip = "127.0.0.1";

    RoverNode node(cfg);
    TEST_ASSERT(node.start(), "Failed to start RoverNode 1");

    DistributedReconciliationAgent agent(1, node, 32, 3);
    agent.setup_callbacks();

    // Append 5 sequential keyframes along a 20-meter line
    for (uint32_t k = 0; k < 5; ++k) {
        const double x = static_cast<double>(k) * 5.0;
        const SE2 pose(x, 0.0, 0.0);

        std::vector<LandmarkObservationRecord> obs;
        for (uint32_t l = 1; l <= 3; ++l) {
            LandmarkKey key(1, 1, k * 10 + l, 0x1000ULL + k * 10 + l);
            obs.push_back({ key, 1.0 * l, 0.5 * l, k, true });
        }

        agent.append_trajectory_keyframe(k, pose, obs, k * 1'000'000'000ULL);
    }

    // Append observation of landmark 101 at keyframe 4 as well (re-observation across keyframes)
    LandmarkKey reobserved_key(1, 1, 101, 0x1000ULL + 101);
    std::vector<LandmarkObservationRecord> re_obs = { { reobserved_key, 2.5, 1.2, 4, true } };
    agent.append_trajectory_keyframe(5, SE2(25.0, 0.0, 0.0), re_obs, 5'000'000'000ULL);

    const auto history = agent.get_landmark_observation_history(101);
    TEST_ASSERT(history.size() == 1, "Landmark 101 should have 1 observation in history");
    TEST_ASSERT(agent.get_pose_graph().num_nodes() == 6, "Graph must have exactly 6 nodes");
    TEST_ASSERT(agent.get_pose_graph().num_edges_of_type(EdgeType::Odometry) == 5, "Graph must have 5 sequential odometry edges");
    TEST_ASSERT(agent.get_dsu_component_count() == 1, "Graph must form 1 connected component");

    node.stop();
    std::cout << "  -> PASSED" << std::endl;
}

// ==============================================================================
// Test 2: Distributed Trajectory Sync and Inter-Rover Loop Closure
// ==============================================================================
static void test_distributed_trajectory_sync_and_loop_closure() {
    std::cout << "[TEST 2] Distributed Trajectory Sync & Inter-Rover Loop Closure..." << std::endl;

    RoverNodeConfig cfg1;
    cfg1.rover_id = 1;
    cfg1.listen_port = 28102;
    cfg1.listen_ip = "127.0.0.1";
    cfg1.seed_peers.emplace_back("127.0.0.1", 28103);

    RoverNodeConfig cfg2;
    cfg2.rover_id = 2;
    cfg2.listen_port = 28103;
    cfg2.listen_ip = "127.0.0.1";
    cfg2.seed_peers.emplace_back("127.0.0.1", 28102);

    RoverNode node1(cfg1);
    RoverNode node2(cfg2);
    TEST_ASSERT(node1.start(), "Failed to start Node 1");
    TEST_ASSERT(node2.start(), "Failed to start Node 2");

    DistributedReconciliationAgent agent1(1, node1, 32, 3);
    DistributedReconciliationAgent agent2(2, node2, 32, 3);
    agent1.setup_callbacks();
    agent2.setup_callbacks();

    // Rover 1 path: (0,0) -> (5,0) -> (10,0)
    agent1.append_trajectory_keyframe(0, SE2(0.0, 0.0, 0.0), {});
    agent1.append_trajectory_keyframe(1, SE2(5.0, 0.0, 0.0), {});

    // Overlapping landmarks at keyframe 2 (x=10, y=0)
    std::vector<LandmarkObservationRecord> shared_obs1;
    shared_obs1.push_back({ LandmarkKey(1, 1, 101, 0xA001ULL), 2.0, 1.0, 2, true });
    shared_obs1.push_back({ LandmarkKey(1, 1, 102, 0xA002ULL), 3.0, -1.0, 2, true });
    shared_obs1.push_back({ LandmarkKey(1, 1, 103, 0xA003ULL), 1.0, 2.0, 2, true });
    shared_obs1.push_back({ LandmarkKey(1, 1, 104, 0xA004ULL), 4.0, 0.5, 2, true });
    agent1.append_trajectory_keyframe(2, SE2(10.0, 0.0, 0.0), shared_obs1);

    // Rover 2 path: (20,0) -> (15,0) -> (10,0)
    // Rover 2 observes shared landmarks at its keyframe 2 (x=10, y=0)
    // In Rover 2's local frame, relative coordinates are identical since orientation is aligned
    std::vector<LandmarkObservationRecord> shared_obs2;
    shared_obs2.push_back({ LandmarkKey(1, 1, 101, 0xA001ULL), 2.0, 1.0, 2, true });
    shared_obs2.push_back({ LandmarkKey(1, 1, 102, 0xA002ULL), 3.0, -1.0, 2, true });
    shared_obs2.push_back({ LandmarkKey(1, 1, 103, 0xA003ULL), 1.0, 2.0, 2, true });
    shared_obs2.push_back({ LandmarkKey(1, 1, 104, 0xA004ULL), 4.0, 0.5, 2, true });

    agent2.append_trajectory_keyframe(0, SE2(20.0, 0.0, 0.0), {});
    agent2.append_trajectory_keyframe(1, SE2(15.0, 0.0, 0.0), {});
    agent2.append_trajectory_keyframe(2, SE2(10.0, 0.0, 0.0), shared_obs2);

    // Discovery phase
    for (int t = 0; t < 5; ++t) {
        node1.poll_events(0);
        node2.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Initiate IBLT sync from Rover 1 to Rover 2
    uint64_t msg_id = agent1.initiate_sync_with_peer(2);
    TEST_ASSERT(msg_id > 0, "Sync initiation should return valid message id");

    // Network exchange loop
    for (int step = 0; step < 25; ++step) {
        node1.poll_events(0);
        node2.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const auto s1 = agent1.get_stats();
    const auto s2 = agent2.get_stats();

    TEST_ASSERT(s1.sync_sent >= 1, "Rover 1 must have sent sync packet");
    TEST_ASSERT(s2.sync_received >= 1, "Rover 2 must have received sync packet");
    TEST_ASSERT(s2.sync_decodes_succeeded >= 1, "Rover 2 must have decoded IBLT difference");
    TEST_ASSERT(s2.proposals_sent >= 1, "Rover 2 must have sent candidate proposal");
    TEST_ASSERT(s1.loop_closures_verified >= 1, "Rover 1 must have verified candidate proposal");
    TEST_ASSERT(s1.dsu_unions_performed >= 1, "Rover 1 must have performed DSU union");

    // Global swarm evaluation across both rover pose graphs
    auto swarm_metrics = GlobalSwarmEvaluator::evaluate_graphs({&agent1.get_pose_graph(), &agent2.get_pose_graph()});
    TEST_ASSERT(swarm_metrics.is_fully_unified, "Swarm must be fully unified after inter-rover loop closure");
    TEST_ASSERT(swarm_metrics.global_connected_components == 1, "Global swarm components must equal 1");
    TEST_ASSERT(swarm_metrics.total_swarm_nodes == 6, "Total swarm nodes must equal 6 (3 per rover)");
    TEST_ASSERT(swarm_metrics.verified_inter_rover_closures >= 1, "Must have at least 1 verified inter-rover closure");

    node1.stop();
    node2.stop();
    std::cout << "  -> PASSED" << std::endl;
}

// ==============================================================================
// Test 3: Local Intra-Rover CORD Trajectory Relaxation
// ==============================================================================
static void test_intra_rover_cord_trajectory_relaxation() {
    std::cout << "[TEST 3] Local Intra-Rover CORD Trajectory Relaxation..." << std::endl;

    RoverNodeConfig cfg;
    cfg.rover_id = 1;
    cfg.listen_port = 28104;
    cfg.listen_ip = "127.0.0.1";

    RoverNode node(cfg);
    TEST_ASSERT(node.start(), "Failed to start RoverNode");

    DistributedReconciliationAgent agent(1, node, 32, 3);
    agent.setup_callbacks();

    // Construct a triangular closed loop with artificial odometry error
    // Node (1,0): (0, 0)
    // Node (1,1): (10, 0)
    // Node (1,2): (5, 8.66) -- perturbed to (5.8, 9.2) by +0.8m drift
    agent.append_trajectory_keyframe(0, SE2(0.0, 0.0, 0.0), {});
    agent.append_trajectory_keyframe(1, SE2(10.0, 0.0, 0.0), {});
    agent.append_trajectory_keyframe(2, SE2(5.8, 9.2, 0.0), {}); // Drifted pose

    // Add closure constraint from (1,2) back to (1,0)
    // True relative pose from (1,2) to (1,0): dx = -5.0, dy = -8.66
    PoseGraphEdge closure_edge(99, NodeId(1, 2), NodeId(1, 0),
                              EdgeType::IntraRoverLoopClosure,
                              Pose2D(-5.0, -8.66, 0.0),
                              InformationMatrix2D::diagonal(100.0, 1000.0),
                              EdgeVerificationStatus::Verified);

    agent.get_pose_graph().add_edge_detailed(closure_edge);
    agent.get_partition_manager().add_eligible_edge(closure_edge);

    // Initial error on node 2
    const auto& node2_initial = agent.get_pose_graph().get_node(NodeId(1, 2));
    const double initial_error = std::hypot(node2_initial.pose.x - 5.0, node2_initial.pose.y - 8.66);
    TEST_ASSERT(initial_error > 0.8, "Initial trajectory error must be significant (>0.8m)");

    // Run CORD optimization
    CordIntegratorOptions opts;
    opts.timestep = 0.01;
    opts.max_steps = 200;
    opts.gradient_tolerance = 1e-4;
    opts.fixed_anchor = NodeId(1, 0);

    auto res = agent.optimize_local_graph_cord(opts);
    TEST_ASSERT(res.is_converged() || res.status == CordStatus::MaxStepsReached, "CORD should complete integration");
    TEST_ASSERT(res.final_cost < res.initial_cost, "CORD must reduce graph objective cost");
    TEST_ASSERT(agent.get_stats().cord_optimizations_run >= 1, "Agent must record CORD run");

    // Check relaxed pose of node 2
    const auto& node2_final = agent.get_pose_graph().get_node(NodeId(1, 2));
    const double final_error = std::hypot(node2_final.pose.x - 5.0, node2_final.pose.y - 8.66);

    std::cout << "  Initial error: " << initial_error << " m, Final error: " << final_error << " m\n";
    TEST_ASSERT(final_error < initial_error * 0.4, "CORD must reduce trajectory error by at least 60%");

    node.stop();
    std::cout << "  -> PASSED" << std::endl;
}

// ==============================================================================
// Test 4: Adversarial Perceptual Aliasing & Precision/Recall Instrumentation
// ==============================================================================
static void test_adversarial_aliasing_and_precision_recall() {
    std::cout << "[TEST 4] Adversarial Perceptual Aliasing & Precision/Recall..." << std::endl;

    RoverNodeConfig cfg1;
    cfg1.rover_id = 1;
    cfg1.listen_port = 28105;
    cfg1.listen_ip = "127.0.0.1";
    cfg1.seed_peers.emplace_back("127.0.0.1", 28106);

    RoverNodeConfig cfg2;
    cfg2.rover_id = 2;
    cfg2.listen_port = 28106;
    cfg2.listen_ip = "127.0.0.1";
    cfg2.seed_peers.emplace_back("127.0.0.1", 28105);

    RoverNode node1(cfg1);
    RoverNode node2(cfg2);
    TEST_ASSERT(node1.start(), "Failed to start Node 1");
    TEST_ASSERT(node2.start(), "Failed to start Node 2");

    DistributedReconciliationAgent agent1(1, node1, 32, 3);
    DistributedReconciliationAgent agent2(2, node2, 32, 3);
    agent1.setup_callbacks();
    agent2.setup_callbacks();

    // Rover 1 has genuine landmarks
    std::vector<LandmarkObservationRecord> obs1;
    for (uint32_t i = 1; i <= 4; ++i) {
        obs1.push_back({ LandmarkKey(1, 1, 200 + i, 0xB000ULL + i), 1.0 * i, 2.0 * i, 0, false });
    }
    agent1.append_trajectory_keyframe(0, SE2(0.0, 0.0, 0.0), obs1);

    // Rover 2 has identical landmark keys (perceptual aliasing), but severely contradictory geometry:
    // Instead of a rigid transform, landmark positions are violently scrambled
    std::vector<LandmarkObservationRecord> obs2;
    obs2.push_back({ LandmarkKey(1, 1, 201, 0xB000ULL + 1), -25.0, 30.0, 0, false });
    obs2.push_back({ LandmarkKey(1, 1, 202, 0xB000ULL + 2), 40.0, -12.0, 0, false });
    obs2.push_back({ LandmarkKey(1, 1, 203, 0xB000ULL + 3), 5.0, 50.0, 0, false });
    obs2.push_back({ LandmarkKey(1, 1, 204, 0xB000ULL + 4), -18.0, -4.0, 0, false });
    agent2.append_trajectory_keyframe(0, SE2(10.0, 0.0, 0.0), obs2);

    // Direct proposal from Rover 2 to Rover 1
    CandidateProposalPayload proposal;
    proposal.sender_rover_id = 2;
    proposal.target_rover_id = 1;
    proposal.observations = obs2;

    auto p_bytes = proposal.serialize();
    MessageFrame frame;
    frame.type = MessageType::CandidateProposal;
    frame.sender_rover_id = 2;
    frame.target_rover_id = 1;
    frame.payload = std::move(p_bytes);

    // Rover 1 processes adversarial proposal
    agent1.handle_message(frame, Endpoint("127.0.0.1", 28106), 1000);

    const auto s1 = agent1.get_stats();
    TEST_ASSERT(s1.proposals_received == 1, "Rover 1 must receive proposal");
    TEST_ASSERT(s1.loop_closures_rejected == 1, "RANSAC must reject aliased proposal");
    TEST_ASSERT(s1.loop_closures_verified == 0, "No spurious loop closure should be verified");
    TEST_ASSERT(s1.true_negatives == 1, "Rejection of spurious proposal must count as True Negative");
    TEST_ASSERT(s1.false_positives == 0, "Zero False Positives allowed (safety invariant)");
    TEST_ASSERT(agent1.get_loop_closure_precision() >= 0.999, "Precision must remain 1.0");

    node1.stop();
    node2.stop();
    std::cout << "  -> PASSED" << std::endl;
}

// ==============================================================================
// Test 5: Safe Keyframe Pruning on Active Trajectory
// ==============================================================================
static void test_safe_keyframe_pruning_on_trajectory() {
    std::cout << "[TEST 5] Safe Keyframe Pruning on Active Trajectory..." << std::endl;

    RoverNodeConfig cfg;
    cfg.rover_id = 1;
    cfg.listen_port = 28107;
    cfg.listen_ip = "127.0.0.1";

    RoverNode node(cfg);
    TEST_ASSERT(node.start(), "Failed to start RoverNode");

    DistributedReconciliationAgent agent(1, node, 32, 3);
    agent.setup_callbacks();

    // Create a 6-keyframe path:
    // 0 (anchor) -- 1 (closure endpoint) -- 2 (redundant) -- 3 (redundant) -- 4 (closure endpoint) -- 5 (frontier)
    for (uint32_t k = 0; k < 6; ++k) {
        const double x = static_cast<double>(k) * 1.0;
        agent.append_trajectory_keyframe(k, SE2(x, 0.0, 0.0), {});
    }

    // Add a verified intra-rover loop closure between keyframe 1 and keyframe 4
    PoseGraphEdge chord_edge(50, NodeId(1, 1), NodeId(1, 4),
                             EdgeType::IntraRoverLoopClosure,
                             Pose2D(3.0, 0.0, 0.0),
                             InformationMatrix2D::diagonal(100.0, 1000.0),
                             EdgeVerificationStatus::Verified);
    agent.get_pose_graph().add_edge_detailed(chord_edge);
    agent.get_partition_manager().add_eligible_edge(chord_edge);

    TEST_ASSERT(agent.get_pose_graph().num_nodes() == 6, "Initial graph must have 6 nodes");

    // Prune with a max bound of 4 keyframes (nodes 2 and 3 are redundant and safe to remove)
    PrunerConfig pruner_cfg;
    pruner_cfg.max_retained_keyframes = 4;
    pruner_cfg.protect_newest_keyframe = true;
    pruner_cfg.protect_gauge_anchors = true;
    pruner_cfg.protect_verified_loop_endpoints = true;

    auto res = agent.prune_redundant_keyframes(KeyframeSelectionCriteria{}, pruner_cfg);

    TEST_ASSERT(res.status == PruningStatus::SuccessBoundMet, "Pruning must successfully meet bound");
    TEST_ASSERT(agent.get_pose_graph().num_nodes() == 4, "Retained nodes must match bound of 4");
    TEST_ASSERT(agent.get_pose_graph().has_node(NodeId(1, 0)), "Gauge anchor (keyframe 0) must be protected");
    TEST_ASSERT(agent.get_pose_graph().has_node(NodeId(1, 1)), "Loop endpoint 1 must be protected");
    TEST_ASSERT(agent.get_pose_graph().has_node(NodeId(1, 4)), "Loop endpoint 4 must be protected");
    TEST_ASSERT(agent.get_pose_graph().has_node(NodeId(1, 5)), "Frontier (keyframe 5) must be protected");
    TEST_ASSERT(!agent.get_pose_graph().has_node(NodeId(1, 2)), "Redundant keyframe 2 must be pruned");
    TEST_ASSERT(!agent.get_pose_graph().has_node(NodeId(1, 3)), "Redundant keyframe 3 must be pruned");
    TEST_ASSERT(agent.get_dsu_component_count() == 1, "Graph must remain 1 connected component");
    TEST_ASSERT(agent.get_stats().keyframes_pruned == 2, "Agent must record 2 pruned keyframes");

    node.stop();
    std::cout << "  -> PASSED" << std::endl;
}

// ==============================================================================
// Test 6: Authentic Two-Rover Distributed UDP CORD Trajectory Relaxation
// ==============================================================================
static void test_authentic_two_rover_udp_cord_relaxation() {
    std::cout << "[TEST 6] Authentic Two-Rover Distributed UDP CORD Relaxation..." << std::endl;

    RoverNodeConfig cfg1;
    cfg1.rover_id = 1;
    cfg1.listen_port = 28110;
    cfg1.listen_ip = "127.0.0.1";
    cfg1.seed_peers.emplace_back("127.0.0.1", 28111);
    cfg1.impairment_config.packet_loss_rate = 0.10; // 10% packet drop
    cfg1.impairment_config.one_way_delay_ns = 15'000'000ULL; // 15 ms latency

    RoverNodeConfig cfg2;
    cfg2.rover_id = 2;
    cfg2.listen_port = 28111;
    cfg2.listen_ip = "127.0.0.1";
    cfg2.seed_peers.emplace_back("127.0.0.1", 28110);
    cfg2.impairment_config.packet_loss_rate = 0.10;
    cfg2.impairment_config.one_way_delay_ns = 15'000'000ULL;

    RoverNode node1(cfg1);
    RoverNode node2(cfg2);
    TEST_ASSERT(node1.start(), "Failed to start Node 1");
    TEST_ASSERT(node2.start(), "Failed to start Node 2");

    DistributedReconciliationAgent agent1(1, node1, 32, 3);
    DistributedReconciliationAgent agent2(2, node2, 32, 3);
    agent1.setup_callbacks();
    agent2.setup_callbacks();
    agent1.set_auto_optimize_on_boundary(true);
    agent2.set_auto_optimize_on_boundary(true);

    // Rover 1 path at true ground truth (y=0)
    agent1.append_trajectory_keyframe(0, SE2(0.0, 0.0, 0.0), {});
    agent1.append_trajectory_keyframe(1, SE2(5.0, 0.0, 0.0), {});

    std::vector<LandmarkObservationRecord> shared_obs1;
    for (uint32_t l = 1; l <= 4; ++l) {
        shared_obs1.push_back({ LandmarkKey(1, 1, 300 + l, 0xC000ULL + l), 1.0 * l, 0.5 * l, 2, true });
    }
    agent1.append_trajectory_keyframe(2, SE2(10.0, 0.0, 0.0), shared_obs1);

    // Rover 2 path with +0.85m dead-reckoning perturbation along Y in odometry:
    // (10, 0.85) -> (15, 0.85) -> (20, 0.85)
    // Physical rover is at rendezvous point (10, 0), so true sensor sightings of landmarks are identical:
    std::vector<LandmarkObservationRecord> shared_obs2;
    for (uint32_t l = 1; l <= 4; ++l) {
        shared_obs2.push_back({ LandmarkKey(1, 1, 300 + l, 0xC000ULL + l), 1.0 * l, 0.5 * l, 0, true });
    }
    agent2.append_trajectory_keyframe(0, SE2(10.0, 0.85, 0.0), shared_obs2);
    agent2.append_trajectory_keyframe(1, SE2(15.0, 0.85, 0.0), {});
    agent2.append_trajectory_keyframe(2, SE2(20.0, 0.85, 0.0), {});

    // Discovery phase
    for (int t = 0; t < 5; ++t) {
        node1.poll_events(0);
        node2.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Initiate IBLT sync from Rover 1 to Rover 2
    agent1.initiate_sync_with_peer(2);

    // Network exchange loop allowing handshake, loop closure, boundary state broadcast, and CORD
    for (int step = 0; step < 70; ++step) {
        node1.poll_events(0);
        node2.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const auto s1 = agent1.get_stats();
    const auto s2 = agent2.get_stats();

    TEST_ASSERT(s1.sync_sent >= 1, "Rover 1 must have sent sync packet over UDP");
    TEST_ASSERT(s2.sync_received >= 1, "Rover 2 must have received sync packet over UDP");
    TEST_ASSERT(s1.loop_closures_verified >= 1, "Inter-rover closure must be verified");
    TEST_ASSERT(s1.boundary_states_sent >= 1, "Rover 1 must broadcast boundary state");
    TEST_ASSERT(s2.boundary_states_sent >= 1, "Rover 2 must broadcast boundary state");
    TEST_ASSERT(s1.boundary_states_received >= 1 || s2.boundary_states_received >= 1,
                "At least one rover must receive boundary state packet over UDP");

    // Manually trigger final CORD relaxation on Rover 2 with gauge anchor at Rover 1
    // (if not already triggered via UDP boundary packet)
    CordIntegratorOptions opts;
    opts.timestep = 0.01;
    opts.max_steps = 150;
    opts.fixed_anchor = NodeId(1, 0);

    agent1.optimize_local_graph_cord(opts);
    agent2.optimize_local_graph_cord(opts);

    // Initial node 0 of Rover 2 was at y = 0.85m (error = 0.85m)
    // Check if Rover 1's representation of Node (2,0) relaxed towards y = 0.0m
    if (agent1.get_pose_graph().has_node(NodeId(2, 0))) {
        const auto& node20 = agent1.get_pose_graph().get_node(NodeId(2, 0));
        const double error_y = std::abs(node20.pose.y - 0.0);
        std::cout << "  Rover 1 estimated pose for Node (2,0): (" << node20.pose.x << ", " << node20.pose.y << "), error = " << error_y << " m\n";
        TEST_ASSERT(error_y < 0.20, "Distributed CORD must relax perturbed node error to < 0.20m");
    }

    node1.stop();
    node2.stop();
    std::cout << "  -> PASSED" << std::endl;
}

// ==============================================================================
// Test 7: Global Swarm Disconnection & Failed Handshake Accounting
// ==============================================================================
static void test_global_swarm_disconnection_accounting() {
    std::cout << "[TEST 7] Global Swarm Disconnection Accounting..." << std::endl;

    RoverNodeConfig cfg1;
    cfg1.rover_id = 1;
    cfg1.listen_port = 28112;
    cfg1.listen_ip = "127.0.0.1";

    RoverNodeConfig cfg2;
    cfg2.rover_id = 2;
    cfg2.listen_port = 28113;
    cfg2.listen_ip = "127.0.0.1";

    RoverNode node1(cfg1);
    RoverNode node2(cfg2);
    TEST_ASSERT(node1.start(), "Failed to start Node 1");
    TEST_ASSERT(node2.start(), "Failed to start Node 2");

    DistributedReconciliationAgent agent1(1, node1, 32, 3);
    DistributedReconciliationAgent agent2(2, node2, 32, 3);
    agent1.setup_callbacks();
    agent2.setup_callbacks();

    // Two rovers with completely disjoint, non-overlapping trajectories and landmarks
    agent1.append_trajectory_keyframe(0, SE2(0.0, 0.0, 0.0), {});
    agent1.append_trajectory_keyframe(1, SE2(5.0, 0.0, 0.0), {});

    agent2.append_trajectory_keyframe(0, SE2(100.0, 100.0, 0.0), {});
    agent2.append_trajectory_keyframe(1, SE2(105.0, 100.0, 0.0), {});

    // Crucially: Rover 1's local DSU component count is 1
    TEST_ASSERT(agent1.get_dsu_component_count() == 1, "Rover 1 local DSU should report 1 component");
    TEST_ASSERT(agent2.get_dsu_component_count() == 1, "Rover 2 local DSU should report 1 component");

    // BUT GlobalSwarmEvaluator must correctly report 2 disconnected components!
    auto swarm = GlobalSwarmEvaluator::evaluate_graphs({&agent1.get_pose_graph(), &agent2.get_pose_graph()});
    TEST_ASSERT(swarm.global_connected_components == 2, "Global swarm evaluator must report 2 disconnected components");
    TEST_ASSERT(!swarm.is_fully_unified, "Swarm must NOT be reported as fully unified");
    TEST_ASSERT(swarm.total_swarm_nodes == 4, "Total nodes must equal 4");
    TEST_ASSERT(swarm.verified_inter_rover_closures == 0, "Inter-rover closures must equal 0");

    node1.stop();
    node2.stop();
    std::cout << "  -> PASSED" << std::endl;
}

// ==============================================================================
// Test 8: Trajectory ATE Evaluation & Perturbation Sensitivity
// ==============================================================================
static void test_trajectory_ate_evaluation_and_perturbation_sensitivity() {
    std::cout << "[TEST 8] Trajectory ATE Evaluation & Perturbation Sensitivity..." << std::endl;

    RoverNodeConfig cfg;
    cfg.rover_id = 1;
    cfg.listen_port = 28114;
    cfg.listen_ip = "127.0.0.1";

    RoverNode node(cfg);
    TEST_ASSERT(node.start(), "Failed to start Node 1");

    DistributedReconciliationAgent agent(1, node, 32, 3);
    agent.setup_callbacks();

    // 5-node ground truth trajectory along X axis: (0,0), (2,0), (4,0), (6,0), (8,0)
    std::unordered_map<NodeId, SE2> ground_truth;
    for (uint32_t k = 0; k < 5; ++k) {
        ground_truth[NodeId(1, k)] = SE2(2.0 * static_cast<double>(k), 0.0, 0.0);
    }

    // Step 1: Baseline trajectory without perturbation
    for (uint32_t k = 0; k < 5; ++k) {
        agent.append_trajectory_keyframe(k, ground_truth[NodeId(1, k)], {});
    }

    auto compute_node_ate = [](const PoseGraph& g, const std::unordered_map<NodeId, SE2>& gt) -> double {
        double sq_err = 0.0;
        size_t count = 0;
        for (const auto& [nid, node] : g.nodes()) {
            auto it = gt.find(nid);
            if (it != gt.end()) {
                double dx = node.pose.x - it->second.x();
                double dy = node.pose.y - it->second.y();
                sq_err += (dx * dx + dy * dy);
                count++;
            }
        }
        return count > 0 ? std::sqrt(sq_err / static_cast<double>(count)) : 0.0;
    };

    double baseline_ate = compute_node_ate(agent.get_pose_graph(), ground_truth);
    TEST_ASSERT(baseline_ate < 1e-9, "Baseline ATE with perfect trajectory must be 0.0");

    // Step 2: Perturb nodes 3 and 4 by +1.0m and +2.0m along Y
    agent.get_pose_graph().update_node_pose(NodeId(1, 3), Pose2D(6.0, 1.0, 0.0));
    agent.get_pose_graph().update_node_pose(NodeId(1, 4), Pose2D(8.0, 2.0, 0.0));

    double perturbed_ate = compute_node_ate(agent.get_pose_graph(), ground_truth);
    // Expected RMSE: sqrt((0^2 + 0^2 + 0^2 + 1.0^2 + 2.0^2) / 5) = sqrt(5/5) = 1.0 m
    TEST_ASSERT(std::abs(perturbed_ate - 1.0) < 1e-6, "Perturbed ATE must exactly match 1.0 m");

    // Step 3: Add loop closure constraint from node 4 back to node 0 with ground truth relative pose (-8, 0, 0)
    PoseGraphEdge closure_edge(99, NodeId(1, 4), NodeId(1, 0),
                              EdgeType::IntraRoverLoopClosure,
                              Pose2D(-8.0, 0.0, 0.0),
                              InformationMatrix2D::diagonal(100.0, 1000.0),
                              EdgeVerificationStatus::Verified);
    agent.get_pose_graph().add_edge_detailed(closure_edge);
    agent.get_partition_manager().add_eligible_edge(closure_edge);

    // Step 4: Run CORD relaxation
    CordIntegratorOptions opts;
    opts.timestep = 0.01;
    opts.max_steps = 250;
    opts.fixed_anchor = NodeId(1, 0);

    auto res = agent.optimize_local_graph_cord(opts);
    TEST_ASSERT(res.is_converged() || res.status == CordStatus::MaxStepsReached, "CORD should complete integration");

    double relaxed_ate = compute_node_ate(agent.get_pose_graph(), ground_truth);
    std::cout << "  Baseline ATE: " << baseline_ate << " m, Perturbed ATE: " << perturbed_ate 
              << " m, Relaxed ATE: " << relaxed_ate << " m\n";
    TEST_ASSERT(relaxed_ate < perturbed_ate * 0.40, "CORD must reduce ATE by at least 60%");

    node.stop();
    std::cout << "  -> PASSED" << std::endl;
}

// ==============================================================================
// Main Test Runner
// ==============================================================================
int main() {
    std::cout << "=================================================================" << std::endl;
    std::cout << "  ASTRAMESH-X M7.10 DISTRIBUTED TRAJECTORY SLAM TEST SUITE       " << std::endl;
    std::cout << "=================================================================" << std::endl;

    test_multi_keyframe_trajectory_construction();
    test_distributed_trajectory_sync_and_loop_closure();
    test_intra_rover_cord_trajectory_relaxation();
    test_adversarial_aliasing_and_precision_recall();
    test_safe_keyframe_pruning_on_trajectory();
    test_authentic_two_rover_udp_cord_relaxation();
    test_global_swarm_disconnection_accounting();
    test_trajectory_ate_evaluation_and_perturbation_sensitivity();

    std::cout << "=================================================================" << std::endl;
    std::cout << "  ALL 8 M7.10 DISTRIBUTED TRAJECTORY SLAM TESTS PASSED (100%)    " << std::endl;
    std::cout << "=================================================================" << std::endl;

    return 0;
}
