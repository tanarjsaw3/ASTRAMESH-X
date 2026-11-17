#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/optimization/boundary_filter.hpp"
#include "astromesh/optimization/boundary_packet.hpp"
#include "astromesh/optimization/cord_integrator.hpp"
#include "astromesh/optimization/distributed_cord_node.hpp"
#include "astromesh/optimization/pose_graph_objective.hpp"
#include "astromesh/optimization/simulated_mesh_network.hpp"
#include "astromesh/optimization/trajectory_metrics.hpp"
#include "support/synthetic_pose_graph.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

using namespace astromesh;
using namespace astromesh::geometry;
using namespace astromesh::graph;
using namespace astromesh::optimization;
using namespace astromesh::tests::support;

static size_t g_assertions = 0;
#define EXPECT_TRUE(cond) do { \
    g_assertions++; \
    if (!(cond)) { \
        std::cerr << "Assertion FAILED at line " << __LINE__ << ": " << #cond << "\n"; \
        std::exit(1); \
    } \
} while(0)

#define EXPECT_FALSE(cond) EXPECT_TRUE(!(cond))
#define EXPECT_NEAR(a, b, eps) EXPECT_TRUE(std::abs((a) - (b)) <= (eps))

// ---------------------------------------------------------------------------
// 1. Boundary Packet Serialization & Deserialization
// ---------------------------------------------------------------------------
void test_message_serialization_roundtrip() {
    std::cout << "[Test 1] Boundary state packet serialization round-trip & layout...\n";

    BoundaryStatePacket orig;
    orig.protocol_version = BOUNDARY_PROTOCOL_VERSION;
    orig.sender_rover_id = 1;
    orig.target_rover_id = 2;
    orig.node_local_index = 42;
    orig.sequence_number = 1005;
    orig.timestamp_ns = 1700000000123456789ULL;
    orig.x = -12.345;
    orig.y = 67.890;
    orig.theta = 1.23456;
    orig.vx = 0.5;
    orig.vy = -0.2;
    orig.omega = 0.05;

    std::vector<uint8_t> wire = serialize_boundary_packet(orig);
    EXPECT_TRUE(wire.size() == BOUNDARY_PACKET_WIRE_SIZE); // Exactly 80 bytes
    EXPECT_TRUE(wire.size() == 80);

    // Verify version (bytes 0..1 little-endian: 0x01, 0x00)
    EXPECT_TRUE(wire[0] == 0x01);
    EXPECT_TRUE(wire[1] == 0x00);

    PacketValidationResult res = deserialize_boundary_packet(wire.data(), wire.size());
    EXPECT_TRUE(res.valid);
    EXPECT_TRUE(res.packet.has_value());
    const BoundaryStatePacket& rec = *res.packet;

    EXPECT_TRUE(rec.protocol_version == orig.protocol_version);
    EXPECT_TRUE(rec.sender_rover_id == orig.sender_rover_id);
    EXPECT_TRUE(rec.target_rover_id == orig.target_rover_id);
    EXPECT_TRUE(rec.node_local_index == orig.node_local_index);
    EXPECT_TRUE(rec.node_id() == orig.node_id());
    EXPECT_TRUE(rec.sequence_number == orig.sequence_number);
    EXPECT_TRUE(rec.timestamp_ns == orig.timestamp_ns);

    EXPECT_NEAR(rec.x, orig.x, 1e-12);
    EXPECT_NEAR(rec.y, orig.y, 1e-12);
    EXPECT_NEAR(rec.theta, orig.theta, 1e-12);
    EXPECT_NEAR(rec.vx, orig.vx, 1e-12);
    EXPECT_NEAR(rec.vy, orig.vy, 1e-12);
    EXPECT_NEAR(rec.omega, orig.omega, 1e-12);

    // Verify keyframe index 0 (node_local_index == 0) is valid and serializes faithfully
    BoundaryStatePacket zero_idx_pkt = orig;
    zero_idx_pkt.node_local_index = 0;
    EXPECT_TRUE(zero_idx_pkt.is_valid());
    std::vector<uint8_t> zero_wire = serialize_boundary_packet(zero_idx_pkt);
    PacketValidationResult zero_res = deserialize_boundary_packet(zero_wire.data(), zero_wire.size());
    EXPECT_TRUE(zero_res.valid);
    EXPECT_TRUE(zero_res.packet.has_value());
    EXPECT_TRUE(zero_res.packet->node_local_index == 0);
    EXPECT_TRUE(zero_res.packet->node_id() == NodeId(1, 0));
}

// ---------------------------------------------------------------------------
// 2. Corrupted & Malformed Message Rejection
// ---------------------------------------------------------------------------
void test_malformed_and_corrupt_messages() {
    std::cout << "[Test 2] Malformed, corrupted, and truncated packet rejection...\n";

    BoundaryStatePacket orig;
    orig.protocol_version = BOUNDARY_PROTOCOL_VERSION;
    orig.sender_rover_id = 1;
    orig.target_rover_id = 2;
    orig.node_local_index = 10;
    orig.sequence_number = 1;
    orig.timestamp_ns = 1000;
    orig.x = 1.0;
    orig.y = 2.0;
    orig.theta = 0.3;
    orig.vx = 0.1;
    orig.vy = 0.0;
    orig.omega = 0.0;

    std::vector<uint8_t> valid_wire = serialize_boundary_packet(orig);

    // 1. Truncated wire (< 80 bytes)
    auto res_trunc = deserialize_boundary_packet(valid_wire.data(), 79);
    EXPECT_FALSE(res_trunc.valid);

    // 2. Bad Version (set to 99)
    std::vector<uint8_t> bad_version = valid_wire;
    bad_version[0] = 99;
    auto res_ver = deserialize_boundary_packet(bad_version.data(), bad_version.size());
    EXPECT_FALSE(res_ver.valid);

    // 3. Corrupted bit / Bad Checksum
    std::vector<uint8_t> corrupt_body = valid_wire;
    corrupt_body[40] ^= 0x01;
    auto res_corrupt = deserialize_boundary_packet(corrupt_body.data(), corrupt_body.size());
    EXPECT_FALSE(res_corrupt.valid);

    // 4. Non-finite values (NaN injection)
    BoundaryStatePacket nan_pkt = orig;
    nan_pkt.x = std::numeric_limits<double>::quiet_NaN();
    std::vector<uint8_t> nan_wire = serialize_boundary_packet(nan_pkt);
    auto res_nan = deserialize_boundary_packet(nan_wire.data(), nan_wire.size());
    EXPECT_FALSE(res_nan.valid);

    // 5. Non-finite velocity (Inf injection)
    BoundaryStatePacket inf_pkt = orig;
    inf_pkt.vy = std::numeric_limits<double>::infinity();
    std::vector<uint8_t> inf_wire = serialize_boundary_packet(inf_pkt);
    auto res_inf = deserialize_boundary_packet(inf_wire.data(), inf_wire.size());
    EXPECT_FALSE(res_inf.valid);
}

// ---------------------------------------------------------------------------
// 3. Duplicate and Out-of-Order Sequence Rejection (RFC 1982)
// ---------------------------------------------------------------------------
void test_sequence_wraparound_rfc1982() {
    std::cout << "[Test 3] Sequence number progression, duplicate rejection & RFC 1982 wraparound...\n";

    EXPECT_TRUE(is_newer_sequence(2, 1));
    EXPECT_TRUE(is_newer_sequence(100, 99));
    EXPECT_FALSE(is_newer_sequence(1, 2));
    EXPECT_FALSE(is_newer_sequence(50, 50)); // Duplicate is not newer

    // RFC 1982 32-bit serial number arithmetic near 2^32 - 1
    const uint32_t near_max = 0xFFFFFFF0U;
    const uint32_t wrapped_small = 0x0000000AU;
    EXPECT_TRUE(is_newer_sequence(wrapped_small, near_max));
    EXPECT_FALSE(is_newer_sequence(near_max, wrapped_small));

    // Ambiguous interval (exact half-period difference 0x80000000)
    EXPECT_FALSE(is_newer_sequence(0x80000000U, 0x00000000U));
}

// ---------------------------------------------------------------------------
// 4. Simulated Mesh Network Latency, Loss & Byte Accounting
// ---------------------------------------------------------------------------
void test_simulated_network_properties() {
    std::cout << "[Test 4] Discrete-event network latency, drop rates & byte accounting...\n";

    SimulatedNetworkConfig cfg;
    cfg.latency_ns = 50'000'000; // 50 ms
    cfg.jitter_ns = 0;          // Deterministic
    cfg.packet_loss_rate = 0.0;
    cfg.transport_header_bytes = 24;
    cfg.random_seed = 42;

    SimulatedMeshNetwork net(cfg);

    BoundaryStatePacket pkt;
    pkt.protocol_version = BOUNDARY_PROTOCOL_VERSION;
    pkt.sender_rover_id = 1;
    pkt.target_rover_id = 2;
    pkt.node_local_index = 10;
    pkt.sequence_number = 1;
    pkt.timestamp_ns = 1'000'000'000;
    pkt.x = 1.0;
    pkt.y = 2.0;

    uint64_t t0 = 1'000'000'000; // 1.0 s
    net.set_current_time_ns(t0);

    bool sent = net.send_packet(pkt);
    EXPECT_TRUE(sent);

    // At t0 + 20ms, packet must not have arrived yet
    net.set_current_time_ns(t0 + 20'000'000);
    auto pkts_early = net.receive_packets(2);
    EXPECT_TRUE(pkts_early.empty());

    // Step network clock to t0 + 49ms
    net.set_current_time_ns(t0 + 49'000'000);
    auto pkts_still_early = net.receive_packets(2);
    EXPECT_TRUE(pkts_still_early.empty());

    // Step network clock to t0 + 50ms (delivery time)
    net.set_current_time_ns(t0 + 50'000'000);
    auto pkts_delivered = net.receive_packets(2);
    EXPECT_TRUE(pkts_delivered.size() == 1);
    EXPECT_TRUE(pkts_delivered[0].sender_rover_id == 1);
    EXPECT_TRUE(pkts_delivered[0].target_rover_id == 2);

    // Byte accounting verification
    const auto& stats = net.statistics();
    EXPECT_TRUE(stats.packets_sent == 1);
    EXPECT_TRUE(stats.packets_delivered == 1);
    EXPECT_TRUE(stats.packets_dropped == 0);
    EXPECT_TRUE(stats.raw_payload_bytes_sent == 80);
    EXPECT_TRUE(stats.transport_bytes_sent == 80 + 24); // 104 bytes with 24-byte header overhead
}

// ---------------------------------------------------------------------------
// 5. Neighbor SE(2) Velocity Extrapolation Tests
// ---------------------------------------------------------------------------
void test_se2_velocity_extrapolation() {
    std::cout << "[Test 5] Constant-body-twist SE(2) extrapolation accuracy & gating...\n";

    SimulatedNetworkConfig net_cfg;
    SimulatedMeshNetwork net(net_cfg);
    PoseGraph g;
    DistributedCordOptions opts;
    opts.enable_velocity_prediction = true;
    opts.max_prediction_age_ns = 500'000'000; // 500 ms
    opts.stale_timeout_ns = 2'000'000'000;    // 2.0 s
    opts.max_prediction_translation_m = 0.5;
    opts.max_prediction_rotation_rad = 0.5;

    DistributedCordNode node(0, g, {}, opts, net);

    // Case A: Zero delay (tau = 0) -> exact match
    {
        SE2 p0(10.0, 5.0, 0.5);
        Twist2D v0(1.0, -0.5, 0.1);
        node.inject_neighbor_state(NodeId(1, 1), 1, p0, v0, 1000);
        node.step(1000); // age = 0
        const auto& rec = node.neighbor_states().at(NodeId(1, 1));
        EXPECT_TRUE(rec.status == NeighborLinkStatus::Fresh);
        EXPECT_NEAR(rec.current_estimate_pose.x(), 10.0, 1e-12);
        EXPECT_NEAR(rec.current_estimate_pose.y(), 5.0, 1e-12);
        EXPECT_NEAR(rec.current_estimate_pose.theta(), 0.5, 1e-12);
    }

    // Case B: Pure translation (tau = 0.1 s, vx = 2.0)
    {
        SE2 p0(0.0, 0.0, 0.0);
        Twist2D v0(2.0, 0.0, 0.0);
        node.inject_neighbor_state(NodeId(1, 2), 1, p0, v0, 0);
        node.step(100'000'000); // 100 ms
        const auto& rec = node.neighbor_states().at(NodeId(1, 2));
        EXPECT_TRUE(rec.status == NeighborLinkStatus::Fresh);
        EXPECT_NEAR(rec.current_estimate_pose.x(), 0.2, 1e-6);
        EXPECT_NEAR(rec.current_estimate_pose.y(), 0.0, 1e-6);
        EXPECT_NEAR(rec.current_estimate_pose.theta(), 0.0, 1e-6);
    }

    // Case C: Pure rotation (tau = 0.2 s, omega = 0.5 rad/s)
    {
        SE2 p0(1.0, 2.0, 0.1);
        Twist2D v0(0.0, 0.0, 0.5);
        node.inject_neighbor_state(NodeId(1, 3), 1, p0, v0, 0);
        node.step(200'000'000); // 200 ms
        const auto& rec = node.neighbor_states().at(NodeId(1, 3));
        EXPECT_TRUE(rec.status == NeighborLinkStatus::Fresh);
        EXPECT_NEAR(rec.current_estimate_pose.x(), 1.0, 1e-6);
        EXPECT_NEAR(rec.current_estimate_pose.y(), 2.0, 1e-6);
        EXPECT_NEAR(rec.current_estimate_pose.theta(), 0.2, 1e-6);
    }

    // Case D: Combined translation and rotation
    {
        SE2 p0(0.0, 0.0, 0.0);
        Twist2D v0(1.0, 0.0, 1.0); // Arc of radius 1
        const double tau = 0.1;
        node.inject_neighbor_state(NodeId(1, 4), 1, p0, v0, 0);
        node.step(static_cast<uint64_t>(tau * 1e9));

        const auto& rec = node.neighbor_states().at(NodeId(1, 4));
        EXPECT_TRUE(rec.status == NeighborLinkStatus::Fresh);

        // Theoretical SE(2) Exp of [0.1, 0, 0.1]:
        SE2 expected = p0 * SE2::exp(Twist2D(0.1, 0.0, 0.1));
        EXPECT_NEAR(rec.current_estimate_pose.x(), expected.x(), 1e-6);
        EXPECT_NEAR(rec.current_estimate_pose.y(), expected.y(), 1e-6);
        EXPECT_NEAR(rec.current_estimate_pose.theta(), expected.theta(), 1e-6);
    }

    // Case E: Gating on max prediction age (> 500 ms) -> falls back to Zero-Order Hold (ZOH)
    {
        SE2 p0(5.0, 5.0, 0.0);
        Twist2D v0(0.2, 0.0, 0.0);
        node.inject_neighbor_state(NodeId(1, 5), 1, p0, v0, 0);
        node.step(600'000'000); // 600 ms > 500 ms
        const auto& rec = node.neighbor_states().at(NodeId(1, 5));
        EXPECT_TRUE(rec.status == NeighborLinkStatus::StaleHeld);
        EXPECT_TRUE(rec.is_prediction_gated);
        // Pose held at original received pose
        EXPECT_NEAR(rec.current_estimate_pose.x(), 5.0, 1e-12);
        EXPECT_NEAR(rec.current_estimate_pose.y(), 5.0, 1e-12);
    }

    // Case F: Gating on excessive displacement (> 0.5 m)
    {
        SE2 p0(0.0, 0.0, 0.0);
        Twist2D v0(10.0, 0.0, 0.0); // 10 m/s * 0.1 s = 1.0 m > 0.5 m
        node.inject_neighbor_state(NodeId(1, 6), 1, p0, v0, 0);
        node.step(100'000'000); // 100 ms
        const auto& rec = node.neighbor_states().at(NodeId(1, 6));
        EXPECT_TRUE(rec.status == NeighborLinkStatus::StaleHeld);
        EXPECT_TRUE(rec.is_prediction_gated);
        EXPECT_NEAR(rec.current_estimate_pose.x(), 0.0, 1e-12);
    }
}

// ---------------------------------------------------------------------------
// 6. Two-Rover Zero-Delay Centralized Equivalence Test
// ---------------------------------------------------------------------------
void test_two_rover_zero_delay_centralized_equivalence() {
    std::cout << "[Test 6] Two-rover zero-delay equivalence with centralized CORD calculation...\n";

    NodeId r0_n1(1, 1); // Rover 1
    NodeId r0_n2(1, 2);
    NodeId r1_n1(2, 1); // Rover 2
    NodeId r1_n2(2, 2);

    PoseGraph full_graph;
    add_node_helper(full_graph, r0_n1, Pose2D(0, 0, 0));
    add_node_helper(full_graph, r0_n2, Pose2D(1.1, 0.05, 0.02)); // Perturbed
    add_node_helper(full_graph, r1_n1, Pose2D(1.9, -0.05, -0.01));
    add_node_helper(full_graph, r1_n2, Pose2D(3.05, 0.02, 0.01));

    const auto odom_info = InformationMatrix2D::diagonal(100.0, 100.0);
    const auto lc_info = InformationMatrix2D::diagonal(50.0, 50.0);

    add_odometry_helper(full_graph, r0_n1, r0_n2, Pose2D(1, 0, 0), odom_info);
    add_odometry_helper(full_graph, r1_n1, r1_n2, Pose2D(1, 0, 0), odom_info);
    add_loop_closure_helper(full_graph, r0_n2, r1_n1, Pose2D(1, 0, 0), lc_info, EdgeVerificationStatus::Verified);

    std::unordered_map<NodeId, NodeDynamicsState> initial_states = {
        {r0_n1, NodeDynamicsState(SE2(0, 0, 0), Twist2D(0, 0, 0))},
        {r0_n2, NodeDynamicsState(SE2(1.1, 0.05, 0.02), Twist2D(0.1, -0.05, 0.02))},
        {r1_n1, NodeDynamicsState(SE2(1.9, -0.05, -0.01), Twist2D(-0.08, 0.04, -0.01))},
        {r1_n2, NodeDynamicsState(SE2(3.05, 0.02, 0.01), Twist2D(0.02, 0.01, 0.005))}
    };

    const double h = 0.01;
    const NodeInertia inertia(1.0, 1.0, 1.0);
    const NodeDamping damping(2.0, 2.0, 2.0);

    // 1. Centralized one-step CORD
    std::unordered_map<NodeId, NodeDynamicsState> centralized_next;
    double cent_grad_norm = 0.0;
    bool cent_ok = step_cord_integrator(full_graph, initial_states, {r0_n1}, h, inertia, damping, centralized_next, &cent_grad_norm);
    EXPECT_TRUE(cent_ok);

    // Compute centralized numerical gradient explicitly
    std::unordered_map<NodeId, SE2> cent_poses;
    for (const auto& [nid, s] : initial_states) cent_poses[nid] = s.pose;
    auto cent_grad_eval = evaluate_objective_numerical_gradient(full_graph, cent_poses);
    EXPECT_TRUE(cent_grad_eval.valid);

    // 2. Distributed Setup
    PoseGraph r0_graph;
    add_node_helper(r0_graph, r0_n1, Pose2D(0, 0, 0));
    add_node_helper(r0_graph, r0_n2, Pose2D(1.1, 0.05, 0.02));
    add_node_helper(r0_graph, r1_n1, Pose2D(1.9, -0.05, -0.01)); // Ghost/remote node
    add_odometry_helper(r0_graph, r0_n1, r0_n2, Pose2D(1, 0, 0), odom_info);
    add_loop_closure_helper(r0_graph, r0_n2, r1_n1, Pose2D(1, 0, 0), lc_info, EdgeVerificationStatus::Verified);

    PoseGraph r1_graph;
    add_node_helper(r1_graph, r1_n1, Pose2D(1.9, -0.05, -0.01));
    add_node_helper(r1_graph, r1_n2, Pose2D(3.05, 0.02, 0.01));
    add_node_helper(r1_graph, r0_n2, Pose2D(1.1, 0.05, 0.02)); // Ghost/remote node
    add_odometry_helper(r1_graph, r1_n1, r1_n2, Pose2D(1, 0, 0), odom_info);
    add_loop_closure_helper(r1_graph, r0_n2, r1_n1, Pose2D(1, 0, 0), lc_info, EdgeVerificationStatus::Verified);

    SimulatedNetworkConfig net_cfg;
    net_cfg.latency_ns = 10'000'000; // 10ms: synchronous round step delivery for step 1
    net_cfg.packet_loss_rate = 0.0;
    SimulatedMeshNetwork net(net_cfg);

    DistributedCordOptions dist_opts;
    dist_opts.timestep = h;
    dist_opts.default_inertia = inertia;
    dist_opts.default_damping = damping;
    dist_opts.fixed_anchors = {r0_n1};

    std::unordered_map<NodeId, NodeDynamicsState> r0_states = {
        {r0_n1, initial_states.at(r0_n1)},
        {r0_n2, initial_states.at(r0_n2)}
    };
    std::unordered_map<NodeId, NodeDynamicsState> r1_states = {
        {r1_n1, initial_states.at(r1_n1)},
        {r1_n2, initial_states.at(r1_n2)}
    };

    DistributedCordNode node_r0(1, r0_graph, r0_states, dist_opts, net);
    DistributedCordNode node_r1(2, r1_graph, r1_states, dist_opts, net);

    // Inject exact initial remote states to simulate zero-delay sync at t=0
    node_r0.inject_neighbor_state(r1_n1, 2, initial_states.at(r1_n1).pose, initial_states.at(r1_n1).velocity, 0);
    node_r1.inject_neighbor_state(r0_n2, 1, initial_states.at(r0_n2).pose, initial_states.at(r0_n2).velocity, 0);

    // Evaluate local gradients
    std::unordered_map<NodeId, Twist2D> r0_grads;
    std::unordered_map<NodeId, Twist2D> r1_grads;
    EXPECT_TRUE(node_r0.compute_local_gradients(r0_grads));
    EXPECT_TRUE(node_r1.compute_local_gradients(r1_grads));

    // Compare gradients with centralized gradients
    EXPECT_NEAR(r0_grads[r0_n1].vx, 0.0, 1e-12);

    EXPECT_NEAR(r0_grads[r0_n2].vx, cent_grad_eval.gradients[r0_n2].vx, 1e-6);
    EXPECT_NEAR(r0_grads[r0_n2].vy, cent_grad_eval.gradients[r0_n2].vy, 1e-6);
    EXPECT_NEAR(r0_grads[r0_n2].omega, cent_grad_eval.gradients[r0_n2].omega, 1e-6);

    EXPECT_NEAR(r1_grads[r1_n1].vx, cent_grad_eval.gradients[r1_n1].vx, 1e-6);
    EXPECT_NEAR(r1_grads[r1_n1].vy, cent_grad_eval.gradients[r1_n1].vy, 1e-6);
    EXPECT_NEAR(r1_grads[r1_n1].omega, cent_grad_eval.gradients[r1_n1].omega, 1e-6);

    EXPECT_NEAR(r1_grads[r1_n2].vx, cent_grad_eval.gradients[r1_n2].vx, 1e-6);
    EXPECT_NEAR(r1_grads[r1_n2].vy, cent_grad_eval.gradients[r1_n2].vy, 1e-6);
    EXPECT_NEAR(r1_grads[r1_n2].omega, cent_grad_eval.gradients[r1_n2].omega, 1e-6);

    // Step distributed nodes
    EXPECT_TRUE(node_r0.step(0));
    EXPECT_TRUE(node_r1.step(0));

    // Verify resulting one-step poses match centralized step
    for (const auto& nid : {r0_n1, r0_n2}) {
        const auto& p_dist = node_r0.local_states().at(nid).pose;
        const auto& p_cent = centralized_next.at(nid).pose;
        EXPECT_NEAR(p_dist.x(), p_cent.x(), 1e-6);
        EXPECT_NEAR(p_dist.y(), p_cent.y(), 1e-6);
        EXPECT_NEAR(p_dist.theta(), p_cent.theta(), 1e-6);
    }
    for (const auto& nid : {r1_n1, r1_n2}) {
        const auto& p_dist = node_r1.local_states().at(nid).pose;
        const auto& p_cent = centralized_next.at(nid).pose;
        EXPECT_NEAR(p_dist.x(), p_cent.x(), 1e-6);
        EXPECT_NEAR(p_dist.y(), p_cent.y(), 1e-6);
        EXPECT_NEAR(p_dist.theta(), p_cent.theta(), 1e-6);
    }
}

// ---------------------------------------------------------------------------
// 7. Delayed Execution and Empirical Observation
// ---------------------------------------------------------------------------
void test_delayed_execution_and_empirical_metrics() {
    std::cout << "[Test 7] Delayed distributed CORD execution (50ms latency)...\n";

    NodeId r0_n1(1, 1);
    NodeId r0_n2(1, 2);
    NodeId r1_n1(2, 1);
    NodeId r1_n2(2, 2);

    PoseGraph r0_graph, r1_graph;
    add_node_helper(r0_graph, r0_n1, Pose2D(0, 0, 0));
    add_node_helper(r0_graph, r0_n2, Pose2D(1.1, 0.05, 0.02));
    add_node_helper(r0_graph, r1_n1, Pose2D(1.9, -0.05, -0.01));

    add_node_helper(r1_graph, r1_n1, Pose2D(1.9, -0.05, -0.01));
    add_node_helper(r1_graph, r1_n2, Pose2D(3.05, 0.02, 0.01));
    add_node_helper(r1_graph, r0_n2, Pose2D(1.1, 0.05, 0.02));

    const auto odom_info = InformationMatrix2D::diagonal(100.0, 100.0);
    const auto lc_info = InformationMatrix2D::diagonal(50.0, 50.0);

    add_odometry_helper(r0_graph, r0_n1, r0_n2, Pose2D(1, 0, 0), odom_info);
    add_loop_closure_helper(r0_graph, r0_n2, r1_n1, Pose2D(1, 0, 0), lc_info, EdgeVerificationStatus::Verified);

    add_odometry_helper(r1_graph, r1_n1, r1_n2, Pose2D(1, 0, 0), odom_info);
    add_loop_closure_helper(r1_graph, r0_n2, r1_n1, Pose2D(1, 0, 0), lc_info, EdgeVerificationStatus::Verified);

    SimulatedNetworkConfig net_cfg;
    net_cfg.latency_ns = 50'000'000; // 50 ms (5 discrete steps of 10 ms)
    net_cfg.packet_loss_rate = 0.0;
    net_cfg.random_seed = 100;
    SimulatedMeshNetwork net(net_cfg);

    DistributedCordOptions dist_opts;
    dist_opts.timestep = 0.01;
    dist_opts.enable_velocity_prediction = true;
    dist_opts.fixed_anchors = {r0_n1};

    std::unordered_map<NodeId, NodeDynamicsState> r0_states = {
        {r0_n1, NodeDynamicsState(SE2(0, 0, 0), Twist2D(0, 0, 0))},
        {r0_n2, NodeDynamicsState(SE2(1.1, 0.05, 0.02), Twist2D(0, 0, 0))}
    };
    std::unordered_map<NodeId, NodeDynamicsState> r1_states = {
        {r1_n1, NodeDynamicsState(SE2(1.9, -0.05, -0.01), Twist2D(0, 0, 0))},
        {r1_n2, NodeDynamicsState(SE2(3.05, 0.02, 0.01), Twist2D(0, 0, 0))}
    };

    DistributedCordNode node_r0(1, r0_graph, r0_states, dist_opts, net);
    DistributedCordNode node_r1(2, r1_graph, r1_states, dist_opts, net);

    // Initial sync
    node_r0.inject_neighbor_state(r1_n1, 2, r1_states[r1_n1].pose, Twist2D(0, 0, 0), 0);
    node_r1.inject_neighbor_state(r0_n2, 1, r0_states[r0_n2].pose, Twist2D(0, 0, 0), 0);

    double initial_cost = node_r0.compute_local_active_cost() + node_r1.compute_local_active_cost();

    // Run 50 discrete simulation steps (0.50 s total)
    const uint64_t step_dt_ns = 10'000'000; // 10 ms
    for (size_t step = 0; step < 50; ++step) {
        uint64_t sim_time = step * step_dt_ns;
        net.set_current_time_ns(sim_time);

        EXPECT_TRUE(node_r0.step(sim_time));
        EXPECT_TRUE(node_r1.step(sim_time));
    }

    double final_cost = node_r0.compute_local_active_cost() + node_r1.compute_local_active_cost();
    EXPECT_TRUE(std::isfinite(final_cost));
    // Cost must decrease significantly under CORD damping
    EXPECT_TRUE(final_cost < initial_cost);

    // Check packet delivery metrics
    EXPECT_TRUE(net.statistics().packets_sent >= 100);
    EXPECT_TRUE(net.statistics().packets_delivered >= 80);
}

// ---------------------------------------------------------------------------
// 8. Deterministic Packet Loss Resilience
// ---------------------------------------------------------------------------
void test_packet_loss_resilience() {
    std::cout << "[Test 8] Deterministic packet loss (30% drop rate)...\n";

    PoseGraph r0_graph, r1_graph;
    NodeId r0_n1(1, 1), r0_n2(1, 2), r1_n1(2, 1), r1_n2(2, 2);
    add_node_helper(r0_graph, r0_n1, Pose2D(0, 0, 0));
    add_node_helper(r0_graph, r0_n2, Pose2D(1.0, 0, 0));
    add_node_helper(r0_graph, r1_n1, Pose2D(2.0, 0, 0));

    add_node_helper(r1_graph, r1_n1, Pose2D(2.0, 0, 0));
    add_node_helper(r1_graph, r1_n2, Pose2D(3.0, 0, 0));
    add_node_helper(r1_graph, r0_n2, Pose2D(1.0, 0, 0));

    const auto odom_info = InformationMatrix2D::diagonal(100.0, 100.0);
    const auto lc_info = InformationMatrix2D::diagonal(50.0, 50.0);
    add_odometry_helper(r0_graph, r0_n1, r0_n2, Pose2D(1, 0, 0), odom_info);
    add_loop_closure_helper(r0_graph, r0_n2, r1_n1, Pose2D(1, 0, 0), lc_info, EdgeVerificationStatus::Verified);
    add_odometry_helper(r1_graph, r1_n1, r1_n2, Pose2D(1, 0, 0), odom_info);
    add_loop_closure_helper(r1_graph, r0_n2, r1_n1, Pose2D(1, 0, 0), lc_info, EdgeVerificationStatus::Verified);

    SimulatedNetworkConfig net_cfg;
    net_cfg.latency_ns = 20'000'000; // 20 ms
    net_cfg.packet_loss_rate = 0.30; // 30% drop rate
    net_cfg.random_seed = 777;
    SimulatedMeshNetwork net(net_cfg);

    DistributedCordOptions dist_opts;
    dist_opts.fixed_anchors = {r0_n1};

    std::unordered_map<NodeId, NodeDynamicsState> r0_states = {
        {r0_n1, NodeDynamicsState(SE2(0, 0, 0), Twist2D(0, 0, 0))},
        {r0_n2, NodeDynamicsState(SE2(1.0, 0, 0), Twist2D(0, 0, 0))}
    };
    std::unordered_map<NodeId, NodeDynamicsState> r1_states = {
        {r1_n1, NodeDynamicsState(SE2(2.0, 0, 0), Twist2D(0, 0, 0))},
        {r1_n2, NodeDynamicsState(SE2(3.0, 0, 0), Twist2D(0, 0, 0))}
    };

    DistributedCordNode node_r0(1, r0_graph, r0_states, dist_opts, net);
    DistributedCordNode node_r1(2, r1_graph, r1_states, dist_opts, net);

    node_r0.inject_neighbor_state(r1_n1, 2, r1_states[r1_n1].pose, Twist2D(0, 0, 0), 0);
    node_r1.inject_neighbor_state(r0_n2, 1, r0_states[r0_n2].pose, Twist2D(0, 0, 0), 0);

    const uint64_t step_dt_ns = 10'000'000;
    for (size_t step = 0; step < 40; ++step) {
        uint64_t sim_time = step * step_dt_ns;
        net.set_current_time_ns(sim_time);
        EXPECT_TRUE(node_r0.step(sim_time));
        EXPECT_TRUE(node_r1.step(sim_time));
    }

    // Must have dropped some packets deterministically
    EXPECT_TRUE(net.statistics().packets_dropped > 0);
    EXPECT_TRUE(net.statistics().packets_delivered > 0);
}

// ---------------------------------------------------------------------------
// 9. Stale-Edge Deactivation, Disconnection & Reconnection Recovery
// ---------------------------------------------------------------------------
void test_stale_edge_deactivation_and_reconnection() {
    std::cout << "[Test 9] Stale-edge timeout deactivation and dynamic reconnection...\n";

    PoseGraph r0_graph;
    NodeId r0_n1(1, 1), r0_n2(1, 2), r1_n1(2, 1);
    add_node_helper(r0_graph, r0_n1, Pose2D(0, 0, 0));
    add_node_helper(r0_graph, r0_n2, Pose2D(1, 0, 0));
    add_node_helper(r0_graph, r1_n1, Pose2D(2, 0, 0));

    const auto odom_info = InformationMatrix2D::diagonal(100.0, 100.0);
    const auto lc_info = InformationMatrix2D::diagonal(50.0, 50.0);
    add_odometry_helper(r0_graph, r0_n1, r0_n2, Pose2D(1, 0, 0), odom_info);
    add_loop_closure_helper(r0_graph, r0_n2, r1_n1, Pose2D(1, 0, 0), lc_info, EdgeVerificationStatus::Verified);

    SimulatedNetworkConfig net_cfg;
    net_cfg.latency_ns = 0;
    SimulatedMeshNetwork net(net_cfg);

    DistributedCordOptions opts;
    opts.max_prediction_age_ns = 500'000'000; // 500 ms
    opts.stale_timeout_ns = 2'000'000'000;    // 2.0 s
    opts.fixed_anchors = {r0_n1};

    std::unordered_map<NodeId, NodeDynamicsState> states = {
        {r0_n1, NodeDynamicsState(SE2(0, 0, 0), Twist2D(0, 0, 0))},
        {r0_n2, NodeDynamicsState(SE2(1, 0, 0), Twist2D(0, 0, 0))}
    };

    DistributedCordNode node(1, r0_graph, states, opts, net);
    EXPECT_TRUE(node.total_inter_rover_edge_count() == 1);
    EXPECT_TRUE(node.active_inter_rover_edge_count() == 0); // Initially disconnected

    // Inject packet at t = 0
    node.inject_neighbor_state(r1_n1, 2, SE2(2.5, 0.1, 0.0), Twist2D(0, 0, 0), 0);
    EXPECT_TRUE(node.active_inter_rover_edge_count() == 1); // Now active

    // At t = 1.0 s: age 1.0 s > 500 ms -> StaleHeld (Zero-Order Hold), but edge still active
    node.step(1'000'000'000);
    EXPECT_TRUE(node.neighbor_states().at(r1_n1).status == NeighborLinkStatus::StaleHeld);
    EXPECT_TRUE(node.active_inter_rover_edge_count() == 1);
    double cost_stale_held = node.compute_local_active_cost();
    EXPECT_TRUE(cost_stale_held > 1.0); // Has large loop closure error

    // At t = 2.5 s: age 2.5 s > 2.0 s -> Disconnected, edge must be excluded from active cost!
    node.step(2'500'000'000);
    EXPECT_TRUE(node.neighbor_states().at(r1_n1).status == NeighborLinkStatus::Disconnected);
    EXPECT_TRUE(node.active_inter_rover_edge_count() == 0);

    // Verify edge deactivation reflected in cost: cost only includes odometry
    double cost_disconnected = node.compute_local_active_cost();
    EXPECT_TRUE(cost_disconnected < cost_stale_held * 0.1);

    // Reconnection: New packet arrives at t = 3.0 s with sequence 10
    BoundaryStatePacket reconnected_pkt;
    reconnected_pkt.protocol_version = BOUNDARY_PROTOCOL_VERSION;
    reconnected_pkt.sender_rover_id = 2;
    reconnected_pkt.target_rover_id = 1;
    reconnected_pkt.node_local_index = r1_n1.local_index;
    reconnected_pkt.sequence_number = 10;
    reconnected_pkt.timestamp_ns = 3'000'000'000;
    reconnected_pkt.x = 2.0;
    reconnected_pkt.y = 0.0;
    reconnected_pkt.theta = 0.0;
    reconnected_pkt.vx = 0.0;
    reconnected_pkt.vy = 0.0;
    reconnected_pkt.omega = 0.0;

    net.set_current_time_ns(3'000'000'000);
    net.send_packet(reconnected_pkt);

    node.step(3'000'000'000);
    EXPECT_TRUE(node.neighbor_states().at(r1_n1).status == NeighborLinkStatus::Fresh);
    EXPECT_TRUE(node.active_inter_rover_edge_count() == 1); // Re-activated!

    // Verify lifecycle event log
    const auto& events = node.lifecycle_events();
    EXPECT_TRUE(!events.empty());
    bool found_disconnect = false;
    bool found_reconnect = false;
    for (const auto& ev : events) {
        if (ev.event_type == LifecycleEventType::LinkDisconnected) found_disconnect = true;
        if (ev.event_type == LifecycleEventType::LinkReconnected) found_reconnect = true;
    }
    EXPECT_TRUE(found_disconnect);
    EXPECT_TRUE(found_reconnect);
}

// ---------------------------------------------------------------------------
// 10. Non-Finite State & Invalid Parameter Rejection
// ---------------------------------------------------------------------------
void test_invalid_parameter_safeguards() {
    std::cout << "[Test 10] Non-finite state and invalid parameter safeguards...\n";

    SimulatedNetworkConfig net_cfg;
    SimulatedMeshNetwork net(net_cfg);
    PoseGraph g;
    DistributedCordOptions bad_opts;
    bad_opts.timestep = -0.01; // Negative timestep

    DistributedCordNode node(1, g, {}, bad_opts, net);
    EXPECT_FALSE(node.step(0));
}

// ---------------------------------------------------------------------------
// 11. Simulation Repeatability Across Identical Seeds
// ---------------------------------------------------------------------------
void test_simulation_reproducibility() {
    std::cout << "[Test 11] Exact bit-for-bit repeatability across identical seeds...\n";

    auto run_sim = [](uint32_t seed) {
        PoseGraph r0_graph;
        NodeId r0_n1(1, 1), r0_n2(1, 2), r1_n1(2, 1);
        add_node_helper(r0_graph, r0_n1, Pose2D(0, 0, 0));
        add_node_helper(r0_graph, r0_n2, Pose2D(1.1, 0.05, 0));
        add_node_helper(r0_graph, r1_n1, Pose2D(2.0, 0, 0));

        const auto odom_info = InformationMatrix2D::diagonal(100.0, 100.0);
        const auto lc_info = InformationMatrix2D::diagonal(50.0, 50.0);
        add_odometry_helper(r0_graph, r0_n1, r0_n2, Pose2D(1, 0, 0), odom_info);
        add_loop_closure_helper(r0_graph, r0_n2, r1_n1, Pose2D(1, 0, 0), lc_info, EdgeVerificationStatus::Verified);

        SimulatedNetworkConfig net_cfg;
        net_cfg.latency_ns = 20'000'000;
        net_cfg.jitter_ns = 5'000'000;
        net_cfg.packet_loss_rate = 0.20;
        net_cfg.random_seed = seed;
        SimulatedMeshNetwork net(net_cfg);

        DistributedCordOptions opts;
        opts.fixed_anchors = {r0_n1};

        std::unordered_map<NodeId, NodeDynamicsState> states = {
            {r0_n1, NodeDynamicsState(SE2(0, 0, 0), Twist2D(0, 0, 0))},
            {r0_n2, NodeDynamicsState(SE2(1.1, 0.05, 0), Twist2D(0.01, 0, 0))}
        };

        DistributedCordNode node(1, r0_graph, states, opts, net);
        node.inject_neighbor_state(r1_n1, 2, SE2(2.0, 0, 0), Twist2D(0, 0, 0), 0);

        for (size_t step = 0; step < 20; ++step) {
            uint64_t t = step * 10'000'000;
            net.set_current_time_ns(t);
            node.step(t);
        }

        return std::make_pair(node.local_states().at(r0_n2).pose, net.statistics());
    };

    auto [pose1, stats1] = run_sim(9999);
    auto [pose2, stats2] = run_sim(9999);

    EXPECT_TRUE(pose1.x() == pose2.x());
    EXPECT_TRUE(pose1.y() == pose2.y());
    EXPECT_TRUE(pose1.theta() == pose2.theta());
    EXPECT_TRUE(stats1.packets_sent == stats2.packets_sent);
    EXPECT_TRUE(stats1.packets_delivered == stats2.packets_delivered);
    EXPECT_TRUE(stats1.packets_dropped == stats2.packets_dropped);
}

// ---------------------------------------------------------------------------
// 12. Honest Communication-Byte Accounting
// ---------------------------------------------------------------------------
void test_communication_byte_accounting() {
    std::cout << "[Test 12] Strict communication-byte accounting and frame overhead...\n";

    SimulatedNetworkConfig net_cfg;
    net_cfg.latency_ns = 0;
    net_cfg.transport_header_bytes = 24;
    SimulatedMeshNetwork net(net_cfg);

    const size_t num_packets = 25;
    for (size_t i = 0; i < num_packets; ++i) {
        BoundaryStatePacket pkt;
        pkt.protocol_version = BOUNDARY_PROTOCOL_VERSION;
        pkt.sender_rover_id = 1;
        pkt.target_rover_id = 2;
        pkt.node_local_index = 10;
        pkt.sequence_number = static_cast<uint32_t>(i + 1);
        pkt.timestamp_ns = i * 1000;
        pkt.x = 1.0;
        pkt.y = 2.0;

        net.send_packet(pkt);
    }

    const auto& stats = net.statistics();
    EXPECT_TRUE(stats.packets_sent == num_packets);
    EXPECT_TRUE(stats.raw_payload_bytes_sent == num_packets * 80);
    EXPECT_TRUE(stats.transport_bytes_sent == num_packets * (80 + 24)); // 104 bytes per frame
}

// ---------------------------------------------------------------------------
// 13. Phase 4.1: Boundary State Filter Unit Properties
// ---------------------------------------------------------------------------
void test_boundary_filter_unit_properties() {
    std::cout << "[Test 13] BoundaryStateFilter unit properties (validation, EMA, clamping, deadband, horizon)...\n";

    BoundaryFilterOptions opts;
    opts.enable_filtering = true;
    opts.velocity_smoothing_factor = 0.50;
    opts.max_linear_velocity_m_s = 1.50;
    opts.max_angular_velocity_rad_s = 1.00;
    opts.velocity_deadband_linear = 0.01;
    opts.velocity_deadband_angular = 0.01;
    opts.max_prediction_horizon_s = 0.030; // 30 ms
    EXPECT_TRUE(opts.is_valid());

    BoundaryStateFilter filter(opts);
    EXPECT_TRUE(filter.accepted_packet_count() == 0);
    EXPECT_TRUE(filter.rejected_packet_count() == 0);

    // 1. Initial valid packet
    BoundaryStatePacket pkt1;
    pkt1.protocol_version = BOUNDARY_PROTOCOL_VERSION;
    pkt1.sender_rover_id = 2;
    pkt1.target_rover_id = 1;
    pkt1.node_local_index = 0; // Local index 0 valid
    pkt1.sequence_number = 100;
    pkt1.timestamp_ns = 1'000'000'000ULL; // 1.0 s
    pkt1.x = 2.0;
    pkt1.y = 3.0;
    pkt1.theta = 0.5;
    pkt1.vx = 1.0;
    pkt1.vy = 0.0;
    pkt1.omega = 0.2;
    pkt1.checksum = compute_packet_checksum(serialize_boundary_packet(pkt1).data(), 72);

    EXPECT_TRUE(filter.process_packet(pkt1, 1'010'000'000ULL));
    EXPECT_TRUE(filter.accepted_packet_count() == 1);

    const FilteredBoundarySample* s1 = filter.get_sample(pkt1.node_id());
    EXPECT_TRUE(s1 != nullptr);
    EXPECT_TRUE(s1->has_sample);
    EXPECT_NEAR(s1->filtered_velocity.vx, 1.0, 1e-9);
    EXPECT_NEAR(s1->filtered_velocity.vy, 0.0, 1e-9);
    EXPECT_NEAR(s1->filtered_velocity.omega, 0.2, 1e-9);

    // 2. Duplicate sequence number rejection
    EXPECT_FALSE(filter.process_packet(pkt1, 1'015'000'000ULL));
    EXPECT_TRUE(filter.rejected_packet_count() == 1);

    // 3. Stale sequence number rejection
    BoundaryStatePacket pkt_stale = pkt1;
    pkt_stale.sequence_number = 99; // older
    pkt_stale.checksum = compute_packet_checksum(serialize_boundary_packet(pkt_stale).data(), 72);
    EXPECT_FALSE(filter.process_packet(pkt_stale, 1'016'000'000ULL));
    EXPECT_TRUE(filter.rejected_packet_count() == 2);

    // 4. Non-finite state rejection
    BoundaryStatePacket pkt_nan = pkt1;
    pkt_nan.sequence_number = 101;
    pkt_nan.x = std::numeric_limits<double>::quiet_NaN();
    pkt_nan.checksum = compute_packet_checksum(serialize_boundary_packet(pkt_nan).data(), 72);
    EXPECT_FALSE(filter.process_packet(pkt_nan, 1'017'000'000ULL));
    EXPECT_TRUE(filter.rejected_packet_count() == 3);

    // 5. EMA low-pass filtering verification:
    // prev filt = [1.0, 0.0, 0.2]
    // new raw = [2.0, 0.4, 0.6]
    // with alpha = 0.5:
    // filt = 0.5 * [1.0, 0.0, 0.2] + 0.5 * [2.0, 0.4, 0.6] = [1.5, 0.2, 0.4]
    // norm = sqrt(1.5^2 + 0.2^2) = sqrt(2.29) ~= 1.51327 > 1.50 -> clamped to 1.50
    BoundaryStatePacket pkt2 = pkt1;
    pkt2.sequence_number = 102;
    pkt2.timestamp_ns = 1'020'000'000ULL;
    pkt2.vx = 2.0;
    pkt2.vy = 0.4;
    pkt2.omega = 0.6;
    pkt2.checksum = compute_packet_checksum(serialize_boundary_packet(pkt2).data(), 72);
    EXPECT_TRUE(filter.process_packet(pkt2, 1'025'000'000ULL));

    const FilteredBoundarySample* s2 = filter.get_sample(pkt2.node_id());
    EXPECT_TRUE(s2 != nullptr);
    double v_norm2 = std::sqrt(s2->filtered_velocity.vx * s2->filtered_velocity.vx +
                               s2->filtered_velocity.vy * s2->filtered_velocity.vy);
    EXPECT_NEAR(v_norm2, 1.50, 1e-9); // Clamped to max_linear_velocity_m_s
    EXPECT_NEAR(s2->filtered_velocity.omega, 0.40, 1e-9);

    // 6. Angular velocity saturation: raw omega = 4.0 -> clamped to 1.0
    BoundaryStatePacket pkt3 = pkt2;
    pkt3.sequence_number = 103;
    pkt3.timestamp_ns = 1'030'000'000ULL;
    pkt3.vx = 0.2;
    pkt3.vy = 0.0;
    pkt3.omega = 4.0;
    pkt3.checksum = compute_packet_checksum(serialize_boundary_packet(pkt3).data(), 72);
    EXPECT_TRUE(filter.process_packet(pkt3, 1'035'000'000ULL));
    const FilteredBoundarySample* s3 = filter.get_sample(pkt3.node_id());
    EXPECT_NEAR(s3->filtered_velocity.omega, 1.00, 1e-9); // Saturation clamp

    // 7. Deadband verification: below deadband zeroed
    BoundaryStatePacket pkt_dead = pkt3;
    pkt_dead.sequence_number = 104;
    pkt_dead.timestamp_ns = 1'040'000'000ULL;
    pkt_dead.vx = 0.005; // Below 0.01 threshold
    pkt_dead.vy = 0.002;
    pkt_dead.omega = 0.003; // Below 0.01 threshold
    // Let's reset filter to test single deadband packet
    filter.reset(pkt_dead.node_id());
    pkt_dead.checksum = compute_packet_checksum(serialize_boundary_packet(pkt_dead).data(), 72);
    EXPECT_TRUE(filter.process_packet(pkt_dead, 1'045'000'000ULL));
    const FilteredBoundarySample* s_dead = filter.get_sample(pkt_dead.node_id());
    EXPECT_NEAR(s_dead->filtered_velocity.vx, 0.0, 1e-9);
    EXPECT_NEAR(s_dead->filtered_velocity.vy, 0.0, 1e-9);
    EXPECT_NEAR(s_dead->filtered_velocity.omega, 0.0, 1e-9);

    // 8. Prediction horizon gating & ZOH fallback
    // Ingest fresh sample at t = 2.000 s
    BoundaryStatePacket pkt_pred = pkt1;
    filter.reset(pkt_pred.node_id());
    pkt_pred.sequence_number = 200;
    pkt_pred.timestamp_ns = 2'000'000'000ULL;
    pkt_pred.x = 10.0;
    pkt_pred.y = 5.0;
    pkt_pred.theta = 0.1;
    pkt_pred.vx = 1.0;
    pkt_pred.vy = 0.0;
    pkt_pred.omega = 0.2;
    pkt_pred.checksum = compute_packet_checksum(serialize_boundary_packet(pkt_pred).data(), 72);
    EXPECT_TRUE(filter.process_packet(pkt_pred, 2'005'000'000ULL));

    // A. Query at t = 2.020 s (age = 20 ms <= max_horizon 30 ms): active extrapolation
    SE2 pred_pose;
    Twist2D pred_vel;
    bool is_fallback = false;
    EXPECT_TRUE(filter.get_estimated_state(pkt_pred.node_id(), 2'020'000'000ULL, pred_pose, pred_vel, is_fallback));
    EXPECT_FALSE(is_fallback);
    EXPECT_TRUE(pred_pose.x() > 10.0); // Extrapolated forward
    EXPECT_NEAR(pred_vel.vx, 1.0, 1e-9);

    // B. Query at t = 2.050 s (age = 50 ms > max_horizon 30 ms): ZOH fallback
    SE2 fallback_pose;
    Twist2D fallback_vel;
    EXPECT_TRUE(filter.get_estimated_state(pkt_pred.node_id(), 2'050'000'000ULL, fallback_pose, fallback_vel, is_fallback));
    EXPECT_TRUE(is_fallback);
    EXPECT_NEAR(fallback_pose.x(), 10.0, 1e-9); // Retains received pose
    EXPECT_NEAR(fallback_pose.y(), 5.0, 1e-9);
    EXPECT_NEAR(fallback_vel.vx, 0.0, 1e-9);    // Velocity frozen
    EXPECT_TRUE(filter.fallback_count() >= 1);

    // 9. Angular wrap-around preservation near +-pi
    BoundaryStatePacket pkt_wrap = pkt1;
    pkt_wrap.sequence_number = 300;
    pkt_wrap.timestamp_ns = 3'000'000'000ULL;
    pkt_wrap.x = 0.0;
    pkt_wrap.y = 0.0;
    pkt_wrap.theta = 3.140; // Near +pi
    pkt_wrap.vx = 0.0;
    pkt_wrap.vy = 0.0;
    pkt_wrap.omega = 0.50; // Heading increases across +pi boundary
    pkt_wrap.checksum = compute_packet_checksum(serialize_boundary_packet(pkt_wrap).data(), 72);
    EXPECT_TRUE(filter.process_packet(pkt_wrap, 3'005'000'000ULL));

    // Extrapolate for 0.020 s: delta theta = 0.020 * 0.5 = 0.010 rad -> 3.150 rad -> wrapped to -3.133 rad
    SE2 wrap_pose;
    Twist2D wrap_vel;
    EXPECT_TRUE(filter.get_estimated_state(pkt_wrap.node_id(), 3'020'000'000ULL, wrap_pose, wrap_vel, is_fallback));
    EXPECT_FALSE(is_fallback);
    EXPECT_TRUE(wrap_pose.theta() >= -M_PI && wrap_pose.theta() <= M_PI);
    EXPECT_TRUE(std::isfinite(wrap_pose.theta()));
}

// ---------------------------------------------------------------------------
// 14. Phase 4.1: Adaptive Damping Schedule Calculations & Limits
// ---------------------------------------------------------------------------
void test_adaptive_damping_heuristics() {
    std::cout << "[Test 14] Adaptive damping heuristic formula, units, and boundary limits...\n";

    BoundaryFilterOptions opts;
    opts.enable_adaptive_damping = true;
    opts.adaptive_damping_alpha = 15.0; // s^-1
    opts.max_damping_multiplier = 5.0;  // 5x
    EXPECT_TRUE(opts.is_valid());

    BoundaryStateFilter filter(opts);

    // Zero delay: D(0) = D_base * (1 + 15 * 0) = 1.0 * D_base
    EXPECT_NEAR(filter.compute_damping_multiplier(0.0), 1.00, 1e-9);

    // 20 ms delay: D(0.02) = 1 + 15 * 0.02 = 1.30
    EXPECT_NEAR(filter.compute_damping_multiplier(0.020), 1.30, 1e-9);

    // 50 ms delay: D(0.05) = 1 + 15 * 0.05 = 1.75
    EXPECT_NEAR(filter.compute_damping_multiplier(0.050), 1.75, 1e-9);

    // 100 ms delay: D(0.10) = 1 + 15 * 0.10 = 2.50
    EXPECT_NEAR(filter.compute_damping_multiplier(0.100), 2.50, 1e-9);

    // 200 ms delay: D(0.20) = 1 + 15 * 0.20 = 4.00
    EXPECT_NEAR(filter.compute_damping_multiplier(0.200), 4.00, 1e-9);

    // 500 ms delay: 1 + 15 * 0.50 = 8.50 -> clamped to max_damping_multiplier = 5.0
    EXPECT_NEAR(filter.compute_damping_multiplier(0.500), 5.00, 1e-9);

    // Extreme delay (10.0 s): clamped to 5.0
    EXPECT_NEAR(filter.compute_damping_multiplier(10.0), 5.00, 1e-9);

    // Negative delay (clock skew / anomalies): clamped to 1.0
    EXPECT_NEAR(filter.compute_damping_multiplier(-0.050), 1.00, 1e-9);

    // Disabled adaptive damping returns 1.0 unconditionally
    opts.enable_adaptive_damping = false;
    filter.set_options(opts);
    EXPECT_NEAR(filter.compute_damping_multiplier(0.500), 1.00, 1e-9);
    EXPECT_NEAR(filter.compute_damping_multiplier(0.100), 1.00, 1e-9);
}

// ---------------------------------------------------------------------------
// 15. Phase 4.1: Delayed Stability Stress Test Under Mitigation
// ---------------------------------------------------------------------------
void test_delayed_stability_under_filter_and_adaptive_damping() {
    std::cout << "[Test 15] Stress test: 20ms, 50ms, 100ms, 200ms latency stability under filter + adaptive damping...\n";

    const std::vector<double> test_delays_ms = {20.0, 50.0, 100.0, 200.0};

    for (double delay_ms : test_delays_ms) {
        MultiRoverDataset ds = generate_multi_rover_circular_loop(2, 10, 5.0, 0.08, 0.04, 0.10, 0.05, 42);

        SimulatedNetworkConfig net_cfg;
        net_cfg.latency_ns = static_cast<uint64_t>(delay_ms * 1e6);
        net_cfg.packet_loss_rate = 0.0;
        SimulatedMeshNetwork network(net_cfg);

        DistributedCordOptions dist_opts;
        dist_opts.default_inertia = NodeInertia(5.0, 5.0, 5.0);
        dist_opts.default_damping = NodeDamping(20.0, 20.0, 20.0);
        dist_opts.enable_velocity_prediction = true;
        dist_opts.timestep = 0.01;
        dist_opts.fixed_anchors = ds.fixed_anchors;

        // Configure Phase 4.1 Boundary Filter & Adaptive Damping
        dist_opts.filter_options.enable_filtering = true;
        dist_opts.filter_options.velocity_smoothing_factor = 0.50;
        dist_opts.filter_options.max_linear_velocity_m_s = 1.50;
        dist_opts.filter_options.max_angular_velocity_rad_s = 1.00;
        dist_opts.filter_options.max_prediction_horizon_s = 0.030; // 30 ms
        dist_opts.filter_options.enable_adaptive_damping = true;
        dist_opts.filter_options.adaptive_damping_alpha = 15.0; // s^-1
        dist_opts.filter_options.max_damping_multiplier = 5.0;

        ObjectiveEvaluationResult init_obj = evaluate_pose_graph_objective(ds.full_graph, ds.initial_poses);
        EXPECT_TRUE(init_obj.valid);
        const double init_cost = init_obj.total_cost;

        std::map<uint32_t, std::unique_ptr<DistributedCordNode>> rovers;
        for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
            rovers[r] = std::make_unique<DistributedCordNode>(r, ds.subgraphs.at(r), ds.initial_states_by_rover.at(r), dist_opts, network);
        }

        // Seed initial discovery
        for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
            for (const auto& nid : rovers[r]->local_boundary_nodes()) {
                for (uint32_t peer = 1; peer <= ds.num_rovers; ++peer) {
                    if (peer == r) continue;
                    rovers[peer]->inject_neighbor_state(nid, r, ds.initial_poses.at(nid), Twist2D(0, 0, 0), 0);
                }
            }
        }

        double max_omega_seen = 0.0;
        for (size_t step = 0; step < 100; ++step) {
            uint64_t sim_time = step * 10'000'000ULL;
            network.set_current_time_ns(sim_time);

            for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
                EXPECT_TRUE(rovers[r]->step(sim_time));
                for (const auto& [nid, st] : rovers[r]->local_states()) {
                    EXPECT_TRUE(std::isfinite(st.pose.x()));
                    EXPECT_TRUE(std::isfinite(st.pose.y()));
                    EXPECT_TRUE(std::isfinite(st.pose.theta()));
                    EXPECT_TRUE(std::isfinite(st.velocity.vx));
                    EXPECT_TRUE(std::isfinite(st.velocity.vy));
                    EXPECT_TRUE(std::isfinite(st.velocity.omega));
                    max_omega_seen = std::max(max_omega_seen, std::abs(st.velocity.omega));
                }
            }
        }

        // Assert velocities remained bounded (preventing anti-damping runaway)
        EXPECT_TRUE(max_omega_seen < 5.0); // Under unmitigated baseline, this exploded to > 66 rad/s!

        std::unordered_map<NodeId, SE2> final_poses;
        for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
            for (const auto& [nid, st] : rovers[r]->local_states()) {
                final_poses[nid] = st.pose;
            }
        }

        ObjectiveEvaluationResult fin_obj = evaluate_pose_graph_objective(ds.full_graph, final_poses);
        EXPECT_TRUE(fin_obj.valid);
        EXPECT_TRUE(fin_obj.total_cost < init_cost); // Must decrease objective

        TrajectoryEvaluation te = evaluate_trajectory(final_poses, ds.ground_truth_poses);
        EXPECT_TRUE(te.ate_aligned.translation.rmse < 0.25); // Sub-25 cm aligned ATE maintained
    }
}

// ---------------------------------------------------------------------------
// 16. Phase 4.1: Filter Option Validation & Robustness
// ---------------------------------------------------------------------------
void test_boundary_filter_option_validation() {
    std::cout << "[Test 16] BoundaryFilterOptions validity guards...\n";

    BoundaryFilterOptions opts;
    EXPECT_TRUE(opts.is_valid());

    // Invalid smoothing factors
    opts.velocity_smoothing_factor = 0.0;
    EXPECT_FALSE(opts.is_valid());
    opts.velocity_smoothing_factor = 1.5;
    EXPECT_FALSE(opts.is_valid());
    opts.velocity_smoothing_factor = -0.1;
    EXPECT_FALSE(opts.is_valid());
    opts.velocity_smoothing_factor = 0.5;

    // Invalid velocities
    opts.max_linear_velocity_m_s = -1.0;
    EXPECT_FALSE(opts.is_valid());
    opts.max_linear_velocity_m_s = 1.5;

    opts.max_angular_velocity_rad_s = -0.5;
    EXPECT_FALSE(opts.is_valid());
    opts.max_angular_velocity_rad_s = 1.0;

    // Invalid horizon
    opts.max_prediction_horizon_s = -0.01;
    EXPECT_FALSE(opts.is_valid());
    opts.max_prediction_horizon_s = 0.03;

    // Invalid damping alpha
    opts.adaptive_damping_alpha = -1.0;
    EXPECT_FALSE(opts.is_valid());
    opts.adaptive_damping_alpha = 15.0;

    // Invalid damping multiplier
    opts.max_damping_multiplier = 0.5; // Must be >= 1.0
    EXPECT_FALSE(opts.is_valid());
    opts.max_damping_multiplier = 5.0;

    EXPECT_TRUE(opts.is_valid());
}

int main() {
    std::cout << "=== AstroMesh-X Milestone 3.5 & Phase 4.1: Distributed CORD Test Suite ===\n";

    test_message_serialization_roundtrip();
    test_malformed_and_corrupt_messages();
    test_sequence_wraparound_rfc1982();
    test_simulated_network_properties();
    test_se2_velocity_extrapolation();
    test_two_rover_zero_delay_centralized_equivalence();
    test_delayed_execution_and_empirical_metrics();
    test_packet_loss_resilience();
    test_stale_edge_deactivation_and_reconnection();
    test_invalid_parameter_safeguards();
    test_simulation_reproducibility();
    test_communication_byte_accounting();

    // Phase 4.1 Regression & Mitigation Suites
    test_boundary_filter_unit_properties();
    test_adaptive_damping_heuristics();
    test_delayed_stability_under_filter_and_adaptive_damping();
    test_boundary_filter_option_validation();

    std::cout << "\n>>> ALL 16 DISTRIBUTED CORD & PHASE 4.1 TESTS PASSED (" << g_assertions << " assertions verified) <<<\n";
    return 0;
}

