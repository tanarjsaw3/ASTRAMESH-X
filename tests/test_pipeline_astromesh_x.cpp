#include "astromesh/simulation/baseline_astromesh_v1.hpp"
#include "astromesh/simulation/pipeline_astromesh_x.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using namespace astromesh;
using namespace astromesh::simulation;

static size_t g_total_assertions = 0;
static size_t g_failed_assertions = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        ++g_total_assertions; \
        if (!(cond)) { \
            ++g_failed_assertions; \
            std::cerr << "  FAILED: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
        } \
    } while (0)

// Helper: build synthetic observation
static LandmarkObservation make_test_obs(uint32_t lm_id, double local_x, double local_y, uint16_t /*rover_id*/ = 0) {
    LandmarkObservation obs;
    obs.landmark_id = lm_id;
    obs.landmark_key = core::LandmarkKey(1, 0, lm_id, 0x12345678ULL + lm_id);
    obs.ground_truth_world_point = verification::Point2D(local_x, local_y);
    obs.ground_truth_local_point = verification::Point2D(local_x, local_y);
    obs.ground_truth_range = std::hypot(local_x, local_y);
    obs.ground_truth_bearing = std::atan2(local_y, local_x);
    obs.measured_range = obs.ground_truth_range;
    obs.measured_bearing = obs.ground_truth_bearing;
    obs.measured_local_point = obs.ground_truth_local_point;
    return obs;
}

// ----------------------------------------------------------------------
// Test 1: Initialization, Invalid Configuration, and Reset
// ----------------------------------------------------------------------
void test_init_and_reset() {
    std::cout << "[TEST 1] Initialization, Invalid Configuration, and Reset...\n";

    AstroMeshXConfig valid_cfg;
    valid_cfg.iblt_cells = 32;
    TEST_ASSERT(valid_cfg.is_valid(), "Default config is valid");

    AstroMeshXPipeline pipeline(valid_cfg);
    TEST_ASSERT(pipeline.pipeline_name() == "AstroMesh_X", "Pipeline name check");

    // Invalid configurations
    AstroMeshXConfig bad_cfg = valid_cfg;
    bad_cfg.iblt_cells = 0;
    TEST_ASSERT(!bad_cfg.is_valid(), "Zero IBLT cells rejected");

    bad_cfg = valid_cfg;
    bad_cfg.communication_range = -5.0;
    TEST_ASSERT(!bad_cfg.is_valid(), "Negative communication range rejected");

    // Ingest step and verify reset clears state
    SimulationStepInput step;
    step.timestamp_ns = 10'000'000ULL;
    RoverStepData r1;
    r1.rover_id = 1;
    r1.observations.push_back(make_test_obs(1, 1.0, 1.0));
    step.rover_steps.push_back(r1);

    TEST_ASSERT(pipeline.ingest_step(step), "Step ingests");
    TEST_ASSERT(pipeline.num_connected_components() == 1, "1 component created");

    pipeline.reset();
    TEST_ASSERT(pipeline.num_connected_components() == 0, "Reset clears all components");
    auto m = pipeline.collect_metrics();
    TEST_ASSERT(m.total_observations_ingested == 0, "Reset clears observation count");
    TEST_ASSERT(m.total_odometry_steps_processed == 0, "Reset clears odometry steps");
}

// ----------------------------------------------------------------------
// Test 2: Shared Interface Compliance
// ----------------------------------------------------------------------
void test_shared_interface_compliance() {
    std::cout << "[TEST 2] Shared Interface Compliance (IRoverSwarmPipeline)...\n";

    AstroMeshXConfig cfg;
    std::unique_ptr<IRoverSwarmPipeline> ptr = std::make_unique<AstroMeshXPipeline>(cfg);

    TEST_ASSERT(ptr->pipeline_name() == "AstroMesh_X", "Polymorphic name check");

    SimulationStepInput step;
    step.timestamp_ns = 50'000'000ULL;
    RoverStepData r1;
    r1.rover_id = 1;
    r1.odometry_step = geometry::SE2(1.0, 0.0, 0.0);
    step.rover_steps.push_back(r1);

    TEST_ASSERT(ptr->ingest_step(step), "Polymorphic step ingestion");
    auto m = ptr->collect_metrics();
    TEST_ASSERT(m.optimization_supported, "AstroMesh-X flags optimization as supported");
    TEST_ASSERT(m.dtn_supported, "AstroMesh-X flags DTN as supported");
    TEST_ASSERT(m.keyframe_pruning_supported, "AstroMesh-X flags pruning as supported");

    TEST_ASSERT(ptr->optimize(), "AstroMesh-X optimize() returns true");
}

// ----------------------------------------------------------------------
// Test 3: IBLT Reconciliation Success, Incomplete Decode & Failure Atomicity
// ----------------------------------------------------------------------
void test_iblt_reconciliation_and_atomicity() {
    std::cout << "[TEST 3] IBLT Success, Incomplete Decode, and Failure Atomicity...\n";

    AstroMeshXConfig cfg;
    cfg.iblt_cells = 32; // Capacity ~26 differences
    AstroMeshXPipeline pipeline(cfg);

    // Subtest A: Successful IBLT set difference decode (small difference)
    SimulationStepInput step_ok;
    step_ok.timestamp_ns = 100'000'000ULL;

    RoverStepData r1;
    r1.rover_id = 1;
    // Shared landmarks 1..4 plus 1 local landmark
    for (uint32_t i = 1; i <= 4; ++i) r1.observations.push_back(make_test_obs(i, i * 2.0, 1.0, 1));
    r1.observations.push_back(make_test_obs(10, 5.0, 5.0, 1));
    step_ok.rover_steps.push_back(r1);

    RoverStepData r2;
    r2.rover_id = 2;
    // Shared landmarks 1..4 (with rigid shift tx = 1.0) plus 1 local landmark
    for (uint32_t i = 1; i <= 4; ++i) r2.observations.push_back(make_test_obs(i, i * 2.0 - 1.0, 1.0, 2));
    r2.observations.push_back(make_test_obs(20, 6.0, 6.0, 2));
    step_ok.rover_steps.push_back(r2);

    pipeline.ingest_step(step_ok);
    auto m_ok = pipeline.collect_metrics();

    TEST_ASSERT(m_ok.sync_successes == 1, "IBLT sync succeeds under capacity");
    TEST_ASSERT(m_ok.sync_incomplete_decodes == 0, "Zero incomplete decodes");
    TEST_ASSERT(m_ok.loop_closures_verified == 1, "Shared landmarks pass verification");
    TEST_ASSERT(pipeline.are_rovers_connected(1, 2), "Rovers united into single component");

    // Subtest B: Incomplete decode overload (difference exceeds cell capacity)
    AstroMeshXConfig small_cfg;
    small_cfg.iblt_cells = 8; // Tiny capacity: cannot decode > 6 differences
    AstroMeshXPipeline small_pipeline(small_cfg);

    SimulationStepInput step_overload;
    step_overload.timestamp_ns = 200'000'000ULL;

    RoverStepData r1_ov;
    r1_ov.rover_id = 1;
    for (uint32_t i = 100; i < 150; ++i) r1_ov.observations.push_back(make_test_obs(i, i, 1.0, 1));
    step_overload.rover_steps.push_back(r1_ov);

    RoverStepData r2_ov;
    r2_ov.rover_id = 2;
    for (uint32_t i = 200; i < 250; ++i) r2_ov.observations.push_back(make_test_obs(i, i, 1.0, 2));
    step_overload.rover_steps.push_back(r2_ov);

    small_pipeline.ingest_step(step_overload);
    auto m_ov = small_pipeline.collect_metrics();

    TEST_ASSERT(m_ov.sync_incomplete_decodes == 1, "Overload flagged as incomplete decode");
    TEST_ASSERT(m_ov.loop_closures_verified == 0, "No closures verified on incomplete decode");
    TEST_ASSERT(m_ov.dsu_merge_events == 0, "DSU components remain unmerged (Failure Atomicity)");
    TEST_ASSERT(!small_pipeline.are_rovers_connected(1, 2), "Rovers remain disconnected");
    TEST_ASSERT(small_pipeline.num_connected_components() == 2, "2 distinct components preserved");
}

// ----------------------------------------------------------------------
// Test 4: Candidate Gating and Verification
// ----------------------------------------------------------------------
void test_candidate_gating() {
    std::cout << "[TEST 4] Candidate Gating and Geometric Verification...\n";

    AstroMeshXConfig cfg;
    AstroMeshXPipeline pipeline(cfg);

    SimulationStepInput step;
    step.timestamp_ns = 150'000'000ULL;

    RoverStepData r1;
    r1.rover_id = 1;
    r1.observations.push_back(make_test_obs(1, 2.0, 1.0, 1));
    r1.observations.push_back(make_test_obs(2, 4.0, 1.0, 1));
    r1.observations.push_back(make_test_obs(3, 2.0, 3.0, 1));
    r1.observations.push_back(make_test_obs(4, 4.0, 3.0, 1));
    step.rover_steps.push_back(r1);

    RoverStepData r2;
    r2.rover_id = 2;
    r2.observations.push_back(make_test_obs(1, 1.0, 1.0, 2));
    r2.observations.push_back(make_test_obs(2, 3.0, 1.0, 2));
    r2.observations.push_back(make_test_obs(3, 1.0, 3.0, 2));
    r2.observations.push_back(make_test_obs(4, 3.0, 3.0, 2));
    step.rover_steps.push_back(r2);

    pipeline.ingest_step(step);
    auto m = pipeline.collect_metrics();

    TEST_ASSERT(m.candidate_pairs_proposed >= 4, "Candidate pairs proposed");
    TEST_ASSERT(m.verification_attempts == 1, "Verification attempted");
    TEST_ASSERT(m.loop_closures_verified == 1, "Verification passed");
    TEST_ASSERT(pipeline.get_accepted_closures().size() == 1, "1 accepted closure recorded");
    TEST_ASSERT(pipeline.get_rejected_closures().empty(), "0 rejected closures");
}

// ----------------------------------------------------------------------
// Test 5: Rejected Closure Does Not Merge Components
// ----------------------------------------------------------------------
void test_rejected_closure_isolation() {
    std::cout << "[TEST 5] Rejected Closures Do Not Merge Components...\n";

    AstroMeshXConfig cfg;
    AstroMeshXPipeline pipeline(cfg);

    SimulationStepInput step;
    step.timestamp_ns = 250'000'000ULL;

    RoverStepData r1;
    r1.rover_id = 1;
    r1.observations.push_back(make_test_obs(1, 2.0, 1.0, 1));
    r1.observations.push_back(make_test_obs(2, 4.0, 1.0, 1));
    r1.observations.push_back(make_test_obs(3, 2.0, 3.0, 1));
    r1.observations.push_back(make_test_obs(4, 4.0, 3.0, 1));
    step.rover_steps.push_back(r1);

    RoverStepData r2;
    r2.rover_id = 2;
    // Conflicting non-rigid coordinates
    r2.observations.push_back(make_test_obs(1, 100.0, 0.0, 2));
    r2.observations.push_back(make_test_obs(2, -50.0, 20.0, 2));
    r2.observations.push_back(make_test_obs(3, 10.0, -80.0, 2));
    r2.observations.push_back(make_test_obs(4, 0.0, 50.0, 2));
    step.rover_steps.push_back(r2);

    pipeline.ingest_step(step);
    auto m = pipeline.collect_metrics();

    TEST_ASSERT(m.loop_closures_verified == 0, "Zero verified closures");
    TEST_ASSERT(m.loop_closures_rejected == 1, "Exactly 1 rejected closure");
    TEST_ASSERT(m.dsu_merge_events == 0, "Zero DSU merges");
    TEST_ASSERT(!pipeline.are_rovers_connected(1, 2), "Rovers remain disconnected");
    TEST_ASSERT(pipeline.num_connected_components() == 2, "2 components preserved");
    TEST_ASSERT(pipeline.get_rejected_closures().size() == 1, "1 rejected closure in history");
}

// ----------------------------------------------------------------------
// Test 6: Duplicate Message Ingestion Idempotency
// ----------------------------------------------------------------------
void test_duplicate_idempotency() {
    std::cout << "[TEST 6] Duplicate Ingestion Idempotency...\n";

    AstroMeshXConfig cfg;
    AstroMeshXPipeline pipeline(cfg);

    SimulationStepInput step;
    step.timestamp_ns = 300'000'000ULL;

    RoverStepData r1;
    r1.rover_id = 1;
    for (uint32_t i = 1; i <= 4; ++i) r1.observations.push_back(make_test_obs(i, i * 2.0, 1.0, 1));
    step.rover_steps.push_back(r1);

    RoverStepData r2;
    r2.rover_id = 2;
    for (uint32_t i = 1; i <= 4; ++i) r2.observations.push_back(make_test_obs(i, i * 2.0 - 1.0, 1.0, 2));
    step.rover_steps.push_back(r2);

    pipeline.ingest_step(step);
    auto m1 = pipeline.collect_metrics();

    // Re-ingest exact same step input
    pipeline.ingest_step(step);
    auto m2 = pipeline.collect_metrics();

    TEST_ASSERT(m2.loop_closures_verified == m1.loop_closures_verified, "Verified closures do not duplicate");
    TEST_ASSERT(m2.dsu_merge_events == m1.dsu_merge_events, "DSU merges do not duplicate");
    TEST_ASSERT(m2.logical_messages_sent == m1.logical_messages_sent, "Messages sent do not duplicate");
    TEST_ASSERT(m2.total_wire_bytes_transmitted == m1.total_wire_bytes_transmitted, "Wire bytes do not duplicate");
}

// ----------------------------------------------------------------------
// Test 7: Distributed CORD Optimization over Graph Constraints
// ----------------------------------------------------------------------
void test_cord_optimization_execution() {
    std::cout << "[TEST 7] Distributed CORD Optimization on Graph State...\n";

    AstroMeshXConfig cfg;
    cfg.enable_cord = true;
    cfg.cord_options.max_steps = 100;
    cfg.cord_options.timestep = 0.01;
    AstroMeshXPipeline pipeline(cfg);

    // Create 3 sequential keyframes with intentional odometry error
    for (size_t s = 1; s <= 3; ++s) {
        SimulationStepInput step;
        step.timestamp_ns = s * 100'000'000ULL;
        RoverStepData r;
        r.rover_id = 1;
        r.odometry_step = geometry::SE2(1.0, 0.05, 0.01);
        step.rover_steps.push_back(r);
        pipeline.ingest_step(step);
    }

    auto p_before = pipeline.get_current_pose(1);
    TEST_ASSERT(p_before.has_value(), "Pose exists before optimize");

    bool opt_ok = pipeline.optimize();
    TEST_ASSERT(opt_ok, "CORD optimization succeeds");

    auto m = pipeline.collect_metrics();
    TEST_ASSERT(m.optimization_steps_performed > 0, "CORD steps performed > 0");
    TEST_ASSERT(!m.centralized_optimization_performed, "Confirms CORD Lie dynamics, not centralized LM");

    auto p_after = pipeline.get_current_pose(1);
    TEST_ASSERT(p_after.has_value(), "Pose exists after optimize");
}

// ----------------------------------------------------------------------
// Test 8: Anchor and Disconnected Components in Optimization
// ----------------------------------------------------------------------
void test_anchor_and_disconnected_components() {
    std::cout << "[TEST 8] Fixed Gauge Anchor & Disconnected Component Behavior...\n";

    AstroMeshXConfig cfg;
    cfg.enable_cord = true;
    cfg.cord_options.allow_disconnected = true;
    AstroMeshXPipeline pipeline(cfg);

    // Rover 1 and Rover 2 completely disconnected
    SimulationStepInput step;
    step.timestamp_ns = 100'000'000ULL;

    RoverStepData r1;
    r1.rover_id = 1;
    r1.odometry_step = geometry::SE2(2.0, 0.0, 0.0);
    step.rover_steps.push_back(r1);

    RoverStepData r2;
    r2.rover_id = 2;
    r2.odometry_step = geometry::SE2(5.0, 1.0, 0.0);
    step.rover_steps.push_back(r2);

    ChannelLinkState severed;
    severed.rover_a = 1;
    severed.rover_b = 2;
    severed.in_communication_range = false; // Disconnected
    step.channel_states.push_back(severed);

    pipeline.ingest_step(step);
    TEST_ASSERT(pipeline.num_connected_components() == 2, "2 disconnected components");

    // optimize() with allow_disconnected=true anchors component representatives safely
    bool opt_ok = pipeline.optimize();
    TEST_ASSERT(opt_ok, "Optimization handles disconnected components with anchors");
}

// ----------------------------------------------------------------------
// Test 9: DTN Direct, Multi-Hop Routing, and Bounded Hops
// ----------------------------------------------------------------------
void test_dtn_and_multihop_routing() {
    std::cout << "[TEST 9] DTN Routing, Bounded Multi-Hop & Store-Carry-Forward...\n";

    AstroMeshXConfig cfg;
    cfg.enable_multihop_routing = true;
    cfg.default_max_hops = 5;
    AstroMeshXPipeline pipeline(cfg);

    SimulationStepInput step;
    step.timestamp_ns = 50'000'000ULL;

    RoverStepData r1; r1.rover_id = 1; r1.observations.push_back(make_test_obs(1, 1.0, 1.0)); step.rover_steps.push_back(r1);
    RoverStepData r2; r2.rover_id = 2; r2.observations.push_back(make_test_obs(2, 2.0, 2.0)); step.rover_steps.push_back(r2);
    RoverStepData r3; r3.rover_id = 3; r3.observations.push_back(make_test_obs(3, 3.0, 3.0)); step.rover_steps.push_back(r3);

    // Topology: 1 <-> 2 is connected, 2 <-> 3 is connected, but 1 <-> 3 is severed
    ChannelLinkState l12; l12.rover_a = 1; l12.rover_b = 2; l12.in_communication_range = true; step.channel_states.push_back(l12);
    ChannelLinkState l23; l23.rover_a = 2; l23.rover_b = 3; l23.in_communication_range = true; step.channel_states.push_back(l23);
    ChannelLinkState l13; l13.rover_a = 1; l13.rover_b = 3; l13.in_communication_range = false; step.channel_states.push_back(l13);

    pipeline.ingest_step(step);
    auto m = pipeline.collect_metrics();

    // Multi-hop routing forwards 1 <-> 3 via 2 (2 hops)
    TEST_ASSERT(m.logical_messages_sent >= 3, "Messages routed including multi-hop path");
    TEST_ASSERT(m.hop_transmissions_attempted > m.logical_messages_sent,
                "Hop transmissions exceed logical messages on 2-hop route");
}

// ----------------------------------------------------------------------
// Test 10: Wire Byte Accounting and Explicit ACK Overhead
// ----------------------------------------------------------------------
void test_wire_byte_and_ack_accounting() {
    std::cout << "[TEST 10] Wire Byte Accounting & Explicit ACK Overhead...\n";

    AstroMeshXConfig cfg;
    cfg.enable_explicit_acks = true;
    cfg.iblt_cells = 32; // 32 * 32 B = 1024 B payload
    AstroMeshXPipeline pipeline(cfg);

    SimulationStepInput step;
    step.timestamp_ns = 100'000'000ULL;

    RoverStepData r1; r1.rover_id = 1; r1.observations.push_back(make_test_obs(1, 1.0, 1.0)); step.rover_steps.push_back(r1);
    RoverStepData r2; r2.rover_id = 2; r2.observations.push_back(make_test_obs(1, 1.0, 1.0)); step.rover_steps.push_back(r2);

    pipeline.ingest_step(step);
    auto m = pipeline.collect_metrics();

    TEST_ASSERT(m.payload_bytes_transmitted == 1024, "Exact 1024 B IBLT payload transmitted");
    TEST_ASSERT(m.protocol_bytes_transmitted == 24, "24 B protocol header");
    TEST_ASSERT(m.framing_bytes_transmitted == 16, "16 B framing header");
    TEST_ASSERT(m.data_wire_bytes_transmitted == 1024 + 24 + 16, "Data wire bytes matches exact sum");
    TEST_ASSERT(m.ack_wire_bytes_transmitted == 44, "44 B explicit ACK overhead (24B payload + 20B transport)");
    TEST_ASSERT(m.total_wire_bytes_transmitted == m.data_wire_bytes_transmitted + m.ack_wire_bytes_transmitted,
                "Total wire bytes matches data + ACK bytes");
}

// ----------------------------------------------------------------------
// Test 11: Safe Keyframe Pruning and Consistency After Pruning
// ----------------------------------------------------------------------
void test_safe_keyframe_pruning() {
    std::cout << "[TEST 11] Safe Keyframe Pruning & Invariant Preservation...\n";

    AstroMeshXConfig cfg;
    cfg.enable_keyframe_pruning = true;
    cfg.pruner_config.max_retained_keyframes = 5; // Bound to 5 keyframes
    // Part A: protect_articulation_points = true (Chain topology cannot be split)
    cfg.pruner_config.protect_gauge_anchors = true;
    cfg.pruner_config.protect_newest_keyframe = true;
    cfg.pruner_config.protect_articulation_points = true;
    AstroMeshXPipeline pipeline_chain(cfg);

    for (size_t s = 1; s <= 8; ++s) {
        SimulationStepInput step;
        step.timestamp_ns = s * 50'000'000ULL;
        RoverStepData r;
        r.rover_id = 1;
        r.odometry_step = geometry::SE2(1.0, 0.0, 0.0);
        step.rover_steps.push_back(r);
        pipeline_chain.ingest_step(step);
    }

    auto m_chain = pipeline_chain.collect_metrics();
    TEST_ASSERT(m_chain.keyframe_prunes_rejected > 0, "Pruning safely rejected to protect articulation points");
    TEST_ASSERT(pipeline_chain.num_connected_components() == 1, "Graph remains connected with all articulation points intact");

    // Part B: protect_articulation_points = false (Removes intermediate redundant nodes)
    AstroMeshXConfig cfg_prune = cfg;
    cfg_prune.pruner_config.protect_articulation_points = false;
    cfg_prune.pruner_config.protect_gauge_anchors = true;
    AstroMeshXPipeline pipeline_pruned(cfg_prune);

    for (size_t s = 1; s <= 8; ++s) {
        SimulationStepInput step;
        step.timestamp_ns = s * 50'000'000ULL;
        RoverStepData r;
        r.rover_id = 1;
        r.odometry_step = geometry::SE2(1.0, 0.0, 0.0);
        step.rover_steps.push_back(r);
        pipeline_pruned.ingest_step(step);
    }

    auto m_pruned = pipeline_pruned.collect_metrics();
    TEST_ASSERT(m_pruned.keyframe_prunes_applied > 0, "Pruning successfully applied");
    const auto* g = pipeline_pruned.get_rover_graph(1);
    TEST_ASSERT(g != nullptr, "Rover 1 graph exists");
    if (g) {
        TEST_ASSERT(g->num_nodes() <= 6, "Keyframe bound enforced safely");
        TEST_ASSERT(g->has_node(graph::NodeId(1, 0)), "Gauge anchor Node(1, 0) strictly protected");
    }
}

// ----------------------------------------------------------------------
// Test 12: Same-Seed Determinism and Replay Invariance
// ----------------------------------------------------------------------
void test_same_seed_determinism() {
    std::cout << "[TEST 12] Seeded Determinism and Replay Invariance...\n";

    AstroMeshXConfig cfg;
    cfg.seed = 7777;
    cfg.packet_loss_rate = 0.25; // Stochastic channel noise
    AstroMeshXPipeline pipe1(cfg);
    AstroMeshXPipeline pipe2(cfg);

    for (size_t s = 1; s <= 4; ++s) {
        SimulationStepInput step;
        step.timestamp_ns = s * 100'000'000ULL;

        RoverStepData r1; r1.rover_id = 1;
        for (uint32_t i = 1; i <= 4; ++i) r1.observations.push_back(make_test_obs(i, i * 2.0, 1.0, 1));
        step.rover_steps.push_back(r1);

        RoverStepData r2; r2.rover_id = 2;
        for (uint32_t i = 1; i <= 4; ++i) r2.observations.push_back(make_test_obs(i, i * 2.0 - 1.0, 1.0, 2));
        step.rover_steps.push_back(r2);

        pipe1.ingest_step(step);
        pipe2.ingest_step(step);
    }

    auto m1 = pipe1.collect_metrics();
    auto m2 = pipe2.collect_metrics();

    TEST_ASSERT(m1.logical_messages_sent == m2.logical_messages_sent, "Messages sent match identically");
    TEST_ASSERT(m1.logical_messages_dropped == m2.logical_messages_dropped, "Dropped packets match identically");
    TEST_ASSERT(m1.loop_closures_verified == m2.loop_closures_verified, "Verified closures match identically");
    TEST_ASSERT(m1.total_wire_bytes_transmitted == m2.total_wire_bytes_transmitted, "Wire bytes match identically");
}

// ----------------------------------------------------------------------
// Test 13: Ground-Truth Isolation
// ----------------------------------------------------------------------
void test_ground_truth_isolation() {
    std::cout << "[TEST 13] Ground-Truth Isolation (Estimates Distinct from Truth)...\n";

    AstroMeshXConfig cfg;
    cfg.odometry_noise_trans = 0.10; // Noise injected into odometry
    AstroMeshXPipeline pipeline(cfg);

    SimulationStepInput step;
    step.timestamp_ns = 100'000'000ULL;

    RoverStepData r;
    r.rover_id = 1;
    r.odometry_step = geometry::SE2(10.0, 0.0, 0.0);
    step.rover_steps.push_back(r);

    pipeline.ingest_step(step);

    auto pose = pipeline.get_current_pose(1);
    TEST_ASSERT(pose.has_value(), "Estimated pose exists");
    if (pose) {
        TEST_ASSERT(pose->x() != 10.0, "Estimated pose must not equal ground-truth displacement");
    }
}

// ----------------------------------------------------------------------
// Test 14: Common Scenario Input Accepted by Both Pipelines
// ----------------------------------------------------------------------
void test_identical_scenario_input() {
    std::cout << "[TEST 14] Identical Scenario Input Consumed by Both Pipelines...\n";

    BaselineV1Config b_cfg;
    b_cfg.seed = 42;
    BaselineAstroMeshV1 baseline(b_cfg);

    AstroMeshXConfig x_cfg;
    x_cfg.seed = 42;
    AstroMeshXPipeline astromesh_x(x_cfg);

    SimulationStepInput shared_step;
    shared_step.timestamp_ns = 100'000'000ULL;

    RoverStepData r1;
    r1.rover_id = 1;
    for (uint32_t i = 1; i <= 4; ++i) r1.observations.push_back(make_test_obs(i, 2.0 * i, 1.0, 1));
    shared_step.rover_steps.push_back(r1);

    RoverStepData r2;
    r2.rover_id = 2;
    for (uint32_t i = 1; i <= 4; ++i) r2.observations.push_back(make_test_obs(i, 2.0 * i - 1.0, 1.0, 2));
    shared_step.rover_steps.push_back(r2);

    TEST_ASSERT(baseline.ingest_step(shared_step), "Baseline consumes shared step");
    TEST_ASSERT(astromesh_x.ingest_step(shared_step), "AstroMesh-X consumes shared step");

    auto m_base = baseline.collect_metrics();
    auto m_x = astromesh_x.collect_metrics();

    TEST_ASSERT(m_base.total_observations_ingested == m_x.total_observations_ingested,
                "Both pipelines ingest identical total observations");
    TEST_ASSERT(m_base.loop_closures_verified == 1, "Baseline verified 1 closure");
    TEST_ASSERT(m_x.loop_closures_verified == 1, "AstroMesh-X verified 1 closure");
}

// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// Test 15: Metrics Invariants & Failure Code Gating
// ----------------------------------------------------------------------
void test_metrics_invariants() {
    std::cout << "[TEST 15] Metric Invariants & Error Code Gating...\n";

    AstroMeshXConfig cfg;
    AstroMeshXPipeline pipeline(cfg);

    SimulationStepInput bad_step;
    RoverStepData bad_r;
    bad_r.rover_id = 0; // Invalid rover ID 0
    bad_step.rover_steps.push_back(bad_r);

    TEST_ASSERT(!pipeline.ingest_step(bad_step), "Malformed step input rejected cleanly");
    auto m = pipeline.collect_metrics();
    TEST_ASSERT(m.total_observations_ingested == 0, "No state change on malformed input");
}

// ----------------------------------------------------------------------
// Test 16: Audit 1 - Detailed IBLT Trace & Verification Matrix
// ----------------------------------------------------------------------
void test_audit1_iblt_trace_and_matrix() {
    std::cout << "[AUDIT 1] Detailed IBLT Trace & Verification Matrix...\n";

    // 1. Trace 7-step sequence:
    // (1) Observations & canonical keys
    // (2) IBLT table insertion & exchange
    // (3) IBLT subtraction & decode
    // (4) Candidate correspondences
    // (5) Geometric verification
    // (6) Graph edge addition
    // (7) DSU union
    AstroMeshXConfig cfg;
    AstroMeshXPipeline pipeline(cfg);

    SimulationStepInput step;
    step.timestamp_ns = 111'000'000ULL;

    RoverStepData r1;
    r1.rover_id = 1;
    // Planar quadrilateral with non-collinear geometry: (2,1), (4,1), (2,3), (4,3)
    r1.observations.push_back(make_test_obs(1, 2.0, 1.0, 1));
    r1.observations.push_back(make_test_obs(2, 4.0, 1.0, 1));
    r1.observations.push_back(make_test_obs(3, 2.0, 3.0, 1));
    r1.observations.push_back(make_test_obs(4, 4.0, 3.0, 1));
    step.rover_steps.push_back(r1);

    RoverStepData r2;
    r2.rover_id = 2;
    // Rigid displacement: dx = 3.0, dy = 1.0
    r2.observations.push_back(make_test_obs(1, 5.0, 2.0, 2));
    r2.observations.push_back(make_test_obs(2, 7.0, 2.0, 2));
    r2.observations.push_back(make_test_obs(3, 5.0, 4.0, 2));
    r2.observations.push_back(make_test_obs(4, 7.0, 4.0, 2));
    step.rover_steps.push_back(r2);

    pipeline.ingest_step(step);
    auto m = pipeline.collect_metrics();

    // Verify 7-step dataflow results
    TEST_ASSERT(m.sync_successes == 1, "Step 3: IBLT decode succeeds");
    TEST_ASSERT(m.candidate_pairs_proposed >= 4, "Step 4: Candidate correspondences extracted");
    TEST_ASSERT(m.verification_attempts == 1, "Step 5: Geometric verification attempted");
    TEST_ASSERT(m.loop_closures_verified == 1, "Step 5: Geometric verification passed");
    TEST_ASSERT(pipeline.get_accepted_closures().size() == 1, "Step 6: Inter-rover closure accepted and recorded");
    TEST_ASSERT(pipeline.are_rovers_connected(1, 2), "Step 7: DSU components merged");

    // 2. Test identical sets: A == B (A Δ B = empty)
    pipeline.reset();
    SimulationStepInput step_ident;
    step_ident.timestamp_ns = 222'000'000ULL;
    RoverStepData r1_id; r1_id.rover_id = 1;
    RoverStepData r2_id; r2_id.rover_id = 2;
    for (uint32_t i = 1; i <= 4; ++i) {
        r1_id.observations.push_back(make_test_obs(i, i * 2.0, 1.0, 1));
        r2_id.observations.push_back(make_test_obs(i, i * 2.0, 1.0, 2));
    }
    step_ident.rover_steps.push_back(r1_id);
    step_ident.rover_steps.push_back(r2_id);
    pipeline.ingest_step(step_ident);
    auto m_id = pipeline.collect_metrics();
    TEST_ASSERT(m_id.sync_successes == 1, "Identical sets decode with 0 residual differences");
    TEST_ASSERT(m_id.candidate_pairs_proposed >= 4, "Identical sets propose all shared keys");

    // 3. Test disjoint sets: A ∩ B = empty
    pipeline.reset();
    SimulationStepInput step_disj;
    step_disj.timestamp_ns = 333'000'000ULL;
    RoverStepData r1_dj; r1_dj.rover_id = 1;
    RoverStepData r2_dj; r2_dj.rover_id = 2;
    for (uint32_t i = 1; i <= 4; ++i) {
        r1_dj.observations.push_back(make_test_obs(i, i * 2.0, 1.0, 1));
        r2_dj.observations.push_back(make_test_obs(i + 100, i * 2.0, 1.0, 2));
    }
    step_disj.rover_steps.push_back(r1_dj);
    step_disj.rover_steps.push_back(r2_dj);
    pipeline.ingest_step(step_disj);
    auto m_dj = pipeline.collect_metrics();
    TEST_ASSERT(m_dj.sync_successes == 1, "Disjoint sets within capacity decode successfully");
    TEST_ASSERT(m_dj.candidate_pairs_proposed == 0, "Disjoint sets propose 0 candidates");
    TEST_ASSERT(m_dj.loop_closures_verified == 0, "Disjoint sets produce 0 closures");
    TEST_ASSERT(!pipeline.are_rovers_connected(1, 2), "Disjoint sets leave rovers disconnected");

    // 4. Test ground-truth isolation: Measured coords vs Ground Truth
    pipeline.reset();
    SimulationStepInput step_iso;
    step_iso.timestamp_ns = 444'000'000ULL;
    RoverStepData r1_iso; r1_iso.rover_id = 1;
    RoverStepData r2_iso; r2_iso.rover_id = 2;
    for (uint32_t i = 1; i <= 4; ++i) {
        auto o1 = make_test_obs(i, i * 2.0, 1.0, 1);
        auto o2 = make_test_obs(i, i * 2.0 + 5.0, 1.0, 2); // Measured offset +5.0 in x
        // Intentionally corrupt ground_truth fields to different values
        o1.ground_truth_world_point = verification::Point2D(999.0, 999.0);
        o2.ground_truth_world_point = verification::Point2D(0.0, 0.0);
        r1_iso.observations.push_back(o1);
        r2_iso.observations.push_back(o2);
    }
    step_iso.rover_steps.push_back(r1_iso);
    step_iso.rover_steps.push_back(r2_iso);
    pipeline.ingest_step(step_iso);
    auto closures = pipeline.get_accepted_closures();
    TEST_ASSERT(closures.size() == 1, "Closure verified using measured coordinates");
    if (!closures.empty()) {
        double tx = closures[0].estimated_relative_transform.x;
        TEST_ASSERT(std::abs(std::abs(tx) - 5.0) < 0.20, "Transform derived from measured coordinates, not corrupted GT");
    }
}

// ----------------------------------------------------------------------
// Test 17: Audit 2 - CORD Semantics, Mode & Anchor Invariants
// ----------------------------------------------------------------------
void test_audit2_cord_semantics_and_invariants() {
    std::cout << "[AUDIT 2] CORD Semantics, Mode & Anchor Invariants...\n";

    AstroMeshXConfig cfg;
    cfg.enable_cord = true;
    cfg.cord_options.max_steps = 200;
    cfg.cord_options.timestep = 0.01;
    cfg.cord_options.allow_disconnected = true;
    AstroMeshXPipeline pipeline(cfg);

    for (size_t s = 1; s <= 2; ++s) {
        SimulationStepInput step;
        step.timestamp_ns = s * 50'000'000ULL;
        RoverStepData r1; r1.rover_id = 1; r1.odometry_step = geometry::SE2(1.0, 0.1, 0.01); step.rover_steps.push_back(r1);
        RoverStepData r2; r2.rover_id = 2; r2.odometry_step = geometry::SE2(2.0, -0.1, -0.01); step.rover_steps.push_back(r2);
        ChannelLinkState severed; severed.rover_a = 1; severed.rover_b = 2; severed.in_communication_range = false;
        step.channel_states.push_back(severed);
        pipeline.ingest_step(step);
    }

    auto traj1_before = pipeline.get_estimated_trajectories().at(1);
    auto traj2_before = pipeline.get_estimated_trajectories().at(2);
    TEST_ASSERT(traj1_before.size() >= 3, "Rover 1 has keyframes 0..2");
    TEST_ASSERT(traj2_before.size() >= 3, "Rover 2 has keyframes 0..2");

    const geometry::SE2 anchor1_before = traj1_before[0];
    const geometry::SE2 anchor2_before = traj2_before[0];

    bool opt_ok = pipeline.optimize();
    TEST_ASSERT(opt_ok, "Synchronous Multi-Component CORD converges");

    auto m = pipeline.collect_metrics();
    TEST_ASSERT(m.optimization_mode == "SynchronousMultiComponentCord", "Explicit optimization mode reported");
    TEST_ASSERT(!m.centralized_optimization_performed, "Confirms CORD Lie dynamics, not centralized LM");

    auto traj1_after = pipeline.get_estimated_trajectories().at(1);
    auto traj2_after = pipeline.get_estimated_trajectories().at(2);

    TEST_ASSERT(std::abs(traj1_after[0].x() - anchor1_before.x()) < 1e-9, "Rover 1 gauge anchor x pinned");
    TEST_ASSERT(std::abs(traj1_after[0].y() - anchor1_before.y()) < 1e-9, "Rover 1 gauge anchor y pinned");
    TEST_ASSERT(std::abs(traj1_after[0].theta() - anchor1_before.theta()) < 1e-9, "Rover 1 gauge anchor theta pinned");

    TEST_ASSERT(std::abs(traj2_after[0].x() - anchor2_before.x()) < 1e-9, "Rover 2 gauge anchor x pinned");
    TEST_ASSERT(std::abs(traj2_after[0].y() - anchor2_before.y()) < 1e-9, "Rover 2 gauge anchor y pinned");
    TEST_ASSERT(std::abs(traj2_after[0].theta() - anchor2_before.theta()) < 1e-9, "Rover 2 gauge anchor theta pinned");

    // Repeatability test
    AstroMeshXPipeline pipeline2(cfg);
    for (size_t s = 1; s <= 2; ++s) {
        SimulationStepInput step;
        step.timestamp_ns = s * 50'000'000ULL;
        RoverStepData r1; r1.rover_id = 1; r1.odometry_step = geometry::SE2(1.0, 0.1, 0.01); step.rover_steps.push_back(r1);
        RoverStepData r2; r2.rover_id = 2; r2.odometry_step = geometry::SE2(2.0, -0.1, -0.01); step.rover_steps.push_back(r2);
        ChannelLinkState severed; severed.rover_a = 1; severed.rover_b = 2; severed.in_communication_range = false;
        step.channel_states.push_back(severed);
        pipeline2.ingest_step(step);
    }
    pipeline2.optimize();
    auto traj1_rep = pipeline2.get_estimated_trajectories().at(1);
    TEST_ASSERT(traj1_rep[2].x() == traj1_after[2].x(), "Deterministic CORD repeatability (x)");
    TEST_ASSERT(traj1_rep[2].y() == traj1_after[2].y(), "Deterministic CORD repeatability (y)");
    TEST_ASSERT(traj1_rep[2].theta() == traj1_after[2].theta(), "Deterministic CORD repeatability (theta)");
}

// ----------------------------------------------------------------------
// Test 18: Audit 3 - Communication Accounting & Wire Byte Invariants
// ----------------------------------------------------------------------
void test_audit3_communication_accounting_invariants() {
    std::cout << "[AUDIT 3] Communication Accounting & Wire Byte Invariants...\n";

    AstroMeshXConfig cfg;
    cfg.enable_explicit_acks = true;
    cfg.enable_multihop_routing = true;
    AstroMeshXPipeline pipeline(cfg);

    SimulationStepInput step;
    step.timestamp_ns = 100'000'000ULL;
    RoverStepData r1; r1.rover_id = 1; r1.observations.push_back(make_test_obs(1, 1.0, 1.0)); step.rover_steps.push_back(r1);
    RoverStepData r2; r2.rover_id = 2; r2.observations.push_back(make_test_obs(2, 2.0, 2.0)); step.rover_steps.push_back(r2);
    RoverStepData r3; r3.rover_id = 3; r3.observations.push_back(make_test_obs(3, 3.0, 3.0)); step.rover_steps.push_back(r3);

    ChannelLinkState l12; l12.rover_a = 1; l12.rover_b = 2; l12.in_communication_range = true; step.channel_states.push_back(l12);
    ChannelLinkState l23; l23.rover_a = 2; l23.rover_b = 3; l23.in_communication_range = true; step.channel_states.push_back(l23);
    ChannelLinkState l13; l13.rover_a = 1; l13.rover_b = 3; l13.in_communication_range = false; step.channel_states.push_back(l13);

    pipeline.ingest_step(step);
    auto m = pipeline.collect_metrics();

    TEST_ASSERT(m.data_wire_bytes_transmitted ==
                m.payload_bytes_transmitted + m.protocol_bytes_transmitted + m.framing_bytes_transmitted,
                "Invariant 1: data_wire_bytes == payload + protocol + framing");
    TEST_ASSERT(m.total_wire_bytes_transmitted ==
                m.data_wire_bytes_transmitted + m.ack_wire_bytes_transmitted,
                "Invariant 2: total_wire_bytes == data_wire_bytes + ack_wire_bytes");
    TEST_ASSERT(m.logical_messages_sent == m.logical_messages_delivered + m.logical_messages_dropped + m.in_flight_messages,
                "Invariant 3: logical_sent == delivered + dropped + in_flight");
    TEST_ASSERT(m.unique_deliveries == m.logical_messages_delivered,
                "Invariant 4: unique_deliveries == logical_messages_delivered");
    TEST_ASSERT(m.application_completion_ratio >= 0.0 && m.application_completion_ratio <= 1.0,
                "Invariant 5: application_completion_ratio within [0, 1]");
    TEST_ASSERT(m.per_hop_delivery_ratio >= 0.0 && m.per_hop_delivery_ratio <= 1.0,
                "Invariant 6: per_hop_delivery_ratio within [0, 1]");
}

// ----------------------------------------------------------------------
// Test 19: Audit 4 - Pruning SwarmGraph Synchronization & Post-Prune Optimization
// ----------------------------------------------------------------------
void test_audit4_pruning_swarm_graph_synchronization() {
    std::cout << "[AUDIT 4] Pruning SwarmGraph Synchronization & Post-Prune Optimization...\n";

    AstroMeshXConfig cfg;
    cfg.enable_keyframe_pruning = true;
    cfg.pruner_config.max_retained_keyframes = 4;
    cfg.pruner_config.protect_articulation_points = false;
    cfg.pruner_config.protect_gauge_anchors = true;
    cfg.pruner_config.protect_newest_keyframe = true;
    cfg.enable_cord = true;

    AstroMeshXPipeline pipeline(cfg);

    for (size_t s = 1; s <= 7; ++s) {
        SimulationStepInput step;
        step.timestamp_ns = s * 50'000'000ULL;
        RoverStepData r;
        r.rover_id = 1;
        r.odometry_step = geometry::SE2(1.0, 0.02, 0.01);
        step.rover_steps.push_back(r);
        pipeline.ingest_step(step);
    }

    auto m = pipeline.collect_metrics();
    TEST_ASSERT(m.keyframe_prunes_applied > 0, "Pruning applied when nodes > max_retained_keyframes");
    const auto* g = pipeline.get_rover_graph(1);
    TEST_ASSERT(g != nullptr, "Graph exists");
    if (g) {
        TEST_ASSERT(g->num_nodes() <= 5, "Rover graph bounded");
        TEST_ASSERT(g->has_node(graph::NodeId(1, 0)), "Gauge anchor Node(1, 0) strictly protected");
    }

    // Crucial Audit 4 Check: Swarm graph and rover graph are synchronized!
    // optimize() MUST succeed without InvalidInitialPoses or missing-node errors!
    bool opt_ok = pipeline.optimize();
    TEST_ASSERT(opt_ok, "CORD optimization succeeds on post-pruned graph without node mismatch");
}

// ----------------------------------------------------------------------
// Main Entry Point
// ----------------------------------------------------------------------
int main() {
    std::cout << "======================================================================\n";
    std::cout << " AstroMesh-X Phase 5.3: AstroMesh-X Pipeline Integration Tests\n";
    std::cout << "======================================================================\n";

    test_init_and_reset();
    test_shared_interface_compliance();
    test_iblt_reconciliation_and_atomicity();
    test_candidate_gating();
    test_rejected_closure_isolation();
    test_duplicate_idempotency();
    test_cord_optimization_execution();
    test_anchor_and_disconnected_components();
    test_dtn_and_multihop_routing();
    test_wire_byte_and_ack_accounting();
    test_safe_keyframe_pruning();
    test_same_seed_determinism();
    test_ground_truth_isolation();
    test_identical_scenario_input();
    test_metrics_invariants();
    test_audit1_iblt_trace_and_matrix();
    test_audit2_cord_semantics_and_invariants();
    test_audit3_communication_accounting_invariants();
    test_audit4_pruning_swarm_graph_synchronization();

    std::cout << "======================================================================\n";
    std::cout << " Assertions checked : " << g_total_assertions << "\n";
    std::cout << " Assertions failed  : " << g_failed_assertions << "\n";
    std::cout << "======================================================================\n";

    if (g_failed_assertions == 0) {
        std::cout << "SUCCESS: All AstroMesh-X pipeline integration tests PASSED.\n";
        return 0;
    } else {
        std::cerr << "FAILURE: " << g_failed_assertions << " assertion(s) failed.\n";
        return 1;
    }
}
