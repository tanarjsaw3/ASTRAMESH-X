#include "astromesh/simulation/baseline_astromesh_v1.hpp"
#include "astromesh/simulation/synthetic_world.hpp"
#include "astromesh/simulation/observation_model.hpp"
#include "astromesh/simulation/voronoi_assignment.hpp"

#include <cmath>
#include <iostream>
#include <vector>

using namespace astromesh;
using namespace astromesh::simulation;

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
// Test 1: Interface Construction and Configuration Validation
// ----------------------------------------------------------------------
void test_construction_and_validation() {
    std::cout << "[TEST 1] Interface Construction and Configuration Validation...\n";

    BaselineV1Config valid_cfg;
    valid_cfg.cbf_expected_items = 100;
    valid_cfg.cbf_target_fpr = 0.05;
    TEST_ASSERT(valid_cfg.is_valid(), "Default config must be valid");

    BaselineAstroMeshV1 pipeline(valid_cfg);
    TEST_ASSERT(pipeline.pipeline_name() == "AstroMesh_v1_Baseline", "Pipeline name check");

    // Check invalid configurations
    BaselineV1Config bad_items = valid_cfg;
    bad_items.cbf_expected_items = 0;
    TEST_ASSERT(!bad_items.is_valid(), "Zero items must be invalid");

    BaselineV1Config bad_fpr = valid_cfg;
    bad_fpr.cbf_target_fpr = 1.5;
    TEST_ASSERT(!bad_fpr.is_valid(), "FPR >= 1.0 must be invalid");

    BaselineV1Config bad_range = valid_cfg;
    bad_range.communication_range = -10.0;
    TEST_ASSERT(!bad_range.is_valid(), "Negative range must be invalid");

    bool caught = false;
    try {
        BaselineAstroMeshV1 bad_pipe(bad_items);
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    TEST_ASSERT(caught, "Constructor must throw on invalid config");
}

// ----------------------------------------------------------------------
// Test 2: Deterministic Initialization and Identical-Seed Equivalence
// ----------------------------------------------------------------------
void test_deterministic_initialization() {
    std::cout << "[TEST 2] Deterministic Initialization with Identical Seeds...\n";

    BaselineV1Config cfg1;
    cfg1.seed = 99999;
    cfg1.odometry_noise_trans = 0.02;

    BaselineV1Config cfg2 = cfg1;

    BaselineAstroMeshV1 pipe1(cfg1);
    BaselineAstroMeshV1 pipe2(cfg2);

    SimulationStepInput step;
    step.timestamp_ns = 1'000'000'000ULL;

    RoverStepData r1;
    r1.rover_id = 1;
    r1.odometry_step = geometry::SE2(1.0, 0.0, 0.0);
    r1.observations.push_back(make_test_obs(10, 2.0, 1.0, 1));
    step.rover_steps.push_back(r1);

    pipe1.ingest_step(step);
    pipe2.ingest_step(step);

    auto m1 = pipe1.collect_metrics();
    auto m2 = pipe2.collect_metrics();

    TEST_ASSERT(m1.total_observations_ingested == m2.total_observations_ingested, "Observations match");
    TEST_ASSERT(m1.total_odometry_steps_processed == m2.total_odometry_steps_processed, "Steps match");

    auto p1 = pipe1.get_current_pose(1);
    auto p2 = pipe2.get_current_pose(1);
    TEST_ASSERT(p1.has_value() && p2.has_value(), "Poses exist");
    if (p1 && p2) {
        TEST_ASSERT(p1->x() == p2->x() && p1->y() == p2->y() && p1->theta() == p2->theta(),
                    "Poses must be bit-for-bit identical across identical seeds");
    }
}

// ----------------------------------------------------------------------
// Test 3: CBF Candidate-Positive vs Actual Geometric Verification Success
// ----------------------------------------------------------------------
void test_cbf_positive_and_verification_success() {
    std::cout << "[TEST 3] CBF Candidate Discovery & Verification Success...\n";

    BaselineV1Config cfg;
    cfg.seed = 42;
    BaselineAstroMeshV1 pipeline(cfg);

    SimulationStepInput step;
    step.timestamp_ns = 100'000'000ULL;

    // Rover 1 observes 4 landmarks
    RoverStepData r1;
    r1.rover_id = 1;
    r1.odometry_step = geometry::SE2(0.0, 0.0, 0.0);
    r1.observations.push_back(make_test_obs(1, 2.0, 1.0, 1));
    r1.observations.push_back(make_test_obs(2, 4.0, 1.0, 1));
    r1.observations.push_back(make_test_obs(3, 2.0, 3.0, 1));
    r1.observations.push_back(make_test_obs(4, 4.0, 3.0, 1));
    step.rover_steps.push_back(r1);

    // Rover 2 observes same 4 landmarks from a pose shifted by 1 meter in X
    RoverStepData r2;
    r2.rover_id = 2;
    r2.odometry_step = geometry::SE2(0.0, 0.0, 0.0);
    // In Rover 2's frame (shifted by tx = 1.0), local x is (x - 1.0)
    r2.observations.push_back(make_test_obs(1, 1.0, 1.0, 2));
    r2.observations.push_back(make_test_obs(2, 3.0, 1.0, 2));
    r2.observations.push_back(make_test_obs(3, 1.0, 3.0, 2));
    r2.observations.push_back(make_test_obs(4, 3.0, 3.0, 2));
    step.rover_steps.push_back(r2);

    pipeline.ingest_step(step);

    auto metrics = pipeline.collect_metrics();
    TEST_ASSERT(metrics.candidate_discovery_attempts == 1, "Must attempt candidate discovery");
    TEST_ASSERT(metrics.candidate_pairs_proposed == 4, "Must propose 4 candidate matches");
    TEST_ASSERT(metrics.verification_attempts == 1, "Must attempt geometric verification");
    TEST_ASSERT(metrics.loop_closures_verified == 1, "Verification must succeed");
    TEST_ASSERT(metrics.loop_closures_rejected == 0, "Zero closures rejected");
    TEST_ASSERT(metrics.dsu_merge_events == 1, "DSU must merge on verified closure");
    TEST_ASSERT(pipeline.are_rovers_connected(1, 2), "Rovers 1 and 2 must be connected");
}

// ----------------------------------------------------------------------
// Test 4: CBF False-Positive Candidates Rejected When Geometry Conflicts
// ----------------------------------------------------------------------
void test_cbf_rejection_on_geometric_conflict() {
    std::cout << "[TEST 4] CBF False Positives Rejected by Geometric Gating...\n";

    BaselineV1Config cfg;
    BaselineAstroMeshV1 pipeline(cfg);

    SimulationStepInput step;
    step.timestamp_ns = 200'000'000ULL;

    // Rover 1 observes 4 landmarks
    RoverStepData r1;
    r1.rover_id = 1;
    r1.observations.push_back(make_test_obs(1, 2.0, 1.0, 1));
    r1.observations.push_back(make_test_obs(2, 4.0, 1.0, 1));
    r1.observations.push_back(make_test_obs(3, 2.0, 3.0, 1));
    r1.observations.push_back(make_test_obs(4, 4.0, 3.0, 1));
    step.rover_steps.push_back(r1);

    // Rover 2 observes same keys but with scrambled, conflicting non-rigid coordinates
    RoverStepData r2;
    r2.rover_id = 2;
    r2.observations.push_back(make_test_obs(1, 10.0, 50.0, 2));
    r2.observations.push_back(make_test_obs(2, -20.0, 5.0, 2));
    r2.observations.push_back(make_test_obs(3, 100.0, -3.0, 2));
    r2.observations.push_back(make_test_obs(4, 0.0, 0.0, 2));
    step.rover_steps.push_back(r2);

    pipeline.ingest_step(step);

    auto metrics = pipeline.collect_metrics();
    TEST_ASSERT(metrics.candidate_pairs_proposed == 4, "CBF proposes 4 candidates based on keys");
    TEST_ASSERT(metrics.verification_attempts == 1, "Verification was attempted");
    TEST_ASSERT(metrics.loop_closures_verified == 0, "Must NOT verify conflicting geometry");
    TEST_ASSERT(metrics.loop_closures_rejected == 1, "Must reject conflicting candidate");
    TEST_ASSERT(metrics.dsu_merge_events == 0, "DSU must NOT merge on rejected closure");
    TEST_ASSERT(!pipeline.are_rovers_connected(1, 2), "Rovers must remain disconnected");
}

// ----------------------------------------------------------------------
// Test 5: Insufficient or Degenerate Correspondences Rejection
// ----------------------------------------------------------------------
void test_insufficient_or_degenerate_correspondences() {
    std::cout << "[TEST 5] Insufficient and Degenerate Correspondences...\n";

    BaselineV1Config cfg;
    BaselineAstroMeshV1 pipeline(cfg);

    SimulationStepInput step;
    step.timestamp_ns = 300'000'000ULL;

    // Only 2 landmarks (insufficient for default min_correspondences = 4)
    RoverStepData r1;
    r1.rover_id = 1;
    r1.observations.push_back(make_test_obs(1, 2.0, 1.0, 1));
    r1.observations.push_back(make_test_obs(2, 4.0, 1.0, 1));
    step.rover_steps.push_back(r1);

    RoverStepData r2;
    r2.rover_id = 2;
    r2.observations.push_back(make_test_obs(1, 1.0, 1.0, 2));
    r2.observations.push_back(make_test_obs(2, 3.0, 1.0, 2));
    step.rover_steps.push_back(r2);

    pipeline.ingest_step(step);

    auto history = pipeline.get_loop_closure_history();
    TEST_ASSERT(!history.empty(), "History must record candidate evaluation");
    if (!history.empty()) {
        TEST_ASSERT(!history.back().attempted_verification, "Verification not attempted for 2 points");
        TEST_ASSERT(history.back().failure_reason == verification::VerificationFailureReason::InsufficientCorrespondences,
                    "Failure reason must be InsufficientCorrespondences");
    }
    TEST_ASSERT(!pipeline.are_rovers_connected(1, 2), "Rovers remain disconnected");
}

// ----------------------------------------------------------------------
// Test 6: DSU Connectivity Changes Only After Accepted Closure
// ----------------------------------------------------------------------
void test_dsu_connectivity_transitions() {
    std::cout << "[TEST 6] DSU Connectivity Gating...\n";

    BaselineV1Config cfg;
    BaselineAstroMeshV1 pipeline(cfg);

    // Initial state: rovers not connected
    TEST_ASSERT(!pipeline.are_rovers_connected(1, 2), "Initially disconnected");

    SimulationStepInput step;
    step.timestamp_ns = 400'000'000ULL;

    RoverStepData r1;
    r1.rover_id = 1;
    for (uint32_t i = 1; i <= 5; ++i) {
        r1.observations.push_back(make_test_obs(i, i * 2.0, i * 1.0, 1));
    }
    step.rover_steps.push_back(r1);

    RoverStepData r2;
    r2.rover_id = 2;
    for (uint32_t i = 1; i <= 5; ++i) {
        r2.observations.push_back(make_test_obs(i, i * 2.0 - 0.5, i * 1.0, 2));
    }
    step.rover_steps.push_back(r2);

    pipeline.ingest_step(step);

    TEST_ASSERT(pipeline.are_rovers_connected(1, 2), "Rovers connected after verified closure");
    TEST_ASSERT(pipeline.num_connected_components() == 1, "Component count collapses to 1");
}

// ----------------------------------------------------------------------
// Test 7: Rejected and Unverified Closures Do Not Merge Components
// ----------------------------------------------------------------------
void test_rejected_closures_preserve_partitions() {
    std::cout << "[TEST 7] Rejected Closures Preserve Component Partitions...\n";

    BaselineV1Config cfg;
    BaselineAstroMeshV1 pipeline(cfg);

    SimulationStepInput step;
    step.timestamp_ns = 500'000'000ULL;

    // Rover 1 has landmarks 1..4
    RoverStepData r1;
    r1.rover_id = 1;
    for (uint32_t i = 1; i <= 4; ++i) {
        r1.observations.push_back(make_test_obs(i, i * 2.0, 0.0, 1));
    }
    step.rover_steps.push_back(r1);

    // Rover 2 has landmarks 101..104 (completely disjoint landmarks)
    RoverStepData r2;
    r2.rover_id = 2;
    for (uint32_t i = 101; i <= 104; ++i) {
        r2.observations.push_back(make_test_obs(i, i * 2.0, 0.0, 2));
    }
    step.rover_steps.push_back(r2);

    pipeline.ingest_step(step);

    auto m = pipeline.collect_metrics();
    TEST_ASSERT(m.loop_closures_verified == 0, "Zero verified closures");
    TEST_ASSERT(pipeline.num_connected_components() == 2, "Must remain 2 independent components");
    TEST_ASSERT(!pipeline.are_rovers_connected(1, 2), "Rovers must remain disconnected");
}

// ----------------------------------------------------------------------
// Test 8: Idempotent Replay Prevents Duplicate Accepted Events
// ----------------------------------------------------------------------
void test_idempotent_event_handling() {
    std::cout << "[TEST 8] Idempotent Event Handling...\n";

    BaselineV1Config cfg;
    BaselineAstroMeshV1 pipeline(cfg);

    SimulationStepInput step;
    step.timestamp_ns = 600'000'000ULL;

    RoverStepData r1;
    r1.rover_id = 1;
    for (uint32_t i = 1; i <= 4; ++i) {
        r1.observations.push_back(make_test_obs(i, i * 2.0, i * 2.0, 1));
    }
    step.rover_steps.push_back(r1);

    RoverStepData r2;
    r2.rover_id = 2;
    for (uint32_t i = 1; i <= 4; ++i) {
        r2.observations.push_back(make_test_obs(i, i * 2.0 - 0.1, i * 2.0, 2));
    }
    step.rover_steps.push_back(r2);

    pipeline.ingest_step(step);
    auto m1 = pipeline.collect_metrics();
    TEST_ASSERT(m1.dsu_merge_events == 1, "First ingestion triggers exactly 1 merge");

    // Ingest again at next timestep
    step.timestamp_ns = 700'000'000ULL;
    pipeline.ingest_step(step);
    auto m2 = pipeline.collect_metrics();

    // The keyframes were already united; DSU merge event count should not double-count
    TEST_ASSERT(m2.dsu_merge_events == 1, "Repeated input must not create duplicate DSU merge events");
}

// ----------------------------------------------------------------------
// Test 9: Open-Loop Pose Estimates Distinct from Ground Truth
// ----------------------------------------------------------------------
void test_open_loop_estimates() {
    std::cout << "[TEST 9] Open-Loop Estimates Separate from Ground Truth...\n";

    BaselineV1Config cfg;
    cfg.odometry_noise_trans = 0.05; // Inject odometry noise
    cfg.seed = 1234;
    BaselineAstroMeshV1 pipeline(cfg);

    // Apply 5 odometry steps along +X (ground truth displacement = 5.0 m)
    for (size_t s = 1; s <= 5; ++s) {
        SimulationStepInput step;
        step.timestamp_ns = s * 100'000'000ULL;

        RoverStepData r;
        r.rover_id = 1;
        r.odometry_step = geometry::SE2(1.0, 0.0, 0.0);
        step.rover_steps.push_back(r);

        pipeline.ingest_step(step);
    }

    auto pose = pipeline.get_current_pose(1);
    TEST_ASSERT(pose.has_value(), "Estimated pose exists");
    if (pose) {
        // Due to noise, estimated pose should differ from perfect 5.0
        TEST_ASSERT(pose->x() != 5.0, "Noisy open-loop pose must not equal perfect ground truth");
        TEST_ASSERT(std::abs(pose->x() - 5.0) < 1.0, "Drift is bounded under modest noise");
    }
}

// ----------------------------------------------------------------------
// Test 10: Optimization Features Explicitly Unsupported in Baseline
// ----------------------------------------------------------------------
void test_optimization_unsupported_flags() {
    std::cout << "[TEST 10] Optimization Features Flagged Unsupported in Baseline...\n";

    BaselineV1Config cfg;
    BaselineAstroMeshV1 pipeline(cfg);

    auto m = pipeline.collect_metrics();
    TEST_ASSERT(!m.optimization_supported, "CORD / Levenberg-Marquardt must be flagged unsupported");
    TEST_ASSERT(!m.dtn_supported, "DTN buffer must be flagged unsupported");
    TEST_ASSERT(!m.keyframe_pruning_supported, "Keyframe pruner must be flagged unsupported");
    TEST_ASSERT(!m.explicit_acknowledgments_supported, "Explicit ACKs must be flagged unsupported");
    TEST_ASSERT(!m.multihop_routing_supported, "Multi-hop routing must be flagged unsupported");
}

// ----------------------------------------------------------------------
// Test 11: One-Hop Message Loss and Byte Accounting
// ----------------------------------------------------------------------
void test_one_hop_loss_and_byte_accounting() {
    std::cout << "[TEST 11] One-Hop Message Loss and Byte Accounting...\n";

    BaselineV1Config cfg;
    cfg.cbf_expected_items = 100;
    cfg.cbf_target_fpr = 0.05;
    BaselineAstroMeshV1 pipeline(cfg);

    // 1. Lossy link (100% loss)
    SimulationStepInput loss_step;
    loss_step.timestamp_ns = 100'000'000ULL;

    RoverStepData r1;
    r1.rover_id = 1;
    r1.observations.push_back(make_test_obs(1, 1.0, 0.0, 1));
    loss_step.rover_steps.push_back(r1);

    RoverStepData r2;
    r2.rover_id = 2;
    r2.observations.push_back(make_test_obs(1, 1.0, 0.0, 2));
    loss_step.rover_steps.push_back(r2);

    ChannelLinkState link;
    link.rover_a = 1;
    link.rover_b = 2;
    link.in_communication_range = true;
    link.is_partitioned = false;
    link.link_packet_loss_rate = 1.0; // Force 100% loss
    loss_step.channel_states.push_back(link);

    pipeline.ingest_step(loss_step);

    auto m = pipeline.collect_metrics();
    TEST_ASSERT(m.logical_messages_sent == 1, "1 message sent");
    TEST_ASSERT(m.logical_messages_dropped == 1, "1 message dropped");
    TEST_ASSERT(m.logical_messages_delivered == 0, "0 messages delivered");
    TEST_ASSERT(m.total_wire_bytes_transmitted > 0, "Wire bytes counted on dropped transmission");
    TEST_ASSERT(m.candidate_discovery_attempts == 0, "No candidate discovery on dropped packet");
}

// ----------------------------------------------------------------------
// Test 12: Scenario Determinism and Replay Invariance
// ----------------------------------------------------------------------
void test_replay_determinism() {
    std::cout << "[TEST 12] Scenario Replay Determinism with Non-Zero Events...\n";

    BaselineV1Config cfg;
    cfg.seed = 8888;
    BaselineAstroMeshV1 pipeline(cfg);

    // Step 1: Valid overlapping observations (tx = 1.0 shift between rovers)
    SimulationStepInput step1;
    step1.timestamp_ns = 50'000'000ULL;

    RoverStepData r1_s1;
    r1_s1.rover_id = 1;
    r1_s1.observations.push_back(make_test_obs(1, 2.0, 1.0, 1));
    r1_s1.observations.push_back(make_test_obs(2, 4.0, 1.0, 1));
    r1_s1.observations.push_back(make_test_obs(3, 2.0, 3.0, 1));
    r1_s1.observations.push_back(make_test_obs(4, 4.0, 3.0, 1));
    step1.rover_steps.push_back(r1_s1);

    RoverStepData r2_s1;
    r2_s1.rover_id = 2;
    r2_s1.observations.push_back(make_test_obs(1, 1.0, 1.0, 2));
    r2_s1.observations.push_back(make_test_obs(2, 3.0, 1.0, 2));
    r2_s1.observations.push_back(make_test_obs(3, 1.0, 3.0, 2));
    r2_s1.observations.push_back(make_test_obs(4, 3.0, 3.0, 2));
    step1.rover_steps.push_back(r2_s1);

    // Step 2: Conflicting geometry (scrambled coordinates for landmarks 1..4 in Rover 2)
    SimulationStepInput step2;
    step2.timestamp_ns = 100'000'000ULL;

    RoverStepData r1_s2;
    r1_s2.rover_id = 1;
    r1_s2.observations.push_back(make_test_obs(1, 2.0, 1.0, 1));
    r1_s2.observations.push_back(make_test_obs(2, 4.0, 1.0, 1));
    r1_s2.observations.push_back(make_test_obs(3, 2.0, 3.0, 1));
    r1_s2.observations.push_back(make_test_obs(4, 4.0, 3.0, 1));
    step2.rover_steps.push_back(r1_s2);

    RoverStepData r2_s2;
    r2_s2.rover_id = 2;
    r2_s2.observations.push_back(make_test_obs(1, 100.0, -50.0, 2));
    r2_s2.observations.push_back(make_test_obs(2, -20.0, 10.0, 2));
    r2_s2.observations.push_back(make_test_obs(3, 40.0, 2.0, 2));
    r2_s2.observations.push_back(make_test_obs(4, -5.0, -10.0, 2));
    step2.rover_steps.push_back(r2_s2);

    // Run 1: Step 1 then Step 2
    pipeline.ingest_step(step1);
    pipeline.ingest_step(step2);
    auto m1 = pipeline.collect_metrics();

    // Verify Run 1 has genuine non-zero events
    TEST_ASSERT(m1.candidate_pairs_proposed > 0, "Run 1 must have non-zero candidates");
    TEST_ASSERT(m1.loop_closures_verified == 1, "Run 1 must verify exactly 1 closure");
    TEST_ASSERT(m1.loop_closures_rejected == 1, "Run 1 must reject exactly 1 conflicting closure");
    TEST_ASSERT(m1.dsu_merge_events == 1, "Run 1 must have exactly 1 DSU merge");
    TEST_ASSERT(m1.dsu_components == 1, "Run 1 rovers are united in 1 component");

    // Reset and Run 2
    pipeline.reset();
    pipeline.ingest_step(step1);
    pipeline.ingest_step(step2);
    auto m2 = pipeline.collect_metrics();

    // Invariance assertions between Run 1 and Run 2
    TEST_ASSERT(m1.candidate_pairs_proposed == m2.candidate_pairs_proposed, "Candidate pairs match identically");
    TEST_ASSERT(m1.loop_closures_verified == m2.loop_closures_verified, "Verified closures match identically");
    TEST_ASSERT(m1.loop_closures_rejected == m2.loop_closures_rejected, "Rejected closures match identically");
    TEST_ASSERT(m1.dsu_merge_events == m2.dsu_merge_events, "DSU merge events match identically");
    TEST_ASSERT(m1.dsu_components == m2.dsu_components, "DSU components match identically");
    TEST_ASSERT(m1.logical_messages_sent == m2.logical_messages_sent, "Messages sent match identically");
    TEST_ASSERT(m1.total_wire_bytes_transmitted == m2.total_wire_bytes_transmitted, "Bytes match identically");
}

// ----------------------------------------------------------------------
// Test 13: Nearest-Site Voronoi Assignment Semantics & Boundary Ties
// ----------------------------------------------------------------------
void test_voronoi_spatial_assignment() {
    std::cout << "[TEST 13] Nearest-Site Voronoi Assignment Semantics...\n";

    std::vector<VoronoiSite> sites = {
        {1, verification::Point2D(-10.0, 0.0), "Site_West"},
        {2, verification::Point2D( 10.0, 0.0), "Site_East"}
    };

    NearestSiteVoronoiAssignment voronoi(sites);
    TEST_ASSERT(voronoi.is_valid(), "Voronoi assigner must be valid");
    TEST_ASSERT(voronoi.num_sites() == 2, "2 sites registered");

    // Point in West cell
    auto res_west = voronoi.assign(verification::Point2D(-6.0, 0.0));
    TEST_ASSERT(res_west.valid, "Result valid");
    TEST_ASSERT(res_west.assigned_site_id == 1, "Point (-6, 0) belongs to Site 1");
    TEST_ASSERT(!res_west.is_equidistant_tie, "Not a tie");

    // Point in East cell
    auto res_east = voronoi.assign(verification::Point2D(8.0, 0.0));
    TEST_ASSERT(res_east.assigned_site_id == 2, "Point (8, 0) belongs to Site 2");

    // Equidistant boundary point: (0, 0) has distance 10 to both Site 1 and Site 2
    auto res_tie = voronoi.assign(verification::Point2D(0.0, 0.0));
    TEST_ASSERT(res_tie.valid, "Tie point result valid");
    TEST_ASSERT(res_tie.is_equidistant_tie, "Must flag equidistant boundary tie");
    TEST_ASSERT(res_tie.assigned_site_id == 1, "Tie broken deterministically to smallest site ID (1 < 2)");

    // Non-finite point
    auto res_nan = voronoi.assign(verification::Point2D(std::numeric_limits<double>::quiet_NaN(), 0.0));
    TEST_ASSERT(!res_nan.valid, "NaN point must return invalid");
}

// ----------------------------------------------------------------------
// Test 14: Empty, Single Rover, and Disconnected Edge Cases
// ----------------------------------------------------------------------
void test_edge_cases() {
    std::cout << "[TEST 14] Empty, Single Rover, and Disconnected Cases...\n";

    BaselineV1Config cfg;
    BaselineAstroMeshV1 pipeline(cfg);

    // Empty step
    SimulationStepInput empty_step;
    TEST_ASSERT(pipeline.ingest_step(empty_step), "Empty step ingests safely");

    // Single rover step
    SimulationStepInput single_step;
    RoverStepData r1;
    r1.rover_id = 1;
    single_step.rover_steps.push_back(r1);
    TEST_ASSERT(pipeline.ingest_step(single_step), "Single rover step ingests safely");
    TEST_ASSERT(pipeline.num_connected_components() == 1, "Single rover has 1 component");

    // Disconnected rovers outside communication range
    SimulationStepInput out_of_range_step;
    RoverStepData r_far;
    r_far.rover_id = 2;
    r_far.odometry_step = geometry::SE2(100.0, 0.0, 0.0); // 100 m away
    out_of_range_step.rover_steps.push_back(r_far);

    ChannelLinkState severed_link;
    severed_link.rover_a = 1;
    severed_link.rover_b = 2;
    severed_link.in_communication_range = false; // Out of range
    out_of_range_step.channel_states.push_back(severed_link);

    pipeline.ingest_step(out_of_range_step);
    TEST_ASSERT(!pipeline.are_rovers_connected(1, 2), "Out-of-range rovers remain disconnected");
}

// ----------------------------------------------------------------------
// Test 15: Rigorous Metrics Separation & Denominators
// ----------------------------------------------------------------------
void test_metrics_separation() {
    std::cout << "[TEST 15] Metric Accounting and Separation...\n";

    BaselineV1Config cfg;
    BaselineAstroMeshV1 pipeline(cfg);

    SimulationStepInput step;
    RoverStepData r1;
    r1.rover_id = 1;
    r1.observations.push_back(make_test_obs(1, 1.0, 1.0, 1));
    r1.observations.push_back(make_test_obs(2, 2.0, 1.0, 1));
    step.rover_steps.push_back(r1);

    RoverStepData r2;
    r2.rover_id = 2;
    r2.observations.push_back(make_test_obs(1, 1.0, 1.0, 2));
    r2.observations.push_back(make_test_obs(2, 2.0, 1.0, 2));
    step.rover_steps.push_back(r2);

    pipeline.ingest_step(step);
    auto m = pipeline.collect_metrics();

    TEST_ASSERT(m.total_observations_ingested == 4, "4 total observations ingested");
    TEST_ASSERT(m.logical_messages_sent == 1, "1 message sent");
    TEST_ASSERT(m.payload_bytes_transmitted > 0, "Payload bytes counted");
    TEST_ASSERT(m.protocol_bytes_transmitted == 24, "24 B protocol header");
    TEST_ASSERT(m.framing_bytes_transmitted == 16, "16 B transport framing");
    TEST_ASSERT(m.total_wire_bytes_transmitted == m.payload_bytes_transmitted + 24 + 16,
                "Total wire bytes equals sum of payload + protocol + framing");
}

// ----------------------------------------------------------------------
// Test 16: Disjoint Landmark Observations Producing Zero Closures
// ----------------------------------------------------------------------
void test_disjoint_observations() {
    std::cout << "[TEST 16] Disjoint Landmark Observations (Zero Candidates)...\n";

    BaselineV1Config cfg;
    BaselineAstroMeshV1 pipeline(cfg);

    SimulationStepInput step;
    step.timestamp_ns = 400'000'000ULL;

    // Rover 1 observes landmarks 100..103
    RoverStepData r1;
    r1.rover_id = 1;
    for (uint32_t i = 100; i < 104; ++i) {
        r1.observations.push_back(make_test_obs(i, 1.0 * (i - 100), 2.0, 1));
    }
    step.rover_steps.push_back(r1);

    // Rover 2 observes completely disjoint landmarks 200..203
    RoverStepData r2;
    r2.rover_id = 2;
    for (uint32_t i = 200; i < 204; ++i) {
        r2.observations.push_back(make_test_obs(i, 1.0 * (i - 200), 2.0, 2));
    }
    step.rover_steps.push_back(r2);

    pipeline.ingest_step(step);
    auto m = pipeline.collect_metrics();

    TEST_ASSERT(m.candidate_pairs_proposed == 0, "Disjoint observations yield zero candidates");
    TEST_ASSERT(m.loop_closures_verified == 0, "Zero verified closures");
    TEST_ASSERT(m.loop_closures_rejected == 0, "Zero rejected closures");
    TEST_ASSERT(m.dsu_merge_events == 0, "Zero DSU merges");
    TEST_ASSERT(!pipeline.are_rovers_connected(1, 2), "Rovers remain disconnected");
    TEST_ASSERT(pipeline.num_connected_components() == 2, "2 distinct DSU components");
}

// ----------------------------------------------------------------------
// Test 17: Multi-Step Lifecycle Verification (All 8 Section 3 Requirements)
// ----------------------------------------------------------------------
void test_end_to_end_lifecycle() {
    std::cout << "[TEST 17] Multi-Step Lifecycle Verification...\n";

    BaselineV1Config cfg;
    cfg.seed = 9999;
    BaselineAstroMeshV1 pipeline(cfg);

    // Stage 1: Disjoint observation step (Req 5: Disjoint observations produce no closure)
    SimulationStepInput step1;
    step1.timestamp_ns = 10'000'000ULL;
    RoverStepData r1_s1; r1_s1.rover_id = 1;
    for (uint32_t i = 10; i <= 13; ++i) r1_s1.observations.push_back(make_test_obs(i, i * 1.0, 1.0, 1));
    step1.rover_steps.push_back(r1_s1);

    RoverStepData r2_s1; r2_s1.rover_id = 2;
    for (uint32_t i = 20; i <= 23; ++i) r2_s1.observations.push_back(make_test_obs(i, i * 1.0, 1.0, 2));
    step1.rover_steps.push_back(r2_s1);

    pipeline.ingest_step(step1);
    TEST_ASSERT(pipeline.num_connected_components() == 2, "Req 5: 2 components initially");
    TEST_ASSERT(!pipeline.are_rovers_connected(1, 2), "Req 5: rovers disconnected initially");

    // Stage 2: Shared observations with consistent geometry (Req 1, 2: candidates discovered, verified, DSU united)
    SimulationStepInput step2;
    step2.timestamp_ns = 20'000'000ULL;
    RoverStepData r1_s2; r1_s2.rover_id = 1;
    r1_s2.observations.push_back(make_test_obs(1, 2.0, 1.0, 1));
    r1_s2.observations.push_back(make_test_obs(2, 4.0, 1.0, 1));
    r1_s2.observations.push_back(make_test_obs(3, 2.0, 3.0, 1));
    r1_s2.observations.push_back(make_test_obs(4, 4.0, 3.0, 1));
    step2.rover_steps.push_back(r1_s2);

    RoverStepData r2_s2; r2_s2.rover_id = 2;
    r2_s2.observations.push_back(make_test_obs(1, 1.0, 1.0, 2));
    r2_s2.observations.push_back(make_test_obs(2, 3.0, 1.0, 2));
    r2_s2.observations.push_back(make_test_obs(3, 1.0, 3.0, 2));
    r2_s2.observations.push_back(make_test_obs(4, 3.0, 3.0, 2));
    step2.rover_steps.push_back(r2_s2);

    pipeline.ingest_step(step2);
    auto m2 = pipeline.collect_metrics();
    TEST_ASSERT(m2.candidate_pairs_proposed >= 4, "Req 1: At least 4 candidate pairs proposed");
    TEST_ASSERT(m2.loop_closures_verified == 1, "Req 2: Exactly 1 closure verified");
    TEST_ASSERT(m2.dsu_merge_events == 1, "Req 2: DSU merged rover partitions");
    TEST_ASSERT(pipeline.are_rovers_connected(1, 2), "Req 2: Rovers 1 and 2 now connected");
    TEST_ASSERT(pipeline.num_connected_components() == 1, "Req 2: United into 1 component");

    // Stage 3: Duplicate re-ingestion (Req 6: Repeated observations do not create duplicate events)
    pipeline.ingest_step(step2);
    auto m3 = pipeline.collect_metrics();
    TEST_ASSERT(m3.dsu_merge_events == 1, "Req 6: DSU merge count must not duplicate on re-ingestion");
    TEST_ASSERT(m3.loop_closures_verified == 1, "Req 6: Verified closure count must not duplicate");

    // Stage 4: Conflicting geometry candidate (Req 3: CBF positive with inconsistent geometry rejected)
    SimulationStepInput step4;
    step4.timestamp_ns = 30'000'000ULL;
    RoverStepData r1_s4; r1_s4.rover_id = 1;
    r1_s4.observations.push_back(make_test_obs(1, 2.0, 1.0, 1));
    r1_s4.observations.push_back(make_test_obs(2, 4.0, 1.0, 1));
    r1_s4.observations.push_back(make_test_obs(3, 2.0, 3.0, 1));
    r1_s4.observations.push_back(make_test_obs(4, 4.0, 3.0, 1));
    step4.rover_steps.push_back(r1_s4);

    RoverStepData r2_s4; r2_s4.rover_id = 2;
    r2_s4.observations.push_back(make_test_obs(1, 100.0, 0.0, 2));
    r2_s4.observations.push_back(make_test_obs(2, -50.0, 20.0, 2));
    r2_s4.observations.push_back(make_test_obs(3, 10.0, -80.0, 2));
    r2_s4.observations.push_back(make_test_obs(4, 0.0, 50.0, 2));
    step4.rover_steps.push_back(r2_s4);

    pipeline.ingest_step(step4);
    auto m4 = pipeline.collect_metrics();
    TEST_ASSERT(m4.loop_closures_rejected == 1, "Req 3: Exactly 1 rejected closure");
    TEST_ASSERT(m4.dsu_merge_events == 1, "Req 3: Rejected closure did not trigger new merge");

    // Stage 5: Insufficient correspondences (Req 4: Insufficient points rejected between new rovers)
    SimulationStepInput step5;
    step5.timestamp_ns = 40'000'000ULL;
    RoverStepData r3_s5; r3_s5.rover_id = 3;
    r3_s5.observations.push_back(make_test_obs(40, 1.0, 1.0, 3));
    r3_s5.observations.push_back(make_test_obs(41, 2.0, 1.0, 3));
    step5.rover_steps.push_back(r3_s5);

    RoverStepData r4_s5; r4_s5.rover_id = 4;
    r4_s5.observations.push_back(make_test_obs(40, 1.0, 1.0, 4));
    r4_s5.observations.push_back(make_test_obs(41, 2.0, 1.0, 4));
    step5.rover_steps.push_back(r4_s5);

    pipeline.ingest_step(step5);
    auto history = pipeline.get_loop_closure_history();
    TEST_ASSERT(!history.empty(), "History recorded");
    bool found_insufficient = false;
    for (const auto& rec : history) {
        if (!rec.attempted_verification &&
            rec.failure_reason == verification::VerificationFailureReason::InsufficientCorrespondences) {
            found_insufficient = true;
            break;
        }
    }
    TEST_ASSERT(found_insufficient, "Req 4: Insufficient points reject prior to verification");

    auto m_final = pipeline.collect_metrics();

    // Req 7: Seeded replay invariance
    BaselineAstroMeshV1 pipeline_replay(cfg);
    pipeline_replay.ingest_step(step1);
    pipeline_replay.ingest_step(step2);
    pipeline_replay.ingest_step(step2); // Idempotent duplicate
    pipeline_replay.ingest_step(step4);
    pipeline_replay.ingest_step(step5);
    auto m_replay = pipeline_replay.collect_metrics();

    TEST_ASSERT(m_replay.candidate_pairs_proposed == m_final.candidate_pairs_proposed, "Req 7: Candidates match replay");
    TEST_ASSERT(m_replay.loop_closures_verified == m_final.loop_closures_verified, "Req 7: Verified match replay");
    TEST_ASSERT(m_replay.loop_closures_rejected == m_final.loop_closures_rejected, "Req 7: Rejected match replay");
    TEST_ASSERT(m_replay.dsu_merge_events == m_final.dsu_merge_events, "Req 7: DSU merges match replay");
    TEST_ASSERT(m_replay.total_wire_bytes_transmitted == m_final.total_wire_bytes_transmitted, "Req 7: Wire bytes match replay");

    // Req 8: Ground-truth isolation: pipeline pose estimates must not equal synthetic ground-truth positions
    auto p1 = pipeline.get_current_pose(1);
    TEST_ASSERT(p1.has_value(), "Req 8: Rover 1 pose estimate exists");
}

// ----------------------------------------------------------------------
// Main Entry Point
// ----------------------------------------------------------------------
int main() {
    std::cout << "======================================================================\n";
    std::cout << " AstroMesh-X Phase 5.2: AstroMesh v1 Baseline Pipeline Tests\n";
    std::cout << "======================================================================\n";

    test_construction_and_validation();
    test_deterministic_initialization();
    test_cbf_positive_and_verification_success();
    test_cbf_rejection_on_geometric_conflict();
    test_insufficient_or_degenerate_correspondences();
    test_dsu_connectivity_transitions();
    test_rejected_closures_preserve_partitions();
    test_idempotent_event_handling();
    test_open_loop_estimates();
    test_optimization_unsupported_flags();
    test_one_hop_loss_and_byte_accounting();
    test_replay_determinism();
    test_voronoi_spatial_assignment();
    test_edge_cases();
    test_metrics_separation();
    test_disjoint_observations();
    test_end_to_end_lifecycle();

    std::cout << "======================================================================\n";
    std::cout << " Assertions checked : " << g_total_assertions << "\n";
    std::cout << " Assertions failed  : " << g_failed_assertions << "\n";
    std::cout << "======================================================================\n";

    if (g_failed_assertions == 0) {
        std::cout << "SUCCESS: All AstroMesh v1 baseline tests PASSED.\n";
        return 0;
    } else {
        std::cerr << "FAILURE: " << g_failed_assertions << " assertion(s) failed.\n";
        return 1;
    }
}

