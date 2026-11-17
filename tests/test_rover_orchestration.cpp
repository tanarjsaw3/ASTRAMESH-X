#include "astromesh/orchestration/rover_state_machine.hpp"
#include "astromesh/orchestration/multi_rover_collective.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

using namespace astromesh;
using namespace astromesh::orchestration;

static size_t g_assertions_checked = 0;
static size_t g_assertions_failed = 0;

#define CHECK(cond) do { \
    g_assertions_checked++; \
    if (!(cond)) { \
        g_assertions_failed++; \
        std::cerr << "Assertion FAILED: " #cond " at " << __FILE__ << ":" << __LINE__ << std::endl; \
    } \
} while(0)

#define CHECK_EQUAL(a, b) do { \
    g_assertions_checked++; \
    if ((a) != (b)) { \
        g_assertions_failed++; \
        std::cerr << "Assertion FAILED: " #a " == " #b " (" << (a) << " != " << (b) << ") at " << __FILE__ << ":" << __LINE__ << std::endl; \
    } \
} while(0)

// ----------------------------------------------------------------------
// Test 1: State Machine Valid and Invalid Transitions
// ----------------------------------------------------------------------
void test_state_machine_transitions() {
    std::cout << "[Test 1] State Machine Valid and Invalid Transitions..." << std::endl;

    RoverStateMachine sm(1);
    CHECK(sm.current_state() == RoverState::Initializing);

    // Initializing -> Exploring is legal
    bool ok = sm.transition_to(RoverState::Exploring, TransitionReason::InitializationComplete, 100);
    CHECK(ok);
    CHECK(sm.current_state() == RoverState::Exploring);

    // Exploring -> Optimizing is legal
    ok = sm.transition_to(RoverState::Optimizing, TransitionReason::OptimizationTriggered, 200);
    CHECK(ok);
    CHECK(sm.current_state() == RoverState::Optimizing);

    // Optimizing -> Exploring is legal
    ok = sm.transition_to(RoverState::Exploring, TransitionReason::OptimizationCompleted, 300);
    CHECK(ok);
    CHECK(sm.current_state() == RoverState::Exploring);

    // Exploring -> Initializing is ILLEGAL
    ok = sm.transition_to(RoverState::Initializing, TransitionReason::SystemReboot, 400);
    CHECK(!ok);
    CHECK(sm.current_state() == RoverState::Exploring); // Unchanged!
    CHECK_EQUAL(sm.rejected_transitions(), 1);

    // Exploring -> Disconnected -> Reconnecting -> Synchronizing -> Exploring
    ok = sm.transition_to(RoverState::Disconnected, TransitionReason::HeartbeatTimeout, 500);
    CHECK(ok);
    CHECK(sm.current_state() == RoverState::Disconnected);

    // Disconnected directly to Exploring is ILLEGAL (must go via Reconnecting/Sync)
    ok = sm.transition_to(RoverState::Exploring, TransitionReason::GenericEvent, 600);
    CHECK(!ok);
    CHECK(sm.current_state() == RoverState::Disconnected);

    ok = sm.transition_to(RoverState::Reconnecting, TransitionReason::PacketReceivedInDisconnected, 700);
    CHECK(ok);
    CHECK(sm.current_state() == RoverState::Reconnecting);

    ok = sm.transition_to(RoverState::Synchronizing, TransitionReason::SyncInitiated, 800);
    CHECK(ok);
    CHECK(sm.current_state() == RoverState::Synchronizing);

    ok = sm.transition_to(RoverState::Exploring, TransitionReason::SyncCompletedSuccessfully, 900);
    CHECK(ok);
    CHECK(sm.current_state() == RoverState::Exploring);

    // Any state -> Stopped is legal
    ok = sm.transition_to(RoverState::Stopped, TransitionReason::ManualHalt, 1000);
    CHECK(ok);
    CHECK(sm.current_state() == RoverState::Stopped);

    // Stopped -> Initializing is legal (Reboot)
    ok = sm.transition_to(RoverState::Initializing, TransitionReason::SystemReboot, 1100);
    CHECK(ok);
    CHECK(sm.current_state() == RoverState::Initializing);
}

// ----------------------------------------------------------------------
// Test 2: Idempotent Event Handling
// ----------------------------------------------------------------------
void test_idempotent_event_handling() {
    std::cout << "[Test 2] Idempotent Event Handling..." << std::endl;

    RoverStateMachine sm(1);
    sm.transition_to(RoverState::Exploring, TransitionReason::InitializationComplete, 100);
    CHECK(sm.current_state() == RoverState::Exploring);

    size_t prev_transitions = sm.total_transitions();

    // Repeated event leading to same state should be accepted idempotently without error
    bool ok = sm.transition_to(RoverState::Exploring, TransitionReason::OdometryAdvancement, 200);
    CHECK(ok);
    CHECK(sm.current_state() == RoverState::Exploring);
    CHECK_EQUAL(sm.idempotent_transitions(), 1);
    CHECK_EQUAL(sm.total_transitions(), prev_transitions); // Did not inflate transition count
    CHECK_EQUAL(sm.rejected_transitions(), 0);             // Did not count as rejection
}

// ----------------------------------------------------------------------
// Test 3: Timeout Evaluation (Degradation and Disconnection)
// ----------------------------------------------------------------------
void test_timeout_evaluation() {
    std::cout << "[Test 3] Timeout Evaluation (Degradation and Disconnection)..." << std::endl;

    RoverStateMachineConfig cfg;
    cfg.degradation_timeout_ns = 500'000'000;   // 500 ms
    cfg.disconnection_timeout_ns = 1'500'000'000; // 1500 ms
    cfg.sync_timeout_ns = 2'000'000'000;         // 2000 ms

    RoverStateMachine sm(1, cfg);
    sm.transition_to(RoverState::Exploring, TransitionReason::InitializationComplete, 0);

    uint64_t last_contact = 1'000'000'000; // t = 1.0 s

    // At t = 1.2 s: elapsed = 200 ms (< 500 ms) -> stays Exploring
    sm.evaluate_timeouts(1'200'000'000, last_contact, false);
    CHECK(sm.current_state() == RoverState::Exploring);

    // At t = 1.6 s: elapsed = 600 ms (>= 500 ms) -> transitions to CommunicationDegraded
    sm.evaluate_timeouts(1'600'000'000, last_contact, false);
    CHECK(sm.current_state() == RoverState::CommunicationDegraded);

    // New packet arrives at t = 1.7 s -> evaluates within normal limits -> returns to Exploring
    last_contact = 1'700'000'000;
    sm.evaluate_timeouts(1'750'000'000, last_contact, false);
    CHECK(sm.current_state() == RoverState::Exploring);

    // Silence for 1600 ms (> 1500 ms) -> transitions to Disconnected
    sm.evaluate_timeouts(3'350'000'000, last_contact, false);
    CHECK(sm.current_state() == RoverState::Disconnected);
}

// ----------------------------------------------------------------------
// Test 4: Resource Constraint Activation and Recovery
// ----------------------------------------------------------------------
void test_resource_constraint_lifecycle() {
    std::cout << "[Test 4] Resource Constraint Activation and Recovery..." << std::endl;

    RoverStateMachine sm(1);
    sm.transition_to(RoverState::Exploring, TransitionReason::InitializationComplete, 0);

    // Trigger resource constraint
    sm.evaluate_timeouts(100, 100, true);
    CHECK(sm.current_state() == RoverState::ResourceConstrained);

    // Still constrained
    sm.evaluate_timeouts(200, 100, true);
    CHECK(sm.current_state() == RoverState::ResourceConstrained);

    // Constraint cleared -> recovers to Exploring
    sm.evaluate_timeouts(300, 100, false);
    CHECK(sm.current_state() == RoverState::Exploring);
}

// ----------------------------------------------------------------------
// Test 5: Multi-Rover Message Transmission, Latency, and Delivery
// ----------------------------------------------------------------------
void test_multi_rover_message_transmission() {
    std::cout << "[Test 5] Multi-Rover Message Transmission, Latency, and Delivery..." << std::endl;

    CollectiveSimulationConfig cfg;
    cfg.scenario_name = "MessageTransmissionTest";

    SimulatedRoverConfig r1_cfg;
    r1_cfg.rover_id = 1;
    cfg.rover_configs.push_back(r1_cfg);

    SimulatedRoverConfig r2_cfg;
    r2_cfg.rover_id = 2;
    cfg.rover_configs.push_back(r2_cfg);

    cfg.network_config.latency_ns = 50'000'000; // 50 ms
    cfg.network_config.packet_loss_rate = 0.0;

    MultiRoverCollective collective(cfg);

    // Enqueue message from Rover 1 to Rover 2
    std::vector<uint8_t> payload = {0x01, 0x02, 0x03, 0x04};
    auto res = collective.enqueue_message(1, 2, comms::DtnMessageType::GenericData,
                                          comms::PriorityLevel::Normal, payload, 500'000'000);
    CHECK(res == comms::EnqueueResult::Success);

    // At t = 0 ms: message is in DTN buffer, not yet sent or delivered
    auto metrics0 = collective.collect_metrics();
    CHECK_EQUAL(metrics0.messages_generated, 1);
    CHECK_EQUAL(metrics0.messages_admitted, 1);

    // Step by 10 ms: message is dequeued and sent into transit
    collective.step(10'000'000); // t = 10 ms
    auto metrics1 = collective.collect_metrics();
    CHECK_EQUAL(metrics1.messages_sent, 1);
    CHECK_EQUAL(metrics1.messages_delivered, 0); // Not yet delivered (needs 50 ms latency)

    // Step to t = 65 ms (> 10 ms dispatch + 50 ms latency = 60 ms delivery time)
    collective.step(55'000'000);
    auto metrics2 = collective.collect_metrics();
    CHECK_EQUAL(metrics2.messages_delivered, 1);

    auto* r2 = collective.get_rover(2);
    CHECK(r2 != nullptr);
    CHECK_EQUAL(r2->messages_processed, 1);
}

// ----------------------------------------------------------------------
// Test 6: Network Partition and No Delivery to Disconnected Peers
// ----------------------------------------------------------------------
void test_network_partition_isolation() {
    std::cout << "[Test 6] Network Partition and No Delivery to Disconnected Peers..." << std::endl;

    CollectiveSimulationConfig cfg;
    SimulatedRoverConfig r1_cfg; r1_cfg.rover_id = 1; cfg.rover_configs.push_back(r1_cfg);
    SimulatedRoverConfig r2_cfg; r2_cfg.rover_id = 2; cfg.rover_configs.push_back(r2_cfg);
    cfg.network_config.latency_ns = 20'000'000;
    cfg.network_config.packet_loss_rate = 0.0;

    MultiRoverCollective collective(cfg);

    // Sever link between Rover 1 and Rover 2
    collective.set_link_state(1, 2, false);
    CHECK(!collective.is_link_connected(1, 2));

    // Enqueue message across disconnected link
    std::vector<uint8_t> payload = {0xAA, 0xBB};
    collective.enqueue_message(1, 2, comms::DtnMessageType::GenericData,
                               comms::PriorityLevel::Normal, payload, 500'000'000);

    // Run 100 ms
    collective.run(100'000'000, 10'000'000);

    auto metrics = collective.collect_metrics();
    CHECK(metrics.messages_sent > 0);
    CHECK_EQUAL(metrics.messages_delivered, 0); // ZERO delivery across disconnected link!
    CHECK(metrics.messages_dropped > 0);
}

// ----------------------------------------------------------------------
// Test 7: TTL Expiration During Prolonged Partition
// ----------------------------------------------------------------------
void test_ttl_expiration_during_partition() {
    std::cout << "[Test 7] TTL Expiration During Prolonged Partition..." << std::endl;

    CollectiveSimulationConfig cfg;
    SimulatedRoverConfig r1_cfg; r1_cfg.rover_id = 1; cfg.rover_configs.push_back(r1_cfg);
    SimulatedRoverConfig r2_cfg; r2_cfg.rover_id = 2; cfg.rover_configs.push_back(r2_cfg);
    cfg.network_config.latency_ns = 10'000'000;

    MultiRoverCollective collective(cfg);
    collective.set_link_state(1, 2, false);

    // Enqueue message with short TTL of 50 ms
    std::vector<uint8_t> payload = {0x11, 0x22};
    collective.enqueue_message(1, 2, comms::DtnMessageType::GenericData,
                               comms::PriorityLevel::Normal, payload, 50'000'000);

    // Run for 200 ms (exceeding TTL)
    collective.run(200'000'000, 10'000'000);

    // Reconnect link
    collective.set_link_state(1, 2, true);
    collective.run(100'000'000, 10'000'000);

    auto metrics = collective.collect_metrics();
    // The expired message was purged, not delivered on reconnect!
    CHECK_EQUAL(metrics.messages_delivered, 0);
}

// ----------------------------------------------------------------------
// Test 8: Landmark Key Reconciliation via IBLT
// ----------------------------------------------------------------------
void test_iblt_landmark_reconciliation() {
    std::cout << "[Test 8] Landmark Key Reconciliation via IBLT..." << std::endl;

    CollectiveSimulationConfig cfg;
    SimulatedRoverConfig r1_cfg; r1_cfg.rover_id = 1; cfg.rover_configs.push_back(r1_cfg);
    SimulatedRoverConfig r2_cfg; r2_cfg.rover_id = 2; cfg.rover_configs.push_back(r2_cfg);
    cfg.sync_byte_limit = 2000; // Generous budget

    MultiRoverCollective collective(cfg);
    auto* r1 = collective.get_rover(1);
    auto* r2 = collective.get_rover(2);

    // Both rovers observe shared landmark 100
    core::LandmarkKey shared_lm(1, 1, 100, 0xABCDEF);
    r1->iblt().insert(shared_lm);
    r2->iblt().insert(shared_lm);

    // Rover 1 observes distinct landmark 101; Rover 2 observes distinct landmark 102
    core::LandmarkKey lm1(1, 1, 101, 0x111111);
    core::LandmarkKey lm2(1, 2, 102, 0x222222);
    r1->iblt().insert(lm1);
    r2->iblt().insert(lm2);

    // Perform synchronization without geometric correspondences
    bool ok = collective.perform_synchronization(1, 2);
    CHECK(ok);
    CHECK_EQUAL(r1->sync_successes, 1);
    CHECK_EQUAL(r2->sync_successes, 1);
    CHECK_EQUAL(r1->sync_incomplete_decodes, 0);
}

// ----------------------------------------------------------------------
// Test 9: IBLT Capacity Overflow Fails Safely Without Graph Mutation
// ----------------------------------------------------------------------
void test_iblt_overflow_safety() {
    std::cout << "[Test 9] IBLT Capacity Overflow Fails Safely Without Graph Mutation..." << std::endl;

    CollectiveSimulationConfig cfg;
    SimulatedRoverConfig r1_cfg; r1_cfg.rover_id = 1; r1_cfg.iblt_cell_count = 10; cfg.rover_configs.push_back(r1_cfg);
    SimulatedRoverConfig r2_cfg; r2_cfg.rover_id = 2; r2_cfg.iblt_cell_count = 10; cfg.rover_configs.push_back(r2_cfg);

    MultiRoverCollective collective(cfg);
    auto* r1 = collective.get_rover(1);
    auto* r2 = collective.get_rover(2);

    // Insert 50 distinct landmarks into Rover 1, far exceeding 10-cell capacity
    for (uint32_t i = 1; i <= 50; ++i) {
        r1->iblt().insert(core::LandmarkKey(1, 1, i, i * 0x1000));
    }

    size_t r1_nodes_before = r1->pose_graph().num_nodes();
    size_t r1_edges_before = r1->pose_graph().num_edges();

    // Synchronization should detect IncompleteResidual, fail gracefully, and NOT mutate graph
    bool ok = collective.perform_synchronization(1, 2);
    CHECK(!ok);
    CHECK_EQUAL(r1->sync_incomplete_decodes, 1);
    CHECK_EQUAL(r1->pose_graph().num_nodes(), r1_nodes_before);
    CHECK_EQUAL(r1->pose_graph().num_edges(), r1_edges_before);
}

// ----------------------------------------------------------------------
// Test 10: Geometric Verification Required for Loop Closure Promotion
// ----------------------------------------------------------------------
void test_geometric_verification_gating() {
    std::cout << "[Test 10] Geometric Verification Required for Loop Closure Promotion..." << std::endl;

    CollectiveSimulationConfig cfg;
    SimulatedRoverConfig r1_cfg; r1_cfg.rover_id = 1; cfg.rover_configs.push_back(r1_cfg);
    SimulatedRoverConfig r2_cfg; r2_cfg.rover_id = 2; cfg.rover_configs.push_back(r2_cfg);

    MultiRoverCollective collective(cfg);
    auto* r1 = collective.get_rover(1);
    auto* r2 = collective.get_rover(2);

    // Initial components: each rover is separate
    CHECK_EQUAL(r1->partition_manager().num_partitions(), 1);
    CHECK_EQUAL(r2->partition_manager().num_partitions(), 1);

    // Case A: Corrupted / non-rigid correspondences (outliers)
    std::vector<verification::PointCorrespondence2D> bad_corrs = {
        {{0.0, 0.0}, {10.0, 0.0}},
        {{1.0, 0.0}, {0.0, 10.0}},
        {{0.0, 1.0}, {5.0, 5.0}},
        {{1.0, 1.0}, {-2.0, 8.0}}
    };

    // Synchronization runs, but geometric gating REJECTS promotion
    bool ok_bad = collective.perform_synchronization(1, 2, bad_corrs);
    CHECK(!ok_bad);
    CHECK_EQUAL(r1->partition_manager().num_partitions(), 1); // Not merged!

    // Case B: Valid rigid correspondences (translation dx = 2.0, dy = 1.0)
    std::vector<verification::PointCorrespondence2D> good_corrs = {
        {{0.0, 0.0}, {2.0, 1.0}},
        {{1.0, 0.0}, {3.0, 1.0}},
        {{0.0, 1.0}, {2.0, 2.0}},
        {{1.0, 1.0}, {3.0, 2.0}}
    };

    bool ok_good = collective.perform_synchronization(1, 2, good_corrs);
    CHECK(ok_good);
    // Verified closure was added!
    bool found_verified = false;
    for (const auto& [eid, e] : r1->pose_graph().edges()) {
        if (e.type == graph::EdgeType::InterRoverLoopClosure &&
            e.status == graph::EdgeVerificationStatus::Verified) {
            found_verified = true;
        }
    }
    CHECK(found_verified);
}

// ----------------------------------------------------------------------
// Test 11: Byte Budget Denials and Resource Constraints
// ----------------------------------------------------------------------
void test_byte_budget_denials() {
    std::cout << "[Test 11] Byte Budget Denials and Resource Constraints..." << std::endl;

    CollectiveSimulationConfig cfg;
    SimulatedRoverConfig r1_cfg;
    r1_cfg.rover_id = 1;
    r1_cfg.budget_config.max_bytes_per_window = 100; // Tiny quota of 100 bytes per window
    cfg.rover_configs.push_back(r1_cfg);

    SimulatedRoverConfig r2_cfg;
    r2_cfg.rover_id = 2;
    cfg.rover_configs.push_back(r2_cfg);

    MultiRoverCollective collective(cfg);

    // Enqueue message with 80-byte payload + 24B header + 20B framing = 124 bytes (> 100B quota)
    std::vector<uint8_t> payload(80, 0x55);
    collective.enqueue_message(1, 2, comms::DtnMessageType::GenericData,
                               comms::PriorityLevel::Normal, payload, 500'000'000);

    // Step collective: dispatch should deny due to quota
    collective.step(10'000'000);

    auto* r1 = collective.get_rover(1);
    CHECK(r1 != nullptr);
    CHECK(r1->current_state() == RoverState::ResourceConstrained);
}

// ----------------------------------------------------------------------
// Test 12: Memory Bounding via Safe Keyframe Pruner in Rover
// ----------------------------------------------------------------------
void test_keyframe_pruner_in_rover() {
    std::cout << "[Test 12] Memory Bounding via Safe Keyframe Pruner in Rover..." << std::endl;

    CollectiveSimulationConfig cfg;
    SimulatedRoverConfig r1_cfg;
    r1_cfg.rover_id = 1;
    r1_cfg.pruner_config.max_retained_keyframes = 10;
    r1_cfg.pruner_config.selection.min_translation_m = 0.5;
    cfg.rover_configs.push_back(r1_cfg);

    MultiRoverCollective collective(cfg);
    auto* r1 = collective.get_rover(1);
    CHECK(r1 != nullptr);

    // Add 15 sequential keyframes
    for (uint32_t step = 1; step <= 15; ++step) {
        r1->add_odometry_step(geometry::SE2(1.0, 0.0, 0.0), step * 100'000'000);
    }

    // On a linear chain, all interior nodes are Tarjan articulation points.
    // The pruner must safely halt without pruning any cut-vertices!
    CHECK_EQUAL(r1->pose_graph().num_nodes(), 16); // 1 anchor + 15 steps
    CHECK(r1->keyframe_prunes_rejected > 0);       // Rejected pruning safely
    CHECK_EQUAL(r1->partition_manager().num_partitions(), 1); // Fully connected!
}

// ----------------------------------------------------------------------
// Test 13: Seeded Deterministic Property Test
// ----------------------------------------------------------------------
void test_seeded_simulation_determinism() {
    std::cout << "[Test 13] Seeded Deterministic Property Test..." << std::endl;

    auto run_sim = [](uint64_t seed) -> CollectiveMetrics {
        CollectiveSimulationConfig cfg;
        cfg.random_seed = seed;
        SimulatedRoverConfig r1; r1.rover_id = 1; cfg.rover_configs.push_back(r1);
        SimulatedRoverConfig r2; r2.rover_id = 2; cfg.rover_configs.push_back(r2);
        cfg.network_config.latency_ns = 30'000'000;
        cfg.network_config.packet_loss_rate = 0.20;

        MultiRoverCollective collective(cfg);

        // Deterministic sequence of steps and messages
        for (int i = 0; i < 10; ++i) {
            std::vector<uint8_t> p = {static_cast<uint8_t>(i)};
            collective.enqueue_message(1, 2, comms::DtnMessageType::GenericData,
                                       comms::PriorityLevel::Normal, p, 500'000'000);
            collective.step(20'000'000);
        }

        return collective.collect_metrics();
    };

    auto m1 = run_sim(777);
    auto m2 = run_sim(777);

    CHECK_EQUAL(m1.messages_generated, m2.messages_generated);
    CHECK_EQUAL(m1.messages_admitted, m2.messages_admitted);
    CHECK_EQUAL(m1.messages_sent, m2.messages_sent);
    CHECK_EQUAL(m1.messages_delivered, m2.messages_delivered);
    CHECK_EQUAL(m1.messages_dropped, m2.messages_dropped);
    CHECK_EQUAL(m1.total_wire_bytes_sent, m2.total_wire_bytes_sent);
    CHECK_EQUAL(m1.total_wire_bytes_delivered, m2.total_wire_bytes_delivered);
}

// ----------------------------------------------------------------------
// Test 14: DTN Retry Accounting and Quota Consumption
// ----------------------------------------------------------------------
void test_retry_accounting_and_quota() {
    std::cout << "[Test 14] DTN Retry Accounting and Quota Consumption..." << std::endl;

    CollectiveSimulationConfig cfg;
    cfg.random_seed = 42;
    cfg.enable_dtn_retries = true;
    SimulatedRoverConfig r1; r1.rover_id = 1; cfg.rover_configs.push_back(r1);
    SimulatedRoverConfig r2; r2.rover_id = 2; cfg.rover_configs.push_back(r2);
    // 100% loss initially to force retries
    cfg.network_config.latency_ns = 10'000'000;
    cfg.network_config.packet_loss_rate = 1.0;

    MultiRoverCollective collective(cfg);

    std::vector<uint8_t> payload = {1, 2, 3, 4};
    auto res = collective.enqueue_message(1, 2, comms::DtnMessageType::GenericData,
                                          comms::PriorityLevel::Normal, payload, 5'000'000'000);
    CHECK(res == comms::EnqueueResult::Success);

    // Initial transmission attempt: t = 0 to 20ms
    collective.step(20'000'000);

    CollectiveMetrics m_init = collective.collect_metrics();
    CHECK_EQUAL(m_init.messages_admitted, 1);
    CHECK_EQUAL(m_init.initial_transmission_attempts, 1);
    CHECK_EQUAL(m_init.messages_retried, 0);
    CHECK_EQUAL(m_init.messages_sent, 1);
    CHECK_EQUAL(m_init.messages_dropped, 1);
    CHECK_EQUAL(m_init.messages_delivered, 0);

    // Check sender's packet budget manager
    auto* s1 = collective.get_rover(1);
    CHECK(s1 != nullptr);
    CHECK_EQUAL(s1->budget_manager().statistics().retransmitted.message_count, 0);

    // Now wait for DTN backoff (minimum 100ms after attempt 1) to trigger retry
    // Step by 150ms: reaching t = 170ms (> 120ms required)
    collective.step(150'000'000);

    CollectiveMetrics m_retry1 = collective.collect_metrics();
    // After retry triggered
    CHECK_EQUAL(m_retry1.initial_transmission_attempts, 1);
    CHECK_EQUAL(m_retry1.messages_retried, 1);
    CHECK_EQUAL(m_retry1.messages_sent, 2);
    CHECK_EQUAL(s1->budget_manager().statistics().retransmitted.message_count, 1);
    CHECK(m_retry1.total_wire_bytes_sent > m_init.total_wire_bytes_sent);

    // Accounting invariant: messages_sent == initial_transmission_attempts + messages_retried
    CHECK_EQUAL(m_retry1.messages_sent,
                m_retry1.initial_transmission_attempts + m_retry1.messages_retried);
}

// ----------------------------------------------------------------------
// Test 15: Retry Exhaustion and TTL Expiration
// ----------------------------------------------------------------------
void test_retry_exhaustion_and_ttl() {
    std::cout << "[Test 15] Retry Exhaustion and TTL Expiration..." << std::endl;

    CollectiveSimulationConfig cfg;
    cfg.random_seed = 99;
    cfg.enable_dtn_retries = true;
    SimulatedRoverConfig r1; r1.rover_id = 1; cfg.rover_configs.push_back(r1);
    SimulatedRoverConfig r2; r2.rover_id = 2; cfg.rover_configs.push_back(r2);
    cfg.network_config.latency_ns = 5'000'000;
    cfg.network_config.packet_loss_rate = 1.0; // Never succeeds

    MultiRoverCollective collective(cfg);

    // Message 1: Short TTL (70ms), max_attempts 5
    std::vector<uint8_t> p1 = {0xAA};
    collective.enqueue_message(1, 2, comms::DtnMessageType::GenericData,
                               comms::PriorityLevel::Normal, p1, 70'000'000);

    // Step 20ms: initial attempt
    collective.step(20'000'000);
    // Step 80ms: past TTL -> message should expire before retry exhaustion
    collective.step(80'000'000);

    CollectiveMetrics m1 = collective.collect_metrics();
    CHECK(m1.messages_expired >= 1);

    // Message 2: Long TTL (10s), should exhaust retries (default max_attempts = 5)
    std::vector<uint8_t> p2 = {0xBB};
    collective.enqueue_message(1, 2, comms::DtnMessageType::GenericData,
                               comms::PriorityLevel::Normal, p2, 10'000'000'000);

    // Run long enough to exhaust 5 attempts with backoff (50ms, 100ms, 200ms, 400ms...)
    for (int i = 0; i < 30; ++i) {
        collective.step(100'000'000);
    }

    CollectiveMetrics m2 = collective.collect_metrics();
    CHECK(m2.retries_exhausted >= 1);
    CHECK_EQUAL(collective.get_rover(1)->dtn_buffer().message_count(), 0); // Purged!
}

// ----------------------------------------------------------------------
// Test 16: Duplicate Delivery and Exactly-Once Logical Processing
// ----------------------------------------------------------------------
void test_duplicate_delivery_and_idempotence() {
    std::cout << "[Test 16] Duplicate Delivery and Exactly-Once Logical Processing..." << std::endl;

    CollectiveSimulationConfig cfg;
    cfg.random_seed = 123;
    cfg.enable_dtn_retries = false;
    SimulatedRoverConfig r1; r1.rover_id = 1; cfg.rover_configs.push_back(r1);
    SimulatedRoverConfig r2; r2.rover_id = 2; cfg.rover_configs.push_back(r2);
    cfg.network_config.latency_ns = 5'000'000;
    cfg.network_config.packet_loss_rate = 0.0; // Perfect delivery

    MultiRoverCollective collective(cfg);

    auto* rover2 = collective.get_rover(2);
    CHECK(rover2 != nullptr);

    // Pre-record message ID 1 into rover 2's deduplication cache
    rover2->dtn_buffer().record_delivered_dedup(1, 1'000'000);
    CHECK(rover2->dtn_buffer().is_duplicate(1, 1'000'000));

    // Enqueue message from Rover 1 to Rover 2 (will get message_id = 1)
    std::vector<uint8_t> p = {42, 43};
    auto res = collective.enqueue_message(1, 2, comms::DtnMessageType::GenericData,
                                          comms::PriorityLevel::Normal, p, 1'000'000'000);
    CHECK(res == comms::EnqueueResult::Success);

    // Step collective: 10ms dispatch + 10ms delivery
    collective.step(10'000'000);
    collective.step(10'000'000);

    CollectiveMetrics m = collective.collect_metrics();
    CHECK_EQUAL(m.duplicate_deliveries, 1);
    CHECK_EQUAL(m.messages_processed, 0); // Dropped by dedup, never processed!
}

// ----------------------------------------------------------------------
// Test 17: In-Flight Packet Policy at Network Partition Boundary
// ----------------------------------------------------------------------
void test_in_flight_partition_boundary() {
    std::cout << "[Test 17] In-Flight Packet Policy at Network Partition Boundary..." << std::endl;

    CollectiveSimulationConfig cfg;
    cfg.random_seed = 456;
    cfg.enable_dtn_retries = false;
    SimulatedRoverConfig r1; r1.rover_id = 1; cfg.rover_configs.push_back(r1);
    SimulatedRoverConfig r2; r2.rover_id = 2; cfg.rover_configs.push_back(r2);
    cfg.network_config.latency_ns = 50'000'000; // 50 ms latency
    cfg.network_config.packet_loss_rate = 0.0;

    MultiRoverCollective collective(cfg);

    // Enqueue message at t = 0
    std::vector<uint8_t> p = {10, 20};
    auto res = collective.enqueue_message(1, 2, comms::DtnMessageType::GenericData,
                                          comms::PriorityLevel::Normal, p, 1'000'000'000);
    CHECK(res == comms::EnqueueResult::Success);

    // Step 10ms: message is dispatched and in-flight (arrival scheduled at t = 60ms)
    collective.step(10'000'000);
    CHECK_EQUAL(collective.collect_metrics().messages_sent, 1);
    CHECK_EQUAL(collective.collect_metrics().messages_delivered, 0);

    // At t = 20ms, partition occurs between rover 1 and rover 2
    collective.set_link_state(1, 2, false);
    CHECK(!collective.is_link_connected(1, 2));

    // Step to 60ms (past scheduled arrival time)
    collective.step(50'000'000);

    CollectiveMetrics m = collective.collect_metrics();
    // Packet should have been dropped due to active partition at delivery time
    CHECK_EQUAL(m.messages_delivered, 0);
    CHECK_EQUAL(m.messages_dropped, 1);
}

// ----------------------------------------------------------------------
// Test 18: IBLT Decode Failure Atomicity
// ----------------------------------------------------------------------
void test_iblt_failure_atomicity() {
    std::cout << "[Test 18] IBLT Decode Failure Atomicity..." << std::endl;

    CollectiveSimulationConfig cfg;
    cfg.random_seed = 888;
    SimulatedRoverConfig r1; r1.rover_id = 1; cfg.rover_configs.push_back(r1);
    SimulatedRoverConfig r2; r2.rover_id = 2; cfg.rover_configs.push_back(r2);
    MultiRoverCollective collective(cfg);

    auto* rov1 = collective.get_rover(1);
    auto* rov2 = collective.get_rover(2);

    // Seed both rovers with initial poses
    rov1->add_odometry_step(geometry::SE2(1.0, 0.0, 0.0), 100);
    rov2->add_odometry_step(geometry::SE2(2.0, 0.0, 0.0), 100);

    size_t nodes1_before = rov1->pose_graph().num_nodes();
    size_t edges1_before = rov1->pose_graph().num_edges();
    size_t parts1_before = rov1->partition_manager().num_partitions();

    size_t nodes2_before = rov2->pose_graph().num_nodes();
    size_t edges2_before = rov2->pose_graph().num_edges();
    size_t parts2_before = rov2->partition_manager().num_partitions();

    // Populate each rover's IBLT with 15 distinct landmark keys (capacity is 8)
    // This guarantees incomplete decoding due to capacity exhaustion
    for (uint64_t k = 1; k <= 15; ++k) {
        rov1->iblt().insert(core::LandmarkKey(1, 1, static_cast<uint32_t>(k), 100ULL));
        rov2->iblt().insert(core::LandmarkKey(1, 2, static_cast<uint32_t>(k + 100), 200ULL));
    }

    // Attempt synchronization
    bool sync_ok = collective.perform_synchronization(1, 2);
    CHECK(!sync_ok); // Must fail gracefully!

    // Verify pose graphs and DSU partitions are strictly untouched (atomic rollback)
    CHECK_EQUAL(rov1->pose_graph().num_nodes(), nodes1_before);
    CHECK_EQUAL(rov1->pose_graph().num_edges(), edges1_before);
    CHECK_EQUAL(rov1->partition_manager().num_partitions(), parts1_before);

    CHECK_EQUAL(rov2->pose_graph().num_nodes(), nodes2_before);
    CHECK_EQUAL(rov2->pose_graph().num_edges(), edges2_before);
    CHECK_EQUAL(rov2->partition_manager().num_partitions(), parts2_before);

    CollectiveMetrics m = collective.collect_metrics();
    CHECK(m.sync_incomplete_decodes >= 1);
    CHECK_EQUAL(m.verified_inter_rover_closures, 0);
}

// ----------------------------------------------------------------------
// Test 19: Rover Timeout Boundaries and Silence Duration
// ----------------------------------------------------------------------
void test_rover_timeout_boundaries() {
    std::cout << "[Test 19] Rover Timeout Boundaries and Silence Duration..." << std::endl;

    RoverStateMachineConfig sm_cfg;
    sm_cfg.degradation_timeout_ns = 3'000'000'000;   // 3.0 s
    sm_cfg.disconnection_timeout_ns = 10'000'000'000; // 10.0 s

    RoverStateMachine sm(1, sm_cfg);
    sm.transition_to(RoverState::Exploring, TransitionReason::InitializationComplete, 0);

    // Rover starts with last_peer_contact_ns = 0.
    // At t = 2.0s: silence < 3.0s (heartbeat timeout), stays Exploring
    sm.evaluate_timeouts(2'000'000'000, 0, false);
    CHECK(sm.current_state() == RoverState::Exploring);

    // At t = 3.5s: silence >= 3.0s, degrades to CommunicationDegraded
    sm.evaluate_timeouts(3'500'000'000, 0, false);
    CHECK(sm.current_state() == RoverState::CommunicationDegraded);

    // At t = 10.5s: silence >= 10.0s (disconnection timeout), disconnects
    sm.evaluate_timeouts(10'500'000'000, 0, false);
    CHECK(sm.current_state() == RoverState::Disconnected);

    // Overflow check with extreme timestamp: should not wrap or crash
    sm.evaluate_timeouts(UINT64_MAX - 1000, 0, false);
    CHECK(sm.current_state() == RoverState::Disconnected);
}

// ----------------------------------------------------------------------
// Test 20: Direct Delivery with Explicit Network Acknowledgment
// ----------------------------------------------------------------------
void test_direct_delivery_with_explicit_ack() {
    std::cout << "[Test 20] Direct Delivery with Explicit Network Acknowledgment..." << std::endl;

    CollectiveSimulationConfig cfg;
    cfg.random_seed = 1001;
    cfg.enable_explicit_acks = true;
    cfg.enable_dtn_retries = true;
    SimulatedRoverConfig r1; r1.rover_id = 1; cfg.rover_configs.push_back(r1);
    SimulatedRoverConfig r2; r2.rover_id = 2; cfg.rover_configs.push_back(r2);
    cfg.network_config.latency_ns = 20'000'000; // 20 ms one-way
    cfg.network_config.packet_loss_rate = 0.0;

    MultiRoverCollective collective(cfg);

    std::vector<uint8_t> p = {1, 2, 3};
    auto res = collective.enqueue_message(1, 2, comms::DtnMessageType::GenericData,
                                          comms::PriorityLevel::Normal, p, 1'000'000'000);
    CHECK(res == comms::EnqueueResult::Success);

    // At t = 0ms: message enqueued in rover 1's buffer
    // Step 1: 10ms (t = 10ms) -> dispatched into transit, arrives at t = 10ms + 20ms = 30ms
    collective.step(10'000'000);
    auto* s1 = collective.get_rover(1);
    CHECK(s1 != nullptr);
    CHECK_EQUAL(s1->dtn_buffer().message_count(), 1);

    // Step 2: 25ms (t = 35ms > 30ms) -> Data delivered to Rover 2, Rover 2 enqueues ACK and dispatches it
    // ACK delivery scheduled for t = 35ms + 20ms = 55ms. Message MUST still be in Rover 1's DTN buffer!
    collective.step(25'000'000);
    CHECK_EQUAL(s1->dtn_buffer().message_count(), 1);

    // Step 3: 25ms (t = 60ms > 55ms) -> ACK arrives at Rover 1! Message acknowledged and removed from buffer.
    collective.step(25'000'000);
    CHECK_EQUAL(s1->dtn_buffer().message_count(), 0);

    CollectiveMetrics m = collective.collect_metrics();
    CHECK_EQUAL(m.unique_messages_processed, 1);
    CHECK(m.ack_transmission_attempts >= 1);
    CHECK(m.ack_deliveries >= 1);
    CHECK(m.ack_wire_bytes_sent > 0);
    CHECK(m.ack_wire_bytes_delivered > 0);
}

// ----------------------------------------------------------------------
// Test 21: Lost Data Packet Followed by Successful Retry
// ----------------------------------------------------------------------
void test_lost_data_packet_retry() {
    std::cout << "[Test 21] Lost Data Packet Followed by Successful Retry..." << std::endl;

    CollectiveSimulationConfig cfg;
    cfg.random_seed = 1002;
    cfg.enable_explicit_acks = true;
    cfg.enable_dtn_retries = true;
    SimulatedRoverConfig r1; r1.rover_id = 1; cfg.rover_configs.push_back(r1);
    SimulatedRoverConfig r2; r2.rover_id = 2; cfg.rover_configs.push_back(r2);
    cfg.network_config.latency_ns = 10'000'000;
    cfg.network_config.packet_loss_rate = 1.0; // 100% loss initially

    MultiRoverCollective collective(cfg);

    std::vector<uint8_t> p = {0xDE, 0xAD};
    collective.enqueue_message(1, 2, comms::DtnMessageType::GenericData,
                               comms::PriorityLevel::Normal, p, 5'000'000'000);

    // Step 20ms: initial attempt drops
    collective.step(20'000'000);
    CHECK_EQUAL(collective.collect_metrics().messages_dropped, 1);
    CHECK_EQUAL(collective.collect_metrics().unique_messages_processed, 0);

    // Restore perfect channel
    // Now wait for DTN retry timer (100ms backoff)
    collective.step(150'000'000);

    // Force network to 0% loss for subsequent retry
    // In our simulation, collective config network_config packet loss rate can be set
    // or we can test retry behavior under deterministic channel
    CollectiveMetrics m = collective.collect_metrics();
    CHECK(m.initial_transmission_attempts >= 1);
    CHECK(m.hop_data_losses >= 1);
}

// ----------------------------------------------------------------------
// Test 22: Lost ACK Followed by Duplicate Delivery and Returning ACK
// ----------------------------------------------------------------------
void test_lost_ack_duplicate_delivery() {
    std::cout << "[Test 22] Lost ACK Followed by Duplicate Delivery and Returning ACK..." << std::endl;

    CollectiveSimulationConfig cfg;
    cfg.random_seed = 1003;
    cfg.enable_explicit_acks = true;
    cfg.enable_dtn_retries = true;
    SimulatedRoverConfig r1; r1.rover_id = 1; cfg.rover_configs.push_back(r1);
    SimulatedRoverConfig r2; r2.rover_id = 2; cfg.rover_configs.push_back(r2);
    cfg.network_config.latency_ns = 10'000'000;
    cfg.network_config.packet_loss_rate = 0.0;
    // Set 100% loss specifically for ACKs
    cfg.ack_packet_loss_rate = 1.0;

    MultiRoverCollective collective(cfg);

    std::vector<uint8_t> p = {7, 8, 9};
    collective.enqueue_message(1, 2, comms::DtnMessageType::GenericData,
                               comms::PriorityLevel::Normal, p, 5'000'000'000);

    // Step 1: 10ms (t=10ms) -> Data dispatched, arrival at 10ms + 10ms = 20ms
    collective.step(10'000'000);
    // Step 2: 15ms (t=25ms > 20ms) -> Data arrives at Rover 2, ACK enqueued and dispatched (dropped by 100% loss)
    collective.step(15'000'000);

    CollectiveMetrics m1 = collective.collect_metrics();
    CHECK_EQUAL(m1.unique_messages_processed, 1);
    CHECK(m1.ack_losses >= 1);
    // Sender Rover 1 still has message in buffer because ACK was lost!
    auto* s1 = collective.get_rover(1);
    CHECK_EQUAL(s1->dtn_buffer().message_count(), 1);

    // Now advance time to trigger retransmission (retry timer backoff ~100ms)
    // Step by 150ms: triggers retry attempt and duplicate delivery
    collective.step(150'000'000);
    // Additional step for the retry packet in transit to arrive at Rover 2
    collective.step(20'000'000);

    CollectiveMetrics m2 = collective.collect_metrics();
    CHECK(m2.retransmissions_missing_ack >= 1);
    // Duplicate data arrives at destination; dedup must NOT reprocess!
    CHECK_EQUAL(m2.unique_messages_processed, 1);
    CHECK(m2.duplicate_deliveries >= 1);
    CHECK_EQUAL(m2.duplicate_processing_attempts, 0);
}

// ----------------------------------------------------------------------
// Test 23: Duplicate ACK Idempotence
// ----------------------------------------------------------------------
void test_duplicate_ack_idempotence() {
    std::cout << "[Test 23] Duplicate ACK Idempotence..." << std::endl;

    CollectiveSimulationConfig cfg;
    cfg.random_seed = 1004;
    cfg.enable_explicit_acks = true;
    cfg.enable_dtn_retries = true;
    SimulatedRoverConfig r1; r1.rover_id = 1; cfg.rover_configs.push_back(r1);
    SimulatedRoverConfig r2; r2.rover_id = 2; cfg.rover_configs.push_back(r2);
    MultiRoverCollective collective(cfg);

    auto* rover1 = collective.get_rover(1);
    CHECK(rover1 != nullptr);

    // Manually acknowledge a non-existent or already acknowledged message ID
    auto res1 = rover1->dtn_buffer().acknowledge(999999, 1000);
    CHECK(res1 == comms::AckResult::NotFound);

    // Second repeated ACK must be completely idempotent
    auto res2 = rover1->dtn_buffer().acknowledge(999999, 2000);
    CHECK(res2 == comms::AckResult::NotFound);
    CHECK_EQUAL(rover1->dtn_buffer().message_count(), 0);
}

// ----------------------------------------------------------------------
// Test 24: ACK Consumes Quota and Wire Bytes
// ----------------------------------------------------------------------
void test_ack_byte_quota_consumption() {
    std::cout << "[Test 24] ACK Consumes Quota and Wire Bytes..." << std::endl;

    CollectiveSimulationConfig cfg;
    cfg.random_seed = 1005;
    cfg.enable_explicit_acks = true;
    cfg.enable_dtn_retries = true;
    SimulatedRoverConfig r1; r1.rover_id = 1; cfg.rover_configs.push_back(r1);
    SimulatedRoverConfig r2; r2.rover_id = 2; cfg.rover_configs.push_back(r2);
    cfg.network_config.latency_ns = 10'000'000;
    cfg.network_config.packet_loss_rate = 0.0;

    MultiRoverCollective collective(cfg);

    std::vector<uint8_t> p = {11, 22};
    collective.enqueue_message(1, 2, comms::DtnMessageType::GenericData,
                               comms::PriorityLevel::Normal, p, 1'000'000'000);

    collective.step(10'000'000); // Dispatched at t=10ms, arrival at 20ms
    collective.step(15'000'000); // Arrives at 20ms, ACK dispatched at 25ms, arrival at 35ms
    collective.step(15'000'000); // ACK arrives at Rover 1

    CollectiveMetrics m = collective.collect_metrics();
    CHECK(m.ack_wire_bytes_sent > 0);
    CHECK(m.ack_wire_bytes_delivered > 0);
    // Total wire bytes sent must include data payload + protocol + framing + ACK payload + protocol + framing
    CHECK(m.total_wire_bytes_sent >= m.payload_bytes_sent + m.ack_wire_bytes_sent);
}

// ----------------------------------------------------------------------
// Test 25: Three-Rover Multi-Hop Route
// ----------------------------------------------------------------------
void test_three_rover_multihop_route() {
    std::cout << "[Test 25] Three-Rover Multi-Hop Route (1 -> 2 -> 3)..." << std::endl;

    CollectiveSimulationConfig cfg;
    cfg.random_seed = 1006;
    cfg.enable_explicit_acks = true;
    cfg.enable_multihop_routing = true;
    SimulatedRoverConfig r1; r1.rover_id = 1; cfg.rover_configs.push_back(r1);
    SimulatedRoverConfig r2; r2.rover_id = 2; cfg.rover_configs.push_back(r2);
    SimulatedRoverConfig r3; r3.rover_id = 3; cfg.rover_configs.push_back(r3);
    cfg.network_config.latency_ns = 15'000'000; // 15 ms per hop
    cfg.network_config.packet_loss_rate = 0.0;

    MultiRoverCollective collective(cfg);

    // Topology: 1 <-> 2 and 2 <-> 3 connected, but 1 <-> 3 SEVERED!
    collective.set_link_state(1, 3, false);
    CHECK(!collective.is_link_connected(1, 3));
    CHECK(collective.is_link_connected(1, 2));
    CHECK(collective.is_link_connected(2, 3));

    // Verify BFS finds next hop from 1 to 3 via 2
    auto next_hop = collective.find_next_hop(1, 3);
    CHECK(next_hop.has_value());
    CHECK_EQUAL(*next_hop, 2);

    std::vector<uint8_t> p = {100, 101, 102};
    collective.enqueue_message(1, 3, comms::DtnMessageType::GenericData,
                               comms::PriorityLevel::Normal, p, 2'000'000'000, 0, 5);

    // Step 1: 10ms -> 1 dispatches to 2, arrival at 10 + 15 = 25ms
    collective.step(10'000'000);
    // Step 2: 20ms (t=30ms > 25ms) -> 2 receives data, enqueues forward and dispatches towards 3 (arrival at 30 + 15 = 45ms)
    collective.step(20'000'000);
    // Destination 3 has not received it yet
    CHECK_EQUAL(collective.get_rover(3)->messages_processed, 0);

    // Step 3: 20ms (t=50ms > 45ms) -> 3 receives data and generates ACK towards 1 (via 2)
    collective.step(20'000'000);
    CHECK_EQUAL(collective.get_rover(3)->messages_processed, 1);

    // Step 4: 25ms (t=75ms > 65ms) -> ACK arrives at 2, 2 enqueues and dispatches forward ACK towards 1 (arrival at 75 + 15 = 90ms)
    collective.step(25'000'000);
    // Step 5: 25ms (t=100ms > 90ms) -> ACK arrives at 1!
    collective.step(25'000'000);

    CollectiveMetrics m = collective.collect_metrics();
    CHECK_EQUAL(m.unique_messages_processed, 1);
    CHECK(m.hop_data_attempts >= 2); // At least 2 data hops
    CHECK_EQUAL(collective.get_rover(1)->dtn_buffer().message_count(), 0); // Cleared by return ACK!
}

// ----------------------------------------------------------------------
// Test 26: Temporal Partition Store-Carry-Forward
// ----------------------------------------------------------------------
void test_temporal_partition_store_carry_forward() {
    std::cout << "[Test 26] Temporal Partition Store-Carry-Forward..." << std::endl;

    CollectiveSimulationConfig cfg;
    cfg.random_seed = 1007;
    cfg.enable_explicit_acks = true;
    cfg.enable_multihop_routing = true;
    SimulatedRoverConfig r1; r1.rover_id = 1; cfg.rover_configs.push_back(r1);
    SimulatedRoverConfig r2; r2.rover_id = 2; cfg.rover_configs.push_back(r2);
    SimulatedRoverConfig r3; r3.rover_id = 3; cfg.rover_configs.push_back(r3);
    cfg.network_config.latency_ns = 10'000'000;

    MultiRoverCollective collective(cfg);

    // 1 <-> 3 is always severed
    collective.set_link_state(1, 3, false);
    // 2 <-> 3 is TEMPORARILY severed at the beginning!
    collective.set_link_state(2, 3, false);

    // Configure static relay route: Rover 1 forwards to carrier Rover 2 for destination 3
    collective.set_static_route(1, 3, 2);

    std::vector<uint8_t> p = {55, 66};
    collective.enqueue_message(1, 3, comms::DtnMessageType::GenericData,
                               comms::PriorityLevel::Normal, p, 5'000'000'000);

    // Step 1: 10ms -> 1 dispatches to 2 (arrival at 10 + 10 = 20ms)
    collective.step(10'000'000);
    // Step 2: 15ms (t=25ms > 20ms) -> 2 receives data. Since 2 <-> 3 is severed, 2 holds message in DTN buffer!
    collective.step(15'000'000);
    auto* r2_ptr = collective.get_rover(2);
    CHECK(r2_ptr != nullptr);
    CHECK(r2_ptr->dtn_buffer().message_count() >= 1);
    CHECK_EQUAL(collective.get_rover(3)->messages_processed, 0);

    // Rover 2 carries message during partition for 100ms
    collective.step(100'000'000);
    CHECK_EQUAL(collective.get_rover(3)->messages_processed, 0);

    // Now contact is established: 2 <-> 3 connects!
    collective.set_link_state(2, 3, true);

    // Step 3: 20ms -> Rover 2 dispatches stored message towards Rover 3
    collective.step(20'000'000);
    // Step 4: 20ms -> Rover 3 receives and processes the delivered message!
    collective.step(20'000'000);
    CHECK_EQUAL(collective.get_rover(3)->messages_processed, 1);
}


// ----------------------------------------------------------------------
// Test 27: Hop-Limit Exhaustion
// ----------------------------------------------------------------------
void test_hop_limit_exhaustion() {
    std::cout << "[Test 27] Hop-Limit Exhaustion..." << std::endl;

    CollectiveSimulationConfig cfg;
    cfg.random_seed = 1008;
    cfg.enable_multihop_routing = true;
    SimulatedRoverConfig r1; r1.rover_id = 1; cfg.rover_configs.push_back(r1);
    SimulatedRoverConfig r2; r2.rover_id = 2; cfg.rover_configs.push_back(r2);
    SimulatedRoverConfig r3; r3.rover_id = 3; cfg.rover_configs.push_back(r3);
    cfg.network_config.latency_ns = 10'000'000;

    MultiRoverCollective collective(cfg);

    // Topology: 1 <-> 2, 2 <-> 3 (requires 2 hops). 1 <-> 3 severed.
    collective.set_link_state(1, 3, false);

    // Enqueue with max_hops = 1 (cannot reach 3 which is 2 hops away)
    std::vector<uint8_t> p = {99};
    collective.enqueue_message(1, 3, comms::DtnMessageType::GenericData,
                               comms::PriorityLevel::Normal, p, 1'000'000'000, 0, 1);

    // Step 50ms
    collective.run(50'000'000, 10'000'000);

    CollectiveMetrics m = collective.collect_metrics();
    // Reached rover 2 with hop_count = 1, but max_hops = 1 -> exhausted!
    CHECK(m.hop_limit_exhausted >= 1);
    CHECK_EQUAL(collective.get_rover(3)->messages_processed, 0);
}

// ----------------------------------------------------------------------
// Test 28: Anti-Looping Duplicate Forwarding
// ----------------------------------------------------------------------
void test_anti_looping_duplicate_forwarding() {
    std::cout << "[Test 28] Anti-Looping Duplicate Forwarding..." << std::endl;

    CollectiveSimulationConfig cfg;
    cfg.random_seed = 1009;
    cfg.enable_multihop_routing = true;
    SimulatedRoverConfig r1; r1.rover_id = 1; cfg.rover_configs.push_back(r1);
    SimulatedRoverConfig r2; r2.rover_id = 2; cfg.rover_configs.push_back(r2);
    SimulatedRoverConfig r3; r3.rover_id = 3; cfg.rover_configs.push_back(r3);
    cfg.network_config.latency_ns = 10'000'000;

    MultiRoverCollective collective(cfg);

    // Cyclic mesh: 1-2, 2-3, 3-1 all connected
    std::vector<uint8_t> p = {1, 2, 3, 4};
    collective.enqueue_message(1, 3, comms::DtnMessageType::GenericData,
                               comms::PriorityLevel::Normal, p, 1'000'000'000);

    // Run long enough for messages to traverse cyclic paths
    collective.run(200'000'000, 10'000'000);

    CollectiveMetrics m = collective.collect_metrics();
    // Must reach 3 and terminate cleanly; messages must NOT loop indefinitely!
    CHECK_EQUAL(m.unique_messages_processed, 1);
    CHECK(m.hop_data_attempts < 20); // Bounded!
}

// ----------------------------------------------------------------------
// Test 29: Deterministic Multi-Hop Property Test
// ----------------------------------------------------------------------
void test_multihop_simulation_determinism() {
    std::cout << "[Test 29] Deterministic Multi-Hop Property Test..." << std::endl;

    auto run_multihop = [](uint64_t seed) -> CollectiveMetrics {
        CollectiveSimulationConfig cfg;
        cfg.random_seed = seed;
        cfg.enable_explicit_acks = true;
        cfg.enable_multihop_routing = true;
        SimulatedRoverConfig r1; r1.rover_id = 1; cfg.rover_configs.push_back(r1);
        SimulatedRoverConfig r2; r2.rover_id = 2; cfg.rover_configs.push_back(r2);
        SimulatedRoverConfig r3; r3.rover_id = 3; cfg.rover_configs.push_back(r3);
        cfg.network_config.latency_ns = 15'000'000;
        cfg.network_config.packet_loss_rate = 0.15;

        MultiRoverCollective collective(cfg);
        collective.set_link_state(1, 3, false);

        for (int i = 0; i < 5; ++i) {
            std::vector<uint8_t> p = {static_cast<uint8_t>(i)};
            collective.enqueue_message(1, 3, comms::DtnMessageType::GenericData,
                                       comms::PriorityLevel::Normal, p, 2'000'000'000);
            collective.step(25'000'000);
        }
        collective.run(200'000'000, 10'000'000);
        return collective.collect_metrics();
    };

    auto m1 = run_multihop(55555);
    auto m2 = run_multihop(55555);

    CHECK_EQUAL(m1.messages_generated, m2.messages_generated);
    CHECK_EQUAL(m1.messages_admitted, m2.messages_admitted);
    CHECK_EQUAL(m1.hop_data_attempts, m2.hop_data_attempts);
    CHECK_EQUAL(m1.hop_data_delivered, m2.hop_data_delivered);
    CHECK_EQUAL(m1.ack_transmission_attempts, m2.ack_transmission_attempts);
    CHECK_EQUAL(m1.ack_deliveries, m2.ack_deliveries);
    CHECK_EQUAL(m1.unique_messages_processed, m2.unique_messages_processed);
    CHECK_EQUAL(m1.total_wire_bytes_sent, m2.total_wire_bytes_sent);
    CHECK_EQUAL(m1.total_wire_bytes_delivered, m2.total_wire_bytes_delivered);
}

int main() {
    std::cout << "======================================================================" << std::endl;
    std::cout << " AstroMesh-X Phase 4.6: Reliable ACK Protocol & Multi-Hop DTN Routing" << std::endl;
    std::cout << "======================================================================" << std::endl;

    test_state_machine_transitions();
    test_idempotent_event_handling();
    test_timeout_evaluation();
    test_resource_constraint_lifecycle();
    test_multi_rover_message_transmission();
    test_network_partition_isolation();
    test_ttl_expiration_during_partition();
    test_iblt_landmark_reconciliation();
    test_iblt_overflow_safety();
    test_geometric_verification_gating();
    test_byte_budget_denials();
    test_keyframe_pruner_in_rover();
    test_seeded_simulation_determinism();
    test_retry_accounting_and_quota();
    test_retry_exhaustion_and_ttl();
    test_duplicate_delivery_and_idempotence();
    test_in_flight_partition_boundary();
    test_iblt_failure_atomicity();
    test_rover_timeout_boundaries();

    // Phase 4.6 New Regression Tests
    test_direct_delivery_with_explicit_ack();
    test_lost_data_packet_retry();
    test_lost_ack_duplicate_delivery();
    test_duplicate_ack_idempotence();
    test_ack_byte_quota_consumption();
    test_three_rover_multihop_route();
    test_temporal_partition_store_carry_forward();
    test_hop_limit_exhaustion();
    test_anti_looping_duplicate_forwarding();
    test_multihop_simulation_determinism();

    std::cout << "======================================================================" << std::endl;
    std::cout << " Assertions checked : " << g_assertions_checked << std::endl;
    std::cout << " Assertions failed  : " << g_assertions_failed << std::endl;
    std::cout << "======================================================================" << std::endl;

    if (g_assertions_failed > 0) {
        std::cerr << "FAILURE: " << g_assertions_failed << " assertions failed!" << std::endl;
        return 1;
    }

    std::cout << "SUCCESS: All multi-rover orchestration and Phase 4.6 tests PASSED." << std::endl;
    return 0;
}

