#include "astromesh/core/iblt.hpp"
#include "astromesh/core/landmark_key.hpp"
#include "astromesh/geometry/se2.hpp"
#include "astromesh/net/distributed_reconciliation.hpp"
#include "astromesh/net/rover_node.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

using namespace astromesh::net;
using namespace astromesh::core;
using namespace astromesh::graph;
using namespace astromesh::geometry;

void test_iblt_serialization_roundtrip() {
    std::cout << "[TEST] Running test_iblt_serialization_roundtrip..." << std::endl;

    InvertibleBloomFilter iblt(32, 3, 1);
    for (uint32_t i = 1; i <= 10; ++i) {
        LandmarkKey key(1, 1, i, 0x123456789ABCDEF0ULL + i);
        assert(iblt.insert(key));
    }

    auto bytes = serialize_iblt_fingerprint(iblt);
    assert(bytes.size() == 12 + 32 * 32); // 1036 bytes, fits in 1 KB envelope

    auto deser_opt = deserialize_iblt_fingerprint(bytes.data(), bytes.size());
    assert(deser_opt.has_value());

    // Reconciling an identical table yields zero difference
    auto diff = iblt.subtract(*deser_opt);
    auto decode = diff.decode();
    assert(decode.is_success());
    assert(decode.left_only.empty());
    assert(decode.right_only.empty());

    std::cout << "  Serialized size: " << bytes.size() << " bytes (1024B cells + 12B header)" << std::endl;
    std::cout << "  -> Passed." << std::endl;
}

void test_candidate_payload_serialization() {
    std::cout << "[TEST] Running test_candidate_payload_serialization..." << std::endl;

    CandidateProposalPayload p;
    p.sender_rover_id = 2;
    p.target_rover_id = 1;
    for (uint32_t i = 1; i <= 4; ++i) {
        LandmarkObservationRecord obs;
        obs.key = LandmarkKey(1, 2, 100 + i, 0xABCDEF00ULL + i);
        obs.local_x = 1.5 * i;
        obs.local_y = -0.5 * i;
        obs.keyframe_index = 3;
        p.observations.push_back(obs);
    }

    auto p_bytes = p.serialize();
    assert(p_bytes.size() == 8 + 4 * 36);

    auto p_deser = CandidateProposalPayload::deserialize(p_bytes.data(), p_bytes.size());
    assert(p_deser.has_value());
    assert(p_deser->sender_rover_id == 2);
    assert(p_deser->target_rover_id == 1);
    assert(p_deser->observations.size() == 4);
    assert(p_deser->observations[0].key == p.observations[0].key);
    assert(std::abs(p_deser->observations[0].local_x - 1.5) < 1e-6);

    CandidateResponsePayload r;
    r.sender_rover_id = 1;
    r.target_rover_id = 2;
    r.verified = true;
    r.inlier_count = 4;
    r.inlier_rmse = 0.012;
    r.relative_transform = SE2(10.0, -2.5, 0.45);
    r.sender_keyframe = 2;
    r.target_keyframe = 3;

    auto r_bytes = r.serialize();
    assert(r_bytes.size() == 56);

    auto r_deser = CandidateResponsePayload::deserialize(r_bytes.data(), r_bytes.size());
    assert(r_deser.has_value());
    assert(r_deser->verified == true);
    assert(r_deser->inlier_count == 4);
    assert(std::abs(r_deser->relative_transform.x() - 10.0) < 1e-6);
    assert(std::abs(r_deser->relative_transform.y() - (-2.5)) < 1e-6);
    assert(std::abs(r_deser->relative_transform.theta() - 0.45) < 1e-6);

    std::cout << "  -> Passed." << std::endl;
}

void test_distributed_reconciliation_over_udp() {
    std::cout << "[TEST] Running test_distributed_reconciliation_over_udp..." << std::endl;

    RoverNodeConfig c1;
    c1.rover_id = 1;
    c1.listen_port = 20601;
    c1.listen_ip = "127.0.0.1";
    c1.seed_peers = { Endpoint("127.0.0.1", 20602) };
    c1.heartbeat_interval_ms = 50;

    RoverNodeConfig c2;
    c2.rover_id = 2;
    c2.listen_port = 20602;
    c2.listen_ip = "127.0.0.1";
    c2.seed_peers = { Endpoint("127.0.0.1", 20601) };
    c2.heartbeat_interval_ms = 50;

    RoverNode node1(c1);
    RoverNode node2(c2);

    assert(node1.start());
    assert(node2.start());

    DistributedReconciliationAgent agent1(1, node1, 32, 3);
    DistributedReconciliationAgent agent2(2, node2, 32, 3);

    agent1.setup_callbacks();
    agent2.setup_callbacks();

    // Exchange heartbeats for discovery
    for (int i = 0; i < 5; ++i) {
        node1.poll_events(0);
        node2.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }

    // Configure Rover 1: Keyframe 0 at (0, 0, 0)
    // Observes landmarks 1, 2, 3, 4, 5
    // Landmarks 3, 4, 5 are shared with Rover 2!
    std::vector<LandmarkObservationRecord> obs1 = {
        { LandmarkKey(1, 1, 1, 0x11111111ULL),  5.0,  0.0, 0 },
        { LandmarkKey(1, 1, 2, 0x22222222ULL),  6.0,  1.0, 0 },
        { LandmarkKey(1, 1, 3, 0x33333333ULL),  2.0,  3.0, 0 }, // Shared
        { LandmarkKey(1, 1, 4, 0x44444444ULL),  4.0,  1.0, 0 }, // Shared
        { LandmarkKey(1, 1, 5, 0x55555555ULL),  3.0, -2.0, 0 }  // Shared
    };
    agent1.add_local_keyframe(0, SE2(0.0, 0.0, 0.0), obs1);

    // Configure Rover 2: Keyframe 0 at (10, 0, 0)
    // In Rover 2's frame (translated by +10 in X), the shared landmarks appear at:
    // LM 3: (2.0 - 10, 3.0) = (-8.0, 3.0)
    // LM 4: (4.0 - 10, 1.0) = (-6.0, 1.0)
    // LM 5: (3.0 - 10, -2.0) = (-7.0, -2.0)
    std::vector<LandmarkObservationRecord> obs2 = {
        { LandmarkKey(1, 1, 3, 0x33333333ULL), -8.0,  3.0, 0 }, // Shared
        { LandmarkKey(1, 1, 4, 0x44444444ULL), -6.0,  1.0, 0 }, // Shared
        { LandmarkKey(1, 1, 5, 0x55555555ULL), -7.0, -2.0, 0 }, // Shared
        { LandmarkKey(1, 2, 6, 0x66666666ULL),  1.0,  2.0, 0 },
        { LandmarkKey(1, 2, 7, 0x77777777ULL),  2.0, -1.0, 0 }
    };
    agent2.add_local_keyframe(0, SE2(10.0, 0.0, 0.0), obs2);

    // Initial DSU state: each rover has 1 isolated component
    assert(agent1.get_dsu_component_count() == 1);
    assert(agent2.get_dsu_component_count() == 1);

    // Rover 1 initiates IBLT synchronization with Rover 2
    uint64_t sync_id = agent1.initiate_sync_with_peer(2);
    assert(sync_id > 0);

    // Event loop:
    // 1. Rover 1 sends IBLT Sync -> Rover 2 receives & decodes symmetric diff
    // 2. Rover 2 sends CandidateProposal -> Rover 1 receives & runs RANSAC
    // 3. Rover 1 verifies loop closure -> unites DSU -> sends CandidateResponse & BoundaryState
    // 4. Rover 2 receives CandidateResponse -> unites DSU -> sends BoundaryState
    for (int step = 0; step < 25; ++step) {
        node1.poll_events(0);
        node2.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(15));

        if (agent1.get_stats().loop_closures_verified >= 1 &&
            agent2.get_stats().loop_closures_verified >= 1) {
            break;
        }
    }

    // Verify verification and DSU union on both sides!
    std::cout << "  Rover 1 sync sent: " << agent1.get_stats().sync_sent
              << " | Rover 2 sync received: " << agent2.get_stats().sync_received << std::endl;
    std::cout << "  Rover 2 decodes succ: " << agent2.get_stats().sync_decodes_succeeded
              << " | decodes failed: " << agent2.get_stats().sync_decodes_failed << std::endl;
    std::cout << "  Rover 2 proposals sent: " << agent2.get_stats().proposals_sent
              << " | Rover 1 proposals received: " << agent1.get_stats().proposals_received << std::endl;
    std::cout << "  Rover 1 loop closures verified: " << agent1.get_stats().loop_closures_verified << std::endl;
    std::cout << "  Rover 2 loop closures verified: " << agent2.get_stats().loop_closures_verified << std::endl;
    std::cout << "  Rover 1 DSU unions: " << agent1.get_stats().dsu_unions_performed << std::endl;
    std::cout << "  Rover 2 DSU unions: " << agent2.get_stats().dsu_unions_performed << std::endl;

    assert(agent1.get_stats().loop_closures_verified >= 1);
    assert(agent2.get_stats().loop_closures_verified >= 1);
    assert(agent1.get_stats().dsu_unions_performed >= 1);
    assert(agent2.get_stats().dsu_unions_performed >= 1);

    // Check that DSU connected components united both rovers' nodes into the same cluster!
    const NodeId n1(1, 0);
    const NodeId n2(2, 0);
    assert(agent1.get_partition_manager().are_connected(n1, n2));
    assert(agent2.get_partition_manager().are_connected(n2, n1));

    // Verify boundary states were exchanged
    assert(agent1.get_stats().boundary_states_sent >= 1);
    assert(agent2.get_stats().boundary_states_received >= 1);

    node1.stop();
    node2.stop();
    std::cout << "  -> Passed." << std::endl;
}

void test_perceptual_aliasing_rejection_over_udp() {
    std::cout << "[TEST] Running test_perceptual_aliasing_rejection_over_udp..." << std::endl;

    RoverNodeConfig c1;
    c1.rover_id = 1;
    c1.listen_port = 20611;
    c1.listen_ip = "127.0.0.1";
    c1.seed_peers = { Endpoint("127.0.0.1", 20612) };

    RoverNodeConfig c2;
    c2.rover_id = 2;
    c2.listen_port = 20612;
    c2.listen_ip = "127.0.0.1";
    c2.seed_peers = { Endpoint("127.0.0.1", 20611) };

    RoverNode node1(c1);
    RoverNode node2(c2);

    assert(node1.start());
    assert(node2.start());

    DistributedReconciliationAgent agent1(1, node1, 32, 3);
    DistributedReconciliationAgent agent2(2, node2, 32, 3);

    agent1.setup_callbacks();
    agent2.setup_callbacks();

    for (int i = 0; i < 5; ++i) {
        node1.poll_events(0);
        node2.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }

    // Rover 1 observes landmarks 1, 2, 3 at standard geometry
    std::vector<LandmarkObservationRecord> obs1 = {
        { LandmarkKey(1, 1, 1, 0x11111111ULL),  2.0,  3.0, 0 },
        { LandmarkKey(1, 1, 2, 0x22222222ULL),  4.0,  1.0, 0 },
        { LandmarkKey(1, 1, 3, 0x33333333ULL),  3.0, -2.0, 0 }
    };
    agent1.add_local_keyframe(0, SE2(0.0, 0.0, 0.0), obs1);

    // Rover 2 observes same keys, BUT with SEVERELY CORRUPTED NON-RIGID GEOMETRY (perceptual aliasing)
    std::vector<LandmarkObservationRecord> obs2 = {
        { LandmarkKey(1, 1, 1, 0x11111111ULL),  50.0, -20.0, 0 },
        { LandmarkKey(1, 1, 2, 0x22222222ULL), -10.0,  80.0, 0 },
        { LandmarkKey(1, 1, 3, 0x33333333ULL),   0.0,   0.0, 0 }
    };
    agent2.add_local_keyframe(0, SE2(10.0, 0.0, 0.0), obs2);

    // Rover 1 initiates sync
    agent1.initiate_sync_with_peer(2);

    for (int step = 0; step < 15; ++step) {
        node1.poll_events(0);
        node2.poll_events(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }

    // Verify rejection: RANSAC must reject invalid non-rigid geometry
    assert(agent1.get_stats().loop_closures_verified == 0);
    assert(agent1.get_stats().loop_closures_rejected >= 1);
    assert(agent1.get_stats().dsu_unions_performed == 0);

    const NodeId n1(1, 0);
    const NodeId n2(2, 0);
    assert(!agent1.get_partition_manager().are_connected(n1, n2));

    node1.stop();
    node2.stop();
    std::cout << "  -> Passed." << std::endl;
}

int main() {
    std::cout << "=================================================================" << std::endl;
    std::cout << "  AstroMesh-X: Distributed Reconciliation & SLAM over UDP Tests  " << std::endl;
    std::cout << "=================================================================" << std::endl;

    test_iblt_serialization_roundtrip();
    test_candidate_payload_serialization();
    test_distributed_reconciliation_over_udp();
    test_perceptual_aliasing_rejection_over_udp();

    std::cout << "=================================================================" << std::endl;
    std::cout << "  All Distributed Reconciliation Tests PASSED successfully!      " << std::endl;
    std::cout << "=================================================================" << std::endl;
    return 0;
}
