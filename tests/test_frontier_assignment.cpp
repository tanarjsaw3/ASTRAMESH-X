#include "astromesh/exploration/multi_rover_coordinator.hpp"

#include <cassert>
#include <iostream>

using namespace astromesh;
using namespace astromesh::verification;
using namespace astromesh::exploration;

void test_claim_wire_serialization() {
    std::cout << "[RUN] test_claim_wire_serialization...\n";

    FrontierClaim claim;
    claim.frontier_id = 0x12345678ULL;
    claim.rover_id = 2;
    claim.target_x = 14.5;
    claim.target_y = -8.2;
    claim.utility_score = 0.85;
    claim.lease_ttl_steps = 45;
    claim.epoch = 3;

    std::vector<uint8_t> buf;
    claim.serialize(buf);
    assert(buf.size() == 44);

    FrontierClaim deserialized;
    assert(FrontierClaim::deserialize(buf.data(), buf.size(), deserialized));
    assert(deserialized.frontier_id == claim.frontier_id);
    assert(deserialized.rover_id == 2);
    assert(deserialized.target_x == 14.5);
    assert(deserialized.target_y == -8.2);
    assert(deserialized.utility_score == 0.85);
    assert(deserialized.lease_ttl_steps == 45);
    assert(deserialized.epoch == 3);

    std::cout << "  -> PASSED: 44-byte FrontierClaim serialization verified.\n";
}

void test_deterministic_conflict_dominance() {
    std::cout << "[RUN] test_deterministic_conflict_dominance...\n";

    // Rule 1: Higher utility dominates
    assert(MultiRoverExplorationCoordinator::candidate_dominates(0.9, 1, 2, 0.7, 1, 1));
    assert(!MultiRoverExplorationCoordinator::candidate_dominates(0.7, 1, 1, 0.9, 1, 2));

    // Rule 2: Equal utility -> earlier epoch dominates
    assert(MultiRoverExplorationCoordinator::candidate_dominates(0.8, 1, 2, 0.8, 2, 1));
    assert(!MultiRoverExplorationCoordinator::candidate_dominates(0.8, 2, 1, 0.8, 1, 2));

    // Rule 3: Equal utility and epoch -> lower rover ID dominates
    assert(MultiRoverExplorationCoordinator::candidate_dominates(0.8, 1, 1, 0.8, 1, 2));
    assert(!MultiRoverExplorationCoordinator::candidate_dominates(0.8, 1, 2, 0.8, 1, 1));

    std::cout << "  -> PASSED: All 3 dominance tie-breaking rules verified.\n";
}

void test_lease_management_and_expiration() {
    std::cout << "[RUN] test_lease_management_and_expiration...\n";

    MultiRoverExplorationCoordinator coord(1);

    // Create local claim at step 10 with TTL 20 (expires at 30)
    auto local_c = coord.create_local_claim(100, Point2D(5.0, 5.0), 0.75, 10, 20);
    assert(coord.is_local_claim_active(15));
    assert(coord.is_local_claim_active(29));
    assert(!coord.is_local_claim_active(30)); // Expired

    // Ingest remote claim from Rover 2 on frontier 200
    FrontierClaim remote_c;
    remote_c.frontier_id = 200;
    remote_c.rover_id = 2;
    remote_c.target_x = 10.0;
    remote_c.target_y = 10.0;
    remote_c.utility_score = 0.80;
    remote_c.lease_ttl_steps = 15;
    remote_c.epoch = 1;

    assert(coord.process_remote_claim(remote_c, 10));
    assert(coord.active_peer_targets(15).size() == 1);

    // Check conflict detection within 8m
    uint16_t conflicting_peer = 0;
    assert(coord.has_peer_conflict(Point2D(12.0, 10.0), 0.70, 15, &conflicting_peer));
    assert(conflicting_peer == 2);

    // Prune at step 50 -> all expired
    size_t pruned = coord.prune_expired_claims(50);
    assert(pruned >= 2);
    assert(coord.active_peer_targets(50).empty());

    std::cout << "  -> PASSED: Lease management, conflict check, and expiration verified.\n";
}

int main() {
    std::cout << "==================================================\n";
    std::cout << " ASTROMESH-X M7.16: FRONTIER ASSIGNMENT TESTS\n";
    std::cout << "==================================================\n";

    test_claim_wire_serialization();
    test_deterministic_conflict_dominance();
    test_lease_management_and_expiration();

    std::cout << "==================================================\n";
    std::cout << " ALL FRONTIER ASSIGNMENT TESTS PASSED (100%)\n";
    std::cout << "==================================================\n";
    return 0;
}
