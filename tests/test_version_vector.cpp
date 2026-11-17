#include "astromesh/core/version_vector.hpp"
#include "astromesh/graph/map_update.hpp"

#include <cassert>
#include <iostream>

using namespace astromesh::core;
using namespace astromesh::graph;
using namespace astromesh::geometry;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "Assertion failed: " << msg << " at line " << __LINE__ << std::endl; \
            return 1; \
        } \
    } while (0)

int main() {
    std::cout << "=== ASTRAMESH-X M7.12 VERSION VECTOR & MAP UPDATE TEST SUITE ===\n";

    // 1. Initialization
    VersionVector v0;
    TEST_ASSERT(v0.get(1) == 0, "Default version for Rover 1 must be 0");
    TEST_ASSERT(v0.get(16) == 0, "Default version for Rover 16 must be 0");
    TEST_ASSERT(v0.get(17) == 0, "Out-of-bound rover must return 0");

    // 2. Monotonic Increments
    VersionVector v1;
    v1.increment(1);
    v1.increment(1);
    v1.increment(2);
    TEST_ASSERT(v1.get(1) == 2, "Rover 1 sequence must be 2");
    TEST_ASSERT(v1.get(2) == 1, "Rover 2 sequence must be 1");
    TEST_ASSERT(v1.get(3) == 0, "Rover 3 sequence must be 0");

    // 3. Dominance and Strict Precedence
    VersionVector v_older;
    v_older.set(1, 2);
    v_older.set(2, 1);

    VersionVector v_newer;
    v_newer.set(1, 3);
    v_newer.set(2, 1);

    TEST_ASSERT(v_newer.dominates_or_equal(v_older), "v_newer must dominate v_older");
    TEST_ASSERT(v_newer.strictly_dominates(v_older), "v_newer must strictly dominate v_older");
    TEST_ASSERT(!v_older.strictly_dominates(v_newer), "v_older cannot dominate v_newer");
    TEST_ASSERT(v_older.dominates_or_equal(v_older), "Self dominance must be true");

    // 4. Concurrency Detection (Partitions)
    VersionVector v_branch_a; // Rover 1 updated locally while partitioned
    v_branch_a.set(1, 3);
    v_branch_a.set(2, 1);

    VersionVector v_branch_b; // Rover 2 updated locally while partitioned
    v_branch_b.set(1, 2);
    v_branch_b.set(2, 2);

    TEST_ASSERT(v_branch_a.is_concurrent_with(v_branch_b), "Branch A and B must be concurrent");
    TEST_ASSERT(!v_branch_a.dominates_or_equal(v_branch_b), "Branch A cannot dominate B");
    TEST_ASSERT(!v_branch_b.dominates_or_equal(v_branch_a), "Branch B cannot dominate A");

    // 5. Merge-Max Causal Reconciliation
    VersionVector v_merged = v_branch_a.merge_max(v_branch_b);
    TEST_ASSERT(v_merged.get(1) == 3, "Merged R1 must be max(3, 2) = 3");
    TEST_ASSERT(v_merged.get(2) == 2, "Merged R2 must be max(1, 2) = 2");
    TEST_ASSERT(v_merged.dominates_or_equal(v_branch_a), "Merged must dominate Branch A");
    TEST_ASSERT(v_merged.dominates_or_equal(v_branch_b), "Merged must dominate Branch B");

    // 6. Serialization / Deserialization Round-Trip
    auto bytes = v_merged.serialize();
    TEST_ASSERT(bytes.size() == VersionVector::WIRE_SIZE, "Serialized size matches WIRE_SIZE");

    auto des_opt = VersionVector::deserialize(bytes.data(), bytes.size());
    TEST_ASSERT(des_opt.has_value(), "Deserialization must succeed");
    TEST_ASSERT(*des_opt == v_merged, "Deserialized vector matches original");

    // Corrupted payload rejection
    bytes[0] = 0x00; // corrupt magic
    auto corrupted_opt = VersionVector::deserialize(bytes.data(), bytes.size());
    TEST_ASSERT(!corrupted_opt.has_value(), "Corrupted magic must be rejected");

    // 7. MapUpdate Creation & Serialization
    MapUpdate kf_update = MapUpdate::make_keyframe(1, 10, 0, SE2(1.5, 2.5, 0.785), v_merged);
    TEST_ASSERT(kf_update.update_id == ((1ULL << 48) | 10ULL), "UpdateID packing exact");
    TEST_ASSERT(kf_update.node_id == NodeId(1, 0), "NodeId matches");
    TEST_ASSERT(kf_update.type == MapUpdateType::KeyframeAdded, "Type is KeyframeAdded");

    auto u_bytes = kf_update.serialize();
    auto u_des_opt = MapUpdate::deserialize(u_bytes.data(), u_bytes.size());
    TEST_ASSERT(u_des_opt.has_value(), "MapUpdate deserialization succeeds");
    TEST_ASSERT(u_des_opt->origin_rover_id == 1, "Origin rover matches");
    TEST_ASSERT(u_des_opt->sequence == 10, "Sequence matches");
    TEST_ASSERT(u_des_opt->transform.x() == 1.5, "SE2 X matches");
    TEST_ASSERT(u_des_opt->causal_context == v_merged, "Causal context vector matches");

    // 8. Landmark Observation Update
    LandmarkKey lk(1, 1, 42, 0xABCDEF1234567890ULL);
    MapUpdate lm_update = MapUpdate::make_landmark_obs(2, 5, 1, lk, 3.14, -2.71, v_merged);
    auto lm_bytes = lm_update.serialize();
    auto lm_des = MapUpdate::deserialize(lm_bytes.data(), lm_bytes.size());
    TEST_ASSERT(lm_des.has_value(), "Landmark update deserializes");
    TEST_ASSERT(lm_des->landmark_key == lk, "Landmark key preserved");
    TEST_ASSERT(lm_des->landmark_local_x == 3.14, "Local X preserved");

    // 9. Edge Verified Update
    MapUpdate edge_update = MapUpdate::make_edge_verified(
        1, 11, NodeId(1, 0), NodeId(2, 0), SE2(10.0, 0.0, 0.0), 5, 0.08, EdgeType::InterRoverLoopClosure, v_merged);
    auto e_bytes = edge_update.serialize();
    auto e_des = MapUpdate::deserialize(e_bytes.data(), e_bytes.size());
    TEST_ASSERT(e_des.has_value(), "Edge update deserializes");
    TEST_ASSERT(e_des->inlier_count == 5, "Inliers preserved");
    TEST_ASSERT(e_des->inlier_rmse == 0.08, "RMSE preserved");
    TEST_ASSERT(e_des->edge_type == EdgeType::InterRoverLoopClosure, "Edge type preserved");

    std::cout << "  ALL VERSION VECTOR & MAP UPDATE TESTS PASSED (100%)\n";
    return 0;
}

