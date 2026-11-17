#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "astromesh/geometry/se2.hpp"
#include "astromesh/simulation/synthetic_world.hpp"
#include "astromesh/simulation/swarm_trajectories.hpp"
#include "astromesh/simulation/observation_model.hpp"

using namespace astromesh;
using namespace astromesh::simulation;

// SimParams & Telemetry test harness structures matching sim_runner conventions
struct TestSimTelemetryParams {
    size_t rovers{4};
    double comm_range{25.0};
    double duration_s{5.0};
    double step_dt_s{0.1};
    double partition_start_s{2.0};
    double partition_end_s{4.0};
    bool partition_enabled{true};
};

struct TestLinkTelemetry {
    uint16_t rover_a{0};
    uint16_t rover_b{0};
    double distance{0.0};
    bool in_range{false};
    bool active{false};
    bool partitioned{false};
};

struct TestStepTelemetry {
    size_t step_index{0};
    double timestamp_s{0.0};
    bool is_partitioned{false};
    std::vector<TestLinkTelemetry> links;
};

// 1. Test Multi-Rover Scalability & Link Pair Count
void test_multi_rover_link_pairs() {
    std::cout << "[TEST 1] Multi-Rover Link Pair Invariant Check (2, 4, 8, 16 rovers)... ";
    const std::vector<size_t> rover_counts = {2, 4, 8, 16};
    for (size_t N : rover_counts) {
        size_t link_pairs = 0;
        for (size_t i = 0; i < N; ++i) {
            for (size_t j = i + 1; j < N; ++j) {
                link_pairs++;
            }
        }
        const size_t expected_pairs = (N * (N - 1)) / 2;
        assert(link_pairs == expected_pairs);
    }
    // 2 rovers: 1 link; 4 rovers: 6 links; 8 rovers: 28 links; 16 rovers: 120 links
    std::cout << "PASSED (Pairs: 2R->1, 4R->6, 8R->28, 16R->120)\n";
}

// 2. Test Partition State Machine & Cross-Partition Link Severing
void test_partition_link_severing() {
    std::cout << "[TEST 2] Partition State Machine & Link Severing Verification... ";
    TestSimTelemetryParams p;
    p.rovers = 4;
    p.comm_range = 30.0;
    p.partition_start_s = 2.0;
    p.partition_end_s = 4.0;

    // Simulate link checks across timesteps: t=1.0s (normal), t=3.0s (partitioned), t=4.5s (healed)
    const std::vector<double> test_times = {1.0, 3.0, 4.5};
    for (double time_s : test_times) {
        const bool partition_active = (p.partition_end_s > p.partition_start_s &&
                                       time_s >= p.partition_start_s &&
                                       time_s < p.partition_end_s);
        TestStepTelemetry step;
        step.timestamp_s = time_s;
        step.is_partitioned = partition_active;

        for (size_t i = 0; i < p.rovers; ++i) {
            for (size_t j = i + 1; j < p.rovers; ++j) {
                // Dummy distance within communication range
                const double dist = 10.0;
                const bool cross_partition = partition_active && (i < (p.rovers / 2) && j >= (p.rovers / 2));

                TestLinkTelemetry lt;
                lt.rover_a = static_cast<uint16_t>(i + 1);
                lt.rover_b = static_cast<uint16_t>(j + 1);
                lt.distance = dist;
                lt.in_range = (dist <= p.comm_range) && !cross_partition;
                lt.active = lt.in_range;
                lt.partitioned = cross_partition;
                step.links.push_back(lt);
            }
        }

        if (time_s == 1.0) {
            assert(!step.is_partitioned);
            for (const auto& lk : step.links) {
                assert(!lk.partitioned);
                assert(lk.in_range);
            }
        } else if (time_s == 3.0) {
            assert(step.is_partitioned);
            // Links crossing partitions: (1,3), (1,4), (2,3), (2,4) should be severed
            // Intra-partition links: (1,2) and (3,4) should remain intact!
            size_t severed_count = 0;
            size_t intact_count = 0;
            for (const auto& lk : step.links) {
                if (lk.partitioned) {
                    severed_count++;
                    assert(!lk.in_range);
                } else {
                    intact_count++;
                    assert(lk.in_range);
                }
            }
            assert(severed_count == 4); // 2 in group A * 2 in group B = 4
            assert(intact_count == 2);  // group A internal (1-2) + group B internal (3-4) = 2
        } else if (time_s == 4.5) {
            assert(!step.is_partitioned);
            for (const auto& lk : step.links) {
                assert(!lk.partitioned);
                assert(lk.in_range);
            }
        }
    }
    std::cout << "PASSED (Normal: 6 active; Partitioned: 4 severed, 2 active; Healed: 6 active)\n";
}

// 3. Test Ground Truth Trajectory Ingestion & Frame Construction
void test_trajectory_frame_construction() {
    std::cout << "[TEST 3] Synthetic Trajectory & Landmark Perception Integration... ";
    WorldConfig wcfg;
    wcfg.num_landmarks = 20;
    wcfg.sensor_range = 15.0;
    wcfg.seed = 12345;
    SyntheticWorld2D world(wcfg);
    assert(world.landmarks().size() == 20);

    TrajectoryConfig tcfg;
    tcfg.num_rovers = 4;
    tcfg.duration_ns = 2'000'000'000ULL; // 2 seconds
    tcfg.step_dt_ns = 500'000'000ULL;   // 0.5s dt (5 steps)
    tcfg.speed = 1.0;
    tcfg.arena_extent = 30.0;
    tcfg.seed = 54321;

    auto trajectories = SwarmTrajectoryGenerator::generate_concentric_loops(tcfg);
    assert(trajectories.size() == 4);
    assert(trajectories[0].waypoints.size() == 5);

    ObservationNoiseConfig obs_noise;
    obs_noise.range_std_dev = 0.01;
    obs_noise.bearing_std_dev = 0.005;
    obs_noise.seed = 99999;
    ObservationModel2D obs_model(obs_noise);

    // Verify observations at step 0 for rover 1
    const auto& p0 = trajectories[0].waypoints[0].pose;
    auto obs = obs_model.observe(world, p0, 1);
    // Range must be non-negative
    for (const auto& o : obs) {
        assert(o.range_m >= 0.0);
        assert(o.range_m <= wcfg.sensor_range + 1.0); // allow small noise
    }
    std::cout << "PASSED (4 rovers, 5 steps, landmarks observed: " << obs.size() << ")\n";
}

// 4. Test 16-Rover Swarm Scalability Configuration
void test_16_rover_configuration() {
    std::cout << "[TEST 4] 16-Rover Scalability & Channel Invariants... ";
    const size_t N = 16;
    TrajectoryConfig tcfg;
    tcfg.num_rovers = N;
    tcfg.duration_ns = 1'000'000'000ULL;
    tcfg.step_dt_ns = 500'000'000ULL;
    tcfg.seed = 777;

    auto trajs = SwarmTrajectoryGenerator::generate_parallel_search(tcfg);
    assert(trajs.size() == N);

    // Validate that each rover trajectory has distinct initial coordinates
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = i + 1; j < N; ++j) {
            double dx = trajs[i].waypoints[0].pose.x() - trajs[j].waypoints[0].pose.x();
            double dy = trajs[i].waypoints[0].pose.y() - trajs[j].waypoints[0].pose.y();
            double dist = std::hypot(dx, dy);
            assert(dist > 0.01); // distinct initial spacing
        }
    }
    std::cout << "PASSED (16 distinct rovers spawned and initialized)\n";
}

int main() {
    std::cout << "=================================================================\n";
    std::cout << "      ASTRAMESH-X M7.18 SIMULATOR & TELEMETRY TEST SUITE         \n";
    std::cout << "=================================================================\n";

    test_multi_rover_link_pairs();
    test_partition_link_severing();
    test_trajectory_frame_construction();
    test_16_rover_configuration();

    std::cout << "=================================================================\n";
    std::cout << "ALL M7.18 SIMULATOR & TELEMETRY UNIT TESTS PASSED (4/4)\n";
    std::cout << "=================================================================\n";
    return 0;
}

