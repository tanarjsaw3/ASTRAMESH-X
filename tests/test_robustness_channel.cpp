#include "astromesh/simulation/pipeline_astromesh_x.hpp"
#include "astromesh/simulation/baseline_astromesh_v1.hpp"
#include "astromesh/simulation/synthetic_world.hpp"
#include "astromesh/simulation/observation_model.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using namespace astromesh;
using namespace astromesh::simulation;

void test_byte_conservation_under_loss_and_delay() {
    std::cout << "[TEST] Running test_byte_conservation_under_loss_and_delay..." << std::endl;

    for (double loss : {0.0, 0.2, 0.5, 0.8}) {
        for (uint64_t delay_ns : {0ULL, 50'000'000ULL, 200'000'000ULL}) {
            AstroMeshXConfig cfg;
            cfg.seed = 12345;
            cfg.default_one_way_delay_ns = delay_ns;
            cfg.packet_loss_rate = loss;
            cfg.enable_cord = false; // Fast test

            AstroMeshXPipeline pipeline(cfg);

            // Generate two steps with 2 rovers observing shared landmarks
            SyntheticWorld2D world(WorldConfig{20, 25.0, 30.0, 42});
            ObservationModel2D obs_model(ObservationNoiseConfig{0.01, 0.005, 42});

            geometry::SE2 pose1(0.0, 0.0, 0.0);
            geometry::SE2 pose2(2.0, 0.0, 0.0);

            SimulationStepInput step0;
            step0.timestamp_ns = 0;
            RoverStepData r1_0{1, 0, geometry::SE2::identity(), obs_model.observe(world, pose1, 1)};
            RoverStepData r2_0{2, 0, geometry::SE2::identity(), obs_model.observe(world, pose2, 2)};
            step0.rover_steps = {r1_0, r2_0};
            step0.channel_states.push_back({1, 2, true, false, loss, delay_ns});

            SimulationStepInput step1;
            step1.timestamp_ns = 200'000'000ULL;
            RoverStepData r1_1{1, 200'000'000ULL, geometry::SE2(0.5, 0.0, 0.0), obs_model.observe(world, geometry::SE2(0.5, 0.0, 0.0), 1)};
            RoverStepData r2_1{2, 200'000'000ULL, geometry::SE2(0.5, 0.0, 0.0), obs_model.observe(world, geometry::SE2(2.5, 0.0, 0.0), 2)};
            step1.rover_steps = {r1_1, r2_1};
            step1.channel_states.push_back({1, 2, true, false, loss, delay_ns});

            assert(pipeline.ingest_step(step0));
            assert(pipeline.ingest_step(step1));

            auto m = pipeline.collect_metrics();

            // Strict byte conservation
            const size_t expected_data_bytes = m.payload_bytes_transmitted + m.protocol_bytes_transmitted + m.framing_bytes_transmitted;
            assert(m.data_wire_bytes_transmitted == expected_data_bytes);

            const size_t expected_total_bytes = m.data_wire_bytes_transmitted + m.ack_wire_bytes_transmitted;
            assert(m.total_wire_bytes_transmitted == expected_total_bytes);

            // Invariants on delivery ratios
            if (m.hop_transmissions_attempted > 0) {
                assert(m.per_hop_delivery_ratio >= 0.0 && m.per_hop_delivery_ratio <= 1.0);
            }
            if (m.logical_messages_sent > 0) {
                assert(m.application_completion_ratio >= 0.0 && m.application_completion_ratio <= 1.0);
            }
        }
    }
    std::cout << "  -> Passed." << std::endl;
}

void test_spurious_retries_under_delay() {
    std::cout << "[TEST] Running test_spurious_retries_under_delay..." << std::endl;

    // Test with delay = 0 ms (RTT = 0 ms < 50 ms): spurious retries should be 0
    {
        AstroMeshXConfig cfg;
        cfg.seed = 42;
        cfg.default_one_way_delay_ns = 0;
        cfg.packet_loss_rate = 0.0;
        cfg.enable_cord = false;

        AstroMeshXPipeline pipeline(cfg);

        SimulationStepInput step0;
        step0.timestamp_ns = 0;
        step0.rover_steps = {
            RoverStepData{1, 0, geometry::SE2::identity(), {}},
            RoverStepData{2, 0, geometry::SE2::identity(), {}}
        };
        step0.channel_states.push_back({1, 2, true, false, 0.0, 0});

        pipeline.ingest_step(step0);
        auto m = pipeline.collect_metrics();
        assert(m.retransmissions_performed == 0);
        assert(m.duplicate_deliveries == 0);
    }

    // Test with delay = 100 ms (RTT = 200 ms > 50 ms and > 150 ms): spurious retries should be 2
    {
        AstroMeshXConfig cfg;
        cfg.seed = 42;
        cfg.default_one_way_delay_ns = 100'000'000ULL;
        cfg.packet_loss_rate = 0.0;
        cfg.enable_cord = false;

        AstroMeshXPipeline pipeline(cfg);

        SimulationStepInput step0;
        step0.timestamp_ns = 0;
        step0.rover_steps = {
            RoverStepData{1, 0, geometry::SE2::identity(), {}},
            RoverStepData{2, 0, geometry::SE2::identity(), {}}
        };
        step0.channel_states.push_back({1, 2, true, false, 0.0, 100'000'000ULL});

        pipeline.ingest_step(step0);
        auto m = pipeline.collect_metrics();
        assert(m.retransmissions_performed == 2);
        assert(m.duplicate_deliveries == 2);
    }

    std::cout << "  -> Passed." << std::endl;
}

void test_in_flight_termination_gating() {
    std::cout << "[TEST] Running test_in_flight_termination_gating..." << std::endl;

    // Simulation duration is 1.0 s (1'000'000'000 ns).
    // If step is at 800 ms with delay = 350 ms: delivery time = 1150 ms > 1000 ms!
    // It should be gated as in_flight_at_termination.
    AstroMeshXConfig cfg;
    cfg.seed = 42;
    cfg.simulation_duration_ns = 1'000'000'000ULL;
    cfg.default_one_way_delay_ns = 350'000'000ULL;
    cfg.packet_loss_rate = 0.0;
    cfg.enable_cord = false;

    AstroMeshXPipeline pipeline(cfg);

    SimulationStepInput step;
    step.timestamp_ns = 800'000'000ULL;
    step.rover_steps = {
        RoverStepData{1, 800'000'000ULL, geometry::SE2::identity(), {}},
        RoverStepData{2, 800'000'000ULL, geometry::SE2::identity(), {}}
    };
    step.channel_states.push_back({1, 2, true, false, 0.0, 350'000'000ULL});

    pipeline.ingest_step(step);
    auto m = pipeline.collect_metrics();
    assert(m.in_flight_at_termination == 1);
    assert(m.in_flight_messages == 1);
    assert(m.logical_messages_delivered == 0);

    std::cout << "  -> Passed." << std::endl;
}

int main() {
    std::cout << "=== AstroMesh-X Phase 6.2 Robustness Unit Tests ===" << std::endl;
    test_byte_conservation_under_loss_and_delay();
    test_spurious_retries_under_delay();
    test_in_flight_termination_gating();
    std::cout << "All Phase 6.2 Robustness unit tests passed successfully!" << std::endl;
    return 0;
}

