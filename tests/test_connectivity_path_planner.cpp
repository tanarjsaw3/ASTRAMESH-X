#include "astromesh/exploration/connectivity_path_planner.hpp"

#include <cassert>
#include <iostream>

using namespace astromesh;
using namespace astromesh::verification;
using namespace astromesh::exploration;
using namespace astromesh::memory;

void test_destination_selection_and_waypoints() {
    std::cout << "[RUN] test_destination_selection_and_waypoints...\n";

    ConnectivityPathPlanner planner(1);
    ExplorationContext ctx;
    ctx.rover_position = Point2D(0.0, 0.0);
    ctx.comm_range_meters = 15.0;
    ctx.active_peers = { Point2D(2.0, 0.0) };
    ctx.memory_pressure = MemoryPressureLevel::Normal;

    Frontier f1; f1.frontier_id = 1; f1.position = Point2D(5.0, 0.0); f1.status = FrontierStatus::Active;
    Frontier f2; f2.frontier_id = 2; f2.position = Point2D(20.0, 0.0); f2.status = FrontierStatus::Active;

    auto plan = planner.plan_destination(ctx, {f1, f2}, {}, 0);
    assert(plan.status == PlannerStatus::Success);
    assert(plan.target_frontier_id == 1);
    assert(!plan.waypoints.empty());
    assert(planner.committed_frontier_id() == 1);

    std::cout << "  -> PASSED: Destination selection and waypoint generation verified.\n";
}

void test_commitment_hysteresis() {
    std::cout << "[RUN] test_commitment_hysteresis...\n";

    ConnectivityPlannerConfig cfg;
    cfg.switching_hysteresis_bonus = 0.20;
    ConnectivityPathPlanner planner(1, cfg);

    ExplorationContext ctx;
    ctx.rover_position = Point2D(0.0, 0.0);
    ctx.comm_range_meters = 20.0;
    ctx.active_peers = { Point2D(0.0, 0.0) };
    ctx.memory_pressure = MemoryPressureLevel::Normal;

    Frontier f1; f1.frontier_id = 101; f1.position = Point2D(6.0, 0.0); f1.status = FrontierStatus::Active;
    Frontier f2; f2.frontier_id = 102; f2.position = Point2D(6.2, 0.0); f2.status = FrontierStatus::Active;

    // First planning
    auto plan1 = planner.plan_destination(ctx, {f1, f2}, {}, 0);
    std::cout << "plan1 target: " << plan1.target_frontier_id << " (util: " << plan1.utility_score << ")" << std::endl;
    uint64_t initial_committed = plan1.target_frontier_id;
    assert(initial_committed == 101 || initial_committed == 102);

    // Minor rover displacement (0.2m): hysteresis prevents switching
    ctx.rover_position = Point2D(0.2, 0.0);
    auto plan2 = planner.plan_destination(ctx, {f1, f2}, {}, 1);
    std::cout << "plan2 target: " << plan2.target_frontier_id << " (util: " << plan2.utility_score << ")" << std::endl;
    assert(plan2.target_frontier_id == initial_committed); // Maintained commitment!

    std::cout << "  -> PASSED: Anti-oscillation commitment hysteresis verified.\n";
}

void test_return_to_swarm_under_critical_pressure() {
    std::cout << "[RUN] test_return_to_swarm_under_critical_pressure...\n";

    ConnectivityPathPlanner planner(1);
    ExplorationContext ctx;
    ctx.rover_position = Point2D(15.0, 10.0);
    ctx.comm_range_meters = 15.0;
    ctx.active_peers = { Point2D(0.0, 0.0) };
    ctx.elected_anchor_position = Point2D(0.0, 0.0);
    ctx.memory_pressure = MemoryPressureLevel::Critical; // CRITICAL PRESSURE

    Frontier f1; f1.frontier_id = 1; f1.position = Point2D(25.0, 15.0); f1.status = FrontierStatus::Active;

    auto plan = planner.plan_destination(ctx, {f1}, {}, 0);
    assert(plan.status == PlannerStatus::ReturnToSwarm);
    assert(plan.target_frontier_id == 0);
    assert(plan.target_position.x == 0.0 && plan.target_position.y == 0.0); // Returning toward peer
    assert(plan.is_fallback == true);

    std::cout << "  -> PASSED: Return-to-Swarm fallback under critical pressure verified.\n";
}

int main() {
    std::cout << "==================================================\n";
    std::cout << " ASTROMESH-X M7.16: CONNECTIVITY PLANNER TESTS\n";
    std::cout << "==================================================\n";

    test_destination_selection_and_waypoints();
    test_commitment_hysteresis();
    test_return_to_swarm_under_critical_pressure();

    std::cout << "==================================================\n";
    std::cout << " ALL CONNECTIVITY PLANNER TESTS PASSED (100%)\n";
    std::cout << "==================================================\n";
    return 0;
}
