#include "astromesh/memory/memory_budget_controller.hpp"

#include <cassert>
#include <iostream>

using namespace astromesh::memory;

void test_budget_transitions_and_hysteresis() {
    std::cout << "[RUN] test_budget_transitions_and_hysteresis..." << std::endl;

    MemoryBudgetConfig cfg;
    cfg.per_rover_budget_bytes = 1000; // 1000 bytes budget for simple arithmetic
    cfg.warning_threshold_ratio = 0.75; // 750 B
    cfg.critical_threshold_ratio = 0.90;// 900 B
    cfg.recovery_threshold_ratio = 0.65;// 650 B
    cfg.reserved_critical_ratio = 0.20; // 200 B reserved

    MemoryBudgetController controller(cfg);

    // Initial check at 500 B (50%) -> Normal
    auto p1 = controller.evaluate_pressure(500, 100);
    assert(p1 == MemoryPressureLevel::Normal);
    assert(!controller.is_budget_infeasible());

    // Usage jumps to 780 B (78%) -> Warning
    auto p2 = controller.evaluate_pressure(780, 100);
    assert(p2 == MemoryPressureLevel::Warning);

    // Usage jumps to 920 B (92%) -> Critical
    auto p3 = controller.evaluate_pressure(920, 100);
    assert(p3 == MemoryPressureLevel::Critical);

    // Hysteresis test: usage drops to 700 B (70%)
    // Since 700 B is above recovery_threshold (650 B), it should stay in Warning, NOT jump back to Normal!
    auto p4 = controller.evaluate_pressure(700, 100);
    assert(p4 == MemoryPressureLevel::Warning);

    // Usage drops to 600 B (60%, below 650 B) -> returns to Normal
    auto p5 = controller.evaluate_pressure(600, 100);
    assert(p5 == MemoryPressureLevel::Normal);

    std::cout << "[PASS] test_budget_transitions_and_hysteresis" << std::endl;
}

void test_budget_infeasibility_detection() {
    std::cout << "[RUN] test_budget_infeasibility_detection..." << std::endl;

    MemoryBudgetConfig cfg;
    cfg.per_rover_budget_bytes = 1000;
    cfg.reserved_critical_ratio = 0.20; // 800 B usable headroom

    MemoryBudgetController controller(cfg);

    // Indispensable footprint = 850 B > 800 B usable headroom
    controller.evaluate_pressure(950, 850);
    assert(controller.is_budget_infeasible());
    assert(controller.metrics().infeasible_budget_events > 0);

    // Indispensable footprint drops back to 400 B -> feasible
    controller.evaluate_pressure(500, 400);
    assert(!controller.is_budget_infeasible());

    std::cout << "[PASS] test_budget_infeasibility_detection" << std::endl;
}

int main() {
    std::cout << "==================================================" << std::endl;
    std::cout << " AstroMesh-X: Memory Budget Controller Unit Tests" << std::endl;
    std::cout << "==================================================" << std::endl;

    test_budget_transitions_and_hysteresis();
    test_budget_infeasibility_detection();

    std::cout << "All Memory Budget Controller tests PASSED." << std::endl;
    return 0;
}

