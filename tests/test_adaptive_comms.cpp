#include "astromesh/net/adaptive_comms.hpp"
#include <iostream>
#include <cassert>

using namespace astromesh::net;

void test_adaptive_comms_nominal() {
    std::cout << "[RUN] test_adaptive_comms_nominal...\n";
    AdaptiveCommsController controller;

    controller.observe_sample(0.01, 5.0, 0.10);
    assert(controller.regime() == NetworkHealthRegime::Nominal);
    assert(controller.current_interval_ms() == 20);
    assert(controller.current_batch_size() == 8);
    assert(controller.current_iblt_cells() == 32);

    std::cout << "  -> PASSED: Nominal regime verified (20ms interval, 8 batch, 32 IBLT cells).\n";
}

void test_adaptive_comms_congested() {
    std::cout << "[RUN] test_adaptive_comms_congested...\n";
    AdaptiveCommsController controller;

    // Simulate rising loss to 20%
    for (int i = 0; i < 15; ++i) {
        controller.observe_sample(0.20, 35.0, 0.30);
    }

    assert(controller.regime() == NetworkHealthRegime::Congested);
    assert(controller.current_interval_ms() == 50);
    assert(controller.current_batch_size() == 4);
    assert(controller.current_iblt_cells() == 64);

    std::cout << "  -> PASSED: Congested regime verified (50ms interval, 4 batch, 64 IBLT cells).\n";
}

void test_adaptive_comms_impaired() {
    std::cout << "[RUN] test_adaptive_comms_impaired...\n";
    AdaptiveCommsController controller;

    // Severe queue backlog and 40% loss
    for (int i = 0; i < 20; ++i) {
        controller.observe_sample(0.40, 150.0, 0.85);
    }

    assert(controller.regime() == NetworkHealthRegime::Impaired);
    assert(controller.current_interval_ms() == 200);
    assert(controller.current_batch_size() == 2);
    assert(controller.current_iblt_cells() == 64);

    // Channel heals back to clean
    for (int i = 0; i < 30; ++i) {
        controller.observe_sample(0.0, 5.0, 0.05);
    }

    assert(controller.regime() == NetworkHealthRegime::Nominal);
    assert(controller.current_interval_ms() == 20);
    assert(controller.current_batch_size() == 8);

    std::cout << "  -> PASSED: Impaired regime and dynamic recovery verified.\n";
}

int main() {
    std::cout << "==================================================\n";
    std::cout << " ASTROMESH-X M7.12: ADAPTIVE COMMS UNIT TESTS\n";
    std::cout << "==================================================\n";

    test_adaptive_comms_nominal();
    test_adaptive_comms_congested();
    test_adaptive_comms_impaired();

    std::cout << "==================================================\n";
    std::cout << " ALL ADAPTIVE COMMS TESTS PASSED (100%)\n";
    std::cout << "==================================================\n";
    return 0;
}

