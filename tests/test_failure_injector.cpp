#include "astromesh/simulation/failure_injector.hpp"
#include <iostream>
#include <cassert>

using namespace astromesh::simulation;

void test_scheduled_partition() {
    std::cout << "[RUN] test_scheduled_partition...\n";
    FailureInjector injector(12345);

    // Link (1, 2) is partitioned from 1000ms to 3000ms
    injector.schedule_partition(1, 2, 1000, 3000);

    // Before partition
    assert(!injector.is_link_partitioned(1, 2, 500));
    assert(!injector.is_link_partitioned(2, 1, 500));

    // During partition
    assert(injector.is_link_partitioned(1, 2, 1500));
    assert(injector.is_link_partitioned(2, 1, 2500));
    assert(injector.should_drop_packet(1, 2, 2000));

    // After partition recovery
    assert(!injector.is_link_partitioned(1, 2, 3500));
    assert(!injector.is_link_partitioned(2, 1, 3500));

    std::cout << "  -> PASSED: Scheduled partition timing verified.\n";
}

void test_node_crash_and_restart() {
    std::cout << "[RUN] test_node_crash_and_restart...\n";
    FailureInjector injector(54321);

    // Rover 3 crashes at 2000ms, restarts at 4000ms
    injector.schedule_node_crash(3, 2000, 4000);

    assert(!injector.is_node_crashed(3, 1000));
    assert(injector.is_node_crashed(3, 2500));
    assert(injector.is_node_crashed(3, 3999));
    assert(!injector.is_node_crashed(3, 4001));

    // Any packet to or from rover 3 while crashed must be dropped
    assert(injector.should_drop_packet(1, 3, 3000));
    assert(injector.should_drop_packet(3, 1, 3000));

    std::cout << "  -> PASSED: Node crash and restart lifecycle verified.\n";
}

void test_gilbert_elliott_loss() {
    std::cout << "[RUN] test_gilbert_elliott_loss...\n";
    FailureInjector injector(9999);

    GilbertElliottParams params;
    params.p_g_loss = 0.0; // 0% loss in good state
    params.p_b_loss = 1.0; // 100% loss in bad state
    params.p_gb = 0.10;
    params.p_bg = 0.10;
    injector.set_gilbert_elliott_params(params);

    int drops = 0;
    constexpr int TRIALS = 1000;
    for (int i = 0; i < TRIALS; ++i) {
        if (injector.should_drop_packet(1, 2, 0)) {
            drops++;
        }
    }

    std::cout << "  -> Total drops over " << TRIALS << " trials: " << drops
              << " (" << (drops * 100.0 / TRIALS) << "%)\n";
    assert(drops > 50 && drops < 950);

    std::cout << "  -> PASSED: Gilbert-Elliott burst transitions verified.\n";
}

int main() {
    std::cout << "==================================================\n";
    std::cout << " ASTROMESH-X M7.12: FAILURE INJECTOR UNIT TESTS\n";
    std::cout << "==================================================\n";

    test_scheduled_partition();
    test_node_crash_and_restart();
    test_gilbert_elliott_loss();

    std::cout << "==================================================\n";
    std::cout << " ALL FAILURE INJECTOR TESTS PASSED (100%)\n";
    std::cout << "==================================================\n";
    return 0;
}

