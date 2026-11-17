#include "astromesh/net/routing_table.hpp"

#include <cassert>
#include <chrono>
#include <cstring>
#include <iostream>
#include <vector>

using namespace astromesh::net;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "Assertion failed: " << msg << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
            std::exit(1); \
        } \
    } while (0)

// 1. Direct Route Installation
static void test_direct_route_installation() {
    std::cout << "[TEST 1] Direct Route Installation..." << std::endl;
    RoutingTable table(1);

    TEST_ASSERT(table.route_count() == 1, "Must have self-route initially");
    TEST_ASSERT(table.lookup_next_hop(1, 1000).value_or(0) == 1, "Self route must point to self");

    bool installed = table.install_direct_route(2, 1000);
    TEST_ASSERT(installed, "Direct route to neighbor 2 must be installed");
    TEST_ASSERT(table.route_count() == 2, "Must have 2 routes now");

    auto next_hop = table.lookup_next_hop(2, 1000);
    TEST_ASSERT(next_hop.has_value() && *next_hop == 2, "Next hop to neighbor 2 must be 2");

    auto r2 = table.get_route(2);
    TEST_ASSERT(r2.has_value(), "Route to 2 must exist");
    TEST_ASSERT(r2->metric == 1, "Direct neighbor metric must be 1");
    TEST_ASSERT(r2->is_reachable(), "Direct neighbor must be reachable");

    std::cout << "  -> PASSED" << std::endl;
}

// 2. Multi-Hop Route Selection
static void test_multihop_route_selection() {
    std::cout << "[TEST 2] Multi-Hop Route Selection..." << std::endl;
    RoutingTable table(1);
    table.install_direct_route(2, 1000);

    // Rover 2 advertises that it can reach Rover 3 with metric 1 (so distance from 1 is 2)
    bool updated = table.update_route_from_advertisement(2, 3, 1, 10, 1000);
    TEST_ASSERT(updated, "Must accept multi-hop route to 3 via 2");

    auto next_hop = table.lookup_next_hop(3, 1000);
    TEST_ASSERT(next_hop.has_value() && *next_hop == 2, "Next hop towards 3 must be 2");

    auto r3 = table.get_route(3);
    TEST_ASSERT(r3.has_value() && r3->metric == 2, "Metric to 3 via 2 must be 2");
    TEST_ASSERT(r3->destination_sequence_number == 10, "Sequence must match destination sequence");

    std::cout << "  -> PASSED" << std::endl;
}

// 3. Better Route Replacement
static void test_better_route_replacement() {
    std::cout << "[TEST 3] Better Route Replacement..." << std::endl;
    RoutingTable table(1);
    table.install_direct_route(2, 1000);
    table.install_direct_route(4, 1000);

    // Rover 2 advertises route to 3 with metric 2 (cost from 1 = 3) at sequence 10
    table.update_route_from_advertisement(2, 3, 2, 10, 1000);
    TEST_ASSERT(table.lookup_next_hop(3, 1000).value_or(0) == 2, "Initial next hop is 2");
    TEST_ASSERT(table.get_route(3)->metric == 3, "Initial metric is 3");

    // Rover 4 advertises route to 3 with metric 1 (cost from 1 = 2) with same sequence 10
    bool replaced = table.update_route_from_advertisement(4, 3, 1, 10, 1000);
    TEST_ASSERT(replaced, "Shorter path from Rover 4 must replace longer path");
    TEST_ASSERT(table.lookup_next_hop(3, 1000).value_or(0) == 4, "Next hop must switch to 4");
    TEST_ASSERT(table.get_route(3)->metric == 2, "Metric must improve to 2");

    // An inferior advertisement from Rover 2 (metric 5) at same sequence must be ignored
    bool inferior = table.update_route_from_advertisement(2, 3, 5, 10, 1000);
    TEST_ASSERT(!inferior, "Inferior route must be rejected");
    TEST_ASSERT(table.lookup_next_hop(3, 1000).value_or(0) == 4, "Next hop must remain 4");

    std::cout << "  -> PASSED" << std::endl;
}

// 4. Stale Advertisement Rejection
static void test_stale_advertisement_rejection() {
    std::cout << "[TEST 4] Stale Advertisement Rejection (Per-Destination Epoch)..." << std::endl;
    RoutingTable table(1);
    table.install_direct_route(2, 1000);

    // Destination 5 has sequence 100
    table.update_route_from_advertisement(2, 5, 2, 100, 1000);
    TEST_ASSERT(table.get_route(5)->destination_sequence_number == 100, "Sequence must be 100");

    // Stale advertisement arrives with sequence 90 (e.g. out-of-order or delayed packet)
    bool stale_accepted = table.update_route_from_advertisement(2, 5, 1, 90, 1000);
    TEST_ASSERT(!stale_accepted, "Stale advertisement with older sequence number must be rejected");
    TEST_ASSERT(table.get_route(5)->metric == 3, "Metric must not be altered by stale route");

    // Fresher advertisement arrives with sequence 102 (even if higher metric due to topology shift)
    bool fresh_accepted = table.update_route_from_advertisement(2, 5, 4, 102, 1000);
    TEST_ASSERT(fresh_accepted, "Fresher sequence number must be accepted unconditionally");
    TEST_ASSERT(table.get_route(5)->destination_sequence_number == 102, "Sequence must update to 102");
    TEST_ASSERT(table.get_route(5)->metric == 5, "Metric must update to 5");

    std::cout << "  -> PASSED" << std::endl;
}

// 5. Split Horizon with Poison Reverse
static void test_poison_reverse() {
    std::cout << "[TEST 5] Split Horizon with Poison Reverse..." << std::endl;
    RoutingTable table(2); // Rover 2 is between Rover 1 and Rover 3
    table.install_direct_route(1, 1000);
    table.install_direct_route(3, 1000);

    // Rover 2 learned route to 4 via Rover 3
    table.update_route_from_advertisement(3, 4, 1, 20, 1000);
    TEST_ASSERT(table.lookup_next_hop(4, 1000).value_or(0) == 3, "Route to 4 goes through 3");

    // When Rover 2 advertises to Rover 1:
    // Route to 4 should be advertised with normal metric = 2 (because next_hop != 1)
    auto adv_to_1 = table.generate_advertisement_payload(1);
    TEST_ASSERT(adv_to_1.size() >= 4 * sizeof(RouteAdvertisementRecord), "Must contain routes");

    bool found_4_in_1 = false;
    for (size_t i = 0; i < adv_to_1.size() / sizeof(RouteAdvertisementRecord); ++i) {
        RouteAdvertisementRecord rec;
        std::memcpy(&rec, adv_to_1.data() + i * sizeof(RouteAdvertisementRecord), sizeof(rec));
        if (rec.destination_rover_id == 4) {
            found_4_in_1 = true;
            TEST_ASSERT(rec.metric == 2, "Metric to 4 advertised to Rover 1 must be 2");
        }
    }
    TEST_ASSERT(found_4_in_1, "Route to 4 must be advertised to Rover 1");

    // When Rover 2 advertises to Rover 3:
    // Route to 4 MUST be poisoned with metric = 16 (because next_hop == 3!)
    auto adv_to_3 = table.generate_advertisement_payload(3);
    bool found_4_in_3 = false;
    for (size_t i = 0; i < adv_to_3.size() / sizeof(RouteAdvertisementRecord); ++i) {
        RouteAdvertisementRecord rec;
        std::memcpy(&rec, adv_to_3.data() + i * sizeof(RouteAdvertisementRecord), sizeof(rec));
        if (rec.destination_rover_id == 4) {
            found_4_in_3 = true;
            TEST_ASSERT(rec.metric == ROUTING_INFINITY_METRIC, "Route to 4 advertised to Rover 3 must be POISONED (16)");
        }
    }
    TEST_ASSERT(found_4_in_3, "Route to 4 must be present in poisoned advertisement to Rover 3");

    std::cout << "  -> PASSED" << std::endl;
}

// 6. Route Expiration
static void test_route_expiration() {
    std::cout << "[TEST 6] Route Expiration & Staleness Timeout..." << std::endl;
    RoutingTable table(1);
    table.install_direct_route(2, 1'000'000'000ULL); // t = 1.0s

    TEST_ASSERT(table.lookup_next_hop(2, 1'500'000'000ULL).has_value(), "Route valid at t = 1.5s (elapsed 0.5s)");
    TEST_ASSERT(table.lookup_next_hop(2, 2'500'000'000ULL).has_value(), "Route valid at t = 2.5s (elapsed 1.5s)");

    // At t = 2.6s, elapsed time is 1.6s > DEFAULT_ROUTE_TIMEOUT_NS (1.5s)
    TEST_ASSERT(!table.lookup_next_hop(2, 2'600'000'000ULL).has_value(), "Route must expire after 1.5s without refresh");

    size_t pruned = table.prune_stale_routes(2'600'000'000ULL);
    TEST_ASSERT(pruned == 1, "Exactly 1 route must be poisoned by prune");
    TEST_ASSERT(table.get_route(2)->metric == ROUTING_INFINITY_METRIC, "Pruned route metric must be 16");

    // Self-route must NEVER expire
    TEST_ASSERT(table.lookup_next_hop(1, 100'000'000'000ULL).value_or(0) == 1, "Self route must never expire");

    std::cout << "  -> PASSED" << std::endl;
}

// 7. Link Failure Invalidation
static void test_link_failure_invalidation() {
    std::cout << "[TEST 7] Link Failure Invalidation..." << std::endl;
    RoutingTable table(1);
    table.install_direct_route(2, 1000);

    // Multiple multi-hop routes depend on Rover 2 as next hop
    table.update_route_from_advertisement(2, 3, 1, 10, 1000);
    table.update_route_from_advertisement(2, 4, 2, 20, 1000);
    table.update_route_from_advertisement(2, 5, 3, 30, 1000);

    TEST_ASSERT(table.lookup_next_hop(3, 1000).value_or(0) == 2, "3 routes via 2");
    TEST_ASSERT(table.lookup_next_hop(4, 1000).value_or(0) == 2, "4 routes via 2");
    TEST_ASSERT(table.lookup_next_hop(5, 1000).value_or(0) == 2, "5 routes via 2");

    // Link to neighbor 2 fails
    table.invalidate_neighbor_routes(2, 2000);

    TEST_ASSERT(!table.lookup_next_hop(2, 2000).has_value(), "Direct route to 2 must be invalid");
    TEST_ASSERT(!table.lookup_next_hop(3, 2000).has_value(), "Route to 3 via 2 must be invalid");
    TEST_ASSERT(!table.lookup_next_hop(4, 2000).has_value(), "Route to 4 via 2 must be invalid");
    TEST_ASSERT(!table.lookup_next_hop(5, 2000).has_value(), "Route to 5 via 2 must be invalid");

    TEST_ASSERT(table.get_route(3)->metric == ROUTING_INFINITY_METRIC, "Poisoned metric must be 16");

    std::cout << "  -> PASSED" << std::endl;
}

// 8. Arbitrary Rover IDs
static void test_arbitrary_rover_ids() {
    std::cout << "[TEST 8] Arbitrary Rover IDs..." << std::endl;
    RoutingTable table(1001); // High non-contiguous ID

    TEST_ASSERT(table.lookup_next_hop(1001, 1000).value_or(0) == 1001, "Self route for 1001");

    table.install_direct_route(2045, 1000);
    TEST_ASSERT(table.lookup_next_hop(2045, 1000).value_or(0) == 2045, "Direct route to 2045");

    table.update_route_from_advertisement(2045, 9999, 2, 50, 1000);
    TEST_ASSERT(table.lookup_next_hop(9999, 1000).value_or(0) == 2045, "Route to 9999 via 2045");
    TEST_ASSERT(table.get_route(9999)->metric == 3, "Metric must be 3");

    std::cout << "  -> PASSED" << std::endl;
}

// 9. Unknown Destination
static void test_unknown_destination() {
    std::cout << "[TEST 9] Unknown Destination Lookup..." << std::endl;
    RoutingTable table(1);

    auto res = table.lookup_next_hop(42, 1000);
    TEST_ASSERT(!res.has_value(), "Unknown destination must return nullopt");

    auto r = table.get_route(42);
    TEST_ASSERT(!r.has_value(), "get_route for unknown destination must return nullopt");

    std::cout << "  -> PASSED" << std::endl;
}

// 10. Invalid Metrics and IDs
static void test_invalid_metrics_and_ids() {
    std::cout << "[TEST 10] Invalid Metrics and IDs..." << std::endl;
    RoutingTable table(1);

    // Target 0
    TEST_ASSERT(!table.install_direct_route(0, 1000), "Cannot install neighbor 0");
    TEST_ASSERT(!table.lookup_next_hop(0, 1000).has_value(), "Destination 0 must be nullopt");

    // Self neighbor
    TEST_ASSERT(!table.install_direct_route(1, 1000), "Cannot install self as neighbor");

    // Advertisement for self
    TEST_ASSERT(!table.update_route_from_advertisement(2, 1, 1, 10, 1000), "Cannot update route to self from advertisement");

    // Advertisement with unreachable metric (16)
    bool installed_unreachable = table.update_route_from_advertisement(2, 3, ROUTING_INFINITY_METRIC, 10, 1000);
    TEST_ASSERT(!installed_unreachable, "Cannot install a route advertised as unreachable");

    std::cout << "  -> PASSED" << std::endl;
}

// 11. Serialization and Deserialization Pipeline
static void test_serialization_and_deserialization() {
    std::cout << "[TEST 11] Serialization & Deserialization Pipeline..." << std::endl;
    RoutingTable table_a(1);
    table_a.install_direct_route(2, 1000);
    table_a.install_direct_route(3, 1000);

    auto payload = table_a.generate_advertisement_payload(2);
    TEST_ASSERT(!payload.empty(), "Payload must not be empty");
    TEST_ASSERT(payload.size() % sizeof(RouteAdvertisementRecord) == 0, "Payload must be multiple of record size");

    RoutingTable table_b(2);
    size_t applied = table_b.apply_advertisement_payload(1, payload.data(), payload.size(), 1000);
    TEST_ASSERT(applied >= 1, "Must apply at least route to 3 via 1");

    auto next_to_3 = table_b.lookup_next_hop(3, 1000);
    TEST_ASSERT(next_to_3.has_value() && *next_to_3 == 1, "Rover 2 must route to 3 via 1");

    std::cout << "  -> PASSED" << std::endl;
}

int main() {
    std::cout << "=================================================================" << std::endl;
    std::cout << "  ASTRAMESH-X M7.11 ROUTING TABLE UNIT TEST SUITE                " << std::endl;
    std::cout << "=================================================================" << std::endl;

    test_direct_route_installation();
    test_multihop_route_selection();
    test_better_route_replacement();
    test_stale_advertisement_rejection();
    test_poison_reverse();
    test_route_expiration();
    test_link_failure_invalidation();
    test_arbitrary_rover_ids();
    test_unknown_destination();
    test_invalid_metrics_and_ids();
    test_serialization_and_deserialization();

    std::cout << "=================================================================" << std::endl;
    std::cout << "  ALL 11 ROUTING TABLE TESTS PASSED (100%)                       " << std::endl;
    std::cout << "=================================================================" << std::endl;
    return 0;
}
