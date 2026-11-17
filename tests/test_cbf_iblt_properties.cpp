#include "astromesh/core/counting_bloom_filter.hpp"
#include "astromesh/core/iblt.hpp"
#include "astromesh/core/landmark_key.hpp"

#include <iostream>
#include <vector>
#include <unordered_set>
#include <random>
#include <cassert>
#include <algorithm>

using astromesh::core::CountingBloomFilter;
using astromesh::core::CBFConfig;
using astromesh::core::CBFInsertResult;
using astromesh::core::CBFRemoveResult;
using astromesh::core::InvertibleBloomFilter;
using astromesh::core::IBLTConfig;
using astromesh::core::IBLTDecodeStatus;
using astromesh::core::IBLTDecodeResult;
using astromesh::core::IBLTCell;
using astromesh::core::LandmarkKey;

static int g_total_assertions = 0;
static int g_failed_assertions = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        ++g_total_assertions; \
        if (!(cond)) { \
            ++g_failed_assertions; \
            std::cerr << "  FAILED: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
        } \
    } while (0)

// Helper to generate deterministic unique landmark keys
static std::vector<LandmarkKey> generate_keys(size_t count, uint64_t seed, uint8_t rover_id = 1) {
    std::mt19937_64 rng(seed);
    std::vector<LandmarkKey> keys;
    keys.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        keys.push_back(LandmarkKey(1, rover_id, static_cast<uint32_t>(i + 1), rng()));
    }
    return keys;
}

// ==============================================================================
// CBF Invariant & Property Tests
// ==============================================================================

void test_cbf_property_no_false_negatives() {
    std::cout << "[CBF Property 1] No False Negatives Invariant...\n";
    // For any set of keys inserted into CBF, contains() must be true for 100% of inserted keys
    const uint64_t seeds[] = {42, 1337, 99999, 20261027};
    for (uint64_t seed : seeds) {
        auto keys = generate_keys(200, seed);
        CountingBloomFilter cbf(200, 0.05);

        for (const auto& k : keys) {
            TEST_ASSERT(cbf.insert(k), "Insertion of fresh key must succeed");
        }

        // Invariant: 0% false negative rate
        for (const auto& k : keys) {
            TEST_ASSERT(cbf.contains(k), "Every inserted key must query as present (zero false negatives)");
        }
    }
}

void test_cbf_property_determinism() {
    std::cout << "[CBF Property 2] Determinism Across Independent Instances...\n";
    // Two filters constructed with identical config and fed identical key sequences
    // must have bit-for-bit identical counter arrays and probe outputs.
    auto keys = generate_keys(150, 777);

    CountingBloomFilter cbf1(150, 0.01);
    CountingBloomFilter cbf2(150, 0.01);

    for (const auto& k : keys) {
        cbf1.insert(k);
        cbf2.insert(k);
    }

    TEST_ASSERT(cbf1.raw_counters() == cbf2.raw_counters(), "Counter arrays must be byte-identical");

    for (const auto& k : keys) {
        std::vector<size_t> p1, p2;
        cbf1.get_probe_indices(k, p1);
        cbf2.get_probe_indices(k, p2);
        TEST_ASSERT(p1 == p2, "Probe indices must match exactly across instances");
    }
}

void test_cbf_property_removal_safety() {
    std::cout << "[CBF Property 3] Removal Safety & Ledger Protection...\n";
    // Attempting to remove keys never inserted must return NotPresent and NEVER decrement any counter.
    auto inserted_keys = generate_keys(100, 111, 1);
    auto uninserted_keys = generate_keys(50, 222, 2);

    CountingBloomFilter cbf(100, 0.05);
    for (const auto& k : inserted_keys) {
        cbf.insert(k);
    }

    auto counters_before = cbf.raw_counters();
    size_t active_before = cbf.active_keys();

    for (const auto& uninserted : uninserted_keys) {
        auto res = cbf.remove_detailed(uninserted);
        TEST_ASSERT(res == CBFRemoveResult::NotPresent, "Uninserted key removal must return NotPresent");
    }

    TEST_ASSERT(cbf.raw_counters() == counters_before, "Uninserted removals must NOT modify any counters");
    TEST_ASSERT(cbf.active_keys() == active_before, "Active key count must remain unchanged");
}

void test_cbf_property_counter_saturation_pinning() {
    std::cout << "[CBF Property 4] Counter Saturation Pinning Invariant...\n";
    // Construct small CBF allowing duplicates to force counter saturation
    CountingBloomFilter cbf = CountingBloomFilter::create_with_dimensions(8, 2, true);
    LandmarkKey k(1, 1, 100, 0xABCDEF);

    // Repeatedly insert key until saturation
    for (size_t i = 0; i < 300; ++i) {
        cbf.insert(k);
    }

    TEST_ASSERT(cbf.saturated_counters() > 0, "Counters must saturate after >255 insertions");

    std::vector<size_t> probes;
    cbf.get_probe_indices(k, probes);
    for (size_t p : probes) {
        TEST_ASSERT(cbf.counter_at(p) == CountingBloomFilter::MAX_COUNTER, "Saturated counters must be pinned at 255");
    }

    // Attempt removal: saturated counters must remain pinned at 255
    auto rem_res = cbf.remove_detailed(k);
    TEST_ASSERT(rem_res == CBFRemoveResult::PinnedCounterSkipped, "Removal with pinned counter returns PinnedCounterSkipped");
    for (size_t p : probes) {
        TEST_ASSERT(cbf.counter_at(p) == CountingBloomFilter::MAX_COUNTER, "Pinned counters must not wrap around or decrement");
    }
}

void test_cbf_property_invalid_input_rejection() {
    std::cout << "[CBF Property 5] Invalid Key & Configuration Rejection...\n";
    CountingBloomFilter cbf(50, 0.05);
    LandmarkKey invalid_key = LandmarkKey::invalid(); // schema == 0 sentinel

    TEST_ASSERT(cbf.insert_detailed(invalid_key) == CBFInsertResult::InvalidKey, "Invalid key rejected on insert");
    TEST_ASSERT(!cbf.contains(invalid_key), "Invalid key contains() returns false");
    TEST_ASSERT(cbf.remove_detailed(invalid_key) == CBFRemoveResult::InvalidKey, "Invalid key rejected on remove");

    // LandmarkKey constructor rejection for schema 0
    bool caught_key = false;
    try {
        LandmarkKey bad_k(0, 1, 1, 0x123);
    } catch (const std::invalid_argument&) {
        caught_key = true;
    }
    TEST_ASSERT(caught_key, "LandmarkKey with schema 0 must throw std::invalid_argument");

    // Invalid configuration

    bool caught = false;
    try {
        CountingBloomFilter bad(0, 0.05);
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    TEST_ASSERT(caught, "expected_items=0 must throw std::invalid_argument");

    caught = false;
    try {
        CountingBloomFilter bad(100, 0.0);
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    TEST_ASSERT(caught, "target_fpr=0.0 must throw std::invalid_argument");
}

// ==============================================================================
// IBLT Invariant & Property Tests
// ==============================================================================

void test_iblt_property_self_reconciliation() {
    std::cout << "[IBLT Property 1] Identity / Self-Reconciliation Invariant...\n";
    // Reconciling an IBLT with itself must decode cleanly to an empty difference.
    const uint64_t seeds[] = {1234, 5678, 91011};
    for (uint64_t seed : seeds) {
        auto keys = generate_keys(50, seed);
        InvertibleBloomFilter iblt(100, 3, 1);
        for (const auto& k : keys) {
            iblt.insert(k);
        }

        IBLTDecodeResult res = iblt.reconcile_with(iblt);
        TEST_ASSERT(res.is_success(), "Self-reconciliation must always succeed");
        TEST_ASSERT(res.left_only.empty(), "Self-reconciliation must have 0 left-only keys");
        TEST_ASSERT(res.right_only.empty(), "Self-reconciliation must have 0 right-only keys");
        TEST_ASSERT(res.unpeeled_cells == 0, "Self-reconciliation residual must be 0");
    }
}

void test_iblt_property_exact_symmetric_difference() {
    std::cout << "[IBLT Property 2] Exact Symmetric Difference Partitioning...\n";
    // Construct A and B with shared keys and known disjoint sets
    // Total symmetric difference |A △ B| = 8 (4 in A\B, 4 in B\A), m = 80 cells
    auto shared = generate_keys(40, 100, 1);
    auto only_a = generate_keys(4, 200, 1);
    auto only_b = generate_keys(4, 300, 2);

    InvertibleBloomFilter table_a(80, 3, 1);
    InvertibleBloomFilter table_b(80, 3, 1);

    for (const auto& k : shared) {
        table_a.insert(k);
        table_b.insert(k);
    }
    for (const auto& k : only_a) {
        table_a.insert(k);
    }
    for (const auto& k : only_b) {
        table_b.insert(k);
    }

    IBLTDecodeResult res = table_a.reconcile_with(table_b);
    TEST_ASSERT(res.is_success(), "Reconciliation within capacity must succeed");
    TEST_ASSERT(res.left_only.size() == only_a.size(), "left_only size matches A\\B");
    TEST_ASSERT(res.right_only.size() == only_b.size(), "right_only size matches B\\A");

    // Verify exact key identities
    std::unordered_set<LandmarkKey> expected_left(only_a.begin(), only_a.end());
    std::unordered_set<LandmarkKey> expected_right(only_b.begin(), only_b.end());

    for (const auto& k : res.left_only) {
        TEST_ASSERT(expected_left.count(k) == 1, "Decoded left key must be in A\\B");
    }
    for (const auto& k : res.right_only) {
        TEST_ASSERT(expected_right.count(k) == 1, "Decoded right key must be in B\\A");
    }
}

void test_iblt_property_commutativity_antisymmetry() {
    std::cout << "[IBLT Property 3] Commutativity / Anti-Symmetry Invariant...\n";
    // B.reconcile_with(A) must produce exactly swapped left_only and right_only
    auto shared = generate_keys(30, 444, 1);
    auto only_a = generate_keys(5, 555, 1);
    auto only_b = generate_keys(3, 666, 2);

    InvertibleBloomFilter table_a(80, 3, 1);
    InvertibleBloomFilter table_b(80, 3, 1);

    for (const auto& k : shared) { table_a.insert(k); table_b.insert(k); }
    for (const auto& k : only_a) { table_a.insert(k); }
    for (const auto& k : only_b) { table_b.insert(k); }

    IBLTDecodeResult res_ab = table_a.reconcile_with(table_b);
    IBLTDecodeResult res_ba = table_b.reconcile_with(table_a);

    TEST_ASSERT(res_ab.is_success() && res_ba.is_success(), "Both directions must succeed");
    TEST_ASSERT(res_ab.left_only == res_ba.right_only, "A\\B matches BA right_only");
    TEST_ASSERT(res_ab.right_only == res_ba.left_only, "B\\A matches BA left_only");
}

void test_iblt_property_table_immutability() {
    std::cout << "[IBLT Property 4] Table Immutability Invariant...\n";
    // Neither subtract() nor decode() nor reconcile_with() may mutate input tables
    auto keys_a = generate_keys(20, 10, 1);
    auto keys_b = generate_keys(50, 20, 2); // Difference 70 >> capacity 30 (will fail)

    InvertibleBloomFilter table_a(30, 3, 1);
    InvertibleBloomFilter table_b(30, 3, 1);

    for (const auto& k : keys_a) table_a.insert(k);
    for (const auto& k : keys_b) table_b.insert(k);

    auto cells_a_before = table_a.cells();
    auto cells_b_before = table_b.cells();

    // Reconcile fails due to overload
    IBLTDecodeResult res = table_a.reconcile_with(table_b);
    TEST_ASSERT(!res.is_success(), "Overloaded reconciliation must report failure");

    // Check cells are unmodified
    TEST_ASSERT(table_a.cells().size() == cells_a_before.size(), "Table A size must not change");
    TEST_ASSERT(table_b.cells().size() == cells_b_before.size(), "Table B size must not change");

    for (size_t i = 0; i < table_a.num_cells(); ++i) {
        const auto& c_now = table_a.cell_at(i);
        const auto& c_pre = cells_a_before[i];
        TEST_ASSERT(c_now.count == c_pre.count &&
                    c_now.key_sum_low == c_pre.key_sum_low &&
                    c_now.key_sum_high == c_pre.key_sum_high &&
                    c_now.checksum_sum == c_pre.checksum_sum, "Table A cell must be untouched");
    }
}

void test_iblt_property_overload_soundness() {
    std::cout << "[IBLT Property 5] Overload Failure Soundness...\n";
    // When difference exceeds table capacity, decode() must report IncompleteResidual,
    // NEVER claiming Success or returning an empty difference.
    auto keys_a = generate_keys(50, 888, 1);
    auto keys_b = generate_keys(50, 999, 2); // 100 disjoint keys vs 40 cells

    InvertibleBloomFilter table_a(40, 3, 1);
    InvertibleBloomFilter table_b(40, 3, 1);

    for (const auto& k : keys_a) table_a.insert(k);
    for (const auto& k : keys_b) table_b.insert(k);

    IBLTDecodeResult res = table_a.reconcile_with(table_b);
    TEST_ASSERT(res.status == IBLTDecodeStatus::IncompleteResidual, "Overload must yield IncompleteResidual");
    TEST_ASSERT(!res.is_success(), "is_success() must be false on overload");
    TEST_ASSERT(res.unpeeled_cells > 0, "Unpeeled cells must be non-zero on overload");
}

void test_iblt_property_configuration_rejection() {
    std::cout << "[IBLT Property 6] Incompatible Configuration Rejection...\n";
    InvertibleBloomFilter t1(50, 3, 1);
    InvertibleBloomFilter t2(60, 3, 1); // differing num_cells
    InvertibleBloomFilter t3(50, 4, 1); // differing num_probes
    InvertibleBloomFilter t4(50, 3, 2); // differing protocol_id

    TEST_ASSERT(!t1.is_compatible_with(t2), "Mismatched cells must be incompatible");
    TEST_ASSERT(!t1.is_compatible_with(t3), "Mismatched probes must be incompatible");
    TEST_ASSERT(!t1.is_compatible_with(t4), "Mismatched protocol must be incompatible");

    bool caught = false;
    try {
        t1.subtract(t2);
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    TEST_ASSERT(caught, "subtract() of incompatible table must throw std::invalid_argument");
}

void test_iblt_property_duplicate_rejection_and_cancellation() {
    std::cout << "[IBLT Property 7] Duplicate Rejection & Additive Cancellation Invariant...\n";
    InvertibleBloomFilter iblt(50, 3, 1);
    LandmarkKey k(1, 1, 42, 0xFEEDBEEF);

    auto cells_empty = iblt.cells();

    // 1. First insert succeeds
    TEST_ASSERT(iblt.insert(k), "First insert must succeed");
    TEST_ASSERT(iblt.active_keys() == 1, "Active keys must be 1");

    // 2. Duplicate insert rejected
    TEST_ASSERT(!iblt.insert(k), "Duplicate insert must be rejected");
    TEST_ASSERT(iblt.active_keys() == 1, "Active keys must remain 1 on duplicate");

    // 3. Removal cancels out key exactly, returning table to initial state
    TEST_ASSERT(iblt.remove(k), "Remove must succeed");
    TEST_ASSERT(iblt.active_keys() == 0, "Active keys must be 0 after remove");

    for (size_t i = 0; i < iblt.num_cells(); ++i) {
        TEST_ASSERT(iblt.cell_at(i).is_empty(), "All cells must be empty after cancellation");
    }
}

int main() {
    std::cout << "======================================================================\n";
    std::cout << " AstroMesh-X Phase 1 M1.4: CBF and IBLT Property & Invariant Tests\n";
    std::cout << "======================================================================\n";

    test_cbf_property_no_false_negatives();
    test_cbf_property_determinism();
    test_cbf_property_removal_safety();
    test_cbf_property_counter_saturation_pinning();
    test_cbf_property_invalid_input_rejection();

    test_iblt_property_self_reconciliation();
    test_iblt_property_exact_symmetric_difference();
    test_iblt_property_commutativity_antisymmetry();
    test_iblt_property_table_immutability();
    test_iblt_property_overload_soundness();
    test_iblt_property_configuration_rejection();
    test_iblt_property_duplicate_rejection_and_cancellation();

    std::cout << "======================================================================\n";
    std::cout << " Assertions checked : " << g_total_assertions << "\n";
    std::cout << " Assertions failed  : " << g_failed_assertions << "\n";
    std::cout << "======================================================================\n";

    if (g_failed_assertions > 0) {
        std::cerr << "FAILED: Property-based tests encountered " << g_failed_assertions << " failures.\n";
        return 1;
    }

    std::cout << "SUCCESS: All CBF & IBLT invariant and property tests PASSED.\n";
    return 0;
}
