#include "astromesh/core/counting_bloom_filter.hpp"
#include "astromesh/core/landmark_key.hpp"

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <iomanip>

using astromesh::core::CountingBloomFilter;
using astromesh::core::LandmarkKey;
using astromesh::core::CBFInsertResult;
using astromesh::core::CBFRemoveResult;

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

void test_construction_and_validation() {
    std::cout << "[TEST 1 & 2] Construction and Parameter Validation...\n";

    // 1. Valid construction by capacity & probability
    CountingBloomFilter cbf(1000, 0.01);
    TEST_ASSERT(cbf.num_counters() > 0, "Counter array size must be positive");
    TEST_ASSERT(cbf.num_probes() > 0, "Probe count must be positive");
    TEST_ASSERT(cbf.active_keys() == 0, "Initially active keys must be 0");
    TEST_ASSERT(cbf.saturated_counters() == 0, "Initially saturated counters must be 0");
    TEST_ASSERT(cbf.estimated_false_positive_rate() == 0.0, "Empty filter estimated FPR must be 0.0");

    // 2. Rejection of zero capacity
    bool caught = false;
    try {
        CountingBloomFilter bad(0, 0.05);
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    TEST_ASSERT(caught, "Zero capacity must throw std::invalid_argument");

    // 3. Rejection of invalid probabilities
    caught = false;
    try {
        CountingBloomFilter bad(100, 0.0);
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    TEST_ASSERT(caught, "p = 0.0 must throw std::invalid_argument");

    caught = false;
    try {
        CountingBloomFilter bad(100, 1.0);
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    TEST_ASSERT(caught, "p = 1.0 must throw std::invalid_argument");

    caught = false;
    try {
        CountingBloomFilter bad(100, -0.2);
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    TEST_ASSERT(caught, "Negative probability must throw std::invalid_argument");

    caught = false;
    try {
        CountingBloomFilter bad(100, std::numeric_limits<double>::quiet_NaN());
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    TEST_ASSERT(caught, "NaN probability must throw std::invalid_argument");

    // 4. try_create factory validation
    auto opt_bad = CountingBloomFilter::try_create(0, 0.01);
    TEST_ASSERT(!opt_bad.has_value(), "try_create with 0 capacity must return nullopt");

    auto opt_good = CountingBloomFilter::try_create(500, 0.05);
    TEST_ASSERT(opt_good.has_value(), "try_create with valid parameters must return object");

    // 5. Explicit (m, k) dimension validation
    caught = false;
    try {
        auto bad = CountingBloomFilter::create_with_dimensions(0, 5);
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    TEST_ASSERT(caught, "Explicit m=0 must throw std::invalid_argument");

    caught = false;
    try {
        auto bad = CountingBloomFilter::create_with_dimensions(100, 0);
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    TEST_ASSERT(caught, "Explicit k=0 must throw std::invalid_argument");
}

void test_insertion_and_presence() {
    std::cout << "[TEST 3] Insertion and Presence Verification...\n";

    CountingBloomFilter cbf(100, 0.01);
    LandmarkKey k1(1, 10, 1, 0x1111ULL);
    LandmarkKey k2(1, 10, 2, 0x2222ULL);

    TEST_ASSERT(!cbf.contains(k1), "Key 1 must not be present initially");
    TEST_ASSERT(!cbf.contains(k2), "Key 2 must not be present initially");

    bool inserted = cbf.insert(k1);
    TEST_ASSERT(inserted, "First insert must succeed");
    TEST_ASSERT(cbf.contains(k1), "Key 1 must be present after insertion (zero false negatives)");
    TEST_ASSERT(cbf.active_keys() == 1, "active_keys must be 1");
    TEST_ASSERT(cbf.total_insertions() == 1, "total_insertions must be 1");

    inserted = cbf.insert(k2);
    TEST_ASSERT(inserted, "Second insert must succeed");
    TEST_ASSERT(cbf.contains(k1), "Key 1 must still be present");
    TEST_ASSERT(cbf.contains(k2), "Key 2 must be present");
    TEST_ASSERT(cbf.active_keys() == 2, "active_keys must be 2");
}

void test_duplicate_policies() {
    std::cout << "[TEST 4] Duplicate Insertion Policy...\n";

    LandmarkKey k(1, 5, 42, 0xABCDEFULL);

    // Default policy: allow_duplicates = false
    CountingBloomFilter cbf_no_dup(100, 0.01, false);
    CBFInsertResult res1 = cbf_no_dup.insert_detailed(k);
    TEST_ASSERT(res1 == CBFInsertResult::Inserted, "First insertion must succeed");

    CBFInsertResult res2 = cbf_no_dup.insert_detailed(k);
    TEST_ASSERT(res2 == CBFInsertResult::Duplicate, "Second insertion with allow_duplicates=false must return Duplicate");
    TEST_ASSERT(cbf_no_dup.active_keys() == 1, "active_keys must remain 1");
    TEST_ASSERT(cbf_no_dup.total_insertions() == 1, "total_insertions must remain 1");

    // Permissive policy: allow_duplicates = true
    CountingBloomFilter cbf_dup(100, 0.01, true);
    res1 = cbf_dup.insert_detailed(k);
    TEST_ASSERT(res1 == CBFInsertResult::Inserted, "First insertion must succeed");

    res2 = cbf_dup.insert_detailed(k);
    TEST_ASSERT(res2 == CBFInsertResult::Inserted, "Second insertion with allow_duplicates=true must succeed");
    TEST_ASSERT(cbf_dup.active_keys() == 1, "Distinct active_keys remains 1");
    TEST_ASSERT(cbf_dup.total_insertions() == 2, "total_insertions increments to 2");
}

void test_query_absent_keys() {
    std::cout << "[TEST 5] Querying Absent Keys on Empty Filter...\n";

    CountingBloomFilter cbf(100, 0.01);
    for (uint32_t i = 1; i <= 20; ++i) {
        LandmarkKey absent(1, 1, i, i * 100);
        TEST_ASSERT(!cbf.contains(absent), "Empty filter must strictly return false for all queries");
    }
}

void test_removal_safety_and_ledger() {
    std::cout << "[TEST 6 & 7] Removal Safety Contract and Ledger Protection...\n";

    CountingBloomFilter cbf(100, 0.01);
    LandmarkKey k1(1, 1, 100, 0xAAAAULL);
    LandmarkKey k2_uninserted(1, 1, 200, 0xBBBBULL);

    cbf.insert(k1);
    TEST_ASSERT(cbf.contains(k1), "k1 present");

    // 1. Attempt to remove key that was never inserted
    CBFRemoveResult rem_bad = cbf.remove_detailed(k2_uninserted);
    TEST_ASSERT(rem_bad == CBFRemoveResult::NotPresent, "Removing uninserted key must return NotPresent");
    TEST_ASSERT(cbf.total_removals() == 0, "No removals counted on uninserted key");
    TEST_ASSERT(cbf.contains(k1), "k1 must still be 100% present (zero counter corruption)");

    // 2. Remove valid key
    CBFRemoveResult rem_good = cbf.remove_detailed(k1);
    TEST_ASSERT(rem_good == CBFRemoveResult::Removed, "Removing valid key must return Removed");
    TEST_ASSERT(cbf.active_keys() == 0, "active_keys must be 0 after removal");
    TEST_ASSERT(cbf.total_removals() == 1, "total_removals must be 1");
    TEST_ASSERT(!cbf.contains(k1), "k1 must not be present after removal");

    // 3. Double-removal of k1
    CBFRemoveResult rem_double = cbf.remove_detailed(k1);
    TEST_ASSERT(rem_double == CBFRemoveResult::NotPresent, "Double removal must return NotPresent");
    TEST_ASSERT(cbf.total_removals() == 1, "total_removals must not increment on double removal");
}

void test_counter_saturation_and_pinning() {
    std::cout << "[TEST 8] Counter Saturation and Pinning Policy...\n";

    // 1 counter, 1 probe, allow_duplicates = true
    auto cbf = CountingBloomFilter::create_with_dimensions(1, 1, true);
    LandmarkKey k(1, 1, 1, 0x1234ULL);

    // Insert 254 times -> counter reaches 254
    for (int i = 0; i < 254; ++i) {
        cbf.insert(k);
    }
    TEST_ASSERT(cbf.counter_at(0) == 254, "Counter must be 254");
    TEST_ASSERT(cbf.saturated_counters() == 0, "No saturated counters yet");

    // 255th insertion -> reaches MAX_COUNTER (255)
    CBFInsertResult res255 = cbf.insert_detailed(k);
    TEST_ASSERT(res255 == CBFInsertResult::CounterSaturated, "255th insertion must report CounterSaturated");
    TEST_ASSERT(cbf.counter_at(0) == 255, "Counter must be 255");
    TEST_ASSERT(cbf.saturated_counters() == 1, "Saturated counters must be 1");

    // 256th insertion -> remains pinned at 255 (NO silent overflow to 0!)
    CBFInsertResult res256 = cbf.insert_detailed(k);
    TEST_ASSERT(res256 == CBFInsertResult::CounterSaturated, "256th insertion must report CounterSaturated");
    TEST_ASSERT(cbf.counter_at(0) == 255, "Counter must NOT wrap around; must stay 255");

    // Removal from saturated counter -> remains PINNED
    CBFRemoveResult rem_res = cbf.remove_detailed(k);
    TEST_ASSERT(rem_res == CBFRemoveResult::PinnedCounterSkipped, "Removal on saturated counter reports PinnedCounterSkipped");
    TEST_ASSERT(cbf.counter_at(0) == 255, "Saturated counter must remain pinned at 255 to prevent false negatives");
}

void test_clear_and_reuse() {
    std::cout << "[TEST 9] Clear and Reuse...\n";

    CountingBloomFilter cbf(100, 0.05);
    for (uint32_t i = 1; i <= 20; ++i) {
        cbf.insert(LandmarkKey(1, 1, i, i * 7));
    }
    TEST_ASSERT(cbf.active_keys() == 20, "Active keys must be 20");

    cbf.clear();
    TEST_ASSERT(cbf.active_keys() == 0, "After clear, active keys must be 0");
    TEST_ASSERT(cbf.total_insertions() == 0, "After clear, total insertions must be 0");
    TEST_ASSERT(cbf.total_removals() == 0, "After clear, total removals must be 0");

    for (size_t i = 0; i < cbf.num_counters(); ++i) {
        TEST_ASSERT(cbf.counter_at(i) == 0, "All counter slots must be 0 after clear");
    }

    // Reuse filter
    LandmarkKey new_key(1, 2, 555, 0x9999ULL);
    TEST_ASSERT(!cbf.contains(new_key), "Key not present before re-insert");
    cbf.insert(new_key);
    TEST_ASSERT(cbf.contains(new_key), "Key present after re-insert");
    TEST_ASSERT(cbf.active_keys() == 1, "active_keys must be 1");
}

void test_edge_cases_and_invalid_keys() {
    std::cout << "[TEST 10] Large Values and Invalid Key Rejection...\n";

    CountingBloomFilter cbf(50, 0.05);

    // Max field values
    LandmarkKey max_key(0xFFFF, 0xFFFF, 0xFFFFFFFF, 0xFFFFFFFFFFFFFFFFULL);
    TEST_ASSERT(cbf.insert(max_key), "Max field value key must insert successfully");
    TEST_ASSERT(cbf.contains(max_key), "Max field value key must be present");
    TEST_ASSERT(cbf.remove(max_key), "Max field value key must remove successfully");
    TEST_ASSERT(!cbf.contains(max_key), "Max field value key absent after removal");

    // Invalid key (schema == 0)
    LandmarkKey invalid_key = LandmarkKey::invalid();
    CBFInsertResult ins_inv = cbf.insert_detailed(invalid_key);
    TEST_ASSERT(ins_inv == CBFInsertResult::InvalidKey, "Invalid key insert must return InvalidKey");
    TEST_ASSERT(!cbf.contains(invalid_key), "contains() on invalid key must return false");

    CBFRemoveResult rem_inv = cbf.remove_detailed(invalid_key);
    TEST_ASSERT(rem_inv == CBFRemoveResult::InvalidKey, "remove() on invalid key must return InvalidKey");
}

void test_repeatable_determinism() {
    std::cout << "[TEST 11] Repeatable Deterministic Probing...\n";

    CountingBloomFilter cbf1(500, 0.01);
    CountingBloomFilter cbf2(500, 0.01);

    for (uint32_t i = 1; i <= 50; ++i) {
        LandmarkKey k(1, 3, i, 0x55555555ULL ^ i);
        cbf1.insert(k);
        cbf2.insert(k);
    }

    TEST_ASSERT(cbf1.raw_counters() == cbf2.raw_counters(), "Identical insertion sequence must yield bitwise identical counter arrays");

    // Verify probe indices match exactly
    LandmarkKey test_k(1, 3, 99, 0x12345678ULL);
    std::vector<size_t> probes1, probes2;
    cbf1.get_probe_indices(test_k, probes1);
    cbf2.get_probe_indices(test_k, probes2);
    TEST_ASSERT(probes1 == probes2, "Probe indices must match exactly across independent filter instances");
}

void test_empirical_false_positive_rate() {
    std::cout << "[TEST 12] Seeded Empirical False-Positive Rate Benchmark...\n";

    const size_t num_insertions = 1000;
    const double target_fpr = 0.05; // 5% target FPR
    const size_t num_queries = 10000;

    CountingBloomFilter cbf(num_insertions, target_fpr);

    std::cout << "  Filter Dimensions: m=" << cbf.num_counters()
              << " counters, k=" << cbf.num_probes() << " probes\n";

    // Insert 1000 disjoint keys from Rover 1
    for (uint32_t i = 1; i <= num_insertions; ++i) {
        // Deterministic pseudo-random descriptor hash derived from i
        uint64_t desc_hash = static_cast<uint64_t>(i) * 0xbf58476d1ce4e5b9ULL;
        LandmarkKey key(1, 1, i, desc_hash);
        cbf.insert(key);
    }

    TEST_ASSERT(cbf.active_keys() == num_insertions, "All 1000 keys must be actively registered");

    // Zero False Negatives Verification:
    // Every single inserted key MUST be recognized as present
    size_t false_negatives = 0;
    for (uint32_t i = 1; i <= num_insertions; ++i) {
        uint64_t desc_hash = static_cast<uint64_t>(i) * 0xbf58476d1ce4e5b9ULL;
        LandmarkKey key(1, 1, i, desc_hash);
        if (!cbf.contains(key)) {
            false_negatives++;
        }
    }
    TEST_ASSERT(false_negatives == 0, "Counting Bloom Filter must have EXACTLY 0 false negatives for inserted keys");

    // Query 10000 completely disjoint keys from Rover 2 (never inserted)
    size_t observed_false_positives = 0;
    for (uint32_t i = 1; i <= num_queries; ++i) {
        // Rover 2 ensures disjoint key namespace
        uint64_t desc_hash = static_cast<uint64_t>(i) * 0x94d049bb133111ebULL;
        LandmarkKey absent_key(1, 2, i, desc_hash);
        if (cbf.contains(absent_key)) {
            observed_false_positives++;
        }
    }

    double empirical_fpr = static_cast<double>(observed_false_positives) / static_cast<double>(num_queries);
    double estimated_fpr = cbf.estimated_false_positive_rate();

    std::cout << "  Queries Tested:          " << num_queries << "\n";
    std::cout << "  Observed False Positives:" << observed_false_positives << "\n";
    std::cout << "  Empirical FPR:           " << std::fixed << std::setprecision(4) << empirical_fpr
              << " (" << (empirical_fpr * 100.0) << "%)\n";
    std::cout << "  Theoretical Target FPR:  " << target_fpr << " (" << (target_fpr * 100.0) << "%)\n";
    std::cout << "  Estimated Filter FPR:    " << estimated_fpr << " (" << (estimated_fpr * 100.0) << "%)\n";

    // Statistical bounds:
    // With N=10000 and target p=0.05, standard deviation is sqrt(p*(1-p)/N) ≈ 0.00218.
    // 5 standard deviations is ≈ 0.0109.
    // Allow a robust window between 0.02 and 0.08 (2.0% to 8.0%) to prevent flaky test failures
    // while strictly verifying the probabilistic behavior matches theory.
    TEST_ASSERT(empirical_fpr >= 0.02 && empirical_fpr <= 0.08,
                "Empirical FPR must lie within reasonable statistical variation of target 0.05");
}

int main() {
    std::cout << "===============================================\n";
    std::cout << " AstroMesh-X: CountingBloomFilter Test Suite   \n";
    std::cout << "===============================================\n";

    test_construction_and_validation();
    test_insertion_and_presence();
    test_duplicate_policies();
    test_query_absent_keys();
    test_removal_safety_and_ledger();
    test_counter_saturation_and_pinning();
    test_clear_and_reuse();
    test_edge_cases_and_invalid_keys();
    test_repeatable_determinism();
    test_empirical_false_positive_rate();

    std::cout << "===============================================\n";
    std::cout << " Total Assertions Checked: " << g_total_assertions << "\n";
    std::cout << " Failed Assertions:        " << g_failed_assertions << "\n";
    std::cout << "===============================================\n";

    if (g_failed_assertions == 0) {
        std::cout << " ALL CBF TESTS PASSED SUCCESSFULLY.\n";
        return 0;
    } else {
        std::cerr << " CBF TEST SUITE FAILED WITH " << g_failed_assertions << " ERRORS.\n";
        return 1;
    }
}
