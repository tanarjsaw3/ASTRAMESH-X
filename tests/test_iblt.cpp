#include "astromesh/core/iblt.hpp"
#include "astromesh/core/landmark_key.hpp"

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <iomanip>

using astromesh::core::InvertibleBloomFilter;
using astromesh::core::IBLTConfig;
using astromesh::core::IBLTCell;
using astromesh::core::IBLTDecodeStatus;
using astromesh::core::IBLTDecodeResult;
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

void test_construction_and_validation() {
    std::cout << "[TEST 1] Construction and Configuration Validation...\n";

    // 1. Valid construction
    InvertibleBloomFilter iblt(100, 3, 1);
    TEST_ASSERT(iblt.num_cells() == 100, "num_cells must match");
    TEST_ASSERT(iblt.num_probes() == 3, "num_probes must match");
    TEST_ASSERT(iblt.protocol_id() == 1, "protocol_id must match");
    TEST_ASSERT(iblt.active_keys() == 0, "Initial active keys must be 0");
    TEST_ASSERT(iblt.estimated_table_memory_bytes() == 100 * sizeof(IBLTCell), "Memory estimate matches cell size");

    // 2. Reject num_cells == 0
    bool caught = false;
    try {
        InvertibleBloomFilter bad(0, 3);
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    TEST_ASSERT(caught, "m=0 must throw std::invalid_argument");

    // 3. Reject num_probes == 0
    caught = false;
    try {
        InvertibleBloomFilter bad(100, 0);
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    TEST_ASSERT(caught, "k=0 must throw std::invalid_argument");

    // 4. Reject num_probes > num_cells
    caught = false;
    try {
        InvertibleBloomFilter bad(2, 3);
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    TEST_ASSERT(caught, "k > m must throw std::invalid_argument");

    // 5. Reject num_probes > 20
    caught = false;
    try {
        InvertibleBloomFilter bad(100, 25);
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    TEST_ASSERT(caught, "k > 20 must throw std::invalid_argument");

    // 6. try_create factory
    auto opt_bad = InvertibleBloomFilter::try_create(0, 3);
    TEST_ASSERT(!opt_bad.has_value(), "try_create with invalid configuration returns nullopt");

    auto opt_good = InvertibleBloomFilter::try_create(50, 3);
    TEST_ASSERT(opt_good.has_value(), "try_create with valid configuration returns object");
}

void test_identical_sets() {
    std::cout << "[TEST 2] Identical Sets Reconciliation (A = B)...\n";

    InvertibleBloomFilter a(80, 3);
    InvertibleBloomFilter b(80, 3);

    for (uint32_t i = 1; i <= 20; ++i) {
        LandmarkKey key(1, 1, i, 0x1000ULL + i);
        a.insert(key);
        b.insert(key);
    }

    InvertibleBloomFilter diff = a - b;
    IBLTDecodeResult res = diff.decode();

    TEST_ASSERT(res.is_success(), "Reconciliation of identical sets must succeed");
    TEST_ASSERT(res.left_only.empty(), "left_only (A \\ B) must be empty");
    TEST_ASSERT(res.right_only.empty(), "right_only (B \\ A) must be empty");
    TEST_ASSERT(res.unpeeled_cells == 0, "No residual unpeeled cells");
}

void test_single_left_difference() {
    std::cout << "[TEST 3] Single Key Unique to Left Side (A \\ B = {k1})...\n";

    InvertibleBloomFilter a(80, 3);
    InvertibleBloomFilter b(80, 3);

    LandmarkKey shared(1, 1, 10, 0xAAAAULL);
    LandmarkKey unique_left(1, 1, 99, 0xBBBBULL);

    a.insert(shared);
    b.insert(shared);

    a.insert(unique_left); // Only in A

    IBLTDecodeResult res = a.reconcile_with(b);

    TEST_ASSERT(res.is_success(), "Single left difference must decode successfully");
    TEST_ASSERT(res.left_only.size() == 1, "left_only size must be 1");
    TEST_ASSERT(res.left_only[0] == unique_left, "left_only must contain unique_left");
    TEST_ASSERT(res.right_only.empty(), "right_only must be empty");
}

void test_single_right_difference() {
    std::cout << "[TEST 4] Single Key Unique to Right Side (B \\ A = {k2})...\n";

    InvertibleBloomFilter a(80, 3);
    InvertibleBloomFilter b(80, 3);

    LandmarkKey shared(1, 1, 10, 0xAAAAULL);
    LandmarkKey unique_right(1, 2, 77, 0xCCCCULL);

    a.insert(shared);
    b.insert(shared);

    b.insert(unique_right); // Only in B

    IBLTDecodeResult res = a.reconcile_with(b);

    TEST_ASSERT(res.is_success(), "Single right difference must decode successfully");
    TEST_ASSERT(res.left_only.empty(), "left_only must be empty");
    TEST_ASSERT(res.right_only.size() == 1, "right_only size must be 1");
    TEST_ASSERT(res.right_only[0] == unique_right, "right_only must contain unique_right");
}

void test_multiple_bidirectional_differences() {
    std::cout << "[TEST 5 & 6] Multiple Bidirectional Differences Separation...\n";

    InvertibleBloomFilter a(120, 3);
    InvertibleBloomFilter b(120, 3);

    // Shared items: 10 keys
    for (uint32_t i = 1; i <= 10; ++i) {
        LandmarkKey shared(1, 1, i, 0x5555ULL + i);
        a.insert(shared);
        b.insert(shared);
    }

    // Left-only items: 3 keys
    std::vector<LandmarkKey> expected_left;
    for (uint32_t i = 101; i <= 103; ++i) {
        LandmarkKey lk(1, 1, i, 0xAAAAULL + i);
        a.insert(lk);
        expected_left.push_back(lk);
    }

    // Right-only items: 4 keys
    std::vector<LandmarkKey> expected_right;
    for (uint32_t i = 201; i <= 204; ++i) {
        LandmarkKey rk(1, 2, i, 0xBBBBULL + i);
        b.insert(rk);
        expected_right.push_back(rk);
    }

    std::sort(expected_left.begin(), expected_left.end());
    std::sort(expected_right.begin(), expected_right.end());

    IBLTDecodeResult res = a.reconcile_with(b);

    TEST_ASSERT(res.is_success(), "Bidirectional differences must decode cleanly");
    TEST_ASSERT(res.left_only.size() == 3, "left_only size must be 3");
    TEST_ASSERT(res.right_only.size() == 4, "right_only size must be 4");
    TEST_ASSERT(res.left_only == expected_left, "left_only keys must match expected A \\ B");
    TEST_ASSERT(res.right_only == expected_right, "right_only keys must match expected B \\ A");
}

void test_duplicate_key_policy() {
    std::cout << "[TEST 7] Duplicate Key Policy (Set Semantics)...\n";

    InvertibleBloomFilter iblt(50, 3);
    LandmarkKey k(1, 1, 42, 0x9999ULL);

    TEST_ASSERT(iblt.insert(k), "First insert must succeed");
    TEST_ASSERT(!iblt.insert(k), "Second insert of same key must return false (set semantics)");
    TEST_ASSERT(iblt.active_keys() == 1, "active_keys must remain 1");

    TEST_ASSERT(iblt.remove(k), "First remove must succeed");
    TEST_ASSERT(!iblt.remove(k), "Second remove must return false");
    TEST_ASSERT(iblt.active_keys() == 0, "active_keys must be 0");
}

void test_large_values_and_boundaries() {
    std::cout << "[TEST 8] Boundary and Large Field Values...\n";

    InvertibleBloomFilter a(50, 3);
    InvertibleBloomFilter b(50, 3);

    LandmarkKey max_key(0xFFFF, 0xFFFF, 0xFFFFFFFF, 0xFFFFFFFFFFFFFFFFULL);
    a.insert(max_key);

    IBLTDecodeResult res = a.reconcile_with(b);
    TEST_ASSERT(res.is_success(), "Max key values must reconcile without truncation");
    TEST_ASSERT(res.left_only.size() == 1, "One left key recovered");
    TEST_ASSERT(res.left_only[0] == max_key, "Recovered max key matches exactly");

    // Invalid key rejection
    LandmarkKey invalid_key = LandmarkKey::invalid();
    TEST_ASSERT(!a.insert(invalid_key), "Invalid key insertion must return false");
    TEST_ASSERT(!a.remove(invalid_key), "Invalid key removal must return false");
}

void test_incompatible_tables() {
    std::cout << "[TEST 9] Incompatible Table Rejection...\n";

    InvertibleBloomFilter t1(100, 3, 1);
    InvertibleBloomFilter t_diff_cells(101, 3, 1);
    InvertibleBloomFilter t_diff_probes(100, 4, 1);
    InvertibleBloomFilter t_diff_proto(100, 3, 2);

    TEST_ASSERT(t1.is_compatible_with(t1), "Self compatibility must be true");
    TEST_ASSERT(!t1.is_compatible_with(t_diff_cells), "Differing cell counts must be incompatible");
    TEST_ASSERT(!t1.is_compatible_with(t_diff_probes), "Differing probe counts must be incompatible");
    TEST_ASSERT(!t1.is_compatible_with(t_diff_proto), "Differing protocol IDs must be incompatible");

    bool caught = false;
    try {
        t1.subtract(t_diff_cells);
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    TEST_ASSERT(caught, "Subtracting incompatible cell count must throw std::invalid_argument");
}

void test_checksum_and_pure_cell_logic() {
    std::cout << "[TEST 10] Checksum and Pure Cell Logic...\n";

    LandmarkKey k1(1, 1, 10, 0x1111ULL);
    LandmarkKey k2(1, 1, 20, 0x2222ULL);

    uint64_t chk1 = InvertibleBloomFilter::compute_checksum(k1);
    uint64_t chk2 = InvertibleBloomFilter::compute_checksum(k2);
    TEST_ASSERT(chk1 != 0, "Checksum must be non-zero");
    TEST_ASSERT(chk1 != chk2, "Distinct keys must have distinct checksums");

    // Bit packing test
    uint64_t low = 0, high = 0;
    InvertibleBloomFilter::key_to_bits(k1, low, high);
    LandmarkKey reconstructed = InvertibleBloomFilter::bits_to_key(low, high);
    TEST_ASSERT(reconstructed == k1, "bits_to_key must perfectly round-trip key");
}

void test_table_immutability_during_decode() {
    std::cout << "[TEST 11 & 12] Table Immutability During Decode (Copy Isolation)...\n";

    // 1. Successful decode leaves table unchanged
    InvertibleBloomFilter a(80, 3);
    InvertibleBloomFilter b(80, 3);
    LandmarkKey k(1, 1, 5, 0x5555ULL);
    a.insert(k);

    InvertibleBloomFilter diff_success = a - b;
    std::vector<IBLTCell> cells_before = diff_success.cells();

    IBLTDecodeResult res_success = diff_success.decode();
    TEST_ASSERT(res_success.is_success(), "Decode succeeded");
    TEST_ASSERT(diff_success.cells().size() == cells_before.size(), "Cell size unchanged");

    for (size_t i = 0; i < cells_before.size(); ++i) {
        TEST_ASSERT(diff_success.cells()[i].count == cells_before[i].count, "Count unchanged after success decode");
        TEST_ASSERT(diff_success.cells()[i].key_sum_low == cells_before[i].key_sum_low, "Key low unchanged");
        TEST_ASSERT(diff_success.cells()[i].checksum_sum == cells_before[i].checksum_sum, "Checksum unchanged");
    }

    // 2. Failed decode leaves table unchanged
    InvertibleBloomFilter small_a(10, 3);
    InvertibleBloomFilter small_b(10, 3);
    for (uint32_t i = 1; i <= 20; ++i) {
        small_a.insert(LandmarkKey(1, 1, i, i * 100));
    }
    InvertibleBloomFilter diff_fail = small_a - small_b;
    std::vector<IBLTCell> fail_cells_before = diff_fail.cells();

    IBLTDecodeResult res_fail = diff_fail.decode();
    TEST_ASSERT(!res_fail.is_success(), "Overloaded decode must fail gracefully");
    TEST_ASSERT(res_fail.status == IBLTDecodeStatus::IncompleteResidual, "Status is IncompleteResidual");
    TEST_ASSERT(res_fail.unpeeled_cells > 0, "Non-zero unpeeled residual cells");

    for (size_t i = 0; i < fail_cells_before.size(); ++i) {
        TEST_ASSERT(diff_fail.cells()[i].count == fail_cells_before[i].count, "Count unchanged after failed decode");
    }
}

void test_determinism_across_instances() {
    std::cout << "[TEST 13] Deterministic Reproduction Across Instances...\n";

    InvertibleBloomFilter a1(100, 3);
    InvertibleBloomFilter a2(100, 3);
    InvertibleBloomFilter b1(100, 3);
    InvertibleBloomFilter b2(100, 3);

    for (uint32_t i = 1; i <= 15; ++i) {
        LandmarkKey k(1, 1, i, i * 777);
        a1.insert(k);
        a2.insert(k);
    }
    for (uint32_t i = 10; i <= 25; ++i) {
        LandmarkKey k(1, 2, i, i * 888);
        b1.insert(k);
        b2.insert(k);
    }

    IBLTDecodeResult res1 = a1.reconcile_with(b1);
    IBLTDecodeResult res2 = a2.reconcile_with(b2);

    TEST_ASSERT(res1.is_success() && res2.is_success(), "Both deterministic runs must succeed");
    TEST_ASSERT(res1.left_only == res2.left_only, "left_only keys must be identical across instances");
    TEST_ASSERT(res1.right_only == res2.right_only, "right_only keys must be identical across instances");
}

void test_deliberate_overload_bounded_behavior() {
    std::cout << "[TEST 14] Graceful Bounded Termination on Overloaded Table...\n";

    // 15 cells, 3 probes, difference of 50 keys (far beyond capacity)
    InvertibleBloomFilter a(15, 3);
    InvertibleBloomFilter b(15, 3);

    for (uint32_t i = 1; i <= 50; ++i) {
        a.insert(LandmarkKey(1, 1, i, i * 99));
    }

    InvertibleBloomFilter diff = a - b;
    IBLTDecodeResult res = diff.decode();

    TEST_ASSERT(!res.is_success(), "Massively overloaded table must fail decoding cleanly");
    TEST_ASSERT(res.status == IBLTDecodeStatus::IncompleteResidual, "Status is IncompleteResidual");
    TEST_ASSERT(res.unpeeled_cells > 0, "Residual unpeeled cells present");
}

void run_experimental_sanity_matrix() {
    std::cout << "\n=======================================================\n";
    std::cout << " AstroMesh-X: IBLT Capacity vs. Difference Experiment  \n";
    std::cout << "=======================================================\n";

    // Table configuration: m = 80 cells, k = 3 probes
    // Theoretical IBLT threshold: c * d <= m, with c ≈ 1.22 for k=3 (Goodrich & Mitzenmacher)
    // Thus d_max ≈ 80 / 1.22 ≈ 65 keys max.
    const size_t m = 80;
    const size_t k = 3;
    const size_t shared_keys = 200;

    std::cout << "  Table Cells (m):      " << m << "\n";
    std::cout << "  Probes (k):           " << k << "\n";
    std::cout << "  Memory Footprint:     " << (m * sizeof(IBLTCell)) << " bytes (measured: sizeof(IBLTCell)=" << sizeof(IBLTCell) << ")\n";
    std::cout << "  Shared Items Base:    " << shared_keys << "\n";
    std::cout << "-------------------------------------------------------\n";
    std::cout << "  Diff (d) | Trials | Success Count | Rate (%) | Correctness\n";
    std::cout << "-------------------------------------------------------\n";

    const std::vector<size_t> test_diffs = {2, 5, 10, 20, 40, 70};
    const size_t num_trials = 10;

    for (size_t d : test_diffs) {
        size_t success_count = 0;
        bool all_correct_when_successful = true;

        for (size_t trial = 0; trial < num_trials; ++trial) {
            InvertibleBloomFilter a(m, k);
            InvertibleBloomFilter b(m, k);

            // Insert 200 shared keys
            for (uint32_t i = 1; i <= shared_keys; ++i) {
                LandmarkKey shared(1, 1, i, static_cast<uint64_t>(i) * 0x11111111ULL + trial);
                a.insert(shared);
                b.insert(shared);
            }

            // Half of d in A, half in B
            size_t d_left = d / 2;
            size_t d_right = d - d_left;

            std::vector<LandmarkKey> expected_left;
            for (uint32_t i = 1; i <= d_left; ++i) {
                LandmarkKey lk(1, 2, i + 1000, static_cast<uint64_t>(i) * 0x22222222ULL + trial * 7);
                a.insert(lk);
                expected_left.push_back(lk);
            }

            std::vector<LandmarkKey> expected_right;
            for (uint32_t i = 1; i <= d_right; ++i) {
                LandmarkKey rk(1, 3, i + 2000, static_cast<uint64_t>(i) * 0x33333333ULL + trial * 13);
                b.insert(rk);
                expected_right.push_back(rk);
            }

            std::sort(expected_left.begin(), expected_left.end());
            std::sort(expected_right.begin(), expected_right.end());

            IBLTDecodeResult res = a.reconcile_with(b);

            if (res.is_success()) {
                success_count++;
                if (res.left_only != expected_left || res.right_only != expected_right) {
                    all_correct_when_successful = false;
                }
            }
        }

        double success_rate = (static_cast<double>(success_count) / static_cast<double>(num_trials)) * 100.0;
        std::cout << "  " << std::setw(8) << d << " | "
                  << std::setw(6) << num_trials << " | "
                  << std::setw(13) << success_count << " | "
                  << std::setw(7) << std::fixed << std::setprecision(1) << success_rate << "% | "
                  << (all_correct_when_successful ? "100% Exact" : "ERROR") << "\n";

        TEST_ASSERT(all_correct_when_successful, "Whenever IBLT decodes, output must be 100% exact");
        if (d <= 20) {
            // Under 80 cells, difference <= 20 is well within capacity and should decode reliably
            TEST_ASSERT(success_count == num_trials, "Differences <= 20 must decode with 100% success on m=80");
        }
    }
    std::cout << "=======================================================\n";
}

int main() {
    std::cout << "===============================================\n";
    std::cout << " AstroMesh-X: InvertibleBloomFilter Test Suite \n";
    std::cout << "===============================================\n";

    test_construction_and_validation();
    test_identical_sets();
    test_single_left_difference();
    test_single_right_difference();
    test_multiple_bidirectional_differences();
    test_duplicate_key_policy();
    test_large_values_and_boundaries();
    test_incompatible_tables();
    test_checksum_and_pure_cell_logic();
    test_table_immutability_during_decode();
    test_determinism_across_instances();
    test_deliberate_overload_bounded_behavior();

    run_experimental_sanity_matrix();

    std::cout << "===============================================\n";
    std::cout << " Total Assertions Checked: " << g_total_assertions << "\n";
    std::cout << " Failed Assertions:        " << g_failed_assertions << "\n";
    std::cout << "===============================================\n";

    if (g_failed_assertions == 0) {
        std::cout << " ALL IBLT TESTS PASSED SUCCESSFULLY.\n";
        return 0;
    } else {
        std::cerr << " IBLT TEST SUITE FAILED WITH " << g_failed_assertions << " ERRORS.\n";
        return 1;
    }
}
