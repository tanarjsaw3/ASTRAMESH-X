#include "astromesh/core/landmark_key.hpp"

#include <iostream>
#include <vector>
#include <set>
#include <unordered_set>
#include <map>
#include <unordered_map>
#include <stdexcept>
#include <cassert>
#include <sstream>

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

void test_construction_and_validity() {
    std::cout << "[TEST] Construction and Validity...\n";

    // 1. Default constructor creates invalid key
    LandmarkKey k_default;
    TEST_ASSERT(!k_default.is_valid(), "Default constructed key must be invalid");
    TEST_ASSERT(k_default == LandmarkKey::invalid(), "Default constructed key must match LandmarkKey::invalid()");
    TEST_ASSERT(k_default.schema_version() == LandmarkKey::INVALID_SCHEMA, "Invalid key has schema 0");

    // 2. Valid explicit construction
    LandmarkKey k1(1, 42, 1001, 0xDEADBEEFCAFEULL);
    TEST_ASSERT(k1.is_valid(), "Explicit key with schema 1 must be valid");
    TEST_ASSERT(k1.schema_version() == 1, "schema_version must match");
    TEST_ASSERT(k1.rover_id() == 42, "rover_id must match");
    TEST_ASSERT(k1.local_index() == 1001, "local_index must match");
    TEST_ASSERT(k1.descriptor_hash() == 0xDEADBEEFCAFEULL, "descriptor_hash must match");

    // 3. Exception on invalid schema version
    bool threw_expected = false;
    try {
        LandmarkKey k_bad(0, 1, 2, 3);
    } catch (const std::invalid_argument&) {
        threw_expected = true;
    }
    TEST_ASSERT(threw_expected, "Constructor with schema_version 0 must throw std::invalid_argument");

    // 4. try_create factory
    LandmarkKey k_out;
    bool ok = LandmarkKey::try_create(0, 10, 20, 30, k_out);
    TEST_ASSERT(!ok, "try_create with schema 0 must return false");

    ok = LandmarkKey::try_create(2, 10, 20, 30, k_out);
    TEST_ASSERT(ok, "try_create with schema 2 must return true");
    TEST_ASSERT(k_out.is_valid(), "Output key from try_create must be valid");
    TEST_ASSERT(k_out.schema_version() == 2, "Output key schema must match");
}

void test_equality_and_inequality() {
    std::cout << "[TEST] Equality and Inequality Semantics...\n";

    LandmarkKey base(1, 10, 100, 0x12345678ULL);
    LandmarkKey identical(1, 10, 100, 0x12345678ULL);

    TEST_ASSERT(base == identical, "Identical keys must compare equal");
    TEST_ASSERT(identical == base, "Equality must be symmetric");
    TEST_ASSERT(!(base != identical), "base != identical must be false");

    // Differ by schema
    LandmarkKey diff_schema(2, 10, 100, 0x12345678ULL);
    TEST_ASSERT(base != diff_schema, "Different schema must not compare equal");
    TEST_ASSERT(!(base == diff_schema), "base == diff_schema must be false");

    // Differ by rover_id
    LandmarkKey diff_rover(1, 11, 100, 0x12345678ULL);
    TEST_ASSERT(base != diff_rover, "Different rover_id must not compare equal");

    // Differ by local_index
    LandmarkKey diff_index(1, 10, 101, 0x12345678ULL);
    TEST_ASSERT(base != diff_index, "Different local_index must not compare equal");

    // Differ by descriptor_hash
    LandmarkKey diff_hash(1, 10, 100, 0x12345679ULL);
    TEST_ASSERT(base != diff_hash, "Different descriptor_hash must not compare equal");
}

void test_ordering_and_containers() {
    std::cout << "[TEST] Ordering Semantics and Ordered Containers...\n";

    LandmarkKey k1(1, 1, 10, 100);
    LandmarkKey k2(1, 1, 10, 200);
    LandmarkKey k3(1, 1, 20, 50);
    LandmarkKey k4(1, 2, 5, 50);
    LandmarkKey k5(2, 1, 1, 1);

    // Asymmetry and strict weak ordering
    TEST_ASSERT(k1 < k2, "k1 must be < k2 based on hash");
    TEST_ASSERT(!(k2 < k1), "k2 < k1 must be false");
    TEST_ASSERT(k2 < k3, "k2 must be < k3 based on local_index");
    TEST_ASSERT(k1 < k3, "Transitivity: k1 < k3");
    TEST_ASSERT(k3 < k4, "k3 must be < k4 based on rover_id");
    TEST_ASSERT(k4 < k5, "k4 must be < k5 based on schema_version");

    // std::set usage
    std::set<LandmarkKey> key_set;
    key_set.insert(k3);
    key_set.insert(k1);
    key_set.insert(k5);
    key_set.insert(k2);
    key_set.insert(k4);
    key_set.insert(k1); // Duplicate

    TEST_ASSERT(key_set.size() == 5, "std::set must contain exactly 5 unique keys");

    // Verify sorted order
    auto it = key_set.begin();
    TEST_ASSERT(*it++ == k1, "Element 0 in set must be k1");
    TEST_ASSERT(*it++ == k2, "Element 1 in set must be k2");
    TEST_ASSERT(*it++ == k3, "Element 2 in set must be k3");
    TEST_ASSERT(*it++ == k4, "Element 3 in set must be k4");
    TEST_ASSERT(*it++ == k5, "Element 4 in set must be k5");

    // std::map usage
    std::map<LandmarkKey, std::string> key_map;
    key_map[k1] = "First";
    key_map[k2] = "Second";
    TEST_ASSERT(key_map[k1] == "First", "std::map lookup must succeed");
    TEST_ASSERT(key_map.find(k3) == key_map.end(), "std::map uninserted key not found");
}

void test_hash_and_unordered_containers() {
    std::cout << "[TEST] Hash Quality and Unordered Containers...\n";

    std::hash<LandmarkKey> hasher;

    LandmarkKey k1(1, 5, 100, 0xABCDEF01ULL);
    LandmarkKey k1_dup(1, 5, 100, 0xABCDEF01ULL);
    LandmarkKey k2(1, 5, 100, 0xABCDEF02ULL);

    TEST_ASSERT(hasher(k1) == hasher(k1_dup), "Identical keys must produce identical hash");
    TEST_ASSERT(hasher(k1) != hasher(k2), "Slightly differing keys should produce distinct hashes");

    // std::unordered_set
    std::unordered_set<LandmarkKey> u_set;
    for (uint32_t i = 0; i < 500; ++i) {
        u_set.insert(LandmarkKey(1, 1, i, static_cast<uint64_t>(i) * 0x5851f42d4c957f2dULL));
    }
    TEST_ASSERT(u_set.size() == 500, "std::unordered_set must contain all 500 keys without lost entries");

    // Query elements
    LandmarkKey query_key(1, 1, 250, static_cast<uint64_t>(250) * 0x5851f42d4c957f2dULL);
    TEST_ASSERT(u_set.find(query_key) != u_set.end(), "Query key must be found in unordered_set");

    LandmarkKey absent_key(1, 1, 9999, 0);
    TEST_ASSERT(u_set.find(absent_key) == u_set.end(), "Absent key must not be found");

    // std::unordered_map
    std::unordered_map<LandmarkKey, uint64_t> u_map;
    u_map[k1] = 42;
    TEST_ASSERT(u_map[k1_dup] == 42, "unordered_map lookup by identical key must succeed");
}

void test_edge_cases_and_formatting() {
    std::cout << "[TEST] Edge Cases and String Formatting...\n";

    // Max values
    LandmarkKey max_key(0xFFFF, 0xFFFF, 0xFFFFFFFF, 0xFFFFFFFFFFFFFFFFULL);
    TEST_ASSERT(max_key.is_valid(), "Key with UINT max values must be valid");
    TEST_ASSERT(max_key.schema_version() == 0xFFFF, "Max schema must match");
    TEST_ASSERT(max_key.rover_id() == 0xFFFF, "Max rover_id must match");
    TEST_ASSERT(max_key.local_index() == 0xFFFFFFFF, "Max local_index must match");
    TEST_ASSERT(max_key.descriptor_hash() == 0xFFFFFFFFFFFFFFFFULL, "Max descriptor_hash must match");

    // Zero field values with valid schema
    LandmarkKey zeros_key(1, 0, 0, 0);
    TEST_ASSERT(zeros_key.is_valid(), "Key with zero fields but valid schema must be valid");
    TEST_ASSERT(zeros_key.rover_id() == 0, "Rover id 0 valid");

    // String formatting
    LandmarkKey inv_key = LandmarkKey::invalid();
    TEST_ASSERT(inv_key.to_string() == "LM[INVALID]", "Invalid key to_string must match sentinel string");

    LandmarkKey fmt_key(1, 2, 3, 0x1A2B);
    std::string str = fmt_key.to_string();
    TEST_ASSERT(str.find("s:1") != std::string::npos, "to_string contains schema");
    TEST_ASSERT(str.find("r:2") != std::string::npos, "to_string contains rover");
    TEST_ASSERT(str.find("i:3") != std::string::npos, "to_string contains index");
    TEST_ASSERT(str.find("0000000000001a2b") != std::string::npos, "to_string contains padded hex hash");

    // Stream operator
    std::ostringstream oss;
    oss << fmt_key;
    TEST_ASSERT(oss.str() == str, "Stream operator output must match to_string()");
}

int main() {
    std::cout << "========================================\n";
    std::cout << " AstroMesh-X: LandmarkKey Test Suite    \n";
    std::cout << "========================================\n";

    test_construction_and_validity();
    test_equality_and_inequality();
    test_ordering_and_containers();
    test_hash_and_unordered_containers();
    test_edge_cases_and_formatting();

    std::cout << "========================================\n";
    std::cout << " Total Assertions Checked: " << g_total_assertions << "\n";
    std::cout << " Failed Assertions:        " << g_failed_assertions << "\n";
    std::cout << "========================================\n";

    if (g_failed_assertions == 0) {
        std::cout << " ALL TESTS PASSED SUCCESSFULLY.\n";
        return 0;
    } else {
        std::cerr << " TEST SUITE FAILED WITH " << g_failed_assertions << " ERRORS.\n";
        return 1;
    }
}
