#pragma once

#include "astromesh/core/landmark_key.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <string>
#include <optional>

namespace astromesh::core {

/**
 * @brief Status result of an insertion attempt into the Counting Bloom Filter.
 */
enum class CBFInsertResult {
    Inserted,          ///< Successfully inserted key.
    Duplicate,         ///< Key was already present (duplicate policy prevented re-insertion).
    InvalidKey,        ///< Key has invalid schema (schema == 0).
    CounterSaturated   ///< One or more counters reached capacity and were pinned.
};

/**
 * @brief Status result of a removal attempt from the Counting Bloom Filter.
 */
enum class CBFRemoveResult {
    Removed,           ///< Successfully removed key and decremented counters.
    NotPresent,        ///< Key was not in the filter (no counters decremented).
    InvalidKey,        ///< Key has invalid schema.
    PinnedCounterSkipped ///< Removed key, but one or more saturated counters were left pinned.
};

/**
 * @brief Configuration parameters for a Counting Bloom Filter.
 */
struct CBFConfig {
    size_t num_counters{0};     ///< Array size m (number of counters).
    size_t num_probes{0};       ///< Number of hash probes k.
    bool allow_duplicates{false}; ///< Whether duplicate insertions increment counters.
};

/**
 * @brief Counting Bloom Filter (CBF) implementation serving as the AstroMesh v1
 *        baseline for decentralized landmark-candidate discovery.
 *
 * Characteristics:
 * - Counter Width: 8-bit counters (0 to 255).
 * - Saturation Policy: Saturated counters (255) are pinned and never wrapped around,
 *   preventing silent corruption and avoiding false negatives.
 * - Deletion Safety Contract: To prevent unsafe deletions that would corrupt counters
 *   belonging to other keys (creating false negatives), an exact local insertion ledger
 *   is maintained. Attempts to remove a key that was not inserted are strictly rejected
 *   with CBFRemoveResult::NotPresent and touch zero counters.
 * - Deterministic Probing: Probe indices are generated using Kirsch-Mitzenmacher
 *   double hashing over the key's 16-byte binary payload, ensuring identical filter
 *   behavior regardless of compiler, STL, or OS architecture.
 */
class CountingBloomFilter {
public:
    static constexpr uint8_t MAX_COUNTER = 255;
    static constexpr size_t MAX_ALLOCATION_COUNTERS = 100'000'000; // 100M counters (~100 MB max)

    /**
     * @brief Construct CBF with explicit expected capacity and target false-positive probability.
     * @param expected_items Expected number of distinct items to insert (n > 0).
     * @param target_fpr Desired false-positive probability (0.0 < p < 1.0).
     * @param allow_duplicates If false, repeated insertions of an existing key are ignored.
     * @throws std::invalid_argument if parameters are out of bounds or calculation overflows.
     */
    CountingBloomFilter(size_t expected_items, double target_fpr, bool allow_duplicates = false);

    /**
     * @brief Construct CBF with explicit lower-level configuration struct.
     * @param config Explicit dimensions and policies.
     * @throws std::invalid_argument if parameters are zero or exceed limits.
     */
    explicit CountingBloomFilter(const CBFConfig& config);

    /**
     * @brief Construct CBF with explicit counter count and probe count.
     * @param num_counters Total counter array size m (num_counters > 0).
     * @param num_probes Number of hash functions k (num_probes > 0).
     * @param allow_duplicates If false, duplicate insertions are rejected.
     * @throws std::invalid_argument if parameters are zero or exceed limits.
     */
    static CountingBloomFilter create_with_dimensions(size_t num_counters, size_t num_probes, bool allow_duplicates = false);

    /**
     * @brief Safe factory method returning std::nullopt on invalid parameters.
     */
    static std::optional<CountingBloomFilter> try_create(size_t expected_items, double target_fpr,
                                                         bool allow_duplicates = false) noexcept;

    // --- Core Operations ---

    /**
     * @brief Insert a landmark key into the filter.
     * @return Detailed status of the insertion.
     */
    CBFInsertResult insert_detailed(const LandmarkKey& key);

    /**
     * @brief Convenience insertion returning true on successful insertion.
     */
    bool insert(const LandmarkKey& key);

    /**
     * @brief Query whether a landmark key may be present in the filter.
     * @note A return of true indicates the key is PROBABLY present (subject to FPR).
     *       A return of false indicates the key is DEFINITELY NOT present.
     */
    bool contains(const LandmarkKey& key) const noexcept;

    /**
     * @brief Safely remove a registered landmark key from the filter.
     * @note Only keys registered in the local ledger will decrement counters.
     *       Unregistered keys return NotPresent and do not modify any counters.
     */
    CBFRemoveResult remove_detailed(const LandmarkKey& key);

    /**
     * @brief Convenience removal returning true if key was registered and removed.
     */
    bool remove(const LandmarkKey& key);

    /**
     * @brief Clears all counters, reset statistics, and empties the insertion ledger.
     */
    void clear() noexcept;

    // --- Probing & Hashing (Public for deterministic inspection) ---

    /**
     * @brief Computes the k probe indices for a given landmark key.
     * @param key The landmark key to probe.
     * @param out_probes Output vector of size k populated with indices in [0, m-1].
     */
    void get_probe_indices(const LandmarkKey& key, std::vector<size_t>& out_probes) const noexcept;

    // --- State and Statistics Queries ---

    size_t num_counters() const noexcept { return num_counters_; }
    size_t num_probes() const noexcept { return num_probes_; }
    bool allow_duplicates() const noexcept { return allow_duplicates_; }

    size_t active_keys() const noexcept { return local_ledger_.size(); }
    size_t total_insertions() const noexcept { return total_insertions_; }
    size_t total_removals() const noexcept { return total_removals_; }
    size_t saturated_counters() const noexcept { return saturated_counters_; }

    /**
     * @brief Inspect the value of a specific counter slot.
     * @throws std::out_of_range if index >= num_counters().
     */
    uint8_t counter_at(size_t index) const;

    /**
     * @brief Computes theoretical false-positive rate given the current active item count:
     *        FPR = (1 - exp(-k * n / m))^k
     */
    double estimated_false_positive_rate() const noexcept;

    /**
     * @brief Returns a const reference to the raw counter array (useful for serialization).
     */
    const std::vector<uint8_t>& raw_counters() const noexcept { return counters_; }

private:
    size_t num_counters_{0};
    size_t num_probes_{0};
    bool allow_duplicates_{false};

    std::vector<uint8_t> counters_;
    std::unordered_map<LandmarkKey, uint32_t> local_ledger_;

    size_t total_insertions_{0};
    size_t total_removals_{0};
    size_t saturated_counters_{0};

    // Deterministic 128-bit hash derivation for key
    static void compute_double_hashes(const LandmarkKey& key, uint64_t& out_h1, uint64_t& out_h2) noexcept;
};

} // namespace astromesh::core
