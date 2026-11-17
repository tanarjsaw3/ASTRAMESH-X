#pragma once

#include "astromesh/core/landmark_key.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>
#include <unordered_set>
#include <string>
#include <optional>

namespace astromesh::core {

/**
 * @brief Represents a single cell in the Invertible Bloom Lookup Table.
 */
struct IBLTCell {
    int32_t count{0};             ///< Net count of items mapped to this cell.
    uint64_t key_sum_low{0};      ///< XOR sum of key low 64 bits (schema, rover_id, local_index).
    uint64_t key_sum_high{0};     ///< XOR sum of key high 64 bits (descriptor_hash).
    uint64_t checksum_sum{0};     ///< XOR sum of 64-bit cryptographic/mixing checksums of keys.

    bool is_empty() const noexcept {
        return count == 0 && key_sum_low == 0 && key_sum_high == 0 && checksum_sum == 0;
    }
};

/**
 * @brief Status enumeration for IBLT difference decoding.
 */
enum class IBLTDecodeStatus {
    Success,            ///< Complete symmetric difference was successfully peeled; residual table is empty.
    IncompleteResidual, ///< Peeling stalled; difference cardinality exceeds table recovery capacity.
    ArithmeticOverflow, ///< Integer overflow/underflow detected in cell counts.
    IncompatibleTables  ///< Tables being reconciled have differing configurations or hash parameters.
};

/**
 * @brief Result structure returned by IBLT difference decoding.
 */
struct IBLTDecodeResult {
    IBLTDecodeStatus status{IBLTDecodeStatus::IncompleteResidual};
    std::vector<LandmarkKey> left_only;  ///< Keys present only in left table (A \ B).
    std::vector<LandmarkKey> right_only; ///< Keys present only in right table (B \ A).
    size_t unpeeled_cells{0};            ///< Number of non-zero cells remaining in residual table.

    bool is_success() const noexcept {
        return status == IBLTDecodeStatus::Success;
    }
};

/**
 * @brief Configuration parameters for an Invertible Bloom Lookup Table.
 */
struct IBLTConfig {
    size_t num_cells{0};       ///< Number of cells in the table (m > 0).
    size_t num_probes{3};      ///< Number of hash probes per key (default: 3, per Goodrich & Mitzenmacher).
    uint32_t protocol_id{1};   ///< Protocol / schema version identifier to ensure table compatibility.
};

/**
 * @brief Invertible Bloom Lookup Table (IBLT) implementation for AstroMesh-X.
 *
 * An IBLT is an invertible probabilistic data structure used to reconcile set
 * differences between two decentralized peers. Given table A from rover A and
 * table B from rover B:
 *
 *   T_diff = A - B
 *
 * Decoding T_diff peels out the symmetric difference (A \ B) and (B \ A),
 * provided the total difference size |A △ B| <= d_max (the table's decoding capacity).
 *
 * CRITICAL SCIENTIFIC DISTINCTION:
 * An IBLT does NOT directly compute the intersection of sets A and B. It reconciles
 * the set differences between them.
 */
class InvertibleBloomFilter {
public:
    static constexpr size_t DEFAULT_PROBES = 3;
    static constexpr size_t MAX_ALLOCATION_CELLS = 1'000'000; // 1M cells (~32 MB max)

    /**
     * @brief Construct an IBLT with explicit configuration.
     * @param config Table configuration parameters.
     * @throws std::invalid_argument if num_cells is 0 or num_probes is invalid.
     */
    explicit InvertibleBloomFilter(const IBLTConfig& config);

    /**
     * @brief Convenience constructor with explicit cell count and probe count.
     */
    explicit InvertibleBloomFilter(size_t num_cells, size_t num_probes = DEFAULT_PROBES, uint32_t protocol_id = 1);

    /**
     * @brief Safe factory method returning std::nullopt on invalid parameters.
     */
    static std::optional<InvertibleBloomFilter> try_create(
        size_t num_cells, size_t num_probes = DEFAULT_PROBES, uint32_t protocol_id = 1) noexcept;

    // --- Core Operations ---

    /**
     * @brief Insert a landmark key into the IBLT.
     * @note Enforces set semantics: duplicate insertion of an already present key is rejected.
     * @return true if key was valid and newly inserted; false if duplicate or invalid.
     */
    bool insert(const LandmarkKey& key);

    /**
     * @brief Remove a previously inserted landmark key from the IBLT.
     * @return true if key was registered and removed; false if not present or invalid.
     */
    bool remove(const LandmarkKey& key);

    /**
     * @brief Computes the difference table T_diff = (*this) - other.
     * @param other The right-hand IBLT to subtract.
     * @return New IBLT representing the difference between the two tables.
     * @throws std::invalid_argument if tables have incompatible configurations.
     */
    InvertibleBloomFilter subtract(const InvertibleBloomFilter& other) const;

    /**
     * @brief Overload of subtraction operator: T_diff = a - b.
     */
    InvertibleBloomFilter operator-(const InvertibleBloomFilter& other) const {
        return subtract(other);
    }

    /**
     * @brief Decode the symmetric difference from this table.
     * @note Operates on an internal copy; this table is NEVER mutated by decode().
     * @return Result containing left_only (A \ B) and right_only (B \ A) keys.
     */
    IBLTDecodeResult decode() const;

    /**
     * @brief Reconcile this table with a peer table directly: (this) - peer -> decode().
     */
    IBLTDecodeResult reconcile_with(const InvertibleBloomFilter& peer) const {
        return subtract(peer).decode();
    }

    /**
     * @brief Resets all cells, counters, and the local insertion ledger.
     */
    void clear() noexcept;

    // --- Introspection & Diagnostics ---

    size_t num_cells() const noexcept { return config_.num_cells; }
    size_t num_probes() const noexcept { return config_.num_probes; }
    uint32_t protocol_id() const noexcept { return config_.protocol_id; }
    size_t active_keys() const noexcept { return local_set_.size(); }

    const std::vector<IBLTCell>& cells() const noexcept { return cells_; }
    std::vector<IBLTCell>& mutable_cells() noexcept { return cells_; }
    const IBLTCell& cell_at(size_t index) const;

    /**
     * @brief Returns estimated memory footprint of the cell table in bytes.
     */
    size_t estimated_table_memory_bytes() const noexcept {
        return cells_.size() * sizeof(IBLTCell);
    }

    /**
     * @brief Checks compatibility between two IBLTs before subtraction.
     */
    bool is_compatible_with(const InvertibleBloomFilter& other) const noexcept {
        return config_.num_cells == other.config_.num_cells &&
               config_.num_probes == other.config_.num_probes &&
               config_.protocol_id == other.config_.protocol_id;
    }

    /**
     * @brief Generates the k distinct cell indices for a given key.
     */
    void get_cell_indices(const LandmarkKey& key, std::vector<size_t>& out_indices) const noexcept;

    // --- Key Binary Serialization Helpers (Public for testing pure cell logic) ---

    static void key_to_bits(const LandmarkKey& key, uint64_t& out_low, uint64_t& out_high) noexcept;
    static LandmarkKey bits_to_key(uint64_t low, uint64_t high) noexcept;
    static uint64_t compute_checksum(const LandmarkKey& key) noexcept;

private:
    IBLTConfig config_;
    std::vector<IBLTCell> cells_;
    std::unordered_set<LandmarkKey> local_set_; // Local insertion ledger enforcing set semantics

    // Internal helper for pure cell peeling
    static bool is_pure_cell(const IBLTCell& cell, LandmarkKey& out_key) noexcept;
};

} // namespace astromesh::core
