#pragma once

#include "astromesh/core/iblt.hpp"
#include "astromesh/core/version_vector.hpp"
#include "astromesh/graph/map_update.hpp"

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace astromesh::net {

enum class ReconcileRoundType : uint8_t {
    VersionVectorExchange = 1,
    IBLTFingerprint = 2,
    ChunkedRetry = 3,
    Complete = 4
};

enum class ReconciliationStatus {
    AlreadySynchronized,
    PeelingSucceeded,
    MultiRoundInProgress,
    DecodeFailedRetrying,
    Completed
};

struct ReconcilerMetrics {
    uint32_t rounds_executed{0};
    uint64_t wire_bytes_transmitted{0};
    uint32_t decodes_attempted{0};
    uint32_t decodes_succeeded{0};
    uint32_t decodes_failed{0};
    uint32_t missing_updates_extracted{0};
    uint32_t duplicate_updates_suppressed{0};
    uint64_t elapsed_ns{0};
    size_t memory_bytes_used{0};

    double decode_success_rate() const noexcept {
        if (decodes_attempted == 0) return 1.0;
        return static_cast<double>(decodes_succeeded) / static_cast<double>(decodes_attempted);
    }
};

/**
 * @brief Partition-Aware Multi-Round IBLT Reconciliation Engine.
 * Combines causal Version Vectors with progressive difference chunking to guarantee
 * convergence even when partition divergence exceeds single-IBLT cell capacity.
 */
class PartitionAwareReconciler {
public:
    static constexpr size_t DEFAULT_IBLT_CELLS = 32;
    static constexpr size_t DEFAULT_IBLT_PROBES = 3;
    static constexpr size_t MAX_LEDGER_ENTRIES = 5000;

    explicit PartitionAwareReconciler(
        uint16_t local_rover_id,
        size_t iblt_cells = DEFAULT_IBLT_CELLS,
        size_t iblt_probes = DEFAULT_IBLT_PROBES);

    uint16_t local_rover_id() const noexcept { return m_local_rover_id; }
    const core::VersionVector& version_vector() const noexcept { return m_version_vector; }

    /**
     * @brief Ingests a new local map update into the ledger, incrementing local sequence.
     */
    uint64_t record_local_update(graph::MapUpdate update);

    /**
     * @brief Applies an incoming remote map update if causally valid and not duplicate.
     * @return true if accepted and applied; false if duplicate or causally stale.
     */
    bool apply_remote_update(const graph::MapUpdate& update);

    /**
     * @brief Evaluates whether reconciliation is required with a peer.
     */
    bool needs_reconciliation(const core::VersionVector& peer_version) const noexcept;

    /**
     * @brief Creates an IBLT fingerprint covering updates not yet witnessed by peer.
     * Filters out causally dominated updates to prevent re-exchanging synchronized data.
     */
    core::InvertibleBloomFilter build_delta_iblt(
        const core::VersionVector& peer_version,
        uint8_t hash_prefix_filter = 0xFF,
        uint8_t hash_prefix_val = 0) const;

    /**
     * @brief Reconciles local state with peer's IBLT.
     * If peeling succeeds, extracts missing keys. If peeling fails, signals chunked retry.
     */
    ReconciliationStatus reconcile_iblt_round(
        const core::InvertibleBloomFilter& peer_iblt,
        const core::VersionVector& peer_version,
        std::vector<core::LandmarkKey>& out_missing_keys_for_peer,
        std::vector<core::LandmarkKey>& out_keys_to_request_from_peer);

    /**
     * @brief Executes multi-round partition recovery chunking until all differences are resolved.
     */
    ReconciliationStatus execute_multi_round_recovery(
        PartitionAwareReconciler& peer_reconciler,
        ReconcilerMetrics& out_metrics);

    /**
     * @brief Baseline: Full-Map Synchronous Exchange (for scientific comparison).
     */
    static void execute_full_map_baseline(
        PartitionAwareReconciler& r1,
        PartitionAwareReconciler& r2,
        ReconcilerMetrics& out_metrics);

    /**
     * @brief Retrieves updates by LandmarkKey.
     */
    std::optional<graph::MapUpdate> get_update_by_key(const core::LandmarkKey& key) const;

    const std::vector<graph::MapUpdate>& ledger() const noexcept { return m_ledger; }
    const ReconcilerMetrics& metrics() const noexcept { return m_metrics; }

private:
    uint16_t m_local_rover_id;
    size_t m_iblt_cells;
    size_t m_iblt_probes;
    core::VersionVector m_version_vector;

    std::vector<graph::MapUpdate> m_ledger;
    std::unordered_map<uint64_t, size_t> m_update_id_to_idx;
    std::unordered_map<core::LandmarkKey, size_t> m_landmark_key_to_idx;

    ReconcilerMetrics m_metrics;
};

} // namespace astromesh::net

