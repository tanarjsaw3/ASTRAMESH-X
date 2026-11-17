#pragma once

#include "astromesh/graph/conflict_graph_merger.hpp"
#include "astromesh/net/partition_reconciler.hpp"

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace astromesh::verification {

struct ConsistencyAuditResult {
    bool is_converged{false};
    bool version_vectors_equal{false};
    size_t dsu_components{0};
    size_t total_keyframes{0};
    size_t total_verified_edges{0};
    size_t pending_sync_items{0};
    double reciprocal_edge_fraction{1.0};
    std::string diagnostic_summary;
};

/**
 * @brief Auditor verifying mathematically rigorous swarm-wide consistency.
 * Validates Version Vector equality, DSU connected components, reciprocal edge recognition,
 * and zero residual synchronization backlog across the swarm.
 */
class GlobalConsistencyAuditor {
public:
    GlobalConsistencyAuditor() = default;

    /**
     * @brief Performs a full consistency audit across a collection of rovers.
     * @param reconcilers Map of rover_id -> pointer to PartitionAwareReconciler
     * @param mergers Map of rover_id -> pointer to ConflictAwareGraphMerger
     * @param expected_dsu_components Expected number of connected components (1 if all mutually reachable)
     */
    static ConsistencyAuditResult audit(
        const std::map<uint16_t, const net::PartitionAwareReconciler*>& reconcilers,
        const std::map<uint16_t, const graph::ConflictAwareGraphMerger*>& mergers,
        size_t expected_dsu_components = 1);
};

} // namespace astromesh::verification

