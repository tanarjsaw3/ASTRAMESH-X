#include "astromesh/verification/global_consistency_auditor.hpp"

#include <sstream>
#include <set>

namespace astromesh::verification {

ConsistencyAuditResult GlobalConsistencyAuditor::audit(
    const std::map<uint16_t, const net::PartitionAwareReconciler*>& reconcilers,
    const std::map<uint16_t, const graph::ConflictAwareGraphMerger*>& mergers,
    size_t expected_dsu_components) {

    ConsistencyAuditResult result;
    if (reconcilers.empty() || mergers.empty()) {
        result.is_converged = false;
        result.diagnostic_summary = "Audit failed: empty rover set.";
        return result;
    }

    // 1. Version Vector Equality Check
    result.version_vectors_equal = true;
    const auto* first_vv = &reconcilers.begin()->second->version_vector();
    for (const auto& [rid, rptr] : reconcilers) {
        if (rptr->version_vector() != *first_vv) {
            result.version_vectors_equal = false;
            break;
        }
    }

    // 2. Global Swarm DSU Component Evaluation
    graph::DisjointSet global_dsu;
    size_t total_kf = 0;
    size_t total_edges = 0;

    for (const auto& [rid, mptr] : mergers) {
        for (const auto& [nid, node] : mptr->graph().nodes()) {
            global_dsu.make_set(nid);
        }
        total_kf += mptr->graph().num_nodes();
        total_edges += mptr->graph().num_edges();
    }

    for (const auto& [rid, mptr] : mergers) {
        for (const auto& [eid, edge] : mptr->graph().edges()) {
            if (edge.status == graph::EdgeVerificationStatus::Verified || edge.type == graph::EdgeType::Odometry) {
                global_dsu.unite(edge.from_node, edge.to_node);
            }
        }
    }

    result.dsu_components = global_dsu.num_components();
    result.total_keyframes = total_kf / mergers.size();
    result.total_verified_edges = total_edges / mergers.size();

    // 3. Pending Sync Items
    size_t pending = 0;
    for (auto it1 = reconcilers.begin(); it1 != reconcilers.end(); ++it1) {
        for (auto it2 = std::next(it1); it2 != reconcilers.end(); ++it2) {
            if (it1->second->needs_reconciliation(it2->second->version_vector())) {
                pending++;
            }
        }
    }
    result.pending_sync_items = pending;

    // 4. Reciprocal Verified Edge Fraction
    size_t total_inter_edges = 0;
    size_t reciprocated_edges = 0;

    for (const auto& [rid, mptr] : mergers) {
        for (const auto& [eid, edge] : mptr->graph().edges()) {
            if (edge.type == graph::EdgeType::InterRoverLoopClosure &&
                edge.status == graph::EdgeVerificationStatus::Verified) {
                total_inter_edges++;
                uint16_t remote_rover = (edge.from_node.rover_id == rid) ? edge.to_node.rover_id : edge.from_node.rover_id;
                auto remote_it = mergers.find(remote_rover);
                if (remote_it != mergers.end()) {
                    if (remote_it->second->is_reciprocal_verified(edge.from_node, edge.to_node)) {
                        reciprocated_edges++;
                    }
                }
            }
        }
    }

    if (total_inter_edges == 0) {
        result.reciprocal_edge_fraction = 1.0;
    } else {
        result.reciprocal_edge_fraction = static_cast<double>(reciprocated_edges) / static_cast<double>(total_inter_edges);
    }

    // 5. Final Convergence Verdict
    bool dsu_ok = (result.dsu_components <= expected_dsu_components);
    bool sync_ok = (result.pending_sync_items == 0);
    bool recip_ok = (result.reciprocal_edge_fraction >= 0.999);

    result.is_converged = result.version_vectors_equal && dsu_ok && sync_ok && recip_ok;

    std::ostringstream oss;
    oss << "Swarm Audit: " << (result.is_converged ? "CONVERGED" : "NOT CONVERGED")
        << " [VV_Equal=" << (result.version_vectors_equal ? "YES" : "NO")
        << ", DSU_Components=" << result.dsu_components << " (expected " << expected_dsu_components << ")"
        << ", PendingSyncPairs=" << result.pending_sync_items
        << ", ReciprocalFraction=" << result.reciprocal_edge_fraction << "]";
    result.diagnostic_summary = oss.str();

    return result;
}

} // namespace astromesh::verification
