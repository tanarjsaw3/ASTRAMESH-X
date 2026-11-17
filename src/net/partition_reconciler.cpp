#include "astromesh/net/partition_reconciler.hpp"

#include <chrono>

namespace astromesh::net {

PartitionAwareReconciler::PartitionAwareReconciler(
    uint16_t local_rover_id,
    size_t iblt_cells,
    size_t iblt_probes)
    : m_local_rover_id(local_rover_id),
      m_iblt_cells(iblt_cells),
      m_iblt_probes(iblt_probes) {
}

uint64_t PartitionAwareReconciler::record_local_update(graph::MapUpdate update) {
    uint64_t seq = m_version_vector.increment(m_local_rover_id);
    update.origin_rover_id = m_local_rover_id;
    update.sequence = seq;
    update.update_id = (static_cast<uint64_t>(m_local_rover_id) << 48) | (seq & 0x0000FFFFFFFFFFFFULL);
    update.causal_context = m_version_vector;

    size_t idx = m_ledger.size();
    m_update_id_to_idx[update.update_id] = idx;
    if (update.landmark_key.is_valid()) {
        m_landmark_key_to_idx[update.landmark_key] = idx;
    }
    m_ledger.push_back(std::move(update));

    m_metrics.memory_bytes_used = m_ledger.size() * sizeof(graph::MapUpdate);
    return seq;
}

bool PartitionAwareReconciler::apply_remote_update(const graph::MapUpdate& update) {
    if (update.origin_rover_id == 0 || update.sequence == 0) {
        return false;
    }

    // 1. Idempotency Check: Have we already causally witnessed or stored this update?
    if (m_update_id_to_idx.find(update.update_id) != m_update_id_to_idx.end()) {
        m_metrics.duplicate_updates_suppressed++;
        return false;
    }

    // 2. Ingest valid update into local ledger
    size_t idx = m_ledger.size();
    m_update_id_to_idx[update.update_id] = idx;
    if (update.landmark_key.is_valid()) {
        m_landmark_key_to_idx[update.landmark_key] = idx;
    }
    m_ledger.push_back(update);

    // 3. Advance local version vector for the update's origin (max observed sequence)
    uint64_t current_seq = m_version_vector.get(update.origin_rover_id);
    m_version_vector.set(update.origin_rover_id, std::max(current_seq, update.sequence));

    // Merge any other causal context carried by the update
    m_version_vector = m_version_vector.merge_max(update.causal_context);

    m_metrics.missing_updates_extracted++;
    m_metrics.memory_bytes_used = m_ledger.size() * sizeof(graph::MapUpdate);
    return true;
}

bool PartitionAwareReconciler::needs_reconciliation(const core::VersionVector& peer_version) const noexcept {
    return m_version_vector != peer_version;
}

core::InvertibleBloomFilter PartitionAwareReconciler::build_delta_iblt(
    const core::VersionVector& peer_version,
    uint8_t hash_prefix_filter,
    uint8_t hash_prefix_val) const {

    core::InvertibleBloomFilter filter(m_iblt_cells, m_iblt_probes, 1);

    for (const auto& update : m_ledger) {
        if (!update.landmark_key.is_valid()) continue;

        // Skip updates that the peer is known to already dominate causally
        if (peer_version.get(update.origin_rover_id) >= update.sequence) {
            continue;
        }

        // Apply hash prefix filter for multi-round chunking
        if (hash_prefix_filter != 0xFF) {
            uint8_t prefix = static_cast<uint8_t>(update.landmark_key.descriptor_hash() & hash_prefix_filter);
            if (prefix != hash_prefix_val) {
                continue;
            }
        }

        filter.insert(update.landmark_key);
    }

    return filter;
}

ReconciliationStatus PartitionAwareReconciler::reconcile_iblt_round(
    const core::InvertibleBloomFilter& peer_iblt,
    const core::VersionVector& peer_version,
    std::vector<core::LandmarkKey>& out_missing_keys_for_peer,
    std::vector<core::LandmarkKey>& out_keys_to_request_from_peer) {

    m_metrics.rounds_executed++;
    m_metrics.decodes_attempted++;

    // 1. Build local delta IBLT
    auto local_iblt = build_delta_iblt(peer_version);

    // 2. Subtract peer IBLT to isolate symmetric difference: (Local - Peer)
    core::InvertibleBloomFilter diff(m_iblt_cells, m_iblt_probes, 1);
    try {
        diff = local_iblt.subtract(peer_iblt);
    } catch (const std::exception&) {
        m_metrics.decodes_failed++;
        return ReconciliationStatus::DecodeFailedRetrying;
    }

    // 3. Peel IBLT difference
    auto decode_res = diff.decode();
    if (!decode_res.is_success()) {
        m_metrics.decodes_failed++;
        return ReconciliationStatus::MultiRoundInProgress;
    }

    m_metrics.decodes_succeeded++;
    out_missing_keys_for_peer = decode_res.left_only;
    out_keys_to_request_from_peer = decode_res.right_only;

    return ReconciliationStatus::PeelingSucceeded;
}

ReconciliationStatus PartitionAwareReconciler::execute_multi_round_recovery(
    PartitionAwareReconciler& peer,
    ReconcilerMetrics& out_metrics) {

    auto start_time = std::chrono::steady_clock::now();

    // Round 0: Version Vector Fast Check
    out_metrics.rounds_executed++;
    out_metrics.wire_bytes_transmitted += 2 * core::VersionVector::WIRE_SIZE;

    if (m_version_vector == peer.version_vector()) {
        auto end_time = std::chrono::steady_clock::now();
        out_metrics.elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
        return ReconciliationStatus::AlreadySynchronized;
    }

    // Round 1: Fast Single-Round IBLT
    out_metrics.rounds_executed++;
    out_metrics.wire_bytes_transmitted += 2 * (12 + m_iblt_cells * 32); // 2 * 1036B
    out_metrics.decodes_attempted += 2;

    auto iblt_local = build_delta_iblt(peer.version_vector());
    auto iblt_peer = peer.build_delta_iblt(m_version_vector);

    std::vector<core::LandmarkKey> missing_for_peer;
    std::vector<core::LandmarkKey> missing_for_local;

    auto status = reconcile_iblt_round(iblt_peer, peer.version_vector(), missing_for_peer, missing_for_local);

    if (status == ReconciliationStatus::PeelingSucceeded) {
        out_metrics.decodes_succeeded += 2;

        // Exchange extracted missing updates
        for (const auto& k : missing_for_peer) {
            auto u_opt = get_update_by_key(k);
            if (u_opt.has_value()) {
                peer.apply_remote_update(*u_opt);
                out_metrics.missing_updates_extracted++;
                out_metrics.wire_bytes_transmitted += sizeof(graph::MapUpdate);
            }
        }
        for (const auto& k : missing_for_local) {
            auto u_opt = peer.get_update_by_key(k);
            if (u_opt.has_value()) {
                apply_remote_update(*u_opt);
                out_metrics.missing_updates_extracted++;
                out_metrics.wire_bytes_transmitted += sizeof(graph::MapUpdate);
            }
        }

        // Causal Version Vector synchronization
        m_version_vector = m_version_vector.merge_max(peer.version_vector());
        peer.m_version_vector = peer.m_version_vector.merge_max(m_version_vector);

        auto end_time = std::chrono::steady_clock::now();
        out_metrics.elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
        return ReconciliationStatus::Completed;
    }

    // Round 2+: Progressive Multi-Round Chunking (prefix mask 0x03 -> 4 chunks)
    out_metrics.decodes_failed++;
    constexpr uint8_t CHUNK_MASK = 0x03; // 4 sub-IBLT chunks

    const auto base_local_version = m_version_vector;
    const auto base_peer_version = peer.version_vector();

    for (uint8_t chunk = 0; chunk < 4; ++chunk) {
        out_metrics.rounds_executed++;
        out_metrics.wire_bytes_transmitted += 2 * (12 + m_iblt_cells * 32);
        out_metrics.decodes_attempted += 2;

        auto sub_local = build_delta_iblt(base_peer_version, CHUNK_MASK, chunk);
        auto sub_peer = peer.build_delta_iblt(base_local_version, CHUNK_MASK, chunk);

        std::vector<core::LandmarkKey> chunk_for_peer;
        std::vector<core::LandmarkKey> chunk_for_local;

        core::InvertibleBloomFilter sub_diff(m_iblt_cells, m_iblt_probes, 1);
        try {
            sub_diff = sub_local.subtract(sub_peer);
            auto dec = sub_diff.decode();
            if (dec.is_success()) {
                out_metrics.decodes_succeeded += 2;
                for (const auto& k : dec.left_only) {
                    auto u_opt = get_update_by_key(k);
                    if (u_opt.has_value()) {
                        peer.apply_remote_update(*u_opt);
                        out_metrics.missing_updates_extracted++;
                        out_metrics.wire_bytes_transmitted += sizeof(graph::MapUpdate);
                    }
                }
                for (const auto& k : dec.right_only) {
                    auto u_opt = peer.get_update_by_key(k);
                    if (u_opt.has_value()) {
                        apply_remote_update(*u_opt);
                        out_metrics.missing_updates_extracted++;
                        out_metrics.wire_bytes_transmitted += sizeof(graph::MapUpdate);
                    }
                }
            } else {
                out_metrics.decodes_failed += 2;
                // Direct exchange fallback for congested chunk
                for (const auto& u : m_ledger) {
                    if (u.landmark_key.is_valid() &&
                        (static_cast<uint8_t>(u.landmark_key.descriptor_hash() & CHUNK_MASK) == chunk)) {
                        if (peer.apply_remote_update(u)) {
                            out_metrics.missing_updates_extracted++;
                        }
                        out_metrics.wire_bytes_transmitted += sizeof(graph::MapUpdate);
                    }
                }
                for (const auto& u : peer.m_ledger) {
                    if (u.landmark_key.is_valid() &&
                        (static_cast<uint8_t>(u.landmark_key.descriptor_hash() & CHUNK_MASK) == chunk)) {
                        if (apply_remote_update(u)) {
                            out_metrics.missing_updates_extracted++;
                        }
                        out_metrics.wire_bytes_transmitted += sizeof(graph::MapUpdate);
                    }
                }
            }
        } catch (const std::exception&) {
            out_metrics.decodes_failed += 2;
        }
    }

    // Final causal synchronization
    m_version_vector = m_version_vector.merge_max(peer.version_vector());
    peer.m_version_vector = peer.m_version_vector.merge_max(m_version_vector);

    auto end_time = std::chrono::steady_clock::now();
    out_metrics.elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    out_metrics.memory_bytes_used = (m_ledger.size() + peer.m_ledger.size()) * sizeof(graph::MapUpdate);
    return ReconciliationStatus::Completed;
}

void PartitionAwareReconciler::execute_full_map_baseline(
    PartitionAwareReconciler& r1,
    PartitionAwareReconciler& r2,
    ReconcilerMetrics& out_metrics) {

    auto start_time = std::chrono::steady_clock::now();
    out_metrics.rounds_executed = 1;

    // Transmit entire ledger of r1 to r2
    for (const auto& u : r1.m_ledger) {
        out_metrics.wire_bytes_transmitted += sizeof(graph::MapUpdate);
        if (r2.apply_remote_update(u)) {
            out_metrics.missing_updates_extracted++;
        } else {
            out_metrics.duplicate_updates_suppressed++;
        }
    }

    // Transmit entire ledger of r2 to r1
    for (const auto& u : r2.m_ledger) {
        out_metrics.wire_bytes_transmitted += sizeof(graph::MapUpdate);
        if (r1.apply_remote_update(u)) {
            out_metrics.missing_updates_extracted++;
        } else {
            out_metrics.duplicate_updates_suppressed++;
        }
    }

    r1.m_version_vector = r1.m_version_vector.merge_max(r2.m_version_vector);
    r2.m_version_vector = r2.m_version_vector.merge_max(r1.m_version_vector);

    auto end_time = std::chrono::steady_clock::now();
    out_metrics.elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    out_metrics.memory_bytes_used = (r1.m_ledger.size() + r2.m_ledger.size()) * sizeof(graph::MapUpdate);
}

std::optional<graph::MapUpdate> PartitionAwareReconciler::get_update_by_key(const core::LandmarkKey& key) const {
    auto it = m_landmark_key_to_idx.find(key);
    if (it != m_landmark_key_to_idx.end() && it->second < m_ledger.size()) {
        return m_ledger[it->second];
    }
    return std::nullopt;
}

} // namespace astromesh::net
