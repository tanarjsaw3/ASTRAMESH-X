#include "astromesh/memory/distributed_compaction_protocol.hpp"

#include <cstring>

namespace astromesh::memory {

static uint32_t compute_fnv1a32(const uint8_t* data, size_t len) noexcept {
    uint32_t h = 0x811c9dc5U;
    for (size_t i = 0; i < len; ++i) {
        h ^= data[i];
        h *= 0x01000193U;
    }
    return h;
}

uint32_t CompactionNotification::compute_crc32() const noexcept {
    // Hash everything except the last 4 bytes (crc32 field)
    return compute_fnv1a32(reinterpret_cast<const uint8_t*>(this), sizeof(CompactionNotification) - sizeof(uint32_t));
}

bool CompactionNotification::verify_crc32() const noexcept {
    return crc32 == compute_crc32();
}

uint32_t CompactionAck::compute_crc32() const noexcept {
    return compute_fnv1a32(reinterpret_cast<const uint8_t*>(this), sizeof(CompactionAck) - sizeof(uint32_t));
}

bool CompactionAck::verify_crc32() const noexcept {
    return crc32 == compute_crc32();
}

CompactionNotification DistributedCompactor::create_notification(
    uint32_t epoch,
    const graph::NodeId& from,
    const graph::NodeId& to,
    const graph::NodeId& compacted,
    const graph::Pose2D& rel_pose,
    float trans_weight,
    float rot_weight) {

    CompactionNotification notif;
    notif.opcode = COMPACTION_NOTIFICATION_OPCODE;
    notif.rover_id = m_local_rover_id;
    notif.compaction_epoch = epoch;
    notif.from_node = from;
    notif.to_node = to;
    notif.compacted_node = compacted;
    notif.rel_tx = rel_pose.x;
    notif.rel_ty = rel_pose.y;
    notif.rel_theta = rel_pose.theta;
    notif.trans_weight = trans_weight;
    notif.rot_weight = rot_weight;
    notif.crc32 = notif.compute_crc32();

    return notif;
}

CompactionAck DistributedCompactor::handle_notification(
    const CompactionNotification& notif,
    graph::PoseGraph& local_graph,
    const core::VersionVector& local_vv) {

    (void)local_vv;
    CompactionAck ack;
    ack.opcode = COMPACTION_ACK_OPCODE;
    ack.acknowledging_rover = m_local_rover_id;
    ack.origin_rover = notif.rover_id;
    ack.compaction_epoch = notif.compaction_epoch;

    if (!notif.verify_crc32()) {
        ack.status = 2; // Rejected (checksum mismatch)
        ack.crc32 = ack.compute_crc32();
        return ack;
    }

    // Check epoch idempotency
    uint32_t current_peer_epoch = m_peer_epochs[notif.rover_id];
    if (notif.compaction_epoch <= current_peer_epoch) {
        ack.status = 0; // Already applied (idempotent success)
        ack.crc32 = ack.compute_crc32();
        return ack;
    }

    // Verify if local graph has the required endpoints
    bool has_from = local_graph.has_node(notif.from_node);
    bool has_to = local_graph.has_node(notif.to_node);
    bool has_compacted = local_graph.has_node(notif.compacted_node);

    if (!has_from || !has_to) {
        // Missing endpoints (possibly partitioned or not yet synchronized) -> Defer
        m_deferred_notifications.push_back(notif);
        ack.status = 1; // Deferred
        ack.crc32 = ack.compute_crc32();
        return ack;
    }

    // Apply the compaction: add compounded edge and remove intermediate node
    if (has_compacted) {
        local_graph.remove_node(notif.compacted_node);
    }

    // Add or replace compounded edge
    graph::InformationMatrix2D info = graph::InformationMatrix2D::diagonal(
        static_cast<double>(notif.trans_weight),
        static_cast<double>(notif.rot_weight));

    graph::Pose2D rel(notif.rel_tx, notif.rel_ty, notif.rel_theta);
    uint64_t eid = m_next_compounded_edge_id++;

    graph::PoseGraphEdge comp_edge(
        eid, notif.from_node, notif.to_node,
        graph::EdgeType::Odometry, rel, info,
        graph::EdgeVerificationStatus::Verified);

    local_graph.add_edge(comp_edge);

    m_peer_epochs[notif.rover_id] = notif.compaction_epoch;
    ack.status = 0; // Applied
    ack.crc32 = ack.compute_crc32();
    return ack;
}

size_t DistributedCompactor::process_deferred(graph::PoseGraph& local_graph) {
    if (m_deferred_notifications.empty()) return 0;

    size_t applied_count = 0;
    std::vector<CompactionNotification> still_deferred;

    for (const auto& notif : m_deferred_notifications) {
        if (local_graph.has_node(notif.from_node) && local_graph.has_node(notif.to_node)) {
            if (local_graph.has_node(notif.compacted_node)) {
                local_graph.remove_node(notif.compacted_node);
            }

            graph::InformationMatrix2D info = graph::InformationMatrix2D::diagonal(
                static_cast<double>(notif.trans_weight),
                static_cast<double>(notif.rot_weight));
            graph::Pose2D rel(notif.rel_tx, notif.rel_ty, notif.rel_theta);
            uint64_t eid = m_next_compounded_edge_id++;

            graph::PoseGraphEdge comp_edge(
                eid, notif.from_node, notif.to_node,
                graph::EdgeType::Odometry, rel, info,
                graph::EdgeVerificationStatus::Verified);

            local_graph.add_edge(comp_edge);
            m_peer_epochs[notif.rover_id] = notif.compaction_epoch;
            applied_count++;
        } else {
            still_deferred.push_back(notif);
        }
    }

    m_deferred_notifications = std::move(still_deferred);
    return applied_count;
}

} // namespace astromesh::memory
