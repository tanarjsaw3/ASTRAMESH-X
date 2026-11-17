#ifndef ASTROMESH_DISTRIBUTED_COMPACTION_PROTOCOL_HPP
#define ASTROMESH_DISTRIBUTED_COMPACTION_PROTOCOL_HPP

#include "astromesh/core/version_vector.hpp"
#include "astromesh/graph/pose_graph.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

namespace astromesh::memory {

constexpr uint8_t COMPACTION_NOTIFICATION_OPCODE = 0xE1;
constexpr uint8_t COMPACTION_ACK_OPCODE = 0xE2;

#pragma pack(push, 1)
/**
 * @brief Wire payload announcing a keyframe compaction event. Bounded to 64 bytes.
 */
struct CompactionNotification {
    uint8_t opcode{COMPACTION_NOTIFICATION_OPCODE};
    uint8_t reserved{0};
    uint16_t rover_id{0};
    uint32_t compaction_epoch{0};
    graph::NodeId from_node;
    graph::NodeId to_node;
    graph::NodeId compacted_node;
    double rel_tx{0.0};
    double rel_ty{0.0};
    double rel_theta{0.0};
    float trans_weight{100.0f};
    float rot_weight{1000.0f};
    uint32_t crc32{0};

    uint32_t compute_crc32() const noexcept;
    bool verify_crc32() const noexcept;
};

/**
 * @brief Acknowledgment for compaction notification. Bounded to 16 bytes.
 */
struct CompactionAck {
    uint8_t opcode{COMPACTION_ACK_OPCODE};
    uint8_t status{0}; // 0 = Applied, 1 = Deferred, 2 = Rejected
    uint16_t acknowledging_rover{0};
    uint16_t origin_rover{0};
    uint32_t compaction_epoch{0};
    uint32_t crc32{0};

    uint32_t compute_crc32() const noexcept;
    bool verify_crc32() const noexcept;
};
#pragma pack(pop)

static_assert(sizeof(CompactionNotification) <= 128, "CompactionNotification exceeds 128B MTU");
static_assert(sizeof(CompactionAck) <= 128, "CompactionAck exceeds 128B MTU");

/**
 * @brief Tracks and applies distributed compaction events across rover graph replicas.
 */
class DistributedCompactor {
public:
    explicit DistributedCompactor(uint16_t local_rover_id) noexcept
        : m_local_rover_id(local_rover_id) {}

    CompactionNotification create_notification(
        uint32_t epoch,
        const graph::NodeId& from,
        const graph::NodeId& to,
        const graph::NodeId& compacted,
        const graph::Pose2D& rel_pose,
        float trans_weight,
        float rot_weight);

    CompactionAck handle_notification(
        const CompactionNotification& notif,
        graph::PoseGraph& local_graph,
        const core::VersionVector& local_vv);

    uint32_t peer_epoch(uint16_t peer_rover_id) const noexcept {
        auto it = m_peer_epochs.find(peer_rover_id);
        return (it != m_peer_epochs.end()) ? it->second : 0;
    }

    size_t deferred_count() const noexcept { return m_deferred_notifications.size(); }

    /**
     * @brief Drains and applies deferred compaction notifications after partition reconciliation.
     */
    size_t process_deferred(graph::PoseGraph& local_graph);

private:
    uint16_t m_local_rover_id{0};
    std::map<uint16_t, uint32_t> m_peer_epochs;
    std::vector<CompactionNotification> m_deferred_notifications;
    uint64_t m_next_compounded_edge_id{9500000ULL};
};

} // namespace astromesh::memory

#endif // ASTROMESH_DISTRIBUTED_COMPACTION_PROTOCOL_HPP
