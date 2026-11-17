#pragma once

#include "astromesh/core/landmark_key.hpp"
#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <cstdint>
#include <map>
#include <optional>
#include <vector>

namespace astromesh::coordination {

constexpr size_t MAX_RENDEZVOUS_LANDMARKS = 16;

#pragma pack(push, 1)
struct RendezvousInquiry {
    uint64_t opportunity_id{0};
    uint16_t rover_src{0};
    uint16_t rover_dst{0};
    graph::NodeId keyframe_src;
    graph::NodeId keyframe_dst;
    uint64_t epoch{0};
    uint32_t num_requested_landmarks{0};
    core::LandmarkKey requested_landmarks[MAX_RENDEZVOUS_LANDMARKS];

    void serialize(std::vector<uint8_t>& buf) const;
    static bool deserialize(const uint8_t* data, size_t len, RendezvousInquiry& out);
};

struct RendezvousResponse {
    uint64_t opportunity_id{0};
    uint16_t rover_src{0};
    uint16_t rover_dst{0};
    graph::NodeId keyframe_id;
    uint8_t status{0}; // 0 = Success, 1 = KeyframeNotFound, 2 = InsufficientLandmarks, 3 = Busy
    uint32_t num_landmarks{0};
    core::LandmarkKey keys[MAX_RENDEZVOUS_LANDMARKS];
    double pos_x[MAX_RENDEZVOUS_LANDMARKS];
    double pos_y[MAX_RENDEZVOUS_LANDMARKS];

    void serialize(std::vector<uint8_t>& buf) const;
    static bool deserialize(const uint8_t* data, size_t len, RendezvousResponse& out);
};

struct RendezvousProposal {
    uint64_t opportunity_id{0};
    graph::NodeId from_node;
    graph::NodeId to_node;
    double tx{0.0};
    double ty{0.0};
    double theta{0.0};
    uint32_t inlier_count{0};
    double inlier_rmse{0.0};
    double confidence{0.0};

    void serialize(std::vector<uint8_t>& buf) const;
    static bool deserialize(const uint8_t* data, size_t len, RendezvousProposal& out);
};
#pragma pack(pop)

struct RendezvousMetrics {
    uint32_t inquiries_sent{0};
    uint32_t inquiries_received{0};
    uint32_t responses_sent{0};
    uint32_t responses_received{0};
    uint32_t proposals_generated{0};
    uint32_t duplicates_suppressed{0};
    uint64_t total_wire_bytes{0};
};

/**
 * @brief Communication-Efficient Decentralized Rendezvous Protocol Agent.
 * Handles inquiry requests, bounded observation replies, and local geometric verification.
 */
class RendezvousProtocolAgent {
public:
    explicit RendezvousProtocolAgent(uint16_t local_rover_id);

    uint16_t local_rover_id() const noexcept { return m_local_rover_id; }
    const RendezvousMetrics& metrics() const noexcept { return m_metrics; }

    /**
     * @brief Generates an inquiry message for a scheduled opportunity.
     */
    RendezvousInquiry create_inquiry(
        uint64_t opp_id,
        uint16_t target_rover,
        const graph::NodeId& local_kf,
        const graph::NodeId& target_kf,
        const std::vector<core::LandmarkKey>& candidate_keys,
        uint64_t epoch = 1);

    /**
     * @brief Handles an incoming inquiry on the responder rover.
     */
    RendezvousResponse handle_inquiry(
        const RendezvousInquiry& inq,
        const graph::PoseGraph& local_graph,
        const std::map<core::LandmarkKey, verification::Point2D>& local_landmark_db);

    /**
     * @brief Processes response on the initiator rover, runs local geometric verification.
     */
    std::optional<RendezvousProposal> process_response(
        const RendezvousResponse& resp,
        const std::map<core::LandmarkKey, verification::Point2D>& local_landmark_db,
        double inlier_threshold_m = 0.25);

private:
    uint16_t m_local_rover_id{0};
    RendezvousMetrics m_metrics;
    std::map<uint64_t, uint64_t> m_recent_inquiries; // opp_id -> last_seen_ms
};

} // namespace astromesh::coordination

