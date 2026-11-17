#include "astromesh/coordination/rendezvous_protocol.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>

namespace astromesh::coordination {

void RendezvousInquiry::serialize(std::vector<uint8_t>& buf) const {
    size_t prev_size = buf.size();
    buf.resize(prev_size + sizeof(RendezvousInquiry));
    std::memcpy(buf.data() + prev_size, this, sizeof(RendezvousInquiry));
}

bool RendezvousInquiry::deserialize(const uint8_t* data, size_t len, RendezvousInquiry& out) {
    if (!data || len < sizeof(RendezvousInquiry)) {
        return false;
    }
    std::memcpy(&out, data, sizeof(RendezvousInquiry));
    return true;
}

void RendezvousResponse::serialize(std::vector<uint8_t>& buf) const {
    size_t prev_size = buf.size();
    buf.resize(prev_size + sizeof(RendezvousResponse));
    std::memcpy(buf.data() + prev_size, this, sizeof(RendezvousResponse));
}

bool RendezvousResponse::deserialize(const uint8_t* data, size_t len, RendezvousResponse& out) {
    if (!data || len < sizeof(RendezvousResponse)) {
        return false;
    }
    std::memcpy(&out, data, sizeof(RendezvousResponse));
    return true;
}

void RendezvousProposal::serialize(std::vector<uint8_t>& buf) const {
    size_t prev_size = buf.size();
    buf.resize(prev_size + sizeof(RendezvousProposal));
    std::memcpy(buf.data() + prev_size, this, sizeof(RendezvousProposal));
}

bool RendezvousProposal::deserialize(const uint8_t* data, size_t len, RendezvousProposal& out) {
    if (!data || len < sizeof(RendezvousProposal)) {
        return false;
    }
    std::memcpy(&out, data, sizeof(RendezvousProposal));
    return true;
}

RendezvousProtocolAgent::RendezvousProtocolAgent(uint16_t local_rover_id)
    : m_local_rover_id(local_rover_id) {}

RendezvousInquiry RendezvousProtocolAgent::create_inquiry(
    uint64_t opp_id,
    uint16_t target_rover,
    const graph::NodeId& local_kf,
    const graph::NodeId& target_kf,
    const std::vector<core::LandmarkKey>& candidate_keys,
    uint64_t epoch) {

    RendezvousInquiry inq;
    inq.opportunity_id = opp_id;
    inq.rover_src = m_local_rover_id;
    inq.rover_dst = target_rover;
    inq.keyframe_src = local_kf;
    inq.keyframe_dst = target_kf;
    inq.epoch = epoch;

    size_t to_copy = std::min(candidate_keys.size(), MAX_RENDEZVOUS_LANDMARKS);
    inq.num_requested_landmarks = static_cast<uint32_t>(to_copy);
    for (size_t i = 0; i < to_copy; ++i) {
        inq.requested_landmarks[i] = candidate_keys[i];
    }

    m_metrics.inquiries_sent++;
    m_metrics.total_wire_bytes += sizeof(RendezvousInquiry);

    return inq;
}

RendezvousResponse RendezvousProtocolAgent::handle_inquiry(
    const RendezvousInquiry& inq,
    const graph::PoseGraph& local_graph,
    const std::map<core::LandmarkKey, verification::Point2D>& local_landmark_db) {

    m_metrics.inquiries_received++;

    RendezvousResponse resp;
    resp.opportunity_id = inq.opportunity_id;
    resp.rover_src = m_local_rover_id;
    resp.rover_dst = inq.rover_src;
    resp.keyframe_id = inq.keyframe_dst;

    // Check if target keyframe exists in local graph
    if (!local_graph.has_node(inq.keyframe_dst)) {
        resp.status = 1; // KeyframeNotFound
        m_metrics.responses_sent++;
        m_metrics.total_wire_bytes += sizeof(RendezvousResponse);
        return resp;
    }

    // Match requested landmarks
    size_t matched = 0;
    for (size_t i = 0; i < inq.num_requested_landmarks && matched < MAX_RENDEZVOUS_LANDMARKS; ++i) {
        const auto& key = inq.requested_landmarks[i];
        auto it = local_landmark_db.find(key);
        if (it != local_landmark_db.end()) {
            resp.keys[matched] = key;
            resp.pos_x[matched] = it->second.x;
            resp.pos_y[matched] = it->second.y;
            matched++;
        }
    }

    resp.num_landmarks = static_cast<uint32_t>(matched);
    if (matched < 3) {
        resp.status = 2; // InsufficientLandmarks
    } else {
        resp.status = 0; // Success
    }

    m_metrics.responses_sent++;
    m_metrics.total_wire_bytes += sizeof(RendezvousResponse);

    return resp;
}

std::optional<RendezvousProposal> RendezvousProtocolAgent::process_response(
    const RendezvousResponse& resp,
    const std::map<core::LandmarkKey, verification::Point2D>& local_landmark_db,
    double inlier_threshold_m) {

    m_metrics.responses_received++;

    if (resp.status != 0 || resp.num_landmarks < 3) {
        return std::nullopt;
    }

    // Associate mutual landmarks
    std::vector<verification::PointCorrespondence2D> corrs;
    for (size_t i = 0; i < resp.num_landmarks; ++i) {
        const auto& key = resp.keys[i];
        auto it = local_landmark_db.find(key);
        if (it != local_landmark_db.end()) {
            // Source = Remote responder point, Target = Local initiator point
            verification::Point2D pt_remote(resp.pos_x[i], resp.pos_y[i]);
            verification::Point2D pt_local = it->second;
            corrs.emplace_back(pt_remote, pt_local, key);
        }
    }

    if (corrs.size() < 3) {
        return std::nullopt;
    }

    // Run geometric verification
    verification::RansacConfig2D cfg;
    cfg.max_iterations = 50;
    cfg.inlier_distance_threshold = inlier_threshold_m;
    cfg.min_correspondences = 3;
    cfg.min_inliers = 3;
    cfg.min_inlier_ratio = 0.40;

    auto verif_res = verification::verify_relative_pose_2d(corrs, cfg);
    if (!verif_res.success) {
        return std::nullopt;
    }

    RendezvousProposal prop;
    prop.opportunity_id = resp.opportunity_id;
    prop.from_node = resp.keyframe_id;
    prop.to_node = graph::NodeId(m_local_rover_id, 1); // Associated with local node
    prop.tx = verif_res.estimated_transform.x;
    prop.ty = verif_res.estimated_transform.y;
    prop.theta = verif_res.estimated_transform.theta;
    prop.inlier_count = static_cast<uint32_t>(verif_res.inlier_count);
    prop.inlier_rmse = verif_res.inlier_rmse;
    prop.confidence = static_cast<double>(verif_res.inlier_count) / (1.0 + 10.0 * verif_res.inlier_rmse);

    m_metrics.proposals_generated++;
    m_metrics.total_wire_bytes += sizeof(RendezvousProposal);

    return prop;
}

} // namespace astromesh::coordination
