#include "astromesh/net/distributed_reconciliation.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>

namespace astromesh::net {

// ==============================================================================
// IBLT Serialization
// ==============================================================================

std::vector<uint8_t> serialize_iblt_fingerprint(const core::InvertibleBloomFilter& iblt) {
    const size_t num_cells = iblt.num_cells();
    std::vector<uint8_t> buf(12 + num_cells * 32);

    uint32_t nc = static_cast<uint32_t>(num_cells);
    uint32_t np = static_cast<uint32_t>(iblt.num_probes());
    uint32_t proto = iblt.protocol_id();

    std::memcpy(&buf[0], &nc, 4);
    std::memcpy(&buf[4], &np, 4);
    std::memcpy(&buf[8], &proto, 4);

    size_t offset = 12;
    for (const auto& cell : iblt.cells()) {
        int32_t cnt = cell.count;
        uint32_t pad = 0;
        uint64_t low = cell.key_sum_low;
        uint64_t high = cell.key_sum_high;
        uint64_t chk = cell.checksum_sum;

        std::memcpy(&buf[offset], &cnt, 4);
        std::memcpy(&buf[offset + 4], &pad, 4);
        std::memcpy(&buf[offset + 8], &low, 8);
        std::memcpy(&buf[offset + 16], &high, 8);
        std::memcpy(&buf[offset + 24], &chk, 8);
        offset += 32;
    }

    return buf;
}

std::optional<core::InvertibleBloomFilter> deserialize_iblt_fingerprint(const uint8_t* data, size_t size) {
    if (!data || size < 12) return std::nullopt;

    uint32_t nc = 0, np = 0, proto = 0;
    std::memcpy(&nc, &data[0], 4);
    std::memcpy(&np, &data[4], 4);
    std::memcpy(&proto, &data[8], 4);

    if (nc == 0 || np == 0 || size != (12 + nc * 32)) return std::nullopt;

    auto opt = core::InvertibleBloomFilter::try_create(nc, np, proto);
    if (!opt.has_value()) return std::nullopt;

    core::InvertibleBloomFilter filter = std::move(*opt);
    auto& cells = filter.mutable_cells();

    size_t offset = 12;
    for (size_t i = 0; i < nc; ++i) {
        int32_t cnt = 0;
        uint64_t low = 0, high = 0, chk = 0;
        std::memcpy(&cnt, &data[offset], 4);
        std::memcpy(&low, &data[offset + 8], 8);
        std::memcpy(&high, &data[offset + 16], 8);
        std::memcpy(&chk, &data[offset + 24], 8);

        cells[i].count = cnt;
        cells[i].key_sum_low = low;
        cells[i].key_sum_high = high;
        cells[i].checksum_sum = chk;
        offset += 32;
    }

    return filter;
}

// ==============================================================================
// Candidate Proposal Serialization
// ==============================================================================

std::vector<uint8_t> CandidateProposalPayload::serialize() const {
    const uint32_t count = static_cast<uint32_t>(observations.size());
    std::vector<uint8_t> buf(8 + count * 36);

    std::memcpy(&buf[0], &sender_rover_id, 2);
    std::memcpy(&buf[2], &target_rover_id, 2);
    std::memcpy(&buf[4], &count, 4);

    size_t offset = 8;
    for (const auto& obs : observations) {
        uint16_t schema = obs.key.schema_version();
        uint16_t r_id = obs.key.rover_id();
        uint32_t l_idx = obs.key.local_index();
        uint64_t d_hash = obs.key.descriptor_hash();

        std::memcpy(&buf[offset], &schema, 2);
        std::memcpy(&buf[offset + 2], &r_id, 2);
        std::memcpy(&buf[offset + 4], &l_idx, 4);
        std::memcpy(&buf[offset + 8], &d_hash, 8);

        std::memcpy(&buf[offset + 16], &obs.local_x, 8);
        std::memcpy(&buf[offset + 24], &obs.local_y, 8);
        const uint32_t kf_and_gt = (obs.keyframe_index & 0x7FFFFFFFU) | (obs.is_ground_truth_inlier ? 0x80000000U : 0U);
        std::memcpy(&buf[offset + 32], &kf_and_gt, 4);

        offset += 36;
    }

    return buf;
}

std::optional<CandidateProposalPayload> CandidateProposalPayload::deserialize(const uint8_t* data, size_t size) {
    if (!data || size < 8) return std::nullopt;

    CandidateProposalPayload p;
    std::memcpy(&p.sender_rover_id, &data[0], 2);
    std::memcpy(&p.target_rover_id, &data[2], 2);

    uint32_t count = 0;
    std::memcpy(&count, &data[4], 4);

    if (size != 8 + count * 36) return std::nullopt;

    p.observations.resize(count);
    size_t offset = 8;
    for (uint32_t i = 0; i < count; ++i) {
        uint16_t schema = 0, r_id = 0;
        uint32_t l_idx = 0;
        uint64_t d_hash = 0;

        std::memcpy(&schema, &data[offset], 2);
        std::memcpy(&r_id, &data[offset + 2], 2);
        std::memcpy(&l_idx, &data[offset + 4], 4);
        std::memcpy(&d_hash, &data[offset + 8], 8);

        p.observations[i].key = core::LandmarkKey(schema, r_id, l_idx, d_hash);
        std::memcpy(&p.observations[i].local_x, &data[offset + 16], 8);
        std::memcpy(&p.observations[i].local_y, &data[offset + 24], 8);
        uint32_t kf_and_gt = 0;
        std::memcpy(&kf_and_gt, &data[offset + 32], 4);
        p.observations[i].keyframe_index = (kf_and_gt & 0x7FFFFFFFU);
        p.observations[i].is_ground_truth_inlier = ((kf_and_gt & 0x80000000U) != 0);

        offset += 36;
    }

    return p;
}

// ==============================================================================
// Candidate Response Serialization
// ==============================================================================

std::vector<uint8_t> CandidateResponsePayload::serialize() const {
    std::vector<uint8_t> buf(56);

    uint8_t v_flag = verified ? 1 : 0;
    double tx = relative_transform.x();
    double ty = relative_transform.y();
    double th = relative_transform.theta();

    std::memcpy(&buf[0], &sender_rover_id, 2);
    std::memcpy(&buf[2], &target_rover_id, 2);
    buf[4] = v_flag;
    buf[5] = 0; buf[6] = 0; buf[7] = 0; // padding

    std::memcpy(&buf[8], &inlier_count, 4);
    std::memcpy(&buf[12], &sender_keyframe, 4);
    std::memcpy(&buf[16], &target_keyframe, 4);
    std::memcpy(&buf[20], &inlier_rmse, 8);
    std::memcpy(&buf[28], &tx, 8);
    std::memcpy(&buf[36], &ty, 8);
    std::memcpy(&buf[44], &th, 8);
    std::memset(&buf[52], 0, 4);

    return buf;
}

std::optional<CandidateResponsePayload> CandidateResponsePayload::deserialize(const uint8_t* data, size_t size) {
    if (!data || size < 56) return std::nullopt;

    CandidateResponsePayload r;
    std::memcpy(&r.sender_rover_id, &data[0], 2);
    std::memcpy(&r.target_rover_id, &data[2], 2);
    r.verified = (data[4] != 0);

    double tx = 0, ty = 0, th = 0;
    std::memcpy(&r.inlier_count, &data[8], 4);
    std::memcpy(&r.sender_keyframe, &data[12], 4);
    std::memcpy(&r.target_keyframe, &data[16], 4);
    std::memcpy(&r.inlier_rmse, &data[20], 8);
    std::memcpy(&tx, &data[28], 8);
    std::memcpy(&ty, &data[36], 8);
    std::memcpy(&th, &data[44], 8);

    r.relative_transform = geometry::SE2(tx, ty, th);
    return r;
}

// ==============================================================================
// DistributedReconciliationAgent Implementation
// ==============================================================================

DistributedReconciliationAgent::DistributedReconciliationAgent(
    uint16_t rover_id,
    RoverNode& net_node,
    size_t iblt_cells,
    size_t iblt_probes)
    : m_rover_id(rover_id),
      m_net_node(net_node),
      m_graph(),
      m_partition_mgr(m_graph),
      m_iblt(iblt_cells, iblt_probes, 1) {

    m_ransac_config.min_correspondences = 3;
    m_ransac_config.min_inliers = 3;
    m_ransac_config.inlier_distance_threshold = 0.50; // 0.5m tolerance
    m_ransac_config.max_iterations = 100;
}

void DistributedReconciliationAgent::setup_callbacks() {
    m_net_node.set_message_handler([this](const MessageFrame& frame, const Endpoint& sender_ep) {
        handle_message(frame, sender_ep, frame.timestamp_ns);
    });
}

void DistributedReconciliationAgent::add_local_keyframe(
    uint32_t keyframe_index,
    const geometry::SE2& pose,
    const std::vector<LandmarkObservationRecord>& observations,
    uint64_t timestamp_ns) {

    const graph::NodeId node_id(m_rover_id, keyframe_index);
    graph::KeyframeNode kf_node(node_id, graph::Pose2D(pose.x(), pose.y(), pose.theta()), timestamp_ns);
    m_graph.add_node(kf_node);

    // Add odometry constraint from previous keyframe
    if (keyframe_index > 0) {
        const graph::NodeId prev_node(m_rover_id, keyframe_index - 1);
        if (m_graph.has_node(prev_node)) {
            const auto& prev_kf = m_graph.get_node(prev_node);
            const geometry::SE2 prev_se2(prev_kf.pose.x, prev_kf.pose.y, prev_kf.pose.theta);
            const auto delta = prev_se2.inverse() * pose;
            const graph::Pose2D delta_pose(delta.x(), delta.y(), delta.theta());

            graph::PoseGraphEdge edge(0, prev_node, node_id, graph::EdgeType::Odometry,
                                      delta_pose, graph::InformationMatrix2D::diagonal(100.0, 1000.0),
                                      graph::EdgeVerificationStatus::Verified);
            m_graph.add_edge_detailed(edge);
            m_partition_mgr.add_eligible_edge(edge);
        }
    } else {
        m_partition_mgr.rebuild(m_graph);
    }

    // Ingest landmark observations into local database and IBLT
    for (const auto& obs : observations) {
        m_iblt.insert(obs.key);
        m_local_observations[obs.key.local_index()] = obs;
        m_landmark_history[obs.key.local_index()].push_back(obs);
    }
}

std::vector<LandmarkObservationRecord> DistributedReconciliationAgent::get_landmark_observation_history(uint32_t landmark_id) const {
    auto it = m_landmark_history.find(landmark_id);
    if (it != m_landmark_history.end()) {
        return it->second;
    }
    return {};
}

void DistributedReconciliationAgent::append_trajectory_keyframe(
    uint32_t keyframe_index,
    const geometry::SE2& pose,
    const std::vector<LandmarkObservationRecord>& observations,
    uint64_t timestamp_ns) {
    add_local_keyframe(keyframe_index, pose, observations, timestamp_ns);
}

optimization::CordIntegratorResult DistributedReconciliationAgent::optimize_local_graph_cord(
    const optimization::CordIntegratorOptions& user_options) {

    optimization::CordIntegratorOptions opts = user_options;

    std::unordered_map<graph::NodeId, geometry::SE2> initial_poses;
    for (const auto& [node_id, node] : m_graph.nodes()) {
        initial_poses[node_id] = geometry::SE2(node.pose.x, node.pose.y, node.pose.theta);
    }

    if (initial_poses.empty()) {
        optimization::CordIntegratorResult res;
        res.status = optimization::CordStatus::InvalidInput;
        res.message = "Graph has no nodes to optimize";
        return res;
    }

    if (!opts.fixed_anchor.has_value()) {
        const graph::NodeId root_anchor(1, 0);
        if (m_graph.has_node(root_anchor)) {
            opts.fixed_anchor = root_anchor;
        } else {
            const graph::NodeId local_anchor(m_rover_id, 0);
            if (m_graph.has_node(local_anchor)) {
                opts.fixed_anchor = local_anchor;
            }
        }
    }

    auto res = optimization::integrate_cord(m_graph, initial_poses, opts);

    if (res.is_converged() || res.status == optimization::CordStatus::MaxStepsReached) {
        for (const auto& [node_id, state] : res.final_states) {
            if (m_graph.has_node(node_id)) {
                m_graph.update_node_pose(node_id, graph::Pose2D(state.pose.x(), state.pose.y(), state.pose.theta()));
            }
        }
        m_stats.cord_optimizations_run++;
    }

    return res;
}

graph::PruningResult DistributedReconciliationAgent::prune_redundant_keyframes(
    const graph::KeyframeSelectionCriteria& criteria,
    const graph::PrunerConfig& config) {

    graph::PrunerConfig cfg = config;
    cfg.selection = criteria;
    graph::KeyframePruner pruner(cfg);

    auto res = pruner.enforce_memory_bound(m_graph, &m_partition_mgr);
    if (res.status == graph::PruningStatus::SuccessBoundMet) {
        m_stats.keyframes_pruned += res.pruned_node_ids.size();
    }
    return res;
}

double DistributedReconciliationAgent::get_loop_closure_precision() const noexcept {
    uint64_t predicted_positives = m_stats.true_positives + m_stats.false_positives;
    if (predicted_positives == 0) return 1.0;
    return static_cast<double>(m_stats.true_positives) / static_cast<double>(predicted_positives);
}

double DistributedReconciliationAgent::get_loop_closure_recall() const noexcept {
    uint64_t actual_positives = m_stats.true_positives + m_stats.false_negatives;
    if (actual_positives == 0) return 1.0;
    return static_cast<double>(m_stats.true_positives) / static_cast<double>(actual_positives);
}

double DistributedReconciliationAgent::get_loop_closure_f1() const noexcept {
    double p = get_loop_closure_precision();
    double r = get_loop_closure_recall();
    if (p + r < 1e-9) return 0.0;
    return 2.0 * (p * r) / (p + r);
}

uint64_t DistributedReconciliationAgent::initiate_sync_with_peer(uint16_t target_rover_id, uint64_t now_ns) {
    auto iblt_bytes = serialize_iblt_fingerprint(m_iblt);
    uint64_t msg_id = m_net_node.enqueue_dtn_message(
        target_rover_id,
        MessageType::LandmarkSync,
        iblt_bytes,
        comms::PriorityLevel::High,
        60'000'000'000ULL,
        now_ns);

    if (msg_id > 0) {
        m_stats.sync_sent++;
    }
    return msg_id;
}

uint64_t DistributedReconciliationAgent::broadcast_boundary_state(
    uint16_t target_rover_id, uint32_t keyframe_index, uint64_t now_ns) {

    const graph::NodeId node_id(m_rover_id, keyframe_index);
    if (!m_graph.has_node(node_id)) return 0;

    const auto& kf_node = m_graph.get_node(node_id);

    optimization::BoundaryStatePacket pkt;
    pkt.protocol_version = optimization::BOUNDARY_PROTOCOL_VERSION;
    pkt.sender_rover_id = m_rover_id;
    pkt.target_rover_id = target_rover_id;
    pkt.node_local_index = keyframe_index;
    pkt.sequence_number = m_boundary_sequence++;
    pkt.timestamp_ns = now_ns;
    pkt.x = kf_node.pose.x;
    pkt.y = kf_node.pose.y;
    pkt.theta = kf_node.pose.theta;
    pkt.vx = 0.0; pkt.vy = 0.0; pkt.omega = 0.0;

    auto bytes = optimization::serialize_boundary_packet(pkt);

    uint64_t msg_id = m_net_node.enqueue_dtn_message(
        target_rover_id,
        MessageType::BoundaryState,
        bytes,
        comms::PriorityLevel::Normal,
        60'000'000'000ULL,
        now_ns);

    if (msg_id > 0) {
        m_stats.boundary_states_sent++;
    }
    return msg_id;
}

void DistributedReconciliationAgent::handle_message(
    const MessageFrame& frame, [[maybe_unused]] const Endpoint& sender_ep, uint64_t now_ns) {

    switch (frame.type) {
        case MessageType::LandmarkSync:
            process_landmark_sync(frame, now_ns);
            break;
        case MessageType::CandidateProposal:
            process_candidate_proposal(frame, now_ns);
            break;
        case MessageType::CandidateResponse:
            process_candidate_response(frame, now_ns);
            break;
        case MessageType::BoundaryState:
            process_boundary_state(frame, now_ns);
            break;
        default:
            break;
    }
}

void DistributedReconciliationAgent::process_landmark_sync(const MessageFrame& frame, uint64_t now_ns) {
    m_stats.sync_received++;

    auto peer_iblt_opt = deserialize_iblt_fingerprint(frame.payload.data(), frame.payload.size());
    if (!peer_iblt_opt.has_value() || !m_iblt.is_compatible_with(*peer_iblt_opt)) {
        m_stats.sync_decodes_failed++;
        return;
    }

    core::InvertibleBloomFilter diff(m_iblt.num_cells(), m_iblt.num_probes(), m_iblt.protocol_id());
    try {
        diff = m_iblt.subtract(*peer_iblt_opt);
    } catch (const std::exception&) {
        m_stats.sync_decodes_failed++;
        return;
    }

    auto decode_res = diff.decode();
    if (!decode_res.is_success()) {
        m_stats.sync_decodes_failed++;
        return;
    }

    m_stats.sync_decodes_succeeded++;

    // Extract shared candidate landmarks: local keys NOT in left_only (i.e. present in peer's set too)
    std::set<core::LandmarkKey> left_only_set(decode_res.left_only.begin(), decode_res.left_only.end());
    std::vector<LandmarkObservationRecord> shared_candidates;

    for (const auto& [lm_id, obs] : m_local_observations) {
        if (left_only_set.find(obs.key) == left_only_set.end()) {
            shared_candidates.push_back(obs);
        }
    }

    if (shared_candidates.size() >= m_ransac_config.min_correspondences) {
        CandidateProposalPayload proposal;
        proposal.sender_rover_id = m_rover_id;
        proposal.target_rover_id = frame.sender_rover_id;
        proposal.observations = std::move(shared_candidates);

        auto p_bytes = proposal.serialize();
        m_net_node.enqueue_dtn_message(
            frame.sender_rover_id,
            MessageType::CandidateProposal,
            p_bytes,
            comms::PriorityLevel::High,
            60'000'000'000ULL,
            now_ns);

        m_stats.proposals_sent++;
    }
}

void DistributedReconciliationAgent::process_candidate_proposal(const MessageFrame& frame, uint64_t now_ns) {
    m_stats.proposals_received++;

    auto proposal_opt = CandidateProposalPayload::deserialize(frame.payload.data(), frame.payload.size());
    if (!proposal_opt.has_value()) return;

    const auto& proposal = *proposal_opt;
    std::vector<verification::PointCorrespondence2D> correspondences;

    uint32_t remote_kf = 0;
    uint32_t local_kf = 0;
    size_t gt_inliers_count = 0;

    for (const auto& remote_obs : proposal.observations) {
        const uint32_t lm_id = remote_obs.key.local_index();
        auto it = m_local_observations.find(lm_id);
        if (it != m_local_observations.end()) {
            if (it->second.is_ground_truth_inlier && remote_obs.is_ground_truth_inlier) {
                gt_inliers_count++;
            }

            // Target is local rover, Source is remote rover
            correspondences.emplace_back(
                verification::Point2D(remote_obs.local_x, remote_obs.local_y),
                verification::Point2D(it->second.local_x, it->second.local_y),
                remote_obs.key);

            remote_kf = remote_obs.keyframe_index;
            local_kf = it->second.keyframe_index;
        }
    }

    const bool is_ground_truth_valid = (gt_inliers_count >= m_ransac_config.min_correspondences);
    bool verified_success = false;

    CandidateResponsePayload resp;
    resp.sender_rover_id = m_rover_id;
    resp.target_rover_id = proposal.sender_rover_id;
    resp.sender_keyframe = local_kf;
    resp.target_keyframe = remote_kf;

    if (correspondences.size() >= m_ransac_config.min_correspondences) {
        auto vres = verification::verify_relative_pose_2d(correspondences, m_ransac_config);
        if (vres.success) {
            verified_success = true;
            m_stats.loop_closures_verified++;

            resp.verified = true;
            resp.inlier_count = static_cast<uint32_t>(vres.inlier_count);
            resp.inlier_rmse = vres.inlier_rmse;
            resp.relative_transform = geometry::SE2(vres.estimated_transform.x,
                                                    vres.estimated_transform.y,
                                                    vres.estimated_transform.theta);

            const graph::NodeId local_node(m_rover_id, local_kf);
            const graph::NodeId remote_node(proposal.sender_rover_id, remote_kf);

            if (!m_graph.has_node(remote_node)) {
                m_graph.add_node(graph::KeyframeNode(remote_node, graph::Pose2D(0, 0, 0), now_ns));
            }
            if (!m_graph.has_node(local_node)) {
                m_graph.add_node(graph::KeyframeNode(local_node, graph::Pose2D(0, 0, 0), now_ns));
            }

            // Add verified loop closure edge to local pose graph (inverting so edge points from remote to local)
            const geometry::SE2 inv_transform = resp.relative_transform.inverse();
            const graph::Pose2D inv_pose(inv_transform.x(), inv_transform.y(), inv_transform.theta());
            graph::PoseGraphEdge edge(0, remote_node, local_node,
                                      graph::EdgeType::InterRoverLoopClosure,
                                      inv_pose,
                                      graph::InformationMatrix2D::diagonal(100.0, 1000.0),
                                      graph::EdgeVerificationStatus::Verified);
            m_graph.add_edge_detailed(edge);

            // Gated promotion unites local DSU
            m_partition_mgr.add_eligible_edge(edge);
            m_stats.dsu_unions_performed++;

            // Broadcast boundary state
            broadcast_boundary_state(proposal.sender_rover_id, local_kf, now_ns);
        } else {
            m_stats.loop_closures_rejected++;
            resp.verified = false;
        }
    } else {
        m_stats.loop_closures_rejected++;
        resp.verified = false;
    }

    // Statistical classification for scientific loop closure precision/recall
    if (is_ground_truth_valid) {
        if (verified_success) {
            m_stats.true_positives++;
        } else {
            m_stats.false_negatives++;
        }
    } else {
        if (verified_success) {
            m_stats.false_positives++;
        } else {
            m_stats.true_negatives++;
        }
    }

    auto r_bytes = resp.serialize();
    m_net_node.enqueue_dtn_message(
        proposal.sender_rover_id,
        MessageType::CandidateResponse,
        r_bytes,
        comms::PriorityLevel::High,
        60'000'000'000ULL,
        now_ns);
}

void DistributedReconciliationAgent::process_candidate_response(const MessageFrame& frame, uint64_t now_ns) {
    auto resp_opt = CandidateResponsePayload::deserialize(frame.payload.data(), frame.payload.size());
    if (!resp_opt.has_value() || !resp_opt->verified) return;

    const auto& resp = *resp_opt;
    const graph::NodeId local_node(m_rover_id, resp.target_keyframe);
    const graph::NodeId remote_node(resp.sender_rover_id, resp.sender_keyframe);

    if (!m_graph.has_node(remote_node)) {
        m_graph.add_node(graph::KeyframeNode(remote_node, graph::Pose2D(0, 0, 0), now_ns));
    }
    if (!m_graph.has_node(local_node)) {
        m_graph.add_node(graph::KeyframeNode(local_node, graph::Pose2D(0, 0, 0), now_ns));
    }

    // Invert relative transform so edge points from remote to local
    const geometry::SE2 inv_transform = resp.relative_transform.inverse();
    const graph::Pose2D inv_pose(inv_transform.x(), inv_transform.y(), inv_transform.theta());

    graph::PoseGraphEdge edge(0, remote_node, local_node,
                              graph::EdgeType::InterRoverLoopClosure,
                              inv_pose,
                              graph::InformationMatrix2D::diagonal(100.0, 1000.0),
                              graph::EdgeVerificationStatus::Verified);
    m_graph.add_edge_detailed(edge);

    m_partition_mgr.add_eligible_edge(edge);
    m_stats.dsu_unions_performed++;
    m_stats.loop_closures_verified++;

    // Reply with our boundary state
    broadcast_boundary_state(resp.sender_rover_id, resp.target_keyframe, now_ns);
}

void DistributedReconciliationAgent::process_boundary_state(const MessageFrame& frame, [[maybe_unused]] uint64_t now_ns) {
    m_stats.boundary_states_received++;

    auto val_res = optimization::deserialize_boundary_packet(frame.payload.data(), frame.payload.size());
    if (!val_res.valid || !val_res.packet.has_value()) return;

    const auto& pkt = *(val_res.packet);
    const graph::NodeId neighbor_node = pkt.node_id();
    const geometry::SE2 neighbor_pose = pkt.pose();
    m_boundary_neighbor_poses[neighbor_node] = neighbor_pose;

    // Update node pose in local graph if it exists
    if (m_graph.has_node(neighbor_node)) {
        m_graph.update_node_pose(neighbor_node, graph::Pose2D(neighbor_pose.x(), neighbor_pose.y(), neighbor_pose.theta()));
    }

    // Auto-trigger distributed CORD relaxation upon receiving boundary state
    if (m_auto_optimize_on_boundary && m_boundary_cord_rounds_executed < m_max_boundary_cord_rounds) {
        m_boundary_cord_rounds_executed++;
        optimization::CordIntegratorOptions opts;
        opts.timestep = 0.01;
        opts.max_steps = 100;
        opts.gradient_tolerance = 1e-4;
        optimize_local_graph_cord(opts);
    }
}

// ==============================================================================
// Global Swarm Evaluator Implementation
// ==============================================================================

SwarmConnectivityMetrics GlobalSwarmEvaluator::evaluate_graphs(
    const std::vector<const graph::PoseGraph*>& graphs) {

    SwarmConnectivityMetrics m;
    graph::DisjointSet global_dsu;
    std::set<std::pair<graph::NodeId, graph::NodeId>> added_edges;

    for (const auto* g : graphs) {
        if (!g) continue;

        for (const auto& [nid, node] : g->nodes()) {
            global_dsu.make_set(nid);
        }

        for (const auto& [eid, edge] : g->edges()) {
            if (edge.status == graph::EdgeVerificationStatus::Verified) {
                auto p = std::make_pair(std::min(edge.from_node, edge.to_node),
                                        std::max(edge.from_node, edge.to_node));
                if (added_edges.insert(p).second) {
                    global_dsu.unite(edge.from_node, edge.to_node);
                    m.total_swarm_edges++;
                    if (edge.type == graph::EdgeType::Odometry) {
                        m.total_odometry_edges++;
                    } else if (edge.type == graph::EdgeType::InterRoverLoopClosure) {
                        m.verified_inter_rover_closures++;
                    }
                }
            }
        }
    }

    m.total_swarm_nodes = global_dsu.num_nodes();
    m.global_connected_components = global_dsu.num_components();
    m.is_fully_unified = (m.global_connected_components == 1 && m.total_swarm_nodes > 0);
    return m;
}

SwarmConnectivityMetrics GlobalSwarmEvaluator::evaluate(
    const std::vector<std::unique_ptr<DistributedReconciliationAgent>>& agents) {

    std::vector<const graph::PoseGraph*> graphs;
    graphs.reserve(agents.size());
    for (const auto& a : agents) {
        if (a) graphs.push_back(&a->get_pose_graph());
    }
    return evaluate_graphs(graphs);
}

} // namespace astromesh::net
