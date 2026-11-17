#include "astromesh/coordination/loop_closure_opportunity.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace astromesh::coordination {

LoopClosureOpportunityManager::LoopClosureOpportunityManager(uint16_t local_rover_id)
    : m_local_rover_id(local_rover_id) {}

bool LoopClosureOpportunityManager::add_opportunity(LoopClosureOpportunity opp) {
    if (opp.rover_a == 0 || opp.rover_b == 0 || opp.rover_a == opp.rover_b) {
        return false;
    }

    // Deduplicate against existing active opportunities
    for (const auto& exist : m_opportunities) {
        if ((exist.keyframe_a == opp.keyframe_a && exist.keyframe_b == opp.keyframe_b) ||
            (exist.keyframe_a == opp.keyframe_b && exist.keyframe_b == opp.keyframe_a)) {
            return false; // Duplicate
        }
    }

    if (opp.opportunity_id == 0) {
        opp.opportunity_id = m_next_opp_id++;
    } else if (opp.opportunity_id >= m_next_opp_id) {
        m_next_opp_id = opp.opportunity_id + 1;
    }

    m_opportunities.push_back(opp);
    return true;
}

size_t LoopClosureOpportunityManager::detect_spatial_opportunities(
    const graph::PoseGraph& local_graph,
    uint16_t remote_rover_id,
    const std::vector<graph::KeyframeNode>& remote_keyframes,
    double max_radius_m,
    uint64_t now_ms,
    uint64_t ttl_ms) {

    if (remote_rover_id == 0 || remote_rover_id == m_local_rover_id) {
        return 0;
    }

    size_t created_count = 0;
    auto local_node_ids = local_graph.nodes_for_rover(m_local_rover_id);

    for (const auto& loc_id : local_node_ids) {
        const auto* loc_node = local_graph.find_node(loc_id);
        if (!loc_node) continue;

        for (const auto& rem_node : remote_keyframes) {
            double dist = std::hypot(loc_node->pose.x - rem_node.pose.x,
                                     loc_node->pose.y - rem_node.pose.y);

            if (dist <= max_radius_m) {
                LoopClosureOpportunity opp;
                opp.rover_a = m_local_rover_id;
                opp.rover_b = remote_rover_id;
                opp.keyframe_a = loc_id;
                opp.keyframe_b = rem_node.id;
                opp.type = OpportunityType::SpatialProximity;
                opp.estimated_spatial_distance_m = dist;

                if (loc_node->timestamp_ns > 0 && rem_node.timestamp_ns > 0) {
                    uint64_t t_diff = (loc_node->timestamp_ns > rem_node.timestamp_ns)
                                          ? (loc_node->timestamp_ns - rem_node.timestamp_ns)
                                          : (rem_node.timestamp_ns - loc_node->timestamp_ns);
                    opp.temporal_separation_ms = t_diff / 1000000ULL;
                }

                // Count mutual landmarks if available
                std::unordered_set<core::LandmarkKey> loc_lms(
                    loc_node->observed_landmarks.begin(), loc_node->observed_landmarks.end());
                size_t shared_lms = 0;
                for (const auto& rlm : rem_node.observed_landmarks) {
                    if (loc_lms.find(rlm) != loc_lms.end()) {
                        shared_lms++;
                    }
                }

                opp.estimated_landmark_overlap = shared_lms;
                opp.expected_correspondences = (shared_lms > 0) ? shared_lms : static_cast<size_t>(std::max(3.0, 10.0 - dist));
                opp.confidence = std::clamp(1.0 - (dist / max_radius_m), 0.1, 0.99);
                opp.expected_wire_cost_bytes = 48 + opp.expected_correspondences * 16;
                opp.created_timestamp_ms = now_ms;
                opp.expiration_timestamp_ms = (ttl_ms > 0) ? (now_ms + ttl_ms) : 0;

                if (add_opportunity(opp)) {
                    created_count++;
                }
            }
        }
    }

    return created_count;
}

size_t LoopClosureOpportunityManager::detect_subgraph_overlap_opportunities(
    uint16_t remote_rover_id,
    const graph::NodeId& remote_kf,
    const std::vector<core::LandmarkKey>& remote_landmarks,
    const graph::PoseGraph& local_graph,
    uint64_t now_ms,
    uint64_t ttl_ms) {

    if (remote_rover_id == 0 || remote_rover_id == m_local_rover_id || remote_landmarks.empty()) {
        return 0;
    }

    std::unordered_set<core::LandmarkKey> rem_set(remote_landmarks.begin(), remote_landmarks.end());
    size_t created_count = 0;

    auto local_node_ids = local_graph.nodes_for_rover(m_local_rover_id);
    for (const auto& loc_id : local_node_ids) {
        const auto* loc_node = local_graph.find_node(loc_id);
        if (!loc_node) continue;

        size_t shared = 0;
        for (const auto& lk : loc_node->observed_landmarks) {
            if (rem_set.find(lk) != rem_set.end()) {
                shared++;
            }
        }

        if (shared >= 3) { // Minimum 3 correspondences required for SE(2) Kabsch
            LoopClosureOpportunity opp;
            opp.rover_a = m_local_rover_id;
            opp.rover_b = remote_rover_id;
            opp.keyframe_a = loc_id;
            opp.keyframe_b = remote_kf;
            opp.type = OpportunityType::SubgraphOverlap;
            opp.estimated_landmark_overlap = shared;
            opp.expected_correspondences = shared;
            opp.confidence = std::clamp(static_cast<double>(shared) / 10.0, 0.3, 1.0);
            opp.expected_wire_cost_bytes = 48 + shared * 16;
            opp.created_timestamp_ms = now_ms;
            opp.expiration_timestamp_ms = (ttl_ms > 0) ? (now_ms + ttl_ms) : 0;

            if (add_opportunity(opp)) {
                created_count++;
            }
        }
    }

    return created_count;
}

size_t LoopClosureOpportunityManager::filter_redundant_opportunities(double spatial_suppression_radius_m) {
    if (m_opportunities.size() < 2) {
        return 0;
    }

    std::vector<bool> keep(m_opportunities.size(), true);
    size_t pruned = 0;

    for (size_t i = 0; i < m_opportunities.size(); ++i) {
        if (!keep[i]) continue;
        for (size_t j = i + 1; j < m_opportunities.size(); ++j) {
            if (!keep[j]) continue;

            const auto& o1 = m_opportunities[i];
            const auto& o2 = m_opportunities[j];

            if (o1.rover_a == o2.rover_a && o1.rover_b == o2.rover_b) {
                // If consecutive or identical keyframes on both sides
                int32_t diff_a = std::abs(static_cast<int32_t>(o1.keyframe_a.local_index) -
                                          static_cast<int32_t>(o2.keyframe_a.local_index));
                int32_t diff_b = std::abs(static_cast<int32_t>(o1.keyframe_b.local_index) -
                                          static_cast<int32_t>(o2.keyframe_b.local_index));

                if (diff_a <= 1 && diff_b <= 1) {
                    // Suppress the one with lower expected correspondences
                    if (o1.expected_correspondences >= o2.expected_correspondences) {
                        keep[j] = false;
                        pruned++;
                    } else {
                        keep[i] = false;
                        pruned++;
                        break;
                    }
                }
            }
        }
    }

    std::vector<LoopClosureOpportunity> filtered;
    filtered.reserve(m_opportunities.size() - pruned);
    for (size_t i = 0; i < m_opportunities.size(); ++i) {
        if (keep[i]) {
            filtered.push_back(m_opportunities[i]);
        }
    }
    m_opportunities = std::move(filtered);

    return pruned;
}

size_t LoopClosureOpportunityManager::prune_expired(uint64_t now_ms) {
    size_t before = m_opportunities.size();
    m_opportunities.erase(
        std::remove_if(m_opportunities.begin(), m_opportunities.end(),
                       [now_ms](const LoopClosureOpportunity& opp) {
                           return opp.is_expired(now_ms);
                       }),
        m_opportunities.end());
    return before - m_opportunities.size();
}

std::optional<LoopClosureOpportunity> LoopClosureOpportunityManager::get_opportunity(uint64_t opp_id) const {
    for (const auto& opp : m_opportunities) {
        if (opp.opportunity_id == opp_id) {
            return opp;
        }
    }
    return std::nullopt;
}

} // namespace astromesh::coordination

