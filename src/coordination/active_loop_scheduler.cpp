#include "astromesh/coordination/active_loop_scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace astromesh::coordination {

ActiveLoopClosureScheduler::ActiveLoopClosureScheduler(uint16_t local_rover_id, SchedulerConfig config)
    : m_local_rover_id(local_rover_id), m_config(config) {}

void ActiveLoopClosureScheduler::set_peer_link_state(uint16_t peer_id, PeerLinkState state) {
    m_peer_states[peer_id] = state;
}

PeerLinkState ActiveLoopClosureScheduler::get_peer_link_state(uint16_t peer_id) const {
    auto it = m_peer_states.find(peer_id);
    if (it != m_peer_states.end()) {
        return it->second;
    }
    return PeerLinkState{};
}

double ActiveLoopClosureScheduler::score_opportunity(
    LoopClosureOpportunity& opp,
    const graph::PoseGraph& local_graph,
    const std::vector<verification::Point2D>& landmarks) const {

    auto info_res = m_info_evaluator.evaluate(opp.keyframe_a, opp.keyframe_b, local_graph, landmarks);
    if (info_res.is_degenerate) {
        opp.priority_score = 0.0;
        return 0.0;
    }

    double I = info_res.total_score;
    double Q = std::min(1.0, static_cast<double>(opp.expected_correspondences) / 10.0);

    PeerLinkState p = get_peer_link_state(opp.rover_b);
    double P_verif = p.historical_verification_success_rate;
    double L_net = std::clamp(1.0 - p.loss_rate_ewma - (p.rtt_ms / 500.0), 0.0, 1.0);
    double C_wire = static_cast<double>(opp.expected_wire_cost_bytes) / 512.0;
    double R_dup = info_res.redundancy_penalty;

    double num = m_config.weight_info * I +
                 m_config.weight_quality * Q +
                 m_config.weight_prob * P_verif +
                 m_config.weight_link * L_net;

    double den = 1.0 + m_config.weight_cost * C_wire + m_config.weight_redundancy * R_dup;

    opp.priority_score = (den > 0.0) ? (num / den) : 0.0;
    return opp.priority_score;
}

std::vector<LoopClosureOpportunity> ActiveLoopClosureScheduler::schedule_batch(
    LoopClosureOpportunityManager& opp_mgr,
    const graph::PoseGraph& local_graph,
    const std::map<uint64_t, std::vector<verification::Point2D>>& opp_landmarks,
    uint64_t wire_budget_bytes,
    size_t max_count) {

    uint64_t budget = (wire_budget_bytes > 0) ? wire_budget_bytes : m_config.max_batch_wire_bytes;
    size_t limit = (max_count > 0) ? max_count : m_config.max_scheduled_per_round;

    auto& opps = opp_mgr.opportunities_mut();

    // 1. Score each opportunity
    for (auto& opp : opps) {
        m_metrics.opportunities_evaluated++;
        std::vector<verification::Point2D> lms;
        auto it = opp_landmarks.find(opp.opportunity_id);
        if (it != opp_landmarks.end()) {
            lms = it->second;
        }
        score_opportunity(opp, local_graph, lms);
    }

    // 2. Deterministic sort
    std::sort(opps.begin(), opps.end(), [](const LoopClosureOpportunity& a, const LoopClosureOpportunity& b) {
        if (std::abs(a.priority_score - b.priority_score) > 1e-4) {
            return a.priority_score > b.priority_score;
        }
        if (a.expected_correspondences != b.expected_correspondences) {
            return a.expected_correspondences > b.expected_correspondences;
        }
        if (a.rover_b != b.rover_b) {
            return a.rover_b < b.rover_b;
        }
        if (a.keyframe_b.local_index != b.keyframe_b.local_index) {
            return a.keyframe_b.local_index < b.keyframe_b.local_index;
        }
        return a.opportunity_id < b.opportunity_id;
    });

    // 3. Select within budget using two-pass fair scheduling (spanning tree first, then reinforcement)
    std::vector<LoopClosureOpportunity> scheduled;
    uint64_t accumulated_bytes = 0;

    // Disjoint set over rover IDs to ensure maximum spanning tree coverage across the entire swarm
    struct RoverDSU {
        std::map<uint16_t, uint16_t> parent;
        uint16_t find(uint16_t x) {
            if (parent.find(x) == parent.end()) parent[x] = x;
            if (parent[x] == x) return x;
            return parent[x] = find(parent[x]);
        }
        void unite(uint16_t a, uint16_t b) {
            parent[find(a)] = find(b);
        }
    } dsu;

    // Pass 1: Spanning tree over rover components to guarantee global gauge connectivity
    for (const auto& opp : opps) {
        if (opp.priority_score <= 0.0) continue;
        if (scheduled.size() >= limit) break;

        if (dsu.find(opp.rover_a) != dsu.find(opp.rover_b)) {
            if (accumulated_bytes + opp.expected_wire_cost_bytes <= budget) {
                dsu.unite(opp.rover_a, opp.rover_b);
                scheduled.push_back(opp);
                accumulated_bytes += opp.expected_wire_cost_bytes;
            }
        }
    }

    // Pass 2: Fill remaining budget with redundant reinforcement opportunities
    for (const auto& opp : opps) {
        if (opp.priority_score <= 0.0) continue;
        if (scheduled.size() >= limit) break;

        bool already_scheduled = false;
        for (const auto& s : scheduled) {
            if (s.opportunity_id == opp.opportunity_id) {
                already_scheduled = true;
                break;
            }
        }

        if (!already_scheduled && (accumulated_bytes + opp.expected_wire_cost_bytes <= budget)) {
            scheduled.push_back(opp);
            accumulated_bytes += opp.expected_wire_cost_bytes;
        }
    }

    m_metrics.opportunities_scheduled += static_cast<uint32_t>(scheduled.size());
    m_metrics.scheduled_wire_bytes += accumulated_bytes;

    return scheduled;
}

} // namespace astromesh::coordination
