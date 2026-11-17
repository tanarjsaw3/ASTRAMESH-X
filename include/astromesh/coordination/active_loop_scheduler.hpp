#pragma once

#include "astromesh/coordination/information_gain.hpp"
#include "astromesh/coordination/loop_closure_opportunity.hpp"

#include <cstdint>
#include <map>
#include <vector>

namespace astromesh::coordination {

struct SchedulerConfig {
    double weight_info{2.0};       // Information gain weight
    double weight_quality{1.5};    // Correspondence quality weight
    double weight_prob{1.0};       // Verification probability weight
    double weight_link{1.0};       // Network link quality weight
    double weight_cost{0.5};       // Wire cost penalty weight
    double weight_redundancy{1.0}; // Redundancy penalty weight
    uint64_t max_batch_wire_bytes{2048};
    size_t max_scheduled_per_round{4};
};

struct PeerLinkState {
    double loss_rate_ewma{0.0};
    double rtt_ms{10.0};
    double historical_verification_success_rate{0.9};
};

struct SchedulerMetrics {
    uint32_t opportunities_evaluated{0};
    uint32_t opportunities_scheduled{0};
    uint32_t opportunities_pruned{0};
    uint64_t scheduled_wire_bytes{0};
};

/**
 * @brief Multi-attribute Active Loop-Closure Scheduler.
 * Evaluates candidate opportunities against information gain, network state, and communication budget.
 */
class ActiveLoopClosureScheduler {
public:
    explicit ActiveLoopClosureScheduler(uint16_t local_rover_id, SchedulerConfig config = {});

    uint16_t local_rover_id() const noexcept { return m_local_rover_id; }
    const SchedulerConfig& config() const noexcept { return m_config; }
    const SchedulerMetrics& metrics() const noexcept { return m_metrics; }

    void set_peer_link_state(uint16_t peer_id, PeerLinkState state);
    PeerLinkState get_peer_link_state(uint16_t peer_id) const;

    /**
     * @brief Computes composite priority score for an opportunity.
     */
    double score_opportunity(
        LoopClosureOpportunity& opp,
        const graph::PoseGraph& local_graph,
        const std::vector<verification::Point2D>& landmarks) const;

    /**
     * @brief Evaluates, scores, and extracts the top prioritized batch of opportunities
     * respecting wire budget and candidate limit.
     */
    std::vector<LoopClosureOpportunity> schedule_batch(
        LoopClosureOpportunityManager& opp_mgr,
        const graph::PoseGraph& local_graph,
        const std::map<uint64_t, std::vector<verification::Point2D>>& opp_landmarks,
        uint64_t wire_budget_bytes = 0,
        size_t max_count = 0);

private:
    uint16_t m_local_rover_id{0};
    SchedulerConfig m_config;
    SchedulerMetrics m_metrics;
    InformationGainEvaluator m_info_evaluator;
    std::map<uint16_t, PeerLinkState> m_peer_states;
};

} // namespace astromesh::coordination

