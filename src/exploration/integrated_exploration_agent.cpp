#include "astromesh/exploration/integrated_exploration_agent.hpp"

namespace astromesh::exploration {

IntegratedExplorationAgent::IntegratedExplorationAgent(
    uint16_t rover_id,
    FrontierDetectorConfig detector_config,
    ConnectivityPlannerConfig planner_config,
    ExplorationUtilityWeights utility_weights,
    memory::MemoryBudgetConfig memory_config)
    : m_rover_id(rover_id),
      m_detector(detector_config),
      m_planner(rover_id, planner_config, utility_weights),
      m_coordinator(rover_id),
      m_memory_controller(memory_config) {}

ExplorationDecision IntegratedExplorationAgent::step(
    uint64_t current_step,
    const verification::Point2D& current_position,
    double current_heading,
    const graph::PoseGraph& local_graph,
    const std::vector<verification::Point2D>& mapped_landmarks,
    const std::vector<verification::Point2D>& active_peers,
    const verification::Point2D& elected_anchor_pos,
    const std::vector<verification::Point2D>& prospective_loop_closures,
    double ewma_network_loss) {

    // 1. Update and detect frontiers from topological graph
    m_detector.update_frontiers(local_graph, mapped_landmarks, current_step);
    m_detector.detect_frontiers(local_graph, mapped_landmarks, m_rover_id, current_step);
    m_coordinator.prune_expired_claims(current_step);

    // 2. Build exploration context
    ExplorationContext ctx;
    ctx.rover_position = current_position;
    ctx.rover_heading = current_heading;
    ctx.memory_pressure = m_memory_controller.current_pressure();
    ctx.active_peers = active_peers;
    ctx.elected_anchor_position = elected_anchor_pos;
    ctx.peer_claimed_targets = m_coordinator.active_peer_targets(current_step);
    ctx.prospective_loop_closures = prospective_loop_closures;
    ctx.comm_range_meters = m_planner.config().comm_range_meters;
    ctx.max_travel_meters = m_planner.config().max_travel_meters;
    ctx.ewma_network_loss = ewma_network_loss;

    // 3. Plan optimal connectivity-aware destination
    auto plan = m_planner.plan_destination(ctx, m_detector.active_frontiers(), mapped_landmarks, current_step);

    ExplorationDecision decision;
    decision.status = plan.status;
    decision.target_frontier_id = plan.target_frontier_id;
    decision.target_position = plan.target_position;
    decision.utility_score = plan.utility_score;
    decision.reason = plan.reason;

    if (!plan.waypoints.empty()) {
        decision.next_waypoint = plan.waypoints.front();
    } else {
        decision.next_waypoint = current_position;
    }

    // 4. Claim frontier if a valid non-fallback frontier was chosen
    if (plan.target_frontier_id > 0 && !plan.is_fallback) {
        m_detector.claim_frontier(plan.target_frontier_id, m_rover_id, 30, current_step);
        decision.claim = m_coordinator.create_local_claim(
            plan.target_frontier_id, plan.target_position, plan.utility_score, current_step, 30);
        decision.has_claim = true;
    } else {
        decision.has_claim = false;
    }

    return decision;
}

bool IntegratedExplorationAgent::handle_remote_claim(const FrontierClaim& claim, uint64_t current_step) {
    bool accepted = m_coordinator.process_remote_claim(claim, current_step);
    if (accepted) {
        // If peer claimed a frontier we are tracking, update local frontier status to Claimed
        auto* f = m_detector.find_frontier_mut(claim.frontier_id);
        if (f) {
            f->status = FrontierStatus::Claimed;
            f->claimed_by_rover = claim.rover_id;
            f->claim_lease_expiration = current_step + claim.lease_ttl_steps;
        }

        // If this dominates our active target, reset planner commitment so we smoothly replan
        if (m_planner.committed_frontier_id() == claim.frontier_id && claim.rover_id != m_rover_id) {
            m_planner.reset_commitment();
        }
    }
    return accepted;
}

void IntegratedExplorationAgent::notify_frontier_reached(uint64_t frontier_id) {
    m_detector.mark_completed(frontier_id);
    if (m_planner.committed_frontier_id() == frontier_id) {
        m_planner.reset_commitment();
    }
}

} // namespace astromesh::exploration
