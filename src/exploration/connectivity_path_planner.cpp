#include "astromesh/exploration/connectivity_path_planner.hpp"

#include <algorithm>
#include <cmath>

namespace astromesh::exploration {

std::string to_string(PlannerStatus status) {
    switch (status) {
        case PlannerStatus::Success: return "Success";
        case PlannerStatus::DegradedConnectivity: return "DegradedConnectivity";
        case PlannerStatus::ReturnToSwarm: return "ReturnToSwarm";
        case PlannerStatus::StationKeepingFallback: return "StationKeepingFallback";
        default: return "Unknown";
    }
}

ConnectivityPathPlanner::ConnectivityPathPlanner(
    uint16_t local_rover_id,
    ConnectivityPlannerConfig config,
    ExplorationUtilityWeights utility_weights)
    : m_local_rover_id(local_rover_id),
      m_config(config),
      m_utility_model(utility_weights) {}

void ConnectivityPathPlanner::reset_commitment() noexcept {
    m_committed_frontier_id = 0;
    m_committed_target = verification::Point2D(0.0, 0.0);
    m_committed_utility = 0.0;
}

std::vector<verification::Point2D> ConnectivityPathPlanner::generate_linear_waypoints(
    const verification::Point2D& from,
    const verification::Point2D& to,
    double step_size) const {

    std::vector<verification::Point2D> wps;
    double dist = std::hypot(to.x - from.x, to.y - from.y);
    if (dist <= 1e-4) {
        wps.push_back(to);
        return wps;
    }

    size_t num_steps = std::max(size_t{1}, static_cast<size_t>(std::ceil(dist / step_size)));
    for (size_t i = 1; i <= num_steps; ++i) {
        double frac = static_cast<double>(i) / static_cast<double>(num_steps);
        wps.push_back(verification::Point2D(from.x + frac * (to.x - from.x),
                                            from.y + frac * (to.y - from.y)));
    }
    return wps;
}

PathPlanResult ConnectivityPathPlanner::plan_destination(
    const ExplorationContext& context,
    const std::vector<Frontier>& available_frontiers,
    const std::vector<verification::Point2D>& local_landmarks,
    uint64_t current_step) {

    PathPlanResult result;

    // 1. Check Return-to-Swarm trigger: Critical Memory Pressure or complete disconnection
    if (context.memory_pressure == memory::MemoryPressureLevel::Critical) {
        // Return toward nearest peer or anchor to trigger loop closures and map compaction
        verification::Point2D return_dest = context.elected_anchor_position;
        if (!context.active_peers.empty()) {
            double min_d = 1e9;
            for (const auto& p : context.active_peers) {
                double d = std::hypot(context.rover_position.x - p.x, context.rover_position.y - p.y);
                if (d < min_d) {
                    min_d = d;
                    return_dest = p;
                }
            }
        }

        result.status = PlannerStatus::ReturnToSwarm;
        result.target_frontier_id = 0;
        result.target_position = return_dest;
        result.travel_distance = std::hypot(return_dest.x - context.rover_position.x,
                                            return_dest.y - context.rover_position.y);
        result.utility_score = 1.0;
        result.is_fallback = true;
        result.reason = "Critical memory pressure: returning toward swarm for compaction";
        result.waypoints = generate_linear_waypoints(context.rover_position, return_dest, m_config.waypoint_step_size_meters);
        reset_commitment();
        return result;
    }

    // 2. Evaluate all available frontiers
    struct CandidateScore {
        Frontier frontier;
        UtilityBreakdown breakdown;
    };

    std::vector<CandidateScore> scored;
    scored.reserve(available_frontiers.size());

    for (const auto& f : available_frontiers) {
        if (!f.is_available(current_step)) continue;

        double d_travel = std::hypot(f.position.x - context.rover_position.x,
                                     f.position.y - context.rover_position.y);
        if (d_travel > m_config.max_travel_meters) continue;

        UtilityBreakdown ub = m_utility_model.evaluate_frontier(f, context, local_landmarks);
        scored.push_back({f, ub});
    }

    // 3. Fallback: No candidate frontiers found
    if (scored.empty()) {
        result.status = PlannerStatus::StationKeepingFallback;
        result.target_frontier_id = 0;
        result.target_position = context.rover_position;
        result.travel_distance = 0.0;
        result.utility_score = 0.0;
        result.is_fallback = true;
        result.reason = "No reachable frontiers within bounds: holding position";
        result.waypoints = {context.rover_position};
        reset_commitment();
        return result;
    }

    // 4. Deterministic sorting: Highest utility -> Lowest travel cost -> Lowest frontier ID
    std::sort(scored.begin(), scored.end(), [](const CandidateScore& a, const CandidateScore& b) {
        if (std::abs(a.breakdown.net_utility - b.breakdown.net_utility) > 1e-4) {
            return a.breakdown.net_utility > b.breakdown.net_utility;
        }
        double d_a = std::hypot(a.frontier.position.x, a.frontier.position.y);
        double d_b = std::hypot(b.frontier.position.x, b.frontier.position.y);
        if (std::abs(d_a - d_b) > 1e-3) {
            return d_a < d_b;
        }
        return a.frontier.frontier_id < b.frontier.frontier_id;
    });

    // 5. Anti-oscillation commitment hysteresis:
    // If we have an active commitment, only switch if new best exceeds current target utility by bonus
    const CandidateScore* chosen = &scored.front();

    if (m_committed_frontier_id != 0) {
        auto it = std::find_if(scored.begin(), scored.end(), [this](const CandidateScore& cs) {
            return cs.frontier.frontier_id == m_committed_frontier_id;
        });

        if (it != scored.end()) {
            double current_util = it->breakdown.net_utility;
            if (scored.front().breakdown.net_utility < current_util + m_config.switching_hysteresis_bonus) {
                // Keep current commitment to prevent oscillatory thrashing
                chosen = &(*it);
            }
        }
    }

    // Update commitment
    m_committed_frontier_id = chosen->frontier.frontier_id;
    m_committed_target = chosen->frontier.position;
    m_committed_utility = chosen->breakdown.net_utility;

    // Check connectivity status at chosen destination
    double min_peer_d = 1e9;
    for (const auto& p : context.active_peers) {
        double d = std::hypot(chosen->frontier.position.x - p.x, chosen->frontier.position.y - p.y);
        if (d < min_peer_d) min_peer_d = d;
    }

    if (context.active_peers.empty() || min_peer_d > m_config.comm_range_meters) {
        result.status = PlannerStatus::DegradedConnectivity;
        result.reason = "Path leads into degraded/disconnected mesh zone";
    } else {
        result.status = PlannerStatus::Success;
        result.reason = "Nominal connected exploration trajectory";
    }

    result.target_frontier_id = chosen->frontier.frontier_id;
    result.target_position = chosen->frontier.position;
    result.travel_distance = std::hypot(chosen->frontier.position.x - context.rover_position.x,
                                        chosen->frontier.position.y - context.rover_position.y);
    result.utility_score = chosen->breakdown.net_utility;
    result.breakdown = chosen->breakdown;
    result.is_fallback = false;
    result.waypoints = generate_linear_waypoints(context.rover_position, chosen->frontier.position, m_config.waypoint_step_size_meters);

    return result;
}

} // namespace astromesh::exploration

