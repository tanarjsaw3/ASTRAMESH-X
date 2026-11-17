#include "astromesh/exploration/exploration_utility.hpp"

#include <algorithm>
#include <cmath>

namespace astromesh::exploration {

ExplorationUtilityModel::ExplorationUtilityModel(ExplorationUtilityWeights weights)
    : m_weights(weights) {}

double ExplorationUtilityModel::compute_dispersion(
    const std::vector<verification::Point2D>& points) noexcept {

    size_t n = points.size();
    if (n < 3) {
        return 0.0;
    }

    double cx = 0.0, cy = 0.0;
    for (const auto& p : points) {
        cx += p.x;
        cy += p.y;
    }
    cx /= static_cast<double>(n);
    cy /= static_cast<double>(n);

    double s_xx = 0.0, s_yy = 0.0, s_xy = 0.0;
    for (const auto& p : points) {
        double dx = p.x - cx;
        double dy = p.y - cy;
        s_xx += dx * dx;
        s_yy += dy * dy;
        s_xy += dx * dy;
    }
    s_xx /= static_cast<double>(n);
    s_yy /= static_cast<double>(n);
    s_xy /= static_cast<double>(n);

    double det = s_xx * s_yy - s_xy * s_xy;
    return (det > 0.0) ? det : 0.0;
}

UtilityBreakdown ExplorationUtilityModel::evaluate_frontier(
    const Frontier& frontier,
    const ExplorationContext& context,
    const std::vector<verification::Point2D>& local_landmarks) const {

    UtilityBreakdown ub;
    ub.is_heuristic_proxy = true; // Clearly marked: 2D dispersion proxy used rather than full joint Fisher matrix

    // 1. Information Gain: Dispersion of landmarks near frontier
    std::vector<verification::Point2D> nearby_lms;
    for (const auto& lm : local_landmarks) {
        double dx = frontier.position.x - lm.x;
        double dy = frontier.position.y - lm.y;
        if (std::hypot(dx, dy) <= frontier.spatial_coverage_radius) {
            nearby_lms.push_back(lm);
        }
    }
    double det = compute_dispersion(nearby_lms);
    ub.information_gain = (det > 0.0) ? std::log(1.0 + det) : 0.2; // Baseline exploratory value

    // 2. Spatial Coverage: Proportional to frontier coverage radius normalized by max travel
    ub.spatial_coverage = std::clamp(frontier.spatial_coverage_radius / 10.0, 0.1, 1.0);

    // 3. Geometric Novelty: Distance from evaluated frontier to current rover position
    double d_rover = std::hypot(frontier.position.x - context.rover_position.x,
                                frontier.position.y - context.rover_position.y);
    ub.geometric_novelty = std::clamp(d_rover / (2.0 * frontier.spatial_coverage_radius), 0.1, 1.0);

    // 4. Loop-Closure Potential: Proximity to known cross-rover loop closure regions
    double loop_bonus = 0.0;
    for (const auto& lc_pt : context.prospective_loop_closures) {
        double d_lc = std::hypot(frontier.position.x - lc_pt.x, frontier.position.y - lc_pt.y);
        loop_bonus += std::exp(-(d_lc * d_lc) / (2.0 * 25.0)); // Gaussian kernel with sigma=5.0m
    }
    ub.loop_closure_potential = std::min(2.0, loop_bonus);

    // 5. Travel Cost: Normalized Euclidean travel distance
    ub.travel_cost = std::min(1.0, d_rover / std::max(1.0, context.max_travel_meters));

    // 6. Network Connectivity Risk: Distance to nearest active peer from the candidate frontier
    double min_peer_dist = 1e9;
    if (context.active_peers.empty()) {
        // Fallback to elected anchor distance
        min_peer_dist = std::hypot(frontier.position.x - context.elected_anchor_position.x,
                                   frontier.position.y - context.elected_anchor_position.y);
    } else {
        for (const auto& peer_pos : context.active_peers) {
            double dp = std::hypot(frontier.position.x - peer_pos.x, frontier.position.y - peer_pos.y);
            if (dp < min_peer_dist) {
                min_peer_dist = dp;
            }
        }
    }

    if (min_peer_dist <= context.comm_range_meters) {
        // Safe communication range: linear scaling with channel loss
        ub.connectivity_risk = (min_peer_dist / context.comm_range_meters) * (1.0 + context.ewma_network_loss);
    } else {
        // Severed communication link: super-linear penalty
        double breach = (min_peer_dist - context.comm_range_meters) / context.comm_range_meters;
        ub.connectivity_risk = 1.0 + 3.0 * breach * (1.0 + context.ewma_network_loss);
    }

    // 7. Memory Pressure Penalty: Scaled from M7.15 controller
    if (context.memory_pressure == memory::MemoryPressureLevel::Normal) {
        ub.memory_penalty = 0.0;
    } else if (context.memory_pressure == memory::MemoryPressureLevel::Warning) {
        ub.memory_penalty = 1.5;
    } else {
        ub.memory_penalty = 5.0; // Critical pressure: heavily suppresses distant exploration
    }

    // 8. Peer Redundancy Penalty: Distance to other rovers' active targets
    double redundancy = 0.0;
    constexpr double R_sep = 8.0;
    for (const auto& target : context.peer_claimed_targets) {
        double dt = std::hypot(frontier.position.x - target.x, frontier.position.y - target.y);
        if (dt < R_sep) {
            redundancy += (1.0 - dt / R_sep);
        }
    }
    ub.redundancy_penalty = redundancy;

    // Combine into net utility
    double num = m_weights.weight_info * ub.information_gain +
                 m_weights.weight_coverage * ub.spatial_coverage +
                 m_weights.weight_novelty * ub.geometric_novelty +
                 m_weights.weight_loop_closure * ub.loop_closure_potential;

    double den = 1.0 +
                 m_weights.weight_travel_cost * ub.travel_cost +
                 m_weights.weight_connectivity * ub.connectivity_risk +
                 m_weights.weight_memory_pressure * ub.memory_penalty +
                 m_weights.weight_redundancy * ub.redundancy_penalty;

    ub.net_utility = (den > 0.0) ? (num / den) : 0.0;
    return ub;
}

} // namespace astromesh::exploration

