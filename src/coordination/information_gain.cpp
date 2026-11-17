#include "astromesh/coordination/information_gain.hpp"

#include <algorithm>
#include <cmath>

namespace astromesh::coordination {

double InformationGainEvaluator::compute_geometric_dispersion(
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

InformationGainResult InformationGainEvaluator::evaluate(
    const graph::NodeId& kf_a,
    const graph::NodeId& kf_b,
    const graph::PoseGraph& graph,
    const std::vector<verification::Point2D>& candidate_landmarks) const {

    InformationGainResult res;

    // 1. Dispersion component
    double det = compute_geometric_dispersion(candidate_landmarks);
    if (det < 1e-4) {
        res.is_degenerate = true;
        res.total_score = 0.0;
        return res;
    }
    res.dispersion_component = std::log(1.0 + det);

    // 2. Count existing verified edges between this rover pair
    size_t existing_edges = 0;
    bool has_direct_edge_between_kfs = false;

    for (const auto& [eid, e] : graph.edges()) {
        if (e.status == graph::EdgeVerificationStatus::Verified) {
            bool matches_pair = (e.from_node.rover_id == kf_a.rover_id && e.to_node.rover_id == kf_b.rover_id) ||
                                (e.from_node.rover_id == kf_b.rover_id && e.to_node.rover_id == kf_a.rover_id);
            if (matches_pair) {
                existing_edges++;
                if ((e.from_node == kf_a && e.to_node == kf_b) || (e.from_node == kf_b && e.to_node == kf_a)) {
                    has_direct_edge_between_kfs = true;
                }
            }
        }
    }

    if (has_direct_edge_between_kfs) {
        // Redundant closure between exactly the same two keyframes
        res.redundancy_penalty = 10.0;
        res.total_score = 0.0;
        return res;
    }

    res.topological_component = 1.0 / (1.0 + static_cast<double>(existing_edges));

    // 3. Drift reduction component
    if (kf_a.rover_id == kf_b.rover_id) {
        int32_t hop_diff = std::abs(static_cast<int32_t>(kf_a.local_index) -
                                    static_cast<int32_t>(kf_b.local_index));
        res.drift_component = std::sqrt(static_cast<double>(hop_diff));
    } else {
        // Cross-rover drift uncertainty scales with keyframe index from origin
        double total_hops = static_cast<double>(kf_a.local_index + kf_b.local_index);
        res.drift_component = std::sqrt(std::max(4.0, total_hops));
    }

    // 4. Redundancy penalty for oversaturated rover pairs
    if (existing_edges >= 4) {
        res.redundancy_penalty = 0.25 * static_cast<double>(existing_edges - 3);
    }

    // 5. Total Score
    double score = m_weights.alpha_drift * res.drift_component +
                   m_weights.alpha_dispersion * res.dispersion_component +
                   m_weights.alpha_topological * res.topological_component -
                   m_weights.alpha_redundancy * res.redundancy_penalty;

    res.total_score = std::max(0.0, score);
    return res;
}

} // namespace astromesh::coordination

