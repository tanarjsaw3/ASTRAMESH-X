#pragma once

#include "astromesh/exploration/frontier.hpp"
#include "astromesh/memory/memory_budget_controller.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <cmath>
#include <cstdint>
#include <map>
#include <vector>

namespace astromesh::exploration {

/**
 * @brief Weighting factors for multi-attribute exploration utility evaluation.
 */
struct ExplorationUtilityWeights {
    double weight_info{1.0};           // Expected information gain / dispersion
    double weight_coverage{0.8};       // Unexplored spatial angular coverage
    double weight_novelty{1.0};        // Distance from nearest existing keyframe
    double weight_loop_closure{1.5};   // Proximity to prospective loop closure rendezvous
    double weight_travel_cost{0.5};    // Travel distance cost
    double weight_connectivity{1.2};   // Network separation risk
    double weight_memory_pressure{1.0};// Penalty for high memory pressure
    double weight_redundancy{1.5};     // Penalty for overlap with other rovers' targets
};

/**
 * @brief Detailed breakdown of individual utility sub-components for transparency and auditing.
 */
struct UtilityBreakdown {
    double information_gain{0.0};
    double spatial_coverage{0.0};
    double geometric_novelty{0.0};
    double loop_closure_potential{0.0};
    double travel_cost{0.0};
    double connectivity_risk{0.0};
    double memory_penalty{0.0};
    double redundancy_penalty{0.0};
    double net_utility{0.0};
    bool is_heuristic_proxy{true};     // True if using 2D dispersion proxy rather than full Fisher matrix
};

/**
 * @brief Contextual operational state of the evaluating rover and its local network.
 */
struct ExplorationContext {
    verification::Point2D rover_position{0.0, 0.0};
    double rover_heading{0.0};
    memory::MemoryPressureLevel memory_pressure{memory::MemoryPressureLevel::Normal};
    std::vector<verification::Point2D> active_peers;
    verification::Point2D elected_anchor_position{0.0, 0.0};
    std::vector<verification::Point2D> peer_claimed_targets;
    std::vector<verification::Point2D> prospective_loop_closures;
    double comm_range_meters{15.0};
    double max_travel_meters{30.0};
    double ewma_network_loss{0.0};
};

/**
 * @brief Evaluates multi-attribute exploration utility with explicit connectivity and memory risk accounting.
 */
class ExplorationUtilityModel {
public:
    explicit ExplorationUtilityModel(ExplorationUtilityWeights weights = {});

    /**
     * @brief Computes net utility score and full component breakdown for a candidate frontier.
     */
    UtilityBreakdown evaluate_frontier(
        const Frontier& frontier,
        const ExplorationContext& context,
        const std::vector<verification::Point2D>& local_landmarks) const;

    const ExplorationUtilityWeights& weights() const noexcept { return m_weights; }
    void set_weights(const ExplorationUtilityWeights& weights) noexcept { m_weights = weights; }

    /**
     * @brief Computes 2D geometric dispersion determinant as a mathematical proxy for spatial information gain.
     */
    static double compute_dispersion(const std::vector<verification::Point2D>& points) noexcept;

private:
    ExplorationUtilityWeights m_weights;
};

} // namespace astromesh::exploration

