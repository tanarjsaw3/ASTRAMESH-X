#pragma once

#include "astromesh/exploration/connectivity_path_planner.hpp"
#include "astromesh/exploration/frontier.hpp"
#include "astromesh/exploration/multi_rover_coordinator.hpp"
#include "astromesh/memory/map_compactor.hpp"
#include "astromesh/memory/memory_budget_controller.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace astromesh::exploration {

/**
 * @brief High-level autonomous exploration decision produced by the integrated agent.
 */
struct ExplorationDecision {
    PlannerStatus status{PlannerStatus::StationKeepingFallback};
    uint64_t target_frontier_id{0};
    verification::Point2D target_position{0.0, 0.0};
    verification::Point2D next_waypoint{0.0, 0.0};
    double utility_score{0.0};
    bool has_claim{false};
    FrontierClaim claim;
    std::string reason;
};

/**
 * @brief Fully integrated autonomous exploration agent coordinating frontier detection,
 * utility ranking, connectivity-aware path planning, multi-rover leases, and memory budgeting.
 */
class IntegratedExplorationAgent {
public:
    IntegratedExplorationAgent(
        uint16_t rover_id,
        FrontierDetectorConfig detector_config = {},
        ConnectivityPlannerConfig planner_config = {},
        ExplorationUtilityWeights utility_weights = {},
        memory::MemoryBudgetConfig memory_config = {});

    /**
     * @brief Executes one planning and coordination tick.
     */
    ExplorationDecision step(
        uint64_t current_step,
        const verification::Point2D& current_position,
        double current_heading,
        const graph::PoseGraph& local_graph,
        const std::vector<verification::Point2D>& mapped_landmarks,
        const std::vector<verification::Point2D>& active_peers,
        const verification::Point2D& elected_anchor_pos,
        const std::vector<verification::Point2D>& prospective_loop_closures = {},
        double ewma_network_loss = 0.0);

    /**
     * @brief Ingests an incoming claim from another rover over multi-hop mesh.
     */
    bool handle_remote_claim(const FrontierClaim& claim, uint64_t current_step);

    /**
     * @brief Notifies the agent that the rover has reached or observed a frontier.
     */
    void notify_frontier_reached(uint64_t frontier_id);

    uint16_t rover_id() const noexcept { return m_rover_id; }

    TopologicalFrontierDetector& detector() noexcept { return m_detector; }
    const TopologicalFrontierDetector& detector() const noexcept { return m_detector; }

    ConnectivityPathPlanner& planner() noexcept { return m_planner; }
    const ConnectivityPathPlanner& planner() const noexcept { return m_planner; }

    MultiRoverExplorationCoordinator& coordinator() noexcept { return m_coordinator; }
    const MultiRoverExplorationCoordinator& coordinator() const noexcept { return m_coordinator; }

    memory::MemoryBudgetController& memory_controller() noexcept { return m_memory_controller; }
    const memory::MemoryBudgetController& memory_controller() const noexcept { return m_memory_controller; }

private:
    uint16_t m_rover_id{0};
    TopologicalFrontierDetector m_detector;
    ConnectivityPathPlanner m_planner;
    MultiRoverExplorationCoordinator m_coordinator;
    memory::MemoryBudgetController m_memory_controller;
    memory::MapCompactor m_compactor;
};

} // namespace astromesh::exploration
