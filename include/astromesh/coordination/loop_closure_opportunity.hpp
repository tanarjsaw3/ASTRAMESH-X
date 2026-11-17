#pragma once

#include "astromesh/core/landmark_key.hpp"
#include "astromesh/graph/pose_graph.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace astromesh::coordination {

enum class OpportunityType : uint8_t {
    SpatialProximity = 1,
    HistoricalRevisit = 2,
    SubgraphOverlap = 3,
    PartitionReconnect = 4
};

struct LoopClosureOpportunity {
    uint64_t opportunity_id{0};
    uint16_t rover_a{0}; // Local rover
    uint16_t rover_b{0}; // Remote candidate rover
    graph::NodeId keyframe_a;
    graph::NodeId keyframe_b;
    OpportunityType type{OpportunityType::SpatialProximity};

    double estimated_spatial_distance_m{0.0};
    uint64_t temporal_separation_ms{0};
    size_t estimated_landmark_overlap{0};
    size_t expected_correspondences{0};
    double verification_feasibility{1.0}; // [0, 1]
    double confidence{0.5};               // [0, 1]
    uint64_t expected_wire_cost_bytes{64}; // Estimated wire budget needed
    double priority_score{0.0};

    uint64_t epoch{0};
    uint64_t created_timestamp_ms{0};
    uint64_t expiration_timestamp_ms{0};

    bool is_expired(uint64_t now_ms) const noexcept {
        return expiration_timestamp_ms > 0 && now_ms >= expiration_timestamp_ms;
    }
};

/**
 * @brief Manages detection, deduplication, and lifecycle of cross-rover loop closure opportunities.
 */
class LoopClosureOpportunityManager {
public:
    static constexpr double DEFAULT_SEARCH_RADIUS_M = 15.0;
    static constexpr uint64_t DEFAULT_OPPORTUNITY_TTL_MS = 10000; // 10 seconds

    explicit LoopClosureOpportunityManager(uint16_t local_rover_id);

    uint16_t local_rover_id() const noexcept { return m_local_rover_id; }

    /**
     * @brief Ingests an opportunity. Deduplicates against existing active opportunities.
     * @return true if added; false if duplicate or invalid.
     */
    bool add_opportunity(LoopClosureOpportunity opp);

    /**
     * @brief Detects spatial proximity candidates between local keyframes and remote keyframes.
     * @return Number of new opportunities created.
     */
    size_t detect_spatial_opportunities(
        const graph::PoseGraph& local_graph,
        uint16_t remote_rover_id,
        const std::vector<graph::KeyframeNode>& remote_keyframes,
        double max_radius_m = DEFAULT_SEARCH_RADIUS_M,
        uint64_t now_ms = 0,
        uint64_t ttl_ms = DEFAULT_OPPORTUNITY_TTL_MS);

    /**
     * @brief Detects opportunities from shared LandmarkKey sets (e.g. from IBLT or map updates).
     * @return Number of new opportunities created.
     */
    size_t detect_subgraph_overlap_opportunities(
        uint16_t remote_rover_id,
        const graph::NodeId& remote_kf,
        const std::vector<core::LandmarkKey>& remote_landmarks,
        const graph::PoseGraph& local_graph,
        uint64_t now_ms = 0,
        uint64_t ttl_ms = DEFAULT_OPPORTUNITY_TTL_MS);

    /**
     * @brief Filters redundant opportunities spatially close to each other.
     * @return Number of redundant opportunities pruned.
     */
    size_t filter_redundant_opportunities(double spatial_suppression_radius_m = 2.0);

    /**
     * @brief Prunes opportunities whose expiration timestamp has passed.
     * @return Number of expired opportunities removed.
     */
    size_t prune_expired(uint64_t now_ms);

    const std::vector<LoopClosureOpportunity>& opportunities() const noexcept {
        return m_opportunities;
    }

    std::vector<LoopClosureOpportunity>& opportunities_mut() noexcept {
        return m_opportunities;
    }

    std::optional<LoopClosureOpportunity> get_opportunity(uint64_t opp_id) const;

    void clear() noexcept {
        m_opportunities.clear();
    }

    size_t size() const noexcept {
        return m_opportunities.size();
    }

private:
    uint16_t m_local_rover_id{0};
    uint64_t m_next_opp_id{1};
    std::vector<LoopClosureOpportunity> m_opportunities;
};

} // namespace astromesh::coordination

