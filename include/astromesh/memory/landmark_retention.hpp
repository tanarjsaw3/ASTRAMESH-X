#ifndef ASTROMESH_LANDMARK_RETENTION_HPP
#define ASTROMESH_LANDMARK_RETENTION_HPP

#include "astromesh/core/landmark_key.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

namespace astromesh::memory {

/**
 * @brief Metadata for a single spatial observation of a landmark.
 */
struct LandmarkObservationMeta {
    graph::NodeId observing_keyframe;
    verification::Point2D local_coord;
    double confidence{1.0};
    uint64_t timestamp_ns{0};
};

/**
 * @brief Rich lifecycle record for an individual landmark.
 */
struct LandmarkRecord {
    core::LandmarkKey key;
    verification::Point2D estimated_position;
    std::vector<LandmarkObservationMeta> observations;
    bool is_loop_closure_support{false};  ///< Protected if supporting a verified closure
    uint32_t cross_rover_sighting_count{0};
    uint64_t last_observed_timestamp{0};
};

/**
 * @brief Configuration for landmark retention and observation pruning.
 */
struct LandmarkRetentionConfig {
    size_t max_landmarks{500};
    size_t max_observations_per_landmark{5};
    double min_confidence{0.20};
    bool preserve_geometric_diversity{true};
};

/**
 * @brief Summary report of a landmark pruning pass.
 */
struct LandmarkPruningSummary {
    size_t observations_pruned{0};
    size_t landmarks_evicted{0};
    size_t protected_landmarks_retained{0};
    size_t remaining_landmarks{0};
};

/**
 * @brief Manages the lifecycle, observation history, and memory-bounded retention of landmarks.
 */
class LandmarkLifecycleManager {
public:
    explicit LandmarkLifecycleManager(const LandmarkRetentionConfig& config = {}) noexcept
        : m_config(config) {}

    bool add_or_update_observation(
        const core::LandmarkKey& key,
        const graph::NodeId& kf_id,
        const verification::Point2D& pt,
        double conf = 1.0,
        uint64_t t_ns = 0);

    void mark_as_loop_closure_support(const core::LandmarkKey& key, bool is_support = true);

    /**
     * @brief Prunes redundant observations and evicts uncritical landmarks to meet budget.
     */
    LandmarkPruningSummary prune_to_budget(size_t target_max_landmarks = 0);

    bool has_landmark(const core::LandmarkKey& key) const noexcept {
        return m_landmarks.find(key) != m_landmarks.end();
    }

    const LandmarkRecord* get_record(const core::LandmarkKey& key) const noexcept {
        auto it = m_landmarks.find(key);
        return (it != m_landmarks.end()) ? &it->second : nullptr;
    }

    size_t landmark_count() const noexcept { return m_landmarks.size(); }
    size_t total_observations() const noexcept;

    std::map<core::LandmarkKey, verification::Point2D> export_active_landmarks() const;

    const LandmarkRetentionConfig& config() const noexcept { return m_config; }
    void set_config(const LandmarkRetentionConfig& config) noexcept { m_config = config; }

private:
    LandmarkRetentionConfig m_config;
    std::map<core::LandmarkKey, LandmarkRecord> m_landmarks;
};

} // namespace astromesh::memory

#endif // ASTROMESH_LANDMARK_RETENTION_HPP

