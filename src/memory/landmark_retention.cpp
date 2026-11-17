#include "astromesh/memory/landmark_retention.hpp"

#include <algorithm>

namespace astromesh::memory {

bool LandmarkLifecycleManager::add_or_update_observation(
    const core::LandmarkKey& key,
    const graph::NodeId& kf_id,
    const verification::Point2D& pt,
    double conf,
    uint64_t t_ns) {

    auto& rec = m_landmarks[key];
    rec.key = key;
    rec.estimated_position = pt; // Store latest localized coordinate
    rec.last_observed_timestamp = t_ns;

    if (kf_id.rover_id != key.rover_id()) {
        rec.cross_rover_sighting_count++;
    }

    // Check if keyframe is already recorded
    for (auto& obs : rec.observations) {
        if (obs.observing_keyframe == kf_id) {
            obs.local_coord = pt;
            obs.confidence = conf;
            obs.timestamp_ns = t_ns;
            return true;
        }
    }

    rec.observations.push_back({kf_id, pt, conf, t_ns});
    return true;
}

void LandmarkLifecycleManager::mark_as_loop_closure_support(const core::LandmarkKey& key, bool is_support) {
    auto it = m_landmarks.find(key);
    if (it != m_landmarks.end()) {
        it->second.is_loop_closure_support = is_support;
    }
}

size_t LandmarkLifecycleManager::total_observations() const noexcept {
    size_t total = 0;
    for (const auto& [_, rec] : m_landmarks) {
        total += rec.observations.size();
    }
    return total;
}

LandmarkPruningSummary LandmarkLifecycleManager::prune_to_budget(size_t target_max_landmarks) {
    LandmarkPruningSummary summary;
    size_t target_max = (target_max_landmarks > 0) ? target_max_landmarks : m_config.max_landmarks;

    // Step 1: Prune redundant observations per landmark
    for (auto& [_, rec] : m_landmarks) {
        if (rec.observations.size() > m_config.max_observations_per_landmark) {
            size_t excess = rec.observations.size() - m_config.max_observations_per_landmark;
            // Sort by confidence descending, keeping highest confidence
            std::sort(rec.observations.begin(), rec.observations.end(),
                      [](const LandmarkObservationMeta& a, const LandmarkObservationMeta& b) {
                          return a.confidence > b.confidence;
                      });
            rec.observations.resize(m_config.max_observations_per_landmark);
            summary.observations_pruned += excess;
        }
    }

    // Step 2: Evict uncritical landmarks if total count exceeds target
    if (target_max > 0 && m_landmarks.size() > target_max) {
        std::vector<core::LandmarkKey> evictable;

        for (const auto& [key, rec] : m_landmarks) {
            if (rec.is_loop_closure_support) {
                summary.protected_landmarks_retained++;
            } else {
                evictable.push_back(key);
            }
        }

        // Sort evictable: lowest cross-rover sightings first, then lowest observation count
        std::sort(evictable.begin(), evictable.end(),
                  [this](const core::LandmarkKey& a, const core::LandmarkKey& b) {
                      const auto& rA = m_landmarks[a];
                      const auto& rB = m_landmarks[b];
                      if (rA.cross_rover_sighting_count != rB.cross_rover_sighting_count) {
                          return rA.cross_rover_sighting_count < rB.cross_rover_sighting_count;
                      }
                      if (rA.observations.size() != rB.observations.size()) {
                          return rA.observations.size() < rB.observations.size();
                      }
                      return rA.last_observed_timestamp < rB.last_observed_timestamp;
                  });

        size_t need_to_evict = m_landmarks.size() - target_max;
        size_t evict_count = std::min(need_to_evict, evictable.size());

        for (size_t i = 0; i < evict_count; ++i) {
            m_landmarks.erase(evictable[i]);
            summary.landmarks_evicted++;
        }
    }

    summary.remaining_landmarks = m_landmarks.size();
    return summary;
}

std::map<core::LandmarkKey, verification::Point2D> LandmarkLifecycleManager::export_active_landmarks() const {
    std::map<core::LandmarkKey, verification::Point2D> out;
    for (const auto& [key, rec] : m_landmarks) {
        out[key] = rec.estimated_position;
    }
    return out;
}

} // namespace astromesh::memory
