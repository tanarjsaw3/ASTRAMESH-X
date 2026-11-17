#ifndef ASTROMESH_MEMORY_TRACKER_HPP
#define ASTROMESH_MEMORY_TRACKER_HPP

#include "astromesh/comms/dtn_buffer.hpp"
#include "astromesh/coordination/loop_closure_opportunity.hpp"
#include "astromesh/core/landmark_key.hpp"
#include "astromesh/core/version_vector.hpp"
#include "astromesh/graph/alignment_graph.hpp"
#include "astromesh/graph/disjoint_set.hpp"
#include "astromesh/graph/pose_graph.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace astromesh::memory {

/**
 * @brief Memory usage breakdown for an individual subsystem.
 */
struct SubsystemMemoryRecord {
    std::string subsystem_name;
    size_t logical_bytes{0};        ///< Plain C++ struct data footprint
    size_t capacity_bytes{0};       ///< Allocated heap capacity including container overhead
    size_t serialized_wire_bytes{0};///< Wire payload footprint when serialized
    size_t item_count{0};           ///< Number of distinct elements tracked
};

/**
 * @brief Comprehensive memory report for a single rover node.
 */
struct RoverMemoryReport {
    uint16_t rover_id{0};
    SubsystemMemoryRecord pose_graph;
    SubsystemMemoryRecord landmarks;
    SubsystemMemoryRecord alignment_graph;
    SubsystemMemoryRecord active_coordination;
    SubsystemMemoryRecord causal_ledger;
    SubsystemMemoryRecord dtn_queue;
    SubsystemMemoryRecord dsu;

    size_t total_logical_bytes() const noexcept {
        return pose_graph.logical_bytes +
               landmarks.logical_bytes +
               alignment_graph.logical_bytes +
               active_coordination.logical_bytes +
               causal_ledger.logical_bytes +
               dtn_queue.logical_bytes +
               dsu.logical_bytes;
    }

    size_t total_capacity_bytes() const noexcept {
        return pose_graph.capacity_bytes +
               landmarks.capacity_bytes +
               alignment_graph.capacity_bytes +
               active_coordination.capacity_bytes +
               causal_ledger.capacity_bytes +
               dtn_queue.capacity_bytes +
               dsu.capacity_bytes;
    }

    size_t total_wire_bytes() const noexcept {
        return pose_graph.serialized_wire_bytes +
               landmarks.serialized_wire_bytes +
               alignment_graph.serialized_wire_bytes +
               active_coordination.serialized_wire_bytes +
               causal_ledger.serialized_wire_bytes +
               dtn_queue.serialized_wire_bytes +
               dsu.serialized_wire_bytes;
    }
};

/**
 * @brief Memory snapshot across the multi-rover swarm.
 */
struct SwarmMemorySnapshot {
    std::map<uint16_t, RoverMemoryReport> rovers;
    size_t process_rss_bytes{0};

    size_t total_swarm_logical_bytes() const noexcept {
        size_t total = 0;
        for (const auto& [_, r] : rovers) {
            total += r.total_logical_bytes();
        }
        return total;
    }

    size_t total_swarm_capacity_bytes() const noexcept {
        size_t total = 0;
        for (const auto& [_, r] : rovers) {
            total += r.total_capacity_bytes();
        }
        return total;
    }
};

/**
 * @brief Queries physical OS Resident Set Size (RSS) in bytes.
 * On Windows, calls GetProcessMemoryInfo. Returns 0 if unavailable.
 */
size_t get_process_rss_bytes() noexcept;

/**
 * @brief Unified memory tracking utility class.
 */
class MemoryTracker {
public:
    static SubsystemMemoryRecord measure_pose_graph(const graph::PoseGraph& graph);
    static SubsystemMemoryRecord measure_landmarks(const std::map<core::LandmarkKey, verification::Point2D>& lms);
    static SubsystemMemoryRecord measure_alignment_graph(const graph::AlignmentGraph& align_graph);
    static SubsystemMemoryRecord measure_opportunities(const coordination::LoopClosureOpportunityManager& opp_mgr);
    static SubsystemMemoryRecord measure_causal_ledger(const core::VersionVector& vv, size_t update_count);
    static SubsystemMemoryRecord measure_dtn_buffer(const comms::DtnBuffer& dtn);
    static SubsystemMemoryRecord measure_dsu(const graph::DisjointSet& dsu);

    static RoverMemoryReport generate_rover_report(
        uint16_t rover_id,
        const graph::PoseGraph& graph,
        const std::map<core::LandmarkKey, verification::Point2D>& lms,
        const graph::AlignmentGraph& align_graph,
        const coordination::LoopClosureOpportunityManager& opp_mgr,
        const core::VersionVector& vv,
        size_t update_count,
        const comms::DtnBuffer& dtn,
        const graph::DisjointSet& dsu);
};

} // namespace astromesh::memory

#endif // ASTROMESH_MEMORY_TRACKER_HPP
