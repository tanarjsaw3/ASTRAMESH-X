#pragma once

#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/alignment_graph.hpp"
#include "astromesh/graph/pose_graph.hpp"

#include <cstdint>
#include <vector>
#include <optional>
#include <cstring>
#include <utility>

namespace astromesh::net {

#pragma pack(push, 1)
struct AlignmentWireRecord {
    uint16_t from_rover{0};
    uint16_t to_rover{0};
    double tx{0.0};
    double ty{0.0};
    double theta{0.0};
    double confidence{0.0};
    double inlier_rmse{0.0};
    uint32_t inlier_count{0};
    uint64_t epoch{0};

    void serialize(std::vector<uint8_t>& buf) const;
    static bool deserialize(const uint8_t* data, size_t len, AlignmentWireRecord& out);
};
#pragma pack(pop)

struct GaugeCoordinatorConfig {
    double cycle_consistency_tolerance_m{0.35};
    bool auto_remediate_cycles{true};
    size_t min_inlier_count{3};
    double min_confidence{0.5};
};

struct GaugeCoordinatorMetrics {
    uint32_t pairwise_alignments_registered{0};
    uint32_t cycles_audited{0};
    uint32_t inconsistent_cycles_detected{0};
    uint32_t edges_invalidated{0};
    uint32_t anchor_re_elections{0};
};

/**
 * @brief Decentralized Gauge Coordinator & Transform Propagator.
 * Maintains the inter-subgraph AlignmentGraph, elects minimum-rover-ID anchor per component,
 * resolves maximum confidence transform path to anchor, and transforms local poses to global frame.
 */
class GaugeCoordinator {
public:
    explicit GaugeCoordinator(uint16_t local_rover_id, GaugeCoordinatorConfig config = {});

    uint16_t local_rover_id() const noexcept { return m_local_rover_id; }
    const GaugeCoordinatorConfig& config() const noexcept { return m_config; }
    const GaugeCoordinatorMetrics& metrics() const noexcept { return m_metrics; }

    graph::AlignmentGraph& alignment_graph() noexcept { return m_alignment_graph; }
    const graph::AlignmentGraph& alignment_graph() const noexcept { return m_alignment_graph; }

    bool register_pairwise_alignment(uint16_t from_rover,
                                    uint16_t to_rover,
                                    const geometry::SE2& transform_to_from,
                                    size_t correspondence_count,
                                    size_t inlier_count,
                                    double inlier_rmse,
                                    double confidence,
                                    uint64_t epoch = 0);

    bool register_wire_record(const AlignmentWireRecord& record);

    bool recompute_gauge();

    graph::CycleAuditResult audit_and_remediate_cycles();

    uint16_t current_anchor_id() const noexcept { return m_current_anchor_id; }
    bool is_anchor() const noexcept { return m_current_anchor_id == m_local_rover_id; }
    bool is_aligned() const noexcept { return m_is_aligned; }

    geometry::SE2 transform_from_anchor() const noexcept { return m_transform_from_anchor; }
    geometry::SE2 transform_to_anchor() const noexcept { return m_transform_from_anchor.inverse(); }

    geometry::SE2 transform_local_pose_to_global(const geometry::SE2& local_pose) const;
    graph::Pose2D transform_local_pose_to_global(const graph::Pose2D& local_pose) const;
    std::pair<double, double> transform_landmark_to_global(double lx, double ly) const;

    graph::PoseGraph transform_pose_graph(const graph::PoseGraph& local_graph) const;

private:
    uint16_t m_local_rover_id{0};
    GaugeCoordinatorConfig m_config;
    GaugeCoordinatorMetrics m_metrics;
    graph::AlignmentGraph m_alignment_graph;

    uint16_t m_current_anchor_id{0};
    geometry::SE2 m_transform_from_anchor{geometry::SE2::identity()};
    bool m_is_aligned{false};
};

} // namespace astromesh::net

