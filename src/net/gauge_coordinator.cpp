#include "astromesh/net/gauge_coordinator.hpp"

#include <algorithm>
#include <cstring>

namespace astromesh::net {

void AlignmentWireRecord::serialize(std::vector<uint8_t>& buf) const {
    size_t prev_size = buf.size();
    buf.resize(prev_size + sizeof(AlignmentWireRecord));
    std::memcpy(buf.data() + prev_size, this, sizeof(AlignmentWireRecord));
}

bool AlignmentWireRecord::deserialize(const uint8_t* data, size_t len, AlignmentWireRecord& out) {
    if (!data || len < sizeof(AlignmentWireRecord)) {
        return false;
    }
    std::memcpy(&out, data, sizeof(AlignmentWireRecord));
    return true;
}

GaugeCoordinator::GaugeCoordinator(uint16_t local_rover_id, GaugeCoordinatorConfig config)
    : m_local_rover_id(local_rover_id),
      m_config(config),
      m_current_anchor_id(local_rover_id),
      m_transform_from_anchor(geometry::SE2::identity()),
      m_is_aligned(true) {}

bool GaugeCoordinator::register_pairwise_alignment(uint16_t from_rover,
                                                   uint16_t to_rover,
                                                   const geometry::SE2& transform_to_from,
                                                   size_t correspondence_count,
                                                   size_t inlier_count,
                                                   double inlier_rmse,
                                                   double confidence,
                                                   uint64_t epoch) {
    if (from_rover == 0 || to_rover == 0 || from_rover == to_rover) {
        return false;
    }
    if (inlier_count < m_config.min_inlier_count || confidence < m_config.min_confidence) {
        return false;
    }

    graph::AlignmentEdge edge;
    edge.edge_id = 0;
    edge.from_rover = from_rover;
    edge.to_rover = to_rover;
    edge.transform_to_from = transform_to_from;
    edge.correspondence_count = correspondence_count;
    edge.inlier_count = inlier_count;
    edge.inlier_rmse = inlier_rmse;
    edge.confidence = confidence;
    edge.epoch = epoch;
    edge.is_active = true;

    if (!m_alignment_graph.add_or_update_edge(edge)) {
        return false;
    }

    m_metrics.pairwise_alignments_registered++;

    if (m_config.auto_remediate_cycles) {
        audit_and_remediate_cycles();
    }

    recompute_gauge();
    return true;
}

bool GaugeCoordinator::register_wire_record(const AlignmentWireRecord& record) {
    geometry::SE2 tx(record.tx, record.ty, record.theta);
    return register_pairwise_alignment(record.from_rover,
                                       record.to_rover,
                                       tx,
                                       record.inlier_count,
                                       record.inlier_count,
                                       record.inlier_rmse,
                                       record.confidence,
                                       record.epoch);
}

graph::CycleAuditResult GaugeCoordinator::audit_and_remediate_cycles() {
    auto audit = m_alignment_graph.audit_cycle_consistency(m_config.cycle_consistency_tolerance_m);
    m_metrics.cycles_audited += static_cast<uint32_t>(audit.evaluated_cycle_count);

    if (audit.has_inconsistent_cycles) {
        m_metrics.inconsistent_cycles_detected += static_cast<uint32_t>(audit.inconsistent_cycles.size());

        if (m_config.auto_remediate_cycles) {
            for (uint64_t bad_eid : audit.suggested_invalidations) {
                if (m_alignment_graph.invalidate_edge(bad_eid)) {
                    m_metrics.edges_invalidated++;
                }
            }
            recompute_gauge();
        }
    }

    return audit;
}

bool GaugeCoordinator::recompute_gauge() {
    auto comps = m_alignment_graph.get_connected_components();
    const std::vector<uint16_t>* my_comp = nullptr;

    for (const auto& comp : comps) {
        if (std::find(comp.begin(), comp.end(), m_local_rover_id) != comp.end()) {
            my_comp = &comp;
            break;
        }
    }

    if (!my_comp || my_comp->empty()) {
        // Local rover is isolated / standalone anchor
        uint16_t prev_anchor = m_current_anchor_id;
        m_current_anchor_id = m_local_rover_id;
        m_transform_from_anchor = geometry::SE2::identity();
        m_is_aligned = true;
        if (prev_anchor != m_local_rover_id) {
            m_metrics.anchor_re_elections++;
        }
        return true;
    }

    // Anchor is strictly minimum rover ID in the component
    uint16_t elected_anchor = my_comp->front(); // Already sorted ascending
    if (elected_anchor != m_current_anchor_id) {
        m_current_anchor_id = elected_anchor;
        m_metrics.anchor_re_elections++;
    }

    auto tx_opt = m_alignment_graph.resolve_transform_from_anchor(m_current_anchor_id, m_local_rover_id);
    if (tx_opt.has_value()) {
        m_transform_from_anchor = *tx_opt;
        m_is_aligned = true;
        return true;
    }

    m_is_aligned = false;
    return false;
}

geometry::SE2 GaugeCoordinator::transform_local_pose_to_global(const geometry::SE2& local_pose) const {
    return m_transform_from_anchor * local_pose;
}

graph::Pose2D GaugeCoordinator::transform_local_pose_to_global(const graph::Pose2D& local_pose) const {
    geometry::SE2 lp(local_pose);
    geometry::SE2 gp = m_transform_from_anchor * lp;
    return gp.to_pose2d();
}

std::pair<double, double> GaugeCoordinator::transform_landmark_to_global(double lx, double ly) const {
    double c = m_transform_from_anchor.cos_theta();
    double s = m_transform_from_anchor.sin_theta();
    double gx = m_transform_from_anchor.x() + c * lx - s * ly;
    double gy = m_transform_from_anchor.y() + s * lx + c * ly;
    return {gx, gy};
}

graph::PoseGraph GaugeCoordinator::transform_pose_graph(const graph::PoseGraph& local_graph) const {
    graph::PoseGraph out_graph = local_graph;
    for (const auto& [nid, kf] : local_graph.nodes()) {
        out_graph.update_node_pose(nid, transform_local_pose_to_global(kf.pose));
    }
    return out_graph;
}

} // namespace astromesh::net
