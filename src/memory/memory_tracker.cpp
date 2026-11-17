#include "astromesh/memory/memory_tracker.hpp"
#include "astromesh/graph/map_update.hpp"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <psapi.h>
#endif

namespace astromesh::memory {

size_t get_process_rss_bytes() noexcept {
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
        return static_cast<size_t>(pmc.WorkingSetSize);
    }
    return 0;
#else
    return 0;
#endif
}

SubsystemMemoryRecord MemoryTracker::measure_pose_graph(const graph::PoseGraph& graph) {
    SubsystemMemoryRecord rec;
    rec.subsystem_name = "PoseGraph";
    rec.item_count = graph.num_nodes() + graph.num_edges();

    // Node: sizeof(KeyframeNode) ~128B + std::map tree overhead ~32B per node
    size_t node_logical = graph.num_nodes() * sizeof(graph::KeyframeNode);
    size_t node_capacity = graph.num_nodes() * (sizeof(graph::KeyframeNode) + 32);
    size_t node_wire = graph.num_nodes() * 48; // NodeId (8B) + Pose (24B) + metadata

    // Edge: sizeof(PoseGraphEdge) ~120B + std::vector/map overhead ~16B per edge
    size_t edge_logical = graph.num_edges() * sizeof(graph::PoseGraphEdge);
    size_t edge_capacity = graph.num_edges() * (sizeof(graph::PoseGraphEdge) + 16);
    size_t edge_wire = graph.num_edges() * 40; // Edge endpoints (16B) + transform (24B)

    rec.logical_bytes = node_logical + edge_logical;
    rec.capacity_bytes = node_capacity + edge_capacity;
    rec.serialized_wire_bytes = node_wire + edge_wire;
    return rec;
}

SubsystemMemoryRecord MemoryTracker::measure_landmarks(
    const std::map<core::LandmarkKey, verification::Point2D>& lms) {

    SubsystemMemoryRecord rec;
    rec.subsystem_name = "Landmarks";
    rec.item_count = lms.size();

    // LandmarkKey (16B) + Point2D (16B) = 32B logical
    rec.logical_bytes = lms.size() * (sizeof(core::LandmarkKey) + sizeof(verification::Point2D));
    // std::map node overhead: 3 pointers (24B) + color/padding (8B) = 32B per element
    rec.capacity_bytes = lms.size() * (sizeof(core::LandmarkKey) + sizeof(verification::Point2D) + 32);
    // Wire: 16B key + 16B coordinates = 32B wire
    rec.serialized_wire_bytes = lms.size() * 32;
    return rec;
}

SubsystemMemoryRecord MemoryTracker::measure_alignment_graph(const graph::AlignmentGraph& align_graph) {
    SubsystemMemoryRecord rec;
    rec.subsystem_name = "AlignmentGraph";
    rec.item_count = align_graph.num_edges();

    // AlignmentEdge ~96B
    rec.logical_bytes = align_graph.num_edges() * sizeof(graph::AlignmentEdge);
    rec.capacity_bytes = align_graph.num_edges() * (sizeof(graph::AlignmentEdge) + 32);
    rec.serialized_wire_bytes = align_graph.num_edges() * 48; // AlignmentWireRecord
    return rec;
}

SubsystemMemoryRecord MemoryTracker::measure_opportunities(
    const coordination::LoopClosureOpportunityManager& opp_mgr) {

    SubsystemMemoryRecord rec;
    rec.subsystem_name = "ActiveCoordination";
    rec.item_count = opp_mgr.size();

    // LoopClosureOpportunity ~88B
    rec.logical_bytes = opp_mgr.size() * sizeof(coordination::LoopClosureOpportunity);
    rec.capacity_bytes = opp_mgr.size() * (sizeof(coordination::LoopClosureOpportunity) + 16);
    rec.serialized_wire_bytes = opp_mgr.size() * 40;
    return rec;
}

SubsystemMemoryRecord MemoryTracker::measure_causal_ledger(
    const core::VersionVector& vv,
    size_t update_count) {

    (void)vv;
    SubsystemMemoryRecord rec;
    rec.subsystem_name = "CausalLedger";
    rec.item_count = 1 + update_count;

    size_t vv_logical = sizeof(core::VersionVector);
    size_t updates_logical = update_count * sizeof(graph::MapUpdate);

    rec.logical_bytes = vv_logical + updates_logical;
    rec.capacity_bytes = vv_logical + update_count * (sizeof(graph::MapUpdate) + 24);
    rec.serialized_wire_bytes = 16 * 8 + update_count * 48;
    return rec;
}

SubsystemMemoryRecord MemoryTracker::measure_dtn_buffer(const comms::DtnBuffer& dtn) {
    SubsystemMemoryRecord rec;
    rec.subsystem_name = "DTNQueue";
    rec.item_count = dtn.message_count();

    rec.logical_bytes = dtn.payload_bytes();
    rec.capacity_bytes = dtn.payload_bytes() + dtn.message_count() * 32; // queue node overhead
    rec.serialized_wire_bytes = dtn.payload_bytes();
    return rec;
}

SubsystemMemoryRecord MemoryTracker::measure_dsu(const graph::DisjointSet& dsu) {
    SubsystemMemoryRecord rec;
    rec.subsystem_name = "DSU";
    rec.item_count = dsu.num_nodes();

    rec.logical_bytes = dsu.num_nodes() * (sizeof(graph::NodeId) * 2 + sizeof(size_t));
    rec.capacity_bytes = dsu.num_nodes() * (sizeof(graph::NodeId) * 2 + sizeof(size_t) + 32);
    rec.serialized_wire_bytes = dsu.num_nodes() * 16;
    return rec;
}

RoverMemoryReport MemoryTracker::generate_rover_report(
    uint16_t rover_id,
    const graph::PoseGraph& graph,
    const std::map<core::LandmarkKey, verification::Point2D>& lms,
    const graph::AlignmentGraph& align_graph,
    const coordination::LoopClosureOpportunityManager& opp_mgr,
    const core::VersionVector& vv,
    size_t update_count,
    const comms::DtnBuffer& dtn,
    const graph::DisjointSet& dsu) {

    RoverMemoryReport rep;
    rep.rover_id = rover_id;
    rep.pose_graph = measure_pose_graph(graph);
    rep.landmarks = measure_landmarks(lms);
    rep.alignment_graph = measure_alignment_graph(align_graph);
    rep.active_coordination = measure_opportunities(opp_mgr);
    rep.causal_ledger = measure_causal_ledger(vv, update_count);
    rep.dtn_queue = measure_dtn_buffer(dtn);
    rep.dsu = measure_dsu(dsu);
    return rep;
}

} // namespace astromesh::memory
