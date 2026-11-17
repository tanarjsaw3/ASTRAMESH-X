#include "astromesh/memory/distributed_compaction_protocol.hpp"

#include <cassert>
#include <iostream>

using namespace astromesh;
using namespace astromesh::graph;
using namespace astromesh::memory;

void test_distributed_compaction_wire_and_application() {
    std::cout << "[RUN] test_distributed_compaction_wire_and_application..." << std::endl;

    // Verify wire sizes
    assert(sizeof(CompactionNotification) <= 128);
    assert(sizeof(CompactionAck) <= 128);
    std::cout << "  -> CompactionNotification wire size: " << sizeof(CompactionNotification) << " B" << std::endl;
    std::cout << "  -> CompactionAck wire size: " << sizeof(CompactionAck) << " B" << std::endl;

    DistributedCompactor compactor1(1); // Rover 1
    DistributedCompactor compactor2(2); // Rover 2

    // Rover 2 maintains a replica of Rover 1's graph
    PoseGraph replica_graph;
    replica_graph.add_node(KeyframeNode(NodeId(1, 1), Pose2D(0, 0, 0)));
    replica_graph.add_node(KeyframeNode(NodeId(1, 2), Pose2D(2, 0, 0)));
    replica_graph.add_node(KeyframeNode(NodeId(1, 3), Pose2D(4, 0, 0)));

    replica_graph.add_edge(PoseGraphEdge(1, NodeId(1, 1), NodeId(1, 2), EdgeType::Odometry, Pose2D(2, 0, 0)));
    replica_graph.add_edge(PoseGraphEdge(2, NodeId(1, 2), NodeId(1, 3), EdgeType::Odometry, Pose2D(2, 0, 0)));

    // Rover 1 creates notification that Node(1, 2) was compacted into compounded edge (1, 1) -> (1, 3)
    auto notif = compactor1.create_notification(
        1, NodeId(1, 1), NodeId(1, 3), NodeId(1, 2), Pose2D(4, 0, 0), 50.0f, 500.0f);

    assert(notif.verify_crc32());

    // Rover 2 receives and applies notification
    core::VersionVector vv;
    auto ack = compactor2.handle_notification(notif, replica_graph, vv);

    assert(ack.status == 0); // Applied
    assert(ack.verify_crc32());
    assert(compactor2.peer_epoch(1) == 1);

    // Replica graph should now have 2 nodes (node 2 removed) and a direct edge from (1, 1) to (1, 3)
    assert(!replica_graph.has_node(NodeId(1, 2)));
    assert(replica_graph.has_node(NodeId(1, 1)));
    assert(replica_graph.has_node(NodeId(1, 3)));
    assert(replica_graph.num_nodes() == 2);

    // Idempotency: receiving the exact same notification again is a safe no-op
    auto ack_repeat = compactor2.handle_notification(notif, replica_graph, vv);
    assert(ack_repeat.status == 0);
    assert(replica_graph.num_nodes() == 2);

    std::cout << "[PASS] test_distributed_compaction_wire_and_application" << std::endl;
}

int main() {
    std::cout << "=================================================" << std::endl;
    std::cout << " AstroMesh-X: Distributed Compaction Unit Tests  " << std::endl;
    std::cout << "=================================================" << std::endl;

    test_distributed_compaction_wire_and_application();

    std::cout << "All Distributed Compaction tests PASSED." << std::endl;
    return 0;
}
