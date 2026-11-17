#include "astromesh/memory/distributed_compaction_protocol.hpp"

#include <cassert>
#include <iostream>

using namespace astromesh;
using namespace astromesh::graph;
using namespace astromesh::memory;

void test_partition_defer_and_recovery() {
    std::cout << "[RUN] test_partition_defer_and_recovery..." << std::endl;

    DistributedCompactor compactor1(1);
    DistributedCompactor compactor2(2);

    // Rover 2 starts with only Node(1, 1), missing Node(1, 2) and Node(1, 3) due to network partition
    PoseGraph replica_graph;
    replica_graph.add_node(KeyframeNode(NodeId(1, 1), Pose2D(0, 0, 0)));

    // Rover 1 compacted Node(1, 2) between (1, 1) and (1, 3)
    auto notif = compactor1.create_notification(
        10, NodeId(1, 1), NodeId(1, 3), NodeId(1, 2), Pose2D(4, 0, 0), 50.0f, 500.0f);

    core::VersionVector vv;
    // Rover 2 receives notification while partitioned
    auto ack = compactor2.handle_notification(notif, replica_graph, vv);

    // Must be safely deferred because target endpoint (1, 3) is missing
    assert(ack.status == 1); // Deferred
    assert(compactor2.deferred_count() == 1);

    // Partition heals: Rover 2 receives Node(1, 3) via reconciliation
    replica_graph.add_node(KeyframeNode(NodeId(1, 3), Pose2D(4, 0, 0)));

    // Process deferred queue
    size_t applied = compactor2.process_deferred(replica_graph);
    assert(applied == 1);
    assert(compactor2.deferred_count() == 0);
    assert(compactor2.peer_epoch(1) == 10);

    // Graph now has direct edge between (1, 1) and (1, 3)
    assert(replica_graph.has_node(NodeId(1, 1)));
    assert(replica_graph.has_node(NodeId(1, 3)));
    assert(replica_graph.num_edges() == 1);

    std::cout << "[PASS] test_partition_defer_and_recovery" << std::endl;
}

int main() {
    std::cout << "==========================================================" << std::endl;
    std::cout << " AstroMesh-X: Compaction Partition Recovery Unit Tests   " << std::endl;
    std::cout << "==========================================================" << std::endl;

    test_partition_defer_and_recovery();

    std::cout << "All Compaction Partition Recovery tests PASSED." << std::endl;
    return 0;
}
