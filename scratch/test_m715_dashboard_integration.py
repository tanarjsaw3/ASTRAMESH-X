#!/usr/bin/env python3
"""
Test M7.15 Memory-Bounded SLAM Telemetry & Dashboard Ingestion
Verifies UDP telemetry collector updates for memory compaction and pressure events.
"""

import json
import socket
import time
import os
import sys
sys.path.insert(0, os.path.abspath("."))

def main():
    print("Testing M7.15 Telemetry Ingestion...")
    from app.telemetry_collector import TelemetryCollector

    collector = TelemetryCollector(port=30005)
    collector.start()

    time.sleep(0.2)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = ("127.0.0.1", 30005)

    # 1. Send memory pressure event
    pressure_event = {
        "type": "memory_pressure",
        "rover_id": 1,
        "current_usage_bytes": 10240,
        "capacity_bytes": 14000,
        "rss_bytes": 5242880,
        "budget_bytes": 12000,
        "pressure_level": "Warning",
        "swarm_logical_bytes": 20480,
        "swarm_rss_bytes": 10485760
    }
    sock.sendto(json.dumps(pressure_event).encode("utf-8"), dest)
    time.sleep(0.2)

    # 2. Send memory compaction event
    compaction_event = {
        "type": "memory_compaction",
        "rover_id": 1,
        "nodes_pruned": 5,
        "landmarks_evicted": 3,
        "wire_bytes": 136,
        "notifications_sent": 2,
        "current_memory_bytes": 6144,
        "pressure_level": "Normal"
    }
    sock.sendto(json.dumps(compaction_event).encode("utf-8"), dest)
    time.sleep(0.3)

    snap = collector.get_snapshot()
    collector.stop()

    gm = snap["global_metrics"]
    print("Global Metrics:", gm)
    assert gm["total_nodes_pruned"] == 5, f"Expected 5, got {gm['total_nodes_pruned']}"
    assert gm["total_landmarks_evicted"] == 3, f"Expected 3, got {gm['total_landmarks_evicted']}"
    assert gm["total_compaction_wire_bytes"] == 136, f"Expected 136, got {gm['total_compaction_wire_bytes']}"
    assert gm["total_compaction_notifications"] == 2, f"Expected 2, got {gm['total_compaction_notifications']}"
    assert gm["peak_swarm_pressure"] == "Warning", f"Expected Warning, got {gm['peak_swarm_pressure']}"
    assert gm["current_swarm_logical_bytes"] == 20480
    assert gm["current_swarm_rss_bytes"] == 10485760

    r1 = snap["rovers"].get("1")
    assert r1 is not None, "Rover 1 not found"
    assert r1["nodes_compacted"] == 5, f"Expected 5, got {r1['nodes_compacted']}"
    assert r1["landmarks_evicted"] == 3, f"Expected 3, got {r1['landmarks_evicted']}"
    assert r1["pressure_level"] == "Normal"
    assert r1["memory_bytes"] == 6144

    print("ALL M7.15 TELEMETRY TESTS PASSED!")
    return 0

if __name__ == "__main__":
    sys.exit(main())
