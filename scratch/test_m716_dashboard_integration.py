#!/usr/bin/env python3
"""
Test M7.16 Autonomous Exploration Telemetry & Dashboard Ingestion
Verifies UDP telemetry collector updates for exploration decision and claim events.
"""

import json
import os
import socket
import sys
import time

sys.path.insert(0, os.path.abspath("."))

def main():
    print("Testing M7.16 Exploration Telemetry Ingestion...")
    from app.telemetry_collector import TelemetryCollector

    collector = TelemetryCollector(port=30006)
    collector.start()

    time.sleep(0.2)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = ("127.0.0.1", 30006)

    # 1. Send exploration decision event
    expl_event = {
        "type": "exploration_decision",
        "rover_id": 1,
        "status": "Success",
        "utility": 0.88,
        "target": {"x": 12.5, "y": 8.0, "frontier_id": 101},
        "completed_delta": 2,
        "total_detected": 15,
        "active_frontiers": 8,
        "coverage_percent": 64.5,
        "uptime_percent": 96.0
    }
    sock.sendto(json.dumps(expl_event).encode("utf-8"), dest)
    time.sleep(0.3)

    snap = collector.get_snapshot()
    collector.stop()

    gm = snap["global_metrics"]
    print("Global Metrics:", gm)
    assert gm["total_frontiers_completed"] == 2, f"Expected 2, got {gm['total_frontiers_completed']}"
    assert gm["total_frontiers_detected"] == 15, f"Expected 15, got {gm['total_frontiers_detected']}"
    assert gm["active_frontiers_count"] == 8, f"Expected 8, got {gm['active_frontiers_count']}"
    assert gm["swarm_coverage_percent"] == 64.5, f"Expected 64.5, got {gm['swarm_coverage_percent']}"
    assert gm["connectivity_uptime_percent"] == 96.0, f"Expected 96.0, got {gm['connectivity_uptime_percent']}"

    r1 = snap["rovers"].get("1")
    assert r1 is not None, "Rover 1 not found"
    assert r1["planner_status"] == "Success"
    assert r1["planner_utility"] == 0.88
    assert r1["exploration_target"]["frontier_id"] == 101
    assert r1["frontiers_completed"] == 2

    print("ALL M7.16 EXPLORATION TELEMETRY TESTS PASSED!")
    return 0

if __name__ == "__main__":
    sys.exit(main())

