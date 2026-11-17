#!/usr/bin/env python3
"""
Test M7.12 Live Dashboard and Telemetry Collector Integration.
Verifies real-time ingestion, SSE fanout, version vector tracking,
partition event logging, and global consistency state snapshots.
"""

import json
import sys
import time
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parent.parent
if str(WORKSPACE) not in sys.path:
    sys.path.insert(0, str(WORKSPACE))

from app.telemetry_collector import TelemetryCollector

def test_telemetry_collector_m712():
    print("=========================================================")
    print("  TESTING M7.12 TELEMETRY COLLECTOR & DASHBOARD STREAM   ")
    print("=========================================================")

    collector = TelemetryCollector(port=29995)
    collector.start()

    sub_q = collector.subscribe()

    # 1. Ingest Rover 1 update with Version Vector
    ev1 = {
        "type": "heartbeat",
        "rover_id": 1,
        "pose": {"x": 1.5, "y": 2.5, "theta": 0.1},
        "peers": [2],
        "version_vector": {"1": 10, "2": 8},
        "routes": [{"dest": 3, "next_hop": 2, "metric": 2}]
    }
    collector.record_event(ev1)

    # 2. Ingest Partition event
    ev2 = {
        "type": "partition",
        "link": [1, 2],
        "status": "SEVERED",
        "duration_s": 15.0
    }
    collector.record_event(ev2)

    # 3. Ingest Adaptive Comms regime change
    ev3 = {
        "type": "adaptive_comms",
        "rover_id": 1,
        "regime": "Congested",
        "loss_pct": 25.0
    }
    collector.record_event(ev3)

    # 4. Ingest Global Consistency Audit
    ev4 = {
        "type": "consistency_audit",
        "dsu_components": 1,
        "is_converged": True,
        "reciprocal_edge_fraction": 1.0
    }
    collector.record_event(ev4)

    # Check snapshot
    snap = collector.get_snapshot()
    assert "1" in snap["rovers"]
    assert snap["rovers"]["1"]["version_vector"] == {"1": 10, "2": 8}
    assert len(snap["partitions"]) >= 1
    assert snap["global_metrics"]["dsu_components"] == 1
    assert snap["global_metrics"]["is_converged"] is True
    assert snap["global_metrics"]["adaptive_regime"] == "Congested"

    # Check SSE queue fanout
    received_types = []
    while not sub_q.empty():
        msg = sub_q.get_nowait()
        received_types.append(msg.get("type"))

    assert "heartbeat" in received_types
    assert "partition" in received_types
    assert "adaptive_comms" in received_types
    assert "consistency_audit" in received_types

    collector.unsubscribe(sub_q)
    collector.stop()

    print(">>> All M7.12 Telemetry & Dashboard tests PASSED (100%).")

if __name__ == "__main__":
    test_telemetry_collector_m712()
