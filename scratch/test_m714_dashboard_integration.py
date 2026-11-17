#!/usr/bin/env python3
"""
Test script for Milestone 7.14 Dashboard & Telemetry Integration.
Sends simulated active_loop_closure telemetry datagrams and validates
the internal swarm state and metric aggregation in TelemetryCollector.
"""

import json
import os
import socket
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from app.telemetry_collector import TelemetryCollector

def test_m714_telemetry_integration():
    print("[RUN] test_m714_telemetry_integration...")
    collector = TelemetryCollector(port=29995)
    collector.start()
    time.sleep(0.2)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    event = {
        "type": "active_loop_closure",
        "rover_id": 1,
        "evaluated": 12,
        "scheduled": 4,
        "wire_bytes": 1408,
        "verified": 4,
        "global_ate_m": 0.042
    }

    payload = json.dumps(event).encode("utf-8")
    sock.sendto(payload, ("127.0.0.1", 29995))
    time.sleep(0.3)

    snap = collector.get_snapshot()
    gm = snap["global_metrics"]

    assert gm["active_opportunities_evaluated"] == 12, f"Expected 12, got {gm['active_opportunities_evaluated']}"
    assert gm["active_opportunities_scheduled"] == 4, f"Expected 4, got {gm['active_opportunities_scheduled']}"
    assert gm["active_wire_bytes_spent"] == 1408, f"Expected 1408, got {gm['active_wire_bytes_spent']}"
    assert gm["rendezvous_proposals_verified"] == 4, f"Expected 4, got {gm['rendezvous_proposals_verified']}"
    assert abs(gm["global_ate_m"] - 0.042) < 1e-4, f"Expected 0.042, got {gm['global_ate_m']}"

    collector.stop()
    sock.close()
    print("[PASS] test_m714_telemetry_integration: Active loop closure telemetry verified.")

if __name__ == "__main__":
    test_m714_telemetry_integration()
