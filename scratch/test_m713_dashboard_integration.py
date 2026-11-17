#!/usr/bin/env python3
import json
import os
import socket
import sys
import time

workspace_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if workspace_dir not in sys.path:
    sys.path.insert(0, workspace_dir)

from app.telemetry_collector import TelemetryCollector

def test_gauge_telemetry_streaming():
    print("[RUN] test_gauge_telemetry_streaming...")
    test_port = 29998
    collector = TelemetryCollector(port=test_port)
    collector.start()

    time.sleep(0.1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Send a gauge alignment event from Rover 2
    event = {
        "type": "gauge_alignment",
        "rover_id": 2,
        "anchor_id": 1,
        "transform_from_anchor": {"x": 15.2, "y": 8.1, "theta": 0.35},
        "cycles_remediated": 1,
        "global_ate_m": 0.042,
        "timestamp": time.time()
    }
    raw = json.dumps(event).encode("utf-8")
    sock.sendto(raw, ("127.0.0.1", test_port))

    time.sleep(0.3)
    snapshot = collector.get_snapshot()

    # Verify metrics updated
    gm = snapshot["global_metrics"]
    assert gm["elected_anchor"] == 1, f"Expected elected_anchor 1, got {gm.get('elected_anchor')}"
    assert gm["cycles_remediated"] == 1, f"Expected cycles_remediated 1, got {gm.get('cycles_remediated')}"
    assert abs(gm["global_ate_m"] - 0.042) < 1e-4, f"Expected global_ate_m 0.042, got {gm.get('global_ate_m')}"

    # Verify rover 2 state updated
    r2 = snapshot["rovers"].get("2")
    assert r2 is not None, "Rover 2 not found in snapshot"
    assert r2["anchor_id"] == 1
    assert r2["is_anchor"] is False
    assert abs(r2["transform_from_anchor"]["x"] - 15.2) < 1e-4

    collector.stop()
    sock.close()
    print("[PASS] test_gauge_telemetry_streaming")

if __name__ == "__main__":
    test_gauge_telemetry_streaming()
    print("All M7.13 Dashboard integration tests PASSED successfully.")

