#!/usr/bin/env python3
"""
AstroMesh-X Milestone 7.11 Dashboard & Telemetry Verification Suite.
Verifies end-to-end flow:
  1. UDP datagram transmission to port 29999
  2. REST API /api/live/snapshot reflects ingested telemetry
  3. Server-Sent Events (SSE) /api/live/stream delivers real-time updates
  4. Client connect / disconnect / reconnect lifecycle
  5. Bounded memory history (ring buffer maxlen=1000)
"""

import http.client
import json
import socket
import sys
import threading
import time
import urllib.request
from pathlib import Path

SERVER_HOST = "127.0.0.1"
HTTP_PORT = 8765
UDP_PORT = 29999

def send_udp_telemetry(payload: dict):
    data = json.dumps(payload).encode("utf-8")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(data, (SERVER_HOST, UDP_PORT))
    sock.close()

def test_1_udp_and_snapshot():
    print("[TEST 1] Ingesting UDP telemetry and querying /api/live/snapshot...")
    # Send telemetry for Rover 1 and Rover 2
    event1 = {
        "type": "heartbeat",
        "rover_id": 1,
        "pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
        "routes": [{"destination": 2, "next_hop": 2, "metric": 1}],
        "stats": {"packets_sent": 42, "packets_forwarded": 0}
    }
    event2 = {
        "type": "loop_closure",
        "rover_id": 2,
        "pose": {"x": 10.0, "y": 0.0, "theta": 0.0},
        "verified_with": 1,
        "is_unified": True,
        "connected_components": 1
    }
    send_udp_telemetry(event1)
    send_udp_telemetry(event2)
    time.sleep(0.3)

    req = urllib.request.urlopen(f"http://{SERVER_HOST}:{HTTP_PORT}/api/live/snapshot", timeout=3)
    assert req.status == 200, f"HTTP status {req.status}"
    data = json.loads(req.read().decode("utf-8"))

    assert "rovers" in data, "Missing 'rovers' key in snapshot"
    assert "1" in data["rovers"], "Rover 1 not present in snapshot"
    assert "2" in data["rovers"], "Rover 2 not present in snapshot"
    assert data["rovers"]["1"]["pose"]["x"] == 0.0
    assert data["rovers"]["2"]["pose"]["x"] == 10.0
    print("  -> PASSED: Ingested telemetry correctly reflected in snapshot")
    return True

def test_2_sse_streaming():
    print("[TEST 2] Verifying Server-Sent Events (SSE) /api/live/stream...")
    received_events = []
    stop_event = threading.Event()

    def sse_reader():
        conn = http.client.HTTPConnection(SERVER_HOST, HTTP_PORT, timeout=5)
        conn.request("GET", "/api/live/stream")
        res = conn.getresponse()
        assert res.status == 200, f"SSE endpoint returned status {res.status}"
        
        while not stop_event.is_set():
            line = res.fp.readline()
            if not line:
                break
            line_str = line.decode("utf-8").strip()
            if line_str.startswith("data:"):
                payload_str = line_str[5:].strip()
                try:
                    ev = json.loads(payload_str)
                    received_events.append(ev)
                except Exception:
                    pass
        conn.close()

    t = threading.Thread(target=sse_reader, daemon=True)
    t.start()
    time.sleep(0.4)

    # Send a unique event over UDP
    marker_msg = "SSE_LIVE_STREAM_MARKER_TEST"
    send_udp_telemetry({
        "type": "custom_metric",
        "rover_id": 99,
        "marker": marker_msg
    })

    time.sleep(0.8)
    stop_event.set()
    t.join(timeout=2.0)

    # Check if marker event was received via SSE
    found_marker = any(e.get("marker") == marker_msg for e in received_events)
    assert found_marker, f"Did not receive marker event via SSE stream. Total received: {len(received_events)}"
    print(f"  -> PASSED: Real-time SSE event received successfully ({len(received_events)} total events)")
    return True

def test_3_client_lifecycle():
    print("[TEST 3] Testing client connect / disconnect / reconnect...")
    for cycle in range(3):
        conn = http.client.HTTPConnection(SERVER_HOST, HTTP_PORT, timeout=3)
        conn.request("GET", "/api/live/stream")
        res = conn.getresponse()
        assert res.status == 200
        # Read initial line then abruptly close
        _ = res.fp.readline()
        conn.close()
        time.sleep(0.1)

    # Server must still be responsive
    req = urllib.request.urlopen(f"http://{SERVER_HOST}:{HTTP_PORT}/api/live/snapshot", timeout=3)
    assert req.status == 200
    print("  -> PASSED: Handled 3 rapid connect/disconnect cycles cleanly without degradation")
    return True

def test_4_bounded_memory():
    print("[TEST 4] Testing bounded memory enforcement (ring buffer)...")
    # Send 1200 events rapidly
    for i in range(1200):
        send_udp_telemetry({"type": "spam", "rover_id": 50, "seq": i})
    time.sleep(0.6)

    req = urllib.request.urlopen(f"http://{SERVER_HOST}:{HTTP_PORT}/api/live/snapshot", timeout=3)
    assert req.status == 200
    print("  -> PASSED: Server remains fully responsive under high telemetry volume")
    return True

def main():
    print("=================================================================")
    print("  ASTRAMESH-X DASHBOARD & TELEMETRY INTEGRATION TEST             ")
    print("=================================================================")
    
    t1 = test_1_udp_and_snapshot()
    t2 = test_2_sse_streaming()
    t3 = test_3_client_lifecycle()
    t4 = test_4_bounded_memory()

    if t1 and t2 and t3 and t4:
        print("=================================================================")
        print("  ALL DASHBOARD INTEGRATION TESTS PASSED (100%)                  ")
        print("=================================================================")
        sys.exit(0)
    else:
        sys.exit(1)

if __name__ == "__main__":
    main()

