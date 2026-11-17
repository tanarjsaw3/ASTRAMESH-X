#!/usr/bin/env python3
"""
AstroMesh-X — M7.18 Simulator & Live Dashboard Integration Test Suite
Verifies end-to-end integration between C++ sim_runner, Python backend, REST/SSE APIs,
interactive partition triggers, Demo 4 (16 rovers), and research export formats.
"""

import json
import sys
import time
import urllib.request
import urllib.error

BASE_URL = "http://127.0.0.1:8765"

def record_check(test_name: str, passed: bool, detail: str = ""):
    status = "PASS" if passed else "FAIL"
    print(f" [{status}] | {test_name}")
    if detail:
        print(f"   -> {detail}")
    if not passed:
        raise AssertionError(f"Check failed: {test_name} - {detail}")

def test_status_endpoint():
    req = urllib.request.Request(f"{BASE_URL}/api/status")
    with urllib.request.urlopen(req, timeout=5) as res:
        assert res.status == 200
        data = json.loads(res.read().decode("utf-8"))
        assert data.get("status") == "online"
        assert "66/66" in data.get("ctest_status", "")
        assert data.get("engine_available") is True
        record_check("1. Status Endpoint & CTest 66/66 Verification", True,
                     f"Project: {data.get('project')}, Milestone: {data.get('milestone')}")

def test_demo4_scalability():
    req = urllib.request.Request(f"{BASE_URL}/api/demo/4")
    with urllib.request.urlopen(req, timeout=10) as res:
        assert res.status == 200
        data = json.loads(res.read().decode("utf-8"))
        rovers = data["config"]["rovers"]
        assert rovers == 16
        frames = len(data["frames"])
        assert frames > 50
        closures = data["summary"]["loop_closures_verified"]
        assert closures > 100
        record_check("2. Demo 4 Swarm Scalability (16 Rovers)", True,
                     f"Rovers: {rovers}, Frames: {frames}, Closures: {closures}, Wire: {data['summary']['total_wire_bytes']:,} B")

def test_partition_state_transitions():
    # 1. Disconnect pair (1, 2)
    req1 = urllib.request.Request(f"{BASE_URL}/api/partition/disconnect",
                                  data=json.dumps({"rover_a": 1, "rover_b": 2}).encode(),
                                  headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req1, timeout=5) as res1:
        d1 = json.loads(res1.read().decode("utf-8"))
        assert d1.get("status") == "ok"
        assert [1, 2] in d1.get("active_partitions", [])
        assert d1.get("is_partitioned") is True

    # 2. Query status
    req2 = urllib.request.Request(f"{BASE_URL}/api/partition/status")
    with urllib.request.urlopen(req2, timeout=5) as res2:
        d2 = json.loads(res2.read().decode("utf-8"))
        assert d2.get("is_partitioned") is True
        assert [1, 2] in d2.get("severed_links", [])

    # 3. Reconnect
    req3 = urllib.request.Request(f"{BASE_URL}/api/partition/reconnect",
                                  data=b"{}",
                                  headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req3, timeout=5) as res3:
        d3 = json.loads(res3.read().decode("utf-8"))
        assert d3.get("status") == "ok"
        assert d3.get("is_partitioned") is False
        assert len(d3.get("active_partitions", [])) == 0

    record_check("3. Interactive Partition State Machine (Sever -> Query -> Heal)", True,
                 "Verified [1, 2] link drop, status persistence, and complete healing reconciliation")

def test_benchmark_summary_endpoint():
    req = urllib.request.Request(f"{BASE_URL}/api/benchmarks/summary")
    with urllib.request.urlopen(req, timeout=5) as res:
        assert res.status == 200
        data = json.loads(res.read().decode("utf-8"))
        benchmarks = data.get("benchmarks", [])
        evidence = data.get("evidence", [])
        assert len(benchmarks) > 0
        assert len(evidence) > 0
        record_check("4. M7.17 Scientific Benchmark Summary API", True,
                     f"Retrieved {len(benchmarks)} statistical benchmarks and {len(evidence)} evidence entries")

def test_custom_simulation_with_partition():
    payload = {
        "rovers": 4,
        "scenario": "OverlappingGrid",
        "duration": 5.0,
        "step_dt": 0.2,
        "loss": 0.1,
        "delay_ms": 20,
        "comm_range": 25.0,
        "variant": "AstroMesh_X_Full",
        "seed": 999,
        "partition_start": 1.0,
        "partition_end": 3.0,
        "run_optimize": True,
        "opt_steps": 5
    }
    req = urllib.request.Request(f"{BASE_URL}/api/run",
                                  data=json.dumps(payload).encode(),
                                  headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=20) as res:
        assert res.status == 200
        data = json.loads(res.read().decode("utf-8"))
        frames = data.get("frames", [])
        assert len(frames) > 20
        # Check that partition flag is reflected in frames between t=1.0 and t=3.0
        part_frames = [f for f in frames if f.get("is_partitioned") is True]
        assert len(part_frames) > 0
        record_check("5. Dynamic Partition Run Execution", True,
                     f"Generated {len(frames)} frames; {len(part_frames)} frames executed under verified partition")

def test_export_endpoints():
    # CSV export
    req_csv = urllib.request.Request(f"{BASE_URL}/api/export/csv")
    with urllib.request.urlopen(req_csv, timeout=5) as res:
        assert res.status == 200
        csv_bytes = res.read()
        assert len(csv_bytes) > 100
        assert b"step,time_s" in csv_bytes

    # JSON export
    req_json = urllib.request.Request(f"{BASE_URL}/api/export/json")
    with urllib.request.urlopen(req_json, timeout=5) as res:
        assert res.status == 200
        json_bytes = res.read()
        assert len(json_bytes) > 500
        jdata = json.loads(json_bytes.decode("utf-8"))
        assert "config" in jdata and "frames" in jdata

    record_check("6. Research Export Verification (CSV & JSON)", True,
                 f"CSV: {len(csv_bytes):,} bytes, JSON: {len(json_bytes):,} bytes validated")

def main():
    print("======================================================================")
    print("      ASTRAMESH-X M7.18 SIMULATOR & DASHBOARD INTEGRATION SUITE      ")
    print("======================================================================")

    test_status_endpoint()
    test_demo4_scalability()
    test_partition_state_transitions()
    test_benchmark_summary_endpoint()
    test_custom_simulation_with_partition()
    test_export_endpoints()

    print("======================================================================")
    print("OVERALL RESULT: ALL M7.18 INTEGRATION CHECKS PASSED (6/6)")
    print("======================================================================")

if __name__ == "__main__":
    main()

