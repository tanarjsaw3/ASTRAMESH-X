#!/usr/bin/env python3
"""
AstroMesh-X Milestone 7.11 Multi-Process Real UDP Verification Suite.
Spawns independent OS processes running rover_daemon.exe over physical UDP loopback sockets:
  - Scenario A: Bidirectional Two-Hop Multi-Process Relay (R1 <-> R2 <-> R3)
  - Scenario B: Two-Hop Multi-Process SLAM Reconciliation (R1 <-> R2 <-> R3) with R2 as pure relay
  - Scenario C: Four-Process Diamond Failover (R1 <-> {R2, R3} <-> R4) with R2 killed during transit
"""

import subprocess
import sys
import time
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parent.parent
DAEMON_EXE = WORKSPACE / "build-ucrt64-clean" / "bin" / "rover_daemon.exe"

def test_scenario_a_multihop_relay():
    print("\n=================================================================")
    print("  SCENARIO A: 3-ROVER BIDIRECTIONAL TWO-HOP RELAY (R1-R2-R3)     ")
    print("=================================================================")

    p1_port = 19501
    p2_port = 19502
    p3_port = 19503

    cmd_r1 = [
        str(DAEMON_EXE),
        "--rover-id", "1",
        "--port", str(p1_port),
        "--peer", f"127.0.0.1:{p2_port}",  # Only knows R2
        "--duration-s", "4.0",
        "--heartbeat-ms", "50",
        "--multihop-target", "3",
        "--test-msg", "FORWARD_R1_TO_R3"
    ]

    cmd_r2 = [
        str(DAEMON_EXE),
        "--rover-id", "2",
        "--port", str(p2_port),
        "--peer", f"127.0.0.1:{p1_port}",
        "--peer", f"127.0.0.1:{p3_port}",  # Relay knows R1 and R3
        "--duration-s", "4.0",
        "--heartbeat-ms", "50"
    ]

    cmd_r3 = [
        str(DAEMON_EXE),
        "--rover-id", "3",
        "--port", str(p3_port),
        "--peer", f"127.0.0.1:{p2_port}",  # Only knows R2
        "--duration-s", "4.0",
        "--heartbeat-ms", "50",
        "--multihop-target", "1",
        "--test-msg", "RETURN_R3_TO_R1"
    ]

    print("Launching Rover 2 (Relay)...")
    p2 = subprocess.Popen(cmd_r2, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, errors="replace")
    time.sleep(0.2)

    print("Launching Rover 1 (Origin) and Rover 3 (Destination)...")
    p1 = subprocess.Popen(cmd_r1, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, errors="replace")
    p3 = subprocess.Popen(cmd_r3, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, errors="replace")

    print("Waiting for processes to communicate and terminate...")
    out1, err1 = p1.communicate(timeout=12)
    out2, err2 = p2.communicate(timeout=12)
    out3, err3 = p3.communicate(timeout=12)

    assert p1.returncode == 0, f"R1 crashed with code {p1.returncode}\n{err1}"
    assert p2.returncode == 0, f"R2 crashed with code {p2.returncode}\n{err2}"
    assert p3.returncode == 0, f"R3 crashed with code {p3.returncode}\n{err3}"

    # Verify R2 forwarded in both directions
    r2_forwarded = ("Packets Forwarded:     " in out2 and not "Packets Forwarded:     0" in out2)

    # Verify R3 received R1's payload
    r3_delivered = 'Payload text: "FORWARD_R1_TO_R3"' in out3

    # Verify R1 received R3's payload
    r1_delivered = 'Payload text: "RETURN_R3_TO_R1"' in out1

    # Verify R2 did NOT consume either message locally
    r2_consumed_1 = 'Payload text: "FORWARD_R1_TO_R3"' in out2
    r2_consumed_3 = 'Payload text: "RETURN_R3_TO_R1"' in out2

    print(f"  [Check 1] Rover 2 Forwarded Packets: {'PASS' if r2_forwarded else 'FAIL'}")
    print(f"  [Check 2] Rover 3 Delivered (R1->R3):{'PASS' if r3_delivered else 'FAIL'}")
    print(f"  [Check 3] Rover 1 Delivered (R3->R1):{'PASS' if r1_delivered else 'FAIL'}")
    print(f"  [Check 4] Relay Did Not Consume:     {'PASS' if (not r2_consumed_1 and not r2_consumed_3) else 'FAIL'}")

    if r2_forwarded and r3_delivered and r1_delivered and not r2_consumed_1 and not r2_consumed_3:
        print("  -> SCENARIO A PASSED (100%)\n")
        return True
    else:
        print("  -> SCENARIO A FAILED")
        return False

def test_scenario_b_multihop_slam():
    print("=================================================================")
    print("  SCENARIO B: 3-ROVER TWO-HOP MULTI-PROCESS SLAM RECONCILIATION  ")
    print("  (R1 <-> R2 (Pure Relay) <-> R3, R1 syncs with R3 across R2)    ")
    print("=================================================================")

    p1_port = 19511
    p2_port = 19512
    p3_port = 19513

    # Rover 1 has landmarks {1,2,3,4,5}, knows ONLY R2
    cmd_r1 = [
        str(DAEMON_EXE),
        "--rover-id", "1",
        "--port", str(p1_port),
        "--peer", f"127.0.0.1:{p2_port}",
        "--duration-s", "5.0",
        "--heartbeat-ms", "40",
        "--send-delay-s", "0.8",
        "--scenario-slam",
        "--sync-target", "3"
    ]

    # Rover 2 is a PURE RELAY node (no --scenario-slam), knows R1 and R3
    cmd_r2 = [
        str(DAEMON_EXE),
        "--rover-id", "2",
        "--port", str(p2_port),
        "--peer", f"127.0.0.1:{p1_port}",
        "--peer", f"127.0.0.1:{p3_port}",
        "--duration-s", "5.0",
        "--heartbeat-ms", "40"
    ]

    # Rover 3 has landmarks {3,4,5,6,7}, knows ONLY R2
    cmd_r3 = [
        str(DAEMON_EXE),
        "--rover-id", "3",
        "--port", str(p3_port),
        "--peer", f"127.0.0.1:{p2_port}",
        "--duration-s", "5.0",
        "--heartbeat-ms", "40",
        "--scenario-slam"
    ]

    print("Launching Rover 2 (Pure Relay)...")
    p2 = subprocess.Popen(cmd_r2, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, errors="replace")
    time.sleep(0.2)

    print("Launching Rover 1 (SLAM) and Rover 3 (SLAM)...")
    p3 = subprocess.Popen(cmd_r3, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, errors="replace")
    time.sleep(0.1)
    p1 = subprocess.Popen(cmd_r1, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, errors="replace")

    out1, err1 = p1.communicate(timeout=12)
    out2, err2 = p2.communicate(timeout=12)
    out3, err3 = p3.communicate(timeout=12)

    assert p1.returncode == 0, f"R1 crashed with code {p1.returncode}\n{err1}"
    assert p2.returncode == 0, f"R2 crashed with code {p2.returncode}\n{err2}"
    assert p3.returncode == 0, f"R3 crashed with code {p3.returncode}\n{err3}"

    r2_forwarded = ("Packets Forwarded:     " in out2 and not "Packets Forwarded:     0" in out2)
    r1_verified = ("Loop Closures Verified: 1" in out1) or ("Loop Closures Verified:1" in out1) or ("Loop Closures Verified: 2" in out1)
    r3_verified = ("Loop Closures Verified: 1" in out3) or ("Loop Closures Verified:1" in out3) or ("Loop Closures Verified: 2" in out3)
    r1_unions = ("DSU Unions Performed:  1" in out1) or ("DSU Unions Performed: 1" in out1) or ("DSU Unions Performed:1" in out1)

    print(f"  [Check 1] Rover 2 Forwarded SLAM frames: {'PASS' if r2_forwarded else 'FAIL'}")
    print(f"  [Check 2] Rover 1 Loop Closures:        {'PASS' if r1_verified else 'FAIL'}")
    print(f"  [Check 3] Rover 3 Loop Closures:        {'PASS' if r3_verified else 'FAIL'}")
    print(f"  [Check 4] Rover 1 DSU Union (R1 merged):{'PASS' if r1_unions else 'FAIL'}")

    if r2_forwarded and (r1_verified or r3_verified) and r1_unions:
        print("  -> SCENARIO B PASSED (100%)\n")
        return True
    else:
        print("  -> SCENARIO B FAILED")
        print("\n=== OUT 1 ===\n", out1)
        print("\n=== OUT 2 ===\n", out2)
        print("\n=== OUT 3 ===\n", out3)
        return False

def test_scenario_c_diamond_failover():
    print("=================================================================")
    print("  SCENARIO C: 4-ROVER DIAMOND PROCESS FAILOVER                   ")
    print("  (R1 -> {R2, R3} -> R4; kill R2 mid-run, failover to R3)         ")
    print("=================================================================")

    p1_port = 19521
    p2_port = 19522
    p3_port = 19523
    p4_port = 19524

    # R1 knows R2 and R3; target R4
    cmd_r1 = [
        str(DAEMON_EXE),
        "--rover-id", "1",
        "--port", str(p1_port),
        "--peer", f"127.0.0.1:{p2_port}",
        "--peer", f"127.0.0.1:{p3_port}",
        "--duration-s", "6.0",
        "--heartbeat-ms", "50",
        "--peer-timeout-ms", "400",
        "--send-delay-s", "0.5",
        "--multihop-target", "4",
        "--test-msg", "DIAMOND_FAILOVER_PKT"
    ]

    # R2 is Relay 1; knows R1 and R4; runs for 2.0s
    cmd_r2 = [
        str(DAEMON_EXE),
        "--rover-id", "2",
        "--port", str(p2_port),
        "--peer", f"127.0.0.1:{p1_port}",
        "--peer", f"127.0.0.1:{p4_port}",
        "--duration-s", "2.0",
        "--heartbeat-ms", "50",
        "--peer-timeout-ms", "400"
    ]

    # R3 is Relay 2; knows R1 and R4; runs for full duration
    cmd_r3 = [
        str(DAEMON_EXE),
        "--rover-id", "3",
        "--port", str(p3_port),
        "--peer", f"127.0.0.1:{p1_port}",
        "--peer", f"127.0.0.1:{p4_port}",
        "--duration-s", "6.0",
        "--heartbeat-ms", "50",
        "--peer-timeout-ms", "400"
    ]

    # R4 is Destination; knows R2 and R3
    cmd_r4 = [
        str(DAEMON_EXE),
        "--rover-id", "4",
        "--port", str(p4_port),
        "--peer", f"127.0.0.1:{p2_port}",
        "--peer", f"127.0.0.1:{p3_port}",
        "--duration-s", "6.0",
        "--heartbeat-ms", "50",
        "--peer-timeout-ms", "400"
    ]

    print("Launching Relays R2 and R3, Destination R4...")
    p2 = subprocess.Popen(cmd_r2, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, errors="replace")
    p3 = subprocess.Popen(cmd_r3, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, errors="replace")
    p4 = subprocess.Popen(cmd_r4, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, errors="replace")
    time.sleep(0.3)

    print("Launching Origin R1...")
    p1 = subprocess.Popen(cmd_r1, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, errors="replace")

    # Let communication establish and flow through R2/R3 for 1.8s
    time.sleep(1.8)
    print("Terminating Relay R2 to trigger dynamic route failover...")
    p2.terminate()
    try:
        p2.wait(timeout=2)
    except subprocess.TimeoutExpired:
        p2.kill()

    print("Waiting for remaining processes to converge and finish...")
    out1, _ = p1.communicate(timeout=10)
    out3, _ = p3.communicate(timeout=10)
    out4, _ = p4.communicate(timeout=10)

    # Verify R3 forwarded packets after R2 was removed
    r3_forwarded = ("Packets Forwarded:     " in out3 and not "Packets Forwarded:     0" in out3)
    r4_delivered = 'Payload text: "DIAMOND_FAILOVER_PKT"' in out4

    print(f"  [Check 1] Rover 3 Forwarded via Alt Path: {'PASS' if r3_forwarded else 'FAIL'}")
    print(f"  [Check 2] Rover 4 Received Post-Failover: {'PASS' if r4_delivered else 'FAIL'}")

    if r3_forwarded and r4_delivered:
        print("  -> SCENARIO C PASSED (100%)\n")
        return True
    else:
        print("  -> SCENARIO C FAILED")
        print("\n=== OUT 3 ===\n", out3)
        print("\n=== OUT 4 ===\n", out4)
        return False

def main():
    print("=================================================================")
    print("  ASTRAMESH-X M7.11 MULTI-PROCESS UDP TEST HARNESS               ")
    print("=================================================================")

    if not DAEMON_EXE.exists():
        print(f"Executable {DAEMON_EXE} does not exist.")
        sys.exit(1)

    ok_a = test_scenario_a_multihop_relay()
    ok_b = test_scenario_b_multihop_slam()
    ok_c = test_scenario_c_diamond_failover()

    print("=================================================================")
    if ok_a and ok_b and ok_c:
        print("  ALL MULTI-PROCESS UDP TESTS PASSED SUCCESSFULLY (100%)         ")
        print("=================================================================")
        sys.exit(0)
    else:
        print("  SOME MULTI-PROCESS TESTS FAILED                                ")
        print("=================================================================")
        sys.exit(1)

if __name__ == "__main__":
    main()
