#!/usr/bin/env python3
"""
AstroMesh-X — Multi-Process Distributed Topological SLAM Verification Test
Spawns two independent OS processes (Rover 1 and Rover 2) executing rover_daemon.exe
with --scenario-slam. Verifies inter-process IBLT reconciliation, Kabsch RANSAC
geometric verification, and DSU topological partition merging over UDP.
"""

import subprocess
import sys
import time

def main():
    print("=================================================================")
    print("  ASTRAMESH-X MULTI-PROCESS DISTRIBUTED SLAM INTEGRATION TEST   ")
    print("=================================================================")

    daemon_exe = r"build-ucrt64-clean\bin\rover_daemon.exe"

    port1 = 21701
    port2 = 21702
    duration_s = 4.0

    cmd1 = [
        daemon_exe,
        "--rover-id", "1",
        "--port", str(port1),
        "--peer", f"127.0.0.1:{port2}",
        "--duration-s", str(duration_s),
        "--scenario-slam",
        "--sync-target", "2"
    ]

    cmd2 = [
        daemon_exe,
        "--rover-id", "2",
        "--port", str(port2),
        "--peer", f"127.0.0.1:{port1}",
        "--duration-s", str(duration_s),
        "--scenario-slam"
    ]

    print(f"[SPAWN] Starting Rover 2 on port {port2} (passive listener)...")
    p2 = subprocess.Popen(cmd2, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print(f"  -> Rover 2 PID: {p2.pid}")

    time.sleep(0.5)

    print(f"[SPAWN] Starting Rover 1 on port {port1} (initiator)...")
    p1 = subprocess.Popen(cmd1, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print(f"  -> Rover 1 PID: {p1.pid}")

    print("[WAIT] Waiting for both processes to complete distributed SLAM...")
    out1, err1 = p1.communicate(timeout=10)
    out2, err2 = p2.communicate(timeout=10)

    print("\n--- Rover 1 Standard Output ---")
    print(out1)
    if err1:
        print("--- Rover 1 Errors ---", err1)

    print("\n--- Rover 2 Standard Output ---")
    print(out2)
    if err2:
        print("--- Rover 2 Errors ---", err2)

    # Verifications
    print("=================================================================")
    print("                    AUDITING VERIFICATIONS                       ")
    print("=================================================================")

    # 1. Check exit codes
    assert p1.returncode == 0, f"Rover 1 exited with code {p1.returncode}"
    assert p2.returncode == 0, f"Rover 2 exited with code {p2.returncode}"
    print("[PASS] Both processes exited cleanly with exit code 0.")

    # 2. Check sync packets
    assert "Sync Packets Sent:     1" in out1, "Rover 1 did not send sync packet"
    assert "Sync Packets Received: 1" in out2, "Rover 2 did not receive sync packet"
    print("[PASS] IBLT synchronization frame transmitted and received over UDP.")

    # 3. Check decode success
    assert "Decodes Succeeded:     1" in out2, "Rover 2 IBLT decode failed"
    print("[PASS] Rover 2 peeled IBLT symmetric difference successfully.")

    # 4. Check proposals
    assert "Proposals Sent:        1" in out2, "Rover 2 did not send proposal"
    assert "Proposals Received:    1" in out1, "Rover 1 did not receive proposal"
    print("[PASS] Candidate proposals transmitted and received over UDP.")

    # 5. Check loop closure verification
    assert "Loop Closures Verified:1" in out1, "Rover 1 failed loop closure verification"
    assert "Loop Closures Verified:1" in out2, "Rover 2 failed loop closure verification"
    print("[PASS] Kabsch RANSAC geometric verification passed on both independent processes.")

    # 6. Check DSU unions
    assert "DSU Unions Performed:  1" in out1, "Rover 1 DSU union failed"
    assert "DSU Unions Performed:  1" in out2, "Rover 2 DSU union failed"
    print("[PASS] Topological DSU partition merged on both independent processes.")

    # 7. Check DSU component count
    assert "DSU Components:        1" in out1, "Rover 1 DSU component count not 1"
    assert "DSU Components:        1" in out2, "Rover 2 DSU component count not 1"
    print("[PASS] Final DSU component count unified to 1 on both independent processes.")

    # 8. Check boundary state exchange
    assert "Boundary States Sent:  1" in out1, "Rover 1 did not send boundary state"
    assert "Boundary States Recv:  1" in out2, "Rover 2 did not receive boundary state"
    print("[PASS] BoundaryStatePacket exchanged across independent OS processes.")

    print("\n=================================================================")
    print("  ALL MULTI-PROCESS DISTRIBUTED SLAM AUDIT CHECKS PASSED (100%)  ")
    print("=================================================================")

if __name__ == "__main__":
    main()

