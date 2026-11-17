#!/usr/bin/env python3
"""
Test Communication Between Two Independent Standalone Rover Processes.
Spawns Rover 1 (port 19401) and Rover 2 (port 19402) as concurrent OS processes.
"""

import subprocess
import sys
import time
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parent.parent
DAEMON_EXE = WORKSPACE / "build-ucrt64-clean" / "bin" / "rover_daemon.exe"

def main():
    print("=================================================================")
    print("      TESTING TWO INDEPENDENT STANDALONE ROVER OS PROCESSES      ")
    print("=================================================================")

    if not DAEMON_EXE.exists():
        print(f"Error: {DAEMON_EXE} not found.")
        sys.exit(1)

    cmd_r1 = [
        str(DAEMON_EXE),
        "--rover-id", "1",
        "--port", "19401",
        "--peer", "127.0.0.1:19402",
        "--duration-s", "3.0",
        "--heartbeat-ms", "80",
        "--test-msg", "Hello from Rover 1"
    ]

    cmd_r2 = [
        str(DAEMON_EXE),
        "--rover-id", "2",
        "--port", "19402",
        "--peer", "127.0.0.1:19401",
        "--duration-s", "3.0",
        "--heartbeat-ms", "80",
        "--test-msg", "Hello from Rover 2"
    ]

    print("Launching Rover 1 process (PID)...")
    p1 = subprocess.Popen(cmd_r1, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print(f"  -> Rover 1 PID: {p1.pid}")

    print("Launching Rover 2 process (PID)...")
    p2 = subprocess.Popen(cmd_r2, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print(f"  -> Rover 2 PID: {p2.pid}")

    print("Waiting for both independent rover processes to communicate and exit...")
    stdout1, stderr1 = p1.communicate(timeout=10)
    stdout2, stderr2 = p2.communicate(timeout=10)

    print(f"\n--- ROVER 1 EXIT CODE: {p1.returncode} ---")
    print(stdout1)
    if stderr1:
        print("STDERR 1:", stderr1)

    print(f"\n--- ROVER 2 EXIT CODE: {p2.returncode} ---")
    print(stdout2)
    if stderr2:
        print("STDERR 2:", stderr2)

    # Verification checks
    assert p1.returncode == 0, f"Rover 1 exited with non-zero code {p1.returncode}"
    assert p2.returncode == 0, f"Rover 2 exited with non-zero code {p2.returncode}"

    # Check mutual reception
    r1_saw_r2 = "RECV Heartbeat from Rover 2" in stdout1 or "from Rover 2" in stdout1
    r2_saw_r1 = "RECV Heartbeat from Rover 1" in stdout2 or "from Rover 1" in stdout2

    r1_got_payload = 'Payload text: "Hello from Rover 2"' in stdout1
    r2_got_payload = 'Payload text: "Hello from Rover 1"' in stdout2

    print("=================================================================")
    print("                     VERIFICATION RESULTS                        ")
    print("=================================================================")
    print(f" Rover 1 discovered Rover 2:   {'PASS' if r1_saw_r2 else 'FAIL'}")
    print(f" Rover 2 discovered Rover 1:   {'PASS' if r2_saw_r1 else 'FAIL'}")
    print(f" Rover 1 got Rover 2 payload:  {'PASS' if r1_got_payload else 'FAIL'}")
    print(f" Rover 2 got Rover 1 payload:  {'PASS' if r2_got_payload else 'FAIL'}")
    print("=================================================================")

    if r1_saw_r2 and r2_saw_r1 and r1_got_payload and r2_got_payload:
        print("SUCCESS: Full bidirectional communication verified between 2 independent OS processes!")
        sys.exit(0)
    else:
        print("FAILED: Did not observe expected inter-process messages.")
        sys.exit(1)

if __name__ == "__main__":
    main()

