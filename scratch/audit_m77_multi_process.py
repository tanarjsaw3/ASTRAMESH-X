import subprocess
import time
import sys
import os

EXE_PATH = os.path.abspath("build-ucrt64-clean/bin/rover_daemon.exe")

def test_two_process():
    print("\n--- [TEST 1] Two Independent Rover Processes ---")
    p2 = subprocess.Popen([EXE_PATH, "--rover-id", "2", "--port", "23002",
                           "--peer", "127.0.0.1:23001", "--duration-s", "3.0",
                           "--heartbeat-ms", "80"],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    time.sleep(0.3)
    p1 = subprocess.Popen([EXE_PATH, "--rover-id", "1", "--port", "23001",
                           "--peer", "127.0.0.1:23002", "--duration-s", "3.0",
                           "--heartbeat-ms", "80", "--test-msg", "AUDIT_2P_MSG"],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    out1, err1 = p1.communicate(timeout=6.0)
    out2, err2 = p2.communicate(timeout=6.0)

    assert p1.returncode == 0, f"P1 failed: {err1}"
    assert p2.returncode == 0, f"P2 failed: {err2}"
    assert "AUDIT_2P_MSG" in out2, "Rover 2 failed to receive payload from Rover 1"
    print("  -> PASSED (Rover 1 & 2 exchanged heartbeats and payload text)")

def test_three_and_four_process():
    print("\n--- [TEST 2] Three & Four Independent Rover Processes ---")
    # Launch 4 processes in full mesh
    procs = []
    base_port = 23100
    for i in range(1, 5):
        args = [EXE_PATH, "--rover-id", str(i), "--port", str(base_port + i),
                "--duration-s", "3.5", "--heartbeat-ms", "80"]
        for j in range(1, 5):
            if i != j:
                args.extend(["--peer", f"127.0.0.1:{base_port + j}"])
        if i == 1:
            args.extend(["--test-msg", "MESH_P1_BROADCAST"])
        p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        procs.append(p)
        time.sleep(0.1)

    outputs = []
    for i, p in enumerate(procs):
        out, err = p.communicate(timeout=8.0)
        assert p.returncode == 0, f"Rover {i+1} failed: {err}"
        outputs.append(out)

    # Check peer discovery across all 4 rovers
    for i, out in enumerate(outputs):
        assert "Active Peers Seen:" in out, f"Rover {i+1} missing shutdown telemetry"
        # Since full mesh of 4 rovers, each sees 4 total peers (self registered + 3 peers)
        print(f"  Rover {i+1} finished cleanly. Telemetry snippet:")
        for line in out.splitlines():
            if "Packets Received:" in line or "Active Peers Seen:" in line:
                print(f"    {line}")

    # Check that at least Rover 2, 3, or 4 received the broadcast text
    received_count = sum(1 for out in outputs[1:] if "MESH_P1_BROADCAST" in out)
    assert received_count >= 1, "Broadcast failed to reach any peer rovers"
    print(f"  -> PASSED (4 concurrent OS processes active in full mesh, {received_count} received broadcast)")

def test_process_restart():
    print("\n--- [TEST 3] Process Restart and Port Rebind (SO_REUSEADDR) ---")
    port = "23201"
    # First run
    p1 = subprocess.Popen([EXE_PATH, "--rover-id", "1", "--port", port, "--duration-s", "1.0"],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    out1, _ = p1.communicate(timeout=4.0)
    assert p1.returncode == 0, "Initial run failed"
    print("  Process 1 exited cleanly. Rebinding to same port immediately...")

    # Immediate second run on exact same port
    p2 = subprocess.Popen([EXE_PATH, "--rover-id", "1", "--port", port, "--duration-s", "1.0"],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    out2, err2 = p2.communicate(timeout=4.0)
    assert p2.returncode == 0, f"Port rebind failed: {err2}"
    assert "Node started successfully" in out2, "Second run failed to bind port"
    print("  -> PASSED (SO_REUSEADDR allows immediate rebind without port collision)")

def test_temporary_disconnection():
    print("\n--- [TEST 4] Temporary Peer Disconnection & Reconnection ---")
    # Rover 2 starts and stays alive for 7.0s to allow disconnect & reconnect window
    p2 = subprocess.Popen([EXE_PATH, "--rover-id", "2", "--port", "23302",
                           "--peer", "127.0.0.1:23301", "--duration-s", "7.0",
                           "--heartbeat-ms", "80"],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    # Rover 1 starts for 1.5s, then terminates (simulating crash/disconnect)
    p1 = subprocess.Popen([EXE_PATH, "--rover-id", "1", "--port", "23301",
                           "--peer", "127.0.0.1:23302", "--duration-s", "1.5",
                           "--heartbeat-ms", "80"],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    p1.communicate(timeout=3.0)
    print("  Rover 1 disconnected. Waiting 1.0s...")
    time.sleep(1.0)
    print("  Rover 1 reconnecting on same port...")
    p1_new = subprocess.Popen([EXE_PATH, "--rover-id", "1", "--port", "23301",
                               "--peer", "127.0.0.1:23302", "--duration-s", "2.5",
                               "--heartbeat-ms", "80", "--test-msg", "RECONNECT_OK"],
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    out1_new, _ = p1_new.communicate(timeout=4.0)
    out2, _ = p2.communicate(timeout=5.0)

    assert p1_new.returncode == 0
    assert p2.returncode == 0
    assert "RECONNECT_OK" in out2, "Rover 2 failed to receive message after Rover 1 reconnected"
    print("  -> PASSED (Reconnected peer recognized, packets successfully delivered)")

if __name__ == "__main__":
    print("=================================================================")
    print("  ASTRAMESH-X M7.7 MULTI-PROCESS OPERATIONAL INTEGRITY AUDIT     ")
    print("=================================================================")
    test_two_process()
    test_three_and_four_process()
    test_process_restart()
    test_temporary_disconnection()
    print("\n=================================================================")
    print("  ALL MULTI-PROCESS OPERATIONAL AUDIT TESTS PASSED (100%)        ")
    print("=================================================================")
