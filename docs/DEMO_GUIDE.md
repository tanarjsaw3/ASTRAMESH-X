# AstroMesh-X — Demonstration Guide & Operator Manual

**Date:** 2026-10-31  
**Project:** AstroMesh-X Decentralized Collaborative Topological SLAM  
**Version:** 0.1.0-RC1  
**Target Environment:** Windows 10/11, PowerShell, MSYS2 UCRT64 GCC 16.2.0, Python 3  

---

## 1. Quick Start Instructions

Launch the interactive Command-Center UI with a single command from the project root:

```powershell
cd 'C:\Users\TANISHKA SAWANT\ASTRAMESH-X'
python app/server.py
```

Then open your browser to:
[http://127.0.0.1:8080](http://127.0.0.1:8080)

Alternatively, run the standalone C++ simulation engine directly from the command line:

```powershell
.\build-ucrt64-clean\bin\sim_runner.exe --help
```

---

## 2. Three Reproducible Demonstration Scenarios

AstroMesh-X provides three pre-configured, deterministic demonstration scenarios accessible via the UI or the CLI:

### Demonstration 1: Basic Multi-Rover SLAM (2 Rovers)
- **Objective:** Demonstrate nominal collaborative mapping, IBLT set reconciliation, closed-form Kabsch/Umeyama RANSAC geometric verification, and continuous trajectory tracking under an ideal communication channel.
- **Parameters:**
  - Rovers: 2
  - Trajectory Geometry: Concentric Loops ($r_1 = 15\,\text{m}, r_2 = 25\,\text{m}$)
  - Channel Conditions: Loss = $0\%$, Delay = $0\,\text{ms}$
  - Comm Range: $30\,\text{m}$ (continuous line-of-sight)
  - Algorithm Variant: `AstroMesh_X_Full`
  - Seed: `42`
- **CLI Launch Command:**
  ```powershell
  .\build-ucrt64-clean\bin\sim_runner.exe --demo 1 --output-json scratch/demo1.json --output-csv scratch/demo1.csv
  ```
- **Observed Behavior:**
  - 81 simulation timesteps execute in $<100\,\text{ms}$.
  - $100\%$ application completion ratio (81/81 logical messages delivered with 0 drops).
  - Graph optimization pass converges successfully via Continuous Riemannian Dynamics (CORD).

---

### Demonstration 2: Communication Failure & DTN Recovery (4 Rovers)
- **Objective:** Demonstrate network resilience under severe packet degradation ($50\%$ channel loss) and high link latency ($100\,\text{ms}$ one-way delay), verifying exponential backoff DTN retransmissions and topological graph merging.
- **Parameters:**
  - Rovers: 4
  - Trajectory Geometry: Overlapping Grid (intersecting search lanes)
  - Channel Conditions: Loss = $50\%$, Delay = $100\,\text{ms}$ ($RTT = 200\,\text{ms}$)
  - Comm Range: $25\,\text{m}$
  - Algorithm Variant: `AstroMesh_X_Full`
  - Seed: `1042`
- **CLI Launch Command:**
  ```powershell
  .\build-ucrt64-clean\bin\sim_runner.exe --demo 2 --output-json scratch/demo2.json --output-csv scratch/demo2.csv
  ```
- **Observed Behavior:**
  - $104$ verified loop closures pass geometric RANSAC gating; $62$ perceptual aliasing candidates rejected.
  - DTN store-and-forward executes $1,353$ retransmissions, sustaining a **$95.71\%$ application delivery ratio** despite dropping half of all physical transmissions.
  - Disjoint Set Union (DSU) partitions successfully merge from 4 initial components down to 2 unified topological clusters.

---

### Demonstration 3: Multi-Rover Swarm with Multi-Hop Mesh (8 Rovers)
- **Objective:** Demonstrate scalability to an 8-rover collective, evaluating Dijkstra multi-hop packet relaying across non-adjacent rovers and decentralized coordination.
- **Parameters:**
  - Rovers: 8
  - Trajectory Geometry: Concentric Loops (8 concentric orbits)
  - Channel Conditions: Loss = $20\%$, Delay = $50\,\text{ms}$
  - Comm Range: $22\,\text{m}$
  - Algorithm Variant: `AstroMesh_X_Full`
  - Seed: `2084`
- **CLI Launch Command:**
  ```powershell
  .\build-ucrt64-clean\bin\sim_runner.exe --demo 3 --output-json scratch/demo3.json --output-csv scratch/demo3.csv
  ```
- **Observed Behavior:**
  - 8 rovers coordinate across 28 pairwise potential communication links.
  - Multi-hop forwarding successfully delivers $1,198$ messages ($98.84\%$ completion ratio) with $2,049$ retransmissions.
  - Total transmitted wire bytes: $5,420,400\,\text{B}$.
  - 21 inter-rover loop closures verified.

---

## 3. Interactive Web Command-Center Features

The web interface at `http://127.0.0.1:8080` contains five dedicated operational sections:

1. **Live Command Center:**
   - **Simulation Canvas:** 2D interactive canvas rendering the $80\,\text{m} \times 80\,\text{m}$ arena, static landmarks, rover position triangles with heading indicators, continuous trajectory histories, communication range circles, and dynamic communication links (green = active; red dashed = degraded).
   - **Playback Bar:** Play/Pause button, step forward/back, timeline scrubber, and speed selector ($0.5\times, 1.0\times, 2.0\times, 4.0\times$).
   - **Rover Details Panel:** Live $(x, y, \theta)$ pose telemetry, visible landmark count, active connected peers, and DSU partition membership.
2. **Network & DTN Telemetry:**
   - Real-time wire byte breakdown: Application Payload, Protocol Header (24 B), Transport Framing (20 B), and ACK Wire Traffic (44 B).
   - Packet counters: Sent, Delivered, Dropped, Retransmitted, Completion Ratio.
   - Cumulative Wire Bytes vs. Time chart.
3. **Performance Analytics:**
   - DSU Connected Components vs. Time chart demonstrating dynamic cluster merging.
   - Loop closure verification quality indicators (True Accepts vs. Rejected Aliases).
   - Absolute Trajectory Error (ATE RMSE) tracking.
4. **Scientific Benchmark Comparison:**
   - Direct interactive comparison across all 7 architectural variants based on real empirical records (`benchmark_phase61_trials.csv` and `benchmark_phase62_trials.csv`).
   - Clear disclosure of the Dual Dataset Scoping Protocol (SC1 complete $N=30$ study vs. SC10 exploratory $27.8\%$ study).
5. **Mission Reports & Export:**
   - One-click buttons to download simulation summary CSV, detailed telemetry JSON, or milestone markdown reports.

---

## 4. Troubleshooting & Verification

To verify full system integrity:
```powershell
# Run the 21 regression unit tests
ctest --test-dir build-ucrt64-clean --output-on-failure

# Test the backend HTTP API server
python scratch/test_api.py
```
Both commands must complete with exit code 0 and 100% assertions passing.

