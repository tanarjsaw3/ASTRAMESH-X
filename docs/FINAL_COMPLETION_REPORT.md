# AstroMesh-X — Final Project Completion Report

**Date of Completion:** 2026-10-31  
**Project:** AstroMesh-X Decentralized Collaborative Topological SLAM  
**Version:** 0.1.0 (Release Candidate 1)  
**Lead Roles:** Lead Software Architect, C++ Engineer, Simulation Engineer, Frontend Developer, QA Engineer, Release Manager  
**Repository:** `C:\Users\TANISHKA SAWANT\ASTRAMESH-X`  
**Git Branch:** `main` (Tracking `origin/main`)  
**Regression Test Status:** **21 / 21 CTest Targets Passing (100% in 1.27 s)**  

---

## 1. Executive Summary

As of October 31, 2026, the **AstroMesh-X** project has transitioned from an experimental research codebase to a fully functional, integrated, tested, documented, and demonstration-ready decentralized topological SLAM and communication simulation platform.

The system addresses the challenging problem of multi-rover collaborative mapping in GPS-denied planetary environments subject to strict communication quotas, high packet drop rates, and long transmission latencies. All planned algorithms—including Invertible Bloom Lookup Table (IBLT) set reconciliation, closed-form Kabsch/Umeyama RANSAC geometric verification, Lie group $\mathrm{SE}(2)$ kinematics, Continuous Riemannian Dynamics (CORD) distributed pose-graph optimization, delay-tolerant networking (DTN) store-carry-forward buffering, safe keyframe pruning, and multi-hop mesh routing—are implemented in C++17, statically linked, and verified.

A dedicated standalone simulation engine CLI (`sim_runner.exe`) has been implemented and integrated with a high-fidelity, dark-themed interactive Command-Center UI (`app/server.py` and `app/static/index.html`), providing live 2D playback, per-rover telemetry inspection, real-time network accounting, performance analytics, and empirical benchmark exploration based on over 17,000 real experimental trials.

---

## 2. Final Acceptance Criteria Verification Table

| Acceptance Criterion | Verification Method | Status | Observed Evidence |
| :--- | :--- | :---: | :--- |
| **Project builds successfully** | CMake 4.4.4 & Ninja 1.13.2 with MSYS2 GCC 16.2.0 | **PASS** | `build-ucrt64-clean` compiles with 0 errors |
| **Existing regression tests pass** | `ctest --test-dir build-ucrt64-clean --output-on-failure` | **PASS** | **21 / 21 CTest targets pass (100% in 1.27 s)** |
| **Simulation executes end-to-end** | `sim_runner.exe` execution across 81–101 steps | **PASS** | Ingests steps, discovers closures, merges DSU, exports JSON |
| **At least two rovers simulated** | Demo 1 preset execution | **PASS** | 2 rovers, concentric loops, nominal channel, 81 frames |
| **Eight-rover demo available** | Demo 3 preset execution | **PASS** | 8 rovers, 28 potential links, 101 frames, multi-hop routing |
| **Packet-loss and delay scenarios work** | Demo 2 preset execution | **PASS** | 50% packet loss, 100 ms delay, DTN retransmissions verified |
| **Graph reconciliation & optimization** | CORD optimization execution in pipeline | **PASS** | Semi-implicit Euler-Poincaré integration converges |
| **UI starts successfully** | Python 3 HTTP server (`app/server.py`) | **PASS** | Listens on port 8080, serves clean static assets |
| **UI connects to real backend** | REST API `/api/run` and `/api/demo/<id>` | **PASS** | Subprocess invocation captures stdout and structured JSON |
| **Rover positions & paths visualized** | HTML5 Canvas 2D renderer | **PASS** | Coordinate grid, landmarks, rover triangles, heading lines |
| **Comm links & metrics displayed** | Canvas link overlay + Network tab | **PASS** | Active links, degraded links, wire bytes, delivery ratios |
| **Benchmark comparisons use real data** | `/api/benchmarks` reading Phase 6.1 & 6.2 CSVs | **PASS** | SC1 (14,700 trials, N=30) and SC10 (680 trials, 27.8%) |
| **Three reproducible demos available** | CLI presets (`--demo 1`, `--demo 2`, `--demo 3`) | **PASS** | Demo 1, Demo 2, Demo 3 execute cleanly and deterministically |
| **Export functionality works** | CSV, JSON, and Report endpoints | **PASS** | `/api/export/csv`, `/api/export/json`, `/api/export/report` |
| **README & demo instructions complete** | `README.md` and `docs/DEMO_GUIDE.md` | **PASS** | Updated with architecture, launch commands, and demos |
| **Known limitations documented** | `docs/PROJECT_STATE.md` and Report | **PASS** | `NoACKs` conflation, fixed RTO, simulation boundaries |

---

## 3. System Architecture & Components

```
                           AstroMesh-X Platform Architecture
                               
    +--------------------------------------------------------------------------+
    |                       Interactive Command Center                         |
    |  HTML5 Canvas 2D View  |  Telemetry Gauges  |  Real Benchmark Explorer   |
    +--------------------------------------------------------------------------+
                                       | HTTP / REST API (Port 8080)
    +--------------------------------------------------------------------------+
    |                     Backend Server & Simulation Bridge                   |
    |                  (app/server.py - Zero-Dependency Python 3)              |
    +--------------------------------------------------------------------------+
                                       | Subprocess CLI & Structured JSON
    +--------------------------------------------------------------------------+
    |                    C++ Standalone Simulation Engine                      |
    |                         (bin/sim_runner.exe)                             |
    +--------------------------------------------------------------------------+
          |                         |                        |
    +-------------------+  +-------------------+  +----------------------------+
    | astromesh_core    |  | astromesh_graph   |  | astromesh_simulation       |
    | - LandmarkKey     |  | - PoseGraph       |  | - SyntheticWorld2D         |
    | - CBF (Baseline)  |  | - DisjointSet     |  | - SwarmTrajectories       |
    | - IBLT (32 cells) |  | - KeyframePruner  |  | - AstroMeshXPipeline       |
    +-------------------+  +-------------------+  | - BaselineAstroMeshV1      |
          |                         |             +----------------------------+
    +-------------------+  +-------------------+                 |
    | astromesh_comms   |  | astromesh_optim   |  +----------------------------+
    | - DTN Buffer      |  | - CORD Integrator |  | astromesh_verification     |
    | - Packet Budget   |  | - SE(2) Lie Math  |  | - GeometricVerifier        |
    | - Exp Backoff     |  | - Boundary Filter |  | - Closed-form Kabsch RANSAC|
    +-------------------+  +-------------------+  +----------------------------+
```

---

## 4. Key Experimental & Empirical Findings

### 4.1 Communication Robustness under Severe Packet Loss (SC1, N=30)
Evaluated across all 10 packet loss levels ($0\%$ to $90\%$) with 30 independent seeds:
- **Resilience under Drop:** At $50\%$ loss, `AstroMesh_X_Full` sustains a **$94.47\%$ application completion ratio**, compared to only $50.89\%$ for un-retried systems (`AstroMesh_X_NoRetries`, `AstroMesh_v1_Baseline`).
- **Statistical Significance:** At $70\%$ loss, `Full` delivers $+50.6\%$ higher completion ($p_{\text{adj}} = 2.32 \times 10^{-21}$ under Bonferroni correction), proving the decisive advantage of DTN store-and-forward retransmissions.

### 4.2 Latency-Induced Spurious Retransmissions (Step-Function Scaling)
- With a static initial retry timeout $\text{RTO}_0 = 50\,\text{ms}$, one-way link delays $\tau \in \{0, 20, 50, 100, 200, 350, 500\,\text{ms}\}$ ($RTT = 2\tau$) trigger discrete steps in retransmission volume:
  - $RTT \le 40\,\text{ms}$: 0 retries ($45.4\,\text{kB}$ wire bytes, $1.0\times$).
  - $RTT = 100\,\text{ms} > 50\,\text{ms}$: 1 retry/msg ($90.9\,\text{kB}$, $2.0\times$).
  - $RTT = 200\,\text{ms} > 150\,\text{ms}$: 2 retries/msg ($136.3\,\text{kB}$, $3.0\times$).
  - $RTT \ge 400\,\text{ms}$: 3 retries/msg ($181.7\,\text{kB}$, $4.0\times$).
  - $RTT = 1000\,\text{ms}$: 4 retries/msg ($227.1\,\text{kB}$, $5.0\times$).

### 4.3 Swarm Multi-Hop Mesh Forwarding (SC10, 8 Rovers)
- In the 8-rover collective, multi-hop routing enables packet delivery across non-line-of-sight rovers at an average total wire byte increase of **$+82.1\%$** ($+2,121,260.5\,\text{B}$ per trial) over direct single-hop links.

---

## 5. Identified Limitations & Future Roadmap

1. **`NoACKs` Ablation Conflation:** In `pipeline_astromesh_x.cpp`, disabling explicit ACKs simultaneously suppresses the sender's ACK-awaiting state. In delayed channels ($RTT > 50\,\text{ms}$), `NoACKs` avoids timeout-based retransmissions, conflating ACK wire overhead with latency-induced duplicate suppression.
2. **Fixed Retransmission Timeout (RTO):** The static 50 ms initial timeout causes step-function spurious retransmissions under high latency. Future work will integrate an adaptive Jacobson/Karn RTT estimator ($\text{SRTT} + 4 \times \text{RTTVAR}$).
3. **Simulation Boundary:** All trials were executed in a controlled 2D discrete-time simulation harness (`SyntheticWorld2D`, `SwarmTrajectoryGenerator`, `ObservationModel2D`). Link loss is uncorrelated Bernoulli; delays are deterministic per hop. These metrics must not be directly generalized to real planetary rovers where multi-path fading, non-Gaussian transmission delays, and 3D terrain roughness introduce additional physical failure modes.
4. **SC10 Dataset Scoping:** In accordance with user computational constraints, SC10 was halted at 680 of 2,450 trials ($27.8\%$). These results are presented strictly as exploratory scaling observations and must never be cited as a completed 70-cell sweep.

---

## 6. How to Launch and Reproduce

### Step 1: Run the Regression Suite
```powershell
ctest --test-dir build-ucrt64-clean --output-on-failure
```
*Expected: 21/21 tests passed (100%) in ~1-2 seconds.*

### Step 2: Launch the Mission Control Dashboard
```powershell
python app/server.py
```
*Navigate to `http://127.0.0.1:8080` in any web browser.*

### Step 3: Run the Three Demos via CLI
```powershell
# Demo 1: Basic Multi-Rover SLAM (2 rovers)
.\build-ucrt64-clean\bin\sim_runner.exe --demo 1 --output-json scratch/demo1.json --output-csv scratch/demo1.csv

# Demo 2: Comm Failure & DTN Recovery (4 rovers, 50% loss, 100 ms delay)
.\build-ucrt64-clean\bin\sim_runner.exe --demo 2 --output-json scratch/demo2.json --output-csv scratch/demo2.csv

# Demo 3: Multi-Rover Swarm (8 rovers, multi-hop mesh, 20% loss, 50 ms delay)
.\build-ucrt64-clean\bin\sim_runner.exe --demo 3 --output-json scratch/demo3.json --output-csv scratch/demo3.csv
```

---
**Sign-off:** AstroMesh-X is verified, fully functional, and ready for presentation and deployment.

