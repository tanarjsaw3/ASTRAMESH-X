# AstroMesh-X — M7.18 Simulator & Live Dashboard User Guide

## 1. Overview & Launching the Simulator

The AstroMesh-X Interactive Research Simulator & Live Evaluation Dashboard provides a software-only, interactive interface for multi-rover topological SLAM, mesh networking, fault injection, and comparative benchmarking.

### System Prerequisites
- **Operating System:** Windows 10/11
- **Toolchain:** MSYS2 UCRT64 GCC 16.2.0 (C++17, Ninja, CMake)
- **Runtime:** Python 3.10+ (Standard library only; zero ROS 2 or Gazebo dependencies)
- **Supported Browsers:** Google Chrome, Microsoft Edge, Mozilla Firefox

### Launch Instructions
1. **Start the Backend Web & Telemetry Server:**
   ```powershell
   python app/server.py
   ```
   *Terminal Output:*
   ```text
   =================================================================
           ASTRAMESH-X INTERACTIVE MISSION CONTROL BACKEND          
   =================================================================
   Server Root:    ...\app\static
   C++ Sim Engine: ...\build-ucrt64-clean\bin\sim_runner.exe
   Listening on:   http://127.0.0.1:8765
   -----------------------------------------------------------------
   UDP Telemetry:  Listening on 127.0.0.1:29999
   ```

2. **Access the Mission Control Dashboard:**
   Open your browser and navigate to:
   [http://127.0.0.1:8765](http://127.0.0.1:8765)

---

## 2. Interface Layout & Navigation

The interface is structured into three primary panels and seven specialized tabs:

### 2.1 Dashboard Panels
1. **Left Panel — Simulation & Fault Injection Controls:**
   - **Preset Demos:** Instant one-click execution of curated research scenarios (Demo 1 to Demo 4).
   - **Interactive Network Fault Injection:** Buttons to sever wireless links (`Sever Partition`, `Sever Link R1-R2`) and trigger topological healing (`Heal & Trigger Reconciliation`).
   - **Parameter Sliders:** Number of rovers ($2$ to $16$), trajectory geometry (Concentric Loops, Overlapping Grid, Parallel Search Sweep), channel loss rate ($0\%$ to $90\%$), latency ($0$ to $500\,\text{ms}$), communication range ($10$ to $50\,\text{m}$), and duration.
   - **Execute Simulation:** Spawns `sim_runner.exe` to run the multi-rover scenario and stream frames.
2. **Center Panel — 2D Lunar Visualizer & Research Replay:**
   - **Topological Lunar Arena:** 80m $\times$ 80m surface with procedural lunar craters, coordinate grid lines, origin crosshair $(0, 0)$, and dynamic $10\,\text{m}$ scale bar.
   - **Rover Representation:** Oriented chevrons with heading vectors, perception radius circles ($10\,\text{m}$), and historical trajectory trails.
   - **Mesh Links & Packet Pulses:** Luminous cyan lines connecting in-range rovers with animated packet pulses traveling along active communication edges. Red dashed lines with $\times$ cross markers indicate severed or partitioned links.
   - **Dynamic Partition Banner:** Real-time visual warning displayed during network partitions, switching to an emerald confirmation banner when topological healing and CORD gauge fixing converge.
   - **Playback Bar:** Play/Pause ($\blacktriangleright / \mathbf{II}$), Step Back ($\mathbf{I}\blacktriangleleft$), Step Forward ($\blacktriangleright\mathbf{I}$), speed selector ($0.5\times$, $1.0\times$, $2.0\times$, $4.0\times$), and scrub bar.
   - **Event Jump Controls:** Instant scrubbing to key scientific events (`⏮ Start`, `⚡ Partition (t=3.0s)`, `🔄 Reconnection (t=6.0s)`, `⚯ First Closure`, `⏭ End`).
   - **Snapshot Export:** `📷 PNG Snapshot` captures high-resolution canvas frames.
3. **Right Panel — Swarm Telemetry & Rover Inspector:**
   - **Swarm Summary Cards:** DSU Components, Verified Loop Closures, Wire Bytes Transmitted, Delivery Rate.
   - **Rover Selector Pills:** Quick selection of any active rover ($R1 \dots R16$).
   - **Rover Inspector Table:** Ground truth SE(2) pose $(x, y, \theta)$, estimated pose, landmark observation count, battery level, memory footprint, pressure status, active peers, and message delivery count.
   - **Real-Time Event Stream:** Log terminal displaying timestamped C++ execution events and API triggers.

### 2.2 Navigation Tabs
- **Live Command Center:** Interactive simulation, canvas visualizer, playback, and telemetry.
- **Live Multi-Hop Mesh (M7.11):** Real-time UDP network topology, DTN queue occupancy, and routing state.
- **Baseline vs AstroMesh-X:** Side-by-side comparative architecture and empirical trade-off analysis.
- **Network & DTN Telemetry:** Comprehensive wire byte breakdown (Payload, Protocol, Framing, ACK) and transport metrics.
- **Performance Analytics & ATE:** Global trajectory ATE RMSE, inlier ratios, and CORD Riemannian convergence.
- **Scientific Benchmarks (M7.12-M7.17):** Peer-reviewed reproducibility table with SHA-256 verified datasets and effect sizes ($d > 2.0$).
- **Mission Reports & Export:** One-click downloads for trajectory CSV, metrics JSON, high-res canvas snapshot PNG, and Markdown report.

---

## 3. Judge Presentation Mode

Click the **`⚖️ Judge Presentation Mode`** button in the header to activate a full-screen, distraction-free evaluation interface designed for academic judges and reviewers.

- **Dual-Pane Layout:** Maximized simulation canvas on the left, synchronized evaluation walkthrough on the right.
- **5-Stage Narrative Walkthrough:**
  1. *Step 1: Swarm Initialization & Independent Odometry* — Demonstrates local SLAM drift across unaligned frames.
  2. *Step 2: Network Degradation & Topological Partition* — Shows wireless disconnect and volatile DTN buffering.
  3. *Step 3: Multi-Hop Reconnection & IBLT Set Reconciliation* — Explains symmetric difference resolution $O(|A \Delta B|)$.
  4. *Step 4: CORD Riemannian Gauge Alignment & Metric Consistency* — Visualizes distributed pose optimization restoring sub-0.5m global consistency.
  5. *Step 5: Memory-Bounded Pruning for Long-Term Autonomy* — Explains marginalization and keyframe compaction under strict memory budgets.

