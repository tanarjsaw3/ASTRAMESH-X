# AstroMesh-X — M7.18 Interactive Research Simulator & Live Evaluation Dashboard Architecture

## 1. Executive Summary & Design Mission

Milestone 7.18 establishes a professional, interactive, fully software-only research simulator and live evaluation dashboard for AstroMesh-X. The dashboard serves as the primary visual evaluation interface for researchers, reviewers, and judges to inspect, control, benchmark, and stress-test the complete multi-rover SLAM and mesh networking pipeline in real time.

### Guiding Principles
1. **Verifiable & Authentic Execution:** Every rover trajectory, pose estimate, communication packet, loop-closure event, and memory compaction step rendered in the dashboard originates from authentic C++ simulation logic or live UDP telemetry streams. No fake rover animations or synthetic metric curves.
2. **100% Software-Only:** Built entirely upon MSYS2 UCRT64 GCC 16.2.0, CMake, Ninja, C++17, Python 3.10+, and standard HTML5/Canvas/CSS/JS without external graphics libraries or hardware-in-the-loop (HIL) requirements.
3. **Scientific Completeness:** Evaluators can inspect microscopic telemetry (individual rover SE(2) poses, landmark observations, battery/pressure levels, routing tables) and macroscopic system behavior (global ATE RMSE, DSU connected components, DTN custody transfers, compaction ratios).
4. **Interactive Fault Injection:** Real-time triggers allow users to sever communication links, partition the swarm into disconnected subgraphs, observe independent local SLAM, and trigger physical/topological healing with IBLT reconciliation and CORD gauge fixing.
5. **Presentation & Replay Excellence:** Includes a dedicated Judge / Presentation Mode with step-by-step guided walkthroughs, research replay with scrubber and jump-to-event controls, side-by-side baseline comparisons, and multi-format research exports (CSV, JSON, Markdown, PNG/SVG).

---

## 2. System Architecture

```
+----------------------------------------------------------------------------------------------------+
|                                      FRONTEND PRESENTATION LAYER                                   |
|                                                                                                    |
|  +-----------------------+  +------------------------+  +--------------------+  +---------------+  |
|  | Aerospace Simulation  |  |    Rover Inspector     |  | Telemetry Monitors |  | Research Mode |  |
|  | Canvas (2D Lunar/Grid)|  | (State, Poses, Graphs) |  | (SLAM, Net, Mem)   |  | (Replay/Judge)|  |
|  +-----------^-----------+  +-----------^------------+  +---------^----------+  +-------^-------+  |
|              |                          |                         |                     |          |
|              +--------------------------+-------------------------+---------------------+          |
|                                         | SSE & REST APIs                                          |
+-----------------------------------------|----------------------------------------------------------+
                                          |
+-----------------------------------------v----------------------------------------------------------+
|                                    BACKEND SERVICE LAYER (Python 3)                                |
|                                                                                                    |
|  +--------------------+  +----------------------+  +---------------------+  +-------------------+  |
|  | HTTP/REST API      |  | SSE Event Dispatcher |  | Live UDP Telemetry  |  | Historical Data   |  |
|  | (/api/run, demo)   |  | (/api/live/stream)   |  | Ingestion (29999)   |  | & Benchmark Loader|  |
|  +---------^----------+  +----------^-----------+  +----------^----------+  +---------^---------+  |
+------------|------------------------|-------------------------|-----------------------|------------+
             | Subprocess execution   | Thread-safe ring buffer | UDP datagrams         | CSV parsing
+------------v------------------------v-------------------------v-----------------------v------------+
|                                      CORE C++ ENGINE & SIMULATION                                  |
|                                                                                                    |
|  +--------------------+  +----------------------+  +---------------------+  +-------------------+  |
|  |  sim_runner.exe    |  | AstroMeshXPipeline   |  | BaselineAstroMeshV1 |  | Multi-Rover Net   |  |
|  | (CLI / Batch /     |  | (IBLT, CORD, DSU,    |  | (Flooding, DSU,     |  | (MultihopRouter,  |  |
|  |  Step Telemetry)   |  |  Active Loop, Comp.) |  |  Centralized Merge) |  |  AdaptiveComms)   |  |
|  +--------------------+  +----------------------+  +---------------------+  +-------------------+  |
+----------------------------------------------------------------------------------------------------+
```

---

## 3. Component Breakdown

### 3.1 C++ Simulation Engine (`src/simulation/sim_runner.cpp`)
- **CLI Options:** Supports `--rovers <N>` (2, 4, 8, 16), `--scenario <ConcentricLoops|OverlappingGrid|ParallelSearch>`, `--duration <sec>`, `--step-dt <sec>`, `--loss <rate>`, `--delay-ms <ms>`, `--comm-range <m>`, `--variant <name>`, `--seed <uint64>`, `--opt-steps <N>`, `--partition-start <sec>`, `--partition-end <sec>`, `--output-json <path>`, `--output-csv <path>`, `--demo <1..4>`.
- **Telemetry Frame Generation:** For each timestep $k$, serializes:
  - Ground truth poses $(x, y, \theta)$ per rover.
  - Estimated odometry / SLAM poses per rover.
  - Observed landmark IDs and sensor radii.
  - Pairwise wireless links (distance, in-range status, active status).
  - Swarm topology: number of DSU connected components, verified loop closures, wire bytes transferred, application delivery ratio, cumulative ATE.
- **Demo Presets:**
  - Demo 1: Basic 2-Rover Nominal SLAM (Concentric Loops, zero loss).
  - Demo 2: 4-Rover Partition & Recovery (Overlapping Grid, 50% loss, link severed at $t=3\text{s}$, healed at $t=6\text{s}$).
  - Demo 3: 8-Rover Swarm with Multi-Hop Mesh (Concentric Loops, 20% loss, 50 ms delay).
  - Demo 4: 16-Rover Scalability & Memory-Bounded Stress Test (Parallel Search, active map compaction).

### 3.2 Backend Service (`app/server.py` & `app/telemetry_collector.py`)
- **Interactive Partition Injection:**
  - `/api/partition/disconnect`: Sever wireless links between specified rover pairs or split swarm into disconnected partitions.
  - `/api/partition/reconnect`: Restore wireless connectivity, trigger reconciliation and gauge alignment.
  - `/api/partition/status`: Query active partition states.
- **Simulation Control:**
  - `/api/run`: POST endpoint with parameter validation to spawn `sim_runner.exe` and return full step telemetry.
  - `/api/demo/<id>`: GET endpoint serving precomputed or dynamically generated interactive demos (1 to 4).
  - `/api/benchmarks/summary`: Returns authentic statistical metrics from M7.12 through M7.17 CSVs.
  - `/api/export/<csv|json|report>`: Downloadable simulation and benchmark data.
  - `/api/live/stream`: Server-Sent Events (SSE) broadcasting real-time UDP telemetry to connected frontends.

### 3.3 Frontend Interactive Mission Control (`app/static/index.html`)
- **Aerospace UI Theme:**
  - Deep space navy/charcoal background (`#0b0f19`), cards (`#111827`), borders (`#1f2937` / `#374151`).
  - High-contrast visual accents: electric cyan (`#00e5ff`), blue (`#2979ff`), purple (`#7c4dff`), emerald (`#00e676`), amber (`#ffab00`), crimson (`#ff1744`).
- **2D Lunar-like Canvas:**
  - Lunar craters (subtle shaded circles with crater rim highlights), rocky obstacle zones, and terrain elevation grid lines.
  - Grid crosshairs, coordinate axes, and dynamic scale bar ($10\,\text{m}$ / $25\,\text{m}$).
  - 60 FPS smooth interpolation for rover positions, headings, sensor perception arcs ($10\,\text{m}$ FOV), and path trails.
  - Animated packet pulses travelling along active network edges.
  - Visual partition flash / severed link markers.
- **Rover Inspector Panel:**
  - Real-time details for selected rover: ID, coordinates, battery level, memory footprint, pressure status, active waypoints, routing peers, and local landmark counts.
- **Four Dedicated Telemetry Monitors:**
  1. *Global SLAM Monitor:* Trajectory ATE RMSE, maximum error, total nodes, verified/rejected loop closures, gauge alignment convergence.
  2. *Network Monitor:* Topology graph, active links, multihop routing table, DTN transit buffer occupancy, packet delivery ratio, custody ACKs.
  3. *Exploration Monitor:* Explored arena percentage, detected/completed frontiers, utility scores, waypoint navigation status.
  4. *Memory Monitor:* Logical graph bytes, process RSS, compaction ratio, evicted landmarks, memory budget headroom.
- **Interactive Network Partition Demonstration:**
  - Direct interactive buttons: "Sever Rover 1-2 Link", "Partition Swarm (50/50)", "Restore All Links".
  - Live status banner indicating partition onset, duration, and subsequent IBLT set reconciliation + gauge alignment recovery.
- **Baseline vs AstroMesh-X Comparison View:**
  - Side-by-side or toggled view.
  - Real comparison curves: ATE vs Time, Wire Bytes vs Time, Memory vs Time.
- **Research Replay Mode:**
  - Scrub bar with frame-level seeking, speed adjustment ($0.5\times$, $1\times$, $2\times$, $4\times$), Play/Pause/Step.
  - Quick jump buttons: "Jump to Partition", "Jump to Recovery", "Jump to First Loop Closure", "Jump to Compaction".
- **Judge / Presentation Mode:**
  - High-impact evaluation interface maximizing the canvas.
  - Step-by-step guided narrative highlighting research novelty (IBLT symmetric difference, CORD Riemannian optimization, active loop closure, memory compaction).
- **Comprehensive Research Exports:**
  - One-click export of trajectory CSV, metrics JSON, high-res canvas PNG/SVG, and Markdown summary report.

---

## 4. Verification & Testing Strategy

1. **C++ Unit & Telemetry Test (`tests/test_m718_simulator_telemetry.cpp`):**
   - Verify serialization of step telemetry frames.
   - Verify link distance and in-range computation for 2, 4, 8, 16 rovers.
   - Verify partition injection logic and disconnect/reconnect transitions.
   - Verify landmark observation encoding and loop closure history formatting.
2. **Python Backend Integration Test (`scratch/test_m718_simulator_integration.py`):**
   - Test `/api/status`, `/api/run`, `/api/demo/1..4`, `/api/benchmarks/summary`.
   - Test partition API endpoints (`/api/partition/disconnect`, `/api/partition/reconnect`).
   - Test export endpoints (`/api/export/csv`, `/api/export/json`).
   - Test SSE connection and message parsing.
3. **Full Regression Suite:**
   - Ensure all 65 existing CTest targets pass without regression, plus the new 66th target.
   - Verify all 13 acceptance checks remain 100% passing.

