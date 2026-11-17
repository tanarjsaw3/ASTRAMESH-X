# AstroMesh-X — M7.18 5-Minute Interactive Demonstration Script

This document provides a structured, step-by-step demonstration walkthrough for researchers, evaluators, and judges reviewing AstroMesh-X.

---

## Demo Overview (Total Duration: 5 Minutes)

| Phase | Time | Action | Focus Area |
| :--- | :--- | :--- | :--- |
| **Phase 1** | 0:00 - 1:00 | Launch & Preset Demos | Multi-Rover SLAM & 60 FPS Visualizer |
| **Phase 2** | 1:00 - 2:15 | Fault Injection & Recovery | Network Partition, Volatile Buffering, IBLT Reconciliation |
| **Phase 3** | 2:15 - 3:15 | Swarm Scalability (16 Rovers) | 120 Wireless Links, Multi-Hop Routing, High-Density Loops |
| **Phase 4** | 3:15 - 4:15 | Baseline vs AstroMesh-X | Bandwidth Conservation & Metric Consistency |
| **Phase 5** | 4:15 - 5:00 | Research Exports & Reproducibility | Verified Datasets, Effect Sizes, Data Artifacts |

---

## Step-by-Step Walkthrough

### Phase 1: Launch & Nominal Swarm Execution (0:00 - 1:00)
1. **Navigate to Command Center:**
   - Open `http://127.0.0.1:8765` in Chrome/Firefox.
   - Point out the header status badges: `C++ Engine: READY`, `66/66 CTest (100%)`, `13/13 Acceptance`.
2. **Execute Demo 1 (Basic 2-Rover SLAM):**
   - Click **`Demo 1: Basic Multi-Rover SLAM (2R)`**.
   - Press **`▶ Play`** on the playback bar.
   - **Key Observation:** Observe the oriented chevrons traversing concentric loops, leaving historical trails, and observing orange static landmarks within their $10\,\text{m}$ sensor radii. Notice the animated cyan packet pulses flowing along the communication link as telemetry is exchanged.
   - Select **Rover 2** in the Rover Inspector to show live coordinate tracking, landmark observation counts, and simulated battery discharge.

---

### Phase 2: Interactive Fault Injection & Partition Recovery (1:00 - 2:15)
1. **Load Demo 2:**
   - Click **`Demo 2: Network Partition & Recovery (4R)`**.
   - Notice that rovers 1, 2, 3, and 4 begin in an Overlapping Grid scenario under a challenging $50\%$ packet loss channel.
2. **Scrub to Partition:**
   - Click **`⚡ Partition (t=3.0s)`** on the jump scrubber.
   - **Key Observation:**
     - The top banner flashes red: `⚠️ NETWORK PARTITION ACTIVE — INDEPENDENT SUBGRAPHS (t=3.0s)`.
     - The links between $\{R1, R2\}$ and $\{R3, R4\}$ turn into red dashed lines with $\times$ severed cross markers.
     - DSU connected components increase from $2$ to $3$.
     - Both subgraphs continue autonomous local SLAM independently without communication.
3. **Scrub to Reconnection & Healing:**
   - Click **`🔄 Reconnection (t=6.0s)`**.
   - **Key Observation:**
     - Wireless connectivity is restored; links turn cyan with packet pulses.
     - The banner updates to emerald: `✅ RECONCILED — IBLT & CORD GAUGE CONVERGED`.
     - Cross-partition loop closures surge from $24$ to $73$.
     - DSU connected components merge back down to $2$, proving topological reconciliation without full graph transmission.

---

### Phase 3: Swarm Scalability Stress Test (2:15 - 3:15)
1. **Load Demo 4:**
   - Click **`Demo 4: Swarm Scalability Stress (16R)`**.
2. **Inspect Scaled Mesh:**
   - Point out the 16 distinct colored rovers spanning a Parallel Search sweep.
   - Note the metric summary: **$726$ verified loop closures**, **$120$ pairwise channel links**, **$98.67\%$ application delivery ratio**, and **$34.5\,\text{MB}$ wire bytes**.
   - Click through several rover pills ($R1$, $R5$, $R12$, $R16$) to demonstrate independent SE(2) pose tracking and localized peer routing tables.
   - Explain how the underlying C++ memory compaction subsystem bounded process RSS and pruned redundant keyframes to prevent out-of-memory crashes.

---

### Phase 4: Baseline vs AstroMesh-X Comparative Architecture (3:15 - 4:15)
1. **Navigate to "Baseline vs AstroMesh-X" Tab:**
   - Click the **`Baseline vs AstroMesh-X`** tab.
2. **Highlight Architectural Differences:**
   - *Set Reconciliation:* AstroMesh-X replaces naive full-graph flooding ($O(|V| + |E|)$) with Invertible Bloom Lookup Tables (IBLT) that resolve symmetric differences in $O(|A \Delta B|)$ wire bytes, yielding an $88.9\%$ reduction in bandwidth overhead.
   - *Gauge Alignment:* Continuous Riemannian Dynamics (CORD) optimizes over the Riemannian manifold $SE(2)$, avoiding heuristic centroid shortcuts and maintaining sub-0.5m ATE RMSE.
   - *Memory Boundedness:* Dynamic map compaction ensures the pose graph operates within a deterministic $512\,\text{KB}$ budget.

---

### Phase 5: Judge Presentation Mode & Research Exports (4:15 - 5:00)
1. **Activate Judge Presentation Mode:**
   - Click the **`⚖️ Judge Presentation Mode`** button in the header.
   - Walk through the 5-stage evaluator guide using **`Next Step`**, demonstrating how each technical pillar was verified through automated tests.
   - Exit the presentation overlay.
2. **Export Verifiable Artifacts:**
   - Click the **`Mission Reports & Export`** tab.
   - Click **`📷 PNG Snapshot`** on the canvas to download a frame capture.
   - Click **`📄 Export Trajectory CSV`** and **`📊 Export Metrics JSON`** to demonstrate that all simulation telemetry is exported in open, reproducible formats.
   - Conclude by pointing out the **`Scientific Benchmarks`** tab showing empirical Cohen's $d > 2.0$ effect sizes across M7.12 to M7.17.

