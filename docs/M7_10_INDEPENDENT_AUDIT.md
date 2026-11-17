# AstroMesh-X — Milestone 7.10 Independent Verification Audit

*Date: 2026-11-01*  
*Auditor: Independent Technical & Scientific Assurance Agent*  
*Constraint: Read-only audit mode — zero source modifications, zero git commands altering history or working tree.*

---

## Executive Summary

| Audit Section | Verdict | Primary Finding |
| :--- | :---: | :--- |
| **1. Repository Integrity** | **PASS** | Branch `main` intact; all 54 benchmark CSVs preserved; git status accurately understood. |
| **2. Multi-Keyframe Trajectories** | **NEEDS IMPROVEMENT** | Odometry chaining works, but heading drift is omitted ($\theta \equiv 0$) and landmark observation map overwrites previous keyframe observations. |
| **3. Distributed CORD Integration** | **FAIL** | Boundary state packets do NOT trigger CORD optimization; reported 86.1% ATE reduction was measured on a *single-agent local test*, not distributed across UDP. |
| **4. Precision and Recall** | **NEEDS IMPROVEMENT** | Ground truth is strictly segregated from SLAM decisions ($FP=0$), but 100% precision/recall was an artifact of scenario construction (all proposals had $\ge 4$ genuine inliers). |
| **5. Benchmark Validity** | **FAIL** | 27 trials executed (not 81); ATE in CSV was computed on static measured edges rather than optimized node poses (producing 0.0% reported reduction across all 27 trials); trials 3 and 9 timed out before loop closures completed, masking component disconnection. |
| **6. Keyframe Pruning** | **PASS (With Scientific Caveats)** | Graph connectivity, anchors, frontiers, and loop endpoints are strictly protected by Tarjan DFS; measurement constraints are excised without Schur complement marginalization. |

**Overall Recommendation:** **NOT READY FOR M7.11** until the discrepancies in CORD trajectory evaluation, DSU swarm accounting, and landmark observation history are corrected.

---

## 1. Audit of Repository Integrity

### Verdict: PASS

### 1.1 Branch and HEAD Status
- **Current Branch:** `main` (tracking `origin/main`).
- **HEAD Commit:** `2a9d5ad` (*feat: bootstrap AstroMesh-X repository (Phase 0)*).
- **Explanation of Clean Baseline vs. Working Tree:**
  Previous sprint reports stated "clean baseline" referring to the Git index having zero staged commits, no merge conflicts, and a clean build directory. In reality, `2a9d5ad` is the only commit in git history. All features from Phases 1 through 7.10 exist as uncommitted files in the working directory because the Operational Guidelines strictly prohibit running `git commit` or `git push` without explicit user authorization.

### 1.2 Tracked, Untracked, and Modified Files
- **Tracked & Modified (11 files):** `.gitignore`, `CMakeLists.txt`, `README.md`, and 8 documentation files in `docs/` (`ARCHITECTURE.md`, `BUILD_LOG.md`, `DECISIONS.md`, `EXPERIMENT_LOG.md`, `PROJECT_STATE.md`, `REFERENCES.md`, `ROADMAP.md`, `TEST_PLAN.md`).
- **Untracked Directories:** `app/`, `include/astromesh/`, `src/`, `tests/`, `scratch/`, and intermediate milestone reports in `docs/`.
- **Integrity Rule Compliance:** Zero files were deleted, reset, or overwritten.

### 1.3 Historical Benchmark Dataset Integrity
All 54 historical CSV files remain bit-for-bit intact, confirmed by physical filesystem query:
- `benchmark_distributed_trials.csv` (81 rows, 7,556 B) — Preserved intact.
- `benchmark_distributed_noisy_trials.csv` (81 rows, 9,021 B) — Preserved intact.
- `benchmark_distributed_trajectory_trials.csv` (27 rows, 4,346 B) — Verified intact.
- `benchmark_phase62_trials.csv` (15,380 rows, 5,739,633 B) — Preserved intact.
- `benchmark_phase61_trials.csv` (2,100 rows, 801,939 B) — Preserved intact.

---

## 2. Audit of Multi-Keyframe Trajectory Implementation

### Verdict: NEEDS IMPROVEMENT

### 2.1 Code Inspected
- File: [`src/net/distributed_reconciliation.cpp`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/src/net/distributed_reconciliation.cpp), lines 225–267 (`add_local_keyframe`, `append_trajectory_keyframe`).
- File: [`include/astromesh/net/distributed_reconciliation.hpp`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/include/astromesh/net/distributed_reconciliation.hpp), lines 105–115.

### 2.2 Mathematical and Algorithmic Analysis
1. **Keyframe Ordering and Odometry Chaining:**
   ```cpp
   const graph::NodeId node_id(m_rover_id, keyframe_index);
   // ...
   if (keyframe_index > 0) {
       const graph::NodeId prev_node(m_rover_id, keyframe_index - 1);
       if (m_graph.has_node(prev_node)) {
           const auto delta = prev_se2.inverse() * pose;
           graph::PoseGraphEdge edge(0, prev_node, node_id, graph::EdgeType::Odometry, ...);
           m_graph.add_edge_detailed(edge);
           m_partition_mgr.add_eligible_edge(edge);
       }
   }
   ```
   - **Finding:** The relative odometry edge correctly computes group difference $\Delta T = T_{k-1}^{-1} T_k \in \mathrm{SE}(2)$.
   - **Shortcut/Assumption:** Chaining strictly looks for `keyframe_index - 1`. If the caller skips indices (e.g. keyframe 0 then keyframe 2), the odometry edge is skipped silently and the graph fractures.
2. **Translational vs. Rotational Drift:**
   - In `benchmark_distributed_trajectory.cpp`, lines 125–130:
     ```cpp
     current_x += dist_odom_noise(rng);
     current_y += 2.0 + dist_odom_noise(rng);
     const SE2 kf_pose(current_x, current_y, 0.0);
     ```
     - **Deficiency:** Rotational heading noise was omitted ($\theta \equiv 0.0$). In real planar rover SLAM, orientation error $\delta \theta$ is the dominant driver of accumulated lateral translational error ($y \sim \sum s_i \sin(\delta \theta_i)$). Setting $\theta = 0$ artificially simplifies the RANSAC estimator.
3. **Landmark Observation Overwriting Defect:**
   - In `add_local_keyframe()`, line 257:
     ```cpp
     for (const auto& obs : observations) {
         m_iblt.insert(obs.key);
         m_local_observations[obs.key.local_index()] = obs;
     }
     ```
     - **Defect:** `m_local_observations` is an associative map `std::map<uint32_t, LandmarkObservationRecord>`. Keying strictly by `landmark_id` (`obs.key.local_index()`) means that if landmark $L$ is observed from keyframe 1 and later re-observed from keyframe 5, the entry for keyframe 5 overwrites keyframe 1.
     - **Consequence:** In multi-keyframe trajectory SLAM, earlier keyframe observation geometry is lost.

---

## 3. Audit of Distributed CORD Integration

### Verdict: FAIL

### 3.1 Code Inspected
- File: [`src/net/distributed_reconciliation.cpp`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/src/net/distributed_reconciliation.cpp), lines 269–310 (`optimize_local_graph_cord`), lines 619–628 (`process_boundary_state`).
- File: [`tests/test_distributed_trajectory_slam.cpp`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/tests/test_distributed_trajectory_slam.cpp), lines 163–220 (`test_distributed_cord_trajectory_relaxation`).

### 3.2 Audit Findings
1. **Passive Boundary State Reception (Zero Optimization Triggered):**
   ```cpp
   void DistributedReconciliationAgent::process_boundary_state(const MessageFrame& frame, [[maybe_unused]] uint64_t now_ns) {
       m_stats.boundary_states_received++;
       auto val_res = optimization::deserialize_boundary_packet(frame.payload.data(), frame.payload.size());
       if (!val_res.valid || !val_res.packet.has_value()) return;
       const auto& pkt = *(val_res.packet);
       m_boundary_neighbor_poses[pkt.node_id()] = pkt.pose();
   }
   ```
   - **Finding:** When a `BoundaryState` packet arrives over UDP, the pose is placed into a local cache `m_boundary_neighbor_poses`. **It does NOT trigger CORD optimization, nor does it update `m_graph`.**
   - The optimization function `optimize_local_graph_cord()` must be called manually by an external driver.
2. **Misleading Test Labeling (Single-Agent Local Test Claimed as "Distributed"):**
   - In `test_distributed_trajectory_slam.cpp`, Test 3 is titled `test_distributed_cord_trajectory_relaxation`.
   - Inspection of Test 3 source code reveals:
     - Only ONE rover node was started (`cfg.rover_id = 1;`).
     - Only ONE agent was instantiated (`agent(1, node, 32, 3);`).
     - The loop closure edge added was an `EdgeType::IntraRoverLoopClosure` on NodeId `(1, 2) -> (1, 0)`.
     - `agent.optimize_local_graph_cord()` was executed purely in-memory on that single rover.
     - **Zero UDP network packets were transmitted or received.**
   - **Conclusion:** Describing Test 3 as "Distributed CORD Trajectory Relaxation over Network Sockets" is technically inaccurate. It is a local intra-rover pose-graph relaxation test.
3. **Independent Reproduction of the Reported 86.1% ATE Reduction:**
   - In Test 3, initial error on node `(1, 2)` was $0.965194\,\text{m}$, and final error after local CORD was $0.134529\,\text{m}$ (an $86.06\%$ error reduction).
   - This result is mathematically authentic for a single-agent intra-rover loop closure, but does **not** represent distributed multi-rover relaxation across UDP sockets.

---

## 4. Audit of Precision and Recall

### Verdict: NEEDS IMPROVEMENT

### 4.1 Code Inspected
- File: [`src/net/distributed_reconciliation.cpp`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/src/net/distributed_reconciliation.cpp), lines 480–573 (`process_candidate_proposal`).
- File: [`include/astromesh/net/distributed_reconciliation.hpp`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/include/astromesh/net/distributed_reconciliation.hpp), lines 25–30 (`LandmarkObservationRecord`).

### 4.2 Segregation of Ground Truth
- In `process_candidate_proposal()`, the incoming observations are passed to `verification::verify_relative_pose_2d(correspondences, m_ransac_config)`.
- **Verification:** `is_ground_truth_inlier` is **never passed to RANSAC**. RANSAC evaluates inliers purely using Euclidean residuals between 2D point coordinates.
- Ground truth is inspected only on lines 561–573 to update statistical counters (`true_positives`, `false_positives`, etc.).
- **Conclusion:** PASS — Ground truth does not contaminate the operational SLAM decision pipeline.

### 4.3 Mathematical Formulation & Boundary Handling
$$\text{Precision} = \frac{TP}{TP + FP}, \quad \text{Recall} = \frac{TP}{TP + FN}$$
- In code (lines 328–336):
  ```cpp
  uint64_t predicted_positives = m_stats.true_positives + m_stats.false_positives;
  if (predicted_positives == 0) return 1.0;
  return static_cast<double>(m_stats.true_positives) / static_cast<double>(predicted_positives);
  ```
- **Deficiency:** If zero proposals are ever evaluated ($TP + FP = 0$), the function returns `1.0`. While safe against division by zero (NaN), returning $100\%$ precision when no tests occurred is scientifically misleading; it should return $0.0$ or NaN if unobserved.
- FPR ($\frac{FP}{FP + TN}$) and FNR ($\frac{FN}{FN + TP}$) are not exposed as member helper methods.

### 4.4 Why 100% Precision and Recall Occurred
- In `tests/benchmark_distributed_trajectory.cpp`, lines 135–160:
  - Every proposal generated between neighboring rovers contained **4 consistent true inliers** ($2\,\text{cm}$ noise) and **1 outlier**.
  - Since $4 \ge \text{min\_correspondences} (3)$, every proposal was a legitimate geometric match (`is_ground_truth_valid == true`).
  - RANSAC successfully filtered the 1 outlier in every single trial, yielding $TP > 0$ and $FN = 0$.
  - Crucially, **zero pure-imposter proposals were injected in the 27 benchmark trials.**
  - Pure imposter proposals were injected only in Test 4 of the unit test suite.
- **Scientific Reality:** The 100% precision and recall reported across all 27 benchmark trials is an artifact of benchmark scenario construction, where no pure false proposals were ever submitted across the wire.

---

## 5. Audit of Benchmark Validity

### Verdict: FAIL

### 5.1 Trial Count Discrepancy
- **Claimed in Plan:** 81 trials ($3 \text{ swarms} \times 3 \text{ depths} \times 3 \text{ channels} \times 3 \text{ seeds}$).
- **Actual Executed Count in CSV:** **27 trials.**
- In `tests/benchmark_distributed_trajectory.cpp`, lines 343–353:
  - Swarm sizes: 3 (`[2, 3, 4]`)
  - Keyframe counts: 3 (`[3, 5, 8]`)
  - Impairment profiles: 3 (`[Nominal, Moderate, Harsh]`)
  - $3 \times 3 \times 3 = 27$ grid cells.
  - Instead of running 3 seeds per cell ($27 \times 3 = 81$), the code ran only 1 seed per cell: `seeds[(trial_id - 1) % seeds.size()]`.
  - **Result:** Only 27 trials were executed.

### 5.2 The 0.0% ATE Reduction Flaw in Benchmark CSV
Inspection of lines 226–234 and 255–260 in `tests/benchmark_distributed_trajectory.cpp` reveals an algorithmic evaluation defect:
```cpp
// Step 5: Evaluate Initial ATE RMSE on verified inter-rover edges
for (const auto& [eid, edge] : g.edges()) {
    if (edge.type == EdgeType::InterRoverLoopClosure && edge.status == EdgeVerificationStatus::Verified) {
        const double est_dist = std::hypot(edge.relative_pose.x, edge.relative_pose.y);
        const double dist_err = std::abs(est_dist - 10.0);
        init_sq_err += dist_err * dist_err;
    }
}

// Step 6: Run CORD
for (size_t i = 0; i < rover_count; ++i) {
    agents[i]->optimize_local_graph_cord(cord_opts);
}

// Step 7: Evaluate Final CORD ATE RMSE
for (const auto& [eid, edge] : g.edges()) {
    if (edge.type == EdgeType::InterRoverLoopClosure && edge.status == EdgeVerificationStatus::Verified) {
        const double est_dist = std::hypot(edge.relative_pose.x, edge.relative_pose.y);
        const double dist_err = std::abs(est_dist - 10.0);
        final_sq_err += dist_err * dist_err;
    }
}
```
- **The Defect:** CORD optimizes **node poses** ($X_i \in \mathrm{SE}(2)$). It does **not** modify `edge.relative_pose` ($Z_{ij}$), which is the fixed measurement constraint estimated by RANSAC!
- Both the "Initial ATE" and "Final CORD ATE" were computed from `edge.relative_pose`.
- As a result, in **all 27 trials in `benchmark_distributed_trajectory_trials.csv`**, `initial_ate_rmse == final_cord_ate_rmse`, and `ate_reduction_pct == 0.0%`.
- The reported 86.1% reduction never occurred in the benchmark CSV.

### 5.3 False Convergence Under Harsh Loss (Trials 3 & 9)
In Trial 3 and Trial 9 (40% packet loss, 50 ms delay):
- Execution loop ran for only 30 steps ($300\,\text{ms}$).
- With 50 ms one-way delay (100 ms RTT) and 40% drops, the 3-way handshake (Sync $\to$ Proposal $\to$ Response) timed out before completing.
- `loop_closures_verified == 0`.
- Because 0 closures occurred, `closures_evaluated == 0`, and the code defaulted ATE to `0.0000m`.
- Furthermore, `final_dsu_components` was logged as `1` because it checked `agents[0]->get_dsu_component_count()`. Since Rover 1's local graph had odometry edges, its local partition count was 1. But Rover 1 never merged with Rover 2!
- Across the actual swarm, there were **2 disjoint components**, but the CSV recorded `final_dsu_components = 1`.

---

## 6. Audit of Keyframe Pruning

### Verdict: PASS (With Scientific Caveats)

### 6.1 Preservation of Critical Topological Elements
Inspection of `src/graph/keyframe_pruner.cpp`:
- **Gauge Anchors:** Explicitly protected (`protect_gauge_anchors = true;` checks `NodeId(r, 0)`).
- **Frontier Keyframes:** Explicitly protected (`protect_newest_keyframe = true;` retains highest local index).
- **Loop Closure Endpoints:** Explicitly protected (`protect_verified_loop_endpoints = true;` and `protect_inter_rover_endpoints = true;`).
- **Graph Connectivity:** Uses Tarjan's bridge/cut-vertex DFS algorithm to identify all articulation points. Articulation points are strictly forbidden from removal.
- **Fail-Safe Behavior:** If the bound cannot be met without removing protected nodes, `LimitUnmetNoSafeCandidate` is returned and the graph is untouched.

### 6.2 Scientific Limitation: Lack of Schur Marginalization
- When a redundant interior node is pruned via `graph.remove_node(victim)`, all incident edges are deleted.
- **Limitation:** No compound constraint (Schur complement / Gaussian marginalization) is inserted between the remaining neighbors. The information along the removed odometry edges is discarded rather than summarized into a virtual factor. While acceptable for a topological heuristic, this is a scientific simplification that must be explicitly acknowledged.

---

## 7. Recommended Fixes (Ranked by Priority)

| Priority | Component | Issue | Action Required |
| :---: | :--- | :--- | :--- |
| **P0** | `benchmark_distributed_trajectory.cpp` | ATE computed on static edges rather than node poses | Rewrite ATE evaluation to compare node poses against ground-truth rover trajectory coordinates before vs after CORD. |
| **P0** | `benchmark_distributed_trajectory.cpp` | DSU swarm count checks only local rover | Compute true global swarm component count by aggregating all rovers' graphs into a global disjoint set. |
| **P1** | `tests/test_distributed_trajectory_slam.cpp` | Test 3 claims distributed CORD but runs single rover locally | Rename Test 3 to clarify it is an intra-rover local CORD test, and add an authentic 2-rover distributed CORD test over UDP. |
| **P1** | `benchmark_distributed_trajectory.cpp` | Only 27 trials executed instead of full 81 | Expand loop to evaluate 3 seeds per parameter cell ($3 \times 3 \times 3 \times 3 = 81$ trials). |
| **P1** | `benchmark_distributed_trajectory.cpp` | 300 ms execution loop too short for 40% loss / 50 ms delay | Increase execution ticks from 30 to 80 ($800\,\text{ms}$) under harsh delay to allow DTN retries to complete. |
| **P2** | `distributed_reconciliation.cpp` | Landmark map overwrites multi-keyframe history | Store observations as `std::map<uint32_t, std::vector<LandmarkObservationRecord>>` to preserve observation history across keyframes. |
| **P2** | `benchmark_distributed_trajectory.cpp` | Zero rotational heading drift | Inject angular noise $\delta \theta \sim \mathcal{N}(0, \sigma_\theta^2)$ during trajectory generation. |
| **P2** | `benchmark_distributed_trajectory.cpp` | 100% precision/recall caused by scenario design | Inject 10%–20% pure imposter proposals ($<3$ inliers) into the benchmark to evaluate empirical false-alarm rejection under network loss. |

---

## 8. Milestone Readiness Verdict

### **VERDICT: NOT READY FOR M7.11**

**Rationale:**
Milestone 7.10 successfully established the foundational multi-keyframe trajectory data flow and keyframe pruning integration, and all 26 CTest regression suites pass. However:
1. The reported benchmark dataset (`benchmark_distributed_trajectory_trials.csv`) recorded **0.0% ATE reduction** in all 27 trials due to evaluating static edge measurements instead of relaxed node poses.
2. The reported 86.1% ATE reduction was derived exclusively from a **single-agent local test**, not distributed UDP execution.
3. Swarm partition convergence under harsh network loss was masked by checking only Rover 1's local graph.

**Next Action:**
Await user review of this audit report. Upon approval, implement the P0 and P1 fixes to achieve genuine distributed CORD validation and 81-trial benchmark integrity before advancing to M7.11.

