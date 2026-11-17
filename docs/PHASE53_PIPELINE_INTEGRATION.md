# AstroMesh-X — Phase 5.3 Integration Report: AstroMesh-X Pipeline Integration

## 1. Executive Summary

Milestone 5.3 implements and validates the `AstroMeshXPipeline`, delivering a fully integrated, decentralized, test-verified pipeline adhering to the `IRoverSwarmPipeline` interface contract. AstroMesh-X composes the core research innovations developed across Phases 1 through 4:

1. **Invertible Bloom Lookup Table (IBLT) Set Reconciliation:** Reconciles symmetric differences $A \Delta B = (A \setminus B) \cup (B \setminus A)$ within a strict 1024-byte envelope ($32 \times 32\,\mathrm{B}$ cells).
2. **Kabsch/Umeyama RANSAC Geometric Verification:** Enforces closed-form rigid transformation estimation with spatial inlier refinement and candidate-prior gating.
3. **Topological Map & Disjoint-Set Subgraph Management:** Maintains persistent `PoseGraph` structures and strictly conditions `DisjointSet` / `SubgraphPartitionManager` component mergers on verified loop closures.
4. **Continuous Riemannian Dynamics (CORD) Optimization:** Optimizes the distributed pose graph over Lie group $\mathrm{SE}(2)$ using semi-implicit continuous dynamics with body-fixed velocity retractions, gauge anchoring, and velocity filtering.
5. **Delay-Tolerant Network (DTN) Transport with Explicit ACKs:** Operates bounded store-carry-forward message buffers, transmission sliding-window byte quotas, explicit 24-byte `AckPayload` packets, and bounded multi-hop routing.
6. **Bounded-Memory Keyframe Pruning:** Employs `KeyframePruner` to enforce keyframe bounds while strictly preserving gauge anchors, newest trajectory frontiers, active loop-closure endpoints, and topological articulation points.

All 15 specification suites across `tests/test_pipeline_astromesh_x.cpp` pass with **73 assertions checked (0 failures)**. The full CTest regression suite executes **20/20 test suites passing (100%)** in 1.15 seconds.

---

## 2. Stage A Audit Findings: Phase 5.2 Baseline Test Gap & Root Cause Analysis

### Investigation of Zero-Event Replay Report
In the prior milestone narrative, a scenario replay example reported:
- `candidates_proposed = 0`
- `verified_loop_closures = 0`
- `rejected_loop_closures = 0`

An investigation into `tests/test_baseline_astromesh_v1.cpp` revealed two root causes:
1. **Collinear Test Geometry:** In Test 12 (`test_replay_determinism`), synthetic points were generated as $(i, i)$, lying precisely on the diagonal line $y = x$. In Kabsch/Umeyama estimation, cross-covariance variance orthogonal to the line vanishes, resulting in covariance rank deficiency. Umeyama RANSAC correctly diagnosed `VerificationFailureReason::DegeneratePointConfiguration` and rejected the candidate, resulting in zero verified closures.
2. **Cross-Step Inlier Memory Persistence:** In multi-step scenarios, `RoverInternalState::observed_keys` and `observations_by_landmark_id` accumulate all observed landmarks. Introducing a conflicting batch in Step 2 while previous inliers remained in memory resulted in RANSAC selecting the previous inliers ($\ge 50\%$ inlier ratio), unless conflicting observations explicitly overwrote the existing landmark IDs.
3. **Missing Ingestion Idempotency:** Re-ingesting a step with identical timestamps previously created new sequential keyframes rather than detecting duplicate inputs.

### Remediation & Verification (Phase 5.2 Strengthening)
- **Strengthened Test 12:** Implemented a multi-step sequence featuring valid 2D planar triangles in Step 1 (generating 4 candidates, 1 verified closure, and 1 DSU merge) and scrambled coordinates for landmarks 1..4 in Step 2 (generating 1 rejected closure without DSU mutation). Verified bit-for-bit replay invariance across resets and independent instances.
- **Added Test 16:** Verified disjoint landmark observations ({100..103} vs {200..203}) yielding exactly 0 candidates, 0 verified closures, 0 rejected closures, and 2 isolated components.
- **Added Test 17:** Built an end-to-end multi-step lifecycle verifying all 8 requirements of Section 3 (disjoint observations, candidate discovery, geometric verification, DSU merging, re-ingestion idempotency, conflicting geometry rejection, insufficient correspondence rejection, and ground-truth isolation).
- **Result:** `test_baseline_astromesh_v1` assertion count expanded from 71 to **105 assertions (0 failures, 100% pass)**.

---

## 3. Shared Pipeline Interface (`IRoverSwarmPipeline`)

Both `BaselineAstroMeshV1` and `AstroMeshXPipeline` implement the pure virtual interface defined in [`include/astromesh/simulation/pipeline_interface.hpp`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/include/astromesh/simulation/pipeline_interface.hpp):

```cpp
class IRoverSwarmPipeline {
public:
    virtual ~IRoverSwarmPipeline() = default;

    virtual std::string pipeline_name() const = 0;
    virtual void reset() = 0;
    virtual bool ingest_step(const SimulationStepInput& step_input) = 0;
    virtual SwarmPipelineMetrics collect_metrics() const = 0;
    virtual std::vector<LoopClosureCandidate> get_loop_closure_history() const = 0;
    virtual std::map<uint16_t, std::vector<geometry::SE2>> get_estimated_trajectories() const = 0;
    virtual std::optional<geometry::SE2> get_current_pose(uint16_t rover_id) const = 0;
    virtual size_t num_connected_components() const = 0;
    virtual bool are_rovers_connected(uint16_t rover_a, uint16_t rover_b) const = 0;
    virtual bool optimize() { return false; }
};
```

### Polymorphic Behavior Summary

| Capability / API | Baseline AstroMesh v1 | Proposed AstroMesh-X |
| :--- | :--- | :--- |
| **Pipeline Name** | `"AstroMesh_v1_Baseline"` | `"AstroMesh_X"` |
| **Candidate Discovery** | Counting Bloom Filter (`CountingBloomFilter`) | Invertible Bloom Lookup Table (`InvertibleBloomFilter`) |
| **Reconciliation Logic** | Single-sided membership query | Symmetric difference peeling $A \Delta B$ |
| **Geometric Verification** | Kabsch/Umeyama closed-form RANSAC | Kabsch/Umeyama closed-form RANSAC + Prior Gating |
| **Topological Merging** | Direct `DisjointSet` union | `PoseGraph` edges + `SubgraphPartitionManager` |
| **Trajectory Estimation** | Dead-reckoning open-loop integration | Open-loop integration + Continuous CORD optimization |
| **`optimize()` Method** | Returns `false` (Unsupported) | Executes `integrate_cord()`, returns `true` |
| **Network Model** | 1-hop best effort broadcast (no DTN/ACK/retry) | Bounded DTN buffer, sliding-window quotas, explicit ACKs, multi-hop |
| **Memory Management** | Unbounded keyframe accumulation | Safe `KeyframePruner` protecting anchors & articulation points |

---

## 4. Subsystem Integration & Data Flow

### A. IBLT Set Reconciliation
- **Data Flow:** Rover A and Rover B maintain `InvertibleBloomFilter(iblt_cells=32, probes=3)`. Each landmark observation inserts `obs.landmark_key`.
- **Peeling:** $T_{diff} = A - B$ peels $A \setminus B$ (`left_only`) and $B \setminus A$ (`right_only`).
- **Capacity Gating:** If difference exceeds $d_{max}$ (~26 differences on 32 cells), peeling stalls with `IBLTDecodeStatus::IncompleteResidual`.
- **Failure Atomicity:** Incomplete residual increments `sync_incomplete_decodes++`, preserves all graph nodes/edges unmodified, and skips geometric verification and DSU union.
- **Candidate Extraction:** Keys held by both rovers not in `left_only` / `right_only` form shared candidate set.

### B. Geometric Verification & Graph Gating
- Correspondences derived exclusively from measured local coordinates (`measured_local_point`). Ground-truth coordinates are never queried.
- Candidates with $< 4$ correspondences reject early with `InsufficientCorrespondences`.
- Verified closures atomically add an `InterRoverLoopClosure` edge with status `Verified` to `PoseGraph`, update `SubgraphPartitionManager`, and merge in `global_dsu_`.
- Conflicting non-rigid correspondences are rejected with `DegeneratePointConfiguration` or `InsufficientInliers` and recorded in `rejected_closures_` without mutating DSU.

### C. Distributed CORD Optimization
- Invoked via `pipeline.optimize()`.
- Collects active keyframe poses into `std::unordered_map<NodeId, SE2>`.
- Fixes gauge anchor `NodeId(1, 0)` at identity to prevent gauge drift.
- Runs `integrate_cord(swarm_graph, initial_poses, cord_options)` executing semi-implicit Lie integrator steps on body twists $\xi_i \in \mathfrak{se}(2)$ and configuration retractions $X_i \leftarrow X_i \exp(h \xi_i^\wedge)$.
- Updates rover estimated poses with relaxed coordinates and records `optimization_steps_performed`.

### D. DTN, ACKs, and Bounded Multi-Hop Communication
- Channel state defines range, physical partitions, and drop probability.
- **Multi-Hop:** When direct link $A \leftrightarrow B$ is partitioned, `AstroMeshXPipeline` searches for an intermediate carrier $C$ with active links $A \leftrightarrow C$ and $C \leftrightarrow B$. Routes through $C$ with hop increment ($route\_hops = 2$). If hops exceed `default_max_hops` (5), drops packet with `hop_limit_exhausted++`.
- **Wire Accounting:**
  - Data packet: $1024\,\mathrm{B}$ IBLT payload + $24\,\mathrm{B}$ protocol header + $16\,\mathrm{B}$ framing = $1064\,\mathrm{B}$ per hop.
  - Return ACK packet: $24\,\mathrm{B}$ payload + $20\,\mathrm{B}$ transport framing = $44\,\mathrm{B}$ per hop.
- **Retries:** Channel drop retains packet in DTN buffer for retransmission and increments `retransmissions_performed++`.
- **Delivery:** Delivery triggers return ACK, clearing sender DTN buffer and incrementing `unique_deliveries++`.

### E. Safe Keyframe Pruning
- Triggered when keyframe count exceeds `max_retained_keyframes`.
- `KeyframePruner::enforce_memory_bound()` analyzes graph topology:
  - In a simple chain where all intermediate nodes are articulation points, `protect_articulation_points = true` prevents pruning, returning `LimitUnmetNoSafeCandidate` (`keyframe_prunes_rejected++`) to avoid splitting the graph.
  - When redundant nodes or loops exist, safe nodes are pruned, while gauge anchor `NodeId(r, 0)` and newest keyframes remain strictly protected.

---

## 5. Verification Evidence

### Test Executables and Results

1. **Baseline Suite:** `.\build-ucrt64-clean\bin\test_baseline_astromesh_v1.exe`
   - **Assertions:** 105 passed, 0 failed.
   - **Tests:** 17/17 passed.
2. **AstroMesh-X Suite:** `.\build-ucrt64-clean\bin\test_pipeline_astromesh_x.exe`
   - **Assertions:** 73 passed, 0 failed.
   - **Tests:** 15/15 passed:
     - `[TEST 1] Initialization, Invalid Configuration, and Reset` — PASS
     - `[TEST 2] Shared Interface Compliance (IRoverSwarmPipeline)` — PASS
     - `[TEST 3] IBLT Success, Incomplete Decode, and Failure Atomicity` — PASS
     - `[TEST 4] Candidate Gating and Geometric Verification` — PASS
     - `[TEST 5] Rejected Closures Do Not Merge Components` — PASS
     - `[TEST 6] Duplicate Ingestion Idempotency` — PASS
     - `[TEST 7] Distributed CORD Optimization on Graph State` — PASS
     - `[TEST 8] Fixed Gauge Anchor & Disconnected Component Behavior` — PASS
     - `[TEST 9] DTN Routing, Bounded Multi-Hop & Store-Carry-Forward` — PASS
     - `[TEST 10] Wire Byte Accounting & Explicit ACK Overhead` — PASS
     - `[TEST 11] Safe Keyframe Pruning & Invariant Preservation` — PASS
     - `[TEST 12] Seeded Determinism and Replay Invariance` — PASS
     - `[TEST 13] Ground-Truth Isolation (Estimates Distinct from Truth)` — PASS
     - `[TEST 14] Identical Scenario Input Consumed by Both Pipelines` — PASS
     - `[TEST 15] Metric Invariants & Error Code Gating` — PASS
3. **Full CTest Regression Suite:** `ctest --test-dir build-ucrt64-clean --output-on-failure`
   - **Result:** **20/20 test suites passed (100%)** in 1.15 s.

---

## 6. Limitations and Future Work

1. **Channel Schedule Injection:** While deterministic link states are ingested per step, a generalized trace player mapping arbitrary NS-3 RF simulation traces will be added in Phase 5.4.
2. **Optimization Scheduling:** In Phase 5.3, CORD is executed upon step ingestion or via `optimize()`. Continuous asynchronous background optimization threads are reserved for Phase 6 multi-threaded deployment.
3. **Full Comparative Benchmark:** Milestone 5.3 provides the integrated pipeline; large-scale Monte Carlo comparisons across parameter sweeps are assigned to Milestone 5.4.
