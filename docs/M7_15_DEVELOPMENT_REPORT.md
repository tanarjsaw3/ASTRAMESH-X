# AstroMesh-X — Milestone 7.15 Development Report
## Dynamic Map Compaction & Memory-Bounded Distributed SLAM

**Status:** Completed & Verified  
**Date:** 2026-11-14  
**Milestone:** 7.15  
**Artifacts Generated:**
- `include/astromesh/memory/*.hpp` (5 headers)
- `src/memory/*.cpp` (5 source modules)
- `tests/test_memory_*.cpp` (7 comprehensive unit/integration test suites)
- `tests/benchmark_m715_memory_bounded_slam.cpp` (45-trial benchmark runner)
- `benchmark_m715_memory_bounded_slam.csv` (Empirical dataset)
- `scratch/test_m715_dashboard_integration.py` (UDP telemetry integration test)
- `docs/M7_15_ARCHITECTURE.md`, `docs/M7_15_IMPLEMENTATION_PLAN.md`, `docs/M7_15_BENCHMARK_METHODOLOGY.md`

---

## 1. Executive Summary

Milestone 7.15 advances AstroMesh-X to memory-bounded distributed SLAM with dynamic topological map compaction. Operating over extended mission timelines on embedded compute platforms requires strict bounds on memory consumption without sacrificing topological graph connectivity or metric accuracy.

Milestone 7.15 establishes:
1. **Unified Memory Accounting:** Rigorous separation of logical data size, container capacity, serialized wire size, and physical process RSS (queried via Windows kernel `GetProcessMemoryInfo`).
2. **Configurable Memory Budget Controller:** Tri-tier pressure model (`Normal` $< 0.75$, `Warning` $\in [0.75, 0.90)$, `Critical` $\ge 0.90$) with a $0.65 \times B$ hysteresis recovery floor to eliminate boundary thrashing.
3. **Conservative Topological Map Compaction:** Compounds intermediate degree-2 odometry nodes ($T_{ik} = T_{ij} \cdot T_{jk}$, information composition $\Omega_{ik} = (J_i \Omega_{ij}^{-1} J_i^T + J_j \Omega_{jk}^{-1} J_j^T)^{-1}$, landmark reassociation) while strictly protecting indispensable nodes (branching nodes, loop closure endpoints, exploration frontiers, gauge anchors, and unpropagated version vectors).
4. **Bounded Landmark Lifecycle Management:** Caps maximum observations per landmark, evicts low-confidence singletons under memory pressure, and protects mutual loop closure support landmarks.
5. **Lightweight Distributed Compaction Protocol:** 68-byte packed `CompactionNotification` and 14-byte `CompactionAck` with monotonic epochs and FNV-1a checksums.
6. **Empirical Validation:** 45/45 benchmark trials passed with up to 78.5% memory reduction, Global ATE $< 0.05\,\text{m}$, zero false positives, and 100% CTest pass rate (52/52).

---

## 2. Component Architecture & Implementation Details

### 2.1 Memory Accounting Framework (`include/astromesh/memory/memory_tracker.hpp`, `src/memory/memory_tracker.cpp`)
- Tracks per-subsystem memory footprints: `PoseGraph` (nodes, edges, adjacency), `LandmarkRegistry` (keys, coordinates, observations), `AlignmentGraph` (edges, cycle history), `Coordination` (opportunities, scheduler), `CausalLedger` (version vectors, update log), `DTN` (transit buffer, payload queues), and `DSU` (parent/rank mappings).
- Implements `get_process_rss_bytes()` on Windows using `K32GetProcessMemoryInfo` from `psapi.lib`.
- Exposes `RoverMemoryReport` and `SwarmMemorySnapshot` with element counts, capacity overhead, and physical RSS.

### 2.2 Memory Budget Controller (`include/astromesh/memory/memory_budget_controller.hpp`, `src/memory/memory_budget_controller.cpp`)
- Evaluates memory usage against configurable soft budget $B$:
  - `Normal`: Usage $< 0.75 \times B$. No compaction required.
  - `Warning`: Usage $\in [0.75 \times B, 0.90 \times B)$. Triggers gentle linear compaction and landmark eviction.
  - `Critical`: Usage $\ge 0.90 \times B$. Triggers aggressive multi-hop linear compaction, single-observation landmark purge, and DTN buffer pruning.
- Hysteresis mechanism prevents rapid state oscillations: once elevated to `Warning` or `Critical`, pressure level only drops back to `Normal` when usage falls strictly below $0.65 \times B$.

### 2.3 Conservative Topological Map Compactor (`include/astromesh/memory/map_compactor.hpp`, `src/memory/map_compactor.cpp`)
- **Compaction Candidate Criteria:**
  - Node must have degree exactly 2 (one incoming odometry edge, one outgoing odometry edge).
  - Both incident edges must be verified odometry edges between the same rover.
  - Node cannot be an active loop closure endpoint.
  - Node cannot be within the exploration frontier (most recent $K_{\text{recent}}$ keyframes).
  - Node cannot be an elected gauge anchor.
- **Compounding Transformation:**
  $$T_{ik} = T_{ij} \cdot T_{jk}$$
  $$\Omega_{ik} = \left( J_i \Omega_{ij}^{-1} J_i^T + J_j \Omega_{jk}^{-1} J_j^T \right)^{-1}$$
- **Landmark Reassociation:** Landmark observations attached to the removed node $j$ are dynamically transferred to node $i$ with transformed relative coordinates $p_i = T_{ij} \cdot p_j$.
- **Zero Fill-In Invariant:** Unlike Schur-complement marginalization which creates dense cliques of fill-in edges, conservative degree-2 compounding replaces two serial edges with one compounded edge, strictly decreasing both node count and edge count without adding fill-in.

### 2.4 Landmark Lifecycle Manager (`include/astromesh/memory/landmark_retention.hpp`, `src/memory/landmark_retention.cpp`)
- Limits maximum observations per landmark to a bounded capacity (default 8 observations).
- Under `Warning` pressure, evicts single-observation landmarks with confidence below threshold.
- Under `Critical` pressure, evicts all single-observation non-loop-closure landmarks.
- Protected landmarks: landmarks tagged as loop-closure support (used in verified pairwise alignments) are never evicted.

### 2.5 Distributed Compaction Protocol (`include/astromesh/memory/distributed_compaction_protocol.hpp`, `src/memory/distributed_compaction_protocol.cpp`)
- **`CompactionNotification` (68 bytes):**
  - Origin rover ID (uint16_t), Monotonic epoch (uint64_t), Removed node ID (uint16_t + uint32_t), Retained start node (uint16_t + uint32_t), Retained end node (uint16_t + uint32_t), Compounded relative pose (3 $\times$ double), FNV-1a checksum (uint32_t).
- **`CompactionAck` (14 bytes):**
  - Origin rover ID (uint16_t), Target rover ID (uint16_t), Monotonic epoch (uint64_t), Status flag (uint16_t).
- Monotonic epoch enforcement guarantees idempotent handling and rejection of obsolete compaction notices.

---

## 3. Verification & Benchmark Summary

### 3.1 Unit & Integration Test Suites (7/7 Passed, 100%)
1. `test_memory_accounting`: Verified subsystem breakdown, capacity overhead, and kernel RSS query.
2. `test_memory_budget_controller`: Verified `Normal` $\to$ `Warning` $\to$ `Critical` transitions and $0.65 \times B$ hysteresis recovery.
3. `test_dynamic_map_compaction`: Verified degree-2 compounding, SE(2) pose accuracy, information composition, and landmark reassociation.
4. `test_landmark_retention`: Verified observation bounding, singleton eviction, and loop-closure protection.
5. `test_distributed_compaction`: Verified 68 B / 14 B wire serialization, FNV-1a checksum validation, and epoch tracking.
6. `test_compaction_partition_recovery`: Verified that compaction preserves DSU partition connectivity and permits subsequent loop closures across partition healing.
7. `test_m715_memory_bounded_integration`: Full end-to-end integration test across multi-rover trajectory, pressure handling, and global ATE evaluation.

### 3.2 Full CTest Regression (52/52 Passed, 100%)
All 45 prior CTests plus the 7 new M7.15 tests executed and passed cleanly:
- 100% tests passed out of 52 (Total real test time: 42.85s).

### 3.3 Dashboard Telemetry Integration
- Verified via `scratch/test_m715_dashboard_integration.py`.
- UDP telemetry correctly ingests `memory_pressure` and `memory_compaction` events, updating `TelemetryCollector` metrics (`total_nodes_pruned`, `total_landmarks_evicted`, `current_swarm_logical_bytes`, `current_swarm_rss_bytes`, `peak_swarm_pressure`).

### 3.4 Acceptance Suite (13/13 Passed, 100%)
- `python scratch/test_acceptance.py` passed all 13 checks including backend server startup, demo executions (2, 4, and 8 rovers), UI playback scrubbing, export validation, and terminal stability.

