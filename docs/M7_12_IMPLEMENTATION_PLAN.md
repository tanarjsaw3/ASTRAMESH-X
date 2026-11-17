# AstroMesh-X — Milestone 7.12 Implementation Plan

**Milestone:** M7.12 — Robust Distributed SLAM Under Network Partitions: Consistency, Recovery, and Adaptive Communication  
**Target:** MSYS2 UCRT64 GCC 16.2.0, Ninja 1.13.2, CMake, C++17  

---

## 1. Work Breakdown Structure (Phases 1 - 7)

### Phase 1: Versioned Distributed Map State & Causality Engine
- **Files to Create:**
  - `include/astromesh/core/version_vector.hpp`
  - `src/core/version_vector.cpp`
  - `include/astromesh/graph/map_update.hpp`
  - `src/graph/map_update.cpp`
- **Deliverables:**
  - `VersionVector`: Array of per-rover monotonic sequence counters, dominance comparison (`<`, `<=`, `==`, `||` concurrency check), binary wire serialization.
  - `MapUpdate`: Distinct update types (KeyframeAdded, LandmarkObserved, EdgeVerified), unique 64-bit ID, vector clock context, idempotent ingestion.
  - Unit tests in `tests/test_version_vector.cpp`.

### Phase 2: Partition-Aware Multi-Round IBLT Reconciliation
- **Files to Create / Extend:**
  - `include/astromesh/net/partition_reconciler.hpp`
  - `src/net/partition_reconciler.cpp`
- **Deliverables:**
  - `PartitionAwareReconciler`: Tracks version vector differences; executes fast vector comparison round, IBLT difference peeling round, and chunked retry round on decode failure.
  - Unit tests in `tests/test_partition_reconciler.cpp`.

### Phase 3: Conflict-Aware Deterministic Graph Merging
- **Files to Create / Extend:**
  - `include/astromesh/graph/conflict_graph_merger.hpp`
  - `src/graph/conflict_graph_merger.cpp`
- **Deliverables:**
  - `ConflictAwareGraphMerger`: Detects duplicate nodes/edges, validates edge provenance, handles conflicting observations deterministically, performs safe DSU rebuilds upon edge invalidation.
  - Unit tests in `tests/test_conflict_graph_merger.cpp`.

### Phase 4: Adaptive Communication Control & Failure-Injection Framework
- **Files to Create:**
  - `include/astromesh/net/adaptive_comms.hpp`
  - `src/net/adaptive_comms.cpp`
  - `include/astromesh/simulation/failure_injector.hpp`
  - `src/simulation/failure_injector.cpp`
- **Deliverables:**
  - `AdaptiveCommsController`: Loss-, RTT-, and queue-regulated rate throttling and priority queuing.
  - `FailureInjector`: Programmatic burst loss (Gilbert-Elliott), packet reordering, jitter, link partition, and node crash/restart.
  - Unit tests in `tests/test_adaptive_comms.cpp` and `tests/test_failure_injector.cpp`.

### Phase 5: Partition Recovery, Consistency Audit & Benchmarks
- **Files to Create:**
  - `include/astromesh/verification/global_consistency_auditor.hpp`
  - `src/verification/global_consistency_auditor.cpp`
  - `tests/test_m712_partition_recovery.cpp`
  - `tests/benchmark_m712_partition_recovery.cpp`
- **Deliverables:**
  - `GlobalConsistencyAuditor`: Verifies convergence across all swarm agents (causal vector equality, DSU unity, zero sync backlog).
  - 45-trial benchmark campaign across 5 topologies generating `benchmark_m712_partition_recovery.csv`.

### Phase 6: Multi-Process UDP & Live Dashboard Verification
- **Files to Create / Extend:**
  - `scratch/test_m712_multiprocess_partitions.py`
  - `scratch/test_m712_dashboard.py`
  - `app/telemetry_collector.py` (add version vector & partition consistency telemetry)
- **Deliverables:**
  - Verification across independent OS processes with simulated partitions and recovery.
  - Real-time dashboard visualization of version vectors and partition status.

### Phase 7: Master Documentation & Release Sign-Off
- **Files to Create / Update:**
  - `docs/M7_12_DEVELOPMENT_REPORT.md`
  - `docs/M7_12_BENCHMARK_METHODOLOGY.md`
  - `docs/M7_12_FINAL_VERIFICATION_AUDIT.md`
  - Update `docs/PROJECT_STATE.md`, `docs/ROADMAP.md`, `docs/BUILD_LOG.md`, `docs/DECISIONS.md`.

---

## 2. CMake Targets Checklist
- `astromesh_core`: Add `version_vector.cpp`.
- `astromesh_graph`: Add `map_update.cpp`, `conflict_graph_merger.cpp`.
- `astromesh_net`: Add `partition_reconciler.cpp`, `adaptive_comms.cpp`.
- `astromesh_simulation`: Add `failure_injector.cpp`.
- `astromesh_verification`: Add `global_consistency_auditor.cpp`.
- New tests:
  - `test_version_vector`
  - `test_partition_reconciler`
  - `test_conflict_graph_merger`
  - `test_adaptive_comms`
  - `test_m712_partition_recovery`
  - `benchmark_m712_partition_recovery`

