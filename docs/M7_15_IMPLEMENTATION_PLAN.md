# AstroMesh-X Milestone 7.15 — Implementation Plan
## Dynamic Map Compaction & Memory-Bounded Distributed SLAM

---

## 1. Execution Stages

### Stage 1: Memory Accounting Framework
- Create `include/astromesh/memory/memory_tracker.hpp` & `src/memory/memory_tracker.cpp`.
- Define `SubsystemMemoryRecord`, `SwarmMemorySnapshot`, and OS RSS query (`get_process_rss_bytes`).
- Implement per-subsystem memory calculation (PoseGraph, Landmarks, AlignmentGraph, Opportunities, VersionVector, DTN, DSU).
- Write `tests/test_memory_accounting.cpp`.

### Stage 2: Memory-Budget Controller
- Create `include/astromesh/memory/memory_budget_controller.hpp` & `src/memory/memory_budget_controller.cpp`.
- Implement `MemoryBudgetConfig`, `MemoryPressureLevel` (`Normal`, `Warning`, `Critical`), hysteresis logic, and budget infeasibility detection.
- Write `tests/test_memory_budget_controller.cpp`.

### Stage 3: Dynamic Map Compactor
- Create `include/astromesh/memory/map_compactor.hpp` & `src/memory/map_compactor.cpp`.
- Implement safe keyframe compounding: identify non-branching intermediate nodes, compose SE(2) relative transforms ($T_{ik} = T_{ij} \cdot T_{jk}$) and information/covariance, protect loop closure endpoints, anchors, and frontiers.
- Write `tests/test_dynamic_map_compaction.cpp`.

### Stage 4: Landmark and Observation Lifecycle
- Create `include/astromesh/memory/landmark_retention.hpp` & `src/memory/landmark_retention.cpp`.
- Implement `LandmarkLifecycleManager`: tracks observation count, recency, cross-rover observations, protects loop closure support landmarks, and bounded capacity eviction while maintaining $\det(S_{\text{geom}}) > 0$.
- Write `tests/test_landmark_retention.cpp`.

### Stage 5: Distributed Compaction Protocol
- Create `include/astromesh/memory/distributed_compaction_protocol.hpp` & `src/memory/distributed_compaction_protocol.cpp`.
- Implement `CompactionNotification` wire message ($\le 128$ B), CRC32 validation, compaction epoch tracking, and idempotent application across rover replicas.
- Write `tests/test_distributed_compaction.cpp`.

### Stage 6: Partition Recovery & Failure Injection Testing
- Write `tests/test_compaction_partition_recovery.cpp`: tests map compaction while partitioned, verifies clean reconciliation upon link reconnection.
- Write `tests/test_m715_memory_bounded_integration.cpp`: integrates all components under severe memory constraints.

### Stage 7: Factorial Benchmark Suite
- Create `tests/benchmark_m715_memory_bounded_slam.cpp`: executes 45-trial benchmark across 2, 4, 8, 16 rovers and long exploration trajectories.
- Verify zero false positives, Global ATE $< 0.50$ m, bounded memory ceiling, and write to `benchmark_m715_memory_bounded_slam.csv`.

### Stage 8: Dashboard Telemetry & System Verification
- Update `app/telemetry_collector.py` for memory telemetry.
- Execute full CTest suite, acceptance tests, and generate M7.15 documentation and ADR-042.

