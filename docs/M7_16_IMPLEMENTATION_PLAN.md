# AstroMesh-X — Milestone 7.16 Implementation Plan
## Autonomous Exploration & Connectivity-Aware Path Planning

**Document Version:** 1.0.0  
**Status:** Approved Roadmap  
**Target Build:** `build-ucrt64-clean`  
**Toolchain:** Windows MSYS2 UCRT64 GCC 16.2.0, Ninja 1.13.2, CMake 4.4.4, C++17  

---

## 1. Execution Stages

### Stage 1: Frontier Data Structures & Detection Engine
- Create `include/astromesh/exploration/frontier.hpp` & `src/exploration/frontier.cpp`.
- Define `Frontier`, `FrontierStatus`, `FrontierConfig`.
- Implement `TopologicalFrontierDetector`: boundary keyframe extraction, radial sector projection, spatial clustering/duplicate suppression ($\epsilon_{\text{frontier}} = 3.0\,\text{m}$), reachability checks, and dynamic lifecycle management.
- Unit Test: `tests/test_frontier_detection.cpp`.

### Stage 2: Exploration Utility Model
- Create `include/astromesh/exploration/exploration_utility.hpp` & `src/exploration/exploration_utility.cpp`.
- Implement multi-attribute utility: information gain proxy ($\log(1 + \det(S))$), spatial coverage, geometric novelty, travel cost, network connectivity risk, memory pressure penalty, and peer redundancy penalty.
- Unit Test: `tests/test_exploration_utility.cpp`.

### Stage 3: Connectivity-Aware Path Planner
- Create `include/astromesh/exploration/connectivity_path_planner.hpp` & `src/exploration/connectivity_path_planner.cpp`.
- Implement candidate destination evaluation, travel cost estimation, anti-oscillation commitment hysteresis ($\Delta_{\text{switch}}$), connectivity breach detection, and `ReturnToSwarm` fallback station-keeping.
- Unit Test: `tests/test_connectivity_path_planner.cpp`.

### Stage 4: Multi-Rover Coordination & Lease Protocol
- Create `include/astromesh/exploration/multi_rover_coordinator.hpp` & `src/exploration/multi_rover_coordinator.cpp`.
- Implement 40-byte `FrontierClaim` wire serialization, lease tracking, deterministic conflict resolution, partition-aware local autonomy, and stale lease reclamation.
- Unit Test: `tests/test_frontier_assignment.cpp`.

### Stage 5: Integrated Exploration Agent
- Create `include/astromesh/exploration/integrated_exploration_agent.hpp` & `src/exploration/integrated_exploration_agent.cpp`.
- Integrate detector, utility model, path planner, lease coordinator with M7.14 `ActiveLoopClosureScheduler`, M7.15 `MemoryBudgetController`, M7.15 `TopologicalMapCompactor`, and M7.13 `GaugeCoordinator`.
- Unit Test: `tests/test_partition_autonomous_exploration.cpp`.
- Integration Test: `tests/test_m716_exploration_integration.cpp`.

### Stage 6: CMake Target Registration & Compilation
- Register `astromesh_exploration` in `CMakeLists.txt`.
- Register the 6 test targets and the benchmark executable in `CMakeLists.txt`.
- Build all targets cleanly via Ninja.

### Stage 7: Empirical Benchmark Campaign
- Create `tests/benchmark_m716_autonomous_exploration.cpp`.
- Run 45-trial benchmark across 5 strategies (Random, Independent Greedy, Connectivity-Unaware, Connectivity-Aware, Full AstroMesh-X) and 4 swarm configurations (2, 4, 8, 16 rovers).
- Output dataset to `benchmark_m716_autonomous_exploration.csv`.

### Stage 8: Dashboard Telemetry Integration
- Update `app/telemetry_collector.py` to ingest exploration telemetry (`exploration_target`, `active_frontiers`, `planner_utility`, `connectivity_status`, `coverage_percentage`).
- Create `scratch/test_m716_dashboard_integration.py` to verify UDP ingestion.

### Stage 9: Full Regression & Acceptance Verification
- Run full CTest suite (all 58 tests).
- Run Python acceptance test suite `scratch/test_acceptance.py`.

### Stage 10: Documentation & Closure
- Deliver `docs/M7_16_BENCHMARK_METHODOLOGY.md`, `docs/M7_16_DEVELOPMENT_REPORT.md`, `docs/M7_16_FINAL_VERIFICATION_AUDIT.md`.
- Append **ADR-043** to `docs/DECISIONS.md`.
- Update `docs/PROJECT_STATE.md`, `docs/ROADMAP.md`, `docs/BUILD_LOG.md`.
- Present final closure report.

