# AstroMesh-X — 72-Hour Sprint: Milestone 0 Baseline Freeze Report

**Date:** 2026-11-01  
**Milestone:** Milestone 0 — Baseline Freeze  
**Status:** **PASSED & VERIFIED**  

---

## 1. Git Status & Working Tree Integrity
- **Branch:** `main` (tracking `origin/main` at commit `2a9d5ad`)
- **Uncommitted Modifications:**
  - Tracked modified: `.gitignore`, `CMakeLists.txt`, `README.md`, `docs/ARCHITECTURE.md`, `docs/BUILD_LOG.md`, `docs/DECISIONS.md`, `docs/EXPERIMENT_LOG.md`, `docs/PROJECT_STATE.md`, `docs/REFERENCES.md`, `docs/ROADMAP.md`, `docs/TEST_PLAN.md`.
  - Untracked directories: `app/`, `include/astromesh/`, `src/`, `tests/`, `docs/` (completed audit and design reports), `scratch/`.
- **Preservation Check:** Zero files were deleted or reset. All existing user modifications and source additions are preserved intact.

---

## 2. CMake Build Verification
- **Toolchain:** MSYS2 UCRT64 GCC 16.2.0 (`x86_64-w64-mingw32`), Ninja 1.13.2, CMake 4.4.4.
- **Linker Configuration:** `-static` (ADR-028), zero missing DLL runtime dependencies.
- **Build Directory:** `build-ucrt64-clean`
- **Result:** **33/33 targets built cleanly with exit code 0**.

---

## 3. CTest Regression Suite
- **Command:** `ctest --test-dir build-ucrt64-clean --output-on-failure`
- **Result:** **21/21 test targets passed (100% pass rate in 18.12 s)**.
  - Tests 1–4: Core keys, CBF baseline, IBLT engine, property tests.
  - Tests 5–8: Pose graph, Kabsch RANSAC verification, DSU merging, adversarial loop closure rejection.
  - Tests 9–13: SE(2) Lie math, objective evaluation, centralized optimizer, CORD integrator, distributed CORD.
  - Tests 14–17: DTN buffer, packet budget, keyframe pruner, rover orchestration.
  - Tests 18–21: Synthetic world, AstroMesh v1 baseline, AstroMesh-X pipeline, channel robustness.

---

## 4. API & Acceptance Test Suite
- **Command:** `python scratch/test_acceptance.py` against `http://localhost:8765`
- **Result:** **12/12 acceptance checks PASSED**:
  1. Backend Server Startup (`app/server.py` on port 8765): PASS
  2. Dashboard UI Loading (`http://localhost:8765`): PASS
  3. Demo 1 (2 rovers nominal): PASS (81 frames, $89.7\,\text{kB}$ wire bytes, 0 failures)
  4. Demo 2 (4 rovers 50% loss, 100 ms delay): PASS (101 frames, 104 closures, $2.59\,\text{MB}$ wire)
  5. Demo 3 (8 rovers 20% loss, 50 ms delay, multi-hop): PASS (101 frames, 28 links, $5.42\,\text{MB}$ wire)
  6. Timeline Scrubbing & Playback Speeds: PASS
  7. Rover Selection & Pose Telemetry: PASS
  8. Network Links & Byte Conservation Invariants: PASS
  9. Export Endpoints (CSV, JSON, Report): PASS
  10. Custom Simulation Parameter Execution: PASS
  11. Terminal Stability (Zero unhandled server crashes): PASS
  12. Empirical Metric Verification & SC10 Scoping Transparency: PASS

---

## 5. Historical Benchmark Dataset Integrity
All existing trial CSV files and summaries were audited for presence and line count:
- `benchmark_phase54_trials.csv`: 601 lines (300 paired Monte Carlo trials) — **INTACT**
- `benchmark_phase54_summary.csv`: 71 lines — **INTACT**
- `benchmark_phase61_trials.csv`: 2,101 lines (2,100 ablation runs) — **INTACT**
- `benchmark_phase61_summary.csv`: 481 lines — **INTACT**
- `benchmark_phase61_failures.csv`: 1 line (0 simulation failures) — **INTACT**
- `benchmark_phase62_trials.csv`: 15,381 lines (14,700 SC1 trials + 680 SC10 trials) — **INTACT**
- `benchmark_phase62_summary.csv`: 3,361 lines — **INTACT**

---

## 6. Robotics Environment Discovery (WSL2 / ROS 2 / Gazebo)
- Probed host environment: `wsl --list --verbose` returned `Ubuntu-22.04 Stopped`.
- Attempted invocation: `wsl -d Ubuntu-22.04` returned `Catastrophic failure Error code: Wsl/Service/E_UNEXPECTED`.
- Native Windows `where.exe ros2` and `where.exe gazebo` confirmed that ROS 2 and Gazebo are not installed natively on this host.
- **Sprint Conclusion:** In accordance with Sprint Rule 10, the distributed multi-rover implementation will execute natively on Windows using MSYS2 UCRT64 GCC 16.2.0, native Winsock/UDP datagram sockets, and independent Windows processes, avoiding unstable OS virtualization.

---

## 7. Milestone 0 Exit Criteria
- [x] Git status verified and uncommitted work preserved.
- [x] Clean CMake/Ninja build confirmed.
- [x] 100% CTest pass rate (21/21 targets).
- [x] Acceptance and demo suites verified.
- [x] Benchmark CSVs confirmed intact.
- **Exit Status: PASSED — Baseline Frozen. Authorized to advance to Milestone 1.**

