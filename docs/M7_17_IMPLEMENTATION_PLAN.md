# AstroMesh-X — Milestone 7.17 Implementation Plan
## End-to-End Scientific Benchmark Validation & Reproducibility

**Status:** Approved  
**Date:** 2026-11-14  
**Milestone:** 7.17  

---

## 1. Plan Overview

Milestone 7.17 executes a comprehensive audit and statistical validation of AstroMesh-X without modifying historical datasets or introducing unverified claims.

```
+---------------------------------------------------------------------------------------------------+
| Stage 0: Historical Dataset Audit & Inventory Creation                                            |
| - Verify byte-exact preservation of all historical CSV files                                      |
| - Map all claims to implementation, generators, raw data columns, and reproduction commands       |
+---------------------------------------------------------------------------------------------------+
                                                  |
                                                  v
+---------------------------------------------------------------------------------------------------+
| Stage 1: Implement 7 Verification & Statistical Test Suites                                       |
| 1. tests/test_m717_ground_truth_integrity.cpp    (No GT leakage, single SE(2) global alignment)   |
| 2. tests/test_m717_ate_alignment.cpp             (Adversarial check: per-rover vs. global ATE)    |
| 3. tests/test_m717_benchmark_fairness.cpp        (Matched initial conditions, seeds, and budgets) |
| 4. tests/test_m717_loop_closure_statistics.cpp   (Confusion matrix, RANSAC thresholds, FP=0 audit)|
| 5. tests/test_m717_network_accounting.cpp        (Total wire vs payload, multihop hop overhead)   |
| 6. tests/test_m717_memory_accounting.cpp         (Logical vs capacity vs kernel RSS)              |
| 7. tests/test_m717_reproducibility.cpp           (Deterministic seed repeatability & coverage)    |
+---------------------------------------------------------------------------------------------------+
                                                  |
                                                  v
+---------------------------------------------------------------------------------------------------+
| Stage 2: Register in CMakeLists.txt & Compile with Ninja                                          |
| - Add all 7 new test targets to build-ucrt64-clean                                                |
| - Verify 100% compilation and pass rate across all 7 targets                                      |
+---------------------------------------------------------------------------------------------------+
                                                  |
                                                  v
+---------------------------------------------------------------------------------------------------+
| Stage 3: Build & Execute Unified Reproducibility Harness (Python)                                 |
| - scratch/reproducibility_harness.py                                                              |
| - Generate benchmark_m717_validation.csv                                                          |
| - Generate benchmark_m717_statistical_summary.csv                                                 |
| - Generate benchmark_m717_evidence_matrix.csv                                                     |
+---------------------------------------------------------------------------------------------------+
                                                  |
                                                  v
+---------------------------------------------------------------------------------------------------+
| Stage 4: Full Regression & Acceptance Verification                                                |
| - Run full CTest suite (all 65 targets, 100% pass)                                                |
| - Run scratch/test_acceptance.py (13/13 criteria, 100% pass)                                      |
| - Verify dashboard telemetry ingestion                                                            |
+---------------------------------------------------------------------------------------------------+
                                                  |
                                                  v
+---------------------------------------------------------------------------------------------------+
| Stage 5: Documentation & Milestone Closure                                                        |
| - docs/M7_17_BENCHMARK_METHODOLOGY.md                                                             |
| - docs/SCIENTIFIC_EVIDENCE_MATRIX.md                                                              |
| - docs/REPRODUCIBILITY_GUIDE.md                                                                   |
| - docs/M7_17_DEVELOPMENT_REPORT.md                                                                |
| - docs/M7_17_FINAL_VERIFICATION_AUDIT.md                                                          |
| - Update ADR-044 in docs/DECISIONS.md                                                             |
| - Update docs/PROJECT_STATE.md, docs/ROADMAP.md, docs/BUILD_LOG.md                                |
+---------------------------------------------------------------------------------------------------+
```

---

## 2. Stage Breakdown & Verification Criteria

### Stage 1: Verification Test Suites
- **`test_m717_ground_truth_integrity`**: Tests that production algorithms (`PoseGraph`, `CORDIntegrator`, `SubgraphAligner`, `GaugeCoordinator`) operate without reading `ground_truth` coordinates. Demonstrates that ground truth is isolated in test fixtures.
- **`test_m717_ate_alignment`**: Constructs a deliberate multi-rover scenario where rovers are internally misaligned by $15.0\,\text{m}$. Proves that naive per-rover alignment falsely reports $\text{ATE} \approx 0.02\,\text{m}$, whereas the mandatory single global rigid SE(2) alignment correctly exposes the true multi-rover error ($\text{ATE} > 7.0\,\text{m}$).
- **`test_m717_benchmark_fairness`**: Verifies that when comparing baselines (e.g. AstroMesh v1 vs AstroMesh-X, or Unbounded vs Memory-Bounded), both algorithms receive identical random seeds, trajectory lengths, noise sequences, and sensor ranges.
- **`test_m717_loop_closure_statistics`**: Tests `GeometricVerifier` on adversarial synthetic sets: random non-rigid imposters are rejected (100% specificity), genuine inliers pass (100% recall), and an adversarially crafted rigid duplicate is tested to demonstrate the theoretical limits of RANSAC.
- **`test_m717_network_accounting`**: Verifies wire byte conservation across 1-hop, 2-hop, and 3-hop transmissions. Demonstrates that multi-hop routing scales wire cost $O(h)$, and verifies DTN volatile queue semantics.
- **`test_m717_memory_accounting`**: Compares `MemoryTracker` logical size, container capacity, and Windows kernel RSS. Verifies that compaction reduces logical memory and graph edge count, while documenting why OS RSS does not instantly plummet.
- **`test_m717_reproducibility`**: Runs identical test simulations twice across fixed seeds (e.g. Seed 42 and Seed 1337), asserting byte-for-byte identical output trajectory nodes and metric calculations.

### Stage 2: CMake & Ninja Integration
- Register the 7 test targets in `CMakeLists.txt` using the standard `-UNDEBUG` release configuration.
- Compile cleanly under MSYS2 UCRT64 GCC 16.2.0.

### Stage 3: Python Unified Reproducibility Harness
- Implement `scratch/reproducibility_harness.py`:
  - Parses historical and current benchmark results.
  - Computes paired differences, bootstrap 95% confidence intervals, Cohen's $d$ effect sizes, and p-values.
  - Verifies cryptographic SHA-256 hashes of all benchmark CSVs.
  - Outputs `benchmark_m717_validation.csv`, `benchmark_m717_statistical_summary.csv`, and `benchmark_m717_evidence_matrix.csv`.

### Stage 4 & 5: Regression, Acceptance, Evidence Matrix & Closure
- Execute CTest (all 65 targets).
- Execute `scratch/test_acceptance.py`.
- Compile master reports and update persistent repository records.

