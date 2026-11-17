# Milestone 7.13 Implementation Plan: Decentralized Gauge Fixing & Globally Consistent Metric SLAM

## 1. Overview & Phased Roadmap

This plan executes Milestone 7.13 across 6 discrete, verifiable implementation stages:
- **Stage 1: Gauge Freedom & Subgraph Alignment Engine (`include/astromesh/geometry/subgraph_aligner.hpp`, `src/geometry/subgraph_aligner.cpp`)**
  - Implement 2D Kabsch/Umeyama SE(2) relative transform estimation from landmark observation sets.
  - Implement spatial variance pre-checks, collinearity/degeneracy rejection, and alignment residual calculation.
  - Unit tests: `tests/test_subgraph_alignment.cpp`.
- **Stage 2: Alignment Graph & Cycle Consistency Auditor (`include/astromesh/graph/alignment_graph.hpp`, `src/graph/alignment_graph.cpp`)**
  - Directed inter-rover alignment graph storing relative transforms, confidence weights, inlier counts, and residuals.
  - Cycle detection and SE(2) Lie cycle-consistency residual evaluation.
  - Identification and rejection of inconsistent edges along conflicting cycles.
  - Unit tests: `tests/test_alignment_graph.cpp`.
- **Stage 3: Decentralized Gauge Coordinator & Transform Propagator (`include/astromesh/net/gauge_coordinator.hpp`, `src/net/gauge_coordinator.cpp`)**
  - Deterministic minimum-rover-ID anchor election per connected component.
  - Maximum confidence spanning tree path resolution from anchor to peripheral rovers.
  - Versioned transform updates with epoch sequence numbers preventing stale overwrites.
  - Global pose transformation: $T_{\text{global}} = T_{\text{anchor}, i} \cdot T_{\text{local}, i}$.
  - Unit tests: `tests/test_distributed_gauge.cpp`.
- **Stage 4: End-to-End Metric Reconciliation & Partition Integration (`tests/test_m713_partition_alignment.cpp`)**
  - Integration with `PartitionAwareReconciler` and `ConflictAwareGraphMerger`.
  - Scenarios: 2-rover different starting frames, 3-rover linear relay alignment, 4-rover cyclic diamond alignment, 8-rover partitioned sub-swarms.
  - Anchor failure/restart and dynamic anchor re-election.
- **Stage 5: Dedicated Empirical Benchmark Suite (`tests/benchmark_m713_gauge_alignment.cpp`)**
  - Output: `benchmark_m713_gauge_alignment.csv` (preserving all historical CSVs).
  - Comparison across 4 strategies: Unaligned Local Maps, Centralized Baseline, Loop-Closure Only, AstroMesh-X Decentralized Gauge Alignment.
  - 2, 3, 4, 8, and 16 rovers under rotation/translation offsets, noisy correspondences, and conflicting cycles.
- **Stage 6: Telemetry, Regression & Master Documentation**
  - Update `app/telemetry_collector.py` and `app/server.py` to stream anchor ownership, alignment graph edges, and cycle consistency metrics.
  - Full CTest regression verification.
  - Deliver `docs/M7_13_DEVELOPMENT_REPORT.md`, `docs/M7_13_BENCHMARK_METHODOLOGY.md`, `docs/M7_13_FINAL_VERIFICATION_AUDIT.md`.
  - Update `docs/PROJECT_STATE.md`, `docs/ROADMAP.md`, `docs/DECISIONS.md` (ADR-040), and `docs/BUILD_LOG.md`.

---

## 2. Key Mathematical Invariants & Verification Criteria

1. **Cycle Consistency Invariant:**
   For any closed cycle $C = (R_1, \dots, R_m, R_1)$ of valid alignment edges:
   $$\| \log(T_{12} \cdot T_{23} \cdots T_{m1}) \|_{\mathfrak{se}(2)} \le \tau_{\text{cycle}} \quad (0.35\,\text{m})$$
2. **Anchor Election Invariant:**
   In any connected component $\mathcal{C} \subseteq \mathcal{G}_{\text{align}}$:
   $$\text{Anchor ID} = \min_{R \in \mathcal{C}} R$$
3. **Idempotence Invariant:**
   Applying the same alignment update multiple times produces identical global poses without transform drift or oscillation.
4. **Gauge-Unbiased ATE Evaluation:**
   ATE evaluation aligns the combined estimated swarm map to ground truth using a single global rigid transformation $T_{\text{eval}} \in \mathrm{SE}(2)$.

