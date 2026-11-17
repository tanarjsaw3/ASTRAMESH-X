# AstroMesh-X — M7.18 Evaluator & Reviewer Guide

This guide is intended for scientific reviewers, judges, and technical evaluators inspecting the mathematical integrity, codebase structure, and verifiable claims of AstroMesh-X.

---

## 1. Scientific & Mathematical Pillars

### 1.1 Invertible Bloom Lookup Table (IBLT) Set Reconciliation
- **Mathematical Foundation:** Eppstein & Goodrich (2011).
- **Core Distinction:** An IBLT reconciles set *differences* (symmetric differences $A \Delta B = (A \setminus B) \cup (B \setminus A)$), rather than computing set intersections or unions directly.
- **Complexity:** When the symmetric difference $|A \Delta B| \le d$, decoding completes in $O(d)$ time and transmits $O(d)$ bytes over the wireless channel, completely independent of the total landmark set size $|A \cup B|$.
- **Implementation:** `include/astromesh/core/iblt.hpp` and `src/core/iblt.cpp`.

### 1.2 Continuous Riemannian Dynamics (CORD) Gauge Fixing
- **Mathematical Foundation:** Tian et al., IEEE Transactions on Robotics (2021).
- **Manifold Optimization:** Distributed pose graph optimization over the Riemannian product manifold $\mathcal{M} = SE(2)^N$. Local gradient flows $\nabla_{\mathcal{M}} f(T)$ are projected onto the tangent space $\mathfrak{se}(2)$, integrated via Riemannian retraction (matrix exponential):
  $$\mathcal{R}_X(\xi) = X \exp(\xi)$$
- **Gauge Invariance:** Decentralized gauge fixing anchors one designated rover subgraph (minimum ID or maximum degree) as the gauge reference, remediating unobservable rotational drift without centralized synchronization.
- **Implementation:** `include/astromesh/optimization/` (`cord_integrator.hpp`, `subgraph_aligner.hpp`, `gauge_coordinator.hpp`).

### 1.3 Active Information-Theoretic Loop Closures
- **Selection Criterion:** RANSAC geometric verification coupled with candidate utility scoring balancing information gain (entropy reduction of pose covariance $\Delta H = \frac{1}{2} \log \det \Sigma$) against communication cost.
- **Outlier Rejection:** Two-point RANSAC over 2D correspondences with chi-squared gating ($\chi^2 < \gamma_{\text{thresh}}$).
- **Implementation:** `include/astromesh/verification/geometric_verifier.hpp` and `include/astromesh/coordination/active_loop_scheduler.hpp`.

### 1.4 Memory-Bounded Compaction
- **Bounded Resource Invariant:** In long-duration planetary exploration, linear memory growth $O(t)$ causes process crashes. AstroMesh-X implements a strict memory budget controller enforcing keyframe marginalization, redundant node pruning, and landmark observation eviction.
- **Compaction Protocol:** Information-preserving marginalization of non-loop keyframes preserves graph connectivity while keeping total logical memory strictly below the configured threshold ($512\,\text{KB}$ nominal).
- **Implementation:** `include/astromesh/memory/` (`memory_budget_controller.hpp`, `map_compactor.hpp`, `landmark_retention_policy.hpp`).

---

## 2. Codebase Map & Critical Files

```
ASTRAMESH-X/
├── app/
│   ├── server.py                   # Python HTTP & REST API server (port 8765)
│   ├── telemetry_collector.py      # UDP datagram ingestion (port 29999) & SSE broadcaster
│   └── static/
│       └── index.html              # Aerospace mission control dashboard & 2D visualizer
├── include/astromesh/
│   ├── core/                       # IBLT, Counting Bloom Filter, LandmarkKey
│   ├── geometry/                   # SE(2) Lie group algebra, rotation, exponential map
│   ├── graph/                      # PoseGraph, Disjoint-Set Union (DSU)
│   ├── optimization/               # CORD Riemannian optimizer, GaugeCoordinator
│   ├── verification/               # RANSAC geometric verification & ATE calculation
│   ├── net/                        # MultihopRouter, DTN buffer, AdaptiveComms
│   ├── memory/                     # MemoryBudgetController, MapCompactor
│   ├── exploration/                # FrontierDetector, InformationGain, PathPlanner
│   └── simulation/                 # SyntheticWorld2D, SwarmTrajectories, ObservationModel
├── src/simulation/
│   └── sim_runner.cpp              # C++ simulation CLI & step-by-step frame generator
├── tests/
│   ├── test_m718_simulator_telemetry.cpp # Telemetry, link invariants, and partition tests
│   └── ... (65 additional C++ test targets)
└── scratch/
    ├── test_acceptance.py          # 13-stage end-to-end acceptance suite
    └── test_m718_simulator_integration.py # 6-stage API & partition integration test
```

---

## 3. Verifying Results Autonomously

Evaluators can reproduce all reported metrics and verify test targets using standard command-line tools:

### Command 1: Run Full CTest Regression (66 Test Targets)
```powershell
ctest --test-dir build-ucrt64-clean --output-on-failure
```
*Expected Result:* `100% tests passed out of 66` in $< 12.0\,\text{s}$.

### Command 2: Run End-to-End Acceptance Suite
```powershell
python scratch/test_acceptance.py
```
*Expected Result:* `13 / 13 PASSED (100%)`.

### Command 3: Run M7.18 Simulator Integration Suite
```powershell
python scratch/test_m718_simulator_integration.py
```
*Expected Result:* `ALL M7.18 INTEGRATION CHECKS PASSED (6/6)`.

### Command 4: Audit Historical Dataset Immutability (SHA-256 Hashes)
```powershell
python scratch/compute_csv_hashes.py
```
*Expected Result:* All benchmark CSVs (`benchmark_phase61_trials.csv` through `benchmark_m717_validation.csv`) match their documented SHA-256 digests byte-identically.

