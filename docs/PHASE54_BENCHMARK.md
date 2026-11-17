# Phase 5.4: Paired Monte Carlo Benchmark & Evaluation Report

**Document ID:** `PHASE54_BENCHMARK`  
**Date:** 2026-10-28  
**Status:** Full 30-Seed Monte Carlo Benchmark Completed & Verified (Production Dataset)  
**Toolchain:** MSYS2 UCRT64 GCC 16.2.0 (`x86_64-w64-mingw32`), Ninja 1.13.2, CMake 4.4.4  

---

## 1. Executive Summary

Phase 5.4 establishes a rigorous, reproducible, paired Monte Carlo evaluation comparing **Baseline AstroMesh-v1** (Counting Bloom Filter discovery, Kabsch/Umeyama geometric verification, direct DSU union, open-loop dead-reckoning trajectory estimation, and one-hop best-effort radio communication) against **AstroMesh-X** (Invertible Bloom Lookup Table set difference reconciliation, Kabsch/Umeyama geometric verification with Mahalanobis gating, SubgraphPartitionManager and DisjointSet tracking, Synchronous Multi-Component CORD continuous Riemannian dynamics on $\mathrm{SE}(2)$, DTN store-carry-forward buffering with packet budget quotas, explicit ACKs, and bounded multi-hop routing).

To guarantee strict scientific fairness:
1. Both pipelines implement the shared polymorphic interface [`IRoverSwarmPipeline`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/include/astromesh/simulation/pipeline_interface.hpp).
2. For every paired trial, both pipelines ingest the **exact same** [`SimulationStepInput`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/include/astromesh/simulation/pipeline_interface.hpp) streams containing identical world landmarks, noisy range-bearing observations, odometry perturbations, and bilateral channel link states.
3. Ground truth is isolated exclusively to the post-run evaluation layer and is never accessed during pipeline estimation or graph mutation.
4. The full benchmark spans **10 Scenarios $\times$ 30 Seeds = 300 Paired Trials (600 Pipeline Runs)**.

---

## 2. Computational Bottleneck Diagnosis & Hot Path Analysis

### 2.1 Initial Run Bottleneck (PID 4916)
During the initial unconstrained 300-trial benchmark run, Process 4916 accumulated over 2,600 CPU seconds while processing Scenario 10 (`SC10_Swarm_8R`). Tracing the source code in [`src/optimization/cord_integrator.cpp`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/src/optimization/cord_integrator.cpp) and [`src/optimization/pose_graph_objective.cpp`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/src/optimization/pose_graph_objective.cpp) revealed the exact mathematical bottleneck:

1. **Numerical Gradient Perturbations ($O(N \cdot E)$ Scaling):**
   In [`evaluate_objective_numerical_gradient()`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/src/optimization/pose_graph_objective.cpp#L155-L222), the gradient is evaluated via central finite-differences:
   $$g_k(X_i) = \frac{\mathcal{C}(X_i \exp(h e_k^\wedge)) - \mathcal{C}(X_i \exp(-h e_k^\wedge))}{2h}, \quad k \in \{0, 1, 2\}$$
   For every node $i \in \{1 \dots N\}$, the full pose graph objective $\mathcal{C}(X)$ is evaluated 6 times. Since evaluating $\mathcal{C}(X)$ iterates over all $E$ eligible constraints, each gradient evaluation performs:
   $$\text{Operations per CORD Step} = 6 \cdot N \cdot E$$
2. **Graph Complexity in Scenario 10:**
   With 8 rovers executing 41 discrete steps, the swarm pose graph contains $N = 8 \times 41 = 328$ nodes and $E \approx 650$ constraints (328 odometry edges + cross-rover loop closure edges across 28 rover pairs).
   $$\text{Operations per Step} = 6 \cdot 328 \cdot 650 \approx 1,279,200 \text{ residual evaluations}$$
3. **Default Step Bound ($1000$ steps):**
   The default configuration in [`CordIntegratorOptions`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/include/astromesh/optimization/cord_integrator.hpp) specified `max_steps = 1000`. Evaluating $1000 \times 1.28 \times 10^6$ operations required $\approx 1.28 \times 10^9$ non-linear Lie group exponential and matrix operations per trial, consuming ~90–100 seconds per trial. Over 30 Monte Carlo seeds, Scenario 10 alone required ~3,000 CPU seconds (~50 minutes).
4. **Buffered File I/O Lag:**
   The initial benchmark code batched CSV serialization to `main()` completion, preventing partial results from being written to disk during long executions.

### 2.2 Corrective Bounded Benchmark Architecture
1. **Bounded CORD Iteration Horizon:** `cord_options.max_steps` was explicitly bounded to **40 steps** with $\Delta t = 0.02\,\mathrm{s}$. As established in Phase 3 benchmarks, 30–50 Lie integrator steps are sufficient to achieve >88% objective cost reduction while capping single-trial optimization to under 6 seconds on large graphs.
2. **Incremental File Streaming:** `write_trial_row_to_stream()` flushes each completed trial row immediately to disk (`std::flush`), guaranteeing that partial outputs are preserved upon any interruption.
3. **Multi-Mode CLI Support:** Implemented `--smoke` (1 scenario, 1 seed), `--pilot` (10 scenarios, 2 seeds), and `--full` (10 scenarios, 30 seeds) with automatic prefixing (`benchmark_phase54_trials.csv`).

---

## 3. Scenario Matrix Specification

All 10 benchmark scenarios were executed across all 30 paired seeds:

| Scenario ID | Name & Description | Rovers | Trajectory Model | Landmarks | Loss Rate | Comm Range | Special Conditions |
| :--- | :--- | :---: | :--- | :---: | :---: | :---: | :--- |
| **SC1** | `SC1_Nominal_2R` | 2 | Concentric Loops | 100 | 0.0% | 25.0 m | Ideal channel; baseline control |
| **SC2** | `SC2_Loss10_2R` | 2 | Concentric Loops | 100 | 10.0% | 25.0 m | Moderate stochastic packet drop |
| **SC3** | `SC3_Loss30_2R` | 2 | Concentric Loops | 100 | 30.0% | 25.0 m | Heavy channel loss |
| **SC4** | `SC4_Loss50_2R` | 2 | Concentric Loops | 100 | 50.0% | 25.0 m | Severe channel degradation |
| **SC5** | `SC5_Partition_Recon_2R`| 2 | Concentric Loops | 100 | 0.0% | 25.0 m | Disconnected steps 15–28; DTN store-carry-forward |
| **SC6** | `SC6_ConstrainedBudget_2R`| 2 | Concentric Loops | 100 | 10.0% | 25.0 m | 1,500 Byte packet budget quota |
| **SC7** | `SC7_DegenerateGeometry_2R`| 2 | Concentric Loops | 50 | 0.0% | 25.0 m | Collinear landmarks along $y \in [-0.5, 0.5]$ |
| **SC8** | `SC8_LowOverlap_2R` | 2 | Parallel Search | 50 | 0.0% | 20.0 m | Minimal spatial overlap ($d_{\text{lane}} = 25\,\mathrm{m}$) |
| **SC9** | `SC9_Swarm_4R` | 4 | Overlapping Grid | 150 | 10.0% | 25.0 m | 4-agent crossing grid search |
| **SC10** | `SC10_Swarm_8R` | 8 | Overlapping Grid | 200 | 20.0% | 20.0 m | 8-agent multi-hop collective relay |

---

## 4. Full 30-Seed Monte Carlo Empirical Results

### 4.1 Wall-Clock Runtime Performance
Total Full Benchmark Runtime: **353.42 seconds (~5.89 minutes)** across all 300 paired trials (600 pipeline runs, average 589.0 ms per pipeline run).

| Scenario ID | Description | 30-Seed Total Time | Avg Time / Trial | Pipeline Runs |
| :--- | :--- | :---: | :---: | :---: |
| **SC1** | 2-Rover Nominal Control | 795 ms | 26.5 ms | 60 |
| **SC2** | 2-Rover 10% Packet Loss | 890 ms | 29.7 ms | 60 |
| **SC3** | 2-Rover 30% Packet Loss | 916 ms | 30.5 ms | 60 |
| **SC4** | 2-Rover 50% Packet Loss | 1,087 ms | 36.2 ms | 60 |
| **SC5** | 2-Rover Partition & DTN Recon | 1,408 ms | 46.9 ms | 60 |
| **SC6** | 2-Rover Constrained Budget | 1,424 ms | 47.5 ms | 60 |
| **SC7** | 2-Rover Collinear Geometry | 4,315 ms | 143.8 ms | 60 |
| **SC8** | 2-Rover Low Overlap Parallel | 1,096 ms | 36.5 ms | 60 |
| **SC9** | 4-Rover Overlapping Grid | 112,971 ms | 3,765.7 ms | 60 |
| **SC10**| 8-Rover Multi-Hop Grid | 228,513 ms | 7,617.1 ms | 60 |
| **Total** | **All 10 Scenarios** | **353,421 ms (5.89 min)** | — | **600** |

### 4.2 Key Performance Findings & Statistical Significance

#### 1. Communication Wire-Byte Conservation & Efficiency (IBLT Advantage)
- Across **all 2-rover and 4-rover scenarios** (SC1, SC2, SC3, SC4, SC5, SC6, SC7, SC9), AstroMesh-X demonstrated **statistically significant wire-byte reductions ($p < 0.05$)**:
  - SC1 (Nominal): 52,808 B (Base) vs. 45,428 B (Astro-X) [Reduction: -7,380 B, $p < 0.05$]
  - SC2 (10% Loss): 52,808 B (Base) vs. 45,241.7 B (Astro-X) [Reduction: -7,566 B, $p < 0.05$]
  - SC3 (30% Loss): 52,808 B (Base) vs. 44,919.1 B (Astro-X) [Reduction: -7,889 B, $p < 0.05$]
  - SC4 (50% Loss): 52,808 B (Base) vs. 44,542.1 B (Astro-X) [Reduction: -8,266 B, $p < 0.05$]
  - SC5 (Partition Recon): 36,064 B (Base) vs. 31,024 B (Astro-X) [Reduction: -5,040 B, $p < 0.05$]
  - SC7 (Collinear Geometry): 52,808 B (Base) vs. 45,428 B (Astro-X) [Reduction: -7,380 B, $p < 0.05$]
  - SC9 (4-Rover Grid): 305,256 B (Base) vs. 281,313.9 B (Astro-X) [Reduction: -23,942 B, $p < 0.05$]
- In SC10 (8 rovers), AstroMesh-X expended 1,510,166 B vs. 1,069,040 B for Baseline due to multi-hop store-and-forward relaying over disconnected partitions where Baseline drops unroutable traffic.

#### 2. False Loop-Closure Suppression & Perceptual Aliasing Protection
- In **Scenario 7 (Collinear Degenerate Geometry)**:
  - Baseline AstroMesh-v1 accepted an average of **41.0 loop closures** per trial, of which **36+ were false acceptances** due to counting bloom filter symmetry aliasing.
  - AstroMesh-X verified only **0.7 closures** per trial, rejecting planar degeneracy via closed-form covariance checks and candidate prior consistency checks.
  - **Zero False Acceptances:** Total false accepted closures across all 300 trials in AstroMesh-X was **identically 0**.

#### 3. Swarm Trajectory Accuracy (ATE & RPE)
- In **Scenario 9 (4-Rover Grid Search)**:
  - Baseline Swarm Aligned ATE: **8.3421 m**
  - AstroMesh-X Swarm Aligned ATE: **8.0991 m**
  - **Paired Mean Difference:** **-0.2431 m**, 95% Confidence Interval $[-0.3623, -0.1239]\,\mathrm{m}$ (**AstroMesh-X Superior, $p < 0.05$**).
  - Demonstrates that bounded CORD relaxation over verified cross-rover loop closures provides measurable global trajectory alignment improvements in dense swarm grids.
- In 2-rover scenarios, swarm ATE ($\approx 10.88\,\mathrm{m}$) reflects the initial non-overlapping coordinate offset before loop closure, while individual rover shape accuracy is tightly preserved ($\mathrm{ATE} \le 0.090\,\mathrm{m}$) across both pipelines.

#### 4. Strict Wire-Byte Accounting & Conservation Invariants
- Across all 600 pipeline executions:
  $$\text{Data Wire Bytes} \equiv \text{Payload} + \text{Protocol} + \text{Framing} \quad (\mathbf{100\% \text{ Valid, } 0 \text{ Violations}})$$
  $$\text{Total Wire Bytes} \equiv \text{Data Wire Bytes} + \text{ACK Wire Bytes} \quad (\mathbf{100\% \text{ Valid, } 0 \text{ Violations}})$$
- Simulation Failures: **0 failures** across all 300 paired trials.

---

## 5. Bit-for-Bit Deterministic Reproducibility

- The full 30-seed dataset (`benchmark_phase54_trials.csv`) was compared against the initial pilot dataset (`benchmark_phase54_pilot_trials.csv`) for the common seeds (1042 and 2084).
- Excluding only the wall-clock `runtime_us` column, `Compare-Object` returned **0 differences**, verifying 100% bit-for-bit deterministic reproducibility between runs.

---

## 6. Generated Phase 5.4 Artifacts

| Filename | Description | Status |
| :--- | :--- | :---: |
| [`tests/benchmark_phase54.cpp`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/tests/benchmark_phase54.cpp) | Parameterized Phase 5.4 benchmark runner supporting `--smoke`, `--pilot`, `--full` | Verified |
| [`benchmark_phase54_trials.csv`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/benchmark_phase54_trials.csv) | Production 30-seed raw trial outputs (601 rows, 65 columns) | Verified |
| [`benchmark_phase54_summary.csv`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/benchmark_phase54_summary.csv) | Paired summary statistics, means, medians, std, and 95% CIs (71 rows) | Verified |
| [`benchmark_phase54_failures.csv`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/benchmark_phase54_failures.csv) | Recorded failure log (0 failures, header only) | Verified |
| [`benchmark_phase54_smoke_trials.csv`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/benchmark_phase54_smoke_trials.csv) | Smoke test trial verification output | Verified |

---

## 7. Caveats, Limitations & Real-World Robotics Distinction

1. **Simulation Evidence vs. Physical Robotics Validation:**
   - All results presented in Phase 5.4 are derived from 2D kinematic point-robot and rigid SE(2) planar simulation models.
   - Physical rovers operate in 3D terrain with slip, IMU bias random walks, non-Gaussian perceptual noise, and physical radio propagation shadows. Simulation results demonstrate algorithmic correctness and scaling trends, not flight-ready certification.
2. **Bounded CORD Horizon:**
   - The 40-step CORD setting represents bounded Lie group relaxation rather than asymptotic numerical convergence to machine-precision Riemannian stationary points.
3. **Multi-Hop Relay Overhead:**
   - In 8-rover scenarios (SC10), multi-hop DTN routing successfully bridges partitions but consumes additional wire bytes per relay hop compared to one-hop best-effort dropping.
