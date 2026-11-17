# AstroMesh-X — Milestone 7.17 Architecture
## End-to-End Scientific Benchmark Validation & Reproducibility Framework

**Status:** Approved  
**Date:** 2026-11-14  
**Milestone:** 7.17  
**Scope:** Scientific Reproducibility, Rigorous Statistical Validation, Evidence Integrity, and Ground-Truth Auditing

---

## 1. Architectural Motivation & Problem Statement

Across Milestones 7.10 through 7.16, AstroMesh-X has implemented extensive decentralized SLAM subsystems:
- Continuous Riemannian Dynamics (CORD) distributed pose-graph optimization over UDP (M7.10).
- Multi-hop distance-vector routing and delay-tolerant store-carry-forward buffering (M7.11).
- Causal version vectors and progressive prefix chunking for partition recovery (M7.12).
- Closed-form SE(2) subgraph alignment and decentralized minimum-ID gauge coordination (M7.13).
- Active information-gain loop closure prioritization and two-pass DSU spanning budget allocation (M7.14).
- Conservative degree-2 topological map compaction and memory budget control (M7.15).
- Topological radial-sector frontier detection and connectivity-aware path planning (M7.16).

While these milestones successfully achieved their target benchmarks, rigorous scientific reporting requires a dedicated audit of experimental methodology, metric formulations, statistical validity, and evidence scope:
1. **Ground-Truth & ATE Invariants:** Ensuring that Absolute Trajectory Error (ATE) uses an unbiased single global rigid $\mathrm{SE}(2)$ transformation across the combined swarm map, eliminating any per-rover alignment shortcuts that could artificially mask internal coordinate divergence.
2. **Information Leakage Prevention:** Verifying that ground-truth coordinates, world anchor labels, or oracle topologies never leak into runtime estimators or decision policies.
3. **Loop-Closure Statistics & Confusion Matrix Reality:** Investigating the reported zero false positives ($FP = 0$) in synthetic benchmarks. Clarifying that zero observed false positives on a synthetic benchmark dataset does not constitute proof of zero false positive probability in real-world unstructured environments.
4. **Transparent Communication Accounting:** Ensuring that communication metrics report total physical wire bytes (including transport headers, framing, multi-hop forwardings, retransmissions, and custody ACKs) rather than only isolated application payloads.
5. **Physical Memory vs. Logical Accounting:** Maintaining strict separation between logical container byte size, allocated container capacity, serialized wire size, and physical OS working-set RSS (queried via kernel APIs).
6. **Unified Reproducibility Harness:** Automating deterministic execution across fixed random seeds, generating paired hypothesis tests (Wilcoxon signed-rank / paired t-test), Cohen's $d$ effect sizes, and 95% bootstrap confidence intervals.

---

## 2. Core Architecture & Verification Invariants

```
+---------------------------------------------------------------------------------------------------+
|                                 AstroMesh-X Scientific Audit Engine                               |
+---------------------------------------------------------------------------------------------------+
                                                  |
           +--------------------------------------+--------------------------------------+
           |                                      |                                      |
           v                                      v                                      v
+-----------------------+              +-----------------------+              +-----------------------+
|  Ground-Truth & ATE   |              |  Statistical Rigor &  |              | Network & Wire Byte   |
|   Integrity Auditor   |              |   Hypothesis Engine   |              |   Accounting Engine   |
+-----------------------+              +-----------------------+              +-----------------------+
| - Single global SE(2) |              | - Paired diff analysis|              | - Total wire bytes    |
|   rigid alignment     |              | - Bootstrap 95% CIs   |              |   = Data + ACK wire   |
| - Zero oracle leakage |              | - Cohen's d effect    |              | - Multi-hop forward   |
| - Strict SE(2) Lie    |              |   sizes               |              |   cost accounting     |
|   transform math      |              | - Non-parametric tests|              | - Volatile queue vs.  |
| - Degeneracy rejection|              |   (Wilcoxon / Mann-W) |              |   durable storage     |
+-----------------------+              +-----------------------+              +-----------------------+
           |                                      |                                      |
           +--------------------------------------+--------------------------------------+
                                                  |
                                                  v
+---------------------------------------------------------------------------------------------------+
|                                Unified Reproducibility Harness                                    |
|                              (scratch/reproducibility_harness.py)                                 |
+---------------------------------------------------------------------------------------------------+
| - Deterministic seed campaign execution (Seeds: 42, 1337, 2026, 9999, 12345)                      |
| - Automated output validation and SHA-256 data hashing                                            |
| - Scientific Evidence Matrix generation: Classified into Measured, Derived, Synthetic, Theoretical|
+---------------------------------------------------------------------------------------------------+
```

### 2.1 Ground-Truth and ATE Audit Specification

In multi-rover distributed SLAM where rovers initialize in unaligned local reference frames $\{ \mathcal{F}_i \}$, let $p_{i,k}^{\text{est}} \in \mathbb{R}^2$ be the estimated position of keyframe $k$ on rover $i$, and $p_{i,k}^{\text{GT}} \in \mathbb{R}^2$ be the corresponding ground-truth coordinate in the global world frame $\mathcal{F}_W$.

#### The Global Rigid Alignment Invariant
Under decentralized gauge fixing (ADR-040), the entire estimated swarm map $\mathcal{M} = \{ p_{i,k}^{\text{est}} \mid \forall i, k \}$ must be aligned to ground truth via **a single rigid transformation** $T_{\text{eval}} \in \mathrm{SE}(2)$:
$$T_{\text{eval}} = \arg\min_{T \in \mathrm{SE}(2)} \frac{1}{N_{\text{total}}} \sum_{i=1}^M \sum_{k=1}^{K_i} \| T \cdot p_{i,k}^{\text{est}} - p_{i,k}^{\text{GT}} \|^2$$

$$\text{Global ATE RMSE} = \sqrt{\frac{1}{N_{\text{total}}} \sum_{i=1}^M \sum_{k=1}^{K_i} \| T_{\text{eval}} \cdot p_{i,k}^{\text{est}} - p_{i,k}^{\text{GT}} \|^2}$$

**Crucial Invariant:** Aligning each rover's subgraph independently ($T_{\text{eval}}^{(i)}$) is **strictly prohibited** in swarm evaluation. Independent per-rover alignment absorbs inter-rover gauge rotation and translation errors, fraudulently reporting low ATE even when rovers are completely misaligned with one another.

#### Zero Oracle Leakage Invariant
1. Estimators (`PoseGraph`, `CORDIntegrator`, `SubgraphAligner`, `GaugeCoordinator`, `TopologicalFrontierDetector`) must have **zero compile-time or runtime access** to ground-truth coordinates.
2. Ground-truth poses exist exclusively within simulation containers (`SimulatedWorld`, `SimulatedSwarmLocus`, test fixtures) and are accessed solely during out-of-band benchmark metric evaluation.

---

### 2.2 Statistical Validation & Hypothesis Testing

All empirical claims comparing AstroMesh-X to baseline algorithms must provide formal statistical validation:

1. **Paired Differences:**
   For trial $j \in \{1, \dots, N\}$, with baseline metric $X_j$ and proposed metric $Y_j$ under identical seeds and parameters:
   $$\Delta_j = Y_j - X_j$$
   Report $\bar{\Delta}$, $\operatorname{Median}(\Delta)$, and standard deviation $\sigma_\Delta$.

2. **Standardized Effect Size (Cohen's $d$ for Paired Samples):**
   $$d = \frac{\bar{\Delta}}{\sigma_\Delta}$$
   - Small effect: $|d| \approx 0.2$
   - Medium effect: $|d| \approx 0.5$
   - Large effect: $|d| \ge 0.8$

3. **Bootstrap 95% Confidence Intervals:**
   Generate $B = 2,000$ resamples with replacement:
   $$\bar{\Delta}^*_{(b)} = \frac{1}{N} \sum_{i=1}^N \Delta_{k_i}, \quad k_i \sim \operatorname{Uniform}(1, \dots, N)$$
   $\text{CI}_{95\%} = [\Delta^*_{0.025}, \Delta^*_{0.975}]$.

4. **Significance Testing:**
   Apply the two-sided **Wilcoxon signed-rank test** for non-parametric paired differences to report exact $p$-values. Do not assert statistical significance unless $p < 0.05$.

---

### 2.3 Loop-Closure Validation & Perceptual Aliasing Invariant

Prior milestones reported $FP = 0$ across synthetic benchmark sweeps. M7.17 conducts an adversarial audit:

1. **Root Cause Analysis of $FP = 0$:**
   In synthetic benchmarks, imposter proposals were generated with randomized non-rigid landmark correspondences where the maximum subset of mutually consistent correspondences was $< 3$. Because `GeometricVerifier` mandates $\ge 3$ RANSAC inliers with residual $\le 0.35\,\text{m}$ and spatial variance $\ge 0.05\,\text{m}^2$, all random imposters were deterministically rejected.
2. **Adversarial Aliasing Boundary:**
   If an adversary injects structured imposters where 3+ landmarks form an identical rigid constellation (perceptual aliasing in repetitive environments), RANSAC will accept the false closure unless bounded by:
   - Candidate prior pose gating ($\| T_{\text{prior}} - T_{\text{cand}} \| \le \tau_{\text{prior}}$).
   - Alignment cycle consistency ($\| \log(T_{\text{cycle}}) \| \le 0.35\,\text{m}$).
3. **Scientific Reporting Rule:**
   Zero observed false positives must be documented strictly as:
   *"Zero false positives observed over the $N$ synthetic evaluation trials ($0/N$). In open, unstructured planetary environments with repetitive terrain, the theoretical false-positive rate is non-zero and bounded by descriptor collision probability and geometric verification thresholds."*

---

### 2.4 Communication & Memory Accounting Invariants

1. **Wire Byte Accounting Invariant:**
   $$\text{Total Wire Bytes} \equiv \text{Data Wire Bytes} + \text{ACK Wire Bytes}$$
   $$\text{Data Wire Bytes} \equiv \text{Payload Bytes} + \text{Header Bytes} + \text{Framing Bytes}$$
   For multi-hop forwarding across $h$ hops, intermediate relays consume wire bandwidth. Total network burden is the sum across all physical transmissions over all hops.
2. **Memory Accounting Hierarchy:**
   - **Logical Size:** Exact sizeof and payload bytes stored in data structures ($\sum \text{sizeof}(V_i) + \sum \text{sizeof}(E_{ij})$).
   - **Container Capacity:** Logical bytes plus dynamic memory allocator overhead and vector capacity reservations ($\sum \text{capacity} \times \text{sizeof}(T)$).
   - **Physical RSS:** Operating system Resident Set Size queried from OS kernel (`GetProcessMemoryInfo` on Windows). RSS includes C runtime heaps, CRT metadata, shared libraries, and heap fragmentation, and does not instantly decrease upon container compaction due to OS page freelist caching.

---

### 2.5 Scientific Evidence Classification System

Every experimental claim in AstroMesh-X is assigned an immutable evidence class:
- **`[MEASURED]`**: Directly observed via physical execution on the target toolchain (e.g. wall-clock time, wire bytes sent over UDP, CTest execution, kernel RSS).
- **`[DERIVED]`**: Calculated deterministically from measured quantities via documented formulas (e.g. ATE RMSE, Cohen's $d$, bootstrap CIs, percentage reductions).
- **`[SYNTHETIC]`**: Measured within the deterministic software simulation environment; representative of algorithmic behavior under simulated sensor noise and network loss, but not physical hardware.
- **`[THEORETICAL]`**: Derived from analytical proofs or asymptotic complexity (e.g. IBLT $O(d)$ peeling bounds, Lie group exponential map exactness, DSU $O(\alpha(n))$ amortized complexity).
- **`[QUALIFIED]`**: Empirical claim subject to documented boundary conditions or limitations (e.g. $FP = 0$ on synthetic test sets).
- **`[UNVERIFIED]`**: Claim requires physical hardware, field robotics, or external infrastructure not currently present in the software-only testbed.

