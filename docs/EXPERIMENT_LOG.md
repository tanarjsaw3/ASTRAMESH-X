# AstroMesh-X — Experiment Log & Hypothesis Tracker

*This document records scientific hypotheses, experimental protocols, planned simulation trials, and empirical findings. All numerical results must be derived from verifiable, reproducible execution scripts.*

---

## 1. Scientific Hypotheses Under Evaluation

- **Hypothesis H1 (Reconciliation Efficiency):** Under low to moderate map divergence between rovers, an Invertible Bloom Lookup Table (IBLT) will transmit fewer total bytes to reconcile landmark sets than a Counting Bloom Filter (CBF) or direct key exchange, within a target payload envelope of 1 KB per exchange.
- **Hypothesis H2 (Optimization Scalability):** Continuous Riemannian Dynamics (CORD) distributed pose-graph optimization will achieve global trajectory consistency with Absolute Trajectory Error (ATE) within 10% of centralized optimization solvers, without requiring a centralized coordinator or full graph transmission.
- **Hypothesis H3 (Delay Tolerance):** Combining DTN store-and-forward buffering with topological landmark reconciliation will maintain swarm map integrity and bounded ATE (target sub-0.5 m) under packet drop rates up to 70% and communication latency up to 60 seconds.

---

## 2. Planned Experiments Matrix

| Exp ID | Title | Primary Independent Variables | Target Dependent Metrics | Planned Phase | Status |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **E-01** | IBLT vs. CBF Set Difference Scaling | Set difference size $d \in [1, 200]$; key count $N \in [10^2, 10^4]$ | Bandwidth (bytes), Decode success rate (%), False-positive rate (%) | Phase 1 | Planned |
| **E-02** | Geometric Gating Under Severe Perceptual Aliasing | Outlier ratio in candidate matches (0% to 80%) | Precision, Recall, False-acceptance rate | Phase 2 | Planned |
| **E-03** | CORD Convergence on SE(2)/SE(3) Pose Graphs | Swarm size (2, 4, 8 rovers); Graph topologies (Grid, Torus) | Iterations to convergence, Communication cost (bytes/iter), Residual error | Phase 3 | Planned |
| **E-04** | Delay-Tolerant Communication Under Packet Loss | Drop probability $p \in [0.0, 0.9]$; Latency $\tau \in [0, 120s]$ | Sync latency, Message retransmissions, Graph partition count | Phase 4 | Planned |
| **E-05** | Multi-Rover Exploration & ATE Trajectory Accuracy | Exploration policy (Voronoi vs. Connectivity-aware); Swarm size (3–10) | ATE (m), Exploration time, Total bandwidth, Map completeness | Phase 5 | Planned |

---

## 3. Executed Experiments & Measured Results

*No experiments have been executed yet in Phase 0 (Repository Bootstrap).*

| Exp ID | Execution Date | Hardware / Environment | Parameter Configuration | Measured Result Summary | Raw Log Artifact |
| :--- | :--- | :--- | :--- | :--- | :--- |
| — | — | — | — | *None yet (Phase 0)* | — |

---

## 4. Empirical Result Guidelines

When logging experimental results in subsequent phases:
1. Every entry must cite the exact script and commit hash used to run the trial.
2. Random seeds must be explicitly recorded for deterministic reproduction.
3. Raw data (CSV, JSON logs) must be placed in `experiments/data/` or documented in run artifacts.
4. Summary plots must be saved with unambiguous axis labels, units, and confidence intervals.
