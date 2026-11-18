# 🛰️ AstroMesh-X: 4-Member Project Responsibilities & Defense Guide

This document defines the clear technical ownership, presentation segments, source code files, and anticipated viva examination topics for each of the 4 team members presenting **AstroMesh-X**.

---

## 📋 Team Division Matrix Overview

| Member | Engineering Role | Core Subsystems Owned | Code Directories | Key Mathematical / DSA Concepts |
| :--- | :--- | :--- | :--- | :--- |
| **Member 1** | **Systems Architect & Comms** | Mission Overview, IBLT Set Reconciliation, DTN Buffering | `src/core/`, `src/comms/`, `include/astromesh/comms/` | Invertible Bloom Lookup Tables (IBLT), $O(d)$ Peel-Decoding, 1 KB Wire Envelope |
| **Member 2** | **Graph Algorithms & Geometry** | Partition Tracking, Loop Closure, Geometric Verification | `src/graph/`, `src/verification/`, `include/astromesh/verification/` | Disjoint-Set Union (DSU) $O(\alpha(V))$, Maximum-Clique Graph, Horn SVD |
| **Member 3** | **Manifold Math & Optimization** | $\mathrm{SE}(2)$ Lie Algebra, Distributed CORD, Gauge Alignment | `src/geometry/`, `src/optimization/`, `tests/test_se2_lie_math.cpp` | Lie Groups $\mathrm{SE}(2)$, Retraction Mappings, Continuous Riemannian Dynamics |
| **Member 4** | **Systems, Telemetry & QA** | Mission Control Dashboard, CTest Suites, Empirical Benchmarks | `app/`, `scratch/test_acceptance.py`, `docs/SCIENTIFIC_EVIDENCE_MATRIX.md` | UDP Telemetry, 66 CTests, ATE Accuracy (0.028 m), Wilcoxon Signed-Rank |

---

## 👤 MEMBER 1: Lead Systems Architect & Probabilistic Structures

### 1. Presentation Ownership (Minutes 00:00 – 08:00)
- **The Extraterrestrial Challenge:**
  - Introduce the problem: Autonomous multi-rover swarms exploring lunar craters (Shackleton) or lava tubes (Marius Hills) with **no GPS and zero base stations**.
  - Explain why existing multi-robot SLAM collapses: Traditional systems stream raw point clouds or keyframe arrays across megabytes of bandwidth, crashing low-power space radio links ($< 100\text{ kbps}$).
- **The 1 KB Wire Solution:**
  - Introduce AstroMesh-X's constant-size set reconciliation paradigm.
  - Explain the Invertible Bloom Lookup Table (IBLT) algebraic structure: `(count, keySum, hashSum)`.
  - Explain how subtracting two tables cell-by-cell ($B_{\text{diff}} = B_i - B_j$) yields the symmetric difference $A \Delta B = (A \setminus B) \cup (B \setminus A)$.
  - Detail $O(d)$ peel-decoding of pure cells ($\text{count} = \pm 1$).
- **Delay-Tolerant Networking (DTN):**
  - Explain how packets are buffered, prioritized, and routed over multi-hop mesh links during line-of-sight RF shadows.

### 2. Primary Source Code to Study
- `src/core/landmark_key.cpp` — 64-bit spatial hashing and landmark key generation.
- `src/comms/dtn_buffer.cpp` — Delay-tolerant priority message store-and-forward buffers.
- `include/astromesh/comms/packet_budget.hpp` — Strict 1,024-byte packet envelope enforcement.

### 3. Key Viva / Defense Questions to Master
- **Q: Why use an IBLT instead of a standard Bloom Filter or Counting Bloom Filter?**
  - *Answer:* A standard Bloom filter only answers set membership with false positives; it cannot extract elements. A Counting Bloom filter handles deletions but still cannot extract missing items. An IBLT stores algebraic XOR sums that allow extracting the exact set difference without sending the underlying data.
- **Q: What happens if the symmetric difference exceeds the 1 KB IBLT capacity?**
  - *Answer:* Peel-decoding stops with residual non-pure cells. AstroMesh-X safely catches this condition and falls back to a hierarchical strata-estimator partition to resolve the differences in smaller sub-buckets.

---

## 👤 MEMBER 2: Graph Theory & Geometric Verification Specialist

### 1. Presentation Ownership (Minutes 08:00 – 16:00)
- **Dynamic Swarm Partition Tracking (DSU):**
  - Explain how the swarm tracks connected components during RF blackouts and dynamic re-connections.
  - Explain the two classic optimizations of Disjoint-Set Union (DSU):
    1. **Path Compression** in `find(x)`: Flattens the component tree so all nodes point directly to the root.
    2. **Union by Rank** in `union(x, y)`: Attaches the smaller tree under the taller tree to minimize tree depth.
  - State the time complexity: $O(\alpha(V))$ amortized, where $\alpha$ is the inverse Ackermann function ($\alpha(V) \le 4$, effectively $O(1)$).
- **Perceptual Aliasing in Planetary Environments:**
  - Explain why repetitive crater regolith causes false-positive loop closures that ruin standard SLAM maps.
- **Maximum-Clique Geometric Consistency Gating:**
  - Formulate the **Pairwise Consistency Graph** $G_c = (V_c, E_c)$ where nodes are candidate landmark matches.
  - Show how edges are defined by rigid Euclidean distance preservation:
    $$\big| \|p_1 - p_2\| - \|p_1' - p_2'\| \big| < \epsilon_{\text{dist}}$$
  - Explain that the true set of inliers corresponds to the **Maximum Clique**.
  - Show how Horn singular value decomposition (SVD) computes the rigid transformation matrix and rejects imposters.

### 2. Primary Source Code to Study
- `include/astromesh/graph/disjoint_set.hpp` — Disjoint Set Union implementation.
- `src/verification/geometric_verifier.cpp` — Pairwise consistency graph and maximum-clique filter.
- `include/astromesh/verification/geometric_verifier.hpp` — Gating parameters and TEASER++ Horn alignment.

### 3. Key Viva / Defense Questions to Master
- **Q: Isn't Maximum Clique an NP-hard problem? How does it run in real-time on rovers?**
  - *Answer:* While general Maximum Clique is NP-hard, the correspondence graph in local scan matching is sparse and small (typically $< 30$ candidate matches). We use fast degree-based pruning and TEASER++ spectral relaxation, converging in polynomial time.
- **Q: How does DSU handle radio link disconnects?**
  - *Answer:* Standard DSU is an incremental merge structure. We combine DSU with timestamped version vectors. When a link drops, topological components are isolated; upon re-establishing contact, DSU reconciles the components in $O(\alpha(V))$ time.

---

## 👤 MEMBER 3: Manifold Mathematics & Optimization Specialist

### 1. Presentation Ownership (Minutes 16:00 – 24:00)
- **The $\mathrm{SE}(2)$ Lie Group Formulation:**
  - Explain why vector Euclidean coordinates $(x, y, \theta)$ fail in robotics: Rotations $\theta \in \mathrm{SO}(2)$ live on a circle $S^1$. Direct angle addition causes gimbal wrap-around errors (e.g., $359^\circ + 2^\circ \ne 361^\circ$).
  - Write down the Special Euclidean Lie group matrix:
    $$X_k = \begin{bmatrix} R(\theta_k) & p_k \\ 0 & 1 \end{bmatrix} \in \mathrm{SE}(2)$$
  - Explain the **Exponential Map** ($\mathfrak{se}(2) \to \mathrm{SE}(2)$) and **Logarithmic Map** ($\mathrm{SE}(2) \to \mathfrak{se}(2)$).
- **Continuous Riemannian Dynamics (CORD):**
  - Formulate the Riemannian residual on the Lie group: $r_{ij} = \mathrm{Log}(T_{ij}^{-1} X_i^{-1} X_j)$.
  - Explain the continuous-time gradient flow on the Riemannian manifold:
    $$\dot{X}_i = -\nabla_{\!\mathcal{R}} E(X_i) = \mathrm{Exp}\left(-\gamma \sum_{j \in \mathcal{N}(i)} \mathrm{Ad}_{X_i}^T \Omega_{ij} r_{ij}\right)$$
  - Emphasize that CORD has proven **Lyapunov stability**, guaranteeing asymptotic convergence without a central server.
- **Decentralized SE(2) Gauge Fixing (Zero Ground-Truth Leakage):**
  - Explain that distributed SLAM graphs have 3 unconstrained degrees of gauge freedom (global rotation and translation).
  - Highlight that AstroMesh-X fixes gauge by anchoring the relative reference frame to the **root node of the DSU component**, eliminating drift down to **0.028 m** with **zero ground-truth leakage**.

### 2. Primary Source Code to Study
- `src/geometry/se2_lie.cpp` & `tests/test_se2_lie_math.cpp` — Lie group exponential/logarithmic maps and adjoint representations.
- `src/optimization/cord_optimizer.cpp` — Continuous Riemannian dynamics distributed optimizer.
- `tests/benchmark_m713_gauge_alignment.cpp` — Ground-truth independent gauge anchoring.

### 3. Key Viva / Defense Questions to Master
- **Q: What is the physical role of the Adjoint operator $\mathrm{Ad}_X$?**
  - *Answer:* The Adjoint representation $\mathrm{Ad}_X$ transforms tangent vectors (Lie algebra velocities/perturbations) from one coordinate frame to another across relative pose transformations.
- **Q: Why is CORD decentralized rather than centralized?**
  - *Answer:* In CORD, each rover only exchanges relative residuals $r_{ij}$ with its immediate 1-hop topological neighbors $\mathcal{N}(i)$. There is no central master node, avoiding single points of failure.

---

## 👤 MEMBER 4: Systems Engineering, Live Simulation & Empirical Verification

### 1. Presentation Ownership (Minutes 24:00 – 35:00)
- **Live Terminal Verification Execution:**
  - Execute the full C++ test runner live in front of the judge:
    ```powershell
    ctest --test-dir build-ucrt64-clean --output-on-failure
    ```
    Highlight: **66 / 66 tests passing (100% in ~12.78s)**.
  - Run the automated 13-point acceptance test suite:
    ```powershell
    python scratch/test_acceptance.py
    ```
    Highlight: **13 / 13 acceptance checks PASS**.
- **Interactive Mission Control Live Demonstration:**
  - Open the live dashboard at [http://localhost:8765](http://localhost:8765).
  - Walk the judge through the **Live Lunar Arena (60 FPS)** with multi-hop packet routing animations.
  - Trigger **`⚡ Sever Radio (Inject Blackout)`**: Show the severed red link and Rover 2's crimson ghost odometry trajectory peeling away from ground truth ($27.97\text{ m}$ drift).
  - Trigger **`🔗 1 KB IBLT Sync & Heal`**: Highlight the expanding cyan shockwave that instantly snaps the drifted ghost path back to ground truth ($0.028\text{ m}$ ATE, $-99.9\%$ error cut).
- **Statistical Significance & Memory Proof:**
  - Present the empirical matrix: **2,450 paired trials** across 8 canonical benchmark datasets.
  - Wilcoxon signed-rank test ($p < 0.001$) and Cohen's $d > 2.0$ prove massive statistical significance.
  - Heap memory reduction of **$-78.5\%$** via serial Lie compounding with zero Hessian fill-in edges.

### 2. Primary Source Code to Study
- `app/server.py` & `app/static/index.html` — Full-stack mission control server and 60 FPS canvas engine.
- `scratch/test_acceptance.py` — Automated verification harness.
- `docs/SCIENTIFIC_EVIDENCE_MATRIX.md` & `README.md` — Statistical benchmark tables and metrics.

### 3. Key Viva / Defense Questions to Master
- **Q: How can we verify that the web dashboard isn't showing pre-recorded dummy animations?**
  - *Answer:* The backend (`app/server.py`) interfaces directly with the compiled C++20 simulation binary (`build-ucrt64-clean/bin/sim_runner.exe`). Every pose update, DSU component count, and wire byte count is computed in real-time by the native C++ engine.
- **Q: How does the dynamic map compactor prevent memory exhaustion?**
  - *Answer:* Conventional marginalization creates dense fill-in cliques in the Hessian matrix. AstroMesh-X replaces redundant linear keyframe chains with composite relative transformations via serial Lie compounding, achieving a 78.5% heap cut with zero fill-in edges.

---

## ⏱️ Recommended Group Presentation Schedule (60 Minutes)

```
┌─────────────────┬──────────┬─────────────────────────────────────────────────────────────────────────┐
│ Time Segment    │ Speaker  │ Agenda & Demonstration Actions                                          │
├─────────────────┼──────────┼─────────────────────────────────────────────────────────────────────────┤
│ 00:00 – 03:00   │ Member 1 │ Project Hook, Aerospace Lunar Challenge, Real-World SLAM Failure Modes   │
│ 03:00 – 08:00   │ Member 1 │ 1 KB IBLT Algebraic Structure, Peel-Decoding, DTN Mesh Routing           │
│ 08:00 – 12:00   │ Member 2 │ Swarm Partition Tracking via Disjoint-Set Union (DSU with Rank & Path)   │
│ 12:00 – 16:00   │ Member 2 │ Perceptual Aliasing, Pairwise Consistency Graph, Maximum-Clique Gating  │
│ 16:00 – 20:00   │ Member 3 │ SE(2) Lie Group Formulation vs Euclidean Vector Spaces                  │
│ 20:00 – 24:00   │ Member 3 │ Continuous Riemannian Dynamics (CORD) & Ground-Truth Free Gauge Fixing   │
│ 24:00 – 28:00   │ Member 4 │ Live Terminal Verification (66 CTests & 13/13 Acceptance Suite)          │
│ 28:00 – 34:00   │ Member 4 │ Live Mission Control UI Demo: RF Blackout Ghost Drift & 1 KB Shockwave   │
│ 34:00 – 38:00   │ Member 4 │ Empirical Benchmarks (2,450 trials, Wilcoxon p < 0.001, -78.5% Heap)     │
│ 38:00 – 40:00   │ Member 1 │ Summary of Novelty, Patent Proposal Draft, IEEE Research Manuscript     │
│ 40:00 – 60:00   │ ALL      │ Interactive Technical Q&A / Faculty Viva Defense                        │
└─────────────────┴──────────┴─────────────────────────────────────────────────────────────────────────┘
```

---

## 📌 Checklist Before Meeting the Evaluator
- [x] Background server active: [http://localhost:8765](http://localhost:8765).
- [x] Have [README.md](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/README.md) open for the high-level metrics table.
- [x] Have [docs/AstroMesh-X_Patent_Proposal_Draft.docx](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/docs/AstroMesh-X_Patent_Proposal_Draft.docx) open for formal architecture diagrams and claims.
- [x] Have [http://localhost:8765/paper.html](http://localhost:8765/paper.html) open for the IEEE double-column research manuscript.
- [x] Terminal open in `C:\Users\TANISHKA SAWANT\ASTRAMESH-X` ready to run `ctest`.
