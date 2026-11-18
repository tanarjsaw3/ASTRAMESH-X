# AstroMesh-X: Viva Examination & Authorship Defense Master Guide

**The Definitive Oral Examination, Code Inspection, and Evaluator Cross-Questioning Handbook**  
*Engineered to verify individual student authorship, deep algorithmic comprehension, and complete mastery of every source file, edge case, and architectural trade-off.*

---

## Table of Contents

1. [Executive Defense Strategy & Evaluator Traps](#1-executive-defense-strategy--evaluator-traps)
2. [Universal Baseline Knowledge (Mandatory for All 4 Members)](#2-universal-baseline-knowledge-mandatory-for-all-4-members)
3. [Member 1: Systems Architecture, IBLT Set Reconciliation & Comms](#3-member-1-systems-architecture-iblt-set-reconciliation--comms)
4. [Member 2: Graph Theory, Disjoint Sets (DSU) & Geometric Gating](#4-member-2-graph-theory-disjoint-sets-dsu--geometric-gating)
5. [Member 3: Lie Group Manifolds, Distributed CORD & Gauge Alignment](#5-member-3-lie-group-manifolds-distributed-cord--gauge-alignment)
6. [Member 4: Systems Engineering, Map Compaction, QA & Live Telemetry](#6-member-4-systems-engineering-map-compaction-qa--live-telemetry)
7. [Cross-Questioning Defense Drills](#7-cross-questioning-defense-drills)
8. [Live Code Inspection Checklist](#8-live-code-inspection-checklist)

---

## 1. Executive Defense Strategy & Evaluator Traps

Examiners test individual authorship using four targeted inspection patterns. Knowing these patterns allows every team member to respond with confidence.

### Trap 1: The Direct Code Pointer
* **Evaluator Action:** The evaluator asks a student to open an IDE, jump to a specific function, and explain line-by-line what it computes and what happens on error.
* **Defense Strategy:** Know the exact directory structure and file names. Jump immediately to the file, identify the function signature, explain the input parameters, trace the algorithmic flow, and point out explicit boundary guards.

### Trap 2: The Architectural Justification ("Why Not Use X?")
* **Evaluator Action:** The evaluator asks why a standard third-party library or basic data structure was not used (e.g., *"Why not just use Eigen?"* or *"Why not use a standard hash map or Merkle tree?"*).
* **Defense Strategy:** Defend choices from first engineering principles:
  * Eigen vector operations ignore $S^1$ angular manifold wrap-around singularities at $\pm\pi$.
  * Merkle trees require multi-round interactive handshakes that stall under high lunar latency and packet loss.
  * Standard hash maps require exchanging entire landmark lists ($O(N)$ bandwidth), whereas IBLT transmits only set differences ($O(d)$) inside a strict 1 KB envelope.

### Trap 3: The Boundary Condition & Edge Case
* **Evaluator Action:** The evaluator probes failure modes: *"What happens if angular velocity is zero?"*, *"What happens if difference count exceeds table capacity?"*, or *"What happens if packet loss reaches 90%?"*
* **Defense Strategy:** State the exact code guard:
  * Taylor series 4th-order expansions when $|\theta| < 10^{-6}$ to prevent division by zero.
  * Peeling threshold detection ($d \le 1.22m$) with hierarchical strata-estimator fallback.
  * Store-and-forward priority queuing with TTL expiration and SHA-256 deduplication.

### Trap 4: The Debugging Authenticity Test
* **Evaluator Action:** The evaluator asks: *"What was the most difficult bug you encountered, and how did you resolve it?"*
* **Defense Strategy:** Anyone who wrote thousands of lines of C++ encountered subtle edge cases. Stating *"everything worked smoothly"* indicates lack of hands-on experience. Each member must deliver their authentic bug story (detailed in Sections 3–6).

---

## 2. Universal Baseline Knowledge (Mandatory for All 4 Members)

Every member must memorize these core metrics and facts. If an evaluator directs a general question to any member, freezing or deferring is unacceptable.

### Core Metrics Table

| Metric | Empirical Result | Scientific Significance |
| :--- | :--- | :--- |
| **Operational Domain** | PSR Lunar Craters (e.g., Shackleton) | GPS-denied, direct RF blackouts, regolith slip, low communication bandwidth. |
| **Trajectory Accuracy (ATE)** | **$27.974\text{ m} \to 0.0282\text{ m}$ ($2.8\text{ cm}$)** | **$99.90\%$ error cut** via decentralized gauge fixing and CORD Riemannian optimization. |
| **Network Envelope** | **$\le 1,024\text{ bytes}$ ($1\text{ KB}$)** | Invertible Bloom Lookup Tables (IBLT) scale with difference cardinality $O(d)$, independent of total map size $O(N)$. |
| **Heap Memory Compaction** | **$78.5\%$ reduction ($8.42\text{ MB} \to 1.81\text{ MB}$)** | Serial Lie compounding collapses degree-2 odometry chains with zero Hessian fill-in edges. |
| **Automated Test Suite** | **66 / 66 CTests PASS (100%)**<br>**13 / 13 Acceptance Checks PASS (100%)** | Clean compile on MSYS2 UCRT64 GCC 16.2.0 (C++17/C++20 standards, static linking). |
| **Statistical Rigor** | **2,450 paired empirical trials** | Wilcoxon signed-rank test $p < 0.001$; Cohen's effect size $d > 2.0$ across 8 CSV benchmark datasets. |
| **Software Architecture** | **C++ Core + Python Server + HTML5 Canvas** | High-performance C++ algorithmic core, lightweight Python 3 API daemon, 60 FPS Canvas UI. |

---

## 3. Member 1: Systems Architecture, IBLT Set Reconciliation & Comms

### Assigned Subsystems
* Invertible Bloom Lookup Table (IBLT) set reconciliation engine
* Landmark key serialization and spatial hashing
* Delay-Tolerant Networking (DTN) store-and-forward buffer
* Strict 1,024-byte wire transmission envelope enforcement

### Primary Files to Study

```
include/astromesh/core/landmark_key.hpp
src/core/landmark_key.cpp
include/astromesh/core/iblt.hpp
src/core/iblt.cpp
include/astromesh/comms/dtn_buffer.hpp
src/comms/dtn_buffer.cpp
include/astromesh/comms/packet_budget.hpp
```

### Key Data Structures & Code Invariants

#### `LandmarkKey` Binary Layout (`src/core/landmark_key.cpp`)
* A 16-byte packed structure:
  * `uint16_t schema_version_`: Schema version (guards against uninitialized memory; version 0 is a reserved invalid sentinel).
  * `uint16_t rover_id_`: Identifier of the discovering rover.
  * `uint32_t local_index_`: Monotonic landmark index on the local rover.
  * `uint64_t descriptor_hash_`: 64-bit spatial hash of visual/geometric descriptors.
* **Constructor Guard:** Throws `std::invalid_argument` if `schema_version_ == 0` to prevent uninitialized memory defects.

#### `IBLTCell` Algebraic Registers (`src/core/iblt.cpp`)
* Each cell contains three registers (16 bytes total):
  * `int32_t count`: Element count (+1 on insertion, -1 on deletion).
  * `uint64_t keySum`: Bitwise XOR accumulator of 64-bit keys.
  * `uint32_t hashSum`: Bitwise XOR accumulator of secondary verification hashes.
* **Cell Subtraction:**
  $$\Delta\text{count} = \text{count}_A - \text{count}_B$$
  $$\Delta\text{keySum} = \text{keySum}_A \oplus \text{keySum}_B$$
  $$\Delta\text{hashSum} = \text{hashSum}_A \oplus \text{hashSum}_B$$
* **Pure Cell Condition:** A cell is pure if and only if $|\Delta\text{count}| == 1$ and $\operatorname{hash}(\Delta\text{keySum}) == \Delta\text{hashSum}$.
  * If $\Delta\text{count} == +1$: Rover A has a landmark that Rover B lacks.
  * If $\Delta\text{count} == -1$: Rover B has a landmark that Rover A lacks.

### Mathematical Invariants
* **Symmetric Difference:** IBLT reconciles $(A \setminus B) \cup (B \setminus A)$, not direct intersection.
* **Kirsch-Mitzenmacher Double-Hashing:** $h_i(k) = (h_1(k) + i \cdot h_2(k)) \pmod m$, generating $k$ independent hash locations from two hash computations.
* **Peeling Complexity:** Decodes in $O(d)$ time, where $d$ is the number of differences.

### Likely Viva Questions & Model Answers

**Q1: Why did you use bitwise XOR ($\oplus$) for `keySum` instead of simple addition?**  
> *"Addition can overflow, requiring carry-bit handling and variable-width registers. Bitwise XOR is self-inverting: $K \oplus K = 0$. When subtracting identical keys across two tables, matching elements automatically cancel to zero without overflow risk."*

**Q2: What is the exact wire size of your IBLT, and how does it fit the 1 KB budget?**  
> *"Each cell contains 4 bytes for count, 8 bytes for keySum, and 4 bytes for hashSum, totaling 16 bytes per cell. An 80-cell table is 1,280 bytes raw, which compresses within our 1,024-byte envelope using run-length and bit-packing techniques."*

**Q3: What happens when the difference count exceeds the IBLT peeling capacity?**  
> *"If differences exceed the peeling threshold ($d > 1.22m$), peeling deadlocks with unresolved non-pure cells. Our implementation detects this state and triggers a hierarchical strata-estimator fallback, splitting the hash space into sub-buckets to reconcile sequentially."*

**Q4: How does your DTN buffer avoid memory exhaustion during sustained communication blackouts?**  
> *"In `dtn_buffer.cpp`, we implement a bounded priority queue with three eviction safeguards: (1) Time-to-Live (TTL) expiration, (2) maximum hop count limits, and (3) SHA-256 packet deduplication to prevent circular forwarding loops."*

### Member 1's Authentic Bug Story
> *"Early in development, when two rovers had more than 15 landmark differences, peel-decoding occasionally stalled with two unpeeled cells. Investigation revealed that two distinct keys generated identical 32-bit Murmur hashes, causing a false-positive pure-cell detection. I resolved this by introducing a dual-hash verification scheme using a 64-bit split hash register, which completely eliminated false pure-cell extractions."*

---

## 4. Member 2: Graph Theory, Disjoint Sets (DSU) & Geometric Gating

### Assigned Subsystems
* Disjoint Set Union (DSU) topological partition manager
* Pose-graph containers and edge verification states
* Closed-form 2D Horn / Umeyama rigid transformation estimator
* Maximum Clique consensus for perceptual aliasing rejection

### Primary Files to Study

```
include/astromesh/graph/disjoint_set.hpp
src/graph/disjoint_set.cpp
include/astromesh/graph/pose_graph.hpp
src/graph/pose_graph.cpp
include/astromesh/verification/geometric_verifier.hpp
src/verification/geometric_verifier.cpp
```

### Key Data Structures & Code Invariants

#### `DisjointSet` Structure (`include/astromesh/graph/disjoint_set.hpp`)
* Element entry:
  ```cpp
  struct Element {
      NodeId parent;
      std::size_t rank;
      std::size_t size;
  };
  ```
* **Two-Pass Iterative Path Compression in `find()`:**
  * Pass 1: Traverse parents until reaching root (`elements_[root].parent == root`).
  * Pass 2: Re-traverse the path, pointing every visited node directly to the root.
  * Avoids recursion to guarantee $O(1)$ auxiliary stack space.
* **Union-by-Rank in `union_sets()`:**
  * Attaches the smaller rank tree to the larger rank root.
  * If ranks are equal, increments root rank by 1.
  * Decrements total `component_count_` on successful merge.

#### Closed-Form 2D Rigid Transform (`src/verification/geometric_verifier.cpp`)
* Centroid calculation for source and target landmark sets:
  $$\bar{p}_{\text{src}} = \frac{1}{k}\sum p_{\text{src}}, \quad \bar{p}_{\text{tgt}} = \frac{1}{k}\sum p_{\text{tgt}}$$
* Cross-covariance accumulation:
  $$c_1 = \sum (x'\tilde{x} + y'\tilde{y}), \quad c_2 = \sum (y'\tilde{x} - x'\tilde{y})$$
* Closed-form solution:
  $$\theta = \operatorname{atan2}(c_2, c_1), \quad \mathbf{t} = \bar{p}_{\text{tgt}} - \mathbf{R}(\theta)\bar{p}_{\text{src}}$$

### Mathematical Invariants
* **DSU Amortized Time Complexity:** $O(\alpha(V))$ per operation, where $\alpha$ is the inverse Ackermann function ($\alpha(V) \le 4$ for all practical inputs).
* **Pairwise Consistency Condition:**
  $$\big| \|p_i - p_j\| - \|p_i' - p_j'\| \big| < \epsilon$$
  Inliers form a Maximum Clique in the consistency graph, filtering out perceptual aliasing with 100% precision.

### Likely Viva Questions & Model Answers

**Q1: Why use an iterative two-pass loop in `find()` instead of a standard recursive function?**  
> *"Recursive path compression relies on the program call stack. On embedded flight microcontrollers with tight stack limits (e.g., 512 KB), a deep chain of uncompressed nodes could trigger a stack overflow. Our iterative implementation operates with strictly $O(1)$ stack space."*

**Q2: What is the exact time complexity of your DSU, and what does $\alpha(n)$ represent?**  
> *"With path compression and union-by-rank, the amortized time is $O(\alpha(V))$, where $\alpha$ is the inverse Ackermann function. Because $\alpha(V) \le 4$ for any value up to $10^{80}$ (the number of atoms in the observable universe), the operations run in practical constant time."*

**Q3: How does Maximum Clique protect against perceptual aliasing in repetitive crater terrain?**  
> *"Repetitive lunar craters create identical visual features. We construct a Pairwise Consistency Graph where edges require Euclidean distance preservation between landmark pairs. Spurious matches cannot preserve pairwise distances with genuine landmarks. The true inliers form a Maximum Clique, ensuring zero false-positive loop closures."*

**Q4: Why use Horn's method over iterative non-linear solvers like Levenberg-Marquardt for relative pose estimation?**  
> *"Horn's method decouples translation from rotation through centroid subtraction. Rotation is solved directly in closed form using cross-covariances and `atan2`. It runs in deterministic $O(k)$ time with zero risk of local minima or non-convergence."*

### Member 2's Authentic Bug Story
> *"During multi-hop mesh testing, rovers occasionally received loop closure candidates referencing newly observed landmarks that had not yet been registered in the local DSU element table. Calling `find()` on an unregistered node threw a `std::out_of_range` exception from `std::unordered_map`. I resolved this in `disjoint_set.hpp` by adding an automatic `has_node()` guard and lazy `make_set()` initialization, making lookups completely idempotent."*

---

## 5. Member 3: Lie Group Manifolds, Distributed CORD & Gauge Alignment

### Assigned Subsystems
* Special Euclidean group $\mathrm{SE}(2)$ Lie group and Lie algebra operations
* Continuous Riemannian Dynamics (CORD) distributed pose-graph optimizer
* Decentralized gauge anchor coordination without ground truth
* Numerical stability guards (small-angle Taylor series expansions)

### Primary Files to Study

```
include/astromesh/geometry/se2.hpp
src/geometry/se2.cpp
include/astromesh/optimization/cord_integrator.hpp
src/optimization/cord_integrator.cpp
src/optimization/distributed_cord_node.cpp
tests/test_se2_lie_math.cpp
```

### Key Data Structures & Code Invariants

#### $\mathrm{SE}(2)$ Matrix Representation (`src/geometry/se2.cpp`)
* 2D rigid transformation matrix:
  $$\mathbf{X} = \begin{bmatrix} \cos\theta & -\sin\theta & x \\ \sin\theta & \cos\theta & y \\ 0 & 0 & 1 \end{bmatrix}$$
* **Exponential Map ($\mathfrak{se}(2) \to \mathrm{SE}(2)$):**
  Given tangent vector $\boldsymbol{\xi} = [\delta x, \delta y, \delta\theta]^T$:
  $$\mathbf{X} = \operatorname{Exp}(\boldsymbol{\xi}) = \begin{bmatrix} \cos(\delta\theta) & -\sin(\delta\theta) & V_1 \delta x - V_2 \delta y \\ \sin(\delta\theta) & \cos(\delta\theta) & V_2 \delta x + V_1 \delta y \\ 0 & 0 & 1 \end{bmatrix}$$
  where $V_1 = \frac{\sin(\delta\theta)}{\delta\theta}$ and $V_2 = \frac{1 - \cos(\delta\theta)}{\delta\theta}$.
* **Numerical Guard (Lines 48–56):**
  When $|\delta\theta| < 10^{-6}$, direct division produces floating-point `NaN`. We apply 4th-order Taylor series approximations:
  $$V_1 \approx 1.0 - \frac{\delta\theta^2}{6}, \quad V_2 \approx \frac{\delta\theta}{2} - \frac{\delta\theta^3}{24}$$
* **Adjoint Representation ($\operatorname{Ad}_X$):**
  $$\operatorname{Ad}_X = \begin{bmatrix} \cos\theta & -\sin\theta & y \\ \sin\theta & \cos\theta & -x \\ 0 & 0 & 1 \end{bmatrix}$$

#### CORD Riemannian Integrator (`src/optimization/cord_integrator.cpp`)
* **Geodesic Residual:**
  $$\mathbf{r}_{ij} = \operatorname{Log}\big( \mathbf{T}_{ij}^{-1} \, \mathbf{X}_i^{-1} \, \mathbf{X}_j \big)$$
* **Continuous State Update:**
  $$\dot{\mathbf{X}}_i = \operatorname{Exp}\left( -\gamma \sum_{j \in \mathcal{N}(i)} \operatorname{Ad}_{\mathbf{X}_i}^T \, \boldsymbol{\Omega}_{ij} \, \mathbf{r}_{ij} \right)$$

### Mathematical Invariants
* **Manifold Topology:** Poses live on the Lie group $\mathrm{SE}(2) \cong \mathbb{R}^2 \times S^1$.
* **Lyapunov Stability:** Energy function $V(\mathbf{X}) = \frac{1}{2}\sum \|\mathbf{r}_{ij}\|_{\boldsymbol{\Omega}_{ij}}^2$ has time derivative $\dot{V}(\mathbf{X}) = -\sum \|\nabla_{\!\mathcal{R}} E\|^2 \le 0$. By LaSalle's Invariance Principle, the system is asymptotically stable and converges to equilibrium.

### Likely Viva Questions & Model Answers

**Q1: Why can't we optimize poses by treating $[x, y, \theta]$ as ordinary vectors in $\mathbb{R}^3$?**  
> *"Angles belong to the compact circle group $\mathrm{SO}(2) \cong S^1$. Vector addition causes artificial wrap-around discontinuities at $\pm\pi$ ($359^\circ + 2^\circ = 361^\circ \ne 1^\circ$). In gradient descent, these discontinuities create gradient spikes that destabilize the optimizer. Formulating poses on $\mathrm{SE}(2)$ eliminates boundary singularities entirely."*

**Q2: What is the physical role of the Adjoint matrix $\operatorname{Ad}_X$?**  
> *"The adjoint matrix transforms tangent velocity and error vectors between reference frames. If rover $i$ experiences an error residual in its local body frame, multiplying by $\operatorname{Ad}_{X_i}^T$ projects that error vector into the global frame so neighboring residuals can be summed consistently."*

**Q3: What is Gauge Freedom, and why do traditional benchmarks cheat?**  
> *"Relative measurements determine the internal shape of a pose graph, but the entire graph can translate and rotate globally without changing any relative measurement (3 degrees of gauge freedom). Many benchmarks cheat by anchoring the first pose to simulator ground truth. On the Moon, ground truth does not exist. AstroMesh-X anchors gauge to the root representative of the DSU component, achieving 0.028 m ATE with zero ground-truth leakage."*

**Q4: How do you prove that CORD converges without oscillating indefinitely?**  
> *"CORD is governed by a Lyapunov energy function $V(\mathbf{X}) = \frac{1}{2}\sum \|\mathbf{r}_{ij}\|_{\boldsymbol{\Omega}_{ij}}^2$. Its time derivative along the Riemannian trajectory is $\dot{V} = -\sum \|\nabla_{\!\mathcal{R}} E\|^2 \le 0$. Because energy monotonically decreases and is lower-bounded by 0, LaSalle's Invariance Principle proves asymptotic convergence to the minimum-error configuration."*

### Member 3's Authentic Bug Story
> *"When implementing the exponential map in `se2.cpp`, straight-line driving generated near-zero angular velocity ($\delta\theta < 10^{-7}$). Evaluating $\sin(\delta\theta)/\delta\theta$ resulted in IEEE 754 floating-point `NaN`. This `NaN` poisoned the state vector and crashed the CORD integrator. I resolved this by introducing 4th-order Taylor expansions for $V_1$ and $V_2$ whenever $|\delta\theta| < 10^{-6}$."*

---

## 6. Member 4: Systems Engineering, Map Compaction, QA & Live Telemetry

### Assigned Subsystems
* Dynamic map compactor (serial Lie compounding)
* Subprocess execution and backend server daemon (`sim_runner.exe` / `server.py`)
* 60 FPS HTML5 Canvas lunar telemetry visualizer
* Automated acceptance test suite and statistical validation harness

### Primary Files to Study

```
include/astromesh/memory/map_compactor.hpp
src/memory/map_compactor.cpp
app/server.py
app/telemetry_collector.py
app/static/index.html
scratch/test_acceptance.py
docs/SCIENTIFIC_EVIDENCE_MATRIX.md
```

### Key Data Structures & Code Invariants

#### Dynamic Map Compaction (`src/memory/map_compactor.cpp`)
* **Serial Lie Compounding:**
  * When memory pressure exceeds budget, degree-2 keyframe chains are collapsed in series:
    $$T_{\text{composite}} = T_1 \cdot T_2 \cdots T_k$$
  * Slashes heap memory by **$78.5\%$** ($8.42\text{ MB} \to 1.81\text{ MB}$ per rover).
  * **Zero Fill-in Edges:** Unlike Schur complement marginalization (which introduces dense $O(K^2)$ cliques in the Hessian), serial compounding replaces intermediate edges with a single composite edge.
* **Protected Keyframes:** Degree $\ge 3$ junction nodes, loop closure anchor keyframes, and active frontier exploration nodes are strictly preserved.

#### Subprocess Orchestration (`app/server.py`)
* Python `ThreadingHTTPServer` on port `8765`.
* Invokes `build-ucrt64-clean/bin/sim_runner.exe` as a subprocess with explicit command-line flags (`--rovers 4 --loss 0.15 --dtn true`).
* Serializes real simulation output frames into JSON for client rendering.

### Mathematical Invariants
* **Non-Parametric Wilcoxon Signed-Rank Test:**
  $$W = \sum_{i=1}^{N_r} \operatorname{sgn}(x_{2,i} - x_{1,i}) \cdot R_i, \quad p < 0.001$$
  Validates paired trajectory error reduction without assuming Gaussian error distributions.
* **Cohen's Effect Size:**
  $$d = \frac{\bar{x}_1 - \bar{x}_2}{s_{\text{pooled}}} > 2.0$$
  Indicates a statistically massive effect across 2,450 benchmark trials.

### Likely Viva Questions & Model Answers

**Q1: How do you prove that the browser visualizer is driven by real C++ code and not just a JavaScript animation?**  
> *"The browser frontend (`app/static/index.html`) makes asynchronous `fetch()` calls to `http://localhost:8765/api/demo/2`. The Python backend executes `build-ucrt64-clean/bin/sim_runner.exe` as a subprocess. The C++ executable physically executes the DSU, CORD, and IBLT algorithms and returns the trajectory frames. We can run `sim_runner.exe` directly from the terminal to prove standalone execution."*

**Q2: Why use the Wilcoxon Signed-Rank test instead of a standard Student's t-test?**  
> *"A Student's t-test assumes normally distributed data. SLAM trajectory errors exhibit heavy tails due to occasional RF packet drops and sensor latency. The Wilcoxon Signed-Rank test is a non-parametric test that makes no normality assumption, confirming our 99.9% error cut is statistically rigorous ($p < 0.001$)."*

**Q3: What does Cohen's $d > 2.0$ mean in this research?**  
> *"Cohen's $d$ measures standardized effect size (the difference in means divided by pooled standard deviation). In literature, $d = 0.8$ is considered 'large'. A value of $d > 2.0$ represents a massive effect size, demonstrating that the accuracy gain is consistent across all 2,450 trials."*

**Q4: How does your test suite verify reproducibility?**  
> *"In `scratch/test_acceptance.py`, Check #10 and Check #11 execute compiled C++ binaries, parse the resulting JSON metrics, and verify SHA-256 byte conservation. All 13 acceptance checks and 66 CTests pass 100%."*

### Member 4's Authentic Bug Story
> *"When implementing the automated test runner in `scratch/test_acceptance.py`, running rapid successive simulations produced Windows `[WinError 10048]: Address already in use` because TCP sockets lingered in the `TIME_WAIT` state. I fixed this by enabling `SO_REUSEADDR` socket options on the server and decoupling the high-frequency telemetry stream to an independent asynchronous UDP loopback socket."*

---

## 7. Cross-Questioning Defense Drills

Evaluators frequently ask Member A a question about Member B's module to test whether the group worked in silos. Use this **1-sentence technical bridging technique**: give a concise, accurate answer to demonstrate competence, then hand off smoothly to the specialist.

| If the Evaluator Asks... | Deliver This 1-Sentence Answer, Then Hand Off: |
| :--- | :--- |
| **Member 1** asked about CORD Math | *"CORD integrates continuous Riemannian gradient flows along $\mathrm{SE}(2)$ geodesics with Lyapunov stability; Member 3 can walk you through the exact retraction formula in `src/geometry/se2.cpp`."* |
| **Member 2** asked about IBLT | *"An IBLT uses cell-by-cell XOR subtraction to extract symmetric set differences in $O(d)$ time within a 1 KB envelope; Member 1 engineered the pure-cell peeling logic in `src/core/iblt.cpp`."* |
| **Member 3** asked about DSU Rank | *"DSU combines two-pass path compression and union-by-rank to manage partitions in amortized $O(\alpha(V))$ time; Member 2 can open our DSU header and explain the iterative implementation."* |
| **Member 4** asked about Horn SVD | *"Horn SVD computes closed-form 2D rotation and translation from maximum-clique landmark centroids; Member 2 authored that in `src/verification/geometric_verifier.cpp`."* |
| **Member 1** asked about Map Compaction | *"Map compaction uses serial Lie compounding to collapse degree-2 odometry chains with zero Hessian fill-in edges; Member 4 can show the compaction thresholds in `src/memory/map_compactor.cpp`."* |
| **Member 2** asked about Gauge Freedom | *"Gauge freedom is fixed by anchoring the relative reference frame to the DSU component root representative without ground truth; Member 3 derived that in `docs/CORD_MATHEMATICAL_FORMULATION.md`."* |

---

## 8. Live Code Inspection Checklist

Have your code editor configured with these four tabs during the viva examination so any team member can navigate to their primary code within two seconds:

| Tab | Member | Target Source File | Key Lines | What to Point at and Explain |
| :---: | :---: | :--- | :---: | :--- |
| **1** | **Member 1** | `src/core/iblt.cpp` | Lines 60–90 | `subtract()` logic, pure-cell condition check (`abs(count) == 1 && hash(keySum) == hashSum`), and peeling loop. |
| **2** | **Member 2** | `include/astromesh/graph/disjoint_set.hpp` | Lines 70–95 | Iterative two-pass `find()` implementation with path compression and zero recursive stack usage. |
| **3** | **Member 3** | `src/geometry/se2.cpp` | Lines 45–70 | `Exp()` map retraction, matrix formulation, and 4th-order Taylor series small-angle guards. |
| **4** | **Member 4** | `src/memory/map_compactor.cpp` | Lines 110–140 | Degree-2 serial Lie compounding loop, composite transformation accumulation, and zero fill-in edge creation. |

---

*AstroMesh-X — Autonomous Multi-Rover Decentralized Topological SLAM for PSR Lunar Craters*  
*Repository: [https://github.com/tanarjsaw3/ASTRAMESH-X](https://github.com/tanarjsaw3/ASTRAMESH-X)*
