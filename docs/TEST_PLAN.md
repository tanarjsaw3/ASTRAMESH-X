# AstroMesh-X — Master Test Plan

This document establishes the verification strategies, test suites, and empirical validation gates for all components of AstroMesh-X. Development follows a test-first methodology: test specifications must precede or accompany implementation.

---

## 1. Test Categorization & Frameworks

| Tier | Scope | Framework | Execution Frequency |
| :--- | :--- | :--- | :--- |
| **Unit Tests (C++)** | Individual algorithmic classes (hashes, IBLT cells, DSU, Lie algebra) | CTest + Catch2 / GoogleTest | Every commit / build |
| **Property Tests (C++)** | Invariant testing (e.g., IBLT invertible decoding properties under random sets) | Custom generative test harness | Nightly / Pre-merge |
| **Integration Tests (C++)** | Multi-module pipelines (e.g. IBLT -> Geometric Gating -> DSU Graph Merge) | CTest integration executables | Phase completion |
| **Simulation Benchmarks (Python)** | Multi-rover swarm execution, packet loss, trajectory estimation | `pytest` + automated benchmark runners | Milestone gates |

---

## 2. Component Verification Specifications

### 2.1 Landmark Keys & Fingerprinting (Phase 1)
- **Deterministic Invariance:** Verify that identical simulated visual/spatial feature vectors produce identical 64-bit/128-bit hashes regardless of machine endianness or memory alignment.
- **Collision Resistance:** Generate $10^6$ synthetic perturbed feature descriptors; verify empirical collision rates remain within theoretical expectations for the chosen hash family (Murmur3 / xxHash).
- **Schema & Rover ID Prefixing:** Ensure cross-rover keys carry namespace headers preventing erroneous self-loop assignment.

### 2.2 Counting Bloom Filter (CBF) Baseline (Phase 1)
- **Insertion & Membership:** Insert $N$ landmark keys; assert 100% true-positive recall on query.
- **False-Positive Tracking:** Query $M$ uninserted random keys; measure empirical false-positive rate (FPR) and verify it matches the theoretical equation:
  $$FPR \approx \left(1 - e^{-k N / m}\right)^k$$
- **Counter Saturation:** Test decrement and deletion operations; verify no underflow occurs on absent keys.

### 2.3 Invertible Bloom Lookup Table (IBLT) Engine (Phase 1)
- **Zero-Difference Identity:** Construct two identical sets $A$ and $B$; subtract $T_{diff} = T_A - T_B$; verify $T_{diff}$ immediately reports 0 differences.
- **Symmetric Difference Recovery:** Construct sets $A$ and $B$ with known symmetric difference $d = |A \oplus B|$. Verify that when $d \le d_{max}$ (decoding capacity), exact set elements $(A \setminus B)$ and $(B \setminus A)$ are fully recovered.
- **Decoding Failure Handling:** Construct sets with $d > d_{max}$; verify that the decoder cleanly signals `DECODE_OVERFLOW_ERROR` rather than returning corrupt or partially hallucinated keys.
- **Negative Entry Arithmetic:** Verify proper sign handling and counter decrements during table subtraction.

### 2.4 Geometric Verification & Loop Closure Gating (Phase 2)
- **True Positive Gating:** Provide pairs of keyframes with known ground-truth relative transforms and noisy landmark observations; verify RANSAC / relative transformation estimator recovers relative pose within $\epsilon \le 0.05$ m, $1.0^\circ$.
- **Perceptual Aliasing Outlier Rejection:** Synthesize visually similar landmark sets placed at disjoint geographic locations; verify the geometric verifier rejects the hypothesis with 0 false-positive acceptance.

### 2.5 Topological Graph & Disjoint Set Union (DSU) (Phase 2)
- **Disjoint Component Tracking:** Initialize $K$ independent subgraphs representing $K$ rovers; verify $K$ distinct sets in DSU.
- **Merge Integrity:** Simulate inter-rover loop closure; verify DSU correctly unites subgraphs and updates relative reference frame transformations without generating cyclic inconsistencies.

### 2.6 Continuous Riemannian Dynamics (CORD) Optimization (Phase 3)
- **Riemannian Gradient Exactness:** Compare analytical Riemannian gradients on $SE(2)$ / $SE(3)$ manifolds against numerical finite-difference approximations.
- **Distributed Convergence:** Run CORD on standard benchmark pose graphs (e.g., 2D Manhattan grid, 3D torus) split across 2–8 simulated rover partitions. Verify cost reduction monotonically approaches the global minimum.
- **Centralized Baseline Parity:** Compare final optimized poses against centralized Gauss-Newton / Levenberg-Marquardt optimization (e.g. GTSAM/g2o reference solutions); verify Absolute Trajectory Error (ATE) difference is within research target bounds.

### 2.7 Delay-Tolerant Networking & Bounded-Memory (Phase 4)
- **Packet Drop Robustness:** Transmit messages across a simulated channel with 10%, 50%, and 90% random packet drops; verify DTN retry logic prevents message loss for high-priority loop closure proposals.
- **Payload Strictness:** Enforce byte-level accounting on serialized packets; assert that discovery packets conform to specified budget targets (evaluating 1 KB, 5 KB, 20 KB configurations).
- **Memory Bounding:** Simulate continuous 10 km exploration trajectory; verify keyframe pruning limits memory footprint to the configured maximum (e.g. $< 50$ MB per rover) without fragmenting the topological backbone.

### 2.8 Multi-Rover Swarm Simulation (Phases 5 & 6)
- **Absolute Trajectory Error (ATE):** Root-mean-square error (RMSE) between estimated trajectory and ground truth:
  $$\text{ATE} = \sqrt{\frac{1}{N} \sum_{i=1}^N \|\text{trans}(T_{gt, i}^{-1} T_{est, i})\|^2}$$
- **Bandwidth Consumption:** Total bytes transmitted per rover per kilometer explored.
- **Synchronization Latency:** Wall-clock time required for all connected rovers to incorporate an inter-rover loop closure into their respective local graphs.
