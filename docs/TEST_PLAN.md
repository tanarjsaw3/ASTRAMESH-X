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
- **Implemented & Verified Test Suite (`tests/test_landmark_key.cpp`):**
  - [x] Default construction creates invalid sentinel (`is_valid() == false`, schema 0).
  - [x] Explicit construction validates schema (throws `std::invalid_argument` on schema 0).
  - [x] `try_create` returns boolean status without throwing.
  - [x] Symmetric equality (`==`, `!=`) across schema, rover_id, local_index, descriptor_hash.
  - [x] Strict weak ordering (`<`) satisfying asymmetry and transitivity.
  - [x] Sorted insertion and retrieval in `std::set` and `std::map`.
  - [x] Deterministic `std::hash<LandmarkKey>` specialization with SplitMix64 mixing.
  - [x] Collision-free storage and retrieval across 500 entries in `std::unordered_set` and `std::unordered_map`.
  - [x] Extreme edge cases: max integers (`UINT16_MAX`, `UINT32_MAX`, `UINT64_MAX`), zero fields.
  - [x] Formatted string representation (`to_string()`) and stream output operator (`<<`).
  - *Result:* 54 assertions executed and verified (0 failures).

### 2.2 Counting Bloom Filter (CBF) Baseline (Phase 1)
- **Insertion & Membership:** Insert $N$ landmark keys; assert 100% true-positive recall on query.
- **False-Positive Tracking:** Query $M$ uninserted random keys; measure empirical false-positive rate (FPR) and verify it matches the theoretical equation:
  $$FPR \approx \left(1 - e^{-k N / m}\right)^k$$
- **Counter Saturation:** Test decrement and deletion operations; verify no underflow occurs on absent keys.
- **Implemented & Verified Test Suite (`tests/test_counting_bloom_filter.cpp`):**
  - [x] Parameter validation: Rejection of $n=0$, $p \le 0$, $p \ge 1$, NaN, and memory overflow thresholds.
  - [x] Dimension factories: Optimal $(m, k)$ calculation and explicit `create_with_dimensions(m, k)`.
  - [x] Insertion presence: 100% true-positive recall (0 false negatives across 1,000 inserted keys).
  - [x] Duplicate policies: `allow_duplicates=false` (idempotent rejection) vs `allow_duplicates=true` (counter increments).
  - [x] Query absent keys on empty filter (100% negative recall).
  - [x] Removal safety contract: Attempting removal of uninserted keys returns `NotPresent` and leaves all counters intact.
  - [x] Saturated counter pinning: 8-bit counters saturate at 255 without wrapping to 0; saturated counters remain pinned on removal.
  - [x] Filter reset & reuse: `clear()` zeroes all counters, clears ledger, and resets operational statistics.
  - [x] Extreme value support: Handles maximum integer field keys and rejects invalid sentinel keys.
  - [x] Cross-instance determinism: Bitwise identical counter arrays and probe sequences across independent filter instances.
  - [x] Empirical false-positive benchmark: 10,000 queries against 1,000 inserted keys observed 502 false positives ($FPR = 5.02\%$, matching the 5.00% design target and 5.03% estimated filter FPR).
  - *Result:* 714 assertions executed and verified (0 failures).

### 2.3 Invertible Bloom Lookup Table (IBLT) Engine (Phase 1)
- **Zero-Difference Identity:** Construct two identical sets $A$ and $B$; subtract $T_{diff} = T_A - T_B$; verify $T_{diff}$ immediately reports 0 differences.
- **Symmetric Difference Recovery:** Construct sets $A$ and $B$ with known symmetric difference $d = |A \oplus B|$. Verify that when $d \le d_{max}$ (decoding capacity), exact set elements $(A \setminus B)$ and $(B \setminus A)$ are fully recovered.
- **Decoding Failure Handling:** Construct sets with $d > d_{max}$; verify that the decoder cleanly signals `IncompleteResidual` rather than returning corrupt or partially hallucinated keys.
- **Negative Entry Arithmetic:** Verify proper sign handling and counter decrements during table subtraction.
- **Implemented & Verified Test Suite (`tests/test_iblt.cpp`):**
  - [x] Construction & parameter validation: Rejection of $m=0, k=0, k > m, k > 20$.
  - [x] Zero-difference identity: $A = B \implies A \Delta B = \emptyset$ (status: `Success`, zero unpeeled cells).
  - [x] Single left-side difference: $A \setminus B = \{k_1\}, B \setminus A = \emptyset$ (status: `Success`).
  - [x] Single right-side difference: $A \setminus B = \emptyset, B \setminus A = \{k_2\}$ (status: `Success`).
  - [x] Multi-item bidirectional separation: Multiple unique keys on both sides correctly partitioned into `left_only` and `right_only`.
  - [x] Set semantics duplicate policy: Inserting duplicate keys rejected; prevents multiset corruption.
  - [x] Full 128-bit key preservation: Maximum integer keys (`UINT16_MAX`, `UINT32_MAX`, `UINT64_MAX`) round-trip without bit truncation.
  - [x] Incompatible table rejection: Rejects subtraction between tables with mismatched $m$, $k$, or protocol ID.
  - [x] Pure-cell checksum validation: 64-bit checksum eliminates false-pure peeling (collision bound $2^{-64}$).
  - [x] Table immutability during decode: Decoder operates on an internal copy; difference table is unmodified whether decoding succeeds or fails.
  - [x] Instance determinism: Identical inputs yield bitwise identical cell tables and decode results.
  - [x] Graceful bounded overload: Difference $d=50$ on $m=15$ terminates cleanly with `IncompleteResidual` and non-zero residual cells.
  - [x] Empirical capacity sanity matrix: Tested $m=80, k=3$ across $d \in \{2, 5, 10, 20, 40, 70\}$ over 10 trials each: 100% exact decoding for $d \le 40$; 0% decode for $d=70$ (clean graceful failure when $d > m / 1.22$).
  - *Result:* 318 assertions executed and verified (0 failures).

### 2.4 Comparative Benchmark & Invariant Property Tests (Phase 1, Milestone 1.4)
- **CBF Invariants:**
  - [x] Zero False Negatives: 100% of inserted keys return positive under query across multiple random seeds (Invariant 1).
  - [x] Independent Instance Determinism: Identical configurations and insertion sequences produce byte-identical counter arrays and probe outputs (Invariant 2).
  - [x] Removal Safety & Ledger Protection: Removing absent keys returns `NotPresent` and leaves 100% of counters unmodified (Invariant 3).
  - [x] Saturated Counter Pinning: Repeated duplicate insertions pin counters at 255 without wraparound; removal skips pinned counters (Invariant 4).
  - [x] Invalid Key & Parameter Rejection: Schema 0 sentinels and non-positive configuration parameters throw or return error statuses (Invariant 5).
- **IBLT Invariants:**
  - [x] Identity / Self-Reconciliation: Reconciling identical tables decodes to empty difference with zero residual cells (Invariant 1).
  - [x] Exact Symmetric Difference: Successfully decodes known symmetric differences into disjoint `left_only` ($A \setminus B$) and `right_only` ($B \setminus A$) (Invariant 2).
  - [x] Commutativity / Anti-Symmetry: Swapping operands swaps `left_only` and `right_only` outputs with identical success status (Invariant 3).
  - [x] Table Immutability: Neither `decode()` nor `reconcile_with()` mutates input tables on success or failure (Invariant 4).
  - [x] Overload Failure Soundness: Difference exceeding table capacity yields `IncompleteResidual`, never falsely claiming success (Invariant 5).
  - [x] Incompatible Configuration Rejection: Reject tables with mismatched $m, k$, or protocol ID (Invariant 6).
  - [x] Duplicate Rejection & Additive Cancellation: Inserting duplicate returns false; removing inserted key restores table to empty state (Invariant 7).
- *Result:* 1,937 assertions executed and verified (0 failures).
- **Comparative Benchmark Suite (`tests/benchmark_cbf_iblt.cpp`):**
  - [x] Evaluated 17 parameterized trials across $N \in \{100, 500, 1000\}$, $d \in [4, 100]$, and shared fractions $60\%$ to $98\%$.
  - [x] Strict 1 KB synchronization envelope evaluation: verified that 32 cells (1024 bytes) decode differences up to $d \le 26$ with 100% precision and recall independent of map size $N$.

### 2.5 Topological Pose-Graph Data Structures (Phase 2, Milestone 2.1)
- **Node & Keyframe Specifications:**
  - [x] Empty graph invariants: empty initialization, 0 nodes, 0 edges, safe `clear()` (Test 1).
  - [x] Keyframe addition & retrieval: stores stable `NodeId(rover_id, local_index)`, planar SE(2) pose, acquisition timestamp, and explicit `LandmarkKey` references (Test 2).
  - [x] Multi-rover node management: enumeration of distinct rover IDs and per-rover node partitioning (Test 3).
  - [x] Node validation & duplicate rejection: rejects `rover_id == 0`, non-finite (NaN, inf) pose coordinates, and duplicate `NodeId` insertions (Test 4).
- **Edge & Constraint Specifications:**
  - [x] Missing endpoint rejection: fails if either source or target node does not exist in graph (Test 5).
  - [x] Self-edge prohibition: rejects edges with `from_node == to_node` (`SelfEdgeProhibited`) (Test 5).
  - [x] Classification & verification status: supports `Odometry`, `IntraRoverLoopClosure`, and `InterRoverLoopClosure` with `Unverified`, `Verified`, and `Rejected` gates (Test 5).
  - [x] Provenance consistency: rejects cross-rover odometry and same-rover inter-rover closures (Test 6).
  - [x] Duplicate edge policy: rejects duplicate edge IDs and duplicate topological constraints of identical type (Test 7).
  - [x] Information matrix validation: verifies positive-definiteness via Sylvester's criterion ($xx > 0$, $xx \cdot yy - xy^2 > 0$, $\det > 0$); rejects indefinite or non-finite matrices (Test 8).
  - [x] Angle normalization: normalizes yaw angles into $[-\pi, \pi)$ (Test 9).
  - [x] Node and incident edge removal: deleting a node automatically cascade-removes all incident edges and updates adjacency indices (Test 10).
  - [x] Extreme boundary identifiers: handles `UINT16_MAX`, `UINT32_MAX`, and `UINT64_MAX` cleanly (Test 11).
  - [x] Deterministic 100-node swarm construction: 2 rovers, 50 keyframes each, 98 odometry edges, 10 intra-rover loop closures, and 4 inter-rover loop closures (112 edges total) (Test 12).
- *Result:* 97 assertions executed and verified (0 failures).

### 2.6 Geometric Verification & Loop Closure Gating (Phase 2, Milestone 2.2)
- **Transform Estimation & Gating Specifications (`tests/test_geometric_verifier.cpp`):**
  - [x] Exact known rigid transforms: pure translation, pure rotation, combined translation and rotation with zero residual RMSE (Test 1).
  - [x] Small measurement noise handling: 2 cm noise with 10 cm inlier threshold recovers pose within tolerance and computes accurate RMSE (Test 2).
  - [x] Outliers mixed with inliers: 10 inliers + 5 gross outliers (66.7% inlier ratio) correctly recovers true pose, masks all inliers, and rejects all outliers (Test 3).
  - [x] Insufficient correspondences gating: < 4 correspondences fails cleanly with `InsufficientCorrespondences` (Test 4).
  - [x] All-outlier inconsistent data rejection: uncorrelated point clouds fail with `NoConsensusFound` and 0 inliers (Test 5).
  - [x] Low inlier ratio rejection: 3 inliers out of 10 (30% < 50% min ratio) fails with `InlierRatioTooLow` (Test 6).
  - [x] Degenerate point configurations: coincident points and collinear/zero-variance samples fail with `DegenerateConfiguration` without NaN/inf (Test 7).
  - [x] Non-finite coordinate inputs: NaN or infinity in source or target coordinates fail safely with `InvalidCoordinates` (Test 8).
  - [x] Parameter bounds validation: non-positive thresholds or invalid ratios fail with `InvalidConfiguration` (Test 9).
  - [x] Determinism with fixed random seed: identical configurations and seeds produce bitwise-identical inlier counts, masks, and poses (Test 10).
  - [x] Angle normalization: estimated yaw angles strictly normalized to $[-\pi, \pi)$ (Test 11).
  - [x] PoseGraph edge promotion and rejection gating: successful verification promotes unverified edge to `Verified` with verified pose; failed verification marks edge `Rejected` and never promotes it (Test 12).
- *Result:* 77 assertions executed and verified (0 failures).

### 2.7 Disjoint Set Union (DSU) Subgraph Merging (Phase 2, Milestone 2.3)
- **DSU & Partition Management Specifications (`tests/test_disjoint_set.cpp`):**
  - [x] Basic DSU operations: `make_set`, `find`, `unite`, `are_connected`, initial singletons, component decrementing, duplicate unite no-op (Test 1).
  - [x] Path compression & union by rank/size: linear chain uniting, path flattening, representative consistency, and per-component member counts (Test 2).
  - [x] Duplicate registration & unknown node handling: rejects invalid sentinel `NodeId(0, 0)`, duplicate `make_set` returns false, unknown queries return nullopt / 0, unite with unknown node fails safely (Test 3).
  - [x] Multiple disconnected components & transitivity: 3 independent multi-rover components, transitive connectivity verification, cross-component bridge uniting (Test 4).
  - [x] Odometry connectivity policy: sequential odometry edges form intra-rover trajectory components while keeping independent rovers disconnected (Test 5).
  - [x] Unverified and rejected loop closures gating: unverified inter-rover and intra-rover candidate closures NEVER merge components; rejected edges NEVER merge components (Test 6).
  - [x] Verification promotion merging: promoting candidate edge to `Verified` merges independent rover subgraphs into a unified component (Test 7).
  - [x] Edge and node removal rebuilding: removing an edge safely splits the component into two disjoint subgraphs; removing a node cascade-removes incident edges and leaves remaining nodes as isolated partitions (Test 8).
  - [x] Subgraph member queries and determinism: `get_partition_members` returns sorted node IDs; `get_partitions_list` produces deterministic ordering; repeated `rebuild` is strictly idempotent (Test 9).
  - [x] Empty graph & isolated nodes: empty graph produces 0 nodes and 0 partitions; isolated nodes without edges form size-1 partitions (Test 10).
  - [x] End-to-end pipeline integration: candidate closure proposal -> geometric verification rejection leaves partitions separated; candidate closure proposal -> geometric verification success promotes edge and merges multi-rover swarm (Test 11).
- *Result:* 153 assertions executed and verified (0 failures).

### 2.8 Perceptual Aliasing & Invalid Loop-Closure Rejection (Phase 2, Milestone 2.4)
- **Adversarial & Rejection Test Specifications (`tests/test_loop_closure_rejection.cpp`):**
  - [x] **Scenario 1: Perceptual Aliasing:** Identical `LandmarkKey` identifiers paired with scrambled, spatially contradictory point coordinates. Proves matching keys alone never pass geometric verification. Rejection transitions candidate edge to `Rejected` (Scenario 1).
  - [x] **Scenario 2: Random False Correspondences:** Uncorrelated uniform random points in source and target frames. Rejection verifies inlier count/ratio falls below consensus requirements (Scenario 2).
  - [x] **Scenario 3: Candidate Prior Inconsistency:** Estimated relative pose conflicts with candidate prior pose beyond configured translation tolerance ($\|t - t_{prior}\| > \text{tol}_{trans}$) or rotation tolerance ($|\theta \ominus \theta_{prior}| > \text{tol}_{rot}$). Fails with `VerificationFailureReason::InconsistentWithCandidatePrior` (Scenario 3).
  - [x] **Scenario 4: Insufficient Correspondences:** Evaluates 0, 1, and 3 correspondences against $N_{min} = 4$. Fails with `VerificationFailureReason::InsufficientCorrespondences` (Scenario 4).
  - [x] **Scenario 5: Degenerate Configurations:** Zero centered spatial variance (coincident source or target points) and ambiguous orientation (zero cross-covariance norm). Fails with `VerificationFailureReason::DegeneratePointConfiguration` (Scenario 5).
  - [x] **Scenario 6: Outlier-Heavy Input:** 85% gross random outliers mixed with 15% inliers. Strictly rejected under standard gating parameters (Scenario 6).
  - [x] **Scenario 7: Non-Finite and Invalid Data:** Rejects NaN and infinity in coordinates (`NonFiniteInputCoordinates`). Rejects negative distance thresholds, zero iterations, $N_{min} < 2$, inlier ratio $> 1.0$, NaN in candidate prior pose, and negative prior tolerances (`InvalidConfiguration`). Proves graph state remains unmutated (Scenario 7).
  - [x] **Scenario 8: Transform-Direction Safety:** Detects frame reversal mistakes ($T_{B \to A}$ vs $T_{A \to B}$) against forward candidate priors and prevents reversed edges from silently passing (Scenario 8).
  - [x] **Scenario 9: Exact Boundary Cases:** Validates exact inlier threshold boundary ($d \le d_{thresh}$ vs $d > d_{thresh}$), inlier count exactly equal to $I_{min}$ (passes) vs $I_{min} - 1$ (fails), and inlier ratio exactly equal to $r_{min}$ (passes) vs slightly below (fails) (Scenario 9).
  - [x] **Scenario 10: Determinism:** Confirms identical outcomes (success, inlier count, RMSE, inlier mask, refined pose) across independent runs with identical random seeds (Scenario 10).
  - [x] **Scenario 11: PoseGraph Gating Safety & Lifecycle:** Safely rejects gating non-existent edge IDs, gating `Odometry` edges, gating edges with missing endpoint nodes, and repeated gating on already `Verified` or `Rejected` edges without state corruption (Scenario 11).
  - [x] **Scenario 12: DSU Subgraph Partition Safety:** Unverified candidates and rejected edges never merge partitions; geometrically verified edge merges partitions; removing verified edge and rebuilding cleanly recovers disjoint components (Scenario 12).
- *Result:* 104 assertions executed and verified (0 failures).

### 2.9 Continuous Riemannian Dynamics (CORD) Optimization (Phase 3)

#### 2.9.1 Milestone 3.2: $\mathrm{SE}(2)$ Lie-Group Operations & Objective Evaluation
- [x] **Group Axioms & Inverses (`test_se2_lie_math`):** Verified $X X^{-1} = I$, $X^{-1} X = I$, and group associativity $(X_1 X_2) X_3 = X_1 (X_2 X_3)$ across non-trivial poses. Hand-calculated composition checked (Test 1 & 2).
- [x] **Point Transformation (`test_se2_lie_math`):** Verified forward $p_{\mathrm{world}} = R p_{\mathrm{body}} + t$ and inverse $p_{\mathrm{body}} = R^\top (p_{\mathrm{world}} - t)$ round-trip against hand-calculated fixtures (Test 3).
- [x] **Exponential & Logarithm Bijections (`test_se2_lie_math`):** Verified $\operatorname{Log}(\exp(\xi)) = \xi$ across parameter sweep covering pure translation, pure rotation, and general twists (Test 4).
- [x] **Small-Angle Taylor Expansions (`test_se2_lie_math`):** Asserted smooth, finite, non-NaN evaluation across the singularity boundary $|\omega|, |\theta| \in [10^{-12}, 10^{-5}]$, verifying relative truncation error $< 10^{-8}$ against full trigonometric forms (Test 5).
- [x] **Branch Cut & Normalization (`test_se2_lie_math`):** Validated angle wrapping behavior at and near $\pm\pi$, multi-turn winding ($3\pi \to -\pi$), and branch-cut boundary preservation (Test 6).
- [x] **Adjoint & Co-Adjoint Consistency (`test_se2_lie_math`):** Verified $\operatorname{Ad}_{X_1 X_2} \xi = \operatorname{Ad}_{X_1}(\operatorname{Ad}_{X_2} \xi)$ and algebraic gyroscopic identity $\xi^\top \operatorname{ad}_\xi^*(M \xi) \equiv 0$ across isotropic and anisotropic mass/inertia tensors (Test 7).
- [x] **Non-Finite & Invalid Input Safety (`test_se2_lie_math`):** Asserted NaN and Inf inputs safely return invalid instances without throwing unhandled exceptions or causing UB (Test 8).
- [x] **Zero Residual & Consistent Measurement (`test_pose_graph_objective`):** Verified $r_{ij} = [0, 0, 0]^\top$ and cost $0.0$ when $\widehat{Z}_{ij} = Z_{ij}$ (Test 1).
- [x] **Hand-Calculated M3.1 Worked Example Parity (`test_pose_graph_objective`):** Asserted exact match against hand calculations: $r_{12} = [0.202458, 0.094979, 0.050000]^\top$ (error $< 10^{-4}$) and cost $\mathcal{C}_{12} = 3.0005$ (error $< 10^{-3}$) (Test 2).
- [x] **Gauge Invariance (`test_pose_graph_objective`):** Asserted $\mathcal{C}(g X) \equiv \mathcal{C}(X)$ to within $10^{-9}$ under arbitrary global rigid transformation $g \in \mathrm{SE}(2)$ (Test 3).
- [x] **Edge Eligibility Policy (`test_pose_graph_objective`):** Verified that Odometry is admitted unless Rejected; Intra/Inter loop closures are admitted ONLY if Verified; Unverified and Rejected loop closures never contribute to cost (Test 4).
- [x] **Invalid Data Rejection (`test_pose_graph_objective`):** Asserted rejection of missing node IDs, NaN poses, and indefinite information matrices with structured `valid = false` (Test 5).
- [x] **Reference Numerical Riemannian Gradient (`test_pose_graph_objective`):** Verified central finite differences on right-trivialized Lie perturbations. Matched exact Riemannian gradient $[20.479063, 8.989792, 20.020839]^\top$ (error $< 10^{-3}$) and proved asymptotic convergence to $\Omega r$ as $r \to 0$ (Test 6).
- *Result:* 244 assertions executed and verified (0 failures).

#### 2.9.2 Milestone 3.3: Centralized Reference Optimizer & Synthetic Benchmarks
- **Ground-Truth Synthetic Topologies:** Generated ground truth trajectories (`StraightChain`, `ClosedLoop`, `GridManhattan`, `MultiComponent`, `LoopClosureGatingTest`) with simulated Gaussian odometry and loop closures (`tests/support/synthetic_pose_graph.hpp`).
- **Centralized Solver Parity:** Implemented centralized Levenberg-Marquardt optimizer on $\mathrm{SE}(2)$ (`astromesh::optimization::optimize_pose_graph`), verifying convergence and $>90\%$ cost reduction across all benchmarks.
- **Exact Gauge Anchor Invariance:** Verified fixed gauge anchor $\Delta \xi_{\text{anchor}} \equiv 0$ preserved exactly ($< 10^{-12}$ drift).
- **Edge Eligibility Gating:** Verified that unverified and rejected loop closures never enter the objective or affect the solution.
- **Disconnected Subgraph Anchoring:** Verified that multi-component graphs anchor 1 node per component or report `MultipleDisconnectedComponents` when disallowed.
- **Trajectory Metrics Hand-Calculations:** Verified hand-calculated examples for raw ATE, 2D Umeyama rigid alignment, and step-1 RPE.
- **Verification Status:** Verified by 128 unit/property test assertions in `test_centralized_optimizer` and automated CSV benchmarking in `benchmark_centralized_optimizer` (100% passed).

#### 2.9.3 Milestone 3.4: CORD Dynamics & Semi-Implicit Geometric Integrator [Completed & Verified]
- [x] **Lyapunov Energy Dissipation:** Total Hamiltonian $\mathcal{H}^k = \mathcal{K}(\xi^k) + \mathcal{C}(X^k)$ strictly satisfies $\mathcal{H}^{k+1} \le \mathcal{H}^k$ across discrete steps until convergence (>92% energy reduction).
- [x] **Continuous Energy Theorem:** Numerical convergence of power derivative $\frac{\mathcal{H}(h)-\mathcal{H}(0)}{h} \to \dot{\mathcal{H}} = -\sum \xi_i^\top D_i \xi_i$ verified within $10^{-3}\,\mathrm{W}$.
- [x] **Damping Limit Stability:** Unconditional numerical stability under high damping without oscillation or divergence.
- [x] **Manifold Invariant Preservation:** Poses stay strictly on $\mathrm{SE}(2)$ via closed-form exponential map retraction.
- *Result:* 154 assertions verified in `test_cord_integrator` (0 failures).

#### 2.9.4 Milestone 3.5: Multi-Robot Distributed Simulation & Neighbor Prediction [Completed & Verified]
- [x] **Boundary State Serialization:** Exact 80-byte little-endian packing and round-trip decoding of `BoundaryStatePacket`.
- [x] **Sequence & Stale Filtering:** Out-of-order, stale, duplicate, and corrupted packets dropped without mutating state.
- [x] **Delay Prediction Accuracy:** Under latency $\tau$, velocity extrapolation $\widehat{X} = X \exp((\tau \xi)^\wedge)$ verified for pure translation, pure rotation, and helical motion.
- [x] **Zero-Delay Synchronous Parity:** Distributed CORD gradients and 1-step updates match centralized CORD identically within $10^{-6}$.
- [x] **Disconnection & Reconnection Gracefulness:** Graceful stale-edge deactivation (> 2.0 s) and automatic dynamic reconnection recovery.
- *Result:* 303 assertions verified in `test_distributed_cord` (0 failures).

#### 2.9.5 Milestone 3.6: Comparative Benchmarks, Ablations & Profiling [Implemented & Audited]
- [x] **Convergence Parity:** At zero delay, distributed CORD achieves 93.6% cost reduction matching centralized optimization baseline.
- [x] **Latency Ablation:** Evaluated delays $\{0, 20, 50, 100, 200, 500\}\,\mathrm{ms}$ with velocity prediction enabled vs disabled, documenting extrapolation lead phase dynamics under velocity reversal. Unconstrained prediction exhibits anti-damping under non-negligible delay.
- [x] **Packet-Loss Ablation:** Evaluated loss rates $\{0\%, 10\%, 25\%, 50\%\}$, proving robust convergence even under 50% loss ($88.4\%$ cost reduction, $\text{ATE} = 0.0903\,\mathrm{m}$).
- [x] **Topology & Rover Scaling:** Benchmarked Straight Chain (3 rovers, 30 nodes, 94.3% cost reduction), Manhattan Grid (3 rovers, 36 nodes, 89.8% cost reduction), and Circular Loop (3 rovers, 30 nodes, 93.1% cost reduction).
- [x] **Network Partition Recovery:** 400 ms link severance survived without numerical breakdown, recovering upon link restoration.
- [x] **Repeatability & Multi-Seed Robustness:** Evaluated seeds $\{42, 101, 777\}$ with bit-for-bit exact reproducibility across reruns.
- [x] **Physical Wire Byte Profiling:** Explicitly separated 80-byte application payload from 20-byte transport framing (100 bytes/packet), accounting for 100% of transmitted wire bytes.
- *Result:* 23 reproducible trials recorded in `benchmark_distributed_cord_results.csv`.

#### 2.9.6 M3.6 Independent Scientific Audit & Regression Verification [Completed & Verified]
- [x] **Boundary Packet Keyframe Index 0 Validation (`test_distributed_cord`):** Verified that `BoundaryStatePacket::is_valid()` permits `node_local_index == 0`. Validates that node `(rover_id, 0)` boundary packets serialize, validate, deserialize, and transmit across simulated mesh networks without drop (Regression Test, 5 assertions).
- [x] **Full-Graph vs Active-Graph Objective Accounting:** Verified distinction between:
  1. *Unified Full-Graph Objective* $\mathcal{C}_{\text{global}}$: Evaluates all eligible odometry and verified loop closures across all rovers once on current estimated poses.
  2. *Active-Graph Objective*: Evaluates currently active edges.
  3. *Local Active Cost Sum*: Sum of `compute_local_active_cost()` across rovers; double-counts inter-rover loop closures and uses asymmetric neighbor estimates. Clarified in documentation.
- [x] **Non-Cryptographic Checksum Characterization:** Verified docstring and documentation accurately describe the 64-bit FNV-1a / SplitMix64 hash as a deterministic non-cryptographic checksum.
- [x] **Delay Prediction Phase-Lead Anti-Damping Analysis:** Empirically verified root cause of negative cost reduction ($-1108\%$ to $-8974\%$) at $\tau \in [20, 200]\,\mathrm{ms}$ with prediction ON as dynamic phase-lead / anti-damping on stiff oscillatory loop closure edges. Classified scientific claims conservatively.
- *Result:* 308 assertions verified in `test_distributed_cord` (suite total: 4,345 assertions across 13 test suites, 0 failures).

### 2.10 Delay-Tolerant Networking & Bounded-Memory (Phase 4)

#### 2.10.1 Milestone 4.1: Delay-Tolerant Boundary State Filtering & Adaptive Damping [Completed & Verified]
- [x] **BoundaryStateFilter Unit Validation (`test_distributed_cord`):** Verified valid packet acceptance, stale and duplicate sequence rejection via RFC 1982 modular comparisons, non-finite (NaN/Inf) coordinate rejection, protocol version guards, and reset/clear lifecycle operations (Test 13).
- [x] **Exponential Moving Average (EMA) Velocity Smoothing:** Verified low-pass smoothing formula $\xi_{\text{filt}} = (1 - \alpha_v) \xi_{\text{prev}} + \alpha_v \xi_{\text{raw}}$ with $\alpha_v = 0.50$, matched against hand calculations (Test 13).
- [x] **Velocity Norm Clamping & Saturation:** Verified radial linear velocity scaling to $v_{\max} = 1.50\,\mathrm{m/s}$ and angular velocity clamping to $\omega_{\max} = 1.00\,\mathrm{rad/s}$ (Test 13).
- [x] **Quiescent Noise Deadbands:** Verified zeroing of linear and angular velocities below configured thresholds ($10^{-4}\,\mathrm{m/s}$ and $10^{-4}\,\mathrm{rad/s}$) to suppress numerical/sensor drift amplification (Test 13).
- [x] **Prediction Horizon Capping & ZOH Fallback:** Verified that when packet delay $\tau > \tau_{\max} = 30\,\mathrm{ms}$, the state estimator cleanly falls back to Zero-Order Hold (ZOH) with zero velocity, preventing phase-lead divergence under large latencies (Test 13).
- [x] **Angular Wrap-Around Geometry on $\mathbb{S}^1$:** Verified smooth, continuous exponential map extrapolation across the $\pm\pi$ branch cut (Test 13).
- [x] **Adaptive Damping Formula & Boundary Bounds:** Verified $D_i(\tau) = D_{\text{base}} \min(\kappa_{\max}, 1 + \alpha_d \tau)$ with $\alpha_d = 15.0\,\mathrm{s}^{-1}$ and $\kappa_{\max} = 5.0$, ceiling clamping, negative delay protection, and disabled toggle parity (Test 14).
- [x] **Delayed Stability Stress Testing:** Verified finite poses, bounded velocities ($|\omega| < 3.91\,\mathrm{rad/s}$), positive cost reduction, and low aligned ATE ($< 0.14\,\mathrm{m}$) across $20\,\mathrm{ms}$, $50\,\mathrm{ms}$, $100\,\mathrm{ms}$, and $200\,\mathrm{ms}$ latencies on circular loops (Test 15, 48,000 per-step finite assertions).
- [x] **Option Validation Guards:** Verified rejection of non-positive velocities, invalid smoothing factors, negative alphas, and invalid damping multipliers (Test 16).
- *Result:* 49,191 assertions verified in `test_distributed_cord` (0 failures).

#### 2.10.2 Milestone 4.2: DTN Store-and-Forward Buffering & Byte Quota Accounting [Completed & Verified]
- [x] **Store-and-Forward Buffer Semantics (`test_dtn_buffer`):** Verified FIFO order within equal priorities, strict high-to-low priority selection, and deterministic priority aging anti-starvation policy (`FairnessPolicy::PriorityAging`) promoting waiting messages across intervals (105 assertions).
- [x] **TTL Boundary & Expiration Dynamics:** Verified exact-boundary expiration (`now >= expiry_time_ns`), rejection of already-expired enqueue attempts, capacity recovery upon expiration purge, and prohibition of delivering expired messages upon link reconnection.
- [x] **Exponential Retry Backoff & Saturation:** Verified saturating backoff calculation $\text{delay} = \min(\text{max\_delay}, \text{initial\_delay} \cdot \text{multiplier}^{\text{attempt}-1})$, attempt limit enforcement (`retries_exhausted`), and retry suppression when retry time exceeds TTL.
- [x] **Bounded Deduplication & Cache Lifecycle:** Verified duplicate message rejection within cache TTL and capacity eviction bounds preventing unbounded memory growth.
- [x] **Partition Tolerance:** Verified that messages for disconnected peers remain safely retained in the buffer during link severance and are dequeued upon reconnection.
- [x] **Physical & Protocol Byte Metering (`test_packet_budget`):** Verified exact arithmetic breakdown across application payload, protocol headers, and transport framing (81 assertions).
- [x] **Sliding Window & Per-Peer Quota Isolation:** Verified exact-boundary acceptance, single-byte rejection above budget, epoch rollover resets, and per-peer budget isolation.
- [x] **Zero Double-Counting & Channel Loss Accounting:** Verified that retransmissions and channel drops increment sent counters without duplicating received bytes, and rejected messages never increment sent bytes.
- [x] **Deterministic Benchmark & Rerun Verification (`benchmark_dtn_buffer`):** 30 parameterized trials executed across buffer capacity, priority mix, partition durations, loss rates, and byte quotas; 100% bit-for-bit reproducible (`audit_benchmark_dtn_rerun.csv`).

#### 2.10.3 Milestone 4.3: Safe Keyframe Pruning and Bounded Pose-Graph Memory [Completed & Verified]
- [x] **Keyframe Admission Criteria (`test_keyframe_pruner`):** Verified translation threshold ($\Delta d \ge d_{\min}$), rotation threshold ($|\Delta \theta| \ge \theta_{\min}$), time delta threshold ($\Delta t \ge t_{\min}$), rejection of sub-threshold poses, and rejection of non-finite (NaN/Inf) inputs (45 assertions).
- [x] **Protected Node Policy:** Verified non-removal of:
  1. *Gauge Anchors:* Reference poses for active connected components are permanently protected.
  2. *Exploration Frontier:* Newest retained keyframe for each rover is protected from deletion.
  3. *Verified Loop Endpoints:* Both endpoints of active `Verified` loop closures are protected.
  4. *Inter-Rover Constraints:* Endpoints of multi-rover boundary constraints are protected.
  5. *Articulation Points:* Cut-vertices that would disconnect the graph are protected.
- [x] **Tarjan Cut-Vertex Articulation Detection:** Verified identification of cut-vertices on linear chains and 2-connected cycles over eligible active connectivity edges (`Odometry` and `Verified` loop closures; `Unverified` and `Rejected` edges excluded).
- [x] **Safe Leaf Node & Redundant Cycle Pruning:** Verified removal of leaf nodes and redundant intermediate cycle nodes, with atomic removal of all incident edges and zero dangling endpoints.
- [x] **Partition Rebuilding Synchronization:** Verified that `SubgraphPartitionManager::rebuild(graph)` is invoked immediately following node deletion, keeping DSU component indices synchronized.
- [x] **Limit Unmet Soundness:** Verified that when the graph exceeds `max_retained_keyframes` but all candidates are protected, the pruner returns `LimitUnmetNoSafeCandidate` without mutating the graph or partition state.
- [x] **Deterministic Spatial Ranking:** Verified candidate ordering by spatial distance to nearest neighbors, with deterministic tie-breaking by ascending `NodeId`.
- [x] **Benchmark and Factor-Removal Ablation (`benchmark_keyframe_pruner`):** Evaluated 13 scenarios across Chain, Loop, Grid, and Disconnected topologies; 100% bit-for-bit reproducible output (`benchmark_keyframe_pruner_results.csv` and `audit_benchmark_keyframe_pruner_rerun.csv`); zero graph disconnections, 100% verified loop-closure preservation; factor removal without marginalization explicitly evaluated as a factor-removal ablation.
- *Result:* 45 assertions verified in `test_keyframe_pruner` (0 failures).

#### 2.10.4 Milestone 4.4: Dynamic Multi-Rover Collective Integration & Orchestration [Completed & Verified]
- [x] **Rover Lifecycle State Machine (`test_rover_orchestration`):** Verified 9 operational states (`Initializing`, `Exploring`, `CommunicationDegraded`, `Disconnected`, `Reconnecting`, `Synchronizing`, `Optimizing`, `ResourceConstrained`, `Stopped`), formal transition matrix enforcement, and rejection of invalid transitions without state corruption.
- [x] **Idempotent Transitions & Diagnostic Telemetry:** Verified that duplicate transition events are idempotent no-ops, and transition logs record monotonic simulated timestamps, previous/new states, transition reasons, and diagnostic descriptions.
- [x] **Timeout Evaluation & Degradation Dynamics:** Verified deterministic transitions from `Exploring` -> `CommunicationDegraded` -> `Disconnected` when last peer contact exceeds configured nanosecond thresholds, and rapid recovery to `Reconnecting` -> `Synchronizing` -> `Exploring` upon inbound message arrival.
- [x] **Discrete-Event Multi-Rover Mesh Network:** Verified scheduled transit queues with propagation delay and jitter, zero leakage of packets across severed communication links, and stochastic packet drop handling.
- [x] **DTN Buffer & Byte Quota Accounting Integration:** Verified that transmission denies due to epoch byte quota exhaustion trigger immediate transition to `ResourceConstrained`, automatically recovering upon accounting window renewal.
- [x] **Landmark Key Set Reconciliation (IBLT):** Verified set difference reconciliation ($A \Delta B$) using `InvertibleBloomFilter`, with graceful fallback reporting when difference size exceeds IBLT cell capacity.
- [x] **Metric Geometric Loop Closure Gating:** Verified that landmark key reconciliation alone does NOT promote graph loop closures. A candidate inter-rover edge is promoted to `EdgeVerificationStatus::Verified` and merged into `SubgraphPartitionManager` if and only if 2D spatial point correspondences pass geometric verification (`verify_relative_pose_2d`). Aliased candidates are safely rejected without DSU corruption.
- [x] **Keyframe Pruner Integration:** Verified bounded pose-graph growth in integrated rovers without violating articulation-point protection or stranding anchors.
- [x] **Seeded Determinism Property Test:** Verified that two independently instantiated collectives supplied with identical seeds and identical event sequences generate bit-for-bit identical state transition sequences, packet counters, and byte statistics.
- [x] **Reproducible Benchmark Suite (`benchmark_multi_rover_collective`):** Evaluated 15 scenarios across partitions, high loss (up to 50%), high latency (50 ms), strict byte quotas, and subswarm clustering; 100% bit-for-bit identity verified between `benchmark_multi_rover_results.csv` and `audit_benchmark_multi_rover_rerun.csv`.
- *Result:* 77 assertions verified in `test_rover_orchestration` (0 failures). All 17 CTest suites pass 100%.

#### 2.10.5 Milestone 4.5: Engineering Audit, Retries Correctness & Benchmark Validity [Completed & Verified]
- [x] **DTN Retry Accounting and Quota Consumption (`test_rover_orchestration` Test 14):** Verified that retries are tracked as extra transmissions, `messages_sent == initial_transmission_attempts + messages_retried`, and retransmissions correctly consume budget quota via `retransmitted.message_count`.
- [x] **Retry Exhaustion and TTL Expiration (Test 15):** Verified that messages expiring before scheduled backoff are purged without retransmission, and messages reaching `max_attempts` trigger `retries_exhausted` and are safely erased from the buffer.
- [x] **Duplicate Delivery & Deduplication Idempotence (Test 16):** Verified that pre-recorded or duplicate deliveries are caught by `is_duplicate()`, increment `duplicate_deliveries`, and bypass application processing to prevent double mutation.
- [x] **In-Flight Packet Partition Boundary Policy (Test 17):** Verified that in-flight packets crossing an active link partition boundary are dropped at the link barrier with zero leakage across partitions.
- [x] **IBLT Decode Failure Atomicity (Test 18):** Verified that incomplete or capacity-exceeded IBLT reconciliation fails gracefully and performs an atomic rollback, leaving node counts, edge counts, and DSU partition indices completely untouched.
- [x] **Rover Silence Duration Boundaries (Test 19):** Verified timeout progression (`Exploring` -> `CommunicationDegraded` -> `Disconnected`) from startup silence ($t_{\text{contact}} = 0$) and safe non-overflowing arithmetic under extreme timestamps (`UINT64_MAX - 1000`).
- *Result:* 119 assertions verified in `test_rover_orchestration` (0 failures). All 17 CTest suites pass 100%.
- *Audit Artifacts:* `audit_phase45_results.csv` and `audit_phase45_rerun.csv` verified 100% bit-for-bit identical (SHA256: `f0a3d8eb86dc38bbd3d642325d58c7801e78b2e4cc900fdd299cc2a15357d823`). Detailed audit report documented in [`docs/PHASE45_AUDIT.md`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/docs/PHASE45_AUDIT.md).

#### 2.10.6 Milestone 4.6: Reliable Acknowledgment Protocol, Multi-Hop DTN Routing, and Delivery Audit [Completed & Verified]
- [x] **Direct Delivery with Explicit Network Acknowledgment (`test_rover_orchestration` Test 20):** Verified that message arrival generates an explicit 24-byte `AckPayload` protocol message transmitted over the network, incurring transit latency, consuming transmission quota, and clearing the sender's DTN buffer upon return arrival.
- [x] **Lost Data Packet Followed by Successful Retry (Test 21):** Verified that channel packet drop triggers sender exponential backoff timer and subsequent retransmission, successfully completing delivery.
- [x] **Lost ACK, Duplicate Delivery & Returning ACK (Test 22):** Verified that dropping the return ACK causes the sender to retransmit the original message; the destination detects the duplicate, suppresses repeated processing (`duplicate_processing_attempts == 0`), increments `duplicate_deliveries`, and re-emits the ACK.
- [x] **Duplicate ACK Idempotence (Test 23):** Verified that receiving duplicate ACKs or ACKs for non-existent messages returns `AckResult::NotFound` with zero state corruption.
- [x] **ACK Byte Quota & Wire Overhead Accounting (Test 24):** Verified that ACK packets consume sender-side transmission budget and are tracked in segregated wire-byte telemetry.
- [x] **Three-Rover Multi-Hop Route (Test 25):** Verified 1 -> 2 -> 3 data forwarding and 3 -> 2 -> 1 reverse ACK forwarding across a multi-hop path when direct link 1 <-> 3 is severed.
- [x] **Temporal Partition Store-Carry-Forward (Test 26):** Verified that intermediate carrier holds message in DTN buffer during a 100 ms partition and dispatches it forward upon link restoration.
- [x] **Hop-Limit Exhaustion (Test 27):** Verified that messages reaching `max_hops` limit are dropped at the intermediate hop, incrementing `hop_limit_exhausted`.
- [x] **Anti-Looping Cycle Suppression (Test 28):** Verified that cyclic mesh topologies do not circulate duplicate forwarded messages infinitely.
- [x] **Deterministic Multi-Hop Property Test (Test 29):** Verified 100% bit-for-bit identical telemetry across repeated multi-hop executions with identical seeds.
- *Result:* 175 assertions verified in `test_rover_orchestration` (0 failures). All 17 CTest suites pass 100%.
- *Benchmark Artifacts:* `benchmark_phase46_results.csv` and `benchmark_phase46_rerun.csv` verified 100% bit-for-bit identical (SHA256: `F0BA363849915DFF3E76684CC4A97BA3B9618417F38DD2E03C9C4B1882E12362`). Detailed audit report documented in [`docs/PHASE46_AUDIT.md`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/docs/PHASE46_AUDIT.md).

### 2.11 Multi-Rover Swarm Simulation (Phases 5 & 6)
- **Absolute Trajectory Error (ATE):** Root-mean-square error (RMSE) between estimated trajectory and ground truth:
  $$\text{ATE} = \sqrt{\frac{1}{N} \sum_{i=1}^N \|\text{trans}(T_{gt, i}^{-1} T_{est, i})\|^2}$$
- **Bandwidth Consumption:** Total bytes transmitted per rover per kilometer explored.
- **Synchronization Latency:** Wall-clock time required for all connected rovers to incorporate an inter-rover loop closure into their respective local graphs.

#### 2.11.1 Milestone 5.1: Deterministic Synthetic World & Trajectory Harness [Completed & Verified]
- [x] **Arena Bounds & Configuration:** Validated rectangular arena limits $[-x_{max}, x_{max}] \times [-y_{max}, y_{max}]$; rejected non-positive or NaN bounds (`test_synthetic_world` Test 1).
- [x] **Deterministic Landmark Distribution:** Uniform spatial sampling derived exclusively from explicit 64-bit seed; confirmed bit-for-bit repeatability across independent runs (Test 2).
- [x] **Visibility Sensor Model:** Radial range limits and angular field-of-view filtering; verified boundary inclusion/exclusion and wrap-around angles (Test 3).
- [x] **Monotonic Multi-Rover Trajectories:** Verified continuous kinematics, bounded turning rates, finite SE(2) coordinates, and strictly increasing timestamps across parallel search, overlapping grid, and concentric loops (Tests 4–6).
- [x] **Metric Landmark Observations:** Local sensor body-frame coordinate transformations with configurable Gaussian range/bearing noise and zero-noise ground-truth modes (Tests 7–8).
- [x] **Correspondence Extraction:** Automated 2D correspondence pairing between rover observations sharing identical landmark keys for Umeyama RANSAC verification (Tests 9–10).
- *Result:* 1,680 assertions verified in `test_synthetic_world` (0 failures).

#### 2.11.2 Milestone 5.2: AstroMesh v1 Baseline Pipeline & Audit Hardening [Completed & Verified]
- [x] **Pipeline Interface Construction & Invalidation (`test_baseline_astromesh_v1` Test 1):** Verified default configuration, rejection of zero/NaN parameters, and strict type safety on `IRoverSwarmPipeline`.
- [x] **Deterministic Replay Invariance (Test 2):** Verified bit-for-bit identical candidate matches, accepted closures, and transmitted bytes across independent pipeline instances with identical seeds.
- [x] **CBF Candidate Discovery vs. Verification Gating (Test 3):** Verified that CBF positive query proposes loop-closure candidate, but DSU does NOT merge until geometric verification succeeds.
- [x] **Adversarial False-Positive Rejection (Test 4):** Verified that synthetic aliased landmarks or inconsistent geometries reported by CBF are caught and rejected by Umeyama RANSAC (`verify_relative_pose_2d`), with zero DSU partition mutation.
- [x] **Degenerate & Insufficient Correspondence Gating (Test 5):** Verified that correspondence counts below threshold ($N < 3$) or collinear spatial layouts reject closure and prevent graph edge promotion.
- [x] **Verified Loop Closure DSU Connectivity Merge (Test 6):** Verified that topologically disconnected rover components merge into a single DSU component upon verified geometric closure acceptance.
- [x] **Rejected/Unverified Closures Invariance (Test 7):** Verified partition count stability ($N=2$) when closure verification fails.
- [x] **Observation Idempotency & Duplicate Prevention (Test 8):** Verified that re-submitting identical observations does not duplicate graph nodes, loop closure edges, or accepted events.
- [x] **Open-Loop Dead-Reckoning Pose Integration (Test 9):** Verified dead-reckoning trajectory integration maintains numerical separation from ground truth; trajectory drift accumulates without smoothing.
- [x] **Centralized Optimizer & CORD Disablement (Test 10):** Verified `optimize()` returns `false` (unsupported operation in baseline) and metrics report no optimization performed.
- [x] **One-Hop Communication & Byte Accounting (Test 11):** Verified best-effort one-hop broadcast wire bytes, logical message counts, transmission attempts, and stochastic packet drop handling.
- [x] **Scenario Determinism Under Channel Noise & Non-Zero Event Replay (Test 12 - Hardened):** Resolved zero-event replay test caused by collinear landmark configuration degeneracy, verified non-zero verified/rejected events (4 candidates, 1 verified, 1 rejected, 1 DSU merge) with bit-for-bit replay identity.
- [x] **Euclidean Nearest-Site Voronoi Partition & Boundary Ties (Test 13):** Verified exact distance partitioning and deterministic tie-breaking ($site\_id_i < site\_id_j$) along cell boundaries.
- [x] **Malformed Inputs & Disconnected Swarms (Test 14):** Handled empty observations, single rovers, and isolated swarms without unhandled exceptions.
- [x] **Telemetry & Metric Separation (Test 15):** Verified distinct metric counters for CBF candidates, verified closures, rejected closures, logical messages, physical transmissions, dropped packets, and wire bytes.
- [x] **Disjoint Landmark Observations (Test 16 - Stage A):** Verified disjoint observations produce 0 candidates, 0 verified closures, 0 rejected closures, leaving components isolated ($N=2$).
- [x] **Comprehensive End-to-End Baseline Lifecycle (Test 17 - Stage A):** End-to-end integration exercising candidate creation, valid acceptance, corrupted geometry rejection, degenerate correspondence rejection, disjoint observations, duplicate step suppression, deterministic replay, and ground-truth isolation.
- *Result:* 105 assertions verified in `test_baseline_astromesh_v1` across 17 test suites (0 failures).

#### 2.11.3 Milestone 5.3: AstroMesh-X Pipeline Integration [Completed & Verified]
- [x] **Initialization, Invalid Configuration & Reset (`test_pipeline_astromesh_x` Test 1):** Verified rejection of zero-rover or non-positive parameter configs, initialization success, and clean state reset.
- [x] **Shared Interface Contract Compliance (Test 2):** Verified polymorphic invocation through `IRoverSwarmPipeline*` for observation ingestion, step progression, component queries, pose retrieval, and metrics.
- [x] **IBLT Reconciliation & Decode Failure Atomicity (Test 3):** Verified exact symmetric-difference reconciliation ($A \triangle B$), safe failure handling on capacity exceedance, and atomic rollback with zero mutated graph nodes or edges.
- [x] **Candidate to Geometric Verification Gating (Test 4):** Verified that IBLT match does not merge components until Kabsch/Umeyama RANSAC verifies geometry and promotes edge.
- [x] **Rejected Closure Gating & Component Isolation (Test 5):** Verified that geometric verification failure transitions edge to Rejected and leaves DSU partitions isolated ($N=2$).
- [x] **Duplicate Ingestion Idempotence (Test 6):** Verified that duplicate step timestamps are detected and suppressed with zero duplicate nodes, edges, or DSU merges.
- [x] **Distributed CORD Optimization on Valid PoseGraph (Test 7):** Verified `optimize()` executes CORD over eligible constraints, updating poses while maintaining finite poses and reduced or stable cost.
- [x] **Gauge Anchor & Disconnected Component Optimization (Test 8):** Verified that CORD fixes the gauge anchor pose $\Delta \xi_{\text{anchor}} \equiv 0$ across multi-component subgraphs without numerical divergence.
- [x] **DTN Multi-Hop, Retry & Partition Recovery (Test 9):** Verified store-carry-forward buffering, bounded multi-hop routing, channel drop retry backoff, and partition recovery.
- [x] **Byte Quota & ACK Segregation (Test 10):** Verified that 24-byte explicit `AckPayload` protocol messages and data messages are separately metered under sliding-window byte quotas.
- [x] **Safe Keyframe Pruning & Invariant Preservation (Test 11):** Verified bounded graph memory using `KeyframePruner`, protecting gauge anchors, newest keyframes, verified loop endpoints, and articulation points.
- [x] **Seeded Determinism & Replay Invariance (Test 12):** Verified bit-for-bit identical candidates, closures, DSU components, and metrics across independent runs with identical seeds.
- [x] **Ground-Truth Isolation (Test 13):** Verified pipeline estimates depend exclusively on noisy observations and odometry without leakage from ground-truth trajectories.
- [x] **Shared Scenario Input Parity (Test 14):** Verified identical `SimulationStepInput` stream accepted by both `BaselineAstroMeshV1` and `AstroMeshXPipeline`, with consistent common telemetry types.
- [x] **Metrics Invariants & Ingestion Guards (Test 15):** Verified telemetry non-negativity, mathematical relations ($\text{reconciliations\_failed} \le \text{reconciliations\_attempted}$, $\text{candidates} = \text{verified} + \text{rejected}$), and graceful handling of empty inputs.
- [x] **Audit 1: Detailed IBLT Trace & Verification Matrix (Test 16):** Full 7-step dataflow trace from observations to DSU union; identical sets ($A \triangle B = \emptyset$), disjoint sets ($A \cap B = \emptyset$), and ground-truth isolation verified under measurement perturbation.
- [x] **Audit 2: CORD Semantics, Mode & Anchor Invariants (Test 17):** Verified `optimization_mode == "SynchronousMultiComponentCord"`, per-component gauge anchor velocity pinning ($\Delta \xi_{\text{anchor}} \equiv 0$), and bit-for-bit repeatability across independent runs.
- [x] **Audit 3: Communication Accounting & Wire Byte Invariants (Test 18):** Verified exact wire byte conservation (`data == payload + header + framing`, `total == data + ack`), logical message terminal outcomes, and delivery ratios bounded in $[0, 1]$.
- [x] **Audit 4: Pruning SwarmGraph Synchronization & Post-Prune Optimization (Test 19):** Verified atomic removal of pruned nodes from `swarm_graph_`, DSU synchronization via `rebuild_global_dsu()`, and successful CORD optimization on pruned graphs without `InvalidInput` errors.
- *Result:* 112 assertions verified in `test_pipeline_astromesh_x` across 19 test suites (0 failures). Full CTest suite passes 20/20 test suites (100%).

#### 2.11.4 Milestone 5.4: Paired Monte Carlo Swarm Benchmark & Performance Evaluation [Completed & Verified]
- [x] **10-Scenario Matrix Coverage:** Implemented paired evaluation across 10 operational conditions:
  - `SC1_Nominal_2R`: Baseline 2-rover nominal communication and exploration.
  - `SC2_Loss10_2R`: 10% packet drop rate under 2-rover rendezvous.
  - `SC3_Loss30_2R`: 30% packet drop rate under 2-rover rendezvous.
  - `SC4_Loss50_2R`: Severe 50% packet drop rate.
  - `SC5_Partition_Recon_2R`: Dynamic network partition with delayed reconnection.
  - `SC6_ConstrainedBudget_2R`: Severely constrained communication byte quotas.
  - `SC7_DegenerateGeometry_2R`: Perceptual aliasing stress test with collinear, symmetric landmark geometry.
  - `SC8_LowOverlap_2R`: Sparsely overlapping field-of-view trajectories.
  - `SC9_Swarm_4R`: 4-rover meshed exploration.
  - `SC10_Swarm_8R`: Scaled 8-rover large-scale collective mapping.
- [x] **Paired Inputs & Identical Ground Truth:** Verified that for every trial $(s, k)$, `BaselineAstroMeshV1` and `AstroMeshXPipeline` receive identical simulated observations, odometry perturbations, and channel loss events.
- [x] **Lie Group CORD Runtime Bounding:** Bounded `cord_options.max_steps = 40` and $\Delta t = 0.02$ to eliminate pathological $O(N \cdot E)$ numerical gradient bottlenecks while relaxing pose-graph constraints.
- [x] **Streaming Trial Serialization:** Implemented crash-resilient CSV output with per-trial row flushing (`write_trial_row_to_stream()` with `out.flush()`).
- [x] **Graph & Optimizer Telemetry:** Instrumented explicit tracking of `graph_nodes`, `graph_edges`, `optimizer_iterations`, and `optimizer_failures`.
- [x] **Full 300-Trial Dataset Verification:** Executed full 30 seeds $\times$ 10 scenarios (300 paired trials, 600 pipeline runs) in 364.5 s. All 70 summary records verified with zero calculation discrepancies. 0 false acceptances under rigid geometry. Detailed report documented in [`docs/PHASE54_BENCHMARK.md`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/docs/PHASE54_BENCHMARK.md).

#### 2.11.5 Milestone 6.1: Controlled Communication Payload Ablation Study [Completed & Audited]
- [x] **7-Variant Architecture:**
  - `AstroMesh_X_Full`: Full proposed pipeline (IBLT, ACKs, DTN retries, multi-hop, RANSAC, DSU, CORD).
  - `AstroMesh_X_RawExchange`: Disables IBLT; exchanges raw landmark key vectors ($16\,\text{B}/\text{key}$).
  - `AstroMesh_X_NoACKs`: Disables hop-by-hop ACK frames (`ack_wire_bytes = 0`).
  - `AstroMesh_X_NoRetries`: Disables DTN packet retransmission on drop (`max_retries = 0`).
  - `AstroMesh_X_NoMultiHop`: Restricts communication to single-hop direct links (`max_hops = 1`).
  - `AstroMesh_X_Minimal`: Minimal transport variant (no ACKs, no multi-hop).
  - `AstroMesh_v1_Baseline`: Baseline AstroMesh v1 (Counting Bloom Filter, Voronoi partitioning, baseline DSU).
- [x] **Identical Shared Seed Inputs:** All 7 variants execute on identical world geometry, landmarks, odometry perturbations, and channel loss drops per $(s, k)$ pair.
- [x] **Multiple Comparisons Statistical Control:** Bonferroni family-wise error rate control across 6 pairwise comparisons against `AstroMesh_X_Full`:
  $$\alpha_{\text{adj}} = \frac{0.05}{6} \approx 0.008333, \quad t_{\text{crit,bonf}} = 2.756 \quad (\nu = 29)$$
- [x] **Byte Accounting & Conservation Invariants:**
  - $\text{Total Transmitted Wire Bytes} \equiv \text{Data Wire Bytes} + \text{ACK Wire Bytes}$
  - $\text{Data Wire Bytes} \equiv \text{Application Payload Bytes} + \text{Header Bytes} + \text{Framing Overhead}$
  - $\text{Application Completion Ratio} \in [0, 1]$, $\text{Per-Hop Delivery Ratio} \in [0, 1]$
- [x] **Pilot Verification:** Completed 140 pipeline runs (2 seeds $\times$ 10 scenarios $\times$ 7 variants) in 83.57 s with 0 failures and 0 invariant violations.
- [x] **Production Execution & Statistical Audit:** 2,100 pipeline runs streaming to `benchmark_phase61_trials.csv`. Verified with 0 malformed rows, 0 duplicate runs, 0 byte conservation violations, and 0 simulation crashes. Independent Python statistical validation matches C++ Bonferroni quantiles. Detailed report documented in [`docs/PHASE61_ABLATIONS.md`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/docs/PHASE61_ABLATIONS.md).

#### 2.11.6 Milestone 6.2: Packet Loss and Communication Delay Robustness Study [Complete & Audited]
- [x] **Grid Parameter Space:**
  - 10 Packet Loss levels: $0\%, 10\%, 20\%, 30\%, 40\%, 50\%, 60\%, 70\%, 80\%, 90\%$
  - 7 Communication Delay levels: $0\,\text{ms}, 20\,\text{ms}, 50\,\text{ms}, 100\,\text{ms}, 200\,\text{ms}, 350\,\text{ms}, 500\,\text{ms}$ (70 channel cells)
  - 7 Evaluated Variants: `AstroMesh_X_Full`, `AstroMesh_X_RawExchange`, `AstroMesh_X_NoACKs`, `AstroMesh_X_NoRetries`, `AstroMesh_X_NoMultiHop`, `AstroMesh_X_Minimal`, `AstroMesh_v1_Baseline`
  - Scenarios: `SC1_Nominal_2R` and `SC10_Swarm_8R`
- [x] **Delay-Induced Retransmissions & ARQ Mechanics:**
  - Initial DTN retry timer = $50\,\text{ms}$ with exponential backoff ($50, 100, 200, 400\,\text{ms}$).
  - Spurious retransmissions explicitly modeled when $RTT = 2\tau > \text{RTO}$, explaining wire byte expansion under high latency for ACK/retry-enabled variants.
  - In-flight messages crossing simulation end time ($t_{\text{arrive}} > \text{duration}$) tracked in `in_flight_at_termination` and safely prevented from forming post-mission loop closures.
- [x] **Channel Robustness Unit Test Target (`test_robustness_channel`):**
  - Verified byte conservation across loss rates and delays.
  - Verified spurious retry counts under $RTT > 50\,\text{ms}$.
  - Verified in-flight termination gating.
  - Registered as CTest target #21; full test suite passes 21/21 targets (100% in 1.90 s).
- [x] **Robustness Benchmark Runner (`benchmark_phase62_robustness.cpp`):**
  - Supports `--smoke`, `--pilot`, `--full`, `--seeds N`, `--scenario <id>`, and `--resume` with trial deduplication.
  - Verified smoke benchmark (28 runs, 0 failures, 100% byte conservation).
  - Verified pilot benchmark (252 runs across SC1 and SC10, 0 failures, 100% byte conservation).
- [x] **Empirical Timing & Scalability Analysis:**
  - Measured SC1: $49.0\,\text{ms/run}$ ($12.0\,\text{min}$ for 14,700 runs).
  - Measured SC10: $4.67\,\text{s/run}$ ($19.1\,\text{hrs}$ for 14,700 runs).
  - Resumable progress architecture guarantees safe interruption and resumption.
- [x] **Production Execution & Statistical Audit:** Full 14,700 SC1 trials verified across all 70 cells and 30 seeds with 0 duplicate keys, 0 failures, 100% byte conservation, and 100% message conservation. 680 SC10 trials verified and analyzed as exploratory multi-hop evidence. Comprehensive findings documented in [`docs/PHASE62_ROBUSTNESS.md`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/docs/PHASE62_ROBUSTNESS.md).

#### 2.11.7 Milestone 6.3 & Final Release Verification [Complete & Audited]
- [x] **Simulation Engine CLI (`sim_runner.exe`):**
  - Parameter validation: `--rovers`, `--scenario`, `--duration`, `--step-dt`, `--loss`, `--delay-ms`, `--comm-range`, `--variant`, `--seed`, `--opt-steps`, `--no-optimize`, `--demo <1|2|3>`, `--output-json`, `--output-csv`.
  - JSON schema conformance: `config`, `summary` (total steps, elapsed compute ms, delivered messages, total wire bytes, verified loop closures, final DSU components, swarm ATE RMSE), and per-step `frames` array (step, time_s, rovers with poses/landmarks/components, active and degraded links, telemetry).
  - Deterministic execution: Verified bit-for-bit repeatability across repeated seed executions for Demos 1, 2, and 3.
- [x] **Backend Server & Subprocess Bridge (`app/server.py`):**
  - Zero external pip dependencies; uses standard library `http.server.ThreadingHTTPServer`.
  - Automated integration test suite [`scratch/test_api.py`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/scratch/test_api.py) verifying 7 test suites:
    1. `GET /api/status`: C++ binary detection and SC1/SC10 benchmark row counts.
    2. `GET /api/demo/1`: Preset demo execution and JSON frame delivery.
    3. `POST /api/run`: Dynamic parameter parsing, C++ subprocess execution, JSON frame return.
    4. `GET /api/benchmarks`: Parsing real SC1 (14,700 rows) and SC10 (680 rows) CSV datasets.
    5. `GET /api/export/csv`: Dynamic CSV telemetry generation from latest simulation run.
    6. `GET /api/export/json`: Telemetry JSON payload export.
    7. `GET /api/export/report`: Formatted markdown mission summary generation.
  - Test result: **7/7 API test suites passed (100%)**.
- [x] **Command Center Web Dashboard (`app/static/index.html`):**
  - Canvas 2D rendering: $80\,\text{m} \times 80\,\text{m}$ world coordinates mapped accurately to screen space.
  - Interactive playback: Play, Pause, Step Back, Step Forward, Time Scrub slider, and $0.5\times\dots4.0\times$ speed multipliers.
  - Live inspectors: Individual rover pose/cluster state, link status, real-time DTN wire-byte charts, and DSU partition convergence graphs.
  - Scientific comparison explorer: Live tabular presentation of empirical SC1 and SC10 benchmark sweeps.
- [x] **Master Regression Suite:**
  - 21/21 CTest suites passing in 1.27 s (100% pass rate).


---

### 2.12 Phase 7: Distributed Networking & Transport Verification (72-Hour Sprint)

#### 2.12.1 Milestone 1: Standalone Rover Networking (`tests/test_rover_net.cpp`) [Completed & Verified]
- [x] **Framing Serialization Round-Trip:** 36-byte header with SplitMix64/FNV-1a checksum. Validates byte-exact packing/unpacking and magic `ASTX` validation.
- [x] **Tampering & Corruption Detection:** Corrupting single bits in magic, version, length, or payload triggers rejection (`std::nullopt`).
- [x] **Sequence Tracking & Duplicate Dropping:** Sliding window (size 64) drops duplicate or old sequence numbers while tracking newest window boundary.
- [x] **Peer Vitality & Heartbeat Timeout:** Peers missing heartbeats past timeout window marked inactive and pruned from active peer queries.
- [x] **Raw UDP Socket Loopback:** Cross-platform non-blocking I/O over IPv4 loopback on dynamic ports.
- [x] **Autonomous Mutual Discovery:** Two concurrent `RoverNode` instances discover each other via heartbeat exchanges and establish bidirectional messaging.
- [x] **Multi-Process Integration:** Verified between 2 independent OS processes (`rover_daemon.exe` with PIDs 5964 and 17592) in `scratch/test_two_processes.py`.

#### 2.12.2 Milestone 2: DTN Impairment & Real-Socket Transport (`tests/test_dtn_udp.cpp`) [Completed & Verified]
- [x] **Channel Impairment Loss Modeling:** 30% stochastic loss verified across 2,000 packets ($28.6\%$ empirical, within $\pm 4\%$).
- [x] **Channel Impairment Latency Buffer:** Outbound propagation delay buffer holds packets until monotonic deadline.
- [x] **Bilateral Link Blackouts:** Bilateral partitioning drops inter-rover packets while allowing unsevered routes to transmit.
- [x] **End-to-End DTN Delivery & ACKs:** Reliable store-and-forward transmission over real UDP sockets; receiver sends explicit 24-byte `AckPayload`; sender frees buffer on ACK.
- [x] **Store-and-Forward Blackout Recovery:** Messages accumulated during bilateral blackout remain safely buffered; all delivered in order upon link restoration.
- [x] **Retransmission & Deduplication:** Dropped packets retransmitted with exponential backoff; duplicate deliveries suppressed at receiver while re-acknowledging.
- [x] **Wire Byte Accounting Invariants:** Strict conservation:
  $$\text{Data Wire Bytes} = \text{Payload} + 36\,\text{B Header} + 20\,\text{B Framing}$$
  $$\text{ACK Wire Bytes} = 24\,\text{B} + 36\,\text{B} + 20\,\text{B} = 80\,\text{B}$$
  $$\text{Total Wire Bytes} \equiv \text{Data Wire Bytes} + \text{ACK Wire Bytes}$$
- [x] **Regression Suite:** 23 of 23 test targets passing cleanly in CTest.

#### 2.12.3 Milestone 3: Distributed Reconciliation Across Processes (`tests/test_distributed_reconciliation.cpp`) [Completed & Verified]
- [x] **IBLT Binary Serialization Round-Trip:** 32-cell IBLT ($1024\,\text{B}$ cells + $12\,\text{B}$ header = $1036\,\text{B}$) serialized and unpacked across memory buffers; verifies cell-by-cell bit integrity.
- [x] **Candidate Proposal & Response Serialization:** Validates binary encoding/decoding of keyframe IDs, 2D landmark coordinates, inlier counts, and SE(2) relative transforms.
- [x] **Real-Socket Distributed Reconciliation:** Two concurrent `RoverNode` instances exchange IBLT fingerprints, peel symmetric difference, request candidate landmarks, verify via Kabsch RANSAC, add inter-rover loop closures, and merge topological partitions in DSU from 2 to 1.
- [x] **Perceptual Aliasing Rejection:** Adversarial candidate with conflicting landmark geometry is rejected by geometric verifier; DSU partition merge is gated, preserving graph partition safety.
- [x] **Multi-Process SLAM Verification:** Verified between two independent OS processes (`rover_daemon.exe` with PIDs 19000 & 8084) in `scratch/test_two_process_slam.py`.
- [x] **Regression Suite:** 24 of 24 test targets passing cleanly in CTest.

#### 2.12.4 Milestone 4: ROS 2 & Gazebo Integration Architecture (`docs/ROS2_GAZEBO_INTEGRATION_PLAN.md`) [Completed & Verified]
- [x] **Host Virtualization Diagnostic:** Documented `Wsl/Service/E_UNEXPECTED` failure without destabilizing system.
- [x] **ROS 2 Interfaces Architecture:** Complete IDL message specifications (`LandmarkSync.msg`, `CandidateProposal.msg`, `CandidateResponse.msg`, `BoundaryState.msg`).
- [x] **ROS 2 Node Architecture:** Complete `RoverSlamNode` design wrapping `DistributedReconciliationAgent`.
- [x] **Gazebo Harmonic Lunar Model:** Complete Shackleton crater SDF world (`shackleton_crater.sdf`) and rocker-bogie URDF/Xacro models.
- [x] **Launch Orchestration:** Multi-rover launch script (`multi_rover_shackleton.launch.py`).

#### 2.12.5 Milestone 5: Distributed Scientific Experiments (`tests/benchmark_distributed_system.cpp`) [Completed & Verified]
- [x] **Factorial Sweep Harness:** 81 trials over 2, 3, 4 rovers; 0%, 20%, 50% packet loss; 0 ms, 50 ms, 100 ms latency; 3 seeds.
- [x] **Automated UDP Port Offsetting:** Unique loopback socket ports per trial preventing port collisions.
- [x] **Dataset Generation:** Output written directly to `benchmark_distributed_trials.csv` (81 rows + header).
- [x] **Data Invariant Verification:**
  - $\text{total\_wire\_bytes} \equiv \text{data\_wire\_bytes} + \text{ack\_wire\_bytes}$ holds for 100% of rows.
  - $\text{data\_wire\_bytes} \equiv \text{payload\_bytes} + \text{header\_bytes} + \text{framing\_bytes}$ holds for 100% of rows.
  - Swarm partition unification rate (`final_dsu_components == 1`): **100.0%**.

#### 2.12.6 Milestone 6: Release & Final Acceptance Audit (`scratch/test_acceptance.py`) [Completed & Verified]
- [x] **CTest Regression:** 24 of 24 targets passing (100% in 2.13 s).
- [x] **Acceptance Suite:** 13 of 13 acceptance criteria passing (`scratch/test_acceptance.py`).
- [x] **Multi-Process SLAM Script:** `scratch/test_two_process_slam.py` passing 100%.

#### 2.12.7 Milestone 7.7: Distributed System and Scientific Integrity Audit (`tests/test_audit_m77.cpp`) [Completed & Verified]
- [x] **UDP Socket Lifecycle:** Winsock initialization, non-blocking modes, port rebind, error codes.
- [x] **Message Framing:** Checksum verification, malformed datagram rejection.
- [x] **DTN Queue Limits:** Max queue size enforcement, TTL boundary purging.
- [x] **IBLT Binary Representation:** 32 cells = 1024 bytes, 1036-byte serialized frame.
- [x] **Geometric Gating:** Rigid transform estimation, perceptual aliasing rejection.
- [x] **Multi-Process Daemon Audit:** 2, 3, 4 concurrent processes, `SO_REUSEADDR` port rebind, temporary peer disconnect/reconnect (`scratch/audit_m77_multi_process.py`).
- [x] **Original Benchmark Audit:** 81/81 trials, 100% wire byte conservation, 100% DSU unification (`scratch/audit_m77_benchmark_analysis.py`).

#### 2.12.8 Milestone 7.8: Reliability Hardening and Scientific Benchmark Upgrade (`tests/benchmark_distributed_noisy.cpp`) [Completed & Verified]
- [x] **Message Schema Validation:** Explicit rejection of unsupported schema versions ($v=0, 2, 999$), corrupted version bytes, and truncated frames in `MessageFrame::deserialize()`.
- [x] **Adaptive Retransmission Timeout:** Jacobson/Karn RTT estimator (`RttEstimator`, RFC 6298) with SRTT ($\alpha=0.125$), RTTVAR ($\beta=0.25$), $RTO = SRTT + 4 \cdot RTTVAR$, Karn's rule skipping retransmissions, exponential backoff, and bounds clamping.
- [x] **Sliding Bitmap Peer Tracking:** 64-bit sliding window bitmap in `PeerTracker`, RFC 1982 modular sequence comparison, wraparound across $0xFFFFFFFF \to 0$, and stale duplicate suppression ($>64$ positions behind head).
- [x] **Scientific Noisy Benchmark Harness:** `benchmark_distributed_noisy.exe` writing to `benchmark_distributed_noisy_trials.csv` across 81 factorial trials with Gaussian landmark noise ($\sigma \in \{0.01, 0.05, 0.10\,\text{m}\}$), odometry drift ($0.02\,\text{m}$), and RANSAC outlier filtering. Verified authentic linear ATE RMSE scaling ($0.0038\,\text{m} \dots 0.0377\,\text{m}$).
- [x] **Wire Byte Accounting Invariants:** 100% exact machine-precision conservation (`total_wire == data_wire + ack_wire` and `data_wire == payload + header + framing`).
- [x] **Topological Unification:** 100.0% swarm partition unification across all 81 trials.
- [x] **Regression Suite:** 25 of 25 CTest targets passing 100% in 4.08 s.
- [x] **Master Release Report:** `docs/SPRINT_FINAL_RELEASE_REPORT.md` delivered.
