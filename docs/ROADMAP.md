# AstroMesh-X — Multi-Phase Development Roadmap

This roadmap defines the sequential, test-first development phases for AstroMesh-X. Each phase establishes explicit, verifiable completion criteria that must be satisfied before transitioning to subsequent phases.

---

## Phase 0: Repository Bootstrap & Project Memory

**Objective:** Establish the development environment, baseline repository structure, persistent project memory docs, operational guidelines, and baseline toolchain discovery without prematurely implementing unverified algorithms.

**Milestones:**
- **M0.1:** Environment discovery, repository configuration (`.gitignore`, `CMakeLists.txt`, `README.md`, `AGENTS.md`).
- **M0.2:** Scaffolding of documentation files (`PROJECT_STATE.md`, `ROADMAP.md`, `ARCHITECTURE.md`, `DECISIONS.md`, `BUILD_LOG.md`, `TEST_PLAN.md`, `EXPERIMENT_LOG.md`, `REFERENCES.md`).
- **M0.3:** Initial CMake configuration validation and Phase 0 acceptance review.

**Completion Criteria:**
- [x] Clean repository layout verified with no fake or stub algorithm code.
- [x] Environment capabilities (compilers, CMake, Python) inspected and logged.
- [x] All persistent memory files established with substantive, project-specific content.
- [x] Explicit scientific caveats (IBLT difference reconciliation, unverified empirical targets) documented.
- [ ] User review and acceptance granted.

---

## Phase 1: Landmark Keys & Set Reconciliation (IBLT vs. CBF Baseline)

**Objective:** Implement and benchmark the core probabilistic and deterministic data structures for landmark identification and set reconciliation between rovers.

**Milestones:**
- **M1.1:** Shared, versioned landmark key/fingerprint specification and C++ implementation.
- **M1.2:** AstroMesh v1 Counting Bloom Filter (CBF) candidate discovery baseline.
- **M1.3:** AstroMesh-X Invertible Bloom Lookup Table (IBLT) set difference reconciliation engine.
- **M1.4:** Unit and property-based test harness evaluating false-positive rates, decoding success thresholds vs. set difference size, and serialized byte footprints.

**Completion Criteria:**
- [ ] C++ unit tests verifying deterministic hash generation and fingerprint collision resistance.
- [ ] CBF implementation verified for insertion, query, deletion, and empirical false-positive rate tracking.
- [ ] IBLT implementation verified for insert, remove, subtraction (reconciliation), and exact decode recovery under capacity limits.
- [ ] Explicit verification that IBLT decodes set *differences* (and flags failure when difference exceeds cell threshold $k$).
- [ ] Comparative memory and communication payload benchmarks under simulated set differences.

---

## Phase 2: Topological Graph Representation & Merging (DSU & Geometric Gating)

**Objective:** Build the decentralized topological map representation, loop closure candidate gating, geometric verification, and Disjoint Set Union (DSU) subgraph merging.

**Milestones:**
- **M2.1:** Topological pose-graph data structures (nodes: keyframe/landmark poses; edges: odometry, intra-rover, and inter-rover loop closures).
- **M2.2:** Two-stage association pipeline: candidate discovery (IBLT/CBF) followed by metric geometric verification (e.g., RANSAC/PnP/relative transform consistency).
- **M2.3:** Disjoint Set Union (DSU) graph component tracking and consistency maintenance.
- **M2.4:** Rejection tests for perceptual aliasing and invalid inter-rover loop closures.

**Completion Criteria:**
- [ ] Unit tests for topological graph creation, edge insertion, and serialization.
- [ ] Geometric verification module rejects synthetic outlier associations.
- [ ] Graph merging with DSU maintains valid connectivity and correctly detects cycles without graph corruption.

---

## Phase 3: Distributed Pose-Graph Optimization via CORD

**Objective:** Implement the Continuous Riemannian Dynamics (CORD) distributed pose-graph optimization algorithm based strictly on published literature (Tian et al.).

**Milestones:**
- **M3.1:** Literature review alignment and mathematical formulation of CORD on SE(2)/SE(3) Riemannian manifolds.
- **M3.2:** Local gradient formulation and continuous-time Riemannian gradient descent step updates.
- **M3.3:** Inter-rover message exchange protocol for boundary variables (poses associated with inter-rover loop closures).
- **M3.4:** Distributed convergence test harness on standard SLAM benchmark graphs (e.g., 2D/3D synthetic tori, Manhattan grid datasets).

**Completion Criteria:**
- [ ] Mathematical validation confirming Riemannian gradient derivation matches published CORD literature.
- [ ] Convergence verification on synthetic distributed pose graphs without central coordination.
- [ ] Comparison against centralized pose graph optimization (e.g., g2o/GTSAM reference solutions) for trajectory accuracy and residual error.

---

## Phase 4: Delay-Tolerant Communication Layer & Bounded-Memory Policies

**Objective:** Implement realistic networking primitives (intermittent links, packet drops, bandwidth caps, propagation delay) and rover memory management.

**Milestones:**
- **M4.1:** Delay-Tolerant Networking (DTN) store-and-forward message buffer with retry and aging.
- **M4.2:** Communication budget enforce engine (measuring payload sizes against 1 KB, 5 KB, 20 KB budget targets).
- **M4.3:** Bounded-memory keyframe pruning and landmark eviction policies.
- **M4.4:** Connectivity-aware exploration state machine (returning to relay positions or clustering when disconnected).

**Completion Criteria:**
- [ ] Simulated link drops and delays handled gracefully without deadlock or message corruption.
- [ ] Bounded-memory manager maintains constant memory profile within configured limits under extended operation.
- [ ] Communication payload strictly tracked in bytes across simulated sessions.

---

## Phase 5: Multi-Rover Simulation Harness & Comparative Baselines

**Objective:** Create an integrated simulation harness to evaluate multi-rover swarms running AstroMesh v1 (baseline) vs. AstroMesh-X.

**Milestones:**
- **M5.1:** Discrete-event or stepping multi-rover simulation environment (Python/C++ bindings).
- **M5.2:** Baseline AstroMesh v1 implementation (CBF + Voronoi exploration + basic DSU merge).
- **M5.3:** AstroMesh-X pipeline integration (IBLT + CORD + connectivity-aware exploration).
- **M5.4:** Automated Monte Carlo batch runners over parameterized communication drop rates and map topologies.

**Completion Criteria:**
- [ ] Head-to-head execution of AstroMesh v1 vs. AstroMesh-X under identical synthetic rover trajectories and random seeds.
- [ ] Automated generation of metrics: Absolute Trajectory Error (ATE), relative pose error, bytes transmitted, synchronization latency, and map completeness.

---

## Phase 6: Experimental Evaluation, Ablations & Project Wrap-Up

**Objective:** Execute comprehensive ablation studies, analyze experimental evidence against research hypotheses, and finalize reproducible artifacts.

**Milestones:**
- **M6.1:** Communication payload ablation (IBLT vs. CBF vs. full graph exchange).
- **M6.2:** Robustness to packet loss and communication delays (0% to 90% drop rates).
- **M6.3:** Scale tests (2 to 10+ simulated rovers).
- **M6.4:** Comprehensive documentation, automated report generation, and final paper preparation.

**Completion Criteria:**
- [ ] Fully populated `docs/EXPERIMENT_LOG.md` with verifiable data and plots.
- [ ] Clear empirical assessment of the proposed targets (1 KB payload, sub-0.5 m ATE, synchronization time).
- [ ] Clean, reproducible pipeline executable with a single orchestration script.
