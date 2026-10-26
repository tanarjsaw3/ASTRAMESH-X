# AstroMesh-X

A research software prototype for collaborative topological SLAM across multiple simulated planetary rovers operating without GPS under constrained, delayed, and unreliable communication.

---

## 1. Project Overview

AstroMesh-X investigates how decentralized rover swarms can build and optimize a shared topological-metric map of unfamiliar terrain without relying on central infrastructure or satellite positioning. Communications in planetary exploration regimes (such as lunar lava tubes, Martian canyons, or cratered terrain) are characterized by severe bandwidth throttling, high packet drop rates, and long propagation delays.

The project evaluates two architectural paradigms side-by-side:
1. **AstroMesh (Baseline / v1):**
   - Fingerprint/landmark keys.
   - Counting Bloom Filter (CBF) candidate discovery.
   - Geometric verification of candidate associations.
   - Topological graph merging and Disjoint Set Union (DSU).
   - Delay-Tolerant Network (DTN) message store-and-forward buffering.
   - Voronoi-style spatial exploration partitioning.
2. **AstroMesh-X (Proposed):**
   - Invertible Bloom Lookup Table (IBLT) set reconciliation for difference detection.
   - Shared, versioned landmark-key scheme.
   - Two-stage candidate discovery and rigorous geometric verification.
   - Topological merging with loop closure gating.
   - Distributed pose-graph optimization via Continuous Riemannian Dynamics (CORD).
   - Connectivity-aware exploration policies and bounded-memory management.

---

## 2. Scientific Principles & Clarifications

To maintain scientific integrity across research, simulation, and publication:
- **Set Reconciliation:** An Invertible Bloom Lookup Table (IBLT) does **not** directly compute set intersections. It is an efficient data structure used to reconcile set **differences** between peers, subject to successful decoding limits.
- **Evaluation Targets vs. Empirical Facts:** Numerical metrics such as 1 KB transmission budgets, >99% precision/recall, sub-0.5 m Absolute Trajectory Error (ATE), and sub-30 s sync times are **proposed design targets under experimental evaluation**, not established achievements.
- **Novelty & Prior Art:** IBLT and CORD are existing published techniques. Their implementation in this repository faithfully references published literature (CORD: Tian et al.; IBLT: Goodrich & Mitzenmacher). Any novel contribution lies in the integration, topological abstraction, and communication-constrained coordination policies, which must be validated empirically.
- **Baselines:** The original AstroMesh v1 pipeline is maintained as a fair benchmark to rigorously isolate and quantify the benefits and overheads introduced by AstroMesh-X.

---

## 3. Repository Architecture

```text
ASTRAMESH-X/
├── .gitignore               # Ignored build and runtime artifacts
├── AGENTS.md                # AI agent operating rules & persistent memory guidelines
├── CMakeLists.txt           # Minimal test-first build configuration
├── README.md                # Project introduction and orientation
├── docs/                    # Persistent project memory and specifications
│   ├── ARCHITECTURE.md      # Conceptual and module architecture
│   ├── BUILD_LOG.md         # Verifiable log of executed build/test commands
│   ├── DECISIONS.md         # Architectural Decision Records (ADRs)
│   ├── EXPERIMENT_LOG.md    # Hypotheses, planned experiments, and empirical results
│   ├── PROJECT_STATE.md     # Single source of truth for current status and next actions
│   ├── REFERENCES.md        # Academic and literature bibliography
│   ├── ROADMAP.md           # Multi-phase roadmap and acceptance criteria
│   └── TEST_PLAN.md         # Verification strategy and test matrix
├── include/                 # C++ header files (to be populated in Milestone 1+)
├── src/                     # C++ core implementation (to be populated in Milestone 1+)
├── tests/                   # C++ unit and integration tests (to be populated in Milestone 1+)
└── experiments/             # Python-based simulation, orchestration, and evaluation scripts
```

---

## 4. Current Phase: Phase 0 (Bootstrap)

- **Phase Status:** Repository bootstrap and persistent memory initialization.
- **Implementation Status:** No algorithm implementation is included in Phase 0. Toolchain and environment discovery complete; documentation and build structure established.

See [PROJECT_STATE.md](docs/PROJECT_STATE.md) for the live state of development.
