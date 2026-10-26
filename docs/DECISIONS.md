# AstroMesh-X — Architectural Decision Records (ADR)

This document records the architectural and engineering decisions made throughout the AstroMesh-X project, detailing context, evaluated alternatives, rationale, and consequences.

---

## ADR-001: Incremental Test-First Development and "No Fake Code" Policy

- **Date:** 2026-10-27
- **Status:** Accepted
- **Context:**
  Many AI-assisted or prototype codebases generate empty class stubs, mock algorithms with no functionality, and fabricated benchmarks, leading to technical debt, false progress indicators, and fragile integration.
- **Decision:**
  Enforce a strict policy: No algorithm stubs, fake classes, or mock functions may be created in early phases. Every module must be designed with explicit tests, implemented incrementally, and verified before advancing.
- **Alternatives Considered:**
  1. *Scaffold all class interfaces upfront:* Rejected because premature interfaces frequently change when real algorithmic requirements (e.g. CORD manifold operations or IBLT cell structures) are derived from primary literature.
  2. *Single monolithic prototype:* Rejected because it prevents modular benchmarking of v1 vs. vX.
- **Consequences:**
  - *Positive:* High confidence in code reliability; clear separation between verified work and future work; reproducible test logs.
  - *Negative:* Initial phases require more disciplined documentation and upfront specification before seeing functional code.

---

## ADR-002: Preservation of AstroMesh v1 as a Co-Existing Baseline

- **Date:** 2026-10-27
- **Status:** Accepted
- **Context:**
  AstroMesh-X introduces IBLT set reconciliation, CORD distributed pose-graph optimization, and connectivity-aware policies. To evaluate the scientific validity of these additions, empirical claims must be benchmarked against the original AstroMesh design.
- **Decision:**
  Retain the original AstroMesh v1 architecture (Counting Bloom Filters, standard Voronoi spatial partitioning, baseline DSU graph merging) within the test and benchmark harness as a first-class baseline.
- **Alternatives Considered:**
  1. *Replace v1 entirely:* Rejected because it eliminates the ability to demonstrate empirical gains in bandwidth efficiency, trajectory error, and convergence time.
  2. *Rely on published numbers from old reports:* Rejected because differing simulation parameters and hardware invalidate direct comparisons.
- **Consequences:**
  - *Positive:* Enables publication-quality ablation studies under identical random seeds and channel noise models.
  - *Negative:* Increases implementation scope by requiring both CBF and IBLT pipelines.

---

## ADR-003: C++ Core with Python Simulation & Evaluation Tooling

- **Date:** 2026-10-27
- **Status:** Accepted
- **Context:**
  Topological SLAM data structures, cryptographic/Murmur hashes, IBLT cell arithmetic, and Riemannian optimization steps require high computational efficiency and strict memory layout control. Conversely, multi-rover orchestration, batch Monte Carlo trials, network packet scheduling, and plotting require high productivity and rapid iteration.
- **Decision:**
  Implement the computational engine (Landmarks, IBLT, CBF, Graph DSU, CORD) in modern C++ (C++17/20 standard) with CMake, while structuring the simulation runner, dataset ingestion, and visualization tooling in Python.
- **Alternatives Considered:**
  1. *Pure Python:* Too slow for continuous Riemannian optimization iterations and large-scale graph merging experiments.
  2. *Pure C++:* High development friction for plotting, statistical analysis, and dynamic network simulation orchestration.
  3. *Rust:* Strong memory safety, but published CORD references and standard robotics optimization frameworks (GTSAM, Eigen) primarily target C++.
- **Consequences:**
  - *Positive:* Maximizes algorithmic performance where needed; enables rich Python plotting and batch automation.
  - *Negative:* Requires managing CMake build toolchains and bindings/data interfaces between C++ and Python.

---

## ADR-004: Strict Framing of IBLT and Target Metrics

- **Date:** 2026-10-27
- **Status:** Accepted
- **Context:**
  Previous informal project notes suggested that IBLT "returns the set intersection" and achieved "sub-0.5 m ATE and 1 KB payloads." Scientifically, an IBLT computes set differences $(A \oplus B)$, and quantitative metrics cannot be asserted as facts prior to empirical experimentation.
- **Decision:**
  Formalize the distinction in all documentation and code:
  - IBLT is documented strictly as a set difference reconciliation structure.
  - Metrics (1 KB, >99% precision/recall, sub-0.5 m ATE, sub-30 s synchronization) are categorized as design targets to be experimentally evaluated.
- **Consequences:**
  - *Positive:* Eliminates scientific inaccuracies and aligns the project with peer-reviewed literature standards.
  - *Negative:* Requires explicit communication in all documentation and presentations.
