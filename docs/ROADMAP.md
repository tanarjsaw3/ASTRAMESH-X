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
- **M1.1:** Shared, versioned landmark key/fingerprint specification and C++ implementation. [Completed & Verified]
- **M1.2:** AstroMesh v1 Counting Bloom Filter (CBF) candidate discovery baseline. [Completed & Verified]
- **M1.3:** AstroMesh-X Invertible Bloom Lookup Table (IBLT) set difference reconciliation engine. [Completed & Verified]
- **M1.4:** Unit and property-based test harness evaluating false-positive rates, decoding success thresholds vs. set difference size, and serialized byte footprints. [Completed & Verified]

**Completion Criteria:**
- [x] C++ unit tests verifying deterministic hash generation and fingerprint collision resistance (M1.1 verified).
- [x] CBF implementation verified for insertion, query, deletion, and empirical false-positive rate tracking (M1.2 verified).
- [x] IBLT implementation verified for insert, remove, subtraction (reconciliation), and exact decode recovery under capacity limits (M1.3 verified).
- [x] Explicit verification that IBLT decodes set *differences* (and flags failure when difference exceeds cell threshold $k$) (M1.3 verified).
- [x] Comparative memory and communication payload benchmarks under simulated set differences (M1.4 verified).


---

## Phase 2: Topological Graph Representation & Merging (DSU & Geometric Gating)

**Objective:** Build the decentralized topological map representation, loop closure candidate gating, geometric verification, and Disjoint Set Union (DSU) subgraph merging.

**Milestones:**
- **M2.1:** Topological pose-graph data structures (nodes: keyframe/landmark poses; edges: odometry, intra-rover, and inter-rover loop closures). [Completed & Verified]
- **M2.2:** Two-stage association pipeline: candidate discovery (IBLT/CBF) followed by metric geometric verification (e.g., RANSAC/PnP/relative transform consistency). [Completed & Verified]
- **M2.3:** Disjoint Set Union (DSU) graph component tracking and consistency maintenance. [Completed & Verified]
- **M2.4:** Rejection tests for perceptual aliasing and invalid inter-rover loop closures. [Completed & Verified]

**Completion Criteria:**
- [x] Unit tests for topological graph creation, edge insertion, and validation (M2.1 verified).
- [x] Geometric verification module rejects synthetic outlier associations (M2.2 verified).
- [x] Graph merging with DSU maintains valid connectivity and partition tracking (M2.3 verified).
- [x] Systematic rejection of perceptually aliased landmarks with matching keys but conflicting geometry (M2.4 verified).
- [x] Rejection of candidate prior pose conflicts, degenerate/coincident configurations, and outlier-heavy mixtures (M2.4 verified).
- [x] DSU partition safety: rejected and unverified loop closures strictly prevented from merging multi-rover subgraphs (M2.4 verified).

---

## Phase 3: Distributed Pose-Graph Optimization via CORD

**Objective:** Implement the Continuous Riemannian Dynamics (CORD) distributed pose-graph optimization algorithm based strictly on published literature (Tian et al.).

**Milestones:**
- **M3.1:** Literature review alignment, mathematical formulation, and distributed architecture design of CORD on $\mathrm{SE}(2)$ manifolds (Completed & Documented in `docs/CORD_LITERATURE_REVIEW.md`, `docs/CORD_MATHEMATICAL_FORMULATION.md`, `docs/CORD_DISTRIBUTED_DESIGN.md`).
- **M3.2:** $\mathrm{SE}(2)$ Lie-group operations, tangent-space geodesic residuals, objective evaluation, and reference Riemannian numerical gradient (`include/astromesh/geometry/se2.hpp`, `src/geometry/se2.cpp`, `include/astromesh/optimization/pose_graph_objective.hpp`, `src/optimization/pose_graph_objective.cpp`, verified by 244 test assertions in `test_se2_lie_math` and `test_pose_graph_objective`). (Completed & Verified)
- **M3.3:** Centralized baseline reference optimizer (Levenberg-Marquardt on $\mathrm{SE}(2)$ with exact gauge fixing and multi-component anchoring), synthetic benchmark suite (`StraightChain`, `ClosedLoop`, `GridManhattan`, `MultiComponent`), trajectory evaluation metrics (ATE/RPE), and reproducible CSV benchmarking (`centralized_optimizer.hpp/.cpp`, `trajectory_metrics.hpp/.cpp`, `test_centralized_optimizer`, `benchmark_centralized_optimizer`). (Completed & Verified)
- **M3.4:** CORD continuous equations of motion, semi-implicit geometric Lie integrator (`CordIntegratorSE2`), and Hamiltonian energy dissipation verification. [Completed & Verified without delay]
- **M3.5:** Multi-robot distributed execution model, serialized boundary state exchange, and neighbor velocity prediction under communication delays. [Implemented & Unit Tested; Index Bug Fixed]
- **M3.6:** Distributed benchmarks, delay/loss ablations, communication byte profiling, and independent scientific audit. [Audited & Corrected; Empirical Limitations Documented]

**Completion Criteria Status:**
- [x] **Mathematical validation of continuous Riemannian dynamics:** Supported. Derived on $\mathrm{SE}(2)$, verifying continuous Lyapunov energy identity $\dot{\mathcal{H}} \le -\sum \xi_i^\top D_i \xi_i \le 0$ identically (`docs/CORD_LITERATURE_REVIEW.md`, `docs/CORD_MATHEMATICAL_FORMULATION.md`).
- [x] **Centralized reference optimizer and synthetic benchmark baseline:** Supported. Levenberg-Marquardt optimizer on $\mathrm{SE}(2)$ verified by 128 unit test assertions and automated CSV benchmarking (M3.3).
- [x] **Zero-delay distributed CORD convergence:** Supported. Gradients and one-step updates match centralized CORD to within $10^{-6}$, achieving 93.8% cost reduction at 0 ms latency (M3.5/M3.6).
- [x] **Stochastic packet drop resilience on 2D grid graphs:** Supported. Distributed CORD achieves 88.9% cost reduction under 50% packet drop on Manhattan grid topologies without divergence.
- [ ] **Velocity-extrapolated neighbor prediction stability under nominal delay:** Contradicted / Unverified. First-order SE(2) tangent velocity prediction is dynamically unstable on stiff circular loop closures across 20 ms–500 ms latency, producing anti-damping phase lead and severe cost explosion (up to -8974% cost reduction). Open-loop tangent extrapolation is not viable without adaptive damping or DTN buffering.
- [ ] **Unconditional monotonic cost reduction under delay:** Contradicted. Delay breaks continuous Hamiltonian dissipation; both predictive extrapolation and Zero-Order Hold (ZOH) exhibit negative cost reduction under intermediate to large latencies (50 ms–200 ms).
- [x] **Physical communication byte accounting:** Supported. 80-byte binary payload verified with non-cryptographic FNV-1a/SplitMix64 checksum, paired with simulated 20-byte transport framing (100 B/packet total). Boundary keyframe index 0 bug resolved.

---

## Phase 4: Delay-Tolerant Communication Layer & Bounded-Memory Policies

**Objective:** Implement realistic networking primitives (intermittent links, packet drops, bandwidth caps, propagation delay), boundary filtering, and rover memory management.

**Milestones:**
- **M4.1:** Delay-Tolerant Boundary State Filtering & Empirical Adaptive Damping Schedule. [Completed & Verified]
- **M4.2:** Delay-Tolerant Networking (DTN) store-and-forward message buffer with retry, aging, and priority queues. [Completed & Verified]
- **M4.4:** Bounded-memory keyframe pruning and landmark eviction policies. [Completed & Verified in Phase 4.3 KeyframePruner]
- **M4.5:** Connectivity-aware exploration state machine and multi-rover orchestration layer. [Completed & Verified in Phase 4.4]
- **M4.6 (Audit):** Multi-Rover Orchestration Correctness, DTN Reliability, and Benchmark Validity Audit. [Completed & Verified in Phase 4.5]
- **M4.6 (Protocol):** Reliable Acknowledgment Protocol, Multi-Hop DTN Routing, and Delivery Audit. [Completed & Verified in Phase 4.6]

**Completion Criteria:**
- [x] **Delay-Tolerant Boundary Filtering & Adaptive Damping:** Supported. `BoundaryStateFilter` and empirical adaptive damping $D_i(\tau)$ restore positive cost reduction and stable velocities across 0 ms to 500 ms delays on circular loops. Verified by 49,191 assertions in `test_distributed_cord` and 41 reproducible trials in `benchmark_phase4_stability_results.csv` (M4.1).
- [x] **DTN Store-and-Forward Buffer:** Supported. `DtnBuffer` implements 3-tier priority, deterministic Priority Aging, exact TTL boundary expiry, exponential retry backoff, bounded dedup cache, and partition tolerance. Verified by 105 assertions in `test_dtn_buffer` and 30 trials in `benchmark_dtn_buffer_results.csv` (M4.2).
- [x] **Communication Payload Strictness:** Supported. `PacketBudgetManager` strictly distinguishes application payload bytes, protocol headers, and transport framing. Enforces sliding window byte quotas, per-peer limits, and max payload sizes with zero double-counting. Verified by 81 assertions in `test_packet_budget` (M4.2/M4.3).
- [x] **Bounded-Memory Management:** Supported. `KeyframePruner` implements deterministic geometric/temporal keyframe selection, Tarjan cut-vertex articulation-point protection, gauge anchor protection, newest keyframe protection, and verified loop-closure endpoint protection. Safely bounds graph size while strictly preserving graph connectivity (`final_components == initial_components`), preserving 100% of verified loop closures, and eliminating dangling edges. Evaluated across 13 scenarios in `benchmark_keyframe_pruner_results.csv` as a factor-removal ablation. Verified by 45 assertions in `test_keyframe_pruner` (Phase 4.3 / M4.4).
- [x] **Connectivity-Aware Multi-Rover Exploration State Machine:** Supported. `RoverStateMachine` implements 9 explicit operational states, formal legal transition matrix, diagnostic telemetry, and nanosecond timeout handling. `MultiRoverCollective` coordinates DTN store-and-forward messaging, byte budget metering, IBLT set difference reconciliation, metric geometric loop verification gating, and automatic DSU partition updates across partitions. Verified by 119 assertions in `test_rover_orchestration` and 15 reproducible scenarios in `audit_phase45_results.csv` (Phase 4.4 / Phase 4.5 Audit).
- [x] **Reliable Explicit Network Acknowledgment & Bounded Multi-Hop DTN Routing:** Supported. Replaced direct pointer simulator shortcut with 24-byte `AckPayload` protocol packet transported across simulated network links with explicit byte budget and loss modeling. Implemented bounded deterministic multi-hop store-carry-forward DTN routing with shortest-path BFS and static carrier routes, decremented `max_hops` limits, and anti-looping duplicate suppression. Verified by 10 new regression tests (Tests 20–29, 175 total assertions in `test_rover_orchestration`) and 10 reproducible scenarios in `benchmark_phase46_results.csv` with bit-for-bit SHA256 reproducibility (M4.6).




---

## Phase 5: Multi-Rover Simulation Harness & Comparative Baselines

**Objective:** Create an integrated simulation harness to evaluate multi-rover swarms running AstroMesh v1 (baseline) vs. AstroMesh-X.

**Milestones:**
- **M5.1:** Deterministic synthetic world & multi-rover trajectory harness (`SyntheticWorld2D`, `SwarmTrajectoryGenerator`, `ObservationModel2D`). [Completed & Verified]
- **M5.2:** Baseline AstroMesh v1 implementation (CBF + Voronoi exploration + basic DSU merge). [Completed & Verified]
- **M5.3:** AstroMesh-X pipeline integration (IBLT + CORD + connectivity-aware exploration). [Completed & Verified]
- **M5.4:** Automated Monte Carlo batch runners over parameterized communication drop rates and map topologies. [Completed & Verified]

**Completion Criteria:**
- [x] Head-to-head execution of AstroMesh v1 vs. AstroMesh-X under identical synthetic rover trajectories and random seeds. Supported. Verified across 300 paired Monte Carlo trials (600 pipeline runs) over 10 balanced scenarios in `benchmark_phase54_trials.csv`.
- [x] Automated generation of metrics: Absolute Trajectory Error (ATE), relative pose error, bytes transmitted, synchronization latency, and map completeness. Supported. Formatted in `benchmark_phase54_summary.csv` with paired differences, standard deviations, and 95% confidence intervals. All byte conservation invariants (100%) and 0 false acceptances held strictly.

---

## Phase 6: Experimental Evaluation, Ablations & Project Wrap-Up

**Objective:** Execute comprehensive ablation studies, analyze experimental evidence against research hypotheses, and finalize reproducible artifacts.

**Milestones:**
- **M6.1:** Communication payload ablation (IBLT vs. raw key exchange vs. best-effort no-ACKs vs. no-retries vs. single-hop routing vs. minimal transport vs. AstroMesh v1 baseline across 10 scenarios x 30 seeds = 2,100 pipeline runs). [Completed & Verified]
- **M6.2:** Robustness to packet loss (0% to 90%) and communication delays (0 ms to 500 ms) across 70 grid cells and 7 variants. [Completed & Audited: SC1 Primary 14,700 trials verified; SC10 Swarm 680 trials exploratory]
- **M6.3:** Scale tests (2 to 8+ simulated rovers via `sim_runner.exe`, Scenarios 1–10). [Completed & Verified]
- **M6.4:** Comprehensive documentation, automated report generation, interactive web UI, and final project release (`docs/FINAL_COMPLETION_REPORT.md`, `docs/DEMO_GUIDE.md`, `app/`). [Completed & Verified]

**Completion Criteria:**
- [x] M6.1 Communication payload ablation completed across 2,100 trials, Bonferroni-corrected 95% CIs computed, and causal breakdown documented in `docs/PHASE61_ABLATIONS.md`.
- [x] M6.2 Packet loss and delay sensitivity benchmark completed across 14,700 SC1 trials (70 cells $\times$ 30 seeds $\times$ 7 variants) and 680 SC10 exploratory trials, 100% byte and message conservation audited, and documented in `docs/PHASE62_ROBUSTNESS.md`.
- [x] Fully populated `docs/EXPERIMENT_LOG.md` with verifiable data and empirical analysis across all benchmark campaigns.
- [x] Clear empirical assessment of the proposed targets (1 KB payload envelope, sub-0.5 m ATE on localized loops, synchronization time).
- [x] Clean, reproducible pipeline executable (`sim_runner.exe`) paired with zero-dependency backend server (`app/server.py`) and interactive Command Center dashboard (`app/static/index.html`).
- [x] Full regression test suite passing 100% (21/21 CTest targets).

---

## Phase 7: 72-Hour Final Engineering Sprint — Distributed Multi-Rover System

**Objective:** Elevate the verified single-process simulation into an authentic, multi-process distributed rover SLAM system communicating over network sockets under real-world communication impairments, preserving all historical research and datasets.

**Sprint Milestones:**
- **M7.0: Baseline Freeze:** Verify working tree, build all 33 targets, pass 21/21 CTests, pass 12/12 API acceptance tests, audit benchmark CSVs, probe WSL2/ROS2 environment. [Completed & Verified]
- **M7.1: Independent Rover Processes:** Standalone rover node architecture (`RoverNode`), 36-byte binary framing (`MessageFrame`), UDP socket transport (`UdpSocket`), sliding-window sequence tracking and peer vitality (`PeerTracker`), standalone daemon (`rover_daemon.exe`). Bidirectional discovery and communication verified between 2 independent OS processes. [Completed & Verified]
- **M7.2: Network Impairment & DTN Integration:** Channel impairment emulator (`ChannelImpairment`) modeling packet loss ($0\%\dots90\%$), transit latency ($0\dots500\,\text{ms}$), and blackouts. Store-and-forward `DtnBuffer` integration, 24-byte explicit ACKs (`AckPayload`), exponential backoff retries, duplicate suppression, and strict wire byte accounting invariants. [Completed & Verified]
- **M7.3: Distributed Reconciliation Across Processes:** Distributed IBLT candidate discovery, geometric verification, DSU partition merging, and boundary state exchange across network sockets between independent rover processes. Verified via `test_distributed_reconciliation` (4/4 tests) and `scratch/test_two_process_slam.py` (PIDs 19000 & 8084 unifying components over UDP). [Completed & Verified]
- **M7.4: ROS 2 & Gazebo Scoping:** Transparent technical audit documenting host virtualization constraints (WSL2 E_UNEXPECTED) and providing complete architectural blueprint for ROS 2 package layout (`astromesh_interfaces`, `astromesh_core`) and Gazebo SDF/URDF nodes in `docs/ROS2_GAZEBO_INTEGRATION_PLAN.md`. [Completed & Verified]
- **M7.5: Distributed Scientific Experiments:** Multi-process benchmark campaign (`benchmark_distributed_trials.csv`) validating distributed convergence, ATE, and wire bytes over UDP under loss ($0\%\dots50\%$) and delay ($0\dots100\,\text{ms}$) across 81 trials. Verified 100% wire conservation, 100% swarm partition unification, and recorded in `benchmark_distributed_trials.csv`. [Completed & Verified]
- **M7.6: Release & Final Deliverables:** Integration regression run (24/24 CTest passed in 2.13 s), final acceptance suite (13/13 passed in `scratch/test_acceptance.py`), completion matrix, exact reproduction guide, and future-work roadmap delivered in `docs/SPRINT_FINAL_RELEASE_REPORT.md`. [Completed & Verified]
- **M7.7: Distributed System and Scientific Integrity Audit:** Independent verification of multi-process execution (2, 3, 4 rovers), UDP socket lifecycle, error handling, DTN queue limits and TTL expiration, IBLT 1024B/1036B representation, full-mesh concurrency, 81-trial benchmark audit, and critical ATE noise analysis. Tested via `test_audit_m77` (25/25 CTests passing), `scratch/audit_m77_multi_process.py`, and documented in `docs/M7_7_DISTRIBUTED_INTEGRITY_AUDIT.md`. [Completed & Verified]
- **M7.8: Reliability Hardening and Scientific Benchmark Upgrade:** Schema version validation (rejection of v0, v2, v999, corrupted, truncated frames), adaptive Jacobson/Karn RTO (`RttEstimator`, RFC 6298, Karn's rule, exponential backoff, clamping), fixed-width 64-bit sliding window bitmap (`PeerTracker`, RFC 1982 wraparound, stale replay suppression), 81-trial Gaussian observation noise benchmark (`benchmark_distributed_noisy_trials.csv`, authentic ATE RMSE scaling $0.0038\,\text{m} \dots 0.0377\,\text{m}$), and full regression verification (25/25 CTest, 13/13 acceptance, multi-process audit, benchmark analysis). Documented in `docs/M7_8_RELIABILITY_AND_SCIENTIFIC_VALIDATION.md`. [Completed & Verified]
- **Phase 7.9: ROS 2 Migration Planning and Environment Readiness:** Environment probe diagnosing `Wsl/Service/E_UNEXPECTED` as disabled CPU virtualization in firmware (`VirtualizationFirmwareEnabled: False`), 100% C++17 POSIX/Linux portability audit, minimal practical ROS 2 node architecture design, and two-rover proof-of-concept specification. Fully documented in `docs/PHASE_7_9_ROS2_MIGRATION_READINESS.md`. [Completed & Verified]
- **M7.10: Multi-Keyframe Trajectory SLAM, Distributed CORD Optimization & Remediation Validation:** Multi-keyframe trajectory ingestion with accumulated Brownian odometry drift, integrated distributed CORD continuous Riemannian dynamics over UDP relaxing trajectory ATE (86.1% error reduction on intra-rover closure, 92.8% on perturbed trajectory, max 57.09% reduction across 81 trials), corrected inter-rover loop closure transform inversion, implemented true global swarm partition accounting (`GlobalSwarmEvaluator::evaluate_graphs()`), adversarial perceptual aliasing rejection instrumenting Precision (100.0%), Recall (100.0%), and zero False Positives ($FP = 0$), bounded-memory keyframe pruning, 26/26 passing CTests in 4.08s, 13/13 passing acceptance tests, and 81-trial factorial validated benchmark sweep recorded in `benchmark_distributed_trajectory_validated.csv`. Fully reconciled confusion matrix ($TP=153, FP=0, TN=162, FN=0$), decoupled proposal counts from graph edges, and matched paired ATE improvement ($0.1887\,\text{m}$) identically with difference of aggregate means. Documented in `docs/M7_10_DEVELOPMENT_REPORT.md`, `docs/M7_10_INDEPENDENT_AUDIT.md`, and `docs/M7_10_REMEDIATION_REPORT.md`. [Completed & Verified]
- **M7.11: Multi-Hop Distributed SLAM Routing & Live UDP Dashboard Streaming:** Distance-vector multi-hop routing with split horizon, poison reverse, and route epochs, hop-by-hop custody forwarding with duplicate custody ACK regeneration, loop prevention via capacity-matched path-vector trail (`uint16_t[8]`, `MaxHops <= 8`), store-carry-forward DTN transit buffering during link disconnections, dual-constraint in-memory capacity bounds (100 frames / 1 MB), delivery acknowledgments, partition detection, 45-trial factorial benchmarks (`benchmark_multihop_trials.csv` and `benchmark_multihop_trials_reconciled.csv`), and live UDP dashboard streaming engine in `app/server.py` streaming real-time pose-graph, routing, and CORD relaxation telemetry over SSE to the web UI. Documented in `docs/M7_11_ARCHITECTURE.md`, `docs/M7_11_IMPLEMENTATION_PLAN.md`, ADR-038, `docs/M7_11_DEVELOPMENT_REPORT.md`, `docs/M7_11_FINAL_VERIFICATION_AUDIT.md`, and `docs/M7_11_RELEASE_SUMMARY.md`. [CLOSED & FROZEN - 30/30 CTests, 13/13 Acceptance Tests, Multi-Process Verified]
- **M7.12: Robust Distributed SLAM Under Network Partitions: Consistency, Recovery, and Adaptive Communication:** Versioned distributed map-state synchronization using causal vector clocks/version vectors, partition-aware multi-round IBLT reconciliation with decode failure retry and backlog tracking, conflict-aware deterministic graph merging with provenance preservation and dynamic connectivity/rebuild semantics, partition recovery and global consistency verification, measured adaptive communication control (loss/delay/queue-aware rate & batch adjustment), deterministic failure-injection framework (loss, burst loss, latency, jitter, reordering, link drop, relay crash/restart), 5-topology factorial benchmark campaign (`benchmark_m712_partition_recovery.csv` passing 45/45 trials, 38.6% to 59.3% bandwidth savings over full-map exchange), and extended live dashboard streaming for partition consistency. [Completed & Verified - 36/36 CTests, 13/13 Acceptance Tests, Multi-Process UDP Verified]
- **M7.13: Decentralized Gauge Fixing & Globally Consistent Metric SLAM:** Closed-form 2D Kabsch/Umeyama SE(2) subgraph alignment (`SubgraphAligner`) with spatial variance pre-filtering ($\operatorname{Var}(P) \ge 0.05\,\text{m}^2$) and 50-iteration RANSAC consensus inlier filtering; directed inter-rover alignment graph (`AlignmentGraph`) with monotonic epochs, Dijkstra maximum-confidence spanning path resolution, and 3-cycle Lie consistency auditor detecting closed-loop contradictions ($\| \log(T_{\text{cycle}}) \| > 0.35\,\text{m}$) and pruning worst-RMSE edges; decentralized gauge coordinator (`GaugeCoordinator`) with deterministic minimum-rover-ID anchor election ($R_{\text{anchor}} = \min_{R \in \mathcal{C}} R$), dynamic anchor failover upon node loss/partition, and 48-byte packed wire record serialization; unbiased swarm ATE evaluation applying a single global rigid transformation $T_{\text{eval}} \in \mathrm{SE}(2)$ to the combined swarm map; empirical benchmark suite (`benchmark_m713_gauge_alignment.csv` passing 45/45 trials 100%, Global ATE $< 0.081\,\text{m}$ across 2, 3, 4, 8, 16 rovers); live UDP dashboard telemetry streaming anchor and cycle metrics. [Completed & Verified - 40/40 CTests, 13/13 Acceptance Tests, Multi-Process Dashboard Verified]
- **M7.14: Active Multi-Robot Loop Closure & Communication-Efficient Rendezvous:** Candidate opportunity tracking and lifecycle management (`LoopClosureOpportunityManager`); decentralized information-gain proxy evaluation combining graph geodesic drift ($\sqrt{d_{\text{graph}}}$), 2D landmark spatial dispersion ($\log(1 + \det(S_{\text{geom}}))$), and topological bridge values (`InformationGainEvaluator`); multi-attribute active loop closure scheduling with network channel state awareness and two-pass DSU spanning tree budget allocation (`ActiveLoopClosureScheduler`); compact three-way rendezvous protocol (`RendezvousProtocolAgent`) with wire messages bounded below 1024 bytes; live UDP telemetry and dashboard streaming; 45-trial factorial empirical benchmark (`benchmark_m714_active_loop_closure.csv`) passing 45/45 trials (100%), 100% precision (0 false positives), Global ATE $< 0.19\,\text{m}$ across all configurations, and up to $95.7\%$ communication wire reduction compared to passive broadcasts. [Completed & Verified - 45/45 CTests, 13/13 Acceptance Tests, Telemetry Verified]
- **M7.15: Dynamic Map Compaction & Memory-Bounded Distributed SLAM:** Unified memory accounting (`MemoryTracker`) strictly distinguishing logical data size, container capacity, serialized wire size, and kernel process RSS queried via Windows `GetProcessMemoryInfo`; configurable memory budget controller (`MemoryBudgetController`) with tri-tier pressure (`Normal` $< 0.75$, `Warning` $\in [0.75, 0.90)$, `Critical` $\ge 0.90$) and a $0.65 \times B$ hysteresis recovery floor; conservative degree-2 topological compounding (`TopologicalMapCompactor`) compounding non-branching odometry chains ($T_{ik} = T_{ij} \cdot T_{jk}$, information composition, landmark reassociation) with zero fill-in while strictly protecting indispensable nodes (branching nodes, loop closure endpoints, exploration frontiers, gauge anchors); bounded landmark lifecycle management (`LandmarkLifecycleManager`) capping observations and evicting low-confidence singletons while protecting mutual loop closure support landmarks; distributed wire compaction protocol (`DistributedCompactionProtocol`) transmitting 68-byte `CompactionNotification` and 14-byte `CompactionAck` with monotonic epochs and FNV-1a checksums; full 45-trial benchmark sweep (`benchmark_m715_memory_bounded_slam.csv`) achieving 45/45 trials PASS (100%), up to 78.5% memory reduction, Global ATE $< 0.05\,\text{m}$, and 100% precision; live dashboard UDP telemetry ingestion (`app/telemetry_collector.py`). [Completed & Verified - 52/52 CTests, 13/13 Acceptance Tests, Telemetry Verified]
- **M7.16: Autonomous Exploration & Connectivity-Aware Path Planning:** Topological radial sector frontier detection (`TopologicalFrontierDetector`) eliminating raster occupancy grids; deterministic multi-attribute utility model (`ExplorationUtilityModel`) balancing information dispersion $\det(S)$, travel cost, geometric novelty, super-linear connectivity breach risk, memory pressure, and peer redundancy; connectivity-aware path planning (`ConnectivityPathPlanner`) with anti-oscillation commitment hysteresis ($\Delta_{\text{switch}} = 0.15$) and safety-critical `ReturnToSwarm` fallback; decentralized multi-rover coordination (`MultiRoverExplorationCoordinator`) exchanging 44-byte packed `FrontierClaim` wire packets with deterministic dominance (utility $\to$ epoch $\to$ ID); integrated exploration agent (`IntegratedExplorationAgent`) coordinating frontier exploration, M7.14 loop closure scheduling, and M7.15 memory-bounded map compaction; live UDP telemetry streaming exploration metrics (`app/telemetry_collector.py`); full 45-trial factorial empirical benchmark (`benchmark_m716_autonomous_exploration.csv`) passing 45/45 trials (100%), Global ATE $< 0.035\,\text{m}$, duplicate exploration $< 4.5\%$, connectivity uptime $\ge 98.5\%$, and 0 false positive loop closures. [Completed & Verified - 58/58 CTests, 13/13 Acceptance Tests, Telemetry Verified]
- **M7.17: End-to-End Scientific Benchmark Validation & Reproducibility:** Comprehensive audit of all prior milestone claims and datasets; enforcement of single global rigid $\mathrm{SE}(2)$ alignment for multi-rover ATE; adversarial verification of ground-truth isolation and metric fraud prevention (`test_m717_ate_alignment`); qualification of synthetic zero false positives ($FP = 0$) versus real-world perceptual aliasing; transparent wire byte accounting ($\text{Total Wire} \equiv \text{Data Wire} + \text{ACK Wire}$); distinction of physical process RSS from logical container capacity; automated reproducibility harness (`scratch/reproducibility_harness.py`) generating 2,000-resample bootstrap 95% confidence intervals, Cohen's $d$ effect sizes ($d > 2.0$, dominant effect across all domains), and Wilcoxon signed-rank significance tests ($p < 0.001$); immutable SHA-256 historical dataset integrity preservation; taxonomic classification of all claims in `docs/SCIENTIFIC_EVIDENCE_MATRIX.md` and `benchmark_m717_evidence_matrix.csv`. [Completed & Verified - 65/65 CTests, 13/13 Acceptance Tests, Reproducibility Harness Verified]
- **M7.18: Interactive Research Simulator & Live Evaluation Dashboard:** Interactive software-only mission control dashboard (`app/static/index.html`) with procedural 2D lunar visualizer, 60 FPS animated packet pulses along active communication links, multi-rover support up to 16 rovers and 120 links (Demo 4), interactive network fault injection (sever/heal partition triggers), real-time Rover Inspector, research replay mode with frame-level scrubbing and event jump controls, comparative Baseline vs AstroMesh-X architecture view, dedicated Judge / Presentation Mode with 5-stage evaluator walkthrough narrative, and one-click research exports (CSV, JSON, PNG snapshot, Markdown report). Verified via C++ test suite `tests/test_m718_simulator_telemetry.cpp` and Python integration suite `scratch/test_m718_simulator_integration.py`. Full CTest regression passing 66/66 (100% in 9.77s) and acceptance suite passing 13/13 (100%). [Completed & Verified - 66/66 CTests, 13/13 Acceptance Tests, Simulator Verified]
- **M7.18 (Manuscript): Research Manuscript & Publication Package:** Complete archival IEEE research paper (`docs/ASTROMESH_X_RESEARCH_MANUSCRIPT.md`) structured across 25 sections in IEEE Transactions on Robotics (T-RO) / RA-L format; independent pre-publication scientific audit (`docs/M7_18_SCIENTIFIC_AUDIT.md`) identifying and correcting data-aggregation conflations (true M7.13 proposed ATE is 0.0282m, 2.8cm, vs 27.97m unaligned baseline); reproducible figures and results compendium (`docs/M7_18_RESULTS_AND_FIGURES.md`); complete scientific reproducibility package (`docs/M7_18_REPRODUCIBILITY_PACKAGE.md`) with automated non-destructive validator (`scratch/validate_reproducibility_package.py`) validating schemas, finite floats, and exact SHA-256 digests across all 8 canonical benchmark CSVs; publication readiness assessment and claims classification matrix (`docs/M7_18_PUBLICATION_READINESS.md`); ADR-046 recorded. All historical datasets preserved byte-identical. [Completed & Verified - 66/66 CTests, 13/13 Acceptance Tests, Publication Package Verified]



