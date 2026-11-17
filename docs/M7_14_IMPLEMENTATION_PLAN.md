# Milestone 7.14 Implementation Plan: Active Multi-Robot Loop Closure & Communication-Efficient Rendezvous

---

## 1. Phased Execution Roadmap

### Stage 1: Loop-Closure Opportunity Detection (`include/astromesh/coordination/loop_closure_opportunity.hpp`, `src/coordination/loop_closure_opportunity.cpp`)
- Define `LoopClosureOpportunity` struct storing candidate keyframes, spatial proximity, temporal separation, landmark overlap, verification feasibility, and priority.
- Implement `LoopClosureOpportunityManager` generating spatial, historical revisitation, and partition-reconnection opportunities.
- Early rejection of redundant/degenerate candidates.
- Unit tests: `tests/test_loop_closure_opportunity.cpp`.

### Stage 2: Information-Gain Evaluation (`include/astromesh/coordination/information_gain.hpp`, `src/coordination/information_gain.cpp`)
- Implement `InformationGainEvaluator` computing the proxy metrics derived in ADR-041:
  - Graph distance / trajectory drift reduction proxy: $\sqrt{d_{\text{graph}}}$.
  - Landmark spatial scatter determinant: $\det(S_{\text{geom}})$.
  - Topological bridge factor: $1.0 / (1.0 + E_{\text{existing}})$.
  - Redundancy penalty for existing close edges.
- Unit tests: `tests/test_information_gain.cpp`.

### Stage 3: Active Loop-Closure Prioritization (`include/astromesh/coordination/active_loop_scheduler.hpp`, `src/coordination/active_loop_scheduler.cpp`)
- Implement multi-attribute scoring function combining information gain, geometric dispersion, correspondence quality, verification probability, network link quality, and wire cost.
- Deterministic tie-breaking and spatial cluster suppression.
- Communication budget enforcement: `schedule_batch(max_count, max_wire_budget)`.
- Unit tests: `tests/test_active_loop_scheduler.cpp`.

### Stage 4: Communication-Efficient Rendezvous Protocol (`include/astromesh/coordination/rendezvous_protocol.hpp`, `src/coordination/rendezvous_protocol.cpp`)
- Define wire messages: `RendezvousInquiry` (32 bytes), `RendezvousResponse` (compact bounded landmarks $\le 16$), and verified `LoopClosureProposal`.
- Implement rendezvous state machine with timeout, retry backoff, and duplicate suppression.
- Unit tests: `tests/test_rendezvous_protocol.cpp`.

### Stage 5: End-to-End System Integration & Failure Injection (`tests/test_m714_active_loop_integration.cpp`)
- Integrate `OpportunityManager`, `ActiveScheduler`, `RendezvousProtocol`, `GeometricVerifier`, `ConflictAwareGraphMerger`, and `GaugeCoordinator`.
- Test scenarios:
  1. Useful 2-rover active loop closure.
  2. 4-rover candidate prioritization under tight bandwidth budget.
  3. False candidate rejection under adversarial aliasing.
  4. Partition recovery rendezvous.
  5. Relay drop / timeout recovery during rendezvous.

### Stage 6: Empirical Benchmark Suite (`tests/benchmark_m714_active_loop_closure.cpp` -> `benchmark_m714_active_loop_closure.csv`)
- 45-trial factorial matrix comparing:
  - A. Passive loop closure.
  - B. Existing loop-closure exchange.
  - C. Active scheduler without information scoring.
  - D. AstroMesh-X active information-aware scheduler.
- Across 2, 3, 4, 8, and 16 rovers under varying noise and bandwidth budgets.

### Stage 7: Telemetry, Verification & Documentation
- Update `app/telemetry_collector.py` for active loop closure metrics.
- Regression verification: all 40 CTests + all new M7.14 tests (100% pass).
- Acceptance verification: 13/13 tests pass.
- Deliver `docs/M7_14_DEVELOPMENT_REPORT.md`, `docs/M7_14_BENCHMARK_METHODOLOGY.md`, `docs/M7_14_FINAL_VERIFICATION_AUDIT.md`.
- Update `docs/PROJECT_STATE.md`, `docs/ROADMAP.md`, `docs/DECISIONS.md` (ADR-041), and `docs/BUILD_LOG.md`.

---

## 2. Key Verification Invariants

1. **Envelope Bound Invariant:** All rendezvous messages $\le 1024$ bytes.
2. **Quality Over Quantity Invariant:** Active scheduling achieves lower Global ATE with $\ge 40\%$ fewer transmitted wire bytes than passive broadcast.
3. **Zero Ground-Truth Contamination:** All candidate scoring, verification, and scheduling decisions rely exclusively on onboard observations and topological metadata.
4. **Conservation of Communication Bytes:** All wire byte meters strictly track payload + headers + ACKs without undercounting.

