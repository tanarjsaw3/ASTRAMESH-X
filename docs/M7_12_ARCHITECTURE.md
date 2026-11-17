# AstroMesh-X — Milestone 7.12 Architecture Specification

**Milestone Title:** M7.12 — Robust Distributed SLAM Under Network Partitions: Consistency, Recovery, and Adaptive Communication  
**Document:** Architecture Specification & Mathematical Formulation  
**Status:** Approved for Implementation  
**Auditor / Architect:** Principal Network Systems Engineer & SLAM Researcher  
**Target Environment:** MSYS2 UCRT64 GCC 16.2.0, Ninja 1.13.2, CMake, C++17  

---

## 1. Research Question & Hypotheses

### 1.1 Central Research Question
*How can decentralized peer-to-peer rovers maintain mathematically consistent, causal, and recoverable topological pose-graph SLAM states under severe, protracted network partitions, delayed and out-of-order packet delivery, relay churn, and burst packet loss without a central authority or global clock?*

### 1.2 Formal Hypotheses
- **Hypothesis 1 (Causal Consistency via Version Vectors):** Tracking per-keyframe and per-landmark map mutations using bounded $O(K)$ Version Vectors (where $K$ is the number of rovers in the swarm) eliminates stale overwrites and permits idempotent reconciliation of concurrently discovered landmarks post-partition without physical timestamp dependencies.
- **Hypothesis 2 (Partition-Aware Multi-Round IBLT):** Combining version vectors with adaptive multi-round Invertible Bloom Lookup Table (IBLT) difference peeling resolves symmetric set differences $A \Delta B > \text{capacity}$ through iterative difference decomposition, reducing transmitted wire bytes by $>60\%$ compared to full-map exchange baselines during partition reconnection.
- **Hypothesis 3 (Conflict-Aware Graph Merge Determinism):** Separating local keyframe indexes from immutable global node identifiers $(rover\_id, local\_index)$ and enforcing provenance-tracked edge insertion with deterministic DSU dynamic rebuilding guarantees that rejected imposter proposals cannot corrupt topological connectivity, preserving partition isolation.
- **Hypothesis 4 (Adaptive Communication Under Network Degradation):** Dynamically modulating message transmission rates, DTN retry intervals, and IBLT cell budgets based on measured link loss, RTT, and queue occupancy prevents transit buffer exhaustion and preserves critical loop-closure verification ratios under $>20\%$ burst channel loss.

---

## 2. System & Failure Models

### 2.1 Swarm System Model
- **Rovers:** A swarm of $N$ autonomous rovers $R = \{r_1, r_2, \dots, r_N\}$, where $N \le 16$.
- **Graph State:** Each rover $r_i$ maintains an authoritative local pose-graph $\mathcal{G}_i = (\mathcal{V}_i, \mathcal{E}_i)$:
  - $\mathcal{V}_i$: Keyframe nodes $v = (r_k, j)$, carrying estimated SE(2) pose $\mathbf{T}_{v} \in \mathrm{SE}(2)$ and landmark observations $\mathcal{Z}_v = \{z_1, z_2, \dots\}$.
  - $\mathcal{E}_i$: Rigid constraints (Odometry, Verified Intra-Rover Loop Closures, Verified Inter-Rover Loop Closures).
- **Communication:** Communication is peer-to-peer over UDP loopback/mesh sockets using the 40-byte `MultihopEnvelope` and 36-byte `MessageFrame` protocols.

### 2.2 Channel Failure & Partition Model
- **Network Partitions:** The communication topology graph $G_t = (R, E_t)$ can partition into $M \ge 2$ disconnected subgraphs $S_1, S_2, \dots, S_M$ for arbitrary durations $T_{\text{part}}$.
- **Packet Loss:** Link-level Bernoulli loss $p_{\text{loss}} \in [0.0, 0.50]$ and 2-state Gilbert-Elliott burst packet loss (Good state loss $p_G = 0.01$, Bad state loss $p_B = 0.60$, transition probabilities $P_{GB}, P_{BG}$).
- **Latency & Jitter:** Unbounded packet transit latency $t_{\text{delay}} \in [0, 500\,\text{ms}]$ and Gaussian jitter $\sigma_{\text{jitter}} = 10\,\text{ms}$.
- **Reordering & Duplication:** Arbitrary message reordering and duplicate frame delivery.
- **Relay Churn:** Intermediate relays can terminate or crash at any moment with complete loss of volatile RAM transit buffers.
- **No Global Wall-Clock:** No synchronized physical clock; physical timestamps are used only for local timeouts. Causal order is defined strictly by logical version vectors.

---

## 3. Core Architecture & Mathematical Formulations

```
+-------------------------------------------------------------------------------+
|                             RoverNode / SLAM Agent                            |
|                                                                               |
|  +--------------------+      +---------------------+      +----------------+  |
|  | VersionVector /    | <--> | PartitionAware      | <--> | ConflictAware  |  |
|  | MapUpdate Ledger   |      | IBLT Reconciler     |      | Graph Merging  |  |
|  +--------------------+      +---------------------+      +----------------+  |
|            ^                            ^                         ^           |
|            |                            |                         |           |
|  +--------------------+      +---------------------+      +----------------+  |
|  | AdaptiveComms      | <--> | FailureInjection    | <--> | Global         |  |
|  | Controller         |      | Framework           |      | Consistency    |  |
|  +--------------------+      +---------------------+      | Auditor        |  |
|            ^                            ^                 +----------------+  |
|            |                            |                                     |
|  +-------------------------------------------------+                          |
|  |       MultihopRouter / RoutingTable / UDP       |                          |
|  +-------------------------------------------------+                          |
+-------------------------------------------------------------------------------+
```

### 3.1 Core Feature A: Versioned Distributed Map State
- **Update Identity:** Each mutation (new keyframe, new landmark observation, verified loop-closure edge) receives an immutable 64-bit `UpdateID`:
  $$\text{UpdateID} = (\text{rover\_id} \ll 48) \mid \text{sequence\_counter}$$
- **Version Vector Representation:**
  For $N$ rovers, a Version Vector $\mathbf{V} \in \mathbb{N}^N$ tracks the highest causal sequence witnessed from each rover:
  $$\mathbf{V} = \langle v_1, v_2, \dots, v_N \rangle$$
- **Causal Dominance Partial Ordering:**
  - $\mathbf{V}_A \le \mathbf{V}_B \iff \forall k \in [1, N]: \mathbf{V}_A[k] \le \mathbf{V}_B[k]$
  - $\mathbf{V}_A < \mathbf{V}_B \iff \mathbf{V}_A \le \mathbf{V}_B \land \mathbf{V}_A \neq \mathbf{V}_B$ (Dominance / Precedence)
  - $\mathbf{V}_A \parallel \mathbf{V}_B \iff \neg(\mathbf{V}_A \le \mathbf{V}_B) \land \neg(\mathbf{V}_B \le \mathbf{V}_A)$ (Concurrent Mutation / Conflict)
- **Idempotency & Deduplication:** Updates with sequence numbers $\le \mathbf{V}_{\text{local}}[\text{origin}]$ are marked duplicate and suppressed without processing.
- **Concurrent Conflict Resolution:**
  When $\mathbf{V}_A \parallel \mathbf{V}_B$ for the same landmark or keyframe:
  - If both observations are geometrically consistent with local map poses ($d_{\mathrm{SE2}} < \epsilon_{\text{inlier}}$), both are retained in the observation history.
  - If observations diverge, the lower rover ID acts as a deterministic tie-breaker for canonical anchor selection, and both variants are preserved under separate provenance tags.

### 3.2 Core Feature B: Partition-Aware Multi-Round IBLT Reconciliation
- **The Capacity Boundary Problem:** An IBLT of $M$ cells (e.g. $M = 32$ cells, 1024 bytes) fails to peel if $|A \Delta B| > \text{decode\_capacity}(M) \approx 0.78 M$.
- **Multi-Round Reconciliation Algorithm:**
  1. **Round 1 (Fast Fingerprint):** Exchange Version Vectors $\mathbf{V}_A, \mathbf{V}_B$. If $\mathbf{V}_A = \mathbf{V}_B$, map state is identical; terminate immediately (0 bytes payload).
  2. **Round 2 (IBLT Set Difference):** If difference is small, exchange $M$-cell IBLT difference. If peeling succeeds, apply missing updates and complete.
  3. **Round 3 (Partitioned Chunking / Multi-Round):** If peeling fails (subtraction yields unresolvable 2-core), negotiate batch window:
     - Partition candidate keys by hash prefix or keyframe range $[k_{\text{start}}, k_{\text{end}}]$.
     - Exchange sub-IBLTs per chunk or deliver missing updates via high-priority DTN batches until full set agreement is achieved.

### 3.3 Core Feature C: Conflict-Aware Graph Merging
- **Provenance Integrity:** Every edge $e = (u, v)$ carries an `EdgeProvenance` struct containing:
  - `origin_rover_id` (originator of proposal)
  - `verifier_rover_id` (verifier of proposal)
  - `relative_transform` $\mathbf{T}_{uv} \in \mathrm{SE}(2)$
  - `inlier_count` ($N_{\text{inliers}} \ge 3$)
  - `inlier_rmse` ($\text{RMSE} \le 0.50\,\text{m}$)
- **Duplicate Edge Suppression:** Edges are identified by unordered pair $(u, v)$ and type. Duplicate edges are deduplicated; if parameters conflict, the edge with lower inlier RMSE and higher inlier count is retained.
- **Dynamic DSU Invalidation & Rebuild:**
  If a loop-closure edge is subsequently invalidated (e.g. during graph optimization or conflict arbitration), standard Disjoint Set Union cannot decrement rank. `SubgraphPartitionManager::rebuild()` re-evaluates all verified edges in $O(|\mathcal{V}| + |\mathcal{E}| \alpha(|\mathcal{V}|))$, restoring mathematically sound disconnected components.

### 3.4 Core Feature D: Partition Recovery & Global Consistency Auditor
- **Rover Convergence Telemetry:**
  Each rover exports a `RoverConsistencyState`:
  ```cpp
  struct RoverConsistencyState {
      uint16_t rover_id;
      VersionVector version_vector;
      uint32_t keyframe_count;
      uint32_t landmark_count;
      uint32_t edge_count;
      uint32_t verified_loop_closures;
      uint32_t rejected_proposals;
      uint32_t dsu_components;
      uint32_t pending_sync_updates;
      bool is_reconciling;
  };
  ```
- **Rigorous Global Convergence Criteria:**
  A swarm is declared **CONVERGED** if and only if all four conditions hold:
  1. $\forall i, j: \mathbf{V}_i = \mathbf{V}_j$ (Mutual causal map equality).
  2. $\forall i, j: \text{dsu\_components}_i = \text{dsu\_components}_j = 1$ (Fully unified swarm topology).
  3. $\sum \text{pending\_sync\_updates} = 0$ (Zero outstanding reconciliation work).
  4. Verified loop closures across all agents are reciprocal and geometrically consistent.

### 3.5 Core Feature E: Adaptive Communication Control
- **Link Impairment Estimator:** Computes rolling packet loss $P_{\text{loss}}$ and RTT $T_{\text{RTT}}$.
- **Dynamic Regulation Rules:**
  - **Loss Gating:** If $P_{\text{loss}} > 0.20$, throttle heartbeat frequency from 50 ms to 150 ms to prevent bandwidth saturation while doubling DTN retry timeouts.
  - **Reconciliation Batching:** If queue occupancy $> 50$ packets, reduce IBLT sync frequency and prioritize E2E and custody ACKs.
  - **Critical vs. Non-Critical Gating:** Landmark observations and loop-closure verifications have `Priority::High`; periodic telemetry pings have `Priority::Low` and are dropped when buffer occupancy exceeds $80\%$.

### 3.6 Core Feature F: Failure-Injection Framework
- Software-only deterministic impairment injector:
  - Packet drop, burst drop (Gilbert-Elliott), latency, jitter, frame reordering.
  - Dynamic link severing (simulating mountain ridges or RF shadowing).
  - Controlled relay kill/restart to test volatile transit queue draining and recovery.

---

## 4. Benchmark Methodology & Acceptance Targets

- **Benchmark Target:** 45+ factorial trials across 5 topologies (2-Rover, 3-Rover Linear, 4-Rover Diamond, 5-Rover Ring/Mesh, and Partition-Reconnect).
- **Baselines:**
  1. *Full-Map Baseline:* Exchanging entire landmark & keyframe tables upon reconnection.
  2. *Single-Round M7.11 IBLT:* Existing baseline without retry.
  3. *Version-Aware Multi-Round M7.12 IBLT:* Adaptive multi-round reconciliation.
- **Metrics:**
  - Wire bytes during reconnection.
  - Time to convergence (ms).
  - Final DSU components (1.0 = fully unified).
  - False positive loop closures ($FP = 0$ target in benchmark).
  - Paired ATE RMSE improvement ($\Delta \text{ATE} \ge 0.0\,\text{m}$).

