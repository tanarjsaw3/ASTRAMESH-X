# AstroMesh-X Milestone 7.15 — Architecture & Specification
## Dynamic Map Compaction & Memory-Bounded Distributed SLAM

**Status:** Under Implementation & Verification  
**Author:** Principal Network Systems & Distributed SLAM Architect  
**Milestone:** 7.15  
**Scope:** 100% Software-Defined, Memory-Bounded Multi-Robot Topological SLAM  

---

## 1. Problem Statement & Motivation

In long-duration multi-robot planetary exploration missions, decentralized SLAM subgraphs and landmark registries grow monotonically with time and distance traversed ($O(T)$ nodes, $O(T)$ landmarks, $O(K \cdot T)$ cross-rover state). On resource-constrained flight avionics with strictly bounded RAM (e.g. radiation-hardened RAD750 or low-power embedded ARM cores with 256 MB to 1 GB RAM), unconstrained graph growth causes:
1. Out-of-memory (OOM) fatal process termination.
2. Latency spikes in graph reconciliation and loop-closure search.
3. Network queue bloat and DTN transit buffer exhaustion.

Milestone 7.15 introduces **Dynamic Map Compaction and Bounded Memory Management** for AstroMesh-X, guaranteeing strict per-rover and swarm-wide memory caps while preserving metric accuracy, topological connectivity, verified loop closures, and gauge alignment.

---

## 2. Core Architectural Pillars

```
+----------------------------------------------------------------------------+
|                          MemoryBudgetController                            |
|  - Tracks Global & Per-Rover Budgets (Normal / Warning / Critical)         |
|  - Manages Hysteresis & Prevents Prune/Restore Oscillations                |
|  - Detects & Honest Infeasibility Reporting                               |
+-------------------------------------+--------------------------------------+
                                      | Triggers Compaction passes
                                      v
+-------------------------------------+--------------------------------------+
|                           MapCompactor                                     |
|  - Conservative Keyframe Compounding (T_ik = T_ij * T_jk)                  |
|  - Covariance / Information Composition (\Sigma_ik = \Sigma_ij + R\Sigma R)|
|  - Indispensable Node Protection (Frontier, Anchors, Loop Endpoints)       |
+-------------------------------------+--------------------------------------+
                                      | Prunes observations
                                      v
+-------------------------------------+--------------------------------------+
|                    LandmarkLifecycleManager                                |
|  - Bounded Landmark DB & Observation History                               |
|  - Loop Closure Support Protection                                         |
|  - Spatial Dispersion Preservation (\det S_geom > 0)                       |
+-------------------------------------+--------------------------------------+
                                      | Disseminates compact epochs
                                      v
+-------------------------------------+--------------------------------------+
|                 DistributedCompactionProtocol                              |
|  - Monotonic Compaction Epochs & Generation IDs                            |
|  - Compact Wire Messages (CompactionNotification <= 128 B)                 |
|  - VersionVector-Aware Partition-Safe Deferral & Idempotent Application    |
+----------------------------------------------------------------------------+
```

---

## 3. Mathematical Formulations

### 3.1 Memory Accounting Distinctions
The memory framework strictly separates four categories of memory measurement:
1. **Logical Payload Size ($M_{\text{logical}}$):** Theoretical sizeof basic data fields ($\sum \text{sizeof}(\text{struct})$).
2. **Container Allocated Capacity ($M_{\text{capacity}}$):** Actual heap allocation including container reserves (`std::vector::capacity() * sizeof(T)`, hash map bucket arrays, node pointers).
3. **Serialized Wire Size ($M_{\text{wire}}$):** Packed binary representation transmitted across network sockets.
4. **Physical Process RSS ($M_{\text{RSS}}$):** Resident Set Size queried from the OS kernel (`WorkingSetSize` via `GetProcessMemoryInfo` on Windows).

Under no circumstances is logical payload size reported as measured RSS.

### 3.2 Conservative Keyframe Compounding vs. Schur Complement Marginalization
True factor-graph marginalization of a keyframe node $x_j$ requires computing the Schur complement on the dense Markov blanket:
$$\Lambda_{\text{marginalized}} = \Lambda_{aa} - \Lambda_{aj} \Lambda_{jj}^{-1} \Lambda_{ja}$$
Marginalizing $x_j$ connects all adjacent landmarks in a dense clique (fill-in), causing the landmark-pose graph density to explode and increasing overall memory consumption.

**AstroMesh-X Conservative Compaction:**
A sequence of non-branching odometry keyframes $x_i \to x_j \to x_k$ is compacted into a single compounded edge $x_i \to x_k$ if and only if:
1. $x_j$ is not an exploration frontier node: $j \ne \max_{n} \{ n \in \mathcal{K}_r \}$.
2. $x_j$ is not an articulation point or loop closure endpoint: $\operatorname{deg}(x_j) = 2$ (only odometry in and odometry out).
3. $x_j$ is not a gauge coordinate anchor: $j \ne R_{\text{anchor}}$.
4. All landmarks observed at $x_j$ are either redundant or reassociated to $x_i$ / $x_k$.

The compounded relative pose and covariance are:
$$T_{ik} = T_{ij} \cdot T_{jk} = \begin{bmatrix} R_{ij} R_{jk} & R_{ij} p_{jk} + p_{ij} \\ 0 & 1 \end{bmatrix}$$
$$\Sigma_{ik} = \Sigma_{ij} + R_{ij} \Sigma_{jk} R_{ij}^T$$
This preserves exact kinematic trajectory constraints with zero fill-in.

### 3.3 Hysteresis & Pressure Level Dynamics
Let $B$ be the configured memory budget and $M(t)$ be current memory usage.
- **Normal:** $M(t) < 0.75 \cdot B$. Normal operations, standard keyframe creation.
- **Warning:** $0.75 \cdot B \le M(t) < 0.90 \cdot B$. Throttles new keyframe creation rate, triggers background landmark pruning.
- **Critical:** $M(t) \ge 0.90 \cdot B$. Halts non-essential keyframe creation, executes aggressive topological compaction.
- **Hysteresis Recovery Threshold:** Once in `Warning` or `Critical`, the controller does not return to `Normal` until memory drops below $0.65 \cdot B$, preventing high-frequency prune-restore oscillation.

---

## 4. Distributed Compaction Protocol Wire Format

### `CompactionNotification` (Opcode `0xE1`)
```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|          Rover ID             |       Compaction Epoch        |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                     From Keyframe Node ID                     |
|       (Rover ID: 2B)          |       (Local Index: 6B)       |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                      To Keyframe Node ID                      |
|       (Rover ID: 2B)          |       (Local Index: 6B)       |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                       Compounded TX (f64)                     |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                       Compounded TY (f64)                     |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                     Compounded Theta (f64)                    |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                      Information Scale (f32)  |   Flags (u8)  |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                       CRC-32 Checksum (u32)                   |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```
Total Wire Size: **64 bytes** $\le 128$ bytes invariant.

