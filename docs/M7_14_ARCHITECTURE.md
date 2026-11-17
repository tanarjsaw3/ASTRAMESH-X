# Milestone 7.14 Architecture: Active Multi-Robot Loop Closure & Communication-Efficient Rendezvous

---

## 1. Problem Statement & Motivation

In multi-rover collaborative topological and metric SLAM (AstroMesh-X), rovers operating in GPS-denied environments (e.g. planetary craters, subterranean lava tubes) accumulate odometric drift and experience intermittent network connectivity. 

While Milestones 7.11–7.13 successfully established:
- Multi-hop store-carry-forward DTN routing (`MultihopRouter`),
- Causal vector-clock ledger reconciliation (`PartitionAwareReconciler`), and
- Decentralized $\mathrm{SE}(2)$ gauge alignment (`GaugeCoordinator`, `AlignmentGraph`),

they operated with **passive loop closure**: rovers either blindly broadcast all keyframes and observations, or opportunistically evaluated candidate closures without prioritizing communication efficiency or information value.

Under constrained planetary RF channels (e.g. 1 KB envelope, limited battery power, high packet loss, multi-hop relay contention), naive all-to-all observation exchange leads to:
1. **Severe Communication Contention:** Flooding the network with candidate keyframe descriptor payloads starves critical state synchronization and DTN custody traffic.
2. **Marginal Information Returns:** Many candidate closures connect keyframes that are spatially redundant with existing closures, contributing negligible reduction in trajectory covariance while consuming scarce bandwidth.
3. **Perceptual Aliasing Vulnerability:** Blindly evaluating hundreds of candidate matches increases the statistical risk of false positive closures ($FP > 0$).

**Milestone 7.14 Objective:** Implement a decentralized, active loop-closure scheduling and rendezvous protocol that actively selects, prioritizes, and schedules the most informative, geometrically diverse, and communication-efficient cross-rover loop closures while strictly enforcing communication byte budgets and zero ROS2/hardware dependencies.

---

## 2. System Architecture & Mathematical Foundations

```
+-----------------------------------------------------------------------------------+
|                           ROVER LOCAL SLAM SUBSYSTEM                              |
|                                                                                   |
|  [ PoseGraph / Odometry ] <---+                                                   |
|             |                 |                                                   |
|             v                 | Verified Loop Closure                             |
|  [ Opportunity Manager ]      |                                                   |
|             |                 |                                                   |
|             v                 |                                                   |
|  [ Active Scheduler ] --------+                                                   |
|             |                 ^                                                   |
|             v                 | (Kabsch/Umeyama + RANSAC)                         |
|  [ Rendezvous Protocol ] ---> [ Geometric Verifier ] <--- [ Observation Cache ]   |
+-------------|---------------------------------------------------------------------+
              |
              | Bounded Binary Messages (Wire Envelopes)
              v
+-----------------------------------------------------------------------------------+
|                       COMMUNICATION & NETWORKING LAYER                             |
|                                                                                   |
|  [ Adaptive Comms Controller ] <---> [ Multihop Router ] <---> [ DTN Buffer ]     |
|  (EWMA Loss, RTT, Queue-aware)       (Split Horizon/DV)        (Store & Forward)  |
+-----------------------------------------------------------------------------------+
```

### 2.1 Mathematical Formulation of Information-Gain Proxy
In ideal centralized SLAM, the mutual information gain $\mathcal{I}(X; z_{ij})$ of a prospective relative pose measurement $z_{ij}$ between keyframes $i$ and $j$ is:
$$\mathcal{I}(X; z_{ij}) = \frac{1}{2} \log \det (I + \Lambda_z J \Sigma_{ij} J^T)$$
where $\Sigma_{ij} = \operatorname{Cov}(T_i^{-1} T_j)$ is the joint pose covariance between nodes $i$ and $j$, $\Lambda_z$ is the measurement information matrix, and $J$ is the measurement Jacobian.

In a fully decentralized, delay-tolerant multi-rover collective without dense covariance matrix sharing ($O(N^2)$ communication cost), full covariance propagation across network links is strictly prohibited. Therefore, we derive a mathematically grounded, verifiable **Decentralized Information-Gain Proxy** $I_{\text{proxy}}(O)$:

$$I_{\text{proxy}}(O) = \alpha_{\text{drift}} \cdot \sqrt{d_{\text{graph}}(i, j)} + \alpha_{\text{disp}} \cdot \log(1.0 + \det(S_{\text{geom}})) + \alpha_{\text{topo}} \cdot \frac{1}{1.0 + E_{\text{existing}}(r_A, r_B)}$$

where:
1. **Accumulated Drift Term $\sqrt{d_{\text{graph}}(i, j)}$:**
   $d_{\text{graph}}(i, j)$ represents the kinematic path distance (or keyframe hop count) separating nodes $i$ and $j$ in the topological graph before loop closure. Closing a loop across a long trajectory loop eliminates drift that scales as $\sigma_{\text{drift}} \propto \sqrt{d}$.
2. **Geometric Dispersion $\det(S_{\text{geom}})$:**
   Let $\{p_k\}_{k=1}^M$ be the mutual landmark points in local frame. Centered scatter matrix:
   $$S_{\text{geom}} = \frac{1}{M} \sum_{k=1}^M (p_k - \bar{p})(p_k - \bar{p})^T \in \mathbb{R}^{2 \times 2}$$
   $\det(S_{\text{geom}})$ measures the 2D spatial area spanned by the landmarks. High determinant guarantees well-conditioned rotational and translational constraint estimation and prevents degenerate collinear alignments.
3. **Topological Bridge Value $E_{\text{existing}}(r_A, r_B)$:**
   Number of verified edges already established between rover $r_A$ and rover $r_B$. If $E=0$ (disconnected sub-swarms), the closure is an essential topological bridge ($\Delta C_{\text{DSU}} = -1$) and receives maximal reward. As $E$ increases, marginal return diminishes harmonically.

### 2.2 Active Opportunity Priority Score
The `ActiveLoopClosureScheduler` evaluates candidate opportunities using a multi-attribute utility function:

$$\text{Priority}(O) = \frac{w_{\text{info}} \cdot I_{\text{proxy}}(O) + w_{\text{qual}} \cdot Q_{\text{corr}}(O) + w_{\text{prob}} \cdot P_{\text{verif}}(O) + w_{\text{link}} \cdot L_{\text{net}}(r_A, r_B)}{1.0 + w_{\text{cost}} \cdot C_{\text{wire}}(O) + w_{\text{dup}} \cdot R_{\text{spatial}}(O)}$$

- $Q_{\text{corr}}(O)$: Ratio of mutual landmarks over minimum required: $\min(1.0, N_{\text{overlap}} / 10.0)$.
- $P_{\text{verif}}(O)$: Historical verification rate of candidate rover pair $r_A, r_B$.
- $L_{\text{net}}(r_A, r_B)$: Network link quality score $1.0 - \text{loss}_{\text{EWMA}} - \frac{\text{RTT}}{500\,\text{ms}}$.
- $C_{\text{wire}}(O)$: Normalized communication cost $N_{\text{landmarks}} \times \text{sizeof}(\text{ObservationRecord}) / \text{Budget}$.
- $R_{\text{spatial}}(O)$: Spatial redundancy penalty if another opportunity was recently verified within radius $\delta_{\text{suppress}} = 3.0\,\text{m}$.

### 2.3 Decentralized Rendezvous Protocol
The exchange protocol operates over existing `MessageFrame` and `MultihopEnvelope` substrates:
1. **Rendezvous Inquiry (`RendezvousInquiry`):**
   Rover $A$ sends a compact inquiry (32 bytes) containing keyframe ID, approximate bounding box of observed landmarks, and request epoch.
2. **Rendezvous Grant / Response (`RendezvousResponse`):**
   Rover $B$ replies with landmark IDs and local 2D coordinates only for mutual landmarks, bounded to `MAX_EXCHANGE_LANDMARKS = 16` (preserving the 1 KB packet limit).
3. **Local Geometric Verification:**
   Rover $A$ runs Kabsch/Umeyama + RANSAC locally. No ground truth is consulted.
4. **Verified Proposal Broadcast (`LoopClosureProposal`):**
   Upon verification, Rover $A$ broadcasts the verified edge update via `MapUpdate` to update the DSU, AlignmentGraph, and GaugeCoordinator.

---

## 3. Communication Byte Invariants

1. **Envelope Bound Invariant:**
   Every rendezvous exchange message fits strictly within `MAX_PAYLOAD_SIZE = 1024` bytes.
2. **Conservation Invariant:**
   Total wire bytes transmitted across the network must equal the sum of inquiry bytes, response bytes, proposal bytes, and hop ACK bytes:
   $$\text{Bytes}_{\text{total}} = \text{Bytes}_{\text{inquiry}} + \text{Bytes}_{\text{response}} + \text{Bytes}_{\text{proposal}} + \text{Bytes}_{\text{ack}}$$
3. **Idempotence & Duplicate Suppression:**
   Repeated delivery of the same inquiry or proposal within a 5-second window is suppressed without re-execution or graph corruption.

