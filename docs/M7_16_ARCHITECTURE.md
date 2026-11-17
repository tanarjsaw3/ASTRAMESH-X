# AstroMesh-X — Milestone 7.16 Architecture Specification
## Autonomous Exploration & Connectivity-Aware Path Planning

**Document Version:** 1.0.0  
**Status:** Approved & Active Design  
**Date:** 2026-11-14  
**Author:** Principal Autonomous Systems & SLAM Architect  

---

## 1. System Vision & Problem Formulation

In distributed multi-rover planetary exploration, rovers must autonomously explore unknown environments, discover scientific landmarks, construct topologically unified metric maps, and coordinate without central control. While Milestones 7.10–7.15 established multi-hop routing, partition recovery, gauge alignment, active loop closures, and memory compaction, rover motions were assumed to follow predefined or random trajectories.

Milestone 7.16 elevates AstroMesh-X to fully **autonomous exploration and connectivity-aware path planning**.

### The Core Challenges:
1. **The Exploration-Exploitation-Connectivity Trilemma:**
   - *Exploration:* Rovers must branch outward into uncharted territory to maximize spatial coverage and discover new landmarks.
   - *Exploitation (Map Quality):* Rovers must revisit known areas or rendezvous with teammates to establish loop closures, constrain odometry drift, and maintain low Global ATE.
   - *Connectivity:* Rovers must not wander beyond wireless communication ranges without bounded return plans, which would risk permanent isolation, transit buffer overflow, and data loss.
2. **Topological vs. Raster Occupancy Representation:**
   - AstroMesh-X is fundamentally a sparse topological landmark graph SLAM system ($G = (V, E)$, landmarks $L \subset \mathbb{R}^2$, $\mathrm{SE}(2)$ keyframes), not a dense 2D occupancy grid raster. Frontier detection must operate directly on topological boundaries and landmark hulls without fabricating fake raster grids.
3. **Decentralized Coordination Under Partitions:**
   - Frontier assignments and exploration leases must be negotiated peer-to-peer over delay-tolerant multi-hop meshes with zero central master. When partitions occur, partitioned sub-swarms must autonomously explore their local regions without deadlock.
4. **Memory-Pressure Feedback:**
   - As rovers approach memory budget limits (`Warning` / `Critical` from M7.15 `MemoryBudgetController`), exploratory path planning must de-prioritize new node creation and prioritize returning to known peers for map compaction, reconciliation, and offloading.

---

## 2. Mathematical Utility Model

### 2.1 Exploration Utility Function
For a candidate frontier $f$ evaluated by rover $r$ at pose $x_r \in \mathrm{SE}(2)$, the net utility $U(f, r)$ is formulated as:

$$U(f, r) = \frac{\mathcal{N}_{\text{gain}}(f, r)}{\mathcal{D}_{\text{cost}}(f, r)}$$

Where the numerator $\mathcal{N}_{\text{gain}}$ balances reward factors:
$$\mathcal{N}_{\text{gain}}(f, r) = w_{\text{info}} \cdot I(f) + w_{\text{cov}} \cdot C(f) + w_{\text{nov}} \cdot N(f) + w_{\text{loop}} \cdot L(f)$$

And the denominator $\mathcal{D}_{\text{cost}}$ penalizes costs, risks, and resource consumption:
$$\mathcal{D}_{\text{cost}}(f, r) = 1.0 + w_{\text{travel}} \cdot \text{Cost}_{\text{travel}}(r, f) + w_{\text{conn}} \cdot \text{Risk}_{\text{conn}}(r, f) + w_{\text{mem}} \cdot P_{\text{mem}}(r) + w_{\text{red}} \cdot \text{Redundancy}(f)$$

### 2.2 Component Definitions

1. **Information Gain Proxy ($I(f)$):**
   - In sparse topological SLAM, Fisher Information Matrix inversion is replaced by the geometric dispersion of prospective landmark clusters:
     $$I(f) = \log(1 + \det(S_{\text{geom}}(f)))$$
     Where $S_{\text{geom}}(f)$ is the 2D spatial scatter matrix of landmarks within sensor radius $R_{\text{sensor}}$ of frontier position $p_f$.
2. **Spatial Coverage ($C(f)$):**
   - Ratio of unexplored angular sector area to total sensor circular area:
     $$C(f) = \frac{\theta_{\text{unexplored}}}{2\pi} \in [0.0, 1.0]$$
3. **Geometric Novelty ($N(f)$):**
   - Proportional to the distance from $p_f$ to the nearest existing keyframe in the local pose graph:
     $$N(f) = \min\left(1.0, \frac{\| p_f - p_{\text{nearest\_kf}} \|_2}{2 R_{\text{sensor}}}\right)$$
4. **Loop-Closure Potential ($L(f)$):**
   - Proximity to known subgraphs of other rovers (derived from M7.14 `LoopClosureOpportunityManager`):
     $$L(f) = \sum_{opp \in \mathcal{O}_{\text{active}}} \exp\left(-\frac{\| p_f - p_{opp} \|^2}{2 \sigma_{\text{rendezvous}}^2}\right)$$
5. **Travel Cost ($\text{Cost}_{\text{travel}}(r, f)$):**
   - Euclidean distance in meters from current rover pose $p_r$ to frontier $p_f$:
     $$\text{Cost}_{\text{travel}}(r, f) = \frac{\| p_r - p_f \|_2}{R_{\text{max\_travel}}}$$
6. **Network Connectivity Risk ($\text{Risk}_{\text{conn}}(r, f)$):**
   - Evaluates the projected distance from $p_f$ to the nearest active communication neighbor $p_{\text{peer}}$:
     $$d_{\text{comm}} = \min_{p \in \mathcal{P}_{\text{active}}} \| p_f - p_{\text{peer}} \|_2$$
     $$\text{Risk}_{\text{conn}}(r, f) = \begin{cases} 
     \frac{d_{\text{comm}}}{R_{\text{comm}}} & \text{if } d_{\text{comm}} \le R_{\text{comm}} \\
     1.0 + \kappa_{\text{sever}} \cdot \left(\frac{d_{\text{comm}} - R_{\text{comm}}}{R_{\text{comm}}}\right) & \text{if } d_{\text{comm}} > R_{\text{comm}}
     \end{cases}$$
7. **Memory Pressure Penalty ($P_{\text{mem}}(r)$):**
   - Sourced directly from M7.15 `MemoryBudgetController`:
     $$P_{\text{mem}}(r) = \begin{cases} 
     0.0 & \text{if } \text{Pressure} = \text{Normal} \\
     1.5 & \text{if } \text{Pressure} = \text{Warning} \\
     5.0 & \text{if } \text{Pressure} = \text{Critical}
     \end{cases}$$
8. **Redundancy Penalty ($\text{Redundancy}(f)$):**
   - Proximity to active exploration claims of peer rovers:
     $$\text{Redundancy}(f) = \sum_{c \in \mathcal{C}_{\text{peer}}} \max\left(0.0, 1.0 - \frac{\| p_f - p_c \|_2}{R_{\text{separation}}}\right)$$

---

## 3. Topological Frontier Detection Architecture

Frontiers are generated strictly from the topological boundary of observed space without raster grids:

```
[ Local PoseGraph + Landmark Observations ]
                    │
                    ▼
       [ Peripheral Keyframe Extraction ]
        (Nodes on boundary / degree < 2)
                    │
                    ▼
      [ Radial Frontier Projection ]
    (Project vectors along unexplored sectors)
                    │
                    ▼
     [ Duplicate Clustering & Suppression ]
     (Merge frontiers within ε_frontier = 3.0m)
                    │
                    ▼
     [ Reachability & Bounds Validation ]
           (Within [-50, 50] x [-50, 50])
                    │
                    ▼
     [ Active Topological Frontier Set ]
```

### Frontier Lifecycle State Machine
```
   [ Detection ]
         │
         ▼
    (Candidate) ──[ Verified reachable ]──► (Active)
         ▲                                     │
         │ [ Lease expired / yield ]          │ [ Claimed by Rover ]
         │                                     ▼
         └───────────────────────────────── (Claimed)
                                               │
                                 ┌─────────────┴─────────────┐
                                 ▼                           ▼
                           (Completed)                   (Expired)
                      [ Arrived / Observed ]         [ Out of reach / merged ]
```

---

## 4. Decentralized Frontier Coordination Protocol

### 4.1 Frontier Claim Message (Wire Format: 40 Bytes)
```
+--------------------+--------------------+--------------------+--------------------+
| FrontierID (8B)    | RoverID (2B)       | Reserved (2B)      | TargetX (8B)       |
+--------------------+--------------------+--------------------+--------------------+
| TargetY (8B)       | UtilityScore (8B)  | LeaseTTL (4B)      | Epoch (4B)         |
+--------------------+--------------------+--------------------+--------------------+
```

### 4.2 Deterministic Conflict Dominance
When Rover $A$ and Rover $B$ claim overlapping frontiers ($\| p_A - p_B \| < R_{\text{separation}}$):
1. **Utility Dominance:** Higher $U(f)$ strictly wins.
2. **Timestamp/Epoch Dominance:** Earlier reservation epoch wins if $|U_A - U_B| < 10^{-4}$.
3. **Identifier Tie-Breaking:** Lower numerical `rover_id` strictly wins if utility and epoch are equal.

The losing rover immediately yields, releases its claim, and deterministically re-evaluates its next best candidate.

---

## 5. Connectivity-Aware Path Planning & Safe Fallbacks

### 5.1 Anti-Oscillation Hysteresis
To prevent rapid target switching caused by minor utility fluctuations:
$$U(f_{\text{new}}) > U(f_{\text{current}}) + \Delta_{\text{switch}} \quad (\Delta_{\text{switch}} = 0.15)$$

### 5.2 Return-to-Connectivity Fallback
When all available candidate frontiers require severing communication ($d_{\text{comm}} > R_{\text{comm}}$) and:
- Rover memory pressure is `Warning` or `Critical`, OR
- Rover has unsynchronized map updates exceeding DTN backlog threshold ($> 10$ updates), OR
- Link to the elected gauge anchor has been degraded for $> T_{\text{max\_disconnected}}$ steps:

The planner switches to `ReturnToSwarm` mode:
- Sets destination toward the nearest active relay rover or the elected gauge anchor.
- Once within $R_{\text{comm}}$ and synchronized, exploration resumes.

