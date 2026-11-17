# Continuous Riemannian Dynamics (CORD) — Literature Review & Primary Source Analysis

*Document tracking Phase 3, Milestone 3.1 of AstroMesh-X.*  
*Review Date: October 2026.*  
*Primary Source: Jaeho Shin, Maani Ghaffari, and Yulun Tian, "Distributed Pose Graph Optimization via Continuous Riemannian Dynamics", Robotics: Science and Systems XXII (RSS 2026), Paper 32; arXiv:2605.11210v1 [cs.RO].*

---

## 1. Bibliographic Reference & Verified Source Details

- **Title:** Distributed Pose Graph Optimization via Continuous Riemannian Dynamics (CORD)
- **Authors:** Jaeho Shin$^1$, Maani Ghaffari$^1$, and Yulun Tian$^2$  
  ($^1$University of Michigan, Ann Arbor, MI, USA; $^2$Institute for Dynamic Systems and Control, ETH Zürich, Switzerland)
- **Publication Venue:** *Robotics: Science and Systems XXII (RSS 2026)*, Delft, Netherlands, July 13–17, 2026.
- **Official Proceedings Entry:** Paper 32, Volume 22, ISBN 979-8-9902848-2-1.  
  URL: [https://www.roboticsproceedings.org/rss22/p032.html](https://www.roboticsproceedings.org/rss22/p032.html)  
  PDF: [http://www.roboticsproceedings.org/rss22/p032.pdf](http://www.roboticsproceedings.org/rss22/p032.pdf)
- **Preprint Version Reviewed:** arXiv:2605.11210v1 [cs.RO], submitted May 2026.  
  URL: [https://arxiv.org/abs/2605.11210](https://arxiv.org/abs/2605.11210)  
  HTML: [https://arxiv.org/html/2605.11210](https://arxiv.org/html/2605.11210)

> [!NOTE]
> This review is conducted directly against the full-text paper and technical appendix of arXiv:2605.11210v1 / RSS 2026 p032. All equations, theorem statements, and variable definitions cited below reference the paper's original numbering.

---

## 2. Context & Related Work: The Distributed PGO Landscape

Pose Graph Optimization (PGO) is the foundational nonlinear back-end in collaborative Simultaneous Localization and Mapping (CSLAM). In multi-robot swarms operating in communication-constrained and GPS-denied environments, centralized optimization represents a single point of failure and incurs unsustainable communication overhead.

### 2.1 Prior Distributed Baselines and Their Limitations

Prior distributed PGO algorithms can be classified into several paradigms, each with fundamental trade-offs identified by Shin, Ghaffari, and Tian:

1. **Gauss-Seidel Block Coordinate Descent (RBCD, RBCD++, IRBCD):**
   - *Mechanism:* Robots update sequentially (or in alternating graph-color cliques) using block coordinate descent (Tian et al., T-RO 2021 [6]; Li et al., RA-L 2024 [19]).
   - *Limitation:* Sequential update dependencies force idle time across robots and scale poorly as swarm size grows.
2. **First-Order Asynchronous Parallel Solvers (ASAPP, Distributed Jacobi):**
   - *Mechanism:* Robots perform parallel preconditioned gradient steps (Tian et al., RA-L 2020 [10]).
   - *Limitation:* Highly sensitive to communication latency. Outdated neighbor poses cause severe gradient oscillations and slow down convergence, requiring excessively small step sizes.
3. **Primal-Dual / Consensus ADMM (MESA, iMESA):**
   - *Mechanism:* Introduces auxiliary edge variables and Lagrange multipliers over Lie groups (McGann et al., ICRA 2024 [7], RSS 2024 [20]).
   - *Limitation:* Sensitive to hyperparameter tuning (penalty parameters $\rho$), often converges prematurely on loose graphs, and historically relies on edge-based communication pairs rather than all-to-all parallel steps.
4. **Majorization-Minimization (AMM-PGO):**
   - *Mechanism:* Constructs surrogate quadratic bounds and employs Nesterov acceleration with adaptive restart (Fan & Murphey, T-RO 2024 [8]).
   - *Limitation:* Tightly coupled to chordal distance relaxations in $SE(3)$; cannot easily incorporate asynchronous communication or delayed neighbor states without destabilizing Nesterov momentum.
5. **Probabilistic Message Passing (GBP, CBS):**
   - *Mechanism:* Gaussian Belief Propagation (Murai et al., T-RO 2024 [9]; Liu & Chli, RA-L 2026 [22]).
   - *Limitation:* Can exhibit convergence instability on loopy graphs without heuristic Hellinger-distance damping.

### 2.2 The Continuous Dynamical Systems Insight

To overcome the fragility of discrete first-order iterations under delayed communication, CORD turns to continuous-time geometric mechanics (Bloch et al. 1996 [11]; Hairer et al. 2006 [29]; Su et al. 2016 [23]; Duruisseaux & Leok 2022 [31]). Rather than discretizing first-order gradient descent, CORD formulates PGO as a **second-order physical system of massive particles moving on a matrix Lie group manifold under potential and dissipative forces**.

---

## 3. Mathematical Problem Formulation in the Primary Paper

### 3.1 PGO Objective (Problem 1)

Let $G = (\mathcal{V}, \mathcal{E})$ denote a directed pose graph:
- Each node $u \in \mathcal{V}$ represents an unknown robot pose $X_u \in SE(3)$ (in AstroMesh-X: $SE(2)$).
- Each directed edge $(u, v) \in \mathcal{E}$ encodes a noisy measured relative pose $X_{uv} \in SE(3)$ from frame $u$ to frame $v$.
- $\Omega_{uv} \succ 0$ is the $6 \times 6$ (in $SE(2)$: $3 \times 3$) symmetric positive-definite information matrix.
- Total number of poses: $N \coloneq |\mathcal{V}|$. The stacked state is $X \in SE(3)^N$.

The paper defines the global cost function $\mathcal{C}(X)$:

$$\mathcal{C}(X) \coloneq \frac{1}{2} \sum_{(u, v) \in \mathcal{E}} \|r_{uv}(X_u, X_v)\|_{\Omega_{uv}}^2 \tag{Problem 1}$$

The paper analyzes two residual formulations:
1. **Geodesic Residual (Manifold On-Group Logarithm):**
   $$r_{uv}(X_u, X_v) \coloneq \operatorname{Log}(X_{uv}^{-1} X_u^{-1} X_v)^\vee \in \mathbb{R}^6$$
   $$\|r_{uv}\|_{\Omega_{uv}}^2 \coloneq r_{uv}^\top \Omega_{uv} r_{uv}$$
2. **Chordal Residual (Embedding Matrix Difference):**
   $$r_{uv}(X_u, X_v) \coloneq X_v - X_u X_{uv} \in \mathbb{R}^{4 \times 4}$$
   $$\|r_{uv}\|_{\Omega_{uv}}^2 \coloneq \operatorname{tr}\left(r_{uv} \Omega_{uv} r_{uv}^\top\right)$$
   where $\Omega_{uv} = \operatorname{blkdiag}(\omega_{R_{uv}} I_3, \omega_{t_{uv}})$.

---

## 4. Continuous-Time Riemannian Dynamics (CORD)

### 4.1 Kinematics on Matrix Lie Groups

For a trajectory $X(t) \in \mathcal{G}$ evolving on a Lie group $\mathcal{G} = SE(3)^N$, the **body velocity** (left-trivialized tangent vector) $\xi_i \in \mathfrak{g}$ for node $i$ is defined by:

$$\xi_i \coloneq (X_i^{-1} \dot{X}_i)^\vee \in \mathbb{R}^6 \iff \dot{X}_i = X_i \, \xi_i^\wedge$$

where $(\cdot)^\wedge : \mathbb{R}^6 \to \mathfrak{se}(3)$ and $(\cdot)^\vee : \mathfrak{se}(3) \to \mathbb{R}^6$ are the standard Lie algebra isomorphism operators.

The full stacked velocity vector is:

$$\xi \coloneq [\xi_1^\top, \xi_2^\top, \dots, \xi_N^\top]^\top \in \mathbb{R}^{6N}$$

### 4.2 Lagrangian Mechanics & Energy Definitions

1. **Generalized Mass / Inertia Matrix:** Let $M \in \mathbb{R}^{6N \times 6N}$ be a symmetric positive-definite generalized mass matrix ($M \succ 0$). $M$ may be constant or state-dependent $M(X)$.
2. **Kinetic Energy:**
   $$\mathcal{T}(\xi) \coloneq \frac{1}{2} \xi^\top M \xi$$
3. **Potential Energy:** Defined as the PGO objective function:
   $$\mathcal{V}(X) \coloneq \mathcal{C}(X)$$
4. **Reduced Lagrangian:** Defined on $\mathcal{G} \times \mathfrak{g}$ via Lagrangian reduction:
   $$\ell(X, \xi) \coloneq \mathcal{T}(\xi) - \mathcal{C}(X) = \frac{1}{2} \xi^\top M \xi - \mathcal{C}(X)$$

### 4.3 Damped Euler–Poincaré Equations (Proposition 1)

In the absence of dissipation, Hamilton's principle of least action $\delta \int \ell \, dt = 0$ yields conservative Euler–Poincaré equations, resulting in perpetual oscillations without convergence.

To induce asymptotic convergence to stationary points of $\mathcal{C}(X)$, the paper introduces a **positive-definite damping matrix** $D \succ 0$ acting as a Rayleigh dissipative generalized force:

$$f_{damp} = -D \xi \in \mathfrak{g}^*$$

Applying the **Lagrange–d'Alembert principle** (accounting for virtual work done by non-conservative forces $\delta \int \ell \, dt + \int \langle f_{damp}, \eta \rangle dt = 0$, where $\eta = X^{-1} \delta X$):

$$\frac{d}{dt}(M \xi) - \operatorname{ad}_\xi^*(M \xi) + D \xi = -\nabla \mathcal{C}(X) \tag{Paper Eq. 3}$$

where:
- $\nabla \mathcal{C}(X)$ is the **Riemannian gradient** (body/left-trivialized gradient):
  $$(\nabla \mathcal{C}(X))_i = \left( X_i^\top \frac{\partial \mathcal{C}}{\partial X_i} \right)^\vee$$
- $\operatorname{ad}_\xi^*$ is the **co-adjoint operator** on $\mathfrak{g}^*$, defined via the dual pairing $\langle \operatorname{ad}_\xi^* \mu, \eta \rangle \coloneq \langle \mu, \operatorname{ad}_\xi \eta \rangle = \langle \mu, [\xi, \eta] \rangle$, where $[\cdot, \cdot]$ is the Lie bracket on $\mathfrak{g}$.
- The term $\operatorname{ad}_\xi^*(M \xi)$ represents **curvature-induced Coriolis and gyroscopic coupling** between body velocity and momentum that arises uniquely on non-Euclidean Lie groups (and vanishes in flat Euclidean vector spaces).

### 4.4 State-Dependent Mass Dynamics (Paper Eq. 4)

When the mass matrix depends on state $M \equiv M(X)$, expanding the total derivative $\frac{d}{dt}(M \xi) = M \dot{\xi} + \dot{M} \xi$ yields the explicit acceleration equation:

$$M \dot{\xi} = -\nabla \mathcal{C}(X) - D \xi + \operatorname{ad}_\xi^*(M \xi) - \dot{M} \xi \tag{Paper Eq. 4}$$

When $M$ is constant, $\dot{M} \equiv 0$ and the last term vanishes.

### 4.5 Equilibrium Point Guarantee (Remark 1)

At steady state, all velocities and accelerations vanish: $\xi = 0$ and $\dot{\xi} = 0$. Substituting into Eq. (3):

$$M(0) - \operatorname{ad}_0^*(0) + D(0) = -\nabla \mathcal{C}(X) \implies \nabla \mathcal{C}(X) = 0$$

> **Fundamental Invariant:** The equilibrium configurations of the damped Euler–Poincaré dynamical system coincide exactly with the **first-order critical points (FOCPs)** of the original PGO problem.

### 4.6 Overdamped Limit: Generalization of Classical Algorithms

Consider scaling the mass matrix by $M = \varepsilon \widetilde{M}$ and taking the overdamped limit $\varepsilon \to 0$ (inertial forces vanish relative to damping):

$$D \xi = -\nabla \mathcal{C}(X) \implies \xi = -D^{-1} \nabla \mathcal{C}(X)$$

Under forward integration with step size $\Delta t$:
- If $D = I$: $\xi = -\nabla \mathcal{C}(X) \implies X_{k+1} = X_k \exp(-\Delta t \, \nabla \mathcal{C}(X)^\wedge)$, recovering **Riemannian Gradient Descent** (Absil et al. 2008 [39]).
- If $D = J^\top \Omega J$ (Gauss–Newton Hessian approximation): $\xi = -(J^\top \Omega J)^{-1} \nabla \mathcal{C}(X)$, recovering the **Gauss–Newton direction** on Lie groups (Dellaert et al. [38]).

Thus, CORD provides a second-order continuum generalization containing Riemannian gradient descent and Gauss–Newton as degenerate overdamped limits.

---

## 5. Geometric Discretization Scheme (Algorithm 1)

To integrate the second-order Lie-group ODE without numerical drift off the manifold, CORD uses a **geometric semi-implicit Euler integrator**:

```text
Algorithm 1: Semi-Implicit Geometric Integrator for CORD (Paper Sec. IV-B)
Input: Initial poses X_0, initial body velocities \xi_0 (default: 0), step size \Delta t
For k = 0, 1, 2, ... until converged:
  1. Evaluate potential force:          F_grad = -\nabla \mathcal{C}(X_k)
  2. Evaluate damping force:            F_damp = -D_k \xi_k
  3. Evaluate co-adjoint geometric force: F_geom = \operatorname{ad}_{\xi_k}^*(M_k \xi_k)
  4. Evaluate variable mass force:      F_varM = -(\dot{M} \xi)_k   (0 if M is constant)
  5. Compute generalized acceleration:  a_k = M_k^{-1} (F_grad + F_damp + F_geom + F_varM)
  6. Forward velocity update:           \xi_{k+1} = \xi_k + \Delta t \cdot a_k
  7. Semi-implicit pose update:         X_{k+1} = X_k \exp((\xi_{k+1} \Delta t)^\wedge)
  8. If state-dependent mass:           Refresh M_{k+1} = M(X_{k+1}), D_{k+1} = D(X_{k+1})
```

> [!IMPORTANT]
> **Semi-Implicit Ordering:** Line 7 uses the *updated* velocity $\xi_{k+1}$ to evaluate the Lie exponential map, rather than $\xi_k$. This symplectic-like construction preserves discrete energy dissipation and guarantees numerical stability without requiring expensive implicit nonlinear solves.

---

## 6. Distributed Decomposition & Velocity Prediction (Algorithm 2)

### 6.1 Block-Diagonal Partitioning Across Robots

Let $R$ denote the number of robots in the swarm. The pose set $\mathcal{V}$ is partitioned into disjoint subsets $\mathcal{V}_1, \dots, \mathcal{V}_R$, where robot $i$ owns $n_i \coloneq |\mathcal{V}_i|$ poses.

CORD selects block-diagonal mass and damping matrices:

$$M \coloneq \operatorname{blkdiag}(M^1, M^2, \dots, M^R), \quad D \coloneq \operatorname{blkdiag}(D^1, D^2, \dots, D^R)$$

where $M^i, D^i \in \mathbb{R}^{6n_i \times 6n_i}$ are strictly local to robot $i$.

Under this block-diagonal structure, the inertial, damping, and co-adjoint operators decouple across robots, yielding local dynamics for robot $i$:

$$M^i \dot{\xi}^i = -\nabla \mathcal{C}^i(X^i, \bar{X}^{\text{neighbors}}) - D^i \xi^i + \operatorname{ad}_{\xi^i}^*(M^i \xi^i) - \dot{M}^i \xi^i \tag{Paper Eq. 5, 7}$$

### 6.2 Local Cost Decomposition

The cost relevant to robot $i$'s variables decomposes into:

$$\mathcal{C}_i(X^i) \coloneq \frac{1}{2} \sum_{(u, v) \in \mathcal{E}_i^{\text{intra}}} \|r_{uv}(X_u, X_v)\|_{\Omega_{uv}}^2 + \sum_{j \in \mathcal{N}_i} \sum_{(u, v) \in \mathcal{E}_{ij}^{\text{inter}}} \|r_{uv}(X_u, \bar{X}_v)\|_{\Omega_{uv}}^2 \tag{Paper Eq. 6}$$

where:
- $\mathcal{E}_i^{\text{intra}}$: Edges connecting two poses owned by robot $i$ (odometry and local loop closures).
- $\mathcal{E}_{ij}^{\text{inter}}$: Verified inter-robot loop closures connecting robot $i$ with neighbor robot $j$.
- $\bar{X}_v$: The pose of neighbor robot $j$'s keyframe, treated as fixed during robot $i$'s step.

### 6.3 Choice of Local Preconditioners ($M^i$ and $D^i$)

Let $H^i \in \mathbb{R}^{6n_i \times 6n_i}$ denote the diagonal block of the Levenberg–Marquardt approximate Hessian corresponding to robot $i$:

$$H^i \coloneq (J_i^\top \Omega_i J_i) + \lambda I$$

CORD defines:

$$M^i \coloneq m \cdot H^i, \quad D^i \coloneq \left(\frac{d}{t} + \epsilon_d\right) H^i \tag{Paper Sec. IV-C}$$

where:
- $m > 0$ is a scalar mass coefficient.
- $d > 0$ is a damping coefficient (the decaying factor $d/t$ mimics Nesterov-like accelerated gradient flows).
- $\epsilon_d > 0$ is a small regularization constant ensuring $D^i \succ 0$ as $t \to \infty$.

Multiplying Eq. (5) by $(M^i)^{-1} = \frac{1}{m} (H^i)^{-1}$ naturally preconditions the gradient acceleration by $(H^i)^{-1}$, aligning the dynamics along local Levenberg–Marquardt descent directions.

### 6.4 Velocity-Based Neighbor State Prediction Under Delay

In real-world multi-robot communications, messages arrive with communication delay $\delta t$, packet drops, and asynchronous clock drift. Evaluating inter-robot edges with stale neighbor poses $\bar{X}_v$ introduces severe gradient perturbations that destabilize parallel gradient solvers.

CORD introduces a **velocity-extrapolation predictor**:
- Each robot transmits its current pose $X_j$ and its **body velocity** $\xi_j$, tagged with transmission timestamp $t_{\text{msg}}$.
- When receiving robot $i$ performs an update at local time $t_{\text{curr}}$, it calculates elapsed delay:
  $$\delta t \coloneq t_{\text{curr}} - t_{\text{msg}}$$
- Robot $i$ computes the **predicted neighbor pose** $\widehat{X}_j$ along the Riemannian geodesic:
  $$\widehat{X}_j \coloneq X_j \exp\left((\xi_j \, \delta t)^\wedge\right) \tag{Paper Alg. 2, Line 7}$$
- Robot $i$ uses $\widehat{X}_j$ to evaluate the inter-robot residual $r_{uv}(X_u, \widehat{X}_j)$.

In the paper's ablation studies (Section VI-D), omitting this predictive extrapolation caused distributed solvers to diverge under a communication delay of 7 steps, whereas CORD with velocity prediction remained strictly stable.

---

## 7. Energy Dissipation & Convergence Analysis (Section V)

### 7.1 Analytical Framework & Assumptions

- **Total System Energy:**
  $$E(X, \xi) \coloneq \mathcal{T}(\xi) + \mathcal{C}(X) = \frac{1}{2} \xi^\top M \xi + \mathcal{C}(X)$$
- **Assumption 1 (Lipschitz-type Gradient of Pullback, Boumal et al. [40]):**
  Let $g_{X_k}(\eta) \coloneq \mathcal{C}(X_k \exp(\eta^\wedge))$ be the pullback of the cost to the tangent space at $X_k$. There exists $L > 0$ such that for any tangent step $\eta$:
  $$\left|g_{X_k}(\eta) - \left(g_{X_k}(0) + \langle \nabla \mathcal{C}(X_k), \eta \rangle\right)\right| \le \frac{L}{2} \|\eta\|^2$$

### 7.2 Continuous-Time Dissipation (Eq. 11)

Differentiating the continuous total energy along the system trajectory:

$$\frac{dE}{dt} = \xi^\top M \dot{\xi} + \langle \nabla \mathcal{C}(X), \xi \rangle$$

Substituting $M \dot{\xi} = -\nabla \mathcal{C}(X) - D \xi + \operatorname{ad}_\xi^*(M \xi)$ and noting that $\xi^\top \operatorname{ad}_\xi^*(M \xi) = 0$ (by skew-symmetry of the Lie bracket adjoint):

$$\frac{dE}{dt} = -\xi^\top D \xi \le -\lambda_{\min}(D) \|\xi\|^2 \le 0 \tag{Paper Eq. 11}$$

Thus, continuous energy strictly decays toward zero velocity and stationary points of $\mathcal{C}$.

### 7.3 Discrete-Time Step-Size Bound (Theorem 1)

In discrete time, discretization errors introduce positive second-order terms. Theorem 1 establishes that under constant $M \succ 0$ and $D \succ 0$, total discrete energy satisfies $\Delta E_k = E_{k+1} - E_k \le 0$ provided:

$$\Delta t \le \frac{2 \, \xi_k^\top D \xi_k}{L \|\xi_{k+1}\|^2 + \|a_k\|_M^2 + 2 \, \xi_k^\top D a_k} \tag{Paper Eq. 12}$$

This provides explicit mathematical guidance for bounding integration step size $\Delta t$ to ensure numerical stability.

---

## 8. Experimental Evaluation, Baselines, and Reported Metrics

### 8.1 Evaluated Benchmark Datasets

The authors evaluated CORD across standard 2D and 3D SLAM datasets:
- **3D Benchmark Graphs:** Sphere-2500, Torus3D, Grid3D, Garage, Cubicle, Rim.
- **Partitioning:** Simulated multi-robot swarms (typically 5 robots) generated using graph partitioning.
- **Communication Regimes:**
  - *Synchronous:* Zero delay, all robots update in parallel.
  - *Asynchronous (Uniform Delay):* Constant delay of 5 integration steps.
  - *Asynchronous (Extreme Delay & Loss):* Random delays in range $[1, 10]$ steps combined with 10% random packet drops.
  - *Edge-Based:* Pairwise alternating communication mimicking MESA.

### 8.2 Baselines & Quantitative Findings

1. **Synchronous PGO:**
   - Evaluated relative optimality gap $(\mathcal{C} - \mathcal{C}^*) / \mathcal{C}^*$, where $\mathcal{C}^*$ is the centralized GTSAM / SE-Sync reference solution.
   - CORD achieved the lowest cost across nearly all 3D datasets for both chordal and geodesic metrics, matching or exceeding AMM-PGO and strictly outperforming Distributed Jacobi (DJ).
2. **Asynchronous PGO:**
   - Distributed Jacobi (ASAPP equivalent) exhibited severe oscillations and required manually tuned, heavily scaled-down step sizes to prevent divergence.
   - CORD's inertial momentum and velocity-based extrapolation produced smooth monotonic cost descent, maintaining lower final cost across all delay conditions.
3. **Runtime & Packet Overhead (Rim Dataset):**
   - Runtime per iteration: CORD 101.9 ms vs. DJ 43.0 ms.
   - Communication payload per packet: CORD 5.9 KB vs. DJ 3.2 KB (the difference stems from transmitting body velocity vectors $\xi \in \mathbb{R}^6$ alongside poses).
   - Iterations to reach $\Delta = 10^{-2}$: CORD required **80 iterations** vs. DJ's **924 iterations** (an $11.5\times$ reduction in communication rounds).

### 8.3 Stated Limitations & Unresolved Questions in the Paper

1. **Theoretical Convergence Scope:**
   - The formal proof of discrete energy dissipation (Theorem 1) is established **only for constant mass $M$ under synchronous communication**.
   - Proof for state-dependent $M(X)$ and asynchronous communication with packet loss remains an open theoretical conjecture, although empirically supported.
2. **Local Optimality vs. Global Verification:**
   - CORD is a local solver. It guarantees convergence to first-order critical points, but does not provide semidefinite relaxation certificates of global optimality (unlike centralized SE-Sync or distributed DC2-PGO).
3. **Hyperparameter Selection:**
   - Parameters $m$, $d$, and $\Delta t$ were determined empirically across datasets rather than computed via automated adaptive rules.

---

## 9. Traceability Summary for AstroMesh-X

| Paper Concept | Primary Reference | AstroMesh-X Mapping / Adaptation |
| :--- | :--- | :--- |
| **Objective** | Problem 1, Eq. 1 | Planar $SE(2)$ geodesic residual $r_{ij} = \operatorname{Log}(Z_{ij}^{-1} X_i^{-1} X_j)^\vee \in \mathbb{R}^3$. |
| **Damped Dynamics** | Prop. 1, Eq. 3 | Specialization to $SE(2)$ Lie algebra and co-adjoint operator $\operatorname{ad}_\xi^*(M \xi) \in \mathbb{R}^3$. |
| **Integrator** | Sec. IV-B, Alg. 1 | Semi-implicit Euler on $SE(2)$ exponential map with numerical singularity protection. |
| **Distributed Model** | Sec. IV-C, Alg. 2 | Multi-rover swarm graph partitions mapped via `SubgraphPartitionManager` and `PoseGraph`. |
| **Delay Predictor** | Alg. 2, Line 7 | Neighbor pose extrapolation $\widehat{X}_j = X_j \exp((\xi_j \delta t)^\wedge)$ within DTN message layer. |
| **Preconditioner** | Sec. IV-C | Local Hessian block $H^i \approx J_i^\top \Omega_i J_i + \lambda I$, evaluating constant vs. adaptive mass. |
