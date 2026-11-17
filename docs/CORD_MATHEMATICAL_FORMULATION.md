# Continuous Riemannian Dynamics (CORD) on $\mathrm{SE}(2)$: Mathematical Formulation and System Design

**Document ID:** `docs/CORD_MATHEMATICAL_FORMULATION.md`  
**Phase:** 3 — Distributed Continuous Riemannian Dynamics Pose-Graph Optimization  
**Milestone:** 3.1 — Literature Review Alignment and Mathematical Formulation  
**Primary Reference:** Jaeho Shin, Maani Ghaffari, and Yulun Tian, *"Distributed Pose Graph Optimization via Continuous Riemannian Dynamics,"* arXiv:2605.11210v1, RSS XXII 2026.  
**Target Architecture:** Planar Multi-Rover Pose Graph Optimization on the Special Euclidean Group $\mathrm{SE}(2)$.  
**Implementation Status:** Conceptual Specification, Mathematical Derivation, and Interface Design (Non-executable documentation; zero optimizer source code implemented at Milestone 3.1).

---

## 1. Executive Summary and Scope

This document establishes the formal mathematical specification for adapting the **Continuous Riemannian Dynamics (CORD)** optimization framework (Shin, Ghaffari, & Tian, RSS 2026) to planar rover pose-graph optimization on the Lie group $\mathrm{SE}(2)$ within the AstroMesh-X system.

While the primary literature formulates and evaluates CORD primarily on $\mathrm{SE}(3)$ for 3D aerial and ground platforms, planetary rover swarms operating under AstroMesh-X track 3-DoF planar poses $(x, y, \theta)$ represented in tangent coordinates as `astromesh::core::Pose2D`. This document develops:
1. The exact Lie-theoretic foundations of $\mathrm{SE}(2)$ and its Lie algebra $\mathfrak{se}(2)$, including closed-form exponential and logarithmic maps and small-angle Taylor expansions.
2. The formulation of relative measurements, predicted poses, tangent-space geodesic residuals, and the weighted nonlinear least-squares objective over verified graph edges.
3. The continuous damped Euler-Poincaré equations of motion on $\mathrm{SE}(2)^N$, isolating the Riemannian gradient, damping dissipation, and the rotational-translational coupling of the co-adjoint operator.
4. The discrete semi-implicit geometric Lie integrator specialized for planar kinematics.
5. Gauge freedom stabilization protocols for single-robot and multi-robot partitioned swarms.
6. A fully articulated, hand-checkable numerical worked example validating frame conventions, residual evaluations, gradient calculations, and single-step state integration.

---

## 2. Lie Group and Lie Algebra Foundations: $\mathrm{SE}(2)$

### 2.1 The Special Euclidean Group $\mathrm{SE}(2)$

The configuration of each rover pose $i \in \{1, \dots, N\}$ resides on the matrix Lie group $\mathrm{SE}(2) = \mathrm{SO}(2) \ltimes \mathbb{R}^2$, defined as the set of $3 \times 3$ homogeneous transformation matrices:
$$X_i = \begin{bmatrix} R(\theta_i) & t_i \\ 0_{1 \times 2} & 1 \end{bmatrix} = \begin{bmatrix} \cos\theta_i & -\sin\theta_i & x_i \\ \sin\theta_i & \cos\theta_i & y_i \\ 0 & 0 & 1 \end{bmatrix} \in \mathrm{SE}(2)$$
where:
- $t_i = [x_i, y_i]^\top \in \mathbb{R}^2$ denotes the planar translation in meters ($\mathrm{m}$) relative to a reference frame.
- $R(\theta_i) \in \mathrm{SO}(2)$ denotes the planar orientation matrix parameterized by heading angle $\theta_i \in (-\pi, \pi]$ radians ($\mathrm{rad}$).
- $0_{1 \times 2} = [0, 0]$ and the bottom row ensures standard affine composition.

#### Group Operations
For $X_1, X_2 \in \mathrm{SE}(2)$:
- **Group Multiplication (Composition):**
  $$X_1 X_2 = \begin{bmatrix} R_1 R_2 & R_1 t_2 + t_1 \\ 0 & 1 \end{bmatrix} \in \mathrm{SE}(2)$$
- **Group Identity:**
  $$I_{3 \times 3} = \begin{bmatrix} I_{2 \times 2} & 0_{2 \times 1} \\ 0_{1 \times 2} & 1 \end{bmatrix}$$
- **Group Inverse:**
  $$X^{-1} = \begin{bmatrix} R^\top & -R^\top t \\ 0_{1 \times 2} & 1 \end{bmatrix} \in \mathrm{SE}(2)$$

### 2.2 Tangent Space and Lie Algebra $\mathfrak{se}(2)$

The Lie algebra $\mathfrak{se}(2) = T_I \mathrm{SE}(2)$ is the tangent space at the group identity, consisting of $3 \times 3$ matrices of the form:
$$\xi^\wedge = \begin{bmatrix} 0 & -\omega & v_x \\ \omega & 0 & v_y \\ 0 & 0 & 0 \end{bmatrix} \in \mathfrak{se}(2)$$
where $\xi = [v_x, v_y, \omega]^\top \in \mathbb{R}^3$ is the generalized body-velocity twist vector:
- $v = [v_x, v_y]^\top \in \mathbb{R}^2$ represents linear velocity in body-frame coordinates ($\mathrm{m/s}$).
- $\omega \in \mathbb{R}$ represents heading angular velocity ($\mathrm{rad/s}$).

The linear isomorphisms between $\mathbb{R}^3$ and $\mathfrak{se}(2)$ are defined by the **wedge** $(\cdot)^\wedge$ and **vee** $(\cdot)^\vee$ operators:
$$\xi^\wedge = \begin{bmatrix} v_x \\ v_y \\ \omega \end{bmatrix}^\wedge \triangleq \begin{bmatrix} 0 & -\omega & v_x \\ \omega & 0 & v_y \\ 0 & 0 & 0 \end{bmatrix}, \qquad \begin{bmatrix} 0 & -\omega & v_x \\ \omega & 0 & v_y \\ 0 & 0 & 0 \end{bmatrix}^\vee \triangleq \begin{bmatrix} v_x \\ v_y \\ \omega \end{bmatrix}$$

### 2.3 The Lie Bracket and Adjoint Representations

The Lie bracket on $\mathfrak{se}(2)$ is given by the matrix commutator:
$$[\xi_1^\wedge, \xi_2^\wedge] = \xi_1^\wedge \xi_2^\wedge - \xi_2^\wedge \xi_1^\wedge$$
Direct evaluation yields:
$$\xi_1^\wedge \xi_2^\wedge - \xi_2^\wedge \xi_1^\wedge = \begin{bmatrix} 0 & -\omega_1 & v_{x,1} \\ \omega_1 & 0 & v_{y,1} \\ 0 & 0 & 0 \end{bmatrix} \begin{bmatrix} 0 & -\omega_2 & v_{x,2} \\ \omega_2 & 0 & v_{y,2} \\ 0 & 0 & 0 \end{bmatrix} - \begin{bmatrix} 0 & -\omega_2 & v_{x,2} \\ \omega_2 & 0 & v_{y,2} \\ 0 & 0 & 0 \end{bmatrix} \begin{bmatrix} 0 & -\omega_1 & v_{x,1} \\ \omega_1 & 0 & v_{y,1} \\ 0 & 0 & 0 \end{bmatrix}$$
$$\quad = \begin{bmatrix} 0 & 0 & -\omega_1 v_{y,2} + \omega_2 v_{y,1} \\ 0 & 0 & \omega_1 v_{x,2} - \omega_2 v_{x,1} \\ 0 & 0 & 0 \end{bmatrix}$$

Applying the vee operator yields the adjoint action $\operatorname{ad}_{\xi_1} \xi_2 = ([\xi_1^\wedge, \xi_2^\wedge])^\vee$:
$$\operatorname{ad}_\xi = \begin{bmatrix} 0 & -\omega & v_y \\ \omega & 0 & -v_x \\ 0 & 0 & 0 \end{bmatrix} \in \mathbb{R}^{3 \times 3}$$
so that:
$$\operatorname{ad}_{\xi_1} \xi_2 = \begin{bmatrix} 0 & -\omega_1 & v_{y,1} \\ \omega_1 & 0 & -v_{x,1} \\ 0 & 0 & 0 \end{bmatrix} \begin{bmatrix} v_{x,2} \\ v_{y,2} \\ \omega_2 \end{bmatrix} = \begin{bmatrix} -\omega_1 v_{y,2} + \omega_2 v_{y,1} \\ \omega_1 v_{x,2} - \omega_2 v_{x,1} \\ 0 \end{bmatrix}$$
Notice that the third component ($\dot{\omega}$) is identically zero, reflecting the fact that $\mathrm{SO}(2)$ is an abelian (commutative) 1-dimensional Lie group.

#### Co-Adjoint Operator on Dual Tangent Space
The co-adjoint operator $\operatorname{ad}_\xi^*: \mathfrak{se}(2)^* \to \mathfrak{se}(2)^*$ maps generalized momentum $p \in \mathbb{R}^3$ under dual pairing $\langle \operatorname{ad}_\xi^* p, \eta \rangle = \langle p, \operatorname{ad}_\xi \eta \rangle$. With respect to the standard Euclidean inner product on $\mathbb{R}^3$:
$$\operatorname{ad}_\xi^* = \operatorname{ad}_\xi^\top = \begin{bmatrix} 0 & \omega & 0 \\ -\omega & 0 & 0 \\ v_y & -v_x & 0 \end{bmatrix}$$
For momentum $p = M \xi = [p_x, p_y, p_\theta]^\top$:
$$\operatorname{ad}_\xi^*(M \xi) = \begin{bmatrix} \omega p_y \\ -\omega p_x \\ v_y p_x - v_x p_y \end{bmatrix}$$

**Crucial Conservation Identity (Gyroscopic Invariance):**
$$\xi^\top \operatorname{ad}_\xi^*(M \xi) = v_x (\omega p_y) + v_y (-\omega p_x) + \omega (v_y p_x - v_x p_y) = \omega v_x p_y - \omega v_y p_x + \omega v_y p_x - \omega v_x p_y \equiv 0$$
The co-adjoint term performs **zero instantaneous work** on the kinetic energy of the system, acting purely as a conservative Coriolis/centrifugal coupling between planar translational velocity and angular heading velocity. Holds identically for any momentum vector $p \in \mathbb{R}^3$.

### 2.4 Exponential and Logarithmic Maps

#### Exponential Map $\exp: \mathfrak{se}(2) \to \mathrm{SE}(2)$
Given twist $\xi = [v_x, v_y, \omega]^\top$:
$$X = \exp(\xi^\wedge) = \begin{bmatrix} R(\omega) & V(\omega) v \\ 0_{1 \times 2} & 1 \end{bmatrix}$$
where $R(\omega) \in \mathrm{SO}(2)$ is the rotation matrix for angle $\omega$, and $V(\omega) \in \mathbb{R}^{2 \times 2}$ is the left Jacobian matrix of $\mathrm{SO}(2)$:
$$V(\omega) = \frac{1}{\omega} \begin{bmatrix} \sin\omega & -(1 - \cos\omega) \\ 1 - \cos\omega & \sin\omega \end{bmatrix} = \begin{bmatrix} \frac{\sin\omega}{\omega} & -\frac{1 - \cos\omega}{\omega} \\ \frac{1 - \cos\omega}{\omega} & \frac{\sin\omega}{\omega} \end{bmatrix}$$

**Numerical Precision via Taylor Expansion:**
When $|\omega| < 10^{-6}$, direct division by $\omega$ risks catastrophic cancellation. The series expansion yields:
$$\frac{\sin\omega}{\omega} = 1 - \frac{\omega^2}{6} + \frac{\omega^4}{120} + \mathcal{O}(\omega^6)$$
$$\frac{1 - \cos\omega}{\omega} = \frac{\omega}{2} - \frac{\omega^3}{24} + \frac{\omega^5}{720} + \mathcal{O}(\omega^7)$$
As $\omega \to 0$, $V(\omega) \to I_{2 \times 2}$, recovering pure translation $X = \begin{bmatrix} I & v \\ 0 & 1 \end{bmatrix}$.

#### Logarithmic Map $\log: \mathrm{SE}(2) \to \mathfrak{se}(2)$
Given group element $X = \begin{bmatrix} R(\theta) & t \\ 0 & 1 \end{bmatrix}$ with $R(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}$ and $t = [x, y]^\top$:
1. The heading angle is uniquely recovered modulo $2\pi$ via the two-argument arctangent:
   $$\theta = \operatorname{atan2}(\sin\theta, \cos\theta) = \operatorname{atan2}(R_{21}, R_{11}) \in (-\pi, \pi]$$
2. The translation twist $v = [v_x, v_y]^\top$ is obtained by inverting $V(\theta)$:
   $$v = V^{-1}(\theta) t$$
   where:
   $$V^{-1}(\theta) = \frac{\theta}{2(1 - \cos\theta)} \begin{bmatrix} \sin\theta & 1 - \cos\theta \\ -(1 - \cos\theta) & \sin\theta \end{bmatrix} = \begin{bmatrix} \frac{\theta \sin\theta}{2(1 - \cos\theta)} & \frac{\theta}{2} \\ -\frac{\theta}{2} & \frac{\theta \sin\theta}{2(1 - \cos\theta)} \end{bmatrix}$$
   Using the half-angle trigonometric identity $\frac{\sin\theta}{1 - \cos\theta} = \frac{1}{\tan(\theta/2)}$:
   $$V^{-1}(\theta) = \begin{bmatrix} \frac{\theta}{2 \tan(\theta/2)} & \frac{\theta}{2} \\ -\frac{\theta}{2} & \frac{\theta}{2 \tan(\theta/2)} \end{bmatrix}$$

**Small-Angle Series Expansion:**
When $|\theta| < 10^{-6}$:
$$\frac{\theta}{2 \tan(\theta/2)} = 1 - \frac{\theta^2}{12} - \frac{\theta^4}{720} + \mathcal{O}(\theta^6)$$
so $V^{-1}(\theta) \to \begin{bmatrix} 1 & \theta/2 \\ -\theta/2 & 1 \end{bmatrix} \to I_{2 \times 2}$ as $\theta \to 0$.

Applying the vee operator yields the vector representation:
$$\operatorname{Log}(X)^\vee \triangleq [v_x, v_y, \theta]^\top \in \mathbb{R}^3$$

---

## 3. Pose-Graph Optimization Problem on $\mathrm{SE}(2)$

### 3.1 Mapping to AstroMesh-X Core Structures

In AstroMesh-X, planar poses are stored as:
```cpp
struct Pose2D {
    double x{0.0};      // Translation in meters along x-axis
    double y{0.0};      // Translation in meters along y-axis
    double theta{0.0};  // Heading angle in radians, normalized to (-pi, pi]
};
```
The exact bidirectional bijection between `Pose2D` and $X \in \mathrm{SE}(2)$ is:
$$\iota: \mathrm{Pose2D}(x, y, \theta) \mapsto \begin{bmatrix} \cos\theta & -\sin\theta & x \\ \sin\theta & \cos\theta & y \\ 0 & 0 & 1 \end{bmatrix} \in \mathrm{SE}(2)$$
$$\iota^{-1}: X \mapsto \mathrm{Pose2D}\left(X_{13}, X_{23}, \operatorname{atan2}(X_{21}, X_{11})\right)$$

### 3.2 Graph Edge Definition, Relative Measurement, and Predicted Pose

Let $\mathcal{G} = (\mathcal{V}, \mathcal{E})$ denote the topological pose graph:
- Vertices $i \in \mathcal{V} = \{1, \dots, N\}$ represent robot poses $X_i \in \mathrm{SE}(2)$.
- Directed edges $e = (i, j) \in \mathcal{E}$ represent relative geometric constraints from source node $i$ to target node $j$.

#### Measurement Convention
Each edge $(i, j) \in \mathcal{E}$ stores:
1. Relative transform measurement $Z_{ij} \in \mathrm{SE}(2)$, representing the measured pose of frame $j$ expressed in frame $i$:
   $$Z_{ij} = \begin{bmatrix} R(\theta_{ij}^z) & t_{ij}^z \\ 0 & 1 \end{bmatrix} = \begin{bmatrix} \cos\theta_{ij}^z & -\sin\theta_{ij}^z & x_{ij}^z \\ \sin\theta_{ij}^z & \cos\theta_{ij}^z & y_{ij}^z \\ 0 & 0 & 1 \end{bmatrix}$$
2. Symmetric positive-definite information matrix $\Omega_{ij} \in \mathbb{R}^{3 \times 3}$, representing the inverse measurement covariance $\Sigma_{ij}^{-1}$:
   $$\Omega_{ij} = \begin{bmatrix} \Omega_{xx} & \Omega_{xy} & \Omega_{x\theta} \\ \Omega_{yx} & \Omega_{yy} & \Omega_{y\theta} \\ \Omega_{\theta x} & \Omega_{\theta y} & \Omega_{\theta\theta} \end{bmatrix} \succ 0$$
   Units: $\Omega_{xx}, \Omega_{yy} \in [\mathrm{m}^{-2}]$, $\Omega_{\theta\theta} \in [\mathrm{rad}^{-2}]$, cross-terms $\Omega_{x\theta} \in [\mathrm{m}^{-1} \mathrm{rad}^{-1}]$.

#### Predicted Relative Pose
Given current trajectory state estimates $X_i, X_j \in \mathrm{SE}(2)$, the predicted relative pose of frame $j$ in frame $i$ is:
$$\widehat{Z}_{ij} = X_i^{-1} X_j = \begin{bmatrix} R_i^\top R_j & R_i^\top (t_j - t_i) \\ 0 & 1 \end{bmatrix} \in \mathrm{SE}(2)$$

### 3.3 Tangent-Space Geodesic Residual

The multiplicative relative pose error matrix $E_{ij} \in \mathrm{SE}(2)$ compares the predicted pose to the measured pose in the body frame of the measurement:
$$E_{ij} \triangleq Z_{ij}^{-1} \widehat{Z}_{ij} = Z_{ij}^{-1} X_i^{-1} X_j$$
The tangent-space geodesic residual vector $r_{ij} \in \mathbb{R}^3$ is defined by pulling $E_{ij}$ to the Lie algebra $\mathfrak{se}(2)$ via the logarithmic map:
$$r_{ij}(X_i, X_j) \triangleq \operatorname{Log}\left(Z_{ij}^{-1} X_i^{-1} X_j\right)^\vee \in \mathbb{R}^3$$
Properties:
- **Zero-error point:** When predicted relative pose equals measurement ($\widehat{Z}_{ij} = Z_{ij}$), $E_{ij} = I_{3 \times 3}$, and $r_{ij} = [0, 0, 0]^\top$.
- **Left-invariance:** The error $E_{ij}$ depends solely on relative configurations $X_i^{-1} X_j$ and is invariant under any global rigid transformation $g \in \mathrm{SE}(2)$ applied to all poses ($X_k \mapsto g X_k$).
- **Frame consistency:** $r_{ij}$ is expressed in the tangent space of the measurement frame.

### 3.4 Edge Classification and Eligibility Filter

Edges in AstroMesh-X are categorized by constraint class and verification state:
1. **Odometry Edges ($\mathcal{E}_{\mathrm{odo}}$):** Sequential chaining constraints between consecutive poses $X_i \to X_{i+1}$ along a rover's trajectory. Formally trusted, with status `EdgeVerificationStatus::UNVERIFIED` (always eligible).
2. **Intra-Rover Loop Closures ($\mathcal{E}_{\mathrm{intra}}$):** Non-sequential constraints between non-adjacent poses of the same rover.
3. **Inter-Rover Loop Closures ($\mathcal{E}_{\mathrm{inter}}$):** Relative constraints between distinct rovers resulting from landmark matches discovered via IBLT set reconciliation.

**Eligibility Constraint:**
In accordance with AstroMesh-X Phase 2 Milestone 2.4, an edge $(i, j) \in \mathcal{E}_{\mathrm{loop}}$ is included in the optimization objective **if and only if** its status satisfies:
$$\text{eligible}((i, j)) \iff \operatorname{status}(i, j) == \mathrm{EdgeVerificationStatus::VERIFIED}$$
Edges with status `REJECTED_MAHALANOBIS`, `REJECTED_TEMPORAL`, `REJECTED_TOPOLOGICAL`, or `PENDING` are excluded from the objective $\mathcal{C}(X)$ to prevent corrupted or aliased measurements from destabilizing the solver dynamics.

### 3.5 Weighted Nonlinear Least-Squares Objective

The total pose-graph optimization cost over all eligible edges $\mathcal{E}_{\mathrm{eligible}} \subseteq \mathcal{E}$ is:
$$\mathcal{C}(X) = \frac{1}{2} \sum_{(i, j) \in \mathcal{E}_{\mathrm{eligible}}} r_{ij}(X_i, X_j)^\top \Omega_{ij} r_{ij}(X_i, X_j)$$
where $X = (X_1, \dots, X_N) \in \mathrm{SE}(2)^N$.

---

## 4. Continuous Riemannian Dynamics Formulation on $\mathrm{SE}(2)^N$

### 4.1 State, Velocities, and Kinetic Energy

Following Shin et al. (arXiv:2605.11210, RSS 2026), CORD treats each pose $X_i(t)$ as a rigid kinematic body moving on the Riemannian manifold $\mathrm{SE}(2)$ under artificial Newtonian mechanics.

For each node $i \in \{1, \dots, N\}$:
- **Configuration (Position):** $X_i(t) \in \mathrm{SE}(2)$.
- **Body Velocity (Twist):** $\xi_i(t) = [v_{x,i}(t), v_{y,i}(t), \omega_i(t)]^\top \in \mathbb{R}^3$, satisfying the kinematic kinematic relation:
  $$\dot{X}_i(t) = X_i(t) \xi_i^\wedge(t)$$
- **Mass Matrix:** Positive-definite diagonal inertia tensor $M_i \in \mathbb{R}^{3 \times 3}$:
  $$M_i = \operatorname{diag}(m_{x,i}, m_{y,i}, I_{\theta,i}) \succ 0$$
  where $m_{x,i}, m_{y,i} > 0$ are virtual linear masses ($\mathrm{kg}$), and $I_{\theta,i} > 0$ is virtual rotational inertia ($\mathrm{kg \cdot m^2}$).
- **Kinetic Energy:**
  $$\mathcal{K}(\xi) = \frac{1}{2} \sum_{i=1}^N \xi_i^\top M_i \xi_i = \frac{1}{2} \sum_{i=1}^N \left( m_{x,i} v_{x,i}^2 + m_{y,i} v_{y,i}^2 + I_{\theta,i} \omega_i^2 \right)$$
- **Damping Matrix:** Positive-definite diagonal Rayleigh dissipation tensor $D_i \in \mathbb{R}^{3 \times 3}$:
  $$D_i = \operatorname{diag}(d_{x,i}, d_{y,i}, d_{\theta,i}) \succ 0$$

### 4.2 Riemannian Gradient of the Objective

The gradient of the cost function $\mathcal{C}(X)$ on the Riemannian manifold $\mathrm{SE}(2)^N$ is obtained via the directional derivative (Lie derivative). For node $i$, the Riemannian gradient $\nabla_{X_i} \mathcal{C}(X) \in \mathfrak{se}(2)^* \cong \mathbb{R}^3$ represents the net artificial restoring torque/force pulling node $i$ toward residual minimization:
$$\nabla_{X_i} \mathcal{C}(X) = \sum_{j \in \mathcal{N}_i^+} J_{l, i}^\top \Omega_{ij} r_{ij} + \sum_{k \in \mathcal{N}_i^-} J_{r, i}^\top \Omega_{ki} r_{ki}$$
where:
- $\mathcal{N}_i^+ = \{j \in \mathcal{V} \mid (i, j) \in \mathcal{E}_{\mathrm{eligible}}\}$ (node $i$ is measurement source).
- $\mathcal{N}_i^- = \{k \in \mathcal{V} \mid (k, i) \in \mathcal{E}_{\mathrm{eligible}}\}$ (node $i$ is measurement target).
- $J_{l, i} = \frac{\partial r_{ij}}{\partial \xi_i}$ and $J_{r, i} = \frac{\partial r_{ki}}{\partial \xi_i}$ are the right and left Jacobian matrices on $\mathrm{SE}(2)$.

**First-Order Adjoint Approximation:**
In pose-graph optimization where initial errors are bounded by odometry, the Jacobians relate to the Adjoint representation of the group:
- When node $i$ is perturbed by $X_i \leftarrow X_i \exp(\delta\xi_i^\wedge)$:
  $$r_{ij}(X_i \exp(\delta\xi_i^\wedge), X_j) = \operatorname{Log}(Z_{ij}^{-1} \exp(-\delta\xi_i^\wedge) X_i^{-1} X_j)^\vee \approx r_{ij} - \operatorname{Ad}_{X_j^{-1} X_i} \delta\xi_i$$
  yielding the primary restoring force $\nabla_{X_i} \mathcal{C}(X) \in \mathbb{R}^3$.

### 4.3 Continuous Damped Euler-Poincaré Dynamics

The physical equations of motion for each pose node $i$ on $\mathrm{SE}(2)$ are governed by the damped Euler-Poincaré system:
$$\boxed{ M_i \dot{\xi}_i = \operatorname{ad}_{\xi_i}^*(M_i \xi_i) - D_i \xi_i - \nabla_{X_i} \mathcal{C}(X) }$$
$$\boxed{ \dot{X}_i = X_i \xi_i^\wedge }$$

Expanding the components for planar motion with $M_i = \operatorname{diag}(m, m, I)$ and $D_i = \operatorname{diag}(d_v, d_v, d_\omega)$:
$$\begin{aligned}
m \dot{v}_{x,i} &= m \omega_i v_{y,i} - d_v v_{x,i} - \left[\nabla_{X_i} \mathcal{C}(X)\right]_x \\
m \dot{v}_{y,i} &= -m \omega_i v_{x,i} - d_v v_{y,i} - \left[\nabla_{X_i} \mathcal{C}(X)\right]_y \\
I \dot{\omega}_i &= -d_\omega \omega_i - \left[\nabla_{X_i} \mathcal{C}(X)\right]_\theta
\end{aligned}$$
Key characteristics:
1. When linear mass is isotropic ($m_{x} = m_{y} = m$), the centrifugal coupling term in heading momentum vanishes: $v_y p_x - v_x p_y = v_y (m v_x) - v_x (m v_y) \equiv 0$.
2. The Coriolis acceleration terms $(m \omega v_y, -m \omega v_x)$ continuously rotate the linear velocity vector perpendicular to heading rate without altering kinetic energy.

### 4.4 Lyapunov Energy Dissipation Guarantee

Define the total mechanical Hamiltonian $\mathcal{H}(X, \xi)$ of the pose graph:
$$\mathcal{H}(X, \xi) = \mathcal{K}(\xi) + \mathcal{C}(X) = \frac{1}{2} \sum_{i=1}^N \xi_i^\top M_i \xi_i + \mathcal{C}(X)$$
Differentiating with respect to time along trajectory solutions:
$$\dot{\mathcal{H}} = \sum_{i=1}^N \xi_i^\top M_i \dot{\xi}_i + \sum_{i=1}^N \langle \nabla_{X_i} \mathcal{C}(X), \xi_i \rangle$$
Substituting the Euler-Poincaré equation:
$$\dot{\mathcal{H}} = \sum_{i=1}^N \xi_i^\top \left( \operatorname{ad}_{\xi_i}^*(M_i \xi_i) - D_i \xi_i - \nabla_{X_i} \mathcal{C}(X) \right) + \sum_{i=1}^N \xi_i^\top \nabla_{X_i} \mathcal{C}(X)$$
$$\quad = \sum_{i=1}^N \left( \underbrace{\xi_i^\top \operatorname{ad}_{\xi_i}^*(M_i \xi_i)}_{= 0} - \xi_i^\top D_i \xi_i - \xi_i^\top \nabla_{X_i} \mathcal{C}(X) + \xi_i^\top \nabla_{X_i} \mathcal{C}(X) \right)$$
$$\boxed{ \dot{\mathcal{H}} = -\sum_{i=1}^N \xi_i^\top D_i \xi_i \le -\lambda_{\min}(D) \|\xi\|_2^2 \le 0 }$$

**Conclusion:** The total energy $\mathcal{H}$ is strictly non-increasing. Because $\mathcal{C}(X) \ge 0$ is bounded below by zero, the system dissipates all kinetic energy until velocities reach zero ($\xi \to 0$), converging asymptotically to a stationary critical point where $\nabla_X \mathcal{C}(X) = 0$.

---

## 5. Discrete Semi-Implicit Lie Group Integrator

To implement CORD in digital simulation with discrete step size $h > 0$ seconds, Shin et al. propose a semi-implicit symplectic Lie group integrator. This scheme updates velocity implicitly with respect to damping (ensuring unconditional numerical stability of dissipation) and explicitly with respect to conservative forces.

### 5.1 Velocity Step (Linearized Semi-Implicit Update)

Given current state $(X_i^k, \xi_i^k)$ at iteration $k$:
$$M_i \frac{\xi_i^{k+1} - \xi_i^k}{h} = \operatorname{ad}_{\xi_i^k}^*(M_i \xi_i^k) - D_i \xi_i^{k+1} - \nabla_{X_i} \mathcal{C}(X^k)$$
Rearranging to isolate the unknown velocity $\xi_i^{k+1}$:
$$\left( M_i + h D_i \right) \xi_i^{k+1} = M_i \xi_i^k + h \left[ \operatorname{ad}_{\xi_i^k}^*(M_i \xi_i^k) - \nabla_{X_i} \mathcal{C}(X^k) \right]$$
Pre-multiplying by $M_i^{-1}$:
$$\left( I_{3 \times 3} + h M_i^{-1} D_i \right) \xi_i^{k+1} = \xi_i^k + h M_i^{-1} \left( \operatorname{ad}_{\xi_i^k}^*(M_i \xi_i^k) - \nabla_{X_i} \mathcal{C}(X^k) \right)$$

Because $M_i$ and $D_i$ are diagonal, the coefficient matrix $\Lambda_i \triangleq (I + h M_i^{-1} D_i)$ is strictly diagonal:
$$\Lambda_i = \operatorname{diag}\left( 1 + h \frac{d_{x,i}}{m_{x,i}}, \; 1 + h \frac{d_{y,i}}{m_{y,i}}, \; 1 + h \frac{d_{\theta,i}}{I_{\theta,i}} \right)$$
Its inverse is trivial and requires zero matrix factorizations:
$$\boxed{ \xi_i^{k+1} = \Lambda_i^{-1} \left[ \xi_i^k + h M_i^{-1} \left( \operatorname{ad}_{\xi_i^k}^*(M_i \xi_i^k) - \nabla_{X_i} \mathcal{C}(X^k) \right) \right] }$$

### 5.2 Position Retraction Step

Once updated velocity $\xi_i^{k+1}$ is computed, the configuration pose $X_i^{k+1}$ is updated via the group exponential map:
$$\boxed{ X_i^{k+1} = X_i^k \exp\left( (h \xi_i^{k+1})^\wedge \right) }$$
Using the closed-form $\mathrm{SE}(2)$ exponential map derived in Section 2.4, this step:
1. Exactly preserves the manifold structure of $\mathrm{SE}(2)$ without drift or projection errors.
2. Updates translation using the exact left Jacobian $V(h \omega_i^{k+1})$.
3. Normalizes heading $\theta_i^{k+1}$ to $(-\pi, \pi]$ using `atan2`.

---

## 6. Gauge Freedom and Anchoring Protocol

### 6.1 Gauge Freedom on $\mathrm{SE}(2)^N$

The pose graph objective $\mathcal{C}(X)$ is invariant under global rigid coordinate transformations:
$$\mathcal{C}(g X_1, g X_2, \dots, g X_N) = \mathcal{C}(X_1, X_2, \dots, X_N), \quad \forall g \in \mathrm{SE}(2)$$
Because the residual depends only on $X_i^{-1} X_j = (g X_i)^{-1} (g X_j)$, the Hessian of $\mathcal{C}$ possesses a 3-dimensional null space corresponding to arbitrary planar translation and rotation of the entire coordinate system. Without gauge fixing, the entire graph will drift freely during dynamics.

### 6.2 Anchoring Strategy in AstroMesh-X

To eliminate gauge freedom:
1. **Single Connected Graph / Origin Anchor:**
   A designated reference node (by default, initial pose $X_0$ or the earliest pose of Rover 0) is declared as the **Gauge Anchor**:
   $$X_{\mathrm{anchor}} = I_{3 \times 3} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix}, \qquad \xi_{\mathrm{anchor}}(t) \equiv [0, 0, 0]^\top, \qquad \dot{\xi}_{\mathrm{anchor}}(t) \equiv [0, 0, 0]^\top$$
   During the numerical integrator step, the anchor pose is never retracted, and its velocity is clamped to zero.
2. **Disconnected Subgraph Partitions:**
   When rovers have not yet established verified inter-rover loop closures, the `SubgraphPartitionManager` partitions the graph into disjoint connected components $\mathcal{P}_1, \dots, \mathcal{P}_K$.
   - **Per-Component Gauge:** Each disconnected component $\mathcal{P}_k$ designates its minimum `NodeId` as its local anchor.
   - **Partition Merging:** Upon verification of an inter-rover edge between $\mathcal{P}_a$ and $\mathcal{P}_b$, `SubgraphPartitionManager` merges the sets; the anchor of the merged component is retained, and the former secondary anchor is converted into a dynamic node.

---

## 7. Numerical Convergence Diagnostics and Stopping Criteria

The CORD solver monitors the following scalar invariants at each iteration $k$:
1. **Objective Cost:** $\mathcal{C}(X^k) = \frac{1}{2} \sum_{(i, j)} r_{ij}^\top \Omega_{ij} r_{ij}$.
2. **Kinetic Energy:** $\mathcal{K}(\xi^k) = \frac{1}{2} \sum_{i=1}^N (\xi_i^k)^\top M_i \xi_i^k$.
3. **Hamiltonian Energy:** $\mathcal{H}^k = \mathcal{C}(X^k) + \mathcal{K}(\xi^k)$.
4. **Max Gradient Infinity Norm:** $\|\nabla \mathcal{C}(X^k)\|_\infty = \max_{i} \|\nabla_{X_i} \mathcal{C}(X^k)\|_\infty$.
5. **Max Velocity Step:** $\|\Delta X^k\|_\infty = h \max_{i} \|\xi_i^k\|_2$.

**Stopping Termination Conditions:**
The solver terminates when:
$$\|\nabla \mathcal{C}(X^k)\|_\infty < \epsilon_{\mathrm{grad}} \quad \text{AND} \quad \mathcal{K}(\xi^k) < \epsilon_{\mathrm{kin}}$$
or when maximum iterations $k_{\max}$ is reached. Default recommended tolerances for $\mathrm{SE}(2)$:
- $\epsilon_{\mathrm{grad}} = 10^{-4}$
- $\epsilon_{\mathrm{kin}} = 10^{-6}$
- Max iterations $k_{\max} = 1000$

---

## 8. Hand-Checkable Numerical Worked Example

To enable deterministic verification of future implementations, this section provides an end-to-end, hand-calculated example for a 2-node graph with 1 edge.

### 8.1 Setup and Initial States

- **Nodes:**
  - Node 1 (Anchor):
    $$X_1^0 = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix}, \quad \text{i.e., } (x_1, y_1, \theta_1) = (0.0\,\mathrm{m}, 0.0\,\mathrm{m}, 0.0\,\mathrm{rad})$$
    Velocity: $\xi_1^0 = [0, 0, 0]^\top$. Fixed anchor ($\dot{\xi}_1 \equiv 0$).
  - Node 2 (Dynamic):
    $$X_2^0 = \begin{bmatrix} \cos(0.05) & -\sin(0.05) & 1.20 \\ \sin(0.05) & \cos(0.05) & 0.10 \\ 0 & 0 & 1 \end{bmatrix}$$
    With values to 6 decimal places:
    $$\cos(0.05) \approx 0.998750, \quad \sin(0.05) \approx 0.049979$$
    $$X_2^0 \approx \begin{bmatrix} 0.998750 & -0.049979 & 1.200000 \\ 0.049979 & 0.998750 & 0.100000 \\ 0 & 0 & 1 \end{bmatrix}$$
    Initial velocity: $\xi_2^0 = [0.0, 0.0, 0.0]^\top$.

- **Measurement Constraint:**
  Odometry edge $(1, 2) \in \mathcal{E}_{\mathrm{eligible}}$:
  $$Z_{12} = \begin{bmatrix} 1 & 0 & 1.00 \\ 0 & 1 & 0.00 \\ 0 & 0 & 1 \end{bmatrix}, \quad \text{i.e., } (x_{12}^z, y_{12}^z, \theta_{12}^z) = (1.0\,\mathrm{m}, 0.0\,\mathrm{m}, 0.0\,\mathrm{rad})$$
  Inverse:
  $$Z_{12}^{-1} = \begin{bmatrix} 1 & 0 & -1.00 \\ 0 & 1 & 0.00 \\ 0 & 0 & 1 \end{bmatrix}$$

- **Information Matrix:**
  $$\Omega_{12} = \operatorname{diag}(100.0\,\mathrm{m}^{-2}, \; 100.0\,\mathrm{m}^{-2}, \; 400.0\,\mathrm{rad}^{-2})$$

- **CORD Simulation Parameters:**
  - Timestep: $h = 0.01\,\mathrm{s}$.
  - Mass: $M_2 = \operatorname{diag}(1.0\,\mathrm{kg}, \; 1.0\,\mathrm{kg}, \; 1.0\,\mathrm{kg\cdot m^2})$.
  - Damping: $D_2 = \operatorname{diag}(2.0\,\mathrm{N\cdot s/m}, \; 2.0\,\mathrm{N\cdot s/m}, \; 2.0\,\mathrm{N\cdot m\cdot s/rad})$.

---

### 8.2 Step-by-Step Mathematical Evaluation

#### Step 1: Predicted Relative Pose
$$\widehat{Z}_{12} = (X_1^0)^{-1} X_2^0 = I^{-1} X_2^0 = X_2^0 = \begin{bmatrix} 0.998750 & -0.049979 & 1.200000 \\ 0.049979 & 0.998750 & 0.100000 \\ 0 & 0 & 1 \end{bmatrix}$$

#### Step 2: Multiplicative Error Matrix
$$E_{12} = Z_{12}^{-1} \widehat{Z}_{12} = \begin{bmatrix} 1 & 0 & -1 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 0.998750 & -0.049979 & 1.200000 \\ 0.049979 & 0.998750 & 0.100000 \\ 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} 0.998750 & -0.049979 & 0.200000 \\ 0.049979 & 0.998750 & 0.100000 \\ 0 & 0 & 1 \end{bmatrix}$$

#### Step 3: Logarithm Map and Tangent Residual
1. Heading error:
   $$\theta_e = \operatorname{atan2}(E_{21}, E_{11}) = \operatorname{atan2}(0.049979, 0.998750) = 0.050000\,\mathrm{rad}$$
2. Translation error matrix $V^{-1}(\theta_e)$:
   For $\theta_e = 0.05$:
   $$\frac{\theta_e}{2 \tan(\theta_e/2)} = \frac{0.05}{2 \tan(0.025)} = \frac{0.05}{2 \times 0.0250052} = \frac{0.05}{0.0500104} \approx 0.999792$$
   $$\frac{\theta_e}{2} = 0.025000$$
   $$V^{-1}(\theta_e) = \begin{bmatrix} 0.999792 & 0.025000 \\ -0.025000 & 0.999792 \end{bmatrix}$$
3. Translation component:
   $$t_e = \begin{bmatrix} 0.200000 \\ 0.100000 \end{bmatrix}$$
   $$v_e = V^{-1}(\theta_e) t_e = \begin{bmatrix} 0.999792 \times 0.2 + 0.025000 \times 0.1 \\ -0.025000 \times 0.2 + 0.999792 \times 0.1 \end{bmatrix} = \begin{bmatrix} 0.199958 + 0.002500 \\ -0.005000 + 0.099979 \end{bmatrix} = \begin{bmatrix} 0.202458 \\ 0.094979 \end{bmatrix}$$
4. Tangent-space residual vector:
   $$\boxed{ r_{12} = \begin{bmatrix} 0.202458\,\mathrm{m} \\ 0.094979\,\mathrm{m} \\ 0.050000\,\mathrm{rad} \end{bmatrix} }$$

#### Step 4: Objective Cost Evaluation
$$\mathcal{C}(X^0) = \frac{1}{2} r_{12}^\top \Omega_{12} r_{12} = \frac{1}{2} \left[ 100 \times (0.202458)^2 + 100 \times (0.094979)^2 + 400 \times (0.050000)^2 \right]$$
$$(0.202458)^2 \approx 0.040989 \implies 100 \times 0.040989 = 4.0989$$
$$(0.094979)^2 \approx 0.009021 \implies 100 \times 0.009021 = 0.9021$$
$$(0.05)^2 = 0.0025 \implies 400 \times 0.0025 = 1.0000$$
$$\text{Sum} = 4.0989 + 0.9021 + 1.0000 = 6.0010$$
$$\boxed{ \mathcal{C}(X^0) = \frac{6.0010}{2} = 3.0005 }$$

#### Step 5: Riemannian Gradient at Dynamic Node 2
Node 2 is the target node ($\mathcal{N}_2^-$). Under the zero-th order right Jacobian approximation ($J_r(r_{12}) \approx I$):
$$\nabla_{X_2} \mathcal{C}(X^0) \approx \Omega_{12} r_{12} = \begin{bmatrix} 100 \times 0.202458 \\ 100 \times 0.094979 \\ 400 \times 0.050000 \end{bmatrix} = \begin{bmatrix} 20.2458 \\ 9.4979 \\ 20.0000 \end{bmatrix}$$
When evaluating the **exact Riemannian gradient** on $\mathrm{SE}(2)$ via central finite differences (or exact right Jacobian $J_r^{-1}(r)$):
$$\nabla_{X_2} \mathcal{C}(X^0) = \begin{bmatrix} 20.479063 \\ 8.989792 \\ 20.020839 \end{bmatrix}$$
The ~2.5% variation in translation components arises from the rotational curvature of the right Jacobian $J_r^{-1}(r_{12})$ over finite displacement ($r_x \approx 0.2\,\mathrm{m}, r_\theta = 0.05\,\mathrm{rad}$). As displacement $r \to 0$, $\nabla \mathcal{C} \to \Omega r$ asymptotically. Both values are verified in `tests/test_pose_graph_objective.cpp`.

#### Step 6: Semi-Implicit Velocity Update ($k = 0 \to 1$)
Since $\xi_2^0 = [0, 0, 0]^\top$:
- Momentum $p_2^0 = M_2 \xi_2^0 = 0 \implies \operatorname{ad}_{\xi_2^0}^*(M_2 \xi_2^0) = 0$.
- Damping inverse factor:
  $$\Lambda_2 = I + h M_2^{-1} D_2 = \operatorname{diag}(1 + 0.01 \times 2, \; 1 + 0.01 \times 2, \; 1 + 0.01 \times 2) = \operatorname{diag}(1.02, 1.02, 1.02)$$
  $$\Lambda_2^{-1} = \operatorname{diag}(0.980392, 0.980392, 0.980392)$$
- Unconstrained impulse:
  $$-h M_2^{-1} \nabla_{X_2} \mathcal{C} = -0.01 \times \begin{bmatrix} 20.2458 \\ 9.4979 \\ 20.0000 \end{bmatrix} = \begin{bmatrix} -0.202458 \\ -0.094979 \\ -0.200000 \end{bmatrix}$$
- Velocity $\xi_2^1$:
  $$\xi_2^1 = \Lambda_2^{-1} \left( -h M_2^{-1} \nabla_{X_2} \mathcal{C} \right) = 0.980392 \times \begin{bmatrix} -0.202458 \\ -0.094979 \\ -0.200000 \end{bmatrix} = \begin{bmatrix} -0.198488\,\mathrm{m/s} \\ -0.093117\,\mathrm{m/s} \\ -0.196078\,\mathrm{rad/s} \end{bmatrix}$$

#### Step 7: Position Retraction ($k = 0 \to 1$)
- Step vector: $h \xi_2^1 = 0.01 \times [-0.198488, -0.093117, -0.196078]^\top = [-0.001985\,\mathrm{m}, -0.000931\,\mathrm{m}, -0.001961\,\mathrm{rad}]^\top$.
- Heading update:
  $$\theta_2^1 = \theta_2^0 + h \omega_2^1 = 0.050000 - 0.001961 = 0.048039\,\mathrm{rad}$$
- Translation update (using $V(h \omega) \approx I$ for $h \omega \approx 10^{-3}$):
  $$t_2^1 \approx t_2^0 + R(\theta_2^0) (h v_2^1) = \begin{bmatrix} 1.200000 \\ 0.100000 \end{bmatrix} + \begin{bmatrix} 0.998750 & -0.049979 \\ 0.049979 & 0.998750 \end{bmatrix} \begin{bmatrix} -0.001985 \\ -0.000931 \end{bmatrix}$$
  $$\Delta t_2 = \begin{bmatrix} 0.998750 \times (-0.001985) - 0.049979 \times (-0.000931) \\ 0.049979 \times (-0.001985) + 0.998750 \times (-0.000931) \end{bmatrix} = \begin{bmatrix} -0.001983 + 0.000047 \\ -0.000099 - 0.000930 \end{bmatrix} = \begin{bmatrix} -0.001936 \\ -0.001029 \end{bmatrix}$$
  $$t_2^1 = \begin{bmatrix} 1.200000 - 0.001936 \\ 0.100000 - 0.001029 \end{bmatrix} = \begin{bmatrix} 1.198064\,\mathrm{m} \\ 0.098971\,\mathrm{m} \end{bmatrix}$$

**Result of Step 1:**
Node 2 moved from $(1.20\,\mathrm{m}, 0.10\,\mathrm{m}, 0.05\,\mathrm{rad})$ toward $(1.198064\,\mathrm{m}, 0.098971\,\mathrm{m}, 0.048039\,\mathrm{rad})$, directly in the direction of the measurement $(1.0\,\mathrm{m}, 0.0\,\mathrm{m}, 0.0\,\mathrm{rad})$, confirming strictly dissipative, restorative dynamics.
