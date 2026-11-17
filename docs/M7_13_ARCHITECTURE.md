# Milestone 7.13 Architecture: Decentralized Gauge Fixing & Globally Consistent Metric SLAM

## 1. Problem Formulation & Theoretical Foundations

### 1.1 The Gauge Freedom Problem in Multi-Rover SLAM
In topological and metric pose-graph SLAM, relative pose constraints between keyframes $i$ and $j$ are formulated as:
$$r_{ij} = \log\left( \bar{T}_{ij}^{-1} \cdot T_i^{-1} \cdot T_j \right) \in \mathfrak{se}(2)$$
The global cost objective is given by:
$$F(T_1, \dots, T_n) = \frac{1}{2} \sum_{(i,j) \in \mathcal{E}} r_{ij}^T \Omega_{ij} r_{ij}$$

Because relative errors depend strictly on differences $T_i^{-1} T_j$, the cost is invariant under any global rigid body action $G \in \mathrm{SE}(2)$:
$$F(G \cdot T_1, \dots, G \cdot T_n) = F(T_1, \dots, T_n), \quad \forall G \in \mathrm{SE}(2)$$
For planar $\mathrm{SE}(2)$ operations, this introduces 3 degrees of gauge freedom:
- Global Translation in $X$ ($\Delta x$)
- Global Translation in $Y$ ($\Delta y$)
- Global Yaw Orientation ($\Delta \theta$)

When $K$ rovers operate in isolated network partitions, their graphs have $3 \times K$ unconstrained gauge freedoms. Merely uniting them in a Disjoint Set Union (DSU) or exchanging topological sets does NOT resolve metric coordinate alignment. Each rover optimizes its local trajectory relative to its arbitrary starting frame $T_{\text{local}, k}(0) = I$.

---

### 1.2 Mathematical Formulation of Inter-Subgraph Alignment
Let rover $R_A$ have keyframes and observed landmarks in local frame $\mathcal{F}_A$, and rover $R_B$ have keyframes and observed landmarks in local frame $\mathcal{F}_B$.
When $R_A$ and $R_B$ observe a set of mutual physical landmarks $\{\mathcal{L}_k\}$, their 2D coordinates satisfy:
$$p_{A, k} = T_{AB} \cdot p_{B, k} + \epsilon_k$$
where $T_{AB} = \begin{bmatrix} R(\theta_{AB}) & t_{AB} \\ 0 & 1 \end{bmatrix} \in \mathrm{SE}(2)$.

#### Closed-Form 2D Kabsch/Umeyama Alignment:
Given $N \ge 3$ verified correspondences $(p_{A, k}, p_{B, k})$:
1. Centroids:
   $$\bar{p}_A = \frac{1}{N} \sum_{k=1}^N p_{A, k}, \quad \bar{p}_B = \frac{1}{N} \sum_{k=1}^N p_{B, k}$$
2. Centered coordinates:
   $$q_{A, k} = p_{A, k} - \bar{p}_A, \quad q_{B, k} = p_{B, k} - \bar{p}_B$$
3. Cross-covariance matrix $H \in \mathbb{R}^{2 \times 2}$:
   $$H = \sum_{k=1}^N q_{A, k} q_{B, k}^T = \begin{bmatrix} S_{xx} & S_{xy} \\ S_{yx} & S_{yy} \end{bmatrix}$$
4. Optimal planar rotation angle $\theta_{AB}$:
   $$\theta_{AB} = \text{atan2}(S_{yx} - S_{xy}, S_{xx} + S_{yy})$$
5. Optimal translation $t_{AB}$:
   $$t_{AB} = \bar{p}_A - R(\theta_{AB}) \cdot \bar{p}_B$$
6. Alignment Residual RMSE:
   $$\text{RMSE}_{AB} = \sqrt{\frac{1}{N} \sum_{k=1}^N \| p_{A, k} - (R(\theta_{AB}) p_{B, k} + t_{AB}) \|^2}$$

#### Geometric Degeneracy Pre-Checks:
An alignment estimate is rejected if:
- $N < 3$ non-collinear correspondences.
- Spatial variance $\text{Var}(q_A) < \sigma^2_{\min}$ (coincident or clustered points).
- Collinearity condition: $\lambda_{\min}(H H^T) / \lambda_{\max}(H H^T) < 10^{-4}$.

---

### 1.3 Alignment Graph and Cycle Consistency
The inter-subgraph alignment topology is represented as a directed graph $\mathcal{G}_{\text{align}} = (\mathcal{V}_{\text{rovers}}, \mathcal{E}_{\text{align}})$, where each edge $e_{ij} = (R_i, R_j)$ carries:
- Relative transform $T_{ij} \in \mathrm{SE}(2)$
- Residual error $\text{RMSE}_{ij}$
- Correspondence inlier count $N_{ij}$
- Version sequence and epoch

#### Cycle Consistency Audit:
Along any closed cycle $C = (R_1, R_2, \dots, R_m, R_1)$:
$$T_{\text{cycle}} = T_{12} \cdot T_{23} \cdots T_{m1}$$
In an ideal, error-free system:
$$T_{\text{cycle}} = I_{3 \times 3} \implies \log(T_{\text{cycle}}) = \mathbf{0}$$
Cycle consistency residual error:
$$e_{\text{cycle}} = \| \log(T_{\text{cycle}}) \|_{\mathfrak{se}(2)} = \sqrt{\delta x^2 + \delta y^2 + (\alpha \cdot \delta \theta)^2}$$
If $e_{\text{cycle}} > \tau_{\text{cycle}}$ (e.g. $0.35\,\text{m}$), the cycle contains an aliased or corrupted alignment edge. The edge with the highest individual RMSE in the cycle is flagged and invalidated.

---

### 1.4 Decentralized Anchor Election Protocol
To establish a global coordinate frame without a centralized coordinator:
1. **Deterministic Anchor Rule:** In every connected component of $\mathcal{G}_{\text{align}}$, the rover with the minimum active ID is elected as the gauge anchor:
   $$\text{Anchor}(C) = \arg\min_{R_i \in C} R_i$$
2. **Anchor Coordinate Frame:** The global frame $\mathcal{F}_{\text{global}}$ is defined to coincide with the local frame of the elected anchor:
   $$T_{\text{global}, \text{Anchor}} = I$$
3. **Transform Propagation via Maximum Confidence Spanning Tree:**
   For each rover $R_k \in C$, its global transform $T_{\text{global}, k}$ is resolved along the path from the anchor in the Maximum Confidence Spanning Tree (MST) of $\mathcal{G}_{\text{align}}$, where edge weight $w_{ij} = N_{ij} / (1.0 + \text{RMSE}_{ij})$.
4. **Idempotence & Monotonicity:** Each transform broadcast carries a version vector and epoch. Stale transform messages cannot revert an updated alignment.

---

### 1.5 Gauge-Correct Ground Truth ATE Evaluation
When comparing estimated poses against ground-truth poses:
$$\text{ATE} = \sqrt{\frac{1}{M} \sum_{i=1}^M \| p_{\text{est}, i} - p_{\text{gt}, i} \|^2}$$
If estimated poses are in an arbitrary global gauge frame $\mathcal{F}_{\text{global}}$ and ground truth is in $\mathcal{F}_{\text{gt}}$, direct subtraction produces meaningless error.
To evaluate metric accuracy without gauge bias:
- **Global Rigid Alignment:** A single optimal rigid transform $T_{\text{eval}} \in \mathrm{SE}(2)$ is computed between the entire estimated swarm map and ground truth using Umeyama alignment.
- **Evaluation Constraint:** The same single transform $T_{\text{eval}}$ is applied to ALL rovers across the entire swarm. Per-rover independent alignment during ATE computation is strictly prohibited.

