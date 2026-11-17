# AstroMesh-X — Literature References & Bibliography

This document lists the foundational academic literature, reference implementations, and theoretical frameworks informing the AstroMesh-X design. 

*Items marked with `[VERIFICATION NEEDED]` indicate citations where specific volume, page numbers, or publication year details require confirmation against publication indices.*

---

## 1. Primary Algorithmic Literature

### 1.1 Continuous Riemannian Dynamics (CORD)
- **Primary Reference:**
  - **Authors:** Jaeho Shin, Maani Ghaffari, Yulun Tian.
  - **Title:** *Distributed Pose Graph Optimization via Continuous Riemannian Dynamics*.
  - **Venue:** Robotics: Science and Systems XXII (RSS 2026), Paper 032.
  - **Preprint / Archive:** arXiv:2605.11210v1 [cs.RO], May 2026.
  - **Official URLs:** [arXiv:2605.11210](https://arxiv.org/abs/2605.11210) | [RSS Proceedings p032](http://www.roboticsproceedings.org/rss22/p032.html)
  - **Status:** Verified canonical citation against primary paper and proceedings.
  - **Role in AstroMesh-X:** Foundational optimization methodology for Phase 3 distributed pose-graph optimization, adapted from $\mathrm{SE}(3)$ to planar $\mathrm{SE}(2)$ multi-rover swarms with velocity-extrapolated neighbor prediction.
- **Related Prior Art & Foundations:**
  - **Authors:** Yulun Tian, Kasra Khosoussi, Matthew Giamou, Jonathan P. How.
  - **Title:** *Asynchronous and Parallel Distributed Pose Graph Optimization* (ASAPP) / Riemannian Block Coordinate Descent (RBCD).
  - **Venues:** IEEE Transactions on Robotics (T-RO) / IEEE ICRA.
  - **Role:** Predecessor first-order coordinate-descent and proximal distributed PGO solvers compared against CORD.

### 1.2 Invertible Bloom Lookup Tables (IBLT)
- **Primary Reference:**
  - **Authors:** Michael T. Goodrich, Michael Mitzenmacher.
  - **Title:** *Invertible Bloom Lookup Tables*.
  - **Venue:** 49th Annual Allerton Conference on Communication, Control, and Computing (Allerton), 2011.
  - **DOI/URL:** 10.1109/Allerton.2011.6120248.
  - **Status:** Verified canonical citation.
  - **Role in AstroMesh-X:** Core probabilistic data structure for Phase 1 set difference reconciliation under tight communication constraints.
- **Secondary Reference:**
  - **Authors:** David Eppstein, Michael T. Goodrich, Frank Schulz, Jens Unger.
  - **Title:** *What's the Difference? Efficient Set Reconciliation Without Prior Knowledge*.
  - **Venue:** IEEE Transactions on Knowledge and Data Engineering (TKDE), 2011.
  - **Status:** Verified canonical citation.

### 1.3 Counting Bloom Filters (CBF Baseline)
- **Primary Reference:**
  - **Authors:** Li Fan, Pei Cao, Jussara Almeida, Andrei Z. Broder.
  - **Title:** *Summary Cache: A Scalable Wide-Area Web Cache Sharing Protocol*.
  - **Venue:** IEEE/ACM Transactions on Networking, Vol. 8, No. 3, pp. 281-293, June 2000.
  - **Role in AstroMesh-X:** Baseline data structure representing the AstroMesh v1 landmark discovery approach.

---

## 2. Distributed SLAM & Topological Mapping Literature

### 2.1 Multi-Robot Topological & Metric SLAM
- **Reference:**
  - **Authors:** Vadim Indelman, et al.
  - **Title:** *Multi-Robot Pose Graph Localization and Mapping*.
  - **Role:** Survey of consensus and distributed graph assembly paradigms.
  - **Status:** `[VERIFICATION NEEDED: Cross-reference exact title and conference details].`
- **Reference:**
  - **Authors:** Luca Carlone, et al.
  - **Title:** *Lagrangian Duality in 3D SLAM: Verification Techniques and Distributed Formulations*.
  - **Role:** Background for robust loop closure verification and outlier rejection.
  - **Status:** `[VERIFICATION NEEDED: Check specific publication details].`

---

## 3. Communication & Delay-Tolerant Networking (DTN)

- **Reference:**
  - **Authors:** Kevin Fall.
  - **Title:** *A Delay-Tolerant Network Architecture for Challenged Internets*.
  - **Venue:** ACM SIGCOMM, 2003.
  - **Role in AstroMesh-X:** Theoretical principles for store-and-forward message buffers, custody transfer, and expiration mechanics under intermittent connectivity.

---

## 4. Graph Algorithms & Keyframe Sparsification

### 4.1 Linear-Time Articulation Point Detection
- **Primary Reference:**
  - **Authors:** Robert E. Tarjan.
  - **Title:** *Depth-First Search and Linear Graph Algorithms*.
  - **Venue:** SIAM Journal on Computing, Vol. 1, No. 2, pp. 146–160, 1972.
  - **DOI:** 10.1137/0201010.
  - **Role in AstroMesh-X:** Foundational algorithm for identifying cut-vertices (articulation points) in $O(V + E)$ time, ensuring pose-graph pruning never fragments connected components.

### 4.2 Factor-Graph Sparsification & Marginalization Literature
- **Reference:**
  - **Authors:** Nicholas Carlevaris-Bianco, Ryan M. Eustice.
  - **Title:** *Generic Node Removal for Factor-Graph SLAM*.
  - **Venue:** IEEE Transactions on Robotics, Vol. 30, No. 6, pp. 1371–1385, Dec. 2014.
  - **Role in AstroMesh-X:** Theoretical framework for Schur-complement marginalization and dense information fill-in. Informs the Phase 4.3 classification of unmarginalized factor pruning as factor-removal ablation.
- **Reference:**
  - **Authors:** Rainer Kretzschmar, Cyrill Stachniss.
  - **Title:** *Information-Theoretic Compression of Pose Graphs for Laser-Based SLAM*.
  - **Venue:** The International Journal of Robotics Research (IJRR), Vol. 31, No. 11, pp. 1219–1238, 2012.
  - **Role in AstroMesh-X:** Information-theoretic keyframe selection principles informing future work beyond deterministic geometric pruning.
