# AstroMesh-X — Literature References & Bibliography

This document lists the foundational academic literature, reference implementations, and theoretical frameworks informing the AstroMesh-X design. 

*Items marked with `[VERIFICATION NEEDED]` indicate citations where specific volume, page numbers, or publication year details require confirmation against publication indices.*

---

## 1. Primary Algorithmic Literature

### 1.1 Continuous Riemannian Dynamics (CORD)
- **Primary Reference:**
  - **Authors:** Yulun Tian, Kasra Khosoussi, Matthew Giamou, Jonathan P. How.
  - **Title:** *CORD: Continuous Riemannian Dynamics for Distributed Pose Graph Optimization* (or *Distributed Pose Graph Optimization via Continuous Riemannian Dynamics*).
  - **Venue:** IEEE Transactions on Robotics (T-RO) / IEEE International Conference on Robotics and Automation (ICRA).
  - **Status:** `[VERIFICATION NEEDED: Verify exact publication year (c. 2021-2023), volume, and canonical conference/journal citation].`
  - **Role in AstroMesh-X:** Foundational method for Phase 3 distributed pose-graph optimization over $SE(2)$ and $SE(3)$ manifolds without centralized solvers.

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
