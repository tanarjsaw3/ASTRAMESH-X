# AstroMesh-X — Scientific Evidence Matrix & Claims Inventory

**Document Version:** 1.0.0  
**Verification Date:** 2026-11-14  
**Audit Milestone:** Milestone 7.17  
**Scope:** Rigorous Classification of Scientific Claims, Evidence Basis, and Limitations

---

## 1. Evidence Classification Taxonomy

To ensure uncompromising scientific integrity, every claim in the AstroMesh-X repository is categorized into one of five rigorous classifications:

- **`[MEASURED]`**: Directly observed through physical execution on the target toolchain (e.g. wall-clock CPU time, socket wire bytes, Windows kernel RSS working set, CTest outcomes).
- **`[DERIVED]`**: Mathematically calculated from measured quantities via documented formulas (e.g. ATE RMSE, Cohen's $d$, bootstrap confidence intervals, percentage reductions).
- **`[SYNTHETIC]`**: Empirically measured within the deterministic 2D/multi-agent simulation testbed under modeled sensor noise and packet drop channels. Validates algorithmic properties, but does not represent real-world robotics hardware.
- **`[THEORETICAL]`**: Established through analytical mathematical proofs or asymptotic computational complexity (e.g. IBLT $O(d)$ peeling bounds, Lie group exponential retraction accuracy, DSU $O(\alpha(n))$ time).
- **`[QUALIFIED]`**: Empirical claim subject to documented boundary conditions or limitations (e.g. zero false positives on synthetic test sets).
- **`[UNVERIFIED]`**: Claim relates to physical deployment, real lunar regolith, or hardware-in-the-loop (HIL) environments not present in this software-only repository.

---

## 2. Core Scientific Claims Inventory

| Claim ID | Claim Statement | Evidence Class | Implementation Source | Operational Scope & Boundary Limitations | Verification Command |
| :---: | :--- | :---: | :--- | :--- | :--- |
| **C01** | **$\mathrm{SE}(2)$ Lie Group Exactness**<br>Group composition, inverse, adjoints, and closed-form exp/log maps are numerically exact with small-angle Taylor expansions. | `[THEORETICAL]` | `include/astromesh/geometry/se2.hpp`<br>`src/geometry/se2.cpp` | Planar $\mathrm{SE}(2)$ rigid body kinematics. Third-order Taylor series used when $|\theta| < 10^{-6}\,\text{rad}$. Does not extend to 3D $\mathrm{SE}(3)$ rotations. | `ninja test_se2_lie_math` |
| **C02** | **Continuous Riemannian Dynamics (CORD) Stability**<br>Semi-implicit Lie group integrator guarantees energy dissipation $\dot{\mathcal{H}} \le 0$ under positive damping. | `[DERIVED]` | `src/optimization/cord_integrator.cpp`<br>`docs/CORD_MATHEMATICAL_FORMULATION.md` | Damped Euler-Poincaré equations on $\mathrm{SE}(2)$. Proved Lyapunov dissipation. Constant-velocity neighbor extrapolation causes anti-damping under latency $> 20\,\text{ms}$ on closed loops. | `ninja test_cord_integrator` |
| **C03** | **IBLT Set Reconciliation Bounded in 1 KB**<br>Invertible Bloom Lookup Table peels symmetric differences $|A \Delta B| \le 26$ within a 1024-byte payload. | `[MEASURED]` | `include/astromesh/core/iblt.hpp`<br>`tests/benchmark_cbf_iblt.cpp` | 32 cells $\times$ 32 bytes = 1024 bytes. Peeling threshold $k \le 0.78M$. Differences $> 26$ fail to peel and require multi-round chunking. | `./build-ucrt64-clean/bin/benchmark_cbf_iblt.exe` |
| **C04** | **Multi-Hop DTN Hop Cost Scaling $O(h)$**<br>Multi-hop packet forwarding across intermediate relays scales total network wire burden proportionally to hop count. | `[MEASURED]` | `src/net/multihop_router.cpp`<br>`include/astromesh/net/multihop_envelope.hpp` | Maximum hops capped at 8. Transit queue is in-memory volatile (not crash-durable). Loop prevention via 8-element visited path trail. | `ninja test_m717_network_accounting` |
| **C05** | **Causal Version Vectors for Partition Recovery**<br>16-rover 64-bit monotonic vector clocks and progressive prefix chunking guarantee reconciliation after partition healing. | `[SYNTHETIC]` | `include/astromesh/core/version_vector.hpp`<br>`src/core/partition_reconciler.cpp` | Verified across 45 synthetic partition trials. Multi-round prefix chunking resolves differences $> 60$. Bandwidth savings: $38.6\%\dots59.3\%$ over full-map replication. | `./build-ucrt64-clean/bin/benchmark_m712_partition_recovery.exe` |
| **C06** | **Decentralized Gauge Fixing Sub-10cm Global ATE**<br>Closed-form 2D Kabsch/Umeyama alignment and minimum-ID anchor election resolve arbitrary initial reference frames. | `[SYNTHETIC]` | `include/astromesh/geometry/subgraph_aligner.hpp`<br>`src/net/gauge_coordinator.cpp` | Single global rigid $\mathrm{SE}(2)$ evaluation alignment. Requires $\ge 3$ non-collinear mutual landmark observations ($\operatorname{Var}(P) \ge 0.05\,\text{m}^2$) between overlapping subgraphs. | `./build-ucrt64-clean/bin/benchmark_m713_gauge_alignment.exe` |
| **C07** | **Zero False Positive Loop Closures ($FP = 0$)**<br>RANSAC consensus ($\ge 3$ inliers, $\le 0.35\,\text{m}$ residual) rejects false loop proposals on benchmark test sets. | `[QUALIFIED]` | `src/verification/geometric_verifier.cpp`<br>`tests/test_m717_loop_closure_statistics.cpp` | **Observed only on synthetic evaluation sets.** In real unstructured environments with repetitive visual terrain, perceptual aliasing can produce identical geometric constellations. Theoretical false positive rate is non-zero. | `ninja test_m717_loop_closure_statistics` |
| **C08** | **Active Loop Closure Communication Reduction $> 90\%$**<br>Two-pass DSU spanning budget allocation reduces network wire bytes by up to $95.7\%$ over passive mesh broadcast. | `[SYNTHETIC]` | `include/astromesh/coordination/active_loop_scheduler.hpp`<br>`src/coordination/active_loop_scheduler.cpp` | Evaluated across 2 to 16 rovers in synthetic benchmark. Information gain uses 2D spatial dispersion determinant $\det(S)$ and graph drift proxy rather than full Fisher information matrix inversion. | `./build-ucrt64-clean/bin/benchmark_m714_active_loop_closure.exe` |
| **C09** | **Conservative Map Compaction Zero Fill-In Invariant**<br>Compounding degree-2 serial odometry chains strictly decreases node and edge counts without generating fill-in edges. | `[THEORETICAL]` | `include/astromesh/memory/map_compactor.hpp`<br>`src/memory/map_compactor.cpp` | Closed-form Lie compounding $T_{ik} = T_{ij} \cdot T_{jk}$. Strictly protects loop closure endpoints, exploration frontiers, gauge anchors, and branching nodes (degree $\ne 2$). | `ninja test_dynamic_map_compaction` |
| **C10** | **Memory Footprint Reduction 34% to 78%**<br>Dynamic compaction and landmark lifecycle management bound memory consumption under mission scaling. | `[MEASURED]` | `include/astromesh/memory/memory_tracker.hpp`<br>`src/memory/memory_tracker.cpp` | Measured across 45 benchmark configurations. Reduces logical bytes and container capacity. Operating system working set RSS exhibits page freelist caching hysteresis. | `./build-ucrt64-clean/bin/benchmark_m715_memory_bounded_slam.exe` |
| **C11** | **Topological Radial Sector Exploration (Grid-Free)**<br>Projects radial sector frontiers from peripheral keyframes without allocating dense 2D/3D occupancy grids. | `[SYNTHETIC]` | `include/astromesh/exploration/frontier.hpp`<br>`src/exploration/frontier.cpp` | Tested in 2D planar simulated environments. Uses 44-byte packed `FrontierClaim` leases with commitment hysteresis ($\Delta_{\text{switch}} = 0.15$). Ray projections approximate continuous obstacle boundaries. | `./build-ucrt64-clean/bin/benchmark_m716_autonomous_exploration.exe` |
| **C12** | **Physical Lunar Surface Deployment & Flight Readiness**<br>Deployability on physical lunar rovers navigating south polar crater terrain. | `[UNVERIFIED]` | `docs/ROS2_GAZEBO_INTEGRATION_PLAN.md` | **Unverified.** AstroMesh-X is currently a 100% software-only C++17 simulation testbed. No hardware, physical rocker-bogie chassis, stereo cameras, lidar sensors, or regolith testing has been performed. | `N/A (Future Work)` |

---

## 3. Ground-Truth and ATE Audit Findings

### 3.1 Fraudulent Per-Rover Alignment vs Rigorous Global Alignment
An adversarial test was implemented in `tests/test_m717_ate_alignment.cpp` where two rovers experienced a deliberate internal coordinate divergence of $15.0\,\text{m}$ in Y:
- **Fraudulent Independent Per-Rover Alignment:** Aligned each rover's subgraph independently to ground truth. Result: $\text{ATE} = 9.74 \times 10^{-17}\,\text{m} \approx 0.0\,\text{m}$! The independent alignment absorbed the coordinate shift, fraudulently masking the massive swarm divergence.
- **Rigorous Single Global Rigid $\mathrm{SE}(2)$ Alignment:** Aligned the combined multi-rover map simultaneously via a single rigid transformation $T_{\text{eval}} \in \mathrm{SE}(2)$. Result: $\text{ATE} = 4.22\,\text{m}$. The error was correctly exposed.
- **Enforced Repository Invariant:** All benchmark evaluations must strictly utilize the single global rigid alignment method.

### 3.2 Information Isolation
Audit of `PoseGraph`, `SubgraphAligner`, `GaugeCoordinator`, and `TopologicalFrontierDetector` confirmed zero usage or reference to ground truth in production classes. Ground-truth coordinates are instantiated solely in simulation worlds and test fixtures for metric evaluation.

---

## 4. Communication & Memory Accounting Findings

1. **Wire Accounting Invariant:**
   Verified in `test_m717_network_accounting.cpp`:
   $$\text{total\_wire\_bytes} \equiv \text{data\_wire\_bytes} + \text{ack\_wire\_bytes}$$
   $$\text{data\_wire\_bytes} \equiv \text{payload\_bytes} + \text{header\_bytes} + \text{framing\_bytes}$$
2. **Volatile DTN Queues:**
   Transit queues in `MultihopRouter` are stored in memory (`std::deque<MultihopTransitFrame>`) and enforce a dual bound of 100 frames and 1 MB. Transit buffers are volatile and reset upon process termination.
3. **Physical RSS Working Set:**
   Queried via Windows kernel `GetProcessMemoryInfo` (`WorkingSetSize`). Demonstrated that while compaction removes logical nodes and frees heap allocations, process RSS may not immediately shrink due to C runtime virtual memory caching.

---

## 5. Scope of Validity for Publication

When preparing manuscripts or technical reports based on AstroMesh-X:
1. Clearly state that results are derived from a high-fidelity **software simulation testbed** operating over real network sockets (UDP loopback).
2. Characterize $FP = 0$ as an observed outcome on the synthetic dataset, accompanied by an analytical discussion of perceptual aliasing limits.
3. Report communication overhead as total wire bytes, including framing, custody ACKs, and multi-hop forwarding overhead.
4. Report ATE computed exclusively with a single global rigid $\mathrm{SE}(2)$ transformation.

