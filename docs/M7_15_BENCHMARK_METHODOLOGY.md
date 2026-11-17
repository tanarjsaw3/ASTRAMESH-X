# AstroMesh-X — Milestone 7.15 Benchmark Methodology & Experimental Protocol

## Dynamic Map Compaction & Memory-Bounded Distributed SLAM

**Status:** Verified & Frozen  
**Date:** 2026-11-14  
**Authors:** AstroMesh-X Systems & SLAM Team  
**Dataset Artifact:** `benchmark_m715_memory_bounded_slam.csv` (45 trials, 100.0% completion)

---

## 1. Objective & Scientific Hypothesis

Milestone 7.15 investigates whether autonomous multi-rover swarms can bound their memory consumption during extended exploration while preserving metric accuracy (Global ATE $< 0.50\,\text{m}$), 100% loop closure precision, and topological graph connectivity under delay-tolerant mesh conditions.

### Hypotheses:
1. **$H_1$ (Memory Boundedness):** Conservative degree-2 topological compounding combined with bounded landmark lifecycle management achieves $> 30\%$ memory reduction under Warning pressure and $> 50\%$ reduction under Critical pressure compared to unbounded map retention.
2. **$H_2$ (Metric Consistency):** Compounding non-branching degree-2 odometry nodes between verified loop closures preserves relative kinematic constraints without increasing Global ATE by more than $0.05\,\text{m}$ relative to uncompacted graphs.
3. **$H_3$ (Topological Integrity):** Indispensable node protection guarantees that DSU connected components, loop closure endpoints, exploration frontiers, and gauge anchors remain intact, resulting in zero false positive reconciliations and zero disconnected subgraphs.
4. **$H_4$ (Wire Overhead):** Distributed compaction notifications and acknowledgments require $\le 128$ bytes on the wire, maintaining low mesh overhead without flooding bandwidth-constrained links.

---

## 2. Experimental Matrix & Evaluation Design

The benchmark evaluates 45 distinct trials across a 3-dimensional parametric sweep:

### 2.1 Independent Variables
1. **Swarm Size ($N \in \{2, 4, 8, 16\}$ rovers):**
   - 2 Rovers: 9 trials (baseline pairwise interaction)
   - 4 Rovers: 12 trials (mesh diamond and linear topologies)
   - 8 Rovers: 12 trials (scaled distributed mesh with partitions)
   - 16 Rovers: 12 trials (large-scale stress test)
2. **Trajectory Length ($K \in \{20, 40, 80, 120\}$ steps per rover):**
   - Evaluates short, medium, long, and continuous mission durations.
3. **Memory Management Strategy:**
   - **Strategy A (`UnboundedBaseline`):** Retains all keyframes, odometry edges, and landmark observations unconditionally.
   - **Strategy B (`LandmarkEvictionOnly`):** Evicts low-confidence singleton landmarks and limits maximum observations per landmark, but retains all pose graph nodes.
   - **Strategy C (`LinearCompactionOnly`):** Compounds degree-2 intermediate odometry nodes, but retains all landmark observations.
   - **Strategy D (`FullMemoryBoundedSLAM`):** Jointly activates hysteresis budget controller, topological degree-2 compaction, bounded landmark lifecycle, and distributed compaction notifications.

---

## 3. Strict Metrics & Measurement Protocols

### 3.1 Memory Distinctions
- **Logical Data Size ($M_{\text{logical}}$):** Exact sum of bytes consumed by active structs ($\text{sizeof}(T) \times N_{\text{elements}}$).
- **Container Capacity Size ($M_{\text{capacity}}$):** Sum of vector capacity and hash map bucket overhead ($\text{capacity} \times \text{sizeof}(T) + \text{bucket\_overhead}$).
- **Serialized Wire Size ($M_{\text{wire}}$):** Actual packed byte length of network payloads (`CompactionNotification` = 68 B, `CompactionAck` = 14 B).
- **Process RSS ($M_{\text{RSS}}$):** Physical Resident Set Size queried directly from the operating system kernel via Windows `GetProcessMemoryInfo` (`WorkingSetSize`). Estimated logical memory is never reported as measured RSS.

### 3.2 Accuracy & Integrity Metrics
- **Global Swarm ATE ($e_{\text{ATE}}$):** Root Mean Square Error evaluated over all swarm keyframes against ground truth using a **single global SE(2) rigid alignment** for the entire swarm. Per-rover alignment during evaluation is strictly prohibited.
- **Topological Precision:** Ratio of true positive loop closures to total accepted closures:
  $$\text{Precision} = \frac{TP}{TP + FP} = 1.000 \quad (100.0\%)$$
- **Connected Components ($C_{\text{DSU}}$):** Verifies that compaction never fragments connected topological partitions.

---

## 4. Benchmark Harness Execution

- **Source:** `tests/benchmark_m715_memory_bounded_slam.cpp`
- **Binary:** `build-ucrt64-clean/bin/benchmark_m715_memory_bounded_slam.exe`
- **Output:** `benchmark_m715_memory_bounded_slam.csv` (45 rows + header, 14 columns)
- **Execution Command:**
  ```powershell
  $env:Path = "C:\msys64\ucrt64\bin;C:\msys64\usr\bin;" + $env:Path
  ./build-ucrt64-clean/bin/benchmark_m715_memory_bounded_slam.exe
  ```

---

## 5. Statistical Summary of Benchmark Results

| Metric | Minimum | Mean | Maximum | Target / Threshold | Status |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Trial Pass Rate** | 100% | 100.0% | 100% | 100% | **PASS** |
| **Global ATE (m)** | 0.0000 | 0.0136 | 0.0468 | $< 0.50\,\text{m}$ | **PASS** |
| **Loop Closure Precision** | 100.0% | 100.0% | 100.0% | 100.0% | **PASS** |
| **False Positive Closures** | 0 | 0 | 0 | 0 | **PASS** |
| **Memory Reduction (Full Strategy D)** | 34.7% | 58.2% | 78.5% | $> 30.0\%$ | **PASS** |
| **Compaction Notification Wire Size** | 68 B | 68 B | 68 B | $\le 128\,\text{B}$ | **PASS** |
| **Compaction Ack Wire Size** | 14 B | 14 B | 14 B | $\le 128\,\text{B}$ | **PASS** |
| **Process RSS (Measured)** | 4.88 MB | 5.12 MB | 5.35 MB | Kernel Query | **PASS** |
| **DSU Components Preservation** | Exact | Exact | Exact | Zero fragmentation | **PASS** |

