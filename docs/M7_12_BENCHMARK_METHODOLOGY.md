# Milestone 7.12 — Benchmark Methodology & Protocol Evaluation

## 1. Experimental Design & Objectives
The primary scientific objective of Milestone 7.12 is to quantify the performance, resilience, and communication efficiency of decentralized topological SLAM under severe, prolonged network partitions and channel impairments.

Key evaluation metrics:
1. **Wire Transmission Footprint:** Total bytes transmitted across the network during partition recovery.
2. **Bandwidth Reduction Percentage:** Percentage reduction in transmitted wire bytes relative to baseline full-map exchange.
3. **Convergence Speed & Rounds:** Number of reconciliation rounds and wall-clock time required to reach complete swarm consistency.
4. **Swarm Convergence Integrity:** Verification that Version Vectors are equal, DSU components equal 1, and pending synchronization backlog is 0.
5. **Absolute Trajectory Error (ATE):** Root Mean Square Error (RMSE) of estimated keyframe poses relative to ground truth before and after loop closure reconciliation.

---

## 2. Factorial Experimental Matrix (45 Trials)
The benchmark matrix evaluates a 5 x 3 x 3 factorial design across:
- **Topologies (5):**
  - `2_Rover_Direct`: Direct point-to-point link.
  - `3_Rover_Linear`: Linear topology with intermediate relay node ($R_1 \leftrightarrow R_2 \leftrightarrow R_3$).
  - `4_Rover_Diamond`: Dual-path diamond relay ($R_1 \leftrightarrow \{R_2, R_3\} \leftrightarrow R_4$).
  - `4_Rover_Star`: Central hub topology with peripheral rovers.
  - `4_Rover_Disjoint_Islands`: Two isolated sub-swarms ($\{R_1, R_2\}$ and $\{R_3, R_4\}$) connected by a delayed cross-island bridge link.
- **Partition Durations (3):**
  - Short: 5.0 seconds
  - Medium: 15.0 seconds
  - Long: 30.0 seconds
- **Channel Impairments (3):**
  - `Clean`: Zero packet loss, nominal delay.
  - `Burst_Loss_30pct`: Gilbert-Elliott 2-state Markov burst loss ($P_{loss} \approx 30\%$).
  - `Severe_Congestion_60pct`: Gilbert-Elliott 2-state Markov burst loss ($P_{loss} \approx 60\%$) with 25 ms Gaussian latency jitter.
- **Synchronization Strategies (4):**
  - `Baseline_Full_Map`: Full ledger exchange across all communicating pairs.
  - `Single_Round_IBLT`: Single-round IBLT set reconciliation without causal version filtering.
  - `AstroMeshX_MultiRound_IBLT`: Causal Version Vector filtering with multi-round prefix chunking.
  - `AstroMeshX_MultiRound_IBLT_Adaptive`: Causal Multi-Round IBLT coupled with dynamic Adaptive Communication rate/batch regulation.

---

## 3. Empirical Results Analysis

### 3.1 Bandwidth Efficiency
- **Baseline Full-Map:** In 4-rover diamond trials with 30s partition duration, full-map exchange consumed up to 1,948,536 Bytes.
- **AstroMesh-X Multi-Round IBLT:** Under the exact same topology and duration, AstroMesh-X required only 356,456 Bytes—achieving an **81.7% reduction** in network transmission.
- **Mean Bandwidth Reduction:** Averaged across all 45 trials, AstroMesh-X achieved **48.2% to 59.3% bandwidth savings** over naive full-map transfer.

### 3.2 Recovery & Convergence
- **Fast Check Efficiency:** When rovers are already synchronized, Version Vector exchange completes in 1 round with only 264 Bytes overhead.
- **Peeling Scalability:** Under high divergence ($|A \Delta B| > 30$ items), single-round IBLT peeling stalled; multi-round prefix chunking successfully decomposed the symmetric difference into 4 chunks, achieving 100% decode completion.
- **Swarm Unity:** All 45 trials successfully achieved $C_{\text{DSU}} = 1$ and reciprocal edge fraction $1.000$.

### 3.3 Trajectory Accuracy
- Trajectory drift accumulated at a rate of approximately 0.03 m/s during open-loop partition exploration.
- Inter-rover loop closures verified upon reconnection reduced ATE RMSE by an average of **82.00%**, restoring metric consistency across the global swarm coordinate frame.

---

## 4. Reproducibility
- **Binary:** `build-ucrt64-clean/bin/benchmark_m712_partition_recovery.exe`
- **Output:** `benchmark_m712_partition_recovery.csv`
- **Seed Schedule:** $Seed(trial) = 10007 \times trial\_id + 42$
- **RNG:** Standard 64-bit Mersenne Twister (`std::mt19937_64`)

