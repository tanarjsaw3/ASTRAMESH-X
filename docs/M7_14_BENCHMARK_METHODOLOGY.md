# AstroMesh-X Milestone 7.14 — Benchmark Methodology & Protocol
## Active Multi-Robot Loop Closure & Communication-Efficient Rendezvous

---

## 1. Experimental Objectives

The Milestone 7.14 benchmark evaluates the performance of active, decentralized loop closure candidate prioritization and communication-efficient rendezvous against traditional baselines in multi-robot topological SLAM. The evaluation addresses three core hypotheses:

1. **Information-Gain Quality Hypothesis:** An information-gain proxy combining graph drift distance, 2D landmark spatial dispersion ($\det S_{\text{geom}}$), and topological bridging yields higher SLAM accuracy per wire byte than counting raw landmark correspondences.
2. **Communication Efficiency Hypothesis:** Fair two-pass budget scheduling (spanning tree prior to redundant reinforcement) achieves global metric consistency (Global ATE $< 0.50$ m) while reducing wire traffic by $> 40\%$ compared to passive all-to-all descriptor broadcasts.
3. **Decentralized Safety & Precision Invariant:** Geometric verification with RANSAC-Kabsch rejection maintains 100% precision (0 false positive edge acceptances) under varying odometric and landmark observation noise.

---

## 2. Factorial Experimental Matrix

The benchmark consists of **45 factorial trial configurations** evaluating 4 coordination strategies across 5 swarm topologies, 3 communication budget tiers, and 3 sensor noise levels:

| Parameter | Levels / Values | Description |
|---|---|---|
| **Topologies** | `2_Rover_Direct`<br>`3_Rover_Linear`<br>`4_Rover_Diamond`<br>`8_Rover_Partitioned`<br>`16_Rover_Large_Swarm` | Swarm scales from pair to linear chain, diamond mesh, split partitioned clusters, and large 16-rover 2D swarm. |
| **Swarm Size ($N$)** | 2, 3, 4, 8, 16 | Number of autonomous physical rovers. |
| **Communication Budget** | 512 B, 1024 B, 2048 B per peer | Strict wire budget per peer interface. Swarm budget scales as $B_{\text{swarm}} = B \times (N-1)$. |
| **Noise Std Dev ($\sigma$)** | 0.01 m, 0.05 m, 0.15 m | Zero-mean Gaussian perturbation applied to local landmark bearing/range observations. |
| **Strategies** | 1. `A_Passive_All_To_All`<br>2. `B_Naive_Spatial_Exchange`<br>3. `C_Active_No_Info_Gain`<br>4. `D_AstroMesh_X_Active_Info_Aware` | 4 distinct scheduling regimes evaluated under identical initial conditions. |

---

## 3. Evaluated Strategies

### Strategy A: Passive All-To-All
- Blind peer-to-peer descriptor broadcast. Rovers transmit uncompressed landmark keyframe descriptors to all visible peers.
- Disregards wire budgets. All identified candidate opportunities are scheduled.
- Serves as the upper-bound communication overhead baseline.

### Strategy B: Naive Spatial Exchange
- Schedules candidate opportunities within spatial visibility radius, capped to a batch count limit.
- No prioritization based on information gain or correspondence quality.
- Fixed rendezvous packet wire expenditure.

### Strategy C: Active No-Info-Gain
- Budget-constrained scheduling.
- Candidates are sorted strictly by raw correspondence count ($N_{\text{corrs}}$) without considering spatial dispersion, graph drift, or network channel health.
- Employs two-pass spanning tree selection to connect the swarm under the wire budget.

### Strategy D: AstroMesh-X Active Info-Aware
- Full implementation of `InformationGainEvaluator` and `ActiveLoopClosureScheduler`.
- Scores candidates via multi-attribute utility:
  $$\text{Score} = \frac{w_I I_{\text{proxy}} + w_Q Q_{\text{geom}} + w_P P_{\text{verif}} + w_L L_{\text{net}}}{1 + w_C C_{\text{wire}} + w_R R_{\text{dup}}}$$
- Enforces two-pass DSU spanning tree budget allocation.
- Executes bounded three-way rendezvous protocol with RANSAC-Kabsch geometric verification and inter-subgraph alignment graph cycle remediation.

---

## 4. Evaluated Metrics

1. **`Candidate_Opportunities`:** Number of pairwise keyframe overlap regions detected with $\ge 3$ mutual landmarks.
2. **`Scheduled_Opportunities`:** Number of opportunities selected for exchange within the wire budget.
3. **`Verified_Closures`:** Number of scheduled opportunities passing RANSAC-Kabsch geometric verification ($\ge 3$ inliers, $\text{RMSE} \le 0.15$ m).
4. **`Rejected_Closures`:** Number of scheduled opportunities rejected by geometric verification.
5. **`True_Positives` & `False_Positives`:** Ground-truth classification of verified edges.
6. **`Precision_Pct`:** $\frac{TP}{TP + FP} \times 100\%$. Evaluated against zero-tolerance false-acceptance standard.
7. **`Global_ATE_m`:** Absolute Trajectory Error (Root Mean Squared Error) of all estimated rover keyframe poses aligned into the global frame via minimum-ID anchor gauge coordinates.
8. **`Total_Wire_Bytes`:** Sum of all serialized network bytes transmitted across the swarm during the trial.
9. **`Bytes_Per_Verified_Closure`:** $\frac{\text{Total\_Wire\_Bytes}}{\text{Verified\_Closures}}$. Quantifies communication cost efficiency.
10. **`Execution_Time_ms`:** Wall-clock duration of opportunity evaluation, scheduling, rendezvous verification, and gauge alignment.
11. **`Status`:** `PASS` if $\text{Global\_ATE\_m} < 0.50$ m; `FAIL` otherwise.

---

## 5. Zero Ground-Truth Leakage Guarantee

All candidate detection, information-gain scoring, link estimation, and scheduling decisions are computed strictly from local landmark estimates and decentralized wire messages. Ground-truth poses and true landmark coordinates are used solely by the evaluation harness `compute_global_swarm_ate()` to assess post-optimization metric accuracy.

---

## 6. Reproducibility Instructions

### Build Command:
```powershell
$env:Path = "C:\msys64\ucrt64\bin;C:\msys64\usr\bin;" + $env:Path
cmake --build build-ucrt64-clean --target benchmark_m714_active_loop_closure
```

### Execution Command:
```powershell
.\build-ucrt64-clean\bin\benchmark_m714_active_loop_closure.exe
```

Output is streamed live to stdout and written to `benchmark_m714_active_loop_closure.csv`.

