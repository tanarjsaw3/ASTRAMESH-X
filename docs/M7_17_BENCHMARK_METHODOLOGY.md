# AstroMesh-X — Milestone 7.17 Benchmark Methodology
## Empirical Verification Protocol & Statistical Evaluation Framework

**Status:** Approved  
**Date:** 2026-11-14  
**Milestone:** 7.17  

---

## 1. Experimental Design & Audit Objectives

Milestone 7.17 establishes a standardized statistical protocol for verifying all experimental metrics across the AstroMesh-X distributed SLAM pipeline.

The benchmark protocol addresses six core evaluation axes:
1. **Metric Rigor:** Evaluation of Absolute Trajectory Error (ATE RMSE) via single global rigid $\mathrm{SE}(2)$ alignment.
2. **Benchmark Fairness:** Paired experimental designs with identical pseudo-random number generator (PRNG) seeds, sensor noises, and mission timelines.
3. **Statistical Confidence:** Paired hypothesis testing, Cohen's $d$ effect sizes, and 2,000-resample non-parametric bootstrap confidence intervals.
4. **Loop-Closure Verification Boundaries:** Rigorous confusion matrix analysis ($TP, FP, TN, FN$), precision/recall, and quantification of adversarial perceptual aliasing limits.
5. **Physical Wire Byte Accounting:** Conservation of total network traffic ($\text{Total Wire} \equiv \text{Data Wire} + \text{ACK Wire}$), including hop-by-hop forwarding amplification.
6. **Physical vs Logical Memory Footprints:** Empirical measurement of logical data size, dynamic container capacity, and Windows kernel Process Working Set (RSS).

---

## 2. Statistical Formulations

### 2.1 Paired Sample Differences & Standardized Effect Size
Given $N$ paired benchmark runs where condition $A$ is the baseline algorithm and condition $B$ is AstroMesh-X:
$$\Delta_i = B_i - A_i, \quad i \in \{1, \dots, N\}$$
$$\bar{\Delta} = \frac{1}{N} \sum_{i=1}^N \Delta_i, \quad s_\Delta = \sqrt{\frac{1}{N-1} \sum_{i=1}^N (\Delta_i - \bar{\Delta})^2}$$

The standardized effect size for paired observations (Cohen's $d$) is:
$$d = \frac{\bar{\Delta}}{s_\Delta}$$

Interpretation scale:
- $|d| < 0.2$: Negligible effect.
- $0.2 \le |d| < 0.5$: Small effect.
- $0.5 \le |d| < 0.8$: Medium effect.
- $|d| \ge 0.8$: Large effect.
- $|d| \ge 1.2$: Very large / dominant effect.

### 2.2 Bootstrap 95% Confidence Intervals
For any metric $M$ (e.g. mean ATE reduction, bandwidth savings percentage):
1. Draw $B = 2,000$ bootstrap resamples $\mathcal{S}_b^* = \{ \Delta_1^*, \dots, \Delta_N^* \}$ with replacement from the observed sample.
2. Compute the statistic $\theta_b^* = f(\mathcal{S}_b^*)$.
3. Sort the bootstrap distribution: $\theta_{(1)}^* \le \dots \le \theta_{(B)}^*$.
4. Extract the empirical percentile interval:
   $$\text{CI}_{95\%} = \left[ \theta^*_{\lfloor 0.025 \cdot B \rfloor}, \; \theta^*_{\lceil 0.975 \cdot B \rceil} \right]$$

### 2.3 Non-Parametric Significance Testing
When comparing paired distributions that may deviate from normality (e.g. latency jitter, loss bursts, RANSAC inlier counts), we compute the two-sided **Wilcoxon signed-rank test**:
1. Rank the absolute differences $| \Delta_i |$ (ignoring $\Delta_i = 0$).
2. Compute rank sums $W^+ = \sum_{\Delta_i > 0} \operatorname{Rank}(|\Delta_i|)$ and $W^- = \sum_{\Delta_i < 0} \operatorname{Rank}(|\Delta_i|)$.
3. Test statistic $W = \min(W^+, W^-)$.
4. Compute asymptotic or exact two-tailed $p$-value. Differences are claimed statistically significant if and only if $p < 0.05$.

---

## 3. Ground-Truth and Metric Definitions

### 3.1 Unbiased Global ATE RMSE
Let the true keyframe poses for rover $r$ at index $k$ be $p_{r,k}^{\text{GT}} \in \mathbb{R}^2$ and estimated poses be $p_{r,k}^{\text{est}} \in \mathbb{R}^2$.
1. Aggregate all estimated and ground-truth keyframe pairs across all $M$ rovers into a single set:
   $$\mathcal{C} = \left\{ \left(p_{r,k}^{\text{est}}, p_{r,k}^{\text{GT}}\right) \mid r \in \{1, \dots, M\}, \; k \in \{1, \dots, K_r\} \right\}$$
2. Solve for the optimal global rigid transformation $T^* \in \mathrm{SE}(2)$ using closed-form 2D Umeyama alignment over the complete set $\mathcal{C}$.
3. Calculate the global RMSE:
   $$\text{ATE RMSE} = \sqrt{\frac{1}{|\mathcal{C}|} \sum_{(p^{\text{est}}, p^{\text{GT}}) \in \mathcal{C}} \| T^* \cdot p^{\text{est}} - p^{\text{GT}} \|^2}$$

### 3.2 Wire Conservation Invariant
For every trial, physical network transport must strictly satisfy:
$$\text{total\_wire\_bytes} \equiv \text{data\_wire\_bytes} + \text{ack\_wire\_bytes}$$
$$\text{data\_wire\_bytes} \equiv \text{payload\_bytes} + \text{header\_bytes} + \text{framing\_bytes}$$

### 3.3 Multi-Hop Relay Accounting
When a payload of size $L$ bytes is routed across $h$ intermediate hops, each physical transmission consumes wire bandwidth:
$$\text{Total Forwarding Wire} = \sum_{j=1}^h \left( \text{FrameHeader} + \text{Envelope} + L \right) + \sum_{j=1}^h \text{CustodyAck}$$

---

## 4. Benchmark Campaign Matrix

The M7.17 audit processes five factorial benchmark datasets covering M7.12 to M7.16:

1. **Partition Recovery Benchmark (`benchmark_m712_partition_recovery.csv`):**
   - 45 trials across 5 topologies (2-Rover Direct, 3-Rover Linear, 4-Rover Linear, 4-Rover Diamond, 4-Rover Broken Failover) and 3 partition regimes (Nominal, Moderate, Severe).
2. **Decentralized Gauge Fixing Benchmark (`benchmark_m713_gauge_alignment.csv`):**
   - 45 trials across 4 swarm sizes (2, 4, 8, 16 rovers) evaluating unaligned local subgraphs vs. centralized ground-truth anchor vs. AstroMesh-X decentralized gauge coordination.
3. **Active Loop-Closure Rendezvous Benchmark (`benchmark_m714_active_loop_closure.csv`):**
   - 45 trials comparing Passive Broadcast, Random Rendezvous, Naive Spatial Windowing, and AstroMesh-X Information-Gain Scheduling.
4. **Memory-Bounded SLAM Benchmark (`benchmark_m715_memory_bounded_slam.csv`):**
   - 45 trials comparing Unbounded Baseline, Landmark Eviction Only, Linear Compaction Only, and Full Memory-Bounded SLAM.
5. **Autonomous Exploration Benchmark (`benchmark_m716_autonomous_exploration.csv`):**
   - 45 trials comparing Random Walk, Greedy Frontier, Uncoordinated Information Gain, Coordinated Agnostic, and Full AstroMesh-X Autonomous Exploration.

