# AstroMesh-X — Phase 6.2 Robustness Study: Packet Loss & Delay Sensitivity

**Status:** Phase 6.2 Complete  
**Date:** 2026-10-29  
**Datasets:**  
- Primary: `benchmark_phase62_trials.csv` (SC1: 14,700 trials, 100% complete)  
- Exploratory: `benchmark_phase62_trials.csv` (SC10: 680 trials, 27.8% partial)  
- Summaries: `benchmark_phase62_summary.csv` (3,360 comparisons), `benchmark_phase62_pilot_summary.csv`  

---

## 1. Executive Summary & Scope Definition

Phase 6.2 investigates the robustness of the **AstroMesh-X** distributed SLAM communication architecture under severe network degradation: independent packet loss rates from $0\%$ to $90\%$ and one-way transmission delays from $0\,\text{ms}$ to $500\,\text{ms}$ ($RTT = 0$ to $1000\,\text{ms}$).

### Dataset Scope Distinction

> [!IMPORTANT]
> **Clear Separation of Study Datasets:**
> - **Primary Benchmark (`SC1_Nominal_2R`):** Fully complete across all 70 channel condition cells ($10\text{ loss levels} \times 7\text{ delay levels}$), evaluated with $N=30$ independent seeds and 7 architectural variants ($14,700$ total trials). All primary statistical conclusions, Bonferroni-corrected hypothesis tests, and confidence intervals are derived exclusively from this complete dataset.
> - **Exploratory Scaling Benchmark (`SC10_Swarm_8R`):** Incomplete exploratory study containing **680 of 2,450 trials** ($27.8\%$ completion across 20 of 70 cells). This run was halted cleanly to respect user computational constraints (~11.5 hours remaining). Results from SC10 are presented strictly as preliminary observational evidence of multi-hop forwarding overhead and must **not** be interpreted as a verified full 70-cell parameter sweep.

> [!WARNING]
> **Simulation Boundary & Non-Generalization Warning:**
> All trials were executed in a controlled 2D discrete-time simulation harness (`SyntheticWorld2D`, `SwarmTrajectoryGenerator`, `ObservationModel2D`). Link packet loss follows an uncorrelated Bernoulli model; delays are deterministic per hop; rover trajectories follow pre-planned geometric paths. **These simulated metrics must not be generalized to real-world planetary rovers**, where RF multi-path fading, burst packet loss, non-Gaussian transmission delays, time-varying network topologies, and 3D terrain roughness will introduce additional complex failure modes.

---

## 2. Experimental Design & Methodology

### 2.1 Channel Condition Grid (70 Cells)

The parameter grid tests the full cartesian product of 10 packet loss rates and 7 one-way delays:
- **Packet Loss Rates ($p_{\text{loss}}$):** $0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9$ ($0\%\dots90\%$)
- **One-Way Delays ($\tau$):** $0, 20, 50, 100, 200, 350, 500\,\text{ms}$ ($RTT = 2\tau \in [0, 1000\,\text{ms}]$)

### 2.2 Architectural Variants Evaluated

| Variant Name | IBLT Discovery | Network ACKs | DTN Retries | Multi-Hop | Description |
| :--- | :---: | :---: | :---: | :---: | :--- |
| `AstroMesh_X_Full` | Yes | Yes (44 B) | Yes (Exp Backoff) | Yes (Dijkstra) | Proposed complete architecture |
| `AstroMesh_X_RawExchange` | No (Raw Keys) | Yes (44 B) | Yes (Exp Backoff) | Yes (Dijkstra) | Uncompressed landmark exchange |
| `AstroMesh_X_NoACKs` | Yes | No | Partial (No RTO) | Yes (Dijkstra) | No explicit ACK frames |
| `AstroMesh_X_NoRetries` | Yes | Yes (44 B) | No | Yes (Dijkstra) | Single attempt per hop |
| `AstroMesh_X_NoMultiHop` | Yes | Yes (44 B) | Yes (Exp Backoff) | No (Direct only) | Single-hop direct line-of-sight |
| `AstroMesh_X_Minimal` | Yes | No | No | No | Bare minimum transport |
| `AstroMesh_v1_Baseline` | CBF Baseline | No | No | No | AstroMesh v1 reference baseline |

### 2.3 DTN Retransmission Protocol & In-Flight Arrival Gating

1. **Exponential Backoff:** Base timeout $\text{RTO}_0 = 50\,\text{ms}$. Cumulative retry schedule: Attempt 1 at $t=0$, Attempt 2 at $t+50\,\text{ms}$, Attempt 3 at $t+150\,\text{ms}$, Attempt 4 at $t+350\,\text{ms}$, Attempt 5 at $t+750\,\text{ms}$ (max 4 retries, 5 attempts total).
2. **Spurious Retransmission Trigger:** When round-trip time $RTT = 2\tau$ exceeds a retry timeout threshold before an ACK can return, a spurious duplicate retransmission is injected onto the wire.
3. **In-Flight Gating:** Packets in flight across the link when the mission duration expires ($t_{\text{mission}} = 8.0\,\text{s}$) are accounted as `in_flight_at_termination` and are gated from delivery.

### 2.4 Verifiable Invariant Auditing

Every row in the 15,380-trial dataset was audited via `scratch/audit_phase62_final.py`:
1. **Data Byte Conservation:** $\text{data\_wire\_bytes} = \text{payload\_bytes} + \text{protocol\_bytes} + \text{framing\_bytes}$ ($100\%$ valid, max error $0$).
2. **Total Byte Conservation:** $\text{total\_wire\_bytes} = \text{data\_wire\_bytes} + \text{ack\_wire\_bytes}$ ($100\%$ valid, max error $0$).
3. **Message Conservation:** $\text{logical\_messages\_sent} = \text{unique\_deliveries} + \text{logical\_messages\_dropped} + \text{in\_flight\_at\_termination}$ ($100\%$ valid across all 14,700 SC1 rows, max error $0$).
4. **Key Uniqueness & Status:** 0 duplicate keys; 0 missing keys; 14,700/14,700 'Success' status; 0 NaN/inf values.

---

## 3. SC1 Primary Benchmark Results (14,700 Trials, N=30)

### 3.1 Delivery Resilience under Packet Loss

Under zero delay ($\tau = 0\,\text{ms}$), application completion ratio demonstrates the massive resilience of DTN retries against severe packet loss:

| Loss Rate | Full ($N=30$) | NoRetries ($N=30$) | Minimal ($N=30$) | Baseline v1 ($N=30$) | Paired Diff (Full vs NoRetries) | $p_{\text{adj}}$ (Bonferroni) |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **0%** | $100.00\%$ | $100.00\%$ | $100.00\%$ | $100.00\%$ | $+0.000$ | $1.0000$ |
| **10%** | $99.67\%$ | $89.67\%$ | $89.67\%$ | $89.67\%$ | $+0.100$ | $1.22 \times 10^{-22}$ |
| **20%** | $99.59\%$ | $79.11\%$ | $79.11\%$ | $79.11\%$ | $+0.205$ | $3.45 \times 10^{-22}$ |
| **30%** | $98.21\%$ | $71.79\%$ | $71.79\%$ | $71.79\%$ | $+0.264$ | $5.12 \times 10^{-21}$ |
| **40%** | $97.48\%$ | $61.87\%$ | $61.87\%$ | $61.87\%$ | $+0.356$ | $8.74 \times 10^{-22}$ |
| **50%** | $94.47\%$ | $50.89\%$ | $50.89\%$ | $50.89\%$ | $+0.436$ | $1.15 \times 10^{-21}$ |
| **60%** | $90.41\%$ | $41.30\%$ | $41.30\%$ | $41.30\%$ | $+0.491$ | $4.01 \times 10^{-21}$ |
| **70%** | $82.60\%$ | $32.03\%$ | $32.03\%$ | $32.03\%$ | $+0.506$ | $2.32 \times 10^{-21}$ |
| **80%** | $64.23\%$ | $22.03\%$ | $22.03\%$ | $22.03\%$ | $+0.422$ | $1.08 \times 10^{-21}$ |
| **90%** | $39.59\%$ | $10.98\%$ | $10.98\%$ | $10.98\%$ | $+0.286$ | $3.11 \times 10^{-21}$ |

**Statistical Conclusion:** DTN exponential backoff delivers statistically significant delivery superiority ($p_{\text{adj}} < 10^{-20}$) across all non-zero loss rates. At $70\%$ loss, AstroMesh-X retains $>82.6\%$ message delivery where un-retried systems drop over two-thirds of all messages.

---

### 3.2 Retransmission Timeout (RTO) Step-Function Scaling

Under zero packet loss ($p_{\text{loss}} = 0.0$), wire byte consumption steps up as link latency increases, driven entirely by spurious retransmissions when $RTT = 2\tau > \text{RTO}_k$:

| Delay $\tau$ | $RTT = 2\tau$ | Active RTO Threshold | Spurious Retries / Msg | Mean Total Wire Bytes | Multiplier vs Baseline |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **0 ms** | $0\,\text{ms}$ | None ($RTT < 50$) | $0.0$ | $45,428\,\text{B}$ | $1.0\times$ |
| **20 ms** | $40\,\text{ms}$ | None ($40 < 50$) | $0.0$ | $45,428\,\text{B}$ | $1.0\times$ |
| **50 ms** | $100\,\text{ms}$ | Retry 1 ($100 > 50$) | $1.0$ (41 retries) | $90,856\,\text{B}$ | $2.0\times$ |
| **100 ms** | $200\,\text{ms}$ | Retry 2 ($200 > 150$) | $2.0$ (82 retries) | $136,284\,\text{B}$ | $3.0\times$ |
| **200 ms** | $400\,\text{ms}$ | Retry 3 ($400 > 350$) | $3.0$ (123 retries) | $181,712\,\text{B}$ | $4.0\times$ |
| **350 ms** | $700\,\text{ms}$ | Retry 3 ($700 < 750$) | $3.0$ (123 retries) | $181,712\,\text{B}$ | $4.0\times$ |
| **500 ms** | $1000\,\text{ms}$ | Retry 4 ($1000 > 750$) | $4.0$ (164 retries) | $227,140\,\text{B}$ | $5.0\times$ (Cap hit) |

```
Wire Bytes vs Delay (SC1 Nominal, Loss = 0%)
  250 kB |                                                  * (227.1 kB, 5x)
  200 kB |                               * (181.7 kB)       * (181.7 kB, 4x)
  150 kB |                * (136.3 kB, 3x)
  100 kB |  * (90.9 kB, 2x)
   50 kB |  * (45.4 kB)   * (45.4 kB)
    0 kB +------------------------------------------------------------------
           0 ms           20 ms          50 ms        100 ms       200 ms       350 ms       500 ms
```

**Key Architectural Insight:** Fixed retransmission timers in high-latency environments produce severe bandwidth inflation. Because $RTT$ exceeds static RTO thresholds, the sender retransmits perfectly healthy messages before the ACK can traverse the reverse link.

---

### 3.3 Trajectory Accuracy (ATE RMSE) & Graph Connectivity

| Metric | Nominal (0%, 0ms) | Mod. Stress (30%, 100ms) | Severe Loss (70%, 0ms) | High Delay (0%, 500ms) | Extreme (80%, 350ms) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Full ATE RMSE (m)** | $10.8833 \pm 0.000$ | $10.8637 \pm 0.082$ | $10.8803 \pm 0.076$ | $10.8833 \pm 0.000$ | $10.8981 \pm 0.089$ |
| **Baseline v1 ATE (m)** | $10.8833 \pm 0.000$ | $10.8944 \pm 0.000$ | $10.8944 \pm 0.000$ | $10.8833 \pm 0.000$ | $10.8944 \pm 0.000$ |
| **Paired Diff (ATE)** | $+0.0000$ | $+0.0307$ | $+0.0141$ | $+0.0000$ | $-0.0037$ |
| **$p_{\text{adj}}$ (Bonferroni)** | $1.0000$ | $0.2009$ | $1.0000$ | $1.0000$ | $1.0000$ |
| **DSU Components (Full)** | $2.0$ | $2.0$ | $2.0$ | $2.0$ | $2.0$ |
| **DSU Components (v1)** | $1.0$ | $1.0$ | $1.0$ | $1.0$ | $1.0$ |

In SC1, trajectory error (ATE aligned RMSE) is statistically indistinguishable across all variants and channel conditions ($p_{\text{adj}} > 0.20$). Final DSU components remain $2.0$ for AstroMesh-X due to IBLT difference capacity thresholding under the two-rover concentric trajectory, identical to the Phase 6.1 baseline behavior.

---

## 4. SC10 Exploratory Swarm Scaling Evidence (680 Trials)

> [!NOTE]
> **Exploratory Status:** As authorized, SC10 execution was halted at **680 trials** across 20 channel condition cells ($19$ cells fully completed with 35 trials, $1$ cell partially completed with 15 trials). The following metrics represent preliminary observational scaling evidence and not a full factorial evaluation.

### 4.1 Multi-Hop Forwarding Wire Overhead

In the 8-rover swarm (`SC10_Swarm_8R`), Dijkstra multi-hop forwarding was evaluated across 97 paired trials against `AstroMesh_X_NoMultiHop`:

| Delay $\tau$ (Loss = 0.0) | `AstroMesh_X_Full` | `AstroMesh_X_NoMultiHop` | `AstroMesh_X_NoACKs` | `AstroMesh_v1_Baseline` |
| :---: | :---: | :---: | :---: | :---: |
| **0 ms** | $1,522,392\,\text{B}$ | $919,640\,\text{B}$ | $1,461,936\,\text{B}$ | $1,069,040\,\text{B}$ |
| **20 ms** | $2,125,144\,\text{B}$ | $919,640\,\text{B}$ | $1,461,936\,\text{B}$ | $1,069,040\,\text{B}$ |
| **50 ms** | $3,647,536\,\text{B}$ | $1,839,280\,\text{B}$ | $1,461,936\,\text{B}$ | $1,069,040\,\text{B}$ |
| **100 ms** | $5,169,928\,\text{B}$ | $2,758,920\,\text{B}$ | $1,461,936\,\text{B}$ | $1,069,040\,\text{B}$ |
| **200 ms** | $6,692,320\,\text{B}$ | $3,678,560\,\text{B}$ | $1,461,936\,\text{B}$ | $1,069,040\,\text{B}$ |
| **350 ms** | $6,692,320\,\text{B}$ | $3,678,560\,\text{B}$ | $1,461,936\,\text{B}$ | $1,069,040\,\text{B}$ |
| **500 ms** | $7,611,960\,\text{B}$ | $4,598,200\,\text{B}$ | $1,461,936\,\text{B}$ | $1,069,040\,\text{B}$ |

- **Multi-Hop Wire Overhead:** Multi-hop packet relaying across intermediate rovers increases total wire byte traffic by **$+82.1\%$** (an average of $+2,121,260.5\,\text{bytes}$ per trial) compared to single-hop communication.
- **Graph Partitioning:** Under the tested concentric loops geometry, rovers outside mutual sensor range did not establish loop closures; DSU components remained at $5.0$ for AstroMesh-X and $1.0$ for `AstroMesh_X_RawExchange` and `Baseline_v1`.

---

## 5. Experimental-Design Limitations & Scientific Discrepancies

### 5.1 Conflation in `AstroMesh_X_NoACKs` Ablation

An important experimental design nuance was identified during the audit:
- In `pipeline_astromesh_x.cpp`, setting `enable_explicit_acks = false` suppresses the generation of ACK packets ($44\,\text{B}/\text{hop}$).
- However, suppressing ACKs also eliminates the sender's ACK-awaiting state. Under high link delay ($\tau \ge 50\,\text{ms}$), `NoACKs` does not wait for ACKs and therefore **never triggers spurious retransmissions**.
- At $\tau = 500\,\text{ms}$ (Loss $0\%$), `NoACKs` consumed $43,624\,\text{B}$ vs. `Full`'s $227,140\,\text{B}$ (an apparent savings of $183.5\,\text{kB}$, far greater than the $1.8\,\text{kB}$ of ACK framing).
- **Scientific Caveat:** The `NoACKs` variant conflates two distinct mechanisms: the overhead of ACK frames and the latency-induced retransmission behavior. Future studies should decouple ACK overhead from timeout triggers by implementing un-ACKed retransmissions or RTT-adaptive timeouts.

### 5.2 RTO Sensitivity to Static Timers

The 50 ms initial retry timeout ($\text{RTO}_0 = 50\,\text{ms}$) was hardcoded. While appropriate for low-latency ad-hoc wireless links ($RTT < 40\,\text{ms}$), it is completely mismatched for deep-space or delayed inter-rover links where $RTT \ge 100\,\text{ms}$. To make AstroMesh-X robust across arbitrary delays, an adaptive Jacobson/Karn RTT estimator ($\text{SRTT} + 4 \times \text{RTTVAR}$) must be implemented.

### 5.3 Metric Distinction: Hop-Level Drops vs Application Drops

In the benchmark runner (`benchmark_phase62_robustness.cpp`), `rec.hop_transmissions_dropped` was populated using `metrics_.logical_messages_dropped`. While `metrics_.hop_transmissions_attempted` accurately counts all physical layer frame attempts across all hops and retries, `hop_transmissions_dropped` reflects application-layer retry-exhausted drops. The true link-layer frame drop rate is governed by $p_{\text{loss}}$ on each hop.

---

## 6. Verification & Test Suite Summary

- **CTest Regression Suite:** 21 of 21 test targets passed ($100\%$) in $1.90\,\text{s}$.
- **Unit Test Target #21 (`test_robustness_channel`):** Passed in $0.12\,\text{s}$ (validating byte conservation arithmetic, exponential backoff timers, and in-flight arrival gating).
- **Trial Key Completeness:** All 14,700 SC1 keys verified with 0 duplicates, 0 missing records, and 0 execution failures.
- **Byte Conservation Invariants:** $100\%$ verified across all 15,380 rows in `benchmark_phase62_trials.csv`.

