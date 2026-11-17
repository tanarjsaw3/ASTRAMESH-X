# AstroMesh-X — Phase 6.2 Interim Scientific & Technical Audit

**Audit Date:** 2026-10-29  
**Target Milestone:** Milestone 6.2 — Packet Loss & Communication Delay Robustness  
**Audit Scope:** Active process monitoring, retry/ACK mechanism inspection, latency event-ordering verification, and wire-byte growth decomposition.

---

## 1. Executive Summary & Process State

1. **Process Preservation & Continuity:**
   - Active task `task-927` executing `benchmark_phase62.exe` (PID: `13536`) has been preserved without interruption or modification.
   - Operating status is verified active and responsive (`Responding: True`, CPU execution time $>8,773\,\text{s}$, working set memory bounded at $\sim 11.6\,\text{MB}$).
   - As of this audit, SC10 has completed 15 of 70 grid cells (511 / 2,450 trial records), actively streaming row-by-row to [`benchmark_phase62_trials.csv`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/benchmark_phase62_trials.csv).
2. **SC1 Dataset State:**
   - `SC1_Nominal_2R` is 100% complete with exactly 14,700 records (70 cells $\times$ 30 seeds $\times$ 7 variants = 210 runs/cell).
   - Invariants strictly verified: 0 duplicate keys, 0 non-finite values, 100% byte conservation (`total == data + ack` and `data == payload + protocol + framing`), and 0 pipeline failures.

---

## 2. Invariant & Behavioral Questions: Technical Findings

### Question 1: Why Do `AstroMesh_X_Full` and `AstroMesh_X_NoACKs` Have Identical Application Completion Ratios Across All SC1 Loss Levels?

Inspection of `src/simulation/pipeline_astromesh_x.cpp` (lines 318–395) and `tests/benchmark_phase62_robustness.cpp` (lines 795–805) reveals the exact architectural reason:

1. **Both Variants Enable DTN Retries:**
   In `AstroMesh_X_NoACKs`, `cfg.enable_dtn_retries` remains `true` (only `cfg.enable_explicit_acks` is set to `false`).
2. **Identical Stochastic Channel Trials:**
   In the simulation pipeline, channel loss is evaluated via Bernoulli trials:
   ```cpp
   if (config_.enable_dtn_retries) {
       size_t first_success_attempt = 5; // 5 = all 5 attempts failed
       for (size_t attempt = 0; attempt < 5; ++attempt) {
           if (loss_rate <= 0.0 || dist_unit_(rng_) >= loss_rate) {
               first_success_attempt = attempt;
               break;
           }
       }
   ```
   Because paired variants receive the identical initial seed (`cfg.seed = seed`), the pseudo-random number generator produces the *exact same sequence* of success/loss outcomes for both `AstroMesh_X_Full` and `AstroMesh_X_NoACKs`.
3. **Application Completion Ratio Formulation:**
   Application completion ratio is defined strictly as:
   $$\text{Application Completion Ratio} = \frac{\text{unique\_deliveries}}{\text{logical\_messages\_sent}}$$
   - `logical_messages_sent` increments once per pairwise communication attempt at each step.
   - `unique_deliveries` increments if and only if `first_success_attempt < 5` and $t_{\text{arrive}} \le \text{duration}$.
   - While `AstroMesh_X_Full` experiences spurious retransmissions under latency, duplicate arrivals increment `metrics_.duplicate_deliveries`, *not* `metrics_.unique_deliveries`.
   - Consequently, `unique_deliveries` and `logical_messages_sent` are mathematically identical between `Full` and `NoACKs` for every single seed and loss rate, yielding identical application completion ratios.

---

### Question 2: Retransmission Timeout (RTO) Schedule vs. Event Ordering

The empirical data for `SC1_Nominal_2R` at 0% loss demonstrates exact step-function scaling in retransmissions as latency increases:

| One-Way Delay ($\tau$) | Round-Trip Time ($RTT = 2\tau$) | Spurious Retries per Step | SC1 Total Retransmissions (41 Steps) | SC1 Total Wire Bytes |
| :---: | :---: | :---: | :---: | :---: |
| **0 ms** | 0 ms | 0 | 0.0 | **45,428 B** |
| **20 ms** | 40 ms | 0 | 0.0 | **45,428 B** |
| **50 ms** | 100 ms | 1 | 41.0 | **90,856 B** ($2 \times 45,428$) |
| **100 ms** | 200 ms | 2 | 82.0 | **136,284 B** ($3 \times 45,428$) |
| **200 ms** | 400 ms | 3 | 123.0 | **181,712 B** ($4 \times 45,428$) |
| **350 ms** | 700 ms | 3 | 123.0 | **181,712 B** ($4 \times 45,428$) |
| **500 ms** | 1000 ms | 4 (max) | 164.0 | **227,140 B** ($5 \times 45,428$) |

#### Verification Against Source Event Schedule
In `include/astromesh/comms/dtn_buffer.hpp` (line 154) and `src/simulation/pipeline_astromesh_x.cpp` (line 316), the retransmission backoff schedule is:
- **Timer 1:** $+50\,\text{ms}$ ($50\,\text{ms}$ cumulative).
- **Timer 2:** $+100\,\text{ms}$ ($150\,\text{ms}$ cumulative).
- **Timer 3:** $+200\,\text{ms}$ ($350\,\text{ms}$ cumulative).
- **Timer 4:** $+400\,\text{ms}$ ($750\,\text{ms}$ cumulative).

#### Event Ordering Analysis:
1. **$\tau = 20\,\text{ms} \implies RTT = 40\,\text{ms}$:**
   - $t = 0\,\text{ms}$: Data sent.
   - $t = 20\,\text{ms}$: Data received; ACK emitted.
   - $t = 40\,\text{ms}$: ACK received at sender.
   - $t = 50\,\text{ms}$: Timer 1 would fire, but ACK already arrived $\implies$ **0 retries**.
2. **$\tau = 50\,\text{ms} \implies RTT = 100\,\text{ms}$:**
   - $t = 50\,\text{ms}$: Timer 1 fires ($100 > 50$) $\implies$ **Retry 1 dispatched**.
   - $t = 100\,\text{ms}$: ACK arrives at sender.
   - $t = 150\,\text{ms}$: Timer 2 would fire, but message is now acknowledged $\implies$ **Exactly 1 retry**.
3. **$\tau = 100\,\text{ms} \implies RTT = 200\,\text{ms}$:**
   - $t = 50\,\text{ms}$: Timer 1 fires ($200 > 50$) $\implies$ **Retry 1 dispatched**.
   - $t = 150\,\text{ms}$: Timer 2 fires ($200 > 150$) $\implies$ **Retry 2 dispatched**.
   - $t = 200\,\text{ms}$: ACK arrives at sender.
   - $t = 350\,\text{ms}$: Timer 3 canceled $\implies$ **Exactly 2 retries**.
4. **$\tau = 200\,\text{ms}$ and $\tau = 350\,\text{ms}$ ($RTT = 400\,\text{ms}$ and $700\,\text{ms}$):**
   - In both cases, $RTT > 350\,\text{ms}$ (Timer 3) but $RTT < 750\,\text{ms}$ (Timer 4).
   - Timers 1, 2, and 3 expire, triggering **exactly 3 retries**. ACK arrives before Timer 4. This explains why $200\,\text{ms}$ and $350\,\text{ms}$ have *identical* retransmission and wire counts.
5. **$\tau = 500\,\text{ms} \implies RTT = 1000\,\text{ms}$:**
   - $RTT > 750\,\text{ms}$, triggering all 4 retries (maximum retry cap reached) $\implies$ **Exactly 4 retries per step** ($4 \times 41 = 164$ retries).

---

### Question 3: Does `AstroMesh_X_NoACKs` Disable ACKs Without Affecting Unrelated Logic?

**Source Code Verification:**
In `src/simulation/pipeline_astromesh_x.cpp`:
- `config_.enable_explicit_acks` strictly gates:
  1. Spurious retransmission generation:
     ```cpp
     if (config_.enable_explicit_acks && one_way_delay_ns > 0) { ... }
     ```
  2. ACK packet wire accounting:
     ```cpp
     if (config_.enable_explicit_acks) {
         metrics_.ack_wire_bytes_transmitted += ack_wire_bytes;
         ...
     }
     ```
- `AstroMesh_X_NoACKs` leaves untouched:
  - Odometry and landmark observation ingestion.
  - DTN retries on packet loss (up to 5 attempts).
  - Multi-hop routing and hop limit checks.
  - Safe keyframe pruning.
  - Umeyama RANSAC geometric verification.
  - Disjoint Set Union (DSU) topological tracking.
  - Continuous Riemannian Dynamics (CORD) on $\mathrm{SE}(2)$.
- **Conclusion:** No unrelated logic or side-effects are introduced by `NoACKs`.

---

### Question 4: Wire-Byte Growth Decomposition — Retransmissions vs. Forwarding

To determine whether wire-byte inflation is caused by retransmissions, forwarding, or both, we evaluate the empirical difference between `AstroMesh_X_Full` and `AstroMesh_X_NoMultiHop` across both scenarios:

#### In SC1 (2 Rovers):
- Multi-hop forwarding requires an intermediate rover $C \notin \{A, B\}$. With only 2 rovers, no intermediate forwarder exists.
- Empirical check: `(full_sc1['total_wire_bytes'] - nomultihop_sc1['total_wire_bytes']).abs().max() == 0`.
- **Verdict for SC1:** Wire-byte growth across delays is **100% caused by spurious retransmissions**. Multi-hop forwarding overhead is exactly 0.

#### In SC10 (8 Rovers):
- With 8 rovers distributed across an arena, multi-hop routing forwards packets across 2 hops when direct links are out of range.
- Mean wire consumption from interim SC10 data:
  - `AstroMesh_X_Full`: $4,719,450.1\,\text{B}$
  - `AstroMesh_X_NoMultiHop`: $2,607,626.8\,\text{B}$
  - Multi-hop forwarding overhead: **$2,111,823.3\,\text{B}$ ($44.7\%$ of total wire bytes)**.
- **Verdict for SC10:** Wire-byte growth is driven by **both** retransmissions (ARQ retry inflation under loss/delay) and **multi-hop forwarding overhead** (forwarding copies across intermediate peers).

---

## 3. Dataset Preservation & Current Task Metrics

- **Raw Data Integrity:** [`benchmark_phase62_trials.csv`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/benchmark_phase62_trials.csv) is completely intact and growing monotonically.
- **Current Dataset Breakdown:**
  - `SC1_Nominal_2R`: 14,700 rows (Complete across all 70 cells $\times$ 30 seeds $\times$ 7 variants).
  - `SC10_Swarm_8R`: 607 rows (Interim, 17 cells completed, Cell 18 active).
  - Total trial rows: 15,307 rows.
- **CTest Verification:** 21/21 test targets passing (100% in 5.22 s). Zero regressions.

---

## 4. Experimental-Design Note: `Full` vs. `NoACKs` Mechanistic Conflation

A critical methodological distinction must be documented regarding what the `AstroMesh_X_NoACKs` ablation actually isolates:

1. **Zero-Delay Regime ($\tau = 0$): Pure Wire Overhead Isolation**
   - In lossless or zero-delay channels where $RTT = 0 < \text{RTO}$, spurious retries are zero for both variants.
   - The observed difference between `AstroMesh_X_Full` and `AstroMesh_X_NoACKs` is strictly $(24 + 20)\,\text{B} = 44\,\text{B}$ per transmission.
   - Here, `NoACKs` cleanly isolates the direct transport framing and protocol header overhead of explicit network acknowledgments.

2. **Delay-Rich Regime ($\tau \ge 50\,\text{ms}$): Conflation with ARQ Timer Behavior**
   - When latency increases such that $RTT = 2\tau > \text{RTO} = 50\,\text{ms}$, `AstroMesh_X_Full` awaits an ACK that arrives after the sender's retry timer has already fired. This triggers 1 to 4 spurious data retransmissions per message.
   - In contrast, because `AstroMesh_X_NoACKs` disables explicit ACKs, the sender does not maintain an ACK-awaiting retry timer. It sends data once (plus loss-triggered retries if loss occurs), never times out waiting for an ACK, and therefore generates **0 spurious retransmissions**.
   - As a result, at $\tau = 500\,\text{ms}$ in SC1:
     - `Full` wire consumption: $224,982.7\,\text{B}$ ($18,040\,\text{B}$ ACK wire bytes + $206,942.7\,\text{B}$ data wire bytes).
     - `NoACKs` wire consumption: $90,457.7\,\text{B}$ ($0\,\text{B}$ ACK wire bytes + $90,457.7\,\text{B}$ data wire bytes).
     - Wire gap: **$134,525\,\text{B}$**, which is $>7.4\times$ greater than the direct ACK wire overhead alone ($18,040\,\text{B}$).
   - **Methodological Conclusion:** In delayed channels, comparing `Full` against `NoACKs` evaluates the *joint compound effect* of:
     - (a) Removing the physical wire footprint of ACK packets, **AND**
     - (b) Removing the delay-induced spurious retransmission storm caused by a static $\text{RTO} < RTT$.
   - **Recommendation for Future Optimization:** An adaptive RTO estimator (e.g. Jacobson/Karn algorithm: $\text{RTO} = \text{SRTT} + 4\times\text{RTTVAR}$) or delay-tolerant boundary filtering would resolve the spurious retransmission penalty in `Full` without requiring the abandonment of reliable acknowledgments. This distinction must be preserved in all Phase 6.2 publications and reports.

