# AstroMesh-X — Phase 4.5 Engineering Audit Report
## Multi-Rover Orchestration Correctness, DTN Reliability, and Benchmark Validity

**Date:** October 28, 2026  
**Auditor:** Antigravity Pair Programmer  
**Target Revision:** `2a9d5adecd151ee02b2d9afd91a66586080d398f`  
**Branch:** `main`  
**Toolchain:** GCC 16.2.0 (MSYS2 UCRT64), CMake 4.2.0, Ninja 1.13.0  
**Audit Artifacts Generated:**
- [`audit_phase45_results.csv`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/audit_phase45_results.csv) (SHA256: `f0a3d8eb86dc38bbd3d642325d58c7801e78b2e4cc900fdd299cc2a15357d823`)
- [`audit_phase45_rerun.csv`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/audit_phase45_rerun.csv) (SHA256: `f0a3d8eb86dc38bbd3d642325d58c7801e78b2e4cc900fdd299cc2a15357d823`)

---

## 1. Executive Summary & Verification Outcomes

An independent, rigorous engineering audit of the Phase 4.4 multi-rover orchestration, DTN store-and-forward subsystem, and benchmark methodology was conducted. The baseline began with 17/17 passing test suites (77 assertions in orchestration).

During deep trace analysis of the message lifecycle and DTN retry loops, **three root-cause defects** were identified, traced, and mathematically corrected:
1. **DTN Retry Discrepancy & Delivery ACK Omission:** The sender's `dtn_buffer().acknowledge()` was never invoked upon destination packet arrival in `DTNRetry` mode, leaving delivered messages queued to continually retransmit. Conversely, in `NoRetry` mode, dropped messages were never removed or capped at `max_attempts`. This caused `NoRetry` to artificially emit more packets than `DTNRetry`.
2. **Benchmark Measurement Blind Spot (`NoPred` vs `FilteredPred`):** Scenarios 8 and 9 generated identical message counts because the benchmark runner only enqueued `GenericData` packets, completely omitting 80-byte `BoundaryState` packets. The `BoundaryFilter` was never stimulated.
3. **Double Counting & Retry Misclassification in Quota Manager:** The budget manager classified all transmissions with `attempt_count > 0` as retransmissions; because `dequeue_next_eligible` incremented `attempt_count` to 1 on initial dispatch, initial transmissions were erroneously registered as retries.

Following targeted, minimal source corrections, the test suite was expanded with 6 comprehensive regression suites (Tests 14–19) asserting exact conservation equations. All 17 test suites pass cleanly with **119/119 verified assertions** (0 failures). Two independent audit runs generated bit-for-bit identical CSV files.

---

## 2. Baseline vs Final Test Suites

| Metric | Phase 4.4 Reported Baseline | Phase 4.5 Engineering Audit (Final) |
| :--- | :--- | :--- |
| **Branch / HEAD** | `main` / `2a9d5adecd...` | `main` / `2a9d5adecd...` |
| **Passing CTest Suites** | 17 / 17 (100%) | 17 / 17 (100%) |
| **Execution Time** | 0.97 s | 0.87 s |
| **`test_rover_orchestration` Suites** | 13 tests (77 assertions) | 19 tests (119 assertions) |
| **Assertion Failures** | 0 | 0 |
| **Benchmark Scenarios Evaluated** | 15 scenarios | 15 scenarios |
| **Benchmark Reproducibility** | Not independently verified | 100% bit-for-bit identical across runs |

---

## 3. Source Files Inspected

- [`include/astromesh/orchestration/rover_state_machine.hpp`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/include/astromesh/orchestration/rover_state_machine.hpp)
- [`src/orchestration/rover_state_machine.cpp`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/src/orchestration/rover_state_machine.cpp)
- [`include/astromesh/orchestration/multi_rover_collective.hpp`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/include/astromesh/orchestration/multi_rover_collective.hpp)
- [`src/orchestration/multi_rover_collective.cpp`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/src/orchestration/multi_rover_collective.cpp)
- [`include/astromesh/comms/dtn_buffer.hpp`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/include/astromesh/comms/dtn_buffer.hpp)
- [`src/comms/dtn_buffer.cpp`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/src/comms/dtn_buffer.cpp)
- [`include/astromesh/comms/packet_budget.hpp`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/include/astromesh/comms/packet_budget.hpp)
- [`src/comms/packet_budget.cpp`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/src/comms/packet_budget.cpp)
- [`include/astromesh/optimization/boundary_packet.hpp`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/include/astromesh/optimization/boundary_packet.hpp)
- [`src/optimization/boundary_packet.cpp`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/src/optimization/boundary_packet.cpp)
- [`include/astromesh/core/iblt.hpp`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/include/astromesh/core/iblt.hpp)
- [`src/core/iblt.cpp`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/src/core/iblt.cpp)
- [`include/astromesh/graph/pose_graph.hpp`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/include/astromesh/graph/pose_graph.hpp)
- [`src/graph/pose_graph.cpp`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/src/graph/pose_graph.cpp)
- [`include/astromesh/verification/geometric_verifier.hpp`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/include/astromesh/verification/geometric_verifier.hpp)
- [`tests/test_rover_orchestration.cpp`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/tests/test_rover_orchestration.cpp)
- [`tests/benchmark_multi_rover_collective.cpp`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/tests/benchmark_multi_rover_collective.cpp)

---

## 4. Defects Discovered and Corrections Applied

### Defect 1: DTN Buffer Delivery Acknowledgment Omission & Spurious Retransmissions
- **Location:** [`src/orchestration/multi_rover_collective.cpp:369-375`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/src/orchestration/multi_rover_collective.cpp#L369-L375)
- **Root Cause:** In Phase 4.4, when an inbound packet was delivered, `total_messages_delivered_++` occurred, but the sending rover was never notified to remove the message via `acknowledge(message_id, now)`. As a result, successfully delivered messages remained in `queued_messages_` and were re-dequeued on every backoff interval (50 ms, 100 ms, etc.). Furthermore, in `NoRetry` mode (`enable_dtn_retries == false`), dropped messages were never removed, while in `DTNRetry` mode they were capped at `max_attempts`. This explained why `High_Loss_30pct_NoRetry` sent 148 packets while `High_Loss_30pct_DTNRetry` sent only 136 packets in Phase 4.4.
- **Fix:**
  1. In `NoRetry` mode, messages are immediately purged upon dispatch via `dtn_buffer().acknowledge(msg.message_id, current_time_ns_)`.
  2. In `DTNRetry` mode, when `deliver_inbound_dtn_messages()` delivers a message, an explicit ACK removes the message from the sender's `dtn_buffer()`.
  3. Destination deduplication cache was integrated via `record_delivered_dedup()` to drop redundant deliveries.

### Defect 2: Initial Transmission Misclassification in Quota Accounting
- **Location:** [`src/orchestration/multi_rover_collective.cpp:260-265`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/src/orchestration/multi_rover_collective.cpp#L260-L265)
- **Root Cause:** `is_retransmission` was calculated as `msg.attempt_count > 0`. Because `dequeue_next_eligible()` increments `attempt_count` from 0 to 1 during the initial dequeue, `msg.attempt_count` was 1 upon dispatch, misclassifying every initial dispatch as a retransmission in `TrafficStatistics::retransmitted`.
- **Fix:** Changed condition to `const bool is_retry = (msg.attempt_count > 1)`. Separate counters `total_initial_attempts_` and `total_messages_retried_` strictly maintain the invariant:
  $$\text{messages\_sent} = \text{initial\_transmission\_attempts} + \text{messages\_retried}$$

### Defect 3: Inactive Boundary Filtering in Latency Scenarios
- **Location:** [`tests/benchmark_multi_rover_collective.cpp:106-121`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/tests/benchmark_multi_rover_collective.cpp#L106-L121)
- **Root Cause:** The benchmark loop generated only 4-byte `GenericData` packets. `BoundaryFilter` operates exclusively on 80-byte `BoundaryState` packets. Consequently, `High_Latency_50ms_NoPred` and `High_Latency_50ms_FilteredPred` had identical packet counts because `BoundaryFilter` was never invoked.
- **Fix:** Injected periodic 80-byte `BoundaryStatePacket` transmissions every 50 ms. Under 50 ms latency, `High_Latency_50ms_FilteredPred` exercises `process_packet` and successfully drops 9 stale boundary packets (`boundary_packets_filtered = 9`), whereas `High_Latency_50ms_NoPred` delivers all packets (`boundary_packets_filtered = 0`).

### Defect 4: Initial Peer Silence Timeout Stalling
- **Location:** [`src/orchestration/rover_state_machine.cpp:84-88`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/src/orchestration/rover_state_machine.cpp#L84-L88)
- **Root Cause:** Silence duration was computed as `(current_time_ns > last_peer_contact_ns) ? (current_time_ns - last_peer_contact_ns) : 0`. For a newly initialized rover where `last_peer_contact_ns == 0`, if `current_time_ns == 0`, silence was 0. If `current_time_ns` advanced, silence was computed as `current_time_ns - 0`, which was valid, but rovers with `last_peer_contact_ns = 0` bypassed timeout handling when `last_peer_contact_ns == 0` was checked as a sentinel.
- **Fix:** Evaluated silence duration from epoch origin if contact was never established, allowing rovers launched in total isolation to properly transition to `CommunicationDegraded` and `Disconnected`.

---

## 5. Paired Retry/No-Retry Investigation & Denominators

Under identical offered traffic (60 generated messages, 60 admitted messages, seed = 42, duration = 500 ms, 30% uniform random loss):

| Metric | `High_Loss_30pct_NoRetry` | `High_Loss_30pct_DTNRetry` | `High_Loss_50pct_DTNRetry` |
| :--- | :--- | :--- | :--- |
| **Offered Messages Generated** | 60 | 60 | 60 |
| **Admitted Messages** | 60 | 60 | 60 |
| **Initial Transmission Attempts** | 60 | 60 | 60 |
| **Retries (Retransmissions)** | 0 | 16 | 33 |
| **Total Messages Sent** | 60 | 76 | 93 |
| **Simulated Network Losses (Drops)** | 17 (28.3% loss) | 18 (23.7% loss) | 46 (49.5% loss) |
| **Messages Delivered (Raw)** | 43 | 58 | 47 |
| **Unique Messages Delivered** | 43 | 58 | 47 |
| **Duplicate Deliveries** | 0 | 0 | 0 |
| **Total Transmitted Wire Bytes** | 2,880 B | 3,648 B | 4,464 B |
| **Total Delivered Wire Bytes** | 2,064 B | 2,784 B | 2,256 B |
| **Network Delivery Ratio ($\frac{\text{Delivered}}{\text{Sent}}$)** | $43 / 60 = 71.7\%$ | $58 / 76 = 76.3\%$ | $47 / 93 = 50.5\%$ |
| **Application Completion Ratio ($\frac{\text{Unique Delivered}}{\text{Admitted}}$)** | $\mathbf{43 / 60 = 71.7\%}$ | $\mathbf{58 / 60 = 96.7\%}$ | $\mathbf{47 / 60 = 78.3\%}$ |

### Analysis of the Discrepancy
In Phase 4.4, the reported figures (148 sent / 107 delivered for NoRetry; 136 sent / 99 delivered for DTNRetry) were corrupted because delivered packets were retransmitted indefinitely in both scenarios due to missing ACK removal. 
Once ACKs and one-shot dispatch were implemented:
1. `NoRetry` dispatches exactly 60 packets with 0 retries and delivers 43 packets (71.7% application completion, matching expected $1 - 0.30 = 0.70$).
2. `DTNRetry` automatically retransmits dropped packets with backoff (50 ms, 100 ms). 16 retransmissions recover 15 dropped messages, achieving **96.7% application completion**.
3. Under 50% loss, 33 retransmissions recover 78.3% of messages within the 500 ms trial window.

---

## 6. Message Accounting Conservation Laws & Invariants

The following invariants hold strictly across the audited architecture:

1. **Transmission Conservation:**
   $$\text{messages\_sent} = \text{initial\_transmission\_attempts} + \text{messages\_retried}$$
   *Audit Outcome:* **PASSED** across all 15 scenarios.

2. **Wire Byte Conservation:**
   $$\text{total\_wire\_bytes\_sent} = \text{payload\_bytes\_sent} + \text{protocol\_bytes\_sent} + \text{framing\_bytes\_sent}$$
   *Audit Outcome:* **PASSED** across all 15 scenarios.

3. **In-Flight & Delivery Balance:**
   $$\text{messages\_sent} = \text{messages\_delivered} + \text{messages\_dropped} + \text{in\_flight\_messages}$$
   *Audit Outcome:* **PASSED** across all 15 scenarios.

4. **Idempotence & Deduplication:**
   $$\text{messages\_delivered} = \text{unique\_messages\_delivered} + \text{duplicate\_deliveries}$$
   *Audit Outcome:* **PASSED** across all 15 scenarios.

---

## 7. IBLT Reconciliation Atomicity and Gating

1. **Symmetric Difference Representation:** IBLT entries store symmetric differences $S_A \oplus S_B$. The implementation does not claim direct intersection.
2. **Transactional Rollback:** When candidate differences exceed IBLT capacity (tested in Test 18 with 15 elements vs capacity 8), `perform_synchronization()` returns `false` gracefully. The pose graphs, node counts, edge counts, and DSU partition structures of both rovers remain strictly identical before and after the failure.
3. **Geometric Verification Gate:** In Test 10, when point correspondences exhibit severe noise/outliers (RMSE > 0.05 m), `verify_loop_closure` returns `Rejected`. The candidate loop closure edge remains in `Unverified` status and the DSU partitions are never merged.

---

## 8. CSV Schema & Metric Definitions

The generated CSVs ([`audit_phase45_results.csv`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/audit_phase45_results.csv) and [`audit_phase45_rerun.csv`](file:///C:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/audit_phase45_rerun.csv)) adhere to the following schema:

| Column | Unit / Type | Denominator / Basis | Description |
| :--- | :--- | :--- | :--- |
| `scenario_name` | string | N/A | Identifier of the test scenario |
| `seed` | uint64 | N/A | PRNG seed used for deterministic simulation |
| `rover_count` | count | N/A | Number of simulated rovers |
| `duration_ms` | ms | N/A | Total simulated duration |
| `step_ms` | ms | N/A | Discrete event simulation step size |
| `latency_ms` | ms | N/A | One-way network channel latency |
| `packet_loss_rate` | fraction | N/A | Stochastic network packet drop probability |
| `partition_duration_ms` | ms | N/A | Duration of bilateral partition window |
| `messages_generated` | count | N/A | Total messages generated by application logic |
| `messages_admitted` | count | `messages_generated` | Messages admitted into DTN store-and-forward buffers |
| `messages_sent` | count | N/A | Total message dispatch events into mesh network |
| `initial_transmission_attempts`| count | `messages_admitted` | First transmission attempt per admitted message |
| `messages_retried` | count | N/A | Retransmission attempts following loss |
| `messages_delivered` | count | `messages_sent` | Total packet arrivals at destination rovers |
| `unique_messages_delivered` | count | `messages_admitted` | Distinct message IDs received at least once |
| `duplicate_deliveries` | count | `messages_delivered` | Arriving packets dropped by deduplication cache |
| `messages_expired` | count | `messages_admitted` | Messages pruned due to TTL exhaustion |
| `messages_dropped` | count | `messages_sent` | Packets dropped by loss or partition severance |
| `messages_rejected` | count | `messages_generated` | Messages rejected upon enqueue (quota / capacity) |
| `retries_exhausted` | count | `messages_admitted` | Messages dropped after reaching 5 retry attempts |
| `in_flight_messages` | count | N/A | Packets remaining in network transit queue at end |
| `messages_processed` | count | `unique_messages_delivered` | Packets decoded and executed by destination logic |
| `payload_bytes_sent` | bytes | N/A | Application payload bytes dispatched |
| `protocol_bytes_sent` | bytes | N/A | DTN message envelope header bytes (24 B / msg) |
| `framing_bytes_sent` | bytes | N/A | Transport/MAC layer framing bytes (20 B / msg) |
| `total_wire_bytes_sent` | bytes | N/A | Sum of payload + protocol + framing bytes sent |
| `total_wire_bytes_delivered` | bytes | N/A | Sum of payload + protocol + framing bytes delivered |
| `boundary_packets_filtered` | count | N/A | Stale/delayed boundary packets pruned by filter |
| `state_transitions_total` | count | N/A | Total state machine transitions executed |
| `sync_attempts` | count | N/A | Total IBLT synchronization sessions initiated |
| `sync_successes` | count | `sync_attempts` | Successful IBLT reconciliations with valid closures |
| `sync_incomplete_decodes` | count | `sync_attempts` | Failed IBLT decodes (capacity exceeded) |
| `sync_budget_denials` | count | `sync_attempts` | Reconciliations aborted due to byte quota limits |
| `queue_peak_messages` | count | N/A | Maximum queue depth observed across simulation |
| `queue_peak_payload_bytes` | bytes | N/A | Maximum queue payload bytes observed |
| `final_connected_components` | count | N/A | DSU connected components at simulation termination |
| `verified_inter_rover_closures`| count | N/A | Inter-rover loop closure edges promoted to Verified |

---

## 9. Reproducibility & Bit-for-Bit Identity

The benchmark binary was executed twice consecutively without modifying code, compiler flags, or seeds:
```powershell
$env:PATH = "C:\msys64\ucrt64\bin;" + $env:PATH
.\build-ucrt64-clean\bin\benchmark_multi_rover_collective.exe audit_phase45_results.csv
.\build-ucrt64-clean\bin\benchmark_multi_rover_collective.exe audit_phase45_rerun.csv
```

**Verification Script:**
```python
import hashlib

h1 = hashlib.sha256(open('audit_phase45_results.csv', 'rb').read()).hexdigest()
h2 = hashlib.sha256(open('audit_phase45_rerun.csv', 'rb').read()).hexdigest()
assert h1 == h2
```

**Result:**
- Both files: exactly 2,853 bytes.
- SHA256: `f0a3d8eb86dc38bbd3d642325d58c7801e78b2e4cc900fdd299cc2a15357d823`.
- Difference: 0 bytes, 0 lines, 0 NaNs, 0 Infs. **100% bit-for-bit deterministic reproducibility**.

---

## 10. Limitations of the Simulated Network Model

The multi-rover collective simulation provides a discrete-event evaluation of DTN buffering, state transitions, and pose-graph synchronization. However, the simulated model exhibits the following physical and operational limitations:

1. **Discrete-Event Channel Model:** Radio channel behavior is modeled using fixed or jittered delays and independent Bernoulli packet loss. It does not model RF path loss, shadowing, multipath fading, Doppler shift, CSMA/CA MAC backoff collisions, or antenna orientation gains.
2. **Simplified Routing Topology:** Packets are scheduled point-to-point via pairwise link states. Multi-hop DTN routing protocols (e.g., Epidemic, PRoPHET, Spray-and-Wait) are not yet implemented.
3. **Synthetic Correspondences:** Candidate correspondences in synchronization tests are synthetic feature point pairs. Feature extraction and photometric description from stereo/RGB-D camera pipelines were not tested on hardware.
4. **Synchronous Optimization Step:** Distributed CORD iterations within the collective are synchronized via discrete time steps; asynchronous decentralized numerical convergence under heterogeneous CPU loads remains to be evaluated.
5. **No Radio Hardware Validation:** Results are purely simulation-based. No claims of real-world radio hardware deployment, RF propagation validation, or hardware-in-the-loop validation are made.
