# AstroMesh-X — Milestone 7.11 Implementation Plan
## Multi-Hop Distributed SLAM Routing & Live UDP Dashboard Streaming (Final Corrected Edition)

**Document ID:** `docs/M7_11_IMPLEMENTATION_PLAN.md`  
**Date:** 2026-11-06 (Amended Protocol Edition)  
**Status:** PROPOSED & READY FOR IMPLEMENTATION REVIEW  
**Author:** Antigravity (AI Coding Assistant)  
**Parent Architecture Document:** [`docs/M7_11_ARCHITECTURE.md`](file:///c:/Users/TANISHKA%20SAWANT/ASTRAMESH-X/docs/M7_11_ARCHITECTURE.md)  
**Target Environment:** Windows 11 + MSYS2 UCRT64 GCC 16.2.0 + Ninja 1.13.2 + CMake 4.4.4 (100% Software-Only)

---

## 1. Implementation Principles & Guardrails

1. **Strict Wire Protocol Compatibility:**
   - Transport framing strictly retains the canonical 36-byte Little-Endian `MessageFrame` header format.
   - Multi-hop envelope (40 bytes Little-Endian, `MAGIC = 'MHTN'`, 8-node path trail) is packed within `MessageFrame.payload`.
   - Zero ground-truth labels on the network wire.
2. **Deterministic Dual-Constraint Capacity:**
   - Both outgoing and transit DTN queues strictly enforce 200 frames and 1 MB memory limits with priority-aware eviction and backpressure.
3. **Loop-Free Path-Vector Forwarding:**
   - Traversal path trail array `uint16_t visited_nodes[8]` prevents loops across arbitrary rover IDs ($1 \dots 65535$) without capacity overflow under `MaxHops <= 8`.
4. **Non-Destructive Git Policy:**
   - Zero Git commits, pushes, or resets without explicit user confirmation.
   - All 5 historical benchmark CSV files preserved untouched.

---

## 2. File & Module Change Breakdown

### New Source Files & Headers

| File Path | Purpose |
|:---|:---|
| `include/astromesh/net/routing_table.hpp` | Distance-vector routing table, route metrics, split horizon with poison reverse, and route sequence/epoch tracking |
| `src/net/routing_table.cpp` | Implementation of distance-vector updates, poison reverse advertising, stale sequence rejection, and route pruning |
| `include/astromesh/net/multihop_router.hpp` | Multi-hop routing engine: path-vector loop prevention, TTL checks, custody ACK regeneration, and transit custody |
| `src/net/multihop_router.cpp` | Implementation of 40-byte `MultihopEnvelope` serialization, forwarding pipeline, and telemetry tracking |
| `include/astromesh/verification/benchmark_ground_truth_registry.hpp` | Out-of-band benchmark ground-truth registry for independent precision/recall evaluation without wire flags |
| `tests/test_multihop_routing.cpp` | Comprehensive 12-test unit test suite covering full protocol test matrix (CTest #27) |
| `app/telemetry_collector.py` | Asynchronous UDP listener thread on port 29999 for live mission control telemetry |

### Modified Existing Files

| File Path | Proposed Modification |
|:---|:---|
| `include/astromesh/net/message_frame.hpp` | Add `MessageType::RoutingAdvertisement` (Type ID 9) and `MessageType::HopCustodyAck` (Type ID 10) to `MessageType` enum |
| `include/astromesh/net/rover_node.hpp` | Integrate `RoutingTable` and `MultihopRouter`, add custody index, transit queues, and wire accounting |
| `src/net/rover_node.cpp` | Update `poll_events()` to route non-local target packets through `MultihopRouter` and regenerate custody ACKs on duplicate MessageIDs |
| `include/astromesh/net/distributed_reconciliation.hpp` | Support initiating multi-hop IBLT sync to remote rovers via routing table |
| `src/net/distributed_reconciliation.cpp` | Clean production wire payload of ground-truth flags; delegate evaluation to external registry |
| `app/server.py` | Mount `/api/live/stream` SSE endpoint and integrate `telemetry_collector.py` |
| `app/static/app.js` | Add Live Telemetry toggle, real-time pose graph canvas rendering, and multi-hop link visualization |
| `CMakeLists.txt` | Add `test_multihop_routing` target to CTest suite |

---

## 3. Detailed Implementation Tasks

### Task 1: Routing Table & Distance-Vector with Poison Reverse & Epochs (Step 1)
- Implement `RouteEntry` structure:
  ```cpp
  struct RouteEntry {
      uint16_t destination_rover_id{0};
      uint16_t next_hop_rover_id{0};
      uint8_t metric{1};              // Hop count (1..15, 16 = infinity)
      uint32_t sequence_number{0};    // Route epoch
      uint64_t last_updated_ns{0};
      bool is_valid(uint64_t now_ns, uint64_t timeout_ns) const;
  };
  ```
- Implement `RoutingTable` class:
  - `update_route(destination, next_hop, metric, seq, now_ns)`: Stale advertisement rejection (`seq < local.seq`).
  - `lookup_next_hop(destination) -> optional<uint16_t>`
  - `prune_stale_routes(now_ns, timeout_ns)`: Marks expired routes as $\text{metric} = 16$.
  - `serialize_advertisement(target_neighbor_id) -> vector<uint8_t>`: Applies **Split Horizon with Poison Reverse** (advertises $\text{metric} = 16$ for routes learned via `target_neighbor_id`).
  - `apply_advertisement(sender_id, payload, now_ns)`: Distance-vector relaxation with metric clamping ($1 \dots 16$).

### Task 2: Multi-Hop Envelope & Forwarding Engine (Step 2)
- Define 40-byte multi-hop envelope header (`MultihopEnvelope`):
  - `MAGIC = 0x4E54484D` (`'MHTN'`)
  - `version = 1`, `flags` (`CUSTODY_ACK_REQUESTED`, `E2E_ACK_REQUESTED`)
  - `origin_rover_id`, `final_target_id`
  - `hop_count`, `max_hops = 5` (max 8)
  - `message_id` (64-bit immutable end-to-end ID)
  - `path_length` ($1 \dots 8$), `reserved[3]`
  - `visited_nodes[8]` (`uint16_t[8]` path-vector trail)
- Implement `MultihopRouter`:
  - `inspect_and_route(frame, sender_ep, now_ns) -> ForwardingAction`
  - Loop detection: scan `visited_nodes[0..path_length-1]`. If `self_id` found $\implies$ drop immediately (`routing_loops_suppressed++`).
  - TTL expiration: `hop_count + 1 >= max_hops` $\implies$ drop with diagnostic (`ttl_expired_drops++`).
  - Transit buffering: If `next_hop` is unreachable, enqueue into transit DTN queue with capacity bounds (200 frames / 1 MB).
  - Forwarding: Increment `hop_count++`, append `visited_nodes[path_length++] = self_id`, retransmit to next hop.

### Task 3: Hop Custody ACKs & Regeneration on Duplicate MessageID (Step 3)
- In `RoverNode::poll_events()`:
  - If a received frame requests custody ACK (`flags & 0x01`): send immediate `HopCustodyAck` to the immediate sender (`frame.sender_rover_id`).
  - Upon receiving `HopCustodyAck`: upstream node marks message as custody transferred, halting link-level retransmissions.
  - Custody ACK Regeneration: If a duplicate `MessageID` is received (due to lost custody ACK or end-to-end ACK loss), regenerate and transmit `HopCustodyAck` immediately without duplicate downstream forwarding.
  - When the final destination delivers the frame to the SLAM agent: destination generates an `EndToEndDeliveryAck` routed multi-hop back to `origin_rover_id`.

### Task 4: Production Wire Purity & Benchmark Registry (Step 4)
- Remove `is_ground_truth_inlier` from wire serialization in `CandidateProposalPayload` and `LandmarkObservationRecord`.
- Implement `GroundTruthBenchmarkRegistry` in `tests/support/` to store landmark ground truth and evaluate TP, FP, TN, FN externally during benchmark sweeps.
- RANSAC acceptance remains 100% agnostic to ground truth.

### Task 5: Integration into `RoverNode` & `DistributedReconciliationAgent` (Step 5)
- In `RoverNode::poll_events()`:
  - Non-local frames forwarded via `MultihopRouter`.
  - Destination frames stripped of `MultihopEnvelope`, exposing logical origin to SLAM agent.
- In `DistributedReconciliationAgent`:
  - Enables IBLT synchronization, candidate proposals, and boundary states across multi-hop paths.
  - Verifies multi-hop loop closures and relaxes global trajectory ATE via distributed CORD.

### Task 6: Unit & Integration Verification Suite (Step 6)
- Implement `tests/test_multihop_routing.cpp` executing all 12 test matrix specifications:
  - Direct equivalence, 3-rover linear relay, arbitrary IDs, TTL boundary, loop suppression, split horizon, custody ACK regeneration, buffer exhaustion, partition recovery, malformed safety, wire purity, and regression integrity.
- Register executable in `CMakeLists.txt` (CTest #27).

### Task 7: Live UDP Dashboard Streaming Engine (Step 7)
- Implement `app/telemetry_collector.py` listening on UDP `127.0.0.1:29999`.
- Mount `/api/live/stream` (SSE) in `app/server.py`.
- Augment `app/static/app.js` with real-time multi-hop beam rendering and live CORD relaxation visualization.

### Task 8: Full Regression & Milestone Verification (Step 8)
- Run complete regression: CTest (27/27), acceptance (13/13), 2-process UDP SLAM, multi-process audit, and all 5 CSV audits.
- Deliver `docs/M7_11_DEVELOPMENT_REPORT.md` and update `docs/PROJECT_STATE.md`.

---

## 4. Potential Risks & Mitigation Strategies

| Risk | Impact | Mitigation Strategy |
|:---|:---:|:---|
| **Counting to Infinity under Link Breakage** | High (routing loops) | Split Horizon with Poison Reverse advertises $\infty$ back to route sources; route epochs reject stale routes. |
| **Path-Vector Overflow** | Zero | `MaxHops <= 8` and `visited_nodes[8]` capacity strictly guaranteed; no valid path can overflow. |
| **Transit DTN Buffer Exhaustion** | Medium (memory consumption) | Dual limits (200 frames / 1 MB). Low-priority evicted first. High-priority triggers backpressure (`CapacityBytesExceeded`). |
| **Relay Volatile Crash / Restart** | Low (loss of transit queue) | Origin maintains End-to-End Delivery Timer ($3 \times \text{MaxHops} \times \text{SRTT}$) and re-originates if unacknowledged. |

---

## 5. Definition of Done (DoD) for Milestone 7.11

- [ ] All code compiles cleanly with 0 warnings on MSYS2 UCRT64 GCC 16.2.0.
- [ ] All 12 test cases in `test_multihop_routing.cpp` pass (CTest 27/27).
- [ ] 3-Rover linear relay verified: Rovers 1 and 3 unify DSU partition and relax ATE through Rover 2.
- [ ] Loop suppression and TTL expiration confirmed with zero circulatory packets.
- [ ] Split horizon with poison reverse confirmed under link severance.
- [ ] Zero ground-truth flags on network wire payloads.
- [ ] Live dashboard streaming verified on port 29999 over SSE.
- [ ] All 5 historical benchmark CSVs preserved bit-for-bit intact.
- [ ] `docs/M7_11_DEVELOPMENT_REPORT.md` and `docs/PROJECT_STATE.md` updated.
