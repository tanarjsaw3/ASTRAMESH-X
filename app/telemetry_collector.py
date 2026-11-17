#!/usr/bin/env python3
"""
AstroMesh-X UDP Telemetry Collector & SSE Streaming Engine
Listens on UDP socket (default port 29999) for live rover network/SLAM telemetry,
maintains real-time swarm topology & routing state, and streams updates via Server-Sent Events (SSE).
"""

import collections
import json
import queue
import socket
import threading
import time
from typing import Any, Dict, List, Optional

DEFAULT_TELEMETRY_PORT = 29999
MAX_EVENT_HISTORY = 1000

class TelemetryCollector:
    def __init__(self, port: int = DEFAULT_TELEMETRY_PORT, host: str = "127.0.0.1"):
        self.host = host
        self.port = port
        self.running = False
        self.sock: Optional[socket.socket] = None
        self.thread: Optional[threading.Thread] = None

        # Thread-safe ring buffer and state cache
        self.lock = threading.Lock()
        self.event_history = collections.deque(maxlen=MAX_EVENT_HISTORY)
        self.subscribers: List[queue.Queue] = []
        self.injected_partitions = set()

        # Current swarm state snapshot
        self.swarm_state: Dict[str, Any] = {
            "timestamp": time.time(),
            "rovers": {},
            "active_links": [],
            "multihop_routes": [],
            "partitions": [],
            "severed_links": [],
            "is_partitioned": False,
            "global_metrics": {
                "packets_forwarded": 0,
                "routing_loops_suppressed": 0,
                "custody_acks": 0,
                "e2e_acks": 0,
                "transit_buffer_occupancy": 0,
                "loop_closures_verified": 0,
                "connected_components": 0,
                "is_unified": False,
                "dsu_components": 1,
                "is_converged": True,
                "reciprocal_edge_fraction": 1.0,
                "adaptive_regime": "Nominal",
                "elected_anchor": 1,
                "cycles_remediated": 0,
                "global_ate_m": 0.0,
                "active_opportunities_evaluated": 0,
                "active_opportunities_scheduled": 0,
                "active_wire_bytes_spent": 0,
                "rendezvous_proposals_verified": 0,
                "total_nodes_pruned": 0,
                "total_landmarks_evicted": 0,
                "total_compaction_notifications": 0,
                "total_compaction_wire_bytes": 0,
                "current_swarm_logical_bytes": 0,
                "current_swarm_rss_bytes": 0,
                "peak_swarm_pressure": "Normal",
                "total_frontiers_detected": 0,
                "total_frontiers_completed": 0,
                "active_frontiers_count": 0,
                "swarm_coverage_percent": 0.0,
                "connectivity_uptime_percent": 100.0
            }
        }

    def start(self):
        """Starts the background UDP listener thread."""
        if self.running:
            return
        self.running = True

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.host, self.port))
            self.sock.settimeout(0.5)
            print(f"[TelemetryCollector] Listening for UDP telemetry on {self.host}:{self.port}")
        except Exception as e:
            print(f"[TelemetryCollector] Warning: could not bind UDP socket {self.host}:{self.port} ({e}).")
            self.sock = None

        self.thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.thread.start()

    def stop(self):
        """Stops the UDP listener."""
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def _listen_loop(self):
        while self.running:
            if not self.sock:
                time.sleep(0.5)
                continue
            try:
                data, addr = self.sock.recvfrom(65535)
                self.ingest_datagram(data, addr)
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    time.sleep(0.1)

    def ingest_datagram(self, raw_bytes: bytes, addr: tuple):
        """Decodes JSON or binary telemetry datagram and broadcasts to subscribers."""
        try:
            text = raw_bytes.decode("utf-8")
            event = json.loads(text)
            self.record_event(event)
        except Exception:
            # Non-JSON payload or raw frame
            pass

    def record_event(self, event: Dict[str, Any]):
        """Records an event, updates current swarm state, and fans out to active SSE queues."""
        if "timestamp" not in event:
            event["timestamp"] = time.time()

        with self.lock:
            self.event_history.append(event)
            self._update_swarm_state(event)

            # Fan out to SSE subscribers
            dead_subscribers = []
            for q in self.subscribers:
                try:
                    q.put_nowait(event)
                except queue.Full:
                    dead_subscribers.append(q)

            for d in dead_subscribers:
                if d in self.subscribers:
                    self.subscribers.remove(d)

    def _update_swarm_state(self, event: Dict[str, Any]):
        """Updates internal swarm representation from received event."""
        etype = event.get("type", "")
        rid = str(event.get("rover_id", 0))

        if rid != "0":
            if rid not in self.swarm_state["rovers"]:
                self.swarm_state["rovers"][rid] = {
                    "rover_id": int(rid),
                    "pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
                    "routes": [],
                    "peers": [],
                    "transit_queue": 0,
                    "last_seen": time.time(),
                    "memory_bytes": 0,
                    "capacity_bytes": 0,
                    "rss_bytes": 0,
                    "pressure_level": "Normal",
                    "budget_bytes": 0,
                    "nodes_compacted": 0,
                    "landmarks_evicted": 0,
                    "exploration_target": {"x": 0.0, "y": 0.0, "frontier_id": 0},
                    "planner_status": "StationKeepingFallback",
                    "planner_utility": 0.0,
                    "frontiers_completed": 0,
                    "stats": {}
                }

            r = self.swarm_state["rovers"][rid]
            r["last_seen"] = time.time()

            if "pose" in event:
                r["pose"] = event["pose"]
            if "routes" in event:
                r["routes"] = event["routes"]
            if "peers" in event:
                r["peers"] = event["peers"]
            if "transit_queue" in event:
                r["transit_queue"] = event["transit_queue"]
            if "version_vector" in event:
                r["version_vector"] = event["version_vector"]
            if "stats" in event:
                r["stats"] = event["stats"]

        # Aggregate global metrics
        gm = self.swarm_state["global_metrics"]
        if etype == "forward":
            gm["packets_forwarded"] += 1
        elif etype == "loop_suppressed":
            gm["routing_loops_suppressed"] += 1
        elif etype == "ack":
            if event.get("kind") == "custody":
                gm["custody_acks"] += 1
            else:
                gm["e2e_acks"] += 1
        elif etype == "loop_closure":
            gm["loop_closures_verified"] += 1
        elif etype == "partition":
            self.swarm_state["partitions"].append(event)
        elif etype == "consistency_audit":
            gm["dsu_components"] = event.get("dsu_components", gm["dsu_components"])
            gm["is_converged"] = event.get("is_converged", gm["is_converged"])
            gm["reciprocal_edge_fraction"] = event.get("reciprocal_edge_fraction", gm["reciprocal_edge_fraction"])
        elif etype == "adaptive_comms":
            gm["adaptive_regime"] = event.get("regime", gm["adaptive_regime"])
        elif etype == "gauge_alignment":
            gm["elected_anchor"] = event.get("anchor_id", gm.get("elected_anchor", 1))
            gm["cycles_remediated"] += event.get("cycles_remediated", 0)
            if "global_ate_m" in event:
                gm["global_ate_m"] = event["global_ate_m"]
            if "anchor_id" in event and rid != "0":
                r_state = self.swarm_state["rovers"][rid]
                r_state["anchor_id"] = event["anchor_id"]
                r_state["is_anchor"] = (event["anchor_id"] == int(rid))
                r_state["transform_from_anchor"] = event.get("transform_from_anchor", {"x": 0.0, "y": 0.0, "theta": 0.0})
        elif etype == "active_loop_closure":
            gm["active_opportunities_evaluated"] += event.get("evaluated", 0)
            gm["active_opportunities_scheduled"] += event.get("scheduled", 0)
            gm["active_wire_bytes_spent"] += event.get("wire_bytes", 0)
            gm["rendezvous_proposals_verified"] += event.get("verified", 0)
            if "global_ate_m" in event:
                gm["global_ate_m"] = event["global_ate_m"]
        elif etype == "memory_compaction":
            gm["total_nodes_pruned"] += event.get("nodes_pruned", 0)
            gm["total_landmarks_evicted"] += event.get("landmarks_evicted", 0)
            gm["total_compaction_wire_bytes"] += event.get("wire_bytes", 0)
            gm["total_compaction_notifications"] += event.get("notifications_sent", 1)
            if rid != "0" and rid in self.swarm_state["rovers"]:
                r_state = self.swarm_state["rovers"][rid]
                r_state["nodes_compacted"] += event.get("nodes_pruned", 0)
                r_state["landmarks_evicted"] += event.get("landmarks_evicted", 0)
                if "current_memory_bytes" in event:
                    r_state["memory_bytes"] = event["current_memory_bytes"]
                if "pressure_level" in event:
                    r_state["pressure_level"] = event["pressure_level"]
        elif etype == "memory_pressure":
            if rid != "0" and rid in self.swarm_state["rovers"]:
                r_state = self.swarm_state["rovers"][rid]
                r_state["memory_bytes"] = event.get("current_usage_bytes", r_state["memory_bytes"])
                r_state["capacity_bytes"] = event.get("capacity_bytes", r_state["capacity_bytes"])
                r_state["rss_bytes"] = event.get("rss_bytes", r_state["rss_bytes"])
                r_state["pressure_level"] = event.get("pressure_level", r_state["pressure_level"])
                r_state["budget_bytes"] = event.get("budget_bytes", r_state["budget_bytes"])
            plevel = event.get("pressure_level", "Normal")
            if plevel in ("Warning", "Critical"):
                gm["peak_swarm_pressure"] = plevel
            if "swarm_logical_bytes" in event:
                gm["current_swarm_logical_bytes"] = event["swarm_logical_bytes"]
            if "swarm_rss_bytes" in event:
                gm["current_swarm_rss_bytes"] = event["swarm_rss_bytes"]
        elif etype == "exploration_decision":
            if rid != "0" and rid in self.swarm_state["rovers"]:
                r_state = self.swarm_state["rovers"][rid]
                r_state["planner_status"] = event.get("status", r_state["planner_status"])
                r_state["planner_utility"] = event.get("utility", r_state["planner_utility"])
                if "target" in event:
                    r_state["exploration_target"] = event["target"]
                if "completed_delta" in event:
                    r_state["frontiers_completed"] += event["completed_delta"]
            gm["total_frontiers_completed"] += event.get("completed_delta", 0)
            if "total_detected" in event:
                gm["total_frontiers_detected"] = event["total_detected"]
            if "active_frontiers" in event:
                gm["active_frontiers_count"] = event["active_frontiers"]
            if "coverage_percent" in event:
                gm["swarm_coverage_percent"] = event["coverage_percent"]
            if "uptime_percent" in event:
                gm["connectivity_uptime_percent"] = event["uptime_percent"]
        elif etype == "frontier_claim":
            if "active_frontiers" in event:
                gm["active_frontiers_count"] = event["active_frontiers"]

        # Re-derive active links & multi-hop paths
        active_links = []
        multihop_routes = []
        now = time.time()

        for rovid, rdata in self.swarm_state["rovers"].items():
            if now - rdata.get("last_seen", 0) > 10.0:
                continue
            src = int(rovid)
            for p in rdata.get("peers", []):
                link = sorted([src, p])
                if link not in active_links:
                    active_links.append(link)

            for route in rdata.get("routes", []):
                dest = route.get("dest")
                next_hop = route.get("next_hop")
                metric = route.get("metric", 1)
                if dest and next_hop and metric > 1:
                    multihop_routes.append({
                        "origin": src,
                        "dest": dest,
                        "next_hop": next_hop,
                        "metric": metric
                    })

        self.swarm_state["active_links"] = active_links
        self.swarm_state["multihop_routes"] = multihop_routes
        self.swarm_state["timestamp"] = now

    def get_snapshot(self) -> Dict[str, Any]:
        """Returns deep copy of current state snapshot."""
        with self.lock:
            snap = json.loads(json.dumps(self.swarm_state))
            snap["severed_links"] = [list(p) for p in self.injected_partitions]
            snap["is_partitioned"] = len(self.injected_partitions) > 0
            return snap

    def inject_partition(self, rover_a: int = 0, rover_b: int = 0, partition_all: bool = False) -> Dict[str, Any]:
        """Manually severs links to simulate a network partition."""
        with self.lock:
            if partition_all or (rover_a == 0 and rover_b == 0):
                rids = sorted([int(r) for r in self.swarm_state["rovers"].keys()])
                if len(rids) >= 2:
                    mid = len(rids) // 2
                    grp1 = rids[:mid]
                    grp2 = rids[mid:]
                    for a in grp1:
                        for b in grp2:
                            self.injected_partitions.add(tuple(sorted((a, b))))
                else:
                    self.injected_partitions.add((1, 2))
            else:
                self.injected_partitions.add(tuple(sorted((int(rover_a), int(rover_b)))))
            
            severed = [list(p) for p in self.injected_partitions]
            self.swarm_state["is_partitioned"] = True
            self.swarm_state["severed_links"] = severed
            event = {
                "type": "partition_injected",
                "action": "sever",
                "rover_a": rover_a,
                "rover_b": rover_b,
                "partition_all": partition_all,
                "active_partitions": severed,
                "timestamp": time.time()
            }
        self.record_event(event)
        return {"status": "ok", "active_partitions": severed, "is_partitioned": True}

    def heal_partitions(self) -> Dict[str, Any]:
        """Restores severed links and simulates topological healing."""
        with self.lock:
            self.injected_partitions.clear()
            self.swarm_state["is_partitioned"] = False
            self.swarm_state["severed_links"] = []
            event = {
                "type": "partition_injected",
                "action": "heal_all",
                "active_partitions": [],
                "timestamp": time.time()
            }
        self.record_event(event)
        return {"status": "ok", "active_partitions": [], "is_partitioned": False}

    def get_partition_status(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "is_partitioned": len(self.injected_partitions) > 0,
                "severed_links": [list(p) for p in self.injected_partitions],
                "count": len(self.injected_partitions)
            }

    def subscribe(self) -> queue.Queue:
        """Subscribes an SSE consumer queue."""
        q = queue.Queue(maxsize=100)
        with self.lock:
            self.subscribers.append(q)
        return q

    def unsubscribe(self, q: queue.Queue):
        """Unsubscribes an SSE consumer queue."""
        with self.lock:
            if q in self.subscribers:
                self.subscribers.remove(q)

# Global collector instance
_COLLECTOR: Optional[TelemetryCollector] = None

def get_telemetry_collector() -> TelemetryCollector:
    global _COLLECTOR
    if _COLLECTOR is None:
        _COLLECTOR = TelemetryCollector()
        _COLLECTOR.start()
    return _COLLECTOR

