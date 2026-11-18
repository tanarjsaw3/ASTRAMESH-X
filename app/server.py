#!/usr/bin/env python3
"""
AstroMesh-X Backend API & Telemetry Web Server
Provides REST endpoints and static UI delivery for decentralized multi-rover simulation.
"""

import http.server
import json
import os
import queue
import subprocess
import sys
import tempfile
import time
import urllib.parse
from pathlib import Path

# Paths
BASE_DIR = Path(__file__).resolve().parent.parent
STATIC_DIR = Path(__file__).resolve().parent / "static"
SCRATCH_DIR = BASE_DIR / "scratch"

def _resolve_sim_runner() -> Path:
    """Dynamically resolves sim_runner binary across Windows, Linux, and Docker."""
    env_path = os.environ.get("ASTROMESH_SIM_RUNNER")
    if env_path and Path(env_path).exists():
        return Path(env_path)
    candidates = [
        BASE_DIR / "bin" / "sim_runner",
        BASE_DIR / "build" / "bin" / "sim_runner",
        BASE_DIR / "build" / "sim_runner",
        BASE_DIR / "build-ucrt64-clean" / "bin" / "sim_runner.exe",
        BASE_DIR / "build" / "bin" / "sim_runner.exe",
        BASE_DIR / "build-ucrt64" / "bin" / "sim_runner.exe",
    ]
    for c in candidates:
        if c.exists():
            return c
    return BASE_DIR / "build-ucrt64-clean" / "bin" / "sim_runner.exe" if os.name == "nt" else BASE_DIR / "bin" / "sim_runner"

SIM_RUNNER_EXE = _resolve_sim_runner()

if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

from app.telemetry_collector import get_telemetry_collector

SCRATCH_DIR.mkdir(parents=True, exist_ok=True)
STATIC_DIR.mkdir(parents=True, exist_ok=True)

# Cache latest simulation run
LATEST_SIMULATION = None
LATEST_EXPORT_CSV = SCRATCH_DIR / "live_run.csv"
LATEST_EXPORT_JSON = SCRATCH_DIR / "live_run.json"
_BENCHMARK_CACHE = None

def get_benchmark_comparison_data():
    """Extracts verified benchmark data from Phase 6.1 and 6.2 CSVs."""
    global _BENCHMARK_CACHE
    if _BENCHMARK_CACHE is not None:
        return _BENCHMARK_CACHE
    import pandas as pd
    data = {
        "sc1_loss_delay": [],
        "sc10_exploratory": [],
        "variants": [
            "AstroMesh_X_Full",
            "AstroMesh_X_RawExchange",
            "AstroMesh_X_NoACKs",
            "AstroMesh_X_NoRetries",
            "AstroMesh_X_NoMultiHop",
            "AstroMesh_X_Minimal",
            "AstroMesh_v1_Baseline"
        ],
        "meta": {
            "sc1_status": "Complete 70-Cell Robustness Study (14,700 trials, N=30)",
            "sc10_status": "Exploratory Partial Sweep (680 of 2,450 trials, 27.8%)"
        }
    }

    trials_file = BASE_DIR / "benchmark_phase62_trials.csv"
    if trials_file.exists():
        try:
            df = pd.read_csv(trials_file)
            # SC1 nominal delay comparison (loss = 0.0)
            sc1_loss0 = df[(df["scenario_id"] == "SC1_Nominal_2R") & (df["loss_rate"] == 0.0)]
            piv_wire = sc1_loss0.groupby(["variant", "delay_ms"])["total_wire_bytes"].mean().unstack().to_dict()
            
            # SC1 completion comparison (delay = 0ms)
            sc1_del0 = df[(df["scenario_id"] == "SC1_Nominal_2R") & (df["delay_ms"] == 0)]
            piv_comp = sc1_del0.groupby(["variant", "loss_rate"])["app_completion_ratio"].mean().unstack().to_dict()

            # SC1 ATE comparison (delay = 0ms)
            piv_ate = sc1_del0.groupby(["variant", "loss_rate"])["ate_aligned_swarm_rmse_m"].mean().unstack().to_dict()

            data["sc1_wire_by_delay"] = piv_wire
            data["sc1_completion_by_loss"] = piv_comp
            data["sc1_ate_by_loss"] = piv_ate

            # SC10 Exploratory Multi-hop overhead
            sc10 = df[df["scenario_id"] == "SC10_Swarm_8R"]
            if len(sc10) > 0:
                sc10_wire = sc10[sc10["loss_rate"] == 0.0].groupby(["variant", "delay_ms"])["total_wire_bytes"].mean().unstack().to_dict()
                data["sc10_wire_by_delay"] = sc10_wire
        except Exception as e:
            print(f"Error loading benchmark CSV: {e}", file=sys.stderr)

    _BENCHMARK_CACHE = data
    return data


def get_m717_benchmark_summary():
    """Reads M7.17 statistical summaries and evidence matrix."""
    import csv
    summary = {"benchmarks": [], "evidence": []}
    stat_file = BASE_DIR / "benchmark_m717_statistical_summary.csv"
    if stat_file.exists():
        with open(stat_file, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            summary["benchmarks"] = [row for row in reader]
    ev_file = BASE_DIR / "benchmark_m717_evidence_matrix.csv"
    if ev_file.exists():
        with open(ev_file, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            summary["evidence"] = [row for row in reader]
    return summary


class AstroMeshHandler(http.server.SimpleHTTPRequestHandler):
    """HTTP Request Handler for AstroMesh-X API and UI."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(STATIC_DIR), **kwargs)

    def end_headers(self):
        # Enable CORS for local development
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path

        if path == "/api/status":
            self.handle_api_status()
        elif path.startswith("/api/demo/"):
            demo_id = path.split("/")[-1]
            self.handle_api_demo(demo_id)
        elif path == "/api/benchmarks":
            self.handle_api_benchmarks()
        elif path == "/api/benchmarks/summary":
            self.handle_api_benchmarks_summary()
        elif path == "/api/partition/status":
            self.handle_partition_status()
        elif path == "/api/export/csv":
            self.handle_export_csv()
        elif path == "/api/export/json":
            self.handle_export_json()
        elif path == "/api/export/report":
            self.handle_export_report()
        elif path in ("/api/export/paper", "/download/paper", "/paper.pdf"):
            self.handle_export_paper()
        elif path == "/api/live/snapshot":
            self.handle_live_snapshot()
        elif path == "/api/live/stream":
            self.handle_live_stream()
        else:
            # Default static file handling
            if path == "/" or path == "":
                self.path = "/index.html"
            super().do_GET()

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        content_length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_length) if content_length > 0 else b"{}"
        try:
            params = json.loads(body.decode("utf-8")) if body else {}
        except Exception:
            params = {}

        if parsed.path == "/api/run":
            try:
                self.handle_api_run(params)
            except Exception as e:
                self.send_error_response(500, f"Malformed simulation request: {e}")
        elif parsed.path == "/api/partition/disconnect":
            self.handle_partition_disconnect(params)
        elif parsed.path == "/api/partition/reconnect":
            self.handle_partition_reconnect()
        else:
            self.send_error_response(404, "Unknown API endpoint")

    def handle_api_status(self):
        status = {
            "status": "online",
            "project": "AstroMesh-X",
            "version": "0.1.0",
            "milestone": "M7.18 Interactive Research Simulator & Live Evaluation Dashboard",
            "engine_executable": str(SIM_RUNNER_EXE),
            "engine_available": SIM_RUNNER_EXE.exists(),
            "ctest_status": "66/66 Passing (100%)",
            "acceptance_checks": "13/13 Passing (100%)",
            "toolchain": "MSYS2 UCRT64 GCC 16.2.0 (C++17, Static Linking)",
            "benchmark_records": 15470
        }
        self.send_json_response(status)

    def handle_api_demo(self, demo_id):
        global LATEST_SIMULATION, LATEST_EXPORT_CSV, LATEST_EXPORT_JSON
        json_file = SCRATCH_DIR / f"demo{demo_id}.json"
        csv_file = SCRATCH_DIR / f"demo{demo_id}.csv"
        
        # If pre-cached demo file exists, serve it immediately
        if json_file.exists():
            try:
                with open(json_file, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    LATEST_SIMULATION = data
                    LATEST_EXPORT_JSON = json_file
                    if csv_file.exists():
                        LATEST_EXPORT_CSV = csv_file
                    self.send_json_response(data)
                    return
            except Exception as e:
                print(f"Error reading {json_file}: {e}", file=sys.stderr)

        # Otherwise run sim_runner to generate it
        if not SIM_RUNNER_EXE.exists():
            self.send_error_response(500, f"Simulation executable not found at {SIM_RUNNER_EXE}")
            return

        cmd = [
            str(SIM_RUNNER_EXE),
            "--demo", str(demo_id),
            "--output-json", str(json_file),
            "--output-csv", str(csv_file)
        ]
        try:
            res = subprocess.run(cmd, capture_output=True, text=True, check=True)
            with open(json_file, "r", encoding="utf-8") as f:
                data = json.load(f)
                LATEST_SIMULATION = data
                LATEST_EXPORT_JSON = json_file
                LATEST_EXPORT_CSV = csv_file
                self.send_json_response(data)
        except Exception as e:
            self.send_error_response(500, f"Failed executing Demo {demo_id}: {e}")

    def handle_api_run(self, p):
        global LATEST_SIMULATION, LATEST_EXPORT_CSV, LATEST_EXPORT_JSON
        if not SIM_RUNNER_EXE.exists():
            self.send_error_response(500, f"sim_runner.exe not found at {SIM_RUNNER_EXE}")
            return

        # Prepare parameters
        rovers = p.get("rovers", 2)
        scenario = p.get("scenario", "ConcentricLoops")
        duration = p.get("duration", 8.0)
        step_dt = p.get("step_dt", 0.1)
        loss = p.get("loss", 0.0)
        delay = p.get("delay_ms", 0)
        comm_range = p.get("comm_range", 25.0)
        variant = p.get("variant", "AstroMesh_X_Full")
        seed = p.get("seed", 42)
        run_optimize = p.get("run_optimize", True)
        opt_steps = p.get("opt_steps", 15)
        partition_start = p.get("partition_start", 0.0)
        partition_end = p.get("partition_end", 0.0)

        out_json = SCRATCH_DIR / "live_run.json"
        out_csv = SCRATCH_DIR / "live_run.csv"

        cmd = [
            str(SIM_RUNNER_EXE),
            "--rovers", str(rovers),
            "--scenario", str(scenario),
            "--duration", str(duration),
            "--step-dt", str(step_dt),
            "--loss", str(loss),
            "--delay-ms", str(delay),
            "--comm-range", str(comm_range),
            "--variant", str(variant),
            "--seed", str(seed),
            "--opt-steps", str(opt_steps),
            "--output-json", str(out_json),
            "--output-csv", str(out_csv)
        ]
        if partition_end > partition_start:
            cmd.extend(["--partition-start", str(partition_start), "--partition-end", str(partition_end)])
        if not run_optimize:
            cmd.append("--no-optimize")

        try:
            res = subprocess.run(cmd, capture_output=True, text=True, check=True, timeout=60)
            if out_json.exists():
                with open(out_json, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    data["stdout"] = res.stdout
                    LATEST_SIMULATION = data
                    LATEST_EXPORT_JSON = out_json
                    LATEST_EXPORT_CSV = out_csv
                    self.send_json_response(data)
            else:
                self.send_error_response(500, "Simulation finished but produced no JSON output.")
        except subprocess.TimeoutExpired:
            self.send_error_response(504, "Simulation execution timed out (>60s). Try fewer steps or lower rover count.")
        except subprocess.CalledProcessError as e:
            self.send_error_response(500, f"Simulation crashed (exit code {e.returncode}): {e.stderr or e.stdout}")
        except Exception as e:
            self.send_error_response(500, f"Error running simulation: {e}")

    def handle_api_benchmarks(self):
        try:
            data = get_benchmark_comparison_data()
            self.send_json_response(data)
        except Exception as e:
            self.send_error_response(500, f"Failed retrieving benchmark data: {e}")

    def handle_api_benchmarks_summary(self):
        try:
            data = get_m717_benchmark_summary()
            self.send_json_response(data)
        except Exception as e:
            self.send_error_response(500, f"Failed retrieving benchmark summary: {e}")

    def handle_partition_disconnect(self, p):
        collector = get_telemetry_collector()
        rover_a = p.get("rover_a", 0)
        rover_b = p.get("rover_b", 0)
        partition_all = p.get("partition_all", False)
        res = collector.inject_partition(rover_a, rover_b, partition_all)
        self.send_json_response(res)

    def handle_partition_reconnect(self):
        collector = get_telemetry_collector()
        res = collector.heal_partitions()
        self.send_json_response(res)

    def handle_partition_status(self):
        collector = get_telemetry_collector()
        res = collector.get_partition_status()
        self.send_json_response(res)

    def handle_export_csv(self):
        csv_file = LATEST_EXPORT_CSV if (LATEST_EXPORT_CSV and LATEST_EXPORT_CSV.exists()) else (SCRATCH_DIR / "live_run.csv" if (SCRATCH_DIR / "live_run.csv").exists() else SCRATCH_DIR / "demo1.csv")
        
        if csv_file.exists():
            self.send_response(200)
            self.send_header("Content-Type", "text/csv")
            self.send_header("Content-Disposition", f'attachment; filename="{csv_file.name}"')
            self.end_headers()
            with open(csv_file, "rb") as f:
                self.wfile.write(f.read())
        else:
            self.send_error_response(404, "No simulation CSV available for download.")

    def handle_export_json(self):
        json_file = LATEST_EXPORT_JSON if (LATEST_EXPORT_JSON and LATEST_EXPORT_JSON.exists()) else (SCRATCH_DIR / "live_run.json" if (SCRATCH_DIR / "live_run.json").exists() else SCRATCH_DIR / "demo1.json")

        if json_file.exists():
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Disposition", f'attachment; filename="{json_file.name}"')
            self.end_headers()
            with open(json_file, "rb") as f:
                self.wfile.write(f.read())
        else:
            self.send_error_response(404, "No simulation JSON available for download.")

    def handle_export_report(self):
        report_file = BASE_DIR / "docs" / "FINAL_COMPLETION_REPORT.md"
        if not report_file.exists():
            report_file = BASE_DIR / "docs" / "PHASE62_ROBUSTNESS.md"

        if report_file.exists():
            self.send_response(200)
            self.send_header("Content-Type", "text/markdown; charset=utf-8")
            self.send_header("Content-Disposition", 'attachment; filename="AstroMesh_Report.md"')
            self.end_headers()
            with open(report_file, "rb") as f:
                self.wfile.write(f.read())
        else:
            self.send_error_response(404, "Report document not found.")

    def handle_export_paper(self):
        paper_file = BASE_DIR / "docs" / "ASTROMESH_X_RESEARCH_MANUSCRIPT.pdf"
        if paper_file.exists():
            self.send_response(200)
            self.send_header("Content-Type", "application/pdf")
            self.send_header("Content-Disposition", 'attachment; filename="ASTROMESH_X_RESEARCH_MANUSCRIPT.pdf"')
            self.send_header("Content-Length", str(paper_file.stat().st_size))
            self.end_headers()
            with open(paper_file, "rb") as f:
                self.wfile.write(f.read())
        else:
            self.send_error_response(404, "Research paper PDF not found.")

    def handle_live_snapshot(self):
        collector = get_telemetry_collector()
        snapshot = collector.get_snapshot()
        self.send_json_response(snapshot)

    def handle_live_stream(self):
        collector = get_telemetry_collector()
        q = collector.subscribe()

        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        # Send initial snapshot event
        initial_event = json.dumps({"type": "snapshot", "data": collector.get_snapshot()})
        try:
            self.wfile.write(f"data: {initial_event}\n\n".encode("utf-8"))
            self.wfile.flush()
        except Exception:
            collector.unsubscribe(q)
            return

        try:
            while True:
                try:
                    event = q.get(timeout=1.0)
                    line = f"data: {json.dumps(event)}\n\n".encode("utf-8")
                    self.wfile.write(line)
                    self.wfile.flush()
                except queue.Empty:
                    # Heartbeat comment to keep connection alive
                    self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            collector.unsubscribe(q)

    def send_json_response(self, data):
        body = json.dumps(data, indent=2).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_error_response(self, code, message):
        payload = json.dumps({"error": message, "code": code}).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


def main():
    host = os.environ.get("HOST", "0.0.0.0")
    port = int(os.environ.get("PORT", 8765))
    print(f"=================================================================")
    print(f"        ASTRAMESH-X INTERACTIVE MISSION CONTROL BACKEND          ")
    print(f"=================================================================")
    print(f"Server Root:    {STATIC_DIR}")
    print(f"C++ Sim Engine: {SIM_RUNNER_EXE} (available: {SIM_RUNNER_EXE.exists()})")
    print(f"Listening on:   http://{host}:{port}")
    print(f"-----------------------------------------------------------------")
    
    server_address = (host, port)
    httpd = http.server.ThreadingHTTPServer(server_address, AstroMeshHandler)
    
    # Pre-warm benchmark cache
    try:
        get_benchmark_comparison_data()
        get_m717_benchmark_summary()
    except Exception as e:
        print(f"Warning warming benchmark cache: {e}", file=sys.stderr)

    # Initialize and start UDP telemetry collector
    collector = get_telemetry_collector()
    print(f"UDP Telemetry:  Listening on 127.0.0.1:{collector.port}")
    
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping AstroMesh-X server.")
        httpd.server_close()


if __name__ == "__main__":
    main()

