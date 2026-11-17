import urllib.request
import json
import time
import subprocess
import sys

def test_server():
    print("Testing server startup...")
    proc = subprocess.Popen([sys.executable, "app/server.py"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(1.5)

    base_url = "http://127.0.0.1:8080"
    try:
        # 1. Test status
        req = urllib.request.urlopen(f"{base_url}/api/status")
        status = json.loads(req.read().decode("utf-8"))
        print("API Status:", status)
        assert status["status"] == "online"
        assert status["engine_available"] == True

        # 2. Test Demo 1
        req = urllib.request.urlopen(f"{base_url}/api/demo/1")
        demo1 = json.loads(req.read().decode("utf-8"))
        print("Demo 1 rovers:", demo1["config"]["rovers"], "frames:", len(demo1["frames"]))
        assert demo1["config"]["rovers"] == 2
        assert len(demo1["frames"]) > 0

        # 3. Test benchmarks data
        req = urllib.request.urlopen(f"{base_url}/api/benchmarks")
        bench = json.loads(req.read().decode("utf-8"))
        print("Benchmarks keys:", list(bench.keys()))
        assert "sc1_wire_by_delay" in bench

        # 4. Test POST /api/run
        post_data = json.dumps({
            "rovers": 2,
            "scenario": "ConcentricLoops",
            "duration": 4.0,
            "loss": 0.0,
            "delay_ms": 0,
            "variant": "AstroMesh_X_Full",
            "seed": 99,
            "opt_steps": 5
        }).encode("utf-8")
        
        req = urllib.request.Request(f"{base_url}/api/run", data=post_data, headers={"Content-Type": "application/json"})
        resp = urllib.request.urlopen(req)
        run_res = json.loads(resp.read().decode("utf-8"))
        print("Custom run completed:", run_res["summary"])
        assert run_res["summary"]["optimization_ok"] == True

        # 5. Test index.html
        req = urllib.request.urlopen(f"{base_url}/")
        html = req.read().decode("utf-8")
        assert "ASTRAMESH-X" in html
        print("UI HTML delivered successfully! Size:", len(html))

        print("\nALL SERVER & API TESTS PASSED PERFECTLY!")
    finally:
        proc.terminate()
        proc.wait()

if __name__ == "__main__":
    test_server()

