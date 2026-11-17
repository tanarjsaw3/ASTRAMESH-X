import csv
import numpy as np

with open('benchmark_phase61_trials.csv', 'r', encoding='utf-8') as f:
    trials = list(csv.DictReader(f))

for sc in ['SC1_Nominal_2R', 'SC7_DegenerateGeometry_2R', 'SC9_Swarm_4R', 'SC10_Swarm_8R']:
    print(f"\n=== Scenario {sc} ===")
    for v in ['AstroMesh_X_Full', 'AstroMesh_X_RawExchange', 'AstroMesh_v1_Baseline']:
        t_sc = [t for t in trials if t['scenario_id'] == sc and t['variant'] == v]
        cands = [float(t['candidates_generated']) for t in t_sc]
        ver = [float(t['closures_verified']) for t in t_sc]
        rej = [float(t['closures_rejected']) for t in t_sc]
        opt_calls = [float(t['optimizer_calls']) for t in t_sc]
        opt_iters = [float(t['optimizer_iterations']) for t in t_sc]
        ate = [float(t['ate_aligned_swarm_rmse_m']) for t in t_sc]
        print(f"  {v:<25} | Cands:{np.mean(cands):>7.1f} | Ver:{np.mean(ver):>5.1f} | Rej:{np.mean(rej):>5.1f} | OptCalls:{np.mean(opt_calls):.1f} | OptIters:{np.mean(opt_iters):>5.1f} | ATE:{np.mean(ate):.4f}m")
