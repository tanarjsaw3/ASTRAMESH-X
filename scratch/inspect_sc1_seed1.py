import csv

with open('benchmark_phase61_trials.csv', 'r', encoding='utf-8') as f:
    trials = list(csv.DictReader(f))

sc1 = [t for t in trials if t['scenario_id'] == 'SC1_Nominal_2R' and int(t['seed']) == 1042]
for t in sc1:
    print(f"{t['variant']:<25} | RawATE:{float(t['ate_raw_rmse_m']):.6f} | AlignedATE:{float(t['ate_aligned_swarm_rmse_m']):.6f} | Iters:{t['optimizer_iterations']} | Closures:{t['closures_verified']}")

