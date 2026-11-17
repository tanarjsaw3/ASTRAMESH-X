import csv

with open('benchmark_phase61_trials.csv') as f:
    rows = list(csv.DictReader(f))

sc10 = [r for r in rows if r['scenario_id'] == 'SC10_Swarm_8R']
seeds_sc10 = len(set(r['seed'] for r in sc10))

print(f"Total completed trials: {len(rows)}/2100 ({(len(rows)/2100)*100:.1f}%)")
print(f"SC10 completed: {len(sc10)}/210 runs ({(len(sc10)/210)*100:.1f}%) across {seeds_sc10}/30 seeds")

