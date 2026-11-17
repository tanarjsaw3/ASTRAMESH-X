import csv

with open('benchmark_phase61_trials.csv') as f:
    rows = list(csv.DictReader(f))

print(f"Total rows read: {len(rows)}")
print("Latest 10 rows:")
for r in rows[-10:]:
    ms = float(r['runtime_us']) / 1000.0
    print(f"Seed {r['seed']:>2} | {r['scenario_id']:<15} | {r['variant']:<25} | Time: {ms:>7.1f} ms | Wire: {float(r['total_wire_bytes']):>10.0f} B | Status: {r['trial_status']}")
