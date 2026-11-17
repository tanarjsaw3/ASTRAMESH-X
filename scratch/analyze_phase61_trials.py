import csv
import math

def analyze():
    with open('benchmark_phase61_trials.csv') as f:
        trials = list(csv.DictReader(f))
    print(f"Total trial rows: {len(trials)}")

    # Check invariants
    inv_violations = 0
    for i, t in enumerate(trials):
        tot = float(t['wire_bytes_total'])
        data = float(t['wire_bytes_data'])
        ack = float(t['wire_bytes_ack'])
        payload = float(t['payload_bytes_total'])
        framing = float(t['framing_bytes_total'])
        app_comp = float(t['application_completion_ratio'])
        hop_del = float(t['per_hop_delivery_ratio'])

        if abs(tot - (data + ack)) > 1e-4:
            print(f"Row {i}: Total wire bytes mismatch: {tot} != {data} + {ack}")
            inv_violations += 1
        if abs(data - (payload + framing)) > 1e-4:
            print(f"Row {i}: Data wire bytes mismatch: {data} != {payload} + {framing}")
            inv_violations += 1
        if not (-1e-4 <= app_comp <= 1.0 + 1e-4):
            print(f"Row {i}: App completion out of bounds: {app_comp}")
            inv_violations += 1
        if not (-1e-4 <= hop_del <= 1.0 + 1e-4):
            print(f"Row {i}: Hop delivery out of bounds: {hop_del}")
            inv_violations += 1
    
    print(f"Invariant violations: {inv_violations}")

    # Check summary
    with open('benchmark_phase61_summary.csv') as f:
        summary = list(csv.DictReader(f))
    print(f"Summary rows: {len(summary)}")

    # Check failures
    with open('benchmark_phase61_failures.csv') as f:
        failures = list(csv.DictReader(f))
    print(f"Recorded failures: {len(failures)}")

if __name__ == '__main__':
    analyze()

