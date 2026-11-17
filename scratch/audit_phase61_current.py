import csv
import math

def audit():
    raw_path = 'benchmark_phase61_trials.csv'
    with open(raw_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    total_lines = len(lines)
    header = lines[0].strip().split(',')
    data_rows = [l.strip().split(',') for l in lines[1:] if l.strip()]
    
    print(f"=== File & Line Accounting ===")
    print(f"Total raw lines in {raw_path}: {total_lines}")
    print(f"Header line present: {'scenario_id' in header[0]}")
    print(f"Header column count: {len(header)}")
    print(f"Data rows parsed: {len(data_rows)}")
    print(f"Explanation of '2,101 rows': Exactly 1 header row + 2,100 data rows = 2,101 total lines in complete CSV.\n")

    # Check column count consistency
    col_mismatches = []
    for idx, row in enumerate(data_rows):
        if len(row) != len(header):
            col_mismatches.append((idx + 2, len(row)))
    print(f"Malformed or truncated rows (column count != {len(header)}): {len(col_mismatches)}")
    if col_mismatches:
        print(f"First 5 malformed lines: {col_mismatches[:5]}")

    # Parse as DictReader
    with open(raw_path, 'r', encoding='utf-8') as f:
        reader = list(csv.DictReader(f))

    # Scenario and seed counts
    scenario_counts = {}
    seed_set = set()
    variant_set = set()
    pairs = set()
    duplicates = []

    for r in reader:
        sc = r['scenario_id']
        sd = r['seed']
        v = r['variant']
        scenario_counts[sc] = scenario_counts.get(sc, 0) + 1
        seed_set.add(sd)
        variant_set.add(v)
        key = (sc, sd, v)
        if key in pairs:
            duplicates.append(key)
        pairs.add(key)

    print(f"\n=== Dataset Matrix Breakdown ===")
    print(f"Unique scenarios: {len(scenario_counts)}")
    print(f"Unique seeds observed: {len(seed_set)}")
    print(f"Unique variants observed: {len(variant_set)} -> {sorted(list(variant_set))}")
    print(f"Duplicate (scenario, seed, variant) tuples: {len(duplicates)}")
    print("Completed trial counts per scenario:")
    for sc, count in sorted(scenario_counts.items()):
        print(f"  {sc:<28}: {count:>4} / 210 runs ({(count/210)*100:>5.1f}%)")

    # Invariant Verification
    inv_total_wire_failures = 0
    inv_data_wire_failures = 0
    inv_app_ratio_failures = 0
    inv_hop_ratio_failures = 0
    reported_trial_failures = 0

    for idx, r in enumerate(reader):
        tot_wire = float(r['total_wire_bytes'])
        data_wire = float(r['data_wire_bytes'])
        ack_wire = float(r['ack_wire_bytes'])
        payload = float(r['payload_bytes'])
        protocol = float(r['protocol_bytes'])
        framing = float(r['framing_bytes'])
        app_ratio = float(r['app_completion_ratio'])
        hop_ratio = float(r['per_hop_delivery_ratio'])
        status = r['trial_status']

        if status != 'Success':
            reported_trial_failures += 1

        # Check total wire byte conservation: total == data + ack
        if abs(tot_wire - (data_wire + ack_wire)) > 1e-3:
            inv_total_wire_failures += 1
            if inv_total_wire_failures <= 3:
                print(f"Row {idx+2}: Wire Conservation Failure: total={tot_wire} != data({data_wire}) + ack({ack_wire})")

        # Check data wire conservation: data == payload + protocol + framing
        if abs(data_wire - (payload + protocol + framing)) > 1e-3:
            inv_data_wire_failures += 1
            if inv_data_wire_failures <= 3:
                print(f"Row {idx+2}: Data Wire Conservation Failure: data={data_wire} != payload({payload}) + proto({protocol}) + frame({framing})")

        # Check completion ratio bounds [0, 1]
        if not (-1e-5 <= app_ratio <= 1.00001):
            inv_app_ratio_failures += 1
        
        # Check hop delivery ratio bounds [0, 1]
        if not (-1e-5 <= hop_ratio <= 1.00001):
            inv_hop_ratio_failures += 1

    print(f"\n=== Invariant & Accounting Audit ({len(reader)} completed runs) ===")
    print(f"Total Wire Byte Conservation Violations: {inv_total_wire_failures}")
    print(f"Data Wire Byte Conservation Violations:  {inv_data_wire_failures}")
    print(f"Application Completion Ratio Violations: {inv_app_ratio_failures}")
    print(f"Per-Hop Delivery Ratio Violations:       {inv_hop_ratio_failures}")
    print(f"Reported Non-Success Trials:             {reported_trial_failures}")

if __name__ == '__main__':
    audit()

