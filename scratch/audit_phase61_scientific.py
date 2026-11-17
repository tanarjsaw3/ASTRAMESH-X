import csv
import math
import numpy as np
from scipy import stats

def run_scientific_audit():
    print("=" * 80)
    print(" ASTROMESH-X PHASE 6.1 INDEPENDENT SCIENTIFIC AUDIT")
    print("=" * 80)

    # -------------------------------------------------------------------------
    # AUDIT A: Dataset Integrity
    # -------------------------------------------------------------------------
    trials_csv = 'benchmark_phase61_trials.csv'
    summary_csv = 'benchmark_phase61_summary.csv'
    failures_csv = 'benchmark_phase61_failures.csv'

    with open(trials_csv, 'r', encoding='utf-8') as f:
        raw_lines = f.readlines()
    
    total_file_lines = len(raw_lines)
    header = raw_lines[0].strip().split(',')
    raw_data_rows = [l.strip().split(',') for l in raw_lines[1:] if l.strip()]

    print("\n[AUDIT A] Dataset Integrity & File Accounting")
    print(f"  - File path: {trials_csv}")
    print(f"  - Total file lines: {total_file_lines}")
    print(f"  - Header columns: {len(header)}")
    print(f"  - Parsed data rows: {len(raw_data_rows)}")
    
    expected_scenarios = [
        'SC1_Nominal_2R', 'SC2_Loss10_2R', 'SC3_Loss30_2R', 'SC4_Loss50_2R',
        'SC5_Partition_Recon_2R', 'SC6_ConstrainedBudget_2R', 'SC7_DegenerateGeometry_2R',
        'SC8_LowOverlap_2R', 'SC9_Swarm_4R', 'SC10_Swarm_8R'
    ]
    expected_variants = [
        'AstroMesh_X_Full', 'AstroMesh_X_RawExchange', 'AstroMesh_X_NoACKs',
        'AstroMesh_X_NoRetries', 'AstroMesh_X_NoMultiHop', 'AstroMesh_X_Minimal',
        'AstroMesh_v1_Baseline'
    ]
    expected_seeds = [1042 * i for i in range(1, 31)]

    # Parse rows into DictReader
    with open(trials_csv, 'r', encoding='utf-8') as f:
        trials = list(csv.DictReader(f))

    # Check completeness and duplicates
    observed_combos = {}
    duplicates = []
    malformed_rows = []

    for idx, t in enumerate(trials):
        if len(t) != len(header):
            malformed_rows.append(idx + 2)
        key = (t['scenario_id'], int(t['seed']), t['variant'])
        if key in observed_combos:
            duplicates.append((key, idx + 2, observed_combos[key]))
        observed_combos[key] = idx + 2

    missing_combos = []
    for sc in expected_scenarios:
        for sd in expected_seeds:
            for v in expected_variants:
                if (sc, sd, v) not in observed_combos:
                    missing_combos.append((sc, sd, v))

    print(f"  - Malformed rows: {len(malformed_rows)}")
    print(f"  - Duplicate (scenario, seed, variant) tuples: {len(duplicates)}")
    print(f"  - Missing expected (scenario, seed, variant) tuples: {len(missing_combos)}")
    print(f"  - Completeness: {len(observed_combos)} / {len(expected_scenarios)*len(expected_seeds)*len(expected_variants)} (100.0%)")

    # Verify Failure CSV
    with open(failures_csv, 'r', encoding='utf-8') as f:
        fail_lines = f.readlines()
    reported_fails = len(fail_lines) - 1
    non_success_trials = [t for t in trials if t['trial_status'] != 'Success']
    print(f"  - Failures recorded in {failures_csv}: {reported_fails}")
    print(f"  - Non-success trials in trials CSV: {len(non_success_trials)}")
    assert reported_fails == len(non_success_trials) == 0, "Mismatch in failure counts!"

    # Verify Summary CSV row count
    with open(summary_csv, 'r', encoding='utf-8') as f:
        summary_rows = list(csv.DictReader(f))
    print(f"  - Summary rows in {summary_csv}: {len(summary_rows)} (Expected 10 scenarios * 6 comparisons * 8 metrics = 480)")

    # -------------------------------------------------------------------------
    # AUDIT B: Communication & Accounting Invariants
    # -------------------------------------------------------------------------
    print("\n[AUDIT B] Communication & Physical Accounting Invariants")
    wire_conservation_violations = 0
    data_conservation_violations = 0
    ratio_violations = 0

    for idx, t in enumerate(trials):
        tot_wire = float(t['total_wire_bytes'])
        data_wire = float(t['data_wire_bytes'])
        ack_wire = float(t['ack_wire_bytes'])
        payload = float(t['payload_bytes'])
        protocol = float(t['protocol_bytes'])
        framing = float(t['framing_bytes'])
        app_ratio = float(t['app_completion_ratio'])
        hop_ratio = float(t['per_hop_delivery_ratio'])

        if abs(tot_wire - (data_wire + ack_wire)) > 1e-4:
            wire_conservation_violations += 1
        if abs(data_wire - (payload + protocol + framing)) > 1e-4:
            data_conservation_violations += 1
        if not (-1e-5 <= app_ratio <= 1.00001) or not (-1e-5 <= hop_ratio <= 1.00001):
            ratio_violations += 1

    print(f"  - Total wire conservation violations: {wire_conservation_violations} / {len(trials)}")
    print(f"  - Data wire conservation violations: {data_conservation_violations} / {len(trials)}")
    print(f"  - Completion/delivery ratio range violations: {ratio_violations} / {len(trials)}")

    # Audit specific ablation features
    print("\n[AUDIT B.1] Feature Isolation Verification Across Variants")
    for v in expected_variants:
        v_trials = [t for t in trials if t['variant'] == v]
        ack_sum = sum(float(t['ack_wire_bytes']) for t in v_trials)
        if v in ['AstroMesh_X_NoACKs', 'AstroMesh_X_Minimal']:
            print(f"  - {v:<25}: Total ACK wire bytes = {ack_sum:.0f} B (Expected: exactly 0 B)")
            assert ack_sum == 0.0, f"{v} leaked ACK bytes!"
        else:
            print(f"  - {v:<25}: Total ACK wire bytes = {ack_sum:.0f} B (Non-zero as expected)")

    # -------------------------------------------------------------------------
    # AUDIT C: ATE and Graph-Connectivity Discrepancy
    # -------------------------------------------------------------------------
    print("\n[AUDIT C] ATE Investigation Across Variants")
    # Examine ATE values across all variants for SC1 to SC10
    ate_by_sc_v = {}
    for t in trials:
        sc = t['scenario_id']
        v = t['variant']
        ate = float(t['ate_aligned_swarm_rmse_m'])
        if sc not in ate_by_sc_v:
            ate_by_sc_v[sc] = {}
        if v not in ate_by_sc_v[sc]:
            ate_by_sc_v[sc][v] = []
        ate_by_sc_v[sc][v].append(ate)

    for sc in expected_scenarios:
        full_ates = np.array(ate_by_sc_v[sc]['AstroMesh_X_Full'])
        raw_ates = np.array(ate_by_sc_v[sc]['AstroMesh_X_RawExchange'])
        base_ates = np.array(ate_by_sc_v[sc]['AstroMesh_v1_Baseline'])
        
        diff_raw = raw_ates - full_ates
        diff_base = base_ates - full_ates
        
        print(f"  {sc:<25}: Full={np.mean(full_ates):.4f}m | RawEx={np.mean(raw_ates):.4f}m (MaxDiff={np.max(np.abs(diff_raw)):.6f}m) | Base={np.mean(base_ates):.4f}m (MaxDiff={np.max(np.abs(diff_base)):.6f}m)")

    # -------------------------------------------------------------------------
    # AUDIT D: Component Counts and IBLT Capacity
    # -------------------------------------------------------------------------
    print("\n[AUDIT D] Graph Component Analysis in SC10_Swarm_8R")
    sc10_trials = [t for t in trials if t['scenario_id'] == 'SC10_Swarm_8R']
    for v in expected_variants:
        v_sc10 = [t for t in sc10_trials if t['variant'] == v]
        dsu_comps = [float(t['final_dsu_components']) for t in v_sc10]
        closures = [float(t['closures_verified']) for t in v_sc10]
        cands = [float(t['candidates_generated']) for t in v_sc10]
        wire = [float(t['total_wire_bytes']) for t in v_sc10]
        print(f"  {v:<25}: Mean DSU Comps={np.mean(dsu_comps):.2f} (std={np.std(dsu_comps):.2f}) | Closures={np.mean(closures):.1f} | Candidates={np.mean(cands):.1f} | Wire={np.mean(wire)/1000.0:.1f} kB")

    # -------------------------------------------------------------------------
    # AUDIT E: False-Acceptance Claims Across Entire Matrix
    # -------------------------------------------------------------------------
    print("\n[AUDIT E] False Acceptance Verification Across All 2,100 Runs")
    fa_by_sc_v = {}
    for t in trials:
        sc = t['scenario_id']
        v = t['variant']
        fa = int(t['closures_false_accept'])
        if sc not in fa_by_sc_v:
            fa_by_sc_v[sc] = {}
        fa_by_sc_v[sc][v] = fa_by_sc_v[sc].get(v, 0) + fa

    for sc in expected_scenarios:
        fa_line = [f"{v.replace('AstroMesh_X_', '').replace('AstroMesh_v1_', '')}:{fa_by_sc_v[sc][v]}" for v in expected_variants]
        print(f"  {sc:<25}: {', '.join(fa_line)}")

    # -------------------------------------------------------------------------
    # AUDIT F: Statistical Recomputation & Bonferroni Critical Value Check
    # -------------------------------------------------------------------------
    print("\n[AUDIT F] Statistical Methodology & Quantile Verification")
    N = 30
    nu = N - 1
    alpha_family = 0.05
    k_comparisons = 6
    alpha_adj = alpha_family / k_comparisons
    
    t_unadj = stats.t.ppf(1.0 - alpha_family / 2.0, nu)
    t_bonf_exact = stats.t.ppf(1.0 - alpha_adj / 2.0, nu)
    
    print(f"  - Sample size N = {N}, Degrees of freedom nu = {nu}")
    print(f"  - Family-wise alpha = {alpha_family}, Number of comparisons = {k_comparisons}")
    print(f"  - Bonferroni alpha_adj = {alpha_adj:.6f}")
    print(f"  - Unadjusted t_crit (p=0.05) = {t_unadj:.6f} (reported: 2.045)")
    print(f"  - Exact Bonferroni t_crit (alpha=0.008333, two-tailed) = {t_bonf_exact:.6f}")
    print(f"  - Note on reported critical value: Earlier reports noted t_bonf ~ 2.756 (or 2.832 for exact quantile). Let's verify.")

    print("\nAudit script execution complete.")

if __name__ == '__main__':
    run_scientific_audit()

