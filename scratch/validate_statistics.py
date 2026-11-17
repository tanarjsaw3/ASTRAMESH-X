import csv
import math
import numpy as np
from scipy import stats

def validate():
    raw_path = 'benchmark_phase61_trials.csv'
    with open(raw_path, 'r', encoding='utf-8') as f:
        trials = list(csv.DictReader(f))

    # Group by (scenario_id, seed)
    trials_by_scenario_seed = {}
    for t in trials:
        sc = t['scenario_id']
        sd = t['seed']
        v = t['variant']
        if sc not in trials_by_scenario_seed:
            trials_by_scenario_seed[sc] = {}
        if sd not in trials_by_scenario_seed[sc]:
            trials_by_scenario_seed[sc][sd] = {}
        trials_by_scenario_seed[sc][sd][v] = t

    scenarios = [
        'SC1_Nominal_2R',
        'SC2_Loss10_2R',
        'SC3_Loss30_2R',
        'SC4_Loss50_2R',
        'SC5_Partition_Recon_2R',
        'SC6_ConstrainedBudget_2R',
        'SC7_DegenerateGeometry_2R',
        'SC8_LowOverlap_2R',
        'SC9_Swarm_4R',
        'SC10_Swarm_8R'
    ]

    variants = [
        'AstroMesh_X_RawExchange',
        'AstroMesh_X_NoACKs',
        'AstroMesh_X_NoRetries',
        'AstroMesh_X_NoMultiHop',
        'AstroMesh_X_Minimal',
        'AstroMesh_v1_Baseline'
    ]

    metrics = [
        ('total_wire_bytes', 'Total Transmitted Wire Bytes (B)', True),
        ('app_completion_ratio', 'Application Completion Ratio', False),
        ('closures_verified', 'Verified Loop Closures', False),
        ('final_dsu_components', 'Final DSU Components', True),
        ('ate_aligned_swarm_rmse_m', 'ATE Aligned Swarm RMSE (m)', True)
    ]

    print("=== Independent Statistical Validation (Bonferroni Multiple Comparisons) ===")
    num_comparisons = len(variants)
    alpha_family = 0.05
    alpha_bonf = alpha_family / num_comparisons
    print(f"Family-wise alpha: {alpha_family}")
    print(f"Number of planned comparisons per scenario: {num_comparisons}")
    print(f"Bonferroni-adjusted alpha: {alpha_bonf:.6f}")

    for sc in scenarios:
        if sc not in trials_by_scenario_seed:
            continue
        sc_seeds = trials_by_scenario_seed[sc]
        # find seeds where Full and all variants exist
        complete_seeds = [sd for sd, v_dict in sc_seeds.items() if 'AstroMesh_X_Full' in v_dict]
        n_seeds = len(complete_seeds)
        if n_seeds < 2:
            continue

        nu = n_seeds - 1
        t_crit_unadj = stats.t.ppf(1.0 - alpha_family / 2.0, nu)
        t_crit_bonf = stats.t.ppf(1.0 - alpha_bonf / 2.0, nu)

        print(f"\n--- Scenario: {sc} (N = {n_seeds} paired trials, nu = {nu}, t_unadj = {t_crit_unadj:.3f}, t_bonf = {t_crit_bonf:.3f}) ---")

        for metric_col, metric_label, lower_is_better in metrics:
            print(f"  Metric: {metric_label}")
            full_vals = [float(sc_seeds[sd]['AstroMesh_X_Full'][metric_col]) for sd in complete_seeds]
            full_mean = np.mean(full_vals)

            for v in variants:
                # check if variant exists in all complete seeds
                paired_seeds = [sd for sd in complete_seeds if v in sc_seeds[sd]]
                if len(paired_seeds) != n_seeds:
                    print(f"    {v:<25}: Incomplete paired observations ({len(paired_seeds)}/{n_seeds})")
                    continue
                
                v_vals = [float(sc_seeds[sd][v][metric_col]) for sd in paired_seeds]
                # paired diff d_i = v_vals - full_vals (or full_vals - v_vals)
                # Convention: diff = Variant - Full
                diffs = np.array(v_vals) - np.array(full_vals)
                d_mean = np.mean(diffs)
                d_std = np.std(diffs, ddof=1)
                se = d_std / math.sqrt(n_seeds)

                ci_unadj_low = d_mean - t_crit_unadj * se
                ci_unadj_high = d_mean + t_crit_unadj * se
                ci_bonf_low = d_mean - t_crit_bonf * se
                ci_bonf_high = d_mean + t_crit_bonf * se

                # Significance determination
                # If Bonferroni CI does not include 0:
                sig = False
                if ci_bonf_low > 0 or ci_bonf_high < 0:
                    sig = True

                decision = "Indistinguishable"
                if sig:
                    if lower_is_better:
                        decision = f"{v} Better" if d_mean < 0 else "Full Better"
                    else:
                        decision = f"{v} Better" if d_mean > 0 else "Full Better"
                elif ci_unadj_low > 0 or ci_unadj_high < 0:
                    decision = "Unadjusted Sig only (rejected after Bonferroni)"

                print(f"    {v:<24} | Full:{full_mean:>10.2f} | Var:{np.mean(v_vals):>10.2f} | Diff:{d_mean:>10.2f} +/- {t_crit_bonf*se:>8.2f} | 95% Bonf CI: [{ci_bonf_low:>10.2f}, {ci_bonf_high:>10.2f}] | {decision}")

if __name__ == '__main__':
    validate()

