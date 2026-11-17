import csv
import math
import numpy as np
from scipy import stats

def compute():
    raw_path = 'benchmark_phase61_trials.csv'
    with open(raw_path, 'r', encoding='utf-8') as f:
        trials = list(csv.DictReader(f))
    
    print(f"Total raw trial rows loaded: {len(trials)}")
    if len(trials) != 2100:
        print(f"Note: Current rows = {len(trials)}/2100. Running computation on available rows.")

    # Group by (scenario, seed)
    data = {}
    for t in trials:
        sc = t['scenario_id']
        sd = t['seed']
        v = t['variant']
        if sc not in data:
            data[sc] = {}
        if sd not in data[sc]:
            data[sc][sd] = {}
        data[sc][sd][v] = t

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
        ('AstroMesh_X_RawExchange', 'RawExchange'),
        ('AstroMesh_X_NoACKs', 'NoACKs'),
        ('AstroMesh_X_NoRetries', 'NoRetries'),
        ('AstroMesh_X_NoMultiHop', 'NoMultiHop'),
        ('AstroMesh_X_Minimal', 'Minimal'),
        ('AstroMesh_v1_Baseline', 'Baseline_v1')
    ]

    metrics = [
        ('total_wire_bytes', 'Total Transmitted Wire Bytes (B)', True),
        ('app_completion_ratio', 'Application Completion Ratio', False),
        ('closures_verified', 'Verified Loop Closures', False),
        ('final_dsu_components', 'Final DSU Components', True),
        ('ate_aligned_swarm_rmse_m', 'ATE Aligned Swarm RMSE (m)', True)
    ]

    alpha_family = 0.05
    num_comparisons = len(variants)
    alpha_bonf = alpha_family / num_comparisons

    results = []

    for sc in scenarios:
        if sc not in data:
            continue
        sc_seeds = data[sc]
        valid_seeds = [sd for sd, v_dict in sc_seeds.items() if 'AstroMesh_X_Full' in v_dict]
        n_seeds = len(valid_seeds)
        if n_seeds < 2:
            continue

        nu = n_seeds - 1
        t_unadj = stats.t.ppf(1.0 - alpha_family / 2.0, nu)
        t_bonf = stats.t.ppf(1.0 - alpha_bonf / 2.0, nu)

        for m_col, m_name, lower_is_better in metrics:
            full_vals = [float(sc_seeds[sd]['AstroMesh_X_Full'][m_col]) for sd in valid_seeds]
            f_mean = np.mean(full_vals)

            for v_key, v_name in variants:
                paired_seeds = [sd for sd in valid_seeds if v_key in sc_seeds[sd]]
                if len(paired_seeds) != n_seeds:
                    continue
                v_vals = [float(sc_seeds[sd][v_key][m_col]) for sd in paired_seeds]
                v_mean = np.mean(v_vals)

                # Paired difference: diff = Variant - Full
                diffs = np.array(v_vals) - np.array(full_vals)
                d_mean = np.mean(diffs)
                d_std = np.std(diffs, ddof=1)
                se = d_std / math.sqrt(n_seeds) if n_seeds > 1 else 0.0

                ci_unadj_low = d_mean - t_unadj * se
                ci_unadj_high = d_mean + t_unadj * se
                ci_bonf_low = d_mean - t_bonf * se
                ci_bonf_high = d_mean + t_bonf * se

                # Paired t-test
                if np.all(diffs == diffs[0]):
                    p_val = 1.0 if diffs[0] == 0 else 0.0
                else:
                    t_stat, p_val = stats.ttest_rel(v_vals, full_vals)

                p_val_adj = min(1.0, p_val * num_comparisons)

                sig_bonf = (ci_bonf_low > 0 or ci_bonf_high < 0) and abs(d_mean) > 1e-9
                sig_unadj = (ci_unadj_low > 0 or ci_unadj_high < 0) and abs(d_mean) > 1e-9

                decision = "Statistically Indistinguishable"
                if sig_bonf:
                    if lower_is_better:
                        decision = f"{v_name} Superior (p_adj < 0.05)" if d_mean < 0 else "AstroMesh_X_Full Superior (p_adj < 0.05)"
                    else:
                        decision = f"{v_name} Superior (p_adj < 0.05)" if d_mean > 0 else "AstroMesh_X_Full Superior (p_adj < 0.05)"
                elif sig_unadj:
                    decision = "Nominal Sig Only (Rejected by Bonferroni)"

                results.append({
                    'scenario': sc,
                    'seeds': n_seeds,
                    'metric': m_name,
                    'variant': v_name,
                    'full_mean': f_mean,
                    'var_mean': v_mean,
                    'diff_mean': d_mean,
                    'diff_std': d_std,
                    'se': se,
                    'ci_bonf_low': ci_bonf_low,
                    'ci_bonf_high': ci_bonf_high,
                    'p_val': p_val,
                    'p_val_adj': p_val_adj,
                    'decision': decision
                })

    print(f"\nSuccessfully evaluated {len(results)} paired comparisons across {len(scenarios)} scenarios.")
    return results

if __name__ == '__main__':
    compute()

