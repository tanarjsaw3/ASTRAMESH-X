import csv
import math
import numpy as np
from scipy import stats

def check_bonferroni():
    with open('benchmark_phase61_trials.csv', 'r', encoding='utf-8') as f:
        trials = list(csv.DictReader(f))

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
        'SC1_Nominal_2R', 'SC2_Loss10_2R', 'SC3_Loss30_2R', 'SC4_Loss50_2R',
        'SC5_Partition_Recon_2R', 'SC6_ConstrainedBudget_2R', 'SC7_DegenerateGeometry_2R',
        'SC8_LowOverlap_2R', 'SC9_Swarm_4R', 'SC10_Swarm_8R'
    ]
    variants = [
        'AstroMesh_X_RawExchange', 'AstroMesh_X_NoACKs', 'AstroMesh_X_NoRetries',
        'AstroMesh_X_NoMultiHop', 'AstroMesh_X_Minimal', 'AstroMesh_v1_Baseline'
    ]
    metrics = [
        ('total_wire_bytes', 'Total_Transmitted_Wire_Bytes', True),
        ('data_wire_bytes', 'Data_Wire_Bytes', True),
        ('ack_wire_bytes', 'ACK_Wire_Bytes', True),
        ('app_completion_ratio', 'Application_Completion_Ratio', False),
        ('per_hop_delivery_ratio', 'Per_Hop_Delivery_Ratio', False),
        ('closures_verified', 'Verified_Loop_Closures', False),
        ('final_dsu_components', 'Final_DSU_Components', True),
        ('ate_aligned_swarm_rmse_m', 'ATE_Aligned_Swarm_RMSE', True)
    ]

    t_approx = 2.756
    t_exact = stats.t.ppf(1.0 - (0.05 / 6) / 2.0, 29) # 2.831553

    print(f"Comparing decisions under t_approx = {t_approx} vs t_exact = {t_exact:.6f}")
    
    discrepancies = []
    total_comparisons = 0

    for sc in scenarios:
        sc_seeds = data[sc]
        valid_seeds = [sd for sd, v_dict in sc_seeds.items() if len(v_dict) == 7]
        assert len(valid_seeds) == 30

        for m_col, m_name, lower_is_better in metrics:
            full_vals = [float(sc_seeds[sd]['AstroMesh_X_Full'][m_col]) for sd in valid_seeds]
            for v in variants:
                total_comparisons += 1
                v_vals = [float(sc_seeds[sd][v][m_col]) for sd in valid_seeds]
                diffs = np.array(v_vals) - np.array(full_vals)
                d_mean = np.mean(diffs)
                d_std = np.std(diffs, ddof=1)
                se = d_std / math.sqrt(30)

                ci_approx = (d_mean - t_approx * se, d_mean + t_approx * se)
                ci_exact = (d_mean - t_exact * se, d_mean + t_exact * se)

                sig_approx = (ci_approx[0] > 0 or ci_approx[1] < 0) and abs(d_mean) > 1e-9
                sig_exact = (ci_exact[0] > 0 or ci_exact[1] < 0) and abs(d_mean) > 1e-9

                if sig_approx != sig_exact:
                    discrepancies.append((sc, v, m_name, d_mean, se, ci_approx, ci_exact, sig_approx, sig_exact))

    print(f"Total pairwise comparisons evaluated: {total_comparisons}")
    print(f"Decision discrepancies between t=2.756 and t={t_exact:.4f}: {len(discrepancies)}")
    if discrepancies:
        for d in discrepancies:
            print(f"  Discrepancy in {d[0]} | {d[1]} | {d[2]}: MeanDiff={d[3]:.4f}, SE={d[4]:.4f}")
            print(f"    Under t=2.756: CI={d[5]}, Sig={d[7]}")
            print(f"    Under t={t_exact:.4f}: CI={d[6]}, Sig={d[8]}")
    else:
        print("  ALL 480 STATISTICAL DECISIONS ARE IDENTICAL between t=2.756 and t=2.831553!")

if __name__ == '__main__':
    check_bonferroni()

