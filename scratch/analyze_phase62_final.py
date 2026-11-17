import pandas as pd
import numpy as np
import scipy.stats as stats
import math

def analyze():
    print("Loading benchmark_phase62_trials.csv...")
    df = pd.read_csv('benchmark_phase62_trials.csv')
    sc1 = df[df['scenario_id'] == 'SC1_Nominal_2R'].copy()
    sc10 = df[df['scenario_id'] == 'SC10_Swarm_8R'].copy()

    print(f"Loaded {len(sc1)} SC1 trials and {len(sc10)} SC10 trials.")

    # -------------------------------------------------------------
    # 1. SC1 PRIMARY BENCHMARK: WIRE BYTES BY VARIANT & DELAY (Loss = 0.0)
    # -------------------------------------------------------------
    print("\n=================================================================")
    print("TABLE 1: SC1 Nominal Wire Bytes (Loss = 0.0) across Delays (Mean ± Std)")
    print("=================================================================")
    sc1_loss0 = sc1[sc1['loss_rate'] == 0.0]
    piv_wire_loss0 = sc1_loss0.pivot_table(
        index='variant',
        columns='delay_ms',
        values='total_wire_bytes',
        aggfunc=lambda x: f"{np.mean(x):.1f} ± {np.std(x, ddof=1):.1f}"
    )
    print(piv_wire_loss0.to_string())

    # Retransmissions at Loss = 0.0 across Delays
    print("\n-----------------------------------------------------------------")
    print("TABLE 2: SC1 Retransmissions (Loss = 0.0) across Delays (Spurious Retries)")
    print("-----------------------------------------------------------------")
    piv_retries_loss0 = sc1_loss0.pivot_table(
        index='variant',
        columns='delay_ms',
        values='retransmissions_performed',
        aggfunc='mean'
    )
    print(piv_retries_loss0.round(2).to_string())

    # -------------------------------------------------------------
    # 2. SC1 PRIMARY BENCHMARK: APPLICATION COMPLETION RATIO BY LOSS (Delay = 0 ms)
    # -------------------------------------------------------------
    print("\n=================================================================")
    print("TABLE 3: SC1 Application Completion Ratio across Loss Rates (Delay = 0 ms)")
    print("=================================================================")
    sc1_delay0 = sc1[sc1['delay_ms'] == 0]
    piv_comp_delay0 = sc1_delay0.pivot_table(
        index='variant',
        columns='loss_rate',
        values='app_completion_ratio',
        aggfunc=lambda x: f"{np.mean(x)*100.0:.2f}%"
    )
    print(piv_comp_delay0.to_string())

    # -------------------------------------------------------------
    # 3. SC1 PRIMARY BENCHMARK: FINAL DSU COMPONENTS (GRAPH CONNECTIVITY)
    # -------------------------------------------------------------
    print("\n=================================================================")
    print("TABLE 4: SC1 Final DSU Components across Loss Rates (Delay = 0 ms, N=30)")
    print("=================================================================")
    piv_dsu_delay0 = sc1_delay0.pivot_table(
        index='variant',
        columns='loss_rate',
        values='final_dsu_components',
        aggfunc='mean'
    )
    print(piv_dsu_delay0.round(2).to_string())

    # -------------------------------------------------------------
    # 4. SC1 PRIMARY BENCHMARK: ATE ALIGNED SWARM RMSE (m)
    # -------------------------------------------------------------
    print("\n=================================================================")
    print("TABLE 5: SC1 ATE Aligned Swarm RMSE (m) across Loss Rates (Delay = 0 ms)")
    print("=================================================================")
    piv_ate_delay0 = sc1_delay0.pivot_table(
        index='variant',
        columns='loss_rate',
        values='ate_aligned_swarm_rmse_m',
        aggfunc='mean'
    )
    print(piv_ate_delay0.round(4).to_string())

    # -------------------------------------------------------------
    # 5. SC1 HIGH DELAY (Delay = 500 ms) ACROSS LOSS RATES
    # -------------------------------------------------------------
    print("\n=================================================================")
    print("TABLE 6: SC1 Completion Ratio under High Delay (500 ms) across Loss")
    print("=================================================================")
    sc1_delay500 = sc1[sc1['delay_ms'] == 500]
    piv_comp_delay500 = sc1_delay500.pivot_table(
        index='variant',
        columns='loss_rate',
        values='app_completion_ratio',
        aggfunc=lambda x: f"{np.mean(x)*100.0:.2f}%"
    )
    print(piv_comp_delay500.to_string())

    print("\n=================================================================")
    print("TABLE 7: SC1 Wire Bytes (B) under High Delay (500 ms) across Loss")
    print("=================================================================")
    piv_wire_delay500 = sc1_delay500.pivot_table(
        index='variant',
        columns='loss_rate',
        values='total_wire_bytes',
        aggfunc='mean'
    )
    print(piv_wire_delay500.round(1).to_string())

    # In-Flight Packets at Termination across Delays
    print("\n=================================================================")
    print("TABLE 8: SC1 In-Flight Packets at Termination across Delays (Mean)")
    print("=================================================================")
    piv_flight = sc1.pivot_table(
        index='variant',
        columns='delay_ms',
        values='in_flight_at_termination',
        aggfunc='mean'
    )
    print(piv_flight.round(3).to_string())

    # -------------------------------------------------------------
    # 6. PAIRED STATISTICAL HYPOTHESIS TESTING (N=30 per cell, Bonferroni-corrected)
    # -------------------------------------------------------------
    print("\n=================================================================")
    print("PAIRED STATISTICAL HYPOTHESIS TESTING (AstroMesh_X_Full vs Variants)")
    print("=================================================================")
    
    # We will test key representative cells:
    # Cell A: Nominal (Loss=0.0, Delay=0ms)
    # Cell B: Moderate Stress (Loss=0.3, Delay=100ms)
    # Cell C: High Loss (Loss=0.7, Delay=0ms)
    # Cell D: High Delay (Loss=0.0, Delay=500ms)
    # Cell E: Extreme Compound (Loss=0.8, Delay=350ms)
    
    test_cells = [
        (0.0, 0, "Nominal Channel (Loss 0%, Delay 0ms)"),
        (0.3, 100, "Moderate Stress (Loss 30%, Delay 100ms)"),
        (0.7, 0, "Severe Packet Loss (Loss 70%, Delay 0ms)"),
        (0.0, 500, "High Latency (Loss 0%, Delay 500ms)"),
        (0.8, 350, "Extreme Compound Stress (Loss 80%, Delay 350ms)")
    ]

    variants = [
        "AstroMesh_X_RawExchange",
        "AstroMesh_X_NoACKs",
        "AstroMesh_X_NoRetries",
        "AstroMesh_X_NoMultiHop",
        "AstroMesh_X_Minimal",
        "AstroMesh_v1_Baseline"
    ]

    metrics = [
        ('total_wire_bytes', 'Total Wire Bytes', True),
        ('app_completion_ratio', 'App Completion Ratio', False),
        ('closures_verified', 'Verified Closures', False),
        ('final_dsu_components', 'Final DSU Components', True),
        ('ate_aligned_swarm_rmse_m', 'ATE Aligned RMSE', True)
    ]

    alpha_bonf = 0.05 / len(variants) # 0.05 / 6 = 0.00833
    t_bonf = stats.t.ppf(1.0 - alpha_bonf / 2.0, 29) # df = 29

    for loss, delay, label in test_cells:
        print(f"\n--- Cell: {label} [Loss={loss}, Delay={delay}ms, N=30] ---")
        cell_df = sc1[(sc1['loss_rate'] == loss) & (sc1['delay_ms'] == delay)]
        
        # Get Full variant values indexed by seed
        full_df = cell_df[cell_df['variant'] == 'AstroMesh_X_Full'].set_index('seed')
        
        for m_col, m_name, lower_better in metrics:
            print(f"\n  Metric: {m_name} ({'Lower' if lower_better else 'Higher'} is better)")
            f_vals = full_df[m_col]
            f_mean = f_vals.mean()
            f_std = f_vals.std(ddof=1)
            
            for v in variants:
                v_df = cell_df[cell_df['variant'] == v].set_index('seed')
                v_vals = v_df.loc[full_df.index, m_col]
                v_mean = v_vals.mean()
                v_std = v_vals.std(ddof=1)
                
                diffs = (v_vals - f_vals).values
                d_mean = np.mean(diffs)
                d_std = np.std(diffs, ddof=1)
                se = d_std / math.sqrt(30)
                
                ci_bonf_low = d_mean - t_bonf * se
                ci_bonf_high = d_mean + t_bonf * se
                
                if np.all(diffs == diffs[0]):
                    p_val = 1.0 if diffs[0] == 0 else 0.0
                else:
                    _, p_val = stats.ttest_rel(v_vals, f_vals)
                
                p_adj = min(1.0, p_val * len(variants))
                
                # Decision
                sig = (ci_bonf_low > 0 or ci_bonf_high < 0) and abs(d_mean) > 1e-9
                if sig:
                    if lower_better:
                        verdict = f"{v} Better" if d_mean < 0 else "Full Better"
                    else:
                        verdict = f"{v} Better" if d_mean > 0 else "Full Better"
                else:
                    verdict = "Indistinguishable"
                
                print(f"    vs {v:25s}: diff={d_mean:+10.3f} (95% CI [{ci_bonf_low:+10.3f}, {ci_bonf_high:+10.3f}]), p_adj={p_adj:.4e} -> {verdict}")

    # -------------------------------------------------------------
    # 7. SC10 EXPLORATORY OBSERVATIONS (680 trials)
    # -------------------------------------------------------------
    print("\n=================================================================")
    print("SC10 EXPLORATORY OBSERVATIONS (8-Rover Swarm, 680 Completed Trials)")
    print("=================================================================")
    print(f"Completed trials: {len(sc10)} / 2,450 ({len(sc10)/2450*100.0:.1f}%)")
    sc10_cells = sc10.groupby(['loss_rate', 'delay_ms']).size()
    print(f"Covered cells: {len(sc10_cells)} / 70 cells")
    print(sc10_cells)

    # Let's inspect SC10 wire bytes and multi-hop routing effect at Loss=0.0 across delays
    sc10_loss0 = sc10[sc10['loss_rate'] == 0.0]
    if len(sc10_loss0) > 0:
        print("\nSC10 Mean Total Wire Bytes at Loss = 0.0 across Delays:")
        piv_sc10_wire = sc10_loss0.pivot_table(index='variant', columns='delay_ms', values='total_wire_bytes', aggfunc='mean')
        print(piv_sc10_wire.round(1).to_string())

        print("\nSC10 Mean Final DSU Components at Loss = 0.0 across Delays:")
        piv_sc10_dsu = sc10_loss0.pivot_table(index='variant', columns='delay_ms', values='final_dsu_components', aggfunc='mean')
        print(piv_sc10_dsu.round(2).to_string())

    # Multi-hop impact in SC10
    sc10_full = sc10[sc10['variant'] == 'AstroMesh_X_Full']
    sc10_nomh = sc10[sc10['variant'] == 'AstroMesh_X_NoMultiHop']
    common_idx = sc10_full.set_index(['loss_rate', 'delay_ms', 'seed']).index.intersection(
        sc10_nomh.set_index(['loss_rate', 'delay_ms', 'seed']).index
    )
    if len(common_idx) > 0:
        wire_full = sc10_full.set_index(['loss_rate', 'delay_ms', 'seed']).loc[common_idx, 'total_wire_bytes']
        wire_nomh = sc10_nomh.set_index(['loss_rate', 'delay_ms', 'seed']).loc[common_idx, 'total_wire_bytes']
        dsu_full = sc10_full.set_index(['loss_rate', 'delay_ms', 'seed']).loc[common_idx, 'final_dsu_components']
        dsu_nomh = sc10_nomh.set_index(['loss_rate', 'delay_ms', 'seed']).loc[common_idx, 'final_dsu_components']
        
        diff_wire = wire_full - wire_nomh
        diff_dsu = dsu_nomh - dsu_full
        print(f"\nMulti-Hop Forwarding Effect in SC10 across {len(common_idx)} paired trials:")
        print(f"  Mean Wire Overhead for MultiHop: +{diff_wire.mean():.1f} bytes (+{(wire_full.mean()/wire_nomh.mean()-1)*100.0:.1f}%)")
        print(f"  Mean DSU Component Improvement: {diff_dsu.mean():+.3f} components merged by MultiHop")

if __name__ == '__main__':
    analyze()

