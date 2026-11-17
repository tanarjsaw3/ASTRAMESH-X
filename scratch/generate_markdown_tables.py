import csv
import math
import numpy as np
from scipy import stats

with open('benchmark_phase61_trials.csv', 'r', encoding='utf-8') as f:
    trials = list(csv.DictReader(f))

# Group by scenario and seed
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
    ('SC1_Nominal_2R', 'SC1 (Nominal 2R)'),
    ('SC2_Loss10_2R', 'SC2 (10% Loss 2R)'),
    ('SC3_Loss30_2R', 'SC3 (30% Loss 2R)'),
    ('SC4_Loss50_2R', 'SC4 (50% Loss 2R)'),
    ('SC5_Partition_Recon_2R', 'SC5 (Partition 2R)'),
    ('SC6_ConstrainedBudget_2R', 'SC6 (Budget 2R)'),
    ('SC7_DegenerateGeometry_2R', 'SC7 (Degen Geom 2R)'),
    ('SC8_LowOverlap_2R', 'SC8 (Low Overlap 2R)'),
    ('SC9_Swarm_4R', 'SC9 (Swarm 4R)'),
    ('SC10_Swarm_8R', 'SC10 (Swarm 8R)')
]

variants = [
    ('AstroMesh_X_Full', 'Full (A)'),
    ('AstroMesh_X_RawExchange', 'RawEx (B)'),
    ('AstroMesh_X_NoACKs', 'NoACK (C)'),
    ('AstroMesh_X_NoRetries', 'NoRet (D)'),
    ('AstroMesh_X_NoMultiHop', 'NoHop (E)'),
    ('AstroMesh_X_Minimal', 'Minimal (F)'),
    ('AstroMesh_v1_Baseline', 'Base v1')
]

metrics = [
    ('total_wire_bytes', 'Total Transmitted Wire Bytes (Bytes)', True),
    ('app_completion_ratio', 'Application Completion Ratio', False),
    ('closures_verified', 'Verified Loop Closures', False),
    ('final_dsu_components', 'Final DSU Components', True),
    ('ate_aligned_swarm_rmse_m', 'ATE Aligned Swarm RMSE (m)', True)
]

for m_key, m_name, lower_is_better in metrics:
    print(f"\n### {m_name}\n")
    print("| Scenario | Seeds | Full (A) | RawEx (B) | NoACK (C) | NoRet (D) | NoHop (E) | Minimal (F) | Base v1 |")
    print("| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |")
    
    for sc_id, sc_label in scenarios:
        if sc_id not in data:
            continue
        sc_seeds = data[sc_id]
        # find seeds where all variants exist
        valid_seeds = [sd for sd, v_dict in sc_seeds.items() if len(v_dict) == 7]
        n = len(valid_seeds)
        if n == 0:
            continue
        
        row_str = f"| **{sc_label}** | {n} "
        for v_key, v_label in variants:
            vals = [float(sc_seeds[sd][v_key][m_key]) for sd in valid_seeds]
            mean_val = np.mean(vals)
            if m_key in ['total_wire_bytes']:
                if mean_val >= 1e5:
                    row_str += f"| {mean_val/1000.0:.1f} kB "
                else:
                    row_str += f"| {mean_val:.0f} B "
            elif m_key in ['app_completion_ratio']:
                row_str += f"| {mean_val*100.0:.1f}% "
            elif m_key in ['closures_verified', 'final_dsu_components']:
                row_str += f"| {mean_val:.1f} "
            else:
                row_str += f"| {mean_val:.2f} m "
        row_str += "|"
        print(row_str)

