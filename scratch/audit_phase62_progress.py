import pandas as pd
import numpy as np

df = pd.read_csv('benchmark_phase62_trials.csv')

print(f"=== Phase 6.2 Active Dataset Audit ===")
print(f"Total Rows: {len(df)}")
sc1 = df[df['scenario_id'] == 'SC1_Nominal_2R']
sc10 = df[df['scenario_id'] == 'SC10_Swarm_8R']

print(f"SC1 Rows: {len(sc1)} (Target: 14700)")
print(f"SC10 Rows: {len(sc10)} (Target: 2450)")

# Key uniqueness
keys = df['scenario_id'] + '|' + df['loss_rate'].round(2).astype(str) + '|' + df['delay_ms'].astype(str) + '|' + df['seed'].astype(str) + '|' + df['variant']
dups = keys.duplicated().sum()
print(f"Duplicate Keys: {dups}")
assert dups == 0, f"Found {dups} duplicates!"

# Cell coverage for SC1
sc1_cells = sc1.groupby(['loss_rate', 'delay_ms']).size()
print(f"SC1 Cells Covered: {len(sc1_cells)} / 70")
print(f"SC1 Min/Max Runs per Cell: {sc1_cells.min()} / {sc1_cells.max()}")

# Byte conservation
data_cons = df['data_byte_conservation_valid'].all()
total_cons = df['total_byte_conservation_valid'].all()
print(f"Data Byte Conservation (100%): {data_cons}")
print(f"Total Byte Conservation (100%): {total_cons}")

# Failures & Statuses
print(f"Trial Status Distribution: {df['trial_status'].value_counts().to_dict()}")

# Finite metrics check
numeric_cols = [
    'runtime_us', 'candidates_generated', 'closures_verified', 'closures_rejected',
    'initial_dsu_components', 'final_dsu_components', 'disconnected_rover_pairs',
    'ate_raw_rmse_m', 'logical_messages_sent', 'logical_messages_delivered',
    'logical_messages_dropped', 'retransmissions_performed', 'data_wire_bytes',
    'ack_wire_bytes', 'total_wire_bytes'
]
non_finite = df[numeric_cols].isna().sum().sum() + np.isinf(df[numeric_cols]).sum().sum()
print(f"Non-Finite Metric Values: {non_finite}")

print("\n--- SC1 (30-Seed Primary Evidence) Wire Bytes Across Delays ---")
sc1_wire_by_delay = sc1.pivot_table(index='variant', columns='delay_ms', values='total_wire_bytes', aggfunc='mean')
print(sc1_wire_by_delay.round(1))

print("\n--- SC1 (30-Seed Primary Evidence) Retransmissions Across Delays ---")
sc1_retries_by_delay = sc1.pivot_table(index='variant', columns='delay_ms', values='retransmissions_performed', aggfunc='mean')
print(sc1_retries_by_delay.round(1))

print("\n--- SC1 (30-Seed Primary Evidence) Delivery Ratio Across Packet Loss ---")
sc1_deliv_by_loss = sc1.pivot_table(index='variant', columns='loss_rate', values='app_completion_ratio', aggfunc='mean')
print(sc1_deliv_by_loss.round(3))

print("\n--- SC1 (30-Seed Primary Evidence) Aligned Swarm ATE RMSE Across Packet Loss ---")
sc1_ate_by_loss = sc1.pivot_table(index='variant', columns='loss_rate', values='ate_aligned_swarm_rmse_m', aggfunc='mean')
print(sc1_ate_by_loss.round(4))

if len(sc10) > 0:
    print(f"\n--- SC10 (Interim Completed Cells: {len(sc10.groupby(['loss_rate', 'delay_ms']))}) ---")
    sc10_cells = sc10.groupby(['loss_rate', 'delay_ms']).size()
    print(f"SC10 Completed Cells: {len(sc10_cells)} / 70")
    print(sc10_cells.head(15))

