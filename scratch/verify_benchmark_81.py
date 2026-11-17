import pandas as pd
import numpy as np
import scipy.stats as st

df = pd.read_csv('benchmark_distributed_trajectory_remediated.csv')

print("--- 1. ROW & SEED AUDIT ---")
print(f"Total rows in CSV: {len(df)}")
unique_seeds = sorted(df['seed'].unique().tolist())
print(f"Unique seeds: {unique_seeds} (count: {len(unique_seeds)})")

print("\n--- 2. FACTORIAL DESIGN AUDIT ---")
config_counts = df.groupby(['rover_count', 'keyframes_per_rover', 'packet_loss_rate', 'delay_ms']).size()
print(f"Total configurations: {len(config_counts)}")
print(f"Trials per configuration: min={config_counts.min()}, max={config_counts.max()}")
print(f"All configurations have exactly 3 trials: {(config_counts == 3).all()}")

print("\n--- 3. COMPLETION & CONVERGENCE ---")
print(f"Completed trials (elapsed_compute_ms > 0): {(df['elapsed_compute_ms'] > 0).sum()} / {len(df)}")
# Timed-out: In our benchmark, trials that failed to unify all DSU components or had retries drop packets
timed_out_or_partitioned = (df['final_dsu_components'] > 1).sum()
print(f"Fully unified swarms (final_dsu_components == 1): {(df['final_dsu_components'] == 1).sum()} / {len(df)} ({ (df['final_dsu_components'] == 1).mean()*100:.2f}%)")
print(f"Partitioned swarms (final_dsu_components > 1): {timed_out_or_partitioned} / {len(df)} ({timed_out_or_partitioned/len(df)*100:.2f}%)")
print("DSU components distribution:")
print(df['final_dsu_components'].value_counts())

print("\n--- 4. ATE & REDUCTION STATISTICS ---")
for col in ['initial_ate_rmse', 'final_cord_ate_rmse', 'ate_reduction_pct']:
    vals = df[col]
    mean_val = vals.mean()
    med_val = vals.median()
    std_val = vals.std(ddof=1)
    sem_val = st.sem(vals)
    ci95 = st.t.interval(0.95, len(vals)-1, loc=mean_val, scale=sem_val)
    print(f"{col}:")
    print(f"  Mean:   {mean_val:.4f}")
    print(f"  Median: {med_val:.4f}")
    print(f"  Std:    {std_val:.4f}")
    print(f"  95% CI: [{ci95[0]:.4f}, {ci95[1]:.4f}]")
    print(f"  Min:    {vals.min():.4f}")
    print(f"  Max:    {vals.max():.4f}")

print("\n--- 5. WIRE BYTE INVARIANT AUDIT ---")
wire_check = (df['total_wire_bytes'] == df['data_wire_bytes'] + df['ack_wire_bytes']).all()
print(f"Wire conservation (total == data + ack): {wire_check}")

