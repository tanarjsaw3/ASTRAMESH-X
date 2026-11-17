import pandas as pd
import numpy as np
import scipy.stats as st

df = pd.read_csv('benchmark_distributed_trajectory_validated.csv')

print("=================================================================")
print("     ASTRAMESH-X VALIDATED BENCHMARK DATASET AUDIT (81 TRIALS)   ")
print("=================================================================")
print(f"Total Rows in CSV: {len(df)}")
print(f"Unique Seeds: {df['seed'].unique().tolist()}")
print(f"Wire Byte Invariant Conservation: {(df['total_wire_bytes'] == df['data_wire_bytes'] + df['ack_wire_bytes']).all()}")

print("\n--- 1. PROPOSAL & CONFUSION MATRIX TOTALS ---")
print(f"Total Genuine Proposals:  {df['genuine_proposals'].sum()}")
print(f"Total Imposter Proposals: {df['imposter_proposals'].sum()}")
print(f"Total Proposals Sent:     {df['proposals_sent'].sum()}")
print(f"Total Proposals Received: {df['proposals_received'].sum()}")
print(f"Total Accepted Proposals: {df['accepted_proposals'].sum()}")
print(f"Total Rejected Proposals: {df['rejected_proposals'].sum()}")

tp = df['true_positives'].sum()
fp = df['false_positives'].sum()
tn = df['true_negatives'].sum()
fn = df['false_negatives'].sum()

print(f"\nConfusion Matrix Raw Totals:")
print(f"  TP: {tp}")
print(f"  FP: {fp}")
print(f"  TN: {tn}")
print(f"  FN: {fn}")

# Global and trial-level rate calculations
precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
specificity = tn / (tn + fp) if (tn + fp) > 0 else 0.0
fpr = fp / (fp + tn) if (fp + tn) > 0 else 0.0
fnr = fn / (fn + tp) if (fn + tp) > 0 else 0.0

print(f"\nAggregate Classification Metrics:")
print(f"  Precision:   {precision:.6f} ({precision*100:.2f}%)")
print(f"  Recall:      {recall:.6f} ({recall*100:.2f}%)")
print(f"  Specificity: {specificity:.6f} ({specificity*100:.2f}%)")
print(f"  FPR:         {fpr:.6f} ({fpr*100:.2f}%)")
print(f"  FNR:         {fnr:.6f} ({fnr*100:.2f}%)")

print("\n--- 2. ATE & CORD REDUCTION METRICS ---")
for col in ['initial_ate_rmse', 'final_cord_ate_rmse', 'ate_absolute_improvement', 'ate_reduction_pct']:
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

print("\n--- 3. UNIFIED VS PARTITIONED SWARMS ---")
unified = df[df['final_dsu_components'] == 1]
partitioned = df[df['final_dsu_components'] > 1]
print(f"Unified Swarms (DSU=1):     {len(unified)} / {len(df)} ({len(unified)/len(df)*100:.2f}%)")
print(f"Partitioned Swarms (DSU>1):   {len(partitioned)} / {len(df)} ({len(partitioned)/len(df)*100:.2f}%)")

print("\nUnified Swarm ATE:")
print(f"  Initial ATE Mean: {unified['initial_ate_rmse'].mean():.4f} m")
print(f"  Final ATE Mean:   {unified['final_cord_ate_rmse'].mean():.4f} m")
print(f"  Reduction Mean:   {unified['ate_reduction_pct'].mean():.2f}%")

print("\nPartitioned Swarm ATE:")
print(f"  Initial ATE Mean: {partitioned['initial_ate_rmse'].mean():.4f} m")
print(f"  Final ATE Mean:   {partitioned['final_cord_ate_rmse'].mean():.4f} m")
print(f"  Reduction Mean:   {partitioned['ate_reduction_pct'].mean():.2f}%")

print("\n--- 4. COMPARISON WITH 0.5M RESEARCH TARGET ---")
sub_half_m = (df['final_cord_ate_rmse'] < 0.50).sum()
print(f"Trials achieving sub-0.5m final ATE: {sub_half_m} / {len(df)} ({sub_half_m/len(df)*100:.2f}%)")
print("Breakdown by keyframe depth:")
print(df.groupby('keyframes_per_rover')['final_cord_ate_rmse'].describe())

