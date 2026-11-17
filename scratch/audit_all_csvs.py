import pandas as pd

csvs = [
    'benchmark_distributed_trials.csv',
    'benchmark_distributed_noisy_trials.csv',
    'benchmark_distributed_trajectory_trials.csv',
    'benchmark_distributed_trajectory_remediated.csv',
    'benchmark_distributed_trajectory_validated.csv'
]

print("=== COMPLETE BENCHMARK CSV REPOSITORY AUDIT ===")
for path in csvs:
    df = pd.read_csv(path)
    wire_cons = (df['total_wire_bytes'] == df['data_wire_bytes'] + df['ack_wire_bytes']).all()
    fp_zero = (df['false_positives'] == 0).all() if 'false_positives' in df.columns else 'N/A'
    print(f"\nDataset: {path}")
    print(f"  Rows: {len(df)}")
    print(f"  Wire byte conservation: {wire_cons}")
    print(f"  Zero false positives (FP==0): {fp_zero}")
    if 'initial_ate_rmse' in df.columns and 'final_cord_ate_rmse' in df.columns:
        print(f"  Initial ATE: {df['initial_ate_rmse'].mean():.4f}m -> Final ATE: {df['final_cord_ate_rmse'].mean():.4f}m (Reduction: {df['ate_reduction_pct'].mean():.2f}%)")
    if 'genuine_proposals' in df.columns:
        print(f"  Genuine proposals: {df['genuine_proposals'].sum()}, Imposter proposals: {df['imposter_proposals'].sum()}, Accepted: {df['accepted_proposals'].sum()}, Rejected: {df['rejected_proposals'].sum()}")
