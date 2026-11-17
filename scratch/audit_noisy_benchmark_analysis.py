import pandas as pd
import numpy as np

def run_noisy_benchmark_audit():
    csv_file = "benchmark_distributed_noisy_trials.csv"
    df = pd.read_csv(csv_file)
    print("=================================================================")
    print("  ASTRAMESH-X M7.8 SCIENTIFIC NOISY BENCHMARK INTEGRITY AUDIT     ")
    print("=================================================================")
    print(f"Total Trial Rows Loaded: {len(df)}")
    assert len(df) == 81, f"Expected 81 rows, got {len(df)}"

    # 1. Unique Trial IDs
    unique_ids = df['trial_id'].nunique()
    print(f"Unique Trial IDs: {unique_ids} / 81")
    assert unique_ids == 81, "Duplicate trial IDs detected!"

    # 2. Factorial Grid
    rovers = sorted(df['rover_count'].unique().tolist())
    sigmas = sorted(df['sigma_landmark'].unique().tolist())
    losses = sorted(df['loss_rate'].unique().tolist())
    seeds = sorted(df['seed'].unique().tolist())

    print(f"Evaluated Rover Counts:     {rovers}")
    print(f"Evaluated Noise Sigmas (m): {sigmas}")
    print(f"Evaluated Loss Rates:       {losses}")
    print(f"Evaluated Seeds:            {seeds}")

    assert rovers == [2, 3, 4], "Rover grid mismatch"
    assert np.allclose(sigmas, [0.01, 0.05, 0.10]), "Sigma grid mismatch"
    assert np.allclose(losses, [0.0, 0.20, 0.50]), "Loss grid mismatch"
    assert seeds == [1001, 2002, 3003], "Seeds mismatch"

    # 3. Wire-Byte Accounting Invariant Check
    total_wire_match = (df['total_wire_bytes'] == df['data_wire_bytes'] + df['ack_wire_bytes']).all()
    data_wire_match = (df['data_wire_bytes'] == df['payload_bytes'] + df['header_bytes'] + df['framing_bytes']).all()
    print(f"\n--- Wire Accounting Invariants ---")
    print(f"Total Wire Conservation (total_wire == data_wire + ack_wire): {total_wire_match}")
    print(f"Data Wire Conservation (data_wire == payload + header + framing): {data_wire_match}")
    assert total_wire_match, "Total wire byte conservation violated!"
    assert data_wire_match, "Data wire byte conservation violated!"

    # 4. Topological Unification
    unification_rate = (df['final_dsu_components'] == 1).mean() * 100.0
    print(f"\n--- Topological SLAM Convergence ---")
    print(f"Topological Partition Unification Success Rate: {unification_rate:.1f}%")
    print(f"Loop Closures Verified (Total): {df['closures_verified'].sum()}")
    print(f"Loop Closures Rejected (Total): {df['closures_rejected'].sum()}")
    print(f"IBLT Decodes Succeeded (Total): {df['decodes_succ'].sum()}")
    print(f"IBLT Decodes Failed (Total):    {df['decodes_fail'].sum()}")

    # 5. Scientific Noise & SLAM Accuracy Analysis
    print(f"\n--- Scientific SLAM Trajectory Accuracy Under Noise ---")
    sigma_grp = df.groupby('sigma_landmark')[['swarm_ate_rmse', 'relative_pose_error']].agg(['mean', 'std', 'min', 'max'])
    print(sigma_grp.to_string())

    overall_mean_ate = df['swarm_ate_rmse'].mean()
    print(f"\nOverall Swarm ATE RMSE: {overall_mean_ate:.4f} m")
    assert overall_mean_ate > 0.0, "ATE RMSE should be non-zero under Gaussian noise injection!"

    # 6. Communication Impairment Scaling
    print(f"\n--- Communication Scaling Under Network Impairment ---")
    loss_grp = df.groupby('loss_rate')[['total_wire_bytes', 'retransmissions', 'duplicate_suppressed', 'elapsed_ms']].mean()
    print(loss_grp.to_string())

    print("\n=================================================================")
    print("  ALL M7.8 SCIENTIFIC BENCHMARK INVARIANTS VERIFIED CLEANLY!     ")
    print("=================================================================")

if __name__ == "__main__":
    run_noisy_benchmark_audit()

