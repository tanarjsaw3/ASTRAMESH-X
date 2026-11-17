import pandas as pd
import numpy as np

def run_benchmark_audit():
    csv_file = "benchmark_distributed_trials.csv"
    df = pd.read_csv(csv_file)
    print("=================================================================")
    print("  ASTRAMESH-X M7.7 SCIENTIFIC BENCHMARK INTEGRITY AUDIT          ")
    print("=================================================================")
    print(f"Total Trial Rows Loaded: {len(df)}")
    assert len(df) == 81, f"Expected 81 rows, got {len(df)}"

    # 1. Check Unique Trial IDs
    unique_ids = df['trial_id'].nunique()
    print(f"Unique Trial IDs: {unique_ids} / 81")
    assert unique_ids == 81, "Duplicate trial IDs detected!"

    # 2. Factorial Design Check: rovers={2,3,4}, loss={0.0, 0.2, 0.5}, delay={0.0, 50.0, 100.0}, seeds=3
    rovers = sorted(df['rover_count'].unique().tolist())
    losses = sorted(df['loss_rate'].unique().tolist())
    delays = sorted(df['delay_ms'].unique().tolist())
    seeds = sorted(df['seed'].unique().tolist())

    print(f"Evaluated Rover Counts: {rovers} (expected [2, 3, 4])")
    print(f"Evaluated Loss Rates:   {losses} (expected [0.0, 0.2, 0.5])")
    print(f"Evaluated Delays (ms):  {delays} (expected [0.0, 50.0, 100.0])")
    print(f"Evaluated Seeds:        {seeds} (expected [1001, 2002, 3003])")

    assert rovers == [2, 3, 4], "Rover grid mismatch"
    assert np.allclose(losses, [0.0, 0.2, 0.5]), "Loss grid mismatch"
    assert np.allclose(delays, [0.0, 50.0, 100.0]), "Delay grid mismatch"
    assert seeds == [1001, 2002, 3003], "Seeds mismatch"

    # Check cell count: 3 * 3 * 3 = 27 cells, each with exactly 3 seeds
    cell_counts = df.groupby(['rover_count', 'loss_rate', 'delay_ms']).size()
    assert len(cell_counts) == 27, f"Expected 27 factorial cells, got {len(cell_counts)}"
    assert (cell_counts == 3).all(), "Every cell must have exactly 3 replications"
    print("Factorial Grid: 27 cells x 3 seeds = 81 trials (100% complete and balanced)")

    # 3. Wire-Byte Accounting Invariant Check
    total_wire_match = (df['total_wire_bytes'] == df['data_wire_bytes'] + df['ack_wire_bytes']).all()
    data_wire_match = (df['data_wire_bytes'] == df['payload_bytes'] + df['header_bytes'] + df['framing_bytes']).all()
    print(f"Wire Invariant (total_wire == data_wire + ack_wire): {total_wire_match}")
    print(f"Wire Invariant (data_wire == payload + header + framing): {data_wire_match}")
    assert total_wire_match, "Total wire byte conservation violated!"
    assert data_wire_match, "Data wire byte conservation violated!"

    # 4. Partition Unification Check
    unification_rate = (df['final_dsu_components'] == 1).mean() * 100.0
    print(f"Topological Partition Unification Success Rate: {unification_rate:.1f}%")
    assert unification_rate == 100.0, "Not all swarms unified their disjoint partitions"

    # 5. ATE RMSE Zero Investigation
    zero_ate_count = (df['swarm_ate_rmse'] == 0.0).sum()
    print(f"Trials with swarm_ate_rmse == 0.0: {zero_ate_count} / 81")

    # 6. Detailed Grouped Statistics
    print("\n--- GROUPED SUMMARY BY ROVER COUNT AND PACKET LOSS ---")
    summary = df.groupby(['rover_count', 'loss_rate']).agg({
        'total_wire_bytes': ['mean', 'std', 'min', 'max'],
        'retransmissions': 'mean',
        'duplicate_suppressed': 'mean',
        'closures_verified': 'mean',
        'final_dsu_components': 'mean',
        'swarm_ate_rmse': 'mean'
    })
    print(summary.to_string())

    print("\n--- GROUPED SUMMARY BY PROPAGATION DELAY ---")
    delay_sum = df.groupby('delay_ms').agg({
        'total_wire_bytes': 'mean',
        'retransmissions': 'mean',
        'duplicate_suppressed': 'mean',
        'closures_verified': 'mean',
        'elapsed_ms': 'mean'
    })
    print(delay_sum.to_string())

    print("\n=================================================================")
    print("  ALL BENCHMARK INTEGRITY AND CONSERVATION CHECKS PASSED (100%)  ")
    print("=================================================================")

if __name__ == "__main__":
    run_benchmark_audit()

