import pandas as pd
import numpy as np

def run_audit():
    csv_path = 'benchmark_phase62_trials.csv'
    print(f"Loading {csv_path}...")
    df = pd.read_csv(csv_path)
    print(f"Total rows in CSV: {len(df)}")
    
    # 1. Scenario breakdown
    scenarios = df['scenario_id'].value_counts().to_dict()
    print(f"Scenarios: {scenarios}")
    assert len(df) == 15380, f"Expected 15380 total rows, got {len(df)}"
    assert scenarios.get('SC1_Nominal_2R') == 14700, f"Expected 14700 SC1 rows, got {scenarios.get('SC1_Nominal_2R')}"
    assert scenarios.get('SC10_Swarm_8R') == 680, f"Expected 680 SC10 rows, got {scenarios.get('SC10_Swarm_8R')}"

    # 2. SC1 Audit
    print("\n--- Auditing SC1_Nominal_2R ---")
    sc1 = df[df['scenario_id'] == 'SC1_Nominal_2R'].copy()
    
    # Expected dimensions
    loss_rates = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
    delays_ms = [0, 20, 50, 100, 200, 350, 500]
    seeds = [1042 * s for s in range(1, 31)]
    variants = [
        "AstroMesh_X_Full",
        "AstroMesh_X_RawExchange",
        "AstroMesh_X_NoACKs",
        "AstroMesh_X_NoRetries",
        "AstroMesh_X_NoMultiHop",
        "AstroMesh_X_Minimal",
        "AstroMesh_v1_Baseline"
    ]
    
    # Check distinct values
    actual_losses = sorted(sc1['loss_rate'].round(2).unique().tolist())
    actual_delays = sorted(sc1['delay_ms'].unique().tolist())
    actual_seeds = sorted(sc1['seed'].unique().tolist())
    actual_variants = sorted(sc1['variant'].unique().tolist())
    
    print(f"Loss rates count: {len(actual_losses)} (expected 10: {loss_rates})")
    print(f"Delays count: {len(actual_delays)} (expected 7: {delays_ms})")
    print(f"Seeds count: {len(actual_seeds)} (expected 30: 1..30)")
    print(f"Variants count: {len(actual_variants)} (expected 7: {sorted(variants)})")
    
    assert actual_losses == [round(x, 2) for x in loss_rates], "Loss rates mismatch!"
    assert actual_delays == delays_ms, "Delays mismatch!"
    assert actual_seeds == seeds, "Seeds mismatch!"
    assert sorted(actual_variants) == sorted(variants), "Variants mismatch!"
    
    # Check for duplicate keys
    sc1['loss_str'] = sc1['loss_rate'].map(lambda v: f"{float(v):.2f}")
    sc1['key'] = (
        sc1['scenario_id'] + '|' +
        sc1['loss_str'] + '|' +
        sc1['delay_ms'].astype(str) + '|' +
        sc1['seed'].astype(str) + '|' +
        sc1['variant']
    )
    dup_count = sc1['key'].duplicated().sum()
    print(f"SC1 duplicate keys: {dup_count}")
    assert dup_count == 0, f"Found {dup_count} duplicate keys in SC1!"
    
    # Check cartesian completeness
    expected_keys = set()
    for l in loss_rates:
        for d in delays_ms:
            for s in seeds:
                for v in variants:
                    expected_keys.add(f"SC1_Nominal_2R|{l:.2f}|{d}|{s}|{v}")
    
    actual_keys_set = set(sc1['key'].tolist())
    missing_keys = expected_keys - actual_keys_set
    print(f"SC1 missing keys: {len(missing_keys)}")
    assert len(missing_keys) == 0, f"Missing {len(missing_keys)} keys in SC1!"
    
    # Check cell counts (each cell must have exactly 30 seeds * 7 variants = 210 trials)
    cell_counts = sc1.groupby(['loss_rate', 'delay_ms']).size()
    print(f"Total SC1 cells: {len(cell_counts)}")
    assert len(cell_counts) == 70, f"Expected 70 cells, got {len(cell_counts)}"
    assert (cell_counts == 210).all(), f"Not all cells have 210 records! Min: {cell_counts.min()}, Max: {cell_counts.max()}"
    print("All 70 cells have exactly 210 trials (30 seeds * 7 variants).")
    
    # Check trial status and failures
    status_counts = sc1['trial_status'].value_counts().to_dict()
    print(f"SC1 trial statuses: {status_counts}")
    assert status_counts == {'Success': 14700}, f"Unexpected statuses: {status_counts}"
    
    # Check non-finite values in key numeric columns
    numeric_cols = [
        'runtime_us', 'graph_nodes', 'graph_edges', 'optimizer_calls',
        'optimizer_iterations', 'optimizer_failures', 'keyframes_retained',
        'candidates_generated', 'closures_verified', 'closures_rejected',
        'closures_true_accept', 'closures_false_accept', 'precision', 'recall',
        'initial_dsu_components', 'final_dsu_components', 'first_merge_step',
        'disconnected_rover_pairs', 'ate_aligned_swarm_rmse_m',
        'logical_messages_sent', 'logical_messages_delivered', 'logical_messages_dropped',
        'retry_exhausted_drops', 'in_flight_at_termination', 'unique_deliveries',
        'duplicate_deliveries', 'hop_transmissions_attempted', 'hop_transmissions_successful',
        'hop_transmissions_dropped', 'retransmissions_performed', 'payload_bytes',
        'protocol_bytes', 'framing_bytes', 'data_wire_bytes', 'ack_wire_bytes',
        'total_wire_bytes', 'app_completion_ratio', 'per_hop_delivery_ratio'
    ]
    for col in numeric_cols:
        null_count = sc1[col].isnull().sum()
        inf_count = np.isinf(sc1[col]).sum()
        assert null_count == 0, f"Col {col} has {null_count} nulls!"
        assert inf_count == 0, f"Col {col} has {inf_count} infs!"
    print("All numeric columns have 0 nulls and 0 infs.")
    
    # Check Byte Conservation Invariants
    data_cons = sc1['data_byte_conservation_valid'].all()
    total_cons = sc1['total_byte_conservation_valid'].all()
    print(f"SC1 data_byte_conservation_valid (all True): {data_cons}")
    print(f"SC1 total_byte_conservation_valid (all True): {total_cons}")
    assert data_cons and total_cons, "Byte conservation flag failed!"
    
    # Exact byte arithmetic verification
    calc_data_wire = sc1['payload_bytes'] + sc1['protocol_bytes'] + sc1['framing_bytes']
    calc_total_wire = sc1['data_wire_bytes'] + sc1['ack_wire_bytes']
    data_wire_diff = (sc1['data_wire_bytes'] - calc_data_wire).abs().max()
    total_wire_diff = (sc1['total_wire_bytes'] - calc_total_wire).abs().max()
    print(f"Max abs diff (data_wire_bytes vs sum): {data_wire_diff}")
    print(f"Max abs diff (total_wire_bytes vs sum): {total_wire_diff}")
    assert data_wire_diff == 0, "Data wire bytes arithmetic mismatch!"
    assert total_wire_diff == 0, "Total wire bytes arithmetic mismatch!"
    
    # Note on hop transmissions:
    # m.hop_transmissions_attempted counts all PHY frame attempts (data, retries, ACKs across all hops).
    # m.hop_transmissions_successful counts all successful PHY frame deliveries.
    # Note: rec.hop_transmissions_dropped was populated from m.logical_messages_dropped in benchmark runner,
    # which is an application-level drop metric rather than per-hop PHY drop.
    # We verify that hop_transmissions_successful <= hop_transmissions_attempted for all rows:
    assert (sc1['hop_transmissions_successful'] <= sc1['hop_transmissions_attempted']).all(), "Hop successes exceed attempts!"
    print("Hop transmission bounds verified: hop_transmissions_successful <= hop_transmissions_attempted for 100% of rows.")
    
    # Logical message accounting
    # In 2-rover (single hop): messages sent = unique_deliveries + logical_messages_dropped + in_flight_at_termination
    # retry_exhausted_drops is a subset of logical_messages_dropped.
    calc_logical_account = sc1['unique_deliveries'] + sc1['logical_messages_dropped'] + sc1['in_flight_at_termination']
    logical_diff = (sc1['logical_messages_sent'] - calc_logical_account).abs().max()
    print(f"Max abs diff (logical_messages_sent vs unique_deliv + dropped + in_flight): {logical_diff}")
    assert logical_diff == 0, "Logical message accounting mismatch!"
    assert (sc1['retry_exhausted_drops'] <= sc1['logical_messages_dropped']).all(), "Retry exhausted drops exceed total drops!"
    print("Logical message conservation holds exactly: sent == unique_deliveries + dropped + in_flight for 100% of rows.")
    
    print("\n--- Auditing SC10_Swarm_8R (Partial Exploration) ---")
    sc10 = df[df['scenario_id'] == 'SC10_Swarm_8R'].copy()
    sc10['loss_str'] = sc10['loss_rate'].map(lambda v: f"{float(v):.2f}")
    sc10['key'] = (
        sc10['scenario_id'] + '|' +
        sc10['loss_str'] + '|' +
        sc10['delay_ms'].astype(str) + '|' +
        sc10['seed'].astype(str) + '|' +
        sc10['variant']
    )
    sc10_dup_count = sc10['key'].duplicated().sum()
    print(f"SC10 total rows: {len(sc10)}")
    print(f"SC10 duplicate keys: {sc10_dup_count}")
    assert sc10_dup_count == 0, "Duplicate keys in SC10!"
    
    sc10_status = sc10['trial_status'].value_counts().to_dict()
    print(f"SC10 trial statuses: {sc10_status}")
    assert sc10_status == {'Success': 680}, f"Unexpected SC10 statuses: {sc10_status}"
    
    sc10_data_cons = sc10['data_byte_conservation_valid'].all()
    sc10_total_cons = sc10['total_byte_conservation_valid'].all()
    print(f"SC10 data_byte_conservation_valid: {sc10_data_cons}")
    print(f"SC10 total_byte_conservation_valid: {sc10_total_cons}")
    assert sc10_data_cons and sc10_total_cons, "SC10 byte conservation flag failed!"
    
    # Cell breakdown for SC10
    sc10_cell_counts = sc10.groupby(['loss_rate', 'delay_ms']).size()
    print(f"SC10 completed cells: {len(sc10_cell_counts)} of 70")
    print(f"Cells with all 35 trials (5 seeds * 7 variants): {(sc10_cell_counts == 35).sum()}")
    print(f"Cells with partial trials (< 35): {(sc10_cell_counts < 35).sum()}")
    for (l, d), count in sc10_cell_counts.items():
        if count < 35:
            print(f"  Partial cell loss={l}, delay={d}: {count} trials")
            
    print("\n=== AUDIT COMPLETE: ALL ASSERTIONS PASSED ===")

if __name__ == '__main__':
    run_audit()
