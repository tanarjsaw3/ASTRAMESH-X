import pandas as pd
import numpy as np

trials_path = 'benchmark_phase62_pilot_trials.csv'
summary_path = 'benchmark_phase62_pilot_summary.csv'
failures_path = 'benchmark_phase62_pilot_failures.csv'

df_trials = pd.read_csv(trials_path)
print(f"=== Auditing {trials_path} ===")
print(f"Total trial rows: {len(df_trials)}")
assert len(df_trials) == 252, f"Expected 252 rows, got {len(df_trials)}"

# Check duplicates
keys = df_trials['scenario_id'] + '|' + df_trials['loss_rate'].round(2).astype(str) + '|' + df_trials['delay_ms'].astype(str) + '|' + df_trials['seed'].astype(str) + '|' + df_trials['variant']
duplicates = keys.duplicated().sum()
print(f"Duplicate trial keys: {duplicates}")
assert duplicates == 0, f"Found {duplicates} duplicate keys!"

# Check byte conservation
data_cons = df_trials['data_byte_conservation_valid'].all()
total_cons = df_trials['total_byte_conservation_valid'].all()
print(f"Data byte conservation (100%): {data_cons}")
print(f"Total byte conservation (100%): {total_cons}")
assert data_cons and total_cons, "Byte conservation check failed!"

# Numerical verification of wire bytes
calc_data_wire = df_trials['payload_bytes'] + df_trials['protocol_bytes'] + df_trials['framing_bytes']
calc_total_wire = df_trials['data_wire_bytes'] + df_trials['ack_wire_bytes']
assert (df_trials['data_wire_bytes'] == calc_data_wire).all(), "Data wire bytes mismatch!"
assert (df_trials['total_wire_bytes'] == calc_total_wire).all(), "Total wire bytes mismatch!"

# Check delivery ratios
valid_app = df_trials[df_trials['app_completion_ratio_valid']]
assert (valid_app['app_completion_ratio'] >= 0.0).all() and (valid_app['app_completion_ratio'] <= 1.0).all()

valid_hop = df_trials[df_trials['per_hop_delivery_ratio_valid']]
assert (valid_hop['per_hop_delivery_ratio'] >= 0.0).all() and (valid_hop['per_hop_delivery_ratio'] <= 1.0).all()

# Check trial statuses
statuses = df_trials['trial_status'].value_counts().to_dict()
print(f"Trial statuses: {statuses}")
assert statuses == {'Success': 252}, f"Unexpected statuses: {statuses}"

# Check failures file
df_fail = pd.read_csv(failures_path)
print(f"Failures recorded in {failures_path}: {len(df_fail)}")
assert len(df_fail) == 0

# Check summary file
df_sum = pd.read_csv(summary_path)
print(f"Total summary comparisons in {summary_path}: {len(df_sum)}")
assert len(df_sum) > 0

print("ALL PILOT AUDIT CHECKS PASSED PERFECTLY!")

