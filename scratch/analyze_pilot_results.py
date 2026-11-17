import pandas as pd

df = pd.read_csv('benchmark_phase62_pilot_trials.csv')

print("=== PILOT EXPERIMENTAL FINDINGS ===")
print("\n--- Mean Total Wire Bytes by Variant & Delay (SC1) ---")
sc1 = df[df['scenario_id'] == 'SC1_Nominal_2R']
pivot_wire = sc1.pivot_table(index='variant', columns='delay_ms', values='total_wire_bytes', aggfunc='mean')
print(pivot_wire.round(1))

print("\n--- Mean In-Flight at Termination by Variant & Delay (SC1) ---")
pivot_flight = sc1.pivot_table(index='variant', columns='delay_ms', values='in_flight_at_termination', aggfunc='mean')
print(pivot_flight.round(2))

print("\n--- Mean Retransmissions by Variant & Delay (SC1) ---")
pivot_retries = sc1.pivot_table(index='variant', columns='delay_ms', values='retransmissions_performed', aggfunc='mean')
print(pivot_retries.round(2))

print("\n--- Mean Final DSU Components by Variant & Packet Loss (SC10) ---")
sc10 = df[df['scenario_id'] == 'SC10_Swarm_8R']
pivot_dsu = sc10.pivot_table(index='variant', columns='loss_rate', values='final_dsu_components', aggfunc='mean')
print(pivot_dsu.round(2))

print("\n--- Mean Aligned ATE RMSE (m) by Variant & Packet Loss (SC1) ---")
pivot_ate = sc1.pivot_table(index='variant', columns='loss_rate', values='ate_aligned_swarm_rmse_m', aggfunc='mean')
print(pivot_ate.round(4))

