import pandas as pd

df = pd.read_csv('benchmark_phase62_trials.csv')
sc1 = df[df['scenario_id'] == 'SC1_Nominal_2R']
sc10 = df[df['scenario_id'] == 'SC10_Swarm_8R']

print("=== SC1 Hop Transmissions & Retransmissions ===")
full_sc1 = sc1[sc1['variant'] == 'AstroMesh_X_Full']
piv_sc1_hops = full_sc1.pivot_table(index='loss_rate', columns='delay_ms', values='hop_transmissions_attempted', aggfunc='mean')
piv_sc1_retries = full_sc1.pivot_table(index='loss_rate', columns='delay_ms', values='retransmissions_performed', aggfunc='mean')
piv_sc1_wire = full_sc1.pivot_table(index='loss_rate', columns='delay_ms', values='total_wire_bytes', aggfunc='mean')

print("--- SC1 Total Wire Bytes ---")
print(piv_sc1_wire.round(1))
print("--- SC1 Retransmissions Performed ---")
print(piv_sc1_retries.round(1))
print("--- SC1 Hop Transmissions Attempted ---")
print(piv_sc1_hops.round(1))

# Compare Full vs NoMultiHop in SC1
sc1_nomultihop = sc1[sc1['variant'] == 'AstroMesh_X_NoMultiHop']
wire_diff = (full_sc1['total_wire_bytes'].values - sc1_nomultihop['total_wire_bytes'].values)
print(f"Max absolute wire difference between Full and NoMultiHop in SC1: {abs(wire_diff).max()}")

if len(sc10) > 0:
    print("\n=== SC10 Hop Transmissions & Retransmissions (Interim) ===")
    full_sc10 = sc10[sc10['variant'] == 'AstroMesh_X_Full']
    nohop_sc10 = sc10[sc10['variant'] == 'AstroMesh_X_NoMultiHop']
    print(f"SC10 Full Mean Total Wire Bytes: {full_sc10['total_wire_bytes'].mean():.1f}")
    print(f"SC10 NoMultiHop Mean Total Wire Bytes: {nohop_sc10['total_wire_bytes'].mean():.1f}")
    wire_diff_sc10 = full_sc10['total_wire_bytes'].mean() - nohop_sc10['total_wire_bytes'].mean()
    print(f"SC10 Mean Wire Overhead from Multi-Hop Forwarding: {wire_diff_sc10:.1f} bytes ({wire_diff_sc10/full_sc10['total_wire_bytes'].mean()*100:.1f}%)")

