import pandas as pd
import re
from datetime import datetime

df = pd.read_csv('benchmark_phase62_trials.csv')
sc1 = df[df['scenario_id'] == 'SC1_Nominal_2R']
sc10 = df[df['scenario_id'] == 'SC10_Swarm_8R']

print(f"Total Rows: {len(df)}")
print(f"SC1 Rows: {len(sc1)} / 14700")
print(f"SC10 Rows: {len(sc10)} / 2450")

# Completed cells in SC10
sc10_cell_counts = sc10.groupby(['loss_rate', 'delay_ms']).size()
completed_cells = len(sc10_cell_counts[sc10_cell_counts == 35])
partial_cells = len(sc10_cell_counts[sc10_cell_counts < 35])
print(f"SC10 Fully Completed Cells (35 runs/cell): {completed_cells} / 70")
print(f"SC10 Partially Completed Cells: {partial_cells}")

# Parse task log for exact cell runtimes
log_path = r'C:\Users\TANISHKA SAWANT\.gemini\antigravity\brain\b74971ce-a1b4-4378-b0e1-572a83b3a31f\.system_generated\tasks\task-927.log'
cell_runtimes = []
cell_names = []

with open(log_path, 'r', encoding='utf-8') as f:
    current_cond = ""
    for line in f:
        m_cond = re.search(r'Channel Condition:\s*(Loss=.*?,\s*Delay=.*?ms)', line)
        if m_cond:
            current_cond = m_cond.group(1)
        m_time = re.search(r'Cell runtime:\s*(\d+)\s*ms', line)
        if m_time:
            runtime_ms = int(m_time.group(1))
            cell_runtimes.append(runtime_ms)
            cell_names.append(current_cond)

print(f"\nTotal Logged Completed Cells in SC10: {len(cell_runtimes)}")
for name, rt in zip(cell_names, cell_runtimes):
    print(f"  {name}: {rt / 1000.0:.2f} s ({rt / 60000.0:.2f} min)")

if len(cell_runtimes) >= 5:
    last_5 = cell_runtimes[-5:]
    last_5_names = cell_names[-5:]
    avg_last_5_ms = sum(last_5) / 5.0
    print(f"\nLast 5 Completed Cells:")
    for name, rt in zip(last_5_names, last_5):
        print(f"  - {name}: {rt / 1000.0:.2f} s ({rt / 60000.0:.2f} min)")
    print(f"Average of Last 5 Cells: {avg_last_5_ms / 1000.0:.2f} s ({avg_last_5_ms / 60000.0:.2f} min)")

    remaining_cells = 70 - len(cell_runtimes)
    est_remaining_sec = remaining_cells * (avg_last_5_ms / 1000.0)
    print(f"\nRemaining Cells: {remaining_cells} / 70")
    print(f"Estimated Remaining Time: {est_remaining_sec / 3600.0:.2f} hours ({est_remaining_sec / 60.0:.1f} minutes)")

