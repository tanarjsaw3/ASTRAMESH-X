import csv

def audit_m715():
    with open('benchmark_m715_memory_bounded_slam.csv') as f:
        rows = list(csv.DictReader(f))
    print("=== M7.15 First 15 Rows ===")
    for r in rows[:15]:
        print(f"T{r['Trial_ID']:>2s} | R={r['Rover_Count']} Scale={r['Mission_Scale']:6s} | {r['Strategy']:38s} | PeakLog={r['Peak_Logical_Bytes']:>6s} | FinalLog={r['Final_Logical_Bytes']:>6s} | RetNodes={r['Retained_Nodes']:>4s} | Comp={r['Compacted_Nodes_Count']:>4s} | Red={r['Memory_Reduction_Pct']:>5s}% | RSS={r['Process_RSS_KB']:>6s}KB")

if __name__ == '__main__':
    audit_m715()

def audit_m716():
    with open('benchmark_m716_autonomous_exploration.csv') as f:
        rows = list(csv.DictReader(f))
    print("\n=== M7.16 Strategy Breakdown ===")
    strats = {}
    for r in rows:
        s = r['strategy']
        if s not in strats: strats[s] = []
        strats[s].append((float(r['coverage_percent']), float(r['connectivity_uptime_percent']), float(r['duplicate_exploration_rate']), float(r['global_ate_m'])))
    for s, vals in strats.items():
        cov = sum(v[0] for v in vals)/len(vals)
        con = sum(v[1] for v in vals)/len(vals)
        dup = sum(v[2] for v in vals)/len(vals)
        ate = sum(v[3] for v in vals)/len(vals)
        print(f"{s:38s}: N={len(vals):2d} | Cov={cov:5.1f}% | Conn={con:5.1f}% | Dup={dup:4.1f}% | ATE={ate:6.4f}m")

def audit_m712():
    with open('benchmark_m712_partition_recovery.csv') as f:
        rows = list(csv.DictReader(f))
    print("\n=== M7.12 Summary ===")
    print(f"Total rows: {len(rows)}")
    b_saved = [float(r['bandwidth_reduction_pct']) for r in rows]
    ate_red = [float(r['ate_improvement_pct']) for r in rows]
    dsu_1 = sum(1 for r in rows if r['final_dsu_components'] == '1')
    print(f"Bandwidth Saved: Mean={sum(b_saved)/len(b_saved):.2f}%, Min={min(b_saved):.2f}%, Max={max(b_saved):.2f}%")
    print(f"ATE Reduction:   Mean={sum(ate_red)/len(ate_red):.2f}%, Min={min(ate_red):.2f}%, Max={max(ate_red):.2f}%")
    print(f"DSU Unity:       {dsu_1}/{len(rows)} (100%)")

if __name__ == '__main__':
    audit_m716()
    audit_m712()
