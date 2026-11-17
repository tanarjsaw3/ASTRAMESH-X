import csv

with open('benchmark_phase61_pilot_summary.csv') as f:
    r = list(csv.DictReader(f))

for sc in ['SC2_Loss10_2R', 'SC4_Loss50_2R', 'SC5_Partition_Recon_2R', 'SC9_Swarm_4R', 'SC10_Swarm_8R']:
    print(f"\n=== Scenario: {sc} ===")
    for row in r:
        if row['scenario_id'] == sc and row['metric_name'] in ['Total_Transmitted_Wire_Bytes', 'Application_Completion_Ratio', 'Final_DSU_Components']:
            ci = f"[{float(row['ci95_bonf_low']):.1f}, {float(row['ci95_bonf_high']):.1f}]"
            print(f"  {row['variant_b']:<25} | {row['metric_name']:<28} | Full:{float(row['ref_mean']):<10.1f} | Var:{float(row['abl_mean']):<10.1f} | Diff:{float(row['paired_diff_mean']):<10.1f} | {row['conclusion']}")

