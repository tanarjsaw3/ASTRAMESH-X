import csv

with open('benchmark_phase61_pilot_summary.csv') as f:
    r = list(csv.DictReader(f))

print(f"{'Scenario':<22} | {'Variant':<25} | {'Metric':<18} | {'Full':<10} | {'Variant':<10} | {'Mean Diff':<10} | {'95% Bonf CI':<24} | {'Conclusion'}")
print("-" * 135)
for row in r[:24]:
    ci = f"[{float(row['ci95_bonf_low']):.2f}, {float(row['ci95_bonf_high']):.2f}]"
    print(f"{row['scenario_id']:<22} | {row['variant_b']:<25} | {row['metric_name']:<18} | {float(row['ref_mean']):<10.2f} | {float(row['abl_mean']):<10.2f} | {float(row['paired_diff_mean']):<10.2f} | {ci:<24} | {row['conclusion']}")
