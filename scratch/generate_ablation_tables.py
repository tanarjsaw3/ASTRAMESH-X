import csv

def generate():
    try:
        with open('benchmark_phase61_summary.csv') as f:
            r = list(csv.DictReader(f))
    except Exception as e:
        print(f"Summary not yet ready: {e}")
        return

    scenarios = [
        'SC1_Nominal_2R',
        'SC2_Loss10_2R',
        'SC3_Loss30_2R',
        'SC4_Loss50_2R',
        'SC5_Partition_Recon_2R',
        'SC6_ConstrainedBudget_2R',
        'SC7_DegenerateGeometry_2R',
        'SC8_LowOverlap_2R',
        'SC9_Swarm_4R',
        'SC10_Swarm_8R'
    ]

    metrics = [
        'Total_Transmitted_Wire_Bytes',
        'Application_Completion_Ratio',
        'Verified_Loop_Closures',
        'Final_DSU_Components',
        'ATE_Aligned_Swarm_RMSE'
    ]

    for m in metrics:
        print(f"\n### Metric: {m}\n")
        print("| Scenario | AstroMesh-X Full | RawExchange | NoACKs | NoRetries | NoMultiHop | Minimal | Baseline v1 |")
        print("| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |")

        for sc in scenarios:
            # find full mean
            full_mean = None
            var_means = {}
            for row in r:
                if row['scenario_id'] == sc and row['metric_name'] == m:
                    full_mean = float(row['ref_mean'])
                    var_name = row['variant_b']
                    var_means[var_name] = float(row['abl_mean'])
            
            if full_mean is not None:
                raw_ex = f"{var_means.get('AstroMesh_X_RawExchange', 0.0):.1f}"
                no_ack = f"{var_means.get('AstroMesh_X_NoACKs', 0.0):.1f}"
                no_ret = f"{var_means.get('AstroMesh_X_NoRetries', 0.0):.1f}"
                no_hop = f"{var_means.get('AstroMesh_X_NoMultiHop', 0.0):.1f}"
                min_tr = f"{var_means.get('AstroMesh_X_Minimal', 0.0):.1f}"
                base_v1 = f"{var_means.get('AstroMesh_v1_Baseline', 0.0):.1f}"
                print(f"| **{sc}** | {full_mean:.1f} | {raw_ex} | {no_ack} | {no_ret} | {no_hop} | {min_tr} | {base_v1} |")

if __name__ == '__main__':
    generate()

