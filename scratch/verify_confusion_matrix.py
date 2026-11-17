import pandas as pd

df = pd.read_csv('benchmark_distributed_trajectory_remediated.csv')

tp = df['true_positives'].sum()
fp = df['false_positives'].sum()
tn = df['true_negatives'].sum()
fn = df['false_negatives'].sum()

proposals_sent = df['proposals_sent'].sum()
proposals_received = df['proposals_received'].sum()
loop_closures_verified = df['loop_closures_verified'].sum()
loop_closures_rejected = df['loop_closures_rejected'].sum()

print("--- CONFUSION MATRIX & PROPOSAL AUDIT ---")
print(f"True Positives (TP):  {tp}")
print(f"False Positives (FP): {fp}")
print(f"True Negatives (TN):  {tn}")
print(f"False Negatives (FN): {fn}")
print(f"Total Proposals Sent:     {proposals_sent}")
print(f"Total Proposals Received: {proposals_received}")
print(f"Total Accepted (Verified):{loop_closures_verified}")
print(f"Total Rejected:           {loop_closures_rejected}")

# Exact rates
precision = tp / (tp + fp) if (tp + fp) > 0 else 1.0
recall = tp / (tp + fn) if (tp + fn) > 0 else 1.0
specificity = tn / (tn + fp) if (tn + fp) > 0 else 1.0
fpr = fp / (fp + tn) if (fp + tn) > 0 else 0.0
fnr = fn / (fn + tp) if (fn + tp) > 0 else 0.0

print(f"\nPrecision:   {precision:.6f} ({precision*100:.2f}%)")
print(f"Recall:      {recall:.6f} ({recall*100:.2f}%)")
print(f"Specificity: {specificity:.6f} ({specificity*100:.2f}%)")
print(f"FPR:         {fpr:.6f} ({fpr*100:.2f}%)")
print(f"FNR:         {fnr:.6f} ({fnr*100:.2f}%)")

