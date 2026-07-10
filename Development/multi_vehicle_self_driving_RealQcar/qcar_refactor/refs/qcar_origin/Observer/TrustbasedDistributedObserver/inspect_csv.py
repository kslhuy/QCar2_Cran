import csv
import math
import os

DIR = os.path.dirname(os.path.abspath(__file__))
fpath = os.path.join(DIR, "trust_weight_log_V0.csv")

cleaned_lines = []
with open(fpath, "r", encoding="utf-8", errors="replace") as f:
    for line in f:
        line = line.replace('\x00', '')
        if len(line) < 131072:
            cleaned_lines.append(line)

r = csv.DictReader(cleaned_lines)
rows = list(r)

print("Total rows:", len(rows))
cols = [
    "trust_1", "v_score_1", "d_score_1", "a_score_1",
    "h_score_1", "b_score_1", "q_factor_1",
    "local_trust_1", "global_trust_1",
    "gamma_host_1", "gamma_local_peer_1", "gamma_self_1",
    "d_host_mean_1", "d_local_mean_1", "d_self_1",
    "w_neighbor_1",
]

for idx in [0, 5, 50, 100, 200]:
    if idx >= len(rows):
        break
    row = rows[idx]
    print(f"\n--- Row {idx} ---")
    for c in cols:
        print(f"  {c}: {row.get(c, 'MISSING')}")

# Count NaN in key cols
nan_counts = {}
for c in cols:
    cnt = sum(1 for row in rows if row.get(c, "") == "nan")
    nan_counts[c] = cnt

print("\n--- NaN counts ---")
for c, cnt in nan_counts.items():
    print(f"  {c}: {cnt}/{len(rows)}")

print("\n--- Summary stats ---")
for c in cols:
    vals = []
    for row in rows:
        raw = row.get(c, "")
        try:
            x = float(raw)
            if not math.isnan(x):
                vals.append(x)
        except (TypeError, ValueError):
            continue
    if not vals:
        print(f"  {c}: no finite data")
        continue
    frac_low = sum(1 for x in vals if x < 0.5) / len(vals)
    print(
        f"  {c}: mean={sum(vals)/len(vals):.4f}, min={min(vals):.4f}, "
        f"max={max(vals):.4f}, frac<0.5={frac_low:.3f}"
    )
