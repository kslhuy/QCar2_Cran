"""Build a compact, reproducible result table and publication-style figures."""
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent


def aggregate(payload):
    rows = payload["results"]
    output = []
    for scenario in dict.fromkeys(row["scenario"] for row in rows):
        for name in dict.fromkeys(row["estimator"] for row in rows):
            group = [r for r in rows if r["scenario"] == scenario and r["estimator"] == name]
            if not group:
                continue
            row = dict(scenario=scenario, estimator=name, drives=len(group))
            for key in group[0]:
                if "rmse" in key:
                    row[key] = float(np.sqrt(np.mean([r[key]**2 for r in group])))
                elif key.startswith("runtime"):
                    row[key] = float(np.median([r[key] for r in group]))
                elif key.endswith("max_m"):
                    row[key] = float(max(r[key] for r in group))
            output.append(row)
    return output


def main():
    plt.rcParams.update({"font.size": 16, "xtick.labelsize": 14, "ytick.labelsize": 14})
    results = HERE/"results"
    payload = json.loads((results/"heldout.json").read_text())
    rows = aggregate(payload)
    summary = dict(protocol=payload["protocol"], partition=payload["partition"],
                   checkpoint_sha256=payload["checkpoint_sha256"],
                   aggregation="root mean square across equal-length drives; median of per-drive latency statistics",
                   results=rows)
    for name in ("closed_loop", "stress_mismatch", "native_rate"):
        path = results/f"{name}.json"
        if path.exists():
            summary[name] = aggregate(json.loads(path.read_text()))
    (results/"summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with (results/"summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    scenarios = list(dict.fromkeys(r["scenario"] for r in rows))
    names = ["ekf", "legacy", "analytic_ablation", "robust"]
    labels = ["Existing EKF", "Legacy GRU", "Analytical ablation", "Revised RobustKLNet"]
    colors = ["#727b83", "#b56576", "#e2ae45", "#137e85"]
    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True, layout="constrained")
    for ax, key, ylabel in zip(axes, ["position_rmse_m", "heading_rmse_rad", "speed_rmse_mps"],
                              ["Position RMSE (m)", "Heading RMSE (rad)", "Speed RMSE (m/s)"]):
        for j, (name, label, color) in enumerate(zip(names, labels, colors)):
            vals = [next(r[key] for r in rows if r["scenario"] == s and r["estimator"] == name) for s in scenarios]
            ax.bar(np.arange(len(scenarios))+(j-1.5)*.2, vals, width=.19, label=label, color=color)
        ax.set_yscale("log")
        ax.set_ylabel(ylabel)
        ax.grid(axis="y", alpha=.2)
    axes[0].legend(ncol=4, loc="upper center", bbox_to_anchor=(.5, 1.25), frameon=False, fontsize=13)
    axes[-1].set_xticks(np.arange(len(scenarios)), [s.replace("_", "\n") for s in scenarios], fontsize=14)
    fig.suptitle(f"Continuous MockQCar estimation: {len(payload['seeds'])} held-out drives per scenario\n"
                 f"{payload['duration']:g} s per drive, {1/payload['dt']:g} Hz estimator, {payload['gps_hz']:g} Hz GPS", fontsize=17)
    fig.savefig(results/"heldout_comparison.png", dpi=180)
    plt.close(fig)
    if "closed_loop" in summary:
        loop_rows = summary["closed_loop"]
        loop_scenarios = list(dict.fromkeys(r["scenario"] for r in loop_rows))
        fig, axes = plt.subplots(2, 1, figsize=(11, 7.2), sharex=True, layout="constrained")
        for ax, key, label_y in zip(axes, ["tracking_rmse_m", "speed_tracking_rmse_mps"],
                                    ["Path tracking RMSE (m)", "Speed tracking RMSE (m/s)"]):
            for j, (name, label, color) in enumerate(zip(["ekf", "robust"], [labels[0], labels[-1]], [colors[0], colors[-1]])):
                vals = [next(r[key] for r in loop_rows if r["scenario"] == s and r["estimator"] == name) for s in loop_scenarios]
                ax.bar(np.arange(len(vals))+(j-.5)*.34, vals, .32, color=color, label=label)
            ax.set_ylabel(label_y)
            ax.grid(axis="y", alpha=.2)
        axes[-1].set_xticks(np.arange(len(loop_scenarios)), [s.replace("_", "\n") for s in loop_scenarios])
        axes[0].set_title("Closed-loop MockQCar with repository Stanley and PID controllers")
        axes[0].legend(frameon=False)
        fig.savefig(results/"closed_loop_tracking.png", dpi=180)
        plt.close(fig)
    tex = [r"\begin{table}[htbp]\centering\small",
           r"\begin{tabular}{lrrrr}\toprule",
           r"Scenario & EKF & Legacy GRU & Analytical & Revised \\\midrule"]
    for s in scenarios:
        vals = [next(r["position_rmse_m"] for r in rows if r["scenario"] == s and r["estimator"] == n) for n in names]
        tex.append(s.replace("_", r"\_")+" & "+" & ".join(f"{v:.4f}" for v in vals)+r" \\")
    tex += [r"\bottomrule\end{tabular}", r"\caption{Position RMSE in metres, continuous held-out simulation. Equal-length drives are pooled by squared error.}\end{table}"]
    (results/"heldout_table.tex").write_text("\n".join(tex), encoding="utf-8")
    for s in scenarios:
        e = next(r for r in rows if r["scenario"] == s and r["estimator"] == "ekf")
        r = next(r for r in rows if r["scenario"] == s and r["estimator"] == "robust")
        print(f"{s:15s} EKF={e['position_rmse_m']:.4f} Robust={r['position_rmse_m']:.4f} m")


if __name__ == "__main__":
    main()
