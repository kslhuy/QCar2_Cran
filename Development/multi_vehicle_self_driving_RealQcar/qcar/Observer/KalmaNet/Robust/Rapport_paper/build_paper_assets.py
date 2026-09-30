"""Rebuild manuscript tables and vector figures from saved simulation evidence.

Run with the Qcar Python environment. This does not train or change the filter.
The diagnostic replay is checked against the saved heldout state trajectory.
"""
from pathlib import Path
import hashlib
import json
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PAPER = Path(__file__).resolve().parent
ROBUST = PAPER.parent
OUT = PAPER / "paper_assets"
OUT.mkdir(exist_ok=True)
sys.path.insert(0, str(ROBUST))

LABELS = {
    "clean": "Clean", "gps_jump": "GPS jump", "gps_ramp": "GPS ramp",
    "gps_freeze": "GPS freeze", "gps_dropout": "GPS dropout",
    "gps_noise": "GPS noise", "heading_bias": "Heading bias",
    "wheel_bias": "Tachometer bias", "wheel_scale": "Tachometer scale",
    "wheel_freeze": "Tachometer freeze", "imu_bias": "IMU bias",
    "steering_bias": "Steering bias", "gps_wheel": "GPS + tachometer",
}
COLORS = {"ekf": "#C26420", "robust": "#147D64", "analytic_ablation": "#376DA3"}
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9,
                     "axes.spines.top": False, "axes.spines.right": False,
                     "pdf.fonttype": 42, "savefig.bbox": "tight"})


def pooled(rows, scenario, estimator, key):
    values = [r[key] for r in rows if r["scenario"] == scenario and r["estimator"] == estimator]
    return float(np.sqrt(np.mean(np.square(values))))


def table(filename, columns, headers, rows):
    text = [r"\begin{tabular}{" + columns + "}", r"\toprule",
            " & ".join(headers) + r" \\", r"\midrule"]
    text += [" & ".join(row) + r" \\" for row in rows]
    text += [r"\bottomrule", r"\end{tabular}"]
    (OUT / filename).write_text("\n".join(text) + "\n", encoding="utf-8")


def main():
    names = ["heldout", "closed_loop", "stress_mismatch", "native_rate"]
    reports = {name: json.loads((ROBUST / f"results/{name}.json").read_text()) for name in names}
    checkpoint = ROBUST / "models/innovation_trust_sim.npz"
    digest = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    assert all(d["checkpoint_sha256"] == digest for d in reports.values())
    assert len(reports["heldout"]["results"]) == 156
    assert len(reports["closed_loop"]["results"]) == 54
    evidence = {}
    for name in names:
        for suffix in (".json", ".npz"):
            source = ROBUST / f"results/{name}{suffix}"
            evidence[str(source.relative_to(ROBUST))] = hashlib.sha256(source.read_bytes()).hexdigest()
    evidence["models/innovation_trust_sim.npz"] = digest
    history_path = ROBUST / "models/innovation_trust_sim.history.json"
    evidence["models/innovation_trust_sim.history.json"] = hashlib.sha256(history_path.read_bytes()).hexdigest()
    (OUT / "evidence_manifest.json").write_text(json.dumps(evidence, indent=2), encoding="utf-8")

    rows = reports["heldout"]["results"]
    scenarios = list(dict.fromkeys(r["scenario"] for r in rows))
    order = ("ekf", "legacy", "analytic_ablation", "robust")
    table("position_table.tex", "lrrrr", ["Condition", "EKF", "Legacy GRU", "Analytical", "Hybrid"],
          [[LABELS[s]] + [f"{pooled(rows,s,e,'position_rmse_m'):.4f}" for e in order] for s in scenarios])
    table("heading_speed_table.tex", "lrrrrrr",
          ["Condition", r"$\psi$: EKF", "Analyt.", "Hybrid", r"$v$: EKF", "Analyt.", "Hybrid"],
          [[LABELS[s]] + [f"{pooled(rows,s,e,k):.4f}" for k in ("heading_rmse_rad", "speed_rmse_mps")
                         for e in ("ekf", "analytic_ablation", "robust")] for s in scenarios])
    cl = reports["closed_loop"]["results"]
    table("closed_loop_table.tex", "lrrrr",
          ["Condition", r"Path: EKF", "Hybrid", r"Speed: EKF", "Hybrid"],
          [[LABELS[s]] + [f"{pooled(cl,s,e,k):.4f}" for k in ("tracking_rmse_m", "speed_tracking_rmse_mps")
                         for e in ("ekf", "robust")]
           for s in dict.fromkeys(r["scenario"] for r in cl)])
    stress = reports["stress_mismatch"]["results"]
    table("stress_table.tex", "lrrr", ["Condition", "EKF", "Analytical", "Hybrid"],
          [[LABELS[s]] + [f"{pooled(stress,s,e,'position_rmse_m'):.4f}"
                         for e in ("ekf", "analytic_ablation", "robust")]
           for s in dict.fromkeys(r["scenario"] for r in stress)])
    runtime = []
    for e, label in zip(order, ["EKF", "Legacy GRU", "Analytical", "Hybrid"]):
        selected = [r for r in rows if r["estimator"] == e]
        runtime.append([label] + [f"{np.median([r[k] for r in selected]):.3f}"
                                 for k in ("runtime_median_ms", "runtime_p95_ms")])
    table("runtime_table.tex", "lrr", ["Method", "Median trial median (ms)", "Median trial p95 (ms)"], runtime)
    train = json.loads((ROBUST / "models/innovation_trust_sim.history.json").read_text())
    with np.load(checkpoint, allow_pickle=False) as payload:
        assert train == json.loads(str(payload["metadata"])), "Training history differs from checkpoint metadata"
    best = [min([x for x in train["history"] if x["round"] == r], key=lambda x: x["val_loss"]) for r in (0, 1)]
    table("training_table.tex", "lrrr", ["Pass", "Selected epoch", "Training loss", "Validation loss"],
          [[str(b["round"]+1), str(b["epoch"]+1), f"{b['train_loss']:.5f}", f"{b['val_loss']:.5f}"] for b in best])

    fig, axes = plt.subplots(1, 2, figsize=(6.8, 2.25), sharey=True)
    for iteration, ax in enumerate(axes):
        h = [x for x in train["history"] if x["round"] == iteration]
        ax.plot([x["epoch"]+1 for x in h], [x["train_loss"] for x in h], label="Training", color=COLORS["ekf"])
        ax.plot([x["epoch"]+1 for x in h], [x["val_loss"] for x in h], label="Validation", color=COLORS["robust"])
        ax.set(title=f"Collection / training pass {iteration+1}", xlabel="Epoch", yscale="log")
        ax.grid(alpha=.2)
    axes[0].set_ylabel("Composite loss")
    axes[1].legend(frameon=False)
    fig.tight_layout()
    fig.savefig(OUT / "training_history.pdf")
    plt.close(fig)

    # The diagnostic is a fresh causal rollout, not invented trust traces.
    import torch
    torch.set_num_threads(1)
    from simulation_benchmark import generate_drive, attacked_samples, model_config
    from robustKLnet import RobustKalmanNetStateEstimator
    drive = generate_drive(101, duration=60.)
    samples, attack, _ = attacked_samples(drive, "gps_ramp", 101+10000)
    cfg = dict(model_config(), estimator_backend="innovation_trust", model_path=str(checkpoint),
               device="cpu", publish_clean_reference_output=False,
               enable_ekf_comparator=False, enable_clean_reference_comparator=False,
               comparator_record_to_file=False)
    adapter = RobustKalmanNetStateEstimator(drive["initial"][:3], cfg)
    prediction, times, learned, consistency, effective = [], [], [], [], []
    for i, sample in enumerate(samples):
        assert adapter.update(**sample)
        core = adapter.trust_filter
        prediction.append(adapter.get_state()[:4])
        if sample["gps_data"] is not None:
            rho = core.predict_trust(core.last_features)
            residual = core.last_measurement - core.last_prediction
            anchor_abs = np.expm1(core.last_features[:, 1]) * core.scales
            prior = np.exp(-np.minimum((np.maximum(abs(residual), .75*anchor_abs) /
                                       np.array([.30, .30, .20, .20]))**4, 60))
            # No freeze is injected in this diagnostic. Show x-channel factors;
            # final gamma_x also includes min coupling across x/y.
            times.append(i*drive["dt"])
            learned.append(rho[0]); consistency.append(prior[0]); effective.append(core.last_trust[0])
    with np.load(ROBUST / "results/heldout.npz") as saved:
        expected = saved["101_gps_ramp_robust"]
        assert np.allclose(prediction, expected, atol=1e-7, rtol=0), "Diagnostic does not match saved benchmark"
        t = np.arange(len(expected))*.02
        fig, axes = plt.subplots(2, 1, figsize=(6.8, 4.1), sharex=True)
        for name, label in [("ekf", "EKF"), ("analytic_ablation", "Analytical"), ("robust", "Hybrid")]:
            err = np.linalg.norm(saved[f"101_gps_ramp_{name}"][:, :2]-drive["truth"][:, :2], axis=1)
            axes[0].plot(t, err, label=label, color=COLORS[name], lw=1.15)
        axes[0].set_ylabel("Position error (m)")
        axes[0].legend(ncol=3, frameon=False)
        axes[1].plot(times, consistency, label=r"Analytical $b_x$", color=COLORS["analytic_ablation"], lw=1)
        axes[1].plot(times, learned, label=r"Learned $\rho_x$", color="#965F95", lw=1)
        axes[1].plot(times, effective, label=r"Applied $\gamma_x$", color=COLORS["robust"], lw=1)
        axes[1].set(xlabel="Simulation time (s)", ylabel="Reliability weight", ylim=(-.04, 1.07))
        axes[1].legend(ncol=3, frameon=False, loc="lower right")
        for ax in axes:
            for a, b in [(13.2, 27.), (33.6, 42.)]:
                ax.axvspan(a, b, color="#D97757", alpha=.12, zorder=-1)
            ax.grid(alpha=.2)
        fig.tight_layout()
        fig.savefig(OUT / "trust_diagnostic.pdf")
        plt.close(fig)

    with np.load(ROBUST / "results/closed_loop.npz") as data:
        fig, axes = plt.subplots(1, 2, figsize=(6.8, 2.65))
        angle = np.linspace(0, 2*np.pi, 500)
        axes[0].plot(2*np.cos(angle), 2*np.sin(angle), "k--", lw=.8, label="Reference")
        for name, label in [("ekf", "EKF car"), ("robust", "Hybrid car")]:
            truth = data[f"401_gps_jump_{name}_truth"]
            axes[0].plot(truth[:, 0], truth[:, 1], color=COLORS[name], lw=1.2, label=label)
            axes[1].plot(np.arange(len(truth))*.02, abs(np.linalg.norm(truth[:, :2], axis=1)-2),
                         color=COLORS[name], lw=1.2, label=label)
        axes[0].set(xlabel="x (m)", ylabel="y (m)", aspect="equal")
        axes[0].legend(frameon=False, fontsize=8, loc="upper right")
        axes[1].set(xlabel="Simulation time (s)", ylabel="Radial path error (m)")
        axes[1].axvspan(10, 24, color="#D97757", alpha=.12)
        for ax in axes: ax.grid(alpha=.2)
        fig.tight_layout()
        fig.savefig(OUT / "closed_loop_example.pdf")
        plt.close(fig)
    print("Paper assets generated; source hashes and diagnostic replay verified.")
    print("Checkpoint SHA-256:", digest)
    print("Runtime:", runtime)
    from build_observer_figures import main as build_observer_figures
    build_observer_figures()


if __name__ == "__main__":
    main()
