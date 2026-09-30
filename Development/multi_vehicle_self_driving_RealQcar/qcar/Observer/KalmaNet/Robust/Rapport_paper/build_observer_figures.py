"""Evidence figures: actual masks, gains, corrections, states and paired results.

Called by build_paper_assets.py. Instrumentation observes the existing estimators;
it does not replace their update equations. Every diagnostic state trajectory is
checked against heldout.npz. No attack label or truth enters an estimator update.
"""
from pathlib import Path
import hashlib
import inspect
import json
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm, TwoSlopeNorm
from matplotlib.lines import Line2D

PAPER = Path(__file__).resolve().parent
ROBUST = PAPER.parent
OUT = PAPER / "paper_assets"
sys.path.insert(0, str(ROBUST))
COLORS = {"ekf": "#C26420", "legacy": "#986EAC",
          "analytic_ablation": "#376DA3", "robust": "#147D64"}
NAMES = {"ekf": "EKF", "legacy": "Legacy GRU",
         "analytic_ablation": "Analytical", "robust": "Hybrid"}
LABELS = {"clean": "Clean", "gps_jump": "GPS jump", "gps_ramp": "GPS ramp",
          "gps_freeze": "GPS freeze", "gps_dropout": "GPS dropout",
          "gps_noise": "GPS noise", "heading_bias": "Heading bias",
          "wheel_bias": "Tachometer bias", "wheel_scale": "Tachometer scale",
          "wheel_freeze": "Tachometer freeze", "imu_bias": "IMU bias",
          "steering_bias": "Steering bias", "gps_wheel": "GPS + tachometer"}
METRICS = ("position_rmse_m", "heading_rmse_rad", "speed_rmse_mps")
METRIC_LABELS = ("Position (m)", "Heading (rad)", "Speed (m/s)")
SCENARIOS = ("gps_jump", "gps_ramp", "gps_freeze", "gps_dropout", "gps_noise",
             "heading_bias", "wheel_bias", "wheel_scale", "gps_wheel")


def save(fig, name):
    fig.savefig(OUT / f"{name}.pdf", bbox_inches="tight")
    fig.savefig(OUT / f"{name}.png", dpi=220, bbox_inches="tight")
    plt.close(fig)


def shade(ax):
    for left, right in ((13.2, 27.), (33.6, 42.)):
        ax.axvspan(left, right, color="#D97757", alpha=.12, zorder=-2)
    ax.grid(alpha=.18)
    ax.set_xlim(0, 60)


def wrap(x):
    return (x + np.pi) % (2*np.pi) - np.pi


def rmse(rows, scenario, method, metric):
    values = np.array([r[metric] for r in rows
                       if r["scenario"] == scenario and r["estimator"] == method])
    assert len(values)
    return float(np.sqrt(np.mean(values**2)))


def error(prediction, truth):
    e = prediction - truth
    e[:, 2] = wrap(e[:, 2])
    return e


def collect_diagnostics(checkpoint):
    import torch
    torch.set_num_threads(1)
    from simulation_benchmark import generate_drive, attacked_samples, model_config
    from innovation_trust import InnovationTrustFilter
    from robustKLnet import RobustKalmanNetStateEstimator
    from Observer.local_state_estimators import EKFStateEstimator

    class ObservedEKF(EKFStateEstimator):
        """Read the original update's local values at its gain-reporting hook."""
        def _set_last_gain(self, gain, measurement_labels=()):
            super()._set_last_gain(gain, measurement_labels)
            if gain is None:
                return
            frame = inspect.currentframe().f_back
            try:
                assert frame.f_code is EKFStateEstimator._update_fallback_ekf.__code__
                local = frame.f_locals
                self.observed_prediction = local["state_pred"].copy()
                self.observed_correction = np.asarray(gain) @ local["y_res"]
                self.observed_gain = np.zeros((4, 4))
                self.observed_residual = np.zeros(4)
                for column, label in enumerate(measurement_labels):
                    j = ("x", "y", "theta", "v").index(label)
                    self.observed_gain[:, j] = gain[:, column]
                    self.observed_residual[j] = local["y_res"][column]
            finally:
                del frame

    drive = generate_drive(101, duration=60.)
    data = {"time": np.arange(len(drive["truth"]))*drive["dt"],
            "truth": drive["truth"],
            "gps_ticks": np.array([s["gps_data"] is not None for s in drive["samples"]])}
    validations = []
    with np.load(ROBUST / "results/heldout.npz", allow_pickle=False) as saved:
        for scenario in SCENARIOS:
            samples, attack, _ = attacked_samples(drive, scenario, 101+10000)
            cfg = model_config()
            estimators = {
                "ekf": ObservedEKF(drive["initial"][:3], cfg),
                "analytic_ablation": InnovationTrustFilter(drive["initial"], cfg),
                "robust": RobustKalmanNetStateEstimator(drive["initial"][:3], dict(
                    cfg, estimator_backend="innovation_trust", model_path=str(checkpoint),
                    device="cpu", publish_clean_reference_output=False,
                    enable_ekf_comparator=False, enable_clean_reference_comparator=False,
                    comparator_record_to_file=False)),
            }
            available = np.zeros((len(samples), 4), dtype=bool)
            measurement = np.full((len(samples), 4), np.nan)
            for i, sample in enumerate(samples):
                available[i, 3] = np.isfinite(sample["motor_tach"])
                measurement[i, 3] = sample["motor_tach"]
                gps = sample["gps_data"]
                if gps is not None:
                    for j, key in enumerate(("x", "y", "theta")):
                        fresh = gps.get("position_valid", gps.get("fresh", gps.get("valid", False)))
                        valid = gps.get("heading_valid", fresh) if j == 2 else fresh
                        available[i, j] = valid and np.isfinite(gps.get(key, np.nan))
                        if available[i, j]:
                            measurement[i, j] = gps[key]
            data[f"{scenario}_availability"] = available
            data[f"{scenario}_measurement"] = measurement
            data[f"{scenario}_attack"] = attack
            for name, estimator in estimators.items():
                history = {key: [] for key in ("state", "prediction", "gain_matrix", "correction", "residual")}
                if name != "ekf":
                    history.update({key: [] for key in ("raw_gain", "trust", "rho", "prior")})
                for i, sample in enumerate(samples):
                    assert estimator.update(**sample), (scenario, name, i)
                    state = estimator.get_state()[:4].copy()
                    history["state"].append(state)
                    if name == "ekf":
                        pred = estimator.observed_prediction
                        residual = estimator.observed_residual
                        gain = estimator.observed_gain
                        correction = estimator.observed_correction
                    else:
                        core = estimator.trust_filter if name == "robust" else estimator
                        pred = core.last_prediction
                        residual = core.last_measurement-pred
                        residual[2] = wrap(residual[2])
                        rho = core.predict_trust(core.last_features)
                        anchor_abs = np.expm1(core.last_features[:, 1])*core.scales
                        prior = np.exp(-np.minimum((np.maximum(abs(residual), .75*anchor_abs)/
                                                  np.array([.30, .30, .20, .20]))**4, 60))
                        if (core.tach_frozen_seconds > .15 and
                                max(abs(sample["motor_tach"]), abs(core.drive_velocity)) > .05):
                            prior[3] = 0.
                        if (core.frozen_time[0] > .15 and core.frozen_time[1] > .15 and
                                core.last_features[0, 5] > .08):
                            prior[:3] = 0.
                        reconstructed_trust = available[i]*prior*rho
                        reconstructed_trust[:2] = min(reconstructed_trust[:2])
                        np.testing.assert_allclose(reconstructed_trust, core.last_trust, atol=1e-12, rtol=1e-12)
                        gain = np.diag(core.last_gain*core.last_trust)
                        correction = gain @ residual
                        for key, value in (("raw_gain", core.last_gain), ("trust", core.last_trust),
                                           ("rho", rho), ("prior", prior)):
                            history[key].append(value.copy())
                    reconstructed_state = pred+correction
                    reconstructed_state[2] = wrap(reconstructed_state[2])
                    if name == "robust":
                        np.testing.assert_allclose(reconstructed_state, core.get_state(), atol=1e-10, rtol=0)
                        # The runtime adapter publishes its core output through float32.
                        reconstructed_state = reconstructed_state.astype(np.float32).astype(float)
                    np.testing.assert_allclose(reconstructed_state, state, atol=1e-10, rtol=0)
                    for key, value in (("prediction", pred), ("gain_matrix", gain),
                                       ("correction", correction), ("residual", residual)):
                        history[key].append(value.copy())
                for key, values in history.items():
                    data[f"{scenario}_{name}_{key}"] = np.asarray(values)
                reference = saved[f"101_{scenario}_{name}"]
                max_error = float(np.max(abs(np.asarray(history["state"])-reference)))
                np.testing.assert_allclose(history["state"], reference, atol=1e-7, rtol=0)
                validations.append(dict(scenario=scenario, estimator=name,
                                        maximum_saved_state_difference=max_error))
            np.testing.assert_allclose(drive["truth"], saved[f"101_{scenario}_truth"], atol=1e-10, rtol=0)
            np.testing.assert_array_equal(attack, saved[f"101_{scenario}_attack"])
            print(f"  Verified mask/gain/state replay: {scenario}", flush=True)
    np.savez_compressed(OUT / "observer_diagnostics.npz", **data)
    provenance = {"seed": 101, "duration_s": 60, "dt_s": .02,
                  "checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
                  "checks": validations,
                  "note": "Gains are sensitivities of each method's own correction, not matched-prior experiments."}
    sources = [Path(__file__), PAPER/"build_paper_assets.py", ROBUST/"innovation_trust.py",
               ROBUST/"robustKLnet.py", ROBUST/"simulation_benchmark.py",
               ROBUST.parent.parent/"local_state_estimators.py"]
    provenance["source_sha256"] = {str(p.relative_to(ROBUST.parents[2])):
                                   hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    (OUT / "observer_diagnostics_manifest.json").write_text(json.dumps(provenance, indent=2))
    return data


def mask_outputs(data):
    scenarios = ("gps_jump", "gps_ramp", "gps_freeze", "gps_dropout",
                 "heading_bias", "wheel_bias", "wheel_scale", "gps_wheel")
    fig, axes = plt.subplots(4, 2, figsize=(7.2, 6.5), sharex=True, layout="constrained")
    ticks = data["gps_ticks"]
    for index, (ax, scenario) in enumerate(zip(axes.flat, scenarios)):
        trust = data[f"{scenario}_robust_trust"][ticks]
        values = np.vstack([data[f"{scenario}_availability"][ticks, 0], trust.T])
        im = ax.imshow(values, aspect="auto", interpolation="nearest", cmap="viridis", vmin=0, vmax=1,
                       extent=(0, 60, 4.5, -.5))
        ax.set_yticks(range(5), [r"$a_{GPS}$", r"$\gamma_x$", r"$\gamma_y$",
                               r"$\gamma_\psi$", r"$\gamma_v$"])
        ax.set_title(f"({chr(97+index)}) {LABELS[scenario]}", fontsize=9, loc="left")
        for start, end in ((13.2, 27), (33.6, 42)):
            ax.plot([start, end], [-.82, -.82], color="#B44430", lw=3, clip_on=False)
        ax.set_xticks([0, 15, 30, 45, 60])
    for ax in axes[-1]:
        ax.set_xlabel("Time (s)")
    fig.colorbar(im, ax=axes, shrink=.7, pad=.02, label="Availability / applied trust (0 to 1)")
    save(fig, "mask_channel_outputs")


def factors(data):
    fig, axes = plt.subplots(3, 1, figsize=(7.1, 5.3), sharex=True, layout="constrained")
    for index, (ax, scenario, channel) in enumerate(zip(axes, ("gps_jump", "gps_ramp", "wheel_scale"), (0, 0, 3))):
        valid = data[f"{scenario}_availability"][:, channel]
        t = data["time"][valid]
        prefix = f"{scenario}_robust_"
        for key, label, color, ls in (("prior", r"Analytical $b_j$", COLORS["analytic_ablation"], "-"),
                                     ("rho", r"Learned $\rho_j$", COLORS["legacy"], "--"),
                                     ("trust", r"Applied $\gamma_j$", COLORS["robust"], "-")):
            ax.plot(t, data[prefix+key][valid, channel], label=label, color=color, lw=1, ls=ls,
                    zorder=4 if key == "rho" else 3)
        ax.set_title(f"({chr(97+index)}) {LABELS[scenario]}: " + ("x channel" if channel == 0 else "v channel"), loc="left", fontsize=9)
        ax.set(ylim=(-.03, 1.07), ylabel="Weight")
        shade(ax)
    axes[0].legend(ncol=3, frameon=False, loc="lower right", fontsize=8)
    axes[-1].set_xlabel("Time (s)")
    save(fig, "trust_factor_comparison")


def gain_correction(data):
    fig, axes = plt.subplots(3, 3, figsize=(7.3, 6.5), sharex=True, layout="constrained")
    for col, (scenario, channel, unit) in enumerate((("gps_jump", 0, "m"), ("gps_ramp", 0, "m"),
                                                    ("wheel_bias", 3, "m/s"))):
        valid = data[f"{scenario}_availability"][:, channel]
        t = data["time"][valid]
        axes[0, col].set_title(LABELS[scenario] + (": x" if channel == 0 else ": v"), fontsize=9)
        for method in ("ekf", "analytic_ablation", "robust"):
            prefix = f"{scenario}_{method}_"
            gain = data[prefix+"gain_matrix"]
            axes[0, col].plot(t, gain[valid, channel, channel], color=COLORS[method], lw=1)
            axes[1, col].plot(t, abs(data[prefix+"correction"][valid, channel]), color=COLORS[method], lw=1)
            err = abs(error(data[prefix+"state"], data["truth"])[:, channel])
            axes[2, col].plot(data["time"], err, color=COLORS[method], lw=1)
        axes[0, col].plot(t, data[f"{scenario}_robust_raw_gain"][valid, channel],
                          color=".35", lw=1, ls="--")
        axes[0, col].set_ylim(-.02, 1.04)
        axes[1, col].set(yscale="symlog", ylabel=f"Correction ({unit})")
        axes[1, col].set_yscale("symlog", linthresh=1e-4)
        axes[1, col].set_ylim(bottom=0)
        axes[2, col].set_yscale("symlog", linthresh=1e-3)
        axes[2, col].set_ylim(bottom=0)
        axes[2, col].set(ylabel=f"State error ({unit})", xlabel="Time (s)")
        for row in range(3):
            shade(axes[row, col])
            axes[row, col].set_xticks([0, 20, 40, 60])
    for ax in axes[0]:
        ax.set_ylabel("Direct gain")
    handles = [Line2D([], [], color=COLORS[m], label=NAMES[m]) for m in ("ekf", "analytic_ablation", "robust")]
    handles.append(Line2D([], [], color=".35", ls="--", label="Hybrid raw K"))
    fig.legend(handles=handles, loc="outside upper center", ncol=4, frameon=False, fontsize=8)
    save(fig, "gain_correction_comparison")


def gain_snapshot(data):
    scenario = "gps_jump"
    tick = int(np.flatnonzero(data[f"{scenario}_attack"] & data[f"{scenario}_availability"][:, 0])[0])
    scales = np.array([.12, .12, .09, .10])
    matrices = [data[f"{scenario}_ekf_gain_matrix"][tick],
                data[f"{scenario}_analytic_ablation_gain_matrix"][tick],
                np.diag(data[f"{scenario}_robust_raw_gain"][tick]),
                data[f"{scenario}_robust_gain_matrix"][tick]]
    titles = ["EKF correction matrix", "Analytical applied matrix",
              "Hybrid raw diagonal K", r"Hybrid applied diag$(K\gamma)$"]
    fig, axes = plt.subplots(2, 2, figsize=(6.7, 5), layout="constrained")
    for ax, matrix, title in zip(axes.flat, matrices, titles):
        normalized = matrix*scales[None, :]/scales[:, None]
        im = ax.imshow(normalized, cmap="RdBu_r", norm=TwoSlopeNorm(vmin=-1, vcenter=0, vmax=1))
        for j in range(4):
            for k in range(4):
                value = normalized[j, k]
                label = "0" if value == 0 else (f"{value:.2f}" if abs(value) >= .01 else f"{value:.0e}")
                ax.text(k, j, label, ha="center", va="center", fontsize=9,
                        color="white" if abs(value) > .65 else "black")
        ax.set_xticks(range(4), ["x", "y", r"$\psi$", "v"])
        ax.set_yticks(range(4), ["x", "y", r"$\psi$", "v"])
        ax.set(xlabel="Measurement residual", ylabel="State correction", title=title)
        ax.title.set_fontsize(9)
    fig.colorbar(im, ax=axes, shrink=.75, label="Normalized correction sensitivity")
    save(fig, "gain_matrix_snapshot")


def state_outputs(data, saved):
    scenario = "gps_wheel"
    t = data["time"]
    truth = data["truth"].copy()
    truth[:, 2] = np.unwrap(truth[:, 2])
    fig, axes = plt.subplots(4, 2, figsize=(7.3, 7.2), sharex=True, layout="constrained")
    labels = ("x (m)", "y (m)", "Heading (rad)", "Speed (m/s)")
    for j in range(4):
        valid = data[f"{scenario}_availability"][:, j]
        z = data[f"{scenario}_measurement"][:, j].copy()
        if j == 2:
            z = truth[:, 2]+wrap(z-data["truth"][:, 2])
        axes[j, 0].plot(t[valid], z[valid], ".", ms=1.5, alpha=.3, color=".35")
        axes[j, 0].plot(t, truth[:, j], color="black", lw=1.4, ls="--")
        for method in ("ekf", "analytic_ablation", "robust", "legacy"):
            state = saved[f"101_{scenario}_{method}"]
            err = error(state, data["truth"])
            if method != "legacy":
                output = truth[:, j]+err[:, j] if j == 2 else state[:, j]
                axes[j, 0].plot(t, output, lw=.95, color=COLORS[method])
            axes[j, 1].plot(t, abs(err[:, j]), lw=.85, color=COLORS[method])
        axes[j, 0].set_ylabel(labels[j])
        axes[j, 1].set_ylabel("Absolute error")
        axes[j, 1].set_yscale("symlog", linthresh=1e-3)
        axes[j, 1].set_ylim(bottom=0)
        for ax in axes[j]:
            shade(ax)
    axes[0, 0].set_title("State outputs and received measurements", fontsize=9)
    axes[0, 1].set_title("Errors, including retained legacy checkpoint", fontsize=9)
    for ax in axes[-1]:
        ax.set_xlabel("Time (s)")
    handles = [Line2D([], [], color="black", ls="--", label="Truth"),
               Line2D([], [], color=".5", ls="", marker=".", label="Measurement")]
    handles += [Line2D([], [], color=COLORS[m], label=NAMES[m]) for m in COLORS]
    fig.legend(handles=handles, loc="outside upper center", ncol=3, frameon=False, fontsize=8)
    save(fig, "observer_state_comparison")


def summary_dots(reports):
    rows = reports["heldout"]["results"]
    scenarios = list(dict.fromkeys(r["scenario"] for r in rows))
    methods = ("ekf", "legacy", "analytic_ablation", "robust")
    fig, axes = plt.subplots(1, 3, figsize=(7.4, 5.3), sharey=True, layout="constrained")
    y = np.arange(len(scenarios))
    for ax, metric, title in zip(axes, METRICS, METRIC_LABELS):
        for method, offset in zip(methods, (-.24, -.08, .08, .24)):
            values = np.array([[r[metric] for r in rows if r["scenario"] == s and r["estimator"] == method]
                               for s in scenarios])
            pooled = np.sqrt(np.mean(values**2, axis=1))
            ax.hlines(y+offset, values.min(axis=1), values.max(axis=1), color=COLORS[method], lw=1)
            ax.scatter(pooled, y+offset, s=15, color=COLORS[method], label=NAMES[method], zorder=3)
        ax.set(xscale="log", xlabel="Complete-drive RMSE", title=title)
        ax.title.set_fontsize(9)
        ax.set_yticks(y, [LABELS[s] for s in scenarios], fontsize=8)
        ax.grid(axis="x", alpha=.2)
    axes[0].invert_yaxis()
    fig.legend(*axes[0].get_legend_handles_labels(), ncol=4, loc="outside upper center", frameon=False, fontsize=8)
    save(fig, "performance_overview")


def ratio_heatmap(ax, values, labels, title):
    im = ax.imshow(values, cmap="BrBG_r", norm=LogNorm(vmin=.01, vmax=100), aspect="auto")
    for row in range(values.shape[0]):
        for col in range(values.shape[1]):
            v = values[row, col]
            text = f"{v:.2f}" if v >= .01 else f"{v:.3f}"
            ax.text(col, row, text, ha="center", va="center", fontsize=8,
                    color="white" if v < .045 or v > 22 else "black")
    ax.set_yticks(range(len(labels)), labels, fontsize=8)
    ax.set_xticks(range(3), ["Position", "Heading", "Speed"], fontsize=8)
    ax.set_title(title, fontsize=9)
    return im


def relative_results(reports):
    rows = reports["heldout"]["results"]
    scenarios = list(dict.fromkeys(r["scenario"] for r in rows))
    fig, axes = plt.subplots(1, 2, figsize=(7.1, 5.3), layout="constrained")
    evidence = {}
    for ax, reference, title in zip(axes, ("ekf", "analytic_ablation"),
                                     ("Hybrid / EKF", "Hybrid / analytical ablation")):
        values = np.array([[rmse(rows, s, "robust", m)/rmse(rows, s, reference, m)
                            for m in METRICS] for s in scenarios])
        im = ratio_heatmap(ax, values, [LABELS[s] for s in scenarios], title)
        evidence[reference] = {s: dict(zip(METRICS, values[i].tolist())) for i, s in enumerate(scenarios)}
    fig.colorbar(im, ax=axes, shrink=.8, pad=.02, label="RMSE ratio: <1 improves, >1 worsens", ticks=[.01, .1, 1, 10, 100])
    save(fig, "relative_performance")
    return evidence


def phase_results(reports, saved):
    scenarios = list(dict.fromkeys(r["scenario"] for r in reports["heldout"]["results"] if r["scenario"] != "clean"))
    rows = []
    for seed in reports["heldout"]["seeds"]:
        for scenario in scenarios:
            truth = saved[f"{seed}_{scenario}_truth"]
            attack = saved[f"{seed}_{scenario}_attack"]
            recovery = np.zeros(len(attack), bool)
            ends = np.flatnonzero(np.diff(attack.astype(int), prepend=0) == -1)
            for end in ends:
                recovery[end:min(end+round(3/reports["heldout"]["dt"]), len(attack))] = True
            assert not np.any(attack & recovery)
            for method in ("ekf", "analytic_ablation", "robust"):
                e = error(saved[f"{seed}_{scenario}_{method}"], truth)
                for phase, selection in (("attack", attack), ("recovery_3s", recovery)):
                    e_sel = e[selection]
                    metrics = [np.sqrt(np.mean(np.sum(e_sel[:, :2]**2, axis=1))),
                               np.sqrt(np.mean(e_sel[:, 2]**2)), np.sqrt(np.mean(e_sel[:, 3]**2))]
                    record = dict(seed=seed, scenario=scenario, estimator=method, phase=phase,
                                  sample_count=int(selection.sum()), **dict(zip(METRICS, map(float, metrics))))
                    rows.append(record)
                    if phase == "attack":
                        original = next(r for r in reports["heldout"]["results"]
                                        if r["seed"] == seed and r["scenario"] == scenario and r["estimator"] == method)
                        for metric in METRICS:
                            np.testing.assert_allclose(record[metric], original["attack_"+metric], atol=1e-12)
    fig, axes = plt.subplots(1, 2, figsize=(7.1, 5.1), layout="constrained")
    for ax, phase, title in zip(axes, ("attack", "recovery_3s"),
                                ("During injection: hybrid / EKF", "First 3 s after removal: hybrid / EKF")):
        selection = [r for r in rows if r["phase"] == phase]
        values = np.array([[rmse(selection, s, "robust", m)/rmse(selection, s, "ekf", m)
                            for m in METRICS] for s in scenarios])
        im = ratio_heatmap(ax, values, [LABELS[s] for s in scenarios], title)
    fig.colorbar(im, ax=axes, shrink=.8, pad=.02, label="RMSE ratio: <1 improves, >1 worsens", ticks=[.01, .1, 1, 10, 100])
    save(fig, "attack_recovery_performance")
    (OUT / "phase_metrics.json").write_text(json.dumps({"recovery_window_s": 3, "results": rows}, indent=2))
    return rows


def control_stress(reports):
    fig = plt.figure(figsize=(7.2, 6.4), layout="constrained")
    gs = fig.add_gridspec(2, 2, height_ratios=(1.35, 1))
    axes = [fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, :])]
    rows = reports["closed_loop"]["results"]
    scenarios = list(dict.fromkeys(r["scenario"] for r in rows))
    for ax, metric, title in zip(axes[:2], ("tracking_rmse_m", "speed_tracking_rmse_mps"),
                                ("(a) Closed-loop path RMSE (m)", "(b) Closed-loop speed RMSE (m/s)")):
        for i, scenario in enumerate(scenarios):
            for method, offset in (("ekf", -.12), ("robust", .12)):
                values = [r[metric] for r in rows if r["scenario"] == scenario and r["estimator"] == method]
                ax.hlines(i+offset, min(values), max(values), color=COLORS[method], lw=1.2)
                ax.scatter(rmse(rows, scenario, method, metric), i+offset, color=COLORS[method], s=18)
        ax.set_yticks(range(len(scenarios)), [LABELS[s] for s in scenarios], fontsize=8)
        ax.invert_yaxis()
        ax.set(xscale="log", title=title)
        ax.title.set_fontsize(9)
        ax.grid(axis="x", alpha=.2)
    rows = reports["stress_mismatch"]["results"]
    scenarios = list(dict.fromkeys(r["scenario"] for r in rows))
    ax = axes[2]
    for method, offset in (("ekf", -.2), ("analytic_ablation", 0), ("robust", .2)):
        values = np.array([[r["position_rmse_m"] for r in rows if r["scenario"] == s and r["estimator"] == method]
                           for s in scenarios])
        center = np.sqrt(np.mean(values**2, axis=1))
        xpos = np.arange(len(scenarios))+offset
        ax.errorbar(xpos, center, yerr=np.vstack([center-values.min(axis=1), values.max(axis=1)-center]),
                    fmt="o", ms=4, capsize=3, color=COLORS[method], label=NAMES[method])
    ax.set_xticks(range(len(scenarios)), [LABELS[s] for s in scenarios], fontsize=8)
    ax.set(yscale="log", ylabel="Position RMSE (m)", title="(c) Calibration mismatch + 5 Hz GPS")
    ax.title.set_fontsize(9)
    fig.legend(*ax.get_legend_handles_labels(), ncol=3, frameon=False, fontsize=8,
               loc="outside upper center")
    ax.grid(axis="y", alpha=.2)
    save(fig, "control_stress_comparison")


def main():
    OUT.mkdir(exist_ok=True)
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 8,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "pdf.fonttype": 42})
    reports = {name: json.loads((ROBUST/f"results/{name}.json").read_text())
               for name in ("heldout", "closed_loop", "stress_mismatch")}
    checkpoint = ROBUST/"models/innovation_trust_sim.npz"
    fingerprint = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    assert all(report["checkpoint_sha256"] == fingerprint for report in reports.values())
    data = collect_diagnostics(checkpoint)
    mask_outputs(data)
    factors(data)
    gain_correction(data)
    gain_snapshot(data)
    with np.load(ROBUST/"results/heldout.npz", allow_pickle=False) as saved:
        state_outputs(data, saved)
        phase_results(reports, saved)
    summary_dots(reports)
    ratios = relative_results(reports)
    control_stress(reports)
    (OUT/"relative_metrics.json").write_text(json.dumps(ratios, indent=2))
    print("Nine additional figure sets generated as vector PDF and reusable PNG.", flush=True)


if __name__ == "__main__":
    main()
