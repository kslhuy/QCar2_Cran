#!/usr/bin/env python3
"""
Plot passive online throttle-acceleration calibration against saved raw data.

The online passive analyser fits a first-order throttle-to-velocity response:

    v_dot = -(1/tau) * v + (K/tau) * throttle

The trust fleet estimator's lightweight acceleration model uses:

    a_dot = -(1/tau) * a + (input_gain/tau) * throttle
    v_dot = a

This script plots both so the difference is visible.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

try:
    import yaml
except ImportError as exc:
    raise RuntimeError("PyYAML is required to read calibration YAML files") from exc


CAL_DIR = Path(__file__).resolve().parent
DEFAULT_RESULTS_DIR = CAL_DIR / "results" / "online_throttle_acceleration"


def _load_yaml(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data if isinstance(data, dict) else {}


def _latest_raw_csv(results_dir: Path) -> Path:
    candidates = sorted(
        results_dir.glob("raw_samples_*.csv"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    if not candidates:
        raise FileNotFoundError(f"No raw_samples_*.csv found in {results_dir}")
    return candidates[0]


def _load_raw_csv(path: Path) -> Dict[str, np.ndarray]:
    rows: List[Dict[str, float]] = []
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for raw in reader:
            rows.append({k: float(v) for k, v in raw.items() if k})

    if not rows:
        raise ValueError(f"No rows found in {path}")

    keys = rows[0].keys()
    return {
        key: np.asarray([row.get(key, float("nan")) for row in rows], dtype=float)
        for key in keys
    }


def _small_step_gain_stats(model: Dict) -> Tuple[float, float]:
    gains = []
    taus = []
    for step in model.get("step_models", []):
        try:
            du = abs(float(step["delta_u"]))
            tau = float(step["tau_s"])
            k_local = float(step["K_local"])
        except (KeyError, TypeError, ValueError):
            continue
        if 0.015 <= du <= 0.09 and tau > 0.0:
            gains.append(k_local / tau)
            taus.append(tau)

    if not gains:
        avg_tau = float(model.get("avg_tau", 0.3))
        avg_k = float(model.get("avg_K", 1.0))
        return avg_tau, avg_k / max(avg_tau, 1e-9)

    return float(np.median(taus)), float(np.median(gains))


def _simulate_velocity_lag(
    throttle: np.ndarray,
    v0: float,
    dt: float,
    tau: float,
    k_velocity: float,
) -> np.ndarray:
    pred = np.zeros_like(throttle, dtype=float)
    pred[0] = v0
    tau = max(float(tau), 1e-9)
    for i in range(1, len(throttle)):
        u = float(throttle[i - 1])
        pred[i] = pred[i - 1] + dt * (
            -(1.0 / tau) * pred[i - 1] + (k_velocity / tau) * u
        )
    return pred


def _simulate_accel_lag(
    throttle: np.ndarray,
    v0: float,
    a0: float,
    dt: float,
    tau: float,
    input_gain: float,
) -> Tuple[np.ndarray, np.ndarray]:
    v_pred = np.zeros_like(throttle, dtype=float)
    a_pred = np.zeros_like(throttle, dtype=float)
    v_pred[0] = v0
    a_pred[0] = a0
    tau = max(float(tau), 1e-9)
    for i in range(1, len(throttle)):
        u = float(throttle[i - 1])
        a_pred[i] = a_pred[i - 1] + dt * (
            -(1.0 / tau) * a_pred[i - 1] + (input_gain / tau) * u
        )
        v_pred[i] = v_pred[i - 1] + a_pred[i] * dt
    return v_pred, a_pred


def _rmse(a: np.ndarray, b: np.ndarray) -> float:
    mask = np.isfinite(a) & np.isfinite(b)
    if not np.any(mask):
        return float("nan")
    return float(np.sqrt(np.mean((a[mask] - b[mask]) ** 2)))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot online throttle acceleration calibration fit"
    )
    parser.add_argument(
        "--results-dir",
        default=str(DEFAULT_RESULTS_DIR),
        help="Directory containing throttle_acceleration_latest.yaml and raw_samples_*.csv",
    )
    parser.add_argument(
        "--yaml",
        default=None,
        help="Calibration YAML path. Defaults to <results-dir>/throttle_acceleration_latest.yaml",
    )
    parser.add_argument(
        "--raw",
        default=None,
        help="Raw CSV path. Defaults to newest raw_samples_*.csv in results-dir",
    )
    parser.add_argument(
        "--sample-dt",
        type=float,
        default=0.02,
        help="Sample period used for raw_samples CSV, seconds",
    )
    parser.add_argument(
        "--input-gain",
        type=float,
        default=None,
        help="Override estimator acceleration input_gain. Default: median K_local/tau_s for small steps",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output PNG path. Defaults to <results-dir>/online_throttle_acceleration_fit.png",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open the Matplotlib figure window after saving the PNG",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    results_dir = Path(args.results_dir).resolve()
    yaml_path = (
        Path(args.yaml).resolve()
        if args.yaml
        else results_dir / "throttle_acceleration_latest.yaml"
    )
    raw_path = Path(args.raw).resolve() if args.raw else _latest_raw_csv(results_dir)
    output_path = (
        Path(args.output).resolve()
        if args.output
        else results_dir / "online_throttle_acceleration_fit.png"
    )

    model = _load_yaml(yaml_path)
    raw = _load_raw_csv(raw_path)

    v = raw["v"]
    throttle = raw["throttle"]
    ax = raw.get("ax", np.full_like(v, np.nan))
    dt = float(args.sample_dt)
    t = np.arange(len(v), dtype=float) * dt

    tau = float(model.get("avg_tau", 0.3))
    k_velocity = float(model.get("avg_K", 1.0))
    robust_tau, robust_input_gain = _small_step_gain_stats(model)
    input_gain = (
        float(args.input_gain)
        if args.input_gain is not None
        else robust_input_gain
    )

    v_velocity_lag = _simulate_velocity_lag(
        throttle=throttle,
        v0=float(v[0]),
        dt=dt,
        tau=tau,
        k_velocity=k_velocity,
    )
    a0 = float(ax[0]) if np.isfinite(ax[0]) else 0.0
    v_accel_lag, a_accel_lag = _simulate_accel_lag(
        throttle=throttle,
        v0=float(v[0]),
        a0=a0,
        dt=dt,
        tau=robust_tau,
        input_gain=input_gain,
    )

    acc_from_v = np.gradient(v, dt)
    rmse_velocity_lag = _rmse(v, v_velocity_lag)
    rmse_accel_lag = _rmse(v, v_accel_lag)

    import matplotlib

    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
    fig.suptitle(
        "Online Throttle Acceleration Calibration: Data vs Model",
        fontsize=13,
        fontweight="bold",
    )

    axes[0].plot(t, v, color="black", linewidth=1.2, label="Measured velocity")
    axes[0].plot(
        t,
        v_velocity_lag,
        color="tab:blue",
        linewidth=1.4,
        label=(
            f"Passive velocity fit: tau={tau:.3f}s, K={k_velocity:.3f}, "
            f"RMSE={rmse_velocity_lag:.3f} m/s"
        ),
    )
    v_span = max(float(np.nanmax(v) - np.nanmin(v)), 1e-6)
    accel_lag_span = float(np.nanmax(v_accel_lag) - np.nanmin(v_accel_lag))
    accel_lag_label = (
        f"Estimator accel lag: tau={robust_tau:.3f}s, "
        f"input_gain={input_gain:.2f}, RMSE={rmse_accel_lag:.3f} m/s"
    )
    if accel_lag_span > 3.0 * v_span:
        ax0_right = axes[0].twinx()
        ax0_right.plot(
            t,
            v_accel_lag,
            color="tab:red",
            linewidth=0.9,
            alpha=0.75,
            label=accel_lag_label + " (right axis)",
        )
        ax0_right.set_ylabel("Accel-lag velocity [m/s]", color="tab:red")
        ax0_right.tick_params(axis="y", labelcolor="tab:red")
        lines, labels = axes[0].get_legend_handles_labels()
        lines2, labels2 = ax0_right.get_legend_handles_labels()
        axes[0].legend(lines + lines2, labels + labels2, loc="best", fontsize=8)
    else:
        axes[0].plot(
            t,
            v_accel_lag,
            color="tab:red",
            linewidth=1.0,
            alpha=0.85,
            label=accel_lag_label,
        )
        axes[0].legend(loc="best", fontsize=8)
    axes[0].set_ylabel("Velocity [m/s]")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, throttle, color="tab:purple", linewidth=1.0)
    axes[1].set_ylabel("Throttle")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(
        t,
        acc_from_v,
        color="tab:green",
        linewidth=0.8,
        alpha=0.7,
        label="dv/dt from measured v",
    )
    axes[2].plot(
        t,
        ax,
        color="0.35",
        linewidth=0.8,
        alpha=0.65,
        label="Raw ax column",
    )
    axes[2].plot(
        t,
        a_accel_lag,
        color="tab:red",
        linewidth=1.2,
        label="Estimator accel lag a",
    )
    axes[2].set_ylabel("Acceleration [m/s^2]")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="best", fontsize=8)

    step_taus = []
    step_gains = []
    step_du = []
    for step in model.get("step_models", []):
        try:
            tau_i = float(step["tau_s"])
            k_i = float(step["K_local"])
            du_i = abs(float(step["delta_u"]))
        except (KeyError, TypeError, ValueError):
            continue
        if tau_i > 0.0 and math.isfinite(tau_i) and math.isfinite(k_i):
            step_taus.append(tau_i)
            step_gains.append(k_i / tau_i)
            step_du.append(du_i)

    if step_taus:
        idx = np.arange(len(step_taus))
        axes[3].scatter(
            idx,
            step_taus,
            c=step_du,
            cmap="viridis",
            s=30,
            label="tau per detected transition",
        )
        axes_gain = axes[3].twinx()
        axes_gain.plot(
            idx,
            step_gains,
            color="tab:orange",
            linewidth=1.0,
            marker=".",
            label="K_local / tau_s",
        )
        axes[3].axhline(robust_tau, color="tab:blue", linestyle="--", linewidth=1.0)
        axes_gain.axhline(
            input_gain, color="tab:red", linestyle="--", linewidth=1.0
        )
        axes[3].set_ylabel("Tau [s]")
        axes_gain.set_ylabel("Derived input_gain [m/s^2/throttle]")
        axes[3].set_xlabel("Detected transition index")
        axes[3].grid(True, alpha=0.3)
        lines, labels = axes[3].get_legend_handles_labels()
        lines2, labels2 = axes_gain.get_legend_handles_labels()
        axes[3].legend(lines + lines2, labels + labels2, loc="best", fontsize=8)
    else:
        axes[3].text(0.02, 0.5, "No step model entries found", transform=axes[3].transAxes)
        axes[3].set_xlabel("Time [s]")

    axes[2].set_ylim(
        max(np.nanpercentile(acc_from_v, 1), -8.0),
        min(np.nanpercentile(acc_from_v, 99), 8.0),
    )
    axes[0].set_xlim(t[0], t[-1])
    axes[-1].set_xlabel("Time [s]")

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=160)

    print(f"[plot] YAML: {yaml_path}")
    print(f"[plot] Raw CSV: {raw_path}")
    print(f"[plot] Saved: {output_path}")
    print(f"[plot] velocity-lag RMSE: {rmse_velocity_lag:.4f} m/s")
    print(f"[plot] accel-lag RMSE: {rmse_accel_lag:.4f} m/s")
    if accel_lag_span > 3.0 * v_span:
        print(
            "[plot] note: accel-lag velocity is on a separate right axis because "
            "this no-drag acceleration model does not settle to a steady speed."
        )
    print(
        "[plot] recommended accel_lag_model: "
        f"tau={robust_tau:.4f}, input_gain={input_gain:.4f}"
    )
    if args.show:
        print("[plot] Close the Matplotlib window to finish.")
        plt.show()
    plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
