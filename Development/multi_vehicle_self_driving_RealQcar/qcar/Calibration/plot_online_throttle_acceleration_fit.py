#!/usr/bin/env python3
"""Plot passive online throttle-acceleration fit and observer diagnostics."""

# CMD examples:
# python .\plot_online_throttle_acceleration_fit.py --metrics-only --export-observer-model
# python .\plot_online_throttle_acceleration_fit.py --show

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

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


def _estimate_deadband_from_steps(model: Dict, k_velocity: float) -> float:
    candidates = []
    for step in model.get("step_models", []):
        try:
            u_to = float(step["u_to"])
            vss = float(step["vss"])
        except (KeyError, TypeError, ValueError):
            continue
        if abs(u_to) < 0.04 or not np.isfinite(vss):
            continue
        candidate = abs(u_to) - abs(vss) / max(abs(k_velocity), 1e-9)
        if 0.0 <= candidate <= 0.03:
            candidates.append(candidate)
    return float(np.nanmedian(candidates)) if candidates else 0.0


def _observer_velocity_lag_params(model: Dict) -> Tuple[float, float, float, float]:
    observer_model = model.get("observer_model", {})
    if isinstance(observer_model, dict):
        velocity_lag = observer_model.get("velocity_lag_model", {})
        accel_lag = observer_model.get("accel_lag_model", {})
        if isinstance(velocity_lag, dict):
            try:
                tau = float(velocity_lag["tau"])
                k_velocity = float(velocity_lag["velocity_gain"])
                input_gain = float(accel_lag.get("input_gain", k_velocity / tau))
                deadband = float(velocity_lag.get("throttle_deadband", 0.0))
                return tau, k_velocity, input_gain, max(deadband, 0.0)
            except (KeyError, TypeError, ValueError):
                pass

    tau = float(model.get("robust_tau", model.get("avg_tau", 0.3)))
    k_velocity = float(model.get("robust_K", model.get("avg_K", 1.0)))
    _tau_from_steps, input_gain = _small_step_gain_stats(model)
    input_gain = float(model.get("robust_input_gain_mps2_per_throttle", input_gain))
    deadband = float(model.get("velocity_lag_throttle_deadband", 0.0))
    if deadband <= 0.0:
        deadband = _estimate_deadband_from_steps(model, k_velocity)
    return tau, k_velocity, input_gain, max(deadband, 0.0)


def _apply_throttle_deadband(throttle: np.ndarray, deadband: float) -> np.ndarray:
    throttle = np.asarray(throttle, dtype=float)
    deadband = max(float(deadband), 0.0)
    if deadband <= 0.0:
        return throttle.copy()
    return np.sign(throttle) * np.maximum(np.abs(throttle) - deadband, 0.0)


def _simulate_velocity_lag(
    throttle: np.ndarray,
    v0: float,
    dt: float,
    tau: float,
    k_velocity: float,
    throttle_deadband: float = 0.0,
) -> np.ndarray:
    pred = np.zeros_like(throttle, dtype=float)
    pred[0] = v0
    tau = max(float(tau), 1e-9)
    throttle_eff = _apply_throttle_deadband(throttle, throttle_deadband)
    for i in range(1, len(throttle)):
        u = float(throttle_eff[i - 1])
        pred[i] = pred[i - 1] + dt * (
            -(1.0 / tau) * pred[i - 1] + (k_velocity / tau) * u
        )
    return pred


def _ema(values: np.ndarray, alpha: float) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return values
    alpha = float(np.clip(alpha, 0.0, 1.0))
    out = np.empty_like(values)
    out[0] = values[0]
    for i in range(1, values.size):
        out[i] = alpha * values[i] + (1.0 - alpha) * out[i - 1]
    return out


def _zero_phase_ema(values: np.ndarray, alpha: float) -> np.ndarray:
    """Forward-backward EMA for offline diagnostics only."""
    forward = _ema(values, alpha)
    backward = _ema(forward[::-1], alpha)[::-1]
    return backward


def _odd_window(requested: int, n: int) -> int:
    window = max(5, int(requested))
    if window % 2 == 0:
        window += 1
    if window >= n:
        window = n - 1 if (n - 1) % 2 == 1 else n - 2
    return max(window, 5)


def _smooth_velocity(
    velocity: np.ndarray,
    method: str,
    window_samples: int,
    poly_order: int,
    ema_alpha: float,
) -> Tuple[np.ndarray, str]:
    velocity = np.asarray(velocity, dtype=float)
    method = str(method).strip().lower()

    if method == "savgol":
        try:
            from scipy.signal import savgol_filter

            window = _odd_window(window_samples, len(velocity))
            poly_order = min(max(int(poly_order), 1), window - 2)
            return (
                savgol_filter(velocity, window, poly_order, mode="interp"),
                f"savgol(window={window}, poly={poly_order})",
            )
        except Exception:
            method = "zero_phase_ema"

    if method == "zero_phase_ema":
        return (
            _zero_phase_ema(velocity, ema_alpha),
            f"zero_phase_ema(alpha={ema_alpha:.2f})",
        )

    if method == "ema":
        return _ema(velocity, ema_alpha), f"ema(alpha={ema_alpha:.2f})"

    return velocity.copy(), "none"


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


def _write_diagnostics_csv(
    path: Path,
    t: np.ndarray,
    velocity: np.ndarray,
    velocity_filtered: np.ndarray,
    throttle: np.ndarray,
    ax: np.ndarray,
    acc_ref: np.ndarray,
    acc_raw_dvdt: np.ndarray,
    fusion: Dict[str, np.ndarray],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "time_s",
                "velocity_mps",
                "velocity_filtered_mps",
                "throttle",
                "imu_ax_mps2",
                "acc_raw_dvdt_mps2",
                "acc_ref_filtered_dvdt_mps2",
                "acc_model_mps2",
                "acc_tach_lpf_mps2",
                "acc_fused_mps2",
                "velocity_residual_mps",
                "model_weight",
                "acc_model_error_mps2",
                "acc_fused_error_mps2",
            ]
        )
        for i in range(len(t)):
            writer.writerow(
                [
                    float(t[i]),
                    float(velocity[i]),
                    float(velocity_filtered[i]),
                    float(throttle[i]),
                    float(ax[i]) if np.isfinite(ax[i]) else float("nan"),
                    float(acc_raw_dvdt[i]),
                    float(acc_ref[i]),
                    float(fusion["a_model"][i]),
                    float(fusion["a_tach"][i]),
                    float(fusion["a_fused"][i]),
                    float(fusion["velocity_residual"][i]),
                    float(fusion["model_weight"][i]),
                    float(fusion["a_model"][i] - acc_ref[i]),
                    float(fusion["a_fused"][i] - acc_ref[i]),
                ]
            )


def _svg_polyline(
    t: np.ndarray,
    y: np.ndarray,
    width: int,
    height: int,
    pad: int,
    y_min: float,
    y_max: float,
) -> str:
    mask = np.isfinite(t) & np.isfinite(y)
    if not np.any(mask):
        return ""
    tt = t[mask]
    yy = y[mask]
    stride = max(1, int(np.ceil(len(tt) / 1800)))
    tt = tt[::stride]
    yy = yy[::stride]
    x_min = float(tt[0])
    x_max = float(tt[-1])
    x_span = max(x_max - x_min, 1e-9)
    y_span = max(y_max - y_min, 1e-9)
    xs = pad + (tt - x_min) / x_span * (width - 2 * pad)
    ys = height - pad - (yy - y_min) / y_span * (height - 2 * pad)
    return " ".join(f"{float(x):.1f},{float(yv):.1f}" for x, yv in zip(xs, ys))


def _series_range(series: List[np.ndarray], clamp_percentile: bool = True) -> Tuple[float, float]:
    vals = np.concatenate([np.asarray(s, dtype=float).reshape(-1) for s in series])
    vals = vals[np.isfinite(vals)]
    if vals.size == 0:
        return -1.0, 1.0
    if clamp_percentile and vals.size > 20:
        y_min = float(np.nanpercentile(vals, 1))
        y_max = float(np.nanpercentile(vals, 99))
    else:
        y_min = float(np.nanmin(vals))
        y_max = float(np.nanmax(vals))
    if abs(y_max - y_min) < 1e-9:
        y_min -= 1.0
        y_max += 1.0
    pad = 0.08 * (y_max - y_min)
    return y_min - pad, y_max + pad


def _svg_panel(
    title: str,
    t: np.ndarray,
    series: List[Tuple[str, np.ndarray, str]],
    ylabel: str,
    y_range: Optional[Tuple[float, float]] = None,
) -> str:
    width = 1180
    height = 250
    pad = 48
    if y_range is None:
        y_min, y_max = _series_range([s[1] for s in series])
    else:
        y_min, y_max = y_range
    y_zero = None
    if y_min < 0.0 < y_max:
        y_zero = height - pad - (0.0 - y_min) / (y_max - y_min) * (height - 2 * pad)

    lines = [
        f'<section><h3>{title}</h3>',
        f'<svg width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        f'<rect x="0" y="0" width="{width}" height="{height}" fill="#ffffff"/>',
        f'<line x1="{pad}" y1="{height-pad}" x2="{width-pad}" y2="{height-pad}" stroke="#999"/>',
        f'<line x1="{pad}" y1="{pad}" x2="{pad}" y2="{height-pad}" stroke="#999"/>',
    ]
    if y_zero is not None:
        lines.append(
            f'<line x1="{pad}" y1="{y_zero:.1f}" x2="{width-pad}" y2="{y_zero:.1f}" stroke="#ddd"/>'
        )

    lines.append(
        f'<text x="10" y="22" font-size="12" fill="#555">{ylabel}</text>'
    )
    lines.append(
        f'<text x="{pad}" y="{height-12}" font-size="11" fill="#555">t={float(t[0]):.1f}s</text>'
    )
    lines.append(
        f'<text x="{width-pad-70}" y="{height-12}" font-size="11" fill="#555">t={float(t[-1]):.1f}s</text>'
    )
    lines.append(
        f'<text x="{pad+4}" y="{pad-10}" font-size="11" fill="#555">{y_max:.3f}</text>'
    )
    lines.append(
        f'<text x="{pad+4}" y="{height-pad+16}" font-size="11" fill="#555">{y_min:.3f}</text>'
    )

    legend_x = pad + 10
    for idx, (label, values, color) in enumerate(series):
        points = _svg_polyline(t, values, width, height, pad, y_min, y_max)
        if points:
            lines.append(
                f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.6"/>'
            )
        legend_y = 20 + idx * 16
        lines.append(
            f'<text x="{legend_x}" y="{legend_y}" font-size="12" fill="{color}">{label}</text>'
        )

    lines.extend(["</svg>", "</section>"])
    return "\n".join(lines)


def _write_html_diagnostics_plot(
    path: Path,
    t: np.ndarray,
    velocity: np.ndarray,
    velocity_filtered: np.ndarray,
    velocity_model: np.ndarray,
    throttle: np.ndarray,
    ax: np.ndarray,
    acc_ref: np.ndarray,
    acc_raw_dvdt: np.ndarray,
    fusion: Dict[str, np.ndarray],
    title: str,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    panels = [
        _svg_panel(
            "Velocity Fit",
            t,
            [
                ("measured v", velocity, "#111111"),
                ("filtered v", velocity_filtered, "#777777"),
                ("velocity-lag model", velocity_model, "#1f77b4"),
            ],
            "m/s",
        ),
        _svg_panel(
            "Throttle Command",
            t,
            [("throttle", throttle, "#7f3c8d")],
            "u",
            y_range=(
                float(np.nanmin(throttle)) - 0.02,
                float(np.nanmax(throttle)) + 0.02,
            ),
        ),
        _svg_panel(
            "Acceleration Comparison",
            t,
            [
                ("offline filtered dv/dt ref", acc_ref, "#2ca02c"),
                ("raw dv/dt", acc_raw_dvdt, "#bbbbbb"),
                ("raw IMU ax", ax, "#555555"),
                ("model a", fusion["a_model"], "#1f77b4"),
                ("fused a", fusion["a_fused"], "#d62728"),
            ],
            "m/s^2",
        ),
        _svg_panel(
            "Residual And Model Weight",
            t,
            [
                ("velocity residual", fusion["velocity_residual"], "#ff7f0e"),
                ("model weight", fusion["model_weight"], "#1f77b4"),
            ],
            "mixed",
            y_range=(
                min(float(np.nanmin(fusion["velocity_residual"])), 0.0),
                max(float(np.nanmax(fusion["model_weight"])), 1.0),
            ),
        ),
        _svg_panel(
            "Acceleration Error vs Offline Reference",
            t,
            [
                ("model error", fusion["a_model"] - acc_ref, "#1f77b4"),
                ("tach lpf error", fusion["a_tach"] - acc_ref, "#17becf"),
                ("fused error", fusion["a_fused"] - acc_ref, "#d62728"),
            ],
            "m/s^2",
        ),
    ]
    html = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>{title}</title>
  <style>
    body {{ font-family: Segoe UI, Arial, sans-serif; margin: 22px; color: #222; }}
    h1 {{ font-size: 20px; }}
    h3 {{ margin: 22px 0 6px; font-size: 15px; }}
    section {{ max-width: 1220px; }}
    svg {{ border: 1px solid #ddd; background: #fff; }}
  </style>
</head>
<body>
  <h1>{title}</h1>
  {''.join(panels)}
</body>
</html>
"""
    path.write_text(html, encoding="utf-8")


def _write_observer_model_yaml(
    path: Path,
    tau: float,
    k_velocity: float,
    input_gain: float,
    throttle_deadband: float,
    velocity_rmse: float,
    accel_rmse: float,
    source_yaml: Path,
    raw_csv: Path,
) -> None:
    payload = {
        "source_calibration": "throttle_acceleration",
        "source_yaml": str(source_yaml),
        "source_raw_csv": str(raw_csv),
        "observer_model": {
            "model_name": "deadband_compensated_velocity_lag",
            "model_type": "first_order_throttle_to_velocity",
            "math": {
                "effective_input": (
                    "u_eff = sign(u) * max(abs(u) - u_dead, 0)"
                ),
                "velocity_dynamics": "v_dot = (K_v * u_eff - v) / tau",
                "acceleration_output": "a_model = (K_v * u_eff - v) / tau",
                "discrete_prediction": "v[k+1] = v[k] + dt * a_model[k]",
                "residual": "r_v = motor_tach - v_pred",
                "fusion": (
                    "a_hat = w_model(r_v) * a_model + "
                    "(1 - w_model(r_v)) * a_tach_lpf"
                ),
            },
            "recommended_longitudinal_model": "velocity_lag",
            "velocity_lag_model": {
                "tau": float(tau),
                "velocity_gain": float(k_velocity),
                "throttle_deadband": float(throttle_deadband),
            },
            "accel_lag_model": {
                "tau": float(tau),
                "input_gain": float(input_gain),
                "note": (
                    "Comparison-only no-drag acceleration lag; use "
                    "velocity_lag_model for observer prediction."
                ),
            },
            "quality": {
                "velocity_rmse_mps": float(velocity_rmse),
                "acceleration_rmse_mps2": float(accel_rmse),
            },
            "copy_to": {
                "local_observer_config": (
                    "Observer/config_local_estimators.yaml -> local.ekf"
                ),
                "trust_fleet_config": (
                    "Observer/TrustbasedDistributedObserver/"
                    "config_trust_estimator.yaml -> vehicle.velocity_lag_model"
                ),
            },
            "config_patch": {
                "local": {
                    "ekf": {
                        "longitudinal_model": "velocity_lag",
                        "use_tachometer_update": True,
                        "velocity_lag_model": {
                            "tau": float(tau),
                            "velocity_gain": float(k_velocity),
                            "throttle_deadband": float(throttle_deadband),
                        },
                        "acceleration_fusion": {
                            "enabled": True,
                            "tach_derivative_alpha": 0.20,
                            "output_alpha": 0.35,
                            "residual_low_mps": 0.04,
                            "residual_high_mps": 0.20,
                            "model_weight_high": 0.90,
                            "model_weight_low": 0.60,
                            "imu_weight": 0.0,
                            "imu_residual_gate_mps2": 1.5,
                        },
                    }
                },
                "trust_based_distributed_observer": {
                    "vehicle": {
                        "longitudinal_model": "velocity_lag",
                        "velocity_lag_model": {
                            "enabled": True,
                            "tau": float(tau),
                            "velocity_gain": float(k_velocity),
                            "throttle_deadband": float(throttle_deadband),
                        },
                    }
                },
            },
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.dump(payload, f, default_flow_style=False, sort_keys=False)


def _model_weight_from_residual(
    residual_abs: float,
    residual_low: float,
    residual_high: float,
    model_weight_high: float,
    model_weight_low: float,
) -> float:
    if residual_abs <= residual_low:
        return model_weight_high
    if residual_abs >= residual_high:
        return model_weight_low
    ratio = (residual_abs - residual_low) / max(residual_high - residual_low, 1e-9)
    return float(model_weight_high + ratio * (model_weight_low - model_weight_high))


def _simulate_runtime_fusion(
    throttle: np.ndarray,
    measured_v: np.ndarray,
    ax: np.ndarray,
    dt: float,
    tau: float,
    k_velocity: float,
    throttle_deadband: float,
    max_accel: float,
    tach_alpha: float,
    output_alpha: float,
    residual_low: float,
    residual_high: float,
    model_weight_high: float,
    model_weight_low: float,
    imu_weight: float,
    imu_gate: float,
) -> Dict[str, np.ndarray]:
    n = len(measured_v)
    a_model = np.zeros(n)
    a_tach = np.zeros(n)
    a_fused = np.zeros(n)
    residual = np.zeros(n)
    model_weight = np.full(n, model_weight_high, dtype=float)
    v_pred = np.zeros(n)
    v_pred[0] = measured_v[0]
    v_hat = float(measured_v[0])
    tau = max(float(tau), 1e-9)
    throttle_eff = _apply_throttle_deadband(throttle, throttle_deadband)

    for i in range(1, n):
        u = float(throttle_eff[i - 1])
        a_model[i] = (k_velocity * u - v_hat) / tau
        a_model[i] = float(np.clip(a_model[i], -max_accel, max_accel))
        v_pred[i] = v_hat + a_model[i] * dt
        residual[i] = float(measured_v[i] - v_pred[i])

        raw_tach_accel = (measured_v[i] - measured_v[i - 1]) / dt
        raw_tach_accel = float(np.clip(raw_tach_accel, -max_accel, max_accel))
        a_tach[i] = tach_alpha * raw_tach_accel + (1.0 - tach_alpha) * a_tach[i - 1]

        w_model = _model_weight_from_residual(
            abs(residual[i]),
            residual_low,
            residual_high,
            model_weight_high,
            model_weight_low,
        )
        model_weight[i] = w_model
        fused = w_model * a_model[i] + (1.0 - w_model) * a_tach[i]
        if imu_weight > 0.0 and np.isfinite(ax[i]) and abs(ax[i] - fused) <= imu_gate:
            fused = (1.0 - imu_weight) * fused + imu_weight * ax[i]
        fused = float(np.clip(fused, -max_accel, max_accel))
        a_fused[i] = output_alpha * fused + (1.0 - output_alpha) * a_fused[i - 1]

        # The real EKF corrects velocity with the tachometer. For this diagnostic
        # plot, use measured velocity as the corrected v_hat for the next model step.
        v_hat = float(measured_v[i])

    return {
        "a_model": a_model,
        "a_tach": a_tach,
        "a_fused": a_fused,
        "velocity_residual": residual,
        "model_weight": model_weight,
        "v_pred": v_pred,
    }


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
        "--reference-filter",
        default="savgol",
        choices=["savgol", "zero_phase_ema", "ema", "none"],
        help="Offline velocity smoother used before differentiating reference acceleration",
    )
    parser.add_argument(
        "--filter-window",
        type=int,
        default=31,
        help="Odd-ish Savitzky-Golay window length in samples",
    )
    parser.add_argument(
        "--filter-poly",
        type=int,
        default=3,
        help="Savitzky-Golay polynomial order",
    )
    parser.add_argument(
        "--filter-alpha",
        type=float,
        default=0.20,
        help="EMA alpha for EMA/zero-phase EMA reference filters",
    )
    parser.add_argument("--max-accel", type=float, default=2.0)
    parser.add_argument("--tach-alpha", type=float, default=0.20)
    parser.add_argument("--output-alpha", type=float, default=0.35)
    parser.add_argument("--residual-low", type=float, default=0.04)
    parser.add_argument("--residual-high", type=float, default=0.20)
    parser.add_argument("--model-weight-high", type=float, default=0.90)
    parser.add_argument("--model-weight-low", type=float, default=0.60)
    parser.add_argument("--imu-weight", type=float, default=0.0)
    parser.add_argument("--imu-gate", type=float, default=1.5)
    parser.add_argument(
        "--throttle-deadband",
        type=float,
        default=None,
        help="Override velocity-lag throttle deadband. Use 0 to disable.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output PNG path. Defaults to <results-dir>/online_throttle_acceleration_fit.png",
    )
    parser.add_argument(
        "--diagnostics-csv",
        default=None,
        help="Optional diagnostics CSV path. Defaults next to the output PNG.",
    )
    parser.add_argument(
        "--html-output",
        default=None,
        help="Fallback dependency-free HTML/SVG plot path.",
    )
    parser.add_argument(
        "--metrics-only",
        action="store_true",
        help="Compute metrics and diagnostics CSV without importing matplotlib.",
    )
    parser.add_argument(
        "--export-observer-model",
        action="store_true",
        help="Write throttle_acceleration_observer_model_latest.yaml from the selected YAML/raw CSV.",
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
    diagnostics_csv_path = (
        Path(args.diagnostics_csv).resolve()
        if args.diagnostics_csv
        else output_path.with_suffix(".diagnostics.csv")
    )
    html_output_path = (
        Path(args.html_output).resolve()
        if args.html_output
        else output_path.with_suffix(".html")
    )

    model = _load_yaml(yaml_path)
    raw = _load_raw_csv(raw_path)

    v = raw["v"]
    throttle = raw["throttle"]
    ax = raw.get("ax", np.full_like(v, np.nan))
    dt = float(args.sample_dt)
    t = np.arange(len(v), dtype=float) * dt

    tau, k_velocity, observer_input_gain, model_deadband = _observer_velocity_lag_params(model)
    throttle_deadband = (
        max(float(args.throttle_deadband), 0.0)
        if args.throttle_deadband is not None
        else model_deadband
    )
    input_gain = (
        float(args.input_gain)
        if args.input_gain is not None
        else observer_input_gain
    )

    v_velocity_lag = _simulate_velocity_lag(
        throttle=throttle,
        v0=float(v[0]),
        dt=dt,
        tau=tau,
        k_velocity=k_velocity,
        throttle_deadband=throttle_deadband,
    )
    a0 = float(ax[0]) if np.isfinite(ax[0]) else 0.0
    v_accel_lag, a_accel_lag = _simulate_accel_lag(
        throttle=throttle,
        v0=float(v[0]),
        a0=a0,
        dt=dt,
        tau=tau,
        input_gain=input_gain,
    )

    v_ref_smooth, filter_label = _smooth_velocity(
        v,
        method=args.reference_filter,
        window_samples=args.filter_window,
        poly_order=args.filter_poly,
        ema_alpha=args.filter_alpha,
    )
    acc_from_v_raw = np.gradient(v, dt)
    acc_ref = np.gradient(v_ref_smooth, dt)
    fusion = _simulate_runtime_fusion(
        throttle=throttle,
        measured_v=v,
        ax=ax,
        dt=dt,
        tau=tau,
        k_velocity=k_velocity,
        throttle_deadband=throttle_deadband,
        max_accel=args.max_accel,
        tach_alpha=args.tach_alpha,
        output_alpha=args.output_alpha,
        residual_low=args.residual_low,
        residual_high=args.residual_high,
        model_weight_high=args.model_weight_high,
        model_weight_low=args.model_weight_low,
        imu_weight=args.imu_weight,
        imu_gate=args.imu_gate,
    )
    rmse_velocity_lag = _rmse(v, v_velocity_lag)
    rmse_accel_lag = _rmse(v, v_accel_lag)
    rmse_model_acc = _rmse(acc_ref, fusion["a_model"])
    rmse_fused_acc = _rmse(acc_ref, fusion["a_fused"])
    rmse_tach_acc = _rmse(acc_ref, fusion["a_tach"])
    v_span = max(float(np.nanmax(v) - np.nanmin(v)), 1e-6)
    accel_lag_span = float(np.nanmax(v_accel_lag) - np.nanmin(v_accel_lag))

    _write_diagnostics_csv(
        diagnostics_csv_path,
        t=t,
        velocity=v,
        velocity_filtered=v_ref_smooth,
        throttle=throttle,
        ax=ax,
        acc_ref=acc_ref,
        acc_raw_dvdt=acc_from_v_raw,
        fusion=fusion,
    )
    observer_model_path = results_dir / "throttle_acceleration_observer_model_latest.yaml"
    if args.export_observer_model:
        _write_observer_model_yaml(
            observer_model_path,
            tau=tau,
            k_velocity=k_velocity,
            input_gain=input_gain,
            throttle_deadband=throttle_deadband,
            velocity_rmse=rmse_velocity_lag,
            accel_rmse=rmse_fused_acc,
            source_yaml=yaml_path,
            raw_csv=raw_path,
        )

    def _print_metrics(saved_plot: bool) -> None:
        print(f"[plot] YAML: {yaml_path}")
        print(f"[plot] Raw CSV: {raw_path}")
        if saved_plot:
            print(f"[plot] Saved: {output_path}")
        print(f"[plot] Diagnostics CSV: {diagnostics_csv_path}")
        if args.export_observer_model:
            print(f"[plot] Observer model YAML: {observer_model_path}")
        print(f"[plot] velocity-lag RMSE: {rmse_velocity_lag:.4f} m/s")
        print(f"[plot] accel-lag RMSE: {rmse_accel_lag:.4f} m/s")
        print(f"[plot] offline reference filter: {filter_label}")
        print(f"[plot] model acceleration RMSE: {rmse_model_acc:.4f} m/s^2")
        print(f"[plot] fused acceleration RMSE: {rmse_fused_acc:.4f} m/s^2")
        print(f"[plot] tach derivative acceleration RMSE: {rmse_tach_acc:.4f} m/s^2")
        if accel_lag_span > 3.0 * v_span:
            print(
                "[plot] note: accel-lag velocity is on a separate right axis because "
                "this no-drag acceleration model does not settle to a steady speed."
            )
        print(
            "[plot] recommended observer velocity_lag_model: "
            f"tau={tau:.4f}, velocity_gain={k_velocity:.4f}, "
            f"throttle_deadband={throttle_deadband:.4f}"
        )

    if args.metrics_only:
        _print_metrics(saved_plot=False)
        return 0

    try:
        import matplotlib

        if not args.show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        _write_html_diagnostics_plot(
            html_output_path,
            t=t,
            velocity=v,
            velocity_filtered=v_ref_smooth,
            velocity_model=v_velocity_lag,
            throttle=throttle,
            ax=ax,
            acc_ref=acc_ref,
            acc_raw_dvdt=acc_from_v_raw,
            fusion=fusion,
            title="Online Throttle Acceleration Fit Diagnostics",
        )
        _print_metrics(saved_plot=False)
        print(f"[plot] HTML fallback plot: {html_output_path}")
        print(f"[plot] matplotlib unavailable; skipped PNG generation: {exc}")
        return 0

    fig, axes = plt.subplots(6, 1, figsize=(14, 14), sharex=True)
    fig.suptitle(
        "Online Throttle Acceleration Calibration: Fit, Residuals, Fusion",
        fontsize=13,
        fontweight="bold",
    )

    axes[0].plot(t, v, color="black", linewidth=1.2, label="Measured velocity")
    axes[0].plot(
        t,
        v_ref_smooth,
        color="0.55",
        linewidth=1.0,
        alpha=0.9,
        label=f"Filtered velocity for offline ref ({filter_label})",
    )
    axes[0].plot(
        t,
        v_velocity_lag,
        color="tab:blue",
        linewidth=1.4,
        label=(
            f"Observer velocity lag: tau={tau:.3f}s, K={k_velocity:.3f}, "
            f"RMSE={rmse_velocity_lag:.3f} m/s"
        ),
    )
    accel_lag_label = (
        f"No-drag accel lag comparison: tau={tau:.3f}s, "
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
        acc_from_v_raw,
        color="0.75",
        linewidth=0.8,
        alpha=0.65,
        label="Raw dv/dt from measured v",
    )
    axes[2].plot(
        t,
        acc_ref,
        color="tab:green",
        linewidth=1.2,
        label=f"Offline reference d(filtered v)/dt ({filter_label})",
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
        fusion["a_model"],
        color="tab:blue",
        linewidth=1.1,
        label=f"Model a=(K*u-v)/tau, RMSE={rmse_model_acc:.3f}",
    )
    axes[2].plot(
        t,
        fusion["a_fused"],
        color="tab:red",
        linewidth=1.2,
        label=f"Runtime fused acceleration, RMSE={rmse_fused_acc:.3f}",
    )
    axes[2].set_ylabel("Acceleration [m/s^2]")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="best", fontsize=8)

    axes[3].plot(
        t,
        fusion["velocity_residual"],
        color="tab:orange",
        linewidth=0.9,
        label="Velocity residual: motorTach - v_pred",
    )
    axes[3].axhline(args.residual_low, color="0.45", linestyle="--", linewidth=0.8)
    axes[3].axhline(-args.residual_low, color="0.45", linestyle="--", linewidth=0.8)
    axes[3].axhline(args.residual_high, color="0.25", linestyle=":", linewidth=0.8)
    axes[3].axhline(-args.residual_high, color="0.25", linestyle=":", linewidth=0.8)
    axes_weight = axes[3].twinx()
    axes_weight.plot(
        t,
        fusion["model_weight"],
        color="tab:blue",
        linewidth=0.9,
        alpha=0.8,
        label="Model trust weight",
    )
    axes[3].set_ylabel("Residual [m/s]")
    axes_weight.set_ylabel("Model weight")
    axes[3].grid(True, alpha=0.3)
    lines, labels = axes[3].get_legend_handles_labels()
    lines2, labels2 = axes_weight.get_legend_handles_labels()
    axes[3].legend(lines + lines2, labels + labels2, loc="best", fontsize=8)

    axes[4].plot(
        t,
        fusion["a_model"] - acc_ref,
        color="tab:blue",
        linewidth=0.8,
        label=f"Model error vs ref, RMSE={rmse_model_acc:.3f}",
    )
    axes[4].plot(
        t,
        fusion["a_tach"] - acc_ref,
        color="tab:cyan",
        linewidth=0.8,
        label=f"Tach derivative error vs ref, RMSE={rmse_tach_acc:.3f}",
    )
    axes[4].plot(
        t,
        fusion["a_fused"] - acc_ref,
        color="tab:red",
        linewidth=0.9,
        label=f"Fused error vs ref, RMSE={rmse_fused_acc:.3f}",
    )
    axes[4].axhline(0.0, color="black", linewidth=0.6)
    axes[4].set_ylabel("Accel error [m/s^2]")
    axes[4].grid(True, alpha=0.3)
    axes[4].legend(loc="best", fontsize=8)

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
        axes_gain = axes[5].twinx()
        axes[5].scatter(
            idx,
            step_taus,
            c=step_du,
            cmap="viridis",
            s=30,
            label="tau per detected transition",
        )
        axes_gain.plot(
            idx,
            step_gains,
            color="tab:orange",
            linewidth=1.0,
            marker=".",
            label="K_local / tau_s",
        )
        axes[5].axhline(tau, color="tab:blue", linestyle="--", linewidth=1.0)
        axes_gain.axhline(
            input_gain, color="tab:red", linestyle="--", linewidth=1.0
        )
        axes[5].set_ylabel("Tau [s]")
        axes_gain.set_ylabel("Derived input_gain [m/s^2/throttle]")
        axes[5].set_xlabel("Detected transition index")
        axes[5].grid(True, alpha=0.3)
        lines, labels = axes[5].get_legend_handles_labels()
        lines2, labels2 = axes_gain.get_legend_handles_labels()
        axes[5].legend(lines + lines2, labels + labels2, loc="best", fontsize=8)
    else:
        axes[5].text(0.02, 0.5, "No step model entries found", transform=axes[5].transAxes)
        axes[5].set_xlabel("Time [s]")

    axes[2].set_ylim(
        max(np.nanpercentile(acc_from_v_raw, 1), -8.0),
        min(np.nanpercentile(acc_from_v_raw, 99), 8.0),
    )
    axes[0].set_xlim(t[0], t[-1])
    axes[-1].set_xlabel("Time [s]")

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=160)

    _print_metrics(saved_plot=True)
    if args.show:
        print("[plot] Close the Matplotlib window to finish.")
        plt.show()
    plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
