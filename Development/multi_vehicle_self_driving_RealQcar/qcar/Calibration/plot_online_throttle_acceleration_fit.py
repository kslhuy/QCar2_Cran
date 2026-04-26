#!/usr/bin/env python3
"""Plot passive online throttle-acceleration fit and observer diagnostics."""

# CMD examples:
# python .\plot_online_throttle_acceleration_fit.py --metrics-only --export-observer-model
# python .\plot_online_throttle_acceleration_fit.py --show
# python .\plot_online_throttle_acceleration_fit.py --results-dir results\online_throttle_acceleration\calibration\61 --show
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


def _constant_command_segments(
    command: np.ndarray,
    quant_step: float,
    min_samples: int,
) -> List[Tuple[int, int, float]]:
    command = np.asarray(command, dtype=float).reshape(-1)
    if command.size == 0:
        return []
    q = np.round(command / quant_step) * quant_step
    segments: List[Tuple[int, int, float]] = []
    start = 0
    for i in range(1, q.size):
        if abs(q[i] - q[start]) > 0.5 * quant_step:
            if i - start >= min_samples:
                segments.append((start, i, float(q[start])))
            start = i
    if q.size - start >= min_samples:
        segments.append((start, q.size, float(q[start])))
    return segments


def _build_velocity_lag_lookup_model_from_raw(
    velocity: np.ndarray,
    throttle: np.ndarray,
    sample_dt: float,
    model: Dict,
) -> Optional[Dict[str, object]]:
    segments = _constant_command_segments(
        throttle,
        quant_step=0.01,
        min_samples=max(5, int(0.5 / max(float(sample_dt), 1e-9))),
    )
    if not segments:
        return None

    steady_by_throttle: Dict[float, List[float]] = {}
    for start, end, u in segments:
        seg_len = end - start
        if seg_len <= 0:
            continue
        tail_start = start + int(0.7 * seg_len)
        tail = np.asarray(velocity[tail_start:end], dtype=float)
        if tail.size < max(3, int(0.20 / max(float(sample_dt), 1e-9))):
            continue
        key = float(round(u, 4))
        steady_by_throttle.setdefault(key, []).append(float(np.mean(tail)))

    if len(steady_by_throttle) < 3:
        return None

    throttle_breakpoints = np.asarray(sorted(steady_by_throttle.keys()), dtype=float)
    steady_state_velocity_breakpoints = np.asarray(
        [
            float(np.median(steady_by_throttle[float(u)]))
            for u in throttle_breakpoints
        ],
        dtype=float,
    )
    zero_idx = int(np.argmin(np.abs(throttle_breakpoints)))
    if abs(float(throttle_breakpoints[zero_idx])) <= 0.005:
        steady_state_velocity_breakpoints[zero_idx] = 0.0
    steady_state_velocity_breakpoints = np.maximum.accumulate(
        steady_state_velocity_breakpoints
    )

    tau_candidates = []
    for step in model.get("step_models", []):
        try:
            tau = float(step["tau_s"])
        except (KeyError, TypeError, ValueError):
            continue
        if np.isfinite(tau) and tau > 0.0:
            tau_candidates.append(tau)
    if not tau_candidates:
        tau_candidates = [float(model.get("robust_tau", model.get("avg_tau", 0.3)))]

    tau_values = np.asarray(tau_candidates, dtype=float)
    tau_med = float(np.nanmedian(tau_values))
    tau_lo = float(np.nanpercentile(tau_values, 15))
    tau_hi = float(np.nanpercentile(tau_values, 85))
    if not np.isfinite(tau_lo) or tau_lo <= 0.0:
        tau_lo = max(0.5 * tau_med, 0.05)
    if not np.isfinite(tau_hi) or tau_hi <= tau_lo:
        tau_hi = max(1.5 * tau_med, tau_lo + 0.05)
    tau_grid = np.unique(
        np.concatenate(
            [np.linspace(tau_lo, tau_hi, 41, dtype=float), np.asarray([tau_med])]
        )
    )
    best_tau = tau_med
    best_rmse = float("inf")
    for tau in tau_grid:
        pred = _simulate_velocity_lag_lookup(
            throttle=throttle,
            v0=float(velocity[0]),
            dt=float(sample_dt),
            lookup_model={
                "tau": float(tau),
                "throttle_breakpoints": throttle_breakpoints,
                "steady_state_velocity_breakpoints": steady_state_velocity_breakpoints,
            },
        )
        err = _rmse(velocity, pred)
        if np.isfinite(err) and err < best_rmse:
            best_rmse = float(err)
            best_tau = float(tau)

    return {
        "tau": float(best_tau),
        "throttle_breakpoints": throttle_breakpoints,
        "steady_state_velocity_breakpoints": steady_state_velocity_breakpoints,
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


def _observer_velocity_lag_lookup_model(model: Dict) -> Optional[Dict[str, object]]:
    observer_model = model.get("observer_model", {})
    lookup = {}
    if isinstance(observer_model, dict):
        raw_lookup = observer_model.get("velocity_lag_lookup_model", {})
        if isinstance(raw_lookup, dict):
            lookup = raw_lookup
    if not lookup:
        raw_lookup = model.get("velocity_lag_lookup_model", {})
        if isinstance(raw_lookup, dict):
            lookup = raw_lookup
    if not lookup:
        return None

    try:
        throttle_breakpoints = np.asarray(
            lookup["throttle_breakpoints"], dtype=float
        ).reshape(-1)
        steady_state_velocity_breakpoints = np.asarray(
            lookup["steady_state_velocity_breakpoints"], dtype=float
        ).reshape(-1)
        tau = float(lookup["tau"])
    except (KeyError, TypeError, ValueError):
        return None

    if (
        throttle_breakpoints.size < 2
        or throttle_breakpoints.size != steady_state_velocity_breakpoints.size
        or not np.all(np.isfinite(throttle_breakpoints))
        or not np.all(np.isfinite(steady_state_velocity_breakpoints))
        or not np.isfinite(tau)
        or tau <= 0.0
    ):
        return None

    return {
        "tau": float(tau),
        "throttle_breakpoints": throttle_breakpoints,
        "steady_state_velocity_breakpoints": steady_state_velocity_breakpoints,
    }


def _lookup_target_velocity(
    throttle: np.ndarray,
    lookup_model: Dict[str, np.ndarray],
) -> np.ndarray:
    return np.interp(
        np.asarray(throttle, dtype=float),
        np.asarray(lookup_model["throttle_breakpoints"], dtype=float),
        np.asarray(lookup_model["steady_state_velocity_breakpoints"], dtype=float),
    )


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


def _simulate_velocity_lag_lookup(
    throttle: np.ndarray,
    v0: float,
    dt: float,
    lookup_model: Dict[str, object],
) -> np.ndarray:
    pred = np.zeros_like(throttle, dtype=float)
    if pred.size == 0:
        return pred
    pred[0] = v0
    tau = max(float(lookup_model["tau"]), 1e-9)
    target = _lookup_target_velocity(throttle, lookup_model)
    for i in range(1, len(throttle)):
        pred[i] = pred[i - 1] + dt * ((float(target[i - 1]) - pred[i - 1]) / tau)
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


def _model_target_velocity_from_throttle(
    throttle: np.ndarray,
    k_velocity: float,
    throttle_deadband: float,
    lookup_model: Optional[Dict[str, np.ndarray]] = None,
) -> np.ndarray:
    if lookup_model is not None:
        return _lookup_target_velocity(throttle, lookup_model)
    throttle_eff = _apply_throttle_deadband(throttle, throttle_deadband)
    return float(k_velocity) * throttle_eff


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
        f'<rect x="0" y="0" width="{width}" height="{height}" fill="#fcfcf8"/>',
    ]
    for frac in (0.2, 0.4, 0.6, 0.8):
        x = pad + frac * (width - 2 * pad)
        y = height - pad - frac * (height - 2 * pad)
        lines.append(
            f'<line x1="{x:.1f}" y1="{pad}" x2="{x:.1f}" y2="{height-pad}" '
            'stroke="#e5e7eb" stroke-dasharray="4 4"/>'
        )
        lines.append(
            f'<line x1="{pad}" y1="{y:.1f}" x2="{width-pad}" y2="{y:.1f}" '
            'stroke="#e5e7eb" stroke-dasharray="4 4"/>'
        )
    lines.extend(
        [
            f'<line x1="{pad}" y1="{height-pad}" x2="{width-pad}" y2="{height-pad}" stroke="#64748b" stroke-width="1.2"/>',
            f'<line x1="{pad}" y1="{pad}" x2="{pad}" y2="{height-pad}" stroke="#64748b" stroke-width="1.2"/>',
        ]
    )
    if y_zero is not None:
        lines.append(
            f'<line x1="{pad}" y1="{y_zero:.1f}" x2="{width-pad}" y2="{y_zero:.1f}" '
            'stroke="#94a3b8" stroke-width="1.1"/>'
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
    notes_html = """
    <div class="note">
      <strong>Reading guide.</strong>
      The blue velocity model is the observer candidate. The red no-drag acceleration-lag curve is only a comparison model and is not the recommended steady-speed model for a real QCar.
    </div>
    """
    panels = [
        _svg_panel(
            "Velocity Fit",
            t,
            [
                ("measured v", velocity, "#111827"),
                ("filtered v", velocity_filtered, "#6b7280"),
                ("velocity model", velocity_model, "#0f766e"),
            ],
            "m/s",
        ),
        _svg_panel(
            "Throttle Command",
            t,
            [("throttle", throttle, "#7c3aed")],
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
                ("offline filtered dv/dt ref", acc_ref, "#166534"),
                ("raw dv/dt", acc_raw_dvdt, "#cbd5e1"),
                ("raw IMU ax", ax, "#c2410c"),
                ("model a", fusion["a_model"], "#1d4ed8"),
                ("fused a", fusion["a_fused"], "#dc2626"),
            ],
            "m/s^2",
        ),
        _svg_panel(
            "Residual And Model Weight",
            t,
            [
                ("velocity residual", fusion["velocity_residual"], "#ea580c"),
                ("model weight", fusion["model_weight"], "#2563eb"),
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
                ("model error", fusion["a_model"] - acc_ref, "#0f766e"),
                ("tach lpf error", fusion["a_tach"] - acc_ref, "#0284c7"),
                ("fused error", fusion["a_fused"] - acc_ref, "#dc2626"),
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
    body {{ font-family: Segoe UI, Arial, sans-serif; margin: 22px; color: #1f2937; background: #f7f7f4; }}
    h1 {{ font-size: 20px; margin-bottom: 10px; }}
    h3 {{ margin: 0 0 6px; font-size: 15px; }}
    .note {{ max-width: 1180px; margin: 0 0 18px; padding: 12px 14px; background: #fff7ed; border-left: 4px solid #ea580c; }}
    .grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(560px, 1fr)); gap: 18px; align-items: start; }}
    section {{ max-width: 1220px; background: white; padding: 12px; border: 1px solid #e5e7eb; box-shadow: 0 1px 2px rgba(15, 23, 42, 0.06); }}
    svg {{ width: 100%; height: auto; background: #fff; }}
  </style>
</head>
<body>
  <h1>{title}</h1>
  {notes_html}
  <div class="grid">{''.join(panels)}</div>
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
    lookup_model: Optional[Dict[str, np.ndarray]] = None,
    lookup_velocity_rmse: float = float("nan"),
    lookup_accel_rmse: float = float("nan"),
    recommended_longitudinal_model: str = "velocity_lag",
) -> None:
    lookup_model = lookup_model if isinstance(lookup_model, dict) else None
    lookup_cfg = None
    if lookup_model is not None:
        lookup_cfg = {
            "enabled": True,
            "tau": float(lookup_model["tau"]),
            "interpolation": "linear",
            "throttle_breakpoints": [
                float(v) for v in np.asarray(lookup_model["throttle_breakpoints"], dtype=float)
            ],
            "steady_state_velocity_breakpoints": [
                float(v)
                for v in np.asarray(
                    lookup_model["steady_state_velocity_breakpoints"], dtype=float
                )
            ],
        }
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
            "recommended_longitudinal_model": str(recommended_longitudinal_model),
            "velocity_lag_model": {
                "tau": float(tau),
                "velocity_gain": float(k_velocity),
                "throttle_deadband": float(throttle_deadband),
            },
            "velocity_lag_lookup_model": lookup_cfg,
            "accel_lag_model": {
                "tau": float(tau),
                "input_gain": float(input_gain),
                "note": (
                    "Comparison-only no-drag acceleration lag; use "
                    "velocity_lag_lookup_model when available, else "
                    "velocity_lag_model, for observer prediction."
                ),
            },
            "quality": {
                "velocity_rmse_mps": float(velocity_rmse),
                "acceleration_rmse_mps2": float(accel_rmse),
                "velocity_lookup_rmse_mps": float(lookup_velocity_rmse),
                "acceleration_lookup_rmse_mps2": float(lookup_accel_rmse),
            },
            "copy_to": {
                "local_observer_config": (
                    "Observer/config_local_estimators.yaml -> local.ekf"
                ),
                "trust_fleet_config": (
                    "Observer/TrustbasedDistributedObserver/"
                    "config_trust_estimator.yaml -> vehicle.velocity_lag_model "
                    "or vehicle.velocity_lag_lookup_model"
                ),
            },
            "config_patch": {
                "local": {
                    "ekf": {
                        "longitudinal_model": str(recommended_longitudinal_model),
                        "use_tachometer_update": True,
                        "velocity_lag_model": {
                            "tau": float(tau),
                            "velocity_gain": float(k_velocity),
                            "throttle_deadband": float(throttle_deadband),
                        },
                        "velocity_lag_lookup_model": lookup_cfg,
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
                        "longitudinal_model": str(recommended_longitudinal_model),
                        "velocity_lag_model": {
                            "enabled": True,
                            "tau": float(tau),
                            "velocity_gain": float(k_velocity),
                            "throttle_deadband": float(throttle_deadband),
                        },
                        "velocity_lag_lookup_model": lookup_cfg,
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
    lookup_model: Optional[Dict[str, np.ndarray]] = None,
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
    v_target = _model_target_velocity_from_throttle(
        throttle=throttle,
        k_velocity=k_velocity,
        throttle_deadband=throttle_deadband,
        lookup_model=lookup_model,
    )

    for i in range(1, n):
        a_model[i] = (float(v_target[i - 1]) - v_hat) / tau
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
    lookup_model = _observer_velocity_lag_lookup_model(model)
    if lookup_model is None:
        lookup_model = _build_velocity_lag_lookup_model_from_raw(
            velocity=v,
            throttle=throttle,
            sample_dt=dt,
            model=model,
        )
    recommended_model = str(
        model.get("recommended_longitudinal_model")
        or model.get("observer_model", {}).get("recommended_longitudinal_model")
        or "velocity_lag"
    ).strip().lower()
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
    v_velocity_lag_lookup = (
        _simulate_velocity_lag_lookup(
            throttle=throttle,
            v0=float(v[0]),
            dt=dt,
            lookup_model=lookup_model,
        )
        if lookup_model is not None
        else None
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
    rmse_velocity_lag = _rmse(v, v_velocity_lag)
    rmse_velocity_lag_lookup = (
        _rmse(v, v_velocity_lag_lookup)
        if v_velocity_lag_lookup is not None
        else float("nan")
    )
    use_lookup_model = bool(
        lookup_model is not None
        and (
            recommended_model == "velocity_lag_lookup"
            or (
                np.isfinite(rmse_velocity_lag_lookup)
                and rmse_velocity_lag_lookup < rmse_velocity_lag
            )
        )
    )
    selected_velocity_model = (
        v_velocity_lag_lookup if use_lookup_model else v_velocity_lag
    )
    selected_tau = (
        float(lookup_model["tau"])
        if use_lookup_model and lookup_model is not None
        else tau
    )
    selected_model_name = (
        "velocity_lag_lookup" if use_lookup_model else "velocity_lag"
    )
    fusion = _simulate_runtime_fusion(
        throttle=throttle,
        measured_v=v,
        ax=ax,
        dt=dt,
        tau=selected_tau,
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
        lookup_model=lookup_model if use_lookup_model else None,
    )
    rmse_accel_lag = _rmse(v, v_accel_lag)
    rmse_model_acc = _rmse(acc_ref, fusion["a_model"])
    rmse_fused_acc = _rmse(acc_ref, fusion["a_fused"])
    rmse_tach_acc = _rmse(acc_ref, fusion["a_tach"])
    rmse_selected_velocity = (
        rmse_velocity_lag_lookup if use_lookup_model else rmse_velocity_lag
    )
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
            lookup_model=lookup_model,
            lookup_velocity_rmse=rmse_velocity_lag_lookup,
            lookup_accel_rmse=rmse_model_acc if use_lookup_model else float("nan"),
            recommended_longitudinal_model=selected_model_name,
        )

    def _print_metrics(saved_plot: bool) -> None:
        print(f"[plot] YAML: {yaml_path}")
        print(f"[plot] Raw CSV: {raw_path}")
        if saved_plot:
            print(f"[plot] Saved: {output_path}")
        print(f"[plot] Diagnostics CSV: {diagnostics_csv_path}")
        if args.export_observer_model:
            print(f"[plot] Observer model YAML: {observer_model_path}")
        print(f"[plot] linear velocity-lag RMSE: {rmse_velocity_lag:.4f} m/s")
        if np.isfinite(rmse_velocity_lag_lookup):
            print(
                "[plot] lookup velocity-lag RMSE: "
                f"{rmse_velocity_lag_lookup:.4f} m/s"
            )
        print(
            "[plot] selected longitudinal model: "
            f"{selected_model_name}, RMSE={rmse_selected_velocity:.4f} m/s"
        )
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
            "[plot] recommended observer model: "
            f"{selected_model_name}"
        )
        if use_lookup_model and lookup_model is not None:
            print(
                "[plot] lookup tau and breakpoints: "
                f"tau={selected_tau:.4f}, "
                f"n={len(lookup_model['throttle_breakpoints'])}"
            )
        else:
            print(
                "[plot] linear velocity-lag params: "
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
            velocity_model=selected_velocity_model,
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

    palette = {
        "ink": "#111827",
        "muted": "#6b7280",
        "teal": "#0f766e",
        "cyan": "#0891b2",
        "green": "#166534",
        "red": "#dc2626",
        "orange": "#ea580c",
        "burnt": "#c2410c",
        "violet": "#7c3aed",
        "blue": "#2563eb",
        "royal": "#1d4ed8",
        "slate": "#475569",
        "light": "#cbd5e1",
    }

    def _style_axis(ax, title: str, ylabel: str) -> None:
        ax.set_title(title, fontsize=11, fontweight="bold", loc="left")
        ax.set_ylabel(ylabel)
        ax.set_facecolor("#fcfcf8")
        ax.grid(True, which="major", color="#d6d3d1", alpha=0.65, linewidth=0.8)
        ax.grid(True, which="minor", color="#e7e5e4", alpha=0.55, linewidth=0.5)
        ax.minorticks_on()
        for spine in ax.spines.values():
            spine.set_color("#a8a29e")

    fig, grid = plt.subplots(3, 2, figsize=(16, 10.5), sharex="col")
    ax_vel, ax_thr = grid[0, 0], grid[0, 1]
    ax_acc, ax_res = grid[1, 0], grid[1, 1]
    ax_err, ax_tau = grid[2, 0], grid[2, 1]
    fig.suptitle(
        "Online Throttle Acceleration Calibration",
        fontsize=15,
        fontweight="bold",
    )
    fig.text(
        0.5,
        0.965,
        "Velocity fit, acceleration comparison, residual fusion, and transition-wise tau spread",
        ha="center",
        va="top",
        fontsize=10,
        color=palette["muted"],
    )

    ax_vel.plot(t, v, color=palette["ink"], linewidth=1.4, label="Measured velocity")
    ax_vel.plot(
        t,
        v_ref_smooth,
        color=palette["muted"],
        linewidth=1.0,
        alpha=0.9,
        label=f"Filtered velocity for offline ref ({filter_label})",
    )
    ax_vel.plot(
        t,
        selected_velocity_model,
        color=palette["teal"],
        linewidth=1.8,
        label=(
            "Observer "
            + (
                f"lookup velocity lag: tau={selected_tau:.3f}s, "
                if use_lookup_model
                else f"velocity lag: tau={tau:.3f}s, K={k_velocity:.3f}, "
            )
            + f"RMSE={rmse_selected_velocity:.3f} m/s"
        ),
    )
    if v_velocity_lag_lookup is not None and use_lookup_model:
        ax_vel.plot(
            t,
            v_velocity_lag,
            color=palette["cyan"],
            linewidth=0.9,
            alpha=0.8,
            label=f"Linear velocity lag fallback, RMSE={rmse_velocity_lag:.3f} m/s",
        )
    accel_lag_label = (
        f"Comparison only: no-drag accel-lag, tau={selected_tau:.3f}s, "
        f"input_gain={input_gain:.2f}, RMSE={rmse_accel_lag:.3f} m/s"
    )
    if accel_lag_span > 3.0 * v_span:
        ax0_right = ax_vel.twinx()
        ax0_right.plot(
            t,
            v_accel_lag,
            color=palette["red"],
            linewidth=1.1,
            alpha=0.72,
            label=accel_lag_label + " (right axis)",
        )
        ax0_right.set_ylabel("No-drag comparison velocity [m/s]", color=palette["red"])
        ax0_right.tick_params(axis="y", labelcolor=palette["red"])
        lines, labels = ax_vel.get_legend_handles_labels()
        lines2, labels2 = ax0_right.get_legend_handles_labels()
        ax_vel.legend(lines + lines2, labels + labels2, loc="upper left", fontsize=8)
    else:
        ax_vel.plot(
            t,
            v_accel_lag,
            color=palette["red"],
            linewidth=1.1,
            alpha=0.85,
            label=accel_lag_label,
        )
        ax_vel.legend(loc="upper left", fontsize=8)
    _style_axis(ax_vel, "Velocity Fit", "Velocity [m/s]")

    ax_thr.plot(t, throttle, color=palette["violet"], linewidth=1.2)
    ax_thr.axhline(0.0, color=palette["muted"], linewidth=0.8, alpha=0.8)
    _style_axis(ax_thr, "Throttle Command", "Throttle")


    ax_acc.plot(
        t,
        ax,
        color=palette["burnt"],
        linewidth=1.05,
        alpha=0.75,
        linestyle="--",
        label="Raw ax column",
    )
    ax_acc.plot(
        t,
        acc_from_v_raw,
        color=palette["light"],
        linewidth=0.9,
        alpha=0.85,
        linestyle="--",
        label="Raw dv/dt from measured v",
    )
    ax_acc.plot(
        t,
        acc_ref,
        color=palette["green"],
        linewidth=1.4,
        label=f"Offline reference d(filtered v)/dt ({filter_label})",
    )

    ax_acc.plot(
        t,
        fusion["a_model"],
        color=palette["royal"],
        linewidth=1.6,
        label=f"Model acceleration, RMSE={rmse_model_acc:.3f}",
    )
    ax_acc.plot(
        t,
        fusion["a_fused"],
        color=palette["red"],
        linewidth=1.4,
        label=f"Runtime fused acceleration, RMSE={rmse_fused_acc:.3f}",
    )
    _style_axis(ax_acc, "Acceleration Comparison", "Acceleration [m/s^2]")
    ax_acc.legend(loc="upper left", fontsize=8)

    ax_res.plot(
        t,
        fusion["velocity_residual"],
        color=palette["orange"],
        linewidth=1.1,
        label="Velocity residual: motorTach - v_pred",
    )
    ax_res.axhline(args.residual_low, color=palette["muted"], linestyle="--", linewidth=0.9)
    ax_res.axhline(-args.residual_low, color=palette["muted"], linestyle="--", linewidth=0.9)
    ax_res.axhline(args.residual_high, color=palette["ink"], linestyle=":", linewidth=0.9)
    ax_res.axhline(-args.residual_high, color=palette["ink"], linestyle=":", linewidth=0.9)
    axes_weight = ax_res.twinx()
    axes_weight.plot(
        t,
        fusion["model_weight"],
        color=palette["blue"],
        linewidth=1.1,
        alpha=0.8,
        label="Model trust weight",
    )
    _style_axis(ax_res, "Residual And Model Weight", "Residual [m/s]")
    axes_weight.set_ylabel("Model weight")
    lines, labels = ax_res.get_legend_handles_labels()
    lines2, labels2 = axes_weight.get_legend_handles_labels()
    ax_res.legend(lines + lines2, labels + labels2, loc="upper left", fontsize=8)

    ax_err.plot(
        t,
        fusion["a_model"] - acc_ref,
        color=palette["teal"],
        linewidth=1.0,
        label=f"Model error vs ref, RMSE={rmse_model_acc:.3f}",
    )
    ax_err.plot(
        t,
        fusion["a_tach"] - acc_ref,
        color=palette["cyan"],
        linewidth=1.0,
        label=f"Tach derivative error vs ref, RMSE={rmse_tach_acc:.3f}",
    )
    ax_err.plot(
        t,
        fusion["a_fused"] - acc_ref,
        color=palette["red"],
        linewidth=1.1,
        label=f"Fused error vs ref, RMSE={rmse_fused_acc:.3f}",
    )
    ax_err.axhline(0.0, color=palette["ink"], linewidth=0.8)
    _style_axis(ax_err, "Acceleration Error", "Accel error [m/s^2]")
    ax_err.legend(loc="upper left", fontsize=8)

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
        axes_gain = ax_tau.twinx()
        scatter = ax_tau.scatter(
            idx,
            step_taus,
            c=step_du,
            cmap="viridis",
            s=40,
            edgecolors="#ffffff",
            linewidths=0.4,
            label="Tau per detected transition",
        )
        axes_gain.plot(
            idx,
            step_gains,
            color=palette["orange"],
            linewidth=1.1,
            marker=".",
            label="Derived input_gain = K_local / tau_s",
        )
        ax_tau.axhline(
            selected_tau,
            color=palette["teal"],
            linestyle="--",
            linewidth=1.2,
            label=f"Selected tau = {selected_tau:.3f}s",
        )
        axes_gain.axhline(
            input_gain, color=palette["red"], linestyle="--", linewidth=1.0
        )
        _style_axis(ax_tau, "Transition-Wise Tau Spread", "Tau [s]")
        axes_gain.set_ylabel("Derived input_gain [m/s^2/throttle]")
        ax_tau.set_xlabel("Detected transition index")
        cbar = fig.colorbar(scatter, ax=ax_tau, pad=0.01)
        cbar.set_label("|Δu| of transition")
        ax_tau.text(
            0.01,
            0.97,
            "Tau is estimated separately for each detected throttle step.\n"
            "Spread means the real car is not one perfect first-order system.",
            transform=ax_tau.transAxes,
            va="top",
            ha="left",
            fontsize=8,
            color=palette["muted"],
            bbox=dict(facecolor="#fff7ed", edgecolor="#fed7aa", boxstyle="round,pad=0.3"),
        )
        lines, labels = ax_tau.get_legend_handles_labels()
        lines2, labels2 = axes_gain.get_legend_handles_labels()
        ax_tau.legend(lines + lines2, labels + labels2, loc="upper right", fontsize=8)
    else:
        ax_tau.text(0.02, 0.5, "No step model entries found", transform=ax_tau.transAxes)
        ax_tau.set_xlabel("Detected transition index")
        _style_axis(ax_tau, "Transition-Wise Tau Spread", "Tau [s]")

    ax_acc.set_ylim(
        max(np.nanpercentile(acc_from_v_raw, 1), -8.0),
        min(np.nanpercentile(acc_from_v_raw, 99), 8.0),
    )
    ax_vel.set_xlim(t[0], t[-1])
    ax_err.set_xlabel("Time [s]")
    ax_tau.set_xlabel("Detected transition index")
    ax_res.set_xlabel("Time [s]")

    fig.tight_layout(rect=(0, 0, 1, 0.955))
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
