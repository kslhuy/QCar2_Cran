"""
Online Calibration Service — Passive Data Collection & Analysis

Threaded service that buffers calibration samples collected during normal
path-following and runs offline analysis on demand:
  - throttle_velocity   : polyfit  v_ss = f(throttle)
  - steering_curvature  : Ackermann  κ = f(steering)
  - throttle_acceleration : first-order tau estimation
  - coupled_kinematic : learned coupled bicycle-like motion model

Sample format:
  [v, throttle, steering, yaw_rate, ax, ay, az, x, y, theta, a_ref, v_raw]

Architecture mirrors OnlineSysIDService (On_Track_SysID).
"""

from __future__ import annotations

import csv
import os
import queue
import sys
import threading
import time
from collections import deque
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    import yaml

    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False


# ---------------------------------------------------------------------------
# Path setup for importing analysis functions from calibration scripts
# ---------------------------------------------------------------------------
_CAL_DIR = os.path.dirname(os.path.abspath(__file__))
_QCAR_DIR = os.path.dirname(_CAL_DIR)
for _p in [_CAL_DIR, _QCAR_DIR]:
    if _p not in sys.path:
        sys.path.insert(0, _p)


# ---------------------------------------------------------------------------
# Results directory
# ---------------------------------------------------------------------------
RESULTS_BASE = os.path.join(_CAL_DIR, "results")


CALIBRATION_SAMPLE_COLUMNS = (
    "v",
    "throttle",
    "steering",
    "yaw_rate",
    "ax",
    "ay",
    "az",
    "x",
    "y",
    "theta",
    "a_ref",
    "v_raw",
)
CALIBRATION_SAMPLE_INDEX = {
    name: idx for idx, name in enumerate(CALIBRATION_SAMPLE_COLUMNS)
}

COUPLED_ACCEL_FEATURES = (
    "bias",
    "throttle",
    "abs_throttle",
    "velocity",
    "abs_velocity",
    "steering",
    "steering_sq",
    "throttle_abs_steering",
    "velocity_steering_sq",
)
COUPLED_YAW_FEATURES = (
    "bias",
    "steering",
    "tan_steering",
    "velocity_steering",
    "velocity_tan_steering",
    "throttle_steering",
)


def _results_dir(calibration_type: str) -> str:
    d = os.path.join(RESULTS_BASE, f"online_{calibration_type}")
    os.makedirs(d, exist_ok=True)
    return d


def _save_yaml(data: dict, filename: str, results_dir: str) -> str:
    path = os.path.join(results_dir, filename)
    if not YAML_AVAILABLE:
        return path
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    return path


def _save_csv(
    rows: List[Dict[str, Any]], filename: str, results_dir: str
) -> str:
    if not rows:
        return ""
    path = os.path.join(results_dir, filename)
    fieldnames = list(rows[0].keys())
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return path


# =========================================================================
# Analysis helpers (self-contained, no imports from 01/03/05 scripts)
# =========================================================================


def _rmse(a: np.ndarray, b: np.ndarray) -> float:
    mask = np.isfinite(a) & np.isfinite(b)
    if not np.any(mask):
        return float("nan")
    return float(np.sqrt(np.mean((a[mask] - b[mask]) ** 2)))


def _ema_signal(values: np.ndarray, alpha: float = 0.2) -> np.ndarray:
    """Small causal smoother used only for passive model fitting."""
    values = np.asarray(values, dtype=float).reshape(-1)
    if values.size == 0:
        return values
    alpha = float(np.clip(alpha, 0.0, 1.0))
    out = np.empty_like(values, dtype=float)
    out[0] = values[0]
    for i in range(1, values.size):
        out[i] = alpha * values[i] + (1.0 - alpha) * out[i - 1]
    return out


def _zero_phase_window_smooth(
    values: np.ndarray,
    window_samples: int,
    passes: int = 2,
) -> np.ndarray:
    """
    Offline zero-phase smoother using symmetric FIR convolution.

    Repeated symmetric moving-average passes approximate a Gaussian-like
    low-pass response while keeping the filtered signal centered in time.
    """
    arr = np.asarray(values, dtype=float).reshape(-1)
    if arr.size == 0:
        return arr

    window = int(max(1, window_samples))
    if window <= 1:
        return arr.copy()
    if window % 2 == 0:
        window += 1

    kernel = np.ones(window, dtype=float) / float(window)
    out = arr.copy()
    pad = window // 2
    passes = max(1, int(passes))
    for _ in range(passes):
        padded = np.pad(out, (pad, pad), mode="edge")
        out = np.convolve(padded, kernel, mode="valid")
    return out


def _window_samples(duration_s: float, sample_dt: float, minimum: int = 3) -> int:
    if sample_dt <= 0.0:
        return int(max(minimum, 3))
    n = int(round(float(duration_s) / float(sample_dt)))
    n = max(int(minimum), n)
    if n % 2 == 0:
        n += 1
    return n


def _wrap_to_pi(values: np.ndarray) -> np.ndarray:
    arr = np.asarray(values, dtype=float)
    return (arr + np.pi) % (2.0 * np.pi) - np.pi


def _wrapped_rmse(a: np.ndarray, b: np.ndarray) -> float:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    mask = np.isfinite(a) & np.isfinite(b)
    if not np.any(mask):
        return float("nan")
    err = _wrap_to_pi(a[mask] - b[mask])
    return float(np.sqrt(np.mean(err ** 2)))


def _sample_column(
    samples: np.ndarray, name: str, default: float = float("nan")
) -> np.ndarray:
    idx = CALIBRATION_SAMPLE_INDEX.get(str(name))
    if idx is None or samples.shape[1] <= idx:
        return np.full(samples.shape[0], float(default), dtype=float)
    return np.asarray(samples[:, idx], dtype=float)


def _coupled_feature_matrix(
    velocity: np.ndarray,
    throttle: np.ndarray,
    steering: np.ndarray,
    feature_names: Tuple[str, ...],
) -> np.ndarray:
    v = np.asarray(velocity, dtype=float).reshape(-1)
    u = np.asarray(throttle, dtype=float).reshape(-1)
    delta = np.asarray(steering, dtype=float).reshape(-1)
    delta_clip = np.clip(delta, -0.7, 0.7)
    tan_delta = np.tan(delta_clip)

    feature_map = {
        "bias": np.ones_like(v),
        "throttle": u,
        "abs_throttle": np.abs(u),
        "velocity": v,
        "abs_velocity": np.abs(v),
        "steering": delta,
        "abs_steering": np.abs(delta),
        "steering_sq": delta * delta,
        "tan_steering": tan_delta,
        "velocity_steering": v * delta,
        "velocity_tan_steering": v * tan_delta,
        "throttle_steering": u * delta,
        "throttle_abs_steering": u * np.abs(delta),
        "velocity_steering_sq": v * delta * delta,
    }
    cols = []
    for name in feature_names:
        cols.append(np.asarray(feature_map.get(name, np.zeros_like(v)), dtype=float))
    return np.column_stack(cols) if cols else np.zeros((v.size, 0), dtype=float)


def _linear_fit_with_metrics(
    design: np.ndarray,
    target: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, Dict[str, float]]:
    design = np.asarray(design, dtype=float)
    target = np.asarray(target, dtype=float).reshape(-1)
    if design.ndim != 2 or design.shape[0] != target.size:
        raise ValueError("design matrix and target have incompatible shapes")

    mask = np.all(np.isfinite(design), axis=1) & np.isfinite(target)
    n_valid = int(np.count_nonzero(mask))
    n_features = int(design.shape[1])
    if n_valid < max(n_features + 2, 8):
        raise ValueError(
            f"not enough valid samples for linear fit ({n_valid} for {n_features} features)"
        )

    coeffs, *_ = np.linalg.lstsq(design[mask], target[mask], rcond=None)
    pred = np.full(target.shape, np.nan, dtype=float)
    pred[mask] = design[mask] @ coeffs

    target_valid = target[mask]
    pred_valid = pred[mask]
    rmse = _rmse(target_valid, pred_valid)
    centered = target_valid - float(np.mean(target_valid))
    denom = float(np.sum(centered ** 2))
    if denom > 1e-12:
        r2 = 1.0 - float(np.sum((target_valid - pred_valid) ** 2)) / denom
    else:
        r2 = float("nan")

    metrics = {
        "n_valid_samples": float(n_valid),
        "rmse": float(rmse),
        "r2": float(r2),
    }
    return coeffs.astype(float), pred, metrics


def _simulate_coupled_kinematic(
    throttle: np.ndarray,
    steering: np.ndarray,
    sample_dt: float,
    x0: float,
    y0: float,
    theta0: float,
    v0: float,
    accel_coeffs: np.ndarray,
    yaw_coeffs: np.ndarray,
    accel_features: Tuple[str, ...],
    yaw_features: Tuple[str, ...],
    max_velocity: float,
    max_acceleration: float,
    max_yaw_rate: float,
) -> Dict[str, np.ndarray]:
    throttle = np.asarray(throttle, dtype=float).reshape(-1)
    steering = np.asarray(steering, dtype=float).reshape(-1)
    n = throttle.size

    x = np.zeros(n, dtype=float)
    y = np.zeros(n, dtype=float)
    theta = np.zeros(n, dtype=float)
    v = np.zeros(n, dtype=float)
    a = np.zeros(n, dtype=float)
    yaw = np.zeros(n, dtype=float)
    if n == 0:
        return {"x": x, "y": y, "theta": theta, "v": v, "a": a, "yaw_rate": yaw}

    x[0] = float(x0)
    y[0] = float(y0)
    theta[0] = float(theta0)
    v[0] = float(np.clip(v0, -max_velocity, max_velocity))

    accel_coeffs = np.asarray(accel_coeffs, dtype=float).reshape(-1)
    yaw_coeffs = np.asarray(yaw_coeffs, dtype=float).reshape(-1)

    for i in range(1, n):
        prev_v = float(v[i - 1])
        prev_theta = float(theta[i - 1])
        u = float(throttle[i - 1])
        delta = float(steering[i - 1])

        acc_feat = _coupled_feature_matrix(
            np.asarray([prev_v], dtype=float),
            np.asarray([u], dtype=float),
            np.asarray([delta], dtype=float),
            accel_features,
        )[0]
        yaw_feat = _coupled_feature_matrix(
            np.asarray([prev_v], dtype=float),
            np.asarray([u], dtype=float),
            np.asarray([delta], dtype=float),
            yaw_features,
        )[0]

        a_pred = float(np.clip(acc_feat @ accel_coeffs, -max_acceleration, max_acceleration))
        yaw_pred = float(np.clip(yaw_feat @ yaw_coeffs, -max_yaw_rate, max_yaw_rate))
        v_next = float(np.clip(prev_v + sample_dt * a_pred, -max_velocity, max_velocity))
        v_mid = 0.5 * (prev_v + v_next)
        theta_mid = prev_theta + 0.5 * sample_dt * yaw_pred

        x[i] = x[i - 1] + sample_dt * v_mid * np.cos(theta_mid)
        y[i] = y[i - 1] + sample_dt * v_mid * np.sin(theta_mid)
        theta[i] = prev_theta + sample_dt * yaw_pred
        v[i] = v_next
        a[i] = a_pred
        yaw[i] = yaw_pred

    return {"x": x, "y": y, "theta": theta, "v": v, "a": a, "yaw_rate": yaw}


def _coupled_kinematic_observer_model(
    coupled_model: Dict[str, Any],
    source_yaml: str,
    raw_csv: str,
) -> Dict[str, Any]:
    model_cfg = {
        "enabled": True,
        "acceleration_features": list(coupled_model["acceleration_features"]),
        "acceleration_coefficients": [
            float(v) for v in coupled_model["acceleration_coefficients"]
        ],
        "yaw_rate_features": list(coupled_model["yaw_rate_features"]),
        "yaw_rate_coefficients": [
            float(v) for v in coupled_model["yaw_rate_coefficients"]
        ],
        "reference_filters": dict(coupled_model["reference_filters"]),
        "limits": dict(coupled_model["limits"]),
        "quality": dict(coupled_model["quality"]),
    }
    return {
        "source_calibration": "coupled_kinematic",
        "source_yaml": str(source_yaml),
        "source_raw_csv": str(raw_csv),
        "observer_model": {
            "model_name": "learned_coupled_kinematic",
            "model_type": "coupled_kinematic_bicycle",
            "math": {
                "acceleration": "a_hat = phi_a(v, throttle, steering) dot beta_a",
                "yaw_rate": "w_hat = phi_w(v, throttle, steering) dot beta_w",
                "velocity": "v[k+1] = v[k] + dt * a_hat[k]",
                "heading": "theta[k+1] = theta[k] + dt * w_hat[k]",
                "position": (
                    "x/y use midpoint velocity and heading integration from the "
                    "learned acceleration and yaw-rate model"
                ),
            },
            "recommended_longitudinal_model": "coupled_kinematic",
            "coupled_kinematic_model": model_cfg,
            "quality": dict(coupled_model["quality"]),
            "config_patch": {
                "local": {
                    "ekf": {
                        "longitudinal_model": "coupled_kinematic",
                        "coupled_kinematic_model": model_cfg,
                    },
                    "robust_kalman_net": {
                        "longitudinal_model": "coupled_kinematic",
                        "coupled_kinematic_model": model_cfg,
                    },
                },
                "trust_based_distributed_observer": {
                    "vehicle": {
                        "longitudinal_model": "coupled_kinematic",
                        "coupled_kinematic_model": model_cfg,
                    }
                },
            },
        },
    }


def _constant_command_segments(
    command: np.ndarray,
    quant_step: float,
    min_samples: int,
    zero_deadband: float = 0.0,
) -> List[Tuple[int, int, float]]:
    """
    Split a command trace into contiguous constant-command regions.

    Passive data often revisits the same throttle level many times. Grouping all
    samples by value mixes transients and steady-state data, so analyses should
    work on dwell segments first and aggregate those segment summaries.
    """
    cmd = np.asarray(command, dtype=float).reshape(-1)
    if cmd.size == 0:
        return []

    q = np.round(cmd / quant_step) * quant_step
    segments: List[Tuple[int, int, float]] = []
    start = 0

    for i in range(1, q.size):
        if abs(q[i] - q[start]) > 0.5 * quant_step:
            if i - start >= min_samples and abs(q[start]) > zero_deadband:
                segments.append((start, i, float(q[start])))
            start = i

    if q.size - start >= min_samples and abs(q[start]) > zero_deadband:
        segments.append((start, q.size, float(q[start])))

    return segments


def _velocity_lag_prediction(
    throttles: np.ndarray,
    v0: float,
    sample_dt: float,
    tau: float,
    velocity_gain: float,
    throttle_deadband: float = 0.0,
) -> np.ndarray:
    pred = np.zeros_like(throttles, dtype=float)
    if pred.size == 0:
        return pred
    pred[0] = float(v0)
    tau = max(float(tau), 1e-9)
    deadband = max(float(throttle_deadband), 0.0)
    if deadband > 0.0:
        throttles_eff = np.sign(throttles) * np.maximum(
            np.abs(throttles) - deadband, 0.0
        )
    else:
        throttles_eff = throttles
    for i in range(1, pred.size):
        u = float(throttles_eff[i - 1])
        pred[i] = pred[i - 1] + sample_dt * (
            -(1.0 / tau) * pred[i - 1]
            + (float(velocity_gain) / tau) * u
        )
    return pred


def _build_velocity_lag_lookup_model(
    segments: List[Tuple[int, int, float]],
    velocities: np.ndarray,
    sample_dt: float,
    tail_fraction: float = 0.30,
) -> Dict[str, Any]:
    """Aggregate segment steady states into a piecewise-linear throttle map."""
    if not segments:
        return {}

    steady_by_throttle: Dict[float, List[float]] = {}
    counts_by_throttle: Dict[float, int] = {}
    min_tail_samples = max(3, int(0.20 / max(float(sample_dt), 1e-9)))
    tail_fraction = float(np.clip(tail_fraction, 0.05, 0.50))

    for start, end, throttle in segments:
        seg_len = end - start
        if seg_len <= 0:
            continue
        tail_start = start + int((1.0 - tail_fraction) * seg_len)
        tail_start = min(max(tail_start, start), max(end - 1, start))
        tail = np.asarray(velocities[tail_start:end], dtype=float)
        if tail.size < min_tail_samples:
            continue
        key = float(round(throttle, 4))
        steady_by_throttle.setdefault(key, []).append(float(np.mean(tail)))
        counts_by_throttle[key] = counts_by_throttle.get(key, 0) + 1

    if len(steady_by_throttle) < 3:
        return {}

    throttle_breakpoints = np.asarray(sorted(steady_by_throttle.keys()), dtype=float)
    steady_state_velocity = np.asarray(
        [
            float(np.median(steady_by_throttle[float(u)]))
            for u in throttle_breakpoints
        ],
        dtype=float,
    )
    counts = np.asarray(
        [int(counts_by_throttle[float(u)]) for u in throttle_breakpoints], dtype=int
    )

    if throttle_breakpoints.size == 0:
        return {}

    zero_idx = int(np.argmin(np.abs(throttle_breakpoints)))
    if abs(float(throttle_breakpoints[zero_idx])) <= 0.5 * 0.01:
        steady_state_velocity[zero_idx] = 0.0
    steady_state_velocity = np.maximum.accumulate(steady_state_velocity)

    return {
        "enabled": True,
        "model_name": "piecewise_linear_velocity_lag_lookup",
        "model_type": "first_order_throttle_to_velocity_lookup",
        "interpolation": "linear",
        "tail_fraction": tail_fraction,
        "throttle_breakpoints": [float(v) for v in throttle_breakpoints],
        "steady_state_velocity_breakpoints": [
            float(v) for v in steady_state_velocity
        ],
        "segments_per_breakpoint": [int(v) for v in counts],
    }


def _velocity_lag_lookup_prediction(
    throttles: np.ndarray,
    v0: float,
    sample_dt: float,
    tau: float,
    throttle_breakpoints: np.ndarray,
    steady_state_velocity_breakpoints: np.ndarray,
) -> np.ndarray:
    pred = np.zeros_like(throttles, dtype=float)
    if pred.size == 0:
        return pred
    pred[0] = float(v0)
    tau = max(float(tau), 1e-9)
    throttle_breakpoints = np.asarray(throttle_breakpoints, dtype=float).reshape(-1)
    steady_state_velocity_breakpoints = np.asarray(
        steady_state_velocity_breakpoints, dtype=float
    ).reshape(-1)
    if (
        throttle_breakpoints.size < 2
        or throttle_breakpoints.size != steady_state_velocity_breakpoints.size
    ):
        return pred
    for i in range(1, pred.size):
        v_ss = float(
            np.interp(
                float(throttles[i - 1]),
                throttle_breakpoints,
                steady_state_velocity_breakpoints,
            )
        )
        pred[i] = pred[i - 1] + sample_dt * ((v_ss - pred[i - 1]) / tau)
    return pred


def _optimize_velocity_lag_lookup_tau(
    velocities: np.ndarray,
    throttles: np.ndarray,
    sample_dt: float,
    tau_values: np.ndarray,
    throttle_breakpoints: np.ndarray,
    steady_state_velocity_breakpoints: np.ndarray,
) -> Tuple[float, float]:
    finite_tau = np.asarray(tau_values, dtype=float)
    finite_tau = finite_tau[np.isfinite(finite_tau) & (finite_tau > 0.0)]
    if finite_tau.size == 0:
        return float("nan"), float("nan")

    tau_med = float(np.nanmedian(finite_tau))
    tau_lo = float(np.nanpercentile(finite_tau, 15))
    tau_hi = float(np.nanpercentile(finite_tau, 85))
    if not np.isfinite(tau_lo) or tau_lo <= 0.0:
        tau_lo = max(0.5 * tau_med, 0.05)
    if not np.isfinite(tau_hi) or tau_hi <= tau_lo:
        tau_hi = max(1.5 * tau_med, tau_lo + 0.05)

    tau_grid = np.unique(
        np.concatenate(
            [
                np.linspace(tau_lo, tau_hi, 41, dtype=float),
                np.asarray([tau_med], dtype=float),
            ]
        )
    )

    best_tau = tau_med
    best_rmse = float("inf")
    for tau in tau_grid:
        pred = _velocity_lag_lookup_prediction(
            throttles=throttles,
            v0=float(velocities[0]),
            sample_dt=sample_dt,
            tau=float(tau),
            throttle_breakpoints=throttle_breakpoints,
            steady_state_velocity_breakpoints=steady_state_velocity_breakpoints,
        )
        rmse = _rmse(velocities, pred)
        if np.isfinite(rmse) and rmse < best_rmse:
            best_rmse = float(rmse)
            best_tau = float(tau)

    return float(best_tau), float(best_rmse)


def _observer_model_summary(
    tau: float,
    velocity_gain: float,
    input_gain: float,
    velocity_rmse: float,
    accel_rmse: float = float("nan"),
    throttle_deadband: float = 0.0,
    lookup_model: Optional[Dict[str, Any]] = None,
    lookup_velocity_rmse: float = float("nan"),
    lookup_accel_rmse: float = float("nan"),
    recommended_longitudinal_model: str = "velocity_lag",
) -> Dict[str, Any]:
    """Observer-ready representation for config_local_estimators.yaml."""
    accel_formula = (
        "u_eff = sign(throttle) * max(abs(throttle) - throttle_deadband, 0); "
        "a_hat = (velocity_gain * u_eff - velocity) / tau"
    )
    lookup_model = lookup_model if isinstance(lookup_model, dict) else {}
    config_patch = {
        "local": {
            "ekf": {
                "longitudinal_model": str(recommended_longitudinal_model),
                "use_tachometer_update": True,
                "velocity_lag_model": {
                    "tau": float(tau),
                    "velocity_gain": float(velocity_gain),
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
                "longitudinal_model": str(recommended_longitudinal_model),
                "velocity_lag_model": {
                    "enabled": True,
                    "tau": float(tau),
                    "velocity_gain": float(velocity_gain),
                    "throttle_deadband": float(throttle_deadband),
                },
            }
        }
    }
    if lookup_model:
        lookup_cfg = {
            "enabled": True,
            "tau": float(lookup_model.get("tau", tau)),
            "interpolation": str(lookup_model.get("interpolation", "linear")),
            "throttle_breakpoints": [
                float(v) for v in lookup_model.get("throttle_breakpoints", [])
            ],
            "steady_state_velocity_breakpoints": [
                float(v)
                for v in lookup_model.get("steady_state_velocity_breakpoints", [])
            ],
        }
        config_patch["local"]["ekf"]["velocity_lag_lookup_model"] = lookup_cfg
        config_patch["trust_based_distributed_observer"]["vehicle"][
            "velocity_lag_lookup_model"
        ] = lookup_cfg
    return {
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
            "velocity_gain": float(velocity_gain),
            "throttle_deadband": float(throttle_deadband),
            "acceleration_formula": accel_formula,
        },
        "velocity_lag_lookup_model": lookup_model,
        "accel_lag_model": {
            "tau": float(tau),
            "input_gain": float(input_gain),
            "note": (
                "This no-drag acceleration-lag model is useful for comparison, "
                "but velocity_lag is recommended for the local observer because "
                "constant throttle converges to a steady speed."
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
                "Observer/TrustbasedDistributedObserver/config_trust_estimator.yaml "
                "-> vehicle.velocity_lag_model or vehicle.velocity_lag_lookup_model"
            ),
        },
        "config_patch": config_patch,
    }


def _analyse_throttle_velocity(
    samples: np.ndarray,
    poly_degree: int = 3,
    min_dwell_s: float = 1.0,
    sample_dt: float = 0.02,
) -> Dict[str, Any]:
    """
    Group buffered samples by throttle level, compute mean steady-state
    velocity per level, and fit a polynomial v = f(throttle).

    Samples: Nx7 [v, throttle, steering, yaw_rate, ax, ay, az]
    """
    if samples.shape[0] < 10:
        return {"error": "not enough samples", "n_samples": int(samples.shape[0])}

    min_dwell_samples = max(5, int(min_dwell_s / sample_dt))
    velocities = np.asarray(samples[:, 0], dtype=float)
    throttles = np.asarray(samples[:, 1], dtype=float)
    segments = _constant_command_segments(
        throttles,
        quant_step=0.01,
        min_samples=min_dwell_samples,
        zero_deadband=0.005,
    )

    if len(segments) < 2:
        return {
            "error": "need at least 2 throttle dwell segments",
            "segments_found": len(segments),
        }

    by_level: Dict[float, List[Dict[str, float]]] = {}
    segment_summaries: List[Dict[str, float]] = []

    for start, end, level in segments:
        v_at_level = velocities[start:end]
        steady_start = int(len(v_at_level) * 0.4)
        v_steady = v_at_level[steady_start:]
        if len(v_steady) < 3:
            continue
        summary = {
            "start_index": int(start),
            "end_index": int(end),
            "duration_s": float((end - start) * sample_dt),
            "throttle": float(level),
            "velocity_mean": float(np.mean(v_steady)),
            "velocity_std": float(np.std(v_steady)),
            "n_samples": int(end - start),
        }
        segment_summaries.append(summary)
        by_level.setdefault(float(level), []).append(summary)

    valid_levels = []
    level_means = []
    level_stds = []
    level_counts = []

    for level in sorted(by_level.keys()):
        rows = by_level[level]
        weights = np.asarray([max(r["n_samples"], 1) for r in rows], dtype=float)
        means = np.asarray([r["velocity_mean"] for r in rows], dtype=float)
        stds = np.asarray([r["velocity_std"] for r in rows], dtype=float)
        valid_levels.append(float(level))
        level_means.append(float(np.average(means, weights=weights)))
        level_stds.append(float(np.average(stds, weights=weights)))
        level_counts.append(int(len(rows)))

    if len(valid_levels) < 2:
        return {"error": "not enough valid throttle levels after segment filtering"}

    t_arr = np.array(valid_levels)
    v_arr = np.array(level_means)
    degree = min(poly_degree, len(t_arr) - 1)
    coeffs = np.polyfit(t_arr, v_arr, degree)
    linear_coeffs = np.polyfit(t_arr, v_arr, 1)
    v_poly = np.polyval(coeffs, t_arr)
    v_linear = np.polyval(linear_coeffs, t_arr)

    return {
        "calibration_type": "throttle_velocity",
        "poly_degree": int(degree),
        "poly_coefficients": [float(c) for c in coeffs],
        "linear_fit": {
            "slope_mps_per_throttle": float(linear_coeffs[0]),
            "intercept_mps": float(linear_coeffs[1]),
            "rmse_mps": _rmse(v_arr, v_linear),
        },
        "quality": {
            "poly_rmse_mps": _rmse(v_arr, v_poly),
            "n_segments_used": int(len(segment_summaries)),
            "note": (
                "Passive fit is based on contiguous throttle dwell segments; "
                "large non-monotonicity or high std means the run was not steady enough."
            ),
        },
        "throttle_values": valid_levels,
        "velocity_means": level_means,
        "velocity_stds": level_stds,
        "segments_per_level": level_counts,
        "segment_summaries": segment_summaries,
        "n_total_samples": int(samples.shape[0]),
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
    }


def _analyse_steering_curvature(
    samples: np.ndarray,
    poly_degree: int = 3,
    wheelbase_nom: float = 0.256,
    min_dwell_s: float = 1.0,
    sample_dt: float = 0.02,
) -> Dict[str, Any]:
    """
    Group by steering level, compute κ = yaw_rate / v, fit polynomial
    and estimate effective wheelbase via Ackermann.

    Samples: Nx7 [v, throttle, steering, yaw_rate, ax, ay, az]
    """
    if samples.shape[0] < 10:
        return {"error": "not enough samples"}

    velocities = samples[:, 0]
    steerings = samples[:, 2]
    yaw_rates = samples[:, 3]

    # Quantise steering to nearest 0.02
    steer_quant = np.round(steerings / 0.02) * 0.02
    unique_levels = np.unique(steer_quant)
    # Filter out near-zero steering
    unique_levels = unique_levels[np.abs(unique_levels) > 0.03]

    if len(unique_levels) < 2:
        return {"error": "need at least 2 steering levels"}

    min_dwell_samples = max(5, int(min_dwell_s / sample_dt))

    steer_cmds = []
    curvatures = []
    vel_means = []
    yaw_means = []

    for level in unique_levels:
        mask = np.abs(steer_quant - level) < 0.015
        v_at = velocities[mask]
        yr_at = yaw_rates[mask]

        if len(v_at) < min_dwell_samples:
            continue

        # Use last 60% as steady-state
        ss = int(len(v_at) * 0.4)
        v_ss = v_at[ss:]
        yr_ss = yr_at[ss:]

        v_mean = float(np.mean(v_ss))
        yr_mean = float(np.mean(yr_ss))

        if abs(v_mean) < 0.01:
            continue

        kappa = yr_mean / v_mean

        steer_cmds.append(float(level))
        curvatures.append(float(kappa))
        vel_means.append(v_mean)
        yaw_means.append(yr_mean)

    if len(steer_cmds) < 2:
        return {"error": "not enough valid steering levels after filtering"}

    s_arr = np.array(steer_cmds)
    k_arr = np.array(curvatures)

    degree = min(poly_degree, len(s_arr) - 1)
    coeffs = np.polyfit(s_arr, k_arr, degree)

    # Ackermann wheelbase: L = |δ| / |κ|
    L_estimates = []
    for s, k in zip(steer_cmds, curvatures):
        if abs(k) > 1e-4 and abs(s) > 0.01:
            L_est = abs(np.tan(s) / k)
            if 0.05 < L_est < 1.0:
                L_estimates.append(L_est)

    L_eff = float(np.median(L_estimates)) if L_estimates else wheelbase_nom

    return {
        "calibration_type": "steering_curvature",
        "poly_degree": int(degree),
        "poly_coefficients": [float(c) for c in coeffs],
        "effective_wheelbase": round(L_eff, 5),
        "wheelbase_nominal": wheelbase_nom,
        "ackermann_model": "kappa = tan(delta_eff) / L_eff",
        "steering_values": steer_cmds,
        "curvature_values": curvatures,
        "velocity_means": vel_means,
        "n_total_samples": int(samples.shape[0]),
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
    }


def _analyse_throttle_acceleration(
    samples: np.ndarray,
    sample_dt: float = 0.02,
    min_step_duration_s: float = 0.5,
    lookahead_ratio: float = 0.632,
) -> Dict[str, Any]:
    """
    Detect throttle transitions in the buffered data and estimate
    first-order model parameters (tau, K) for each transition.

    Samples: Nx7 [v, throttle, steering, yaw_rate, ax, ay, az]
    """
    if samples.shape[0] < 20:
        return {"error": "not enough samples"}

    velocities = np.asarray(samples[:, 0], dtype=float)
    throttles = np.asarray(samples[:, 1], dtype=float)
    accel_x = np.asarray(samples[:, 4], dtype=float)
    velocities_smooth = _ema_signal(velocities, alpha=0.2)

    min_step_samples = max(5, int(min_step_duration_s / sample_dt))

    # Segment by constant throttle regions. Include zero-throttle segments so
    # coast/stop transitions can be modeled, but down-weight them later for the
    # observer's nominal small-step gain.
    segments = _constant_command_segments(
        throttles,
        quant_step=0.01,
        min_samples=min_step_samples,
        zero_deadband=-1.0,
    )

    # Find consecutive segments with different throttle levels
    step_models = []
    for i in range(len(segments) - 1):
        s1_start, s1_end, u_from = segments[i]
        s2_start, s2_end, u_to = segments[i + 1]

        if abs(u_to - u_from) < 0.005:
            continue

        # v0 = mean of last 30% of pre-step segment
        pre_slice = velocities[s1_start:s1_end]
        pre_slice_smooth = velocities_smooth[s1_start:s1_end]
        v0_region = pre_slice[int(len(pre_slice) * 0.7):]
        v0 = float(np.mean(v0_region)) if len(v0_region) > 0 else 0.0
        v0_fit_region = pre_slice_smooth[int(len(pre_slice_smooth) * 0.7):]
        v0_fit = float(np.mean(v0_fit_region)) if len(v0_fit_region) > 0 else v0

        # vss = mean of last 30% of step segment
        step_slice = velocities[s2_start:s2_end]
        step_slice_smooth = velocities_smooth[s2_start:s2_end]
        vss_region = step_slice[int(len(step_slice) * 0.7):]
        vss = float(np.mean(vss_region)) if len(vss_region) > 0 else 0.0
        vss_fit_region = step_slice_smooth[int(len(step_slice_smooth) * 0.7):]
        vss_fit = float(np.mean(vss_fit_region)) if len(vss_fit_region) > 0 else vss

        delta_v = vss - v0
        delta_v_fit = vss_fit - v0_fit
        delta_u = u_to - u_from

        if abs(delta_v) < 1e-4:
            continue

        # Estimate tau via 63.2% crossing on the smoothed response.
        target_v = v0_fit + 0.632 * delta_v_fit
        step_times = np.arange(len(step_slice)) * sample_dt

        tau = None
        fit_method = "t63_crossing"
        direction = 1.0 if delta_v_fit > 0 else -1.0
        for j in range(len(step_slice_smooth) - 1):
            if direction > 0:
                if step_slice_smooth[j] <= target_v <= step_slice_smooth[j + 1]:
                    frac = (
                        (target_v - step_slice_smooth[j])
                        / (step_slice_smooth[j + 1] - step_slice_smooth[j] + 1e-12)
                    )
                    tau = step_times[j] + frac * sample_dt
                    break
            else:
                if step_slice_smooth[j] >= target_v >= step_slice_smooth[j + 1]:
                    frac = (
                        (target_v - step_slice_smooth[j])
                        / (step_slice_smooth[j + 1] - step_slice_smooth[j] - 1e-12)
                    )
                    tau = step_times[j] + frac * sample_dt
                    break

        # If enough of the response is usable, log fit is less sensitive to one
        # noisy crossing. Keep t63 as fallback because passive data can be short.
        denom = v0_fit - vss_fit
        ratio = (step_slice_smooth - vss_fit) / (denom + 1e-12)
        mask = np.isfinite(ratio) & (ratio > 0.05) & (ratio < 0.95)
        if np.count_nonzero(mask) >= 5:
            slope, _ = np.polyfit(step_times[mask], np.log(ratio[mask]), 1)
            if slope < -1e-8:
                tau = float(-1.0 / slope)
                fit_method = "log_fit"

        if tau is None or tau <= 0:
            tau = float(step_times[-1] * 0.5) if len(step_times) > 0 else float("nan")
            fit_method = "fallback_half_segment"

        K_local = delta_v / delta_u if abs(delta_u) > 1e-6 else float("nan")
        a0 = delta_v / tau if tau > 1e-4 else float("nan")

        if np.isfinite(tau) and tau > 0:
            lead_time = float(-tau * np.log(1.0 - lookahead_ratio))
            t63 = float(tau)
            t90 = float(2.302585 * tau)
            t95 = float(2.995732 * tau)
        else:
            lead_time = t63 = t90 = t95 = float("nan")

        # Per-step fit quality using the identified velocity-lag response.
        v_step_pred = _velocity_lag_prediction(
            throttles=np.full_like(step_slice, u_to, dtype=float),
            v0=v0,
            sample_dt=sample_dt,
            tau=tau,
            velocity_gain=K_local,
        )
        ax_step = accel_x[s2_start:s2_end]
        a_step_model = (K_local * u_to - step_slice) / max(tau, 1e-9)
        a_step_from_v = np.gradient(step_slice_smooth, sample_dt)

        step_models.append({
            "u_from": u_from,
            "u_to": u_to,
            "delta_u": float(delta_u),
            "v0": v0,
            "vss": vss,
            "delta_v": float(delta_v),
            "tau_s": float(tau),
            "K_local": float(K_local),
            "a0_mps2": float(a0),
            "input_gain_mps2_per_throttle": (
                float(K_local / tau) if tau > 1e-9 else float("nan")
            ),
            "velocity_rmse_mps": _rmse(step_slice, v_step_pred),
            "accel_model_vs_dvdt_rmse_mps2": _rmse(a_step_from_v, a_step_model),
            "accel_model_vs_imu_rmse_mps2": _rmse(ax_step, a_step_model),
            "t63_s": t63,
            "t90_s": t90,
            "t95_s": t95,
            "lead_time_s": lead_time,
            "fit_method": fit_method,
        })

    if not step_models:
        return {"error": "no valid throttle transitions detected"}

    tau_values = np.asarray([m["tau_s"] for m in step_models], dtype=float)
    k_values = np.asarray([m["K_local"] for m in step_models], dtype=float)
    gain_values = np.asarray(
        [m["input_gain_mps2_per_throttle"] for m in step_models], dtype=float
    )
    delta_u_values = np.asarray([abs(m["delta_u"]) for m in step_models], dtype=float)

    finite = np.isfinite(tau_values) & np.isfinite(k_values) & (tau_values > 0.0)
    small_step = finite & (delta_u_values >= 0.015) & (delta_u_values <= 0.09)
    robust_mask = small_step if np.count_nonzero(small_step) >= 3 else finite

    avg_tau = float(np.nanmean(tau_values[finite]))
    avg_K = float(np.nanmean(k_values[finite]))
    robust_tau = float(np.nanmedian(tau_values[robust_mask]))
    robust_K = float(np.nanmedian(k_values[robust_mask]))
    robust_input_gain = float(np.nanmedian(gain_values[robust_mask]))

    # Estimate a small symmetric throttle deadband from steady endpoints:
    # v_ss ~= K * sign(u) * max(abs(u)-u_dead, 0).
    deadband_candidates = []
    for step in step_models:
        try:
            u_to = float(step["u_to"])
            vss = float(step["vss"])
        except (KeyError, TypeError, ValueError):
            continue
        if abs(u_to) < 0.04 or not np.isfinite(vss):
            continue
        candidate = abs(u_to) - abs(vss) / max(abs(robust_K), 1e-9)
        if 0.0 <= candidate <= 0.03:
            deadband_candidates.append(candidate)
    throttle_deadband = (
        float(np.nanmedian(deadband_candidates)) if deadband_candidates else 0.0
    )

    v_pred = _velocity_lag_prediction(
        throttles=throttles,
        v0=float(velocities[0]),
        sample_dt=sample_dt,
        tau=robust_tau,
        velocity_gain=robust_K,
        throttle_deadband=throttle_deadband,
    )
    throttles_eff = np.sign(throttles) * np.maximum(
        np.abs(throttles) - throttle_deadband, 0.0
    )
    a_model = (robust_K * throttles_eff - velocities) / max(robust_tau, 1e-9)
    a_from_v = np.gradient(velocities_smooth, sample_dt)
    velocity_rmse = _rmse(velocities, v_pred)
    accel_rmse = _rmse(a_from_v, a_model)
    lookup_model = _build_velocity_lag_lookup_model(
        segments=segments,
        velocities=velocities,
        sample_dt=sample_dt,
    )
    lookup_tau = float("nan")
    lookup_v_pred = np.zeros_like(velocities, dtype=float)
    lookup_a_model = np.zeros_like(velocities, dtype=float)
    lookup_velocity_rmse = float("nan")
    lookup_accel_rmse = float("nan")
    recommended_longitudinal_model = "velocity_lag"
    if lookup_model:
        throttle_breakpoints = np.asarray(
            lookup_model.get("throttle_breakpoints", []), dtype=float
        )
        steady_state_velocity_breakpoints = np.asarray(
            lookup_model.get("steady_state_velocity_breakpoints", []), dtype=float
        )
        lookup_tau, lookup_velocity_rmse = _optimize_velocity_lag_lookup_tau(
            velocities=velocities,
            throttles=throttles,
            sample_dt=sample_dt,
            tau_values=tau_values[finite],
            throttle_breakpoints=throttle_breakpoints,
            steady_state_velocity_breakpoints=steady_state_velocity_breakpoints,
        )
        if np.isfinite(lookup_tau) and lookup_tau > 0.0:
            lookup_v_pred = _velocity_lag_lookup_prediction(
                throttles=throttles,
                v0=float(velocities[0]),
                sample_dt=sample_dt,
                tau=lookup_tau,
                throttle_breakpoints=throttle_breakpoints,
                steady_state_velocity_breakpoints=steady_state_velocity_breakpoints,
            )
            lookup_v_ss = np.interp(
                throttles,
                throttle_breakpoints,
                steady_state_velocity_breakpoints,
            )
            lookup_a_model = (lookup_v_ss - velocities) / max(lookup_tau, 1e-9)
            lookup_accel_rmse = _rmse(a_from_v, lookup_a_model)
            lookup_model["tau"] = float(lookup_tau)
            lookup_model["velocity_rmse_mps"] = float(lookup_velocity_rmse)
            lookup_model["acceleration_rmse_mps2"] = float(lookup_accel_rmse)
            if (
                np.isfinite(lookup_velocity_rmse)
                and (
                    not np.isfinite(velocity_rmse)
                    or lookup_velocity_rmse < 0.95 * velocity_rmse
                )
            ):
                recommended_longitudinal_model = "velocity_lag_lookup"

    return {
        "calibration_type": "throttle_acceleration",
        "model_type": "first_order_lag",
        "avg_tau": avg_tau,
        "avg_K": avg_K,
        "robust_tau": robust_tau,
        "robust_K": robust_K,
        "robust_input_gain_mps2_per_throttle": robust_input_gain,
        "velocity_lag_throttle_deadband": throttle_deadband,
        "recommended_longitudinal_model": recommended_longitudinal_model,
        "velocity_lag_lookup_model": lookup_model,
        "observer_model": _observer_model_summary(
            tau=robust_tau,
            velocity_gain=robust_K,
            input_gain=robust_input_gain,
            velocity_rmse=velocity_rmse,
            accel_rmse=accel_rmse,
            throttle_deadband=throttle_deadband,
            lookup_model=lookup_model,
            lookup_velocity_rmse=lookup_velocity_rmse,
            lookup_accel_rmse=lookup_accel_rmse,
            recommended_longitudinal_model=recommended_longitudinal_model,
        ),
        "quality": {
            "velocity_lag_rmse_mps": velocity_rmse,
            "accel_model_vs_dvdt_rmse_mps2": accel_rmse,
            "accel_model_vs_imu_rmse_mps2": _rmse(accel_x, a_model),
            "velocity_lag_lookup_rmse_mps": float(lookup_velocity_rmse),
            "accel_lookup_vs_dvdt_rmse_mps2": float(lookup_accel_rmse),
            "n_segments_detected": int(len(segments)),
            "n_small_steps_for_robust_fit": int(np.count_nonzero(robust_mask)),
        },
        "lookahead_ratio": lookahead_ratio,
        "step_models": step_models,
        "n_transitions": len(step_models),
        "n_total_samples": int(samples.shape[0]),
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
    }


def _analyse_coupled_kinematic(
    samples: np.ndarray,
    sample_dt: float = 0.02,
    velocity_filter_window_s: float = 0.18,
    acceleration_filter_window_s: float = 0.14,
    yaw_rate_filter_window_s: float = 0.16,
    pose_filter_window_s: float = 0.30,
    max_velocity: float = 3.0,
    max_acceleration: float = 4.0,
    max_yaw_rate: float = 8.0,
) -> Dict[str, Any]:
    """
    Fit a data-driven coupled kinematic model using filtered online references.

    Expected sample columns:
      [v, throttle, steering, yaw_rate, ax, ay, az, x, y, theta, a_ref, v_raw]
    """
    if samples.shape[0] < 40:
        return {"error": "not enough samples"}
    if samples.shape[1] < len(CALIBRATION_SAMPLE_COLUMNS):
        return {
            "error": (
                "sample stream does not contain pose/reference channels; "
                "restart the vehicle-side client with the updated calibration sample format"
            )
        }

    velocity_ref_meas = _sample_column(samples, "v")
    velocity_raw = _sample_column(samples, "v_raw", default=float("nan"))
    throttle = _sample_column(samples, "throttle")
    steering = _sample_column(samples, "steering")
    yaw_rate = _sample_column(samples, "yaw_rate")
    accel_x = _sample_column(samples, "ax")
    x_meas = _sample_column(samples, "x")
    y_meas = _sample_column(samples, "y")
    theta_meas = _sample_column(samples, "theta")
    accel_ref_meas = _sample_column(samples, "a_ref", default=0.0)

    use_velocity_raw = bool(
        np.any(np.isfinite(velocity_raw)) and np.nanstd(velocity_raw) > 1e-6
    )
    velocity_source = velocity_raw if use_velocity_raw else velocity_ref_meas

    v_window = _window_samples(velocity_filter_window_s, sample_dt, minimum=5)
    a_window = _window_samples(acceleration_filter_window_s, sample_dt, minimum=5)
    yaw_window = _window_samples(yaw_rate_filter_window_s, sample_dt, minimum=5)
    pose_window = _window_samples(pose_filter_window_s, sample_dt, minimum=7)

    velocity_ref = _zero_phase_window_smooth(velocity_source, window_samples=v_window)
    yaw_rate_ref = _zero_phase_window_smooth(yaw_rate, window_samples=yaw_window)
    x_ref = _zero_phase_window_smooth(x_meas, window_samples=pose_window)
    y_ref = _zero_phase_window_smooth(y_meas, window_samples=pose_window)
    theta_ref = _zero_phase_window_smooth(
        np.unwrap(theta_meas),
        window_samples=pose_window,
    )
    accel_from_velocity = np.gradient(velocity_ref, sample_dt)
    accel_from_velocity = _zero_phase_window_smooth(
        accel_from_velocity,
        window_samples=a_window,
    )
    accel_state_ref = _zero_phase_window_smooth(
        accel_ref_meas,
        window_samples=a_window,
    )
    accel_imu_ref = _zero_phase_window_smooth(
        accel_x,
        window_samples=a_window,
    )

    accel_state_energy = float(np.nanmedian(np.abs(accel_state_ref)))
    velocity_ref_label = "raw tach velocity" if use_velocity_raw else "estimator velocity"
    if np.isfinite(accel_state_energy) and accel_state_energy > 1e-3:
        acceleration_ref = 0.35 * accel_state_ref + 0.65 * accel_from_velocity
        acceleration_reference_name = (
            f"0.65 * d/dt(filtered {velocity_ref_label}) + 0.35 * filtered observer acceleration"
        )
    else:
        acceleration_ref = accel_from_velocity
        acceleration_reference_name = f"d/dt(filtered {velocity_ref_label})"

    accel_design = _coupled_feature_matrix(
        velocity_ref,
        throttle,
        steering,
        COUPLED_ACCEL_FEATURES,
    )
    yaw_design = _coupled_feature_matrix(
        velocity_ref,
        throttle,
        steering,
        COUPLED_YAW_FEATURES,
    )

    try:
        accel_coeffs, _accel_fit, accel_fit_metrics = _linear_fit_with_metrics(
            accel_design,
            acceleration_ref,
        )
        yaw_coeffs, _yaw_fit, yaw_fit_metrics = _linear_fit_with_metrics(
            yaw_design,
            yaw_rate_ref,
        )
    except ValueError as exc:
        return {"error": str(exc)}

    sim = _simulate_coupled_kinematic(
        throttle=throttle,
        steering=steering,
        sample_dt=sample_dt,
        x0=float(x_ref[0]),
        y0=float(y_ref[0]),
        theta0=float(theta_ref[0]),
        v0=float(velocity_ref[0]),
        accel_coeffs=accel_coeffs,
        yaw_coeffs=yaw_coeffs,
        accel_features=COUPLED_ACCEL_FEATURES,
        yaw_features=COUPLED_YAW_FEATURES,
        max_velocity=max_velocity,
        max_acceleration=max_acceleration,
        max_yaw_rate=max_yaw_rate,
    )

    quality = {
        "velocity_rmse_mps": _rmse(velocity_ref, sim["v"]),
        "acceleration_rmse_mps2": _rmse(acceleration_ref, sim["a"]),
        "yaw_rate_rmse_radps": _rmse(yaw_rate_ref, sim["yaw_rate"]),
        "theta_rmse_rad": _wrapped_rmse(theta_ref, sim["theta"]),
        "x_rmse_m": _rmse(x_ref, sim["x"]),
        "y_rmse_m": _rmse(y_ref, sim["y"]),
        "acceleration_fit_rmse_mps2": float(accel_fit_metrics["rmse"]),
        "acceleration_fit_r2": float(accel_fit_metrics["r2"]),
        "yaw_fit_rmse_radps": float(yaw_fit_metrics["rmse"]),
        "yaw_fit_r2": float(yaw_fit_metrics["r2"]),
        "accel_ref_vs_imu_rmse_mps2": _rmse(acceleration_ref, accel_imu_ref),
        "n_total_samples": int(samples.shape[0]),
    }
    reference_filters = {
        "velocity_source": "v_raw" if use_velocity_raw else "v",
        "velocity_filter": "offline_zero_phase_boxcar_x2",
        "velocity_filter_window_samples": int(v_window),
        "acceleration_filter": "offline_zero_phase_boxcar_x2",
        "acceleration_filter_window_samples": int(a_window),
        "yaw_rate_filter": "offline_zero_phase_boxcar_x2",
        "yaw_rate_filter_window_samples": int(yaw_window),
        "pose_filter": "offline_zero_phase_boxcar_x2",
        "pose_filter_window_samples": int(pose_window),
        "acceleration_reference": acceleration_reference_name,
    }
    coupled_model = {
        "enabled": True,
        "acceleration_features": list(COUPLED_ACCEL_FEATURES),
        "acceleration_coefficients": [float(v) for v in accel_coeffs],
        "yaw_rate_features": list(COUPLED_YAW_FEATURES),
        "yaw_rate_coefficients": [float(v) for v in yaw_coeffs],
        "reference_filters": reference_filters,
        "limits": {
            "max_velocity": float(max_velocity),
            "max_acceleration": float(max_acceleration),
            "max_yaw_rate": float(max_yaw_rate),
        },
        "quality": dict(quality),
    }
    observer_model = _coupled_kinematic_observer_model(
        coupled_model=coupled_model,
        source_yaml="",
        raw_csv="",
    )["observer_model"]

    return {
        "calibration_type": "coupled_kinematic",
        "model_type": "coupled_kinematic_bicycle",
        "recommended_longitudinal_model": "coupled_kinematic",
        "reference_filters": reference_filters,
        "reference_statistics": {
            "velocity_mean_mps": float(np.nanmean(velocity_ref)),
            "velocity_std_mps": float(np.nanstd(velocity_ref)),
            "velocity_raw_mean_mps": float(np.nanmean(velocity_source)),
            "velocity_raw_std_mps": float(np.nanstd(velocity_source)),
            "steering_std_rad": float(np.nanstd(steering)),
            "throttle_std": float(np.nanstd(throttle)),
        },
        "acceleration_model": {
            "feature_names": list(COUPLED_ACCEL_FEATURES),
            "coefficients": [float(v) for v in accel_coeffs],
            "fit_rmse_mps2": float(accel_fit_metrics["rmse"]),
            "fit_r2": float(accel_fit_metrics["r2"]),
        },
        "yaw_rate_model": {
            "feature_names": list(COUPLED_YAW_FEATURES),
            "coefficients": [float(v) for v in yaw_coeffs],
            "fit_rmse_radps": float(yaw_fit_metrics["rmse"]),
            "fit_r2": float(yaw_fit_metrics["r2"]),
        },
        "simulated_quality": quality,
        "coupled_kinematic_model": coupled_model,
        "observer_model": observer_model,
        "n_total_samples": int(samples.shape[0]),
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
    }


# =========================================================================
# Service class
# =========================================================================


class OnlineCalibrationService:
    """
    Threaded online calibration service.

    Public API:
    - start(collect=False)
    - shutdown()
    - start_collection() / stop_collection()
    - clear_buffer()
    - submit_sample(sample)
    - trigger_analyse(calibration_type)
    - get_status()
    """

    SAMPLE_SIZE = len(CALIBRATION_SAMPLE_COLUMNS)

    def __init__(
        self,
        logger=None,
        vehicle_id: int = 0,
        sample_dt: float = 0.02,
        buffer_size: int = 20000,
        sample_queue_size: int = 4096,
        poly_degree: int = 3,
        wheelbase_nom: float = 0.256,
    ):
        self.logger = logger
        self.vehicle_id = int(vehicle_id)
        self.sample_dt = float(sample_dt)
        self.buffer_size = int(buffer_size)
        self.poly_degree = int(poly_degree)
        self.wheelbase_nom = float(wheelbase_nom)

        # Worker communication
        self._sample_queue: queue.Queue = queue.Queue(maxsize=sample_queue_size)
        self._command_queue: queue.Queue = queue.Queue(maxsize=64)
        self._sample_buffer: deque = deque(maxlen=self.buffer_size)

        # State
        self._running = False
        self._collecting = False
        self._analysing = False
        self._worker: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._buffer_lock = threading.RLock()
        self._state_lock = threading.RLock()

        # Counters
        self._samples_received = 0
        self._samples_accepted = 0
        self._samples_dropped = 0
        self._last_analysis_time = 0.0
        self._last_analysis_error = ""
        self._last_analysis_result: Dict[str, Any] = {}

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start(self, collect: bool = False) -> bool:
        with self._state_lock:
            if self._running:
                self._collecting = bool(collect) or self._collecting
                return True

            self._stop_event.clear()
            self._collecting = bool(collect)
            self._worker = threading.Thread(
                target=self._worker_loop,
                name=f"online_cal_worker_v{self.vehicle_id}",
                daemon=True,
            )
            self._worker.start()
            self._running = True

        self._log_info(
            f"[OnlineCal] Worker started (collecting={self._collecting})"
        )
        return True

    def shutdown(self) -> None:
        with self._state_lock:
            if not self._running:
                return
            self._running = False
            self._collecting = False
            self._stop_event.set()
            try:
                self._command_queue.put_nowait({"type": "shutdown"})
            except queue.Full:
                pass
            worker = self._worker

        if worker and worker.is_alive():
            worker.join(timeout=5.0)
        self._log_info("[OnlineCal] Worker stopped")

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------
    def start_collection(self) -> bool:
        with self._state_lock:
            if not self._running:
                self.start(collect=True)
            self._collecting = True
        return True

    def stop_collection(self) -> bool:
        with self._state_lock:
            self._collecting = False
        return True

    def clear_buffer(self) -> None:
        with self._buffer_lock:
            self._sample_buffer.clear()
        with self._state_lock:
            self._samples_received = 0
            self._samples_accepted = 0
            self._samples_dropped = 0

    def trigger_analyse(
        self, calibration_type: str, options: Optional[Dict[str, Any]] = None
    ) -> Tuple[bool, str]:
        with self._state_lock:
            if not self._running:
                return False, "Service is not running"
            if self._analysing:
                return False, "Analysis already in progress"

        cmd = {
            "type": "analyse",
            "calibration_type": str(calibration_type),
            "options": options or {},
        }
        try:
            self._command_queue.put_nowait(cmd)
            return True, f"Analysis request queued: {calibration_type}"
        except queue.Full:
            return False, "Command queue is full"

    # ------------------------------------------------------------------
    # Data path
    # ------------------------------------------------------------------
    def submit_sample(
        self, sample: np.ndarray, timestamp: Optional[float] = None
    ) -> bool:
        arr = np.asarray(sample, dtype=np.float32).reshape(-1)
        if arr.size != self.SAMPLE_SIZE or not np.all(np.isfinite(arr)):
            return False

        with self._state_lock:
            if not self._running or not self._collecting:
                return False
            self._samples_received += 1

        item = (float(timestamp) if timestamp is not None else time.time(), arr.copy())
        try:
            self._sample_queue.put_nowait(item)
            return True
        except queue.Full:
            try:
                self._sample_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._sample_queue.put_nowait(item)
                with self._state_lock:
                    self._samples_dropped += 1
                return True
            except queue.Full:
                with self._state_lock:
                    self._samples_dropped += 1
                return False

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------
    def get_status(self) -> Dict[str, Any]:
        with self._state_lock:
            running = self._running
            collecting = self._collecting
            analysing = self._analysing
            received = self._samples_received
            accepted = self._samples_accepted
            dropped = self._samples_dropped
            last_time = self._last_analysis_time
            last_error = self._last_analysis_error
            last_result = dict(self._last_analysis_result)

        with self._buffer_lock:
            buffered = len(self._sample_buffer)

        return {
            "worker_running": running,
            "collecting": collecting,
            "analysing": analysing,
            "buffered_samples": buffered,
            "buffer_capacity": self.buffer_size,
            "samples_received": received,
            "samples_accepted": accepted,
            "samples_dropped": dropped,
            "last_analysis_time": last_time,
            "last_analysis_error": last_error,
            "last_analysis_result": last_result,
        }

    # ------------------------------------------------------------------
    # Internal worker
    # ------------------------------------------------------------------
    def _worker_loop(self) -> None:
        while not self._stop_event.is_set():
            self._process_one_command(timeout=0.02)
            self._drain_sample_queue(max_batch=256)

    def _process_one_command(self, timeout: float = 0.0) -> None:
        try:
            cmd = self._command_queue.get(timeout=timeout)
        except queue.Empty:
            return

        cmd_type = cmd.get("type")
        if cmd_type == "shutdown":
            return
        if cmd_type == "analyse":
            cal_type = cmd.get("calibration_type", "throttle_velocity")
            options = cmd.get("options", {})
            self._run_analysis(cal_type, options)

    def _drain_sample_queue(self, max_batch: int = 256) -> None:
        for _ in range(max_batch):
            try:
                _ts, sample = self._sample_queue.get_nowait()
            except queue.Empty:
                return

            with self._buffer_lock:
                self._sample_buffer.append(sample.copy())

            with self._state_lock:
                self._samples_accepted += 1

    def _run_analysis(
        self, calibration_type: str, options: Dict[str, Any]
    ) -> None:
        with self._state_lock:
            if self._analysing:
                return
            self._analysing = True
            self._last_analysis_error = ""

        with self._buffer_lock:
            if len(self._sample_buffer) == 0:
                data = np.zeros((0, self.SAMPLE_SIZE), dtype=np.float32)
            else:
                data = np.asarray(self._sample_buffer, dtype=np.float32)

        if data.shape[0] < 10:
            msg = f"Not enough samples ({data.shape[0]})"
            with self._state_lock:
                self._last_analysis_error = msg
                self._analysing = False
            self._log_warn(f"[OnlineCal] {msg}")
            return

        self._log_info(
            f"[OnlineCal] Running {calibration_type} analysis on {data.shape[0]} samples"
        )

        try:
            if calibration_type == "throttle_velocity":
                result = _analyse_throttle_velocity(
                    data,
                    poly_degree=options.get("poly_degree", self.poly_degree),
                    sample_dt=self.sample_dt,
                )
            elif calibration_type == "steering_curvature":
                result = _analyse_steering_curvature(
                    data,
                    poly_degree=options.get("poly_degree", self.poly_degree),
                    wheelbase_nom=options.get("wheelbase_nom", self.wheelbase_nom),
                    sample_dt=self.sample_dt,
                )
            elif calibration_type == "throttle_acceleration":
                result = _analyse_throttle_acceleration(
                    data,
                    sample_dt=self.sample_dt,
                    lookahead_ratio=options.get("lookahead_ratio", 0.632),
                )
            elif calibration_type == "coupled_kinematic":
                result = _analyse_coupled_kinematic(
                    data,
                    sample_dt=self.sample_dt,
                    velocity_filter_window_s=float(
                        options.get("velocity_filter_window_s", 0.18)
                    ),
                    acceleration_filter_window_s=float(
                        options.get("acceleration_filter_window_s", 0.14)
                    ),
                    yaw_rate_filter_window_s=float(
                        options.get("yaw_rate_filter_window_s", 0.16)
                    ),
                    pose_filter_window_s=float(
                        options.get("pose_filter_window_s", 0.30)
                    ),
                    max_velocity=float(options.get("max_velocity", 3.0)),
                    max_acceleration=float(options.get("max_acceleration", 4.0)),
                    max_yaw_rate=float(options.get("max_yaw_rate", 8.0)),
                )
            else:
                result = {"error": f"Unknown calibration type: {calibration_type}"}

            # Save results
            if "error" not in result:
                rdir = _results_dir(calibration_type)
                ts = time.strftime("%Y%m%d_%H%M%S")
                latest_yaml_path = os.path.join(rdir, f"{calibration_type}_latest.yaml")
                raw_csv_path = os.path.join(rdir, f"raw_samples_{ts}.csv")
                if calibration_type == "coupled_kinematic":
                    result = dict(result)
                    result["observer_model"] = _coupled_kinematic_observer_model(
                        coupled_model=result["coupled_kinematic_model"],
                        source_yaml=latest_yaml_path,
                        raw_csv=raw_csv_path,
                    )["observer_model"]
                _save_yaml(result, f"{calibration_type}_{ts}.yaml", rdir)
                _save_yaml(result, f"{calibration_type}_latest.yaml", rdir)
                if isinstance(result.get("observer_model"), dict):
                    observer_model = {
                        "source_calibration": calibration_type,
                        "timestamp": result.get("timestamp", ""),
                        "observer_model": result["observer_model"],
                    }
                    _save_yaml(
                        observer_model,
                        f"{calibration_type}_observer_model_{ts}.yaml",
                        rdir,
                    )
                    _save_yaml(
                        observer_model,
                        f"{calibration_type}_observer_model_latest.yaml",
                        rdir,
                    )

                # Also save raw buffer as CSV
                rows = []
                cols = list(CALIBRATION_SAMPLE_COLUMNS)
                for sample_idx, row in enumerate(data):
                    sample_row = {
                        "sample_index": int(sample_idx),
                        "time_s": float(sample_idx * self.sample_dt),
                    }
                    sample_row.update(
                        {c: float(row[i]) for i, c in enumerate(cols)}
                    )
                    rows.append(sample_row)
                _save_csv(rows, f"raw_samples_{ts}.csv", rdir)

                self._log_info(
                    f"[OnlineCal] {calibration_type} analysis saved to {rdir}"
                )
            else:
                self._log_warn(f"[OnlineCal] Analysis error: {result['error']}")

            with self._state_lock:
                self._last_analysis_time = time.time()
                self._last_analysis_result = result
                self._last_analysis_error = result.get("error", "")
                self._analysing = False

        except Exception as exc:
            with self._state_lock:
                self._last_analysis_error = str(exc)
                self._analysing = False
            self._log_error(f"[OnlineCal] Analysis failed", exc)

    # ------------------------------------------------------------------
    # Logging helpers
    # ------------------------------------------------------------------
    def _log_info(self, msg: str) -> None:
        if self.logger is None:
            print(msg)
            return
        if hasattr(self.logger, "logger"):
            self.logger.logger.info(msg)
        else:
            print(msg)

    def _log_warn(self, msg: str) -> None:
        if self.logger is None:
            print(msg)
            return
        if hasattr(self.logger, "log_warning"):
            self.logger.log_warning(msg)
        elif hasattr(self.logger, "logger"):
            self.logger.logger.warning(msg)
        else:
            print(msg)

    def _log_error(self, msg: str, exc: Optional[Exception] = None) -> None:
        if self.logger is None:
            print(f"{msg}: {exc}" if exc else msg)
            return
        if hasattr(self.logger, "log_error"):
            self.logger.log_error(msg, exc)
        elif hasattr(self.logger, "logger"):
            self.logger.logger.error(f"{msg}: {exc}" if exc else msg)
        else:
            print(f"{msg}: {exc}" if exc else msg)
