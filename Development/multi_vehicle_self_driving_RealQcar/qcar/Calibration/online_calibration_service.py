"""
Online Calibration Service — Passive Data Collection & Analysis

Threaded service that buffers calibration samples collected during normal
path-following and runs offline analysis on demand:
  - throttle_velocity   : polyfit  v_ss = f(throttle)
  - steering_curvature  : Ackermann  κ = f(steering)
  - throttle_acceleration : first-order tau estimation

Sample format: 7-element [v, throttle, steering, yaw_rate, ax, ay, az]

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


def _observer_model_summary(
    tau: float,
    velocity_gain: float,
    input_gain: float,
    velocity_rmse: float,
    accel_rmse: float = float("nan"),
    throttle_deadband: float = 0.0,
) -> Dict[str, Any]:
    """Observer-ready representation for config_local_estimators.yaml."""
    accel_formula = (
        "u_eff = sign(throttle) * max(abs(throttle) - throttle_deadband, 0); "
        "a_hat = (velocity_gain * u_eff - velocity) / tau"
    )
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
        "recommended_longitudinal_model": "velocity_lag",
        "velocity_lag_model": {
            "tau": float(tau),
            "velocity_gain": float(velocity_gain),
            "throttle_deadband": float(throttle_deadband),
            "acceleration_formula": accel_formula,
        },
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
        },
        "copy_to": {
            "local_observer_config": (
                "Observer/config_local_estimators.yaml -> local.ekf"
            ),
            "trust_fleet_config": (
                "Observer/TrustbasedDistributedObserver/config_trust_estimator.yaml "
                "-> vehicle.velocity_lag_model"
            ),
        },
        "config_patch": {
            "local": {
                "ekf": {
                    "longitudinal_model": "velocity_lag",
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
                    "longitudinal_model": "velocity_lag",
                    "velocity_lag_model": {
                        "enabled": True,
                        "tau": float(tau),
                        "velocity_gain": float(velocity_gain),
                        "throttle_deadband": float(throttle_deadband),
                    },
                }
            }
        },
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
        if abs(k) > 1e-3 and abs(s) > 0.01:
            L_est = abs(s) / abs(k)
            if 0.05 < L_est < 1.0:
                L_estimates.append(L_est)

    L_eff = float(np.median(L_estimates)) if L_estimates else wheelbase_nom

    return {
        "calibration_type": "steering_curvature",
        "poly_degree": int(degree),
        "poly_coefficients": [float(c) for c in coeffs],
        "effective_wheelbase": round(L_eff, 5),
        "wheelbase_nominal": wheelbase_nom,
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

    return {
        "calibration_type": "throttle_acceleration",
        "model_type": "first_order_lag",
        "avg_tau": avg_tau,
        "avg_K": avg_K,
        "robust_tau": robust_tau,
        "robust_K": robust_K,
        "robust_input_gain_mps2_per_throttle": robust_input_gain,
        "velocity_lag_throttle_deadband": throttle_deadband,
        "observer_model": _observer_model_summary(
            tau=robust_tau,
            velocity_gain=robust_K,
            input_gain=robust_input_gain,
            velocity_rmse=velocity_rmse,
            accel_rmse=accel_rmse,
            throttle_deadband=throttle_deadband,
        ),
        "quality": {
            "velocity_lag_rmse_mps": velocity_rmse,
            "accel_model_vs_dvdt_rmse_mps2": accel_rmse,
            "accel_model_vs_imu_rmse_mps2": _rmse(accel_x, a_model),
            "n_segments_detected": int(len(segments)),
            "n_small_steps_for_robust_fit": int(np.count_nonzero(robust_mask)),
        },
        "lookahead_ratio": lookahead_ratio,
        "step_models": step_models,
        "n_transitions": len(step_models),
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

    SAMPLE_SIZE = 7  # [v, throttle, steering, yaw_rate, ax, ay, az]

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
            else:
                result = {"error": f"Unknown calibration type: {calibration_type}"}

            # Save results
            if "error" not in result:
                rdir = _results_dir(calibration_type)
                ts = time.strftime("%Y%m%d_%H%M%S")
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
                cols = ["v", "throttle", "steering", "yaw_rate", "ax", "ay", "az"]
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
