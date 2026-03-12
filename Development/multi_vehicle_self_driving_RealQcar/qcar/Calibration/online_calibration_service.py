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

    velocities = samples[:, 0]
    throttles = samples[:, 1]

    # Quantise throttle to nearest 0.01 to group
    throttle_quant = np.round(throttles, 2)
    unique_levels = np.unique(throttle_quant)

    # Filter out zero/near-zero throttle
    unique_levels = unique_levels[np.abs(unique_levels) > 0.005]
    if len(unique_levels) < 2:
        return {"error": "need at least 2 throttle levels", "levels_found": 0}

    min_dwell_samples = max(5, int(min_dwell_s / sample_dt))

    level_means = []
    level_stds = []
    valid_levels = []

    for level in unique_levels:
        mask = np.abs(throttle_quant - level) < 0.005
        v_at_level = velocities[mask]
        if len(v_at_level) < min_dwell_samples:
            continue
        # Use last 60% as steady-state (skip transient)
        steady_start = int(len(v_at_level) * 0.4)
        v_steady = v_at_level[steady_start:]
        if len(v_steady) < 3:
            continue
        valid_levels.append(float(level))
        level_means.append(float(np.mean(v_steady)))
        level_stds.append(float(np.std(v_steady)))

    if len(valid_levels) < 2:
        return {"error": "not enough valid throttle levels after filtering"}

    t_arr = np.array(valid_levels)
    v_arr = np.array(level_means)
    degree = min(poly_degree, len(t_arr) - 1)
    coeffs = np.polyfit(t_arr, v_arr, degree)

    return {
        "calibration_type": "throttle_velocity",
        "poly_degree": int(degree),
        "poly_coefficients": [float(c) for c in coeffs],
        "throttle_values": valid_levels,
        "velocity_means": level_means,
        "velocity_stds": level_stds,
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

    velocities = samples[:, 0]
    throttles = samples[:, 1]
    n = len(throttles)

    # Detect transitions: find points where throttle changes significantly
    throttle_quant = np.round(throttles, 2)
    transitions = []
    min_step_samples = max(5, int(min_step_duration_s / sample_dt))

    # Segment by constant throttle regions
    segments = []
    seg_start = 0
    for i in range(1, n):
        if abs(throttle_quant[i] - throttle_quant[seg_start]) > 0.005:
            if i - seg_start >= min_step_samples:
                segments.append((seg_start, i, float(throttle_quant[seg_start])))
            seg_start = i
    # Last segment
    if n - seg_start >= min_step_samples:
        segments.append((seg_start, n, float(throttle_quant[seg_start])))

    # Find consecutive segments with different throttle levels
    step_models = []
    for i in range(len(segments) - 1):
        s1_start, s1_end, u_from = segments[i]
        s2_start, s2_end, u_to = segments[i + 1]

        if abs(u_to - u_from) < 0.005:
            continue

        # v0 = mean of last 30% of pre-step segment
        pre_slice = velocities[s1_start:s1_end]
        v0_region = pre_slice[int(len(pre_slice) * 0.7):]
        v0 = float(np.mean(v0_region)) if len(v0_region) > 0 else 0.0

        # vss = mean of last 30% of step segment
        step_slice = velocities[s2_start:s2_end]
        vss_region = step_slice[int(len(step_slice) * 0.7):]
        vss = float(np.mean(vss_region)) if len(vss_region) > 0 else 0.0

        delta_v = vss - v0
        delta_u = u_to - u_from

        if abs(delta_v) < 1e-4:
            continue

        # Estimate tau via 63.2% crossing
        target_v = v0 + 0.632 * delta_v
        step_times = np.arange(len(step_slice)) * sample_dt

        tau = None
        direction = 1.0 if delta_v > 0 else -1.0
        for j in range(len(step_slice) - 1):
            if direction > 0:
                if step_slice[j] <= target_v <= step_slice[j + 1]:
                    frac = (target_v - step_slice[j]) / (step_slice[j + 1] - step_slice[j] + 1e-12)
                    tau = step_times[j] + frac * sample_dt
                    break
            else:
                if step_slice[j] >= target_v >= step_slice[j + 1]:
                    frac = (target_v - step_slice[j]) / (step_slice[j + 1] - step_slice[j] - 1e-12)
                    tau = step_times[j] + frac * sample_dt
                    break

        if tau is None or tau <= 0:
            tau = float(step_times[-1] * 0.5) if len(step_times) > 0 else float("nan")

        K_local = delta_v / delta_u if abs(delta_u) > 1e-6 else float("nan")
        a0 = delta_v / tau if tau > 1e-4 else float("nan")

        if np.isfinite(tau) and tau > 0:
            lead_time = float(-tau * np.log(1.0 - lookahead_ratio))
            t63 = float(tau)
            t90 = float(2.302585 * tau)
            t95 = float(2.995732 * tau)
        else:
            lead_time = t63 = t90 = t95 = float("nan")

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
            "t63_s": t63,
            "t90_s": t90,
            "t95_s": t95,
            "lead_time_s": lead_time,
        })

    if not step_models:
        return {"error": "no valid throttle transitions detected"}

    avg_tau = float(np.nanmean([m["tau_s"] for m in step_models]))
    avg_K = float(np.nanmean([m["K_local"] for m in step_models]))

    return {
        "calibration_type": "throttle_acceleration",
        "model_type": "first_order_lag",
        "avg_tau": avg_tau,
        "avg_K": avg_K,
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

                # Also save raw buffer as CSV
                rows = []
                cols = ["v", "throttle", "steering", "yaw_rate", "ax", "ay", "az"]
                for row in data:
                    rows.append({c: float(row[i]) for i, c in enumerate(cols)})
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
