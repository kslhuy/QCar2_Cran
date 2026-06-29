#!/usr/bin/env python3
"""
05_throttle_acceleration_calibration.py
=======================================
QCar Calibration - Throttle Step -> Acceleration Dynamics Lookup
----------------------------------------------------------------
Runs throttle step transitions and identifies a first-order lag model per
transition:

    tau * dv/dt + v = K * u

For each step (u_from -> u_to), the script estimates:
  - tau (time constant)
  - local gain K_local = Delta v_ss / Delta u
  - initial acceleration a0 = Delta v_ss / tau
  - rise timing (t63, t90, t95)
  - lead-time suggestion for preview scheduling

Default transitions are adjacent up/down steps up to throttle 0.3:
  0.0->0.1, 0.1->0.2, 0.2->0.3, 0.3->0.2, 0.2->0.1, 0.1->0.0

Supports THREE modes:
  --sim      : pure Python first-order motor model (no QLabs, no hardware)
  --qlabs    : QLabs virtual QCar via readRobots() (run initPlatoon.py first)
  (default)  : physical QCar hardware

Outputs:
  results/throttle_accel_step_raw_<tag>.csv
  results/throttle_accel_lookup_<tag>.csv
  results/throttle_accel_model_<tag>.yaml
"""

import argparse
import csv
import os
import sys
import time
from typing import Dict, List, Tuple

import yaml

import numpy as np

_CAL_DIR = os.path.dirname(os.path.abspath(__file__))
_QCAR_DIR = os.path.dirname(_CAL_DIR)
for _p in [_CAL_DIR, _QCAR_DIR]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

RESULTS_DIR = os.path.join(_CAL_DIR, "results", "05_throttle_acceleration_calibration")
os.makedirs(RESULTS_DIR, exist_ok=True)


class CSVLogger:
    """Simple CSV logger to results/ with ASCII-safe terminal output."""

    def __init__(
        self, filename: str, columns: List[str], results_dir: str = RESULTS_DIR
    ):
        self.path = os.path.join(results_dir, filename)
        self.columns = columns
        self._file = None
        self._writer = None

    def __enter__(self):
        self._file = open(self.path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.columns)
        return self

    def write(self, *values):
        self._writer.writerow(
            [f"{v:.6f}" if isinstance(v, float) else v for v in values]
        )

    def __exit__(self, *args):
        if self._file:
            self._file.close()
        print(f"[CSVLogger] Saved -> {self.path}")


class FirstOrderMotorSim:
    """Lightweight first-order motor simulation: tau * dv/dt + v = K * u."""

    def __init__(self, tau: float = 0.8, K: float = 5.0, noise_std: float = 0.01):
        self.tau = tau
        self.K = K
        self.noise_std = noise_std
        self.v = 0.0

    def step(self, u: float, dt: float) -> float:
        dvdt = (self.K * u - self.v) / self.tau
        self.v += dvdt * dt
        return float(self.v + np.random.randn() * self.noise_std)

    def reset(self):
        self.v = 0.0


def save_yaml(data: dict, filename: str, results_dir: str = RESULTS_DIR) -> str:
    path = os.path.join(results_dir, filename)
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    print(f"[YAML] Saved -> {path}")
    return path


def print_section(title: str):
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)


# ---------------------------------------------------------------------------
# CLI arguments
# ---------------------------------------------------------------------------
parser = argparse.ArgumentParser(
    description="QCar throttle-step to acceleration dynamics calibration"
)
parser.add_argument(
    "--sim",
    action="store_true",
    help="Pure Python first-order model (no QLabs, no hardware)",
)
parser.add_argument(
    "--qlabs",
    action="store_true",
    help="Connect to QLabs virtual QCar (run initPlatoon.py first)",
)
parser.add_argument(
    "--actor",
    type=str,
    default="QC2_0",
    help="QLabs actor name, e.g. QC2_0 (used with --qlabs)",
)
parser.add_argument(
    "--throttle_levels",
    type=str,
    default="0.0,0.1,0.2,0.3",
    help="Comma-separated throttle levels used to build transitions",
)
parser.add_argument(
    "--max_throttle",
    type=float,
    default=0.3,
    help="Maximum throttle value kept from --throttle_levels",
)
parser.add_argument(
    "--all_pairs",
    action="store_true",
    help="Test all non-identical pairs of levels (not only adjacent steps)",
)
parser.add_argument(
    "--no_down",
    action="store_true",
    help="Skip descending steps for adjacent mode",
)
parser.add_argument(
    "--pre_hold",
    type=float,
    default=5.0,
    help="Time [s] to hold u_from before each step (default 5.0s)",
)
parser.add_argument(
    "--step_time",
    type=float,
    default=10.0,
    help="Time [s] to hold u_to during each step",
)
parser.add_argument(
    "--settle_time",
    type=float,
    default=5.0,
    help="Time [s] to coast at u_from between transitions to reach steady state",
)
parser.add_argument(
    "--settle_threshold",
    type=float,
    default=0.02,
    help="Velocity std threshold [m/s] to consider settled (if 0, use fixed settle_time)",
)
parser.add_argument(
    "--baseline_window",
    type=float,
    default=0.25,
    help="Window [s] at end of pre-hold used for v0 estimate",
)
parser.add_argument(
    "--steady_window",
    type=float,
    default=0.5,
    help="Window [s] at end of step used for v_ss estimate",
)
parser.add_argument(
    "--accel_peak_window",
    type=float,
    default=0.5,
    help="Window [s] from step start used for measured peak acceleration",
)
parser.add_argument(
    "--accel_alpha",
    type=float,
    default=0.25,
    help="Low-pass alpha for finite-difference acceleration [0..1]",
)
parser.add_argument(
    "--accel_source",
    type=str,
    default="tach",
    choices=["tach", "imu"],
    help="Acceleration source: tach=dv/dt from motorTach, imu=accelerometer axis",
)
parser.add_argument(
    "--imu_axis",
    type=int,
    default=0,
    help="Accelerometer axis index for longitudinal acceleration (default: 0)",
)
parser.add_argument(
    "--imu_sign",
    type=float,
    default=1.0,
    help="Sign multiplier applied to selected IMU axis (+1 or -1)",
)
parser.add_argument(
    "--imu_remove_bias",
    action="store_true",
    help="Subtract IMU bias estimated during pre-hold window",
)
parser.add_argument(
    "--imu_bias_window",
    type=float,
    default=0.25,
    help="Window [s] at end of pre-hold used for IMU bias estimate",
)
parser.add_argument(
    "--lookahead_ratio",
    type=float,
    default=0.632,
    help="Ratio for lead-time suggestion (0.632 means 63.2%% response)",
)
parser.add_argument(
    "--dt",
    type=float,
    default=0.02,
    help="Control loop timestep [s] (default 20ms)",
)
parser.add_argument(
    "--sim_tau",
    type=float,
    default=0.8,
    help="Simulation-only motor time constant [s]",
)
parser.add_argument(
    "--sim_gain",
    type=float,
    default=5.0,
    help="Simulation-only gain K [m/s per throttle unit]",
)
parser.add_argument(
    "--tag",
    type=str,
    default="",
    help="Optional output tag (default uses mode name)",
)
parser.add_argument(
    "--no_plot",
    action="store_true",
    help="Do not plot results at the end",
)
args = parser.parse_args()


def _parse_levels(levels_str: str, max_throttle: float) -> List[float]:
    values = []
    for tok in levels_str.split(","):
        tok = tok.strip()
        if not tok:
            continue
        values.append(float(tok))

    # Keep [0, max_throttle], unique, sorted
    kept = sorted(set(float(v) for v in values if 0.0 <= v <= max_throttle + 1e-12))
    if 0.0 not in kept:
        kept.insert(0, 0.0)
    return kept


def _build_transitions(
    levels: List[float], all_pairs: bool, include_down: bool
) -> List[Tuple[float, float]]:
    transitions: List[Tuple[float, float]] = []

    if all_pairs:
        for u_from in levels:
            for u_to in levels:
                if abs(u_to - u_from) > 1e-12:
                    transitions.append((u_from, u_to))
        return transitions

    for i in range(len(levels) - 1):
        transitions.append((levels[i], levels[i + 1]))

    if include_down:
        for i in range(len(levels) - 1, 0, -1):
            transitions.append((levels[i], levels[i - 1]))

    return transitions


LEVELS = _parse_levels(args.throttle_levels, args.max_throttle)
TRANSITIONS = _build_transitions(
    LEVELS, all_pairs=args.all_pairs, include_down=(not args.no_down)
)

if len(LEVELS) < 2:
    print(
        "[ERROR] Need at least two valid throttle levels in [0, max_throttle]. "
        f"Got: {LEVELS}"
    )
    sys.exit(1)

if len(TRANSITIONS) == 0:
    print("[ERROR] No transitions to run.")
    sys.exit(1)

if args.dt <= 0.0:
    print("[ERROR] --dt must be > 0.")
    sys.exit(1)

if args.pre_hold < 0.0 or args.step_time <= 0.0:
    print("[ERROR] Require --pre_hold >= 0 and --step_time > 0.")
    sys.exit(1)

if args.imu_axis not in (0, 1, 2):
    print("[ERROR] --imu_axis must be 0, 1, or 2.")
    sys.exit(1)

if args.imu_bias_window < 0.0:
    print("[ERROR] --imu_bias_window must be >= 0.")
    sys.exit(1)

if not (0.0 <= args.accel_alpha <= 1.0):
    print("[ERROR] --accel_alpha must be in [0, 1].")
    sys.exit(1)

if not (0.0 < args.lookahead_ratio < 1.0):
    print("[ERROR] --lookahead_ratio must be in (0, 1).")
    sys.exit(1)

if args.tag:
    TAG = f"_{args.tag}"
elif args.sim:
    TAG = "_sim"
elif args.qlabs:
    TAG = "_qlabs"
else:
    TAG = "_hardware"


# ---------------------------------------------------------------------------
# Hardware interface
# ---------------------------------------------------------------------------
class QCarHardwareInterface:
    """Unified wrapper for physical and QLabs virtual QCar access."""

    def __init__(self, qlabs_mode: bool = False, actor_name: str = "QC2_0"):
        self._qcar = None
        self._qlabs_mode = qlabs_mode
        self._actor_name = actor_name

    def connect(self):
        from pal.products.qcar import QCar

        if self._qlabs_mode:
            from qvl.multi_agent import readRobots

            robots = readRobots()
            if self._actor_name not in robots:
                raise RuntimeError(
                    f"Actor '{self._actor_name}' not found in QLabs. "
                    f"Available: {list(robots.keys())}"
                )
            hil_port = robots[self._actor_name].get("hilPort")
            if hil_port is None:
                raise RuntimeError(
                    f"hilPort=None for actor '{self._actor_name}'. "
                    "QLabs model may not be running."
                )
            self._qcar = QCar(readMode=1, hilPort=hil_port)
            print(
                f"[QLABS] Connected to virtual QCar '{self._actor_name}' "
                f"(hilPort={hil_port})"
            )
        else:
            self._qcar = QCar(readMode=1)
            print("[HW] Physical QCar connected.")

    def send_throttle(self, throttle: float, steering: float = 0.0):
        self._qcar.write(
            throttle=float(np.clip(throttle, -1.0, 1.0)),
            steering=float(np.clip(steering, -0.5, 0.5)),
        )

    def read_state(self) -> Tuple[float, np.ndarray]:
        self._qcar.read()
        velocity = float(getattr(self._qcar, "motorTach", 0.0))
        accel_raw = getattr(self._qcar, "accelerometer", np.zeros(3))
        accel = np.asarray(accel_raw, dtype=float).reshape(-1)
        if accel.size < 3:
            accel_padded = np.zeros(3, dtype=float)
            accel_padded[: accel.size] = accel
            accel = accel_padded
        return velocity, accel[:3]

    def stop(self):
        if self._qcar is None:
            return
        self.send_throttle(0.0)
        time.sleep(0.5)
        try:
            self._qcar.terminate()
        except Exception:
            pass
        print("[QCAR] Vehicle stopped/disconnected.")


def _crossing_time(
    t: np.ndarray, y: np.ndarray, target: float, direction: float
) -> float:
    """Return first interpolated crossing time of y(t) across target."""
    if direction >= 0.0:
        idx = np.where(y >= target)[0]
    else:
        idx = np.where(y <= target)[0]
    if idx.size == 0:
        return float("nan")

    i = int(idx[0])
    if i <= 0:
        return float(t[0])

    t0, t1 = float(t[i - 1]), float(t[i])
    y0, y1 = float(y[i - 1]), float(y[i])
    dy = y1 - y0
    if abs(dy) < 1e-12:
        return t1
    frac = float(np.clip((target - y0) / dy, 0.0, 1.0))
    return t0 + frac * (t1 - t0)


def _estimate_tau(
    t: np.ndarray, y: np.ndarray, y0: float, yss: float
) -> Tuple[float, str]:
    """
    Estimate first-order tau from a general step response:
        y(t) = yss + (y0 - yss) * exp(-t/tau)
    """
    delta = yss - y0
    if abs(delta) < 1e-8:
        return float("nan"), "no_change"

    # Preferred: log fit on ratio r(t) = (y - yss)/(y0 - yss) = exp(-t/tau)
    denom = y0 - yss
    r = (y - yss) / (denom + 1e-12)
    mask = np.isfinite(r) & (r > 0.05) & (r < 0.95)
    if np.count_nonzero(mask) >= 5:
        slope, _ = np.polyfit(t[mask], np.log(r[mask]), 1)
        if slope < -1e-8:
            return float(-1.0 / slope), "log_fit"

    # Fallback: t63 crossing
    target_63 = y0 + 0.6321205588 * delta
    direction = 1.0 if delta > 0.0 else -1.0
    t63 = _crossing_time(t, y, target_63, direction)
    if np.isfinite(t63) and t63 > 0.0:
        return float(t63), "t63_crossing"

    return float("nan"), "fit_failed"


def _collect_step_response(
    interface,
    sim_mode: bool,
    u_from: float,
    u_to: float,
    dt: float,
    pre_steps: int,
    step_steps: int,
    accel_alpha: float,
    accel_source: str,
    imu_axis: int,
    imu_sign: float,
    imu_remove_bias: bool,
    imu_bias_window: float,
    transition_idx: int,
    raw_log: CSVLogger,
    t_global0: float,
) -> Dict[str, object]:
    pre_v: List[float] = []
    pre_imu: List[float] = []
    step_t: List[float] = []
    step_v: List[float] = []
    step_a: List[float] = []

    effective_source = accel_source
    imu_failed = False
    imu_bias = 0.0
    prev_v = None
    prev_t = None  # wall-clock time of previous sample
    a_filt = 0.0
    every_sec_steps = max(1, int(round(1.0 / dt)))

    def _read_sample(throttle_cmd: float) -> Tuple[float, np.ndarray]:
        if sim_mode:
            v_sample = interface.step(throttle_cmd, dt)
            return float(v_sample), np.array([np.nan, np.nan, np.nan], dtype=float)
        interface.send_throttle(throttle_cmd)
        return interface.read_state()

    def _tach_accel(v_curr: float, dt_actual: float) -> float:
        if prev_v is None or dt_actual <= 0.0:
            return 0.0
        return float((v_curr - prev_v) / dt_actual)

    # Pre-hold at u_from
    phase_t0 = time.time()
    for k in range(pre_steps):
        loop_start = time.time()
        v, accel_vec = _read_sample(u_from)

        # Compute actual dt for tach derivative
        now = time.time()
        dt_actual = (now - prev_t) if prev_t is not None else dt

        if effective_source == "imu":
            a_imu = float(accel_vec[imu_axis]) * imu_sign
            if np.isfinite(a_imu):
                a_raw = a_imu
                pre_imu.append(a_imu)
            else:
                if not imu_failed:
                    print(
                        "  [WARN] IMU acceleration unavailable. "
                        "Falling back to tach derivative."
                    )
                imu_failed = True
                effective_source = "tach"
                a_raw = _tach_accel(v, dt_actual)
        else:
            a_raw = _tach_accel(v, dt_actual)

        a_filt = accel_alpha * a_raw + (1.0 - accel_alpha) * a_filt
        prev_v = v
        prev_t = now
        pre_v.append(v)

        elapsed = now - t_global0
        t_rel = now - phase_t0
        raw_log.write(
            transition_idx,
            "pre",
            elapsed,
            t_rel,
            u_from,
            v,
            a_filt,
            effective_source,
        )

        # Wall-clock-aligned sleep: target next sample at phase_t0 + (k+1)*dt
        t_target = phase_t0 + (k + 1) * dt
        t_sleep = t_target - time.time()
        if t_sleep > 0.0:
            time.sleep(t_sleep)

        if (k + 1) % every_sec_steps == 0:
            print(f"  pre  t={t_rel:5.2f}s  v={v: .4f} m/s  (loop {(time.time()-loop_start)*1e3:.1f}ms)")

    if effective_source == "imu" and imu_remove_bias and len(pre_imu) > 0:
        bias_steps = max(1, int(round(imu_bias_window / dt)))
        imu_bias = float(np.mean(pre_imu[-min(len(pre_imu), bias_steps) :]))
        print(f"  IMU bias estimate: {imu_bias:.4f} m/s^2")

    # Step to u_to
    step_t0 = time.time()
    for k in range(step_steps):
        loop_start = time.time()
        v, accel_vec = _read_sample(u_to)

        # Compute actual dt for tach derivative
        now = time.time()
        dt_actual = (now - prev_t) if prev_t is not None else dt

        if effective_source == "imu":
            a_imu = float(accel_vec[imu_axis]) * imu_sign
            if np.isfinite(a_imu):
                a_raw = a_imu - imu_bias
            else:
                if not imu_failed:
                    print(
                        "  [WARN] IMU acceleration unavailable during step. "
                        "Falling back to tach derivative."
                    )
                imu_failed = True
                effective_source = "tach"
                a_raw = _tach_accel(v, dt_actual)
        else:
            a_raw = _tach_accel(v, dt_actual)

        a_filt = accel_alpha * a_raw + (1.0 - accel_alpha) * a_filt
        prev_v = v
        prev_t = now

        t_rel = now - step_t0
        step_t.append(t_rel)
        step_v.append(v)
        step_a.append(a_filt)

        elapsed = now - t_global0
        raw_log.write(
            transition_idx,
            "step",
            elapsed,
            t_rel,
            u_to,
            v,
            a_filt,
            effective_source,
        )

        # Wall-clock-aligned sleep: target next sample at step_t0 + (k+1)*dt
        t_target = step_t0 + (k + 1) * dt
        t_sleep = t_target - time.time()
        if t_sleep > 0.0:
            time.sleep(t_sleep)

        if (k + 1) % every_sec_steps == 0:
            print(
                f"  step t={t_rel:5.2f}s  v={v: .4f} m/s  a={a_filt: .4f} m/s^2"
                f"  (loop {(time.time()-loop_start)*1e3:.1f}ms)"
            )

    if accel_source == "imu":
        source_used = "imu_fallback_tach" if imu_failed else "imu"
    else:
        source_used = "tach"

    return {
        "pre_v": np.asarray(pre_v, dtype=float),
        "step_t": np.asarray(step_t, dtype=float),
        "step_v": np.asarray(step_v, dtype=float),
        "step_a": np.asarray(step_a, dtype=float),
        "accel_source_used": source_used,
        "imu_bias_mps2": float(imu_bias),
    }


def _analyse_transition(
    u_from: float,
    u_to: float,
    pre_v: np.ndarray,
    step_t: np.ndarray,
    step_v: np.ndarray,
    step_a: np.ndarray,
    accel_source_used: str,
    imu_bias_mps2: float,
    dt: float,
    baseline_window: float,
    steady_window: float,
    accel_peak_window: float,
    lookahead_ratio: float,
) -> Dict[str, object]:
    baseline_steps = max(1, int(round(baseline_window / dt)))
    steady_steps = max(2, int(round(steady_window / dt)))
    peak_steps = max(1, int(round(accel_peak_window / dt)))

    v0 = float(np.mean(pre_v[-min(len(pre_v), baseline_steps) :]))
    vss = float(np.mean(step_v[-min(len(step_v), steady_steps) :]))

    delta_u = float(u_to - u_from)
    delta_v = float(vss - v0)
    tau_s, fit_method = _estimate_tau(step_t, step_v, v0, vss)

    if abs(delta_u) > 1e-12:
        k_local = float(delta_v / delta_u)
    else:
        k_local = float("nan")

    if np.isfinite(tau_s) and tau_s > 0.0:
        a0_model = float(delta_v / tau_s)
        t63 = float(tau_s)
        t90 = float(2.302585093 * tau_s)
        t95 = float(2.995732274 * tau_s)
        lead_time = float(-tau_s * np.log(1.0 - lookahead_ratio))
    else:
        a0_model = float("nan")
        t63 = float("nan")
        t90 = float("nan")
        t95 = float("nan")
        lead_time = float("nan")

    early_a = step_a[: min(len(step_a), peak_steps)]
    if early_a.size > 0:
        direction = 1.0 if delta_u >= 0.0 else -1.0
        if direction >= 0.0:
            a_peak_meas = float(np.max(early_a))
        else:
            a_peak_meas = float(np.min(early_a))
        a_mean_early = float(np.mean(early_a))
    else:
        a_peak_meas = float("nan")
        a_mean_early = float("nan")

    return {
        "u_from": float(u_from),
        "u_to": float(u_to),
        "delta_u": delta_u,
        "accel_source": accel_source_used,
        "imu_bias_mps2": float(imu_bias_mps2),
        "v0_mps": v0,
        "vss_mps": vss,
        "delta_v_mps": delta_v,
        "tau_s": tau_s,
        "k_local_mps_per_throttle": k_local,
        "a0_model_mps2": a0_model,
        "a_peak_meas_mps2": a_peak_meas,
        "a_mean_early_mps2": a_mean_early,
        "t63_s": t63,
        "t90_s": t90,
        "t95_s": t95,
        "lead_time_s": lead_time,
        "fit_method": fit_method,
    }


def _settle_at_throttle(
    interface,
    throttle: float,
    dt: float,
    settle_time: float = 5.0,
    threshold: float = 0.02,
):
    """Hold throttle and wait until velocity stabilises (like 01's coast-back).

    If threshold > 0, will monitor velocity std over a 1s window and stop early
    once std < threshold. Otherwise, just waits settle_time seconds.
    """
    print(f"  [settle] Holding throttle={throttle:.3f} for up to {settle_time:.1f}s ...")
    n_steps = max(1, int(settle_time / dt))
    window_size = max(5, int(1.0 / dt))  # 1-second rolling window
    recent_v: List[float] = []
    t0 = time.time()

    for k in range(n_steps):
        interface.send_throttle(throttle)
        v, _ = interface.read_state()
        recent_v.append(v)

        # Check convergence with rolling std
        if threshold > 0.0 and len(recent_v) >= window_size:
            v_window = recent_v[-window_size:]
            v_std = float(np.std(v_window))
            if v_std < threshold:
                elapsed = time.time() - t0
                v_mean = float(np.mean(v_window))
                print(
                    f"  [settle] Converged at t={elapsed:.1f}s  "
                    f"v={v_mean:.4f} m/s  std={v_std:.4f}"
                )
                return

        # Wall-clock aligned sleep
        t_target = t0 + (k + 1) * dt
        t_sleep = t_target - time.time()
        if t_sleep > 0.0:
            time.sleep(t_sleep)

        # Progress every 2 seconds
        if (k + 1) % max(1, int(2.0 / dt)) == 0:
            v_mean = float(np.mean(recent_v[-window_size:]))
            v_std = float(np.std(recent_v[-window_size:])) if len(recent_v) >= window_size else float('nan')
            print(f"  [settle] t={time.time()-t0:.1f}s  v={v_mean:.4f} m/s  std={v_std:.4f}")

    elapsed = time.time() - t0
    v_mean = float(np.mean(recent_v[-window_size:]))
    print(f"  [settle] Done (fixed {elapsed:.1f}s)  v={v_mean:.4f} m/s")


def run_calibration(
    interface,
    sim_mode: bool,
    transitions: List[Tuple[float, float]],
    accel_source: str,
    imu_axis: int,
    imu_sign: float,
    imu_remove_bias: bool,
    imu_bias_window: float,
):
    dt = args.dt
    pre_steps = max(1, int(round(args.pre_hold / dt)))
    step_steps = max(5, int(round(args.step_time / dt)))

    raw_csv = f"throttle_accel_step_raw{TAG}.csv"
    lookup_csv = f"throttle_accel_lookup{TAG}.csv"

    results: List[Dict[str, object]] = []

    with CSVLogger(
        raw_csv,
        [
            "transition_idx",
            "phase",
            "time_s",
            "rel_time_s",
            "throttle_cmd",
            "velocity_mps",
            "accel_mps2",
            "accel_source",
        ],
    ) as raw_log, CSVLogger(
        lookup_csv,
        [
            "transition",
            "u_from",
            "u_to",
            "delta_u",
            "accel_source",
            "imu_bias_mps2",
            "v0_mps",
            "vss_mps",
            "delta_v_mps",
            "tau_s",
            "k_local_mps_per_throttle",
            "a0_model_mps2",
            "a_peak_meas_mps2",
            "a_mean_early_mps2",
            "t63_s",
            "t90_s",
            "t95_s",
            "lead_time_s",
            "fit_method",
        ],
    ) as lut_log:
        t_global0 = time.time()

        for idx, (u_from, u_to) in enumerate(transitions, start=1):
            print_section(
                f"Transition {idx}/{len(transitions)}: {u_from:.3f} -> {u_to:.3f}"
            )

            # ── Settle at u_from before starting (like 01's coast-back) ──
            if not sim_mode:
                _settle_at_throttle(
                    interface, u_from, dt,
                    settle_time=args.settle_time,
                    threshold=args.settle_threshold,
                )
            elif hasattr(interface, 'reset'):
                # Sim mode: if transitioning from 0, reset to ensure clean start
                if u_from == 0.0:
                    interface.reset()
                else:
                    # Run sim at u_from until settled
                    for _ in range(max(1, int(args.settle_time / dt))):
                        interface.step(u_from, dt)

            data = _collect_step_response(
                interface=interface,
                sim_mode=sim_mode,
                u_from=u_from,
                u_to=u_to,
                dt=dt,
                pre_steps=pre_steps,
                step_steps=step_steps,
                accel_alpha=args.accel_alpha,
                accel_source=accel_source,
                imu_axis=imu_axis,
                imu_sign=imu_sign,
                imu_remove_bias=imu_remove_bias,
                imu_bias_window=imu_bias_window,
                transition_idx=idx,
                raw_log=raw_log,
                t_global0=t_global0,
            )

            info = _analyse_transition(
                u_from=u_from,
                u_to=u_to,
                pre_v=data["pre_v"],
                step_t=data["step_t"],
                step_v=data["step_v"],
                step_a=data["step_a"],
                accel_source_used=data["accel_source_used"],
                imu_bias_mps2=data["imu_bias_mps2"],
                dt=dt,
                baseline_window=args.baseline_window,
                steady_window=args.steady_window,
                accel_peak_window=args.accel_peak_window,
                lookahead_ratio=args.lookahead_ratio,
            )

            transition_name = f"{u_from:.3f}->{u_to:.3f}"
            lut_log.write(
                transition_name,
                info["u_from"],
                info["u_to"],
                info["delta_u"],
                info["accel_source"],
                info["imu_bias_mps2"],
                info["v0_mps"],
                info["vss_mps"],
                info["delta_v_mps"],
                info["tau_s"],
                info["k_local_mps_per_throttle"],
                info["a0_model_mps2"],
                info["a_peak_meas_mps2"],
                info["a_mean_early_mps2"],
                info["t63_s"],
                info["t90_s"],
                info["t95_s"],
                info["lead_time_s"],
                info["fit_method"],
            )
            results.append(info)

            print(
                f"  tau={info['tau_s']:.4f}s  "
                f"K_local={info['k_local_mps_per_throttle']:.4f}  "
                f"a0={info['a0_model_mps2']:.4f} m/s^2  "
                f"lead={info['lead_time_s']:.4f}s  "
                f"src={info['accel_source']}  "
                f"({info['fit_method']})"
            )

    return results


def print_summary(results: List[Dict[str, object]]):
    print_section("Throttle-Step Dynamics Summary")
    print(
        f"  {'Step':>13}  {'source':>15}  {'tau[s]':>8}  {'Kloc':>9}  "
        f"{'a0[m/s2]':>10}  {'lead[s]':>8}"
    )
    print(f"  {'-' * 13}  {'-' * 15}  {'-' * 8}  {'-' * 9}  {'-' * 10}  {'-' * 8}")

    for row in results:
        step_name = f"{row['u_from']:.2f}->{row['u_to']:.2f}"
        print(
            f"  {step_name:>13}  {str(row['accel_source']):>15}  {row['tau_s']:8.4f}  "
            f"{row['k_local_mps_per_throttle']:9.4f}  "
            f"{row['a0_model_mps2']:10.4f}  {row['lead_time_s']:8.4f}"
        )


def plot_calibration_results(raw_path: str, tag: str, results_dir: str):
    import os

    try:
        import matplotlib.pyplot as plt
        import pandas as pd
    except ImportError:
        print("[WARN] matplotlib or pandas not available. Skipping plot.")
        return

    if not os.path.exists(raw_path):
        print(f"[WARN] Raw data file {raw_path} not found. Skipping plot.")
        return

    try:
        df = pd.read_csv(raw_path)
    except Exception as e:
        print(f"[WARN] Could not load raw data for plotting: {e}")
        return

    transitions = df["transition_idx"].unique()
    num_transitions = len(transitions)

    if num_transitions == 0:
        return

    # Create subplots based on number of transitions
    # We will plot up to 10 transitions in one figure to avoid enormous plots
    max_plots = min(num_transitions, 10)
    fig, axes = plt.subplots(max_plots, 2, figsize=(12, 3 * max_plots), sharex=False)

    # Ensure axes is 2D even for 1 transition
    if max_plots == 1:
        axes = [axes]

    for i, t_idx in enumerate(transitions[:max_plots]):
        df_t = df[df["transition_idx"] == t_idx]

        ax_v = axes[i][0]
        ax_v.plot(
            df_t["time_s"], df_t["velocity_mps"], label="Velocity (m/s)", color="blue"
        )
        ax_v.plot(
            df_t["time_s"],
            df_t["throttle_cmd"],
            label="Cmd",
            color="black",
            linestyle="--",
        )
        ax_v.set_ylabel("Velocity / Cmd")
        ax_v.legend(loc="upper left")
        ax_v.grid(True)
        u_from = df_t["throttle_cmd"].iloc[0]
        u_to = df_t["throttle_cmd"].iloc[-1]
        ax_v.set_title(f"Transition {t_idx}: Throttle {u_from:.2f} -> {u_to:.2f}")

        ax_a = axes[i][1]
        ax_a.plot(
            df_t["time_s"], df_t["accel_mps2"], label="Accel (m/s^2)", color="red"
        )
        ax_a.set_ylabel("Accel [m/s^2]")
        ax_a.legend(loc="upper left")
        ax_a.grid(True)
        ax_a.set_title("Acceleration Profile")

        if i == max_plots - 1:
            ax_v.set_xlabel("Time [s]")
            ax_a.set_xlabel("Time [s]")

    plt.tight_layout()
    save_path = os.path.join(results_dir, f"throttle_accel_plot{tag}.png")
    plt.savefig(save_path)
    print(f"[PLOT] Saved summary figure to {save_path}")
    plt.show()


def main():
    if args.sim and args.qlabs:
        print("[ERROR] Cannot use --sim and --qlabs together.")
        sys.exit(1)

    if args.sim:
        mode_label = "SIMULATION (Python)"
    elif args.qlabs:
        mode_label = f"QLABS virtual QCar '{args.actor}'"
    else:
        mode_label = "PHYSICAL hardware"

    accel_source = args.accel_source
    if args.sim and accel_source == "imu":
        print("[WARN] --accel_source imu is unavailable in --sim. Using tach instead.")
        accel_source = "tach"

    print_section("QCar Throttle-Step -> Acceleration Calibration")
    print(f"  Mode           : {mode_label}")
    print(f"  Levels         : {LEVELS}")
    print(f"  Transitions    : {[(round(a, 3), round(b, 3)) for a, b in TRANSITIONS]}")
    print(f"  pre_hold       : {args.pre_hold}s")
    print(f"  step_time      : {args.step_time}s")
    print(f"  dt             : {args.dt}s")
    print(f"  max_throttle   : {args.max_throttle}")
    print(f"  accel_source   : {accel_source}")
    if accel_source == "imu":
        print(f"  imu_axis/sign  : {args.imu_axis} / {args.imu_sign}")
        print(f"  imu_remove_bias: {bool(args.imu_remove_bias)}")
        print(f"  imu_bias_window: {args.imu_bias_window}s")
    print(f"  lookahead_ratio: {args.lookahead_ratio}")

    if args.sim:
        interface = FirstOrderMotorSim(
            tau=args.sim_tau, K=args.sim_gain, noise_std=0.01
        )
    else:
        interface = QCarHardwareInterface(qlabs_mode=args.qlabs, actor_name=args.actor)
        interface.connect()

    try:
        rows = run_calibration(
            interface=interface,
            sim_mode=args.sim,
            transitions=TRANSITIONS,
            accel_source=accel_source,
            imu_axis=args.imu_axis,
            imu_sign=args.imu_sign,
            imu_remove_bias=args.imu_remove_bias,
            imu_bias_window=args.imu_bias_window,
        )
    finally:
        if args.sim:
            # Keep behavior consistent with hardware scripts: finish at zero
            interface.reset()
        else:
            interface.stop()

    if len(rows) == 0:
        print("[ERROR] No results recorded.")
        sys.exit(1)

    print_summary(rows)

    model_yaml = {
        "description": "Throttle-step to acceleration dynamics lookup table",
        "mode": "simulation" if args.sim else ("qlabs" if args.qlabs else "hardware"),
        "model_assumption": {
            "equation": "tau * dv/dt + v = K_local * u (local per transition)",
            "velocity_response": "v(t) = v_ss + (v0 - v_ss) * exp(-t/tau)",
            "accel_response": "a(t) = ((v_ss - v0)/tau) * exp(-t/tau)",
        },
        "settings": {
            "throttle_levels": LEVELS,
            "max_throttle": float(args.max_throttle),
            "all_pairs": bool(args.all_pairs),
            "include_down": bool(not args.no_down),
            "pre_hold_s": float(args.pre_hold),
            "step_time_s": float(args.step_time),
            "baseline_window_s": float(args.baseline_window),
            "steady_window_s": float(args.steady_window),
            "accel_peak_window_s": float(args.accel_peak_window),
            "accel_alpha": float(args.accel_alpha),
            "accel_source_requested": str(args.accel_source),
            "accel_source_effective": str(accel_source),
            "imu_axis": int(args.imu_axis),
            "imu_sign": float(args.imu_sign),
            "imu_remove_bias": bool(args.imu_remove_bias),
            "imu_bias_window_s": float(args.imu_bias_window),
            "lookahead_ratio": float(args.lookahead_ratio),
            "dt_s": float(args.dt),
        },
        "lookup_table": rows,
        "usage": {
            "lookup_key": "select by nearest (u_from, u_to) transition",
            "preview_formula": "lead_time = -tau * ln(1 - ratio)",
            "example_ratio": float(args.lookahead_ratio),
            "example_note": (
                "If ratio=0.632, lead_time approx tau. "
                "Use this to command next throttle earlier."
            ),
        },
    }

    save_yaml(model_yaml, f"throttle_accel_model{TAG}.yaml")
    # Canonical filename for downstream tooling
    save_yaml(model_yaml, "throttle_accel_model.yaml")

    print("\n[OK] Throttle-step acceleration calibration complete.")

    if not args.no_plot:
        raw_csv_path = os.path.join(RESULTS_DIR, f"throttle_accel_step_raw{TAG}.csv")
        plot_calibration_results(raw_csv_path, TAG, RESULTS_DIR)


if __name__ == "__main__":
    main()
