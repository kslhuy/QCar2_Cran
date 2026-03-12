#!/usr/bin/env python3
"""
08_system_id_experiments.py
============================
QCar / Limo  –  System-Identification Experiments
---------------------------------------------------
Adapted from the F1/10 ``controller_node.py`` (ROS + VESC) to the
QCar / Limo hardware interfaces used in this project.

Four experiments are implemented:

  Exp 1  –  Steering Mapping
      Sends a constant throttle and a constant steering command.
      Records the resulting yaw-rate so the user can extract
      ``steering_to_servo_gain`` and ``steering_to_servo_offset``.

  Exp 3  –  Safe Acceleration / Deceleration
      Commands a ramp-up throttle phase followed by coast-down (throttle=0)
      through the standard QCar.write() interface — no raw current needed.
      Records velocity & acceleration to characterise the longitudinal
      dynamics.

  Exp 4  –  Steering Triangle Sweep (constant speed)
      Holds a constant throttle while sweeping the steering angle in a
      symmetric triangle pattern:  0 → +max_angle → -max_angle → 0  over
      ``sweep_time`` seconds, preceded by a ``warmup_time`` straight-line
      phase for the car to reach steady speed.  Three equal-rate phases
      give uniform excitation; both sweep directions reveal hysteresis.

  Exp 5  –  Velocity / Triangle Steering Sweep
      Commands a constant target velocity (via a simple P-controller on
      throttle) while sweeping the steering angle in the same triangle
      pattern (0 → +max → -max → 0), also preceded by a warm-up phase.
      Records lateral dynamics (yaw-rate, lateral acceleration) for
      tyre-friction identification (feeds into ``analyse_tires.py``).

Supports THREE modes:
  --sim      : pure Python kinematic bicycle + first-order motor model
  --qlabs    : QLabs virtual QCar via readRobots()
  (default)  : physical QCar or Limo hardware

Usage examples:
  # Simulation — Experiment 1 (steering mapping)
  python 08_system_id_experiments.py --sim --experiment 1

  # QLabs — Experiment 4 (steering sweep)
  python 08_system_id_experiments.py --qlabs --actor QC2_0 --experiment 4

  # Real QCar hardware — Experiment 3 (accel/decel)
  python 08_system_id_experiments.py --experiment 3 --accel_time 3 --decel_time 4

  # Real Limo hardware — Experiment 5 (velocity/steering sweep)
  python 08_system_id_experiments.py --vehicle_type Limo --experiment 5

Output (per experiment):
  results/08_system_id/<experiment_tag>_timeseries_<mode>.csv
  results/08_system_id/<experiment_tag>_summary_<mode>.yaml
  results/08_system_id/<experiment_tag>_<mode>.png
"""

import argparse
import os
import sys
import time
import numpy as np
from typing import Optional

# ── Project path setup ────────────────────────────────────────────────────────
_CAL_DIR = os.path.dirname(os.path.abspath(__file__))
_QCAR_DIR = os.path.dirname(_CAL_DIR)
for _p in [_CAL_DIR, _QCAR_DIR]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from calibration_utils import (
    CSVLogger,
    FirstOrderMotorSim,
    BicycleSteeringSim,
    plot_calibration_map,
    save_yaml,
    print_section,
)

# ── CLI ───────────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(
    description="QCar/Limo system-identification experiments "
    "(adapted from F1/10 controller_node.py)",
    formatter_class=argparse.RawTextHelpFormatter,
)

# Mode selection
parser.add_argument("--sim", action="store_true",
                    help="Pure Python simulation (no QCar / Limo)")
parser.add_argument("--qlabs", action="store_true",
                    help="QLabs virtual QCar (run initPlatoon.py first)")
parser.add_argument("--actor", type=str, default="QC2_0",
                    help="QLabs actor name (with --qlabs)")
parser.add_argument("--vehicle_type", type=str, default="Qcar",
                    choices=["Qcar", "Limo"],
                    help="Vehicle platform: 'Qcar' or 'Limo'")

# Experiment selector
parser.add_argument("--experiment", "-e", type=int, required=True,
                    choices=[1, 3, 4, 5],
                    help=("Experiment number:\n"
                          "  1  Steering mapping (const throttle + const steering)\n"
                          "  3  Safe accel / decel via throttle commands\n"
                          "  4  Steering sweep at constant throttle\n"
                          "  5  Velocity / steering sweep (tyre dynamics)"))

# ── Experiment 1 parameters ──────────────────────────────────────────────────
parser.add_argument("--const_throttle", type=float, default=0.1,
                    help="[Exp 1,3,4] Constant throttle command (default 0.12)")
parser.add_argument("--const_steering", type=float, default=0.0,
                    help="[Exp 1] Constant steering command (default 0.0)")
parser.add_argument("--hold_time", type=float, default=10.0,
                    help="[Exp 1] How long to hold the command [s] (default 10)")

# ── Experiment 3 parameters ──────────────────────────────────────────────────
parser.add_argument("--accel_throttle", type=float, default=0.15,
                    help="[Exp 3] Throttle during acceleration phase (default 0.15)")
parser.add_argument("--accel_time", type=float, default=5.0,
                    help="[Exp 3] Duration of acceleration phase [s] (default 5)")
parser.add_argument("--decel_time", type=float, default=5.0,
                    help="[Exp 3] Duration of deceleration (coast) phase [s] (default 5)")
parser.add_argument("--wait_before", type=float, default=2.0,
                    help="[Exp 3] Stand-still delay before starting [s] (default 2)")

# ── Experiment 4 & 5 parameters ──────────────────────────────────────────────
parser.add_argument("--max_angle", type=float, default=0.5,
                    help="[Exp 4,5] Maximum steering angle magnitude (default 0.5)")
parser.add_argument("--sweep_time", type=float, default=10.0,
                    help="[Exp 4,5] Total duration of triangle sweep [s] (default 10)")
parser.add_argument("--warmup_time", type=float, default=0.5,
                    help="[Exp 4,5] Straight-line warm-up before sweep [s] (default 0.5)")

# ── Experiment 5 specific ────────────────────────────────────────────────────
parser.add_argument("--target_velocity", type=float, default=0.5,
                    help="[Exp 5] Target velocity [m/s] (default 0.5)")
parser.add_argument("--vel_kp", type=float, default=0.3,
                    help="[Exp 5] P-gain for velocity controller (default 0.3)")
parser.add_argument("--vel_ff", type=float, default=0.10,
                    help="[Exp 5] Feed-forward throttle per m/s (default 0.10)")

# ── General ───────────────────────────────────────────────────────────────────
parser.add_argument("--dt", type=float, default=0.02,
                    help="Control-loop time step [s] (default 0.02 = 50 Hz)")
parser.add_argument("--wheelbase", type=float, default=0.256,
                    help="Nominal wheelbase [m] (default 0.256 for QCar)")
parser.add_argument("--tag", type=str, default="",
                    help="Optional tag for output filenames")

args = parser.parse_args()

# Validate mode
if args.sim and args.qlabs:
    print("[ERROR] Cannot combine --sim and --qlabs.  Pick one.")
    sys.exit(1)

# Output directory
RESULTS_DIR = os.path.join(_CAL_DIR, "results", "08_system_id")
os.makedirs(RESULTS_DIR, exist_ok=True)

# Auto-tag
if args.tag:
    TAG = f"_{args.tag}"
elif args.sim:
    TAG = "_sim"
elif args.qlabs:
    TAG = "_qlabs"
else:
    TAG = f"_{args.vehicle_type.lower()}"

EXP_NAMES = {1: "steering_mapping", 3: "safe_accel_decel",
              4: "steering_sweep", 5: "velocity_steering_sweep"}
EXP_TAG = EXP_NAMES[args.experiment]


# ═════════════════════════════════════════════════════════════════════════════
#  Hardware interface  (QCar / Limo / QLabs)
# ═════════════════════════════════════════════════════════════════════════════


class UnifiedHardwareInterface:
    """
    Thin wrapper around the real QCar object (physical or QLabs).
    Provides:
        .send(throttle, steering)
        .read_state() -> dict  {velocity, yaw_rate, accel_x, accel_y}
        .stop() / .close()
    """

    def __init__(self, vehicle_type: str = "Qcar",
                 qlabs_mode: bool = False, actor_name: str = "QC2_0"):
        self._qcar = None
        self._vehicle_type = vehicle_type
        self._qlabs_mode = qlabs_mode
        self._actor_name = actor_name

    # ── connect ───────────────────────────────────────────────────────────
    def connect(self):
        from pal.products.qcar import QCar

        if self._qlabs_mode:
            from qvl.multi_agent import readRobots
            robots = readRobots()
            if self._actor_name not in robots:
                raise RuntimeError(
                    f"Actor '{self._actor_name}' not found.  "
                    f"Available: {list(robots.keys())}.  "
                    f"Did initPlatoon.py run successfully?")
            hil_port = robots[self._actor_name].get("hilPort")
            if hil_port is None:
                raise RuntimeError(
                    f"hilPort=None for '{self._actor_name}'.")
            self._qcar = QCar(readMode=1, hilPort=hil_port)
            print(f"[QLABS] Connected to '{self._actor_name}' "
                  f"(hilPort={hil_port})")
        else:
            self._qcar = QCar(readMode=1)
            print(f"[{self._vehicle_type.upper()}] Hardware connected.")

    # ── send ──────────────────────────────────────────────────────────────
    def send(self, throttle: float, steering: float):
        """Send throttle + steering to the vehicle.

        QCar : throttle ∈ [-1, 1]  (normalised motor command)
        Limo : throttle is velocity [m/s] – clip appropriately
        """
        if self._vehicle_type == "Limo":
            thr = float(np.clip(throttle, -0.3, 1.2))
        else:
            thr = float(np.clip(throttle, -1.0, 1.0))
        steer = float(np.clip(steering, -0.5, 0.5))
        self._qcar.write(throttle=thr, steering=steer)

    # ── read ──────────────────────────────────────────────────────────────
    def read_state(self) -> dict:
        """Return a dict with current sensor readings."""
        self._qcar.read()
        v = float(getattr(self._qcar, "motorTach", 0.0))
        gyro = getattr(self._qcar, "gyroscope", [0.0, 0.0, 0.0])
        accel = getattr(self._qcar, "accelerometer", [0.0, 0.0, 0.0])
        return {
            "velocity": v,
            "yaw_rate": float(gyro[2]),
            "accel_x": float(accel[0]),
            "accel_y": float(accel[1]),
        }

    # ── stop / close ─────────────────────────────────────────────────────
    def stop(self):
        self.send(0.0, 0.0)
        time.sleep(0.5)

    def close(self):
        self.stop()
        if self._qcar is not None:
            try:
                self._qcar.terminate()
            except Exception:
                pass
        label = "QLABS" if self._qlabs_mode else self._vehicle_type.upper()
        print(f"[{label}] Disconnected.")


# ═════════════════════════════════════════════════════════════════════════════
#  Simulation wrapper  (matches the same .send / .read_state interface)
# ═════════════════════════════════════════════════════════════════════════════


class SimulatedVehicle:
    """Combine FirstOrderMotorSim + BicycleSteeringSim to mimic
    the hardware interface in pure Python."""

    def __init__(self, wheelbase: float = 0.256):
        self.motor = FirstOrderMotorSim(tau=0.8, K=5.0, noise_std=0.01)
        self.bike = BicycleSteeringSim(wheelbase=wheelbase, noise_std=0.001)
        self._last_v = 0.0
        self._last_yaw_rate = 0.0
        self._dt = 0.02  # will be overwritten in step()

    def send(self, throttle: float, steering: float, dt: float = 0.02):
        self._dt = dt
        v = self.motor.step(throttle, dt)
        self._last_v = v
        _, _, theta = self.bike.step(v, steering, dt)
        # yaw-rate = v / L * tan(delta)  + noise (already embedded in bike)
        self._last_yaw_rate = (v / self.bike.L) * np.tan(steering)

    def read_state(self) -> dict:
        return {
            "velocity": self._last_v,
            "yaw_rate": self._last_yaw_rate + np.random.randn() * 0.002,
            "accel_x": 0.0,  # not modelled
            "accel_y": self._last_v * self._last_yaw_rate,  # centripetal approx
        }

    def stop(self):
        self.motor.reset()
        self.bike.reset()
        self._last_v = 0.0
        self._last_yaw_rate = 0.0

    def close(self):
        self.stop()
        print("[SIM] Simulation ended.")


# ═════════════════════════════════════════════════════════════════════════════
#  Experiment implementations
# ═════════════════════════════════════════════════════════════════════════════


def _triangle_steer(frac: float, max_angle: float) -> float:
    """Symmetric triangle sweep:  0 → +max → -max → 0  over frac ∈ [0, 1].

    Phase 1 (frac 0.00 – 0.25):  0         → +max_angle
    Phase 2 (frac 0.25 – 0.75): +max_angle → -max_angle
    Phase 3 (frac 0.75 – 1.00): -max_angle →  0

    Three phases with equal angular rate give uniform excitation and
    symmetric coverage.  Crossing zero at frac=0.50 provides a clean
    reference; both sweep directions reveal hysteresis.
    """
    if frac <= 0.25:
        # Phase 1: 0 → +max_angle
        return max_angle * (frac / 0.25)
    elif frac <= 0.75:
        # Phase 2: +max_angle → -max_angle
        return max_angle * (1.0 - 4.0 * (frac - 0.25))
    else:
        # Phase 3: -max_angle → 0
        return max_angle * (-1.0 + 4.0 * (frac - 0.75))


def _make_csv_and_cols(exp_tag: str):
    """Return (csv_filename, column_list) for all experiments."""
    cols = ["time_s", "throttle_cmd", "steering_cmd",
            "velocity_ms", "yaw_rate_rads", "accel_x", "accel_y"]
    fn = f"{exp_tag}_timeseries{TAG}.csv"
    return fn, cols


# ── Experiment 1 — Steering Mapping ──────────────────────────────────────────

def run_exp1_steering_mapping(iface, sim_mode: bool):
    """
    Exp 1: Send constant throttle + constant steering for ``hold_time`` seconds.
    Measure yaw-rate → derive effective steering gain.

    Original F1/10 equivalent: experiment==1  (send_const_vesc_cmd)
      – There it sent constant eRPM + constant servo PWM.
      – Here we send throttle + steering through QCar.write().
    """
    print_section("Experiment 1 — Steering Mapping")
    print(f"  Throttle  : {args.const_throttle}")
    print(f"  Steering  : {args.const_steering}")
    print(f"  Duration  : {args.hold_time} s")

    csv_fn, cols = _make_csv_and_cols(EXP_TAG)
    dt = args.dt
    n_steps = int(args.hold_time / dt)

    velocities, yaw_rates = [], []

    with CSVLogger(csv_fn, cols, results_dir=RESULTS_DIR) as log:
        t0 = time.time()
        for step in range(n_steps):
            t_now = step * dt

            if sim_mode:
                iface.send(args.const_throttle, args.const_steering, dt)
            else:
                iface.send(args.const_throttle, args.const_steering)

            state = iface.read_state()
            velocities.append(state["velocity"])
            yaw_rates.append(state["yaw_rate"])

            log.write(t_now, args.const_throttle, args.const_steering,
                      state["velocity"], state["yaw_rate"],
                      state["accel_x"], state["accel_y"])

            if not sim_mode:
                time.sleep(dt)

            # Live feedback every second
            if (step + 1) % int(1.0 / dt) == 0:
                print(f"  t={t_now+dt:.1f}s  v={state['velocity']:.4f} m/s  "
                      f"yaw_rate={state['yaw_rate']:.4f} rad/s")

    # ── Analysis ──────────────────────────────────────────────────────────
    # Discard first 20 % as transient
    n_discard = max(1, int(0.2 * len(velocities)))
    v_ss = float(np.mean(velocities[n_discard:]))
    yr_ss = float(np.mean(yaw_rates[n_discard:]))

    # If v_ss > 0 and yr_ss != 0 → turning radius R = v / yaw_rate
    if abs(yr_ss) > 1e-4 and abs(v_ss) > 1e-4:
        R_meas = v_ss / yr_ss
        kappa_meas = 1.0 / R_meas
        # Ackermann: delta_eff = atan(L / R)
        delta_eff = np.arctan2(args.wheelbase, abs(R_meas))
        if args.const_steering != 0:
            gain = delta_eff / args.const_steering
            offset = 0.0  # single-point → offset unknown; need multiple points
        else:
            gain = float("nan")
            offset = 0.0
    else:
        R_meas = float("inf")
        kappa_meas = 0.0
        delta_eff = 0.0
        gain = float("nan")
        offset = 0.0

    summary = {
        "experiment": 1,
        "description": "Steering mapping — constant throttle + steering",
        "throttle_cmd": args.const_throttle,
        "steering_cmd": args.const_steering,
        "v_steady_state_ms": round(v_ss, 5),
        "yaw_rate_steady_rads": round(yr_ss, 5),
        "turning_radius_m": round(R_meas, 4) if np.isfinite(R_meas) else None,
        "curvature_1_m": round(kappa_meas, 5),
        "delta_eff_rad": round(delta_eff, 5),
        "steering_to_servo_gain_estimate": round(gain, 5) if np.isfinite(gain) else None,
        "note": ("Run with multiple --const_steering values to get gain+offset. "
                 "gain = delta_eff / cmd;  offset = delta_eff - gain*cmd."),
    }
    save_yaml(summary, f"{EXP_TAG}_summary{TAG}.yaml", results_dir=RESULTS_DIR)

    print(f"\n  v_ss         = {v_ss:.4f} m/s")
    print(f"  yaw_rate_ss  = {yr_ss:.5f} rad/s")
    print(f"  R            = {R_meas:.3f} m")
    print(f"  delta_eff    = {np.degrees(delta_eff):.2f} deg")
    if np.isfinite(gain):
        print(f"  gain (steer→servo) ~= {gain:.4f}")
    print()
    return summary


# ── Experiment 3 — Safe Acceleration / Deceleration ──────────────────────────

def run_exp3_safe_accel_decel(iface, sim_mode: bool):
    """
    Exp 3: Accelerate with constant throttle, then coast (throttle=0).

    Original F1/10 equivalent: experiment==3  (drive_accel_decel)
      – There it sent AckermannDriveStamped with jerk=512 flag for
        raw acceleration control.
      – Here we simply step the throttle up and then set it to 0,
        recording velocity vs. time.  Safer and compatible with
        both QCar and Limo.
    """
    print_section("Experiment 3 — Safe Acceleration / Deceleration")
    print(f"  Throttle    : {args.accel_throttle}")
    print(f"  Accel phase : {args.accel_time} s")
    print(f"  Decel phase : {args.decel_time} s")
    print(f"  Wait before : {args.wait_before} s")

    csv_fn, cols = _make_csv_and_cols(EXP_TAG)
    dt = args.dt
    total_time = args.wait_before + args.accel_time + args.decel_time
    n_steps = int(total_time / dt)

    times, vels, accels_x = [], [], []

    with CSVLogger(csv_fn, cols, results_dir=RESULTS_DIR) as log:
        prev_v = 0.0
        for step in range(n_steps):
            t_now = step * dt

            # Phase logic
            if t_now < args.wait_before:
                thr = 0.0
                phase = "WAIT"
            elif t_now < args.wait_before + args.accel_time:
                thr = args.accel_throttle
                phase = "ACCEL"
            else:
                thr = 0.0
                phase = "DECEL"

            steer = 0.0  # straight line

            if sim_mode:
                iface.send(thr, steer, dt)
            else:
                iface.send(thr, steer)

            state = iface.read_state()
            v = state["velocity"]
            a_x = (v - prev_v) / dt if step > 0 else 0.0
            prev_v = v

            times.append(t_now)
            vels.append(v)
            accels_x.append(a_x)

            log.write(t_now, thr, steer,
                      state["velocity"], state["yaw_rate"],
                      state["accel_x"], state["accel_y"])

            if not sim_mode:
                time.sleep(dt)

            if (step + 1) % int(1.0 / dt) == 0:
                print(f"  [{phase:5s}] t={t_now+dt:.1f}s  "
                      f"v={v:.4f} m/s  a_x={a_x:.3f} m/s^2")

    # ── Analysis ──────────────────────────────────────────────────────────
    times = np.array(times)
    vels = np.array(vels)
    accels_x = np.array(accels_x)

    # Peak velocity & max acceleration
    v_peak = float(np.max(vels))
    idx_peak = int(np.argmax(vels))
    t_peak = float(times[idx_peak])

    # Acceleration phase stats
    mask_acc = (times >= args.wait_before) & (times < args.wait_before + args.accel_time)
    a_mean_acc = float(np.mean(accels_x[mask_acc])) if mask_acc.any() else 0.0

    # Deceleration phase stats
    mask_dec = times >= (args.wait_before + args.accel_time)
    a_mean_dec = float(np.mean(accels_x[mask_dec])) if mask_dec.any() else 0.0

    summary = {
        "experiment": 3,
        "description": "Safe accel / decel via throttle commands",
        "accel_throttle": args.accel_throttle,
        "accel_time_s": args.accel_time,
        "decel_time_s": args.decel_time,
        "v_peak_ms": round(v_peak, 5),
        "t_peak_s": round(t_peak, 3),
        "mean_accel_ms2": round(a_mean_acc, 4),
        "mean_decel_ms2": round(a_mean_dec, 4),
    }
    save_yaml(summary, f"{EXP_TAG}_summary{TAG}.yaml", results_dir=RESULTS_DIR)

    # Plot velocity vs time
    plot_calibration_map(
        times, vels,
        x_label="Time [s]",
        y_label="Velocity [m/s]",
        poly_deg=1,
        title="Exp 3 — Throttle Step: Accel then Coast",
        filename=f"{EXP_TAG}_vel_vs_time{TAG}.png",
        results_dir=RESULTS_DIR,
    )

    print(f"\n  v_peak = {v_peak:.4f} m/s  at t={t_peak:.2f} s")
    print(f"  mean accel = {a_mean_acc:.4f} m/s^2")
    print(f"  mean decel = {a_mean_dec:.4f} m/s^2")
    return summary


# ── Experiment 4 — Steering Sweep at constant throttle ───────────────────────

def run_exp4_steering_sweep(iface, sim_mode: bool):
    """
    Exp 4: Hold constant throttle while sweeping steering in a triangle
    pattern:  0 → +max_angle → -max_angle  over sweep_time seconds.

    Starting from zero lets the car reach steady speed before any
    lateral excitation.  Both sweep directions reveal hysteresis.

    Original F1/10 equivalent: experiment==4  (increase_servo_position)
      – There it swept the raw servo PWM value at constant eRPM.
      – Here we sweep the QCar steering command with a triangle wave.
    """
    print_section("Experiment 4 — Steering Triangle Sweep (constant throttle)")
    print(f"  Throttle    : {args.const_throttle}")
    print(f"  Max angle   : ±{args.max_angle}")
    print(f"  Warm-up     : {args.warmup_time} s  (straight line)")
    print(f"  Sweep time  : {args.sweep_time} s")
    print(f"  Pattern     : 0 → +{args.max_angle} → -{args.max_angle} → 0")

    csv_fn, cols = _make_csv_and_cols(EXP_TAG)
    dt = args.dt
    total_time = args.warmup_time + args.sweep_time
    n_steps = int(total_time / dt)

    times, steer_cmds, yaw_rates, velocities, phases = [], [], [], [], []

    with CSVLogger(csv_fn, cols, results_dir=RESULTS_DIR) as log:
        for step in range(n_steps):
            t_now = step * dt

            if t_now < args.warmup_time:
                # Warm-up: drive straight to build speed
                steer = 0.0
                phase = "WARMUP"
            else:
                t_sweep = t_now - args.warmup_time
                frac = t_sweep / args.sweep_time    # 0 → 1
                steer = _triangle_steer(frac, args.max_angle)
                if frac <= 0.25:
                    phase = "0→+max"
                elif frac <= 0.75:
                    phase = "+max→-max"
                else:
                    phase = "-max→0"

            if sim_mode:
                iface.send(args.const_throttle, steer, dt)
            else:
                iface.send(args.const_throttle, steer)

            state = iface.read_state()

            times.append(t_now)
            steer_cmds.append(steer)
            yaw_rates.append(state["yaw_rate"])
            velocities.append(state["velocity"])
            phases.append(phase)

            log.write(t_now, args.const_throttle, steer,
                      state["velocity"], state["yaw_rate"],
                      state["accel_x"], state["accel_y"])

            if not sim_mode:
                time.sleep(dt)

            if (step + 1) % int(1.0 / dt) == 0:
                print(f"  [{phase:>9s}] t={t_now+dt:.1f}s  steer={steer:.4f}  "
                      f"v={state['velocity']:.4f}  yr={state['yaw_rate']:.4f}")

            # Warn near end
            if t_now >= args.warmup_time:
                frac_check = (t_now - args.warmup_time) / args.sweep_time
                if frac_check >= 0.9 and step % int(1.0 / dt) == 0:
                    print("  [!] Sweep ending soon — prepare to stop bag recording")

    # ── Analysis ──────────────────────────────────────────────────────────
    times = np.array(times)
    steer_cmds = np.array(steer_cmds)
    yaw_rates = np.array(yaw_rates)
    velocities = np.array(velocities)
    phases = np.array(phases)

    # Separate sweep-only data (exclude warm-up)
    sweep_mask = phases != "WARMUP"
    v_min = 0.05
    valid = sweep_mask & (np.abs(velocities) > v_min)

    # Curvature = yaw_rate / velocity
    curvatures = np.zeros_like(yaw_rates)
    curvatures[valid] = yaw_rates[valid] / velocities[valid]

    # Phase masks for directional analysis
    phase_cfg = {
        "0→+max":  {"color": "royalblue",  "marker": "^", "label": "Phase 1: 0→+max"},
        "+max→-max": {"color": "tomato",    "marker": "o", "label": "Phase 2: +max→-max"},
        "-max→0":  {"color": "seagreen",   "marker": "v", "label": "Phase 3: -max→0"},
    }

    # Global polynomial fit (sweep-only, valid velocity)
    if np.sum(valid) > 4:
        poly_yr = np.polyfit(steer_cmds[valid], yaw_rates[valid], 3)
        poly_k = np.polyfit(steer_cmds[valid], curvatures[valid], 3)
    else:
        poly_yr = np.array([0.0])
        poly_k = np.array([0.0])

    # ── Plot 1: Time-series overview ──────────────────────────────────────
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

        # Color background bands by phase
        phase_colors = {"WARMUP": "#eeeeee", "0→+max": "#d0e0ff",
                        "+max→-max": "#ffe0d0", "-max→0": "#d0f0d0"}
        for ax in axes:
            prev_phase, seg_start = phases[0], times[0]
            for i in range(1, len(times)):
                if phases[i] != prev_phase or i == len(times) - 1:
                    ax.axvspan(seg_start, times[i], alpha=0.25,
                               color=phase_colors.get(prev_phase, "#ffffff"),
                               zorder=0)
                    prev_phase = phases[i]
                    seg_start = times[i]

        axes[0].plot(times, steer_cmds, "k-", linewidth=1.2)
        axes[0].set_ylabel("Steering cmd [-]")
        axes[0].set_title(f"Exp 4 — Triangle Sweep  (throttle={args.const_throttle})")

        axes[1].plot(times, yaw_rates, "b-", linewidth=1.0)
        axes[1].set_ylabel("Yaw rate [rad/s]")

        axes[2].plot(times, velocities, "g-", linewidth=1.0)
        axes[2].set_ylabel("Velocity [m/s]")
        axes[2].set_xlabel("Time [s]")

        for ax in axes:
            ax.grid(True, alpha=0.3)
        fig.tight_layout()
        path_ts = os.path.join(RESULTS_DIR, f"{EXP_TAG}_timeseries{TAG}.png")
        fig.savefig(path_ts, dpi=130)
        plt.close(fig)
        print(f"[plot] Saved → {path_ts}")
    except Exception as e:
        print(f"[plot] Time-series plot skipped: {e}")

    # ── Plot 2: Steering → Yaw Rate (phase-coloured + global fit) ────────
    try:
        fig, ax = plt.subplots(figsize=(8, 5))
        for pname, cfg in phase_cfg.items():
            m = (phases == pname) & valid
            if np.any(m):
                ax.scatter(steer_cmds[m], yaw_rates[m], s=18, alpha=0.6,
                           color=cfg["color"], marker=cfg["marker"],
                           label=cfg["label"], zorder=3)
        # Global poly fit curve
        if len(poly_yr) > 1:
            x_fit = np.linspace(steer_cmds[valid].min(),
                                steer_cmds[valid].max(), 200)
            ax.plot(x_fit, np.polyval(poly_yr, x_fit), "k--", linewidth=1.5,
                    label=f"Poly-3 fit (all)")
        ax.set_xlabel("Steering command [-]")
        ax.set_ylabel("Yaw rate [rad/s]")
        ax.set_title("Exp 4 — Steering → Yaw Rate  (phase-coloured)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        path_yr = os.path.join(RESULTS_DIR,
                               f"{EXP_TAG}_steer_vs_yawrate{TAG}.png")
        fig.savefig(path_yr, dpi=130)
        plt.close(fig)
        print(f"[plot] Saved → {path_yr}")
    except Exception as e:
        print(f"[plot] Steer-vs-yawrate plot skipped: {e}")

    # ── Plot 3: Steering → Curvature (phase-coloured + global fit) ───────
    try:
        fig, ax = plt.subplots(figsize=(8, 5))
        for pname, cfg in phase_cfg.items():
            m = (phases == pname) & valid
            if np.any(m):
                ax.scatter(steer_cmds[m], curvatures[m], s=18, alpha=0.6,
                           color=cfg["color"], marker=cfg["marker"],
                           label=cfg["label"], zorder=3)
        if len(poly_k) > 1:
            x_fit = np.linspace(steer_cmds[valid].min(),
                                steer_cmds[valid].max(), 200)
            ax.plot(x_fit, np.polyval(poly_k, x_fit), "k--", linewidth=1.5,
                    label=f"Poly-3 fit (all)")
        ax.set_xlabel("Steering command [-]")
        ax.set_ylabel("Curvature [1/m]")
        ax.set_title("Exp 4 — Steering → Curvature  (phase-coloured)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        path_k = os.path.join(RESULTS_DIR,
                              f"{EXP_TAG}_steer_vs_curvature{TAG}.png")
        fig.savefig(path_k, dpi=130)
        plt.close(fig)
        print(f"[plot] Saved → {path_k}")
    except Exception as e:
        print(f"[plot] Steer-vs-curvature plot skipped: {e}")

    # ── Per-phase statistics ──────────────────────────────────────────────
    phase_stats = {}
    for pname in ["0→+max", "+max→-max", "-max→0"]:
        m = (phases == pname) & valid
        if np.any(m):
            phase_stats[pname] = {
                "n_samples": int(np.sum(m)),
                "mean_velocity_ms": round(float(np.mean(velocities[m])), 5),
                "mean_yaw_rate_rads": round(float(np.mean(yaw_rates[m])), 5),
                "steer_range": [round(float(np.min(steer_cmds[m])), 4),
                                round(float(np.max(steer_cmds[m])), 4)],
            }

    # Warmup stats
    m_wu = phases == "WARMUP"
    warmup_exit_vel = float(velocities[m_wu][-1]) if np.any(m_wu) else 0.0

    summary = {
        "experiment": 4,
        "description": "Triangle steering sweep (0→+max→-max→0) at constant throttle",
        "const_throttle": args.const_throttle,
        "max_angle": args.max_angle,
        "sweep_pattern": f"0 → +{args.max_angle} → -{args.max_angle} → 0",
        "warmup_time_s": args.warmup_time,
        "warmup_exit_velocity_ms": round(warmup_exit_vel, 5),
        "sweep_time_s": args.sweep_time,
        "poly_steer_to_yawrate": [float(c) for c in poly_yr],
        "poly_steer_to_curvature": [float(c) for c in poly_k],
        "mean_velocity_ms": round(float(np.mean(velocities[sweep_mask])), 5),
        "phase_stats": phase_stats,
    }
    save_yaml(summary, f"{EXP_TAG}_summary{TAG}.yaml", results_dir=RESULTS_DIR)

    print(f"\n  Warmup exit velocity = {warmup_exit_vel:.4f} m/s")
    print(f"  Sweep mean velocity  = {summary['mean_velocity_ms']:.4f} m/s")
    for pname, ps in phase_stats.items():
        print(f"  [{pname:>9s}]  v̄={ps['mean_velocity_ms']:.4f}  "
              f"ẏ̄={ps['mean_yaw_rate_rads']:.5f}  "
              f"steer=[{ps['steer_range'][0]:.3f}, {ps['steer_range'][1]:.3f}]")
    return summary


# ── Experiment 5 — Velocity / Steering Sweep (tyre dynamics) ─────────────────

def run_exp5_velocity_steering_sweep(iface, sim_mode: bool):
    """
    Exp 5: Maintain a target velocity with a P-controller on throttle
    while sweeping the steering angle in a triangle pattern
    (0 → +max → -max).  Records lateral dynamics (yaw rate, lateral
    acceleration) needed for tyre friction fitting (→ analyse_tires.py).

    Starting from zero ensures stable velocity before lateral excitation.

    Original F1/10 equivalent: experiment==5  (increase_steering_angle)
      – There it commanded constant speed via AckermannDriveStamped while
        sweeping the steering_angle field.
      – Here we emulate "constant speed" with a simple P + feedforward
        throttle controller to keep velocity near ``target_velocity``.
    """
    print_section("Experiment 5 — Velocity / Triangle Steering Sweep")
    print(f"  Target vel  : {args.target_velocity} m/s")
    print(f"  Max angle   : ±{args.max_angle}")
    print(f"  Warm-up     : {args.warmup_time} s  (straight line)")
    print(f"  Sweep time  : {args.sweep_time} s")
    print(f"  Pattern     : 0 → +{args.max_angle} → -{args.max_angle} → 0")
    print(f"  Vel Kp      : {args.vel_kp}   FF : {args.vel_ff}")

    csv_fn = f"{EXP_TAG}_timeseries{TAG}.csv"
    cols = ["time_s", "throttle_cmd", "steering_cmd",
            "velocity_ms", "yaw_rate_rads", "accel_x", "accel_y",
            "slip_angle_est_rad"]
    dt = args.dt
    total_time = args.warmup_time + args.sweep_time
    n_steps = int(total_time / dt)

    times, steer_cmds, yaw_rates, velocities = [], [], [], []
    lat_accels, slip_angles_est, phases = [], [], []

    with CSVLogger(csv_fn, cols, results_dir=RESULTS_DIR) as log:
        for step in range(n_steps):
            t_now = step * dt

            if t_now < args.warmup_time:
                steer = 0.0
                phase = "WARMUP"
            else:
                frac = (t_now - args.warmup_time) / args.sweep_time
                steer = _triangle_steer(frac, args.max_angle)
                if frac <= 0.25:
                    phase = "0→+max"
                elif frac <= 0.75:
                    phase = "+max→-max"
                else:
                    phase = "-max→0"

            # Read current velocity (use previous reading for control)
            state = iface.read_state()
            v = state["velocity"]

            # P + FF velocity controller
            v_err = args.target_velocity - v
            thr = args.vel_ff * args.target_velocity + args.vel_kp * v_err
            thr = float(np.clip(thr, 0.0, 1.0))

            if sim_mode:
                iface.send(thr, steer, dt)
            else:
                iface.send(thr, steer)

            state = iface.read_state()
            v = state["velocity"]
            yr = state["yaw_rate"]
            a_y = state["accel_y"]

            # Estimate slip angle: beta ≈ atan(lr * yaw_rate / v)
            lr = args.wheelbase * 0.5  # rear axle to CG approx
            if abs(v) > 0.05:
                beta_est = np.arctan2(lr * yr, v)
            else:
                beta_est = 0.0

            times.append(t_now)
            steer_cmds.append(steer)
            yaw_rates.append(yr)
            velocities.append(v)
            lat_accels.append(a_y)
            slip_angles_est.append(beta_est)
            phases.append(phase)

            log.write(t_now, thr, steer, v, yr,
                      state["accel_x"], a_y, beta_est)

            if not sim_mode:
                time.sleep(dt)

            if (step + 1) % int(1.0 / dt) == 0:
                print(f"  [{phase:>9s}] t={t_now+dt:.1f}s  steer={steer:.4f}  "
                      f"v={v:.4f}  yr={yr:.4f}  beta={np.degrees(beta_est):.2f} deg")

            if t_now >= args.warmup_time:
                frac_check = (t_now - args.warmup_time) / args.sweep_time
                if frac_check >= 0.9 and step % int(1.0 / dt) == 0:
                    print("  [!] Sweep ending soon — stop bag recording")

    # ── Analysis ──────────────────────────────────────────────────────────
    times = np.array(times)
    steer_cmds = np.array(steer_cmds)
    yaw_rates = np.array(yaw_rates)
    velocities = np.array(velocities)
    lat_accels = np.array(lat_accels)
    slip_angles_est = np.array(slip_angles_est)
    phases = np.array(phases)

    # Separate sweep-only data (exclude warm-up)
    sweep_mask = phases != "WARMUP"
    v_min = 0.05
    valid = sweep_mask & (np.abs(velocities) > v_min)

    # Phase config for coloured plots
    phase_cfg = {
        "0→+max":    {"color": "royalblue", "marker": "^", "label": "Phase 1: 0→+max"},
        "+max→-max": {"color": "tomato",    "marker": "o", "label": "Phase 2: +max→-max"},
        "-max→0":    {"color": "seagreen",  "marker": "v", "label": "Phase 3: -max→0"},
    }

    # Global polynomial fits (sweep-only)
    if np.sum(valid) > 4:
        poly_yr = np.polyfit(steer_cmds[valid], yaw_rates[valid], 3)
        poly_tyre = np.polyfit(slip_angles_est[valid], lat_accels[valid], 3)
    else:
        poly_yr = np.array([0.0])
        poly_tyre = np.array([0.0])

    # ── Plot 1: Time-series overview ──────────────────────────────────────
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)

        # Color background bands by phase
        phase_colors = {"WARMUP": "#eeeeee", "0→+max": "#d0e0ff",
                        "+max→-max": "#ffe0d0", "-max→0": "#d0f0d0"}
        for ax in axes:
            prev_phase, seg_start = phases[0], times[0]
            for i in range(1, len(times)):
                if phases[i] != prev_phase or i == len(times) - 1:
                    ax.axvspan(seg_start, times[i], alpha=0.25,
                               color=phase_colors.get(prev_phase, "#ffffff"),
                               zorder=0)
                    prev_phase = phases[i]
                    seg_start = times[i]

        axes[0].plot(times, steer_cmds, "k-", linewidth=1.2)
        axes[0].set_ylabel("Steering cmd [-]")
        axes[0].set_title(f"Exp 5 — Vel/Steering Sweep  (target={args.target_velocity} m/s)")

        axes[1].plot(times, velocities, "g-", linewidth=1.0)
        axes[1].axhline(args.target_velocity, color="gray", ls=":", lw=0.8)
        axes[1].set_ylabel("Velocity [m/s]")

        axes[2].plot(times, yaw_rates, "b-", linewidth=1.0)
        axes[2].set_ylabel("Yaw rate [rad/s]")

        axes[3].plot(times, np.degrees(slip_angles_est), "m-", linewidth=1.0)
        axes[3].set_ylabel("Slip angle [deg]")
        axes[3].set_xlabel("Time [s]")

        for ax in axes:
            ax.grid(True, alpha=0.3)
        fig.tight_layout()
        path_ts = os.path.join(RESULTS_DIR, f"{EXP_TAG}_timeseries{TAG}.png")
        fig.savefig(path_ts, dpi=130)
        plt.close(fig)
        print(f"[plot] Saved → {path_ts}")
    except Exception as e:
        print(f"[plot] Time-series plot skipped: {e}")

    # ── Plot 2: Steering → Yaw Rate (phase-coloured) ─────────────────────
    try:
        fig, ax = plt.subplots(figsize=(8, 5))
        for pname, cfg in phase_cfg.items():
            m = (phases == pname) & valid
            if np.any(m):
                ax.scatter(steer_cmds[m], yaw_rates[m], s=18, alpha=0.6,
                           color=cfg["color"], marker=cfg["marker"],
                           label=cfg["label"], zorder=3)
        if len(poly_yr) > 1:
            x_fit = np.linspace(steer_cmds[valid].min(),
                                steer_cmds[valid].max(), 200)
            ax.plot(x_fit, np.polyval(poly_yr, x_fit), "k--", linewidth=1.5,
                    label="Poly-3 fit (all)")
        ax.set_xlabel("Steering command [-]")
        ax.set_ylabel("Yaw rate [rad/s]")
        ax.set_title("Exp 5 — Steering → Yaw Rate  (phase-coloured, const vel)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        path_yr = os.path.join(RESULTS_DIR,
                               f"{EXP_TAG}_steer_vs_yawrate{TAG}.png")
        fig.savefig(path_yr, dpi=130)
        plt.close(fig)
        print(f"[plot] Saved → {path_yr}")
    except Exception as e:
        print(f"[plot] Steer-vs-yawrate plot skipped: {e}")

    # ── Plot 3: Slip Angle → Lateral Accel (tyre curve, phase-coloured) ──
    try:
        fig, ax = plt.subplots(figsize=(8, 5))
        for pname, cfg in phase_cfg.items():
            m = (phases == pname) & valid
            if np.any(m):
                ax.scatter(slip_angles_est[m], lat_accels[m], s=18, alpha=0.6,
                           color=cfg["color"], marker=cfg["marker"],
                           label=cfg["label"], zorder=3)
        if len(poly_tyre) > 1 and np.any(valid):
            x_fit = np.linspace(slip_angles_est[valid].min(),
                                slip_angles_est[valid].max(), 200)
            ax.plot(x_fit, np.polyval(poly_tyre, x_fit), "k--", linewidth=1.5,
                    label="Poly-3 fit (all)")
        ax.set_xlabel("Estimated slip angle [rad]")
        ax.set_ylabel("Lateral acceleration [m/s²]")
        ax.set_title("Exp 5 — Slip Angle → Lat Accel  (tyre curve, phase-coloured)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        path_tyre = os.path.join(RESULTS_DIR,
                                 f"{EXP_TAG}_slip_vs_accel{TAG}.png")
        fig.savefig(path_tyre, dpi=130)
        plt.close(fig)
        print(f"[plot] Saved → {path_tyre}")
    except Exception as e:
        print(f"[plot] Slip-vs-accel plot skipped: {e}")

    # ── Per-phase statistics ──────────────────────────────────────────────
    phase_stats = {}
    for pname in ["0→+max", "+max→-max", "-max→0"]:
        m = (phases == pname) & valid
        if np.any(m):
            phase_stats[pname] = {
                "n_samples": int(np.sum(m)),
                "mean_velocity_ms": round(float(np.mean(velocities[m])), 5),
                "mean_yaw_rate_rads": round(float(np.mean(yaw_rates[m])), 5),
                "mean_lat_accel_ms2": round(float(np.mean(lat_accels[m])), 5),
                "steer_range": [round(float(np.min(steer_cmds[m])), 4),
                                round(float(np.max(steer_cmds[m])), 4)],
            }

    # Warmup stats
    m_wu = phases == "WARMUP"
    warmup_exit_vel = float(velocities[m_wu][-1]) if np.any(m_wu) else 0.0

    summary = {
        "experiment": 5,
        "description": "Triangle velocity/steering sweep (0→+max→-max→0) for tyre dynamics",
        "target_velocity_ms": args.target_velocity,
        "vel_kp": args.vel_kp,
        "vel_ff": args.vel_ff,
        "max_angle": args.max_angle,
        "sweep_pattern": f"0 → +{args.max_angle} → -{args.max_angle} → 0",
        "warmup_time_s": args.warmup_time,
        "warmup_exit_velocity_ms": round(warmup_exit_vel, 5),
        "sweep_time_s": args.sweep_time,
        "poly_steer_to_yawrate": [float(c) for c in poly_yr],
        "poly_slip_to_lat_accel": [float(c) for c in poly_tyre],
        "mean_velocity_ms": round(float(np.mean(velocities[sweep_mask])), 5),
        "phase_stats": phase_stats,
        "note": "CSV timeseries can be fed into analyse_tires.py for Pacejka fitting.",
    }
    save_yaml(summary, f"{EXP_TAG}_summary{TAG}.yaml", results_dir=RESULTS_DIR)

    print(f"\n  Warmup exit velocity = {warmup_exit_vel:.4f} m/s")
    print(f"  Sweep mean velocity  = {summary['mean_velocity_ms']:.4f} m/s")
    for pname, ps in phase_stats.items():
        print(f"  [{pname:>9s}]  v̄={ps['mean_velocity_ms']:.4f}  "
              f"ẏ̄={ps['mean_yaw_rate_rads']:.5f}  "
              f"ā_y={ps['mean_lat_accel_ms2']:.5f}  "
              f"steer=[{ps['steer_range'][0]:.3f}, {ps['steer_range'][1]:.3f}]")
    return summary


# ═════════════════════════════════════════════════════════════════════════════
#  Entry point
# ═════════════════════════════════════════════════════════════════════════════


def main():
    if args.sim:
        mode_label = "SIMULATION (Python math)"
    elif args.qlabs:
        mode_label = f"QLABS virtual QCar '{args.actor}'"
    else:
        mode_label = f"PHYSICAL {args.vehicle_type}"

    print_section(f"System-ID Experiment {args.experiment}: {EXP_NAMES[args.experiment]}")
    print(f"  Mode         : {mode_label}")
    print(f"  Vehicle type : {args.vehicle_type}")
    print(f"  dt           : {args.dt} s  ({1.0/args.dt:.0f} Hz)")
    print(f"  Wheelbase    : {args.wheelbase} m")

    # ── Build interface ───────────────────────────────────────────────────
    if args.sim:
        iface = SimulatedVehicle(wheelbase=args.wheelbase)
    else:
        iface = UnifiedHardwareInterface(
            vehicle_type=args.vehicle_type,
            qlabs_mode=args.qlabs,
            actor_name=args.actor,
        )
        iface.connect()

    sim_mode = args.sim

    # ── Run selected experiment ───────────────────────────────────────────
    try:
        if args.experiment == 1:
            run_exp1_steering_mapping(iface, sim_mode)
        elif args.experiment == 3:
            run_exp3_safe_accel_decel(iface, sim_mode)
        elif args.experiment == 4:
            run_exp4_steering_sweep(iface, sim_mode)
        elif args.experiment == 5:
            run_exp5_velocity_steering_sweep(iface, sim_mode)
    except KeyboardInterrupt:
        print("\n[!] Experiment interrupted by user.")
    finally:
        iface.stop()
        if not sim_mode:
            iface.close()
        else:
            iface.close()

    print(f"\n[OK] Experiment {args.experiment} complete.  "
          f"Results in: {RESULTS_DIR}")


if __name__ == "__main__":
    main()
