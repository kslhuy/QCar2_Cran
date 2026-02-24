#!/usr/bin/env python3
"""
01_throttle_velocity_calibration.py
=====================================
QCar Calibration – Throttle → Velocity Mapping
------------------------------------------------
Runs a staircase sequence of throttle commands on the QCar and records the
resulting steady-state velocity at each level.

Supports THREE modes:
  --sim      : pure Python first-order motor model  (no QLabs, no hardware)
  --qlabs    : QLabs virtual QCar via readRobots()  (run initPlatoon.py first)
  (default)  : physical QCar hardware

Usage (Python sim):
    python 01_throttle_velocity_calibration.py --sim

Usage (QLabs virtual):
    python 01_throttle_velocity_calibration.py --qlabs --actor QC2_0

Usage (real hardware):
    python 01_throttle_velocity_calibration.py --throttle_levels 0.02,0.04,0.06,0.08,0.1,0.15,0.2

Output:
    results/throttle_velocity_map.csv
    results/throttle_velocity_map.png
    results/throttle_velocity_poly.yaml   ← polynomial coefficients v = f(u)
"""

import argparse
import sys
import os
import time
import numpy as np

# ── Add project root to path ──────────────────────────────────────────────────
_CAL_DIR = os.path.dirname(os.path.abspath(__file__))
_QCAR_DIR = os.path.dirname(_CAL_DIR)
for _p in [_CAL_DIR, _QCAR_DIR]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from calibration_utils import (
    CSVLogger,
    FirstOrderMotorSim,
    plot_calibration_map,
    save_yaml,
    print_section,
)

# ── CLI arguments ─────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description="QCar throttle → velocity calibration")
parser.add_argument(
    "--sim",
    action="store_true",
    help="Pure Python first-order model (no QLabs, no hardware)",
)
parser.add_argument(
    "--qlabs",
    action="store_true",
    help="Connect to a QLabs virtual QCar. Run initPlatoon.py first.",
)
parser.add_argument(
    "--actor",
    type=str,
    default="QC2_0",
    help="QLabs actor name, e.g. QC2_0, QC2_1 (used with --qlabs)",
)
parser.add_argument(
    "--throttle_levels",
    type=str,
    default="0.1,0.15,0.2,0.25,0.3,0.35",
    help="Comma-separated throttle command levels to test",
)
parser.add_argument(
    "--settle_time",
    type=float,
    default=10.0,
    help="Time (s) to wait at each throttle level for velocity to settle",
)
parser.add_argument(
    "--measure_time",
    type=float,
    default=2.0,
    help="Time (s) to average velocity at end of settle period",
)
parser.add_argument(
    "--dt",
    type=float,
    default=0.02,
    help="Control loop time step [s] (default 20ms = 50Hz)",
)
parser.add_argument(
    "--poly_deg", type=int, default=2, help="Degree of polynomial fit v = f(throttle)"
)
parser.add_argument(
    "--tag", type=str, default="", help="Optional tag appended to output filenames"
)
args = parser.parse_args()

SCRIPT_RESULTS_DIR = os.path.join(
    _CAL_DIR, "results", "01_throttle_velocity_calibration"
)

# Controller tuning notes:
# - Use this map to estimate PID feedforward slope (throttle per m/s).
# - The smallest throttle that yields stable nonzero speed is a good
#   starting point for pid.min_throttle in config_controller_*.yaml.
# - Keep tested levels within your intended gear limit (DRIVE_1/2/3).

# ── Parse throttle levels ──────────────────────────────────────────────────────
# ── Parse throttle levels ──────────────────────────────────────────────────────
throttle_levels = [float(x) for x in args.throttle_levels.split(",")]

# Determine tag based on mode (overwrites previous runs unless --tag is manually set)
if args.tag:
    tag = f"_{args.tag}"
elif args.sim:
    tag = "_sim"
elif args.qlabs:
    tag = "_qlabs"
else:
    tag = "_hardware"


# ─────────────────────────────────────────────────────────────────────────────
#  Hardware interface wrapper
# ─────────────────────────────────────────────────────────────────────────────


class QCarHardwareInterface:
    """
    Unified QCar interface for three modes:
      Physical hardware (default) : QCar(readMode=1)  – real car
      QLabs virtual (--qlabs)     : QCar(readMode=1, hilPort=...)  – virtual car
      Python sim (--sim)          : FirstOrderMotorSim  – no QCar object at all
    """

    def __init__(self, qlabs_mode: bool = False, actor_name: str = "QC2_0"):
        self._qcar = None
        self._qlabs_mode = qlabs_mode
        self._actor_name = actor_name

    def connect(self):
        try:
            from pal.products.qcar import QCar

            if self._qlabs_mode:
                # QLabs: discover the virtual car's HIL port from readRobots()
                from qvl.multi_agent import readRobots

                robots = readRobots()
                if self._actor_name not in robots:
                    raise RuntimeError(
                        f"Actor '{self._actor_name}' not found in QLabs. "
                        f"Available: {list(robots.keys())}. "
                        f"Did initPlatoon.py run successfully?"
                    )
                hil_port = robots[self._actor_name].get("hilPort")
                if hil_port is None:
                    raise RuntimeError(
                        f"hilPort=None for '{self._actor_name}'. "
                        f"The QLabs real-time model may not have started."
                    )
                self._qcar = QCar(readMode=1, hilPort=hil_port)
                print(
                    f"[QLABS] Connected to virtual QCar '{self._actor_name}' "
                    f"(hilPort={hil_port})"
                )
            else:
                self._qcar = QCar(readMode=1)
                print("[HW] Physical QCar connected.")
        except Exception as exc:
            print(f"[ERROR] Connection failed: {exc}")
            raise

    def send_throttle(self, throttle: float, steering: float = 0.0):
        if self._qcar:
            self._qcar.write(
                throttle=float(np.clip(throttle, -1.0, 1.0)),
                steering=float(np.clip(steering, -0.5, 0.5)),
            )

    def read_velocity(self) -> float:
        """Return velocity from encoder/virtual tachometer [m/s]."""
        if self._qcar:
            self._qcar.read()
            return float(getattr(self._qcar, "motorTach", 0.0))
        return 0.0

    def stop(self):
        if self._qcar:
            self.send_throttle(0.0)
            time.sleep(0.5)
            try:
                self._qcar.terminate()
            except Exception:
                pass
            label = "QLABS" if self._qlabs_mode else "HW"
            print(f"[{label}] QCar disconnected.")


# ─────────────────────────────────────────────────────────────────────────────
#  Main calibration routine
# ─────────────────────────────────────────────────────────────────────────────


def run_calibration(interface, sim_mode: bool):
    """
    Execute the staircase throttle test and return arrays of
    (throttle_cmds, v_ss, v_std).
    """
    dt = args.dt
    settle_steps = int(args.settle_time / dt)
    measure_steps = int(args.measure_time / dt)

    throttle_cmds = []
    v_steady_means = []
    v_steady_stds = []

    csv_filename = f"throttle_velocity_map{tag}.csv"
    raw_filename = f"throttle_velocity_time_series{tag}.csv"

    # Open both summary logger and raw data logger
    with (
        CSVLogger(
            csv_filename,
            ["throttle_cmd", "v_ss_mean", "v_ss_std", "v_min", "v_max"],
            results_dir=SCRIPT_RESULTS_DIR,
        ) as summary_log,
        CSVLogger(
            raw_filename,
            ["time_s", "throttle_cmd", "velocity_ms"],
            results_dir=SCRIPT_RESULTS_DIR,
        ) as raw_log,
    ):
        start_time = time.time()

        for throttle in throttle_levels:
            print_section(f"Testing throttle = {throttle:.3f}")

            # --- Apply throttle and wait ---
            velocities_settle = []
            for step in range(settle_steps):
                if sim_mode:
                    v = interface.step(throttle, dt)
                else:
                    interface.send_throttle(throttle)
                    v = interface.read_velocity()

                # Log raw data
                elapsed_total = time.time() - start_time
                raw_log.write(elapsed_total, throttle, v)

                velocities_settle.append(v)
                time.sleep(dt)

                # Live feedback every second
                if (step + 1) % int(1.0 / dt) == 0:
                    elapsed = (step + 1) * dt
                    print(f"  t={elapsed:.1f}s  v={v:.4f} m/s")

            # --- Measure at steady state (last measure_steps) ---
            v_ss_samples = velocities_settle[-measure_steps:]
            v_ss = float(np.mean(v_ss_samples))
            v_std = float(np.std(v_ss_samples))
            v_min_ = float(np.min(v_ss_samples))
            v_max_ = float(np.max(v_ss_samples))

            print(f"  ✓ v_ss = {v_ss:.4f} m/s  (±{v_std:.4f})")

            throttle_cmds.append(throttle)
            v_steady_means.append(v_ss)
            v_steady_stds.append(v_std)
            summary_log.write(throttle, v_ss, v_std, v_min_, v_max_)

            # --- Coast back to zero before next level ---
            if not sim_mode:
                interface.send_throttle(0.0)
                time.sleep(1.0)
            elif hasattr(interface, "reset"):
                interface.reset()

    return np.array(throttle_cmds), np.array(v_steady_means), np.array(v_steady_stds)


def summarise_results(throttle_cmds, v_means, poly_coeffs):
    print_section("Calibration Summary")
    print(f"  {'Throttle':>10}  {'v_ss (m/s)':>12}")
    print(f"  {'-' * 10}  {'-' * 12}")
    for u, v in zip(throttle_cmds, v_means):
        print(f"  {u:10.3f}  {v:12.4f}")

    print(f"\n  Polynomial fit (deg {args.poly_deg}): v = ", end="")
    terms = []
    for i, c in enumerate(poly_coeffs):
        power = len(poly_coeffs) - 1 - i
        if power == 0:
            terms.append(f"{c:.4f}")
        elif power == 1:
            terms.append(f"{c:.4f}·u")
        else:
            terms.append(f"{c:.4f}·u^{power}")
    print(" + ".join(terms))


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────


def main():
    # Exactly one mode must be active
    if args.sim and args.qlabs:
        print("[ERROR] Cannot use --sim and --qlabs at the same time. Pick one.")
        sys.exit(1)

    if args.sim:
        mode_label = "SIMULATION (Python math)"
    elif args.qlabs:
        mode_label = f"QLABS virtual QCar '{args.actor}'"
    else:
        mode_label = "PHYSICAL hardware"

    print_section("QCar Throttle → Velocity Calibration")
    print(f"  Mode      : {mode_label}")
    print(f"  Levels    : {throttle_levels}")
    print(f"  Settle    : {args.settle_time}s  Measure: {args.measure_time}s")

    if args.sim:
        print()
        print("  NOTE: --sim uses a pure Python first-order model.")
        print("        For QLabs, use --qlabs (after running initPlatoon.py).")

    # --- Build interface ---
    if args.sim:
        interface = FirstOrderMotorSim(tau=0.8, K=5.0, noise_std=0.01)
    else:
        interface = QCarHardwareInterface(
            qlabs_mode=args.qlabs,
            actor_name=args.actor,
        )
        interface.connect()

    try:
        throttle_cmds, v_means, v_stds = run_calibration(interface, args.sim)
    finally:
        if not args.sim:
            interface.stop()

    if len(v_means) < 2:
        print("[ERROR] Not enough data points to fit a polynomial. Exiting.")
        sys.exit(1)

    # --- Polynomial fit ---
    _, poly_coeffs = plot_calibration_map(
        throttle_cmds,
        v_means,
        x_label="Throttle command [−]",
        y_label="Steady-state velocity [m/s]",
        poly_deg=args.poly_deg,
        title=f"QCar: Throttle → Velocity {'(SIM)' if args.sim else ''}",
        filename=f"throttle_velocity_map{tag}.png",
        results_dir=SCRIPT_RESULTS_DIR,
    )

    summarise_results(throttle_cmds, v_means, poly_coeffs)

    # --- Save polynomial coefficients ---
    result_data = {
        "description": "Throttle → velocity polynomial fit. v = poly(throttle)",
        "mode": "simulation" if args.sim else "hardware",
        "poly_degree": args.poly_deg,
        "coefficients": [float(c) for c in poly_coeffs],
        "usage": "v_ss = np.polyval(coefficients, throttle_cmd)",
        "measured_points": {
            "throttle": throttle_cmds.tolist(),
            "v_ss_mean": v_means.tolist(),
        },
    }
    save_yaml(
        result_data, f"throttle_velocity_poly{tag}.yaml", results_dir=SCRIPT_RESULTS_DIR
    )

    print("\n[✓] Calibration complete.")


if __name__ == "__main__":
    main()
