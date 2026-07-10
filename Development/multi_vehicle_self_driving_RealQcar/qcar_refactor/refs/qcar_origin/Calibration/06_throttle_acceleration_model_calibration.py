#!/usr/bin/env python3
"""
06_throttle_acceleration_model_calibration.py
=============================================
QCar Calibration - Throttle to Acceleration (Drag Model)
--------------------------------------------------------
Identifies parameters for a continuous acceleration model representing
both forward and reverse driving with aerodynamic/rolling drag and friction.

Mathematical Model (Discrete):
    u_th = max(u, 0)
    u_br = max(-u, 0)
    motor_accel_req = k_th * u_th - k_br * u_br

    # Motor lag (thrust builds up slowly)
    motor_accel[k+1] = motor_accel[k] + (dt / tau) * (motor_accel_req[k] - motor_accel[k])

    # Drag and Coulomb friction
    if abs(v[k]) > 0:
        drag_accel = c_v * v[k] + c_fric * sign(v[k])
    else:
        drag_accel = min(abs(motor_accel[k+1]), c_fric) * sign(motor_accel[k+1])

    # True kinematic acceleration (what an IMU would measure)
    kinematic_accel[k+1] = motor_accel[k+1] - drag_accel

    # Velocity update
    v[k+1] = v[k] + kinematic_accel[k+1] * dt

Parameters identified globally across a sequence of driving commands:
  - k_th  : Forward throttle to motor thrust gain (m/s^2 per u)
  - k_br  : Reverse throttle to motor thrust gain (m/s^2 per u)
  - tau   : Motor time constant (s), first-order lag
  - c_v   : Linear drag coefficient (1/s)
  - c_fric: Coulomb friction equivalent acceleration (m/s^2)

Supports THREE modes:
  --sim      : purely synthetic data using the exact equations + noise
  --qlabs    : QLabs virtual QCar via readRobots()
  (default)  : physical QCar hardware
"""

import argparse
import csv
import os
import sys
import time
from typing import Dict, List, Tuple

import yaml
import numpy as np
from scipy.optimize import minimize

_CAL_DIR = os.path.dirname(os.path.abspath(__file__))
_QCAR_DIR = os.path.dirname(_CAL_DIR)
for _p in [_CAL_DIR, _QCAR_DIR]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

RESULTS_DIR = os.path.join(
    _CAL_DIR, "results", "06_throttle_acceleration_model_calibration"
)
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


class AccelDragMotorSim:
    """Simulation model using the requested motor thrust and drag/friction physics."""

    def __init__(
        self, tau=0.2, k_th=6.0, k_br=6.0, c_v=0.5, c_fric=0.1, noise_std=0.01
    ):
        self.tau = tau
        self.k_th = k_th
        self.k_br = k_br
        self.c_v = c_v
        self.c_fric = c_fric
        self.noise_std = noise_std
        self.v = 0.0
        self.motor_accel = 0.0

    def step(self, u: float, dt: float) -> float:
        u_th = max(u, 0)
        u_br = max(-u, 0)
        motor_accel_req = self.k_th * u_th - self.k_br * u_br

        # motor_accel_k+1
        self.motor_accel = self.motor_accel + (dt / self.tau) * (
            motor_accel_req - self.motor_accel
        )

        # Friction logic (prevent friction from causing reverse movement when near zero)
        drag_accel = self.c_v * self.v
        if abs(self.v) > 0.01:
            drag_accel += self.c_fric * np.sign(self.v)
        else:
            # If stationary, friction opposes motor thrust up to c_fric
            if abs(self.motor_accel) <= self.c_fric:
                drag_accel += self.motor_accel
            else:
                drag_accel += self.c_fric * np.sign(self.motor_accel)

        kinematic_accel = self.motor_accel - drag_accel

        # v_k+1
        self.v = self.v + kinematic_accel * dt

        # Make sure v doesn't artificially cross zero if it was zero and thrust is too low
        if abs(self.v) < 0.001 and abs(self.motor_accel) <= self.c_fric:
            self.v = 0.0

        return float(self.v + np.random.randn() * self.noise_std)

    def reset(self):
        self.v = 0.0
        self.motor_accel = 0.0


class QCarHardwareInterface:
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
                raise RuntimeError(f"Actor '{self._actor_name}' not found in QLabs.")
            hil_port = robots[self._actor_name].get("hilPort")
            self._qcar = QCar(readMode=1, hilPort=hil_port)
            print(
                f"[QLABS] Connected to virtual QCar '{self._actor_name}' (hilPort={hil_port})"
            )
        else:
            self._qcar = QCar(readMode=1)
            print("[HW] Physical QCar connected.")

    def send_throttle(self, throttle: float, steering: float = 0.0):
        self._qcar.write(
            throttle=float(np.clip(throttle, -1.0, 1.0)),
            steering=float(np.clip(steering, -0.5, 0.5)),
        )

    def read_velocity(self) -> float:
        self._qcar.read()
        return float(getattr(self._qcar, "motorTach", 0.0))

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


def print_section(title: str):
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)


def simulate_trajectory(times, throttles, k_th, k_br, tau, c_v, c_fric):
    """Simulate the velocity history given a parameter set and measured command history."""
    n = len(times)
    sim_v = np.zeros(n)
    sim_motor_accel = np.zeros(n)
    sim_kinematic_accel = np.zeros(n)

    if n == 0:
        return sim_v, sim_motor_accel, sim_kinematic_accel

    motor_accel = 0.0
    v = 0.0
    sim_v[0] = v
    sim_motor_accel[0] = motor_accel
    sim_kinematic_accel[0] = 0.0

    for i in range(1, n):
        dt = times[i] - times[i - 1]
        if dt <= 0:
            dt = 1e-6

        u = throttles[i - 1]
        u_th = max(u, 0)
        u_br = max(-u, 0)
        motor_accel_req = k_th * u_th - k_br * u_br

        motor_accel = motor_accel + (dt / tau) * (motor_accel_req - motor_accel)

        # Apply Drag & Friction
        drag_accel = c_v * v
        if abs(v) > 0.01:
            drag_accel += c_fric * np.sign(v)
        else:
            if abs(motor_accel) <= c_fric:
                drag_accel += motor_accel
            else:
                drag_accel += c_fric * np.sign(motor_accel)

        kinematic_accel = motor_accel - drag_accel
        v = v + kinematic_accel * dt

        # Prevent floating point zero crossings due to static friction
        if abs(v) < 0.001 and abs(motor_accel) <= c_fric:
            v = 0.0

        sim_v[i] = v
        sim_motor_accel[i] = motor_accel
        sim_kinematic_accel[i] = kinematic_accel

    return sim_v, sim_motor_accel, sim_kinematic_accel


def objective_function(params, times, throttles, measured_v):  # , c_fric bound
    k_th, k_br, tau, c_v, c_fric = params
    sim_v, _, _ = simulate_trajectory(times, throttles, k_th, k_br, tau, c_v, c_fric)
    return np.mean((sim_v - measured_v) ** 2)


def main():
    parser = argparse.ArgumentParser(
        description="QCar motor thrust and drag model calibration"
    )
    parser.add_argument("--sim", action="store_true", help="Pure Python simulation")
    parser.add_argument(
        "--qlabs", action="store_true", help="Connect to QLabs virtual QCar"
    )
    parser.add_argument("--actor", type=str, default="QC2_0", help="QLabs actor name")
    parser.add_argument(
        "--dt", type=float, default=0.02, help="Control loop timestep [s]"
    )
    parser.add_argument(
        "--step_time", type=float, default=2.0, help="Hold time per step [s]"
    )
    parser.add_argument("--tag", type=str, default="", help="Optional output tag")
    parser.add_argument("--no_plot", action="store_true", help="Skip plotting results")
    args = parser.parse_args()

    if args.sim and args.qlabs:
        print("[ERROR] Cannot use --sim and --qlabs together.")
        sys.exit(1)

    TAG = (
        f"_{args.tag}"
        if args.tag
        else ("_sim" if args.sim else ("_qlabs" if args.qlabs else "_hardware"))
    )

    # Define the sequence of throttle commands to test
    # We include a 'double-tap' reverse pattern (forward -> 0 -> negative -> 0 -> negative)
    # properly engage the ESC's reverse mode instead of getting stuck in brake mode.
    SEQUENCE = [0.0, 0.1, 0.2, 0.3, 0.2, 0.0, -0.2, 0.0, -0.1, -0.2, -0.3, -0.2, 0.0]

    print_section(f"Starting Calibration Sequence (Mode: {TAG[1:].upper()})")
    print(f"Sequence: {SEQUENCE}")

    if args.sim:
        interface = AccelDragMotorSim(tau=0.25, k_th=5.0, k_br=4.5, c_v=0.4, c_fric=0.1)
    else:
        interface = QCarHardwareInterface(qlabs_mode=args.qlabs, actor_name=args.actor)
        interface.connect()

    times = []
    throttles = []
    velocities = []

    raw_csv = f"throttle_drag_model_raw{TAG}.csv"

    every_sec_steps = max(1, int(round(1.0 / args.dt)))
    t_global0 = time.time()

    try:
        with CSVLogger(raw_csv, ["time_s", "throttle_cmd", "velocity_mps"]) as raw_log:
            for step_idx, u_target in enumerate(SEQUENCE):
                print(
                    f"  Step {step_idx + 1}/{len(SEQUENCE)}: command u={u_target:.2f}"
                )

                step_steps = max(1, int(round(args.step_time / args.dt)))
                for k in range(step_steps):
                    if args.sim:
                        v = interface.step(u_target, args.dt)
                    else:
                        interface.send_throttle(u_target)
                        v = interface.read_velocity()

                    elapsed = time.time() - t_global0

                    times.append(elapsed)
                    throttles.append(u_target)
                    velocities.append(v)

                    raw_log.write(elapsed, u_target, v)

                    if not args.sim:
                        time.sleep(args.dt)

                    if (k + 1) % every_sec_steps == 0:
                        print(f"    t={elapsed:4.1f}s  v={v: .4f} m/s")

    finally:
        if args.sim:
            interface.reset()
        else:
            interface.stop()

    print_section("Optimization")
    t_arr = np.array(times)
    u_arr = np.array(throttles)
    v_arr = np.array(velocities)

    # Re-zero time array
    t_arr = t_arr - t_arr[0]

    # Initial guess: [k_th, k_br, tau, c_v, c_fric]
    initial_guess = [5.0, 5.0, 0.2, 0.5, 0.1]
    bounds = [
        (0.1, 20.0),  # k_th
        (0.1, 20.0),  # k_br
        (0.01, 2.0),  # tau
        (0.01, 5.0),  # c_v
        (0.0, 2.0),  # c_fric
    ]

    print("Running least squares optimization...")
    res = minimize(
        objective_function,
        initial_guess,
        args=(t_arr, u_arr, v_arr),
        bounds=bounds,
        method="L-BFGS-B",
    )

    if res.success:
        print("[OK] Optimization successful!")
    else:
        print(f"[WARN] Optimization might have issues: {res.message}")

    best_k_th, best_k_br, best_tau, best_c_v, best_c_fric = res.x

    print(f"\nIdentified Parameters:")
    print(f"  k_th   : {best_k_th:.4f} m/s^2 per u")
    print(f"  k_br   : {best_k_br:.4f} m/s^2 per u")
    print(f"  tau    : {best_tau:.4f} s")
    print(f"  c_v    : {best_c_v:.4f} 1/s  (linear drag)")
    print(f"  c_fric : {best_c_fric:.4f} m/s^2 (Coulomb friction)")

    # Evaluate fit quality
    sim_v, sim_motor_a, sim_kinematic_a = simulate_trajectory(
        t_arr, u_arr, best_k_th, best_k_br, best_tau, best_c_v, best_c_fric
    )
    rmse = np.sqrt(np.mean((sim_v - v_arr) ** 2))
    print(f"  RMSE   : {rmse:.4f} m/s")

    # Save to YAML
    model_yaml = {
        "description": "Motor thrust and kinematic drag continuous model",
        "mode": "simulation" if args.sim else ("qlabs" if args.qlabs else "hardware"),
        "model": {
            "motor_accel_req": "k_th * max(u, 0) - k_br * max(-u, 0)",
            "motor_accel_dyn": "motor_accel[k+1] = motor_accel[k] + (dt / tau) * (motor_accel_req - motor_accel[k])",
            "kinematic_accel": "motor_accel[k+1] - c_v * v[k] - c_fric * sign(v[k])",
            "velocity_dyn": "v[k+1] = v[k] + kinematic_accel * dt",
        },
        "identified_parameters": {
            "k_th": float(best_k_th),
            "k_br": float(best_k_br),
            "tau": float(best_tau),
            "c_v": float(best_c_v),
            "c_fric": float(best_c_fric),
            "rmse_velocity_mps": float(rmse),
        },
        "sequence_tested": SEQUENCE,
    }

    yaml_name = f"throttle_drag_model{TAG}.yaml"
    yaml_path = os.path.join(RESULTS_DIR, yaml_name)
    with open(yaml_path, "w") as f:
        yaml.dump(model_yaml, f, sort_keys=False)
    print(f"[YAML] Saved parameters to {yaml_path}")

    # Save canonical generic file
    canon_path = os.path.join(RESULTS_DIR, "throttle_drag_model.yaml")
    with open(canon_path, "w") as f:
        yaml.dump(model_yaml, f, sort_keys=False)

    # Plot
    if not args.no_plot:
        try:
            import matplotlib.pyplot as plt

            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

            ax1.plot(t_arr, v_arr, "k.", label="Measured v", alpha=0.5)
            ax1.plot(t_arr, sim_v, "r-", label="Simulated v (Fitted)", linewidth=2)
            ax1.set_ylabel("Velocity [m/s]")
            ax1.grid(True)
            ax1.legend(loc="upper right")
            ax1.set_title(
                f"Fit RMSE: {rmse:.3f} m/s\n"
                f"k_th={best_k_th:.1f}, k_br={best_k_br:.1f}, tau={best_tau:.2f}s, "
                f"c_v={best_c_v:.2f}, c_fric={best_c_fric:.2f}"
            )

            ax2.plot(t_arr, u_arr, "b--", label="Throttle cmd (u)")
            ax2.plot(t_arr, sim_motor_a, "g-", label="Motor Thrust [m/s^2]")
            ax2.plot(
                t_arr,
                sim_kinematic_a,
                "m-",
                label="Kinematic Accel (IMU) [m/s^2]",
                alpha=0.7,
            )
            ax2.set_ylabel("Command / Accel")
            ax2.set_xlabel("Time [s]")
            ax2.grid(True)
            ax2.legend(loc="upper right")

            plt.tight_layout()
            plot_path = os.path.join(RESULTS_DIR, f"throttle_drag_model_plot{TAG}.png")
            plt.savefig(plot_path)
            print(f"[PLOT] Saved summary figure to {plot_path}")
            plt.show()

        except ImportError:
            print("[WARN] matplotlib not installed, skipping plot.")

    print("\n[OK] Complete.")


if __name__ == "__main__":
    main()
