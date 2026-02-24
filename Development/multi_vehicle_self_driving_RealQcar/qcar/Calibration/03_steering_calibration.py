#!/usr/bin/env python3
"""
03_steering_calibration.py
===========================
QCar Calibration – Steering Command → Curvature / Turning Radius
-----------------------------------------------------------------
Drives the QCar at a constant low speed with each steering command level and
measures the resulting yaw-rate to compute turning radius and curvature.

Also extracts the effective wheelbase by fitting the Ackermann model:
    δ_eff = arctan(L_eff / R)

Supports --sim mode using a kinematic bicycle model (no hardware needed).

Usage (Python sim):
    python 03_steering_calibration.py --sim

Usage (QLabs virtual):
    python 03_steering_calibration.py --qlabs --actor QC2_0 --throttle_levels 0.1 --interactive

Usage (real hardware):
python 03_steering_calibration.py --sim --interactive --run_time 0.1 --settle_time 0.1 --throttle_levels 0.1 --steer_levels 0.1
Output:
    results/steering_curvature_map.csv
    results/steering_curvature_map.png
    results/steering_calibration.yaml   ← effective wheelbase, poly fit
"""

import argparse
import sys
import os
import time
import numpy as np
import threading
import queue

_CAL_DIR = os.path.dirname(os.path.abspath(__file__))
_QCAR_DIR = os.path.dirname(_CAL_DIR)
for _p in [_CAL_DIR, _QCAR_DIR]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from calibration_utils import (
    CSVLogger,
    BicycleSteeringSim,
    FirstOrderMotorSim,
    plot_calibration_map,
    save_yaml,
    print_section,
)

# ── CLI arguments ─────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description="QCar steering → curvature calibration")
parser.add_argument(
    "--sim",
    action="store_true",
    help="Use simulated kinematic bicycle model (no hardware)",
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
    help="QLabs actor name, e.g. QC2_0, QC2_1 (used with --qlabs)",
)
parser.add_argument(
    "--steer_levels",
    type=str,
    default="-0.1,0.1,-0.2,0.2,-0.3,0.3,-0.4,0.4",
    help="Comma-separated steering command values to test",
)
parser.add_argument(
    "--throttle_levels",
    type=str,
    default="0.2",
    help="Comma-separated throttle command values to test",
)
parser.add_argument(
    "--speed_sim",
    type=float,
    default=0.30,
    help="Constant velocity in simulation mode [m/s]",
)
parser.add_argument(
    "--run_time", type=float, default=10.0, help="Duration [s] of each steering level"
)
parser.add_argument(
    "--settle_time",
    type=float,
    default=0.5,
    help="Initial settling time [s] before measuring yaw-rate",
)
parser.add_argument("--dt", type=float, default=0.02, help="Control loop time step [s]")
parser.add_argument(
    "--wheelbase_nom",
    type=float,
    default=0.256,
    help="Nominal wheelbase [m] for model comparison",
)
parser.add_argument(
    "--poly_deg",
    type=int,
    default=3,
    help="Degree of polynomial fit  delta_cmd → curvature",
)
parser.add_argument(
    "--tag", type=str, default="", help="Optional tag for output filenames"
)
parser.add_argument(
    "--interactive",
    "-i",
    action="store_true",
    help="Enable interactive mode to approve/retry/modify each test point",
)
args = parser.parse_args()

SCRIPT_RESULTS_DIR = os.path.join(_CAL_DIR, "results", "03_steering_calibration")

# Controller tuning notes:
# - Use steering_cmd -> curvature fit to validate PP-map turn behavior.
# - L_eff helps align wheelbase assumptions used by the lateral controller.
# - If turns are too aggressive/slow, recalibrate here before changing
#   PP-map gains like m_l1/q_l1 or hard_turn_kappa.

steer_levels = [float(x) for x in args.steer_levels.split(",")]
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
#  Hardware interface
# ─────────────────────────────────────────────────────────────────────────────


class QCarSteeringHardwareInterface:
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
            if hil_port is None:
                raise RuntimeError("hilPort is None for virtual car.")
            self._qcar = QCar(readMode=1, hilPort=hil_port)
            print(
                f"[QLABS] Connected to virtual QCar '{self._actor_name}' (hilPort={hil_port})"
            )
        else:
            self._qcar = QCar(readMode=1)
            print("[HW] Physical QCar connected.")

    def send(self, throttle: float, steering: float):
        self._qcar.write(
            throttle=float(np.clip(throttle, -1.0, 1.0)),
            steering=float(np.clip(steering, -0.5, 0.5)),
        )

    def read_state(self):
        """Return (velocity_ms, yaw_rate_rads, heading_rad)."""
        self._qcar.read()
        v = float(getattr(self._qcar, "motorTach", 0.0))
        # Prefer IMU if available, fall back to encoder-derived yaw-rate
        yaw_rate = float(getattr(self._qcar, "gyroscope", [0, 0, 0])[2])
        # heading not directly available from motorTach; integrate yaw_rate
        heading = 0.0
        return v, yaw_rate, heading

    def stop(self):
        self.send(0.0, 0.0)
        time.sleep(0.5)
        # if self._qcar:
        #     self._qcar.terminate()
        print("[HW] QCar stopped.")


# ─────────────────────────────────────────────────────────────────────────────
#  Data collection
# ─────────────────────────────────────────────────────────────────────────────


# ─────────────────────────────────────────────────────────────────────────────
#  Data collection
# ─────────────────────────────────────────────────────────────────────────────


def _get_interactive_input(prompt: str, hw_interface, sim_mode: bool) -> str:
    """Non-blocking input to keep hardware/QLabs connection alive (heartbeat)."""
    print(prompt, end="", flush=True)
    q = queue.Queue()

    def _read_kbd():
        try:
            line = sys.stdin.readline()
            q.put(line.strip().lower() if line else "q")
        except EOFError:
            q.put("q")

    t = threading.Thread(target=_read_kbd, daemon=True)
    t.start()

    while t.is_alive():
        if not sim_mode and hw_interface is not None:
            # Send 0.0 commands and flush receive buffer to prevent timeout
            hw_interface.send(0.0, 0.0)
            try:
                hw_interface.read_state()
            except Exception:
                pass
        t.join(0.02)  # wait 20ms

    try:
        return q.get_nowait()
    except queue.Empty:
        return ""


def collect_steering_data(hw_interface, motor_sim, steer_sim, sim_mode: bool):
    """
    Return arrays: (steer_cmds, curvatures, radii, yaw_rates_mean, velocities_mean, throttle_cmds)
    """
    dt = args.dt

    # We will build these lists dynamically
    all_steer_cmds, all_curvatures, all_radii = [], [], []
    all_yaw_means, all_vel_means, all_throttle_cmds = [], [], []

    csv_fn = f"steering_curvature_map{tag}.csv"

    # We need to manage the loops manually to support retries/modifications
    # Convert to mutable lists so we can modify them if user requests
    current_throttles = list(throttle_levels)
    # We will iterate through throttles

    # Updated columns to include throttle_cmd
    with CSVLogger(
        csv_fn,
        [
            "throttle_cmd",
            "steer_cmd",
            "velocity_mean",
            "yaw_rate_mean",
            "radius_m",
            "curvature_1_m",
        ],
        results_dir=SCRIPT_RESULTS_DIR,
    ) as log:
        # Outer loop: Throttles
        t_idx = 0
        while t_idx < len(current_throttles):
            throttle = current_throttles[t_idx]
            print_section(f"Testing Throttle = {throttle:.3f}")

            # Prepare steering levels for this throttle (refresh from args default or keep modified?)
            # Usually we want to reset the steering list for a new throttle.
            current_steers = list(steer_levels)

            # Inner loop: Steering angles
            s_idx = 0
            while s_idx < len(current_steers):
                steer = current_steers[s_idx]

                # --- Run the test ---
                print(f"  Steering = {steer:+.2f} ... ", end="", flush=True)

                total_steps = int(args.run_time / dt)
                settle_steps = int(args.settle_time / dt)

                yaw_rates = []
                velocities = []

                if sim_mode:
                    steer_sim.reset()
                    motor_sim.reset()

                for step in range(total_steps):
                    if sim_mode:
                        v = motor_sim.step(throttle, dt)
                        # Use fixed sim velocity for consistent curvature results if desired,
                        # or use motor model velocity 'v' for more realism.
                        # The original code used args.speed_sim logic, preserving it:
                        v_use = args.speed_sim if args.speed_sim > 0 else v

                        x, y, theta = steer_sim.step(v_use, steer, dt)
                        # Ideal bicycle model yaw rate
                        yaw_rate = v_use / args.wheelbase_nom * np.tan(steer)
                    else:
                        hw_interface.send(throttle, steer)
                        v, yaw_rate, _ = hw_interface.read_state()
                        v_use = v
                        time.sleep(dt)

                    if step >= settle_steps:
                        yaw_rates.append(yaw_rate)
                        velocities.append(v_use)

                # Stop car after test
                if not sim_mode:
                    hw_interface.send(0.0, 0.0)
                    # small pause
                    time.sleep(0.1)

                # --- Compute metrics ---
                yaw_mean = float(np.mean(yaw_rates)) if yaw_rates else 0.0
                vel_mean = float(np.mean(velocities)) if velocities else 0.0

                if abs(yaw_mean) < 1e-6 or abs(vel_mean) < 1e-6:
                    radius = np.inf
                    curvature = 0.0
                else:
                    radius = vel_mean / abs(yaw_mean)
                    curvature = np.sign(steer) * (1.0 / radius)

                print(
                    f"yaw={yaw_mean:+.4f}, v={vel_mean:.3f}, R={radius:.3f}, k={curvature:+.4f}"
                )

                # --- Interactive Interaction ---
                if args.interactive:
                    print(
                        f"\n  [Interactive] Throttle={throttle:.2f}, Steer={steer:.2f}"
                    )
                    print(
                        f"    Measured: Yaw={yaw_mean:.3f}, Vel={vel_mean:.3f}, Curv={curvature:.3f}"
                    )
                    print(
                        "    Actions: [Enter]=Keep, [r]=Retry, [m]=Modify, [s]=Skip, [q]=Quit"
                    )

                    user_input = _get_interactive_input(
                        "    Select action: ", hw_interface, sim_mode
                    )

                    if user_input == "r":
                        print("    -> Retrying same point...")
                        continue  # Restart same s_idx

                    elif user_input == "m":
                        try:
                            # Simple modification: ask for new throttle/steer
                            new_th_str = _get_interactive_input(
                                f"    Enter new throttle (current {throttle}): ",
                                hw_interface,
                                sim_mode,
                            )
                            new_st_str = _get_interactive_input(
                                f"    Enter new steer (current {steer}): ",
                                hw_interface,
                                sim_mode,
                            )

                            if new_th_str:
                                throttle = float(new_th_str)
                                # Update current_throttles list for consistency if you want,
                                # but usually we just want to change THIS test point.
                                # Let's just update the local variables and RETRY with them.
                                # To do this properly without breaking the loops,
                                # we can just restart this loop iteration with new values.
                                # NOTE: This modifies the 'throttle' variable for this iteration only.
                                # If we want to persist it, we'd need more complex logic.
                                # For now, let's assume 'm' allows retrying THIS point with new values.

                            if new_st_str:
                                steer = float(new_st_str)

                            print(
                                f"    -> Retrying with Throttle={throttle}, Steer={steer}..."
                            )

                            # We need to re-run with these new values.
                            # If we just 'continue', it will reload 'steer' from current_steers[s_idx].
                            # So we need to temporarily override the lists or use a 'custom_run' flag.
                            # A simpler way: just modify the lists in place?
                            # Or simpler:
                            # Just set a one-shot override?

                            # Let's insert the new test point at the current index and stay here?
                            # actually, if we modify, we usually want to replace the current planned point.
                            if new_st_str:
                                current_steers[s_idx] = steer
                            if new_th_str:
                                current_throttles[t_idx] = throttle

                            continue  # Restart loop with updated list values

                        except ValueError:
                            print("    Invalid input. Retrying original...")
                            continue

                    elif user_input == "s":
                        print("    -> Skipping...")
                        s_idx += 1
                        continue

                    elif user_input in ["q", "f"]:
                        print("    -> Finishing early. Saving collected data...")
                        return (
                            np.array(all_steer_cmds),
                            np.array(all_curvatures),
                            np.array(all_radii),
                            np.array(all_yaw_means),
                            np.array(all_vel_means),
                            np.array(all_throttle_cmds),
                        )

                    # Default / Enter -> Keep
                    print("    -> Keeping data.")

                # If we get here (Keep), save data and advance
                all_steer_cmds.append(steer)
                all_curvatures.append(curvature)
                all_radii.append(radius)
                all_yaw_means.append(yaw_mean)
                all_vel_means.append(vel_mean)
                all_throttle_cmds.append(throttle)

                log.write(throttle, steer, vel_mean, yaw_mean, radius, curvature)

                # Advance to next steering angle
                s_idx += 1

                # Stop and wait between angles (if not already stopped)
                if not sim_mode:
                    hw_interface.send(0.0, 0.0)
                    time.sleep(1.0)

            # Advance to next throttle level
            t_idx += 1

    return (
        np.array(all_steer_cmds),
        np.array(all_curvatures),
        np.array(all_radii),
        np.array(all_yaw_means),
        np.array(all_vel_means),
        np.array(all_throttle_cmds),
    )


# ─────────────────────────────────────────────────────────────────────────────
#  Ackermann model fitting
# ─────────────────────────────────────────────────────────────────────────────


def fit_ackermann(steer_cmds: np.ndarray, curvatures: np.ndarray) -> dict:
    """
    Fit effective wheelbase from  curvature = tan(delta_eff) / L_eff.
    For small angles: curvature ≈ delta_eff / L_eff
    → L_eff = delta_eff / curvature  (median over nonzero levels)
    """
    valid = np.abs(curvatures) > 1e-3
    if valid.sum() < 2:
        return {
            "L_eff_m": args.wheelbase_nom,
            "note": "Not enough data for Ackermann fit",
        }

    L_estimates = np.abs(steer_cmds[valid]) / np.abs(curvatures[valid])
    L_eff = float(np.median(L_estimates))
    return {
        "L_eff_m": round(L_eff, 5),
        "L_nom_m": args.wheelbase_nom,
        "L_error_pct": round(
            100.0 * abs(L_eff - args.wheelbase_nom) / args.wheelbase_nom, 2
        ),
    }


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────


def main():
    print_section("QCar Steering → Curvature Calibration")
    if args.sim:
        interface_label = "SIMULATION (Bicycle Model)"
    elif args.qlabs:
        interface_label = f"QLABS virtual QCar '{args.actor}'"
    else:
        interface_label = "PHYSICAL hardware"

    print(f"  Mode         : {interface_label}")
    print(f"  Steer levels : {args.steer_levels}")
    print(f"  Throttles    : {throttle_levels}")
    print(f"  Run time/lvl : {args.run_time}s  settle: {args.settle_time}s")

    hw_interface = None
    motor_sim = FirstOrderMotorSim(tau=0.5, K=5.0, noise_std=0.005)
    steer_sim = BicycleSteeringSim(wheelbase=args.wheelbase_nom, noise_std=0.001)

    if not args.sim:
        hw_interface = QCarSteeringHardwareInterface(
            qlabs_mode=args.qlabs,
            actor_name=args.actor,
        )
        hw_interface.connect()

    try:
        (
            steer_cmds,
            curvatures,
            radii,
            yaw_means,
            vel_means,
            throttle_cmds_out,
        ) = collect_steering_data(hw_interface, motor_sim, steer_sim, args.sim)
    finally:
        if hw_interface is not None:
            hw_interface.stop()

    # --- Analysis per throttle level ---
    ackermann_results = {}
    poly_coeffs_dict = {}

    unique_throttles = sorted(list(set(throttle_cmds_out)))

    for th in unique_throttles:
        mask = np.abs(throttle_cmds_out - th) < 1e-5
        st_sub = steer_cmds[mask]
        k_sub = curvatures[mask]

        # Polynomial fit
        _, poly = plot_calibration_map(
            st_sub,
            k_sub,
            x_label="Steering command [−]",
            y_label="Curvature κ [1/m]",
            poly_deg=args.poly_deg,
            title=f"QCar Steering → Curvature (Throttle={th:.2f})",
            filename=f"steering_curvature_map_T{th:.2f}{tag}.png",
            results_dir=SCRIPT_RESULTS_DIR,
        )
        poly_coeffs_dict[f"throttle_{th:.2f}"] = [float(c) for c in poly]

        # Ackermann fit
        ackermann_results[f"throttle_{th:.2f}"] = fit_ackermann(st_sub, k_sub)

    # Global fit (using all data)
    print_section("Global Calibration Summary (All Throttles)")
    _, poly_global = plot_calibration_map(
        steer_cmds,
        curvatures,
        x_label="Steering command [−]",
        y_label="Curvature κ [1/m]",
        poly_deg=args.poly_deg,
        title=f"QCar Steering → Curvature (Combined)",
        filename=f"steering_curvature_map_combined{tag}.png",
        results_dir=SCRIPT_RESULTS_DIR,
    )

    ackermann_global = fit_ackermann(steer_cmds, curvatures)
    print(f"  Global Ackermann fit: {ackermann_global}")

    # --- Save results ---
    result = {
        "description": "QCar steering command → curvature calibration",
        "mode": "simulation" if args.sim else "hardware",
        "poly_degree": args.poly_deg,
        "poly_coefficients_global": [float(c) for c in poly_global],
        "poly_coefficients_per_throttle": poly_coeffs_dict,
        "ackermann_global": ackermann_global,
        "ackermann_per_throttle": ackermann_results,
        "measured_points": {
            "throttle_cmd": throttle_cmds_out.tolist(),
            "steering_cmd": steer_cmds.tolist(),
            "curvature_1_m": curvatures.tolist(),
            "radius_m": [float(r) if np.isfinite(r) else None for r in radii],
            "yaw_rate_rads": yaw_means.tolist(),
            "velocity_ms": vel_means.tolist(),
        },
    }
    save_yaml(result, f"steering_calibration{tag}.yaml", results_dir=SCRIPT_RESULTS_DIR)

    print("\n[✓] Steering calibration complete.")


if __name__ == "__main__":
    main()
