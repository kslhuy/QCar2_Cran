#!/usr/bin/env python3
"""
02_motor_model_identification.py
=================================
QCar Calibration – First-Order Motor Model Identification
----------------------------------------------------------
Applies a single throttle step from rest, records the velocity response, and
fits the first-order model:

        τ · dv/dt + v = K · u

to extract the time constant τ [s] and steady-state gain K [m/s per throttle unit].
Uses the identified model to suggest PID gains (ZN, IMC, ITAE).

Supports THREE modes:
  --sim      : pure Python first-order motor model  (no QLabs, no hardware)
  --qlabs    : QLabs virtual QCar via readRobots()  (run initPlatoon.py first)
  (default)  : physical QCar hardware

Usage (Python sim):
    python 02_motor_model_identification.py --sim

Usage (QLabs virtual):
    python 02_motor_model_identification.py --qlabs --actor QC2_0 --throttle 0.10

Usage (real hardware):
    python 02_motor_model_identification.py --throttle 0.10 --duration 8

Output:
    results/motor_model_id.yaml          ← tau, K, suggested PID gains
    results/step_response_{tag}.png      ← measured vs. fitted step response
    results/step_response_{tag}.csv      ← raw time-series
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
    FirstOrderMotorSim,
    fit_first_order,
    plot_step_response,
    pid_gains_ziegler_nichols,
    pid_gains_imc,
    pid_gains_itae,
    save_yaml,
    print_section,
)

# ── CLI arguments ─────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(
    description="QCar motor model identification (step response)"
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
    help="QLabs actor name, e.g. QC2_0, QC2_1 (used with --qlabs)",
)
parser.add_argument(
    "--throttle", type=float, default=0.10, help="Step throttle command to apply [0–1]"
)
parser.add_argument(
    "--duration", type=float, default=20.0, help="Total recording duration [s]"
)
parser.add_argument(
    "--warmup",
    type=float,
    default=0.1,
    help="Time [s] to hold throttle=0 before applying the step",
)
parser.add_argument("--dt", type=float, default=0.02, help="Control loop time step [s]")
parser.add_argument(
    "--tag", type=str, default="", help="Optional tag for output filenames"
)
parser.add_argument(
    "--interactive",
    "-i",
    action="store_true",
    help="Enable interactive mode to specify start/end throttle steps manually",
)
args = parser.parse_args()

# Controller tuning notes:
# - K (m/s per throttle) and tau (s) characterize the speed-loop plant.
# - Use the suggested PID as a starting point, then retune on-track.
# - For PP-map tuning, this script helps you estimate how fast throttle
#   changes become actual speed changes in corners.

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
#  Hardware interface (same pattern as Script 01)
# ─────────────────────────────────────────────────────────────────────────────


class QCarHardwareInterface:
    """Unified QCar wrapper for physical, QLabs virtual, and (unused directly) sim modes."""

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
                    f"Actor '{self._actor_name}' not in QLabs. "
                    f"Available: {list(robots.keys())}. "
                    f"Did initPlatoon.py run?"
                )
            hil_port = robots[self._actor_name].get("hilPort")
            if hil_port is None:
                raise RuntimeError(
                    f"hilPort=None for '{self._actor_name}'. "
                    f"QLabs real-time model not started yet."
                )
            self._qcar = QCar(readMode=1, hilPort=hil_port)
            print(
                f"[QLABS] Connected to virtual QCar '{self._actor_name}' "
                f"(hilPort={hil_port})"
            )
        else:
            self._qcar = QCar(readMode=1)
            print("[HW] Physical QCar connected.")

    def send(self, throttle: float, steering: float = 0.0):
        self._qcar.write(
            throttle=float(np.clip(throttle, -1.0, 1.0)),
            steering=float(np.clip(steering, -0.5, 0.5)),
        )

    def read_velocity(self) -> float:
        self._qcar.read()
        return float(getattr(self._qcar, "motorTach", 0.0))

    def stop(self):
        self.send(0.0)
        time.sleep(0.5)
        # if self._qcar:
        #     try:
        #         self._qcar.terminate()
        #     except Exception:
        #         pass
        label = "QLABS" if self._qlabs_mode else "HW"
        print(f"[{label}] QCar stopped.")


# ─────────────────────────────────────────────────────────────────────────────
#  Step response recording
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
            hw_interface.send(0.0)
            try:
                hw_interface.read_velocity()
            except Exception:
                pass
        t.join(0.02)  # wait 20ms

    try:
        return q.get_nowait()
    except queue.Empty:
        return ""


# ─────────────────────────────────────────────────────────────────────────────
#  Step response recording
# ─────────────────────────────────────────────────────────────────────────────


def record_step_response(
    interface,
    sim_mode: bool,
    throttle_start: float,
    throttle_end: float,
    duration: float,
    warmup: float,
):
    """Record velocity during a throttle step from start -> end.

    Returns: t, v  (numpy arrays)
    """
    dt = args.dt
    warmup_steps = int(warmup / dt)
    total_steps = int(duration / dt)
    step_start_idx = warmup_steps

    t_list = []
    v_list = []
    throttle_list = []

    csv_fn = f"step_response{tag}.csv"

    # We append to the CSV if it exists (in interactive mode), or overwrite?
    # For now, let's just overwrite per run or use unique names if we wanted history.
    # But CSVLogger usually overwrites.

    # In interactive mode, maybe we want to log chunks?
    # Let's keep it simple: log the current specific step test.

    with CSVLogger(csv_fn, ["time_s", "throttle_cmd", "velocity_ms"]) as log:
        for i in range(total_steps):
            t_now = i * dt

            # Before step_start: hold throttle_start
            # After step_start: jump to throttle_end
            if i < step_start_idx:
                current_u = throttle_start
            else:
                current_u = throttle_end

            if sim_mode:
                v = interface.step(current_u, dt)
            else:
                interface.send(current_u)
                v = interface.read_velocity()
                time.sleep(dt)

            t_list.append(t_now)
            v_list.append(v)
            throttle_list.append(current_u)
            log.write(t_now, current_u, v)

            # Live progress every second
            if (i + 1) % int(1.0 / dt) == 0:
                print(f"  t={t_now:.1f}s  u={current_u:.3f}  v={v:.4f} m/s")

    t = np.array(t_list)
    v = np.array(v_list)

    # Cut out the warmup phase for analysis?
    # Actually, for differential analysis, we might want to see the transition.
    # But fit_first_order prefers starting at t=0 with y(0) being the initial state?
    # No, fit_first_order assumes simple step from 0.
    # We need to adjust:
    # y_fit(t) = y_start + (y_final - y_start) * (1 - exp(-t/tau))

    # Let's return the WHOLE trace, and let analyse handle slicing/shifting.
    return t, v, step_start_idx


# ─────────────────────────────────────────────────────────────────────────────
#  Analysis
# ─────────────────────────────────────────────────────────────────────────────


def analyse(
    t: np.ndarray,
    v: np.ndarray,
    step_start_idx: int,
    throttle_start: float,
    throttle_end: float,
):
    """Fit differential model, compute gains, print results.

    Model:
        Δv(t) = K * Δu * (1 - exp(-t/tau))
        v(t) = v_start + K * (u_end - u_start) * (1 - exp(-(t - t_step)/tau))
    """
    dt = args.dt

    # 1. Determine v_start (average of pre-step) and v_end (average of post-step)
    # pre-step: from 0 to step_start_idx
    if step_start_idx > 5:
        v_start_measured = float(
            np.mean(v[max(0, step_start_idx - int(0.5 / dt)) : step_start_idx])
        )
    else:
        v_start_measured = v[0]

    # post-step: last 1s
    v_end_measured = float(np.mean(v[-int(1.0 / dt) :]))

    delta_v_ss = v_end_measured - v_start_measured
    delta_u = throttle_end - throttle_start

    print_section("Step Response Analysis")
    print(
        f"  Throttle step : {throttle_start:.3f} -> {throttle_end:.3f} (Δu={delta_u:.3f})"
    )
    print(f"  v_start       : {v_start_measured:.4f} m/s")
    print(f"  v_end         : {v_end_measured:.4f} m/s")
    print(f"  Δv_ss         : {delta_v_ss:.4f} m/s")

    if abs(delta_u) < 1e-4:
        print("  [WARN] delta_u is too small for analysis.")
        return None, None

    # Prepare data for fitting:
    # t_fit starts at 0 when step happens
    t_full = t
    v_full = v

    t_after = t[step_start_idx:] - t[step_start_idx]  # shift to 0
    v_after = v[step_start_idx:]

    # We want to fit: y(t) = v_start + (K*delta_u) * (1 - exp(-t/tau))
    # Or simpler: y_norm(t) = v_after - v_start.
    # Then fit standard first order to y_norm.

    v_response = v_after - v_start_measured

    try:
        # We pass y_ss = delta_v_ss
        tau, K_gain_abs = fit_first_order(t_after, v_response, y_ss=delta_v_ss)
        # K_gain_abs is the steady state change value (approx delta_v_ss)
        # K = change_in_v / change_in_u
        K = K_gain_abs / delta_u

        print(f"  tau           : {tau:.4f} s")
        print(f"  K (Δv/Δu)     : {K:.4f} m/s per unit")
    except Exception as exc:
        print(f"  [WARN] Model fitting failed: {exc}")
        tau = None
        K = None

    # --- Plot ---
    v_fitted_full = None
    if tau is not None:
        # Reconstruct full fitted curve for plotting overlay
        # Before step: v_start
        # After step: v_start + K*delta_u*(...)
        v_fitted_full = np.zeros_like(v_full)
        v_fitted_full[:step_start_idx] = v_start_measured

        # for t >= step
        t_segment = t_full[step_start_idx:] - t_full[step_start_idx]
        v_fitted_full[step_start_idx:] = v_start_measured + (K * delta_u) * (
            1.0 - np.exp(-t_segment / tau)
        )

    plot_step_response(
        t=t_full,
        v_measured=v_full,
        v_cmd=v_end_measured,  # showing final target
        v_fitted=v_fitted_full,
        tau=tau,
        K=K,  # This label might be misleading in plot_step_response if it expects K absolute.
        # But plot_step_response just prints what we pass.
        # Actually plot_step_response logic for v63 calculation assumes zero start.
        # We might need to ignore the auto-annotation or accept it's approximate.
        title=f"Step Response ({throttle_start:.2f}->{throttle_end:.2f})",
        filename=f"step_response{tag}.png",
    )

    # Returns tau, K
    return tau, K


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────


def main():
    if args.sim and args.qlabs:
        print("[ERROR] Cannot use --sim and --qlabs at the same time.")
        sys.exit(1)

    if args.sim:
        mode_label = "SIMULATION (Python math)"
    elif args.qlabs:
        mode_label = f"QLABS virtual QCar '{args.actor}'"
    else:
        mode_label = "PHYSICAL hardware"

    print_section("QCar Motor Model Identification")
    print(f"  Mode : {mode_label}")

    # Initialize interface
    if args.sim:
        # If interactive, we might want to reset this between runs?
        # But motor state persists in real life.
        # For sim, passing it around is better.
        interface = FirstOrderMotorSim(tau=0.8, K=5.0, noise_std=0.015)
    else:
        interface = QCarHardwareInterface(
            qlabs_mode=args.qlabs,
            actor_name=args.actor,
        )
        interface.connect()

    # Storage for accepted results
    # List of dicts: { 'u_start': ..., 'u_end': ..., 'tau': ..., 'K': ... }
    accepted_results = []

    try:
        if args.interactive:
            print("\n[Interactive Mode Enabled]")

            # Default start
            curr_u_start = 0.0
            curr_u_end = 0.1

            while True:
                print(f"\n--- Next Test Setup ---")
                print(f"  Start Throttle : {curr_u_start}")
                print(f"  End Throttle   : {curr_u_end}")
                print("  Options: [R]un, [C]hange, [Q]uit/Finish")

                choice = _get_interactive_input("  Select > ", interface, args.sim)

                if choice == "c":
                    try:
                        s_str = _get_interactive_input(
                            f"    New Start ({curr_u_start}): ", interface, args.sim
                        )
                        e_str = _get_interactive_input(
                            f"    New End   ({curr_u_end})  : ", interface, args.sim
                        )
                        if s_str:
                            curr_u_start = float(s_str)
                        if e_str:
                            curr_u_end = float(e_str)
                    except ValueError:
                        print("    Invalid input.")
                    continue
                elif choice == "q":
                    break
                elif choice == "r" or choice == "":
                    pass  # Proceed to run
                else:
                    continue

                # Run the test
                print(f"\nRunning step {curr_u_start} -> {curr_u_end}...")

                # Perform the step
                t, v, step_idx = record_step_response(
                    interface,
                    args.sim,
                    throttle_start=curr_u_start,
                    throttle_end=curr_u_end,
                    duration=args.duration,
                    warmup=args.warmup,
                )

                # Analyse
                tau, K = analyse(t, v, step_idx, curr_u_start, curr_u_end)

                if tau is not None:
                    print(f"\n  Result: K={K:.4f}, tau={tau:.4f}")
                    # Ask to keep
                    k_choice = _get_interactive_input(
                        "  [K]eep this result? (y/n) > ", interface, args.sim
                    )
                    if k_choice in ["y", "yes", "k"]:
                        accepted_results.append(
                            {
                                "u_start": curr_u_start,
                                "u_end": curr_u_end,
                                "tau": tau,
                                "K": K,
                            }
                        )
                        print("  -> Saved.")

                        # Auto-advance for convenience?
                        # curr_u_start = curr_u_end
                        # curr_u_end = round(curr_u_end + 0.1, 2)
                    else:
                        print("  -> Discarded.")

                # Stop car after test?
                # Ideally we leave it at v_end if we want to chain?
                # But for safety, maybe stop or hold?
                # The prompt implies we are designing the next test.
                # If we stop here, the car stops.
                # The Next Test Setups `Start Throttle` implies we will ramp to it?
                # `record_step_response` has a warmup phase where it holds `throttle_start`.
                # So it handles the transition.
                if not args.sim:
                    interface.send(
                        0.0
                    )  # Safety stop between interactive planning steps

        else:
            # ORIGINAL NON-INTERACTIVE MODE
            # We treat it as 0 -> args.throttle
            t, v, step_idx = record_step_response(
                interface,
                args.sim,
                throttle_start=0.0,
                throttle_end=args.throttle,
                duration=args.duration,
                warmup=args.warmup,
            )
            tau, K = analyse(t, v, step_idx, 0.0, args.throttle)
            if tau and K:
                accepted_results.append(
                    {"u_start": 0.0, "u_end": args.throttle, "tau": tau, "K": K}
                )

    finally:
        if not args.sim:
            interface.stop()

    # --- Final Savings ---
    if not accepted_results:
        print("\nNo valid results to save.")
        return

    # Calculate averages
    avg_K = float(np.mean([r["K"] for r in accepted_results]))
    avg_tau = float(np.mean([r["tau"] for r in accepted_results]))

    # PID suggestions based on Average
    print_section(f"Final Averaged Model (N={len(accepted_results)})")
    print(f"  Avg K   : {avg_K:.4f}")
    print(f"  Avg tau : {avg_tau:.4f} s")

    # Generate PID suggestions for the average model
    # (Rest of PID code is reused or copied here)
    pid_results = {}
    L_est = 0.1 * avg_tau
    zn = pid_gains_ziegler_nichols(avg_K, avg_tau, L_est)
    imc = pid_gains_imc(avg_K, avg_tau)
    itae = pid_gains_itae(avg_K, avg_tau, L_est)

    print_section("Suggested PID Gains (Based on Avg Model)")
    hdr = f"  {'Method':>12}  {'Kp':>8}  {'Ki':>8}  {'Kd':>8}"
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))
    for name, g in [("Ziegler-Nichols", zn), ("IMC (λ=0.5τ)", imc), ("ITAE", itae)]:
        print(f"  {name:>15}  {g['Kp']:8.4f}  {g['Ki']:8.4f}  {g['Kd']:8.4f}")

    pid_results = {"ZN": zn, "IMC": imc, "ITAE": itae}

    result = {
        "description": "QCar motor model identification (Interactive/Avg)",
        "mode": "simulation" if args.sim else "hardware",
        "average_model": {
            "tau_s": round(float(avg_tau), 5),
            "K_mps_per_throttle": round(avg_K, 5),
        },
        "individual_trials": accepted_results,
        "suggested_pid": pid_results,
        "usage": "Use average_model values for configuration.",
    }

    save_yaml(result, f"motor_model_id{tag}.yaml")
    save_yaml(result, "motor_model_id.yaml")
    print("\n[✓] Motor model identification complete.")


if __name__ == "__main__":
    main()
