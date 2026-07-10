#!/usr/bin/env python3
"""
04_pid_autotuner.py
====================
QCar Calibration – PID Gain Auto-Tuner
---------------------------------------
Loads the motor model identified by 02_motor_model_identification.py and
computes PID controller gains using three classic tuning rules:

  1. Ziegler-Nichols open-loop  (fast, can be aggressive)
  2. IMC (Internal Model Control) (smooth, recommended for real car)
  3. ITAE (minimum ITAE, good disturbance rejection)

Optionally writes recommended (IMC) gains directly to the controller config YAML.

Usage:
    # Load from default results/motor_model_id.yaml:
    python 04_pid_autotuner.py

    # Specify a custom model file:
    python 04_pid_autotuner.py --model results/motor_model_id_20260219_223500.yaml

    # Apply IMC gains to controller config:
    python 04_pid_autotuner.py --apply --config_path ../Controller/config/controller_config.yaml

Output:
    results/pid_gains_recommendation.yaml
"""

import argparse
import sys
import os
import numpy as np

_CAL_DIR = os.path.dirname(os.path.abspath(__file__))
_QCAR_DIR = os.path.dirname(_CAL_DIR)
for _p in [_CAL_DIR, _QCAR_DIR]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from Development.multi_vehicle_self_driving_RealQcar.qcar_refactor.refs.qcar_origin.Calibration.calibration_utils import (
    load_yaml,
    save_yaml,
    pid_gains_ziegler_nichols,
    pid_gains_imc,
    pid_gains_itae,
    print_section,
    RESULTS_DIR,
)

# ── CLI arguments ─────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(
    description="QCar PID gain auto-tuner from motor model"
)
parser.add_argument(
    "--model",
    type=str,
    default=os.path.join(
        RESULTS_DIR, "02_motor_model_identification", "motor_model_id.yaml"
    ),
    help="Path to motor_model_id.yaml from script 02",
)
parser.add_argument(
    "--lambda_imc",
    type=float,
    default=None,
    help="IMC closed-loop time constant [s] "
    "(default = 0.5 * tau, faster → smaller lambda)",
)
parser.add_argument(
    "--dead_time_frac",
    type=float,
    default=0.1,
    help="Dead time as fraction of tau for ZN/ITAE (default 0.1)",
)
parser.add_argument(
    "--apply",
    action="store_true",
    help="Write recommended (IMC) gains to controller config YAML",
)
parser.add_argument(
    "--config_path",
    type=str,
    default=os.path.join(_QCAR_DIR, "Controller", "config", "controller_config.yaml"),
    help="Path to the controller config YAML to update (needs --apply)",
)
parser.add_argument(
    "--method",
    type=str,
    default="IMC",
    choices=["ZN", "IMC", "ITAE"],
    help="Which method to apply when --apply is set",
)
args = parser.parse_args()

SCRIPT_RESULTS_DIR = os.path.join(RESULTS_DIR, "04_pid_autotuner")


# ─────────────────────────────────────────────────────────────────────────────
#  Load motor model
# ─────────────────────────────────────────────────────────────────────────────


def load_motor_model() -> dict:
    """Load and validate motor_model_id.yaml."""
    if not os.path.exists(args.model):
        print(f"[ERROR] Motor model file not found: {args.model}")
        print("  → Run 02_motor_model_identification.py first to generate it.")
        sys.exit(1)

    import yaml

    with open(args.model) as f:
        data = yaml.safe_load(f)

    model_block = data.get("model", {})
    tau = model_block.get("tau_s")
    K = model_block.get("K_mps_per_throttle")

    if tau is None or K is None:
        print(
            f"[ERROR] Motor model YAML is missing 'model.tau_s' or "
            f"'model.K_mps_per_throttle'. Check: {args.model}"
        )
        sys.exit(1)

    return {"tau": float(tau), "K": float(K), "v_ss": data.get("v_ss_measured", None)}


# ─────────────────────────────────────────────────────────────────────────────
#  Compute gains
# ─────────────────────────────────────────────────────────────────────────────


def compute_all_gains(tau: float, K: float) -> dict:
    L = args.dead_time_frac * tau
    return {
        "ZN": pid_gains_ziegler_nichols(K, tau, L),
        "IMC": pid_gains_imc(K, tau, args.lambda_imc),
        "ITAE": pid_gains_itae(K, tau, L),
    }


def print_gain_table(gains: dict, tau: float, K: float):
    print_section("PID Gain Recommendations")
    print(f"  Motor model:  τ = {tau:.4f} s    K = {K:.4f} m/s per throttle unit")
    print(
        f"  Dead time:    L = {args.dead_time_frac * tau:.4f} s  ({100 * args.dead_time_frac:.0f}% of τ)"
    )
    if args.lambda_imc:
        print(f"  IMC lambda:   λ = {args.lambda_imc:.4f} s (custom)")
    else:
        print(f"  IMC lambda:   λ = {0.5 * tau:.4f} s (default = 0.5·τ)")

    print()
    print(f"  {'Method':>20}  {'Kp':>10}  {'Ki':>10}  {'Kd':>10}  Notes")
    print(f"  {'─' * 20}  {'─' * 10}  {'─' * 10}  {'─' * 10}  ─────────────────────")

    notes = {
        "ZN": "Fast, may overshoot",
        "IMC": "★ Smooth (recommended)",
        "ITAE": "Good disturbance rejection",
    }
    for method, g in gains.items():
        print(
            f"  {method:>20}  {g['Kp']:10.4f}  {g['Ki']:10.4f}  {g['Kd']:10.4f}  {notes[method]}"
        )

    print()
    print("  PIDVelocityController config (recommended – IMC):")
    imc = gains["IMC"]
    print(f"    kp: {imc['Kp']}")
    print(f"    ki: {imc['Ki']}")
    print(f"    kd: {imc['Kd']}")


# ─────────────────────────────────────────────────────────────────────────────
#  Simulation validation
# ─────────────────────────────────────────────────────────────────────────────


def simulate_pid_response(
    Kp: float,
    Ki: float,
    Kd: float,
    tau: float,
    K: float,
    v_ref: float = 0.5,
    duration: float = 10.0,
    dt: float = 0.02,
) -> dict:
    """
    Simulate the PID-controlled first-order motor to validate gains.
    Returns performance metrics: overshoot, settling_time, steady_state_error.
    """
    steps = int(duration / dt)
    v = 0.0
    ei = 0.0
    prev_e = 0.0
    ei_max = 5.0  # anti-windup

    v_history = []
    for i in range(steps):
        e = v_ref - v
        ei = np.clip(ei + e * dt, -ei_max, ei_max)
        de = (e - prev_e) / dt if i > 0 else 0.0
        u = Kp * e + Ki * ei + Kd * de
        u = np.clip(u, -0.3, 0.3)  # max_throttle clamp
        prev_e = e

        # First-order motor: τ·dv/dt = K·u - v
        dvdt = (K * u - v) / tau
        v = v + dvdt * dt
        v_history.append(v)

    v_arr = np.array(v_history)
    t_arr = np.arange(steps) * dt

    v_final = float(np.mean(v_arr[-int(1.0 / dt) :]))
    overshoot = float(max(0.0, (v_arr.max() - v_ref) / v_ref * 100))
    sse = abs(v_final - v_ref)

    # Settling time: first time v stays within ±5% of v_ref
    band = 0.05 * v_ref
    settling_idx = steps  # default: never
    for i in range(len(v_arr) - int(0.5 / dt)):
        window = v_arr[i : i + int(0.5 / dt)]
        if np.all(np.abs(window - v_ref) <= band):
            settling_idx = i
            break
    settling_time = t_arr[settling_idx] if settling_idx < steps else float("inf")

    return {
        "v_final": round(v_final, 4),
        "overshoot_pct": round(overshoot, 2),
        "settling_time_s": round(settling_time, 2)
        if settling_time != float("inf")
        else None,
        "steady_state_error": round(sse, 4),
    }


def print_simulation_results(gains: dict, tau: float, K: float, v_ref: float = 0.5):
    print_section("Simulated Closed-Loop Performance (v_ref = 0.5 m/s)")
    print(
        f"  {'Method':>20}  {'Overshoot %':>12}  {'Settle [s]':>12}  {'SSE [m/s]':>12}"
    )
    print(f"  {'─' * 20}  {'─' * 12}  {'─' * 12}  {'─' * 12}")
    for method, g in gains.items():
        perf = simulate_pid_response(g["Kp"], g["Ki"], g["Kd"], tau, K, v_ref)
        s_time = f"{perf['settling_time_s']:.2f}" if perf["settling_time_s"] else "∞"
        print(
            f"  {method:>20}  {perf['overshoot_pct']:12.2f}  {s_time:>12}  "
            f"{perf['steady_state_error']:12.4f}"
        )


# ─────────────────────────────────────────────────────────────────────────────
#  Apply gains to controller config
# ─────────────────────────────────────────────────────────────────────────────


def apply_gains_to_config(gains: dict, method: str):
    """Write recommended gains to controller_config YAML under pid section."""
    import yaml

    if not os.path.exists(args.config_path):
        print(f"[WARN] Controller config not found at: {args.config_path}")
        print("       Gains NOT applied. Create the config first.")
        return

    with open(args.config_path) as f:
        cfg = yaml.safe_load(f) or {}

    g = gains[method]
    # Traverse/create the path: longitudinal → pid
    if "longitudinal" not in cfg:
        cfg["longitudinal"] = {}
    if "pid" not in cfg["longitudinal"]:
        cfg["longitudinal"]["pid"] = {}

    cfg["longitudinal"]["pid"]["kp"] = g["Kp"]
    cfg["longitudinal"]["pid"]["ki"] = g["Ki"]
    cfg["longitudinal"]["pid"]["kd"] = g["Kd"]

    # Backup original
    backup = args.config_path + ".bak"
    import shutil

    shutil.copy2(args.config_path, backup)
    print(f"  Backed up original config to: {backup}")

    with open(args.config_path, "w") as f:
        yaml.dump(cfg, f, default_flow_style=False, sort_keys=False)

    print(f"  [✓] Applied {method} gains to: {args.config_path}")
    print(f"      kp={g['Kp']}, ki={g['Ki']}, kd={g['Kd']}")


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────


def main():
    print_section("QCar PID Auto-Tuner")
    print(f"  Motor model file: {args.model}")

    model = load_motor_model()
    tau = model["tau"]
    K = model["K"]

    # --- Compute gains ---
    gains = compute_all_gains(tau, K)
    print_gain_table(gains, tau, K)

    # --- Simulate closed-loop with those gains ---
    print_simulation_results(gains, tau, K)

    # --- Save recommendations ---
    result = {
        "description": "PID gain recommendations from motor model identification",
        "motor_model": {"tau_s": round(tau, 5), "K_mps_per_throttle": round(K, 5)},
        "tuning_parameters": {
            "dead_time_frac": args.dead_time_frac,
            "L_s": round(args.dead_time_frac * tau, 5),
            "lambda_imc": args.lambda_imc if args.lambda_imc else round(0.5 * tau, 5),
        },
        "gains": {m: {k: v for k, v in g.items()} for m, g in gains.items()},
        "recommended": "IMC",
        "usage": (
            "Copy 'gains.IMC' (or preferred method) into your "
            "controller_config.yaml under 'longitudinal.pid'"
        ),
    }
    save_yaml(result, "pid_gains_recommendation.yaml", results_dir=SCRIPT_RESULTS_DIR)

    # --- Apply if requested ---
    if args.apply:
        print_section(f"Applying {args.method} Gains to Config")
        apply_gains_to_config(gains, args.method)

    print("\n[✓] PID auto-tuner complete.")


if __name__ == "__main__":
    main()
