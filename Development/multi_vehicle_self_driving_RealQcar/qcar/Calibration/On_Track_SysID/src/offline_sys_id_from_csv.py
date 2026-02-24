#!/usr/bin/env python3

import argparse
import csv
import math
import os
import sys
from typing import Dict, List, Tuple

import numpy as np


VX_CANDIDATES = ["v_x", "vx", "vx_meas", "vx_est", "vx_true", "v_x_meas", "v_x_est"]
VY_CANDIDATES = ["v_y", "vy", "vy_est", "vy_meas", "vy_true", "v_y_est", "v_y_meas"]
OMEGA_CANDIDATES = ["omega", "r", "r_meas", "r_est", "omega_meas", "yaw_rate", "yawrate"]
DELTA_CANDIDATES = ["delta", "steering", "steering_angle", "steer", "steer_cmd"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run On_Track_SysID from offline CSV data by adapting columns "
            "to [v_x, v_y, omega, delta]."
        )
    )
    parser.add_argument("--csv", required=True, help="Input CSV path.")
    parser.add_argument("--racecar-version", required=True, help="Model folder name (e.g. SIM, NUC2).")
    parser.add_argument(
        "--dt",
        type=float,
        default=0.02,
        help="Training/sample timestep in seconds. Default: 0.02.",
    )
    parser.add_argument(
        "--speed-threshold",
        type=float,
        default=0.3,
        help="Keep only rows where v_x > threshold. Default: 0.3 m/s.",
    )
    parser.add_argument(
        "--speed-use-abs",
        action="store_true",
        help="Apply speed gate on abs(v_x) instead of v_x.",
    )
    parser.add_argument("--vx-column", help="Override v_x column name.")
    parser.add_argument("--vy-column", help="Override v_y column name.")
    parser.add_argument("--omega-column", help="Override omega/r column name.")
    parser.add_argument("--delta-column", help="Override delta/steering column name.")
    parser.add_argument(
        "--save-lut-name",
        default="offline_sysid",
        help="LUT filename prefix (used only with --generate-lut).",
    )
    parser.add_argument(
        "--generate-lut",
        action="store_true",
        help="Generate lookup table after fitting.",
    )
    parser.add_argument(
        "--plot-model",
        action="store_true",
        help="Plot model evolution (matplotlib windows).",
    )
    parser.add_argument(
        "--package-path",
        help="Path to On_Track_SysID package root. Default resolves from script location.",
    )
    parser.add_argument(
        "--export-filtered-csv",
        help="Optional path to export adapted filtered [v_x,v_y,omega,delta] CSV.",
    )
    return parser.parse_args()


def resolve_column(
    columns: List[str], explicit: str, candidates: List[str], signal_name: str
) -> str:
    column_map = {col.strip().lower(): col for col in columns}
    if explicit:
        key = explicit.strip().lower()
        if key not in column_map:
            raise ValueError(f"{signal_name} column '{explicit}' not found in CSV headers.")
        return column_map[key]

    for candidate in candidates:
        key = candidate.lower()
        if key in column_map:
            return column_map[key]

    raise ValueError(
        f"Could not auto-detect {signal_name} column. "
        f"Provide --{signal_name}-column explicitly."
    )


def load_data(csv_path: str, selected_cols: Tuple[str, str, str, str]) -> np.ndarray:
    vx_col, vy_col, omega_col, delta_col = selected_cols
    rows = []
    skipped = 0

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("CSV has no header row.")

        for row in reader:
            try:
                vx = float(row[vx_col])
                vy = float(row[vy_col])
                omega = float(row[omega_col])
                delta = float(row[delta_col])
                if any(math.isnan(x) or math.isinf(x) for x in (vx, vy, omega, delta)):
                    skipped += 1
                    continue
                rows.append([vx, vy, omega, delta])
            except Exception:
                skipped += 1

    if not rows:
        raise ValueError("No valid rows found for selected columns.")

    data = np.asarray(rows, dtype=np.float64)
    if skipped > 0:
        print(f"[WARN] Skipped {skipped} invalid rows during CSV parsing.")
    return data


def apply_speed_gate(data: np.ndarray, threshold: float, use_abs: bool) -> np.ndarray:
    vx = np.abs(data[:, 0]) if use_abs else data[:, 0]
    return data[vx > threshold]


def compute_equivalent_stiffness(
    c_pf: List[float], c_pr: List[float], model: Dict[str, float]
) -> Tuple[float, float]:
    b_f, c_f, d_f, _ = c_pf
    b_r, c_r, d_r, _ = c_pr

    f_zf = model["m"] * 9.81 * model["l_r"] / model["l_wb"]
    f_zr = model["m"] * 9.81 * model["l_f"] / model["l_wb"]

    c_f_eq = f_zf * b_f * c_f * d_f
    c_r_eq = f_zr * b_r * c_r * d_r
    return c_f_eq, c_r_eq


def export_filtered_csv(path: str, data: np.ndarray) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["v_x", "v_y", "omega", "delta"])
        writer.writerows(data.tolist())


def main() -> int:
    args = parse_args()
    from helpers.train_model import get_model_param, nn_train

    package_path = args.package_path
    if not package_path:
        package_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

    if args.dt <= 0.0:
        raise ValueError("--dt must be positive.")

    if not os.path.isfile(args.csv):
        raise FileNotFoundError(f"CSV not found: {args.csv}")

    with open(args.csv, "r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("CSV has no header row.")
        headers = reader.fieldnames

    vx_col = resolve_column(headers, args.vx_column, VX_CANDIDATES, "vx")
    vy_col = resolve_column(headers, args.vy_column, VY_CANDIDATES, "vy")
    omega_col = resolve_column(headers, args.omega_column, OMEGA_CANDIDATES, "omega")
    delta_col = resolve_column(headers, args.delta_column, DELTA_CANDIDATES, "delta")

    print(
        f"[INFO] Column mapping: v_x='{vx_col}', v_y='{vy_col}', "
        f"omega='{omega_col}', delta='{delta_col}'"
    )

    raw_data = load_data(args.csv, (vx_col, vy_col, omega_col, delta_col))
    gated_data = apply_speed_gate(raw_data, args.speed_threshold, args.speed_use_abs)

    print(f"[INFO] Raw rows: {raw_data.shape[0]}")
    print(f"[INFO] Rows after speed gate: {gated_data.shape[0]} (threshold={args.speed_threshold})")

    if gated_data.shape[0] < 20:
        raise ValueError(
            "Too few rows after speed gate. Need at least 20 rows for filtering/training. "
            "Lower --speed-threshold or provide longer data."
        )

    if args.export_filtered_csv:
        export_filtered_csv(args.export_filtered_csv, gated_data)
        print(f"[INFO] Exported filtered data to: {args.export_filtered_csv}")

    c_pf_identified, c_pr_identified = nn_train(
        training_data=gated_data.astype(np.float32),
        racecar_version=args.racecar_version,
        save_LUT_name=args.save_lut_name,
        plot_model=args.plot_model,
        dt=args.dt,
        generate_lut=args.generate_lut,
        package_path=package_path,
        always_plot_last_iteration=False,
    )

    model = get_model_param(args.racecar_version, package_path=package_path)
    c_f_eq, c_r_eq = compute_equivalent_stiffness(c_pf_identified, c_pr_identified, model)

    print("")
    print("[RESULT] Identified Pacejka")
    print(f"  C_Pf: {c_pf_identified}")
    print(f"  C_Pr: {c_pr_identified}")
    print("[RESULT] Equivalent linear stiffness")
    print(f"  Cf_eq: {c_f_eq:.4f}")
    print(f"  Cr_eq: {c_r_eq:.4f}")

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"[ERROR] {exc}")
        sys.exit(1)
