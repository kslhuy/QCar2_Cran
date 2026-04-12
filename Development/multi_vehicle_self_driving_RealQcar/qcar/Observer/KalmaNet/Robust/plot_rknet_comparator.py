import argparse
import csv
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

import matplotlib.pyplot as plt
import numpy as np

# python plot_rknet_comparator.py --no-show

DEFAULT_LOG_DIR = Path(__file__).resolve().parent / "logs" / "comparator"
DEFAULT_EXPORT_DIR = Path(__file__).resolve().parent / "logs" / "plots"


def _load_csv_rows(filepath: Path) -> Dict[str, np.ndarray]:
    with filepath.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows: List[dict] = list(reader)

    if not rows:
        raise SystemExit(f"No data rows found in {filepath}")

    numeric_columns = [
        "timestamp",
        "tick",
        "robust_x",
        "robust_y",
        "robust_theta",
        "robust_v",
        "ekf_x",
        "ekf_y",
        "ekf_theta",
        "ekf_v",
        "delta_x",
        "delta_y",
        "delta_theta",
        "delta_v",
        "position_error_norm",
        "heading_error",
        "velocity_error",
        "gps_valid",
        "pred_mask_mean",
        "pred_mask_min",
        "pred_mask_max",
        "mask_imu_mean",
        "mask_steer_mean",
        "mask_wheel_mean",
        "mask_selected_score",
        "mask_imu_active",
        "mask_steer_active",
        "mask_wheel_active",
        "meas_mask_x",
        "meas_mask_y",
        "meas_mask_psi",
        "meas_mask_v",
        "meas_mask_w",
        "K_norm",
        "K_x_x",
        "K_y_y",
        "K_psi_psi",
        "K_v_v",
        "K_w_w",
        "innov_x",
        "innov_y",
        "innov_psi",
        "innov_v",
        "innov_w",
        "pred_x",
        "pred_y",
    ]
    data: Dict[str, np.ndarray] = {}
    for column in numeric_columns:
        values = []
        for row in rows:
            raw = row.get(column, "")
            try:
                values.append(float(raw))
            except (TypeError, ValueError):
                values.append(np.nan)
        data[column] = np.asarray(values, dtype=np.float64)

    data["source"] = np.asarray([row.get("source", "") for row in rows], dtype=object)
    data["mask_selected_branch"] = np.asarray(
        [row.get("mask_selected_branch", "") for row in rows], dtype=object
    )
    return data


def _resolve_input_file(filepath: Optional[str]) -> Path:
    if filepath:
        candidate = Path(filepath)
        if not candidate.is_absolute():
            candidate = Path.cwd() / candidate
        return candidate.resolve()

    csv_files = sorted(DEFAULT_LOG_DIR.glob("*.csv"))
    if not csv_files:
        raise SystemExit(f"No comparator CSV files found in {DEFAULT_LOG_DIR}")
    return csv_files[-1]


def _safe_float(value: float) -> Optional[float]:
    if value is None or not np.isfinite(value):
        return None
    return float(value)


def _finite_count(values: np.ndarray) -> int:
    return int(np.isfinite(values).sum())


def _series_stats(values: np.ndarray, *, absolute: bool = False) -> Dict[str, Optional[float]]:
    series = np.abs(values) if absolute else values
    finite = series[np.isfinite(series)]
    if finite.size == 0:
        return {
            "count": 0,
            "mean": None,
            "std": None,
            "min": None,
            "max": None,
            "median": None,
            "rmse": None,
        }

    return {
        "count": int(finite.size),
        "mean": _safe_float(np.mean(finite)),
        "std": _safe_float(np.std(finite)),
        "min": _safe_float(np.min(finite)),
        "max": _safe_float(np.max(finite)),
        "median": _safe_float(np.median(finite)),
        "rmse": _safe_float(np.sqrt(np.mean(np.square(finite)))),
    }


def _value_distribution(values: np.ndarray) -> Dict[str, int]:
    distribution: Dict[str, int] = {}
    for raw in values:
        key = str(raw).strip() if str(raw).strip() else "unknown"
        distribution[key] = distribution.get(key, 0) + 1
    return dict(sorted(distribution.items()))


def _compute_summary(filepath: Path, data: Dict[str, np.ndarray], time_axis: np.ndarray) -> Dict[str, Any]:
    total_samples = int(len(time_axis))
    model_used = int(np.count_nonzero(data["source"] == "model"))
    fallback_used = int(np.count_nonzero(data["source"] == "fallback"))
    gps_valid_count = int(np.count_nonzero(data["gps_valid"] > 0.5))

    summary: Dict[str, Any] = {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_csv": str(filepath),
        "log_name": filepath.name,
        "sample_count": total_samples,
        "duration_seconds": _safe_float(time_axis[-1]) if total_samples else 0.0,
        "time_start_seconds": _safe_float(time_axis[0]) if total_samples else 0.0,
        "time_end_seconds": _safe_float(time_axis[-1]) if total_samples else 0.0,
        "usage_summary": {
            "model_samples": model_used,
            "fallback_samples": fallback_used,
            "gps_valid_samples": gps_valid_count,
            "model_ratio": _safe_float(model_used / total_samples) if total_samples else None,
            "fallback_ratio": _safe_float(fallback_used / total_samples) if total_samples else None,
            "gps_valid_ratio": _safe_float(gps_valid_count / total_samples) if total_samples else None,
        },
        "selected_branch_counts": _value_distribution(data["mask_selected_branch"]),
        "series_metrics": {
            "position_error_norm": _series_stats(data["position_error_norm"]),
            "heading_error_signed": _series_stats(data["heading_error"]),
            "heading_error_abs": _series_stats(data["heading_error"], absolute=True),
            "velocity_error_signed": _series_stats(data["velocity_error"]),
            "velocity_error_abs": _series_stats(data["velocity_error"], absolute=True),
            "delta_x": _series_stats(data["delta_x"]),
            "delta_y": _series_stats(data["delta_y"]),
            "delta_theta": _series_stats(data["delta_theta"]),
            "delta_v": _series_stats(data["delta_v"]),
            "innov_x": _series_stats(data["innov_x"]),
            "innov_y": _series_stats(data["innov_y"]),
            "innov_psi": _series_stats(data["innov_psi"]),
            "innov_v": _series_stats(data["innov_v"]),
            "K_norm": _series_stats(data["K_norm"]),
            "K_x_x": _series_stats(data["K_x_x"]),
            "K_y_y": _series_stats(data["K_y_y"]),
            "K_psi_psi": _series_stats(data["K_psi_psi"]),
            "K_v_v": _series_stats(data["K_v_v"]),
            "mask_selected_score": _series_stats(data["mask_selected_score"]),
            "mask_imu_mean": _series_stats(data["mask_imu_mean"]),
            "mask_steer_mean": _series_stats(data["mask_steer_mean"]),
            "mask_wheel_mean": _series_stats(data["mask_wheel_mean"]),
        },
        "trajectory_summary": {
            "robust_start_xy": [
                _safe_float(data["robust_x"][0]) if total_samples else None,
                _safe_float(data["robust_y"][0]) if total_samples else None,
            ],
            "robust_end_xy": [
                _safe_float(data["robust_x"][-1]) if total_samples else None,
                _safe_float(data["robust_y"][-1]) if total_samples else None,
            ],
            "ekf_start_xy": [
                _safe_float(data["ekf_x"][0]) if total_samples else None,
                _safe_float(data["ekf_y"][0]) if total_samples else None,
            ],
            "ekf_end_xy": [
                _safe_float(data["ekf_x"][-1]) if total_samples else None,
                _safe_float(data["ekf_y"][-1]) if total_samples else None,
            ],
        },
        "data_quality": {
            "finite_position_error_samples": _finite_count(data["position_error_norm"]),
            "finite_heading_error_samples": _finite_count(data["heading_error"]),
            "finite_velocity_error_samples": _finite_count(data["velocity_error"]),
            "finite_mask_score_samples": _finite_count(data["mask_selected_score"]),
        },
    }
    return summary


def _export_artifacts(
    filepath: Path,
    fig1: plt.Figure,
    fig2: plt.Figure,
    summary: Dict[str, Any],
    output_dir: Optional[str],
) -> Dict[str, str]:
    export_dir = Path(output_dir).expanduser() if output_dir else DEFAULT_EXPORT_DIR
    if not export_dir.is_absolute():
        export_dir = (Path.cwd() / export_dir).resolve()
    export_dir.mkdir(parents=True, exist_ok=True)

    base_name = filepath.stem
    fig1_path = export_dir / f"{base_name}_figure1.png"
    fig2_path = export_dir / f"{base_name}_figure2.png"
    json_path = export_dir / f"{base_name}_summary.json"

    fig1.savefig(fig1_path, dpi=180, bbox_inches="tight")
    fig2.savefig(fig2_path, dpi=180, bbox_inches="tight")

    summary["artifacts"] = {
        "figure_1_png": str(fig1_path),
        "figure_2_png": str(fig2_path),
        "summary_json": str(json_path),
    }

    with json_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)

    return {
        "figure_1_png": str(fig1_path),
        "figure_2_png": str(fig2_path),
        "summary_json": str(json_path),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot Robust KalmanNet vs EKF comparator logs"
    )
    parser.add_argument(
        "--file",
        help="Comparator CSV file. If omitted, the latest file under logs/comparator is used.",
    )
    parser.add_argument(
        "--output-dir",
        help="Directory where the PNG figures and JSON summary are saved.",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Save outputs without opening the Matplotlib window.",
    )
    args = parser.parse_args()

    filepath = _resolve_input_file(args.file)
    data = _load_csv_rows(filepath)
    time_axis = data["timestamp"] - data["timestamp"][0]

    fig1, axes1 = plt.subplots(3, 2, figsize=(12, 8))
    fig1.suptitle(f"RKNet Comparator (1/2): {filepath.name}", fontsize=14)

    # Figure 1, Row 0: Kalman Gain Diagonals and Mask Decision Summary
    ax = axes1[0, 0]
    ax.plot(time_axis, data["K_x_x"], label="K(x,x) -> X", linewidth=1.2)
    ax.plot(time_axis, data["K_y_y"], label="K(y,y) -> Y", linewidth=1.2)
    ax.plot(time_axis, data["K_psi_psi"], label="K(psi,psi) -> Head", linewidth=1.2)
    ax.plot(time_axis, data["K_v_v"], label="K(v,v) -> V", linewidth=1.2)
    ax.set_title("Kalman Gain (K) Diagonals")
    ax.set_xlabel("time [s]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes1[0, 1]
    branch_to_id = {"imu": 0.0, "steer": 1.0, "wheel": 2.0}
    selected_ids = np.asarray(
        [branch_to_id.get(name, np.nan) for name in data["mask_selected_branch"]],
        dtype=np.float64,
    )
    ax.plot(time_axis, selected_ids, label="selected branch", linewidth=1.3)
    ax.plot(time_axis, data["mask_imu_mean"], label="pred imu mask", alpha=0.5, linestyle=":")
    ax.plot(time_axis, data["mask_wheel_mean"], label="pred wheel mask", alpha=0.5, linestyle=":")
    ax.set_title("Predictor Mask Summary (used only if 'nn' predictor)")
    ax.set_xlabel("time [s]")
    ax.set_yticks([0.0, 1.0, 2.0])
    ax.set_yticklabels(["imu", "steer", "wheel"])
    ax.set_ylim(-0.2, 2.2)
    ax.grid(True, alpha=0.3)
    ax1_twin = ax.twinx()
    ax1_twin.plot(
        time_axis,
        data["mask_selected_score"],
        label="selected score",
        linewidth=1.1,
        color="tab:red",
    )
    ax1_twin.set_ylim(-0.05, 1.05)
    lines_1, labels_1 = ax.get_legend_handles_labels()
    lines_2, labels_2 = ax1_twin.get_legend_handles_labels()
    ax.legend(lines_1 + lines_2, labels_1 + labels_2, loc="upper right", fontsize=8)

    # Figure 1, Row 1: State Differences and Source/GPS Flags
    ax = axes1[1, 0]
    ax.plot(time_axis, data["delta_x"], label="dx")
    ax.plot(time_axis, data["delta_y"], label="dy")
    ax.plot(time_axis, data["delta_theta"], label="dtheta")
    ax.plot(time_axis, data["delta_v"], label="dv")
    ax.set_title("Signed State Differences (Robust - EKF)")
    ax.set_xlabel("time [s]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes1[1, 1]
    model_mask = data["source"] == "model"
    fallback_mask = data["source"] == "fallback"
    ax.plot(time_axis, model_mask.astype(float), label="model used", linewidth=1.2)
    ax.plot(time_axis, fallback_mask.astype(float), label="fallback used", linewidth=1.2)
    ax.plot(time_axis, data["gps_valid"], label="gps valid", linewidth=1.2)
    ax.set_title("Source / GPS Flags")
    ax.set_xlabel("time [s]")
    ax.set_ylim(-0.1, 1.1)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    # Figure 1, Row 2: Measurement Update Masks and Innovations
    ax = axes1[2, 0]
    ax.plot(time_axis, data["meas_mask_x"], label="mask x (GPS)", linewidth=1.2)
    ax.plot(time_axis, data["meas_mask_y"], label="mask y (GPS)", linewidth=1.2)
    ax.plot(time_axis, data["meas_mask_psi"], label="mask psi (IMU)", linewidth=1.2)
    ax.plot(time_axis, data["meas_mask_v"], label="mask v (Wheel)", linewidth=1.2)
    ax.set_title("Measurement Update Masks (Attenuates Innovation)")
    ax.set_xlabel("time [s]")
    ax.set_ylim(-0.05, 1.05)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes1[2, 1]
    ax.plot(time_axis, data["innov_x"], label="innov x", linewidth=1.2)
    ax.plot(time_axis, data["innov_y"], label="innov y", linewidth=1.2)
    ax.plot(time_axis, data["innov_psi"], label="innov psi", linewidth=1.2)
    ax.plot(time_axis, data["innov_v"], label="innov v", linewidth=1.2)
    ax.set_title("Innovations (z - H*x_pred)")
    ax.set_xlabel("time [s]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    fig1.tight_layout()

    # --------------- FIGURE 2 ---------------
    fig2, axes2 = plt.subplots(3, 2, figsize=(12, 8))
    fig2.suptitle(f"RKNet Comparator (2/2): {filepath.name}", fontsize=14)

    # Figure 2, Row 0: Trajectory and Errors
    ax = axes2[0, 0]
    ax.plot(data["robust_x"], data["robust_y"], label="Robust (Update)", linewidth=1.8)
    ax.plot(data["pred_x"], data["pred_y"], label="Robust (Pred)", linewidth=1.2, linestyle=":")
    ax.plot(data["ekf_x"], data["ekf_y"], label="EKF", linewidth=1.4, linestyle="--")
    ax.set_title("XY Trajectory")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes2[0, 1]
    ax.plot(time_axis, data["position_error_norm"], label="Position error norm")
    ax.plot(time_axis, np.abs(data["heading_error"]), label="|Heading error|")
    ax.plot(time_axis, np.abs(data["velocity_error"]), label="|Velocity error|")
    ax.set_title("Absolute Errors")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("Error")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    # Figure 2, Row 1: States (X, Y) over time
    ax = axes2[1, 0]
    ax.plot(time_axis, data["robust_x"], label="Robust X")
    ax.plot(time_axis, data["ekf_x"], label="EKF X", linestyle="--")
    ax.set_title("X Position over Time")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("x [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes2[1, 1]
    ax.plot(time_axis, data["robust_y"], label="Robust Y")
    ax.plot(time_axis, data["ekf_y"], label="EKF Y", linestyle="--")
    ax.set_title("Y Position over Time")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    # Figure 2, Row 2: States (Theta, V) over time
    ax = axes2[2, 0]
    ax.plot(time_axis, data["robust_theta"], label="Robust Theta")
    ax.plot(time_axis, data["ekf_theta"], label="EKF Theta", linestyle="--")
    ax.set_title("Heading (Theta) over Time")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("theta [rad]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes2[2, 1]
    ax.plot(time_axis, data["robust_v"], label="Robust V")
    ax.plot(time_axis, data["ekf_v"], label="EKF V", linestyle="--")
    ax.set_title("Velocity over Time")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("v [m/s]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    fig2.tight_layout()
    summary = _compute_summary(filepath, data, time_axis)
    artifacts = _export_artifacts(filepath, fig1, fig2, summary, args.output_dir)

    print(f"Saved figure 1: {artifacts['figure_1_png']}")
    print(f"Saved figure 2: {artifacts['figure_2_png']}")
    print(f"Saved summary: {artifacts['summary_json']}")

    if not args.no_show:
        plt.show()
    else:
        plt.close(fig1)
        plt.close(fig2)


if __name__ == "__main__":
    main()
