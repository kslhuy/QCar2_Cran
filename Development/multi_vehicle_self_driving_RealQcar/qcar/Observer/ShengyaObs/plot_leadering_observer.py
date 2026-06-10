"""
Plot LeaderingObserver recordings.

Usage:
    python plot_leadering_observer.py --vehicle-id 1
    python plot_leadering_observer.py --csv path/to/leadering_observer_v1_*.csv
    python plot_leadering_observer.py --csv path/to/leadering.csv --leader-csv path/to/telemetry_vehicle_0.csv
    python plot_leadering_observer.py --include-baselines
"""
import argparse
import glob
import os
import shutil
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

INNOVATION_BASELINE_START_S = 30.0
INNOVATION_BASELINE_END_S = 40.0
INNOVATION_ATTACK_START_S = 40.0
INNOVATION_ATTACK_END_S = 40.5
INNOVATION_SIGMA_MULTIPLIER = 3.0
ERROR_STATS_START_S = 40.0
ERROR_STATS_END_S = 40.5
HAT_TAU_ATTACK_START_S = 40.0
HAT_TAU_ATTACK_DURATION_S = 0.5
HAT_TAU_ATTACK_BIAS_S = 2.5


def default_output_dir() -> str:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(script_dir, "..", "..", "observer_recordings"))


def default_data_log_dir() -> str:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(script_dir, "..", "..", "data_logs"))


def find_latest_file(output_dir: str, vehicle_id: int,
                     prefix: str = "leadering_observer") -> Optional[str]:
    pattern = os.path.join(output_dir, "**", f"{prefix}_v{vehicle_id}_*.csv")
    files = glob.glob(pattern, recursive=True)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def find_latest_leader_telemetry(data_log_dir: str, leader_vehicle_id: int = 0) -> Optional[str]:
    pattern = os.path.join(data_log_dir, "**", f"telemetry_vehicle_{leader_vehicle_id}.csv")
    files = glob.glob(pattern, recursive=True)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def _raw_time(df: pd.DataFrame, csv_path: str) -> pd.Series:
    if "timestamp" in df.columns:
        return pd.to_numeric(df["timestamp"], errors="coerce")
    if "time" in df.columns:
        return pd.to_numeric(df["time"], errors="coerce")
    raise ValueError(f"CSV file must contain a 'time' or 'timestamp' column: {csv_path}")


def _relative_time(df: pd.DataFrame, csv_path: str) -> pd.Series:
    t = _raw_time(df, csv_path)
    return t - t.min()


def _has_timestamp(dataset: dict) -> bool:
    return "timestamp" in dataset["df"].columns


def _apply_common_timestamp_timebase(datasets, raw_leader: Optional[dict]) -> bool:
    all_datasets = list(datasets)
    if raw_leader is not None:
        all_datasets.append(raw_leader)
    if not all_datasets or not all(_has_timestamp(dataset) for dataset in all_datasets):
        return False

    starts = []
    for dataset in all_datasets:
        ts = pd.to_numeric(dataset["df"]["timestamp"], errors="coerce")
        finite_ts = ts[np.isfinite(ts)]
        if finite_ts.empty:
            return False
        starts.append(float(finite_ts.min()))

    common_t0 = max(starts)
    for dataset in all_datasets:
        dataset["t"] = pd.to_numeric(dataset["df"]["timestamp"], errors="coerce") - common_t0
    return True


def _first_series(df: pd.DataFrame, names) -> Optional[pd.Series]:
    for name in names:
        data = _series(df, name)
        if data is not None:
            return data
    return None


def _series(df: pd.DataFrame, name: str) -> Optional[pd.Series]:
    if name not in df.columns:
        return None
    data = pd.to_numeric(df[name], errors="coerce")
    if data.notna().sum() == 0:
        return None
    return data


def _leader_truth_dataset(leader_csv_path: Optional[str]) -> Optional[dict]:
    if not leader_csv_path:
        return None
    df = pd.read_csv(leader_csv_path)
    truth = {}
    truth["x"] = _first_series(df, ("x", "true_leader_x"))
    truth["v"] = _first_series(df, ("v", "true_leader_v"))
    truth["a"] = _first_series(df, ("accel_x", "acceleration", "accel_magnitude", "true_leader_a"))
    truth = {name: data for name, data in truth.items() if data is not None}
    if not truth:
        return None
    return {
        "path": leader_csv_path,
        "label": "leader telemetry",
        "df": df,
        "t": _relative_time(df, leader_csv_path),
        "truth": truth,
    }


def _copy_leader_telemetry_to_observer_dir(leader_csv_path: Optional[str], observer_csv_path: str) -> Optional[str]:
    if not leader_csv_path:
        return None

    source_path = os.path.abspath(leader_csv_path)
    target_dir = os.path.dirname(os.path.abspath(observer_csv_path))
    target_path = os.path.join(target_dir, os.path.basename(source_path))

    if os.path.abspath(source_path) == os.path.abspath(target_path):
        return target_path

    try:
        shutil.copy2(source_path, target_path)
        return target_path
    except OSError as exc:
        print(f"Failed to copy leader telemetry CSV to observer directory: {exc}")
        return None


def _innovation_threshold_from_baseline(t: pd.Series, innovation: pd.Series) -> Optional[dict]:
    baseline_mask = (t >= INNOVATION_BASELINE_START_S) & (t < INNOVATION_BASELINE_END_S)
    baseline = innovation[baseline_mask].dropna()
    if baseline.empty:
        return None

    mean = float(baseline.mean())
    std = float(baseline.std(ddof=0))
    margin = INNOVATION_SIGMA_MULTIPLIER * std
    return {
        "mean": mean,
        "std": std,
        "upper": mean + margin,
        "lower": mean - margin,
    }


def _print_innovation_detection_stats(t: pd.Series, innovation: pd.Series, threshold: dict) -> None:
    windows = [
        ("baseline", INNOVATION_BASELINE_START_S, INNOVATION_BASELINE_END_S),
        ("attack", INNOVATION_ATTACK_START_S, INNOVATION_ATTACK_END_S),
        ("post", INNOVATION_ATTACK_END_S, float(t.max())),
    ]

    print(
        "Innovation steady-state threshold: "
        f"mean={threshold['mean']:.6g}, std={threshold['std']:.6g}, "
        f"lower={threshold['lower']:.6g}, upper={threshold['upper']:.6g}"
    )
    for name, start_s, end_s in windows:
        mask = (t >= start_s) & (t < end_s)
        data = innovation[mask].dropna()
        if data.empty:
            continue
        detected = (data < threshold["lower"]) | (data > threshold["upper"])
        print(
            f"Innovation detection {name} {start_s:g}-{end_s:g}s: "
            f"{int(detected.sum())}/{len(data)} = {100.0 * float(detected.mean()):.2f}%"
        )


def _load_dataset(csv_path: str, label: str) -> dict:
    df = pd.read_csv(csv_path)
    return {
        "path": csv_path,
        "label": label,
        "df": df,
        "t": _relative_time(df, csv_path),
    }


def _format_error_stats(label: str, t: pd.Series, err_data: pd.Series) -> str:
    t_arr = np.asarray(t, dtype=float)
    err = np.asarray(err_data, dtype=float)
    mask = (
        np.isfinite(t_arr)
        & np.isfinite(err)
        & (t_arr >= ERROR_STATS_START_S)
        & (t_arr < ERROR_STATS_END_S)
    )
    valid = err[mask]
    if valid.size == 0:
        return f"{label}: no valid error {ERROR_STATS_START_S:g}-{ERROR_STATS_END_S:g}s"

    rmse = float(np.sqrt(np.mean(valid ** 2)))
    mean = float(np.mean(valid))
    mae = float(np.mean(np.abs(valid)))
    return f"{label}: RMSE={rmse:.4g}, mean={mean:.4g}, MAE={mae:.4g}"


def plot_leadering_observer(csv_path: str, save: bool = True,
                            compare_csv_path: Optional[str] = None,
                            high_gain_csv_path: Optional[str] = None,
                            ekf_csv_path: Optional[str] = None,
                            leader_csv_path: Optional[str] = None) -> Optional[str]:
    datasets = [_load_dataset(csv_path, "leadering observer")]
    if compare_csv_path:
        datasets.append(_load_dataset(compare_csv_path, "classical Luenberger"))
    if high_gain_csv_path:
        datasets.append(_load_dataset(high_gain_csv_path, "high-gain Luenberger"))
    if ekf_csv_path:
        datasets.append(_load_dataset(ekf_csv_path, "classical EKF"))

    df = datasets[0]["df"]
    t = datasets[0]["t"]
    raw_leader = _leader_truth_dataset(leader_csv_path)
    timestamp_aligned = _apply_common_timestamp_timebase(datasets, raw_leader)
    t = datasets[0]["t"]
    true_source_label = "leader telemetry CSV" if raw_leader is not None else "V2V truth in observer CSV"
    timebase_label = "timestamp-aligned" if timestamp_aligned else "relative start-aligned"

    fig, axes = plt.subplots(5, 2, figsize=(13, 12), sharex=True)
    axs = axes.ravel()

    state_specs = [
        ("x", "Position [m]", "Position error [m]", "true_leader_x", "zeta_hat_x", "err_x"),
        ("v", "Velocity [m/s]", "Velocity error [m/s]", "true_leader_v", "zeta_hat_v", "err_v"),
        ("a", "Acceleration [m/s^2]", "Acceleration error [m/s^2]", "true_leader_a", "zeta_hat_a", "err_a"),
    ]

    for idx, (name, ylabel, err_ylabel, true_col, hat_col, err_col) in enumerate(state_specs):
        true_t = t
        true_data = _series(df, true_col)
        true_label = f"leader true {name} (V2V)"
        if raw_leader is not None and name in raw_leader["truth"]:
            true_t = raw_leader["t"]
            true_data = raw_leader["truth"][name]
            true_label = f"leader telemetry {name}"

        if true_data is not None:
            axs[idx].plot(true_t, true_data, label=true_label, linewidth=2.2, color="#111111")
        for dataset in datasets:
            data_df = dataset["df"]
            data_t = dataset["t"]
            hat_data = _series(data_df, hat_col)
            if hat_data is not None:
                axs[idx].plot(
                    data_t,
                    hat_data,
                    label=f"{dataset['label']} {name}_hat",
                    linestyle="--",
                    linewidth=2,
                )
        axs[idx].set_ylabel(ylabel)
        axs[idx].set_title(f"Leader state vs estimates {name} ({true_source_label})")
        if name == "v":
            axs[idx].set_ylim(0.0, 1.25)
        elif name == "a":
            axs[idx].set_ylim(-1.0, 1.0)
        axs[idx].legend()

        error_stats = []
        error_ax = axs[idx + 4]
        for dataset in datasets:
            data_df = dataset["df"]
            data_t = dataset["t"]
            err_data = _series(data_df, err_col)
            data_true = _series(data_df, true_col)
            hat_data = _series(data_df, hat_col)
            # Keep fallback error on the observer/V2V time base for alignment with
            # recorded communication measurements.
            if err_data is None and data_true is not None and hat_data is not None:
                err_data = hat_data - data_true
            if err_data is not None:
                error_ax.plot(data_t, err_data, label=f"{dataset['label']} error")
                error_stats.append(_format_error_stats(dataset["label"], data_t, err_data))

        error_ax.axhline(0.0, label="0", color="#2ca02c", linestyle=":", linewidth=1.4)
        if name in ("v", "a"):
            error_ax.set_ylim(-0.5, 0.5)
        error_ax.axvspan(
            ERROR_STATS_START_S,
            ERROR_STATS_END_S,
            color="#f2c94c",
            alpha=0.13,
            label="RMSE window" if idx == 0 else None,
        )
        error_ax.set_title(f"Estimation error {name} (hat - V2V truth)")
        error_ax.set_ylabel(err_ylabel)
        if error_stats:
            error_ax.text(
                0.01,
                0.98,
                f"Stats {ERROR_STATS_START_S:g}-{ERROR_STATS_END_S:g}s\n" + "\n".join(error_stats),
                transform=error_ax.transAxes,
                va="top",
                ha="left",
                fontsize=8,
                bbox={
                    "facecolor": "white",
                    "edgecolor": "#d0d0d0",
                    "alpha": 0.82,
                    "boxstyle": "round,pad=0.25",
                },
            )

    y_zeta = _series(df, "y_zeta")
    if y_zeta is not None:
        axs[3].plot(t, y_zeta, label="filtered delayed output y_zeta", linewidth=2.2, color="#111111")
    for dataset in datasets:
        zeta0 = _series(dataset["df"], "zeta_hat_0")
        if zeta0 is not None:
            axs[3].plot(dataset["t"], zeta0, label=f"{dataset['label']} zeta_hat_0", linestyle="--", linewidth=2)
    axs[3].set_ylabel("Integral output")
    axs[3].set_title("Output state reconstruction")
    axs[3].legend()

    for dataset in datasets:
        hat_tau = _series(dataset["df"], "hat_tau")
        if hat_tau is not None:
            axs[7].plot(dataset["t"], hat_tau, label=f"{dataset['label']} hat_tau", linewidth=2)
    axs[7].axvspan(
        HAT_TAU_ATTACK_START_S,
        HAT_TAU_ATTACK_START_S + HAT_TAU_ATTACK_DURATION_S,
        label=f"hat_tau attack +{HAT_TAU_ATTACK_BIAS_S:g}s",
        color="#d62728",
        alpha=0.16,
    )
    axs[7].set_ylabel("Delay [s]")
    axs[7].set_title("Estimated communication delay hat_tau(t)")
    axs[7].legend()

    threshold = None
    for dataset in datasets:
        innovation = _series(dataset["df"], "innovation")
        if innovation is not None:
            axs[8].plot(dataset["t"], innovation, label=f"{dataset['label']} innovation", linewidth=2)
            if threshold is None:
                threshold = _innovation_threshold_from_baseline(dataset["t"], innovation)
    axs[8].axhline(0.0, label="0", color="#2ca02c", linestyle=":", linewidth=1.8)
    if threshold is not None:
        primary_innovation = _series(df, "innovation")
        if primary_innovation is not None:
            _print_innovation_detection_stats(t, primary_innovation, threshold)
    axs[8].set_ylabel("Innovation")
    axs[8].set_title("Innovation steady-state detection")
    axs[8].set_ylim(-0.1, 0.1)
    axs[8].axvspan(
        HAT_TAU_ATTACK_START_S,
        HAT_TAU_ATTACK_START_S + HAT_TAU_ATTACK_DURATION_S,
        label=f"hat_tau attack {HAT_TAU_ATTACK_START_S:g}-{HAT_TAU_ATTACK_START_S + HAT_TAU_ATTACK_DURATION_S:g}s",
        color="#d62728",
        alpha=0.12,
    )
    axs[8].legend()

    axs[9].axis("off")

    for ax in axs:
        ax.grid(True, alpha=0.3)
    for ax in axs[-2:]:
        ax.set_xlabel("Time [s]")

    fig.suptitle(
        "Leadering Observer\n"
        f"State comparison uses leader telemetry ({timebase_label}).",
        fontsize=12,
    )
    fig.tight_layout(rect=[0, 0.03, 1, 0.94])

    figure_path = None
    if save:
        base = os.path.splitext(os.path.basename(csv_path))[0]
        suffix = "_comparison" if compare_csv_path or high_gain_csv_path or ekf_csv_path else "_leadering_observer"
        figure_path = os.path.join(os.path.dirname(csv_path), f"{base}{suffix}.png")
        fig.savefig(figure_path, dpi=200, bbox_inches="tight")
        print(f"Saved figure: {figure_path}")

    plt.show(block=False)
    return figure_path


def main():
    parser = argparse.ArgumentParser(description="Plot LeaderingObserverEstimator recordings.")
    parser.add_argument("--csv", type=str, help="Path to a leadering_observer_v*.csv file.")
    parser.add_argument("--compare-csv", type=str, help="Path to a classical_luenberger_observer_v*.csv file.")
    parser.add_argument("--high-gain-csv", type=str, help="Path to a high_gain_luenberger_observer_v*.csv file.")
    parser.add_argument("--ekf-csv", type=str, help="Path to a classical_ekf_observer_v*.csv file.")
    parser.add_argument(
        "--include-baselines",
        action="store_true",
        help="Also auto-load classical, high-gain, and EKF baseline CSV files when paths are omitted.",
    )
    parser.add_argument(
        "--leader-csv",
        type=str,
        help=(
            "Path to leader telemetry_vehicle_0.csv. If omitted, the latest "
            "telemetry_vehicle_0.csv under --data-log-dir is used when available."
        ),
    )
    parser.add_argument("--vehicle-id", type=int, default=1, help="Observer vehicle ID. Default: 1.")
    parser.add_argument("--output-dir", type=str, default=None, help="Directory containing observer recordings.")
    parser.add_argument("--data-log-dir", type=str, default=None, help="Directory containing telemetry data_logs.")
    parser.add_argument("--no-save", action="store_true", help="Do not save the figure PNG.")
    args = parser.parse_args()

    csv_path = args.csv
    output_dir = os.path.abspath(args.output_dir) if args.output_dir else default_output_dir()
    data_log_dir = os.path.abspath(args.data_log_dir) if args.data_log_dir else default_data_log_dir()
    if csv_path is None:
        csv_path = find_latest_file(output_dir, args.vehicle_id, prefix="leadering_observer")
        if csv_path is None:
            print(f"No leadering observer recording found in {output_dir}")
            return
    compare_csv_path = args.compare_csv
    if args.include_baselines and compare_csv_path is None:
        compare_csv_path = find_latest_file(
            output_dir,
            args.vehicle_id,
            prefix="classical_luenberger_observer",
        )
    high_gain_csv_path = args.high_gain_csv
    if args.include_baselines and high_gain_csv_path is None:
        high_gain_csv_path = find_latest_file(
            output_dir,
            args.vehicle_id,
            prefix="high_gain_luenberger_observer",
        )
    ekf_csv_path = args.ekf_csv
    if args.include_baselines and ekf_csv_path is None:
        ekf_csv_path = find_latest_file(
            output_dir,
            args.vehicle_id,
            prefix="classical_ekf_observer",
        )
    leader_csv_path = args.leader_csv
    if leader_csv_path is None:
        leader_csv_path = find_latest_leader_telemetry(data_log_dir, leader_vehicle_id=0)

    print(f"Using CSV file: {csv_path}")
    if compare_csv_path is not None:
        print(f"Using compare CSV file: {compare_csv_path}")
    if high_gain_csv_path is not None:
        print(f"Using high-gain CSV file: {high_gain_csv_path}")
    if ekf_csv_path is not None:
        print(f"Using EKF CSV file: {ekf_csv_path}")
    if leader_csv_path is not None:
        print(f"Using leader telemetry CSV file: {leader_csv_path}")
        copied_leader_csv_path = _copy_leader_telemetry_to_observer_dir(leader_csv_path, csv_path)
        if copied_leader_csv_path is not None:
            print(f"Copied leader telemetry CSV to: {copied_leader_csv_path}")
    else:
        print("No telemetry_vehicle_0.csv found; using V2V true_leader_* columns for state comparison.")
    plot_leadering_observer(
        csv_path,
        save=not args.no_save,
        compare_csv_path=compare_csv_path,
        high_gain_csv_path=high_gain_csv_path,
        ekf_csv_path=ekf_csv_path,
        leader_csv_path=leader_csv_path,
    )


if __name__ == "__main__":
    main()
