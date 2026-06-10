"""
Plot recordings from one ParallelObserverEstimator trial.

Usage:
    python plot_parallel_observers.py --vehicle-id 1
    python plot_parallel_observers.py --trial-dir path/to/trial_v1_parallel_observers_*
    python plot_parallel_observers.py --output-dir path/to/observer_recordings/attack_timestamp
    python plot_parallel_observers.py --trial-dir path/to/trial --leader-csv path/to/telemetry_vehicle_0.csv
"""
import argparse
import glob
import os
from typing import Dict, Optional

from plot_leadering_observer import (
    _copy_leader_telemetry_to_observer_dir,
    default_data_log_dir,
    default_output_dir,
    find_latest_leader_telemetry,
    plot_leadering_observer,
)


OBSERVER_PREFIXES = {
    "leadering_observer": "leadering",
    "classical_luenberger_observer": "classical Luenberger",
    "high_gain_luenberger_observer": "high-gain Luenberger",
    "classical_ekf_observer": "classical EKF",
}


def _latest_file(pattern: str) -> Optional[str]:
    files = glob.glob(pattern, recursive=True)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def find_latest_parallel_trial(output_dir: str, vehicle_id: int) -> Optional[str]:
    pattern = os.path.join(output_dir, "**", f"trial_v{vehicle_id}_parallel_observers_*")
    trial_dirs = [path for path in glob.glob(pattern, recursive=True) if os.path.isdir(path)]
    if not trial_dirs:
        return None
    return max(trial_dirs, key=os.path.getmtime)


def find_observer_csvs(trial_dir: str, vehicle_id: int) -> Dict[str, str]:
    csv_paths = {}
    for prefix in OBSERVER_PREFIXES:
        pattern = os.path.join(trial_dir, f"{prefix}_v{vehicle_id}_*.csv")
        csv_path = _latest_file(pattern)
        if csv_path is not None:
            csv_paths[prefix] = csv_path
    return csv_paths


def find_trial_leader_telemetry(trial_dir: str) -> Optional[str]:
    return _latest_file(os.path.join(trial_dir, "telemetry_vehicle_0.csv"))


def plot_parallel_observers(trial_dir: str, vehicle_id: int = 1,
                            leader_csv_path: Optional[str] = None,
                            save: bool = True) -> Optional[str]:
    csv_paths = find_observer_csvs(trial_dir, vehicle_id)
    if not csv_paths:
        print(f"No observer CSV files found in {trial_dir}")
        return None

    primary_csv_path = csv_paths.get("leadering_observer")
    if primary_csv_path is None:
        print(
            "No leadering_observer CSV found. The existing comparison plotter "
            "uses leadering_observer as the primary time base."
        )
        return None

    print(f"Using parallel observer trial directory: {trial_dir}")
    for prefix, label in OBSERVER_PREFIXES.items():
        csv_path = csv_paths.get(prefix)
        if csv_path is not None:
            print(f"Using {label} CSV file: {csv_path}")
        else:
            print(f"Missing {label} CSV file in trial directory.")

    if leader_csv_path is not None:
        print(f"Using leader telemetry CSV file: {leader_csv_path}")
        copied_leader_csv_path = _copy_leader_telemetry_to_observer_dir(
            leader_csv_path,
            primary_csv_path,
        )
        if copied_leader_csv_path is not None:
            print(f"Copied leader telemetry CSV to: {copied_leader_csv_path}")
    else:
        print("No telemetry_vehicle_0.csv found; using V2V true_leader_* columns for state comparison.")

    return plot_leadering_observer(
        primary_csv_path,
        save=save,
        compare_csv_path=csv_paths.get("classical_luenberger_observer"),
        high_gain_csv_path=csv_paths.get("high_gain_luenberger_observer"),
        ekf_csv_path=csv_paths.get("classical_ekf_observer"),
        leader_csv_path=leader_csv_path,
    )


def main():
    parser = argparse.ArgumentParser(description="Plot all ParallelObserverEstimator recordings in one trial.")
    parser.add_argument(
        "--trial-dir",
        type=str,
        help="Directory containing one parallel observer trial CSV set.",
    )
    parser.add_argument("--vehicle-id", type=int, default=1, help="Observer vehicle ID. Default: 1.")
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Directory containing observer recordings. Used to auto-find the latest parallel trial.",
    )
    parser.add_argument(
        "--leader-csv",
        type=str,
        help=(
            "Path to leader telemetry_vehicle_0.csv. If omitted, the script first checks "
            "the trial directory, then the latest telemetry_vehicle_0.csv under --data-log-dir."
        ),
    )
    parser.add_argument("--data-log-dir", type=str, default=None, help="Directory containing telemetry data_logs.")
    parser.add_argument("--no-save", action="store_true", help="Do not save the figure PNG.")
    args = parser.parse_args()

    output_dir = os.path.abspath(args.output_dir) if args.output_dir else default_output_dir()
    trial_dir = os.path.abspath(args.trial_dir) if args.trial_dir else None
    if trial_dir is None:
        trial_dir = find_latest_parallel_trial(output_dir, args.vehicle_id)
        if trial_dir is None:
            print(f"No parallel observer trial found in {output_dir}")
            return

    leader_csv_path = args.leader_csv
    if leader_csv_path is None:
        leader_csv_path = find_trial_leader_telemetry(trial_dir)
    if leader_csv_path is None:
        data_log_dir = os.path.abspath(args.data_log_dir) if args.data_log_dir else default_data_log_dir()
        leader_csv_path = find_latest_leader_telemetry(data_log_dir, leader_vehicle_id=0)

    plot_parallel_observers(
        trial_dir,
        vehicle_id=args.vehicle_id,
        leader_csv_path=leader_csv_path,
        save=not args.no_save,
    )


if __name__ == "__main__":
    main()
