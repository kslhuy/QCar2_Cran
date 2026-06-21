"""
Plot distributed high-gain observer recordings.

Examples:
    python plot_distributed_hg.py
    python plot_distributed_hg.py --list
    python plot_distributed_hg.py --legacy
    python plot_distributed_hg.py path/to/dist_hg_v0_20260621_180000.csv --save
"""
import argparse
import csv
import glob
import os
import re
import sys
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import numpy as np

plt = None
GridSpec = None


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
QCAR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))

RECORDING_DIRS = [
    os.path.join(QCAR_DIR, "observer_recordings"),
    os.path.join(QCAR_DIR, "GUI", "observer_recordings"),
    os.path.join(QCAR_DIR, "real_recordings"),
    os.path.join(SCRIPT_DIR, "observer_recordings"),
    os.path.join(SCRIPT_DIR, "real_recordings"),
]


def list_recordings(directories: List[str], pattern: str) -> List[Tuple[str, datetime, int]]:
    recordings = []
    for directory in directories:
        if not os.path.exists(directory):
            continue
        for path in glob.glob(os.path.join(directory, pattern)):
            recordings.append((
                path,
                datetime.fromtimestamp(os.path.getmtime(path)),
                os.path.getsize(path),
            ))
    recordings.sort(key=lambda item: item[1], reverse=True)
    return recordings


def find_latest_recording(directories: List[str], pattern: str) -> Optional[str]:
    recordings = list_recordings(directories, pattern)
    return recordings[0][0] if recordings else None


def format_file_size(size: int) -> str:
    if size < 1024:
        return f"{size} B"
    if size < 1024 * 1024:
        return f"{size / 1024:.1f} KB"
    return f"{size / (1024 * 1024):.2f} MB"


def load_data(filepath: str) -> Dict[str, np.ndarray]:
    with open(filepath, newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        rows = list(reader)

    if not rows:
        raise ValueError(f"Recording is empty: {filepath}")

    data = {}
    for column in reader.fieldnames or []:
        values = []
        for row in rows:
            raw = row.get(column, "")
            try:
                values.append(float(raw))
            except (TypeError, ValueError):
                values.append(np.nan)
        data[column] = np.asarray(values, dtype=float)
    return data


def normalize_time(data: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
    normalized = {key: value.copy() for key, value in data.items()}
    normalized["time"] = normalized["time"] - normalized["time"][0]
    return normalized


def detect_vehicle_ids(data: Dict[str, np.ndarray]) -> List[int]:
    ids = []
    for col in data:
        match = re.fullmatch(r"x_vec_[xp](\d+)", col)
        if match:
            ids.append(int(match.group(1)))
    return sorted(set(ids))


def x_col(prefix: str, vid: int, component: str, data: Dict[str, np.ndarray]) -> Optional[str]:
    candidates = [
        f"{prefix}_{component}{vid}",
        f"{prefix}_p{vid}" if component == "x" else None,
    ]
    for candidate in candidates:
        if candidate and candidate in data:
            return candidate
    return None


def plot_estimated_states(data: Dict[str, np.ndarray], vehicle_ids: List[int],
                          ax_x, ax_v, ax_a) -> None:
    time = data["time"]
    colors = plt.cm.tab10(np.linspace(0, 1, max(len(vehicle_ids), 1)))

    for color, vid in zip(colors, vehicle_ids):
        col_x = x_col("x_vec", vid, "x", data)
        col_v = x_col("x_vec", vid, "v", data)
        col_a = x_col("x_vec", vid, "a", data)
        label = f"Car {vid}"

        if col_x:
            ax_x.plot(time, data[col_x], label=label, color=color, linewidth=1.4)
        if col_v:
            ax_v.plot(time, data[col_v], label=label, color=color, linewidth=1.4)
        if col_a:
            ax_a.plot(time, data[col_a], label=label, color=color, linewidth=1.4)

        true_x = f"true_position_{vid}"
        true_v = f"true_velocity_{vid}"
        true_a = f"true_acceleration_{vid}"
        if true_x in data and not np.isnan(data[true_x]).all():
            ax_x.plot(time, data[true_x], color=color, linestyle="--", linewidth=1.0, alpha=0.75)
        if true_v in data and not np.isnan(data[true_v]).all():
            ax_v.plot(time, data[true_v], color=color, linestyle="--", linewidth=1.0, alpha=0.75)
        if true_a in data and not np.isnan(data[true_a]).all():
            ax_a.plot(time, data[true_a], color=color, linestyle="--", linewidth=1.0, alpha=0.75)

    ax_x.set_title("Estimated Position x (solid) vs True (dashed)")
    ax_v.set_title("Estimated Velocity v (solid) vs True (dashed)")
    ax_a.set_title("Estimated Acceleration a (solid) vs True (dashed)")
    ax_x.set_ylabel("x [m]")
    ax_v.set_ylabel("v [m/s]")
    ax_a.set_ylabel("a [m/s^2]")
    for ax in [ax_x, ax_v, ax_a]:
        ax.grid(True, alpha=0.3, linewidth=0.5)
        ax.legend(loc="best", ncol=2)


def plot_errors(data: Dict[str, np.ndarray], vehicle_ids: List[int], ax) -> None:
    time = data["time"]
    colors = plt.cm.tab10(np.linspace(0, 1, max(len(vehicle_ids), 1)))
    any_line = False

    for color, vid in zip(colors, vehicle_ids):
        est = x_col("x_vec", vid, "x", data)
        true = f"true_position_{vid}"
        if est and true in data and not np.isnan(data[true]).all():
            ax.plot(time, data[est] - data[true], label=f"x err {vid}",
                    color=color, linewidth=1.2)
            any_line = True

    if "meas_err_x" in data:
        ax.plot(time, data["meas_err_x"], label="local meas err",
                color="black", linestyle="--", linewidth=1.1)
        any_line = True
    elif "meas_err_rel_pos" in data:
        ax.plot(time, data["meas_err_rel_pos"], label="local meas err",
                color="black", linestyle="--", linewidth=1.1)
        any_line = True

    ax.set_title("Position Estimation Error")
    ax.set_ylabel("error [m]")
    ax.axhline(0.0, color="0.35", linewidth=0.7)
    ax.grid(True, alpha=0.3, linewidth=0.5)
    if any_line:
        ax.legend(loc="best", ncol=2)


def plot_terms(data: Dict[str, np.ndarray], vid: int, ax) -> None:
    time = data["time"]
    for prefix, label, color in [
        ("dynamics", "dynamics x", "#1f77b4"),
        ("measurement", "measurement x", "#2ca02c"),
        ("consensus", "consensus x", "#d62728"),
    ]:
        col = x_col(prefix, vid, "x", data)
        if col:
            ax.plot(time, data[col], label=label, color=color, linewidth=1.2)

    ax.set_title(f"Observer Terms Applied to Car {vid}")
    ax.set_ylabel("term")
    ax.axhline(0.0, color="0.35", linewidth=0.7)
    ax.grid(True, alpha=0.3, linewidth=0.5)
    ax.legend(loc="best")


def plot_consensus(data: Dict[str, np.ndarray], ax) -> None:
    time = data["time"]
    ax2 = ax.twinx()

    if "consensus_norm" in data:
        ax.plot(time, data["consensus_norm"], label="consensus norm",
                color="#d62728", linewidth=1.2)
    if "neighbor_count" in data:
        ax2.plot(time, data["neighbor_count"], label="neighbors",
                 color="#1f77b4", linewidth=1.2)

    ax.set_title("Consensus Activity")
    ax.set_ylabel("norm", color="#d62728")
    ax2.set_ylabel("neighbor count", color="#1f77b4")
    ax.grid(True, alpha=0.3, linewidth=0.5)

    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc="best")


def plot_controls(data: Dict[str, np.ndarray], vehicle_ids: List[int], ax) -> None:
    time = data["time"]
    colors = plt.cm.tab10(np.linspace(0, 1, max(len(vehicle_ids), 1)))

    for color, vid in zip(colors, vehicle_ids):
        col = f"collective_control_{vid}"
        if col in data:
            ax.plot(time, data[col], label=f"u{vid}", color=color, linewidth=1.2)

    ax.set_title("Collective Throttle From V2V")
    ax.set_ylabel("throttle")
    ax.grid(True, alpha=0.3, linewidth=0.5)
    ax.legend(loc="best", ncol=4)


def plot_fleet_states(data: Dict[str, np.ndarray], vehicle_ids: List[int], ax_x, ax_v) -> None:
    time = data["time"]
    colors = plt.cm.tab10(np.linspace(0, 1, max(len(vehicle_ids), 1)))

    for color, vid in zip(colors, vehicle_ids):
        col_x = f"fleet_x_{vid}"
        col_v = f"fleet_v_{vid}"
        if col_x in data:
            ax_x.plot(time, data[col_x], label=f"Car {vid}", color=color, linewidth=1.2)
        if col_v in data:
            ax_v.plot(time, data[col_v], label=f"Car {vid}", color=color, linewidth=1.2)

    ax_x.set_title("fleet_states x")
    ax_v.set_title("fleet_states v")
    ax_x.set_ylabel("x [m]")
    ax_v.set_ylabel("v [m/s]")
    for ax in [ax_x, ax_v]:
        ax.grid(True, alpha=0.3, linewidth=0.5)
        ax.legend(loc="best", ncol=2)


def create_plot(data: Dict[str, np.ndarray], filepath: str, term_vehicle_id: Optional[int]):
    global plt, GridSpec
    if plt is None or GridSpec is None:
        import matplotlib.pyplot as plt_module
        from matplotlib.gridspec import GridSpec as grid_spec_class

        plt = plt_module
        GridSpec = grid_spec_class
        plt.rcParams.update({
            "font.size": 10,
            "axes.titlesize": 10,
            "axes.labelsize": 10,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "legend.fontsize": 8,
            "figure.titlesize": 11,
        })

    data = normalize_time(data)
    vehicle_ids = detect_vehicle_ids(data)
    if not vehicle_ids:
        raise ValueError("No x_vec vehicle columns found in CSV.")

    selected_vid = term_vehicle_id if term_vehicle_id is not None else vehicle_ids[0]
    if selected_vid not in vehicle_ids:
        selected_vid = vehicle_ids[0]

    basename = os.path.basename(filepath)
    duration = data["time"][-1] - data["time"][0]
    fig = plt.figure(figsize=(15, 13))
    fig.suptitle(
        f"Distributed High-Gain Observer\n{basename} | samples={len(data['time'])} | duration={duration:.2f}s",
        fontweight="bold",
    )

    gs = GridSpec(5, 2, figure=fig, hspace=0.48, wspace=0.28,
                  top=0.92, bottom=0.06, left=0.07, right=0.96)

    ax_x = fig.add_subplot(gs[0, 0])
    ax_v = fig.add_subplot(gs[0, 1])
    ax_a = fig.add_subplot(gs[1, 0])
    plot_estimated_states(data, vehicle_ids, ax_x, ax_v, ax_a)

    ax_err = fig.add_subplot(gs[1, 1])
    plot_errors(data, vehicle_ids, ax_err)

    ax_terms = fig.add_subplot(gs[2, 0])
    plot_terms(data, selected_vid, ax_terms)

    ax_consensus = fig.add_subplot(gs[2, 1])
    plot_consensus(data, ax_consensus)

    ax_fleet_x = fig.add_subplot(gs[3, 0])
    ax_fleet_v = fig.add_subplot(gs[3, 1])
    plot_fleet_states(data, vehicle_ids, ax_fleet_x, ax_fleet_v)

    ax_control = fig.add_subplot(gs[4, :])
    plot_controls(data, vehicle_ids, ax_control)
    ax_control.set_xlabel("Time [s]")

    return fig


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot distributed HG observer recordings")
    parser.add_argument("filepath", nargs="?", help="CSV file to plot")
    parser.add_argument("--dir", "-d", help="Recording directory")
    parser.add_argument("--list", "-l", action="store_true", help="List available recordings")
    parser.add_argument("--legacy", action="store_true",
                        help="Search old dist_luenberger_*.csv names")
    parser.add_argument("--vehicle-id", type=int,
                        help="Vehicle block used for observer-term subplot")
    parser.add_argument("--save", "-s", action="store_true", help="Save figure without asking")
    parser.add_argument("--output", "-o", help="Output PNG path")
    args = parser.parse_args()

    pattern = "dist_luenberger_*.csv" if args.legacy else "dist_hg_*.csv"
    search_dirs = [args.dir] if args.dir else RECORDING_DIRS

    if args.list:
        recordings = list_recordings(search_dirs, pattern)
        if not recordings:
            print(f"No recordings found for pattern {pattern}.")
            return
        for idx, (path, mtime, size) in enumerate(recordings[:30], start=1):
            print(f"{idx:2d}. {mtime:%Y-%m-%d %H:%M:%S} {format_file_size(size):>9}  {path}")
        return

    filepath = args.filepath or find_latest_recording(search_dirs, pattern)
    if not filepath:
        print(f"No recordings found for pattern {pattern}.")
        print("Use --legacy to read old HG logs that were saved with the Luenberger prefix.")
        sys.exit(1)

    data = load_data(filepath)
    fig = create_plot(data, filepath, args.vehicle_id)

    figure_dir = os.path.join(SCRIPT_DIR, "figure")
    os.makedirs(figure_dir, exist_ok=True)
    default_output = os.path.join(
        figure_dir,
        os.path.basename(filepath)
        .replace("dist_hg", "figure_hg")
        .replace("dist_luenberger", "figure_hg_legacy")
        .replace(".csv", ".png"),
    )

    if args.save or args.output:
        output = args.output if args.output else default_output
        os.makedirs(os.path.dirname(output) or ".", exist_ok=True)
        fig.savefig(output, dpi=150, bbox_inches="tight")
        print(f"Figure saved to: {output}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
