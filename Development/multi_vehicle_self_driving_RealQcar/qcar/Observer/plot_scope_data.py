"""
Plot recorded local and fleet observer CSV files.

Examples:
    python plot_scope_data.py --type local
    python plot_scope_data.py --type fleet
    python plot_scope_data.py --type both
    python plot_scope_data.py --file scope_recordings/local/local_V0.csv
    python plot_scope_data.py --dir scope_recordings --type both
"""

import argparse
import glob
import os
from typing import List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


DEFAULT_RECORDING_DIRS = [
    "scope_recordings",
    os.path.join("GUI", "scope_recordings"),
    os.path.join("..", "scope_recordings"),
    os.path.join("..", "GUI", "scope_recordings"),
    os.path.join("..", "..", "scope_recordings"),
    os.path.join("..", "..", "GUI", "scope_recordings"),
]


def find_recording_dir(explicit_dir: Optional[str] = None) -> Optional[str]:
    """Return a recording directory containing local and/or fleet CSV folders."""
    candidates = [explicit_dir] if explicit_dir else DEFAULT_RECORDING_DIRS

    for candidate in candidates:
        if not candidate:
            continue
        local_dir = os.path.join(candidate, "local")
        fleet_dir = os.path.join(candidate, "fleet")
        if os.path.isdir(local_dir) or os.path.isdir(fleet_dir):
            return os.path.abspath(candidate)

    return None


def list_csv_files(recording_dir: str, data_type: str) -> List[str]:
    """Return all CSV files for a local or fleet recording folder."""
    subdir = os.path.join(recording_dir, data_type)
    if not os.path.isdir(subdir):
        return []
    return sorted(glob.glob(os.path.join(subdir, "*.csv")))


def latest_csv(recording_dir: str, data_type: str) -> Optional[str]:
    """Return the most recent CSV for the requested recording type."""
    files = list_csv_files(recording_dir, data_type)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def detect_file_type(filepath: str) -> str:
    """Infer whether a CSV is local or fleet data."""
    lowered = filepath.lower()
    if "local" in lowered:
        return "local"
    if "fleet" in lowered:
        return "fleet"

    df = pd.read_csv(filepath, nrows=1)
    return "fleet" if any(col.startswith("fleet_") for col in df.columns) else "local"


def load_recording(filepath: str) -> pd.DataFrame:
    """Load a CSV recording and normalize time to start at zero."""
    df = pd.read_csv(filepath)
    if "time" not in df.columns:
        raise ValueError(f"'time' column not found in {filepath}")

    df = df.copy()
    df["time"] = df["time"] - df["time"].iloc[0]
    return df


def plot_local_data(filepath: str, axes: Optional[dict] = None) -> None:
    """Plot one local observer CSV recording."""
    df = load_recording(filepath)
    standalone = axes is None

    if standalone:
        fig = plt.figure(figsize=(15, 11))
        grid = fig.add_gridspec(3, 2)
        ax_traj = fig.add_subplot(grid[0:2, :])
        ax_vel = fig.add_subplot(grid[2, 0])
        ax_heading = fig.add_subplot(grid[2, 1])
        ax_control = None
    else:
        ax_traj = axes.get("trajectory")
        ax_vel = axes.get("velocity")
        ax_heading = axes.get("heading")
        ax_control = axes.get("control")

    times = df["time"]

    if ax_traj is not None:
        ax_traj.plot(df["x"], df["y"], label="Estimated path", linewidth=2)

        if {"x_gps", "y_gps"}.issubset(df.columns):
            if "gps_valid" in df.columns:
                gps_df = df[df["gps_valid"] > 0.5]
            else:
                gps_df = df

            if not gps_df.empty:
                ax_traj.plot(
                    gps_df["x_gps"],
                    gps_df["y_gps"],
                    "r.",
                    label="GPS",
                    alpha=0.5,
                    markersize=3,
                )

        ax_traj.set_title(f"Local trajectory: {os.path.basename(filepath)}")
        ax_traj.set_xlabel("X [m]")
        ax_traj.set_ylabel("Y [m]")
        ax_traj.grid(True)
        ax_traj.axis("equal")
        ax_traj.legend()

    if ax_vel is not None and "velocity" in df.columns:
        ax_vel.plot(times, df["velocity"], label="Velocity", linewidth=2)
        if "v_ref" in df.columns:
            ax_vel.plot(times, df["v_ref"], "--", label="v_ref")
        if "acceleration" in df.columns:
            ax_vel_twin = ax_vel.twinx()
            ax_vel_twin.plot(times, df["acceleration"], "r:", label="Acceleration")
            ax_vel_twin.set_ylabel("Acceleration [m/s^2]")
        ax_vel.set_title("Velocity")
        ax_vel.set_xlabel("Time [s]")
        ax_vel.set_ylabel("Speed [m/s]")
        ax_vel.grid(True)
        ax_vel.legend(loc="upper left")

    if ax_heading is not None and "theta" in df.columns:
        ax_heading.plot(times, np.degrees(df["theta"]), label="Estimated heading")
        if "theta_gps" in df.columns:
            if "gps_valid" in df.columns:
                gps_df = df[df["gps_valid"] > 0.5]
            else:
                gps_df = df
            if not gps_df.empty:
                ax_heading.plot(
                    gps_df["time"],
                    np.degrees(gps_df["theta_gps"]),
                    "r.",
                    label="GPS heading",
                    markersize=3,
                )
        ax_heading.set_title("Heading")
        ax_heading.set_xlabel("Time [s]")
        ax_heading.set_ylabel("Angle [deg]")
        ax_heading.grid(True)
        ax_heading.legend()

    if ax_control is not None:
        if "steering" in df.columns:
            ax_control.plot(times, df["steering"], label="Steering")
        if "throttle" in df.columns:
            ax_control.plot(times, df["throttle"], label="Throttle")
        ax_control.set_title("Control inputs")
        ax_control.set_xlabel("Time [s]")
        ax_control.grid(True)
        ax_control.legend()

    if standalone:
        plt.tight_layout()
        plt.show()


def _fleet_vehicle_indices(df: pd.DataFrame) -> List[int]:
    indices = []
    for column in df.columns:
        if column.startswith("fleet_x_"):
            try:
                indices.append(int(column.rsplit("_", 1)[-1]))
            except ValueError:
                pass
    return sorted(set(indices))


def plot_fleet_data(
    filepath: str,
    axes: Optional[dict] = None,
    selected_vehicles: Optional[Sequence[int]] = None,
) -> None:
    """Plot one fleet observer CSV recording."""
    df = load_recording(filepath)
    vehicle_indices = _fleet_vehicle_indices(df)

    if selected_vehicles is not None:
        vehicle_indices = [idx for idx in vehicle_indices if idx in selected_vehicles]

    if not vehicle_indices:
        raise ValueError(f"No fleet vehicle columns found in {filepath}")

    standalone = axes is None
    if standalone:
        fig = plt.figure(figsize=(15, 10))
        grid = fig.add_gridspec(2, 2)
        ax_traj = fig.add_subplot(grid[0, 0])
        ax_consensus = fig.add_subplot(grid[0, 1])
        ax_trust = fig.add_subplot(grid[1, :])
    else:
        ax_traj = axes.get("trajectory")
        ax_consensus = axes.get("consensus")
        ax_trust = axes.get("trust")

    times = df["time"]
    colors = plt.cm.tab10(np.linspace(0, 1, max(vehicle_indices) + 1))

    if ax_traj is not None:
        for idx in vehicle_indices:
            x_col = f"fleet_x_{idx}"
            y_col = f"fleet_y_{idx}"
            if x_col in df.columns and y_col in df.columns:
                ax_traj.plot(df[x_col], df[y_col], label=f"Vehicle {idx}", color=colors[idx])
                ax_traj.plot(df[x_col].iloc[-1], df[y_col].iloc[-1], "o", color=colors[idx])
        ax_traj.set_title(f"Fleet trajectories: {os.path.basename(filepath)}")
        ax_traj.set_xlabel("X [m]")
        ax_traj.set_ylabel("Y [m]")
        ax_traj.grid(True)
        ax_traj.axis("equal")
        ax_traj.legend()

    if ax_consensus is not None:
        if "consensus_error" in df.columns:
            ax_consensus.plot(times, df["consensus_error"], "k-", label="Consensus error")
            ax_consensus.legend()
        else:
            ax_consensus.text(0.5, 0.5, "No consensus_error column", ha="center", va="center")
        ax_consensus.set_title("Consensus")
        ax_consensus.set_xlabel("Time [s]")
        ax_consensus.grid(True)

    if ax_trust is not None:
        trust_columns = [col for col in df.columns if col.startswith("trust_")]
        if trust_columns:
            for column in trust_columns:
                idx = int(column.rsplit("_", 1)[-1])
                if idx in vehicle_indices:
                    ax_trust.plot(times, df[column], label=f"Trust {idx}", color=colors[idx])
            ax_trust.legend()
            ax_trust.set_ylim(-0.05, 1.05)
        else:
            ax_trust.text(0.5, 0.5, "No trust columns", ha="center", va="center")
        ax_trust.set_title("Trust scores")
        ax_trust.set_xlabel("Time [s]")
        ax_trust.grid(True)

    if standalone:
        plt.tight_layout()
        plt.show()


def plot_both(
    local_file: Optional[str],
    fleet_file: Optional[str],
    selected_vehicles: Optional[Sequence[int]] = None,
) -> None:
    """Plot local and fleet recordings side by side."""
    if not local_file and not fleet_file:
        raise ValueError("No local or fleet CSV files were provided")

    fig = plt.figure(figsize=(20, 10))
    local_grid = fig.add_gridspec(2, 2, left=0.05, right=0.48)
    fleet_grid = fig.add_gridspec(2, 2, left=0.54, right=0.98)

    if local_file:
        local_axes = {
            "trajectory": fig.add_subplot(local_grid[0, :]),
            "velocity": fig.add_subplot(local_grid[1, 0]),
            "heading": fig.add_subplot(local_grid[1, 1]),
            "control": None,
        }
        plot_local_data(local_file, axes=local_axes)

    if fleet_file:
        fleet_axes = {
            "trajectory": fig.add_subplot(fleet_grid[0, 0]),
            "consensus": fig.add_subplot(fleet_grid[0, 1]),
            "trust": fig.add_subplot(fleet_grid[1, :]),
        }
        plot_fleet_data(fleet_file, axes=fleet_axes, selected_vehicles=selected_vehicles)

    plt.tight_layout()
    plt.show()


def parse_vehicle_list(raw_value: Optional[str]) -> Optional[List[int]]:
    """Parse a comma-separated list of fleet vehicle indices."""
    if not raw_value:
        return None
    return [int(part.strip()) for part in raw_value.split(",") if part.strip()]


def resolve_files(
    recording_dir: Optional[str],
    file_arg: Optional[str],
    data_type: str,
) -> Tuple[Optional[str], Optional[str]]:
    """Resolve the target local and fleet CSV files for plotting."""
    if file_arg:
        file_type = detect_file_type(file_arg)
        return (file_arg, None) if file_type == "local" else (None, file_arg)

    resolved_dir = find_recording_dir(recording_dir)
    if resolved_dir is None:
        raise FileNotFoundError(
            "No recording directory found. Use --dir or provide --file directly."
        )

    if data_type == "local":
        return latest_csv(resolved_dir, "local"), None
    if data_type == "fleet":
        return None, latest_csv(resolved_dir, "fleet")

    return latest_csv(resolved_dir, "local"), latest_csv(resolved_dir, "fleet")


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Plot recorded observer CSV files")
    parser.add_argument(
        "--type",
        choices=["local", "fleet", "both"],
        default="local",
        help="Which recording type to plot when --file is not provided",
    )
    parser.add_argument("--file", help="Plot a specific CSV file")
    parser.add_argument("--dir", help="Recording root directory containing local/ and fleet/")
    parser.add_argument(
        "--vehicles",
        help="Comma-separated fleet vehicle indices to plot, for example: 0,1,3",
    )
    return parser.parse_args()


def main() -> None:
    """Main command-line entry point."""
    args = parse_args()
    selected_vehicles = parse_vehicle_list(args.vehicles)
    local_file, fleet_file = resolve_files(args.dir, args.file, args.type)

    if args.file:
        if local_file:
            plot_local_data(local_file)
        else:
            plot_fleet_data(fleet_file, selected_vehicles=selected_vehicles)
        return

    if args.type == "local":
        if not local_file:
            raise FileNotFoundError("No local CSV recording found")
        plot_local_data(local_file)
        return

    if args.type == "fleet":
        if not fleet_file:
            raise FileNotFoundError("No fleet CSV recording found")
        plot_fleet_data(fleet_file, selected_vehicles=selected_vehicles)
        return

    plot_both(local_file, fleet_file, selected_vehicles=selected_vehicles)


if __name__ == "__main__":
    main()
