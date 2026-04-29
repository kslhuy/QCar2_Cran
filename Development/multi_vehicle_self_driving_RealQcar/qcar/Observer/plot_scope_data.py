from __future__ import annotations

"""
Plot recorded local and fleet observer CSV files.

Examples:
    python plot_scope_data.py
    python plot_scope_data.py fleet
    python plot_scope_data.py local --pick 2
    python plot_scope_data.py both
    python plot_scope_data.py --list fleet
    python plot_scope_data.py --file scope_recordings/fleet/fleet_V0.csv
"""

import argparse
import glob
import os
import sys
from datetime import datetime
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np


DEFAULT_RECORDING_DIRS = [
    "scope_recordings",
    os.path.join("GUI", "scope_recordings"),
    os.path.join("..", "scope_recordings"),
    os.path.join("..", "GUI", "scope_recordings"),
    os.path.join("..", "..", "scope_recordings"),
    os.path.join("..", "..", "GUI", "scope_recordings"),
]


def get_pyplot():
    """Import Matplotlib only when plotting is actually requested."""
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise RuntimeError(
            "Matplotlib could not be imported. The current Python environment "
            "has an incompatible NumPy/Matplotlib installation."
        ) from exc

    return plt


def get_pandas():
    """Import pandas only when CSV loading is actually requested."""
    try:
        import pandas as pd
    except Exception as exc:
        raise RuntimeError(
            "Pandas could not be imported. The current Python environment "
            "has an incompatible NumPy/Pandas installation."
        ) from exc

    return pd


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


def list_csv_files_by_time(recording_dir: str, data_type: str) -> List[str]:
    """Return CSV files sorted from newest to oldest."""
    files = list_csv_files(recording_dir, data_type)
    return sorted(files, key=os.path.getmtime, reverse=True)


def latest_csv(recording_dir: str, data_type: str) -> Optional[str]:
    """Return the most recent CSV for the requested recording type."""
    files = list_csv_files_by_time(recording_dir, data_type)
    return files[0] if files else None


def detect_file_type(filepath: str) -> str:
    """Infer whether a CSV is local or fleet data."""
    lowered = filepath.lower()
    if "local" in lowered:
        return "local"
    if "fleet" in lowered:
        return "fleet"

    pd = get_pandas()
    df = pd.read_csv(filepath, nrows=1)
    return "fleet" if any(col.startswith("fleet_") for col in df.columns) else "local"


def load_recording(filepath: str) -> pd.DataFrame:
    """Load a CSV recording and normalize time to start at zero."""
    pd = get_pandas()
    df = pd.read_csv(filepath)
    if "time" not in df.columns:
        raise ValueError(f"'time' column not found in {filepath}")

    df = df.copy()
    df["time"] = df["time"] - df["time"].iloc[0]
    return df


def plot_local_data(filepath: str, axes: Optional[dict] = None) -> None:
    """Plot one local observer CSV recording."""
    plt = get_pyplot()
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


def _vehicle_has_data(df: pd.DataFrame, vehicle_idx: int) -> bool:
    """Return True when a fleet vehicle column set contains non-placeholder data."""
    vehicle_columns = [
        f"fleet_x_{vehicle_idx}",
        f"fleet_y_{vehicle_idx}",
        f"fleet_theta_{vehicle_idx}",
        f"fleet_v_{vehicle_idx}",
        f"fleet_a_{vehicle_idx}",
    ]

    present_columns = [column for column in vehicle_columns if column in df.columns]
    if not present_columns:
        return False

    values = df[present_columns].fillna(0.0).to_numpy(dtype=float)
    return bool(np.any(np.abs(values) > 1e-12))


def _fleet_vehicle_indices(df: pd.DataFrame) -> List[int]:
    indices = []
    for column in df.columns:
        if column.startswith("fleet_x_"):
            try:
                indices.append(int(column.rsplit("_", 1)[-1]))
            except ValueError:
                pass
    return [idx for idx in sorted(set(indices)) if _vehicle_has_data(df, idx)]


def _build_vehicle_colors(vehicle_indices: Sequence[int]) -> Dict[int, tuple]:
    plt = get_pyplot()
    color_values = plt.cm.tab10(np.linspace(0, 1, max(len(vehicle_indices), 1)))
    return {vehicle_idx: color_values[pos] for pos, vehicle_idx in enumerate(vehicle_indices)}


def _plot_fleet_series(
    ax,
    df: pd.DataFrame,
    times: pd.Series,
    vehicle_indices: Sequence[int],
    colors: Dict[int, tuple],
    column_prefix: str,
    title: str,
    ylabel: str,
    transform: Optional[Callable[[pd.Series], pd.Series]] = None,
) -> None:
    """Plot one fleet state component against time for all selected vehicles."""
    plotted = False

    for idx in vehicle_indices:
        column = f"{column_prefix}_{idx}"
        if column not in df.columns:
            continue

        values = df[column]
        if transform is not None:
            values = transform(values)

        ax.plot(times, values, linewidth=1.8, label=f"Vehicle {idx}", color=colors[idx])
        plotted = True

    if plotted:
        ax.legend()
    else:
        ax.text(0.5, 0.5, f"No {column_prefix} columns", ha="center", va="center")

    ax.set_title(title)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(ylabel)
    ax.grid(True)


def plot_fleet_data(
    filepath: str,
    axes: Optional[dict] = None,
    selected_vehicles: Optional[Sequence[int]] = None,
) -> None:
    """Plot one fleet observer CSV recording."""
    plt = get_pyplot()
    df = load_recording(filepath)
    vehicle_indices = _fleet_vehicle_indices(df)

    if selected_vehicles is not None:
        vehicle_indices = [idx for idx in vehicle_indices if idx in selected_vehicles]

    if not vehicle_indices:
        raise ValueError(f"No fleet vehicle columns found in {filepath}")

    standalone = axes is None
    if standalone:
        fig = plt.figure(figsize=(16, 10))
        grid = fig.add_gridspec(2, 2)
        ax_traj = fig.add_subplot(grid[0, 0])
        ax_velocity = fig.add_subplot(grid[0, 1])
        ax_heading = fig.add_subplot(grid[1, 0])
        ax_acceleration = fig.add_subplot(grid[1, 1])
    else:
        ax_traj = axes.get("trajectory")
        ax_velocity = axes.get("velocity")
        ax_heading = axes.get("heading")
        ax_acceleration = axes.get("acceleration")

    times = df["time"]
    colors = _build_vehicle_colors(vehicle_indices)

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

    if ax_velocity is not None:
        _plot_fleet_series(
            ax_velocity,
            df,
            times,
            vehicle_indices,
            colors,
            "fleet_v",
            "Fleet velocity",
            "Velocity [m/s]",
        )

    if ax_heading is not None:
        _plot_fleet_series(
            ax_heading,
            df,
            times,
            vehicle_indices,
            colors,
            "fleet_theta",
            "Fleet heading",
            "Heading [deg]",
            transform=np.degrees,
        )

    if ax_acceleration is not None:
        _plot_fleet_series(
            ax_acceleration,
            df,
            times,
            vehicle_indices,
            colors,
            "fleet_a",
            "Fleet acceleration",
            "Acceleration [m/s^2]",
        )

    if standalone:
        plt.tight_layout()
        plt.show()


def plot_both(
    local_file: Optional[str],
    fleet_file: Optional[str],
    selected_vehicles: Optional[Sequence[int]] = None,
) -> None:
    """Plot local and fleet recordings side by side."""
    plt = get_pyplot()
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
            "velocity": fig.add_subplot(fleet_grid[0, 1]),
            "heading": fig.add_subplot(fleet_grid[1, 0]),
            "acceleration": fig.add_subplot(fleet_grid[1, 1]),
        }
        plot_fleet_data(fleet_file, axes=fleet_axes, selected_vehicles=selected_vehicles)

    plt.tight_layout()
    plt.show()


def parse_vehicle_list(raw_value: Optional[str]) -> Optional[List[int]]:
    """Parse a comma-separated list of fleet vehicle indices."""
    if not raw_value:
        return None
    return [int(part.strip()) for part in raw_value.split(",") if part.strip()]


def print_available_files(recording_dir: str, data_type: str) -> None:
    """Print indexed CSV recordings for easier selection from the terminal."""
    files = list_csv_files_by_time(recording_dir, data_type)
    if not files:
        print(f"No {data_type} CSV files found in {recording_dir}")
        return

    print(f"\nAvailable {data_type} recordings in {recording_dir}:")
    for index, filepath in enumerate(files, start=1):
        modified = datetime.fromtimestamp(os.path.getmtime(filepath))
        print(f"  [{index}] {os.path.basename(filepath)}  ({modified:%Y-%m-%d %H:%M:%S})")


def select_csv(recording_dir: str, data_type: str, pick_index: Optional[int]) -> Optional[str]:
    """Select latest or indexed CSV recording for the given type."""
    files = list_csv_files_by_time(recording_dir, data_type)
    if not files:
        return None

    if pick_index is None:
        return files[0]

    if pick_index < 1 or pick_index > len(files):
        raise IndexError(
            f"{data_type} pick index {pick_index} is out of range. "
            f"Use --list {data_type} to see available files."
        )

    return files[pick_index - 1]


def prompt_choice(prompt: str, options: Sequence[str], default: Optional[str] = None) -> str:
    """Prompt the user to choose one value from a short option list."""
    option_lookup = {option.lower(): option for option in options}
    default_label = f" [{default}]" if default else ""

    while True:
        raw_value = input(f"{prompt}{default_label}: ").strip().lower()
        if not raw_value and default is not None:
            return default
        if raw_value in option_lookup:
            return option_lookup[raw_value]
        print(f"Choose one of: {', '.join(options)}")


def prompt_yes_no(prompt: str, default: bool = True) -> bool:
    """Prompt the user for a yes/no answer."""
    default_label = "Y/n" if default else "y/N"

    while True:
        raw_value = input(f"{prompt} [{default_label}]: ").strip().lower()
        if not raw_value:
            return default
        if raw_value in {"y", "yes"}:
            return True
        if raw_value in {"n", "no"}:
            return False
        print("Enter y or n.")


def prompt_pick_index(recording_dir: str, data_type: str) -> Optional[int]:
    """Prompt the user to choose the latest file or an indexed file."""
    files = list_csv_files_by_time(recording_dir, data_type)
    if not files:
        return None

    use_latest = prompt_yes_no(f"Use latest {data_type} recording", default=True)
    if use_latest:
        return None

    print_available_files(recording_dir, data_type)
    while True:
        raw_value = input(f"Select {data_type} file index: ").strip()
        try:
            pick_index = int(raw_value)
        except ValueError:
            print("Enter a number from the list.")
            continue

        if 1 <= pick_index <= len(files):
            return pick_index

        print(f"Enter a number between 1 and {len(files)}.")


def prompt_vehicle_selection(data_type: str) -> Optional[List[int]]:
    """Prompt for optional fleet vehicle filtering."""
    if data_type not in {"fleet", "both"}:
        return None

    use_all = prompt_yes_no("Plot all fleet vehicles", default=True)
    if use_all:
        return None

    while True:
        raw_value = input("Enter vehicle IDs separated by commas (example: 0,1,2): ").strip()
        try:
            selected = parse_vehicle_list(raw_value)
        except ValueError:
            print("Invalid vehicle list. Example: 0,1,2")
            continue

        if selected:
            return selected

        print("Enter at least one vehicle ID.")


def resolve_files(
    recording_dir: Optional[str],
    file_arg: Optional[str],
    data_type: str,
    pick_index: Optional[int],
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
        return select_csv(resolved_dir, "local", pick_index), None
    if data_type == "fleet":
        return None, select_csv(resolved_dir, "fleet", pick_index)

    return (
        select_csv(resolved_dir, "local", pick_index),
        select_csv(resolved_dir, "fleet", pick_index),
    )


def resolve_files_explicit(
    recording_dir: str,
    data_type: str,
    local_pick_index: Optional[int] = None,
    fleet_pick_index: Optional[int] = None,
) -> Tuple[Optional[str], Optional[str]]:
    """Resolve local and fleet files using independent indices."""
    if data_type == "local":
        return select_csv(recording_dir, "local", local_pick_index), None
    if data_type == "fleet":
        return None, select_csv(recording_dir, "fleet", fleet_pick_index)

    return (
        select_csv(recording_dir, "local", local_pick_index),
        select_csv(recording_dir, "fleet", fleet_pick_index),
    )


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Plot recorded observer CSV files")
    parser.add_argument(
        "plot_type",
        nargs="?",
        choices=["local", "fleet", "both"],
        default="fleet",
        help="Recording type to plot. Default: fleet",
    )
    parser.add_argument(
        "--type",
        choices=["local", "fleet", "both"],
        dest="type_override",
        help="Optional alias for the positional plot type",
    )
    parser.add_argument("--file", help="Plot a specific CSV file")
    parser.add_argument("--dir", help="Recording root directory containing local/ and fleet/")
    parser.add_argument(
        "--pick",
        type=int,
        help="Pick a recording by index from newest to oldest. Use --list to see indices.",
    )
    parser.add_argument(
        "--list",
        nargs="?",
        const="fleet",
        choices=["local", "fleet", "both"],
        help="List available recordings and exit. Default: fleet",
    )
    parser.add_argument(
        "--vehicles",
        help="Comma-separated fleet vehicle indices to plot, for example: 0,1,3",
    )
    return parser.parse_args()


def main() -> None:
    """Main command-line entry point."""
    args = parse_args()
    interactive_mode = len(sys.argv) == 1
    data_type = args.type_override or args.plot_type
    selected_vehicles = parse_vehicle_list(args.vehicles)

    if args.file and args.pick is not None:
        raise ValueError("Use either --file or --pick, not both")

    resolved_dir = find_recording_dir(args.dir)
    if interactive_mode:
        if resolved_dir is None:
            raise FileNotFoundError("No recording directory found. Use --dir to point to scope_recordings.")

        print("Interactive plot selection\n")
        data_type = prompt_choice("Plot type (local/fleet/both)", ["local", "fleet", "both"], default="fleet")
        selected_vehicles = prompt_vehicle_selection(data_type)

        if data_type == "both":
            if prompt_yes_no("Use latest local and latest fleet recordings", default=True):
                local_file, fleet_file = resolve_files_explicit(resolved_dir, "both")
            else:
                print("\nSelect local recording")
                local_pick = prompt_pick_index(resolved_dir, "local")
                print("\nSelect fleet recording")
                fleet_pick = prompt_pick_index(resolved_dir, "fleet")
                local_file, fleet_file = resolve_files_explicit(
                    resolved_dir,
                    "both",
                    local_pick_index=local_pick,
                    fleet_pick_index=fleet_pick,
                )
        else:
            pick_index = prompt_pick_index(resolved_dir, data_type)
            local_file, fleet_file = resolve_files_explicit(
                resolved_dir,
                data_type,
                local_pick_index=pick_index,
                fleet_pick_index=pick_index,
            )

        if data_type == "local":
            if not local_file:
                raise FileNotFoundError("No local CSV recording found")
            plot_local_data(local_file)
            return

        if data_type == "fleet":
            if not fleet_file:
                raise FileNotFoundError("No fleet CSV recording found")
            plot_fleet_data(fleet_file, selected_vehicles=selected_vehicles)
            return

        plot_both(local_file, fleet_file, selected_vehicles=selected_vehicles)
        return

    if args.list:
        if resolved_dir is None:
            raise FileNotFoundError("No recording directory found. Use --dir to point to scope_recordings.")

        if args.list in ("local", "both"):
            print_available_files(resolved_dir, "local")
        if args.list in ("fleet", "both"):
            print_available_files(resolved_dir, "fleet")
        return

    local_file, fleet_file = resolve_files(args.dir, args.file, data_type, args.pick)

    if args.file:
        if local_file:
            plot_local_data(local_file)
        else:
            plot_fleet_data(fleet_file, selected_vehicles=selected_vehicles)
        return

    if data_type == "local":
        if not local_file:
            raise FileNotFoundError("No local CSV recording found")
        plot_local_data(local_file)
        return

    if data_type == "fleet":
        if not fleet_file:
            raise FileNotFoundError("No fleet CSV recording found")
        plot_fleet_data(fleet_file, selected_vehicles=selected_vehicles)
        return

    plot_both(local_file, fleet_file, selected_vehicles=selected_vehicles)


if __name__ == "__main__":
    main()
