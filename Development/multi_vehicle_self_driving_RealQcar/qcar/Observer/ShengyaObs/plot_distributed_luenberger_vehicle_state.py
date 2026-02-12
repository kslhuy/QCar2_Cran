"""
Plot local vehicle states and measurements from Distributed Luenberger recordings.

This script focuses on a single vehicle's absolute state and local measurements,
using the CSV files produced by DistributedLuenbergerRecorder.

Usage examples:
    python plot_distributed_luenberger_vehicle_state.py --vehicle-id 1
    python plot_distributed_luenberger_vehicle_state.py --csv path/to/file.csv
"""
import argparse
import glob
import os
from typing import Optional, Dict

import matplotlib.pyplot as plt
import pandas as pd


def find_latest_file(output_dir: str, vehicle_id: int) -> Optional[str]:
    """Find the latest recording CSV for the given vehicle ID in output_dir."""
    pattern = os.path.join(output_dir, f"dist_luenberger_v{vehicle_id}_*.csv")
    files = glob.glob(pattern)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def plot_vehicle_state_single(csv_path: str) -> None:
    """Plot local vehicle states and local measurements from a single recorder CSV file."""
    df = pd.read_csv(csv_path)

    if 'time' not in df.columns:
        raise ValueError("CSV file must contain a 'time' column.")

    # Extract vehicle_id from filename (e.g., dist_luenberger_v1_*.csv -> 1)
    import re
    basename = os.path.basename(csv_path)
    match = re.search(r'_v(\d+)_', basename)
    vehicle_id = int(match.group(1)) if match else None

    t = df['time']
    t = t - t.min()  # Normalize time to start at 0
    pos = df.get('position', pd.Series([0.0] * len(df)))
    vel = df.get('velocity', pd.Series([0.0] * len(df)))
    acc = df.get('acceleration', pd.Series([0.0] * len(df)))
    u = df.get('control_input', pd.Series([0.0] * len(df)))
    rel_p = df.get('local_measurement_p', pd.Series([0.0] * len(df)))
    local_v = df.get('local_measurement_v', pd.Series([0.0] * len(df)))
    
    # Leader (vehicle 0) states
    leader_pos = df.get('true_position_0', pd.Series([0.0] * len(df)))
    leader_vel = df.get('true_velocity_0', pd.Series([0.0] * len(df)))
    leader_acc = df.get('true_acceleration_0', pd.Series([0.0] * len(df)))
    
    # True measurements for this vehicle (from V2V or local)
    true_pos = None
    true_vel = None
    true_acc = None
    if vehicle_id is not None:
        true_pos_col = f'true_position_{vehicle_id}'
        true_vel_col = f'true_velocity_{vehicle_id}'
        true_acc_col = f'true_acceleration_{vehicle_id}'
        if true_pos_col in df.columns:
            true_pos = df[true_pos_col]
        if true_vel_col in df.columns:
            true_vel = df[true_vel_col]
        if true_acc_col in df.columns:
            true_acc = df[true_acc_col]

    # Define colors for consistency
    vehicle_color = '#1f77b4'  # Blue (estimated)
    true_color = '#2ca02c'     # Green (true measurement)
    leader_color = '#d62728'   # Red
    
    fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    axs = axes.ravel()

    if true_pos is not None:
        axs[0].plot(t, true_pos, label="true (measured)", color=true_color, linewidth=2)
    axs[0].plot(t, leader_pos, label="leader (p0)", color=leader_color, linestyle='--', alpha=0.7)
    axs[0].set_ylabel("Position [m]")
    axs[0].set_title("Absolute position p_i")
    axs[0].legend()

    if true_vel is not None:
        axs[1].plot(t, true_vel, label="true (measured)", color=true_color, linewidth=2)
    axs[1].plot(t, leader_vel, label="leader (v0)", color=leader_color, linestyle='--', alpha=0.7)
    axs[1].set_ylabel("Velocity [m/s]")
    axs[1].set_title("Absolute velocity v_i")
    axs[1].legend()

    if true_acc is not None:
        axs[2].plot(t, true_acc, label="true (measured)", color=true_color, linewidth=2)
    axs[2].plot(t, leader_acc, label="leader (a0)", color=leader_color, linestyle='--', alpha=0.7)
    axs[2].set_ylabel("Acceleration [m/s²]")
    axs[2].set_title("Absolute acceleration a_i")
    axs[2].legend()

    axs[3].plot(t, u, label="vehicle", color=vehicle_color)
    axs[3].set_ylabel("Throttle input")
    axs[3].set_title("Control input (throttle) u_i")
    axs[3].legend()

    axs[4].plot(t, rel_p, label="vehicle", color=vehicle_color)
    axs[4].set_ylabel("Distance to preceding [m]")
    axs[4].set_xlabel("Time [s]")
    axs[4].set_title("Local relative position measurement pi - p_{i-1}")
    # Limit y-axis upper bound to 1 m
    axs[4].set_ylim(top=1.0)
    axs[4].legend()

    axs[5].plot(t, local_v, label="vehicle", color=vehicle_color)
    axs[5].set_ylabel("Local vel [m/s]")
    axs[5].set_xlabel("Time [s]")
    axs[5].set_title("Local velocity measurement v_i")
    axs[5].legend()

    for ax in axs:
        ax.grid(True, alpha=0.3)

    fig.suptitle(f"Distributed Luenberger - vehicle states from {os.path.basename(csv_path)}")
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    plt.show(block=False)

    _maybe_save_figure(fig, csv_path, suffix="_vehicle_state")


def plot_multi_vehicle_states(records: Dict[int, str]) -> None:
    """Plot states for multiple vehicles (1-3) on shared axes.

    records: dict[vehicle_id -> csv_path]
    """
    dataframes: Dict[int, pd.DataFrame] = {}
    for vid, path in records.items():
        df = pd.read_csv(path)
        if 'time' not in df.columns:
            continue
        dataframes[vid] = df

    if not dataframes:
        print("No valid CSV files with 'time' column found.")
        return

    # Define colors for each vehicle (consistent across all subplots)
    vehicle_colors = {
        0: '#d62728',  # Red for leader
        1: '#1f77b4',  # Blue
        2: '#ff7f0e',  # Orange
        3: '#2ca02c',  # Green
    }
    
    fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=False)
    axs = axes.ravel()

    # Plot leader state first (from any available dataframe - all should have same leader data)
    first_df = next(iter(dataframes.values()))
    leader_color = vehicle_colors[0]
    if 'true_position_0' in first_df.columns:
        t_leader = first_df['time']
        t_leader = t_leader - t_leader.min()  # Normalize time to start at 0
        axs[0].plot(t_leader, first_df['true_position_0'], label='Leader (p0)', 
                   color=leader_color, linestyle='--', linewidth=2, alpha=0.8)
    if 'true_velocity_0' in first_df.columns:
        axs[1].plot(t_leader, first_df['true_velocity_0'], label='Leader (v0)', 
                   color=leader_color, linestyle='--', linewidth=2, alpha=0.8)
    if 'true_acceleration_0' in first_df.columns:
        axs[2].plot(t_leader, first_df['true_acceleration_0'], label='Leader (a0)', 
                   color=leader_color, linestyle='--', linewidth=2, alpha=0.8)

    # Position, velocity, acceleration, control, distance to preceding, local velocity
    for vid, df in sorted(dataframes.items()):
        label_true = f"V{vid}"
        color = vehicle_colors.get(vid, '#000000')  # Default to black if vid not in dict
        t = df['time']
        t = t - t.min()  # Normalize time to start at 0
        
        # Plot true measurements (solid lines)
        true_pos_col = f'true_position_{vid}'
        true_vel_col = f'true_velocity_{vid}'
        true_acc_col = f'true_acceleration_{vid}'
        if true_pos_col in df.columns:
            axs[0].plot(t, df[true_pos_col], label=label_true, color=color, linewidth=2)
        if true_vel_col in df.columns:
            axs[1].plot(t, df[true_vel_col], label=label_true, color=color, linewidth=2)
        if true_acc_col in df.columns:
            axs[2].plot(t, df[true_acc_col], label=label_true, color=color, linewidth=2)
        
        # Control and measurements
        if 'control_input' in df.columns:
            axs[3].plot(t, df['control_input'], label=f"V{vid}", color=color)
        if 'local_measurement_p' in df.columns:
            axs[4].plot(t, df['local_measurement_p'], label=f"V{vid}", color=color)
        if 'local_measurement_v' in df.columns:
            axs[5].plot(t, df['local_measurement_v'], label=f"V{vid}", color=color)

    axs[0].set_ylabel("Position [m]")
    axs[0].set_title("Absolute position")
    axs[0].legend()

    axs[1].set_ylabel("Velocity [m/s]")
    axs[1].set_title("Absolute velocity")
    axs[1].legend()

    axs[2].set_ylabel("Acceleration [m/s²]")
    axs[2].set_title("Absolute acceleration")
    axs[2].legend()

    axs[3].set_ylabel("Throttle input")
    axs[3].set_title("Control input (throttle)")
    axs[3].legend()

    axs[4].set_ylabel("Distance to preceding [m]")
    axs[4].set_xlabel("Time [s]")
    axs[4].set_title("Local relative position measurement pi - p_{i-1}")
    # Limit y-axis upper bound to 1 m
    axs[4].set_ylim(top=1.0)
    axs[4].legend()

    axs[5].set_ylabel("Local vel [m/s]")
    axs[5].set_xlabel("Time [s]")
    axs[5].set_title("Local velocity measurement")
    axs[5].legend()

    for ax in axs:
        ax.grid(True, alpha=0.3)

    # Use first path for default save name
    first_path = next(iter(records.values()))
    fig.suptitle("Distributed Luenberger - vehicle states (vehicles 1-3)")
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    plt.show(block=False)

    _maybe_save_figure(fig, first_path, suffix="_vehicles_1_3_state")


def _maybe_save_figure(fig: plt.Figure, csv_path: str, suffix: str) -> None:
    """Ask user whether to save the current figure to file."""
    try:
        ans = input("Save figure to file? [y/N]: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        ans = 'n'

    if ans == 'y':
        base = os.path.splitext(os.path.basename(csv_path))[0]
        default_name = base + suffix + ".png"
        
        # Save to ShengyaObs/figure directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        figure_dir = os.path.join(script_dir, "figure")
        
        # Create figure directory if it doesn't exist
        os.makedirs(figure_dir, exist_ok=True)
        
        try:
            name = input(f"Enter filename to save (default: {default_name}): ").strip()
        except (EOFError, KeyboardInterrupt):
            name = ''
        if not name:
            name = default_name
        save_path = os.path.join(figure_dir, name)
        fig.savefig(save_path, dpi=150)
        print(f"Figure saved to: {save_path}")
    else:
        print("Figure not saved.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot local vehicle states from Distributed Luenberger recorder CSV."
    )
    parser.add_argument(
        "--csv",
        type=str,
        default="",
        help=(
            "Path to CSV file. If omitted, uses latest "
            "dist_luenberger_v<vehicle-id>_*.csv in observer_recordings."
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="",
        help=(
            "Directory where recorder CSV files are stored. "
            "Default: qcar/observer_recordings relative to this script."
        ),
    )
    args = parser.parse_args()

    # Resolve output directory: default to qcar/observer_recordings (one level above Observer)
    if args.output_dir:
        output_dir = args.output_dir
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # script_dir = qcar/Observer/ShengyaObs -> go up two levels to qcar, then observer_recordings
        output_dir = os.path.abspath(os.path.join(script_dir, "..", "..", "observer_recordings"))

    # If user explicitly specifies a CSV, keep old single-vehicle behavior
    if args.csv:
        csv_path = args.csv
        if not os.path.exists(csv_path):
            print(f"CSV file not found: {csv_path}")
            return
        print(f"Using CSV file: {csv_path}")
        plot_vehicle_state_single(csv_path)
        return

    # 自动：读取车辆 1 到 3 的最新 CSV 文件
    records: Dict[int, str] = {}
    for vid in (1, 2, 3):
        path = find_latest_file(output_dir, vid)
        if path and os.path.exists(path):
            records[vid] = path

    if not records:
        print("No recorder CSV files found for vehicles 1-3. Check --output-dir.")
        return

    print("Using latest CSV files:")
    for vid, path in sorted(records.items()):
        print(f"  Vehicle {vid}: {path}")

    plot_multi_vehicle_states(records)


if __name__ == "__main__":
    main()
