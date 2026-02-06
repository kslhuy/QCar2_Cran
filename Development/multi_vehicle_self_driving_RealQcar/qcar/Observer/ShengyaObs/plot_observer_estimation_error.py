"""
Plot estimation error of Distributed Luenberger Observer.

This script calculates:
1. True relative states from single CSV file (containing V2V synchronized data)
2. Estimated relative states from x_vec_before (from the same CSV)
3. Estimation errors and visualizes them in a 3x3 grid

The observer CSV contains:
- leader_position, leader_velocity, leader_acceleration (vehicle 0 states)
- true_position_i, true_velocity_i, true_acceleration_i (vehicle i states via V2V)
- x_vec_before_i_j (estimated states for vehicle i)

True relative states are: p_i - p0 + di0, v_i - v0, a_i - a0
Where di0 = i * d + h * (v1 + v2 + ... + vi), computed from true states
      d = 0.4, h = 0.3

Usage:
    python plot_observer_estimation_error.py --csv path/to/file.csv
    python plot_observer_estimation_error.py --vehicle-id 1
    python plot_observer_estimation_error.py --output-dir path/to/observer_recordings
"""
import argparse
import glob
import os
from typing import Optional

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


# Constants
D = 0.95  # Desired distance
H = 1  # Time headway


def find_latest_file(output_dir: str, vehicle_id: int) -> Optional[str]:
    """Find the latest recording CSV for the given vehicle ID in output_dir."""
    pattern = os.path.join(output_dir, f"dist_luenberger_v{vehicle_id}_*.csv")
    files = glob.glob(pattern)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def calculate_true_relative_states(df: pd.DataFrame, vehicle_id: int, observer_size: int = 3) -> tuple:
    """
    Calculate true relative states for a specific vehicle from its CSV file.
    
    The CSV contains:
    - true_position_0, true_velocity_0, true_acceleration_0 (vehicle 0 via V2V)
    - true_position_i, true_velocity_i, true_acceleration_i (vehicle i via V2V)
    
    Args:
        df: DataFrame from the vehicle's observer recording
        vehicle_id: ID of the vehicle (1, 2, or 3)
        observer_size: Number of follower vehicles
    
    Returns:
        time: Time array
        true_relative_states: [n_samples, 3*observer_size] array of true relative states
        di0_values_all: [n_samples, observer_size] array of di0 values
    """
    n_samples = len(df)
    time = df['time'].values
    true_relative_states = np.zeros((n_samples, 3 * observer_size))
    di0_values_all = np.zeros((n_samples, observer_size))
    
    # Get leader (vehicle 0) states
    leader_p = df['true_position_0'].values
    leader_v = df['true_velocity_0'].values
    leader_a = df['true_acceleration_0'].values
    
    # Calculate relative states for all vehicles
    for i in range(1, observer_size + 1):
        # Get vehicle i's true states (from V2V)
        vehicle_p = df[f'true_position_{i}'].values
        vehicle_v = df[f'true_velocity_{i}'].values
        vehicle_a = df[f'true_acceleration_{i}'].values
        
        # Calculate di0 = i*D + H*(v1 + v2 + ... + vi)
        sum_velocities = np.zeros(n_samples)
        for j in range(1, i + 1):
            sum_velocities += df[f'true_velocity_{j}'].values
        
        di0 = i * D + H * sum_velocities
        di0_values_all[:, i - 1] = di0
        
        # Calculate relative states: pi - p0 + di0, vi - v0, ai - a0
        relative_p = vehicle_p - leader_p + di0
        relative_v = vehicle_v - leader_v
        relative_a = vehicle_a - leader_a
        
        # Store in result array [p1, v1, a1, p2, v2, a2, ...]
        idx = (i - 1) * 3
        true_relative_states[:, idx] = relative_p
        true_relative_states[:, idx + 1] = relative_v
        true_relative_states[:, idx + 2] = relative_a
    
    return time, true_relative_states, di0_values_all


def extract_estimated_states(df: pd.DataFrame, observer_size: int = 3) -> np.ndarray:
    """
    Extract estimated relative states from x_vec_before columns.
    
    Returns:
        estimated_states: [n_samples, 3*observer_size] array of estimated states
    """
    n_samples = len(df)
    estimated_states = np.zeros((n_samples, 3 * observer_size))
    
    for vehicle_idx in range(1, observer_size + 1):
        col_idx = vehicle_idx - 1
        
        # Extract x_vec_before components for this vehicle
        p_col = f'x_vec_before_p{vehicle_idx}'
        v_col = f'x_vec_before_v{vehicle_idx}'
        a_col = f'x_vec_before_a{vehicle_idx}'
        
        base_idx = col_idx * 3
        if p_col in df.columns:
            estimated_states[:, base_idx] = df[p_col].values
        if v_col in df.columns:
            estimated_states[:, base_idx + 1] = df[v_col].values
        if a_col in df.columns:
            estimated_states[:, base_idx + 2] = df[a_col].values
    
    return estimated_states


def calculate_estimation_error(true_states: np.ndarray, estimated_states: np.ndarray) -> np.ndarray:
    """
    Calculate estimation error: true_states - estimated_states
    
    Args:
        true_states: [n_samples, 3*observer_size]
        estimated_states: [n_samples, 3*observer_size]
    
    Returns:
        errors: [n_samples, 3*observer_size]
    """
    return true_states - estimated_states


def _steady_state_error(time: np.ndarray, error: np.ndarray, conv_time: Optional[float], duration: float = 10.0) -> float:
    """
    Compute steady-state RMS error from convergence time over a duration window.
    
    Args:
        time: Time array
        error: Error array
        conv_time: Convergence time (start point for SS calculation)
        duration: Duration window in seconds (default 10s)
    
    Returns:
        RMS error over the window, or nan if not enough data
    """
    if error.size == 0 or conv_time is None:
        return float('nan')
    
    # Find indices in the window [conv_time, conv_time + duration]
    t_start = conv_time
    t_end = conv_time + duration
    
    mask = (time >= t_start) & (time <= t_end)
    if not np.any(mask):
        # If convergence window doesn't exist, use last 10s of data
        if len(time) > 0:
            t_start = max(time[0], time[-1] - duration)
            mask = time >= t_start
        else:
            return float('nan')
    
    if not np.any(mask):
        return float('nan')
    
    # Calculate RMS
    ss_window = error[mask]
    return float(np.sqrt(np.mean(ss_window ** 2)))


def _convergence_time(time: np.ndarray, error: np.ndarray, tol: float) -> Optional[float]:
    """Compute convergence time: first time after which |error| <= tol for all remaining samples."""
    if time.size == 0 or error.size == 0:
        return None
    within = np.abs(error) <= tol
    if not np.any(within):
        return None
    # Find earliest index where all subsequent samples stay within tolerance
    for idx in range(len(error)):
        if np.all(within[idx:]):
            return float(time[idx])
    return None


def plot_estimation_error(csv_path: str, observer_size: int = 3) -> None:
    """
    Plot estimation error in a 3x3 grid (rows=vehicles, cols=state components).
    
    Args:
        csv_path: Path to the CSV file from current vehicle
        observer_size: Number of follower vehicles (default 3)
    """
    # Extract vehicle ID and timestamp from csv_path
    filename = os.path.basename(csv_path)
    parts = filename.split('_')
    current_vehicle_id = int(parts[2].replace('v', '')) if len(parts) > 2 else 1
    timestamp = os.path.splitext(filename)[0].split('_')[-2] + '_' + os.path.splitext(filename)[0].split('_')[-1]
    
    # Load CSV
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} samples from vehicle {current_vehicle_id}")
    
    # Calculate true relative states from V2V data
    time, true_states, di0_values = calculate_true_relative_states(df, current_vehicle_id, observer_size)
    
    # Extract estimated states
    estimated_states = extract_estimated_states(df, observer_size)
    
    # Calculate errors
    errors = calculate_estimation_error(true_states, estimated_states)
    
    # Exclude last 5 seconds from plotting
    if len(time) > 0:
        time_cutoff = time[-1] - 10.0
        cutoff_idx = np.searchsorted(time, time_cutoff)
        time = time[:cutoff_idx]
        errors = errors[:cutoff_idx, :]
        print(f"Plotting {len(time)} samples (excluding last 5 seconds)")
    
    # Create 3x3 subplot grid
    fig, axes = plt.subplots(observer_size, 3, figsize=(15, 12), sharex=True)
    
    state_labels = ['Estimation Error of pi - p0 + di0 [m]', 'Estimation Error of vi - v0 [m/s]', 'Estimation Error of ai - a0 [m/s²]']
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c']  # Blue, Orange, Green for vehicles 1, 2, 3
    
    for vehicle_idx in range(1, observer_size + 1):
        row_idx = vehicle_idx - 1
        col_idx_base = vehicle_idx - 1
        
        # Get error components for this vehicle
        base_error_idx = col_idx_base * 3
        
        for state_idx in range(3):  # Position, Velocity, Acceleration
            ax = axes[row_idx, state_idx]
            error_col = base_error_idx + state_idx
            error_series = errors[:, error_col]
            peak = float(np.max(np.abs(error_series))) if error_series.size > 0 else 0.0
            tol = max(0.02 * peak, 1e-1)
            conv_time = _convergence_time(time, error_series, tol)
            ss_err = _steady_state_error(time, error_series, conv_time)
            
            color = colors[row_idx]
            ax.plot(time, error_series, color=color, linewidth=1.5, alpha=0.8)
            ax.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.3)
            ax.grid(True, alpha=0.3)

            conv_text = "N/A" if conv_time is None else f"{conv_time:.2f}s"
            stats_text = f"Conv: {conv_text}\nSS RMS: {ss_err:.3e}"
            ax.text(
                0.98,
                0.98,
                stats_text,
                transform=ax.transAxes,
                ha='right',
                va='top',
                fontsize=8,
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.7, edgecolor='none'),
            )
            
            # Labels
            ax.set_ylabel(state_labels[state_idx])
            if row_idx == observer_size - 1:
                ax.set_xlabel('Time [s]')
            
            # Title for first row
            # if row_idx == 0:
            #     if state_idx == 0:
            #         ax.set_title(f'Vehicle {vehicle_idx}')
            #     elif state_idx == 1:
            #         ax.set_title(f'Vehicle {vehicle_idx}')
            #     else:
            #         ax.set_title(f'Vehicle {vehicle_idx}')
            
            # Row label on the left
            if state_idx == 0:
                ax.text(-0.35, 0.5, f'Vehicle {vehicle_idx}', 
                       transform=ax.transAxes, fontsize=12, fontweight='bold',
                       va='center', ha='right', rotation=90)
            
            # ax.grid(True, alpha=0.3)
    
    # Overall title
    fig.suptitle(f'Estimation Error of Distributed Observer (Vehicle {current_vehicle_id})_{timestamp}\n' + 
                 f'True states via V2V, Estimated states from observer', 
                fontsize=14, fontweight='bold')
    fig.tight_layout(rect=[0.05, 0.03, 1, 0.95])
    
    plt.show(block=False)
    
    # Ask user to save
    _maybe_save_figure(fig, csv_path, current_vehicle_id, timestamp)


def _maybe_save_figure(fig: plt.Figure, csv_path: str, vehicle_id: str, timestamp: str) -> None:
    """Ask user whether to save the figure to file."""
    try:
        ans = input("Save figure to file? [y/N]: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        ans = 'n'
    
    if ans == 'y':
        # Create figure directory if it doesn't exist
        script_dir = os.path.dirname(os.path.abspath(__file__))
        figure_dir = os.path.join(script_dir, 'figure')
        os.makedirs(figure_dir, exist_ok=True)
        
        # Generate filename
        default_name = f"estimated_error_of_distributed_observer_{vehicle_id}_{timestamp}.png"
        try:
            name = input(f"Enter filename to save (default: {default_name}): ").strip()
        except (EOFError, KeyboardInterrupt):
            name = ''
        
        if not name:
            name = default_name
        
        save_path = os.path.join(figure_dir, name)
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Figure saved to: {save_path}")
    else:
        print("Figure not saved.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot estimation error from Distributed Luenberger observer CSV.\n" +
                   "All true states are recorded via V2V in the same CSV."
    )
    parser.add_argument(
        "--csv",
        type=str,
        default="",
        help=(
            "Path to CSV file of current vehicle. If omitted, uses latest "
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
    parser.add_argument(
        "--vehicle-id",
        type=int,
        default=1,
        help="Current vehicle ID for which to plot error (default: 1)",
    )
    args = parser.parse_args()
    
    # Resolve output directory
    if args.output_dir:
        output_dir = args.output_dir
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.abspath(os.path.join(script_dir, "..", "..", "observer_recordings"))
    
    # If user explicitly specifies a CSV, use it
    if args.csv:
        csv_path = args.csv
        if not os.path.exists(csv_path):
            print(f"CSV file not found: {csv_path}")
            return
        print(f"Using CSV file: {csv_path}")
        plot_estimation_error(csv_path)
        return
    
    # Otherwise find latest file for specified vehicle
    csv_path = find_latest_file(output_dir, args.vehicle_id)
    if not csv_path or not os.path.exists(csv_path):
        print(f"No recorder CSV file found for vehicle {args.vehicle_id}. Check --output-dir.")
        print(f"Looking in: {output_dir}")
        return
    
    print(f"Using CSV file: {csv_path}")
    plot_estimation_error(csv_path)


if __name__ == "__main__":
    main()
