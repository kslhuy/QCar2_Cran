"""
Distributed Luenberger Estimator Data Plotter

Interactive tool for viewing and analyzing observer recordings.

Usage:
    python plot_distributed_luenberger.py --fake     # Plot fake vehicle recordings
    python plot_distributed_luenberger.py --real     # Real vehicle recordings
    python plot_distributed_luenberger.py -i         # Interactive mode
"""
import os
import sys
import glob
import argparse
from datetime import datetime, timedelta
from typing import Optional, List, Tuple

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# Configure matplotlib for cleaner plots
plt.rcParams.update({
    'font.size': 10,
    'axes.titlesize': 10,
    'axes.labelsize': 10,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 8,
    'figure.titlesize': 10,
})

# Recording directory paths
RECORDING_DIRS = {
    'fake': [
        os.path.join('..','..', 'GUI', 'observer_recordings'),      # qcar/GUI/observer_recordings
        os.path.join('..', '..', 'observer_recordings'),            # qcar/observer_recordings (向上两级)
        os.path.join('GUI', 'observer_recordings'),
        'observer_recordings',
    ],
    'real': [
        os.path.join('..','..', 'real_recordings'),
        os.path.join('..', '..', 'observer_recordings'),            # qcar/observer_recordings
        'real_recordings',
    ]
}


def find_latest_recording(directories: List[str], 
                          pattern: str = "dist_luenberger_*.csv") -> Optional[str]:
    """Find the most recent recording file."""
    for base_dir in directories:
        if os.path.exists(base_dir):
            files = glob.glob(os.path.join(base_dir, pattern))
            if files:
                return max(files, key=os.path.getmtime)
    return None


def list_recordings(directories: List[str], 
                    pattern: str = "dist_luenberger_*.csv") -> List[Tuple[str, datetime, int]]:
    """List all recordings with metadata."""
    recordings = []
    for base_dir in directories:
        if os.path.exists(base_dir):
            files = glob.glob(os.path.join(base_dir, pattern))
            for f in files:
                mtime = datetime.fromtimestamp(os.path.getmtime(f))
                size = os.path.getsize(f)
                recordings.append((f, mtime, size))
    recordings.sort(key=lambda x: x[1], reverse=True)
    return recordings


def format_duration(seconds: float) -> str:
    """Format duration in human-readable format."""
    if seconds < 60:
        return f"{seconds:.1f}s"
    elif seconds < 3600:
        mins = int(seconds // 60)
        secs = seconds % 60
        return f"{mins}m {secs:.0f}s"
    else:
        hours = int(seconds // 3600)
        mins = int((seconds % 3600) // 60)
        return f"{hours}h {mins}m"


def format_file_size(bytes_size: int) -> str:
    """Format file size in human-readable format."""
    if bytes_size < 1024:
        return f"{bytes_size} B"
    elif bytes_size < 1024 * 1024:
        return f"{bytes_size / 1024:.1f} KB"
    else:
        return f"{bytes_size / (1024*1024):.2f} MB"


def interactive_select(directories: List[str], source_name: str) -> Optional[str]:
    """Interactive console menu to select a recording."""
    recordings = list_recordings(directories)
    
    if not recordings:
        print(f"\n  No {source_name} recordings found.")
        return None
    
    print(f"\n{'='*70}")
    print(f"  📊 {source_name.upper()} RECORDINGS")
    print(f"{'='*70}\n")
    
    # Group by vehicle
    by_vehicle = {}
    for path, mtime, size in recordings:
        # Extract vehicle ID from filename
        basename = os.path.basename(path)
        vid = "?"
        if "_v" in basename:
            try:
                vid = basename.split("_v")[1].split("_")[0]
            except:
                pass
        if vid not in by_vehicle:
            by_vehicle[vid] = []
        by_vehicle[vid].append((path, mtime, size))
    
    # Display with indices, grouped by vehicle
    all_files = []
    idx = 1
    max_per_vehicle = 3  # Show max 3 per vehicle
    
    for vid in sorted(by_vehicle.keys()):
        print(f"  🚗 Vehicle {vid}:")
        for path, mtime, size in by_vehicle[vid][:max_per_vehicle]:
            # Calculate duration from CSV
            try:
                df = pd.read_csv(path)
                duration = df['time'].max() - df['time'].min()
                samples = len(df)
                dur_str = format_duration(duration)
            except:
                dur_str = "?"
                samples = 0
            
            # Format timestamp
            timestamp = mtime.strftime("%m-%d %H:%M:%S")
            
            print(f"    [{idx:2d}] {timestamp} | {dur_str:>8} | {samples:>5} samples | {format_file_size(size):>8}")
            all_files.append(path)
            idx += 1
        
        if len(by_vehicle[vid]) > max_per_vehicle:
            print(f"        ... and {len(by_vehicle[vid]) - max_per_vehicle} more files")
        print()
    
    print(f"  [0] Cancel")
    print(f"{'='*70}")
    
    # Get user input
    while True:
        try:
            choice = input("\n  Enter number to plot (or 0 to cancel): ").strip()
            if choice == '0' or choice.lower() == 'q':
                return None
            choice_idx = int(choice) - 1
            if 0 <= choice_idx < len(all_files):
                return all_files[choice_idx]
            print(f"  Invalid choice. Enter 1-{len(all_files)}")
        except ValueError:
            print("  Please enter a number")
        except KeyboardInterrupt:
            return None


def load_data(filepath: str) -> pd.DataFrame:
    """Load recording data from CSV file."""
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Recording file not found: {filepath}")
    df = pd.read_csv(filepath)
    return df


def detect_observer_size(df: pd.DataFrame) -> int:
    """Detect observer_size from column names."""
    return len([c for c in df.columns if c.startswith('x_vec_before_p')])


def detect_fleet_size(df: pd.DataFrame) -> int:
    """Detect fleet_size from column names."""
    return len([c for c in df.columns if c.startswith('fleet_x_')])


def plot_observer_states(df: pd.DataFrame, observer_size: int, 
                         ax_pos: plt.Axes, ax_vel: plt.Axes, ax_acc: plt.Axes):
    """Plot relative position, velocity, and acceleration estimates."""
    time = df['time'].values
    colors = plt.cm.tab10(np.linspace(0, 1, max(observer_size, 1)))
    
    for i in range(observer_size):
        vid = i + 1
        col_pos = f'x_vec_after_p{vid}'
        if col_pos in df.columns:
            ax_pos.plot(time, df[col_pos], label=f'Vehicle {vid}', color=colors[i], linewidth=1.5)
        
        col_vel = f'x_vec_after_v{vid}'
        if col_vel in df.columns:
            ax_vel.plot(time, df[col_vel], label=f'Vehicle {vid}', color=colors[i], linewidth=1.5)
        
        col_acc = f'x_vec_after_a{vid}'
        if col_acc in df.columns:
            ax_acc.plot(time, df[col_acc], label=f'Vehicle {vid}', color=colors[i], linewidth=1.5)
    
    ax_pos.set_ylabel('Pos [m]')
    ax_pos.set_title('Relative Position (pi - p0 + di0)')
    ax_pos.legend(loc='upper right', ncol=2)
    ax_pos.grid(True, alpha=0.3, linewidth=0.5)
    
    ax_vel.set_ylabel('Vel [m/s]')
    ax_vel.set_title('Relative Velocity (vi - v0)')
    ax_vel.legend(loc='upper right', ncol=2)
    ax_vel.grid(True, alpha=0.3, linewidth=0.5)
    
    ax_acc.set_ylabel('Acc [m/s²]')
    ax_acc.set_title('Relative Acceleration (ai - a0)')
    ax_acc.legend(loc='upper right', ncol=2)
    ax_acc.grid(True, alpha=0.3, linewidth=0.5)


def plot_observer_terms(df: pd.DataFrame, observer_size: int, ax: plt.Axes, vehicle_idx: int = 1):
    """Plot dynamics, measurement, and consensus terms."""
    time = df['time'].values
    
    col_dyn = f'dynamics_p{vehicle_idx}'
    col_meas = f'measurement_p{vehicle_idx}'
    col_cons = f'consensus_p{vehicle_idx}'
    
    if col_dyn in df.columns:
        ax.plot(time, df[col_dyn], label='Dynamics', linewidth=1.2, color='#1f77b4')
    if col_meas in df.columns:
        ax.plot(time, df[col_meas], label='Measurement', linewidth=1.2, color='#2ca02c')
    if col_cons in df.columns:
        ax.plot(time, df[col_cons], label='Consensus', linewidth=1.2, color='#d62728')
    
    ax.set_ylabel('Value')
    ax.set_title(f'Observer Terms (Vehicle {vehicle_idx})')
    ax.legend(loc='upper right', ncol=3)
    ax.grid(True, alpha=0.3, linewidth=0.5)
    ax.axhline(y=0, color='gray', linestyle='-', linewidth=0.5, alpha=0.5)


def plot_measurement_error(df: pd.DataFrame, ax: plt.Axes):
    """Plot measurement error over time."""
    time = df['time'].values
    
    if 'meas_err_rel_pos' in df.columns:
        ax.plot(time, df['meas_err_rel_pos'], label='Pos Error', linewidth=1.2, color='#1f77b4')
    if 'meas_err_vel' in df.columns:
        ax.plot(time, df['meas_err_vel'], label='Vel Error', linewidth=1.2, color='#ff7f0e')
    
    ax.set_ylabel('Error')
    ax.set_title('Measurement Error')
    ax.legend(loc='upper right', ncol=2)
    ax.grid(True, alpha=0.3, linewidth=0.5)
    ax.axhline(y=0, color='gray', linestyle='-', linewidth=0.5, alpha=0.5)


def plot_fleet_states(df: pd.DataFrame, fleet_size: int, ax_pos: plt.Axes, ax_vel: plt.Axes):
    """Plot absolute fleet positions and velocities."""
    time = df['time'].values
    colors = plt.cm.Set1(np.linspace(0, 1, max(fleet_size, 1)))
    
    for vid in range(fleet_size):
        col_x = f'fleet_x_{vid}'
        col_v = f'fleet_v_{vid}'
        label = 'Leader' if vid == 0 else f'F{vid}'
        ls = '--' if vid == 0 else '-'
        lw = 1.5 if vid == 0 else 1.2
        
        if col_x in df.columns:
            ax_pos.plot(time, df[col_x], label=label, color=colors[vid], linewidth=lw, linestyle=ls)
        if col_v in df.columns:
            ax_vel.plot(time, df[col_v], label=label, color=colors[vid], linewidth=lw, linestyle=ls)
    
    ax_pos.set_ylabel('X [m]')
    ax_pos.set_title('Fleet Positions')
    ax_pos.legend(loc='upper right', ncol=2)
    ax_pos.grid(True, alpha=0.3, linewidth=0.5)
    
    ax_vel.set_ylabel('V [m/s]')
    ax_vel.set_title('Fleet Velocities')
    ax_vel.legend(loc='upper right', ncol=2)
    ax_vel.grid(True, alpha=0.3, linewidth=0.5)


def plot_consensus_info(df: pd.DataFrame, ax: plt.Axes):
    """Plot consensus information."""
    time = df['time'].values
    ax2 = ax.twinx()
    
    if 'neighbor_count' in df.columns:
        ax.plot(time, df['neighbor_count'], label='Neighbors', linewidth=1.2, color='#1f77b4')
    
    if 'consensus_norm' in df.columns:
        ax2.plot(time, df['consensus_norm'], label='Cons. Norm', linewidth=1.2, color='#d62728')
    
    ax.set_ylabel('Count', color='#1f77b4')
    ax2.set_ylabel('Norm', color='#d62728')
    ax.set_title('Consensus Info')
    ax.grid(True, alpha=0.3, linewidth=0.5)
    
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc='upper right', ncol=2)


def create_full_plot(df: pd.DataFrame, title: str = "Observer Analysis", filepath: str = ""):
    """Create comprehensive multi-panel plot."""
    # Filter out last 10 seconds of data
    max_time = df['time'].max()
    cutoff_time = max_time - 10.0
    df = df[df['time'] <= cutoff_time].copy()
    
    observer_size = detect_observer_size(df)
    fleet_size = detect_fleet_size(df)
    
    # Calculate stats
    duration = df['time'].max() - df['time'].min()
    samples = len(df)
    
    fig = plt.figure(figsize=(14, 10))
    
    # Title with stats (title already includes data source)
    stats_str = f"Duration: {format_duration(duration)} | Samples: {samples} | Fleet: {fleet_size} vehicles"
    fig.suptitle(f"{title}\n{stats_str}", fontsize=9, fontweight='bold')
    
    gs = GridSpec(4, 2, figure=fig, hspace=0.4, wspace=0.3, 
                  top=0.92, bottom=0.06, left=0.08, right=0.95)
    
    ax_pos = fig.add_subplot(gs[0, 0])
    ax_vel = fig.add_subplot(gs[0, 1])
    ax_acc = fig.add_subplot(gs[1, 0])
    plot_observer_states(df, observer_size, ax_pos, ax_vel, ax_acc)
    
    ax_terms = fig.add_subplot(gs[1, 1])
    plot_observer_terms(df, observer_size, ax_terms, vehicle_idx=1)
    
    ax_meas = fig.add_subplot(gs[2, 0])
    ax_cons = fig.add_subplot(gs[2, 1])
    plot_measurement_error(df, ax_meas)
    plot_consensus_info(df, ax_cons)
    
    ax_fleet_pos = fig.add_subplot(gs[3, 0])
    ax_fleet_vel = fig.add_subplot(gs[3, 1])
    plot_fleet_states(df, fleet_size, ax_fleet_pos, ax_fleet_vel)
    
    ax_fleet_pos.set_xlabel('Time [s]')
    ax_fleet_vel.set_xlabel('Time [s]')
    
    return fig


def main():
    parser = argparse.ArgumentParser(
        description='Plot Distributed Luenberger Observer recordings',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python plot_distributed_luenberger.py -i --fake   # Interactive fake vehicle
  python plot_distributed_luenberger.py --fake      # Latest fake recording
  python plot_distributed_luenberger.py --real      # Latest real recording
        """
    )
    parser.add_argument('filepath', nargs='?', help='Path to CSV file')
    
    source_group = parser.add_mutually_exclusive_group()
    source_group.add_argument('--fake', '-f', action='store_true', help='Fake vehicle recordings')
    source_group.add_argument('--real', '-r', action='store_true', help='Real vehicle recordings')
    
    parser.add_argument('--interactive', '-i', action='store_true', help='Interactive selection')
    parser.add_argument('--list', '-l', action='store_true', help='List recordings')
    parser.add_argument('--dir', '-d', help='Custom directory')
    parser.add_argument('--save', '-s', action='store_true', help='Save plot to file')
    parser.add_argument('--output', '-o', help='Output file path')
    
    args = parser.parse_args()
    
    # Determine search directories
    if args.dir:
        search_dirs = [args.dir]
        source_name = "custom"
    elif args.fake:
        search_dirs = RECORDING_DIRS['fake']
        source_name = "fake vehicle"
    elif args.real:
        search_dirs = RECORDING_DIRS['real']
        source_name = "real vehicle"
    else:
        search_dirs = RECORDING_DIRS['fake'] + RECORDING_DIRS['real']
        source_name = "auto"
    
    # List mode
    if args.list:
        recordings = list_recordings(search_dirs)
        print(f"\n=== {source_name.upper()} RECORDINGS ===\n")
        if not recordings:
            print("No recordings found.")
        else:
            for i, (path, mtime, size) in enumerate(recordings[:20]):
                print(f"  {i+1}. [{mtime.strftime('%Y-%m-%d %H:%M')}] {format_file_size(size):>8} | {path}")
        return
    
    # Find file to plot
    if args.filepath:
        filepath = args.filepath
    elif args.interactive:
        filepath = interactive_select(search_dirs, source_name)
        if not filepath:
            print("\nCancelled.")
            return
    else:
        filepath = find_latest_recording(search_dirs)
        if not filepath:
            print(f"\nNo recordings found for {source_name}")
            print("\nTry: python plot_distributed_luenberger.py -i --fake")
            sys.exit(1)
        print(f"Using latest: {filepath}")
    
    # Load and plot
    try:
        df = load_data(filepath)
        print(f"Loaded {len(df)} samples")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    
    basename = os.path.basename(filepath)
    # Get file modification time
    mtime = datetime.fromtimestamp(os.path.getmtime(filepath))
    time_str = mtime.strftime("%Y-%m-%d %H:%M:%S")
    title = f"Distributed Luenberger Observer\nData: {basename} (Created: {time_str})"
    
    fig = create_full_plot(df, title, filepath)
    
    # Auto-save figure to figure/ directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    figure_dir = os.path.join(script_dir, 'figure')
    os.makedirs(figure_dir, exist_ok=True)
    
    # Generate output filename: replace 'dist' with 'figure' and .csv with .png
    figure_filename = basename.replace('dist_luenberger', 'figure_luenberger').replace('.csv', '.png')
    output_path = os.path.join(figure_dir, figure_filename)
    
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Figure saved to: {output_path}")
    
    if not args.save:
        plt.show()


if __name__ == "__main__":
    main()
