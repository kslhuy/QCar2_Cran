"""
Neural Observer Data Plotter

Unified visualization tool for both 1-layer and 2-layer neural observer recordings.

Creates multi-panel plots showing:
- State estimation vs measurements
- Unknown input / tire residual estimates
- Estimation errors
- Training loss (2-layer only)
- X-Y trajectory

Supports both recording modes:
- '1layer': First-layer observer data (qLPV, Differentiator-UIO)
- '2layer': Full two-layer neural observer data

Interactvity:
- Click legend items to toggle lines on/off.
- Hover over data points to see values (requires 'mplcursors' package).

Usage:
    python plot_neural_obs_data.py [path/to/recording.csv]
    python plot_neural_obs_data.py --mode 1layer  # Select from 1-layer recordings
    python plot_neural_obs_data.py --mode 2layer  # Select from 2-layer recordings
    
If no path is provided, opens an interactive file selector.
"""

import os
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from pathlib import Path
from typing import Optional, Dict, Literal, List, Union
from datetime import datetime

# Try to import pandas for CSV loading
try:
    import pandas as pd
    PANDAS_AVAILABLE = True
except ImportError:
    PANDAS_AVAILABLE = False

# Try to import mplcursors for hover interactivity
try:
    import mplcursors
    MPLCURSORS_AVAILABLE = True
except ImportError:
    MPLCURSORS_AVAILABLE = False


RecordingMode = Literal['1layer', '2layer']
PlotType = Literal['all', 'trajectory', 'states', 'errors', 'debug', 'acceleration', 'tire', 'control', 'loss']


# =============================================================================
# Publication-Quality Style Configuration
# =============================================================================

def setup_publication_style():
    """
    Configure matplotlib for publication-quality figures.
    Based on IEEE/journal standards for clear, professional plots.
    """
    # Use a clean style base
    plt.style.use('seaborn-v0_8-whitegrid')
    
    # Publication-quality settings
    pub_params = {
        # Figure
        'figure.figsize': (7, 5),
        'figure.dpi': 150,
        'figure.facecolor': 'white',
        'figure.edgecolor': 'white',
        'figure.autolayout': False,
        
        # Font settings (use serif for publications, or sans-serif for modern look)
        'font.family': 'serif',
        'font.serif': ['Times New Roman', 'DejaVu Serif', 'Computer Modern Roman'],
        'font.size': 10,
        'mathtext.fontset': 'cm',  # Computer Modern for math
        
        # Axes
        'axes.labelsize': 11,
        'axes.titlesize': 12,
        'axes.titleweight': 'bold',
        'axes.linewidth': 0.8,
        'axes.edgecolor': '#333333',
        'axes.labelcolor': '#333333',
        'axes.spines.top': True,
        'axes.spines.right': True,
        'axes.grid': True,
        'axes.axisbelow': True,
        
        # Grid
        'grid.color': '#E0E0E0',
        'grid.linewidth': 0.5,
        'grid.linestyle': '-',
        'grid.alpha': 0.7,
        
        # Ticks
        'xtick.labelsize': 9,
        'ytick.labelsize': 9,
        'xtick.direction': 'in',
        'ytick.direction': 'in',
        'xtick.major.size': 4,
        'ytick.major.size': 4,
        'xtick.minor.size': 2,
        'ytick.minor.size': 2,
        'xtick.major.width': 0.8,
        'ytick.major.width': 0.8,
        'xtick.color': '#333333',
        'ytick.color': '#333333',
        
        # Legend
        'legend.fontsize': 8,
        'legend.frameon': True,
        'legend.framealpha': 0.95,
        'legend.edgecolor': '#CCCCCC',
        'legend.fancybox': False,
        'legend.shadow': False,
        'legend.handlelength': 1.5,
        'legend.handletextpad': 0.5,
        'legend.columnspacing': 1.0,
        'legend.borderpad': 0.4,
        
        # Lines
        'lines.linewidth': 1.5,
        'lines.markersize': 4,
        
        # Savefig
        'savefig.dpi': 300,
        'savefig.bbox': 'tight',
        'savefig.pad_inches': 0.05,
        'savefig.transparent': False,
    }
    
    mpl.rcParams.update(pub_params)


# Publication color palette (colorblind-friendly)
COLORS = {
    'measured': '#E74C3C',      # Red - measurements
    'estimated': '#3498DB',     # Blue - neural/final estimates  
    'true': '#27AE60',          # Green - ground truth
    'layer1': '#9B59B6',        # Purple - first layer
    'neural': '#2980B9',        # Dark blue - neural estimates
    'highlight': '#F39C12',     # Orange - highlights
    'secondary': '#1ABC9C',     # Teal - secondary
    'gray': '#7F8C8D',          # Gray - auxiliary
}

# Line styles for distinguishing data series
LINE_STYLES = {
    'measured': {'color': COLORS['measured'], 'linestyle': '-', 'linewidth': 1.2, 'alpha': 0.8},
    'estimated': {'color': COLORS['estimated'], 'linestyle': '-', 'linewidth': 1.8, 'alpha': 0.95},
    'true': {'color': COLORS['true'], 'linestyle': '--', 'linewidth': 1.5, 'alpha': 0.9},
    'layer1': {'color': COLORS['layer1'], 'linestyle': '-.', 'linewidth': 1.3, 'alpha': 0.85},
}

MARKER_STYLES = {
    'measured': {'color': COLORS['measured'], 'marker': 'o', 'markersize': 2, 'linestyle': 'none', 'alpha': 0.6},
    'estimated': {'color': COLORS['estimated'], 'marker': 's', 'markersize': 2.5, 'linestyle': 'none', 'alpha': 0.8},
    'true': {'color': COLORS['true'], 'marker': '^', 'markersize': 3, 'linestyle': 'none', 'alpha': 0.7},
}


def load_data(filepath: str) -> Dict[str, np.ndarray]:
    """
    Load recorded data from CSV file.
    
    Args:
        filepath: Path to CSV file
        
    Returns:
        Dictionary with column names as keys and numpy arrays as values
    """
    if PANDAS_AVAILABLE:
        df = pd.read_csv(filepath)
        return {col: df[col].values for col in df.columns}
    else:
        # Fallback using numpy
        data = np.genfromtxt(filepath, delimiter=',', names=True)
        return {name: data[name] for name in data.dtype.names}


def detect_recording_mode(data: Dict[str, np.ndarray]) -> RecordingMode:
    """
    Detect the recording mode from loaded data.
    
    Args:
        data: Dictionary of recorded data
        
    Returns:
        Recording mode ('1layer' or '2layer')
    """
    # 2-layer mode has NN outputs (w_r_nn) and loss column
    if 'w_r_nn' in data or 'loss' in data:
        return '2layer'
    else:
        return '1layer'


def find_recordings(base_dirs: Union[str, List[str]] = "neural_obs_recordings",
                    recursive: bool = True) -> List[Path]:
    """
    Find all recording files in specified directories.
    
    Args:
        base_dirs: Directory or list of directories to search
        recursive: Whether to search subdirectories
        
    Returns:
        List of Path objects for found recordings
    """
    if isinstance(base_dirs, str):
        base_dirs = [base_dirs]
        
    # If default is used, also look relative to this script's location
    if base_dirs == ["neural_obs_recordings"]:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        local_rec_dir = os.path.join(script_dir, "neural_obs_recordings")
        if local_rec_dir not in base_dirs:
            base_dirs.append(local_rec_dir)
        
    all_files = []
    seen_paths = set()
    
    for base_dir in base_dirs:
        if not os.path.exists(base_dir):
            continue
            
        path_obj = Path(base_dir)
        pattern = "*.csv"
        
        if recursive:
            files = list(path_obj.rglob(pattern))
        else:
            files = list(path_obj.glob(pattern))
            
        for f in files:
            # Avoid duplicates if paths overlap
            abs_path = f.resolve()
            if abs_path not in seen_paths:
                all_files.append(f)
                seen_paths.add(abs_path)
        
    # Sort by modification time, newest first
    all_files.sort(key=lambda x: x.stat().st_mtime, reverse=True)
    return all_files


def select_directory_interactive(base_dirs: List[str]) -> Optional[str]:
    """
    Interactive CLI menu to select a source directory.
    """
    # Filter only existing directories
    valid_dirs = [d for d in base_dirs if os.path.exists(d)]
    
    if not valid_dirs:
        print("No recording directories found.")
        return None
        
    print("\nSelect Source Directory:")
    print("-" * 50)
    for i, d in enumerate(valid_dirs):
        # Count files in dir
        path = Path(d)
        count = len(list(path.glob("*.csv"))) + len(list(path.glob("**/*.csv")))
        print(f"{i+1}. {d:<40} ({count} files)")
        
    print(f"{len(valid_dirs)+1}. [Manual Input]")
    print("-" * 50)
    
    while True:
        try:
            selection = input(f"\nSelect directory (1-{len(valid_dirs)+1}) or 'q': ").strip()
            if selection.lower() == 'q':
                return None
                
            if selection == "":
                idx = 0
            else:
                idx = int(selection) - 1
            
            if 0 <= idx < len(valid_dirs):
                return valid_dirs[idx]
            elif idx == len(valid_dirs):
                # Manual input
                manual = input("Enter directory path: ").strip()
                if os.path.exists(manual):
                    return manual
                else:
                    print(f"Directory not found: {manual}")
            else:
                print("Invalid selection.")
        except ValueError:
            print("Please enter a number.")
        except KeyboardInterrupt:
            return None


def select_recording_interactive(search_dir: str, 
                                 mode_filter: Optional[RecordingMode] = None) -> Optional[str]:
    """
    Interactive CLI menu to select a recording file from a specific directory.
    """
    files = find_recordings(search_dir, recursive=True)
    
    if not files:
        print(f"No recordings found in {search_dir}")
        return None
        
    # Filter by mode if requested
    if mode_filter:
        files = [f for f in files if f"_{mode_filter}_" in f.name]
        
    if not files:
        print(f"No recordings found matching mode '{mode_filter}' in {search_dir}")
        return None
        
    print(f"\nRecordings in '{search_dir}':")
    print("-" * 100)
    print(f"{'#':<4} {'Date':<18} {'Mode':<8} {'Size':<8} {'Filename'}")
    print("-" * 100)
    
    # List latest 20 files
    limit = 20
    display_files = files[:limit]
    
    for i, f in enumerate(display_files):
        # Detect mode from filename
        if '_1layer_' in f.name:
            mode = '1layer'
        elif '_2layer_' in f.name:
            mode = '2layer'
        elif f.parent.name == 'Layer1':
            mode = '1layer'
        elif f.parent.name == 'Layer2':
            mode = '2layer'
        else:
            mode = '?'
            
        dt = datetime.fromtimestamp(f.stat().st_mtime)
        size_kb = f.stat().st_size / 1024
        
        print(f"{i+1:<4} {dt.strftime('%Y-%m-%d %H:%M'):<18} {mode:<8} {size_kb:<6.1f}KB  {f.name}")
        
    if len(files) > limit:
        print(f"... and {len(files) - limit} more (newest shown)")
        
    print("-" * 100)
    
    while True:
        try:
            selection = input(f"\nSelect file (1-{len(display_files)}) or 'q' to quit [1]: ").strip()
            if selection.lower() == 'q':
                return None
            
            if selection == "":
                idx = 0
            else:
                idx = int(selection) - 1
                
            if 0 <= idx < len(display_files):
                return str(display_files[idx])
            else:
                print("Invalid selection.")
        except ValueError:
            print("Please enter a number.")
        except KeyboardInterrupt:
            # Propagate up
            raise


def select_plot_type_interactive() -> PlotType:
    """
    Interactive CLI menu to select plot type.
    """
    print("\nSelect Plot Type:")
    print("1. Show All (Overview)")
    print("2. XY Trajectory")
    print("3. States (Velocity, Yaw Rate)")
    print("4. Estimation Errors")
    print("5. Debug (NN Outputs / Tire Residuals)")
    print("6. Acceleration")
    print("7. Tire Forces & Slip Angles (Article)")
    print("8. Trajectory + Control Inputs (Article)")
    print("9. Training Loss (Article)")

    
    mapping = {
        '1': 'all',
        '2': 'trajectory',
        '3': 'states',
        '4': 'errors',
        '5': 'debug',
        '6': 'acceleration',
        '7': 'tire',
        '8': 'control',
        '9': 'loss'
    }
    
    while True:
        try:
            sel = input("\nSelect view [1]: ").strip()
            if sel == "": 
                return 'all'
            if sel.lower() == 'q':
                sys.exit(0)
            
            if sel in mapping:
                return mapping[sel]
            else:
                print("Invalid selection.")
        except KeyboardInterrupt:
            sys.exit(0)


def enable_interactive_legend(fig):
    """
    Enable clicking on legend line to toggle line visibility.
    """
    def on_pick(event):
        # On the pick event, find the original line corresponding to the legend proxy
        legend = event.artist
        is_visible = legend.get_visible()
        
        # Determine the index of the picked legend item
        # This is a bit tricky in matplotlib, usually easier to map
        # artist -> original line
        
        # Better approach: map legend text/handle to original line
        # But wait, legend.get_lines() returns new artists used in legend
        pass 
        
    # Correct implementation: bind pick_event
    # We need to map legend items to original lines
    pass


def make_plot_interactive(fig, axes_list):
    """
    Add interactivity to the figure:
    1. Legend picking to toggle lines
    2. Hover cursors (if mplcursors available)
    """
    
    # 1. Legend Picking
    # We need to collect all lines and legends
    all_legends = [ax.get_legend() for ax in axes_list if ax.get_legend() is not None]
    
    lines_map = {} # map legend_line -> original_line
    
    for ax in axes_list:
        leg = ax.get_legend()
        if leg:
            leg.set_draggable(True) # Make legend draggable
            
            # Map legend items to original lines
            # logic: for each handle/text pair in legend
            # handle is the proxy artist in legend window
            # orig_handle is the artist in the axes
            # But legend() creates new handles. 
            # We can use the fact that leg.legendHandles[i] corresponds to lines[i] usually
            # if we can find them.
            
            # Simple approach: enable picking on legend lines
            for leg_line, orig_line in zip(leg.get_lines(), ax.lines):
                leg_line.set_picker(5)  # 5 pts tolerance
                lines_map[leg_line] = orig_line
                
    def on_pick(event):
        leg_line = event.artist
        orig_line = lines_map.get(leg_line)
        if orig_line:
            vis = not orig_line.get_visible()
            orig_line.set_visible(vis)
            # Change alpha of legend line to indicate status
            leg_line.set_alpha(1.0 if vis else 0.2)
            fig.canvas.draw()
            
    fig.canvas.mpl_connect('pick_event', on_pick)


def add_subplot_label(ax, label, fontsize=12, fontweight='bold', loc='upper left'):
    """Add a subplot label (a), (b), etc. for publication figures."""
    ax.text(-0.12, 1.05, f'({label})', transform=ax.transAxes, 
            fontsize=fontsize, fontweight=fontweight, va='top', ha='left')


def format_axis_scientific(ax, axis='y', scilimits=(-2, 2)):
    """Format axis with scientific notation if needed."""
    if axis in ['y', 'both']:
        ax.ticklabel_format(axis='y', style='sci', scilimits=scilimits)
    if axis in ['x', 'both']:
        ax.ticklabel_format(axis='x', style='sci', scilimits=scilimits)


def plot_1layer_data(data: Dict[str, np.ndarray], 
                     title: str = "1-Layer Observer Analysis",
                     save_path: Optional[str] = None,
                     plot_type: PlotType = 'all',
                     dist_mode: Optional[str] = None):
    """
    Create publication-quality plots for 1-layer observer data.
    """
    # Setup publication style
    setup_publication_style()
    
    time = data.get('time', np.arange(len(list(data.values())[0])))
    axes = []
    
    # =========================================================================
    # Figure Configuration Based on Plot Type
    # =========================================================================
    
    if plot_type == 'all':
        fig = plt.figure(figsize=(14, 12))
        gs = fig.add_gridspec(3, 3, hspace=0.35, wspace=0.30,
                              left=0.08, right=0.95, top=0.92, bottom=0.08)
    elif plot_type == 'trajectory':
        fig = plt.figure(figsize=(7, 6))
        gs = fig.add_gridspec(1, 1, left=0.12, right=0.95, top=0.92, bottom=0.12)
    elif plot_type == 'states':
        fig = plt.figure(figsize=(12, 9))
        gs = fig.add_gridspec(2, 2, hspace=0.30, wspace=0.28, left=0.08, right=0.96, top=0.95, bottom=0.08)
    elif plot_type == 'errors':
        fig = plt.figure(figsize=(12, 5))
        gs = fig.add_gridspec(1, 2, wspace=0.25, left=0.08, right=0.95, top=0.88, bottom=0.15)
    elif plot_type == 'debug':
        fig = plt.figure(figsize=(14, 5))
        gs = fig.add_gridspec(1, 3, wspace=0.28, left=0.06, right=0.98, top=0.88, bottom=0.15)
    elif plot_type == 'acceleration':
        fig = plt.figure(figsize=(12, 5))
        gs = fig.add_gridspec(1, 2, wspace=0.25, left=0.08, right=0.95, top=0.88, bottom=0.15)

    if plot_type != 'all':
        fig.suptitle(title, fontsize=13, fontweight='bold', y=0.98)
    else:
        fig.suptitle(title, fontsize=14, fontweight='bold', y=0.97)
    
    # Helper function
    def is_active(categories):
        return plot_type == 'all' or plot_type in categories

    subplot_idx = 0  # For labeling subplots
    labels = 'abcdefghijklmnopqrstuvwxyz'
    
    # =========================================================================
    # 1. Longitudinal Velocity
    # =========================================================================
    if is_active(['states']):
        pos = gs[0, 0] if plot_type == 'all' else gs[0, 0]
        ax = fig.add_subplot(pos)
        
        ax.plot(time, data.get('vx_meas', []), color=COLORS['measured'], 
                linestyle='-', linewidth=1.0, alpha=0.6, label='Measured')
        ax.plot(time, data.get('vx_est', []), color=COLORS['estimated'], 
                linestyle='-', linewidth=1.8, alpha=0.95, label='Estimated')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(r'$v_x$ (m/s)')
        ax.set_title('Longitudinal Velocity', pad=8)
        ax.legend(loc='upper right', framealpha=0.95)
        add_subplot_label(ax, labels[subplot_idx])
        subplot_idx += 1
        axes.append(ax)

    # =========================================================================
    # 2. Lateral Velocity
    # =========================================================================
    if is_active(['states']):
        pos = gs[0, 1] if plot_type == 'all' else gs[0, 1]
        ax = fig.add_subplot(pos)
        
        ax.plot(time, data.get('vy_est', []), color=COLORS['estimated'], 
                linestyle='-', linewidth=1.8, alpha=0.95, label='Estimated')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(r'$v_y$ (m/s)')
        ax.set_title('Lateral Velocity', pad=8)
        ax.legend(loc='upper right', framealpha=0.95)
        add_subplot_label(ax, labels[subplot_idx])
        subplot_idx += 1
        axes.append(ax)

    # =========================================================================
    # 3. Yaw Rate
    # =========================================================================
    if is_active(['states']):
        pos = gs[0, 2] if plot_type == 'all' else gs[1, 0]
        ax = fig.add_subplot(pos)
        
        ax.plot(time, data.get('r_meas', []), color=COLORS['measured'], 
                linestyle='-', linewidth=1.0, alpha=0.6, label='Measured')
        ax.plot(time, data.get('r_est', []), color=COLORS['estimated'], 
                linestyle='-', linewidth=1.8, alpha=0.95, label='Estimated')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(r'$\dot{\psi}$ (rad/s)')
        ax.set_title('Yaw Rate', pad=8)
        ax.legend(loc='upper right', framealpha=0.95)
        add_subplot_label(ax, labels[subplot_idx])
        subplot_idx += 1
        axes.append(ax)

    # =========================================================================
    # 3b. Yaw Angle (states view only)
    # =========================================================================
    if plot_type == 'states':
        ax = fig.add_subplot(gs[1, 1])
        psi_est = data.get('psi_est', [])
        psi_meas = data.get('psi_meas', [])
        
        # subsample to keep scatter readable for long traces
        step = max(1, len(time) // 200)
        if len(psi_meas) > 0:
            ax.scatter(time[::step], np.rad2deg(psi_meas)[::step],
                       c=COLORS['measured'], s=18, alpha=0.6, marker='o', label='GPS')
        if len(psi_est) > 0:
            ax.plot(time, np.rad2deg(psi_est), color=COLORS['estimated'], linestyle='-', linewidth=1.4, alpha=0.9, label='Estimated')
            ax.scatter(time[::step], np.rad2deg(psi_est)[::step], c=COLORS['estimated'], s=10, alpha=0.85, marker='s')
        if 'psi_true' in data and np.any(data['psi_true']):
            ax.scatter(time[::step], np.rad2deg(data['psi_true'])[::step], c=COLORS['true'], s=16, alpha=0.8, marker='^', label='True')
        if 'psi_uio' in data:
            ax.scatter(time[::step], np.rad2deg(data['psi_uio'])[::step], c=COLORS['layer1'], s=12, alpha=0.75, marker='D', label='Layer 1')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(r'$\psi$ (deg)')
        ax.set_title('Yaw Angle', pad=8)
        ax.legend(loc='upper right', framealpha=0.95)
        add_subplot_label(ax, labels[subplot_idx])
        subplot_idx += 1
        axes.append(ax)

    # =========================================================================
    # 4. X-Y Trajectory
    # =========================================================================
    if is_active(['trajectory']):
        pos = gs[1, 2] if plot_type == 'all' else gs[0, 0]
        ax = fig.add_subplot(pos)
        
        X_est = data.get('X_est', [])
        Y_est = data.get('Y_est', [])
        X_meas = data.get('X_meas', [])
        Y_meas = data.get('Y_meas', [])
        
        if len(X_est) > 0:
            # GPS measurements (background)
            ax.scatter(X_meas, Y_meas, c=COLORS['measured'], s=8, alpha=0.4, 
                       label='GPS', zorder=2, marker='o')
            # Estimated trajectory (foreground)
            ax.plot(X_est, Y_est, color=COLORS['estimated'], linewidth=2.0, 
                    alpha=0.95, label='Estimated', zorder=3)
            # Start/End markers
            ax.scatter(X_est[0], Y_est[0], c=COLORS['true'], s=100, marker='o', 
                       edgecolors='white', linewidths=1.5, label='Start', zorder=4)
            ax.scatter(X_est[-1], Y_est[-1], c=COLORS['highlight'], s=100, marker='s', 
                       edgecolors='white', linewidths=1.5, label='End', zorder=4)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Vehicle Trajectory', pad=8)
        ax.legend(loc='best', framealpha=0.95)
        ax.set_aspect('equal', adjustable='box')
        add_subplot_label(ax, labels[subplot_idx])
        subplot_idx += 1
        axes.append(ax)

    # =========================================================================
    # 5. Yaw Angle (only in 'all' view)
    # =========================================================================
    if plot_type == 'all':
        ax = fig.add_subplot(gs[1, 1])
        psi_est = data.get('psi_est', [])
        psi_meas = data.get('psi_meas', [])
        
        if len(psi_est) > 0:
            ax.plot(time, np.rad2deg(psi_meas), color=COLORS['measured'], 
                    linestyle='-', linewidth=1.0, alpha=0.6, label='GPS')
            ax.plot(time, np.rad2deg(psi_est), color=COLORS['estimated'], 
                    linestyle='-', linewidth=1.8, alpha=0.95, label='Estimated')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(r'$\psi$ (deg)')
        ax.set_title('Yaw Angle', pad=8)
        ax.legend(loc='upper right', framealpha=0.95)
        add_subplot_label(ax, labels[subplot_idx])
        subplot_idx += 1
        axes.append(ax)

    # =========================================================================
    # 6. Tire Residuals / Disturbances
    # =========================================================================
    if is_active(['debug']):
        pos = gs[1, 0:2] if plot_type == 'all' else gs[0, 0]
        ax = fig.add_subplot(pos)
        
        # Determine disturbance mode
        use_general = dist_mode == 'general' or (dist_mode is None and 'd_vx' in data)

        if use_general:
            # 3D General Disturbances
            ax.plot(time, data.get('d_vx', []), color=COLORS['estimated'], 
                    linestyle='-', linewidth=1.5, label=r'$\hat{d}_{v_x}$')
            ax.plot(time, data.get('d_vy', []), color=COLORS['true'], 
                    linestyle='-', linewidth=1.5, label=r'$\hat{d}_{v_y}$')
            ax.plot(time, data.get('d_r', []), color=COLORS['highlight'], 
                    linestyle='-', linewidth=1.5, label=r'$\hat{d}_{r}$')
            
            # True values if available
            if 'd_vx_true' in data:
                ax.plot(time, data['d_vx_true'], color=COLORS['estimated'], 
                        linestyle='--', linewidth=1.2, alpha=0.7, label=r'$d_{v_x}$ (true)')
            if 'd_vy_true' in data:
                ax.plot(time, data['d_vy_true'], color=COLORS['true'], 
                        linestyle='--', linewidth=1.2, alpha=0.7, label=r'$d_{v_y}$ (true)')
            if 'd_r_true' in data:
                ax.plot(time, data['d_r_true'], color=COLORS['highlight'], 
                        linestyle='--', linewidth=1.2, alpha=0.7, label=r'$d_{r}$ (true)')
                
            ax.set_ylabel('Disturbance')
            ax.set_title('Disturbance Estimates', pad=8)
        else:
            # 2D Tire Residuals
            ax.plot(time, data.get('w_r', []), color=COLORS['estimated'], 
                    linestyle='-', linewidth=1.5, label=r'$\hat{w}_r$')
            ax.plot(time, data.get('w_f', []), color=COLORS['highlight'], 
                    linestyle='-', linewidth=1.5, label=r'$\hat{w}_f$')
            
            if 'w_r_true' in data and np.any(data['w_r_true']):
                ax.plot(time, data['w_r_true'], color=COLORS['estimated'], 
                        linestyle='--', linewidth=1.2, alpha=0.7, label=r'$w_r$ (true)')
            if 'w_f_true' in data and np.any(data['w_f_true']):
                ax.plot(time, data['w_f_true'], color=COLORS['highlight'], 
                        linestyle='--', linewidth=1.2, alpha=0.7, label=r'$w_f$ (true)')
                
            ax.set_ylabel('Tire Residual (N)')
            ax.set_title('Unknown Input Estimates', pad=8)
            
        ax.set_xlabel('Time (s)')
        ax.legend(loc='upper right', framealpha=0.95, ncol=2)
        add_subplot_label(ax, labels[subplot_idx])
        subplot_idx += 1
        axes.append(ax)

    # =========================================================================
    # 7. Control Inputs
    # =========================================================================
    if is_active(['debug']):
        pos = gs[2, 0] if plot_type == 'all' else gs[0, 1]
        ax = fig.add_subplot(pos)
        
        ax.plot(time, data.get('steering', []), color=COLORS['estimated'], 
                linestyle='-', linewidth=1.5, label='Steering (rad)')
        ax.plot(time, data.get('throttle', []), color=COLORS['highlight'], 
                linestyle='-', linewidth=1.5, label='Throttle')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Control Input')
        ax.set_title('Control Inputs', pad=8)
        ax.legend(loc='upper right', framealpha=0.95)
        add_subplot_label(ax, labels[subplot_idx])
        subplot_idx += 1
        axes.append(ax)

    # =========================================================================
    # 8. Position Error
    # =========================================================================
    if is_active(['errors']):
        pos = gs[2, 1] if plot_type == 'all' else gs[0, 0]
        ax = fig.add_subplot(pos)
        
        X_err = np.array(data.get('X_est', [])) - np.array(data.get('X_meas', []))
        Y_err = np.array(data.get('Y_est', [])) - np.array(data.get('Y_meas', []))
        
        if len(X_err) > 0 and len(Y_err) > 0:
            pos_err = np.sqrt(X_err**2 + Y_err**2) * 100  # Convert to cm
            ax.plot(time, pos_err, color=COLORS['estimated'], linewidth=1.5, alpha=0.9)
            mean_err = np.mean(pos_err)
            ax.axhline(y=mean_err, color=COLORS['highlight'], linestyle='--', 
                       linewidth=1.5, label=f'Mean: {mean_err:.2f} cm')
            ax.fill_between(time, 0, pos_err, color=COLORS['estimated'], alpha=0.15)
            ax.legend(loc='upper right', framealpha=0.95)
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position Error (cm)')
        ax.set_title('Position Estimation Error', pad=8)
        add_subplot_label(ax, labels[subplot_idx])
        subplot_idx += 1
        axes.append(ax)

    # =========================================================================
    # 9. GPS Valid
    # =========================================================================
    if is_active(['errors', 'debug']):
        if plot_type == 'all': 
            pos = gs[2, 2]
        elif plot_type == 'errors': 
            pos = gs[0, 1]
        elif plot_type == 'debug': 
            pos = gs[0, 2]
        else: 
            pos = None
        
        if pos:
            ax = fig.add_subplot(pos)
            gps_valid = data.get('gps_valid', [])
            
            if len(gps_valid) > 0:
                ax.fill_between(time, 0, gps_valid, color=COLORS['true'], 
                               alpha=0.3, label='GPS Valid')
                ax.plot(time, gps_valid, color=COLORS['true'], linewidth=1.2)
            
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('GPS Valid')
            ax.set_title('GPS Availability', pad=8)
            ax.set_ylim(-0.1, 1.1)
            ax.set_yticks([0, 1])
            ax.set_yticklabels(['Invalid', 'Valid'])
            add_subplot_label(ax, labels[subplot_idx])
            subplot_idx += 1
            axes.append(ax)

    # =========================================================================
    # 10. Acceleration
    # =========================================================================
    if is_active(['acceleration']):
        pos_ax = gs[0, 0] if plot_type == 'acceleration' else None
        pos_ay = gs[0, 1] if plot_type == 'acceleration' else None
        
        if pos_ax:
            ax = fig.add_subplot(pos_ax)
            ax.plot(time, data.get('ax_meas', []), color=COLORS['measured'], 
                    linestyle='-', linewidth=1.5, label='Measured')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(r'$a_x$ (m/s²)')
            ax.set_title('Longitudinal Acceleration', pad=8)
            ax.legend(loc='upper right', framealpha=0.95)
            add_subplot_label(ax, labels[subplot_idx])
            subplot_idx += 1
            axes.append(ax)
            
        if pos_ay:
            ax = fig.add_subplot(pos_ay)
            ax.plot(time, data.get('ay_meas', []), color=COLORS['measured'], 
                    linestyle='-', linewidth=1.5, label='Measured')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(r'$a_y$ (m/s²)')
            ax.set_title('Lateral Acceleration', pad=8)
            ax.legend(loc='upper right', framealpha=0.95)
            add_subplot_label(ax, labels[subplot_idx])
            subplot_idx += 1
            axes.append(ax)

    # =========================================================================
    # Tire Force Decomposition (separate figure for 1-layer)
    # =========================================================================
    if is_active(['debug']) and ('Fyr_est' in data or 'Fyr_true' in data):
        has_true = 'Fyr_true' in data and np.any(data['Fyr_true'])
        has_est = 'Fyr_est' in data and np.any(data['Fyr_est'])
        
        if has_true or has_est:
            fig_tire = plt.figure(figsize=(14, 8))
            gs_tire = fig_tire.add_gridspec(2, 2, hspace=0.30, wspace=0.25,
                                             left=0.08, right=0.95, top=0.92, bottom=0.10)
            fig_tire.suptitle('Tire Force Decomposition', fontsize=13, fontweight='bold')
            
            tire_labels = 'abcd'
            
            # Rear Lateral Force
            ax_fyr = fig_tire.add_subplot(gs_tire[0, 0])
            if has_true:
                ax_fyr.plot(time, data['Fyr_true'], color=COLORS['true'], 
                           linestyle='-', linewidth=1.5, label=r'$F_{yr}$ (true)')
            if has_est:
                ax_fyr.plot(time, data['Fyr_est'], color=COLORS['estimated'], 
                           linestyle='-', linewidth=1.8, label=r'$\hat{F}_{yr}$ (total)')
            if 'Fyr_linear_only' in data:
                ax_fyr.plot(time, data['Fyr_linear_only'], color=COLORS['layer1'], 
                           linestyle='--', linewidth=1.2, alpha=0.8, label=r'$C_r\hat{\alpha}_r$ (linear)')
            ax_fyr.set_xlabel('Time (s)')
            ax_fyr.set_ylabel(r'$F_{yr}$ (N)')
            ax_fyr.set_title('Rear Lateral Force', pad=8)
            ax_fyr.legend(loc='best', framealpha=0.95)
            add_subplot_label(ax_fyr, tire_labels[0])
            
            # Front Lateral Force
            ax_fyf = fig_tire.add_subplot(gs_tire[0, 1])
            if has_true:
                ax_fyf.plot(time, data['Fyf_true'], color=COLORS['true'], 
                           linestyle='-', linewidth=1.5, label=r'$F_{yf}$ (true)')
            if 'Fyf_est' in data:
                ax_fyf.plot(time, data['Fyf_est'], color=COLORS['estimated'], 
                           linestyle='-', linewidth=1.8, label=r'$\hat{F}_{yf}$ (total)')
            if 'Fyf_linear_only' in data:
                ax_fyf.plot(time, data['Fyf_linear_only'], color=COLORS['layer1'], 
                           linestyle='--', linewidth=1.2, alpha=0.8, label=r'$C_f\hat{\alpha}_f$ (linear)')
            ax_fyf.set_xlabel('Time (s)')
            ax_fyf.set_ylabel(r'$F_{yf}$ (N)')
            ax_fyf.set_title('Front Lateral Force', pad=8)
            ax_fyf.legend(loc='best', framealpha=0.95)
            add_subplot_label(ax_fyf, tire_labels[1])
            
            # Rear Slip Angle
            ax_ar = fig_tire.add_subplot(gs_tire[1, 0])
            if 'alpha_r' in data:
                ax_ar.plot(time, np.rad2deg(data['alpha_r']), color=COLORS['true'], 
                          linestyle='-', linewidth=1.5, label=r'$\alpha_r$ (true)')
            if 'alpha_r_est' in data:
                ax_ar.plot(time, np.rad2deg(data['alpha_r_est']), color=COLORS['estimated'], 
                          linestyle='--', linewidth=1.5, label=r'$\hat{\alpha}_r$ (est)')
            ax_ar.set_xlabel('Time (s)')
            ax_ar.set_ylabel(r'$\alpha_r$ (deg)')
            ax_ar.set_title('Rear Slip Angle', pad=8)
            ax_ar.legend(loc='best', framealpha=0.95)
            add_subplot_label(ax_ar, tire_labels[2])
            
            # Front Slip Angle
            ax_af = fig_tire.add_subplot(gs_tire[1, 1])
            if 'alpha_f' in data:
                ax_af.plot(time, np.rad2deg(data['alpha_f']), color=COLORS['true'], 
                          linestyle='-', linewidth=1.5, label=r'$\alpha_f$ (true)')
            if 'alpha_f_est' in data:
                ax_af.plot(time, np.rad2deg(data['alpha_f_est']), color=COLORS['estimated'], 
                          linestyle='--', linewidth=1.5, label=r'$\hat{\alpha}_f$ (est)')
            ax_af.set_xlabel('Time (s)')
            ax_af.set_ylabel(r'$\alpha_f$ (deg)')
            ax_af.set_title('Front Slip Angle', pad=8)
            ax_af.legend(loc='best', framealpha=0.95)
            add_subplot_label(ax_af, tire_labels[3])

    # Add interactivity
    make_plot_interactive(fig, axes)
    
    if save_path:
        if plot_type != 'all':
            p = Path(save_path)
            save_path = str(p.parent / f"{p.stem}_{plot_type}{p.suffix}")
        fig.savefig(save_path, dpi=300, bbox_inches='tight', facecolor='white')
        print(f"Figure saved to: {save_path}")
    
    plt.show()




def plot_2layer_data(data: Dict[str, np.ndarray], 
                     title: str = "2-Layer Neural Observer Analysis",
                     save_path: Optional[str] = None,
                     plot_type: PlotType = 'all',
                     dist_mode: Optional[str] = None):
    """
    Create publication-quality plots for 2-layer neural observer data.
    """
    # Setup publication style
    setup_publication_style()
    
    time = data.get('time', np.arange(len(list(data.values())[0])))
    axes = []
    subplot_idx = 0
    labels = 'abcdefghijklmnopqrstuvwxyz'
    
    # =========================================================================
    # Figure Configuration Based on Plot Type
    # =========================================================================
    
    if plot_type == 'all':
        fig = plt.figure(figsize=(16, 18))
        gs = fig.add_gridspec(6, 3, hspace=0.32, wspace=0.28,
                              left=0.06, right=0.96, top=0.94, bottom=0.05)
    elif plot_type == 'trajectory':
        fig = plt.figure(figsize=(7, 6))
        gs = fig.add_gridspec(1, 1, left=0.12, right=0.95, top=0.92, bottom=0.12)
    elif plot_type == 'states':
        fig = plt.figure(figsize=(12, 9))
        gs = fig.add_gridspec(2, 2, hspace=0.30, wspace=0.28, left=0.08, right=0.96, top=0.95, bottom=0.08)
    elif plot_type == 'errors':
        fig = plt.figure(figsize=(12, 5))
        gs = fig.add_gridspec(1, 2, wspace=0.25, left=0.08, right=0.95, top=0.88, bottom=0.15)
    elif plot_type == 'debug':
        fig = plt.figure(figsize=(16, 12))
        gs = fig.add_gridspec(3, 3, hspace=0.32, wspace=0.28,
                              left=0.06, right=0.96, top=0.94, bottom=0.08)
    elif plot_type == 'acceleration':
        fig = plt.figure(figsize=(12, 5))
        gs = fig.add_gridspec(1, 2, wspace=0.25, left=0.08, right=0.95, top=0.88, bottom=0.15)

    def is_active(categories):
        return plot_type == 'all' or plot_type in categories

    # =========================================================================
    # 1. Longitudinal Velocity
    # =========================================================================
    if is_active(['states']):
        pos = gs[0, 0] if plot_type == 'all' else gs[0, 0]
        ax = fig.add_subplot(pos)
        
        # Background: Measured
        # ax.plot(time, data.get('vx_meas', []), color=COLORS['measured'], 
        #         linestyle='-', linewidth=0.8, alpha=0.5, label='Measured')
        # 1st Layer
        if 'vx_uio' in data:
            ax.plot(time, data['vx_uio'], color=COLORS['layer1'], 
                    linestyle='-.', linewidth=1.2, alpha=0.8, label='Layer 1')
        # Neural Estimate (main)
        ax.plot(time, data.get('vx_est', []), color=COLORS['estimated'], 
                linestyle='-', linewidth=1.8, alpha=0.95, label='Neural')
        # True (reference)
        if 'vx_true' in data and np.any(data['vx_true']):
            ax.plot(time, data['vx_true'], color=COLORS['true'], 
                    linestyle='--', linewidth=1.5, alpha=0.85, label='True')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(r'$v_x$ (m/s)')
        ax.set_title('Longitudinal Velocity', pad=8)
        ax.legend(loc='upper right', framealpha=0.95, ncol=2)
        subplot_idx += 1
        axes.append(ax)

    # =========================================================================
    # 2. Lateral Velocity
    # =========================================================================
    if is_active(['states']):
        pos = gs[0, 1] if plot_type == 'all' else gs[0, 1]
        ax = fig.add_subplot(pos)
        
        if 'vy_uio' in data:
            ax.plot(time, data['vy_uio'], color=COLORS['layer1'], 
                    linestyle='-.', linewidth=1.2, alpha=0.8, label='Layer 1')
        ax.plot(time, data.get('vy_est', []), color=COLORS['estimated'], 
                linestyle='-', linewidth=1.8, alpha=0.95, label='Neural')
        if 'vy_true' in data and np.any(data['vy_true']):
            ax.plot(time, data['vy_true'], color=COLORS['true'], 
                    linestyle='--', linewidth=1.5, alpha=0.85, label='True')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(r'$v_y$ (m/s)')
        ax.set_title('Lateral Velocity', pad=8)
        ax.legend(loc='upper right', framealpha=0.95)
        subplot_idx += 1
        axes.append(ax)

    # =========================================================================
    # 3. Yaw Rate
    # =========================================================================
    if is_active(['states']):
        pos = gs[0, 2] if plot_type == 'all' else gs[1, 0]
        ax = fig.add_subplot(pos)
        
        # ax.plot(time, data.get('r_meas', []), color=COLORS['measured'], 
        #         linestyle='-', linewidth=0.8, alpha=0.5, label='Measured')
        if 'r_uio' in data:
            ax.plot(time, data['r_uio'], color=COLORS['layer1'], 
                    linestyle='-.', linewidth=1.2, alpha=0.8, label='Layer 1')
        ax.plot(time, data.get('r_est', []), color=COLORS['estimated'], 
                linestyle='-', linewidth=1.8, alpha=0.95, label='Neural')
        if 'r_true' in data and np.any(data['r_true']):
            ax.plot(time, data['r_true'], color=COLORS['true'], 
                    linestyle='--', linewidth=1.5, alpha=0.85, label='True')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(r'$\dot{\psi}$ (rad/s)')
        ax.set_title('Yaw Rate', pad=8)
        ax.legend(loc='upper right', framealpha=0.95, ncol=2)
        subplot_idx += 1
        axes.append(ax)

    # =========================================================================
    # 3b. Yaw Angle (states view only)
    # =========================================================================
    if plot_type == 'states':
        ax = fig.add_subplot(gs[1, 1])
        psi_est = data.get('psi_est', [])
        psi_meas = data.get('psi_meas', [])
        
        step = max(1, len(time) // 200)
        if len(psi_meas) > 0:
            ax.scatter(time[::step], np.rad2deg(psi_meas)[::step], c=COLORS['measured'], s=18, alpha=0.6, marker='o', label='GPS')
        if 'psi_true' in data and np.any(data['psi_true']):
            ax.scatter(time[::step], np.rad2deg(data['psi_true'])[::step], c=COLORS['true'], s=16, alpha=0.8, marker='^', label='True')
        if 'psi_uio' in data:
            ax.scatter(time[::step], np.rad2deg(data['psi_uio'])[::step], c=COLORS['layer1'], s=12, alpha=0.75, marker='D', label='Layer 1')
        if len(psi_est) > 0:
            # ax.plot(time, np.rad2deg(psi_est), color=COLORS['estimated'], linestyle='-', linewidth=1.4, alpha=0.9, label='Neural')
            ax.scatter(time[::step], np.rad2deg(psi_est)[::step], c=COLORS['estimated'], s=10, alpha=0.85, marker='s')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(r'$\psi$ (deg)')
        ax.set_title('Yaw Angle', pad=8)
        ax.legend(loc='upper right', framealpha=0.95)
        subplot_idx += 1
        axes.append(ax)

    # =========================================================================
    # 4-6. Positions (X, Y, Yaw) - Only in 'all' view
    # =========================================================================
    if plot_type == 'all':
        # X Position
        ax4 = fig.add_subplot(gs[1, 0])
        ax4.plot(time, data.get('X_meas', []), color=COLORS['measured'], 
                linestyle='-', linewidth=0.8, alpha=0.5, label='GPS')
        if 'X_true' in data and np.any(data['X_true']):
            ax4.plot(time, data['X_true'], color=COLORS['true'], 
                    linestyle='--', linewidth=1.2, alpha=0.7, label='True')
        if 'X_uio' in data:
            ax4.plot(time, data['X_uio'], color=COLORS['layer1'], 
                    linestyle='-.', linewidth=1.0, alpha=0.7, label='Layer 1')
        ax4.plot(time, data.get('X_est', []), color=COLORS['estimated'], 
                linestyle='-', linewidth=1.5, alpha=0.95, label='Neural')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('X (m)')
        ax4.set_title('X Position', pad=8)
        ax4.legend(loc='upper right', framealpha=0.95)
        subplot_idx += 1
        axes.append(ax4)

        # Y Position
        ax5 = fig.add_subplot(gs[1, 1])
        ax5.plot(time, data.get('Y_meas', []), color=COLORS['measured'], 
                linestyle='-', linewidth=0.8, alpha=0.5, label='GPS')
        if 'Y_true' in data and np.any(data['Y_true']):
            ax5.plot(time, data['Y_true'], color=COLORS['true'], 
                    linestyle='--', linewidth=1.2, alpha=0.7, label='True')
        if 'Y_uio' in data:
            ax5.plot(time, data['Y_uio'], color=COLORS['layer1'], 
                    linestyle='-.', linewidth=1.0, alpha=0.7, label='Layer 1')
        ax5.plot(time, data.get('Y_est', []), color=COLORS['estimated'], 
                linestyle='-', linewidth=1.5, alpha=0.95, label='Neural')
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Y (m)')
        ax5.set_title('Y Position', pad=8)
        ax5.legend(loc='upper right', framealpha=0.95)
        subplot_idx += 1
        axes.append(ax5)

        # Yaw Angle
        ax6 = fig.add_subplot(gs[1, 2])
        psi_est = data.get('psi_est', [])
        psi_meas = data.get('psi_meas', [])
        if len(psi_est) > 0:
            ax6.plot(time, np.rad2deg(psi_meas), color=COLORS['measured'], 
                    linestyle='-', linewidth=0.8, alpha=0.5, label='GPS')
            if 'psi_true' in data and np.any(data['psi_true']):
                ax6.plot(time, np.rad2deg(data['psi_true']), color=COLORS['true'], 
                        linestyle='--', linewidth=1.2, alpha=0.7, label='True')
            if 'psi_uio' in data:
                ax6.plot(time, np.rad2deg(data['psi_uio']), color=COLORS['layer1'], 
                        linestyle='-.', linewidth=1.0, alpha=0.7, label='Layer 1')
            ax6.plot(time, np.rad2deg(psi_est), color=COLORS['estimated'], 
                    linestyle='-', linewidth=1.5, alpha=0.95, label='Neural')
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel(r'$\psi$ (deg)')
        ax6.set_title('Yaw Angle', pad=8)
        ax6.legend(loc='upper right', framealpha=0.95)
        subplot_idx += 1
        axes.append(ax6)

    # =========================================================================
    # 7. NN Tire Residuals / General Disturbances
    # =========================================================================
    if is_active(['debug']):
        pos = gs[3, 0:2] if plot_type == 'all' else gs[0, 0:2]
        ax = fig.add_subplot(pos)
        
        # Determine disturbance mode
        use_general = dist_mode == 'general' or (dist_mode is None and 'd_vx_nn' in data)

        if use_general:
            # True Disturbances (Solid lines - reference)
            if 'd_vx_true' in data:
                ax.plot(time, data['d_vx_true'], color=COLORS['estimated'], 
                        linestyle='-', linewidth=1.8, alpha=0.9, label=r'$d_{v_x}$ (true)')
            if 'd_vy_true' in data:
                ax.plot(time, data['d_vy_true'], color=COLORS['true'], 
                        linestyle='-', linewidth=1.8, alpha=0.9, label=r'$d_{v_y}$ (true)')
            if 'd_r_true' in data:
                ax.plot(time, data['d_r_true'], color=COLORS['highlight'], 
                        linestyle='-', linewidth=1.8, alpha=0.9, label=r'$d_{r}$ (true)')
            
            # Neural Estimates (Points/Markers)
            ax.scatter(time[::5], data.get('d_vx_nn', [])[::5], color=COLORS['estimated'], 
                      s=12, alpha=0.7, marker='o', label=r'$\hat{d}_{v_x}$ (NN)')
            ax.scatter(time[::5], data.get('d_vy_nn', [])[::5], color=COLORS['true'], 
                      s=12, alpha=0.7, marker='s', label=r'$\hat{d}_{v_y}$ (NN)')
            ax.scatter(time[::5], data.get('d_r_nn', [])[::5], color=COLORS['highlight'], 
                      s=12, alpha=0.7, marker='^', label=r'$\hat{d}_{r}$ (NN)')
            
            # First Layer (dashed)
            if 'd_vx_uio' in data:
                ax.plot(time, data['d_vx_uio'], color=COLORS['estimated'], 
                        linestyle='--', linewidth=1.2, alpha=0.6, label=r'$d_{v_x}$ (L1)')
                ax.plot(time, data['d_vy_uio'], color=COLORS['true'], 
                        linestyle='--', linewidth=1.2, alpha=0.6, label=r'$d_{v_y}$ (L1)')
                ax.plot(time, data['d_r_uio'], color=COLORS['highlight'], 
                        linestyle='--', linewidth=1.2, alpha=0.6, label=r'$d_{r}$ (L1)')
        
            ax.set_ylabel('Disturbance')
            ax.set_title('Disturbance Estimation Comparison', pad=8)
        else:
            # 2D Tire Residuals
            if 'w_r_true' in data:
                ax.plot(time, data['w_r_true'], color=COLORS['estimated'], 
                        linestyle='-', linewidth=1.8, alpha=0.9, label=r'$w_r$ (true)')
            if 'w_r_uio' in data:
                ax.plot(time, data.get('w_r_uio', []), color=COLORS['layer1'], 
                        linestyle='--', linewidth=1.5, alpha=0.75, label=r'$\hat{w}_r$ (L1)')
            # ax.scatter(time[::3], data.get('w_r_nn', [])[::3], color=COLORS['estimated'], 
            #           s=10, alpha=0.7, marker='o', label=r'$\hat{w}_r$ (NN)')
            ax.plot(time, data.get('w_r_nn', []), color=COLORS['estimated'], 
                    linestyle='-', linewidth=1.5, alpha=1, label=r'$\hat{w}_r$ (NN)')
            ax.set_ylabel('Tire Residual (N)')
            ax.set_title('Rear Tire Residual Estimation', pad=8)

        ax.set_xlabel('Time (s)')
        ax.legend(loc='upper right', framealpha=0.95, ncol=3)
        subplot_idx += 1
        axes.append(ax)
        
        # Front tire residual (separate subplot for clarity)
        if not use_general and 'w_f_true' in data:
            pos_wf = gs[3, 2] if plot_type == 'all' else gs[0, 2]
            ax_wf = fig.add_subplot(pos_wf)
            
            ax_wf.plot(time, data['w_f_true'], color=COLORS['highlight'], 
                      linestyle='-', linewidth=1.8, alpha=0.9, label=r'$w_f$ (true)')
            if 'w_f_uio' in data:
                ax_wf.plot(time, data.get('w_f_uio', []), color=COLORS['layer1'], 
                          linestyle='--', linewidth=1.5, alpha=0.75, label=r'$\hat{w}_f$ (L1)')
            ax_wf.scatter(time[::3], data.get('w_f_nn', [])[::3], color=COLORS['highlight'], 
                         s=10, alpha=0.7, marker='s', label=r'$\hat{w}_f$ (NN)')
            
            ax_wf.set_xlabel('Time (s)')
            ax_wf.set_ylabel('Tire Residual (N)')
            ax_wf.set_title('Front Tire Residual Estimation', pad=8)
            ax_wf.legend(loc='upper right', framealpha=0.95)
            subplot_idx += 1
            axes.append(ax_wf)

    # =========================================================================
    # 8. Acceleration
    # =========================================================================
    if is_active(['acceleration']):
        pos_ax = gs[0, 0] if plot_type == 'acceleration' else None
        pos_ay = gs[0, 1] if plot_type == 'acceleration' else None
        
        if pos_ax:
            ax = fig.add_subplot(pos_ax)
            ax.plot(time, data.get('ax_meas', []), color=COLORS['measured'], 
                    linestyle='-', linewidth=1.5, alpha=0.9, label='Measured')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(r'$a_x$ (m/s²)')
            ax.set_title('Longitudinal Acceleration', pad=8)
            ax.legend(loc='upper right', framealpha=0.95)
            subplot_idx += 1
            axes.append(ax)
            
        if pos_ay:
            ax = fig.add_subplot(pos_ay)
            ax.plot(time, data.get('ay_meas', []), color=COLORS['measured'], 
                    linestyle='-', linewidth=1.5, alpha=0.9, label='Measured')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(r'$a_y$ (m/s²)')
            ax.set_title('Lateral Acceleration', pad=8)
            ax.legend(loc='upper right', framealpha=0.95)
            subplot_idx += 1
            axes.append(ax)

    # =========================================================================
    # 9. Control Inputs
    # =========================================================================
    if is_active(['debug']):
        pos = gs[2, 1] if plot_type == 'all' else gs[1, 1]
        ax = fig.add_subplot(pos)
        
        ax.plot(time, data.get('steering', []), color=COLORS['estimated'], 
                linestyle='-', linewidth=1.5, label=r'Steering $\delta$ (rad)')
        ax.plot(time, data.get('throttle', []), color=COLORS['highlight'], 
                linestyle='-', linewidth=1.5, label='Throttle')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Control Input')
        ax.set_title('Control Inputs', pad=8)
        ax.legend(loc='upper right', framealpha=0.95)
        subplot_idx += 1
        axes.append(ax)

    # =========================================================================
    # 10. Tire Force Comparison (Debug view)
    # =========================================================================
    if is_active(['debug']) and ('Fyr_layer_1' in data or 'Fyr_true' in data):
        has_layer_1 = 'Fyr_layer_1' in data and np.any(data['Fyr_layer_1'])
        has_layer_2 = 'Fyr_layer_2' in data and np.any(data['Fyr_layer_2'])
        has_true = 'Fyr_true' in data and np.any(data['Fyr_true'])
        
        if has_layer_1 or has_layer_2 or has_true:
            if plot_type == 'all':
                pos_fyr = gs[4, 0]
                pos_fyf = gs[4, 1]
                pos_alpha_r = gs[5, 0]
                pos_alpha_f = gs[5, 1]
            else:
                pos_fyf = gs[1, 0]
                pos_alpha_f = gs[1, 2]
                pos_fyr = gs[2, 0]
                pos_alpha_r = gs[2, 2]
            
            # Rear Lateral Force
            ax_fyr = fig.add_subplot(pos_fyr)
            if has_true:
                ax_fyr.plot(time, data['Fyr_true'], color=COLORS['true'], 
                           linestyle='-', linewidth=1.8, alpha=0.9, label=r'$F_{yr}$ (true)')
            if has_layer_1:
                ax_fyr.plot(time, data['Fyr_layer_1'], color=COLORS['layer1'], 
                           linestyle='-.', linewidth=1.5, alpha=0.85, label='Layer 1')
            if has_layer_2:
                ax_fyr.scatter(time[::3], data['Fyr_layer_2'][::3], color=COLORS['estimated'], 
                              s=10, alpha=0.75, marker='o', label='Layer 2')
            ax_fyr.set_xlabel('Time (s)')
            ax_fyr.set_ylabel(r'$F_{yr}$ (N)')
            ax_fyr.set_title('Rear Lateral Force', pad=8)
            ax_fyr.legend(loc='best', framealpha=0.95)
            subplot_idx += 1
            axes.append(ax_fyr)
            
            # Front Lateral Force
            ax_fyf = fig.add_subplot(pos_fyf)
            if has_true:
                ax_fyf.plot(time, data['Fyf_true'], color=COLORS['true'], 
                           linestyle='-', linewidth=1.8, alpha=0.9, label=r'$F_{yf}$ (true)')
            if has_layer_1:
                ax_fyf.plot(time, data['Fyf_layer_1'], color=COLORS['layer1'], 
                           linestyle='-.', linewidth=1.5, alpha=0.85, label='Layer 1')
            if has_layer_2:
                ax_fyf.scatter(time[::3], data['Fyf_layer_2'][::3], color=COLORS['estimated'], 
                              s=10, alpha=0.75, marker='o', label='Layer 2')
            ax_fyf.set_xlabel('Time (s)')
            ax_fyf.set_ylabel(r'$F_{yf}$ (N)')
            ax_fyf.set_title('Front Lateral Force', pad=8)
            ax_fyf.legend(loc='best', framealpha=0.95)
            subplot_idx += 1
            axes.append(ax_fyf)
            
            # Rear Slip Angle
            ax_ar = fig.add_subplot(pos_alpha_r)
            if has_true and 'alpha_r' in data:
                ax_ar.plot(time, data['alpha_r'], color=COLORS['true'], 
                          linestyle='-', linewidth=1.8, alpha=0.9, label='True')
            if has_layer_1 and 'alpha_r_layer_1' in data:
                ax_ar.plot(time, data['alpha_r_layer_1'], color=COLORS['layer1'], 
                          linestyle='-.', linewidth=1.5, alpha=0.85, label='Layer 1')
            if has_layer_2 and 'alpha_r_layer_2' in data:
                ax_ar.scatter(time[::3], data['alpha_r_layer_2'][::3], color=COLORS['estimated'], 
                             s=10, alpha=0.75, marker='o', label='Layer 2')
            ax_ar.set_xlabel('Time (s)')
            ax_ar.set_ylabel(r'$\alpha_r$ (deg)')
            ax_ar.set_title('Rear Slip Angle', pad=8)
            ax_ar.legend(loc='best', framealpha=0.95)
            subplot_idx += 1
            axes.append(ax_ar)
            
            # Front Slip Angle
            ax_af = fig.add_subplot(pos_alpha_f)
            if has_true and 'alpha_f' in data:
                ax_af.plot(time, data['alpha_f'], color=COLORS['true'], 
                          linestyle='-', linewidth=1.8, alpha=0.9, label='True')
            if has_layer_1 and 'alpha_f_layer_1' in data:
                ax_af.plot(time, data['alpha_f_layer_1'], color=COLORS['layer1'], 
                          linestyle='-.', linewidth=1.5, alpha=0.85, label='Layer 1')
            if has_layer_2 and 'alpha_f_layer_2' in data:
                ax_af.scatter(time[::3], data['alpha_f_layer_2'][::3], color=COLORS['estimated'], 
                             s=10, alpha=0.75, marker='o', label='Layer 2')
            ax_af.set_xlabel('Time (s)')
            ax_af.set_ylabel(r'$\alpha_f$ (deg)')
            ax_af.set_title('Front Slip Angle', pad=8)
            ax_af.legend(loc='best', framealpha=0.95)
            subplot_idx += 1
            axes.append(ax_af)

    # =========================================================================
    # 11. Training Loss
    # =========================================================================
    if is_active(['debug']):
        if plot_type == 'debug':
            pos = gs[0, 2]
        elif plot_type == 'all':
            pos = gs[2, 2]
        else:
            pos = None
            
        if pos:
            ax = fig.add_subplot(pos)
            loss = data.get('loss', [])
            if len(loss) > 0:
                gps_valid = data.get('gps_valid', np.ones_like(loss))
                valid_mask = np.array(gps_valid, dtype=bool)
                if np.any(valid_mask):
                    ax.semilogy(time[valid_mask], np.abs(loss[valid_mask]) + 1e-10, 
                               color=COLORS['estimated'], linewidth=1.2, alpha=0.9)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Loss')
            ax.set_title('Training Loss (log scale)', pad=8)
            subplot_idx += 1
            axes.append(ax)

    # =========================================================================
    # 12. X-Y Trajectory
    # =========================================================================
    if is_active(['trajectory']):
        pos = gs[2, 0] if plot_type == 'all' else gs[0, 0]
        ax = fig.add_subplot(pos)
        
        X_est = data.get('X_est', [])
        Y_est = data.get('Y_est', [])
        X_meas = data.get('X_meas', [])
        Y_meas = data.get('Y_meas', [])
        
        if len(X_est) > 0:
            # GPS (background)
            ax.scatter(X_meas, Y_meas, c=COLORS['measured'], s=6, alpha=0.35, 
                       label='GPS', zorder=2, marker='o')
            
            # Layer 1
            if 'X_uio' in data and 'Y_uio' in data:
                ax.plot(data['X_uio'], data['Y_uio'], color=COLORS['layer1'], 
                       linestyle='-.', linewidth=1.5, alpha=0.7, label='Layer 1', zorder=3)
            
            # True trajectory
            if 'X_true' in data and 'Y_true' in data:
                ax.plot(data['X_true'], data['Y_true'], color=COLORS['true'], 
                       linestyle='--', linewidth=1.5, alpha=0.8, label='True', zorder=3)
            
            # Neural estimate (main)
            ax.plot(X_est, Y_est, color=COLORS['estimated'], linewidth=2.2, 
                    alpha=0.95, label='Neural', zorder=4)
            
            # Start/End
            ax.scatter(X_est[0], Y_est[0], c=COLORS['true'], s=120, marker='o', 
                       edgecolors='white', linewidths=2, label='Start', zorder=5)
            ax.scatter(X_est[-1], Y_est[-1], c=COLORS['highlight'], s=120, marker='s', 
                       edgecolors='white', linewidths=2, label='End', zorder=5)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Vehicle Trajectory', pad=8)
        ax.legend(loc='best', framealpha=0.95)
        ax.set_aspect('equal', adjustable='box')
        subplot_idx += 1
        axes.append(ax)

    # =========================================================================
    # 13. Position Error
    # =========================================================================
    if is_active(['errors']):
        pos = gs[3, 2] if plot_type == 'all' else gs[0, 0]
        ax = fig.add_subplot(pos)
        
        X_err = np.array(data.get('X_est', [])) - np.array(data.get('X_meas', []))
        Y_err = np.array(data.get('Y_est', [])) - np.array(data.get('Y_meas', []))
        
        if len(X_err) > 0 and len(Y_err) > 0:
            pos_err = np.sqrt(X_err**2 + Y_err**2) * 100  # cm
            ax.plot(time, pos_err, color=COLORS['estimated'], linewidth=1.5, alpha=0.9)
            mean_err = np.mean(pos_err)
            ax.axhline(y=mean_err, color=COLORS['highlight'], linestyle='--', 
                       linewidth=1.5, label=f'Mean: {mean_err:.2f} cm')
            ax.fill_between(time, 0, pos_err, color=COLORS['estimated'], alpha=0.15)
            ax.legend(loc='upper right', framealpha=0.95)
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position Error (cm)')
        ax.set_title('Position Estimation Error', pad=8)
        subplot_idx += 1
        axes.append(ax)

    # =========================================================================
    # 14. GPS Availability (errors view)
    # =========================================================================
    if is_active(['errors']):
        pos = gs[0, 1] if plot_type == 'errors' else None
        if plot_type == 'all':
            # In 'all' view, skip (already crowded)
            pos = None

        if pos:
            ax = fig.add_subplot(pos)
            gps_valid = data.get('gps_valid', [])

            if len(gps_valid) > 0:
                ax.fill_between(time, 0, gps_valid, color=COLORS['true'],
                               alpha=0.3, label='GPS Valid')
                ax.plot(time, gps_valid, color=COLORS['true'], linewidth=1.2)

            ax.set_xlabel('Time (s)')
            ax.set_ylabel('GPS Valid')
            ax.set_title('GPS Availability', pad=8)
            ax.set_ylim(-0.1, 1.1)
            ax.set_yticks([0, 1])
            ax.set_yticklabels(['Invalid', 'Valid'])
            subplot_idx += 1
            axes.append(ax)

    # Add interactivity
    make_plot_interactive(fig, axes)
    
    if save_path:
        if plot_type != 'all':
            p = Path(save_path)
            save_path = str(p.parent / f"{p.stem}_{plot_type}{p.suffix}")
        fig.savefig(save_path, dpi=300, bbox_inches='tight', facecolor='white')
        print(f"Figure saved to: {save_path}")
    
    plt.show()


def plot_tire_slip_article(data: Dict[str, np.ndarray],
                           title: str = "Tire Force and Slip Angle Estimation",
                           save_path: Optional[str] = None):
    """
    Create a focused publication-quality figure showing only:
    - Tire residual estimates (w_r, w_f)
    - Slip angles (alpha_r, alpha_f)
    
    Optimized for journal/conference article figures (2x2 layout).
    """
    setup_publication_style()
    
    time = data.get('time', np.arange(len(list(data.values())[0])))
    
    # Create 2x2 figure - optimal for single column journal width
    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    fig.suptitle(title, fontsize=13, fontweight='bold', y=0.98)
    
    labels = 'abcd'
    subplot_idx = 0
    
    # =========================================================================
    # Row 1: Tire Residual Estimates
    # =========================================================================
    
    # (a) Rear Tire Residual
    ax_wr = axes[0, 0]
    
    # True value (solid, reference)
    if 'w_r_true' in data and np.any(data['w_r_true']):
        ax_wr.plot(time, data['w_r_true'], color=COLORS['true'], 
                   linestyle='-', linewidth=2.0, alpha=0.9, label=r'$w_r$ (true)')
    
    # Layer 1 estimate (dashed)
    if 'w_r_uio' in data and np.any(data['w_r_uio']):
        ax_wr.plot(time, data['w_r_uio'], color=COLORS['layer1'], 
                   linestyle='--', linewidth=1.5, alpha=0.8, label=r'$\hat{w}_r$ (Layer 1)')
    elif 'w_r' in data and np.any(data['w_r']):
        ax_wr.plot(time, data['w_r'], color=COLORS['layer1'], 
                   linestyle='--', linewidth=1.5, alpha=0.8, label=r'$\hat{w}_r$ (Est)')
    
    # Neural estimate (markers, subsampled for clarity)
    if 'w_r_nn' in data and np.any(data['w_r_nn']):
        step = max(1, len(time) // 200)
        ax_wr.plot(time[::step], data['w_r_nn'][::step], color=COLORS['estimated'], 
                      alpha=1, label=r'$\hat{w}_r$ (NN)', zorder=3)
    
    ax_wr.set_xlabel('Time (s)')
    ax_wr.set_ylabel(r'$w_r$ (N)')
    ax_wr.set_title('Rear Tire Residual', pad=8)
    ax_wr.legend(loc='best', framealpha=0.95)
    add_subplot_label(ax_wr, labels[subplot_idx])
    subplot_idx += 1
    
    # (b) Front Tire Residual
    ax_wf = axes[0, 1]
    
    if 'w_f_true' in data and np.any(data['w_f_true']):
        ax_wf.plot(time, data['w_f_true'], color=COLORS['true'], 
                   linestyle='-', linewidth=2.0, alpha=0.9, label=r'$w_f$ (true)')
    
    if 'w_f_uio' in data and np.any(data['w_f_uio']):
        ax_wf.plot(time, data['w_f_uio'], color=COLORS['layer1'], 
                   linestyle='--', linewidth=1.5, alpha=0.8, label=r'$\hat{w}_f$ (Layer 1)')
    elif 'w_f' in data and np.any(data['w_f']):
        ax_wf.plot(time, data['w_f'], color=COLORS['layer1'], 
                   linestyle='--', linewidth=1.5, alpha=0.8, label=r'$\hat{w}_f$ (Est)')
    
    if 'w_f_nn' in data and np.any(data['w_f_nn']):
        step = max(1, len(time) // 200)
        ax_wf.plot(time[::step], data['w_f_nn'][::step], color=COLORS['estimated'], 
                      alpha=1, label=r'$\hat{w}_f$ (NN)', zorder=3)
    
    ax_wf.set_xlabel('Time (s)')
    ax_wf.set_ylabel(r'$w_f$ (N)')
    ax_wf.set_title('Front Tire Residual', pad=8)
    ax_wf.legend(loc='best', framealpha=0.95)
    add_subplot_label(ax_wf, labels[subplot_idx])
    subplot_idx += 1
    
    # =========================================================================
    # Row 2: Slip Angles
    # =========================================================================
    
    # (c) Rear Slip Angle
    ax_ar = axes[1, 0]
    
    # Check for 2-layer data first, then 1-layer
    if 'alpha_r' in data and np.any(data['alpha_r']):
        ax_ar.plot(time, np.rad2deg(data['alpha_r']), color=COLORS['true'], 
                   linestyle='-', linewidth=2.0, alpha=0.9, label=r'$\alpha_r$ (true)')
    
    if 'alpha_r_layer_1' in data and np.any(data['alpha_r_layer_1']):
        ax_ar.plot(time, np.rad2deg(data['alpha_r_layer_1']), color=COLORS['layer1'], 
                   linestyle='--', linewidth=1.5, alpha=0.8, label=r'$\hat{\alpha}_r$ (Layer 1)')
    elif 'alpha_r_est' in data and np.any(data['alpha_r_est']):
        ax_ar.plot(time, np.rad2deg(data['alpha_r_est']), color=COLORS['layer1'], 
                   linestyle='--', linewidth=1.5, alpha=0.8, label=r'$\hat{\alpha}_r$ (Est)')
    
    if 'alpha_r_layer_2' in data and np.any(data['alpha_r_layer_2']):
        step = max(1, len(time) // 200)
        ax_ar.plot(time[::step], np.rad2deg(data['alpha_r_layer_2'][::step]), 
                      color=COLORS['estimated'], alpha=1, 
                      label=r'$\hat{\alpha}_r$ (NN)', zorder=3)
    
    ax_ar.set_xlabel('Time (s)')
    ax_ar.set_ylabel(r'$\alpha_r$ (deg)')
    ax_ar.set_title('Rear Slip Angle', pad=8)
    ax_ar.legend(loc='best', framealpha=0.95)
    add_subplot_label(ax_ar, labels[subplot_idx])
    subplot_idx += 1
    
    # (d) Front Slip Angle
    ax_af = axes[1, 1]
    
    if 'alpha_f' in data and np.any(data['alpha_f']):
        ax_af.plot(time, np.rad2deg(data['alpha_f']), color=COLORS['true'], 
                   linestyle='-', linewidth=2.0, alpha=0.9, label=r'$\alpha_f$ (true)')
    
    if 'alpha_f_layer_1' in data and np.any(data['alpha_f_layer_1']):
        ax_af.plot(time, np.rad2deg(data['alpha_f_layer_1']), color=COLORS['layer1'], 
                   linestyle='--', linewidth=1.5, alpha=0.8, label=r'$\hat{\alpha}_f$ (Layer 1)')
    elif 'alpha_f_est' in data and np.any(data['alpha_f_est']):
        ax_af.plot(time, np.rad2deg(data['alpha_f_est']), color=COLORS['layer1'], 
                   linestyle='--', linewidth=1.5, alpha=0.8, label=r'$\hat{\alpha}_f$ (Est)')
    
    if 'alpha_f_layer_2' in data and np.any(data['alpha_f_layer_2']):
        step = max(1, len(time) // 200)
        ax_af.plot(time[::step], np.rad2deg(data['alpha_f_layer_2'][::step]), 
                      color=COLORS['estimated'], alpha=1, 
                      label=r'$\hat{\alpha}_f$ (NN)', zorder=3)
    
    ax_af.set_xlabel('Time (s)')
    ax_af.set_ylabel(r'$\alpha_f$ (deg)')
    ax_af.set_title('Front Slip Angle', pad=8)
    ax_af.legend(loc='best', framealpha=0.95)
    add_subplot_label(ax_af, labels[subplot_idx])
    
    # Adjust layout
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    
    if save_path:
        fig.savefig(save_path, dpi=300, bbox_inches='tight', facecolor='white')
        print(f"Figure saved to: {save_path}")
    
    plt.show()
    return fig


def plot_trajectory_control_article(data: Dict[str, np.ndarray],
                                    title: str = "Vehicle Trajectory and Control Inputs",
                                    save_path: Optional[str] = None):
    """
    Create a focused publication-quality figure showing:
    - XY Trajectory
    - Steering angle
    - Throttle input
    - Position estimation error
    
    Optimized for journal/conference article figures (2x2 layout).
    """
    setup_publication_style()
    
    time = data.get('time', np.arange(len(list(data.values())[0])))
    
    # Create 2x2 figure
    fig = plt.figure(figsize=(12, 9))
    gs = fig.add_gridspec(2, 2, hspace=0.30, wspace=0.28,
                          left=0.08, right=0.96, top=0.95, bottom=0.08)
    
    # =========================================================================
    # (top-left) XY Trajectory
    # =========================================================================
    ax_traj = fig.add_subplot(gs[0, 0])
    
    X_est = data.get('X_est', [])
    Y_est = data.get('Y_est', [])
    X_meas = data.get('X_meas', [])
    Y_meas = data.get('Y_meas', [])
    
    if len(X_est) > 0:
        # GPS measurements (background)
        ax_traj.scatter(X_meas, Y_meas, c=COLORS['measured'], s=8, alpha=0.4, 
                       label='GPS', zorder=2, marker='o')
        
        # # Layer 1 if available
        if 'X_uio' in data and 'Y_uio' in data:
            ax_traj.plot(data['X_uio'], data['Y_uio'], color=COLORS['layer1'], 
                        linestyle='-.', linewidth=1.5, alpha=0.7, label='Layer 1', zorder=3)
        
        # True trajectory if available
        # if 'X_true' in data and 'Y_true' in data and np.any(data['X_true']):
        #     ax_traj.plot(data['X_true'], data['Y_true'], color=COLORS['true'], 
        #                 linestyle='--', linewidth=1.8, alpha=0.85, label='True', zorder=3)
        
        # Neural/Final estimate (main)
        ax_traj.plot(X_est, Y_est, color=COLORS['estimated'], linewidth=1.8, 
                    alpha=0.95, label='Estimated', zorder=4)
        
        # Start/End markers
        ax_traj.scatter(X_est[0], Y_est[0], c=COLORS['true'], s=120, marker='o', 
                       edgecolors='white', linewidths=2, label='Start', zorder=5)
        ax_traj.scatter(X_est[-1], Y_est[-1], c=COLORS['highlight'], s=120, marker='s', 
                       edgecolors='white', linewidths=2, label='End', zorder=5)
    
    ax_traj.set_xlabel('X (m)')
    ax_traj.set_ylabel('Y (m)')
    ax_traj.set_title('Vehicle Trajectory', pad=8)
    ax_traj.legend(loc='best', framealpha=0.95, fontsize=8)
    ax_traj.set_aspect('equal', adjustable='box')
    
    # =========================================================================
    # (top-right) Position Estimation Error
    # =========================================================================
    ax_err = fig.add_subplot(gs[0, 1])
    
    X_err = np.array(data.get('X_est', [])) - np.array(data.get('X_meas', []))
    Y_err = np.array(data.get('Y_est', [])) - np.array(data.get('Y_meas', []))
    
    if len(X_err) > 0 and len(Y_err) > 0:
        pos_err = np.sqrt(X_err**2 + Y_err**2) * 100  # Convert to cm
        ax_err.plot(time, pos_err, color=COLORS['estimated'], linewidth=1.5, alpha=0.9)
        mean_err = np.mean(pos_err)
        ax_err.axhline(y=mean_err, color=COLORS['highlight'], linestyle='--', 
                       linewidth=1.5, label=f'Mean: {mean_err:.2f} cm')
        ax_err.fill_between(time, 0, pos_err, color=COLORS['estimated'], alpha=0.15)
        ax_err.legend(loc='upper right', framealpha=0.95)
    
    ax_err.set_xlabel('Time (s)')
    ax_err.set_ylabel('Position Error (cm)')
    ax_err.set_title('Position Estimation Error', pad=8)
    
    # =========================================================================
    # (bottom-left) Steering Angle
    # =========================================================================
    ax_steer = fig.add_subplot(gs[1, 0])
    
    steering = data.get('steering', [])
    if len(steering) > 0:
        ax_steer.plot(time, np.rad2deg(steering), color=COLORS['estimated'], 
                     linestyle='-', linewidth=1.8, alpha=0.95)
        ax_steer.fill_between(time, 0, np.rad2deg(steering), color=COLORS['estimated'], alpha=0.15)
    
    ax_steer.set_xlabel('Time (s)')
    ax_steer.set_ylabel(r'$\delta$ (deg)')
    ax_steer.set_title('Steering Angle', pad=8)
    ax_steer.axhline(y=0, color='gray', linestyle='--', linewidth=0.8, alpha=0.5)
    
    # =========================================================================
    # (bottom-right) Throttle Input
    # =========================================================================
    ax_throttle = fig.add_subplot(gs[1, 1])
    
    throttle = data.get('throttle', [])
    if len(throttle) > 0:
        ax_throttle.plot(time, throttle, color=COLORS['highlight'], 
                        linestyle='-', linewidth=1.8, alpha=0.95, label='Throttle')
        ax_throttle.fill_between(time, 0, throttle, color=COLORS['highlight'], alpha=0.15)

    # If this is 2-layer data and training loss is available, show loss (log scale)
    loss = data.get('loss', None)
    ax_loss = None
    if loss is not None and len(loss) > 0:
        # plot loss on a secondary y-axis (semilog)
        ax_loss = ax_throttle.twinx()
        # prefer GPS-valid mask if present
        gps_valid = data.get('gps_valid', np.ones_like(loss))
        valid_mask = np.array(gps_valid, dtype=bool)
        if np.any(valid_mask):
            x_loss = np.array(time)[valid_mask]
            y_loss = np.abs(np.array(loss)[valid_mask]) + 1e-10
        else:
            x_loss = np.array(time)
            y_loss = np.abs(np.array(loss)) + 1e-10

        # subsample for readability
        step = max(1, len(x_loss) // 300)
        ax_loss.semilogy(x_loss[::step], y_loss[::step], color=COLORS['estimated'],
                         linewidth=1.2, alpha=0.9, label='Loss')
        ax_loss.set_ylabel('Loss', color=COLORS['estimated'])
        ax_loss.tick_params(axis='y', colors=COLORS['estimated'])

    ax_throttle.set_xlabel('Time (s)')
    ax_throttle.set_ylabel('Throttle')
    ax_throttle.set_title('Throttle Input' + (" / Loss" if ax_loss is not None else ""), pad=8)
    ax_throttle.axhline(y=0, color='gray', linestyle='--', linewidth=0.8, alpha=0.5)

    # unified legend when loss axis present
    if ax_loss is not None:
        h1, l1 = ax_throttle.get_legend_handles_labels()
        h2, l2 = ax_loss.get_legend_handles_labels()
        ax_throttle.legend(h1 + h2, l1 + l2, loc='upper right', framealpha=0.95)
    else:
        ax_throttle.legend(loc='upper right', framealpha=0.95)
    
    if save_path:
        fig.savefig(save_path, dpi=300, bbox_inches='tight', facecolor='white')
        print(f"Figure saved to: {save_path}")
    
    plt.show()
    return fig


def plot_training_loss_article(data: Dict[str, np.ndarray],
                               title: str = "Training Loss",
                               save_path: Optional[str] = None):
    """
    Create a dedicated publication-quality figure for the neural-observer
    training loss (log scale).

    Layout: single axes with semilog-y loss curve.
    GPS-available regions are highlighted; GPS-denied regions are shaded.
    """
    setup_publication_style()

    time = data.get('time', np.arange(len(list(data.values())[0])))
    loss = data.get('loss', None)

    if loss is None or len(loss) == 0:
        print("No training-loss data found in this recording.")
        return None

    loss = np.array(loss, dtype=float)
    time = np.array(time, dtype=float)

    fig, ax = plt.subplots(figsize=(8, 4.5))

    # GPS valid mask ----------------------------------------------------------
    gps_valid = data.get('gps_valid', np.ones_like(loss))
    valid_mask = np.array(gps_valid, dtype=bool)

    # Shade GPS-denied regions
    denied = ~valid_mask
    if np.any(denied):
        diff = np.diff(denied.astype(int))
        starts = np.where(diff == 1)[0] + 1
        ends   = np.where(diff == -1)[0] + 1
        if denied[0]:
            starts = np.concatenate(([0], starts))
        if denied[-1]:
            ends = np.concatenate((ends, [len(denied)]))
        for s, e in zip(starts, ends):
            ax.axvspan(time[s], time[min(e, len(time)-1)],
                       color='gray', alpha=0.12, zorder=0)
        # add a single legend proxy for the shaded region
        ax.fill_between([], [], [], color='gray', alpha=0.12, label='GPS denied')

    # Plot loss (semilog) -----------------------------------------------------
    abs_loss = np.abs(loss) + 1e-10  # avoid log(0)

    if np.any(valid_mask):
        ax.semilogy(time[valid_mask], abs_loss[valid_mask],
                    color=COLORS['estimated'], linewidth=1.5, alpha=0.95,
                    label='Loss (GPS avail.)')
    if np.any(denied):
        ax.semilogy(time[denied], abs_loss[denied],
                    color=COLORS['highlight'], linewidth=1.2, alpha=0.8,
                    linestyle='--', label='Loss (GPS denied)')

    # Statistics annotation ---------------------------------------------------
    if np.any(valid_mask):
        mean_loss = np.mean(abs_loss[valid_mask])
        final_loss = abs_loss[valid_mask][-1]
        ax.axhline(y=mean_loss, color=COLORS['measured'], linestyle=':',
                   linewidth=1.2, alpha=0.7,
                   label=f'Mean: {mean_loss:.2e}')
        ax.annotate(f'Final: {final_loss:.2e}',
                    xy=(time[valid_mask][-1], final_loss),
                    xytext=(-60, 15), textcoords='offset points',
                    fontsize=8, color=COLORS['estimated'],
                    arrowprops=dict(arrowstyle='->', color=COLORS['estimated'],
                                    lw=1.0))

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Loss (log scale)')
    ax.set_title(title, pad=10)
    ax.legend(loc='upper right', framealpha=0.95, fontsize=8)
    ax.grid(True, which='both', alpha=0.3)

    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=300, bbox_inches='tight', facecolor='white')
        print(f"Figure saved to: {save_path}")

    plt.show()
    return fig


def plot_neural_obs_data(data: Dict[str, np.ndarray], 
                          title: str = "Neural Observer Analysis",
                          save_path: Optional[str] = None,
                          plot_type: PlotType = 'all',
                          dist_mode: Optional[str] = None):
    """
    Auto-detect mode and create appropriate publication-quality plots.
    
    Args:
        data: Dictionary of recorded data columns
        title: Figure title
        save_path: Optional path to save figure (PNG/PDF/SVG recommended)
        plot_type: Type of plot ('all', 'trajectory', 'states', 'errors', 'debug', 'acceleration', 'tire', 'control')
        dist_mode: Disturbance mode ('tire', 'general', or None for auto)
    """
    # Handle dedicated tire plot type
    if plot_type == 'tire':
        plot_tire_slip_article(data, title="Tire Force and Slip Angle Estimation", save_path=save_path)
        return
    
    # Handle dedicated trajectory + control plot type
    if plot_type == 'control':
        plot_trajectory_control_article(data, title="Vehicle Trajectory and Control Inputs", save_path=save_path)
        return
    
    # Handle dedicated training loss plot
    if plot_type == 'loss':
        plot_training_loss_article(data, title="Training Loss", save_path=save_path)
        return
    
    mode = detect_recording_mode(data)
    
    if mode == '1layer':
        plot_1layer_data(data, title=title, save_path=save_path, plot_type=plot_type, dist_mode=dist_mode)
    else:
        plot_2layer_data(data, title=title, save_path=save_path, plot_type=plot_type, dist_mode=dist_mode)


def main():
    """Main entry point for the plotting tool."""
    parser = argparse.ArgumentParser(description='Plot neural observer recorded data')
    parser.add_argument('file', nargs='?', help='Path to CSV recording file')
    parser.add_argument('--save', '-s', help='Path to save the figure')
    parser.add_argument('--dir', '-d', 
                        help='Directory to search for recordings (default: search all relevant dirs)')
    parser.add_argument('--mode', '-m', choices=['1layer', '2layer'],
                        help='Filter recordings by mode')
    parser.add_argument('--type', '-t', choices=['all', 'trajectory', 'states', 'errors', 'debug', 'acceleration', 'tire', 'control', 'loss'],
                        help='Plot type to show (tire = Tire & Slip Angles, control = Trajectory & Control, loss = Training Loss)')
    parser.add_argument('--list', '-l', action='store_true',
                        help='List available recordings and exit')
    parser.add_argument('--dist-mode', choices=['tire', 'general', 'auto'], default='auto',
                        help='Select disturbance plotting mode (tire=2D residuals, general=3D disturbances)')
    
    args = parser.parse_args()
    
    # List recordings if requested
    if args.list:
        # Use defaults if dir not specified
        dirs = args.dir if args.dir else "neural_obs_recordings"
        if not args.dir:
             # If no dir arg, we use our fancy multi-dir finder
             dirs = [
                 "neural_obs_recordings",
                 "test_recordings",
                 "2LayerObs/test_recordings",
                 "1LayerObs/integration_test_recordings"
             ]
             
        files = find_recordings(dirs, recursive=True)
        print(f"\nAll recordings found:")
        for f in files:
             mode = '2layer' if '_2layer_' in f.name else '1layer' if '_1layer_' in f.name else '?'
             print(f"- {f.parent.name}/{f.name} ({mode})")
        sys.exit(0)
    
    # Determine file path
    if args.file:
        filepath = args.file
    else:
        # Use interactive selection
        # Define default directories
        default_dirs = [
             "neural_obs_recordings",
             "test_recordings",
             "2LayerObs/test_recordings",
             "1LayerObs/integration_test_recordings"
        ]
        
        # If user passed a dir, use that as the only option (unless they want to select from it?)
        # If user passed --dir, assume they want to list files in that dir directly
        if args.dir:
            selected_dir = args.dir
        else:
            # 1. Select Directory
            selected_dir = select_directory_interactive(default_dirs)
            
        if not selected_dir:
            sys.exit(0)
            
        # 2. Select File
        filepath = select_recording_interactive(selected_dir, mode_filter=args.mode)
        
        if filepath is None:
            sys.exit(0)
            
        print(f"\nSelected: {filepath}")
    
    if not os.path.exists(filepath):
        print(f"File not found: {filepath}")
        sys.exit(1)
        
    # Select plot type if not specified
    if args.type:
        plot_type = args.type
    else:
        # If user provided file arg, maybe they want all by default?
        # But if they selected interactively, maybe ask them?
        # Let's ask if it was interactive selection or if explicitly requested?
        # A good UI choice: If user didn't specify --type, ask them.
        plot_type = select_plot_type_interactive()
    
    # Load and plot data
    print(f"Loading data from: {filepath}")
    data = load_data(filepath)
    print(f"Loaded {len(data.get('time', []))} samples")
    
    mode = detect_recording_mode(data)
    print(f"Detected recording mode: {mode}")
    print(f"Plot type: {plot_type}")
    
    title = f"Observer - {Path(filepath).stem}"
    dist_mode = args.dist_mode if args.dist_mode != 'auto' else None
    plot_neural_obs_data(data, title=title, save_path=args.save, plot_type=plot_type, dist_mode=dist_mode)


if __name__ == '__main__':
    main()
