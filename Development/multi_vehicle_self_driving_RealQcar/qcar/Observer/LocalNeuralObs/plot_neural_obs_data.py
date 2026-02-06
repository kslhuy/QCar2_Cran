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
PlotType = Literal['all', 'trajectory', 'states', 'errors', 'debug', 'acceleration']


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

    
    mapping = {
        '1': 'all',
        '2': 'trajectory',
        '3': 'states',
        '4': 'errors',
        '5': 'debug',
        '6': 'acceleration'
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
    
    # # 2. Hover Cursors
    # if MPLCURSORS_AVAILABLE:
    #     cursor = mplcursors.cursor(hover=True)
        
    #     @cursor.connect("add")
    #     def on_add(sel):
    #         # Customize tooltip text
    #         # sel.target is (x, y)
    #         # sel.artist.get_label() gives line label
    #         label = sel.artist.get_label()
    #         sel.annotation.set_text(f"{label}\nt={sel.target[0]:.2f}, y={sel.target[1]:.4f}")
            
    # else:
    #     print("\nTip: Install 'mplcursors' for hover-over values: pip install mplcursors")


def plot_1layer_data(data: Dict[str, np.ndarray], 
                     title: str = "1-Layer Observer Analysis",
                     save_path: Optional[str] = None,
                     plot_type: PlotType = 'all',
                     dist_mode: Optional[str] = None):
    """
    Create plots for 1-layer observer data.
    """
    time = data.get('time', np.arange(len(list(data.values())[0])))
    
    # Configure figure based on plot type
    if plot_type == 'all':
        fig = plt.figure(figsize=(16, 10))
        gs = fig.add_gridspec(3, 3, hspace=0.35, wspace=0.3)
    elif plot_type == 'trajectory':
        fig = plt.figure(figsize=(10, 8))
        gs = fig.add_gridspec(1, 1)
    elif plot_type == 'states':
        fig = plt.figure(figsize=(16, 6))
        gs = fig.add_gridspec(1, 3, wspace=0.3)
    elif plot_type == 'errors':
        fig = plt.figure(figsize=(12, 6))
        gs = fig.add_gridspec(1, 2, wspace=0.3)
    elif plot_type == 'debug':
        fig = plt.figure(figsize=(16, 6))
        gs = fig.add_gridspec(1, 3, wspace=0.3)
    elif plot_type == 'acceleration':
        fig = plt.figure(figsize=(12, 6))
        gs = fig.add_gridspec(1, 2, wspace=0.3)

    # Apply global small font for better visualization
    plt.rcParams.update({'font.size': 8, 'axes.titlesize': 9, 'axes.labelsize': 8, 'legend.fontsize': 7})
    
    fig.suptitle(title, fontsize=11, fontweight='bold')
    axes = []
    
    # =========================================================================
    # Plotting Logic
    # =========================================================================
    
    # helper to check if we should plot this component
    def is_active(categories):
        return plot_type == 'all' or plot_type in categories

    # 1. Longitudinal Velocity
    if is_active(['states']):
        pos = gs[0, 0] if plot_type == 'all' else gs[0, 0]
        ax = fig.add_subplot(pos)
        ax.plot(time, data.get('vx_est', []), 'b.', label='Estimated', markersize=2)
        ax.plot(time, data.get('vx_meas', []), 'r.', label='Measured', markersize=2)
        # ax.set_xlabel('Time [s]')
        ax.set_ylabel('$v_x$ [m/s]')
        ax.set_title('Longitudinal Velocity')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        axes.append(ax)

    # 2. Lateral Velocity
    if is_active(['states']):
        pos = gs[0, 1] if plot_type == 'all' else gs[0, 1]
        ax = fig.add_subplot(pos)
        ax.plot(time, data.get('vy_est', []), 'b.', label='Estimated', markersize=2)
        # ax.set_xlabel('Time [s]')
        ax.set_ylabel('$v_y$ [m/s]')
        ax.set_title('Lateral Velocity (Estimated)')
        ax.grid(True, alpha=0.3)
        axes.append(ax)

    # 3. Yaw Rate
    if is_active(['states']):
        pos = gs[0, 2] if plot_type == 'all' else gs[0, 2]
        ax = fig.add_subplot(pos)
        ax.plot(time, data.get('r_est', []), 'b.', label='Estimated', markersize=2)
        ax.plot(time, data.get('r_meas', []), 'r.', label='Measured', markersize=2)
        # ax.set_xlabel('Time [s]')
        ax.set_ylabel('$r$ [rad/s]')
        ax.set_title('Yaw Rate')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        axes.append(ax)

    # 4. X-Y Trajectory (Swapped with Tire Residuals, now smaller)
    if is_active(['trajectory']):
        pos = gs[1, 2] if plot_type == 'all' else gs[0, 0]
        ax = fig.add_subplot(pos)
        X_est = data.get('X_est', [])
        Y_est = data.get('Y_est', [])
        X_meas = data.get('X_meas', [])
        Y_meas = data.get('Y_meas', [])
        
        if len(X_est) > 0:
            ax.plot(X_est, Y_est, 'b-', label='Estimated', linewidth=2)
            ax.plot(X_meas, Y_meas, 'r.', label='GPS', markersize=3, alpha=0.7)
            ax.plot(X_est[0], Y_est[0], 'go', markersize=10, label='Start')
            ax.plot(X_est[-1], Y_est[-1], 'rs', markersize=10, label='End')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_title('X-Y Trajectory')
        ax.legend(loc='best', fontsize=8)
        ax.axis('equal')
        ax.grid(True, alpha=0.3)
        axes.append(ax)

    # 5. Yaw Angle (Only in All view for now to save space in states view)
    if is_active([]): # Skip for dedicated views unless added
        pass
    if plot_type == 'all':
        ax = fig.add_subplot(gs[1, 1])
        psi_est = data.get('psi_est', [])
        psi_meas = data.get('psi_meas', [])
        if len(psi_est) > 0:
            ax.plot(time, np.rad2deg(psi_est), 'b.', label='Estimated', markersize=2)
            ax.plot(time, np.rad2deg(psi_meas), 'r.', label='GPS', markersize=2)
        # ax.set_xlabel('Time [s]')
        ax.set_ylabel('$\\psi$ [deg]')
        ax.set_title('Yaw Angle')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        axes.append(ax)

    # 6. Tire Residuals / General Disturbances
    if is_active(['debug']):
        pos = gs[1, 0:2] if plot_type == 'all' else gs[0, 0]
        ax = fig.add_subplot(pos)
        
        # Determine disturbance mode
        use_general = False
        if dist_mode == 'general':
            use_general = True
        elif dist_mode == 'tire':
            use_general = False
        else:
            # Auto-detect
            use_general = 'd_vx' in data

        if use_general:
            # 3D General Disturbances
            ax.plot(time, data.get('d_vx', []), 'b-', label='$d_{vx}$ (Est)', linewidth=1)
            ax.plot(time, data.get('d_vy', []), 'g-', label='$d_{vy}$ (Est)', linewidth=1)
            ax.plot(time, data.get('d_r', []), 'm-', label='$d_{r}$ (Est)', linewidth=1)
            
            # True values if available
            if 'd_vx_true' in data:
                ax.plot(time, data['d_vx_true'], 'b:', label='$d_{vx}$ (True)', linewidth=1, alpha=0.6)
            if 'd_vy_true' in data:
                ax.plot(time, data['d_vy_true'], 'g:', label='$d_{vy}$ (True)', linewidth=1, alpha=0.6)
            if 'd_r_true' in data:
                ax.plot(time, data['d_r_true'], 'm:', label='$d_{r}$ (True)', linewidth=1, alpha=0.6)
                
            ax.set_ylabel('Disturbance')
            ax.set_title('General Disturbance Estimates')
        else:
            # 2D Tire Residuals (Legacy/Tire Mode)
            ax.plot(time, data.get('w_r', []), 'b.', label='$w_r$ (est)', markersize=2)
            ax.plot(time, data.get('w_f', []), 'r.', label='$w_f$ (est)', markersize=2)
            
            # Plot true values if available
            if 'w_r_true' in data and np.any(data['w_r_true']):
                 ax.plot(time, data['w_r_true'], 'b.', label='$w_r$ (true)', markersize=1)
            if 'w_f_true' in data and np.any(data['w_f_true']):
                 ax.plot(time, data['w_f_true'], 'r.', label='$w_f$ (true)', markersize=1)
                 
            ax.set_ylabel('Tire Residual [N]')
            ax.set_title('Unknown Input Estimates (Tire Residuals)')
            
        ax.set_xlabel('Time [s]')
        ax.legend(loc='upper right', fontsize=8)
        # ax.grid(True, alpha=0.3)
        axes.append(ax)

    # 7. Control Inputs (Simplified: no twin axis)
    if is_active(['debug']):
        pos = gs[2, 0] if plot_type == 'all' else gs[0, 1]
        ax = fig.add_subplot(pos)
        ax.plot(time, data.get('steering', []), 'b.', label='Steering [rad]', markersize=2)
        ax.plot(time, data.get('throttle', []), 'r.', label='Throttle', markersize=2)
        ax.set_title('Control Inputs')
        ax.legend(loc='upper right', fontsize=7)
        ax.grid(True, alpha=0.3)
        axes.append(ax)

    # 8. Position Error
    if is_active(['errors']):
        pos = gs[2, 1] if plot_type == 'all' else gs[0, 0]
        ax = fig.add_subplot(pos)
        X_err = np.array(data.get('X_est', [])) - np.array(data.get('X_meas', []))
        Y_err = np.array(data.get('Y_est', [])) - np.array(data.get('Y_meas', []))
        if len(X_err) > 0 and len(Y_err) > 0:
            pos_err = np.sqrt(X_err**2 + Y_err**2)
            ax.plot(time, pos_err * 100, 'b.', markersize=2) 
            ax.axhline(y=np.mean(pos_err) * 100, color='r', linestyle='--', 
                        label=f'Mean: {np.mean(pos_err)*100:.1f} cm')
            ax.legend(loc='upper right', fontsize=8)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Position Error [cm]')
        ax.set_title('Position Estimation Error')
        ax.grid(True, alpha=0.3)
        axes.append(ax)

    # 9. GPS Valid
    if is_active(['errors', 'debug']):
        if plot_type == 'all': pos = gs[2, 2]
        elif plot_type == 'errors': pos = gs[0, 1]
        elif plot_type == 'debug': pos = gs[0, 2]
        else: pos = None
        
        if pos:
            ax = fig.add_subplot(pos)
            gps_valid = data.get('gps_valid', [])
            if len(gps_valid) > 0:
                ax.fill_between(time, 0, gps_valid, alpha=0.3, color='green', label='GPS Valid')
                ax.plot(time, gps_valid, 'g-', linewidth=1)
            ax.set_xlabel('Time [s]')
            ax.set_ylabel('GPS Valid')
            ax.set_title('GPS Availability')
            ax.set_ylim(-0.1, 1.1)
            ax.grid(True, alpha=0.3)
            ax.grid(True, alpha=0.3)
            axes.append(ax)

    # 10. Acceleration (New)
    if is_active(['acceleration']):
        pos_ax = gs[0, 0] if plot_type == 'acceleration' else None
        pos_ay = gs[0, 1] if plot_type == 'acceleration' else None
        
        if pos_ax:
            ax = fig.add_subplot(pos_ax)
            ax.plot(time, data.get('ax_meas', []), 'r.', label='Measured', markersize=2)
            ax.set_ylabel('$a_x$ [m/s^2]')
            ax.set_title('Longitudinal Acceleration')
            ax.legend(loc='upper right', fontsize=8)
            ax.grid(True, alpha=0.3)
            axes.append(ax)
            
        if pos_ay:
            ax = fig.add_subplot(pos_ay)
            ax.plot(time, data.get('ay_meas', []), 'r.', label='Measured', markersize=2)
            ax.set_ylabel('$a_y$ [m/s^2]')
            ax.set_title('Lateral Acceleration')
            ax.legend(loc='upper right', fontsize=8)
            ax.grid(True, alpha=0.3)
            axes.append(ax)

    # Add interactivity
    make_plot_interactive(fig, axes)
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    if save_path:
        if plot_type != 'all':
            p = Path(save_path)
            save_path = str(p.parent / f"{p.stem}_{plot_type}{p.suffix}")
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Figure saved to: {save_path}")
    
    plt.show()




def plot_2layer_data(data: Dict[str, np.ndarray], 
                     title: str = "2-Layer Neural Observer Analysis",
                     save_path: Optional[str] = None,
                     plot_type: PlotType = 'all',
                     dist_mode: Optional[str] = None):
    """
    Create comprehensive plots for 2-layer neural observer data.
    """
    time = data.get('time', np.arange(len(list(data.values())[0])))
    
    # Configure figure based on plot type
    if plot_type == 'all':
        fig = plt.figure(figsize=(16, 18))
        gs = fig.add_gridspec(6, 3, hspace=0.4, wspace=0.3)
    elif plot_type == 'trajectory':
        fig = plt.figure(figsize=(10, 8))
        gs = fig.add_gridspec(1, 1)
    elif plot_type == 'states':
        fig = plt.figure(figsize=(16, 6))
        gs = fig.add_gridspec(1, 3, wspace=0.3)
    elif plot_type == 'errors':
        fig = plt.figure(figsize=(12, 6))
        gs = fig.add_gridspec(1, 2, wspace=0.3)
    elif plot_type == 'debug':
        fig = plt.figure(figsize=(16, 12)) # larger for 2 layer debug with tire forces
        gs = fig.add_gridspec(3, 3, hspace=0.35, wspace=0.3)
    elif plot_type == 'acceleration':
        fig = plt.figure(figsize=(12, 6))
        gs = fig.add_gridspec(1, 2, wspace=0.3)

    # Apply global small font for better visualization
    plt.rcParams.update({'font.size': 7, 'axes.titlesize': 8, 'axes.labelsize': 8, 'legend.fontsize': 6})
    
    fig.suptitle(title, fontsize=10, fontweight='bold')
    axes = []
    
    def is_active(categories):
        return plot_type == 'all' or plot_type in categories

    # =========================================================================
    # Plot Logic
    # =========================================================================
    
    # 1. Longitudinal Velocity
    if is_active(['states']):
        pos = gs[0, 0] if plot_type == 'all' else gs[0, 0]
        ax = fig.add_subplot(pos)
        # Plot Measured first (background)
        ax.plot(time, data.get('vx_meas', []), 'r.', label='Measured', markersize=1)
        # Plot 1st Layer
        if 'vx_uio' in data:
            ax.plot(time, data['vx_uio'], 'c.', label='1st Layer', markersize=1)
        # Plot Neural Estimate
        ax.plot(time, data.get('vx_est', []), 'b.', label='Neural', markersize=2)
        # Plot True (Foreground)
        if 'vx_true' in data and np.any(data['vx_true']):
             ax.plot(time, data['vx_true'], 'g--', label='True', linewidth=1.5)
        
        # ax.set_xlabel('Time [s]')
        ax.set_ylabel('$v_x$ [m/s]')
        ax.set_title('Longitudinal Velocity')
        ax.legend(loc='upper right', fontsize='small')
        ax.grid(True, alpha=0.3)
        axes.append(ax)

    # 2. Lateral Velocity
    if is_active(['states']):
        pos = gs[0, 1] if plot_type == 'all' else gs[0, 1]
        ax = fig.add_subplot(pos)
        if 'vy_uio' in data:
            ax.plot(time, data['vy_uio'], 'c.', label='1st Layer', markersize=1)
        ax.plot(time, data.get('vy_est', []), 'b.', label='Neural', markersize=2)
        if 'vy_true' in data and np.any(data['vy_true']):
            ax.plot(time, data['vy_true'], 'g--', label='True', linewidth=1.5)
        
        # ax.set_xlabel('Time [s]')
        ax.set_ylabel('$v_y$ [m/s]')
        ax.set_title('Lateral Velocity')
        ax.legend(loc='upper right', fontsize='small')
        ax.grid(True, alpha=0.3)
        axes.append(ax)

    # 3. Yaw Rate
    if is_active(['states']):
        pos = gs[0, 2] if plot_type == 'all' else gs[0, 2]
        ax = fig.add_subplot(pos)
        ax.plot(time, data.get('r_meas', []), 'r.', label='Measured', markersize=1)
        if 'r_uio' in data:
            ax.plot(time, data['r_uio'], 'c.', label='1st Layer', markersize=1)
        ax.plot(time, data.get('r_est', []), 'b.', label='Neural', markersize=2)
        if 'r_true' in data and np.any(data['r_true']):
            ax.plot(time, data['r_true'], 'g.', label='True', markersize=2)
        
        # ax.set_xlabel('Time [s]')
        ax.set_ylabel('$r$ [rad/s]')
        ax.set_title('Yaw Rate')
        ax.legend(loc='upper right', fontsize='small')
        ax.grid(True, alpha=0.3)
        axes.append(ax)

    # 4. Positions (X, Y, Yaw) -> Only in All view usually, or specific if requested
    if plot_type == 'all':
        # X Position
        ax4 = fig.add_subplot(gs[1, 0])
        # ax4.plot(time, data.get('X_meas', []), 'r.', label='GPS', markersize=1)
        if 'X_true' in data and np.any(data['X_true']):
            ax4.plot(time, data['X_true'], 'r.', label='True', markersize=2, alpha=0.6)
        if 'X_uio' in data:
            ax4.plot(time, data['X_uio'], 'c.', label='1st Layer', markersize=1)
        ax4.plot(time, data.get('X_est', []), 'b.', label='Neural', markersize=1)
        # ax4.set_title('X Position'); ax4.grid(True, alpha=0.3); axes.append(ax4)
        ax4.set_ylabel('$X$ [m]')
        ax4.legend(loc='upper right', fontsize='small')

        # Y Position
        ax5 = fig.add_subplot(gs[1, 1])
        # ax5.plot(time, data.get('Y_meas', []), 'r.', label='GPS', markersize=1)
        if 'Y_true' in data and np.any(data['Y_true']):
            ax5.plot(time, data['Y_true'], 'r.', label='True', markersize=2, alpha=0.6)
        if 'Y_uio' in data:
            ax5.plot(time, data['Y_uio'], 'c.', label='1st Layer', markersize=1)
        ax5.plot(time, data.get('Y_est', []), 'b.', label='Neural', markersize=1)
        # ax5.set_title('Y Position'); ax5.grid(True, alpha=0.3); axes.append(ax5)
        ax5.set_ylabel('$Y$ [m]')
        ax5.legend(loc='upper right', fontsize='small')

        # Yaw Angle
        ax6 = fig.add_subplot(gs[1, 2])
        psi_est = data.get('psi_est', []); 
        # psi_meas = data.get('psi_meas', [])
        if len(psi_est) > 0:
            # ax6.plot(time, np.rad2deg(psi_meas), 'r.', label='GPS', markersize=1)
            if 'psi_true' in data and np.any(data['psi_true']):
                ax6.plot(time, np.rad2deg(data['psi_true']), 'r.', label='True', markersize=2 , alpha = 0.6)
            if 'psi_uio' in data:
                ax6.plot(time, np.rad2deg(data['psi_uio']), 'c.', label='1st Layer', markersize=1)
            ax6.plot(time, np.rad2deg(psi_est), 'b.', label='Neural', markersize=1)
        # ax6.set_title('Yaw Angle'); ax6.grid(True, alpha=0.3); axes.append(ax6)
        ax6.set_ylabel('$yaw$ [deg]')
        ax6.legend(loc='upper right', fontsize='small')

    # 7. NN Tire Residuals / General Disturbances
    if is_active(['debug']):
        pos = gs[3, 0:2] if plot_type == 'all' else gs[0, 0]
        ax = fig.add_subplot(pos)
        
        # Determine disturbance mode
        use_general = False
        if dist_mode == 'general':
            use_general = True
        elif dist_mode == 'tire':
            use_general = False
        else:
            # Auto-detect
            use_general = 'd_vx_nn' in data

        if use_general:
            # 3D General Disturbances
            
            # True Disturbances (Solid lines)
            if 'd_vx_true' in data:
                ax.plot(time, data['d_vx_true'], 'b-', label='$d_{vx}$ (True)', linewidth=1.5, alpha=0.9)
            if 'd_vy_true' in data:
                ax.plot(time, data['d_vy_true'], 'g-', label='$d_{vy}$ (True)', linewidth=1.5, alpha=0.9)
            if 'd_r_true' in data:
                ax.plot(time, data['d_r_true'], 'r-', label='$d_{r}$ (True)', linewidth=1.5, alpha=0.9)
            
            # Neural Estimates (Dashed/Points)
            ax.plot(time, data.get('d_vx_nn', []), 'b.', label='$d_{vx}$ (NN)', markersize=3)
            ax.plot(time, data.get('d_vy_nn', []), 'g.', label='$d_{vy}$ (NN)', markersize=3)
            ax.plot(time, data.get('d_r_nn', []), 'r.', label='$d_{r}$ (NN)', markersize=3)
            
            # First Layer Estimates (Cyan/Yellow/Thin) - if available
            if 'd_vx_uio' in data:
                ax.plot(time, data['d_vx_uio'], 'b--', label='$d_{vx}$ (1st)', linewidth=2, alpha=0.75)
                ax.plot(time, data['d_vy_uio'], 'g--', label='$d_{vy}$ (1st)', linewidth=2, alpha=0.75)
                ax.plot(time, data['d_r_uio'], 'r--', label='$d_{r}$ (1st)', linewidth=2, alpha=0.75)
        
            ax.set_ylabel('Disturbance')
            ax.legend(loc='upper right', fontsize=8)

            ax.set_title('General Disturbance Estimates (NN vs True)')
        else:
            # 2D Tire Residuals (Legacy)
            if 'w_r_true' in data :
                ax.plot(time, data['w_r_true'], 'b-', label='$w_r$ (True)', linewidth=1.5 , alpha=0.9)
                ax.plot(time, data['w_f_true'], 'g-', label='$w_f$ (True)', linewidth=1.5 , alpha=0.9)

            if 'w_r_uio' in data:
                ax.plot(time, data.get('w_r_uio', []), 'b--', label='$w_r$ (1st)', linewidth=2, alpha=0.75)
                ax.plot(time, data.get('w_f_uio', []), 'g--', label='$w_f$ (1st)', linewidth=2, alpha=0.75)
                
            ax.plot(time, data.get('w_r_nn', []), 'b.', label='$w_r$ (NN)', markersize=3)
            ax.plot(time, data.get('w_f_nn', []), 'r.', label='$w_f$ (NN)', markersize=3)

            ax.legend(loc='upper right', fontsize=8)

    # 8. Acceleration (New - for acceleration view or all)
    if is_active(['acceleration']):
         pos_ax = gs[0, 0] if plot_type == 'acceleration' else None
         pos_ay = gs[0, 1] if plot_type == 'acceleration' else None
         
         if pos_ax:
            ax = fig.add_subplot(pos_ax)
            ax.plot(time, data.get('ax_meas', []), 'r.', label='Measured', markersize=2)
            ax.set_ylabel('$a_x$ [m/s^2]')
            ax.set_title('Longitudinal Acceleration')
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
            axes.append(ax)
            
         if pos_ay:
            ax = fig.add_subplot(pos_ay)
            ax.plot(time, data.get('ay_meas', []), 'r.', label='Measured', markersize=2)
            ax.set_ylabel('$a_y$ [m/s^2]')
            ax.set_title('Lateral Acceleration')
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
            axes.append(ax)


    # 8. Control Inputs (Simplified: no twin axis)
    if is_active(['debug']):
        pos = gs[2, 1] if plot_type == 'all' else gs[0, 1]
        ax = fig.add_subplot(pos)
        ax.plot(time, data.get('steering', []), 'b.', label='Steering [rad]', markersize=2)
        ax.plot(time, data.get('throttle', []), 'r.', label='Throttle', markersize=2)
        ax.set_title('Control Inputs')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        axes.append(ax)

    # NEW: Tire Force Comparison Plots (Only in debug view and tire mode)
    if is_active(['debug']) and 'Fyr_layer_1' in data:
        # Check if we have layer data
        has_layer_1 = 'Fyr_layer_1' in data and np.any(data['Fyr_layer_1'])
        has_layer_2 = 'Fyr_layer_2' in data and np.any(data['Fyr_layer_2'])
        has_true = 'Fyr_true' in data and np.any(data['Fyr_true'])
        
        if has_layer_1 or has_layer_2 or has_true:
            # Determine grid positions based on plot type
            if plot_type == 'all':
                # In 'all' view, use dedicated new rows (4 and 5) to avoid overlay
                pos_fyr = gs[4, 0]
                pos_fyf = gs[4, 1]  
                pos_alpha_r = gs[5, 0]
                pos_alpha_f = gs[5, 1] 
            else:
                # In debug-only view, use the pairing layout
                # Row 0: Tire residuals, Control inputs, Loss
                # Row 1: Front Axle (Fyf, alpha_f)
                # Row 2: Rear Axle (Fyr, alpha_r)
                pos_fyf = gs[1, 0]
                pos_alpha_f = gs[1, 1]
                pos_fyr = gs[2, 0]
                pos_alpha_r = gs[2, 1]
            
            # Plot 1: Rear Lateral Force (Fyr)
            ax_fyr = fig.add_subplot(pos_fyr)
            if has_true:
                ax_fyr.plot(time, data['Fyr_true'], 'r-', label='True', linewidth=1.5, alpha=0.8)
            if has_layer_1:
                ax_fyr.plot(time, data['Fyr_layer_1'], 'c--', label='Layer 1', linewidth=1.5, alpha=0.7)
            if has_layer_2:
                ax_fyr.plot(time, data['Fyr_layer_2'], 'b.', label='Layer 2', markersize=2)
            ax_fyr.set_ylabel('$F_{yr}$ [N]')
            ax_fyr.set_title('Rear Lateral Force')
            ax_fyr.legend(loc='best', fontsize=7)
            ax_fyr.grid(True, alpha=0.3)
            axes.append(ax_fyr)
            
            # Plot 2: Front Lateral Force (Fyf)
            ax_fyf = fig.add_subplot(pos_fyf)
            if has_true:
                ax_fyf.plot(time, data['Fyf_true'], 'r-', label='True', linewidth=1.5, alpha=0.8)
            if has_layer_1:
                ax_fyf.plot(time, data['Fyf_layer_1'], 'c--', label='Layer 1', linewidth=1.5, alpha=0.7)
            if has_layer_2:
                ax_fyf.plot(time, data['Fyf_layer_2'], 'b.', label='Layer 2', markersize=2)
            ax_fyf.set_ylabel('$F_{yf}$ [N]')
            ax_fyf.set_title('Front Lateral Force')
            ax_fyf.legend(loc='best', fontsize=7)
            ax_fyf.grid(True, alpha=0.3)
            axes.append(ax_fyf)
            
            # Plot 3: Rear Slip Angle (alpha_r)
            ax_alpha_r = fig.add_subplot(pos_alpha_r)
            if has_true and 'alpha_r' in data:
                ax_alpha_r.plot(time, np.rad2deg(data['alpha_r']), 'r-', label='True', linewidth=1.5, alpha=0.8)
            if has_layer_1 and 'alpha_r_layer_1' in data:
                ax_alpha_r.plot(time, np.rad2deg(data['alpha_r_layer_1']), 'c--', label='Layer 1', linewidth=1.5, alpha=0.7)
            if has_layer_2 and 'alpha_r_layer_2' in data:
                ax_alpha_r.plot(time, np.rad2deg(data['alpha_r_layer_2']), 'b.', label='Layer 2', markersize=2)
            ax_alpha_r.set_ylabel(r'$\alpha_r$ [deg]')
            ax_alpha_r.set_title('Rear Slip Angle')
            ax_alpha_r.legend(loc='best', fontsize=7)
            ax_alpha_r.grid(True, alpha=0.3)
            axes.append(ax_alpha_r)
            
            # Plot 4: Front Slip Angle (alpha_f)
            ax_alpha_f = fig.add_subplot(pos_alpha_f)
            if has_true and 'alpha_f' in data:
                ax_alpha_f.plot(time, np.rad2deg(data['alpha_f']), 'r-', label='True', linewidth=1.5, alpha=0.8)
            if has_layer_1 and 'alpha_f_layer_1' in data:
                ax_alpha_f.plot(time, np.rad2deg(data['alpha_f_layer_1']), 'c--', label='Layer 1', linewidth=1.5, alpha=0.7)
            if has_layer_2 and 'alpha_f_layer_2' in data:
                ax_alpha_f.plot(time, np.rad2deg(data['alpha_f_layer_2']), 'b.', label='Layer 2', markersize=2)
            ax_alpha_f.set_ylabel(r'$\alpha_f$ [deg]')
            ax_alpha_f.set_title('Front Slip Angle')
            ax_alpha_f.legend(loc='best', fontsize=7)
            ax_alpha_f.grid(True, alpha=0.3)
            axes.append(ax_alpha_f)


    # 9. Training Loss
    if is_active(['debug']):
        if plot_type == 'debug':
            pos = gs[0, 2]
        elif plot_type == 'all':
            pos = gs[2, 2]
        else:
            pos = gs[1, 0]
        ax = fig.add_subplot(pos)
        loss = data.get('loss', [])
        if len(loss) > 0:
            gps_valid = data.get('gps_valid', np.ones_like(loss))
            valid_mask = np.array(gps_valid, dtype=bool)
            if np.any(valid_mask):
                ax.plot(time[valid_mask], np.abs(loss[valid_mask]) + 1e-10, 'b.', markersize=2)
                ax.set_yscale('log')
        # ax.set_xlabel('Time [s]')
        ax.set_ylabel('Loss')
        ax.set_title('Training Loss (log scale)')
        ax.grid(True, alpha=0.3)
        axes.append(ax)

    # 10. X-Y Trajectory (Swapped with Tire Residuals, now smaller)
    if is_active(['trajectory']):
        pos = gs[2, 0] if plot_type == 'all' else gs[0, 0]
        ax = fig.add_subplot(pos)
        X_est = data.get('X_est', [])
        Y_est = data.get('Y_est', [])
        X_meas = data.get('X_meas', [])
        Y_meas = data.get('Y_meas', [])
        
        if len(X_est) > 0:
            ax.plot(X_est, Y_est, 'b-', label='Neural Est.', linewidth=2)
            ax.plot(X_meas, Y_meas, 'r.', label='GPS Measured', markersize=3, alpha=0.7)
            
            if 'X_uio' in data and 'Y_uio' in data:
                ax.plot(data['X_uio'], data['Y_uio'], 'c:', label='1st Layer', linewidth=1.5)
            
            if 'X_true' in data and 'Y_true' in data:
                ax.plot(data['X_true'], data['Y_true'], 'g--', label='True', linewidth=1)
                
            ax.plot(X_est[0], Y_est[0], 'go', markersize=10, label='Start')
            ax.plot(X_est[-1], Y_est[-1], 'rs', markersize=10, label='End')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_title('X-Y Trajectory')
        ax.legend(loc='best')
        ax.axis('equal')
        ax.grid(True, alpha=0.3)
        axes.append(ax)

    # 11. Position Error
    if is_active(['errors']):
        pos = gs[3, 2] if plot_type == 'all' else gs[0, 0]
        ax = fig.add_subplot(pos)
        X_err = np.array(data.get('X_est', [])) - np.array(data.get('X_meas', []))
        Y_err = np.array(data.get('Y_est', [])) - np.array(data.get('Y_meas', []))
        if len(X_err) > 0 and len(Y_err) > 0:
            pos_err = np.sqrt(X_err**2 + Y_err**2)
            ax.plot(time, pos_err * 100, 'b.', markersize=2)  # Convert to cm
            ax.axhline(y=np.mean(pos_err) * 100, color='r', linestyle='--', 
                         label=f'Mean: {np.mean(pos_err)*100:.1f} cm')
            ax.legend(loc='upper right')
        # ax.set_xlabel('Time [s]')
        ax.set_ylabel('Position Error [cm]')
        ax.set_title('Position Estimation Error')
        ax.grid(True, alpha=0.3)
        axes.append(ax)

    # Add interactivity
    make_plot_interactive(fig, axes)
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    if save_path:
        if plot_type != 'all':
            p = Path(save_path)
            save_path = str(p.parent / f"{p.stem}_{plot_type}{p.suffix}")
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Figure saved to: {save_path}")
    
    plt.show()


def plot_neural_obs_data(data: Dict[str, np.ndarray], 
                          title: str = "Neural Observer Analysis",
                          save_path: Optional[str] = None,
                          plot_type: PlotType = 'all',
                          dist_mode: Optional[str] = None):
    """
    Auto-detect mode and create appropriate plots.
    """
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
    parser.add_argument('--type', '-t', choices=['all', 'trajectory', 'states', 'errors', 'debug'],
                        help='Plot type to show')
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
    
    title = f"{'1-Layer' if mode == '1layer' else '2-Layer Neural'} Observer - {Path(filepath).stem}"
    dist_mode = args.dist_mode if args.dist_mode != 'auto' else None
    plot_neural_obs_data(data, title=title, save_path=args.save, plot_type=plot_type, dist_mode=dist_mode)


if __name__ == '__main__':
    main()
