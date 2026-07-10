"""
Interactive Scope Data Plotter

This script provides an interactive interface for plotting scope recordings from
both fake (simulation) and real QCar vehicles. It supports:
- Multiple recording directories (fake vehicle vs real vehicle)
- Local and fleet data visualization
- Interactive file selection (latest, oldest, or browse all)
- Playback mode with estimation scopes visualization

Usage:
    python plot_scope_data.py                    # Interactive mode
    python plot_scope_data.py --type local       # Plot local data
    python plot_scope_data.py --type fleet       # Plot fleet data
    python plot_scope_data.py --file <path>      # Plot specific file
    python plot_scope_data.py --playback         # Playback mode
"""

import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
from datetime import datetime
from typing import List, Tuple, Optional, Dict

# =============================================================================
# Configuration
# =============================================================================

# Define recording directories for different vehicle types
RECORDING_PATHS = {
    "fake_vehicle": [
        os.path.join("GUI", "scope_recordings"),
        os.path.join("..", "GUI", "scope_recordings"),
        os.path.join("..", "..", "GUI", "scope_recordings"),
        "scope_recordings",
    ],
    "real_vehicle": [
        "scope_recordings",
        os.path.join("..", "scope_recordings"),
        os.path.join("..", "..", "scope_recordings"),
    ]
}

# Try to import playback capabilities
try:
    from Development.multi_vehicle_self_driving_RealQcar.qcar_refactor.refs.qcar_origin.Observer.estimation_scopes import (
        EstimationScopeManager,
        ScopeDataPlayer,
        MULTISCOPE_AVAILABLE
    )
    PLAYBACK_AVAILABLE = MULTISCOPE_AVAILABLE
except ImportError:
    try:
        # Try relative import
        from Development.multi_vehicle_self_driving_RealQcar.qcar_refactor.refs.qcar_origin.Observer.estimation_scopes import (
            EstimationScopeManager,
            ScopeDataPlayer,
            MULTISCOPE_AVAILABLE
        )
        PLAYBACK_AVAILABLE = MULTISCOPE_AVAILABLE
    except ImportError:
        PLAYBACK_AVAILABLE = False
        print("[plot_scope_data] Note: Playback mode not available (estimation_scopes not found)")


# =============================================================================
# Utility Functions
# =============================================================================

def find_recording_directories() -> Dict[str, List[str]]:
    """Find all available recording directories for each vehicle type."""
    found_dirs = {"fake_vehicle": [], "real_vehicle": []}
    
    for vehicle_type, paths in RECORDING_PATHS.items():
        for path in paths:
            if os.path.exists(path):
                local_dir = os.path.join(path, 'local')
                fleet_dir = os.path.join(path, 'fleet')
                
                if os.path.exists(local_dir) or os.path.exists(fleet_dir):
                    abs_path = os.path.abspath(path)
                    if abs_path not in found_dirs[vehicle_type]:
                        found_dirs[vehicle_type].append(abs_path)
    
    return found_dirs


def get_all_files(directory: str, data_type: str = None) -> List[Tuple[str, datetime, str]]:
    """
    Get all CSV files from a directory with their metadata.
    
    Args:
        directory: Base recording directory
        data_type: 'local', 'fleet', or None for both
        
    Returns:
        List of tuples: (filepath, modification_time, data_type)
    """
    files = []
    
    subdirs = []
    if data_type is None or data_type == 'local':
        subdirs.append(('local', os.path.join(directory, 'local')))
    if data_type is None or data_type == 'fleet':
        subdirs.append(('fleet', os.path.join(directory, 'fleet')))
    
    for dtype, subdir in subdirs:
        if os.path.exists(subdir):
            csv_files = glob.glob(os.path.join(subdir, "*.csv"))
            for f in csv_files:
                try:
                    mtime = datetime.fromtimestamp(os.path.getmtime(f))
                    files.append((f, mtime, dtype))
                except:
                    pass
    
    return files


def sort_files(files: List[Tuple[str, datetime, str]], 
               order: str = 'newest') -> List[Tuple[str, datetime, str]]:
    """Sort files by modification time."""
    reverse = (order == 'newest')
    return sorted(files, key=lambda x: x[1], reverse=reverse)


def get_file_info(filepath: str) -> dict:
    """Extract information from a recording file."""
    info = {
        'filepath': filepath,
        'filename': os.path.basename(filepath),
        'size_kb': os.path.getsize(filepath) / 1024,
        'modified': datetime.fromtimestamp(os.path.getmtime(filepath)),
    }
    
    # Determine type from path or filename
    if 'local' in filepath.lower():
        info['type'] = 'local'
    elif 'fleet' in filepath.lower():
        info['type'] = 'fleet'
    else:
        # Try to detect from content
        try:
            df = pd.read_csv(filepath, nrows=1)
            if 'consensus_error' in df.columns or any('fleet_' in c for c in df.columns):
                info['type'] = 'fleet'
            else:
                info['type'] = 'local'
        except:
            info['type'] = 'unknown'
    
    # Get row count
    try:
        info['rows'] = sum(1 for _ in open(filepath)) - 1  # Subtract header
    except:
        info['rows'] = 0
    
    return info


def get_latest_file(directory: str, pattern: str = "*.csv") -> Optional[str]:
    """Get the latest file in a directory matching the pattern (legacy support)."""
    files = glob.glob(os.path.join(directory, pattern))
    if not files:
        return None
    return max(files, key=os.path.getctime)


# =============================================================================
# Interactive Menu System
# =============================================================================

def clear_screen():
    """Clear the terminal screen."""
    os.system('cls' if os.name == 'nt' else 'clear')


def print_header(title: str):
    """Print a styled header."""
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)


def print_menu(options: List[str], title: str = "Options"):
    """Print a numbered menu."""
    print(f"\n{title}:")
    for i, opt in enumerate(options, 1):
        print(f"  {i}. {opt}")
    print(f"  0. Back / Exit")


def get_choice(max_choice: int, prompt: str = "Enter choice") -> int:
    """Get a numeric choice from user."""
    while True:
        try:
            choice = input(f"\n{prompt} (0-{max_choice}): ").strip()
            if choice == '':
                return 0
            choice = int(choice)
            if 0 <= choice <= max_choice:
                return choice
            print(f"Please enter a number between 0 and {max_choice}")
        except ValueError:
            print("Invalid input. Please enter a number.")


def interactive_file_browser(files: List[Tuple[str, datetime, str]], 
                             page_size: int = 10) -> Optional[str]:
    """
    Interactive file browser with pagination.
    
    Returns selected filepath or None if cancelled.
    """
    if not files:
        print("No files found.")
        return None
    
    current_page = 0
    total_pages = (len(files) - 1) // page_size + 1
    
    while True:
        clear_screen()
        print_header(f"File Browser (Page {current_page + 1}/{total_pages})")
        
        start_idx = current_page * page_size
        end_idx = min(start_idx + page_size, len(files))
        page_files = files[start_idx:end_idx]
        
        print(f"\n{'#':<4} {'Type':<6} {'Date':<20} {'Size':<10} {'Filename'}")
        print("-" * 80)
        
        for i, (filepath, mtime, dtype) in enumerate(page_files, start_idx + 1):
            filename = os.path.basename(filepath)
            date_str = mtime.strftime("%Y-%m-%d %H:%M:%S")
            size_kb = os.path.getsize(filepath) / 1024
            print(f"{i:<4} {dtype:<6} {date_str:<20} {size_kb:>7.1f}KB  {filename}")
        
        print("\n" + "-" * 80)
        print("Navigation: [N]ext page | [P]rev page | [S]ort | [F]ilter | [0] Back")
        print("Enter file number to select, or navigation key:")
        
        choice = input("> ").strip().lower()
        
        if choice == '0' or choice == 'q':
            return None
        elif choice == 'n' and current_page < total_pages - 1:
            current_page += 1
        elif choice == 'p' and current_page > 0:
            current_page -= 1
        elif choice == 's':
            print("\nSort by: [1] Newest first | [2] Oldest first | [3] Name")
            sort_choice = input("> ").strip()
            if sort_choice == '1':
                files = sort_files(files, 'newest')
            elif sort_choice == '2':
                files = sort_files(files, 'oldest')
            elif sort_choice == '3':
                files = sorted(files, key=lambda x: os.path.basename(x[0]))
            current_page = 0
        elif choice == 'f':
            print("\nFilter by type: [1] Local only | [2] Fleet only | [3] All")
            filter_choice = input("> ").strip()
            if filter_choice == '1':
                files = [f for f in files if f[2] == 'local']
            elif filter_choice == '2':
                files = [f for f in files if f[2] == 'fleet']
            current_page = 0
            total_pages = (len(files) - 1) // page_size + 1 if files else 1
        else:
            try:
                file_num = int(choice)
                if 1 <= file_num <= len(files):
                    return files[file_num - 1][0]
            except ValueError:
                pass


def select_vehicle_type(found_dirs: Dict[str, List[str]]) -> Optional[Tuple[str, str]]:
    """
    Interactive vehicle type and directory selection.
    
    Returns (vehicle_type, directory_path) or None if cancelled.
    """
    clear_screen()
    print_header("Select Vehicle Type & Recording Directory")
    
    options = []
    option_map = []
    
    for v_type in ['fake_vehicle', 'real_vehicle']:
        dirs = found_dirs.get(v_type, [])
        for d in dirs:
            display_name = f"{v_type.replace('_', ' ').title()}: {d}"
            options.append(display_name)
            option_map.append((v_type, d))
    
    if not options:
        print("\nNo recording directories found!")
        print("Expected locations:")
        for v_type, paths in RECORDING_PATHS.items():
            print(f"  {v_type}: {paths}")
        input("\nPress Enter to continue...")
        return None
    
    print_menu(options, "Available Recording Locations")
    choice = get_choice(len(options))
    
    if choice == 0:
        return None
    
    return option_map[choice - 1]


def select_data_type() -> Optional[str]:
    """Select data type to plot."""
    print_menu(['Local data only', 'Fleet data only', 'Both (side by side)'], 
               "Select Data Type")
    choice = get_choice(3)
    
    if choice == 0:
        return None
    elif choice == 1:
        return 'local'
    elif choice == 2:
        return 'fleet'
    else:
        return 'both'


def select_file_order() -> Optional[str]:
    """Select file ordering/selection method."""
    print_menu(['Latest file', 'Oldest file', 'Browse all files'], 
               "Select File")
    choice = get_choice(3)
    
    if choice == 0:
        return None
    elif choice == 1:
        return 'latest'
    elif choice == 2:
        return 'oldest'
    else:
        return 'browse'


def select_vehicles_to_plot(df: pd.DataFrame) -> Optional[List[int]]:
    """Interactive selection of vehicles to plot from fleet data."""
    fleet_cols = [c for c in df.columns if c.startswith('fleet_x_')]
    vehicle_indices = sorted([int(c.split('_')[-1]) for c in fleet_cols])
    
    if not vehicle_indices:
        return None
    
    print(f"\nAvailable vehicles: {vehicle_indices}")
    print("\nSelect vehicles to plot:")
    print("  1. All vehicles")
    print("  2. Specific vehicles")
    print("  0. Cancel")
    
    choice = get_choice(2)
    
    if choice == 0:
        return None
    elif choice == 1:
        return vehicle_indices
    else:
        v_input = input(f"Enter vehicle indices (comma-separated, available: {vehicle_indices}): ")
        try:
            selected = [int(x.strip()) for x in v_input.split(',')]
            # Validate
            selected = [v for v in selected if v in vehicle_indices]
            return selected if selected else vehicle_indices
        except:
            return vehicle_indices


# =============================================================================
# Interactive Data Viewer with Time Slider
# =============================================================================

class InteractiveDataViewer:
    """
    Interactive viewer for recorded scope data with time slider navigation.
    
    Features:
    - Time slider to navigate through full recording
    - Window size control (how many seconds visible at once)
    - Play/Pause with animation
    - Speed control (0.5x to 4x)
    - Auto Y-axis scaling
    - Data decimation for performance
    """
    
    MAX_DISPLAY_POINTS = 2000  # Max points to display for performance
    
    def __init__(self, filepath: str, data_type: str = 'local'):
        """
        Initialize the interactive viewer.
        
        Args:
            filepath: Path to CSV file
            data_type: 'local' or 'fleet'
        """
        self.filepath = filepath
        self.data_type = data_type
        self.df = None
        self.times = None
        self.duration = 0.0
        
        # Playback state
        self.playing = False
        self.speed = 1.0
        self.current_time = 0.0
        self.window_size = 30.0  # seconds visible
        
        # Animation
        self.animation = None
        self.last_update_time = 0.0
        
        # UI elements
        self.fig = None
        self.axes = {}
        self.lines = {}
        self.slider_time = None
        self.slider_window = None
        self.slider_speed = None
        self.btn_play = None
        self.text_info = None
        
    def load_data(self) -> bool:
        """Load data from CSV file."""
        try:
            self.df = pd.read_csv(self.filepath)
            if 'time' not in self.df.columns:
                print("Error: 'time' column not found")
                return False
            
            # Normalize time to start from 0
            t_start = self.df['time'].iloc[0]
            self.times = (self.df['time'] - t_start).values
            self.duration = self.times[-1]
            
            print(f"Loaded: {len(self.df)} samples, {self.duration:.2f}s duration")
            return True
        except Exception as e:
            print(f"Error loading file: {e}")
            return False
    
    def _decimate_data(self, start_idx: int, end_idx: int) -> np.ndarray:
        """
        Get indices for decimated data to limit display points.
        Returns indices that should be displayed.
        """
        num_points = end_idx - start_idx
        if num_points <= self.MAX_DISPLAY_POINTS:
            return np.arange(start_idx, end_idx)
        
        # Downsample to MAX_DISPLAY_POINTS
        step = num_points / self.MAX_DISPLAY_POINTS
        return np.array([int(start_idx + i * step) for i in range(self.MAX_DISPLAY_POINTS)])
    
    def _find_time_range_indices(self, t_start: float, t_end: float) -> Tuple[int, int]:
        """Find the indices corresponding to a time range."""
        start_idx = np.searchsorted(self.times, t_start)
        end_idx = np.searchsorted(self.times, t_end)
        return int(max(0, start_idx)), int(min(len(self.times), end_idx))
    
    def _update_plots(self, t_center: float = None):
        """Update all plots for the current time window."""
        if t_center is None:
            t_center = self.current_time
        
        # Calculate visible window
        half_window = self.window_size / 2
        t_start = max(0, t_center - half_window)
        t_end = min(self.duration, t_center + half_window)
        
        # Get data indices
        start_idx, end_idx = self._find_time_range_indices(t_start, t_end)
        display_indices = self._decimate_data(start_idx, end_idx)
        
        if len(display_indices) == 0:
            return
        
        times_view = self.times[display_indices]
        
        # Update each plot based on data type
        if self.data_type == 'local':
            self._update_local_plots(display_indices, times_view)
        else:
            self._update_fleet_plots(display_indices, times_view)
        
        # Update axis limits
        for ax_name, ax in self.axes.items():
            if ax_name != 'trajectory':
                ax.set_xlim(t_start, t_end)
        
        # Update info text
        if self.text_info:
            self.text_info.set_text(
                f"Time: {t_center:.1f}s / {self.duration:.1f}s | "
                f"Window: {self.window_size:.0f}s | "
                f"Speed: {self.speed:.1f}x | "
                f"Points: {len(display_indices)}"
            )
        
        self.fig.canvas.draw_idle()
    
    def _update_local_plots(self, indices: np.ndarray, times: np.ndarray):
        """Update local data plots."""
        df = self.df
        
        # Trajectory (X vs Y) - show path in visible window
        if 'trajectory' in self.lines:
            x_data = df['x'].values[indices]
            y_data = df['y'].values[indices]
            self.lines['trajectory'].set_data(x_data, y_data)
            # Axis limits are fixed at X=(-5,5), Y=(-2,2)
        
        # Velocity
        if 'velocity' in self.lines and 'velocity' in df.columns:
            self.lines['velocity'].set_data(times, df['velocity'].values[indices])
        
        # Heading
        if 'theta' in self.lines and 'theta' in df.columns:
            self.lines['theta'].set_data(times, np.degrees(df['theta'].values[indices]))
        
        # Steering
        if 'steering' in self.lines and 'steering' in df.columns:
            self.lines['steering'].set_data(times, df['steering'].values[indices])
        
        # Throttle
        if 'throttle' in self.lines and 'throttle' in df.columns:
            self.lines['throttle'].set_data(times, df['throttle'].values[indices])
    
    def _update_fleet_plots(self, indices: np.ndarray, times: np.ndarray):
        """Update fleet data plots."""
        df = self.df
        
        # Find vehicles
        fleet_cols = [c for c in df.columns if c.startswith('fleet_x_')]
        vehicle_indices = sorted([int(c.split('_')[-1]) for c in fleet_cols])
        
        for v_idx in vehicle_indices:
            x_col = f'fleet_x_{v_idx}'
            y_col = f'fleet_y_{v_idx}'
            
            # Trajectory
            if f'traj_{v_idx}' in self.lines and x_col in df.columns:
                self.lines[f'traj_{v_idx}'].set_data(
                    df[x_col].values[indices],
                    df[y_col].values[indices]
                )
            
            # Velocity
            v_col = f'fleet_v_{v_idx}'
            if f'vel_{v_idx}' in self.lines and v_col in df.columns:
                self.lines[f'vel_{v_idx}'].set_data(times, df[v_col].values[indices])
            
            # Trust score
            trust_col = f'trust_{v_idx}'
            if f'trust_{v_idx}' in self.lines and trust_col in df.columns:
                self.lines[f'trust_{v_idx}'].set_data(times, df[trust_col].values[indices])
        
        # Consensus error
        if 'consensus' in self.lines and 'consensus_error' in df.columns:
            self.lines['consensus'].set_data(times, df['consensus_error'].values[indices])
    
    def _on_time_slider_change(self, val):
        """Handle time slider change."""
        self.current_time = val
        self._update_plots(val)
    
    def _on_window_slider_change(self, val):
        """Handle window size slider change."""
        self.window_size = val
        self._update_plots()
    
    def _on_speed_slider_change(self, val):
        """Handle speed slider change."""
        self.speed = val
    
    def _on_play_button(self, event):
        """Handle play/pause button."""
        self.playing = not self.playing
        self.btn_play.label.set_text('Pause' if self.playing else 'Play')
        
        if self.playing:
            self._last_wall_time = None
            self._start_animation()
        else:
            self._stop_animation()
    
    def _start_animation(self):
        """Start the playback animation using matplotlib.animation."""
        from matplotlib.animation import FuncAnimation
        
        def animate_frame(frame):
            if not self.playing:
                return
            
            import time as time_module
            current_wall = time_module.time()
            
            if self._last_wall_time is None:
                self._last_wall_time = current_wall
                return
            
            dt = (current_wall - self._last_wall_time) * self.speed
            self._last_wall_time = current_wall
            
            self.current_time += dt
            
            if self.current_time >= self.duration:
                self.current_time = 0.0  # Loop
            
            # Update slider (this triggers _on_time_slider_change which updates plots)
            self.slider_time.set_val(self.current_time)
        
        self.animation = FuncAnimation(
            self.fig, animate_frame, interval=50, blit=False, cache_frame_data=False
        )
        self.fig.canvas.draw_idle()
    
    def _stop_animation(self):
        """Stop the playback animation."""
        if self.animation is not None:
            self.animation.event_source.stop()
            self.animation = None
    
    def _create_local_layout(self):
        """Create layout for local data visualization."""
        from matplotlib.widgets import Slider, Button
        
        # Create figure with subplots - more bottom margin for controls
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.subplots_adjust(bottom=0.15)
        gs = self.fig.add_gridspec(3, 3, height_ratios=[2, 1, 1], hspace=0.35, wspace=0.3)
        
        # Trajectory (large, left side) - X=(-5,5), Y=(-2,2)
        self.axes['trajectory'] = self.fig.add_subplot(gs[0:2, 0:2])
        self.axes['trajectory'].set_xlabel('X [m]', fontsize=9)
        self.axes['trajectory'].set_ylabel('Y [m]', fontsize=9)
        self.axes['trajectory'].set_title('Trajectory', fontsize=10)
        self.axes['trajectory'].grid(True)
        self.axes['trajectory'].set_xlim(-5, 5)
        self.axes['trajectory'].set_ylim(-2, 2)
        self.axes['trajectory'].tick_params(labelsize=8)
        self.lines['trajectory'], = self.axes['trajectory'].plot([], [], 'b-', lw=2, label='Path')
        
        # Velocity (right top) - V=(-5,5)
        self.axes['velocity'] = self.fig.add_subplot(gs[0, 2])
        self.axes['velocity'].set_ylabel('Velocity [m/s]', fontsize=9)
        self.axes['velocity'].set_ylim(-5, 5)
        self.axes['velocity'].grid(True)
        self.axes['velocity'].tick_params(labelsize=8)
        self.lines['velocity'], = self.axes['velocity'].plot([], [], 'b.', markersize=2)
        
        # Heading (right middle) - deg=(-180,180)
        self.axes['heading'] = self.fig.add_subplot(gs[1, 2])
        self.axes['heading'].set_ylabel('Heading [deg]', fontsize=9)
        self.axes['heading'].set_ylim(-180, 180)
        self.axes['heading'].grid(True)
        self.axes['heading'].tick_params(labelsize=8)
        self.lines['theta'], = self.axes['heading'].plot([], [], 'g.', markersize=2)
        
        # Controls (bottom left) - (-1,1)
        self.axes['control'] = self.fig.add_subplot(gs[2, 0:2])
        self.axes['control'].set_ylabel('Control', fontsize=9)
        self.axes['control'].set_xlabel('Time [s]', fontsize=9)
        self.axes['control'].set_ylim(-1, 1)
        self.axes['control'].grid(True)
        self.axes['control'].tick_params(labelsize=8)
        self.lines['steering'], = self.axes['control'].plot([], [], 'r.', markersize=2, label='Steering')
        self.lines['throttle'], = self.axes['control'].plot([], [], 'k.', markersize=2, label='Throttle')
        self.axes['control'].legend(loc='upper right', fontsize=8)
        
        # Info panel (bottom right)
        self.axes['info'] = self.fig.add_subplot(gs[2, 2])
        self.axes['info'].axis('off')
        self.text_info = self.axes['info'].text(0.5, 0.5, '', ha='center', va='center',
                                                  fontsize=8, transform=self.axes['info'].transAxes)
        
        # Control widgets at bottom
        self._create_control_widgets(gs)
    
    def _create_fleet_layout(self):
        """Create layout for fleet data visualization."""
        from matplotlib.widgets import Slider, Button
        
        # Create figure with subplots - more bottom margin for controls
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.subplots_adjust(bottom=0.15)
        gs = self.fig.add_gridspec(3, 2, height_ratios=[2, 1, 1], hspace=0.35, wspace=0.3)
        
        # Fleet trajectory - X=(-5,5), Y=(-2,2)
        self.axes['trajectory'] = self.fig.add_subplot(gs[0:2, 0])
        self.axes['trajectory'].set_xlabel('X [m]', fontsize=9)
        self.axes['trajectory'].set_ylabel('Y [m]', fontsize=9)
        self.axes['trajectory'].set_title('Fleet Trajectories', fontsize=10)
        self.axes['trajectory'].set_xlim(-5, 5)
        self.axes['trajectory'].set_ylim(-2, 2)
        self.axes['trajectory'].grid(True)
        self.axes['trajectory'].tick_params(labelsize=8)
        
        # Initialize fleet lines
        fleet_cols = [c for c in self.df.columns if c.startswith('fleet_x_')]
        vehicle_indices = sorted([int(c.split('_')[-1]) for c in fleet_cols])
        num_vehicles = max(vehicle_indices) + 1 if vehicle_indices else 1
        colors = plt.cm.jet(np.linspace(0, 1, num_vehicles))
        
        for v_idx in vehicle_indices:
            color = colors[v_idx]
            self.lines[f'traj_{v_idx}'], = self.axes['trajectory'].plot(
                [], [], '-', color=color, lw=2, label=f'V{v_idx}'
            )
        self.axes['trajectory'].legend(fontsize=8)
        
        # Velocities - V=(-5,5)
        self.axes['velocity'] = self.fig.add_subplot(gs[0, 1])
        self.axes['velocity'].set_ylabel('Velocity [m/s]', fontsize=9)
        self.axes['velocity'].set_ylim(-5, 5)
        self.axes['velocity'].grid(True)
        self.axes['velocity'].tick_params(labelsize=8)
        
        for v_idx in vehicle_indices:
            color = colors[v_idx]
            self.lines[f'vel_{v_idx}'], = self.axes['velocity'].plot(
                [], [], '.', color=color, markersize=2
            )
        
        # Consensus error
        self.axes['consensus'] = self.fig.add_subplot(gs[1, 1])
        self.axes['consensus'].set_ylabel('Consensus Error', fontsize=9)
        self.axes['consensus'].grid(True)
        self.axes['consensus'].tick_params(labelsize=8)
        self.lines['consensus'], = self.axes['consensus'].plot([], [], 'k.', markersize=2)
        
        # Trust scores - range (-1, 1)
        self.axes['trust'] = self.fig.add_subplot(gs[2, 0])
        self.axes['trust'].set_ylabel('Trust Score', fontsize=9)
        self.axes['trust'].set_xlabel('Time [s]', fontsize=9)
        self.axes['trust'].set_ylim(-1, 1)
        self.axes['trust'].grid(True)
        self.axes['trust'].tick_params(labelsize=8)
        
        for v_idx in vehicle_indices:
            color = colors[v_idx]
            self.lines[f'trust_{v_idx}'], = self.axes['trust'].plot(
                [], [], '.', color=color, markersize=2, label=f'V{v_idx}'
            )
        self.axes['trust'].legend(fontsize=7, loc='upper right')
        
        # Info panel
        self.axes['info'] = self.fig.add_subplot(gs[2, 1])
        self.axes['info'].axis('off')
        self.text_info = self.axes['info'].text(0.5, 0.5, '', ha='center', va='center',
                                                  fontsize=8, transform=self.axes['info'].transAxes)
        
        self._create_control_widgets(gs)
    
    def _create_control_widgets(self, gs):
        """Create control widgets (sliders and buttons)."""
        from matplotlib.widgets import Slider, Button
        
        # Create axes for controls - better spacing
        ax_time = self.fig.add_axes([0.12, 0.06, 0.60, 0.025])
        ax_window = self.fig.add_axes([0.12, 0.025, 0.22, 0.025])
        ax_speed = self.fig.add_axes([0.40, 0.025, 0.22, 0.025])
        ax_play = self.fig.add_axes([0.75, 0.02, 0.12, 0.04])
        
        # Time slider
        self.slider_time = Slider(
            ax_time, 'Time', 0, self.duration, valinit=0,
            valstep=0.1, color='lightblue'
        )
        self.slider_time.on_changed(self._on_time_slider_change)
        
        # Window size slider
        self.slider_window = Slider(
            ax_window, 'Window', 5, max(60, self.duration), valinit=self.window_size,
            valstep=5, color='lightgreen'
        )
        self.slider_window.on_changed(self._on_window_slider_change)
        
        # Speed slider
        self.slider_speed = Slider(
            ax_speed, 'Speed', 0.25, 4.0, valinit=1.0,
            valstep=0.25, color='lightyellow'
        )
        self.slider_speed.on_changed(self._on_speed_slider_change)
        
        # Play button
        self.btn_play = Button(ax_play, '▶ Play', color='lightgray', hovercolor='gray')
        self.btn_play.on_clicked(self._on_play_button)
    
    def show(self):
        """Display the interactive viewer."""
        if not self.load_data():
            return
        
        # Create layout based on data type
        if self.data_type == 'local':
            self._create_local_layout()
        else:
            self._create_fleet_layout()
        
        # Set title
        filename = os.path.basename(self.filepath)
        self.fig.suptitle(f'Interactive Viewer: {filename}', fontsize=12, fontweight='bold')
        
        # Initial plot
        self._update_plots(0)
        
        plt.show()


# =============================================================================
# Plotting Functions
# =============================================================================

def plot_local_data(filepath: str, ax_override: dict = None):
    """
    Plot local estimation data.
    
    Args:
        filepath: Path to CSV file
        ax_override: Optional dict of axes to plot on (for combined plots)
    """
    print(f"Plotting local data from: {filepath}")
    
    try:
        df = pd.read_csv(filepath)
    except Exception as e:
        print(f"Error reading file: {e}")
        return
    
    # Check for required columns
    if 'time' not in df.columns:
        print("Error: 'time' column not found in data")
        return
    
    # Normalize time to start from 0
    t_start = df['time'].iloc[0]
    t_end = df['time'].iloc[-1]
    duration = t_end - t_start
    print(f"  Duration: {duration:.2f} seconds ({len(df)} samples)")
    print(f"  Time normalized: {t_start:.3f} -> 0.0")

    standalone = ax_override is None
    
    if standalone:
        fig = plt.figure(figsize=(15, 12))
        gs = fig.add_gridspec(3, 3)
        ax1 = fig.add_subplot(gs[0:2, 0:2])
        ax2 = fig.add_subplot(gs[0, 2])
        ax3 = fig.add_subplot(gs[1, 2])
        ax4 = fig.add_subplot(gs[2, :])
    else:
        ax1 = ax_override.get('trajectory')
        ax2 = ax_override.get('velocity')
        ax3 = ax_override.get('heading')
        ax4 = ax_override.get('control')

    # 1. Trajectory (X vs Y)
    if ax1:
        ax1.plot(df['x'], df['y'], 'b-', label='Estimated Path', linewidth=2)
        
        if 'x_gps' in df.columns and 'gps_valid' in df.columns:
            valid_gps = df[df['gps_valid'] == 1]
            if not valid_gps.empty:
                ax1.plot(valid_gps['x_gps'], valid_gps['y_gps'], 'rx', 
                        label='GPS', markersize=4, alpha=0.6)
        elif 'x_gps' in df.columns:
            ax1.plot(df['x_gps'], df['y_gps'], 'rx', label='GPS', markersize=4, alpha=0.6)
        
        ax1.set_title('Vehicle Trajectory (Local Frame)')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.legend()
        ax1.grid(True)
        ax1.axis('equal')

    # Time starts from 0
    times = df['time'] - df['time'].iloc[0]

    # 2. Velocity & Acceleration
    if ax2:
        ax2.plot(times, df['velocity'], 'b-', label='Velocity')
        if 'v_ref' in df.columns:
            ax2.plot(times, df['v_ref'], 'g--', label='Ref Vel')
        ax2.set_title('Velocity')
        ax2.set_ylabel('Speed [m/s]')
        ax2.legend(loc='upper left')
        ax2.grid(True)
        
        if 'acceleration' in df.columns:
            ax2b = ax2.twinx()
            ax2b.plot(times, df['acceleration'], 'r:', label='Accel', alpha=0.5)
            ax2b.set_ylabel('Accel [m/s²]')

    # 3. Orientation (Theta)
    if ax3:
        ax3.plot(times, np.degrees(df['theta']), 'b-', label='Est Theta')
        if 'theta_gps' in df.columns:
            if 'gps_valid' in df.columns:
                valid_gps = df[df['gps_valid'] == 1]
                if not valid_gps.empty:
                    valid_times = valid_gps['time'] - df['time'].iloc[0]
                    ax3.plot(valid_times, np.degrees(valid_gps['theta_gps']), 
                            'r.', label='GPS Theta', markersize=2)
            else:
                ax3.plot(times, np.degrees(df['theta_gps']), 'r.', 
                        label='GPS Theta', markersize=2)
                
        ax3.set_title('Heading (Theta)')
        ax3.set_ylabel('Angle [deg]')
        ax3.legend()
        ax3.grid(True)

    # 4. Control Inputs
    if ax4:
        if 'steering' in df.columns and 'throttle' in df.columns:
            ax4.plot(times, df['steering'], 'g-', label='Steering')
            ax4.plot(times, df['throttle'], 'k--', label='Throttle')
            ax4.set_title('Control Inputs')
            ax4.set_xlabel('Time [s]')
            ax4.legend()
            ax4.grid(True)
            ax4.set_ylim(-1.1, 1.1)

    if standalone:
        plt.tight_layout()
        plt.show()


def plot_fleet_data(filepath: str, ax_override: dict = None, 
                    selected_vehicles: List[int] = None):
    """
    Plot fleet estimation data.
    
    Args:
        filepath: Path to CSV file
        ax_override: Optional dict of axes to plot on
        selected_vehicles: List of vehicle indices to plot (None for all)
    """
    print(f"Plotting fleet data from: {filepath}")
    
    try:
        df = pd.read_csv(filepath)
    except Exception as e:
        print(f"Error reading file: {e}")
        return
    
    # Check for required columns
    if 'time' not in df.columns:
        print("Error: 'time' column not found in data")
        return
    
    # Normalize time to start from 0
    t_start = df['time'].iloc[0]
    t_end = df['time'].iloc[-1]
    duration = t_end - t_start
    print(f"  Duration: {duration:.2f} seconds ({len(df)} samples)")
    print(f"  Time normalized: {t_start:.3f} -> 0.0")

    # Identify vehicles
    fleet_cols = [c for c in df.columns if c.startswith('fleet_x_')]
    vehicle_indices = sorted([int(c.split('_')[-1]) for c in fleet_cols])
    
    if selected_vehicles is not None:
        vehicle_indices = [v for v in vehicle_indices if v in selected_vehicles]
    
    if not vehicle_indices:
        print("No vehicles found in data!")
        return
    
    print(f"  Vehicles found: {vehicle_indices}")
    if selected_vehicles:
        print(f"  Plotting vehicles: {selected_vehicles}")
    
    # Time starts from 0
    times = df['time'] - df['time'].iloc[0]
    colors = plt.cm.jet(np.linspace(0, 1, max(vehicle_indices) + 1))
    
    standalone = ax_override is None
    
    if standalone:
        fig = plt.figure(figsize=(15, 10))
        gs = fig.add_gridspec(2, 2)
        ax1 = fig.add_subplot(gs[0, 0])
        ax2 = fig.add_subplot(gs[0, 1])
        ax3 = fig.add_subplot(gs[1, :])
    else:
        ax1 = ax_override.get('trajectory')
        ax2 = ax_override.get('consensus')
        ax3 = ax_override.get('trust')

    # 1. Fleet Positions (X vs Y Map)
    if ax1:
        for idx in vehicle_indices:
            x_col = f'fleet_x_{idx}'
            y_col = f'fleet_y_{idx}'
            
            if x_col in df.columns and y_col in df.columns:
                if df[x_col].abs().max() > 0.001 or df[y_col].abs().max() > 0.001:
                    ax1.plot(df[x_col], df[y_col], '-', label=f'Vehicle {idx}', 
                            color=colors[idx])
                    ax1.plot(df[x_col].iloc[-1], df[y_col].iloc[-1], 'o', 
                            color=colors[idx], markersize=8)
        
        ax1.set_title('Fleet Trajectories (Local Frame of Observer)')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.legend()
        ax1.grid(True)
        ax1.axis('equal')

    # 2. Consensus Error
    if ax2:
        if 'consensus_error' in df.columns:
            ax2.plot(times, df['consensus_error'], 'k-', label='Consensus Error')
            ax2.set_title('Consensus Convergence')
            ax2.set_xlabel('Time [s]')
            ax2.set_ylabel('Error')
            ax2.grid(True)
            if df['consensus_error'].min() > 0:
                ax2.set_yscale('log')
        else:
            ax2.text(0.5, 0.5, "No Consensus Error Data", ha='center', 
                    transform=ax2.transAxes)

    # 3. Trust Scores
    if ax3:
        trust_cols = [c for c in df.columns if c.startswith('trust_')]
        
        if trust_cols:
            for col in trust_cols:
                idx = int(col.split('_')[-1])
                if selected_vehicles is None or idx in selected_vehicles:
                    ax3.plot(times, df[col], '-', label=f'Trust V{idx}', 
                            color=colors[idx])
            
            ax3.set_title('Peer Trust Scores')
            ax3.set_xlabel('Time [s]')
            ax3.set_ylabel('Trust Score [0-1]')
            ax3.legend()
            ax3.grid(True)
            ax3.set_ylim(-0.1, 1.1)
        else:
            ax3.text(0.5, 0.5, "No Trust Scores Recorded", ha='center',
                    transform=ax3.transAxes)

    if standalone:
        plt.tight_layout()
        plt.show()


def plot_both(local_file: str, fleet_file: str):
    """Plot local and fleet data side by side."""
    fig = plt.figure(figsize=(20, 12))
    
    # Local data on left side
    gs_local = fig.add_gridspec(3, 2, left=0.05, right=0.48)
    ax_local = {
        'trajectory': fig.add_subplot(gs_local[0:2, :]),
        'velocity': fig.add_subplot(gs_local[2, 0]),
        'heading': fig.add_subplot(gs_local[2, 1]),
    }
    
    # Fleet data on right side
    gs_fleet = fig.add_gridspec(3, 2, left=0.52, right=0.98)
    ax_fleet = {
        'trajectory': fig.add_subplot(gs_fleet[0, :]),
        'consensus': fig.add_subplot(gs_fleet[1, 0]),
        'trust': fig.add_subplot(gs_fleet[1:, 1]),
    }
    
    if local_file:
        plot_local_data(local_file, ax_local)
    
    if fleet_file:
        plot_fleet_data(fleet_file, ax_fleet)
    
    fig.suptitle('Local & Fleet Estimation Data', fontsize=14)
    plt.show()


# =============================================================================
# Playback Functions
# =============================================================================

def playback_data(filepath: str, data_type: str, speed: float = 1.0, 
                  max_vehicles: int = 5):
    """
    Playback recorded data using estimation scopes.
    
    Args:
        filepath: Path to CSV file
        data_type: 'local' or 'fleet'
        speed: Playback speed multiplier
        max_vehicles: Maximum vehicles for fleet playback
    """
    if not PLAYBACK_AVAILABLE:
        print("Playback mode not available!")
        print("Make sure estimation_scopes.py is accessible and MultiScope is installed.")
        return
    
    print(f"\nStarting playback: {filepath}")
    print(f"Type: {data_type}, Speed: {speed}x")
    print("Press Ctrl+C to stop playback\n")
    
    try:
        if data_type == 'local':
            mgr = EstimationScopeManager.create_default_local_scopes(
                fps=30, time_window=30.0
            )
        else:
            mgr = EstimationScopeManager.create_default_fleet_scopes(
                fps=30, time_window=30.0, max_vehicles=max_vehicles
            )
        
        mgr.start(threaded=False)
        print(f"Active presets: {list(mgr.presets.keys())}")
        
        player = ScopeDataPlayer(filepath)
        if player.load(max_vehicles=max_vehicles):
            player.play(mgr, speed=speed)
        else:
            print("Failed to load recording file!")
        
        mgr.stop()
        print("Playback completed!")
        
    except KeyboardInterrupt:
        print("\nPlayback interrupted by user")
        mgr.stop()
    except Exception as e:
        print(f"Playback error: {e}")


def interactive_playback_menu(directory: str):
    """Interactive menu for playback options."""
    clear_screen()
    print_header("Playback Mode")
    
    if not PLAYBACK_AVAILABLE:
        print("\n[!] Playback mode is not available!")
        print("    Missing: estimation_scopes.py or MultiScope library")
        input("\nPress Enter to continue...")
        return
    
    # Get all files
    files = get_all_files(directory)
    files = sort_files(files, 'newest')
    
    if not files:
        print("No recording files found!")
        input("\nPress Enter to continue...")
        return
    
    # Select data type
    print_menu(['Playback local data', 'Playback fleet data'], "Select Playback Type")
    choice = get_choice(2)
    
    if choice == 0:
        return
    
    data_type = 'local' if choice == 1 else 'fleet'
    
    # Filter files by type
    filtered_files = [f for f in files if f[2] == data_type]
    
    if not filtered_files:
        print(f"No {data_type} recordings found!")
        input("\nPress Enter to continue...")
        return
    
    # Select file
    filepath = interactive_file_browser(filtered_files)
    
    if filepath is None:
        return
    
    # Speed selection
    print("\nPlayback speed:")
    print("  1. 0.5x (Slow)")
    print("  2. 1.0x (Normal)")
    print("  3. 2.0x (Fast)")
    print("  4. Custom")
    
    speed_choice = get_choice(4, "Select speed")
    
    if speed_choice == 0:
        return
    elif speed_choice == 1:
        speed = 0.5
    elif speed_choice == 2:
        speed = 1.0
    elif speed_choice == 3:
        speed = 2.0
    else:
        try:
            speed = float(input("Enter speed (0.1 - 5.0): "))
            speed = max(0.1, min(5.0, speed))
        except:
            speed = 1.0
    
    # Max vehicles for fleet
    max_vehicles = 5
    if data_type == 'fleet':
        try:
            max_vehicles = int(input("Max vehicles to display (default 5): ") or "5")
        except:
            max_vehicles = 5
    
    # Start playback
    playback_data(filepath, data_type, speed, max_vehicles)


# =============================================================================
# Main Interactive Interface
# =============================================================================

def main_menu():
    """Main interactive menu."""
    while True:
        clear_screen()
        print_header("QCar Scope Data Plotter")
        
        found_dirs = find_recording_directories()
        total_dirs = sum(len(v) for v in found_dirs.values())
        
        print(f"\nFound {total_dirs} recording location(s)")
        for v_type, dirs in found_dirs.items():
            if dirs:
                print(f"  {v_type.replace('_', ' ').title()}: {len(dirs)} dir(s)")
        
        options = [
            "Plot data (static matplotlib)",
            "Interactive viewer (with time slider)",
            "Playback mode (real-time visualization)",
            "Quick plot - Latest local",
            "Quick plot - Latest fleet",
            "Show recording directories",
            "Add custom directory",
        ]
        
        print_menu(options, "Main Menu")
        choice = get_choice(len(options))
        
        if choice == 0:
            print("\nGoodbye!")
            break
            
        elif choice == 1:
            # Static plot mode
            result = select_vehicle_type(found_dirs)
            if result is None:
                continue
            
            vehicle_type, directory = result
            
            data_type = select_data_type()
            if data_type is None:
                continue
            
            file_order = select_file_order()
            if file_order is None:
                continue
            
            files = get_all_files(directory, 
                                  data_type if data_type != 'both' else None)
            
            if not files:
                print("No files found!")
                input("\nPress Enter to continue...")
                continue
            
            if file_order == 'latest':
                files = sort_files(files, 'newest')
                if data_type == 'both':
                    local_file = next((f[0] for f in files if f[2] == 'local'), None)
                    fleet_file = next((f[0] for f in files if f[2] == 'fleet'), None)
                    plot_both(local_file, fleet_file)
                elif data_type == 'local':
                    plot_local_data(files[0][0])
                else:
                    plot_fleet_data(files[0][0])
                    
            elif file_order == 'oldest':
                files = sort_files(files, 'oldest')
                if data_type == 'both':
                    local_file = next((f[0] for f in files if f[2] == 'local'), None)
                    fleet_file = next((f[0] for f in files if f[2] == 'fleet'), None)
                    plot_both(local_file, fleet_file)
                elif data_type == 'local':
                    plot_local_data(files[0][0])
                else:
                    plot_fleet_data(files[0][0])
                    
            else:  # browse
                files = sort_files(files, 'newest')
                filepath = interactive_file_browser(files)
                if filepath:
                    info = get_file_info(filepath)
                    if info['type'] == 'local':
                        plot_local_data(filepath)
                    else:
                        # Ask about vehicle selection for fleet
                        df = pd.read_csv(filepath)
                        selected = select_vehicles_to_plot(df)
                        if selected:
                            plot_fleet_data(filepath, selected_vehicles=selected)
                            
        elif choice == 2:
            # Interactive viewer mode
            result = select_vehicle_type(found_dirs)
            if result is None:
                continue
            
            vehicle_type, directory = result
            
            # Select data type
            print_menu(['Local data', 'Fleet data'], "Select Data Type")
            type_choice = get_choice(2)
            
            if type_choice == 0:
                continue
            
            data_type = 'local' if type_choice == 1 else 'fleet'
            
            # Get files and let user browse
            files = get_all_files(directory, data_type)
            files = sort_files(files, 'newest')
            
            if not files:
                print(f"No {data_type} files found!")
                input("\nPress Enter to continue...")
                continue
            
            filepath = interactive_file_browser(files)
            
            if filepath:
                print(f"\nLaunching interactive viewer for: {os.path.basename(filepath)}")
                viewer = InteractiveDataViewer(filepath, data_type)
                viewer.show()
                            
        elif choice == 3:
            # Playback mode
            result = select_vehicle_type(found_dirs)
            if result:
                _, directory = result
                interactive_playback_menu(directory)
                
        elif choice == 4:
            # Quick plot latest local
            found = False
            for v_type in ['fake_vehicle', 'real_vehicle']:
                for d in found_dirs.get(v_type, []):
                    files = get_all_files(d, 'local')
                    if files:
                        files = sort_files(files, 'newest')
                        plot_local_data(files[0][0])
                        found = True
                        break
                if found:
                    break
            
            if not found:
                print("No local data files found!")
                input("\nPress Enter to continue...")
                
        elif choice == 5:
            # Quick plot latest fleet
            found = False
            for v_type in ['fake_vehicle', 'real_vehicle']:
                for d in found_dirs.get(v_type, []):
                    files = get_all_files(d, 'fleet')
                    if files:
                        files = sort_files(files, 'newest')
                        plot_fleet_data(files[0][0])
                        found = True
                        break
                if found:
                    break
            
            if not found:
                print("No fleet data files found!")
                input("\nPress Enter to continue...")
                
        elif choice == 6:
            # Show directories
            clear_screen()
            print_header("Recording Directories")
            
            for v_type, dirs in found_dirs.items():
                print(f"\n{v_type.replace('_', ' ').title()}:")
                if dirs:
                    for d in dirs:
                        local_count = len(glob.glob(os.path.join(d, 'local', '*.csv')))
                        fleet_count = len(glob.glob(os.path.join(d, 'fleet', '*.csv')))
                        print(f"  {d}")
                        print(f"    Local files: {local_count}, Fleet files: {fleet_count}")
                else:
                    print("  No directories found")
            
            input("\nPress Enter to continue...")
            
        elif choice == 7:
            # Add custom directory
            clear_screen()
            print_header("Add Custom Directory")
            
            custom_path = input("\nEnter full path to recording directory: ").strip()
            
            if custom_path and os.path.exists(custom_path):
                # Check if it has local or fleet subdirs
                local_dir = os.path.join(custom_path, 'local')
                fleet_dir = os.path.join(custom_path, 'fleet')
                
                if os.path.exists(local_dir) or os.path.exists(fleet_dir):
                    print(f"\nSelect vehicle type for this directory:")
                    print("  1. Fake Vehicle (simulation)")
                    print("  2. Real Vehicle")
                    
                    v_choice = get_choice(2)
                    
                    if v_choice == 1:
                        RECORDING_PATHS["fake_vehicle"].insert(0, custom_path)
                        print(f"Added as fake vehicle directory")
                    elif v_choice == 2:
                        RECORDING_PATHS["real_vehicle"].insert(0, custom_path)
                        print(f"Added as real vehicle directory")
                else:
                    print("Directory does not contain 'local' or 'fleet' subdirectories!")
            else:
                print("Directory not found!")
            
            input("\nPress Enter to continue...")


# =============================================================================
# Command Line Interface
# =============================================================================

def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Interactive QCar Scope Data Plotter",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python plot_scope_data.py                     # Interactive mode
  python plot_scope_data.py --type local        # Plot latest local data
  python plot_scope_data.py --type fleet        # Plot latest fleet data
  python plot_scope_data.py --file data.csv     # Plot specific file
  python plot_scope_data.py --playback          # Playback mode
  python plot_scope_data.py --dir /path/to/dir  # Use specific directory
        """
    )
    
    parser.add_argument('--type', type=str, choices=['local', 'fleet', 'both'],
                        help="Type of data to plot")
    parser.add_argument('--file', type=str,
                        help="Specific file to plot")
    parser.add_argument('--dir', type=str,
                        help="Base directory for recordings")
    parser.add_argument('--playback', action='store_true',
                        help="Enable playback mode")
    parser.add_argument('--speed', type=float, default=1.0,
                        help="Playback speed (default: 1.0)")
    parser.add_argument('--vehicles', type=str,
                        help="Comma-separated vehicle indices to plot")
    parser.add_argument('--interactive', '-i', action='store_true',
                        help="Force interactive mode")
    parser.add_argument('--vehicle-type', type=str, choices=['fake', 'real'],
                        help="Filter by vehicle type (fake or real)")
    
    return parser.parse_args()


def main():
    """Main entry point."""
    args = parse_args()
    
    # Force interactive mode
    if args.interactive or (not args.type and not args.file and not args.playback):
        main_menu()
        return
    
    # Handle specific file
    if args.file:
        if not os.path.exists(args.file):
            print(f"File not found: {args.file}")
            return
        
        info = get_file_info(args.file)
        
        if args.playback:
            playback_data(args.file, info['type'], args.speed)
        else:
            if info['type'] == 'local':
                plot_local_data(args.file)
            else:
                selected = None
                if args.vehicles:
                    try:
                        selected = [int(x.strip()) for x in args.vehicles.split(',')]
                    except:
                        pass
                plot_fleet_data(args.file, selected_vehicles=selected)
        return
    
    # Find directory
    found_dirs = find_recording_directories()
    
    if args.dir:
        directory = args.dir
    else:
        # Use first found directory based on vehicle type filter
        directory = None
        search_order = ['fake_vehicle', 'real_vehicle']
        
        if args.vehicle_type == 'fake':
            search_order = ['fake_vehicle']
        elif args.vehicle_type == 'real':
            search_order = ['real_vehicle']
        
        for v_type in search_order:
            if found_dirs.get(v_type):
                directory = found_dirs[v_type][0]
                break
    
    if not directory:
        print("No recording directory found!")
        print("Use --dir to specify a directory or run in interactive mode.")
        return
    
    # Get files
    files = get_all_files(directory, args.type if args.type != 'both' else None)
    
    if not files:
        print(f"No files found in {directory}")
        return
    
    files = sort_files(files, 'newest')
    
    if args.playback:
        if args.type:
            data_files = [f for f in files if f[2] == args.type]
            if data_files:
                playback_data(data_files[0][0], args.type, args.speed)
            else:
                print(f"No {args.type} files found!")
        else:
            # Default to local
            local_files = [f for f in files if f[2] == 'local']
            if local_files:
                playback_data(local_files[0][0], 'local', args.speed)
    else:
        if args.type == 'both':
            local_file = next((f[0] for f in files if f[2] == 'local'), None)
            fleet_file = next((f[0] for f in files if f[2] == 'fleet'), None)
            plot_both(local_file, fleet_file)
        elif args.type == 'local':
            local_files = [f for f in files if f[2] == 'local']
            if local_files:
                plot_local_data(local_files[0][0])
        elif args.type == 'fleet':
            fleet_files = [f for f in files if f[2] == 'fleet']
            if fleet_files:
                selected = None
                if args.vehicles:
                    try:
                        selected = [int(x.strip()) for x in args.vehicles.split(',')]
                    except:
                        pass
                plot_fleet_data(fleet_files[0][0], selected_vehicles=selected)


if __name__ == "__main__":
    main()
