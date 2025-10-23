"""
Enhanced Data Logger for Vehicle Observer System
Provides structured logging for both local states and fleet states with plotting capabilities.
Uses centralized DataStructures for consistency.
"""

import os
import csv
import json
import numpy as np
import pandas as pd
import time
from datetime import datetime
from typing import Dict, List, Optional, Any
import threading
from scipy.io import savemat
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation



class ObserverDataLogger:
    """
    Enhanced data logger for vehicle observer system that saves data in multiple formats:
    1. CSV files for easy plotting
    2. JSON files for structured data
    3. MATLAB .mat files for MATLAB analysis
    """
    
    def __init__(self, vehicle_id: int, fleet_size: int, log_dir: str = "data_logs", 
                 create_new_run: bool = True, custom_run_name: str = None):
        """
        Initialize the data logger.
        
        Args:
            vehicle_id: ID of the vehicle
            fleet_size: Total number of vehicles in fleet
            log_dir: Directory to save data files
            create_new_run: If True, creates new timestamped directory. If False, reuses "current_run"
            custom_run_name: Custom name for the run directory (overrides timestamp)
        """
        self.vehicle_id = vehicle_id
        self.fleet_size = fleet_size
        self.log_dir = log_dir
        self.create_new_run = create_new_run
        
        # Determine data directory based on settings
        if custom_run_name:
            # Use custom name
            self.run_timestamp = custom_run_name
            self.data_dir = os.path.join(log_dir, f"run_{custom_run_name}")
        elif create_new_run:
            # Create new timestamped directory
            self.run_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.data_dir = os.path.join(log_dir, f"run_{self.run_timestamp}")
        else:
            # Reuse existing "current_run" directory
            self.run_timestamp = "current"
            self.data_dir = os.path.join(log_dir, "current_run")
        
        # Create data directory
        os.makedirs(self.data_dir, exist_ok=True)
        
        # Data storage
        self.local_state_data = []
        self.fleet_state_data = []
        self.control_data = []
        self.gps_data = []
        
        # File handles for real-time CSV logging
        self.csv_files = {}
        self.csv_writers = {}
        self._init_csv_files()
        
        # Thread safety
        self.lock = threading.RLock()
        
        print(f"DataLogger initialized for Vehicle {vehicle_id}")
        print(f"Data directory: {self.data_dir}")
    
    def _init_csv_files(self):
        """Initialize CSV files for real-time logging."""
        
        # Local state CSV
        local_csv_path = os.path.join(self.data_dir, f"local_state_vehicle_{self.vehicle_id}.csv")
        self.csv_files['local'] = open(local_csv_path, 'w', newline='')
        self.csv_writers['local'] = csv.writer(self.csv_files['local'])
        self.csv_writers['local'].writerow([
            'timestamp', 'vehicle_id', 'x', 'y', 'theta', 'velocity', 
            'gps_available', 'dt', 'gps_x', 'gps_y', 'gps_theta', 'gps_velocity',
            'control_steering', 'control_acceleration', 'actual_acceleration'
        ])
        
        # Fleet state CSV  
        fleet_csv_path = os.path.join(self.data_dir, f"fleet_states_vehicle_{self.vehicle_id}.csv")
        self.csv_files['fleet'] = open(fleet_csv_path, 'w', newline='')
        self.csv_writers['fleet'] = csv.writer(self.csv_files['fleet'])
        
        # Dynamic header based on fleet size
        header = ['timestamp', 'observer_vehicle_id']
        for i in range(self.fleet_size):
            header.extend([f'vehicle_{i}_x', f'vehicle_{i}_y', f'vehicle_{i}_theta', f'vehicle_{i}_velocity'])
        self.csv_writers['fleet'].writerow(header)
    
    def log_local_state(self, timestamp: float, local_state: np.ndarray, 
                       measured_state: Optional[np.ndarray], control_input: np.ndarray,
                       gps_available: bool, dt: float, actual_acceleration: float = 0.0):
        """
        Log local state data.
        
        Args:
            timestamp: GPS-synchronized timestamp
            local_state: Estimated local state [x, y, theta, v]
            measured_state: GPS measurement [x, y, theta, v] or None
            control_input: Control input [steering, acceleration]
            gps_available: Whether GPS is available
            dt: Time step
            actual_acceleration: Actual measured acceleration
        """
        with self.lock:
            # Store in memory
            data_entry = {
                'timestamp': timestamp,
                'vehicle_id': self.vehicle_id,
                'local_state': local_state.copy(),
                'measured_state': measured_state.copy() if measured_state is not None else None,
                'control_input': control_input.copy(),
                'gps_available': gps_available,
                'dt': dt,
                'actual_acceleration': actual_acceleration
            }
            self.local_state_data.append(data_entry)
            
            # Write to CSV immediately
            gps_values = measured_state if measured_state is not None else [np.nan, np.nan, np.nan, np.nan]
            csv_row = [
                timestamp, self.vehicle_id,
                local_state[0], local_state[1], local_state[2], local_state[3],
                gps_available, dt,
                gps_values[0], gps_values[1], gps_values[2], gps_values[3],
                control_input[0], control_input[1], actual_acceleration
            ]
            self.csv_writers['local'].writerow(csv_row)
            self.csv_files['local'].flush()

    
    def log_fleet_states(self, timestamp: float, fleet_states: np.ndarray):
        """
        Log fleet state estimates.
        
        Args:
            timestamp: GPS-synchronized timestamp
            fleet_states: Fleet state estimates [state_dim x fleet_size]
        """
        with self.lock:
            # Store in memory
            data_entry = {
                'timestamp': timestamp,
                'observer_vehicle_id': self.vehicle_id,
                'fleet_states': fleet_states.copy()
            }
            self.fleet_state_data.append(data_entry)
            
            # Write to CSV immediately
            csv_row = [timestamp, self.vehicle_id]
            for i in range(self.fleet_size):
                csv_row.extend([
                    fleet_states[0, i],  # x
                    fleet_states[1, i],  # y
                    fleet_states[2, i],  # theta
                    fleet_states[3, i]   # velocity
                ])
            self.csv_writers['fleet'].writerow(csv_row)
            self.csv_files['fleet'].flush()
    
    def save_matlab_data(self, filename: Optional[str] = None):
        """
        Save all logged data to MATLAB .mat format.
        
        Args:
            filename: Optional custom filename, otherwise auto-generated
        """
        with self.lock:
            if filename is None:
                filename = f"observer_data_vehicle_{self.vehicle_id}_{self.run_timestamp}.mat"
            
            mat_path = os.path.join(self.data_dir, filename)
            
            # Prepare data for MATLAB
            matlab_data = {
                'metadata': {
                    'vehicle_id': self.vehicle_id,
                    'fleet_size': self.fleet_size,
                    'run_timestamp': self.run_timestamp,
                    'total_samples': len(self.local_state_data)
                }
            }
            
            if self.local_state_data:
                # Local state data
                n_samples = len(self.local_state_data)
                matlab_data['local_states'] = {
                    'timestamps': np.array([d['timestamp'] for d in self.local_state_data]),
                    'x': np.array([d['local_state'][0] for d in self.local_state_data]),
                    'y': np.array([d['local_state'][1] for d in self.local_state_data]),
                    'theta': np.array([d['local_state'][2] for d in self.local_state_data]),
                    'velocity': np.array([d['local_state'][3] for d in self.local_state_data]),
                    'gps_available': np.array([d['gps_available'] for d in self.local_state_data]),
                    'dt': np.array([d['dt'] for d in self.local_state_data])
                }
                
                # GPS measurements (with NaN for missing)
                gps_x = np.full(n_samples, np.nan)
                gps_y = np.full(n_samples, np.nan)
                gps_theta = np.full(n_samples, np.nan)
                gps_velocity = np.full(n_samples, np.nan)
                
                for i, d in enumerate(self.local_state_data):
                    if d['measured_state'] is not None:
                        gps_x[i] = d['measured_state'][0]
                        gps_y[i] = d['measured_state'][1]
                        gps_theta[i] = d['measured_state'][2]
                        gps_velocity[i] = d['measured_state'][3]
                
                matlab_data['gps_measurements'] = {
                    'x': gps_x,
                    'y': gps_y,
                    'theta': gps_theta,
                    'velocity': gps_velocity
                }
                
                # Control inputs
                matlab_data['control_inputs'] = {
                    'steering': np.array([d['control_input'][0] for d in self.local_state_data]),
                    'acceleration': np.array([d['control_input'][1] for d in self.local_state_data])
                }
            
            if self.fleet_state_data:
                # Fleet state data
                fleet_timestamps = np.array([d['timestamp'] for d in self.fleet_state_data])
                fleet_states_array = np.array([d['fleet_states'] for d in self.fleet_state_data])
                
                matlab_data['fleet_estimates'] = {
                    'timestamps': fleet_timestamps,
                    'states': fleet_states_array  # Shape: [n_samples, state_dim, fleet_size]
                }
                
                # Separate arrays for each vehicle
                for i in range(self.fleet_size):
                    matlab_data[f'vehicle_{i}_estimates'] = {
                        'timestamps': fleet_timestamps,
                        'x': fleet_states_array[:, 0, i],
                        'y': fleet_states_array[:, 1, i],
                        'theta': fleet_states_array[:, 2, i],
                        'velocity': fleet_states_array[:, 3, i]
                    }
            
            # Save to MATLAB file
            savemat(mat_path, matlab_data)
            # print(f"MATLAB data saved to: {mat_path}")
            return mat_path
    
    def save_json_data(self, filename: Optional[str] = None):
        """
        Save all logged data to JSON format.
        
        Args:
            filename: Optional custom filename, otherwise auto-generated
        """
        with self.lock:
            if filename is None:
                filename = f"observer_data_vehicle_{self.vehicle_id}_{self.run_timestamp}.json"
            
            json_path = os.path.join(self.data_dir, filename)
            
            # Convert numpy arrays to lists for JSON serialization
            json_data = {
                'metadata': {
                    'vehicle_id': self.vehicle_id,
                    'fleet_size': self.fleet_size,
                    'run_timestamp': self.run_timestamp,
                    'total_samples': len(self.local_state_data)
                },
                'local_state_data': [],
                'fleet_state_data': []
            }
            
            # Convert local state data
            for entry in self.local_state_data:
                json_entry = {
                    'timestamp': entry['timestamp'],
                    'vehicle_id': entry['vehicle_id'],
                    'local_state': entry['local_state'].tolist(),
                    'measured_state': entry['measured_state'].tolist() if entry['measured_state'] is not None else None,
                    'control_input': entry['control_input'].tolist(),
                    'gps_available': entry['gps_available'],
                    'dt': entry['dt']
                }
                json_data['local_state_data'].append(json_entry)
            
            # Convert fleet state data
            for entry in self.fleet_state_data:
                json_entry = {
                    'timestamp': entry['timestamp'],
                    'observer_vehicle_id': entry['observer_vehicle_id'],
                    'fleet_states': entry['fleet_states'].tolist()
                }
                json_data['fleet_state_data'].append(json_entry)
            
            # Save to JSON file
            with open(json_path, 'w') as f:
                json.dump(json_data, f, indent=2)
            
            # print(f"JSON data saved to: {json_path}")
            return json_path
    
    def close(self):
        """Close all file handles and save final data."""
        with self.lock:
            # Close CSV files properly
            try:
                for name, file_handle in self.csv_files.items():
                    if file_handle and not file_handle.closed:
                        file_handle.flush()  # Ensure all data is written
                        file_handle.close()
                        # print(f"Closed CSV file: {name}")
                self.csv_files.clear()
                self.csv_writers.clear()
            except Exception as e:
                print(f"Error closing CSV files: {e}")
            
            # Save data in other formats
            try:
                matlab_path = self.save_matlab_data()
                json_path = self.save_json_data()
                
                print(f"DataLogger for Vehicle {self.vehicle_id} closed.")
                # print(f"DataLogger for Vehicle {self.vehicle_id} closed.")
                # print(f"Total local state samples: {len(self.local_state_data)}")
                # print(f"Total fleet state samples: {len(self.fleet_state_data)}")
                # print(f"Data saved in multiple formats:")
                # print(f"  - CSV: {self.data_dir}")
                # print(f"  - MATLAB: {matlab_path}")
                # print(f"  - JSON: {json_path}")
            except Exception as e:
                print(f"Error saving final data: {e}")
    
    def __del__(self):
        """Destructor to ensure files are closed."""
        try:
            if hasattr(self, 'csv_files') and self.csv_files:
                for file_handle in self.csv_files.values():
                    if file_handle and not file_handle.closed:
                        file_handle.close()
        except:
            pass  # Ignore errors during cleanup


class FleetDataVisualizer:
    """
    Visualization tool for fleet observer data.
    Can plot data from CSV files or directly from DataLogger objects.
    """
    
    def __init__(self, data_dir: str):
        """
        Initialize visualizer.
        
        Args:
            data_dir: Directory containing data files
        """
        self.data_dir = data_dir
    
    def _calculate_relative_time(self, data: pd.DataFrame) -> np.ndarray:
        """
        Calculate relative time from 0 to end of data.
        
        Args:
            data: DataFrame containing timestamp column
            
        Returns:
            relative_time: Array of relative time values starting from 0
        """
        if 'timestamp' not in data.columns:
            # Fallback to sample indices if no timestamp
            return np.arange(len(data))
        
        # Calculate relative time by subtracting the first timestamp
        first_timestamp = data['timestamp'].iloc[0]
        relative_time = data['timestamp'] - first_timestamp
        
        return relative_time.values
    
    def plot_vehicle_trajectory(self, vehicle_id: int, show_gps: bool = True, 
                              save_fig: bool = True, filename: Optional[str] = None):
        """
        Plot trajectory for a specific vehicle.
        
        Args:
            vehicle_id: Vehicle ID to plot
            show_gps: Whether to show GPS measurements
            save_fig: Whether to save the figure
            filename: Custom filename for saved figure
        """
        # Load data from CSV
        csv_path = os.path.join(self.data_dir, f"local_state_vehicle_{vehicle_id}.csv")
        
        if not os.path.exists(csv_path):
            print(f"Data file not found: {csv_path}")
            return
        
        import pandas as pd
        data = pd.read_csv(csv_path)
        
        # Create figure
        plt.figure(figsize=(12, 8))
        
        # Plot estimated trajectory
        plt.plot(data['x'], data['y'], 'b-', linewidth=2, label='Estimated Trajectory')
        plt.plot(data['x'].iloc[0], data['y'].iloc[0], 'go', markersize=8, label='Start')
        plt.plot(data['x'].iloc[-1], data['y'].iloc[-1], 'ro', markersize=8, label='End')
        
        # Plot GPS measurements if available and requested
        if show_gps and 'gps_x' in data.columns and 'gps_available' in data.columns:
            # Only plot GPS when it's actually available (not just last value)
            gps_data = data[data['gps_available'] == True].copy()
            if not gps_data.empty:
                plt.scatter(gps_data['gps_x'], gps_data['gps_y'], 
                           c='red', alpha=0.5, s=20, label='GPS Measurements')
        
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title(f'Vehicle {vehicle_id} Trajectory')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        if save_fig:
            if filename is None:
                filename = f"vehicle_{vehicle_id}_trajectory.png"
            fig_path = os.path.join(self.data_dir, filename)
            plt.savefig(fig_path, dpi=300, bbox_inches='tight')
            print(f"Trajectory plot saved: {fig_path}")
        
        plt.show()
    
    def plot_fleet_trajectories_and_states(self, observer_vehicle_id: int, save_fig: bool = True, 
                                          filename: Optional[str] = None, show_velocity: bool = True,
                                          show_heading: bool = False):
        """
        Plot trajectories and state variables for all vehicles as estimated by one observer.
        
        Args:
            observer_vehicle_id: ID of the vehicle that made the estimates
            save_fig: Whether to save the figure
            filename: Custom filename for saved figure
            show_velocity: Whether to show velocity subplot
            show_heading: Whether to show heading subplot
        """
        # Load fleet data from CSV
        csv_path = os.path.join(self.data_dir, f"fleet_states_vehicle_{observer_vehicle_id}.csv")
        
        if not os.path.exists(csv_path):
            print(f"Fleet data file not found: {csv_path}")
            return
        
        import pandas as pd
        data = pd.read_csv(csv_path)
        
        # Determine fleet size from columns
        vehicle_cols = [col for col in data.columns if col.startswith('vehicle_') and col.endswith('_x')]
        fleet_size = len(vehicle_cols)
        
        # Determine subplot layout based on what to show
        subplot_count = 1  # Always show trajectory
        if show_velocity:
            subplot_count += 1
        if show_heading:
            subplot_count += 1
        
        # Create figure with subplots
        if subplot_count == 1:
            fig, axes = plt.subplots(1, 1, figsize=(12, 8))
            axes = [axes]  # Make it a list for consistent indexing
        elif subplot_count == 2:
            fig, axes = plt.subplots(1, 2, figsize=(20, 8))
        else:  # subplot_count == 3
            fig, axes = plt.subplots(1, 3, figsize=(24, 8))
        
        fig.suptitle(f'Fleet States Estimation by Vehicle {observer_vehicle_id}', fontsize=16)
        
        # Color map for different vehicles
        colors = plt.cm.tab10(np.linspace(0, 1, fleet_size))
        
        # Plot 1: Trajectories (always shown)
        ax_traj = axes[0]
        for i in range(fleet_size):
            x_col = f'vehicle_{i}_x'
            y_col = f'vehicle_{i}_y'
            
            if x_col in data.columns and y_col in data.columns:
                ax_traj.plot(data[x_col], data[y_col], color=colors[i], linewidth=2, 
                           label=f'Vehicle {i}')
                ax_traj.plot(data[x_col].iloc[0], data[y_col].iloc[0], 'o', 
                           color=colors[i], markersize=8, alpha=0.8)
                ax_traj.plot(data[x_col].iloc[-1], data[y_col].iloc[-1], 's', 
                           color=colors[i], markersize=8, alpha=0.8)
        
        ax_traj.set_xlabel('X Position (m)')
        ax_traj.set_ylabel('Y Position (m)')
        ax_traj.set_title('Fleet Trajectories')
        ax_traj.legend()
        ax_traj.grid(True, alpha=0.3)
        ax_traj.axis('equal')
        
        plot_idx = 1
        
        # Plot 2: Velocity (if requested)
        if show_velocity:
            ax_vel = axes[plot_idx]
            relative_time = self._calculate_relative_time(data)
            for i in range(fleet_size):
                vel_col = f'vehicle_{i}_velocity'
                
                if vel_col in data.columns:
                    ax_vel.plot(relative_time, data[vel_col], color=colors[i], 
                              linewidth=2, label=f'Vehicle {i}')
            
            ax_vel.set_xlabel('Time (s)')
            ax_vel.set_ylabel('Velocity (m/s)')
            ax_vel.set_title('Fleet Velocities')
            ax_vel.legend()
            ax_vel.grid(True, alpha=0.3)
            plot_idx += 1
        
        # Plot 3: Heading (if requested)
        if show_heading:
            ax_heading = axes[plot_idx]
            relative_time = self._calculate_relative_time(data)
            for i in range(fleet_size):
                heading_col = f'vehicle_{i}_theta'
                
                if heading_col in data.columns:
                    # Convert from radians to degrees for better readability
                    heading_deg = np.rad2deg(data[heading_col])
                    ax_heading.plot(relative_time, heading_deg, color=colors[i], 
                                  linewidth=2, label=f'Vehicle {i}')
            
            ax_heading.set_xlabel('Time (s)')
            ax_heading.set_ylabel('Heading (degrees)')
            ax_heading.set_title('Fleet Headings')
            ax_heading.legend()
            ax_heading.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_fig:
            if filename is None:
                suffix = ""
                if show_velocity and show_heading:
                    suffix = "_trajectories_velocity_heading"
                elif show_velocity:
                    suffix = "_trajectories_velocity"
                elif show_heading:
                    suffix = "_trajectories_heading"
                else:
                    suffix = "_trajectories"
                filename = f"fleet_states_observer_{observer_vehicle_id}{suffix}.png"
            fig_path = os.path.join(self.data_dir, filename)
            plt.savefig(fig_path, dpi=300, bbox_inches='tight')
            print(f"Fleet states plot saved: {fig_path}")
        
        plt.show()

    def plot_fleet_trajectories(self, observer_vehicle_id: int, save_fig: bool = True, 
                               filename: Optional[str] = None):
        """
        Plot trajectories for all vehicles as estimated by one observer.
        This is a wrapper function that maintains backward compatibility.
        
        Args:
            observer_vehicle_id: ID of the vehicle that made the estimates
            save_fig: Whether to save the figure
            filename: Custom filename for saved figure
        """
        # Call the enhanced function with only trajectory plotting
        self.plot_fleet_trajectories_and_states(
            observer_vehicle_id=observer_vehicle_id,
            save_fig=save_fig,
            filename=filename,
            show_velocity=False,
            show_heading=False
        )
    
    def plot_state_comparison(self, vehicle_id: int, save_fig: bool = True):
        """
        Plot comparison between estimated state and GPS measurements.
        
        Args:
            vehicle_id: Vehicle ID to plot
            save_fig: Whether to save the figure
        """
        # Load data from CSV
        csv_path = os.path.join(self.data_dir, f"local_state_vehicle_{vehicle_id}.csv")
        
        if not os.path.exists(csv_path):
            print(f"Data file not found: {csv_path}")
            return
        
        import pandas as pd
        data = pd.read_csv(csv_path)
        
        # Calculate relative time
        relative_time = self._calculate_relative_time(data)
        
        # Check if acceleration data is available
        has_acceleration = 'actual_acceleration' in data.columns
        
        # Create subplots - 3x2 if acceleration available, else 2x2
        if has_acceleration:
            fig, axes = plt.subplots(3, 2, figsize=(15, 14))
        else:
            fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle(f'Vehicle {vehicle_id} State Estimation vs GPS', fontsize=16)
        
        # Position X
        axes[0, 0].plot(relative_time, data['x'], 'b-', label='Estimated', linewidth=2)
        if 'gps_x' in data.columns and 'gps_available' in data.columns:
            # Only plot GPS when it's actually available
            gps_data = data[data['gps_available'] == True].copy()
            if not gps_data.empty:
                gps_relative_time = self._calculate_relative_time(gps_data)
                axes[0, 0].scatter(gps_relative_time, gps_data['gps_x'], 
                                  c='red', alpha=0.7, s=15, label='GPS')
        axes[0, 0].set_ylabel('X Position (m)')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        # Position Y
        axes[0, 1].plot(relative_time, data['y'], 'b-', label='Estimated', linewidth=2)
        if 'gps_y' in data.columns and 'gps_available' in data.columns:
            # Only plot GPS when it's actually available
            gps_data = data[data['gps_available'] == True].copy()
            if not gps_data.empty:
                gps_relative_time = self._calculate_relative_time(gps_data)
                axes[0, 1].scatter(gps_relative_time, gps_data['gps_y'], 
                                  c='red', alpha=0.7, s=15, label='GPS')
        axes[0, 1].set_ylabel('Y Position (m)')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        # Heading
        axes[1, 0].plot(relative_time, np.rad2deg(data['theta']), 'b-', 
                       label='Estimated', linewidth=2)
        if 'gps_theta' in data.columns and 'gps_available' in data.columns:
            # Only plot GPS when it's actually available
            gps_data = data[data['gps_available'] == True].copy()
            if not gps_data.empty:
                gps_relative_time = self._calculate_relative_time(gps_data)
                axes[1, 0].scatter(gps_relative_time, np.rad2deg(gps_data['gps_theta']), 
                                  c='red', alpha=0.7, s=15, label='GPS')
        axes[1, 0].set_ylabel('Heading (degrees)')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
        
        # Velocity
        axes[1, 1].plot(relative_time, data['velocity'], 'b-', label='Estimated', linewidth=2)
        if 'gps_velocity' in data.columns and 'gps_available' in data.columns:
            # Only plot GPS when it's actually available
            gps_data = data[data['gps_available'] == True].copy()
            if not gps_data.empty:
                gps_relative_time = self._calculate_relative_time(gps_data)
                axes[1, 1].scatter(gps_relative_time, gps_data['gps_velocity'], 
                                  c='red', alpha=0.7, s=15, label='GPS')
        axes[1, 1].set_ylabel('Velocity (m/s)')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        
        # Acceleration (if available)
        if has_acceleration:
            axes[2, 0].plot(relative_time, data['actual_acceleration'], 'g-', 
                           label='Actual Acceleration', linewidth=2)
            if 'control_acceleration' in data.columns:
                axes[2, 0].plot(relative_time, data['control_acceleration'], 'orange', 
                               linestyle='--', label='Control Acceleration', linewidth=1.5, alpha=0.7)
            axes[2, 0].set_ylabel('Acceleration (m/s²)')
            axes[2, 0].set_xlabel('Time (s)')
            axes[2, 0].legend()
            axes[2, 0].grid(True, alpha=0.3)
            
            # Control Steering (if acceleration plot is shown)
            axes[2, 1].plot(relative_time, data['control_steering'], 'purple', 
                           label='Steering Control', linewidth=2)
            axes[2, 1].set_ylabel('Steering Angle (rad)')
            axes[2, 1].set_xlabel('Time (s)')
            axes[2, 1].legend()
            axes[2, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_fig:
            filename = f"vehicle_{vehicle_id}_state_comparison.png"
            fig_path = os.path.join(self.data_dir, filename)
            plt.savefig(fig_path, dpi=300, bbox_inches='tight')
            print(f"State comparison plot saved: {fig_path}")
        
        plt.show()



# Example usage functions
def demo_plotting(data_dir: str):
    """
    Demonstrate plotting capabilities.
    
    Args:
        data_dir: Directory containing the data files
    """
    visualizer = FleetDataVisualizer(data_dir)
    
    # Plot individual vehicle trajectory
    visualizer.plot_vehicle_trajectory(vehicle_id=0)
    
    # Plot fleet trajectories
    visualizer.plot_fleet_trajectories(observer_vehicle_id=0)
    
    # Plot state comparison
    visualizer.plot_state_comparison(vehicle_id=0)


if __name__ == "__main__":
    # Example of how to use the data logger
    print("Observer Data Logger Example")
    
    # Create logger for vehicle 0 in a fleet of 2
    logger = ObserverDataLogger(vehicle_id=0, fleet_size=2)
    
    # Simulate some data
    for i in range(100):
        timestamp = i * 0.01
        local_state = np.array([i * 0.1, i * 0.05, i * 0.01, 1.0])
        measured_state = local_state + np.random.normal(0, 0.01, 4) if i % 5 == 0 else None
        control_input = np.array([0.1, 0.5])
        
        logger.log_local_state(timestamp, local_state, measured_state, control_input, 
                              measured_state is not None, 0.01)
        
        fleet_states = np.random.normal(0, 1, (4, 2))
        logger.log_fleet_states(timestamp, fleet_states)
    
    # Close logger (saves data)
    logger.close()
    
    print("Example data generated!")
