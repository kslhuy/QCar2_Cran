"""
Basic Data Logger for Vehicle Observer System - Core functionality only
This version provides data logging without plotting dependencies.
"""

import os
import csv
import json
import numpy as np
import time
from datetime import datetime
from typing import Dict, List, Optional, Any
import threading
from scipy.io import savemat


class BasicObserverDataLogger:
    """
    Basic data logger for vehicle observer system - core functionality only.
    Saves data in CSV, JSON, and MATLAB formats without plotting dependencies.
    """
    
    def __init__(self, vehicle_id: int, fleet_size: int, log_dir: str = "data_logs"):
        """
        Initialize the basic data logger.
        
        Args:
            vehicle_id: ID of the vehicle
            fleet_size: Total number of vehicles in fleet
            log_dir: Directory to save data files
        """
        self.vehicle_id = vehicle_id
        self.fleet_size = fleet_size
        self.log_dir = log_dir
        
        # Create timestamp for this run
        self.run_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Create data directory
        self.data_dir = os.path.join(log_dir, f"run_{self.run_timestamp}")
        os.makedirs(self.data_dir, exist_ok=True)
        
        # Data storage
        self.local_state_data = []
        self.fleet_state_data = []
        
        # File handles for real-time CSV logging
        self.csv_files = {}
        self.csv_writers = {}
        self._init_csv_files()
        
        # Thread safety
        self.lock = threading.RLock()
        
        print(f"BasicDataLogger initialized for Vehicle {vehicle_id}")
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
            'control_steering', 'control_acceleration'
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
                       gps_available: bool, dt: float):
        """Log local state data."""
        with self.lock:
            # Store in memory
            data_entry = {
                'timestamp': timestamp,
                'vehicle_id': self.vehicle_id,
                'local_state': local_state.copy(),
                'measured_state': measured_state.copy() if measured_state is not None else None,
                'control_input': control_input.copy(),
                'gps_available': gps_available,
                'dt': dt
            }
            self.local_state_data.append(data_entry)
            
            # Write to CSV immediately
            gps_values = measured_state if measured_state is not None else [np.nan, np.nan, np.nan, np.nan]
            csv_row = [
                timestamp, self.vehicle_id,
                local_state[0], local_state[1], local_state[2], local_state[3],
                gps_available, dt,
                gps_values[0], gps_values[1], gps_values[2], gps_values[3],
                control_input[0], control_input[1]
            ]
            self.csv_writers['local'].writerow(csv_row)
            self.csv_files['local'].flush()
    
    def log_fleet_states(self, timestamp: float, fleet_states: np.ndarray):
        """Log fleet state estimates."""
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
        """Save all logged data to MATLAB .mat format."""
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
            print(f"MATLAB data saved to: {mat_path}")
            return mat_path
    
    def save_json_data(self, filename: Optional[str] = None):
        """Save all logged data to JSON format."""
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
            
            print(f"JSON data saved to: {json_path}")
            return json_path
    
    def close(self):
        """Close all file handles and save final data."""
        with self.lock:
            # Close CSV files
            for file_handle in self.csv_files.values():
                file_handle.close()
            
            # Save data in other formats
            matlab_path = self.save_matlab_data()
            json_path = self.save_json_data()
            
            print(f"BasicDataLogger for Vehicle {self.vehicle_id} closed.")
            print(f"Total local state samples: {len(self.local_state_data)}")
            print(f"Total fleet state samples: {len(self.fleet_state_data)}")
            print(f"Data saved in multiple formats:")
            print(f"  - CSV: {self.data_dir}")
            print(f"  - MATLAB: {matlab_path}")
            print(f"  - JSON: {json_path}")
    
    def __del__(self):
        """Destructor to ensure files are closed."""
        try:
            self.close()
        except:
            pass  # Ignore errors during cleanup


# Simplified usage example
if __name__ == "__main__":
    print("Basic Observer Data Logger Test")
    
    # Create logger for vehicle 0 in a fleet of 2
    logger = BasicObserverDataLogger(vehicle_id=0, fleet_size=2)
    
    # Simulate some data
    for i in range(20):
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
    
    print("Basic test completed!")
    print("Check the 'data_logs' directory for generated files.")
    print("Use the MATLAB script 'plot_observer_data.m' to visualize the data.")
