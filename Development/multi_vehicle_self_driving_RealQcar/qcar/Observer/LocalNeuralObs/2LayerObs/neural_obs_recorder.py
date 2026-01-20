"""
Neural Observer Data Recorder

Specialized data recorder for NeuralLuenbergerEstimator that saves:
- 6D estimated states (v_x, v_y, ψ, r, X, Y)
- Measurements from sensors
- Neural network tire residual outputs
- First-layer observer states (if enabled)
- Control inputs and training statistics

Records data to CSV files for offline plotting and analysis.
"""

import os
import csv
import time
from datetime import datetime
from typing import Dict, List, Optional
import numpy as np


class NeuralObsRecorder:
    """
    Data recorder for neural observer experiments.
    
    Records all relevant data from NeuralLuenbergerEstimator updates
    to a CSV file for later analysis and plotting.
    """
    
    # Column definitions for CSV output
    COLUMNS = [
        # Estimated 6D state
        'vx_est', 'vy_est', 'psi_est', 'r_est', 'X_est', 'Y_est',
        # Measurements
        'vx_meas', 'r_meas', 'psi_meas', 'X_meas', 'Y_meas', 'ay_meas',
        # Neural network outputs (tire residuals)
        'w_r', 'w_f',
        # First-layer observer state (UIO)
        'vx_uio', 'vy_uio', 'psi_uio', 'r_uio', 'X_uio', 'Y_uio',
        # Control inputs
        'steering', 'throttle',
        # Training statistics
        'loss', 'gps_valid',
    ]
    
    def __init__(self, output_dir: str = "neural_obs_recordings", name: str = "neural_obs"):
        """
        Initialize the neural observer recorder.
        
        Args:
            output_dir: Directory to save recordings
            name: Prefix for the recording file name
        """
        self.output_dir = output_dir
        self.name = name
        self.file = None
        self.writer = None
        self.recording = False
        self.start_time = 0.0
        self.record_count = 0
        self.filepath = None
    
    def start(self) -> str:
        """
        Start recording to a new CSV file.
        
        Returns:
            Path to the created CSV file
        """
        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filepath = os.path.join(self.output_dir, f"{self.name}_{timestamp}.csv")
        
        columns = ['time'] + self.COLUMNS
        self.file = open(self.filepath, 'w', newline='', buffering=8192)
        self.writer = csv.DictWriter(self.file, fieldnames=columns)
        self.writer.writeheader()
        
        self.recording = True
        self.start_time = time.time()
        self.record_count = 0
        
        return self.filepath
    
    def record(self,
               t: float,
               state_6d: np.ndarray,
               measurements: Dict[str, float],
               nn_outputs: np.ndarray,
               uio_state: Optional[np.ndarray] = None,
               steering: float = 0.0,
               throttle: float = 0.0,
               loss: float = 0.0,
               gps_valid: bool = False):
        """
        Record a single data sample.
        
        Args:
            t: Time in seconds
            state_6d: Estimated 6D state [vx, vy, psi, r, X, Y]
            measurements: Dict with measurement values
            nn_outputs: Neural network outputs [w_r, w_f]
            uio_state: Optional first-layer (UIO) state [vx, vy, psi, r, X, Y]
            steering: Steering command
            throttle: Throttle command
            loss: Training loss value
            gps_valid: Whether GPS was valid for this update
        """
        if not self.recording or self.writer is None:
            return
        
        try:
            row = {'time': t}
            
            # Estimated state
            row['vx_est'] = state_6d[0]
            row['vy_est'] = state_6d[1]
            row['psi_est'] = state_6d[2]
            row['r_est'] = state_6d[3]
            row['X_est'] = state_6d[4]
            row['Y_est'] = state_6d[5]
            
            # Measurements
            row['vx_meas'] = measurements.get('vx', 0.0)
            row['r_meas'] = measurements.get('r', 0.0)
            row['psi_meas'] = measurements.get('psi', 0.0)
            row['X_meas'] = measurements.get('X', 0.0)
            row['Y_meas'] = measurements.get('Y', 0.0)
            row['ay_meas'] = measurements.get('ay', 0.0)
            
            # Neural network outputs
            nn_flat = nn_outputs.flatten() if hasattr(nn_outputs, 'flatten') else nn_outputs
            row['w_r'] = nn_flat[0] if len(nn_flat) > 0 else 0.0
            row['w_f'] = nn_flat[1] if len(nn_flat) > 1 else 0.0
            
            # UIO state
            if uio_state is not None and len(uio_state) >= 6:
                row['vx_uio'] = uio_state[0]
                row['vy_uio'] = uio_state[1]
                row['psi_uio'] = uio_state[2]
                row['r_uio'] = uio_state[3]
                row['X_uio'] = uio_state[4]
                row['Y_uio'] = uio_state[5]
            else:
                row['vx_uio'] = 0.0
                row['vy_uio'] = 0.0
                row['psi_uio'] = 0.0
                row['r_uio'] = 0.0
                row['X_uio'] = 0.0
                row['Y_uio'] = 0.0
            
            # Control inputs
            row['steering'] = steering
            row['throttle'] = throttle
            
            # Training stats
            row['loss'] = loss
            row['gps_valid'] = 1 if gps_valid else 0
            
            self.writer.writerow(row)
            self.record_count += 1
            
        except Exception as e:
            # Silently fail to avoid disrupting observer operation
            pass
    
    def stop(self) -> int:
        """
        Stop recording and close the file.
        
        Returns:
            Number of records written
        """
        self.recording = False
        if self.file:
            try:
                self.file.flush()
                self.file.close()
            except Exception:
                pass
            self.file = None
            self.writer = None
        
        return self.record_count
    
    def is_recording(self) -> bool:
        """Check if currently recording."""
        return self.recording
    
    def get_filepath(self) -> Optional[str]:
        """Get the current recording file path."""
        return self.filepath
    
    def get_record_count(self) -> int:
        """Get the number of records written."""
        return self.record_count


def load_recorded_data(filepath: str) -> Dict[str, np.ndarray]:
    """
    Load recorded data from CSV file.
    
    Args:
        filepath: Path to the CSV file
        
    Returns:
        Dictionary with column names as keys and numpy arrays as values
    """
    import pandas as pd
    
    df = pd.read_csv(filepath)
    return {col: df[col].values for col in df.columns}
