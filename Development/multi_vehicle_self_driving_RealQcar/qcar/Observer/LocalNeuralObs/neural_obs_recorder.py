"""
Neural Observer Data Recorder

Unified data recorder for both 1-layer and 2-layer neural observer architectures.

Recording Modes:
- '1layer': Records first-layer observer data (qLPV, Differentiator-UIO, etc.)
    - 6D estimated states (v_x, v_y, ψ, r, X, Y)
    - Unknown input estimates (w_r, w_f - tire residuals)
    - Measurements and control inputs

- '2layer': Records full two-layer neural observer data
    - 6D estimated states from neural layer
    - Neural network tire residual outputs (learned)
    - First-layer observer states (UIO)
    - Training statistics (loss, GPS valid)

Records data to CSV files for offline plotting and analysis.
"""

import os
import csv
import time
from datetime import datetime
from typing import Dict, List, Optional, Literal
import numpy as np


RecordingMode = Literal['1layer', '2layer']


class NeuralObsRecorder:
    """
    Unified data recorder for neural observer experiments.
    
    Supports both 1-layer (first-layer only) and 2-layer (neural + first-layer)
    recording modes.
    """
    
    # Column definitions for 1-layer mode
    COLUMNS_1LAYER = [
        # Estimated 6D state from first-layer observer
        'vx_est', 'vy_est', 'psi_est', 'r_est', 'X_est', 'Y_est',
        # Unknown input estimates (tire residuals from first-layer)
        'w_r', 'w_f',
        # True unknown inputs (for ground truth comparison in sims)
        'w_r_true', 'w_f_true',
        # Measurements
        'vx_meas', 'r_meas', 'psi_meas', 'X_meas', 'Y_meas', 'ay_meas',
        # Control inputs
        'steering', 'throttle',
        # GPS status
        'gps_valid',
    ]
    
    # Column definitions for 2-layer mode (includes everything from 1-layer plus NN/training data)
    COLUMNS_2LAYER = [
        # Estimated 6D state from neural layer (second layer)
        'vx_est', 'vy_est', 'psi_est', 'r_est', 'X_est', 'Y_est',
        # Measurements
        'vx_meas', 'r_meas', 'psi_meas', 'X_meas', 'Y_meas', 'ay_meas',
        # Neural network outputs (learned tire residuals)
        'w_r_nn', 'w_f_nn',
        # First-layer observer state (UIO/qLPV)
        'vx_uio', 'vy_uio', 'psi_uio', 'r_uio', 'X_uio', 'Y_uio',
        # First-layer unknown input estimates
        'w_r_uio', 'w_f_uio',
        # Control inputs
        'steering', 'throttle',
        # Training statistics
        'loss', 'gps_valid',
        # Observer Gain and Innovation (for debugging)
        'L_psi', 'innov_psi',
        # True Ground Truth (Sim only)
        'vx_true', 'vy_true', 'psi_true', 'r_true', 'X_true', 'Y_true',
        'w_r_true', 'w_f_true',
        # True tire forces (from selected tire model: pacejka, dynamic_linear, etc.)
        'Fyr_true', 'Fyf_true',
        # Linear tire forces (reference model: F = C * alpha)
        'Fyr_linear', 'Fyf_linear',
        # Slip angles (for verification)
        'alpha_r', 'alpha_f',
    ]
    
    def __init__(self, 
                 output_dir: str = "neural_obs_recordings", 
                 name: str = "neural_obs",
                 mode: RecordingMode = '2layer'):
        """
        Initialize the neural observer recorder.
        
        Args:
            output_dir: Directory to save recordings
            name: Prefix for the recording file name
            mode: Recording mode - '1layer' for first-layer only, '2layer' for full
        """
        if os.path.isabs(output_dir):
            base_output_dir = output_dir
        else:
            # Resolve relative paths relative to this script's directory (LocalNeuralObs)
            # This ensures recordings are saved in the component folder, not the CWD (GUI)
            base_dir = os.path.dirname(os.path.abspath(__file__))
            base_output_dir = os.path.join(base_dir, output_dir)
            
        # Add subfolder for layer type
        if mode == '1layer':
            self.output_dir = os.path.join(base_output_dir, "Layer1")
        else:
            self.output_dir = os.path.join(base_output_dir, "Layer2")
            
        self.name = name
        self.mode = mode
        self.file = None
        self.writer = None
        self.recording = False
        self.start_time = 0.0
        self.record_count = 0
        self.filepath = None
        
        # Select columns based on mode
        self.columns = self.COLUMNS_1LAYER if mode == '1layer' else self.COLUMNS_2LAYER
    
    def start(self, filename: Optional[str] = None, append: bool = False) -> str:
        """
        Start recording to a CSV file.
        
        Args:
            filename: Optional specific filename to save to. If None, generates timestamped name.
            append: If True and file exists, appends to it. If False, overwrites.
            
        Returns:
            Path to the created/opened CSV file
        """
        os.makedirs(self.output_dir, exist_ok=True)
        
        if filename:
            if not filename.endswith('.csv'):
                filename += '.csv'
            self.filepath = os.path.join(self.output_dir, filename)
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            mode_suffix = f"_{self.mode}"
            self.filepath = os.path.join(self.output_dir, f"{self.name}{mode_suffix}_{timestamp}.csv")
        
        # Determine mode and whether to write header
        file_mode = 'a' if append else 'w'
        write_header = True
        
        if append and os.path.exists(self.filepath):
            # Check if file has content to decide on header
            if os.path.getsize(self.filepath) > 0:
                write_header = False
                
        self.file = open(self.filepath, file_mode, newline='', buffering=8192)
        
        columns = ['time'] + self.columns
        self.writer = csv.DictWriter(self.file, fieldnames=columns)
        
        if write_header:
            self.writer.writeheader()
        
        self.recording = True
        self.start_time = time.time()
        # Reset count only if new file, otherwise we might want to track total (but here we track session count)
        self.record_count = 0 
        
        return self.filepath
    
    def record_1layer(self,
                      t: float,
                      state_6d: np.ndarray,
                      unknown_input: np.ndarray,
                      measurements: Dict[str, float],
                      steering: float = 0.0,
                      throttle: float = 0.0,
                      gps_valid: bool = False,
                      true_unknown_input: Optional[np.ndarray] = None):
        """
        Record a single data sample for 1-layer observer.
        
        Args:
            t: Time in seconds
            state_6d: Estimated 6D state [vx, vy, psi, r, X, Y]
            unknown_input: Unknown input estimate [w_r, w_f] (tire residuals)
            measurements: Dict with measurement values
            steering: Steering command
            throttle: Throttle command
            gps_valid: Whether GPS was valid for this update
            true_unknown_input: Ground truth [w_r, w_f] (optional, for sim verification)
        """
        if not self.recording or self.writer is None or self.mode != '1layer':
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
            
            # Unknown input estimates (first-layer tire residuals)
            w = np.asarray(unknown_input).flatten()
            row['w_r'] = w[0] if len(w) > 0 else 0.0
            row['w_f'] = w[1] if len(w) > 1 else 0.0
            
            # True unknown inputs (if available)
            if true_unknown_input is not None:
                w_true = np.asarray(true_unknown_input).flatten()
                row['w_r_true'] = w_true[0] if len(w_true) > 0 else 0.0
                row['w_f_true'] = w_true[1] if len(w_true) > 1 else 0.0
            else:
                 row['w_r_true'] = 0.0
                 row['w_f_true'] = 0.0
            
            # Measurements
            row['vx_meas'] = measurements.get('vx', 0.0)
            row['r_meas'] = measurements.get('r', 0.0)
            row['psi_meas'] = measurements.get('psi', 0.0)
            row['X_meas'] = measurements.get('X', 0.0)
            row['Y_meas'] = measurements.get('Y', 0.0)
            row['ay_meas'] = measurements.get('ay', 0.0)
            
            # Control inputs
            row['steering'] = steering
            row['throttle'] = throttle
            
            # GPS status
            row['gps_valid'] = 1 if gps_valid else 0
            
            self.writer.writerow(row)
            self.record_count += 1
            
        except Exception as e:
            # Silently fail to avoid disrupting observer operation
            pass
    
    def record_2layer(self,
                      t: float,
                      state_6d: np.ndarray,
                      measurements: Dict[str, float],
                      nn_outputs: np.ndarray,
                      uio_state: Optional[np.ndarray] = None,
                      uio_unknown_input: Optional[np.ndarray] = None,
                      steering: float = 0.0,
                      throttle: float = 0.0,
                      loss: float = 0.0,
                      gps_valid: bool = False,
                      L_psi: float = 0.0,
                      innov_psi: float = 0.0,
                      state_true_6d: Optional[np.ndarray] = None,
                      unknown_input_true: Optional[np.ndarray] = None,
                      tire_info: Optional[Dict] = None):
        """
        Record a single data sample for 2-layer neural observer.
        
        Args:
            t: Time in seconds
            state_6d: Estimated 6D state [vx, vy, psi, r, X, Y] from neural layer
            measurements: Dict with measurement values
            nn_outputs: Neural network outputs [w_r, w_f] (learned tire residuals)
            uio_state: Optional first-layer (UIO) state [vx, vy, psi, r, X, Y]
            uio_unknown_input: Optional first-layer unknown input estimate [w_r, w_f]
            steering: Steering command
            throttle: Throttle command
            loss: Training loss value
            gps_valid: Whether GPS was valid for this update
            L_psi: Observer gain for yaw state from yaw measurement
            innov_psi: Yaw innovation (measurement error)
            state_true_6d: Optional ground truth state [vx, vy, psi, r, X, Y]
            unknown_input_true: Optional ground truth unknown inputs [w_r, w_f]
            tire_info: Optional dict with tire force data for debugging:
                       {'Fyr_true', 'Fyf_true', 'Fyr_linear', 'Fyf_linear', 'alpha_r', 'alpha_f'}
        """
        if not self.recording or self.writer is None or self.mode != '2layer':
            return
        
        try:
            row = {'time': t}
            
            # Estimated state (neural layer)
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
            row['w_r_nn'] = nn_flat[0] if len(nn_flat) > 0 else 0.0
            row['w_f_nn'] = nn_flat[1] if len(nn_flat) > 1 else 0.0
            
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
            
            # UIO unknown input estimates
            if uio_unknown_input is not None:
                uio_w = np.asarray(uio_unknown_input).flatten()
                row['w_r_uio'] = uio_w[0] if len(uio_w) > 0 else 0.0
                row['w_f_uio'] = uio_w[1] if len(uio_w) > 1 else 0.0
            else:
                row['w_r_uio'] = 0.0
                row['w_f_uio'] = 0.0
            
            # Control inputs
            row['steering'] = steering
            row['throttle'] = throttle
            
            # Training stats
            row['loss'] = loss
            row['gps_valid'] = 1 if gps_valid else 0
            
            # Observer Gain and Innovation (for debugging)
            row['L_psi'] = L_psi
            row['innov_psi'] = innov_psi

            # Ground Truth
            if state_true_6d is not None:
                 row['vx_true'] = state_true_6d[0]
                 row['vy_true'] = state_true_6d[1]
                 row['psi_true'] = state_true_6d[2]
                 row['r_true'] = state_true_6d[3]
                 row['X_true'] = state_true_6d[4]
                 row['Y_true'] = state_true_6d[5]
            else:
                 row['vx_true'] = 0.0
                 row['vy_true'] = 0.0
                 row['psi_true'] = 0.0
                 row['r_true'] = 0.0
                 row['X_true'] = 0.0
                 row['Y_true'] = 0.0

            if unknown_input_true is not None:
                 w_true_flat = unknown_input_true.flatten()
                 row['w_r_true'] = w_true_flat[0] if len(w_true_flat) > 0 else 0.0
                 row['w_f_true'] = w_true_flat[1] if len(w_true_flat) > 1 else 0.0
            else:
                 row['w_r_true'] = 0.0
                 row['w_f_true'] = 0.0
            
            # Tire force information (for debugging tire residual estimation)
            # F_true: actual forces from selected tire model (pacejka, etc.)
            # F_linear: reference forces from linear model (F = C * alpha)
            # Residual should be: w = F_true - F_linear
            if tire_info is not None:
                row['Fyr_true'] = tire_info.get('Fyr_true', 0.0)
                row['Fyf_true'] = tire_info.get('Fyf_true', 0.0)
                row['Fyr_linear'] = tire_info.get('Fyr_linear', 0.0)
                row['Fyf_linear'] = tire_info.get('Fyf_linear', 0.0)
                row['alpha_r'] = tire_info.get('alpha_r', 0.0)
                row['alpha_f'] = tire_info.get('alpha_f', 0.0)
            else:
                row['Fyr_true'] = 0.0
                row['Fyf_true'] = 0.0
                row['Fyr_linear'] = 0.0
                row['Fyf_linear'] = 0.0
                row['alpha_r'] = 0.0
                row['alpha_f'] = 0.0
            
            self.writer.writerow(row)
            self.record_count += 1
            
        except Exception as e:
            # Silently fail to avoid disrupting observer operation
            pass
    
    def record(self,
               t: float,
               state_6d: np.ndarray,
               measurements: Dict[str, float],
               nn_outputs: np.ndarray,
               uio_state: Optional[np.ndarray] = None,
               steering: float = 0.0,
               throttle: float = 0.0,
               loss: float = 0.0,
               gps_valid: bool = False,
               L_psi: float = 0.0,
               innov_psi: float = 0.0,
               state_true_6d: Optional[np.ndarray] = None,
               unknown_input_true: Optional[np.ndarray] = None):
        """
        Legacy record method for backward compatibility with 2-layer mode.
        
        Args: See record_2layer for parameter descriptions.
        """
        self.record_2layer(
            t=t,
            state_6d=state_6d,
            measurements=measurements,
            nn_outputs=nn_outputs,
            uio_state=uio_state,
            uio_unknown_input=None,
            steering=steering,
            throttle=throttle,
            loss=loss,
            gps_valid=gps_valid,
            L_psi=L_psi,
            innov_psi=innov_psi,
            state_true_6d=state_true_6d,
            unknown_input_true=unknown_input_true
        )
    
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
    
    def get_mode(self) -> RecordingMode:
        """Get the current recording mode."""
        return self.mode


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


def detect_recording_mode(filepath: str) -> RecordingMode:
    """
    Detect the recording mode from a CSV file.
    
    Args:
        filepath: Path to the CSV file
        
    Returns:
        Recording mode ('1layer' or '2layer')
    """
    import pandas as pd
    
    df = pd.read_csv(filepath, nrows=0)  # Just read headers
    columns = set(df.columns)
    
    # 2-layer mode has NN outputs and loss
    if 'w_r_nn' in columns or 'loss' in columns:
        return '2layer'
    else:
        return '1layer'
