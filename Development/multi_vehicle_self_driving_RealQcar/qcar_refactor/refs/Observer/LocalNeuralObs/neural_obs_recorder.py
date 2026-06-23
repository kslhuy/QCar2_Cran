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

    def __init__(self, 
                 output_dir: str = "neural_obs_recordings", 
                 name: str = "neural_obs",
                 mode: RecordingMode = '2layer',
                 disturbance_mode: str = 'tire'):
        """
        Initialize the neural observer recorder.
        
        Args:
            output_dir: Directory to save recordings
            name: Prefix for the recording file name
            mode: Recording mode - '1layer' for first-layer only, '2layer' for full
            disturbance_mode: 'tire' (2D) or 'general' (3D)
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
        self.disturbance_mode = disturbance_mode
        self.file = None
        self.writer = None
        self.recording = False
        self.start_time = 0.0
        self.record_count = 0
        self.filepath = None
        
        # Define base column sets
        self.columns = self._get_columns()

    def _get_columns(self) -> List[str]:
        """Generate column list based on mode and disturbances"""
        cols = []
        
        # 1. State Estimates (Common)
        cols.extend(['vx_est', 'vy_est', 'psi_est', 'r_est', 'X_est', 'Y_est'])
        
        # 2. Unknown Inputs / Disturbances
        if self.disturbance_mode == 'general':
            # 3D General Disturbances
            dist_cols = ['d_vx', 'd_vy', 'd_r']
        else:
            # 2D Tire Residuals
            dist_cols = ['w_r', 'w_f']
            
        # Add NN outputs for 2-layer mode first (primary interest)
        if self.mode == '2layer':
            cols.extend([f"{c}_nn" for c in dist_cols])
            
        # 3. First-Layer / Single Layer components
        if self.mode == '2layer':
            cols.extend(['vx_uio', 'vy_uio', 'psi_uio', 'r_uio', 'X_uio', 'Y_uio'])
            # UIO 1st layer estimates
            cols.extend([f"{c}_uio" for c in dist_cols])
        else:
            # 1-layer mode: estimated unknowns are just the base names or _est?
            # Classically 1-layer output was saved as 'w_r', 'w_f'.
            cols.extend(dist_cols)

        # 4. True Ground Truths (Sim only)
        # Only record columns for the active disturbance mode
        if self.disturbance_mode == 'general':
            # General mode: 3D velocity disturbances
            cols.extend(['d_vx_true', 'd_vy_true', 'd_r_true'])
        else:
            # Tire mode: 2D tire residuals + detailed tire debugging info
            cols.extend(['w_r_true', 'w_f_true'])
            # Tire force debugging columns (only for tire mode)
            cols.extend(['Fyr_true', 'Fyf_true', 'Fyr_linear', 'Fyf_linear', 'alpha_r', 'alpha_f'])
            
            # Observer tire force estimates (both modes)
            cols.extend(['Fyr_est', 'Fyf_est', 'Fyr_linear_only', 'Fyf_linear_only', 'alpha_r_est', 'alpha_f_est'])
            
            # Layer 1 and Layer 2 tire force estimates (only in 2-layer mode)
            if self.mode == '2layer':
                cols.extend(['Fyr_layer_1', 'Fyf_layer_1', 'Fyr_linear_only_1', 'Fyf_linear_only_1', 'alpha_r_layer_1', 'alpha_f_layer_1'])
                cols.extend(['Fyr_layer_2', 'Fyf_layer_2', 'alpha_r_layer_2', 'alpha_f_layer_2'])
            
        # 5. Measurements & Inputs & Stats
        cols.extend(['vx_meas', 'r_meas', 'psi_meas', 'X_meas', 'Y_meas', 'ax_meas', 'ay_meas'])
        cols.extend(['steering', 'throttle', 'gps_valid'])
        
        if self.mode == '2layer':
            cols.extend(['loss'])
            
        # Ground truth states
        cols.extend(['vx_true', 'vy_true', 'psi_true', 'r_true', 'X_true', 'Y_true'])
        
        return cols
    
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
    
    
    def _fill_dist_to_row(self, row: Dict, values: Optional[np.ndarray], suffix: str = ""):
        """Helper to fill disturbance columns based on mode"""
        if values is None:
            vals = []
        else:
            vals = np.asarray(values).flatten()

        if self.disturbance_mode == 'general':
            # 3D: d_vx, d_vy, d_r
            names = ['d_vx', 'd_vy', 'd_r']
        else:
            # 2D: w_r, w_f
            names = ['w_r', 'w_f']
            
        full_names = [f"{n}{suffix}" for n in names]
        
        for i, name in enumerate(full_names):
            if i < len(vals):
                row[name] = float(vals[i])
            else:
                row[name] = 0.0

    def record_1layer(self,
                      t: float,
                      state_6d: np.ndarray,
                      unknown_input: np.ndarray,
                      measurements: Dict[str, float],
                      steering: float = 0.0,
                      throttle: float = 0.0,
                      gps_valid: bool = False,
                      true_unknown_input: Optional[np.ndarray] = None,
                      tire_info_true: Optional[Dict] = None,
                      tire_info_est: Optional[Dict] = None,
                      state_true_6d: Optional[np.ndarray] = None):
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
            tire_info_true: Ground truth tire info from plant (Fyr_true, Fyf_true, etc.)
            tire_info_est: Observer's tire force estimates (Fyr_est, Fyf_est, etc.)
            state_true_6d: Ground truth state [vx, vy, psi, r, X, Y]
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
            self._fill_dist_to_row(row, unknown_input, suffix="")
            
            # True unknown inputs (if available)
            self._fill_dist_to_row(row, true_unknown_input, suffix="_true")
            
            # Measurements
            row['vx_meas'] = measurements.get('vx', 0.0)
            row['r_meas'] = measurements.get('r', 0.0)
            row['psi_meas'] = measurements.get('psi', 0.0)
            row['X_meas'] = measurements.get('X', 0.0)
            row['Y_meas'] = measurements.get('Y', 0.0)
            row['ay_meas'] = measurements.get('ay', 0.0)
            row['ax_meas'] = measurements.get('ax', 0.0)
            
            # Control inputs
            row['steering'] = steering
            row['throttle'] = throttle
            
            # GPS status
            row['gps_valid'] = 1 if gps_valid else 0
            
            # Tire force info (true from plant)
            if self.disturbance_mode == 'tire':
                if tire_info_true is not None:
                    row['Fyr_true'] = tire_info_true.get('Fyr_true', 0.0)
                    row['Fyf_true'] = tire_info_true.get('Fyf_true', 0.0)
                    row['Fyr_linear'] = tire_info_true.get('Fyr_linear', 0.0)
                    row['Fyf_linear'] = tire_info_true.get('Fyf_linear', 0.0)
                    row['alpha_r'] = tire_info_true.get('alpha_r', 0.0)
                    row['alpha_f'] = tire_info_true.get('alpha_f', 0.0)
                
                # Observer's estimated tire forces
                if tire_info_est is not None:
                    row['Fyr_est'] = tire_info_est.get('Fyr_est', 0.0)
                    row['Fyf_est'] = tire_info_est.get('Fyf_est', 0.0)
                    row['Fyr_linear_only'] = tire_info_est.get('Fyr_linear_only', 0.0)
                    row['Fyf_linear_only'] = tire_info_est.get('Fyf_linear_only', 0.0)
                    row['alpha_r_est'] = tire_info_est.get('alpha_r', 0.0)
                    row['alpha_f_est'] = tire_info_est.get('alpha_f', 0.0)
            
            # Ground truth state
            if state_true_6d is not None:
                row['vx_true'] = state_true_6d[0]
                row['vy_true'] = state_true_6d[1]
                row['psi_true'] = state_true_6d[2]
                row['r_true'] = state_true_6d[3]
                row['X_true'] = state_true_6d[4]
                row['Y_true'] = state_true_6d[5]
            
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
                      state_true_6d: Optional[np.ndarray] = None,
                      unknown_input_true: Optional[np.ndarray] = None,
                      disturbances_true: Optional[np.ndarray] = None,
                      tire_info_true: Optional[Dict] = None,
                      tire_info_layer_1: Optional[dict] = None,
                      tire_info_layer_2: Optional[dict] = None):
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
            state_true_6d: Optional ground truth state [vx, vy, psi, r, X, Y]
            unknown_input_true: Optional ground truth tire residuals [w_r, w_f]
            disturbances_true: Optional ground truth general disturbances [d_vx, d_vy, d_r]
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
            row['ax_meas'] = measurements.get('ax', 0.0)
            
            # Neural network outputs
            self._fill_dist_to_row(row, nn_outputs, suffix="_nn")
            
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
            self._fill_dist_to_row(row, uio_unknown_input, suffix="_uio")
            
            # Control inputs
            row['steering'] = steering
            row['throttle'] = throttle
            
            # Training stats
            row['loss'] = loss
            row['gps_valid'] = 1 if gps_valid else 0

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
            
            # True unknown inputs (tire residuals)
            self._fill_dist_to_row(row, unknown_input_true, suffix="_true")
            

            
            # Tire force information (for debugging tire residual estimation)
            # F_true: actual forces from selected tire model (pacejka, etc.)
            # F_linear: reference forces from linear model (F = C * alpha)
            # Residual should be: w = F_true - F_linear
            if self.disturbance_mode == 'tire':
                if tire_info_true is not None:
                    row['Fyr_true'] = tire_info_true.get('Fyr_true', 0.0)
                    row['Fyf_true'] = tire_info_true.get('Fyf_true', 0.0)
                    row['Fyr_linear'] = tire_info_true.get('Fyr_linear', 0.0)
                    row['Fyf_linear'] = tire_info_true.get('Fyf_linear', 0.0)
                    row['alpha_r'] = tire_info_true.get('alpha_r', 0.0)
                    row['alpha_f'] = tire_info_true.get('alpha_f', 0.0)
                else:
                    row['Fyr_true'] = 0.0
                    row['Fyf_true'] = 0.0
                    row['Fyr_linear'] = 0.0
                    row['Fyf_linear'] = 0.0
                    row['alpha_r'] = 0.0
                    row['alpha_f'] = 0.0
                
                # Observer's combined tire force estimates (best available estimate)
                # In 2-layer mode, use Layer 2 estimates if available, else Layer 1
                best_est = tire_info_layer_2 if tire_info_layer_2 is not None else tire_info_layer_1
                if best_est is not None:
                    row['Fyr_est'] = best_est.get('Fyr_est', 0.0)
                    row['Fyf_est'] = best_est.get('Fyf_est', 0.0)
                    row['Fyr_linear_only'] = best_est.get('Fyr_linear_only', 0.0)
                    row['Fyf_linear_only'] = best_est.get('Fyf_linear_only', 0.0)
                    row['alpha_r_est'] = best_est.get('alpha_r', 0.0)
                    row['alpha_f_est'] = best_est.get('alpha_f', 0.0)
                else:
                    row['Fyr_est'] = 0.0
                    row['Fyf_est'] = 0.0
                    row['Fyr_linear_only'] = 0.0
                    row['Fyf_linear_only'] = 0.0
                    row['alpha_r_est'] = 0.0
                    row['alpha_f_est'] = 0.0
                
                # Layer 1 tire force estimates
                if tire_info_layer_1 is not None:
                    row['Fyr_layer_1'] = tire_info_layer_1.get('Fyr_est', tire_info_layer_1.get('Fyr_linear_est', 0.0))
                    row['Fyf_layer_1'] = tire_info_layer_1.get('Fyf_est', tire_info_layer_1.get('Fyf_linear_est', 0.0))
                    row['Fyr_linear_only_1'] = tire_info_layer_1.get('Fyr_linear_only', 0.0)
                    row['Fyf_linear_only_1'] = tire_info_layer_1.get('Fyf_linear_only', 0.0)
                    row['alpha_r_layer_1'] = tire_info_layer_1.get('alpha_r', 0.0)
                    row['alpha_f_layer_1'] = tire_info_layer_1.get('alpha_f', 0.0)
                else:
                    row['Fyr_layer_1'] = 0.0
                    row['Fyf_layer_1'] = 0.0
                    row['Fyr_linear_only_1'] = 0.0
                    row['Fyf_linear_only_1'] = 0.0
                    row['alpha_r_layer_1'] = 0.0
                    row['alpha_f_layer_1'] = 0.0
                
                # Layer 2 tire force estimates
                if tire_info_layer_2 is not None:
                    row['Fyr_layer_2'] = tire_info_layer_2.get('Fyr_est', tire_info_layer_2.get('Fyr_linear_est', 0.0))
                    row['Fyf_layer_2'] = tire_info_layer_2.get('Fyf_est', tire_info_layer_2.get('Fyf_linear_est', 0.0))
                    row['alpha_r_layer_2'] = tire_info_layer_2.get('alpha_r', 0.0)
                    row['alpha_f_layer_2'] = tire_info_layer_2.get('alpha_f', 0.0)
                else:
                    row['Fyr_layer_2'] = 0.0
                    row['Fyf_layer_2'] = 0.0
                    row['alpha_r_layer_2'] = 0.0
                    row['alpha_f_layer_2'] = 0.0
            else : 
                # True general disturbances (always record if available)
                if disturbances_true is not None:
                    # Manually fill strictly 3D disturbances
                    d_vals = np.asarray(disturbances_true).flatten()
                    row['d_vx_true'] = float(d_vals[0]) if len(d_vals) > 0 else 0.0
                    row['d_vy_true'] = float(d_vals[1]) if len(d_vals) > 1 else 0.0
                    row['d_r_true'] = float(d_vals[2]) if len(d_vals) > 2 else 0.0
                else:
                    row['d_vx_true'] = 0.0
                    row['d_vy_true'] = 0.0
                    row['d_r_true'] = 0.0
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
