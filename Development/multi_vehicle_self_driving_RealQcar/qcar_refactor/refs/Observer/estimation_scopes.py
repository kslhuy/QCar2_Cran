"""
Estimation Scopes - Modular Visualization System for State Estimators

This module provides a pluggable, non-blocking visualization system for 
local and fleet state estimators using MultiScope from pal.utilities.scope.

Features:
- Modular presets for different signal groups
- Non-blocking queue-based updates
- Real-time and recorded data support
- Easy extension via preset pattern

Usage:
    from Observer.estimation_scopes import EstimationScopeManager, LocalStatePreset
    
    scope_mgr = EstimationScopeManager(fps=30, time_window=60.0)
    scope_mgr.add_preset(LocalStatePreset())
    scope_mgr.start()
    
    # In control loop:
    scope_mgr.sample(t, data)
    
    # When done:
    scope_mgr.stop()
"""

import numpy as np
import time
import threading
import queue
import csv
import os
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any, Callable
from datetime import datetime

# try:
#     from pal.utilities.scope import MultiScope
#     MULTISCOPE_AVAILABLE = True
# except ImportError:
#     MULTISCOPE_AVAILABLE = False
#     print("[estimation_scopes] Warning: MultiScope not available. Visualization disabled.")


# ==============================================================================
# Configuration Classes
# ==============================================================================

@dataclass
class AxisConfig:
    """Configuration for a single axis in a scope."""
    row: int
    col: int
    y_label: str
    y_lim: tuple = (-1.0, 1.0)
    x_label: str = ""
    time_window: Optional[float] = None  # Uses default if None
    signals: List[Dict[str, Any]] = field(default_factory=list)
    is_xy: bool = False  # True for XY plots
    row_span: int = 1
    col_span: int = 1
    x_lim: tuple = None  # For XY plots


@dataclass
class ScopeConfig:
    """Configuration for a MultiScope instance."""
    title: str
    rows: int = 3
    cols: int = 1
    fps: int = 30
    time_window: float = 60.0
    enabled: bool = True
    axes: List[AxisConfig] = field(default_factory=list)


# ==============================================================================
# Base Preset Class
# ==============================================================================

class EstimationScopePreset(ABC):
    """
    Abstract base class for scope presets.
    
    Subclasses define how to configure a scope and sample data.
    """
    
    def __init__(self, name: str, enabled: bool = True):
        self.name = name
        self.enabled = enabled
        self.scope: Optional['MultiScope'] = None
        self.config: Optional[ScopeConfig] = None
    
    @abstractmethod
    def get_config(self) -> ScopeConfig:
        """Return scope configuration."""
        pass
    
    @abstractmethod
    def sample(self, t: float, data: dict) -> None:
        """Sample data and update scope signals."""
        pass
    
    def initialize(self, fps: int = 30, time_window: float = 60.0) -> bool:
        """Initialize the scope with this preset's configuration."""
        if not MULTISCOPE_AVAILABLE:
            return False
        
        try:
            self.config = self.get_config()
            self.config.fps = fps
            self.config.time_window = time_window
            
            self.scope = MultiScope(
                rows=self.config.rows,
                cols=self.config.cols,
                title=self.config.title,
                fps=self.config.fps
            )
            
            # Configure all axes
            for i, axis_cfg in enumerate(self.config.axes):
                tw = axis_cfg.time_window or self.config.time_window
                
                if axis_cfg.is_xy:
                    self.scope.addXYAxis(
                        row=axis_cfg.row,
                        col=axis_cfg.col,
                        rowSpan=axis_cfg.row_span,
                        xLabel=axis_cfg.x_label,
                        yLabel=axis_cfg.y_label,
                        xLim=axis_cfg.x_lim,
                        yLim=axis_cfg.y_lim
                    )
                else:
                    self.scope.addAxis(
                        row=axis_cfg.row,
                        col=axis_cfg.col,
                        timeWindow=tw,
                        yLabel=axis_cfg.y_label,
                        yLim=axis_cfg.y_lim,
                        xLabel=axis_cfg.x_label if axis_cfg.x_label else None
                    )
                
                # Attach signals
                for sig in axis_cfg.signals:
                    self.scope.axes[i].attachSignal(
                        name=sig.get('name', ''),
                        width=sig.get('width', 1)
                    )
            
            return True
            
        except Exception as e:
            print(f"[{self.name}] Failed to initialize scope: {e}")
            return False
    
    def close(self):
        """Close the scope."""
        self.scope = None


# ==============================================================================
# Local Estimator Presets
# ==============================================================================

class LocalStatePreset(EstimationScopePreset):
    """
    Visualize local state estimation: x, y, theta, velocity.
    
    Data keys expected:
        - x, y, theta, velocity (estimated)
        - x_gps, y_gps, theta_gps (GPS reference, optional)
    """
    
    def __init__(self, enabled: bool = True):
        super().__init__("LocalState", enabled)
    
    def get_config(self) -> ScopeConfig:
        return ScopeConfig(
            title="Local State Estimation",
            rows=4,
            cols=1,
            axes=[
                AxisConfig(
                    row=0, col=0,
                    y_label="X Position [m]",
                    y_lim=(-5, 5),
                    signals=[
                        {"name": "x_est", "width": 2},
                        {"name": "x_gps", "width": 1}
                    ]
                ),
                AxisConfig(
                    row=1, col=0,
                    y_label="Y Position [m]",
                    y_lim=(-5, 5),
                    signals=[
                        {"name": "y_est", "width": 2},
                        {"name": "y_gps", "width": 1}
                    ]
                ),
                AxisConfig(
                    row=2, col=0,
                    y_label="Heading [rad]",
                    y_lim=(-3.5, 3.5),
                    signals=[
                        {"name": "theta_est", "width": 2},
                        {"name": "theta_gps", "width": 1}
                    ]
                ),
                AxisConfig(
                    row=3, col=0,
                    y_label="Velocity [m/s]",
                    y_lim=(0, 2),
                    x_label="Time [s]",
                    signals=[
                        {"name": "v_est", "width": 2},
                        {"name": "v_ref", "width": 1}
                    ]
                ),
            ]
        )
    
    def sample(self, t: float, data: dict) -> None:
        if self.scope is None:
            return
        
        try:
            # Position X
            x_est = data.get('x', 0.0)
            x_gps = data.get('x_gps', x_est)
            self.scope.axes[0].sample(t, [x_est, x_gps])
            
            # Position Y
            y_est = data.get('y', 0.0)
            y_gps = data.get('y_gps', y_est)
            self.scope.axes[1].sample(t, [y_est, y_gps])
            
            # Heading
            theta_est = data.get('theta', 0.0)
            theta_gps = data.get('theta_gps', theta_est)
            self.scope.axes[2].sample(t, [theta_est, theta_gps])
            
            # Velocity
            v_est = data.get('velocity', 0.0)
            v_ref = data.get('v_ref', 0.0)
            self.scope.axes[3].sample(t, [v_est, v_ref])
            
        except Exception as e:
            pass  # Non-blocking, ignore errors


class LocalEstimationErrorPreset(EstimationScopePreset):
    """
    Visualize estimation errors: GPS vs estimated difference.
    
    Data keys expected:
        - x, y, theta, velocity (estimated)
        - x_gps, y_gps, theta_gps (GPS reference)
    """
    
    def __init__(self, enabled: bool = True):
        super().__init__("LocalError", enabled)
    
    def get_config(self) -> ScopeConfig:
        return ScopeConfig(
            title="Local Estimation Error",
            rows=3,
            cols=1,
            axes=[
                AxisConfig(
                    row=0, col=0,
                    y_label="Position Error [m]",
                    y_lim=(-1, 1),
                    signals=[
                        {"name": "x_err", "width": 2},
                        {"name": "y_err", "width": 2}
                    ]
                ),
                AxisConfig(
                    row=1, col=0,
                    y_label="Heading Error [rad]",
                    y_lim=(-0.5, 0.5),
                    signals=[{"name": "theta_err", "width": 2}]
                ),
                AxisConfig(
                    row=2, col=0,
                    y_label="Acceleration [m/s²]",
                    y_lim=(-2, 2),
                    x_label="Time [s]",
                    signals=[{"name": "accel", "width": 2}]
                ),
            ]
        )
    
    def sample(self, t: float, data: dict) -> None:
        if self.scope is None:
            return
        
        try:
            # Position error
            x_err = data.get('x', 0.0) - data.get('x_gps', data.get('x', 0.0))
            y_err = data.get('y', 0.0) - data.get('y_gps', data.get('y', 0.0))
            self.scope.axes[0].sample(t, [x_err, y_err])
            
            # Heading error
            theta_err = data.get('theta', 0.0) - data.get('theta_gps', data.get('theta', 0.0))
            self.scope.axes[1].sample(t, [theta_err])
            
            # Acceleration
            accel = data.get('acceleration', 0.0)
            self.scope.axes[2].sample(t, [accel])
            
        except Exception as e:
            pass


class LocalControlPreset(EstimationScopePreset):
    """
    Visualize control signals: steering, throttle.
    
    Data keys expected:
        - steering, throttle
        - velocity, v_ref
    """
    
    def __init__(self, enabled: bool = True):
        super().__init__("LocalControl", enabled)
    
    def get_config(self) -> ScopeConfig:
        return ScopeConfig(
            title="Control Signals",
            rows=3,
            cols=1,
            axes=[
                AxisConfig(
                    row=0, col=0,
                    y_label="Velocity [m/s]",
                    y_lim=(0, 1.5),
                    signals=[
                        {"name": "v_meas", "width": 2},
                        {"name": "v_ref", "width": 1}
                    ]
                ),
                AxisConfig(
                    row=1, col=0,
                    y_label="Throttle [%]",
                    y_lim=(-0.3, 0.3),
                    signals=[{"name": "throttle", "width": 2}]
                ),
                AxisConfig(
                    row=2, col=0,
                    y_label="Steering [rad]",
                    y_lim=(-0.6, 0.6),
                    x_label="Time [s]",
                    signals=[{"name": "steering", "width": 2}]
                ),
            ]
        )
    
    def sample(self, t: float, data: dict) -> None:
        if self.scope is None:
            return
        
        try:
            v = data.get('velocity', 0.0)
            v_ref = data.get('v_ref', 0.0)
            self.scope.axes[0].sample(t, [v, v_ref])
            
            throttle = data.get('throttle', 0.0)
            self.scope.axes[1].sample(t, [throttle])
            
            steering = data.get('steering', 0.0)
            self.scope.axes[2].sample(t, [steering])
            
        except Exception as e:
            pass


# ==============================================================================
# Fleet Estimator Presets
# ==============================================================================

class FleetPositionPreset(EstimationScopePreset):
    """
    Visualize fleet vehicle positions (X-Y plot).
    
    Data keys expected:
        - fleet_states: np.ndarray of shape (state_dim, num_vehicles)
          where state_dim >= 4 [x, y, theta, v, ...]
    """
    
    def __init__(self, max_vehicles: int = 5, enabled: bool = True):
        super().__init__("FleetPosition", enabled)
        self.max_vehicles = max_vehicles
    
    def get_config(self) -> ScopeConfig:
        return ScopeConfig(
            title="Fleet Vehicle Positions",
            rows=1,
            cols=1,
            axes=[
                AxisConfig(
                    row=0, col=0,
                    y_label="Y Position [m]",
                    x_label="X Position [m]",
                    y_lim=(-3, 3),
                    x_lim=(-3, 3),
                    is_xy=True,
                    signals=[
                        {"name": f"Vehicle_{i}", "width": 2}
                        for i in range(self.max_vehicles)
                    ]
                ),
            ]
        )
    
    def sample(self, t: float, data: dict) -> None:
        if self.scope is None:
            return
        
        try:
            fleet_states = data.get('fleet_states')
            if fleet_states is None:
                return
            
            num_vehicles = min(fleet_states.shape[1], self.max_vehicles)
            
            for i in range(num_vehicles):
                x = fleet_states[0, i]
                y = fleet_states[1, i]
                self.scope.axes[0].sample(t, [[x, y]])
            
        except Exception as e:
            pass


class FleetStatePreset(EstimationScopePreset):
    """
    Visualize fleet state estimates over time.
    
    Data keys expected:
        - fleet_states: np.ndarray of shape (state_dim, num_vehicles)
        - vehicle_id: int (own vehicle ID)
    """
    
    def __init__(self, max_vehicles: int = 3, enabled: bool = True):
        super().__init__("FleetState", enabled)
        self.max_vehicles = max_vehicles
    
    def get_config(self) -> ScopeConfig:
        signals_per_axis = [
            {"name": f"V{i}", "width": 2} for i in range(self.max_vehicles)
        ]
        
        return ScopeConfig(
            title="Fleet State Estimates",
            rows=4,
            cols=1,
            axes=[
                AxisConfig(
                    row=0, col=0,
                    y_label="X Position [m]",
                    y_lim=(-5, 5),
                    signals=signals_per_axis.copy()
                ),
                AxisConfig(
                    row=1, col=0,
                    y_label="Y Position [m]",
                    y_lim=(-5, 5),
                    signals=signals_per_axis.copy()
                ),
                AxisConfig(
                    row=2, col=0,
                    y_label="Heading [rad]",
                    y_lim=(-3.5, 3.5),
                    signals=signals_per_axis.copy()
                ),
                AxisConfig(
                    row=3, col=0,
                    y_label="Velocity [m/s]",
                    y_lim=(0, 2),
                    x_label="Time [s]",
                    signals=signals_per_axis.copy()
                ),
            ]
        )
    
    def sample(self, t: float, data: dict) -> None:
        if self.scope is None:
            return
        
        try:
            fleet_states = data.get('fleet_states')
            if fleet_states is None:
                return
            
            num_vehicles = min(fleet_states.shape[1], self.max_vehicles)
            
            # X positions
            x_vals = [fleet_states[0, i] for i in range(num_vehicles)]
            self.scope.axes[0].sample(t, x_vals)
            
            # Y positions
            y_vals = [fleet_states[1, i] for i in range(num_vehicles)]
            self.scope.axes[1].sample(t, y_vals)
            
            # Headings
            theta_vals = [fleet_states[2, i] for i in range(num_vehicles)]
            self.scope.axes[2].sample(t, theta_vals)
            
            # Velocities
            v_vals = [fleet_states[3, i] for i in range(num_vehicles)]
            self.scope.axes[3].sample(t, v_vals)
            
        except Exception as e:
            pass


class FleetConsensusPreset(EstimationScopePreset):
    """
    Visualize fleet consensus metrics.
    
    Data keys expected:
        - consensus_error: float (average error across fleet)
        - trust_scores: dict {vehicle_id: score}
        - fleet_size: int
    """
    
    def __init__(self, max_vehicles: int = 5, enabled: bool = True):
        super().__init__("FleetConsensus", enabled)
        self.max_vehicles = max_vehicles
    
    def get_config(self) -> ScopeConfig:
        return ScopeConfig(
            title="Fleet Consensus Metrics",
            rows=2,
            cols=1,
            axes=[
                AxisConfig(
                    row=0, col=0,
                    y_label="Consensus Error [m]",
                    y_lim=(0, 1),
                    signals=[{"name": "error", "width": 2}]
                ),
                AxisConfig(
                    row=1, col=0,
                    y_label="Trust Scores",
                    y_lim=(0, 1.2),
                    x_label="Time [s]",
                    signals=[
                        {"name": f"trust_V{i}", "width": 1}
                        for i in range(self.max_vehicles)
                    ]
                ),
            ]
        )
    
    def sample(self, t: float, data: dict) -> None:
        if self.scope is None:
            return
        
        try:
            # Consensus error
            error = data.get('consensus_error', 0.0)
            self.scope.axes[0].sample(t, [error])
            
            # Trust scores
            trust_scores = data.get('trust_scores', {})
            scores = []
            for i in range(self.max_vehicles):
                score = trust_scores.get(i, 0.5)
                scores.append(score)
            self.scope.axes[1].sample(t, scores)
            
        except Exception as e:
            pass


# ==============================================================================
# Distributed Luenberger Observer Preset
# ==============================================================================

class DistributedLuenbergerPreset(EstimationScopePreset):
    """
    Visualize DistributedLuenbergerEstimator internal signals.
    
    Data keys expected (from estimator.get_debug_data()):
        - x_vec_before: np.ndarray [3*observer_size] - state before update
        - x_vec_after: np.ndarray [3*observer_size] - state after update
        - dynamics_term: np.ndarray [3*observer_size]
        - measurement_term: np.ndarray [3*observer_size]
        - consensus_term: np.ndarray [3*observer_size]
        - measurement_error: np.ndarray [2]
        - neighbor_count: int
        - consensus_norm: float
    """
    
    def __init__(self, observer_size: int = 3, enabled: bool = True):
        super().__init__("DistributedLuenberger", enabled)
        self.observer_size = observer_size
    
    def get_config(self) -> ScopeConfig:
        # Create signals for each follower vehicle
        position_signals = [{"name": f"p{i+1}", "width": 2} for i in range(self.observer_size)]
        velocity_signals = [{"name": f"v{i+1}", "width": 2} for i in range(self.observer_size)]
        term_signals = [
            {"name": "dynamics", "width": 2},
            {"name": "measurement", "width": 2},
            {"name": "consensus", "width": 1}
        ]
        
        return ScopeConfig(
            title="Distributed Luenberger Observer",
            rows=4,
            cols=1,
            axes=[
                # Row 0: Relative position estimates
                AxisConfig(
                    row=0, col=0,
                    y_label="Rel. Position [m]",
                    y_lim=(-5, 10),
                    signals=position_signals
                ),
                # Row 1: Relative velocity estimates  
                AxisConfig(
                    row=1, col=0,
                    y_label="Rel. Velocity [m/s]",
                    y_lim=(-1, 1),
                    signals=velocity_signals
                ),
                # Row 2: Observer terms comparison (for vehicle 1)
                AxisConfig(
                    row=2, col=0,
                    y_label="Observer Terms",
                    y_lim=(-2, 2),
                    signals=term_signals
                ),
                # Row 3: Measurement error
                AxisConfig(
                    row=3, col=0,
                    y_label="Meas. Error",
                    y_lim=(-1, 1),
                    x_label="Time [s]",
                    signals=[
                        {"name": "pos_err", "width": 2},
                        {"name": "vel_err", "width": 2}
                    ]
                ),
            ]
        )
    
    def sample(self, t: float, data: dict) -> None:
        if self.scope is None:
            return
        
        try:
            # Get state vectors
            x_vec = data.get('x_vec_after', data.get('x_vec_before', None))
            if x_vec is None:
                return
            
            # Row 0: Relative position estimates (index 0 for each vehicle)
            positions = []
            for i in range(self.observer_size):
                idx = i * 3  # Each vehicle has 3 states: p, v, a
                pos = x_vec[idx] if idx < len(x_vec) else 0.0
                positions.append(pos)
            self.scope.axes[0].sample(t, positions)
            
            # Row 1: Relative velocity estimates (index 1 for each vehicle)
            velocities = []
            for i in range(self.observer_size):
                idx = i * 3 + 1
                vel = x_vec[idx] if idx < len(x_vec) else 0.0
                velocities.append(vel)
            self.scope.axes[1].sample(t, velocities)
            
            # Row 2: Observer terms (just for vehicle 1 to avoid clutter)
            dynamics = data.get('dynamics_term', np.zeros(3))
            measurement = data.get('measurement_term', np.zeros(3))
            consensus = data.get('consensus_term', np.zeros(3))
            
            # Use position component (index 0) for visualization
            dyn_val = dynamics[0] if len(dynamics) > 0 else 0.0
            meas_val = measurement[0] if len(measurement) > 0 else 0.0
            cons_val = consensus[0] if len(consensus) > 0 else 0.0
            self.scope.axes[2].sample(t, [dyn_val, meas_val, cons_val])
            
            # Row 3: Measurement error
            meas_err = data.get('measurement_error', np.zeros(2))
            pos_err = meas_err[0] if len(meas_err) > 0 else 0.0
            vel_err = meas_err[1] if len(meas_err) > 1 else 0.0
            self.scope.axes[3].sample(t, [pos_err, vel_err])
            
        except Exception as e:
            pass  # Non-blocking, ignore errors


# ==============================================================================
# Data Recording
# ==============================================================================

class ScopeDataRecorder:
    """Records scope data to CSV for offline playback."""
    
    def __init__(self, output_dir: str = "scope_recordings", max_vehicles: int = 5):
        self.output_dir = output_dir
        self.max_vehicles = max_vehicles
        self.file = None
        self.writer = None
        self.recording = False
        self.start_time = 0.0
        self.columns = []
    
    def start(self, columns: List[str], name: str = "scope", overwrite: bool = False):
        """
        Start recording with given columns.
        
        Args:
            columns: List of column names to record
            name: Short prefix for the file name (e.g., 'local', 'fleet')
            overwrite: If True, overwrites {name}.csv. If False, creates {name}_{timestamp}.csv
        """
        os.makedirs(self.output_dir, exist_ok=True)
        
        if overwrite:
            filepath = os.path.join(self.output_dir, f"{name}.csv")
        else:
            timestamp = datetime.now().strftime("%H%M%S")
            filepath = os.path.join(self.output_dir, f"{name}_{timestamp}.csv")
        
        self.columns = ['time'] + columns
        self.file = open(filepath, 'w', newline='', buffering=8192)
        self.writer = csv.DictWriter(self.file, fieldnames=self.columns)
        self.writer.writeheader()
        self.recording = True
        self.start_time = time.time()
        
        return filepath
    
    def _flatten_data(self, data: dict) -> dict:
        """
        Flatten complex data types for CSV storage.
        
        - fleet_states (ndarray): flatten to fleet_x_0, fleet_y_0, etc.
        - trust_scores (dict): flatten to trust_0, trust_1, etc.
        """
        flattened = {}
        
        for key, value in data.items():
            if key == 'fleet_states' and hasattr(value, 'shape'):
                # Flatten numpy array: shape (state_dim, num_vehicles)
                # state_dim: [x, y, theta, velocity, acceleration]
                state_names = ['x', 'y', 'theta', 'v', 'a']
                num_states = min(value.shape[0], len(state_names))
                num_vehicles = min(value.shape[1], self.max_vehicles)
                
                for v_idx in range(num_vehicles):
                    for s_idx in range(num_states):
                        col_name = f"fleet_{state_names[s_idx]}_{v_idx}"
                        flattened[col_name] = value[s_idx, v_idx]
            elif key == 'trust_scores' and isinstance(value, dict):
                # Flatten trust scores dict
                for v_idx in range(self.max_vehicles):
                    col_name = f"trust_{v_idx}"
                    flattened[col_name] = value.get(v_idx, 0.5)
            else:
                # Scalar values
                if isinstance(value, (int, float)):
                    flattened[key] = value
        
        return flattened
    
    def record(self, t: float, data: dict):
        """Record a data sample with flattened complex types."""
        if not self.recording or self.writer is None:
            return
        
        try:
            # Flatten complex data types
            flattened = self._flatten_data(data)
            
            row = {'time': t}
            for col in self.columns[1:]:
                row[col] = flattened.get(col, 0.0)
            self.writer.writerow(row)
        except Exception:
            pass
    
    def stop(self):
        """Stop recording."""
        self.recording = False
        if self.file:
            try:
                self.file.flush()
                self.file.close()
            except Exception:
                pass
            self.file = None
            self.writer = None


class ScopeDataPlayer:
    """Plays back recorded scope data from CSV."""

    def __init__(self, filepath: str):
        self.filepath = filepath
        self.data = []
        self.columns = []

    def load(self, max_vehicles: int = 5) -> bool:
        """
        Load data from CSV and reconstruct complex types.
        
        Args:
            max_vehicles: Maximum number of vehicles to reconstruct fleet data for
        """
        if not os.path.exists(self.filepath):
            print(f"[ScopePlayer] File not found: {self.filepath}")
            return False

        try:
            with open(self.filepath, 'r', newline='') as f:
                reader = csv.DictReader(f)
                self.columns = reader.fieldnames
                self.data = []
                
                for row in reader:
                    processed_row = self._reconstruct_row(row, max_vehicles)
                    self.data.append(processed_row)

            print(f"[ScopePlayer] Loaded {len(self.data)} samples from {self.filepath}")
            return True

        except Exception as e:
            print(f"[ScopePlayer] Error loading file: {e}")
            return False
    
    def _reconstruct_row(self, row: dict, max_vehicles: int) -> dict:
        """
        Reconstruct complex data types from flattened CSV row.
        
        - fleet_x_0, fleet_y_0, etc. -> fleet_states ndarray
        - trust_0, trust_1, etc. -> trust_scores dict
        """
        processed = {}
        
        # Check if we have fleet data columns
        state_names = ['x', 'y', 'theta', 'v', 'a']
        has_fleet = any(f"fleet_{state_names[0]}_0" in k for k in row.keys())
        has_trust = any(f"trust_0" in k for k in row.keys())
        
        if has_fleet:
            # Reconstruct fleet_states array
            # Find how many vehicles are in the data
            num_vehicles = 0
            for v_idx in range(max_vehicles):
                if f"fleet_x_{v_idx}" in row:
                    num_vehicles = v_idx + 1
            
            if num_vehicles > 0:
                fleet_states = np.zeros((len(state_names), num_vehicles))
                for v_idx in range(num_vehicles):
                    for s_idx, s_name in enumerate(state_names):
                        col_name = f"fleet_{s_name}_{v_idx}"
                        if col_name in row:
                            try:
                                fleet_states[s_idx, v_idx] = float(row[col_name])
                            except ValueError:
                                fleet_states[s_idx, v_idx] = 0.0
                processed['fleet_states'] = fleet_states
        
        if has_trust:
            # Reconstruct trust_scores dict
            trust_scores = {}
            for v_idx in range(max_vehicles):
                col_name = f"trust_{v_idx}"
                if col_name in row:
                    try:
                        trust_scores[v_idx] = float(row[col_name])
                    except ValueError:
                        trust_scores[v_idx] = 0.5
            if trust_scores:
                processed['trust_scores'] = trust_scores
        
        # Process remaining scalar columns
        for k, v in row.items():
            # Skip fleet and trust columns (already processed)
            if k.startswith('fleet_') or k.startswith('trust_'):
                continue
            try:
                processed[k] = float(v)
            except ValueError:
                processed[k] = 0.0
        
        return processed

    def play(self, manager: 'EstimationScopeManager', speed: float = 1.0, 
             loop: bool = False) -> None:
        """
        Play back data to the scope manager.
        
        Args:
            manager: Target scope manager
            speed: Playback speed multiplier (1.0 = real-time, 0.0 = as fast as possible)
            loop: Whether to loop playback
        """
        if not self.data:
            if not self.load():
                return

        # Normalize time to start from 0
        if self.data:
            t_start = self.data[0]['time']
            t_end = self.data[-1]['time']
            duration = t_end - t_start
            print(f"[ScopePlayer] Playback duration: {duration:.2f} seconds ({len(self.data)} samples)")
            print(f"[ScopePlayer] Time normalized: {t_start:.3f} -> 0.0")
            print(f"[ScopePlayer] Starting playback (Speed: {speed}x)...")
        
        try:
            while True:
                start_wall_time = time.time()
                start_sim_time = self.data[0]['time'] if self.data else 0.0
                
                for i, row in enumerate(self.data):
                    current_sim_time = row['time']
                    
                    # Calculate wait time if speed > 0
                    if speed > 0:
                        elapsed_sim = current_sim_time - start_sim_time
                        target_wall_time = start_wall_time + (elapsed_sim / speed)
                        sleep_time = target_wall_time - time.time()
                        
                        if sleep_time > 0:
                            time.sleep(sleep_time)
                    
                    # Normalize time to start from 0 for display
                    normalized_time = current_sim_time - start_sim_time
                    
                    # Sample and update with normalized time
                    manager.sample(normalized_time, row)
                    
                    # Update GUI periodically (e.g. every 33ms for 30fps)
                    # or on every sample if speed is slow enough
                    manager.update()
                    
                if not loop:
                    break
                    
                print("[ScopePlayer] Replaying...")
                
        except KeyboardInterrupt:
            print("[ScopePlayer] Playback interrupted")


# ==============================================================================
# Main Scope Manager
# ==============================================================================

class EstimationScopeManager:
    """
    Main manager for estimation visualization scopes.
    
    Provides:
    - Non-blocking data sampling via queue
    - Thread-based scope refresh
    - Multiple preset management
    - Recording capability
    """
    
    def __init__(self, fps: int = 30, time_window: float = 60.0, enabled: bool = True, 
                 headless: bool = False, downsample_factor: int = 1):
        """
        Initialize scope manager.
        
        Args:
            fps: Frames per second for scope refresh
            time_window: Default time window for plots
            enabled: Master enable switch
            headless: If True, disables visualization (MultiScope) but allows recording
            downsample_factor: Only display every Nth sample (1=all, 2=half, etc.)
                              Recording always captures all samples.
        """
        self.fps = fps
        self.time_window = time_window
        self.enabled = enabled and (MULTISCOPE_AVAILABLE or headless)  # Allow if headless even if no scope
        self.headless = headless
        self.downsample_factor = max(1, downsample_factor)
        
        # Presets
        self.presets: Dict[str, EstimationScopePreset] = {}
        
        # Non-blocking queue
        self.data_queue = queue.Queue(maxsize=100)
        
        # Refresh thread
        self._refresh_thread: Optional[threading.Thread] = None
        self._running = False
        
        # Recording
        self.recorder = ScopeDataRecorder()
        self.recording = False
        
        # Downsample counter (to limit display rate)
        self._sample_counter = 0
        self._display_sample_counter = 0
    
    def add_preset(self, preset: EstimationScopePreset) -> bool:
        """
        Add a preset to the manager.
        
        Args:
            preset: EstimationScopePreset instance
            
        Returns:
            True if preset was added and initialized successfully
        """
        if not self.enabled:
            return False
        
        if not preset.enabled:
            return False
        
        # In headless mode, we skip scope initialization but add the preset
        # so it can still process data (if needed) or just be tracked
        if self.headless:
            self.presets[preset.name] = preset
            print(f"[ScopeManager] Added preset (Headless): {preset.name}")
            return True
            
        if preset.initialize(fps=self.fps, time_window=self.time_window):
            self.presets[preset.name] = preset
            print(f"[ScopeManager] Added preset: {preset.name}")
            return True
        
        return False
    
    def remove_preset(self, name: str) -> bool:
        """Remove a preset by name."""
        if name in self.presets:
            self.presets[name].close()
            del self.presets[name]
            return True
        return False
    
    def sample(self, t: float, data: dict) -> None:
        """
        Sample data for all presets (non-blocking).
        
        Args:
            t: Current time in seconds
            data: Dictionary containing all signal data
        """
        if not self.enabled or not self._running:
            return
        
        try:
            # Always record at full rate if enabled
            if self.recording:
                self.recorder.record(t, data)
            
            # Apply downsampling for display only
            self._display_sample_counter += 1
            if self._display_sample_counter >= self.downsample_factor:
                self._display_sample_counter = 0
                # Non-blocking put to display queue
                self.data_queue.put_nowait((t, data.copy()))
                
        except queue.Full:
            pass  # Drop sample if queue is full
    
    def start(self, threaded: bool = True) -> None:
        """
        Start the scope refresh.
        
        Args:
            threaded: If True, runs refresh loop in background thread.
                      If False, user must call update() manually in loop.
        """
        if not self.enabled or self._running:
            return
        
        self._running = True
        
        if threaded:
            self._refresh_thread = threading.Thread(
                target=self._refresh_loop,
                daemon=True,
                name="ScopeRefreshThread"
            )
            self._refresh_thread.start()
            print("[ScopeManager] Started refresh thread")
        else:
            print("[ScopeManager] Started in manual mode (call update() in loop)")
    
    def stop(self) -> None:
        """Stop the scope refresh thread."""
        self._running = False
        
        if self._refresh_thread:
            self._refresh_thread.join(timeout=2.0)
            self._refresh_thread = None
        
        # Close all presets
        for preset in self.presets.values():
            preset.close()
        
        # Stop recording
        if self.recording:
            self.recorder.stop()
            self.recording = False
        
        print("[ScopeManager] Stopped")
    
    def update(self) -> None:
        """
        Perform one update cycle (process queue + refresh scopes).
        Call this manually in the control loop if threaded=False.
        """
        if not self.enabled or not self._running:
            return

        try:
            # Process all pending samples
            while not self.data_queue.empty():
                try:
                    t, data = self.data_queue.get_nowait()
                    
                    # Sample all presets
                    for preset in self.presets.values():
                        preset.sample(t, data)
                        
                except queue.Empty:
                    break
            
            # Refresh scopes (only if MultiScope available and not headless)
            if MULTISCOPE_AVAILABLE and not self.headless:
                MultiScope.refreshAll()
                
        except Exception as e:
            print(f"[ScopeManager] Update error: {e}")
            
    def _refresh_loop(self) -> None:
        """Main refresh loop running in separate thread."""
        while self._running:
            self.update()
            
            # Sleep to maintain refresh rate
            time.sleep(1.0 / self.fps)
    
    def start_recording(self, columns: List[str] = None, max_vehicles: int = 5, 
                         name: str = "scope") -> str:
        """
        Start recording scope data.
        
        Args:
            columns: List of data columns to record. If None, uses default 
                     columns including both local and fleet data.
            max_vehicles: Maximum number of vehicles to record fleet data for
            name: Short prefix for the file name (e.g., 'local', 'fleet')
            
        Returns:
            Path to the recording file
        """
        if columns is None:
            # Local estimator columns
            columns = ['x', 'y', 'theta', 'velocity', 'acceleration',
                      'x_gps', 'y_gps', 'theta_gps', 'steering', 'throttle',
                      'v_ref', 'consensus_error']
            
            # Fleet state columns (flattened): fleet_x_0, fleet_y_0, etc.
            state_names = ['x', 'y', 'theta', 'v', 'a']
            for v_idx in range(max_vehicles):
                for s_name in state_names:
                    columns.append(f"fleet_{s_name}_{v_idx}")
            
            # Trust score columns: trust_0, trust_1, etc.
            for v_idx in range(max_vehicles):
                columns.append(f"trust_{v_idx}")
        
        # Update recorder with max_vehicles setting
        self.recorder.max_vehicles = max_vehicles
        
        filepath = self.recorder.start(columns, name=name)
        self.recording = True
        print(f"[ScopeManager] Recording started: {filepath}")
        return filepath
    
    def stop_recording(self) -> None:
        """Stop recording."""
        self.recording = False
        self.recorder.stop()
        print("[ScopeManager] Recording stopped")
    
    @staticmethod
    def create_default_local_scopes(fps: int = 30, time_window: float = 60.0) -> 'EstimationScopeManager':
        """
        Factory method to create scope manager with default local presets.
        
        Returns:
            Configured EstimationScopeManager
        """
        mgr = EstimationScopeManager(fps=fps, time_window=time_window)
        mgr.add_preset(LocalStatePreset())
        mgr.add_preset(LocalEstimationErrorPreset())
        mgr.add_preset(LocalControlPreset())
        return mgr
    
    @staticmethod
    def create_default_fleet_scopes(fps: int = 30, time_window: float = 60.0,
                                    max_vehicles: int = 5) -> 'EstimationScopeManager':
        """
        Factory method to create scope manager with default fleet presets.
        
        Returns:
            Configured EstimationScopeManager
        """
        mgr = EstimationScopeManager(fps=fps, time_window=time_window)
        mgr.add_preset(FleetPositionPreset(max_vehicles=max_vehicles))
        mgr.add_preset(FleetStatePreset(max_vehicles=max_vehicles))
        mgr.add_preset(FleetConsensusPreset(max_vehicles=max_vehicles))
        return mgr
    
    @staticmethod
    def create_all_scopes(fps: int = 30, time_window: float = 60.0,
                          max_vehicles: int = 5) -> 'EstimationScopeManager':
        """
        Factory method to create scope manager with all presets.
        
        Returns:
            Configured EstimationScopeManager
        """
        mgr = EstimationScopeManager(fps=fps, time_window=time_window)
        
        # Local presets
        mgr.add_preset(LocalStatePreset())
        mgr.add_preset(LocalEstimationErrorPreset())
        mgr.add_preset(LocalControlPreset())
        
        # Fleet presets
        mgr.add_preset(FleetPositionPreset(max_vehicles=max_vehicles))
        mgr.add_preset(FleetStatePreset(max_vehicles=max_vehicles))
        mgr.add_preset(FleetConsensusPreset(max_vehicles=max_vehicles))
        
        return mgr


# ==============================================================================
# Convenience functions
# ==============================================================================

def create_scope_manager(
    preset_names: List[str] = None,
    fps: int = 30,
    time_window: float = 60.0,
    max_vehicles: int = 5
) -> EstimationScopeManager:
    """
    Create a scope manager with specified presets.
    
    Args:
        preset_names: List of preset names to enable
            Options: 'local_state', 'local_error', 'local_control',
                    'fleet_position', 'fleet_state', 'fleet_consensus'
        fps: Refresh rate
        time_window: Time window for plots
        max_vehicles: Max vehicles for fleet plots
        
    Returns:
        Configured EstimationScopeManager
    """
    if preset_names is None:
        preset_names = ['local_state']
    
    mgr = EstimationScopeManager(fps=fps, time_window=time_window)
    
    preset_map = {
        'local_state': lambda: LocalStatePreset(),
        'local_error': lambda: LocalEstimationErrorPreset(),
        'local_control': lambda: LocalControlPreset(),
        'fleet_position': lambda: FleetPositionPreset(max_vehicles),
        'fleet_state': lambda: FleetStatePreset(max_vehicles),
        'fleet_consensus': lambda: FleetConsensusPreset(max_vehicles),
    }
    
    for name in preset_names:
        if name in preset_map:
            mgr.add_preset(preset_map[name]())
    
    return mgr
