"""
Simplified Neural Layer-2 Disturbance Estimator with Luenberger Observer

A clean implementation of the 2-layer observer architecture:
- Layer 1 (Teacher): TireEstimatorSimple → estimates tire residuals [w_r, w_f]
- Layer 2 (Student): Luenberger Observer + Neural Network → estimates general disturbances [d_vx, d_vy, d_r]

=============================================================================
ARCHITECTURE (from doc_layer2.md)
=============================================================================
                                    
    ┌─────────────────────────────────────────────────────────────────┐
    │                         LAYER 1 (TEACHER)                       │
    │   TireEstimatorSimple (EKF with augmented tire residuals)       │
    │                                                                 │
    │   State: x̂₁ = [vx, vy, ψ, r, X, Y]                             │
    │   Tire:  ŵ  = [w_r, w_f]                                       │
    └─────────────────────────────────────────────────────────────────┘
                              │
                              │  Provides tire residuals ŵ
                              ▼
    ┌─────────────────────────────────────────────────────────────────┐
    │                   LAYER 2 LUENBERGER OBSERVER                   │
    │                                                                 │
    │   Observer: x̂₂[k+1] = A_d·x̂₂[k] + B_d·u + E_d·ŵ + G_d·d̂      │
    │                       + L·(y[k] - C·x̂₂[k])                     │
    │                                                                 │
    │   Neural Network learns disturbance d̂ = [d_vx, d_vy, d_r]      │
    └─────────────────────────────────────────────────────────────────┘

=============================================================================
OBSERVER EQUATION (Discrete-Time)
=============================================================================
    x̂₂[k+1] = A_d·x̂₂[k] + B_d·u[k] + E_d·ŵ[k] + G_d·d̂[k] + L·(y[k] - C·x̂₂[k])

Where:
    - A_d, B_d, E_d: Discretized system matrices (from qLPV dynamics)
    - G_d: Disturbance injection matrix (discretized)
    - L: Observer gain (6×5 for full GPS, 6×2 for IMU-only)
    - ŵ: Tire residuals from Layer 1
    - d̂: General disturbances from NN

=============================================================================
DISTURBANCE MODEL
=============================================================================
The NN estimates disturbances d = [d_vx, d_vy, d_r] that capture:
- IMU bias / gyro bias
- Discretization / time delay effects
- Low-speed singularities (1/vx coupling)
- Yaw wrap artifacts
- Unmodeled aero/grade/load transfer

These are injected directly into the dynamics:
    v̇_x += d_vx
    v̇_y += d_vy
    ṙ   += d_r

=============================================================================
"""

import numpy as np
import torch
import torch.nn as nn
import time
from typing import Optional, Dict, Tuple
from dataclasses import dataclass
from pathlib import Path
import sys
from scipy.linalg import expm

# Import paths setup
current_dir = Path(__file__).parent
local_neural_obs_dir = current_dir.parent
observer_dir = local_neural_obs_dir.parent

# 1. Base Class (from Observer)
sys.path.insert(0, str(observer_dir))
from local_state_estimators import LocalStateEstimatorBase, wrap_to_pi

# 2. Dependencies in LocalNeuralObs (Parent)
sys.path.insert(0, str(local_neural_obs_dir))
from qlpv_vehicle_dynamics_obs import (
    SchedulingParameters,
    QLPVVehicleDynamicsObs,
    get_default_vehicle_params,
    IDX_VX, IDX_VY, IDX_PSI, IDX_R, IDX_X, IDX_Y, STATE_DIM,
)
from neural_obs_recorder import NeuralObsRecorder

# 3. Layer 1 Estimator (from LocalNeuralObs/1LayerObs)
one_layer_dir = local_neural_obs_dir / "1LayerObs"
sys.path.insert(0, str(one_layer_dir))
from tire_estimator_simple import TireEstimatorSimple

# 4. LMI Gain Scheduler (from current directory or LocalNeuralObs depending on file loc)
sys.path.insert(0, str(current_dir))
from Design_LMI_neural import NeuralQLPVGainScheduler

# 5. Neural Network Components (from current directory)
from neural_network import (
    NeuralObserverNet, 
    save_model, 
    load_model, 
    create_optimizer
)


# =============================================================================
# DISCRETIZATION HELPER
# =============================================================================

def discretize_system_zoh(A_c: np.ndarray, B_c: np.ndarray, E_c: np.ndarray, 
                          G_c: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Discretize continuous-time system using Zero-Order Hold (ZOH).
    
    Continuous: ẋ = A·x + B·u + E·w + G·d
    Discrete:   x[k+1] = A_d·x[k] + B_d·u[k] + E_d·w[k] + G_d·d[k]
    
    Uses matrix exponential for accurate discretization.
    """
    n = A_c.shape[0]
    m_B = B_c.shape[1]
    m_E = E_c.shape[1]
    m_G = G_c.shape[1]
    
    # Build augmented matrix for ZOH discretization
    # [A  B  E  G]
    # [0  0  0  0]  <- for B
    # [0  0  0  0]  <- for E  
    # [0  0  0  0]  <- for G
    aug_size = n + m_B + m_E + m_G
    M = np.zeros((aug_size, aug_size))
    M[:n, :n] = A_c
    M[:n, n:n+m_B] = B_c
    M[:n, n+m_B:n+m_B+m_E] = E_c
    M[:n, n+m_B+m_E:] = G_c
    
    # Compute matrix exponential
    M_d = expm(M * dt)
    
    # Extract discrete matrices
    A_d = M_d[:n, :n]
    B_d = M_d[:n, n:n+m_B]
    E_d = M_d[:n, n+m_B:n+m_B+m_E]
    G_d = M_d[:n, n+m_B+m_E:]
    
    return A_d, B_d, E_d, G_d


# =============================================================================
# LEARNING BATCH
# =============================================================================

@dataclass
class LearningSample:
    """Single training sample for the neural network."""
    z: np.ndarray      # NN input
    r_dyn: np.ndarray  # Target residual [d_vx, d_vy, d_r]
    weight: float      # Sample quality weight (gating)


class LearningBuffer:
    """
    Simple ring buffer for online learning.
    """
    
    def __init__(self, max_size: int = 500):
        self.max_size = max_size
        self.buffer = []
        self.idx = 0
    
    def add(self, sample: LearningSample):
        """Add a sample to the buffer."""
        if len(self.buffer) < self.max_size:
            self.buffer.append(sample)
        else:
            self.buffer[self.idx] = sample
            self.idx = (self.idx + 1) % self.max_size
    
    def sample_batch(self, batch_size: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Sample a random batch from the buffer."""
        if len(self.buffer) < batch_size:
            batch_size = len(self.buffer)
        
        if batch_size == 0:
            return None, None, None
        
        indices = np.random.choice(len(self.buffer), batch_size, replace=False)
        
        z_batch = np.array([self.buffer[i].z for i in indices])
        r_batch = np.array([self.buffer[i].r_dyn for i in indices])
        w_batch = np.array([self.buffer[i].weight for i in indices])
        
        return z_batch, r_batch, w_batch
    
    def __len__(self):
        return len(self.buffer)


# =============================================================================
# LAYER 2 NEURAL ESTIMATOR WITH LUENBERGER OBSERVER
# =============================================================================

class NeuralLayer2Simple(LocalStateEstimatorBase):
    """
    Layer-2 Neural Disturbance Estimator with Luenberger Observer
    
    Inherits from LocalStateEstimatorBase for integration with VehicleObserver.
    
    Implements a proper observer structure with neural network learning:
    - Layer 1 (Teacher): TireEstimatorSimple → provides tire residuals ŵ = [w_r, w_f]
    - Layer 2 (Observer): Luenberger observer + NN → estimates general disturbances d̂ = [d_vx, d_vy, d_r]
    
    Observer Equation (Discrete-Time):
        x̂₂[k+1] = A_d·x̂₂[k] + B_d·u[k] + E_d·ŵ[k] + G_d·d̂[k] + L·(y[k] - C·x̂₂[k])
    
    Where:
        - A_d, B_d, E_d: Discretized system matrices (from qLPV dynamics)
        - G_d: Discretized disturbance injection matrix  
        - L: LMI-designed observer gain matrix (via NeuralQLPVGainScheduler)
        - ŵ: Tire residuals from Layer 1
        - d̂: General disturbances from NN
    
    Key principles:
    1. Layer-1 provides tire residuals (physically meaningful)
    2. Layer-2 NN learns "everything else" (biases, artifacts, etc.)
    3. Proper observer dynamics with prediction + correction
    4. Train only when conditions are good (gating)
    5. LMI-designed observer gains (no default fallback)
    
    Usage:
        estimator = NeuralLayer2Simple(sample_time=0.02)
        
        # In control loop:
        state_corrected = estimator.update(motor_tach, steering, throttle, dt, gyro_z, gps_data)
        disturbance = estimator.get_disturbance()
    """
    
    # State dimensions
    INTERNAL_STATE_DIM = STATE_DIM  # 6
    MEAS_DIM_FULL = 5  # [vx, r, ψ, X, Y]
    MEAS_DIM_IMU = 2   # [vx, r]

    
    def __init__(self,
                 initial_pose: Optional[np.ndarray] = None,
                 sample_time: float = 0.02,
                 vehicle_params: Optional[Dict] = None,
                 config: Optional[Dict] = None,
                 logger=None,
                 hidden_dim: int = 32,
                 learning_rate: float = 0.001,
                 buffer_size: int = 500,
                 batch_size: int = 32,
                 enable_learning: bool = True):
        """
        Initialize Layer-2 Neural Estimator with Luenberger Observer.
        
        Args:
            initial_pose: Initial pose [X, Y, ψ] (for base class compatibility)
            sample_time: Sample time Ts [s]
            vehicle_params: Vehicle parameters dict
            config: Configuration dict with keys:
                - enable_recording: bool (default False)
                - recording_output_dir: str (default '.')
                - recording_filename: str (default None)
                - vx_range: [min, max] for LMI gain scheduling
                - delta_max: max steering for LMI gain scheduling
                - hinf_gamma: H∞ gamma for LMI (default 1.5)
                - use_gain_scheduling: bool (default True)
            logger: Logger instance
            hidden_dim: Hidden layer dimension for NN
            learning_rate: Learning rate for NN training
            buffer_size: Size of experience replay buffer
            batch_size: Batch size for training
            enable_learning: Whether to train the NN online
        """
        # Initialize base class
        super().__init__(initial_pose=initial_pose, logger=logger)
        
        # Store config
        self.config = config or {}
        self.Ts = sample_time
        
        # Base class compatibility: state_dim is 4D [X, Y, θ, v]
        # Internal state is 6D [vx, vy, ψ, r, X, Y]
        
        # =====================================================
        # VEHICLE DYNAMICS MODEL (from qLPV)
        # =====================================================
        self.vehicle_params = vehicle_params or get_default_vehicle_params()
        self.min_vx = self.vehicle_params.get('vx_min', 0.1)
        
        # Centralized dynamics model for Layer 2 with GENERAL disturbances
        # (Layer 1 uses 'tire', Layer 2 estimates general disturbances)
        self.dynamics = QLPVVehicleDynamicsObs(
            vehicle_params=self.vehicle_params,
            min_vx=self.min_vx,
            disturbance_mode='general'  # Layer 2 uses general disturbances [d_vx, d_vy, d_r]
        )
        
        # =====================================================
        # LAYER 1: TIRE ESTIMATOR (TEACHER)
        # =====================================================
        self.layer1 = TireEstimatorSimple(
            sample_time=sample_time,
            vehicle_params=vehicle_params
        )
        
        # =====================================================
        # LAYER 2: LUENBERGER OBSERVER STATE
        # =====================================================
        # 6D internal state: [vx, vy, ψ, r, X, Y]
        self.state_nn_6d = np.zeros(self.INTERNAL_STATE_DIM)
        
        # Initialize from initial_pose if provided
        if initial_pose is not None:
            self.state_nn_6d[IDX_X] = initial_pose[0]  # X
            self.state_nn_6d[IDX_Y] = initial_pose[1]  # Y  
            self.state_nn_6d[IDX_PSI] = initial_pose[2]  # ψ
        
        # Alias for compatibility
        self.x_hat = self.state_nn_6d
        
        # =====================================================
        # LAYER 2: NEURAL NETWORK
        # =====================================================
        # 3. Neural Network (Student)
        self.input_dim = 10   # [vx, vy, psi, r, delta, accel, alpha_f, alpha_r, vx_meas, r_meas]
        self.output_dim = 3   # [d_vx, d_vy, d_r]
        self.hidden_dim = hidden_dim
        self.learning_rate = learning_rate
        
        self.nn = NeuralObserverNet(
            input_dim=self.input_dim,
            hidden_dim=hidden_dim,
            output_dim=self.output_dim
        )
        
        # Current disturbance estimate
        self.d_hat = np.zeros(3)  # [d_vx, d_vy, d_r]
        
        # =====================================================
        # G MATRIX: Disturbance Injection
        # =====================================================
        # d affects [vx, vy, r] (indices 0, 1, 3)
        # G_c is continuous-time: ẋ = A·x + ... + G·d
        self.G_c = np.zeros((self.INTERNAL_STATE_DIM, 3))
        self.G_c[IDX_VX, 0] = 1.0   # d_vx → v̇_x
        self.G_c[IDX_VY, 1] = 1.0   # d_vy → v̇_y
        self.G_c[IDX_R, 2] = 1.0    # d_r  → ṙ
        
        # =====================================================
        # OBSERVER GAIN L (LMI-BASED, STRICT)
        # =====================================================
        self._initialize_lmi_observer_gain()
        
        # =====================================================
        # LEARNING COMPONENTS
        # =====================================================
        self.enable_learning = enable_learning
        self.buffer = LearningBuffer(max_size=buffer_size)
        self.batch_size = batch_size
        
        self.optimizer = torch.optim.Adam(
            self.nn.parameters(),
            lr=learning_rate,
            weight_decay=1e-4
        )
        
        # Training statistics
        self.loss_history = []
        self.train_step = 0
        
        # =====================================================
        # STATE STORAGE (for compatibility)
        # =====================================================
        self.x_corrected = np.zeros(self.INTERNAL_STATE_DIM)  # Alias for state_nn_6d
        
        # =====================================================
        # GATING THRESHOLDS
        # =====================================================
        self.vx_min_train = 0.3        # Don't train below this speed
        self.innovation_max = 1.0       # Max innovation norm for training
        self.yaw_jump_threshold = 0.5   # Detect yaw wrap
        
        self._prev_psi = 0.0
        self._current_control = np.zeros(2)
        self._last_steering = 0.0
        
        # =====================================================
        # RECORDING CAPABILITY
        # =====================================================
        self.enable_recording = self.config.get('enable_recording', False)
        self.recorder: Optional[NeuralObsRecorder] = None
        
        if self.enable_recording:
            self.recorder = NeuralObsRecorder(
                mode='2layer',  # Layer 2 records general disturbances
                output_dir=self.config.get('recording_output_dir', '.'),
                filename=self.config.get('recording_filename'),
                append_mode=self.config.get('recording_append_mode', False)
            )
        
        # Ground truth provider (for debugging/plotting)
        self.ground_truth_provider = None
        
        # Tire info storage for Layer 2 (general disturbances)
        self.tire_info_layer_2 = {
            'd_vx': 0.0, 'd_vy': 0.0, 'd_r': 0.0,
            'd_vx_true': 0.0, 'd_vy_true': 0.0, 'd_r_true': 0.0,
        }
        
        # Update counter for base class compatibility
        self.update_count = 0
        self.last_update_time = time.time()
    
    def _initialize_lmi_observer_gain(self):
        """
        Initialize observer gain matrix L using LMI-based polytopic qLPV scheduling.
        
        Uses NeuralQLPVGainScheduler to compute gains with guaranteed stability.
        Raises exception if LMI fails (no fallback).
        """
        # LMI parameters from config
        vx_range = self.config.get('vx_range', [0.5, 3.0])
        delta_max = self.config.get('delta_max', 0.4)
        hinf_gamma = self.config.get('hinf_gamma', 1.5)
        use_gain_scheduling = self.config.get('use_gain_scheduling', True)
        n_vx_vertices = self.config.get('n_vx_vertices', 3)
        n_delta_vertices = self.config.get('n_delta_vertices', 3)
        
        # Create gain scheduler with disturbance_mode='general' (matches Layer 2)
        self.gain_scheduler = NeuralQLPVGainScheduler(
            vehicle_params=self.vehicle_params,
            vx_range=tuple(vx_range),
            delta_max=delta_max,
            n_vx_vertices=n_vx_vertices,
            n_delta_vertices=n_delta_vertices,
            lmi_method='hinf',
            hinf_gamma=hinf_gamma,
            use_common_lyapunov=True,  # Robust design
            discrete=True,
            sample_time=self.Ts,
            verbose=False,
            disturbance_mode='general',  # Layer 2 uses general disturbances
            dynamics_model=self.dynamics  # Share dynamics model
        )
        
        # Compute LMI gains (strict, no fallback)
        success = self.gain_scheduler.compute_gains_lmi()
        
        if not success:
            raise ValueError(
                "LMI gain computation failed for NeuralLayer2Simple. "
                "Check vehicle parameters, vx_range, or hinf_gamma settings."
            )
        
        self._use_gain_scheduling = use_gain_scheduling
        self._gain_method = 'qlpv_scheduled_discrete'
        
        # Get initial gain at nominal operating point
        self.L = self.gain_scheduler.get_scheduled_gain(1.0, 0.0)
    
    def get_scheduled_gain(self, vx: float, delta: float) -> np.ndarray:
        """Get interpolated observer gain for current operating point."""
        if self._use_gain_scheduling:
            return self.gain_scheduler.get_scheduled_gain(vx, delta)
        return self.L
    
    def get_observer_gain(self) -> np.ndarray:
        """Get current observer gain matrix L (6×5)."""
        return self.L.copy()
    
    def get_gain_method(self) -> str:
        """Get the method used to compute observer gains."""
        return self._gain_method
    
    def get_gain_info(self) -> Dict:
        """Get detailed information about gain computation."""
        return {
            'method': self._gain_method,
            'use_scheduling': self._use_gain_scheduling,
            'n_vertices': self.gain_scheduler.n_vertices,
            'current_L_norm': np.linalg.norm(self.L),
        }
    
    def set_ground_truth_provider(self, provider):
        """Set ground truth provider for recording (e.g., MockQCar instance)."""
        self.ground_truth_provider = provider

    
    # =========================================================
    # MAIN UPDATE (Base Class Interface)
    # =========================================================
    
    def update(self, motor_tach: float, steering: float, throttle: float, dt: float,
               gyro_z: float = 0.0, gps_data: Optional[Dict] = None,
               acceleration: Optional[np.ndarray] = None) -> bool:
        """
        Update state estimate with sensor data (LocalStateEstimatorBase interface).
        
        Args:
            motor_tach: Motor tachometer reading (velocity)
            steering: Steering angle command
            throttle: Throttle command (acceleration proxy)
            dt: Time step
            gyro_z: Z-axis gyroscope reading (angular velocity)
            gps_data: Optional GPS data dict with keys: x, y, theta, valid
            acceleration: Optional acceleration data [ax, ay]
            
        Returns:
            True if update successful
        """
        try:
            # Build measurement vector [vx, r, psi, X, Y, ay]
            vx_meas = motor_tach
            r_meas = gyro_z
            
            gps_valid = gps_data is not None and gps_data.get('valid', False)
            
            if gps_valid:
                psi_meas = gps_data.get('theta', self.state_nn_6d[IDX_PSI])
                X_meas = gps_data.get('x', self.state_nn_6d[IDX_X])
                Y_meas = gps_data.get('y', self.state_nn_6d[IDX_Y])
                measurement = np.array([vx_meas, r_meas, psi_meas, X_meas, Y_meas])
                if acceleration is not None:
                    ay_meas = acceleration[1] if len(acceleration) > 1 else 0.0
                    measurement = np.hstack([measurement, ay_meas])
            else:
                measurement = np.array([vx_meas, r_meas])
                if acceleration is not None:
                    ay_meas = acceleration[1] if len(acceleration) > 1 else 0.0
                    measurement = np.hstack([measurement, ay_meas])
            
            # Build control vector [delta, accel]
            control = np.array([steering, throttle])
            
            # Call internal update
            self._update_internal(measurement, control, dt, gps_valid)
            
            # Update base class state for compatibility
            self.state = self._extract_4d_state()
            self.last_update_time = time.time()
            self.update_count += 1
            
            # Record if enabled
            if self.enable_recording and self.recorder is not None:
                self._record_step(measurement, acceleration)
            
            return True
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("NeuralLayer2Simple update failed", e)
            return False
    
    def _update_internal(self, measurement: np.ndarray, control: np.ndarray,
                         dt: float, gps_valid: bool = True) -> np.ndarray:
        """
        Internal update step with Luenberger observer dynamics.
        
        Seven-phase update flow:
        1. Sensor processing and Layer 1 update
        2. Compute system matrices at current operating point
        3. Discretize system (ZOH)
        4. Prepare NN input and get disturbance prediction
        5. Observer state prediction + correction
        6. Compute training label from innovation
        7. Train NN (if enabled and conditions good)
        
        Observer Equation (Discrete-Time):
            x̂₂[k+1] = A_d·x̂₂[k] + B_d·u[k] + E_d·ŵ[k] + G_d·d̂[k] + L·(y[k] - C·x̂₂[k])
        
        Args:
            measurement: [vx, r, psi, X, Y, ay] (6D) or [vx, r, ay] (3D for IMU-only)
            control: [delta, accel]
            dt: Sample time
            gps_valid: Whether GPS measurements are available
            
        Returns:
            Corrected state estimate [vx, vy, psi, r, X, Y]
        """
        y = np.array(measurement).flatten()
        u = np.array(control).flatten()
        self._current_control = u
        self._last_steering = u[0]
        
        # =====================================================
        # STEP 1: LAYER 1 (TEACHER) UPDATE
        # =====================================================
        # Update Layer 1 with measurement → get tire residuals
        x1_updated, w_hat = self.layer1.update(y, u, dt)
        
        # =====================================================
        # STEP 2: EXTRACT MEASUREMENTS
        # =====================================================
        vx_meas = y[0]
        r_meas = y[1]
        
        if gps_valid and len(y) >= 5:
            psi_meas = y[2]
            X_meas = y[3]
            Y_meas = y[4]
            # Full measurement: [vx, r, ψ, X, Y]
            y_obs = np.array([vx_meas, r_meas, psi_meas, X_meas, Y_meas])
        else:
            # Use Layer 2 predictions for unavailable measurements
            psi_meas = self.state_nn_6d[IDX_PSI]
            X_meas = self.state_nn_6d[IDX_X]
            Y_meas = self.state_nn_6d[IDX_Y]
            # IMU-only: [vx, r]
            y_obs = np.array([vx_meas, r_meas])
        
        # =====================================================
        # STEP 3: COMPUTE SYSTEM MATRICES (qLPV)
        # =====================================================
        rho = self.dynamics.compute_scheduling_params(self.state_nn_6d, u[0])
        
        A_c = self.dynamics.compute_A_matrix(rho)
        B_c = self.dynamics.compute_B_matrix(rho)
        E_c = self.dynamics.compute_E_matrix(rho)
        
        # Discretize system with ZOH
        A_d, B_d, E_d, G_d = discretize_system_zoh(A_c, B_c, E_c, self.G_c, dt)
        
        # =====================================================
        # STEP 4: PREPARE NN INPUT AND GET PREDICTION
        # =====================================================
        tire_info = self.layer1.get_tire_info()
        
        z = self._prepare_nn_input(
            self.state_nn_6d, u, tire_info,
            vx_meas=vx_meas, r_meas=r_meas
        )
        
        # Get NN disturbance prediction
        self.d_hat = self._predict_disturbance(z)
        
        # Update Layer 2 tire_info with current disturbance estimates
        self.tire_info_layer_2['d_vx'] = self.d_hat[0]
        self.tire_info_layer_2['d_vy'] = self.d_hat[1]
        self.tire_info_layer_2['d_r'] = self.d_hat[2]
        
        # =====================================================
        # STEP 5: OBSERVER UPDATE (Prediction + Correction)
        # =====================================================
        # Get scheduled gain for current operating point
        if self._use_gain_scheduling:
            L = self.get_scheduled_gain(vx_meas, u[0])
            self.L = L  # Update current gain
        else:
            L = self.L
        
        # Get sensor-dependent C and L matrices
        if gps_valid and len(y) >= 5:
            C = self._compute_C_matrix_full()
            L_avail = L
        else:
            C = self._compute_C_matrix_imu()
            L_avail = L[:, :self.MEAS_DIM_IMU]
        
        # Discrete-time prediction: x̂_pred = A_d·x̂ + B_d·u + E_d·w + G_d·d
        x_pred = A_d @ self.state_nn_6d + B_d @ u + E_d @ w_hat + G_d @ self.d_hat
        
        # Innovation: y - C·x̂[k]
        innovation = y_obs - C @ self.state_nn_6d
        
        # Wrap heading innovation for ψ (index 2 in full measurement)
        if gps_valid and len(y) >= 5 and len(innovation) >= 3:
            innovation[2] = wrap_to_pi(innovation[2])
        
        # Correction: x̂[k+1] = x̂_pred + L·innovation
        self.state_nn_6d = x_pred + L_avail @ innovation
        
        # Wrap heading state to [-π, π]
        self.state_nn_6d[IDX_PSI] = wrap_to_pi(self.state_nn_6d[IDX_PSI])
        
        # Update alias for compatibility
        self.x_hat = self.state_nn_6d
        self.x_corrected = self.state_nn_6d.copy()
        
        # =====================================================
        # STEP 6: COMPUTE TRAINING LABEL
        # =====================================================
        # Training label is the innovation scaled by what the NN should learn
        # This represents the "unexplained" dynamics after nominal + tire model
        alpha = self._compute_gating_weight(self.state_nn_6d, y)
        
        # The training target is based on observer innovation
        # Project innovation to disturbance space [d_vx, d_vy, d_r]
        if gps_valid and len(innovation) >= 3:
            # Use innovation in dynamic channels
            r_dyn = np.array([
                innovation[0],  # vx innovation → d_vx
                innovation[1] * 0.1,  # r innovation → (contributes to) d_vy
                innovation[1],  # r innovation → d_r
            ])
        else:
            r_dyn = np.array([innovation[0], 0.0, innovation[1]])
        
        # =====================================================
        # STEP 7: TRAIN NN (if enabled and conditions good)
        # =====================================================
        if self.enable_learning and alpha > 0.1:
            sample = LearningSample(z=z, r_dyn=r_dyn, weight=alpha)
            self.buffer.add(sample)
            
            if len(self.buffer) >= self.batch_size:
                self._train_step()
        
        return self.state_nn_6d.copy()
    
    def _extract_4d_state(self) -> np.ndarray:
        """
        Extract 4D state [X, Y, ψ, v_x] for base class compatibility.
        
        Base class: [x, y, theta, velocity]
        Internal 6D: [vx, vy, ψ, r, X, Y]
        """
        return np.array([
            self.state_nn_6d[IDX_X],    # X
            self.state_nn_6d[IDX_Y],    # Y
            self.state_nn_6d[IDX_PSI],  # ψ
            self.state_nn_6d[IDX_VX],   # v_x
        ])
    
    def _record_step(self, measurement: np.ndarray, acceleration: Optional[np.ndarray]):
        """Record current step to recorder if enabled."""
        if self.recorder is None:
            return
        
        # Get ground truth if available
        state_true_6d = None
        unknown_input_true = None
        disturbances_true = None
        tire_info_true = None
        if self.ground_truth_provider is not None:
            try:
                state_true_6d = self.ground_truth_provider.get_true_state()
                unknown_input_true = self.ground_truth_provider.get_true_residuals()
                if hasattr(self.ground_truth_provider, 'get_true_disturbances'):
                    disturbances_true = self.ground_truth_provider.get_true_disturbances()
                if hasattr(self.ground_truth_provider, 'get_tire_info'):
                    tire_info_true = self.ground_truth_provider.get_tire_info()
            except:
                pass
        
        # Build measurement dict
        vx_meas = measurement[0] if len(measurement) > 0 else 0.0
        r_meas = measurement[1] if len(measurement) > 1 else 0.0
        psi_meas = measurement[2] if len(measurement) > 2 else self.state_nn_6d[IDX_PSI]
        X_meas = measurement[3] if len(measurement) > 3 else self.state_nn_6d[IDX_X]
        Y_meas = measurement[4] if len(measurement) > 4 else self.state_nn_6d[IDX_Y]
        ay_meas = measurement[5] if len(measurement) > 5 else 0.0
        ax_meas = acceleration[0] if acceleration is not None and len(acceleration) > 0 else 0.0
        
        measurements = {
            'vx': vx_meas, 'r': r_meas, 'psi': psi_meas,
            'X': X_meas, 'Y': Y_meas, 'ay': ay_meas, 'ax': ax_meas,
        }
        
        # Record 2-layer data with correct argument names
        self.recorder.record_2layer(
            t=self.update_count * self.Ts,
            state_6d=self.state_nn_6d,
            measurements=measurements,
            nn_outputs=self.d_hat,
            uio_state=self.layer1.get_state(),
            uio_unknown_input=self.layer1.get_tire_residuals(),
            steering=self._current_control[0],
            throttle=self._current_control[1],
            loss=self.get_training_loss(),
            gps_valid=True,
            state_true_6d=state_true_6d,
            unknown_input_true=unknown_input_true,
            disturbances_true=disturbances_true,
            tire_info_true=tire_info_true,
            tire_info_layer_1=self.layer1.get_tire_info(),
            tire_info_layer_2=self.tire_info_layer_2,
        )

    
    def _compute_C_matrix_full(self) -> np.ndarray:
        """
        Compute full C matrix for GPS+IMU measurements.
        
        Measurements: [vx, r, ψ, X, Y] (5D)
        """
        C = np.zeros((self.MEAS_DIM_FULL, self.INTERNAL_STATE_DIM))
        C[0, IDX_VX] = 1.0   # vx measurement
        C[1, IDX_R] = 1.0    # r measurement
        C[2, IDX_PSI] = 1.0  # ψ measurement
        C[3, IDX_X] = 1.0    # X measurement
        C[4, IDX_Y] = 1.0    # Y measurement
        return C
    
    def _compute_C_matrix_imu(self) -> np.ndarray:
        """
        Compute IMU-only C matrix.
        
        Measurements: [vx, r] (2D)
        """
        C = np.zeros((self.MEAS_DIM_IMU, self.INTERNAL_STATE_DIM))
        C[0, IDX_VX] = 1.0   # vx measurement
        C[1, IDX_R] = 1.0    # r measurement
        return C
    
    def _layer1_predict(self, u: np.ndarray, dt: float) -> np.ndarray:
        """
        Get Layer 1's one-step prediction WITHOUT updating.
        
        This computes: x̂₁(k+1|k) = f_nom(x̂₁(k|k), u) + E·ŵ
        """
        # Get current Layer 1 state and residuals
        x1 = self.layer1.x_hat.copy()
        w = self.layer1.w_hat.copy()
        
        vx = x1[IDX_VX]
        delta = u[0]
        
        # Use Layer 1's matrices
        A_a, B_a, C_a = self.layer1.compute_augmented_matrices(vx, delta)
        
        # Extract A, B, E from augmented matrices
        A = A_a[:6, :6]
        B = B_a[:6, :]
        E = A_a[:6, 6:]
        
        # Euler prediction (simple)
        x_pred = x1 + dt * (A @ x1 + B @ u + E @ w)
        
        return x_pred
    
    def _prepare_nn_input(self, x: np.ndarray, u: np.ndarray, 
                          tire_info: Dict, vx_meas: float, r_meas: float) -> np.ndarray:
        """
        Prepare NN input vector.
        
        z = [vx, vy, psi, r, delta, accel, alpha_f, alpha_r, vx_meas, r_meas]
        """
        return np.array([
            x[IDX_VX],                    # Estimated vx
            x[IDX_VY],                    # Estimated vy
            x[IDX_PSI],                   # Estimated psi
            x[IDX_R],                     # Estimated r
            u[0],                         # Steering angle delta
            u[1],                         # Acceleration
            tire_info['alpha_f'],         # Front slip angle
            tire_info['alpha_r'],         # Rear slip angle
            vx_meas,                      # Measured vx
            r_meas,                       # Measured r
        ])
    
    def _compute_gating_weight(self, x: np.ndarray, y: np.ndarray) -> float:
        """
        Compute training sample weight based on observability conditions.
        
        Returns alpha ∈ [0, 1] where:
        - 0 = don't use this sample
        - 1 = fully trustworthy sample
        
        Gating conditions:
        - vx < vx_min → α = 0
        - Yaw wrap detected → α = 0
        - High innovation → reduce α
        """
        alpha = 1.0
        
        # Low speed condition
        vx = abs(x[IDX_VX])
        if vx < self.vx_min_train:
            return 0.0
        
        # Yaw wrap detection
        psi = x[IDX_PSI]
        psi_jump = abs(psi - self._prev_psi)
        if psi_jump > self.yaw_jump_threshold:
            alpha = 0.0
        self._prev_psi = psi
        
        # Innovation norm (measurement prediction error)
        # Simple check: compare measured vs estimated vx and r
        vx_error = abs(y[0] - x[IDX_VX])
        r_error = abs(y[1] - x[IDX_R])
        innov_norm = np.sqrt(vx_error**2 + r_error**2)
        
        if innov_norm > self.innovation_max:
            alpha *= 0.5  # Reduce but don't zero out
        
        return alpha
    
    def _predict_disturbance(self, z: np.ndarray) -> np.ndarray:
        """
        Get NN disturbance prediction.
        
        Args:
            z: NN input vector
            
        Returns:
            d = [d_vx, d_vy, d_r]
        """
        self.nn.eval()
        with torch.no_grad():
            z_tensor = torch.tensor(z, dtype=torch.float32).unsqueeze(0)
            d = self.nn(z_tensor).squeeze().numpy()
        return d
    
    def _train_step(self):
        """
        Perform one training step on a batch from the buffer.
        """
        z_batch, r_batch, w_batch = self.buffer.sample_batch(self.batch_size)
        
        if z_batch is None:
            return
        
        self.nn.train()
        
        # Convert to tensors
        z_tensor = torch.tensor(z_batch, dtype=torch.float32)
        r_tensor = torch.tensor(r_batch, dtype=torch.float32)
        w_tensor = torch.tensor(w_batch, dtype=torch.float32).unsqueeze(1)
        
        # Forward pass
        d_pred = self.nn(z_tensor)
        
        # Weighted MSE loss
        # L = Σ αᵢ ‖NN(zᵢ) - rᵢ‖²
        loss = (w_tensor * (d_pred - r_tensor)**2).mean()
        
        # Backward pass
        self.optimizer.zero_grad()
        loss.backward()
        
        # Gradient clipping for stability
        torch.nn.utils.clip_grad_norm_(self.nn.parameters(), max_norm=1.0)
        
        self.optimizer.step()
        
        # Track loss
        self.loss_history.append(loss.item())
        self.train_step += 1
    
    # =========================================================
    # GETTERS
    # =========================================================
    
    def get_state(self) -> np.ndarray:
        """Get 4D state [X, Y, ψ, v_x] for base class compatibility."""
        return self._extract_4d_state()
    
    def get_state_6d(self) -> np.ndarray:
        """Get full 6D internal state [vx, vy, ψ, r, X, Y]."""
        return self.state_nn_6d.copy()
    
    def get_layer1_state(self) -> np.ndarray:
        """Get Layer 1 (teacher) state estimate"""
        return self.layer1.get_state()
    
    def get_disturbance(self) -> np.ndarray:
        """Get current disturbance estimate [d_vx, d_vy, d_r]"""
        return self.d_hat.copy()
    
    def get_tire_residuals(self) -> np.ndarray:
        """Get tire residuals from Layer 1 [w_r, w_f]"""
        return self.layer1.get_tire_residuals()
    
    def get_tire_info(self) -> Dict:
        """Get tire information from Layer 1"""
        return self.layer1.get_tire_info()
    
    def get_training_loss(self) -> float:
        """Get most recent training loss"""
        if len(self.loss_history) > 0:
            return self.loss_history[-1]
        return 0.0
    
    def get_info(self) -> Dict:
        """Get full estimator info for logging/debugging."""
        return {
            # Layer 1 (Teacher)
            'x_layer1': self.layer1.get_state(),
            'w_tire': self.layer1.get_tire_residuals(),
            'tire_info': self.layer1.get_tire_info(),
            
            # Layer 2 (Student)
            'x_corrected': self.x_corrected.copy(),
            'd_hat': self.d_hat.copy(),
            
            # Training stats
            'buffer_size': len(self.buffer),
            'train_steps': self.train_step,
            'last_loss': self.get_training_loss(),
        }
    
    def reset(self, initial_state: Optional[np.ndarray] = None, initial_pose: Optional[np.ndarray] = None):
        """
        Reset estimator.
        
        Args:
            initial_state: Optional initial 6D state [vx, vy, psi, r, X, Y]
            initial_pose: Optional initial 3D pose [X, Y, ψ] (base class compatible)
        """
        # Reset Layer 1
        self.layer1.reset(initial_state)
        
        # Reset Layer 2 observer state
        if initial_state is not None:
            self.state_nn_6d = np.array(initial_state).flatten()[:self.INTERNAL_STATE_DIM]
            if len(self.state_nn_6d) < self.INTERNAL_STATE_DIM:
                self.state_nn_6d = np.concatenate([self.state_nn_6d, np.zeros(self.INTERNAL_STATE_DIM - len(self.state_nn_6d))])
        elif initial_pose is not None:
            self.state_nn_6d = np.zeros(self.INTERNAL_STATE_DIM)
            self.state_nn_6d[IDX_X] = initial_pose[0]
            self.state_nn_6d[IDX_Y] = initial_pose[1]
            self.state_nn_6d[IDX_PSI] = initial_pose[2]
        else:
            self.state_nn_6d = np.zeros(self.INTERNAL_STATE_DIM)
        
        # Update aliases
        self.x_hat = self.state_nn_6d
        self.x_corrected = self.state_nn_6d.copy()
        
        # Reset disturbance estimate
        self.d_hat = np.zeros(3)
        
        # Reset gating state
        self._prev_psi = 0.0
        
        # Clear buffer but keep trained NN
        self.buffer = LearningBuffer(max_size=self.buffer.max_size)
        
        # Reset update counter
        self.update_count = 0
        
        # Reset base class state
        self.state = self._extract_4d_state()
    
    def close_recorder(self):
        """Close and save recording data if recorder is active."""
        if self.recorder is not None:
            self.recorder.close()
            self.recorder = None

    
    # =========================================================
    # MODEL I/O
    # =========================================================
    
    def save_model(self, path: str):
        """Save the neural network model."""
        save_model(self.nn, path)
    
    def load_model(self, path: str):
        """Load a pretrained neural network model."""
        self.nn = load_model(
            filepath=path,
            input_dim=self.input_dim,
            hidden_dim=self.hidden_dim, # self.hidden_dim needs to be stored in __init__? 
                                      # It is passed to __init__ but not stored.
                                      # I should store it or hardcode.
                                      # Let's check __init__.
            output_dim=self.output_dim
        )
        # Re-create optimizer
        self.optimizer = create_optimizer(self.nn, learning_rate=self.learning_rate)


# =============================================================================
# FACTORY FUNCTION
# =============================================================================

def create_neural_layer2(sample_time: float = 0.02,
                         vehicle_params: Optional[Dict] = None,
                         enable_learning: bool = True,
                         **kwargs) -> NeuralLayer2Simple:
    """
    Create a Layer-2 Neural Estimator.
    
    Args:
        sample_time: Sample time [s]
        vehicle_params: Vehicle parameters
        enable_learning: Whether to train online
        **kwargs: Additional arguments (hidden_dim, learning_rate, etc.)
        
    Returns:
        Configured NeuralLayer2Simple instance
    """
    return NeuralLayer2Simple(
        sample_time=sample_time,
        vehicle_params=vehicle_params,
        enable_learning=enable_learning,
        **kwargs
    )


# =============================================================================
# EXAMPLE USAGE
# =============================================================================

if __name__ == "__main__":
    """Example demonstrating the 2-layer estimator."""
    
    print("=== 2-Layer Neural Observer Demo ===\n")
    
    # Create estimator
    estimator = NeuralLayer2Simple(
        sample_time=0.02,
        hidden_dim=32,
        enable_learning=True
    )
    
    # Initialize
    estimator.reset(initial_state=np.array([1.0, 0, 0, 0, 0, 0]))
    
    print("Architecture:")
    print("  Layer 1 (Teacher): TireEstimatorSimple")
    print("    - State: [vx, vy, psi, r, X, Y]")
    print("    - Output: tire residuals [w_r, w_f]")
    print("")
    print("  Layer 2 (Student): Neural Network")
    print("    - Input: [vx, vy, psi, r, delta, accel, alpha_f, alpha_r, vx_meas, r_meas]")
    print("    - Output: disturbances [d_vx, d_vy, d_r]")
    print("")
    
    # Simulate several steps
    print("Simulation steps:")
    for i in range(10):
        # Simulated measurements [vx, r, psi, X, Y, ay]
        # measurement = [1.0 + 0.05*i, 0.1, 0.03*i, 0.5*i, 0.1*i, 0.3]
        vx = 1.0 + 0.05*i
        r = 0.1
        psi = 0.03*i
        X = 0.5*i
        Y = 0.1*i
        ay = 0.3
        
        # Controls
        steering = 0.1
        throttle = 0.5
        
        # Prepare GPS data
        gps_data = {'x': X, 'y': Y, 'theta': psi, 'valid': True}
        acceleration = np.array([0.0, ay])
        
        # Update (returns bool)
        success = estimator.update(
            motor_tach=vx,
            steering=steering,
            throttle=throttle,
            dt=0.02,
            gyro_z=r,
            gps_data=gps_data,
            acceleration=acceleration
        )
        
        state_corrected = estimator.get_state_6d()
        info = estimator.get_info()
        
        if i % 3 == 0:
            print(f"\nStep {i+1}:")
            print(f"  Layer 1 state: vx={info['x_layer1'][0]:.2f}, vy={info['x_layer1'][1]:.3f}")
            print(f"  Tire residuals: w_r={info['w_tire'][0]:.3f}, w_f={info['w_tire'][1]:.3f}")
            print(f"  Disturbance: d_vx={info['d_hat'][0]:.4f}, d_vy={info['d_hat'][1]:.4f}, d_r={info['d_hat'][2]:.4f}")
            print(f"  Corrected state: vx={state_corrected[0]:.2f}, vy={state_corrected[1]:.3f}")
            print(f"  Buffer size: {info['buffer_size']}, Train steps: {info['train_steps']}")
    
    print("\n=== Demo Complete ===")
