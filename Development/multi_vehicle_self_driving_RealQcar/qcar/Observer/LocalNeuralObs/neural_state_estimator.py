"""
Neural State Estimator for Local Observer (Second Layer)

Implements a Luenberger-style observer enhanced with neural network learning
to compensate for unknown dynamics and disturbances in vehicle motion.

This is the SECOND LAYER neural observer that uses f_nn for disturbance compensation.
Uses 6D internal state matching first-layer observers (qLPV, Differentiator-UIO).

Internal 6D State: x̂ = [v_x, v_y, ψ, r, X, Y]ᵀ
    - v_x: Longitudinal velocity (body frame)
    - v_y: Lateral velocity (body frame)
    - ψ: Yaw angle
    - r: Yaw rate
    - X: Global X position
    - Y: Global Y position

Output 4D State (for LocalStateEstimatorBase): [X, Y, ψ, v_x]

NN Input: [v_x, v_y, ψ, r, δ, a] or [v_x, v_y, ψ, r, δ, a, a_x, a_y]
NN Output: f_nn = [w_r, w_f]ᵀ (tire force residuals)

Observer Equation:
    x̂[k+1] = A(ρ)·x̂[k] + B(ρ)·u[k] + E(ρ)·f_nn[k] + L·(y[k] - C·x̂[k])
"""

import numpy as np
import torch
import time
import os
from typing import Optional, Dict, Tuple
from dataclasses import dataclass

# Import local modules
import sys
from pathlib import Path
parent_dir = Path(__file__).parent
sys.path.insert(0, str(parent_dir))

from neural_network import (
    NeuralObserverNet, LearningBatch, ModelQueue,
    save_model, load_model, create_optimizer
)
from gradient_solver import GradientSolver, create_weight_matrix
from uio_observers import create_first_layer_observer, FirstLayerObserverBase

# Import base class from parent directory
sys.path.insert(0, str(parent_dir.parent))
from local_state_estimators import LocalStateEstimatorBase


@dataclass
class SchedulingParameters:
    """Scheduling parameters ρ for qLPV system"""
    inv_vx: float      # 1/v_x
    sin_delta: float   # sin(δ)
    cos_delta: float   # cos(δ)
    vx: float          # v_x
    vy: float          # v_y
    sin_psi: float     # sin(ψ)
    cos_psi: float     # cos(ψ)

    @classmethod
    def from_state_and_input(cls, state: np.ndarray, delta: float, 
                              min_vx: float = 0.5) -> 'SchedulingParameters':
        """Compute scheduling parameters from state and steering input"""
        vx = max(abs(state[0]), min_vx)
        vy = state[1]
        psi = state[2]
        
        return cls(
            inv_vx=1.0 / vx,
            sin_delta=np.sin(delta),
            cos_delta=np.cos(delta),
            vx=vx,
            vy=vy,
            sin_psi=np.sin(psi),
            cos_psi=np.cos(psi)
        )


class NeuralLuenbergerEstimator(LocalStateEstimatorBase):
    """
    Neural network-enhanced Luenberger observer (Second Layer)
    
    Combines first-layer observer (qLPV/Differentiator-UIO) with neural network
    learning to estimate unknown tire forces and disturbances.
    
    Uses 6D internal state: [v_x, v_y, ψ, r, X, Y]
    Outputs 4D state for LocalStateEstimatorBase: [X, Y, ψ, v_x]
    
    Key features:
        - 6D internal state matching first-layer observers
        - Neural network predicts tire force residuals [w_r, w_f]
        - Optional IMU acceleration inputs for NN
        - Compatible with LocalStateEstimatorBase interface (4D output)
    """
    
    # 6D internal state indices
    IDX_VX = 0
    IDX_VY = 1
    IDX_PSI = 2
    IDX_R = 3
    IDX_X = 4
    IDX_Y = 5
    INTERNAL_STATE_DIM = 6
    
    def __init__(self, initial_pose: Optional[np.ndarray] = None,
                 config: Dict = None, logger=None):
        """
        Initialize neural Luenberger estimator
        
        Args:
            initial_pose: Initial pose [X, Y, ψ] (global coordinates)
            config: Configuration dictionary with neural network parameters
            logger: Logger instance
        """
        super().__init__(initial_pose, logger)
        
        # Load configuration
        config = config or {}
        self.config = self._load_config(config)
        
        # Neural network setup
        self.input_dim = self.config['input_dim']
        self.hidden_dim = self.config['hidden_dim']
        self.output_dim = self.config['output_dim']
        self.use_acceleration = self.config.get('use_acceleration', False)
        
        # Adjust input dim if using acceleration
        if self.use_acceleration:
            self.input_dim = 8  # [v_x, v_y, ψ, r, δ, a, a_x, a_y]
        
        # Initialize neural network
        self.model = NeuralObserverNet(
            self.input_dim,
            self.hidden_dim,
            self.output_dim
        )
        
        # Load pretrained model if specified
        if self.config['load_pretrained'] and os.path.exists(self.config['model_path']):
            try:
                self.model = load_model(
                    self.config['model_path'],
                    self.input_dim,
                    self.hidden_dim,
                    self.output_dim
                )
                if self.logger:
                    self.logger.logger.info(f"Loaded pretrained neural observer model from {self.config['model_path']}")
            except Exception as e:
                if self.logger:
                    self.logger.log_error("Failed to load pretrained model", e)
        
        # Optimizer
        self.optimizer = create_optimizer(
            self.model,
            self.config['learning_rate'],
            self.config['weight_decay']
        )
        
        # Vehicle parameters
        self.vehicle_params = self._default_vehicle_params()
        if self.config.get('vehicle_params') is not None:
            self.vehicle_params.update(self.config['vehicle_params'])
        
        # Extract commonly used parameters
        self.lf = self.vehicle_params['lf']
        self.lr = self.vehicle_params['lr']
        self.m = self.vehicle_params['m']
        self.Iz = self.vehicle_params['Iz']
        self.Cf = self.vehicle_params['Cf']
        self.Cr = self.vehicle_params['Cr']
        
        # 6D Internal state: [v_x, v_y, ψ, r, X, Y]
        self.state_nn_6d = np.zeros(self.INTERNAL_STATE_DIM)
        if initial_pose is not None:
            # Map [X, Y, ψ] to internal state
            self.state_nn_6d[self.IDX_X] = initial_pose[0]
            self.state_nn_6d[self.IDX_Y] = initial_pose[1]
            self.state_nn_6d[self.IDX_PSI] = initial_pose[2] if len(initial_pose) > 2 else 0.0
        
        # Neural network output (tire residuals)
        self.f_nn = np.zeros((self.output_dim, 1))  # [w_r, w_f]
        
        # Observer gains (6x6 for 6D state)
        self._initialize_observer_gains()
        
        # First-layer observer (optional, for two-layer architecture)
        self.use_first_layer = self.config.get('use_first_layer', True)
        self.first_layer_observer = None
        if self.use_first_layer:
            self._initialize_first_layer_observer()
        
        # Gradient solver for neural network training
        self._initialize_gradient_solver()
        
        # Learning components
        self.learning_mode = self.config['learning_mode']
        if self.learning_mode == 'learningby_dict':
            self.learning_batch = LearningBatch(self.config['dict_size'])
        elif self.learning_mode == 'continuous_learning':
            self.model_queue = ModelQueue(
                self.input_dim,
                self.output_dim,
                queue_size=3,
                hidden_dim=self.hidden_dim
            )
        else:
            self.learning_batch = None
            self.model_queue = None
        
        # Sensitivity matrix for gradient computation
        self.dx_df = np.zeros((self.INTERNAL_STATE_DIM, self.output_dim))
        
        # Batch training
        self.batch_size = self.config['batch_size']
        self.batch_loss = 0.0
        self.batch_count = 0
        
        # Loss tracking
        self.loss_history = []
        self.update_count = 0
        
        # Weight matrix for loss function (6D)
        self.weight_matrix = create_weight_matrix({
            'v_x': self.config['weight_vx'],
            'v_y': self.config['weight_vy'],
            'psi': self.config['weight_psi'],
            'psi_dot': self.config['weight_psi_dot'],
            'X': self.config.get('weight_X', 1.0),
            'Y': self.config.get('weight_Y', 1.0)
        })
        
        # Weight matrices for composite UIO loss
        self.weight_matrices = {
            'T_ref': create_weight_matrix({
                'v_x': self.config.get('weight_ref_vx', 5.0),
                'v_y': self.config.get('weight_ref_vy', 5.0),
                'psi': self.config.get('weight_ref_psi', 10.0),
                'psi_dot': self.config.get('weight_ref_r', 5.0),
                'X': self.config.get('weight_ref_X', 10.0),
                'Y': self.config.get('weight_ref_Y', 10.0)
            }),
            'T_y': np.eye(self.INTERNAL_STATE_DIM),
            'T_uio': create_weight_matrix({
                'v_x': self.config.get('weight_uio_vx', 5.0),
                'v_y': self.config.get('weight_uio_vy', 10.0),
                'psi': self.config.get('weight_uio_psi', 5.0),
                'psi_dot': self.config.get('weight_uio_r', 10.0),
                'X': self.config.get('weight_uio_X', 1.0),
                'Y': self.config.get('weight_uio_Y', 1.0)
            })
        }
        
        # Trajectory reference for composite UIO loss [X, Y, ψ, v_ref] or 6D
        self.trajectory_ref = None
        self.trajectory_ref_6d = np.zeros(self.INTERNAL_STATE_DIM)
        
        # First-layer state for composite loss
        self.state_uio = np.zeros(self.INTERNAL_STATE_DIM)
        
        # Minimum velocity threshold
        self.min_vx = 0.5
    
    def _load_config(self, config: Dict) -> Dict:
        """Load configuration with defaults"""
        default_config = {
            'input_dim': 6,  # [v_x, v_y, ψ, r, δ, a]
            'hidden_dim': 24,
            'output_dim': 2,  # [w_r, w_f]
            'learning_rate': 0.005,
            'batch_size': 3,
            'weight_decay': 0.0,
            'observer_gain': 0.5,
            'loss_type': 'measurement_full',
            'weight_vx': 10,
            'weight_vy': 20000,
            'weight_psi': 10,
            'weight_psi_dot': 20000,
            'weight_X': 1.0,
            'weight_Y': 1.0,
            'lambda_regularization': 0.0,
            # Composite UIO loss weights
            'weight_ref_vx': 5.0,
            'weight_ref_vy': 5.0,
            'weight_ref_psi': 10.0,
            'weight_ref_r': 5.0,
            'weight_ref_X': 10.0,
            'weight_ref_Y': 10.0,
            'weight_uio_vx': 5.0,
            'weight_uio_vy': 10.0,
            'weight_uio_psi': 5.0,
            'weight_uio_r': 10.0,
            'weight_uio_X': 1.0,
            'weight_uio_Y': 1.0,
            'learning_mode': 'learningby_dict',
            'dict_size': 20,
            'model_path': 'trained_data/neural_observer_model.pt',
            'load_pretrained': False,
            'sample_time': 0.02,
            # First-layer observer config
            'use_first_layer': True,
            'first_layer_type': 'qlpv',  # 'qlpv' or 'differentiator_uio'
            'vehicle_params': None,
            # Acceleration input (optional)
            'use_acceleration': False,
        }
        
        default_config.update(config)
        return default_config
    
    def _default_vehicle_params(self) -> Dict:
        """Default vehicle parameters (QCar scale)"""
        return {
            'lf': 0.11,      # Distance CG to front axle [m]
            'lr': 0.11,      # Distance CG to rear axle [m]
            'm': 3.5,        # Mass [kg]
            'Iz': 0.05,      # Yaw inertia [kg·m²]
            'Cf': 50.0,      # Front cornering stiffness [N/rad]
            'Cr': 50.0,      # Rear cornering stiffness [N/rad]
            'vx_min': 0.5,   # Minimum velocity
        }
    
    def _initialize_observer_gains(self):
        """Initialize observer gain matrices for 6D state"""
        gain = self.config['observer_gain']
        # Diagonal observer gain for 6D state
        self.L = np.diag([gain * 2, gain * 2, gain, gain * 2, gain * 0.5, gain * 0.5])
    
    def _initialize_first_layer_observer(self):
        """Initialize first-layer observer (qLPV or Differentiator-UIO)"""
        try:
            observer_type = self.config.get('first_layer_type', 'qlpv')
            
            self.first_layer_observer = create_first_layer_observer(
                observer_type=observer_type,
                sample_time=self.config['sample_time'],
                vehicle_params=self.vehicle_params
            )
            
            if self.logger:
                self.logger.logger.info(f"Initialized first-layer observer ({observer_type})")
        
        except Exception as e:
            if self.logger:
                self.logger.log_error("Failed to initialize first-layer observer", e)
            self.use_first_layer = False
    
    def _initialize_gradient_solver(self):
        """Initialize gradient solver for neural network training"""
        # C matrix for 6D state (identity for full state observation)
        C = np.eye(self.INTERNAL_STATE_DIM)
        
        # E matrix for tire residual injection (6x2)
        E = self._compute_E_matrix(None)
        
        observer_matrices = {
            'A': np.eye(self.INTERNAL_STATE_DIM),
            'C': C,
            'D': E,  # D in gradient solver corresponds to E (residual injection)
            'K': self.L
        }
        
        self.gradient_solver = GradientSolver(
            observer_matrices,
            self.config['sample_time']
        )
    
    def _compute_scheduling_params(self, state: np.ndarray, delta: float) -> SchedulingParameters:
        """Compute scheduling parameters from current state and input"""
        return SchedulingParameters.from_state_and_input(state, delta, self.min_vx)
    
    def _compute_A_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """Compute state matrix A(ρ) for qLPV system (6x6)"""
        A = np.zeros((self.INTERNAL_STATE_DIM, self.INTERNAL_STATE_DIM))
        
        inv_vx = rho.inv_vx
        cos_d = rho.cos_delta
        vx = rho.vx
        vy = rho.vy
        cos_psi = rho.cos_psi
        sin_psi = rho.sin_psi
        
        # v_y dynamics
        A[1, 1] = -(self.Cr + self.Cf * cos_d) / (self.m * vx)
        A[1, 3] = -(self.Cf * self.lf * cos_d - self.Cr * self.lr) / (self.m * vx) - vx
        
        # ψ dynamics: ψ̇ = r
        A[2, 3] = 1.0
        
        # r dynamics
        A[3, 1] = -(self.Cf * self.lf * cos_d - self.Cr * self.lr) / (self.Iz * vx)
        A[3, 3] = -(self.Cf * self.lf**2 * cos_d + self.Cr * self.lr**2) / (self.Iz * vx)
        
        # X dynamics: Ẋ = v_x·cos(ψ) - v_y·sin(ψ)
        A[4, 0] = cos_psi
        A[4, 1] = -sin_psi
        A[4, 2] = -vx * sin_psi - vy * cos_psi
        
        # Y dynamics: Ẏ = v_x·sin(ψ) + v_y·cos(ψ)
        A[5, 0] = sin_psi
        A[5, 1] = cos_psi
        A[5, 2] = vx * cos_psi - vy * sin_psi
        
        return A
    
    def _compute_B_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """Compute input matrix B(ρ) (6x2)"""
        B = np.zeros((self.INTERNAL_STATE_DIM, 2))
        
        cos_d = rho.cos_delta
        
        # v_x: affected by acceleration
        B[0, 1] = 1.0
        
        # v_y: affected by steering
        B[1, 0] = self.Cf * cos_d / self.m
        
        # r: affected by steering
        B[3, 0] = self.Cf * self.lf * cos_d / self.Iz
        
        return B
    
    def _compute_E_matrix(self, rho: Optional[SchedulingParameters]) -> np.ndarray:
        """Compute residual injection matrix E(ρ) (6x2)"""
        E = np.zeros((self.INTERNAL_STATE_DIM, 2))
        
        cos_d = 1.0 if rho is None else rho.cos_delta
        
        # v_y: both residuals contribute
        E[1, 0] = 1.0 / self.m
        E[1, 1] = cos_d / self.m
        
        # r: both residuals create moment
        E[3, 0] = -self.lr / self.Iz
        E[3, 1] = self.lf * cos_d / self.Iz
        
        return E
    
    def _compute_C_matrix(self) -> np.ndarray:
        """Compute output matrix C (identity for full state)"""
        return np.eye(self.INTERNAL_STATE_DIM)
    
    def _prepare_nn_input(self, steering: float, throttle: float, 
                          accel: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Prepare neural network input vector
        
        Args:
            steering: Steering angle δ
            throttle: Throttle/acceleration command
            accel: Optional IMU acceleration [a_x, a_y]
        
        Returns:
            NN input array (6D or 8D)
        """
        vx, vy, psi, r, X, Y = self.state_nn_6d
        
        if self.use_acceleration and accel is not None:
            # 8D input: [v_x, v_y, ψ, r, δ, a, a_x, a_y]
            nn_input = np.array([
                vx, vy, psi, r, steering, throttle,
                accel[0], accel[1]
            ]).reshape(self.input_dim, 1)
        else:
            # 6D input: [v_x, v_y, ψ, r, δ, a]
            nn_input = np.array([
                vx, vy, psi, r, steering, throttle
            ]).reshape(self.input_dim, 1)
        
        return nn_input
    
    def update(self, motor_tach: float, steering: float, throttle: float, dt: float,
               gyro_z: float = 0.0, gps_data: Optional[Dict] = None,
               accel: Optional[np.ndarray] = None) -> bool:
        """
        Update state estimate with sensor data and neural learning
        
        Args:
            motor_tach: Motor tachometer reading (velocity v_x)
            steering: Steering angle command δ
            throttle: Throttle command (acceleration)
            dt: Time step
            gyro_z: Z-axis gyroscope reading (yaw rate r)
            gps_data: Optional GPS data dict with keys: x, y, theta, valid
            accel: Optional IMU acceleration [a_x, a_y]
        
        Returns:
            True if update successful
        """
        try:
            # === Prepare NN Input ===
            nn_input = self._prepare_nn_input(steering, throttle, accel)
            
            # === Get Neural Network Prediction (f_nn = [w_r, w_f]) ===
            nn_input_tensor = torch.from_numpy(nn_input).float()
            
            if self.learning_mode == 'continuous_learning' and self.model_queue is not None:
                if len(self.model_queue.models) > 0:
                    self.f_nn = self.model_queue.predict(nn_input_tensor).detach().numpy()
                else:
                    self.f_nn = self.model(nn_input_tensor).detach().numpy()
            else:
                self.f_nn = self.model(nn_input_tensor).detach().numpy()
            
            # === First-Layer Observer Update (if enabled) ===
            w_hat = self.f_nn.squeeze()  # Default: use NN prediction
            
            if self.use_first_layer and self.first_layer_observer is not None:
                # Prepare measurement for first-layer observer
                # Format: [v_x, r, ψ, X, Y, a_y]
                vx_meas = motor_tach
                r_meas = gyro_z
                psi_meas = self.state_nn_6d[self.IDX_PSI]
                X_meas = self.state_nn_6d[self.IDX_X]
                Y_meas = self.state_nn_6d[self.IDX_Y]
                ay_meas = r_meas * vx_meas  # Approximate a_y = r * v_x
                
                if gps_data is not None and gps_data.get('valid', False):
                    psi_meas = gps_data.get('theta', psi_meas)
                    X_meas = gps_data.get('x', X_meas)
                    Y_meas = gps_data.get('y', Y_meas)
                
                if accel is not None:
                    ay_meas = accel[1]  # Use actual a_y from IMU
                
                measurement = np.array([vx_meas, r_meas, psi_meas, X_meas, Y_meas, ay_meas])
                control = np.array([steering, throttle])
                
                # Update first-layer observer
                state_uio, w_uio = self.first_layer_observer.update(
                    measurement, control, 
                    f_nn=self.f_nn,
                    acceleration=accel
                )
                
                # Store first-layer state for composite loss calculation
                self.state_uio = state_uio.copy()
                
                # Use first-layer w estimate for observer dynamics
                w_hat = w_uio
            
            # === Compute System Matrices ===
            rho = self._compute_scheduling_params(self.state_nn_6d, steering)
            A = self._compute_A_matrix(rho)
            B = self._compute_B_matrix(rho)
            E = self._compute_E_matrix(rho)
            C = self._compute_C_matrix()
            
            # === State Prediction ===
            # x̂_pred = x̂ + dt·(A·x̂ + B·u + E·w)
            u = np.array([steering, throttle])
            x_dot = A @ self.state_nn_6d + B @ u + E @ w_hat
            state_pred = self.state_nn_6d + dt * x_dot
            
            # === Measurement Update ===
            if gps_data is not None and gps_data.get('valid', False):
                # Build measurement vector
                measurement = np.array([
                    motor_tach,  # v_x
                    0.0,  # v_y (not directly measured)
                    gps_data.get('theta', state_pred[self.IDX_PSI]),  # ψ
                    gyro_z,  # r
                    gps_data.get('x', state_pred[self.IDX_X]),  # X
                    gps_data.get('y', state_pred[self.IDX_Y])  # Y
                ])
                
                # Innovation: y - C·x̂_pred
                innovation = measurement - C @ state_pred
                # Don't correct v_y with innovation (not measured)
                innovation[1] = 0.0
                
                # Correction: x̂ = x̂_pred + L·innovation
                self.state_nn_6d = state_pred + self.L @ innovation
            else:
                # Prediction only
                self.state_nn_6d = state_pred
            
            # === Sync with Base Class State (4D) ===
            self.state = self._extract_4d_state()
            
            # === Update Sensitivity for Gradient Computation ===
            self.dx_df = self.gradient_solver.gradient_solver_discrete(self.dx_df)
            
            # === Neural Network Training ===
            if gps_data is not None and gps_data.get('valid', False):
                self._train_network(nn_input, w_hat, gps_data, motor_tach)
            
            self.last_update_time = time.time()
            self.update_count += 1
            
            return True
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Neural observer update error", e)
            return False
    
    def _train_network(self, nn_input: np.ndarray, w_hat: np.ndarray,
                       gps_data: Dict, motor_tach: float):
        """Train neural network with computed gradients"""
        # Build full measurement for loss computation
        measurement_full = np.array([
            motor_tach,
            0.0,  # v_y
            gps_data.get('theta', self.state_nn_6d[self.IDX_PSI]),
            self.state_nn_6d[self.IDX_R],
            gps_data.get('x', self.state_nn_6d[self.IDX_X]),
            gps_data.get('y', self.state_nn_6d[self.IDX_Y])
        ]).reshape(-1, 1)
        
        f_uk = w_hat.reshape(-1, 1)
        
        # Choose loss function based on config
        loss_type = self.config.get('loss_type', 'measurement_full')
        
        if loss_type == 'composite_uio' and self.trajectory_ref is not None:
            # Use composite UIO loss with trajectory reference
            dL_df, loss = self.gradient_solver.chain_rule_composite_uio(
                self.trajectory_ref_6d,  # Reference trajectory
                measurement_full,         # Measurement
                self.state_nn_6d,          # Neural observer state
                self.state_uio,            # First-layer UIO state
                self.dx_df,
                f_uk,
                self.f_nn,
                self.weight_matrices,
                self.config['lambda_regularization']
            )
        else:
            # Use standard measurement loss
            dL_df, loss = self.gradient_solver.chain_rule_full_measurement(
                measurement_full,
                self.state_nn_6d,
                self.dx_df,
                f_uk,
                self.f_nn,
                self.weight_matrix,  # Already 6x6
                self.config['lambda_regularization']
            )
        
        if dL_df is None:
            return
        
        if self.learning_mode == 'learningby_dict':
            # Dictionary-based learning
            self.learning_batch.add_feature(
                nn_input,
                self.f_nn,
                self.state_nn_6d,
                measurement_full,
                f_uk,
                self.dx_df,
                self.update_count
            )
            
            if len(self.learning_batch) >= self.batch_size:
                self._train_on_batch(nn_input, dL_df)
        else:
            # Normal batch training
            self._accumulate_batch_loss(nn_input, dL_df)
    
    def _train_on_batch(self, nn_input: np.ndarray, dL_df: np.ndarray):
        """Train on accumulated batch"""
        total_gradient = np.zeros_like(dL_df)
        
        for feature, f_nn_old, state_hat, target, f_uk_old, dx_df_old, _ in self.learning_batch.feature_dict:
            feature_tensor = torch.from_numpy(feature).float()
            f_nn_new = self.model(feature_tensor).detach().numpy()
            
            dL_df_new, loss_new = self.gradient_solver.chain_rule_full_measurement(
                target, state_hat, dx_df_old, f_uk_old, f_nn_new,
                self.weight_matrix,  # Already 6x6
                self.config['lambda_regularization']
            )
            
            if dL_df_new is not None:
                total_gradient += dL_df_new
        
        avg_gradient = total_gradient / len(self.learning_batch)
        
        nn_input_tensor = torch.from_numpy(nn_input).float()
        loss_pytorch = self.model.myloss(self.model(nn_input_tensor), avg_gradient)
        
        self.optimizer.zero_grad()
        loss_pytorch.backward()
        self.optimizer.step()
        
        self.loss_history.append(loss_pytorch.item())
    
    def _accumulate_batch_loss(self, nn_input: np.ndarray, dL_df: np.ndarray):
        """Accumulate loss for batch training"""
        nn_input_tensor = torch.from_numpy(nn_input).float()
        loss_pytorch = self.model.myloss(self.model(nn_input_tensor), dL_df)
        
        self.batch_loss += loss_pytorch
        self.batch_count += 1
        
        if self.batch_count >= self.batch_size:
            avg_loss = self.batch_loss / self.batch_size
            
            self.optimizer.zero_grad()
            avg_loss.backward()
            self.optimizer.step()
            
            self.loss_history.append(avg_loss.item())
            
            self.batch_loss = 0.0
            self.batch_count = 0
    
    def _extract_4d_state(self) -> np.ndarray:
        """
        Extract 4D state for LocalStateEstimatorBase compatibility
        
        Maps: [v_x, v_y, ψ, r, X, Y] -> [X, Y, ψ, v_x]
        """
        vx, vy, psi, r, X, Y = self.state_nn_6d
        return np.array([X, Y, psi, vx])
    
    def get_state(self) -> np.ndarray:
        """
        Get current state estimate (4D for base class compatibility)
        
        Returns:
            numpy array [X, Y, ψ, v_x]
        """
        return self._extract_4d_state()
    
    def get_state_6d(self) -> np.ndarray:
        """
        Get full 6D internal state estimate
        
        Returns:
            numpy array [v_x, v_y, ψ, r, X, Y]
        """
        return self.state_nn_6d.copy()
    
    def get_tire_residuals(self) -> np.ndarray:
        """Get current tire residual estimates from NN [w_r, w_f]"""
        return self.f_nn.squeeze().copy()
    
    def set_trajectory_reference(self, ref_pose: np.ndarray, 
                                  ref_velocity: Optional[float] = None,
                                  ref_heading_rate: float = 0.0):
        """
        Set trajectory reference from controller for composite UIO loss
        
        This method should be called from the controller (e.g., FollowingPathState)
        to provide the current waypoint reference for the composite loss function.
        
        Args:
            ref_pose: Reference pose [X, Y] or [X, Y, θ] from StanleyController.get_reference_pose()
            ref_velocity: Reference velocity (optional, uses current if None)
            ref_heading_rate: Reference heading rate (optional)
        
        Example usage in following_path_state.py:
            p_ref, th_ref = self.steering_controller.get_reference_pose()
            observer.set_trajectory_reference(
                np.array([p_ref[0], p_ref[1], th_ref]),
                ref_velocity=self.target_speed
            )
        """
        self.trajectory_ref = ref_pose.copy()
        
        # Build 6D reference: [v_x, v_y, ψ, r, X, Y]
        if len(ref_pose) >= 3:
            # [X, Y, θ] format
            self.trajectory_ref_6d[self.IDX_X] = ref_pose[0]
            self.trajectory_ref_6d[self.IDX_Y] = ref_pose[1]
            self.trajectory_ref_6d[self.IDX_PSI] = ref_pose[2]
        else:
            # [X, Y] format only
            self.trajectory_ref_6d[self.IDX_X] = ref_pose[0]
            self.trajectory_ref_6d[self.IDX_Y] = ref_pose[1]
            self.trajectory_ref_6d[self.IDX_PSI] = self.state_nn_6d[self.IDX_PSI]
        
        # Set reference velocity
        if ref_velocity is not None:
            self.trajectory_ref_6d[self.IDX_VX] = ref_velocity
        else:
            self.trajectory_ref_6d[self.IDX_VX] = self.state_nn_6d[self.IDX_VX]
        
        # Set reference heading rate and lateral velocity
        self.trajectory_ref_6d[self.IDX_R] = ref_heading_rate
        self.trajectory_ref_6d[self.IDX_VY] = 0.0  # Assume zero lateral velocity at reference
    
    def get_trajectory_reference(self) -> Optional[np.ndarray]:
        """Get current trajectory reference (6D)"""
        if self.trajectory_ref is not None:
            return self.trajectory_ref_6d.copy()
        return None
    
    def reset(self, initial_pose: Optional[np.ndarray] = None):
        """Reset estimator state"""
        self.state_nn_6d = np.zeros(self.INTERNAL_STATE_DIM)
        
        if initial_pose is not None:
            # Map [X, Y, ψ] to internal state
            self.state_nn_6d[self.IDX_X] = initial_pose[0]
            self.state_nn_6d[self.IDX_Y] = initial_pose[1]
            if len(initial_pose) > 2:
                self.state_nn_6d[self.IDX_PSI] = initial_pose[2]
        
        # Sync base class state
        self.state = self._extract_4d_state()
        
        # Reset neural network state
        self.f_nn = np.zeros((self.output_dim, 1))
        self.dx_df = np.zeros((self.INTERNAL_STATE_DIM, self.output_dim))
        
        # Reset batch training
        self.batch_loss = 0.0
        self.batch_count = 0
        
        # Clear learning batch
        if self.learning_batch is not None:
            self.learning_batch.clear()
        
        # Reset first-layer observer
        if self.first_layer_observer is not None:
            self.first_layer_observer.reset(self.state_nn_6d)
    
    def save_model(self, filepath: Optional[str] = None):
        """Save trained model to file"""
        if filepath is None:
            filepath = self.config['model_path']
        
        save_model(self.model, filepath)
        
        if self.logger:
            self.logger.logger.info(f"Saved neural observer model to {filepath}")
    
    def get_loss_history(self) -> list:
        """Get training loss history"""
        return self.loss_history.copy()
