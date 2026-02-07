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
from typing import Dict, Optional, Tuple
from dataclasses import dataclass

# Import local modules
import sys
from pathlib import Path
parent_dir = Path(__file__).parent
sys.path.insert(0, str(parent_dir))

# Import CVXPY for LMI-based gain design
try:
    import cvxpy as cp
    CVXPY_AVAILABLE = True
except ImportError:
    CVXPY_AVAILABLE = False
    print("Warning: cvxpy not available. LMI-based gain design disabled, using default gains.")

# Import scipy for pole placement fallback
try:
    from scipy.signal import place_poles
    from scipy.linalg import solve_continuous_lyapunov
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

from neural_network import (
    NeuralObserverNet, LearningBatch, ModelQueue,
    save_model, load_model, create_optimizer
)
from gradient_solver import GradientSolver, create_weight_matrix
from config_loader import load_neural_obs_config, flatten_config, merge_with_overrides
# Import unified recorder from parent LocalNeuralObs directory
parent_local_neural_dir = parent_dir.parent
sys.path.insert(0, str(parent_local_neural_dir))
from neural_obs_recorder import NeuralObsRecorder

# Import first-layer observer from 1LayerObs directory
one_layer_dir = parent_dir.parent / "1LayerObs"
sys.path.insert(0, str(one_layer_dir))
from firstLayerObserverBase import create_first_layer_observer, FirstLayerObserverBase



from Design_LMI_neural import (
    # Discrete-time LMI 
    discretize_system_zoh,
    # Gain scheduler
    NeuralQLPVGainScheduler,
)

# Import base class from parent directory
try:
    # When imported as part of the Observer package
    from Observer.local_state_estimators import LocalStateEstimatorBase
except ImportError:
    # Fallback for direct execution or testing
    # qcar/Observer/LocalNeuralObs/2LayerObs -> qcar/Observer
    sys.path.insert(0, str(parent_dir.parent.parent))
    from local_state_estimators import LocalStateEstimatorBase

# Import centralized qLPV vehicle dynamics
from qlpv_vehicle_dynamics_obs import (
    SchedulingParameters,
    QLPVVehicleDynamicsObs,
    get_default_vehicle_params,
    IDX_VX, IDX_VY, IDX_PSI, IDX_R, IDX_X, IDX_Y, STATE_DIM,
    MEAS_IDX_VX, MEAS_IDX_R, MEAS_IDX_PSI, MEAS_IDX_X, MEAS_IDX_Y, MEAS_IDX_AY, MEAS_DIM,
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
    
    # 6D internal state indices (from qlpv_vehicle_dynamics_obs)
    IDX_VX = IDX_VX  # 0
    IDX_VY = IDX_VY  # 1
    IDX_PSI = IDX_PSI  # 2
    IDX_R = IDX_R  # 3
    IDX_X = IDX_X  # 4
    IDX_Y = IDX_Y  # 5
    INTERNAL_STATE_DIM = STATE_DIM  # 6
    
    # Measurement dimensions for observer design
    # Full measurements: [vx, r, ψ, X, Y] (5D) - directly measurable states
    # IMU-only: [vx, r] (2D) - always available
    MEAS_DIM_FULL = 5  # vx, r, ψ, X, Y
    MEAS_DIM_IMU = 2   # vx, r
    
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

        # Adjust output dim based on disturbance mode
        self.disturbance_mode = self.config.get('disturbance_mode', 'tire')
        expected_udim = 3 if self.disturbance_mode == 'general' else 2
        
        if self.output_dim != expected_udim:
            if self.logger:
                self.logger.logger.info(f"Overriding configured output_dim ({self.output_dim}) with {expected_udim} for mode '{self.disturbance_mode}'")
            self.output_dim = expected_udim
            # Update config to reflect reality
            self.config['output_dim'] = expected_udim
        
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
        
        # Minimum velocity threshold (from parameters_qcar.yaml)
        # Lower value allows estimation closer to zero when vehicle stops
        self.min_vx = self.vehicle_params['vx_min']
        
        # Centralized vehicle dynamics (single source of truth)
        # MUST be created before _initialize_gradient_solver which calls _compute_E_matrix
        self.dynamics = QLPVVehicleDynamicsObs(
            vehicle_params=self.vehicle_params,
            min_vx=self.min_vx,
            disturbance_mode=self.disturbance_mode
        )
        
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
        self.output_first_layer_only = self.config.get('output_first_layer_only', False)
        print(f"use_first_layer: {self.use_first_layer}")
        print(f"output_first_layer_only: {self.output_first_layer_only}")
        
        # Safety check: Cannot output first layer only if first layer is disabled
        if self.output_first_layer_only and not self.use_first_layer:
            if self.logger:
                self.logger.logger.warning("Config Warning: output_first_layer_only=True but use_first_layer=False. Disabling output_first_layer_only to prevent stall.")
            self.output_first_layer_only = False

        # Threshold for low-speed override (default 0.3 m/s)
        self.override_threshold = self.config.get('override_threshold', 0.1)

        self.first_layer_observer = None
        # if self.use_first_layer:
        # Still creat first layer observer for 2-layer architecture (even not use)
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
            # Push initial model so queue is not empty
            self.model_queue.push_model(self.model)
        else:
            self.learning_batch = None
            self.model_queue = None
        
        # Gradient computation method: 'autodiff' or 'analytical'
        self.gradient_method = self.config.get('gradient_method', 'autodiff')
        # Fall back to analytical if torch not available
        if self.gradient_method == 'autodiff' and not torch.cuda.is_available() and not hasattr(torch, 'Tensor'):
            self.gradient_method = 'analytical'
        
        # Sensitivity matrix for gradient computation (analytical method)
        self.dx_df = np.zeros((self.INTERNAL_STATE_DIM, self.output_dim))
        
        # Batch training
        self.batch_size = self.config['batch_size']
        self.batch_loss = 0.0  # For analytical
        self.batch_loss_autodiff = None  # For autodiff (torch tensor)
        self.batch_count = 0
        
        # Loss tracking
        self.loss_history = []
        self.update_count = 0
        
        # Weight matrix for loss function (6D)
        self.weight_matrix, self.weight_matrices = self._create_weight_matrices()
        
        # Trajectory reference for composite UIO loss (reduced dimension)
        # trajectory_ref: actual reference values [X, Y, ψ] or [X, Y, ψ, v_x]
        # ref_indices: indices in 6D state corresponding to trajectory_ref
        self.trajectory_ref = None  # Can be 3D or 4D depending on available reference
        self.ref_indices = None     # Indices in state_nn_6d that correspond to trajectory_ref
        
        # First-layer state for composite loss
        self.state_uio = np.zeros(self.INTERNAL_STATE_DIM)

        self.tire_info_layer_2 = {
            'Fyr_linear_est': 0,
            'Fyf_linear_est': 0,
            'alpha_r': 0,
            'alpha_f': 0,
        }
        
        # Data recording for later plotting
        self.recorder = None
        self._recording_start_time = 0.0
        if self.config.get('enable_recording', False):
            output_dir = self.config.get('recording_output_dir', 'neural_obs_recordings')
            self.recorder = NeuralObsRecorder(
                output_dir=output_dir, 
                mode='2layer',
                disturbance_mode=self.disturbance_mode
            )
            
            filename = self.config.get('recording_filename')
            append_mode = self.config.get('recording_append_mode', False)
            
            filepath = self.recorder.start(filename=filename, append=append_mode)
            
            if self.logger:
                if append_mode:
                   self.logger.logger.info(f"Appending neural observer recording to: {filepath}")
                else:
                   self.logger.logger.info(f"Started neural observer recording to: {filepath}")
            self._recording_start_time = time.time()
        
        # Ground truth provider for simulation (injected by fake_vehicle)
        self.ground_truth_provider = None
        self.logger.logger.info(f"Observer Neural 2-Layer initialized")


    def set_ground_truth_provider(self, provider):
        """Set a provider for ground truth state (e.g., MockQCar)"""
        self.ground_truth_provider = provider
    
    def _load_config(self, config: Dict) -> Dict:
        """Load configuration from YAML file and merge with overrides."""
        # Load from YAML file
        yaml_config = load_neural_obs_config()
        base_config = flatten_config(yaml_config)
        
        # Merge with runtime overrides
        return merge_with_overrides(base_config, config)
    
    def _create_weight_matrices(self) -> Tuple[np.ndarray, Dict]:
        """
        Create all weight matrices from config.
        
        Returns:
            Tuple of (measurement_weight_matrix, composite_weight_matrices_dict)
        """
        cfg = self.config
        
        # Standard measurement loss weights (6D - kept for backward compatibility)
        measurement_weights = create_weight_matrix({
            'v_x': cfg['weight_vx'],
            'v_y': cfg['weight_vy'],
            'psi': cfg['weight_psi'],
            'psi_dot': cfg['weight_psi_dot'],
            'X': cfg['weight_X'],
            'Y': cfg['weight_Y']
        })
        
        # Composite UIO loss weights
        composite_weights = {
            'T_ref': create_weight_matrix({
                'v_x': cfg['weight_ref_vx'],
                'v_y': cfg['weight_ref_vy'],
                'psi': cfg['weight_ref_psi'],
                'psi_dot': cfg['weight_ref_r'],
                'X': cfg['weight_ref_X'],
                'Y': cfg['weight_ref_Y']
            }),
            'T_y': np.eye(self.INTERNAL_STATE_DIM),
            'T_uio': create_weight_matrix({
                'v_x': cfg['weight_uio_vx'],
                'v_y': cfg['weight_uio_vy'],
                'psi': cfg['weight_uio_psi'],
                'psi_dot': cfg['weight_uio_r'],
                'X': cfg['weight_uio_X'],
                'Y': cfg['weight_uio_Y']
            })
        }
        
        return measurement_weights, composite_weights
    
    def _default_vehicle_params(self) -> Dict:
        """Default vehicle parameters - uses centralized defaults"""
        return get_default_vehicle_params()
    
    def _initialize_observer_gains(self):
        """
        - 'qlpv_scheduled': Polytopic qLPV gain scheduling with LMI
        
        """
        method = self.config.get('gain_design_method', 'default')
        self._gain_method = method
        self._gain_scheduler = None
        
        # Get LMI parameters from config
        lmi_decay_rate = self.config.get('lmi_decay_rate', 0.5)
        use_gain_scheduling = self.config.get('use_gain_scheduling', False)
        
        # For advanced methods, we need A, C, E matrices at nominal operating point
        if method in ['lmi', 'hinf', 'l2', 'qlpv_scheduled']:
            # Nominal state: straight driving at moderate speed
            nominal_vx = self.config.get('nominal_vx', 1.5)
            nominal_state = np.array([nominal_vx, 0.0, 0.0, 0.0, 0.0, 0.0])
            nominal_delta = 0.0
            rho = self.dynamics.compute_scheduling_params(nominal_state, nominal_delta)
            
            # Use discrete-time system for design (more accurate for digital implementation)
            sample_time = self.config.get('sample_time', 0.01)
            
            # Compute discrete matrices: x[k+1] = A_d x[k] + ...
            # Note: C matrix is static and doesn't change with discretization
            A_c = self.dynamics.compute_A_matrix(rho)
            B_c = self.dynamics.compute_B_matrix(rho)
            C = self.dynamics.compute_C_matrix(rho , mode='5D_GPS_IMU')
            E_c = self.dynamics.compute_E_matrix(rho)
            
            # Discretize
            A_d, _, E_d = discretize_system_zoh(A_c,B_c,E_c, sample_time)

            # # Get design parameters
            # contraction_rate = self.config.get('contraction_rate', 0.95)
            
            # Try qLPV scheduled gains first if requested
            if method == 'qlpv_scheduled' or use_gain_scheduling:
                if self._try_qlpv_scheduled_gains(lmi_decay_rate):
                    return
            

        # Default: simple diagonal gains
        self._set_default_gains()
    
    def _try_qlpv_scheduled_gains(self, decay_rate: float) -> bool:
        """
        Try to compute qLPV scheduled gains using discrete-time LMI.
        
        Uses discrete-time Schur-form H∞ design with common Lyapunov for
        robust stability across the velocity/steering polytope.
        """
        if not CVXPY_AVAILABLE:
            return False
        
        try:
            vx_range = tuple(self.config.get('vx_range', [0.1, 2.0]))
            delta_max = self.config.get('delta_max', 0.4)
            n_vx_vertices = self.config.get('n_vx_vertices', 3)
            n_delta_vertices = self.config.get('n_delta_vertices', 3)
            use_common_lyapunov = self.config.get('use_common_lyapunov', True)
            sample_time = self.config.get('sample_time', 0.01)
            contraction_rate = self.config.get('contraction_rate', 0.95)
            hinf_gamma = self.config.get('hinf_gamma', 2.0)
            lmi_method = self.config.get('gain_design_method', 'hinf')
            disturbance_mode = self.config.get('disturbance_mode', 'tire')
            
            self._gain_scheduler = NeuralQLPVGainScheduler(
                vehicle_params=self.vehicle_params,
                vx_range=vx_range,
                delta_max=delta_max,
                n_vx_vertices=n_vx_vertices,
                n_delta_vertices=n_delta_vertices,
                decay_rate=decay_rate,
                lmi_method=lmi_method,
                hinf_gamma=hinf_gamma,
                use_common_lyapunov=use_common_lyapunov,
                discrete=True,  # Use discrete-time design
                sample_time=sample_time,
                contraction_rate=contraction_rate,
                verbose=False,
                disturbance_mode=disturbance_mode,
                dynamics_model=self.dynamics  # Pass shared dynamics model
            )
            
            if self._gain_scheduler.compute_gains_lmi():
                # Get nominal gain for L (used as fallback)
                nominal_vx = (vx_range[0] + vx_range[1]) / 2
                self.L = self._gain_scheduler.get_scheduled_gain(nominal_vx, 0.0)
                self._gain_method = 'qlpv_scheduled_discrete'
                # self.logger.logger.info("Success qLPV gain scheduling self.L: ", self.L)
                return True
            else:
                self._gain_scheduler = None
                raise ValueError("Optimization failed: Could not find feasible LMI solution for qLPV gains.")
        except Exception as e:
            self._gain_scheduler = None
            if self.logger:
                self.logger.log_error("qLPV gain scheduling failed", e)
            raise ValueError(f"CRITICAL: qLPV gain scheduling failed: {e}. Default gains are forbidden.")
    

    def _set_default_gains(self):
        """
        Set simple default observer gains.
        
        L is 6×5 mapping measurements [vx, r, ψ, X, Y] to state corrections.
        Designed for the fixed selection matrix C (5×6).
        """
        gain = self.config.get('observer_gain', 0.5)
        # Tuned gains for 6D state corrected by 5D measurement [vx, r, ψ, X, Y]
        # L shape: (state_dim, meas_dim) = (6, 5)
        # Columns correspond to: vx_meas, r_meas, ψ_meas, X_meas, Y_meas
        # Rows correspond to: vx, vy, ψ, r, X, Y states
        L = np.zeros((self.INTERNAL_STATE_DIM, self.MEAS_DIM_FULL))
        
        # vx state correction from vx measurement
        L[self.IDX_VX, 0] = gain * 2.0
        
        # vy state correction (not directly measured, small cross-coupling)
        L[self.IDX_VY, 0] = gain * 0.1  # Small vx coupling
        L[self.IDX_VY, 1] = gain * 0.5  # r coupling (helps vy estimation)
        
        # ψ state correction from ψ measurement
        L[self.IDX_PSI, 2] = gain * 1.0
        
        # r state correction from r measurement
        L[self.IDX_R, 1] = gain * 2.0
        
        # X state correction from X measurement
        L[self.IDX_X, 3] = gain * 0.5
        
        # Y state correction from Y measurement
        L[self.IDX_Y, 4] = gain * 0.5
        
        self.L = L
        self._gain_method = 'default'
    
    def get_scheduled_gain(self, vx: float, delta: float) -> np.ndarray:
        """
        Get interpolated observer gain for current operating point.
        
        If using qLPV scheduling, returns the interpolated gain.
        Otherwise returns the fixed L matrix.
        
        Args:
            vx: Current longitudinal velocity
            delta: Current steering angle
            
        Returns:
            L: Observer gain matrix (6 × 6)
        """
        if self._gain_scheduler is not None and self._gain_method.startswith('qlpv_scheduled'):
            return self._gain_scheduler.get_scheduled_gain(vx, delta)
        else:
            return self.L.copy()
    

    def _initialize_first_layer_observer(self):
        """Initialize first-layer observer (qLPV or Differentiator-UIO)"""
        try:
            observer_type = self.config.get('first_layer_type', 'qlpv')
            disturbance_mode = self.config.get('disturbance_mode', 'tire')
            
            # Validation: output_dim must match disturbance_mode
            expected_udim = 3 if disturbance_mode == 'general' else 2
            if self.output_dim != expected_udim:
                 if self.logger:
                     self.logger.logger.warning(
                         f"Config Mismatch: output_dim={self.output_dim} but disturbance_mode='{disturbance_mode}' "
                         f"(expected {expected_udim})."
                     )
            
            self.first_layer_observer = create_first_layer_observer(
                observer_type=observer_type,
                sample_time=self.config['sample_time'],
                vehicle_params=self.vehicle_params,
                use_8d_system=self.config.get('use_8d_system', False),
                dynamics_model=self.dynamics,  # Pass shared dynamics model
                disturbance_mode=disturbance_mode
            )
            
            # Alias for compatibility with external tools (including fake_vehicle monkey patcher)
            self.observer = self.first_layer_observer
            
            if self.logger:
                self.logger.logger.info(f"Initialized first-layer observer ({observer_type}, mode={disturbance_mode})")
        
        except Exception as e:
            if self.logger:
                self.logger.log_error("Failed to initialize first-layer observer", e)
            self.use_first_layer = False
            self.output_first_layer_only = False

    
    def _initialize_gradient_solver(self):
        """
        Initialize gradient solver for neural network training.
        
        Uses matrices at nominal operating point (same as gain design)
        so that sensitivity propagation is consistent.
        """
        # Compute matrices at nominal operating point
        nominal_vx = self.config.get('nominal_vx', 1.5)
        nominal_state = np.array([nominal_vx, 0.0, 0.0, 0.0, 0.0, 0.0])
        rho = self.dynamics.compute_scheduling_params(nominal_state, 0.0)
        
        A = self.dynamics.compute_A_matrix(rho)
        E = self.dynamics.compute_E_matrix(rho)
        C = self.dynamics.compute_C_matrix(rho, mode='5D_GPS_IMU')  # Fixed selection matrix (5×6)
        
        observer_matrices = {
            'A': A,
            'C': C,
            'D': E,  # D in gradient solver corresponds to E (residual injection)
        }
        
        self.gradient_solver = GradientSolver(
            self.config['sample_time'],
            observer_matrices
        )
    
    
    def _compute_C_matrix_avail(self, gps_valid: bool) -> np.ndarray:
        """
        Compute C matrix for available sensors.
        
        When GPS is valid: use full C (5×6) for [vx, r, ψ, X, Y]
        When GPS invalid: use IMU-only C (2×6) for [vx, r]
        
        Args:
            gps_valid: Whether GPS measurements are valid
            
        Returns:
            C matrix with appropriate rows for available sensors
        """
        # Create a dummy rho since C might depend on it (though for these modes it actually doesn't much, 
        # except a_y if we were using it, but here we are using selection matrices)
        # We need rho to satisfy the interface.
        vx = max(abs(self.state_nn_6d[self.IDX_VX]), self.min_vx)
        delta = getattr(self, '_last_steering', 0.0)
        rho = self.dynamics.compute_scheduling_params(self.state_nn_6d, delta)
        
        if gps_valid:
            return self.dynamics.compute_C_matrix(rho, mode='5D_GPS_IMU')
        else:
            return self.dynamics.compute_C_matrix(rho, mode='4D_IMU_ONLY')
    
    def _get_L_avail(self, gps_valid: bool) -> np.ndarray:
        """
        Get observer gain matrix for available sensors.
        
        When GPS valid: use full L (6×5)
        When GPS invalid: use IMU-only L (6×2) - only first 2 columns
        
        Args:
            gps_valid: Whether GPS measurements are valid
            
        Returns:
            L matrix with appropriate columns for available sensors
        """
        vx_current = max(abs(self.state_nn_6d[self.IDX_VX]), self.min_vx)
        delta = getattr(self, '_last_steering', 0.0)
        L_full = self.get_scheduled_gain(vx_current, delta)
        
        if gps_valid:
            return L_full  # 6×5 (full measurement correction)
        else:
            # IMU-only: use only first 2 columns corresponding to vx, r
            return L_full[:, :self.MEAS_DIM_IMU]  # 6×2
    
    
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
               acceleration: Optional[np.ndarray] = None) -> bool:
        """
        Update state estimate with sensor data and neural learning.
        
        Seven-phase update flow:
            1. Sensor Processing (IMU always, GPS when valid)
            2. Neural Network Prediction
            3. First-Layer Observer Update (if enabled)
            4. Compute System Matrices
            5. State Update (Predict + Correct)
            6. Sensitivity Propagation (matches observer dynamics)
            7. Neural Network Training
        
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
            # Store inputs for training phase
            self._last_steering = steering
            self._last_throttle = throttle
            control_u = np.array([steering, throttle])

            
            # ========== PHASE 1: SENSOR PROCESSING ==========
            # IMU data (always available at high rate)
            vx_meas = motor_tach
            r_meas = gyro_z
            ax_meas, ay_meas = self._compute_acceleration(acceleration, vx_meas, r_meas)
            
            # GPS validity check (position/orientation only when valid)
            gps_valid = gps_data is not None and gps_data.get('valid', False)
            
            if gps_valid:
                psi_meas = gps_data.get('theta', self.state_nn_6d[self.IDX_PSI])
                X_meas = gps_data.get('x', self.state_nn_6d[self.IDX_X])
                Y_meas = gps_data.get('y', self.state_nn_6d[self.IDX_Y])
            else:
                # Use predicted values when GPS unavailable
                psi_meas = self.state_nn_6d[self.IDX_PSI]
                X_meas = self.state_nn_6d[self.IDX_X]
                Y_meas = self.state_nn_6d[self.IDX_Y]
            
            # ========== PHASE 2: NEURAL NETWORK PREDICTION ==========
            if not self.output_first_layer_only:
                nn_input = self._prepare_nn_input(steering, throttle, acceleration)
                self.f_nn = self._get_nn_prediction(nn_input).squeeze()
            else:
                self.f_nn = np.zeros((self.output_dim, 1))
            
            # ========== PHASE 3: FIRST-LAYER OBSERVER (if enabled) ==========
            w_hat = self.f_nn.squeeze()  # Default: use NN prediction
            
            if self.use_first_layer and self.first_layer_observer is not None:
                # Build measurement for first-layer observer
                if gps_valid:
                    # Full measurement: [v_x, r, ψ, X, Y, a_y , a_x]
                    measurement_1L = np.array([vx_meas, r_meas, psi_meas, X_meas, Y_meas , ay_meas , ax_meas ])
                else:
                    # IMU-only measurement: [v_x, r, a_y, a_x]
                    # The 1st layer observer handles partial measurements by using its own predictions
                    measurement_1L = np.array([vx_meas, r_meas, ay_meas , ax_meas ])
                    
                
                # Update first-layer observer
                state_uio, w_uio = self.first_layer_observer.update(
                    measurement_1L, control_u, 
                    acceleration=acceleration,
                    gps_available=gps_valid,
                    dt=dt
                )
                
                # Store first-layer state for composite loss calculation
                if len(state_uio) > self.INTERNAL_STATE_DIM:
                     self.state_uio = state_uio[:self.INTERNAL_STATE_DIM].copy()
                else:
                     self.state_uio = state_uio
                
                # Use first-layer w estimate for observer dynamics
                w_hat = w_uio
            
            if self.output_first_layer_only and self.use_first_layer and self.first_layer_observer is not None:
                # Bypass mode: directly use first-layer state
                self.state_nn_6d = self.state_uio.copy()
                self.state = self._extract_4d_state()
            
            # ========== LOW SPEED OVERRIDE CHECK ==========
            # print("vx_meas", vx_meas   )

            if self.use_first_layer and self.first_layer_observer is not None and abs(vx_meas) < self.override_threshold :
                # Low speed / Start / Stop condition:
                # Override Neural Observer with 1st Layer Observer (Analytic/Kalman)
                # predictable behavior at low speeds.
                
                # 1. Override state
                self.state_nn_6d = self.state_uio.copy()
                
                # 2. Override disturbance estimate (for consistency in plotting/logging)
                self.f_nn = w_uio.reshape(self.output_dim, 1) if w_uio.ndim == 1 else w_uio
                
                # 3. Update public 4D state
                self.state = self._extract_4d_state()
                
                # print("state_nn_6d", self.state_nn_6d)
                # print("override", self.state)
                # 4. Skip NN training for this step (don't train on override data)
                # We still want to log data, so we don't return early, but we
                # set a flag or just skip the update logic below
                
                # To skip the complex update logic below, we can use an else block
                # for the standard update
                # Define L_avail for logging consistency (set to zero as we are overriding)
                if gps_valid:
                    L_avail = np.zeros((self.INTERNAL_STATE_DIM, self.MEAS_DIM_FULL))
                else:
                    L_avail = np.zeros((self.INTERNAL_STATE_DIM, self.MEAS_DIM_IMU))

            else:

                # ========== PHASE 4: COMPUTE DISCRETE SYSTEM MATRICES ==========
                rho = self.dynamics.compute_scheduling_params(self.state_nn_6d, steering)
                
                # Discretize system at current operating point (ZOH)
                # This matches the discrete-time LMI gain design
                
                # Keep continuous A, E for sensitivity update (gradient solver uses continuous)
                A_c = self.dynamics.compute_A_matrix(rho)
                B_c = self.dynamics.compute_B_matrix(rho)
                E_c = self.dynamics.compute_E_matrix(rho)

                A_d, B_d, E_d = discretize_system_zoh(A_c,B_c,E_c, dt)
                
                # ========== PHASE 5: STATE UPDATE (Discrete Predict + Correct) ==========
                # self.f_nn = self.f_nn.squeeze()  # Ensure f_nn is correct shape for dynamics
                self.tire_info_layer_2 =  self.dynamics._calculate_tire_info(self.state_nn_6d[IDX_VX] , self.state_nn_6d[IDX_VY] , self.state_nn_6d[IDX_R] , steering , self.f_nn[0] , self.f_nn[1] )
                # Discrete-time observer: x̂[k+1] = A_d·x̂[k] + B_d·u[k] + E_d·w[k] + L·(y[k] - C·x̂[k])
                # This structure matches the discrete-time LMI gain design.
                #
                # ALWAYS correct with available sensors:
                #   - IMU (vx, r): always available at high rate
                #   - GPS (ψ, X, Y): only when valid
                
                # Get sensor-dependent C and L matrices
                C_avail = self._compute_C_matrix_avail(gps_valid)
                L_avail = self._get_L_avail(gps_valid)
                
                # Build measurement vector based on sensor availability
                if gps_valid:
                    # Full measurement: [vx, r, ψ, X, Y]
                    y_avail = np.array([vx_meas, r_meas, psi_meas, X_meas, Y_meas])
                else:
                    # IMU-only: [vx, r]
                    y_avail = np.array([vx_meas, r_meas])
                
                # Discrete-time prediction: x̂_pred = A_d·x̂ + B_d·u + E_d·w
                state_pred = A_d @ self.state_nn_6d + B_d @ control_u + E_d @ self.f_nn.squeeze()
                
                # Innovation: y - C·x̂[k] (computed at current state, not predicted)
                # This is the "current measurement" form for discrete observer
                innovation = y_avail - C_avail @ self.state_nn_6d
                
                # Wrap heading innovation to [-pi, pi]
                if gps_valid:
                    # Yaw measurement is at index 2 in 5D GPS measurement vector
                    innovation[2] = (innovation[2] + np.pi) % (2 * np.pi) - np.pi
                
                # Correction: x̂[k+1] = x̂_pred + L·innovation
                self.state_nn_6d = state_pred + L_avail @ innovation
                
                # print("Normal", self.state_nn_6d)

                # Wrap heading state to [-pi, pi]
                # Ensures estimated yaw stays in the same range as GPS sensors
                self.state_nn_6d[self.IDX_PSI] = (self.state_nn_6d[self.IDX_PSI] + np.pi) % (2 * np.pi) - np.pi
                
                # Sync with base class state (4D)
                self.state = self._extract_4d_state()
                
                # ========== PHASE 6: SENSITIVITY UPDATE (matches Phase 5) ==========
                # Use discrete Luenberger sensitivity matching ZOH discretization and actual sensor usage
                # Pass L_avail and C_avail to correctly capture the actual closed-loop dynamics (including IMU feedback)
                # gps_valid=True because we ALWAYS apply correction (either full or IMU-only)
                self.dx_df = self.gradient_solver.gradient_solver_luenberger_discrete(
                    self.dx_df, A_d, L_avail, E_d, C_avail, gps_valid=True
                )
                
                # ========== PHASE 7: NEURAL NETWORK TRAINING ==========
                # Only train when we have ground truth (GPS valid)
                if gps_valid:
                    self._train_network(nn_input, w_hat, gps_data, motor_tach,gyro_z)
            
            # ========== PHASE 7.5: DATA RECORDING ==========
            if self.recorder is not None and self.recorder.is_recording():
                t = time.time() - self._recording_start_time
                measurements = {
                    'vx': vx_meas,
                    'r': r_meas,
                    'psi': psi_meas,
                    'X': X_meas,
                    'Y': Y_meas,
                    'ay': ay_meas,
                    'ax': ax_meas,
                }
                # Get last loss from history if available
                last_loss = self.loss_history[-1] if self.loss_history else 0.0
                # Debug: Extract Yaw Gain and Innovation of 2 layer
                L_psi_val = 0.0
                innov_psi_val = 0.0
                if not self.output_first_layer_only:
                    if gps_valid:
                        # L_avail is full 6x5. Psi state is row 2. Psi meas is col 2.
                        L_psi_val = float(L_avail[self.IDX_PSI, 2])
                    # innov_psi_val = float(innovation[2])
                
                # Fetch Ground Truth if available (Sim only)
                state_true_6d = None
                unknown_input_true = None
                disturbances_true = None
                tire_info = None
                if self.ground_truth_provider is not None:
                    try:
                        state_true_6d = self.ground_truth_provider.get_true_state()
                        unknown_input_true = self.ground_truth_provider.get_true_residuals()
                        
                        if hasattr(self.ground_truth_provider, 'get_true_disturbances'):
                            disturbances_true = self.ground_truth_provider.get_true_disturbances()
                            
                        # Get tire force info for debugging residual estimation
                        if hasattr(self.ground_truth_provider, 'get_tire_info'):
                            tire_info = self.ground_truth_provider.get_tire_info()
                    except Exception:
                        pass


                # Get first-layer unknown input (w_uio) if available
                uio_unknown_input = None
                if self.use_first_layer and self.first_layer_observer is not None:
                    try:
                        uio_unknown_input = w_uio
                    except Exception:
                        pass
                            
                # print("tire_info_layer_1", self.first_layer_observer.tire_info_layer_1)
                # print("tire_info_layer_2", self.tire_info_layer_2)
                self.recorder.record_2layer(
                    t=t,
                    state_6d=self.state_nn_6d,
                    measurements=measurements,
                    nn_outputs=self.f_nn,
                    uio_state=self.state_uio if self.use_first_layer else None,
                    uio_unknown_input=uio_unknown_input,  # First-layer tire residual estimates
                    steering=steering,
                    throttle=throttle,
                    loss=last_loss,
                    gps_valid=gps_valid,
                    state_true_6d=state_true_6d,
                    unknown_input_true=unknown_input_true,
                    disturbances_true=disturbances_true,
                    tire_info_true=tire_info , # Tire force data for debugging
                    tire_info_layer_1= self.first_layer_observer.tire_info_layer_1 if self.first_layer_observer is not None else None,
                    tire_info_layer_2= self.tire_info_layer_2
                )
            
            self.last_update_time = time.time()
            self.update_count += 1
            
            return True
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Neural observer update error", e)
            return False
    
    def _compute_acceleration(self, accel: Optional[np.ndarray], vx: float, r: float) -> float:
        """
        Compute lateral acceleration from IMU or approximation.
        
        When IMU is available, use direct measurement.
        When IMU is absent, use centripetal approximation a_y ≈ r·v_x,
        but with protection for small v_x and reasonable clipping.
        
        Args:
            accel: Optional IMU acceleration [a_x, a_y]
            vx: Longitudinal velocity
            r: Yaw rate
            
        Returns:
            acceleration  ax and ay
        """
        if accel is not None:
            # Direct IMU measurement (preferred)
            return float(accel[0]), float(accel[1])
        
        return 0, 0
    
    def _get_nn_prediction(self, nn_input: np.ndarray) -> np.ndarray:
        """
        Get neural network prediction f_nn = [w_r, w_f].
        
        Args:
            nn_input: Prepared NN input vector
            
        Returns:
            NN output as numpy array (output_dim, 1)
        """
        nn_input_tensor = torch.from_numpy(nn_input).float()
        
        if self.learning_mode == 'continuous_learning' and self.model_queue is not None:
            if len(self.model_queue.models) > 0:
                return self.model_queue.predict(nn_input_tensor).detach().numpy()
        
        return self.model(nn_input_tensor).detach().numpy()
    
    def _train_network(self, nn_input: np.ndarray, w_hat: np.ndarray,
                       gps_data: Dict, motor_tach: float,gyro_z: float):
        """
        Train neural network with computed gradients.
        
        Supports two modes based on self.gradient_method:
            - 'autodiff': PyTorch automatic differentiation
            - 'analytical': Manual sensitivity propagation
        """
        # Build full measurement for loss computation
        measurement_full = np.array([
            motor_tach,
            self.state_nn_6d[self.IDX_VY],  # v_y
            gps_data.get('theta', self.state_nn_6d[self.IDX_PSI]),
            gyro_z,
            gps_data.get('x', self.state_nn_6d[self.IDX_X]),
            gps_data.get('y', self.state_nn_6d[self.IDX_Y])
        ]).reshape(-1, 1)
        
        f_uk = w_hat.reshape(-1, 1)
        
        # Get actual control inputs (stored in update phase)
        steering = getattr(self, '_last_steering', 0.0)
        throttle = getattr(self, '_last_throttle', 0.0)
        
        rho = self.dynamics.compute_scheduling_params(self.state_nn_6d, steering)
        A = self.dynamics.compute_A_matrix(rho)
        B = self.dynamics.compute_B_matrix(rho)
        E = self.dynamics.compute_E_matrix(rho)
        vx_current = max(abs(self.state_nn_6d[self.IDX_VX]), self.min_vx)
        L = self.get_scheduled_gain(vx_current, steering)
        u = np.array([steering, throttle])  # Use actual throttle
        dt = self.config.get('sample_time', 0.01)
        
        # ========== AUTODIFF MODE ==========
        if self.gradient_method == 'autodiff':
            loss = self._compute_loss_autodiff(
                nn_input, measurement_full.flatten(), A, B, E, L, u, dt
            )
            
            if loss is None:
                return
            
            # Accumulate loss for batch training
            if self.batch_loss_autodiff is None:
                self.batch_loss_autodiff = loss
            else:
                self.batch_loss_autodiff = self.batch_loss_autodiff + loss
            self.batch_count += 1
            
            if self.batch_count >= self.batch_size:
                avg_loss = self.batch_loss_autodiff / self.batch_size
                
                self.optimizer.zero_grad()
                avg_loss.backward()
                self.optimizer.step()
                
                # Continuous Learning: Update model queue
                if self.learning_mode == 'continuous_learning' and self.model_queue is not None:
                    self.model_queue.push_model(self.model)
                
                self.loss_history.append(avg_loss.item())
                
                self.batch_loss_autodiff = None
                self.batch_count = 0
            return
        
        # ========== ANALYTICAL MODE ==========
        # Choose loss function based on config
        loss_type = self.config.get('loss_type', 'measurement_full')
        
        if loss_type == 'composite_uio' and self.trajectory_ref is not None:
            # Use composite UIO loss with trajectory reference (reduced dimension)
            dL_df, loss = self.gradient_solver.chain_rule_composite_uio(
                self.trajectory_ref,     # Reduced reference (e.g., [X, Y, ψ])
                measurement_full,         # Measurement
                self.state_nn_6d,          # Neural observer state (6D)
                self.state_uio,            # First-layer UIO state (6D)
                self.dx_df,
                f_uk,
                self.f_nn,
                self.weight_matrices,
                self.config['lambda_regularization'],
                ref_indices=self.ref_indices  # Indices mapping ref to 6D state
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
            
            # Continuous Learning: Update model queue
            if self.learning_mode == 'continuous_learning' and self.model_queue is not None:
                self.model_queue.push_model(self.model)
            
            self.loss_history.append(avg_loss.item())
            
            self.batch_loss = 0.0
            self.batch_count = 0
    
    def _observer_step_torch(self, 
                             state: torch.Tensor,
                             f_nn: torch.Tensor,
                             A: torch.Tensor,
                             B: torch.Tensor,
                             E: torch.Tensor,
                             L: torch.Tensor,
                             C: torch.Tensor,
                             u: torch.Tensor,
                             y: torch.Tensor,
                             dt: float) -> torch.Tensor:
        """
        Differentiable observer step for autodiff gradient computation.
        
        Implements: x̂[k+1] = x̂[k] + dt·(A·x̂ + B·u + E·f_nn + L·(y - C·x̂))
        
        All tensors should have requires_grad=True where gradients are needed.
        
        Args:
            state: Current state estimate tensor (6,)
            f_nn: Neural network output tensor (2,) - requires_grad=True
            A, B, E, L, C: System matrices as tensors
            u: Control input tensor
            y: Measurement tensor
            dt: Sample time
            
        Returns:
            Updated state estimate tensor (differentiable)
        """
        # CRITICAL: Ensure all vectors are 1D to avoid broadcasting issues
        state = state.flatten()
        f_nn = f_nn.flatten()
        u = u.flatten()
        y = y.flatten()
        
        # Continuous-time dynamics: x_dot = A·x + B·u + E·f_nn
        x_dot = A @ state + B @ u + E @ f_nn
        
        # Prediction: x_pred = x + dt·x_dot
        x_pred = state + dt * x_dot
        
        # Innovation and correction: x_new = x_pred + L·(y - C·x_pred)
        innovation = y - C @ x_pred
        x_new = x_pred + L @ innovation
        
        return x_new
    
    def _compute_loss_autodiff(self, 
                               nn_input: np.ndarray,
                               measurement: np.ndarray,
                               A: np.ndarray,
                               B: np.ndarray,
                               E: np.ndarray,
                               L: np.ndarray,
                               u: np.ndarray,
                               dt: float) -> Optional[torch.Tensor]:
        """
        Compute loss using autodiff through observer dynamics.
        
        Args:
            nn_input: Input to neural network
            measurement: Current measurement full , fake v_y 
            A, B, E, L: System matrices
            u: Control input
            dt: Sample time
            
        Returns:
            Differentiable loss tensor or None if failed
        """
        try:
            # IMPORTANT: Use float64 throughout to avoid dtype mixing and graph breaks
            # The neural network model should handle double precision
            nn_input_t = torch.from_numpy(nn_input.astype(np.float64))
            state_t = torch.from_numpy(self.state_nn_6d.astype(np.float64))
            A_t = torch.from_numpy(A.astype(np.float64))
            B_t = torch.from_numpy(B.astype(np.float64))
            E_t = torch.from_numpy(E.astype(np.float64))
            L_t = torch.from_numpy(L.astype(np.float64))
            # Use proper selection matrix C (5×6), not identity
            C_np = self._compute_C_matrix()
            C_t = torch.from_numpy(C_np.astype(np.float64))
            u_t = torch.from_numpy(u.astype(np.float64))
            # Measurement should match C dimensions (5D)
            y_t = torch.from_numpy(measurement.flatten()[:self.MEAS_DIM_FULL].astype(np.float64))
            measurement_ful_t = torch.from_numpy(measurement.flatten().astype(np.float64))
            # Forward pass through neural network (convert to double for consistency)
            f_nn_t = self.model(nn_input_t.float()).double()  # Model expects float, then convert
            
            # Differentiable observer step
            x_new = self._observer_step_torch(
                state_t, f_nn_t, A_t, B_t, E_t, L_t, C_t, u_t, y_t, dt
            )
            
            # Compute loss based on type
            loss_type = self.config.get('loss_type', 'measurement_full')
            W_t = torch.from_numpy(self.weight_matrix.astype(np.float64))
            
            if loss_type == 'composite_uio' and self.trajectory_ref is not None:
                # Composite UIO loss
                x_uio_t = torch.from_numpy(self.state_uio.astype(np.float64))
                ref_t = torch.from_numpy(self.trajectory_ref.astype(np.float64))
                f_uk_t = torch.from_numpy(self.f_nn.flatten().astype(np.float64))
                
                # Convert weight matrices
                weight_matrices_t = {
                    'T_ref': torch.from_numpy(
                        self.weight_matrices.get('T_ref', np.eye(len(self.trajectory_ref))).astype(np.float64)
                    ),
                    'T_y': torch.from_numpy(
                        self.weight_matrices.get('T_y', np.eye(self.MEAS_DIM_FULL)).astype(np.float64)
                    ),
                    'T_uio': torch.from_numpy(
                        self.weight_matrices.get('T_uio', np.eye(self.INTERNAL_STATE_DIM)).astype(np.float64)
                    ),
                }
                
                loss = self.gradient_solver.compute_loss_composite_uio_autodiff(
                    x_new, x_uio_t, ref_t, y_t, C_t, weight_matrices_t,
                    f_nn_t, f_uk_t, self.config['lambda_regularization'],
                    self.ref_indices
                )
            else:
                # Standard measurement full loss (fake 6x1 measurement)
                f_uk_t = torch.from_numpy(self.f_nn.flatten().astype(np.float64))
                loss = self.gradient_solver.compute_loss_measurement_autodiff(
                    x_new, measurement_ful_t, W_t, f_nn_t, f_uk_t,
                    self.config['lambda_regularization']
                )
            
            return loss
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Autodiff loss computation failed", e)
            return None
    
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
    
    def get_estimated_tire_forces(self, steering: float = 0.0) -> dict:
        """
        Get estimated tire forces: F_estimated = F_linear + w_estimated
        
        This allows comparison with true tire forces to verify observer accuracy.
        
        Args:
            steering: Current steering angle [rad] (needed for slip angle computation)
            
        Returns:
            Dict with:
            - Fyr_est: Estimated rear lateral tire force [N]
            - Fyf_est: Estimated front lateral tire force [N]
            - Fyr_linear: Linear reference force (Cr * alpha_r) [N]
            - Fyf_linear: Linear reference force (Cf * alpha_f) [N]
            - w_r: Estimated rear tire residual [N]
            - w_f: Estimated front tire residual [N]
            - alpha_r: Estimated rear slip angle [rad]
            - alpha_f: Estimated front slip angle [rad]
        """
        # Get current state estimates
        vx = max(abs(self.state_nn_6d[self.IDX_VX]), self.min_vx)
        vy = self.state_nn_6d[self.IDX_VY]
        r = self.state_nn_6d[self.IDX_R]
        
        # Compute slip angles (same formula as observer dynamics)
        alpha_f = steering - (vy + self.lf * r) / vx
        alpha_r = -(vy - self.lr * r) / vx
        
        # Linear tire forces (reference model)
        Fyf_linear = self.Cf * alpha_f
        Fyr_linear = self.Cr * alpha_r
        
        # Estimated residuals from NN
        w = self.f_nn.squeeze()
        w_r = w[0] if len(w) > 0 else 0.0
        w_f = w[1] if len(w) > 1 else 0.0
        
        # Estimated total forces
        Fyr_est = Fyr_linear + w_r
        Fyf_est = Fyf_linear + w_f
        
        return {
            'Fyr_est': Fyr_est,
            'Fyf_est': Fyf_est,
            'Fyr_linear': Fyr_linear,
            'Fyf_linear': Fyf_linear,
            'w_r': w_r,
            'w_f': w_f,
            'alpha_r': alpha_r,
            'alpha_f': alpha_f,
        }
    
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
        # Build trajectory reference with corresponding indices
        # State indices: [v_x=0, v_y=1, ψ=2, r=3, X=4, Y=5]
        
        ref_values = []
        ref_indices = []
        
        # Position reference (X, Y) - always available from controller
        if len(ref_pose) >= 2:
            ref_values.append(ref_pose[0])  # X
            ref_indices.append(self.IDX_X)
            ref_values.append(ref_pose[1])  # Y
            ref_indices.append(self.IDX_Y)
        
        # Heading reference (ψ) - available if ref_pose has 3 elements
        if len(ref_pose) >= 3:
            ref_values.append(ref_pose[2])  # ψ
            ref_indices.append(self.IDX_PSI)
        
        # Velocity reference (v_x) - optional
        if ref_velocity is not None:
            ref_values.append(ref_velocity)
            ref_indices.append(self.IDX_VX)
        
        # Heading rate reference (r) - optional
        if ref_heading_rate != 0.0:
            ref_values.append(ref_heading_rate)
            ref_indices.append(self.IDX_R)
        
        # Store as numpy arrays
        self.trajectory_ref = np.array(ref_values)
        self.ref_indices = np.array(ref_indices)
    
    def get_trajectory_reference(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Get current trajectory reference
        
        Returns:
            Tuple of (reference values, corresponding indices in 6D state)
            or None if no reference set
        """
        if self.trajectory_ref is not None:
            return self.trajectory_ref.copy(), self.ref_indices.copy()
        return None

    def get_gain_info(self) -> Dict:
        """Get detailed information about the observer gain design"""
        info = {
            'method': getattr(self, '_gain_method', 'unknown'),
            'L_matrix': self.L,
            'gain_scheduler_active': self._gain_scheduler is not None,
        }
        
        if hasattr(self, '_hinf_gamma_achieved'):
            info['hinf_gamma'] = self._hinf_gamma_achieved
        
        if self._gain_scheduler is not None:
            info['scheduler_info'] = self._gain_scheduler.get_vertex_info()
        
        return info
    def get_observer_gain(self) -> np.ndarray:
        """Return the current observer gain matrix L"""
        return self.L.copy()
    
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
    
    def stop_recording(self) -> int:
        """
        Stop data recording and close the file.
        
        Returns:
            Number of records written
        """
        if self.recorder is not None:
            count = self.recorder.stop()
            if self.logger and count > 0:
                self.logger.logger.info(f"Stopped neural observer recording: {count} records")
            return count
        return 0
    
    def is_recording(self) -> bool:
        """Check if currently recording data."""
        return self.recorder is not None and self.recorder.is_recording()
    
    def get_recording_filepath(self) -> str:
        """Get the current recording file path."""
        if self.recorder is not None:
            return self.recorder.get_filepath()
        return None
