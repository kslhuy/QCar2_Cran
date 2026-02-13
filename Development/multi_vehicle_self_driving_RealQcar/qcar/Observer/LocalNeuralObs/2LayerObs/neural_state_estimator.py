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
    NeuralObserverNet, GRUTireResidualNet, create_network,
    LearningBatch, SelectiveLearningBatch, ModelQueue,
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
    MEAS_IDX_VX, MEAS_IDX_R, MEAS_IDX_PSI, MEAS_IDX_X, MEAS_IDX_Y, MEAS_IDX_AY, MEAS_IDX_AX, MEAS_DIM,
    C_MATRIX_MODES,
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
    # Full measurements: [vx, r, ψ, X, Y, a_y, a_x] (7D) - GPS + IMU + accel
    # IMU-only: [vx, r, a_y, a_x] (4D) - always available
    MEAS_DIM_FULL = 7  # vx, r, ψ, X, Y, a_y, a_x
    MEAS_DIM_IMU = 4   # vx, r, a_y, a_x
    
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
        
        # Network architecture selection: 'mlp' (original) or 'gru' (recommended)
        self.network_type = self.config.get('network_type', 'mlp')
        self.output_scale = self.config.get('output_scale', self.config.get('f_max', 50.0))
        
        # Gradient clipping max norm (0 = disabled)
        self.grad_clip_norm = self.config.get('grad_clip_norm', 1.0)
        
        # Sign flip option: if NN learns opposite sign of expected tire residual
        # Set nn_output_sign_flip: true in YAML to negate NN output
        self._nn_sign_flip = self.config.get('nn_output_sign_flip', False)
        
        # Initialize neural network using factory
        self.model = create_network(
            self.network_type,
            self.input_dim,
            self.hidden_dim,
            self.output_dim,
            self.output_scale
        )
        
        # Load pretrained model if specified
        if self.config['load_pretrained'] and os.path.exists(self.config['model_path']):
            try:
                self.model = load_model(
                    self.config['model_path'],
                    self.input_dim,
                    self.hidden_dim,
                    self.output_dim,
                    network_type=self.network_type,
                    output_scale=self.output_scale
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
        
        # Measurement configuration
        self.c_matrix_mode = self.config.get('observer_gain_design', {}).get('c_matrix_mode', '7D_FULL')
        if self.c_matrix_mode not in C_MATRIX_MODES:
             # Fallback to 7D if unknown mode in config
             print(f"Warning: Unknown c_matrix_mode '{self.c_matrix_mode}', using '7D_FULL'")
             self.c_matrix_mode = '7D_FULL'
        
        self.active_meas_indices = C_MATRIX_MODES[self.c_matrix_mode]
        self.meas_dim = len(self.active_meas_indices)

        
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
            self.learning_batch = SelectiveLearningBatch(
                max_size=self.config['dict_size'],
                novelty_threshold=self.config.get('novelty_threshold', 0.05),
                min_excitation=self.config.get('min_excitation', 0.02),
                excitation_bonus_weight=self.config.get('excitation_bonus_weight', 2.0),
                use_diversity_replacement=self.config.get('use_diversity_replacement', True)
            )
        elif self.learning_mode == 'continuous_learning':
            self.model_queue = ModelQueue(
                self.input_dim,
                self.output_dim,
                queue_size=3,
                hidden_dim=self.hidden_dim,
                network_type=self.network_type,
                output_scale=self.output_scale
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

        # Previous NN output for temporal smoothness (physics_tire loss)
        self.f_nn_prev = np.zeros(self.output_dim)
        # Last measured accelerations for physics constraint
        self._last_ay_meas = 0.0
        self._last_ax_meas = 0.0
        
        # r_dot finite-difference state (for physics_tire V2)
        self._r_meas_prev = 0.0
        self._r_dot_meas = 0.0
        self._rdot_ema_alpha = self.config.get('rdot_ema_alpha', 0.3)
        
        # 2nd-order Butterworth filter state for r_dot (much better than EMA)
        # Cutoff at ~3 Hz for 100 Hz sample rate
        self._rdot_butter_x = [0.0, 0.0]  # Input history
        self._rdot_butter_y = [0.0, 0.0]  # Output history
        # Butterworth coefficients for fc=3Hz, fs=100Hz (2nd order)
        # Computed from: scipy.signal.butter(2, 3, fs=100)
        self._rdot_butter_b = [0.00782525, 0.01565050, 0.00782525]
        self._rdot_butter_a = [1.0, -1.73472577, 0.76602678]
        
        # Previous measurement for prediction_error loss (stored from last step)
        self._prev_y_avail = None
        self._prev_C_avail = None
        self._prev_D_avail = None
        self._prev_F_avail = None
        self._prev_u = None
        
        # Diagnostic logging
        self._diag_w_star_history = []  # Track physics target for debugging

        self.tire_info_layer_2 = {
            'Fyr_est': 0,
            'Fyf_est': 0,
            'Fyr_linear_only': 0,
            'Fyf_linear_only': 0,
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
        if self.logger:
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
            }),
            # Physics-informed tire loss scalar weights
            'w_physics_target': cfg.get('weight_physics_target', 30.0),
            'w_ay': cfg.get('weight_ay_constraint', 0.0),  # Legacy, disabled by default
            'w_smooth': cfg.get('weight_smooth', 2.0),
            'lambda_bound': cfg.get('lambda_bound', 0.1),
            'f_max': cfg.get('f_max', 50.0),
            'lambda_warmstart': cfg.get('lambda_warmstart', 5.0),
            'warmstart_decay': cfg.get('warmstart_decay', 500),
            # Prediction error loss specific weights
            'pred_w_vx': cfg.get('pred_w_vx', 3.0),
            'pred_w_r': cfg.get('pred_w_r', 5.0),
            'pred_w_psi': cfg.get('pred_w_psi', 1.0),
            'pred_w_X': cfg.get('pred_w_X', 0.5),
            'pred_w_Y': cfg.get('pred_w_Y', 0.5),
            'pred_w_ay': cfg.get('pred_w_ay', 5.0),
            'pred_w_ax': cfg.get('pred_w_ax', 3.0),
            'pred_lambda_l2': cfg.get('pred_lambda_l2', 0.5),
            'pred_w_smooth': cfg.get('pred_w_smooth', 1.0),
            'pred_lambda_bound': cfg.get('pred_lambda_bound', 0.5),
            'pred_f_max': cfg.get('pred_f_max', 25.0),
            'pred_lambda_warmstart': cfg.get('pred_lambda_warmstart', 2.0),
            'pred_warmstart_decay': cfg.get('pred_warmstart_decay', 300),
            'pred_w_uio': cfg.get('pred_w_uio', 0.1),
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
            C = self.dynamics.compute_C_matrix(rho, mode='7D_FULL')
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
        print("Using default gains")
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
            c_matrix_mode = self.c_matrix_mode
            default_gain_matrix = self.config.get('observer_gain_design', {}).get('default_gain_matrix', None)
            
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
                dynamics_model=self.dynamics,  # Pass shared dynamics model
                c_matrix_mode=c_matrix_mode,
                default_gain_matrix=default_gain_matrix
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
        Set default observer gains.
        
        Uses manual 'default_gain_matrix' from config if available.
        Otherwise generates a robust diagonal-ish gain matrix adapted to
        the current c_matrix_mode.
        """
        # 1. Try manual config override
        manual_gain = self.config.get('observer_gain_design', {}).get('default_gain_matrix', None)
        if manual_gain is not None:
             manual_gain = np.asarray(manual_gain)
             if manual_gain.shape == (self.INTERNAL_STATE_DIM, self.meas_dim):
                 self.L = manual_gain
                 self._gain_method = 'manual_default'
                 return
                 
        # 2. Programmatic generation based on c_matrix_mode
        gain_scale = self.config.get('observer_gain', 0.5)
        
        # Mapping: (meas_idx, state_idx) -> base_gain
        # Defines conceptual couplings regardless of mode
        _GAIN_MAP = {
            (MEAS_IDX_VX, self.IDX_VX): 2.0,   # vx -> vx
            (MEAS_IDX_VX, self.IDX_AX): 0.5,   # vx -> ax (if state 8D, here we just use what fits)
            
            (MEAS_IDX_R, self.IDX_R): 2.0,     # r -> r
            (MEAS_IDX_R, self.IDX_VY): 0.5,    # r -> vy
            
            (MEAS_IDX_PSI, self.IDX_PSI): 1.0, # psi -> psi
            
            (MEAS_IDX_X, self.IDX_X): 0.5,     # X -> X
            (MEAS_IDX_Y, self.IDX_Y): 0.5,     # Y -> Y
            
            (MEAS_IDX_AY, self.IDX_VY): 1.5,   # ay -> vy (critical)
            (MEAS_IDX_AY, self.IDX_R): 0.3,    # ay -> r
            
            (MEAS_IDX_AX, self.IDX_VX): 0.5,   # ax -> vx
        }
        
        L = np.zeros((self.INTERNAL_STATE_DIM, self.meas_dim))
        
        # Map full 7D index to current column index
        idx_to_col = {meas_idx: col for col, meas_idx in enumerate(self.active_meas_indices)}
        
        for (meas_idx, state_idx), val in _GAIN_MAP.items():
            if meas_idx in idx_to_col and state_idx < self.INTERNAL_STATE_DIM:
                 col = idx_to_col[meas_idx]
                 L[state_idx, col] = val * gain_scale
                 
        self.L = L
        self._gain_method = 'default_generated'
    
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
        
        When GPS is valid: use full C (7×6) for [vx, r, ψ, X, Y, a_y, a_x]
        When GPS invalid: use IMU-only C (4×6) for [vx, r, a_y, a_x]
        
        Args:
            gps_valid: Whether GPS measurements are valid
            
        Returns:
            C matrix with appropriate rows for available sensors
        """
        # rho needed for a_y/a_x rows which depend on scheduling parameters
        delta = getattr(self, '_last_steering', 0.0)
        rho = self.dynamics.compute_scheduling_params(self.state_nn_6d, delta)
        
        if gps_valid:
            # Use configured mode (e.g., 7D_FULL, 6D_WITH_AY, etc.)
            return self.dynamics.compute_C_matrix(rho, mode=self.c_matrix_mode)
        else:
            # IMU-only fallback: intersection of current mode and 4D_IMU_ONLY
            # Ideally should compute dynamic intersection, but for safety/simplicity
            # we check if current mode supports the IMU subset.
            # For now, keep hardcoded fallback to 4D_IMU_ONLY as it's separate from gain scheduling logic
            # used in _get_L_avail fallback.
            # BUT: if we change C, we must ensure L matches C.
            # In _get_L_avail we slice L. Here we return C.
            # Let's return 4D_IMU_ONLY C matrix for now, assuming typical usage.
            return self.dynamics.compute_C_matrix(rho, mode='4D_IMU_ONLY')
    
    def _get_L_avail(self, gps_valid: bool) -> np.ndarray:
        """
        Get observer gain matrix for available sensors.
        
        When GPS valid: use full L (6×M)
        When GPS invalid: use L for available non-GPS sensors (subset of columns)
        
        Args:
            gps_valid: Whether GPS measurements are valid
            
        Returns:
            L matrix with appropriate columns for available sensors
        """
        vx_current = max(abs(self.state_nn_6d[self.IDX_VX]), self.min_vx)
        delta = getattr(self, '_last_steering', 0.0)
        # Get full scheduled gain (6 x meas_dim)
        L_full = self.get_scheduled_gain(vx_current, delta)
        
        if gps_valid:
            return L_full
        else:
            # Fallback to IMU-only
            # We want columns of L_full corresponding to [vx, r, ay, ax]
            # BUT: L_full is ordered by c_matrix_mode.
            # We need to intersect current mode with 4D_IMU_ONLY.
            
            # Indices of available measurements in global 7D convention
            available_indices_7d = C_MATRIX_MODES['4D_IMU_ONLY'] # [0, 1, 5, 6]
            
            # Map 7D index -> current L column index
            # This relies on L_full columns matching self.active_meas_indices
            idx_map = {idx7d: col for col, idx7d in enumerate(self.active_meas_indices)}
            
            # Collect valid columns (if the current mode actually has them)
            valid_cols = []
            for idx7d in available_indices_7d:
                if idx7d in idx_map:
                    valid_cols.append(idx_map[idx7d])
            
            if not valid_cols:
                # Emergency fallback if intersection is empty (shouldn't happen with standard modes)
                return L_full 
                
            return L_full[:, valid_cols]
    
    
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
            
            # Store accelerations for physics-informed tire loss and autodiff
            self._last_ay_meas = ay_meas
            self._last_ax_meas = ax_meas
            
            # Compute r_dot via finite difference with 2nd-order Butterworth filter
            # Much less noisy than simple EMA, critical for physics_tire loss
            dt = self.config.get('sample_time', 0.02)
            if dt > 0:
                r_dot_raw = (r_meas - self._r_meas_prev) / dt
                
                # 2nd-order Butterworth IIR filter (fc~3Hz at 100Hz sample rate)
                b = self._rdot_butter_b
                a = self._rdot_butter_a
                # y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
                y_new = (b[0] * r_dot_raw 
                        + b[1] * self._rdot_butter_x[0] 
                        + b[2] * self._rdot_butter_x[1]
                        - a[1] * self._rdot_butter_y[0]
                        - a[2] * self._rdot_butter_y[1])
                
                # Update filter state
                self._rdot_butter_x[1] = self._rdot_butter_x[0]
                self._rdot_butter_x[0] = r_dot_raw
                self._rdot_butter_y[1] = self._rdot_butter_y[0]
                self._rdot_butter_y[0] = y_new
                
                self._r_dot_meas = y_new
            self._r_meas_prev = r_meas
            
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
                # Store for warmstart in prediction_error loss (autodiff path)
                self._last_w_hat_L1 = w_uio.flatten().copy()
            
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
                    D_avail = np.zeros((self.MEAS_DIM_FULL, 2))
                    F_avail = np.zeros((self.MEAS_DIM_FULL, self.output_dim))
                else:
                    L_avail = np.zeros((self.INTERNAL_STATE_DIM, self.MEAS_DIM_IMU))
                    D_avail = np.zeros((self.MEAS_DIM_IMU, 2))
                    F_avail = np.zeros((self.MEAS_DIM_IMU, self.output_dim))

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
                
                # ========== PHASE 4.5: STORE PRE-UPDATE STATE FOR PREDICTION ERROR LOSS ==========
                # The prediction error loss needs the PRIOR state x̂[k] and sensitivity ∂x̂[k]/∂f
                # BEFORE the correction step. After Phase 5, self.state_nn_6d becomes x̂[k+1]
                # which already incorporates y[k], making the prediction error trivially small.
                self._state_prior = self.state_nn_6d.copy()
                self._dx_df_prior = self.dx_df.copy()
                
                # ========== PHASE 5: STATE UPDATE (Discrete Predict + Correct) ==========
                # Discrete-time observer: x̂[k+1] = A_d·x̂[k] + B_d·u[k] + E_d·w[k] + L·(y[k] - C·x̂[k])
                # This structure matches the discrete-time LMI gain design.
                #
                # ALWAYS correct with available sensors:
                #   - IMU (vx, r): always available at high rate
                #   - GPS (ψ, X, Y): only when valid
                
                # Get sensor-dependent C, D, F and L matrices
                C_avail = self._compute_C_matrix_avail(gps_valid)
                L_avail = self._get_L_avail(gps_valid)
                
                # D and F matrices for feedthrough (a_y/a_x have D·u and F·w terms)
                if gps_valid:
                    D_avail = self.dynamics.compute_D_matrix(rho, active_indices=C_MATRIX_MODES[self.c_matrix_mode])
                    F_avail = self.dynamics.compute_F_matrix(rho, active_indices=C_MATRIX_MODES[self.c_matrix_mode])
                else:
                    D_avail = self.dynamics.compute_D_matrix(rho, active_indices=C_MATRIX_MODES['4D_IMU_ONLY'])
                    F_avail = self.dynamics.compute_F_matrix(rho, active_indices=C_MATRIX_MODES['4D_IMU_ONLY'])
                
                # Build measurement vector based on sensor availability
                if gps_valid:
                    # Full measurement: [vx, r, ψ, X, Y, a_y, a_x]
                    # This needs to match the order of self.active_meas_indices
                    y_full_7d = np.array([vx_meas, r_meas, psi_meas, X_meas, Y_meas, ay_meas, ax_meas])
                    y_avail = y_full_7d[self.active_meas_indices]
                else:
                    # IMU-only: [vx, r, a_y, a_x]
                    y_avail = np.array([vx_meas, r_meas, ay_meas, ax_meas])
                
                # Discrete-time prediction: x̂_pred = A_d·x̂ + B_d·u + E_d·w
                state_pred = A_d @ self.state_nn_6d + B_d @ control_u + E_d @ self.f_nn.squeeze()
                
                # Innovation: y - (C·x̂ + D·u + F·w)
                # D·u and F·w are feedthrough terms for a_y/a_x measurements
                # (zero for vx, r, ψ, X, Y rows)
                y_predicted = C_avail @ self.state_nn_6d + D_avail @ control_u + F_avail @ self.f_nn.squeeze()
                innovation = y_avail - y_predicted
                
                # Wrap heading innovation to [-pi, pi]
                if gps_valid and self.IDX_PSI in self.active_meas_indices:
                    # Find the column index of PSI in C_avail
                    psi_col_idx = np.where(np.array(C_MATRIX_MODES[self.c_matrix_mode]) == self.IDX_PSI)[0]
                    if len(psi_col_idx) > 0:
                        innovation[psi_col_idx[0]] = (innovation[psi_col_idx[0]] + np.pi) % (2 * np.pi) - np.pi
                
                # Correction: x̂[k+1] = x̂_pred + L_avail @ innovation
                # print("L_avail", L_avail)
                self.state_nn_6d = state_pred + L_avail @ innovation
                
                # print("Normal", self.state_nn_6d)

                # Wrap heading state to [-pi, pi]
                # Ensures estimated yaw stays in the same range as GPS sensors
                self.state_nn_6d[self.IDX_PSI] = (self.state_nn_6d[self.IDX_PSI] + np.pi) % (2 * np.pi) - np.pi
                
                # Sync with base class state (4D)
                self.state = self._extract_4d_state()
                self.tire_info_layer_2 =  self.dynamics._calculate_tire_info(self.state_nn_6d[IDX_VX] , self.state_nn_6d[IDX_VY] , self.state_nn_6d[IDX_R] , steering , self.f_nn[0] , self.f_nn[1] )

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
                
                # Store current measurement data for prediction_error loss (used next step)
                self._prev_y_avail = y_avail.copy()
                self._prev_C_avail = C_avail.copy()
                self._prev_D_avail = D_avail.copy()
                self._prev_F_avail = F_avail.copy()
                self._prev_u = control_u.copy()
            
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
                
                # Fetch Ground Truth if available (Sim only)
                state_true_6d = None
                unknown_input_true = None
                disturbances_true = None
                tire_info = None
                if self.ground_truth_provider is not None:
                    try:
                        state_true_6d = self.ground_truth_provider.get_true_state()
                        
                        if hasattr(self.ground_truth_provider, 'get_true_disturbances'):
                            disturbances_true = self.ground_truth_provider.get_true_disturbances()
                            
                        # Get tire force info for debugging residual estimation
                        if hasattr(self.ground_truth_provider, 'get_tire_info'):
                            tire_info = self.ground_truth_provider.get_tire_info()
                            unknown_input_true = self.ground_truth_provider.get_true_residuals()

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
                output = self.model_queue.predict(nn_input_tensor).detach().numpy()
            else:
                output = self.model(nn_input_tensor).detach().numpy()
        else:
            output = self.model(nn_input_tensor).detach().numpy()
        
        # Apply sign flip if configured (fixes sign convention mismatch)
        # Set nn_output_sign_flip: true in config if NN learns opposite sign
        if getattr(self, '_nn_sign_flip', False):
            output = -output
        
        return output
    
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
                
                # Diagnostic: log gradient norm and direction (every 50 batches)
                if hasattr(self, '_autodiff_batch_count'):
                    self._autodiff_batch_count += 1
                else:
                    self._autodiff_batch_count = 0
                if self._autodiff_batch_count % 50 == 0:
                    total_norm = 0.0
                    # Compute gradient w.r.t. output layer to see direction
                    output_grad_sum = 0.0
                    for name, p in self.model.named_parameters():
                        if p.grad is not None:
                            total_norm += p.grad.data.norm(2).item() ** 2
                            if 'fc_out' in name or 'fc3' in name:
                                output_grad_sum += p.grad.data.sum().item()
                    total_norm = total_norm ** 0.5
                    f_nn_now = self.f_nn.flatten() if hasattr(self, 'f_nn') else [0, 0]
                    # Also log Layer1 estimate if available
                    w_L1 = getattr(self, '_last_w_hat_L1', None)
                    w_L1_str = f"[{w_L1[0]:.3f},{w_L1[1]:.3f}]" if w_L1 is not None else "N/A"
                    print(f"[NN Train step={self.update_count}] "
                          f"loss={avg_loss.item():.4f} grad_norm={total_norm:.4f} "
                          f"out_grad_dir={output_grad_sum:.4f} "
                          f"f_nn=[{f_nn_now[0]:.3f},{f_nn_now[1]:.3f}] "
                          f"w_L1={w_L1_str}")
                
                # Gradient clipping to prevent exploding gradients
                if self.grad_clip_norm > 0:
                    torch.nn.utils.clip_grad_norm_(self.model.parameters(), self.grad_clip_norm)
                
                self.optimizer.step()
                
                # Detach GRU hidden state after training step (prevent graph accumulation)
                if hasattr(self.model, 'detach_hidden'):
                    self.model.detach_hidden()
                
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
        
        if loss_type == 'physics_tire':
            # Physics-informed tire residual loss (RECOMMENDED for tire learning)
            # Uses direct IMU ay constraint + smoothness + soft bounds
            # Get Layer 1 residual estimate for warm start
            w_hat_L1 = w_hat if self.use_first_layer and self.first_layer_observer is not None else None
            
            dL_df, loss = self.gradient_solver.chain_rule_physics_informed_tire(
                measurement_full,
                self.state_nn_6d,
                self.state_uio,
                self.dx_df,
                self.f_nn,
                self.f_nn_prev,
                self.vehicle_params,
                steering,
                self._last_ay_meas,
                self.weight_matrices,
                ref_indices=self.ref_indices,
                reference=self.trajectory_ref,
                r_dot_meas=self._r_dot_meas,
                w_hat_L1=w_hat_L1,
                step_count=self.update_count,
            )
            # Update previous f_nn for smoothness term
            self.f_nn_prev = self.f_nn.squeeze().copy()
            
            # Diagnostic logging for physics target debugging
            if self.update_count % 200 == 0:
                try:
                    vx_d = max(abs(self.state_nn_6d[self.IDX_VX]), self.min_vx)
                    vy_d = self.state_nn_6d[self.IDX_VY]
                    r_d = self.state_nn_6d[self.IDX_R]
                    alpha_f_d = steering - (vy_d + self.lf * r_d) / vx_d
                    alpha_r_d = -(vy_d - self.lr * r_d) / vx_d
                    w_star = self.gradient_solver._solve_physics_target(
                        self.vehicle_params, steering, self._last_ay_meas,
                        self._r_dot_meas, alpha_f_d, alpha_r_d
                    )
                    print(f"[Diag step={self.update_count}] "
                          f"w*=[{w_star[0,0]:.2f},{w_star[1,0]:.2f}] "
                          f"f_nn=[{self.f_nn.flatten()[0]:.2f},{self.f_nn.flatten()[1]:.2f}] "
                          f"r_dot={self._r_dot_meas:.3f} ay={self._last_ay_meas:.3f} "
                          f"loss={loss:.4f}")
                except Exception:
                    pass
        
        elif loss_type == 'prediction_error':
            # Self-supervised prediction error loss (innovation-based)
            # Uses PRE-UPDATE state x̂[k] to compute the prior prediction error:
            #   innovation = y[k] - (C·x̂[k] + D·u + F·w)
            # The gradient dL/df flows through:
            #   1. Accumulated sensitivity C·(∂x̂[k]/∂f) from past steps
            #   2. Direct feedthrough F (for ay, ax measurements)
            if hasattr(self, '_state_prior') and self._state_prior is not None:
                # Use PRE-UPDATE state and sensitivity (stored before Phase 5)
                rho_pred = self.dynamics.compute_scheduling_params(self._state_prior, steering)
                A_c = self.dynamics.compute_A_matrix(rho_pred)
                B_c = self.dynamics.compute_B_matrix(rho_pred)
                E_c = self.dynamics.compute_E_matrix(rho_pred)
                from Design_LMI_neural import discretize_system_zoh
                A_d_pred, B_d_pred, E_d_pred = discretize_system_zoh(A_c, B_c, E_c, dt)
                
                # Get current measurement y[k] as target
                gps_valid_now = True  # We only train when GPS is valid
                C_now = self._compute_C_matrix_avail(gps_valid_now)
                D_now = self.dynamics.compute_D_matrix(rho_pred, active_indices=self.active_meas_indices)
                F_now = self.dynamics.compute_F_matrix(rho_pred, active_indices=self.active_meas_indices)
                y_full_7d = np.array([
                    motor_tach, gyro_z,
                    gps_data.get('theta', self._state_prior[self.IDX_PSI]),
                    gps_data.get('x', self._state_prior[self.IDX_X]),
                    gps_data.get('y', self._state_prior[self.IDX_Y]),
                    self._last_ay_meas,
                    getattr(self, '_last_ax_meas', 0.0)
                ])
                y_now = y_full_7d[self.active_meas_indices]
                
                # Scale-aware measurement weights from config
                wm = self.weight_matrices
                meas_weight_map = {
                    MEAS_IDX_VX: wm.get('pred_w_vx', 3.0),
                    MEAS_IDX_R: wm.get('pred_w_r', 5.0),
                    MEAS_IDX_PSI: wm.get('pred_w_psi', 1.0),
                    MEAS_IDX_X: wm.get('pred_w_X', 0.5),
                    MEAS_IDX_Y: wm.get('pred_w_Y', 0.5),
                    MEAS_IDX_AY: wm.get('pred_w_ay', 5.0),
                    MEAS_IDX_AX: wm.get('pred_w_ax', 3.0),
                }
                w_diag = np.array([meas_weight_map.get(idx, 1.0) for idx in self.active_meas_indices])
                W_pred = np.diag(w_diag)
                
                # Get Layer 1 tire estimate for warmstart
                w_hat_L1_pred = w_hat if self.use_first_layer and self.first_layer_observer is not None else None
                
                # Build weight matrices for prediction loss (all from config)
                pred_weight_matrices = {
                    'W_pred': W_pred,
                    'w_smooth': wm.get('pred_w_smooth', wm.get('w_smooth', 0.5)),
                    'lambda_bound': wm.get('pred_lambda_bound', wm.get('lambda_bound', 0.1)),
                    'f_max': wm.get('pred_f_max', wm.get('f_max', 50.0)),
                    'T_uio': wm.get('T_uio', np.eye(self.INTERNAL_STATE_DIM)),
                    'w_uio_scale': wm.get('pred_w_uio', 0.1),
                    'lambda_l2': wm.get('pred_lambda_l2', 0.0),
                    'lambda_warmstart': wm.get('pred_lambda_warmstart', 0.0),
                    'warmstart_decay': wm.get('pred_warmstart_decay', 300),
                    'step_count': self.update_count,
                    'w_hat_L1': w_hat_L1_pred,
                    'w_corr': wm.get('pred_w_corr', wm.get('w_corr', 1.0)),
                }
                
                state_uio_arg = self.state_uio if self.use_first_layer else None
                
                dL_df, loss = self.gradient_solver.chain_rule_prediction_error(
                    y_now,
                    self._state_prior,      # PRE-UPDATE state x̂[k]
                    self._dx_df_prior,      # PRE-UPDATE sensitivity ∂x̂[k]/∂f
                    self.f_nn,
                    self.f_nn_prev,
                    A_d_pred, B_d_pred, E_d_pred,
                    C_now, D_now, F_now,
                    u,
                    pred_weight_matrices,
                    state_hat_uio=state_uio_arg,
                )
                self.f_nn_prev = self.f_nn.squeeze().copy()
            else:
                # First step: no prior state yet, skip
                dL_df = np.zeros((1, self.output_dim))
                loss = 0.0
        
        elif loss_type == 'composite_uio' and self.trajectory_ref is not None:
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
            # Dictionary-based learning with selective data curation
            # Build extra context for loss functions that need it
            extra_ctx = None
            if loss_type == 'physics_tire':
                w_hat_L1 = w_hat if self.use_first_layer and self.first_layer_observer is not None else None
                extra_ctx = {
                    'ay_meas': float(self._last_ay_meas),
                    'r_dot_meas': float(self._r_dot_meas),
                    'steering': float(getattr(self, '_last_steering', 0.0)),
                    'state_uio': self.state_uio.copy(),
                    'f_nn_prev': self.f_nn_prev.copy(),
                    'w_hat_L1': w_hat_L1.copy() if w_hat_L1 is not None else None,
                }
            elif loss_type == 'prediction_error':
                # Store PRE-UPDATE state and CURRENT measurement for batch replay
                # The prediction error uses innovation = y[k] - (C·x̂_prior + D·u + F·w)
                rho_ctx = self.dynamics.compute_scheduling_params(self._state_prior, steering)
                # Rebuild y_now for storage (same as in training block above)
                y_full_7d_ctx = np.array([
                    motor_tach, gyro_z,
                    gps_data.get('theta', self._state_prior[self.IDX_PSI]),
                    gps_data.get('x', self._state_prior[self.IDX_X]),
                    gps_data.get('y', self._state_prior[self.IDX_Y]),
                    self._last_ay_meas,
                    getattr(self, '_last_ax_meas', 0.0)
                ])
                y_now_ctx = y_full_7d_ctx[self.active_meas_indices]
                w_hat_L1_ctx = w_hat.flatten().copy() if self.use_first_layer and self.first_layer_observer is not None else None
                extra_ctx = {
                    'state_prior': self._state_prior.copy(),
                    'dx_df_prior': self._dx_df_prior.copy(),
                    'y_now': y_now_ctx.copy(),
                    'C_now': self._compute_C_matrix_avail(True),
                    'D_now': self.dynamics.compute_D_matrix(rho_ctx, active_indices=self.active_meas_indices),
                    'F_now': self.dynamics.compute_F_matrix(rho_ctx, active_indices=self.active_meas_indices),
                    'u': u.copy(),
                    'state_uio': self.state_uio.copy() if self.use_first_layer else None,
                    'f_nn_prev': self.f_nn_prev.copy(),
                    'w_hat_L1': w_hat_L1_ctx,
                }
            
            accepted = self.learning_batch.add_feature(
                nn_input,
                self.f_nn,
                self.state_nn_6d,
                measurement_full,
                f_uk,
                self.dx_df,
                self.update_count,
                extra_context=extra_ctx
            )
            
            # Log selective learning stats periodically
            if hasattr(self.learning_batch, 'get_stats') and self.update_count % 500 == 0:
                stats = self.learning_batch.get_stats()
                if self.logger:
                    self.logger.logger.info(
                        f"Dict stats: accepted={stats['total_accepted']}/{stats['total_offered']} "
                        f"({stats['acceptance_rate']:.1%}), size={stats['current_size']}, "
                        f"avg_excitation={stats['avg_excitation']:.3f}"
                    )
            
            if len(self.learning_batch) >= self.batch_size:
                self._train_on_batch(nn_input, dL_df)
        else:
            # Normal batch training
            self._accumulate_batch_loss(nn_input, dL_df)
    
    def _train_on_batch(self, nn_input: np.ndarray, dL_df: np.ndarray):
        """Train on accumulated batch using correct loss function for each entry.
        
        IMPORTANT: Uses the configured loss_type to recompute gradients for
        each dictionary entry, not just chain_rule_full_measurement.
        """
        loss_type = self.config.get('loss_type', 'measurement_full')
        total_gradient = np.zeros_like(dL_df)
        
        for entry in self.learning_batch.feature_dict:
            # Unpack entry: 8 elements (7 original + extra_context dict)
            feature = entry[0]
            f_nn_old = entry[1]
            state_hat = entry[2]
            target = entry[3]
            f_uk_old = entry[4]
            dx_df_old = entry[5]
            # entry[6] = time_step
            extra_ctx = entry[7] if len(entry) > 7 else {}
            
            feature_tensor = torch.from_numpy(feature).float()
            f_nn_new = self.model(feature_tensor).detach().numpy()
            
            if loss_type == 'physics_tire' and extra_ctx:
                # Use physics-informed tire loss with stored context
                dL_df_new, loss_new = self.gradient_solver.chain_rule_physics_informed_tire(
                    target,             # measurement_full at that time
                    state_hat,          # state_nn_6d at that time
                    extra_ctx['state_uio'],   # UIO state at that time
                    dx_df_old,          # sensitivity at that time
                    f_nn_new,           # fresh NN output (recomputed)
                    extra_ctx['f_nn_prev'],   # previous f_nn at that time
                    self.vehicle_params,
                    extra_ctx['steering'],
                    extra_ctx['ay_meas'],
                    self.weight_matrices,
                    ref_indices=self.ref_indices,
                    reference=self.trajectory_ref,
                    r_dot_meas=extra_ctx.get('r_dot_meas', 0.0),
                    w_hat_L1=extra_ctx.get('w_hat_L1'),
                    step_count=self.update_count,
                )
            elif loss_type == 'prediction_error' and extra_ctx:
                # Prediction error loss using stored PRE-UPDATE state and measurement
                state_prior = extra_ctx.get('state_prior', state_hat)
                dx_df_prior = extra_ctx.get('dx_df_prior', dx_df_old)
                y_now = extra_ctx.get('y_now')
                C_now = extra_ctx.get('C_now')
                D_now = extra_ctx.get('D_now')
                F_now = extra_ctx.get('F_now')
                u_ctx = extra_ctx.get('u', np.zeros(2))
                state_uio_ctx = extra_ctx.get('state_uio')
                f_nn_prev_ctx = extra_ctx.get('f_nn_prev', np.zeros(self.output_dim))
                
                if y_now is not None and C_now is not None:
                    # Scale-aware measurement weights from config
                    wm = self.weight_matrices
                    meas_weight_map = {
                        MEAS_IDX_VX: wm.get('pred_w_vx', 3.0),
                        MEAS_IDX_R: wm.get('pred_w_r', 5.0),
                        MEAS_IDX_PSI: wm.get('pred_w_psi', 1.0),
                        MEAS_IDX_X: wm.get('pred_w_X', 0.5),
                        MEAS_IDX_Y: wm.get('pred_w_Y', 0.5),
                        MEAS_IDX_AY: wm.get('pred_w_ay', 5.0),
                        MEAS_IDX_AX: wm.get('pred_w_ax', 3.0),
                    }
                    w_diag = np.array([meas_weight_map.get(idx, 1.0) for idx in self.active_meas_indices])
                    W_pred = np.diag(w_diag)
                    
                    # Get w_hat_L1 from stored extra context
                    w_hat_L1_batch = extra_ctx.get('w_hat_L1')
                    
                    pred_weight_matrices = {
                        'W_pred': W_pred,
                        'w_smooth': wm.get('pred_w_smooth', wm.get('w_smooth', 0.5)),
                        'lambda_bound': wm.get('pred_lambda_bound', wm.get('lambda_bound', 0.1)),
                        'f_max': wm.get('pred_f_max', wm.get('f_max', 50.0)),
                        'T_uio': wm.get('T_uio', np.eye(self.INTERNAL_STATE_DIM)),
                        'w_uio_scale': wm.get('pred_w_uio', 0.1),
                        'lambda_l2': wm.get('pred_lambda_l2', 0.0),
                        'lambda_warmstart': wm.get('pred_lambda_warmstart', 0.0),
                        'warmstart_decay': wm.get('pred_warmstart_decay', 300),
                        'step_count': self.update_count,
                        'w_hat_L1': w_hat_L1_batch,
                        'w_corr': wm.get('pred_w_corr', wm.get('w_corr', 1.0)),
                    }
                    # A_d, B_d, E_d are not used inside chain_rule_prediction_error
                    dummy_A = np.zeros((self.INTERNAL_STATE_DIM, self.INTERNAL_STATE_DIM))
                    dL_df_new, loss_new = self.gradient_solver.chain_rule_prediction_error(
                        y_now, state_prior, dx_df_prior, f_nn_new, f_nn_prev_ctx,
                        dummy_A, dummy_A, dummy_A,  # A_d, B_d, E_d unused
                        C_now, D_now, F_now, u_ctx,
                        pred_weight_matrices,
                        state_hat_uio=state_uio_ctx,
                    )
                else:
                    dL_df_new = np.zeros_like(total_gradient)
                    loss_new = 0.0
            elif loss_type == 'composite_uio' and self.trajectory_ref is not None:
                # Composite UIO loss
                dL_df_new, loss_new = self.gradient_solver.chain_rule_composite_uio(
                    self.trajectory_ref,
                    target,
                    state_hat,
                    extra_ctx.get('state_uio', self.state_uio),
                    dx_df_old,
                    f_uk_old,
                    f_nn_new,
                    self.weight_matrices,
                    self.config['lambda_regularization'],
                    ref_indices=self.ref_indices
                )
            else:
                # Standard measurement loss (default fallback)
                dL_df_new, loss_new = self.gradient_solver.chain_rule_full_measurement(
                    target, state_hat, dx_df_old, f_uk_old, f_nn_new,
                    self.weight_matrix,
                    self.config['lambda_regularization']
                )
            
            if dL_df_new is not None:
                total_gradient += dL_df_new
        
        avg_gradient = total_gradient / len(self.learning_batch)
        
        nn_input_tensor = torch.from_numpy(nn_input).float()
        loss_pytorch = self.model.myloss(self.model(nn_input_tensor), avg_gradient)
        
        self.optimizer.zero_grad()
        loss_pytorch.backward()
        
        # Gradient clipping
        if self.grad_clip_norm > 0:
            torch.nn.utils.clip_grad_norm_(self.model.parameters(), self.grad_clip_norm)
        
        self.optimizer.step()
        
        # Detach GRU hidden state
        if hasattr(self.model, 'detach_hidden'):
            self.model.detach_hidden()
        
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
            
            # Gradient clipping
            if self.grad_clip_norm > 0:
                torch.nn.utils.clip_grad_norm_(self.model.parameters(), self.grad_clip_norm)
            
            self.optimizer.step()
            
            # Detach GRU hidden state
            if hasattr(self.model, 'detach_hidden'):
                self.model.detach_hidden()
            
            # Continuous Learning: Update model queue
            if self.learning_mode == 'continuous_learning' and self.model_queue is not None:
                self.model_queue.push_model(self.model)
            
            self.loss_history.append(avg_loss.item())
            
            self.batch_loss = 0.0
            self.batch_count = 0
    
    def _observer_step_torch(self, 
                             state: torch.Tensor,
                             f_nn: torch.Tensor,
                             A_d: torch.Tensor,
                             B_d: torch.Tensor,
                             E_d: torch.Tensor,
                             L: torch.Tensor,
                             C: torch.Tensor,
                             D: torch.Tensor,
                             F: torch.Tensor,
                             u: torch.Tensor,
                             y: torch.Tensor,
                             dt: float) -> torch.Tensor:
        """
        Differentiable observer step for autodiff gradient computation.
        
        Uses DISCRETE (ZOH) matrices to match the actual observer update:
            x̂[k+1] = A_d·x̂[k] + B_d·u[k] + E_d·f_nn[k] + L·(y[k] - C·x̂[k] - D·u - F·f_nn)
        
        All tensors should have requires_grad=True where gradients are needed.
        
        Args:
            state: Current state estimate tensor (6,)
            f_nn: Neural network output tensor (2,) - requires_grad=True
            A_d, B_d, E_d: DISCRETE system matrices (from ZOH discretization)
            L: Observer gain matrix
            C, D, F: Output matrices
            u: Control input tensor
            y: Measurement tensor
            dt: Sample time (unused, kept for interface compatibility)
            
        Returns:
            Updated state estimate tensor (differentiable)
        """
        # CRITICAL: Ensure all vectors are 1D to avoid broadcasting issues
        state = state.flatten()
        f_nn = f_nn.flatten()
        u = u.flatten()
        y = y.flatten()
        
        # Discrete-time prediction: x̂_pred = A_d·x̂ + B_d·u + E_d·f_nn
        x_pred = A_d @ state + B_d @ u + E_d @ f_nn
        
        # Innovation with feedthrough: y - (C·x̂ + D·u + F·f_nn)
        # NOTE: Innovation uses the PRIOR state x̂[k], not x_pred
        y_predicted = C @ state + D @ u + F @ f_nn
        innovation = y - y_predicted
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
            # Use DISCRETE matrices (ZOH) to match the actual observer update
            from Design_LMI_neural import discretize_system_zoh as _zoh
            A_d_np, B_d_np, E_d_np = _zoh(A, B, E, dt)
            A_t = torch.from_numpy(A_d_np.astype(np.float64))
            B_t = torch.from_numpy(B_d_np.astype(np.float64))
            E_t = torch.from_numpy(E_d_np.astype(np.float64))
            L_t = torch.from_numpy(L.astype(np.float64))
            # Use C matrix matching current mode
            C_np = self._compute_C_matrix_avail(True)  # autodiff runs only when GPS valid
            C_t = torch.from_numpy(C_np.astype(np.float64))
            # D and F matrices for feedthrough (a_y/a_x have D·u and F·w terms)
            delta = getattr(self, '_last_steering', 0.0)
            rho = self.dynamics.compute_scheduling_params(self.state_nn_6d, delta)
            # Use active indices to ensure dimensions match C
            D_np = self.dynamics.compute_D_matrix(rho, active_indices=self.active_meas_indices)
            F_np = self.dynamics.compute_F_matrix(rho, active_indices=self.active_meas_indices)
            D_t = torch.from_numpy(D_np.astype(np.float64))
            F_t = torch.from_numpy(F_np.astype(np.float64))
            u_t = torch.from_numpy(u.astype(np.float64))
            
            # Measurement should match C dimensions
            # The passed 'measurement' is in analytical convention [vx, vy, psi, r, X, Y] (6D).
            # We must remap it to the 7D measurement convention used by C_MATRIX_MODES:
            #   [vx(0), r(1), psi(2), X(3), Y(4), ay(5), ax(6)]
            meas_analytical = measurement.flatten()  # [vx, vy, psi, r, X, Y]
            meas_7d = np.array([
                meas_analytical[0],                            # MEAS_IDX_VX=0: vx
                meas_analytical[3],                            # MEAS_IDX_R=1:  r (gyro_z)
                meas_analytical[2],                            # MEAS_IDX_PSI=2: psi
                meas_analytical[4],                            # MEAS_IDX_X=3:  X
                meas_analytical[5],                            # MEAS_IDX_Y=4:  Y
                self._last_ay_meas,                            # MEAS_IDX_AY=5: ay
                getattr(self, '_last_ax_meas', 0.0),           # MEAS_IDX_AX=6: ax
            ])
            y_selected = meas_7d[self.active_meas_indices]
            y_t = torch.from_numpy(y_selected.astype(np.float64))
            
            # Keep 6D state-convention measurement [vx, vy, psi, r, X, Y] for
            # compute_loss_measurement_autodiff (compares against 6D state x̂)
            measurement_ful_t = torch.from_numpy(meas_analytical.astype(np.float64))
            # Forward pass through neural network (convert to double for consistency)
            f_nn_t = self.model(nn_input_t.float()).double()  # Model expects float, then convert
            
            # Differentiable observer step
            x_new = self._observer_step_torch(
                state_t, f_nn_t, A_t, B_t, E_t, L_t, C_t, D_t, F_t, u_t, y_t, dt
            )
            
            # Compute loss based on type
            loss_type = self.config.get('loss_type', 'measurement_full')
            W_t = torch.from_numpy(self.weight_matrix.astype(np.float64))
            
            if loss_type == 'physics_tire':
                # Physics-informed tire residual loss V2 (autodiff)
                x_uio_t = torch.from_numpy(self.state_uio.astype(np.float64))
                f_nn_prev_t = torch.from_numpy(self.f_nn_prev.astype(np.float64))
                
                T_uio_np = self.weight_matrices.get('T_uio', np.eye(self.INTERNAL_STATE_DIM))
                weight_matrices_t = {
                    'T_uio': torch.from_numpy(T_uio_np.astype(np.float64)),
                    'w_physics_target': self.weight_matrices.get('w_physics_target', 30.0),
                    'w_ay': self.weight_matrices.get('w_ay', 0.0),
                    'w_smooth': self.weight_matrices.get('w_smooth', 2.0),
                    'lambda_bound': self.weight_matrices.get('lambda_bound', 0.1),
                    'f_max': self.weight_matrices.get('f_max', 50.0),
                    'lambda_warmstart': self.weight_matrices.get('lambda_warmstart', 5.0),
                    'warmstart_decay': self.weight_matrices.get('warmstart_decay', 500),
                }
                
                ref_t = None
                if self.trajectory_ref is not None:
                    ref_t = torch.from_numpy(self.trajectory_ref.astype(np.float64))
                    T_ref_np = self.weight_matrices.get('T_ref', np.eye(len(self.trajectory_ref)))
                    weight_matrices_t['T_ref'] = torch.from_numpy(T_ref_np.astype(np.float64))
                
                # Prepare warm start data
                w_hat_L1_t = None
                if self.use_first_layer and self.first_layer_observer is not None:
                    w_hat_L1_np = self.f_nn.flatten()  # Current w_hat from Layer 1
                    if hasattr(self, '_last_w_hat_L1'):
                        w_hat_L1_np = self._last_w_hat_L1
                    w_hat_L1_t = torch.from_numpy(w_hat_L1_np.astype(np.float64))
                
                loss = self.gradient_solver.compute_loss_physics_informed_tire_autodiff(
                    x_new, x_uio_t, f_nn_t, f_nn_prev_t,
                    self.vehicle_params,
                    self._last_steering,
                    self._last_ay_meas,
                    weight_matrices_t,
                    reference=ref_t,
                    ref_indices=self.ref_indices,
                    r_dot_meas=self._r_dot_meas,
                    w_hat_L1=w_hat_L1_t,
                    step_count=self.update_count,
                )
                # Update previous f_nn for smoothness
                self.f_nn_prev = f_nn_t.detach().numpy().flatten().copy()
            elif loss_type == 'prediction_error':
                # Open-loop prediction error loss (autodiff)
                #
                # KEY INSIGHT: Use the OPEN-LOOP prediction (no L correction) so the
                # prediction error truly reflects dynamics mismatch from wrong f_nn.
                #
                # Using x_new (post-correction) fails because L drives x_new toward y,
                # making pred_error ≈ 0 regardless of f_nn. The gradient exists but
                # the error signal it multiplies is near-zero.
                #
                # Using detached state_prior fails because gradient only flows through F
                # (ay/ax channels), missing the dominant vx/r channels.
                #
                # SOLUTION: Compute open-loop prediction differentiably:
                #   x_pred = A_d·x̂[k] + B_d·u + E_d·f_nn  (no L correction)
                #   ŷ = C·x_pred + D·u + F·f_nn
                #   error = ŷ - y[k]
                #
                # Gradient: dŷ/df_nn = C·E_d + F
                # This gives gradient from ALL channels through E_d (especially r and vx)
                # PLUS ay/ax through F. No L absorption problem.
                #
                f_nn_prev_t = torch.from_numpy(self.f_nn_prev.astype(np.float64))
                
                # Use PRE-UPDATE state x̂[k] (detached — that's fine, we get gradient
                # through E_d·f_nn in x_pred and F·f_nn in ŷ)
                # Fallback to current state if _state_prior not set (first step)
                prior_state = getattr(self, '_state_prior', self.state_nn_6d)
                state_prior_t = torch.from_numpy(prior_state.astype(np.float64))
                
                # Ensure all tensors are 1D
                f_nn_flat = f_nn_t.flatten()
                f_nn_prev_flat = f_nn_prev_t.flatten()
                state_prior_flat = state_prior_t.flatten()
                u_flat = u_t.flatten()
                y_flat = y_t.flatten()
                
                # Open-loop 1-step prediction (DIFFERENTIABLE through E_d·f_nn)
                # x_pred[k+1|k] = A_d·x̂[k] + B_d·u[k] + E_d·f_nn[k]
                x_pred = A_t @ state_prior_flat + B_t @ u_flat + E_t @ f_nn_flat
                
                # Predicted measurement from open-loop state
                # ŷ[k+1|k] = C·x_pred + D·u + F·f_nn
                y_hat_pred = C_t @ x_pred + D_t @ u_flat + F_t @ f_nn_flat
                pred_error = y_hat_pred - y_flat
                
                # DIAGNOSTIC: Log y, y_hat, error for ay channel (every 200 steps)
                if hasattr(self, '_diag_counter'):
                    self._diag_counter += 1
                else:
                    self._diag_counter = 0
                if self._diag_counter % 200 == 0:
                    # Find ay index in active measurements
                    ay_local_idx = None
                    for i, idx in enumerate(self.active_meas_indices):
                        if idx == MEAS_IDX_AY:
                            ay_local_idx = i
                            break
                    if ay_local_idx is not None:
                        ay_meas = y_flat[ay_local_idx].item()
                        ay_pred = y_hat_pred[ay_local_idx].item()
                        ay_err = pred_error[ay_local_idx].item()
                        # Also compute the f_nn contribution to ay via E and F
                        E_ay = E_t[ay_local_idx, :].detach().numpy() if ay_local_idx < E_t.shape[0] else [0, 0]
                        F_ay = F_t[ay_local_idx, :].detach().numpy() if ay_local_idx < F_t.shape[0] else [0, 0]
                        f_contrib = (F_t[ay_local_idx, :] @ f_nn_flat).item() if ay_local_idx < F_t.shape[0] else 0
                        print(f"[DIAG step={self.update_count}] ay: meas={ay_meas:.4f} pred={ay_pred:.4f} "
                              f"err={ay_err:.4f} f_nn=[{f_nn_flat[0].item():.3f},{f_nn_flat[1].item():.3f}] "
                              f"F·f_nn={f_contrib:.4f}")
                
                # Scale-aware measurement weights from config
                wm = self.weight_matrices
                meas_weight_map = {
                    MEAS_IDX_VX: wm.get('pred_w_vx', 5.0),
                    MEAS_IDX_R: wm.get('pred_w_r', 15.0),
                    MEAS_IDX_PSI: wm.get('pred_w_psi', 1.0),
                    MEAS_IDX_X: wm.get('pred_w_X', 0.5),
                    MEAS_IDX_Y: wm.get('pred_w_Y', 0.5),
                    MEAS_IDX_AY: wm.get('pred_w_ay', 30.0),
                    MEAS_IDX_AX: wm.get('pred_w_ax', 5.0),
                }
                w_diag = np.array([meas_weight_map.get(idx, 1.0) for idx in self.active_meas_indices])
                W_pred = torch.tensor(np.diag(w_diag), dtype=f_nn_t.dtype)
                
                # Term 1: Open-loop prediction error
                # Gradient: 2·W·(C·E_d + F)^T · error — uses ALL measurement channels
                loss_pred = pred_error @ W_pred @ pred_error
                
                # Term 2: Temporal smoothness (light — don't fight learning)
                w_smooth = wm.get('pred_w_smooth', wm.get('w_smooth', 0.1))
                f_diff = f_nn_flat - f_nn_prev_flat
                loss_smooth = w_smooth * torch.sum(f_diff ** 2)
                
                # Term 3: Soft bound (only active when |f_nn| > f_max)
                f_max = wm.get('pred_f_max', wm.get('f_max', 25.0))
                lambda_bound = wm.get('pred_lambda_bound', wm.get('lambda_bound', 0.1))
                loss_bound = torch.tensor(0.0, dtype=f_nn_flat.dtype)
                for i in range(f_nn_flat.shape[0]):
                    excess = torch.clamp(torch.abs(f_nn_flat[i]) - f_max, min=0.0)
                    loss_bound = loss_bound + lambda_bound * excess ** 2
                
                # Term 4: L2 magnitude penalty (very light)
                lambda_l2 = wm.get('pred_lambda_l2', 0.0)
                loss_l2 = lambda_l2 * torch.sum(f_nn_flat ** 2) if lambda_l2 > 0 else torch.tensor(0.0, dtype=f_nn_flat.dtype)
                
                # Term 5: Warm start from Layer 1 estimate (decaying)
                loss_warmstart = torch.tensor(0.0, dtype=f_nn_flat.dtype)
                lambda_warmstart = wm.get('pred_lambda_warmstart', 0.0)
                warmstart_decay = wm.get('pred_warmstart_decay', 100)
                if self.use_first_layer and lambda_warmstart > 0 and warmstart_decay > 0:
                    import math
                    decay_factor = math.exp(-self.update_count / warmstart_decay)
                    w_hat_L1_arr = getattr(self, '_last_w_hat_L1', None)
                    if w_hat_L1_arr is not None:
                        w_hat_L1_t = torch.tensor(w_hat_L1_arr.flatten(), dtype=f_nn_flat.dtype)
                        effective_lambda = lambda_warmstart * decay_factor
                        warmstart_err = f_nn_flat - w_hat_L1_t
                        loss_warmstart = effective_lambda * torch.sum(warmstart_err ** 2)
                
                # Term 6: Optional UIO consistency (very low weight)
                loss_uio = torch.tensor(0.0, dtype=f_nn_flat.dtype)
                if self.use_first_layer:
                    x_uio_t = torch.from_numpy(self.state_uio.astype(np.float64)).flatten()
                    w_uio_scale = wm.get('pred_w_uio', 0.02)
                    state_error = state_prior_flat - x_uio_t
                    T_uio_np = wm.get('T_uio', np.eye(self.INTERNAL_STATE_DIM))
                    T_uio = torch.tensor(T_uio_np, dtype=f_nn_flat.dtype)
                    loss_uio = w_uio_scale * (state_error @ T_uio @ state_error)
                
                # Term 7: Tire residual correlation — penalize sign disagreement
                # Both tires saturate together (same sign) during cornering.
                # L_corr = w_corr * (w_r - w_f)²
                # This couples the two outputs and prevents opposite-sign divergence.
                w_corr = wm.get('pred_w_corr', wm.get('w_corr', 1.0))
                loss_corr = torch.tensor(0.0, dtype=f_nn_flat.dtype)
                if w_corr > 0 and f_nn_flat.shape[0] >= 2:
                    loss_corr = w_corr * (f_nn_flat[0] - f_nn_flat[1]) ** 2
                
                loss = loss_pred + loss_smooth + loss_bound + loss_l2 + loss_warmstart + loss_uio + loss_corr
                self.f_nn_prev = f_nn_t.detach().numpy().flatten().copy()
            elif loss_type == 'composite_uio' and self.trajectory_ref is not None:
                # Composite UIO loss
                x_uio_t = torch.from_numpy(self.state_uio.astype(np.float64))
                ref_t = torch.from_numpy(self.trajectory_ref.astype(np.float64))
                f_uk_t = torch.from_numpy(self.f_nn.flatten().astype(np.float64))
                
                # Convert weight matrices
                # T_y must match measurement dimension (y_t), which depends on c_matrix_mode
                actual_meas_dim = len(self.active_meas_indices)
                T_y_np = self.weight_matrices.get('T_y', np.eye(actual_meas_dim))
                # Resize T_y if it doesn't match actual measurement dimension
                if T_y_np.shape[0] != actual_meas_dim:
                    T_y_np = np.eye(actual_meas_dim)
                
                weight_matrices_t = {
                    'T_ref': torch.from_numpy(
                        self.weight_matrices.get('T_ref', np.eye(len(self.trajectory_ref))).astype(np.float64)
                    ),
                    'T_y': torch.from_numpy(T_y_np.astype(np.float64)),
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
        self.f_nn_prev = np.zeros(self.output_dim)
        self._last_ay_meas = 0.0
        self._last_ax_meas = 0.0
        self.dx_df = np.zeros((self.INTERNAL_STATE_DIM, self.output_dim))
        
        # Reset GRU hidden state if applicable
        if hasattr(self.model, 'reset_hidden'):
            self.model.reset_hidden()
        
        # Reset prediction_error previous data
        self._prev_y_avail = None
        self._prev_C_avail = None
        self._prev_D_avail = None
        self._prev_F_avail = None
        self._prev_u = None
        
        # Reset Butterworth filter state
        self._rdot_butter_x = [0.0, 0.0]
        self._rdot_butter_y = [0.0, 0.0]
        self._r_meas_prev = 0.0
        self._r_dot_meas = 0.0
        
        # Reset batch training
        self.batch_loss = 0.0
        self.batch_count = 0
        
        # Clear learning batch
        if self.learning_batch is not None:
            self.learning_batch.clear()
        # Measurement configuration
        self.c_matrix_mode = self.config.get('observer_gain_design', {}).get('c_matrix_mode', '7D_FULL')
        if self.c_matrix_mode not in C_MATRIX_MODES:
             # Fallback to 7D if unknown mode in config
             print(f"Warning: Unknown c_matrix_mode '{self.c_matrix_mode}', using '7D_FULL'")
             self.c_matrix_mode = '7D_FULL'
        
        self.active_meas_indices = C_MATRIX_MODES[self.c_matrix_mode]
        self.meas_dim = len(self.active_meas_indices)

        # Internal state (6D)
        self.state_nn_6d = np.zeros(6)
        
        # Initialize observer gains
        self._initialize_observer_gains()
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
