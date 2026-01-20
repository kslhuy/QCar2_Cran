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
from neural_obs_recorder import NeuralObsRecorder

# Import first-layer observer from 1LayerObs directory
one_layer_dir = parent_dir.parent / "1LayerObs"
sys.path.insert(0, str(one_layer_dir))
from firstLayerObserverBase import create_first_layer_observer, FirstLayerObserverBase

# Import LMI gain design utilities from differentiator_uio_observer
from differentiator_uio_observer import (
    QLPVGainScheduler,
    compute_lmi_observer_gain,
    compute_pole_placement_gain,
    validate_observer_gain,
)

# Import base class from parent directory
sys.path.insert(0, str(parent_dir.parent))
from local_state_estimators import LocalStateEstimatorBase

# Import centralized qLPV vehicle dynamics
from qlpv_vehicle_dynamics_obs import (
    SchedulingParameters,
    QLPVVehicleDynamicsObs,
    get_default_vehicle_params,
    IDX_VX, IDX_VY, IDX_PSI, IDX_R, IDX_X, IDX_Y, STATE_DIM,
    MEAS_IDX_VX, MEAS_IDX_R, MEAS_IDX_PSI, MEAS_IDX_X, MEAS_IDX_Y, MEAS_IDX_AY, MEAS_DIM,
)

# Note: SchedulingParameters is imported from qlpv_vehicle_dynamics_obs


# =============================================================================
# H∞ and L2 LMI-based Observer Gain Design
# =============================================================================

def compute_hinf_lmi_observer_gain(A: np.ndarray, C: np.ndarray, E: np.ndarray,
                                    gamma: float = 2.0, decay_rate: float = 0.5,
                                    verbose: bool = False) -> np.ndarray:
    """
    Compute observer gain L using H∞ LMI-based design (Bounded Real Lemma).
    
    For the observer error dynamics with disturbance injection:
        ė = (A - LC)e + E·d
        z = e  (performance output = state estimation error)
    
    The H∞ criterion guarantees: ||z||₂ ≤ γ||d||₂
    i.e., the L2-gain from disturbance d to estimation error e is bounded by γ.
    
    This is achieved by solving the Bounded Real Lemma LMI:
    
        [A^T P + P A - C^T Y^T - Y C + αP    P E  ]
        [            E^T P                  -γ²I  ] < 0
        
    with P > 0, where Y = P @ L.
    
    Recovery: L = P⁻¹ @ Y
    
    Args:
        A: State matrix (n × n)
        C: Output matrix (m × n)  
        E: Disturbance injection matrix (n × p) - links unknown inputs to state
        gamma: H∞ performance bound (smaller = better attenuation, but harder to satisfy)
        decay_rate: Minimum exponential decay rate α (controls convergence speed)
        verbose: Print solver output
        
    Returns:
        L: Observer gain matrix (n × m)
        
    Raises:
        ValueError: If the LMI problem is infeasible or CVXPY not available
        
    Note:
        Smaller gamma provides better disturbance rejection but may be infeasible.
        Typical values: gamma ∈ [1.0, 5.0], decay_rate ∈ [0.1, 1.0]
    """
    if not CVXPY_AVAILABLE:
        raise ValueError("CVXPY is not available. Install with: pip install cvxpy")
    
    n = A.shape[0]  # State dimension
    m = C.shape[0]  # Measurement dimension
    p = E.shape[1]  # Disturbance dimension
    
    # Decision variables
    P = cp.Variable((n, n), symmetric=True)  # Lyapunov matrix
    Y = cp.Variable((n, m))  # Y = P @ L
    
    # Numerical tolerances
    eps = 1e-5  # Strict inequality margin
    P_min = 1e-4
    P_max = 1e3  # Reduced upper bound for better conditioning
    
    # Build the H∞ LMI block matrix constraint (standard BRL form)
    # Top-left: A^T P + P A - C^T Y^T - Y C + αP
    top_left = A.T @ P + P @ A - C.T @ Y.T - Y @ C + decay_rate * P
    
    # Build full LMI matrix
    lmi_top = cp.hstack([top_left, P @ E])
    lmi_bot = cp.hstack([E.T @ P, -gamma**2 * np.eye(p)])
    lmi_full = cp.vstack([lmi_top, lmi_bot])
    
    constraints = [
        P >> P_min * np.eye(n),  # P > 0 (positive definite)
        P << P_max * np.eye(n),  # Upper bound on P for conditioning
        lmi_full << -eps * np.eye(n + p),  # H∞ LMI < 0
    ]
    
    # Objective: minimize trace(P) + regularization on Y (for well-conditioned gain)
    gamma_reg = 0.1  # Increased regularization for robustness
    objective = cp.Minimize(cp.trace(P) + gamma_reg * cp.norm(Y, 'fro'))
    
    # Solve the SDP
    problem = cp.Problem(objective, constraints)
    try:
        problem.solve(solver=cp.SCS, verbose=verbose, max_iters=20000, eps=1e-5)
    except Exception as e:
        # Try alternative solver
        try:
            problem.solve(solver=cp.CVXOPT, verbose=verbose)
        except:
            raise ValueError(f"H∞ LMI solver failed: {e}")
    
    if problem.status not in ['optimal', 'optimal_inaccurate']:
        raise ValueError(f"H∞ LMI problem infeasible. Status: {problem.status}. "
                        f"Try increasing gamma (current: {gamma})")
    
    # Recover observer gain: L = P⁻¹ @ Y
    P_val = P.value
    Y_val = Y.value
    
    if P_val is None or Y_val is None:
        raise ValueError("H∞ LMI solver returned None values")
    
    try:
        L = np.linalg.solve(P_val, Y_val)
    except np.linalg.LinAlgError:
        L = np.linalg.pinv(P_val) @ Y_val
    
    # Validate stability by checking eigenvalues
    A_cl = A - L @ C
    eigenvalues = np.linalg.eigvals(A_cl)
    max_real = np.max(np.real(eigenvalues))
    
    if max_real >= 0:
        raise ValueError(f"H∞ LMI produced unstable observer (max real eig = {max_real:.4f})")
    
    # Limit gain magnitude to prevent numerical issues
    L = np.clip(L, -50.0, 50.0)
    
    return L


def compute_l2_lmi_observer_gain(A: np.ndarray, C: np.ndarray, E: np.ndarray,
                                  gamma: float = 2.0, decay_rate: float = 0.5,
                                  verbose: bool = False) -> np.ndarray:
    """
    Compute observer gain L using L2 gain-bounded LMI design.
    
    Similar to H∞ but with a different weighting on the performance output.
    The L2-gain criterion bounds the energy amplification from disturbance to error.
    
    This solves a simplified version of the Bounded Real Lemma:
    
        A^T P + P A - C^T Y^T - Y C + (1/γ²)I + αP < 0
        
    with an additional constraint on disturbance rejection via E.
    
    Args:
        A: State matrix (n × n)
        C: Output matrix (m × n)
        E: Disturbance injection matrix (n × p)
        gamma: L2 gain bound
        decay_rate: Minimum exponential decay rate
        verbose: Print solver output
        
    Returns:
        L: Observer gain matrix (n × m)
    """
    if not CVXPY_AVAILABLE:
        raise ValueError("CVXPY is not available")
    
    n = A.shape[0]
    m = C.shape[0]
    p = E.shape[1]
    
    P = cp.Variable((n, n), symmetric=True)
    Y = cp.Variable((n, m))
    
    eps = 1e-6
    
    # L2 performance LMI with disturbance weighting
    # Combines decay rate constraint with L2 bound on E·d
    performance_term = (1.0 / gamma**2) * E @ E.T
    lmi_constraint = (A.T @ P + P @ A - C.T @ Y.T - Y @ C + 
                      decay_rate * P + performance_term)
    
    constraints = [
        P >> eps * np.eye(n),
        P << 1e4 * np.eye(n),
        lmi_constraint << -eps * np.eye(n),
    ]
    
    gamma_reg = 0.01
    objective = cp.Minimize(cp.trace(P) + gamma_reg * cp.norm(Y, 'fro'))
    
    problem = cp.Problem(objective, constraints)
    try:
        problem.solve(solver=cp.SCS, verbose=verbose, max_iters=15000, eps=1e-6)
    except Exception as e:
        try:
            problem.solve(solver=cp.CVXOPT, verbose=verbose)
        except:
            raise ValueError(f"L2 LMI solver failed: {e}")
    
    if problem.status not in ['optimal', 'optimal_inaccurate']:
        raise ValueError(f"L2 LMI problem infeasible. Status: {problem.status}")
    
    P_val = P.value
    Y_val = Y.value
    
    if P_val is None or Y_val is None:
        raise ValueError("L2 LMI solver returned None values")
    
    try:
        L = np.linalg.solve(P_val, Y_val)
    except np.linalg.LinAlgError:
        L = np.linalg.pinv(P_val) @ Y_val
    
    if not validate_observer_gain(A, C, L):
        raise ValueError("Computed L2 gain does not produce stable observer")
    
    L = np.clip(L, -100.0, 100.0)
    
    return L

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
        
        # Minimum velocity threshold (from parameters_qcar.yaml)
        self.min_vx = self.vehicle_params.get('vx_min', 0.5)
        
        # Centralized vehicle dynamics (single source of truth)
        # MUST be created before _initialize_gradient_solver which calls _compute_E_matrix
        self.dynamics = QLPVVehicleDynamicsObs(
            vehicle_params=self.vehicle_params,
            min_vx=self.min_vx
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
        
        # Data recording for later plotting
        self.recorder = None
        self._recording_start_time = 0.0
        if self.config.get('enable_recording', False):
            output_dir = self.config.get('recording_output_dir', 'neural_obs_recordings')
            self.recorder = NeuralObsRecorder(output_dir=output_dir)
            filepath = self.recorder.start()
            if self.logger:
                self.logger.logger.info(f"Started neural observer recording to: {filepath}")
            self._recording_start_time = time.time()
    
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
        
        # Standard measurement loss weights
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
        Initialize observer gain matrices using configurable design method.
        
        Supported methods (from config['gain_design_method']):
            - 'default': Simple diagonal gains based on observer_gain parameter
            - 'lmi': Standard LMI-based Lyapunov design
            - 'hinf': H∞ LMI design with bounded disturbance attenuation
            - 'l2': L2-gain bounded LMI design
            - 'qlpv_scheduled': Polytopic qLPV gain scheduling with LMI
        
        Falls back gracefully if advanced methods fail (LMI -> pole placement -> default)
        """
        method = self.config.get('gain_design_method', 'default')
        self._gain_method = method
        self._gain_scheduler = None
        
        # Get LMI parameters from config
        hinf_gamma = self.config.get('hinf_gamma', 1.0)
        l2_gamma = self.config.get('l2_gamma', 2.0)
        lmi_decay_rate = self.config.get('lmi_decay_rate', 0.5)
        use_gain_scheduling = self.config.get('use_gain_scheduling', False)
        
        # For advanced methods, we need A, C, E matrices at nominal operating point
        if method in ['lmi', 'hinf', 'l2', 'qlpv_scheduled']:
            # Nominal state: straight driving at moderate speed
            nominal_vx = self.config.get('nominal_vx', 1.5)
            nominal_state = np.array([nominal_vx, 0.0, 0.0, 0.0, 0.0, 0.0])
            nominal_delta = 0.0
            
            rho = self._compute_scheduling_params(nominal_state, nominal_delta)
            A = self._compute_A_matrix(rho)
            C = self._compute_C_matrix()
            E = self._compute_E_matrix(rho)
            
            # Try qLPV scheduled gains first if requested
            if method == 'qlpv_scheduled' or use_gain_scheduling:
                if self._try_qlpv_scheduled_gains(lmi_decay_rate):
                    return
            
            # Try H∞ LMI design
            if method == 'hinf':
                if self._try_hinf_design(A, C, E, hinf_gamma, lmi_decay_rate):
                    return
            
            # Try L2 LMI design
            if method == 'l2':
                if self._try_l2_design(A, C, E, l2_gamma, lmi_decay_rate):
                    return
            
            # Try standard LMI design
            if method == 'lmi':
                if self._try_lmi_design(A, C, lmi_decay_rate):
                    return
            
            # Try pole placement as final fallback before default
            if SCIPY_AVAILABLE:
                try:
                    self.L = compute_pole_placement_gain(A, C)
                    if validate_observer_gain(A, C, self.L):
                        self._gain_method = 'pole_placement'
                        return
                except Exception:
                    pass
        
        # Default: simple diagonal gains
        self._set_default_gains()
    
    def _try_qlpv_scheduled_gains(self, decay_rate: float) -> bool:
        """Try to compute qLPV scheduled gains"""
        if not CVXPY_AVAILABLE:
            return False
        
        try:
            vx_range = tuple(self.config.get('vx_range', [0.5, 3.0]))
            delta_max = self.config.get('delta_max', 0.4)
            n_vx_vertices = self.config.get('n_vx_vertices', 3)
            n_delta_vertices = self.config.get('n_delta_vertices', 3)
            use_common_lyapunov = self.config.get('use_common_lyapunov', True)
            
            self._gain_scheduler = QLPVGainScheduler(
                vehicle_params=self.vehicle_params,
                vx_range=vx_range,
                delta_max=delta_max,
                n_vx_vertices=n_vx_vertices,
                n_delta_vertices=n_delta_vertices,
                decay_rate=decay_rate,
                use_common_lyapunov=use_common_lyapunov,
                verbose=False
            )
            
            if self._gain_scheduler.compute_gains_lmi():
                # Get nominal gain for L (used as fallback)
                nominal_vx = (vx_range[0] + vx_range[1]) / 2
                self.L = self._gain_scheduler.get_scheduled_gain(nominal_vx, 0.0)
                self._gain_method = 'qlpv_scheduled'
                return True
            else:
                self._gain_scheduler = None
                return False
        except Exception as e:
            self._gain_scheduler = None
            return False
    
    def _try_hinf_design(self, A: np.ndarray, C: np.ndarray, E: np.ndarray,
                          gamma: float, decay_rate: float) -> bool:
        """Try H∞ LMI design"""
        if not CVXPY_AVAILABLE:
            return False
        
        try:
            self.L = compute_hinf_lmi_observer_gain(A, C, E, gamma=gamma,
                                                    decay_rate=decay_rate, verbose=False)
            self._gain_method = 'hinf'
            self._hinf_gamma_achieved = gamma
            return True
        except Exception as e:
            # Try with relaxed gamma
            try:
                relaxed_gamma = gamma * 2.0
                self.L = compute_hinf_lmi_observer_gain(A, C, E, gamma=relaxed_gamma,
                                                        decay_rate=decay_rate, verbose=False)
                self._gain_method = 'hinf_relaxed'
                self._hinf_gamma_achieved = relaxed_gamma
                return True
            except Exception:
                return False
    
    def _try_l2_design(self, A: np.ndarray, C: np.ndarray, E: np.ndarray,
                        gamma: float, decay_rate: float) -> bool:
        """Try L2 LMI design"""
        if not CVXPY_AVAILABLE:
            return False
        
        try:
            self.L = compute_l2_lmi_observer_gain(A, C, E, gamma=gamma,
                                                  decay_rate=decay_rate, verbose=False)
            self._gain_method = 'l2'
            return True
        except Exception:
            return False
    
    def _try_lmi_design(self, A: np.ndarray, C: np.ndarray, decay_rate: float) -> bool:
        """Try standard LMI design"""
        if not CVXPY_AVAILABLE:
            return False
        
        try:
            self.L = compute_lmi_observer_gain(A, C, decay_rate=decay_rate, verbose=False)
            self._gain_method = 'lmi'
            return True
        except Exception:
            return False
    
    def _set_default_gains(self):
        """Set simple diagonal observer gains"""
        gain = self.config.get('observer_gain', 0.5)
        # Tuned diagonal gains for 6D state [vx, vy, psi, r, X, Y]
        self.L = np.diag([gain * 2, gain * 2, gain, gain * 2, gain * 0.5, gain * 0.5])
        self._gain_method = 'default'
    
    def get_gain_method(self) -> str:
        """Return the method used to compute observer gains"""
        return getattr(self, '_gain_method', 'unknown')
    
    def get_observer_gain(self) -> np.ndarray:
        """Return the current observer gain matrix L"""
        return self.L.copy()
    
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
        if self._gain_scheduler is not None and self._gain_method == 'qlpv_scheduled':
            return self._gain_scheduler.get_scheduled_gain(vx, delta)
        else:
            return self.L.copy()
    
    def get_gain_info(self) -> Dict:
        """Get detailed information about the observer gain design"""
        info = {
            'method': self.get_gain_method(),
            'L_matrix': self.L,
            'gain_scheduler_active': self._gain_scheduler is not None,
        }
        
        if hasattr(self, '_hinf_gamma_achieved'):
            info['hinf_gamma'] = self._hinf_gamma_achieved
        
        if self._gain_scheduler is not None:
            info['scheduler_info'] = self._gain_scheduler.get_vertex_info()
        
        return info
    
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
        """Compute scheduling parameters - delegates to centralized dynamics"""
        return self.dynamics.compute_scheduling_params(state, delta)
    
    def _compute_A_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """Compute state matrix A(ρ) - delegates to centralized dynamics"""
        return self.dynamics.compute_A_matrix(rho)
    
    def _compute_B_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """Compute input matrix B(ρ) - delegates to centralized dynamics"""
        return self.dynamics.compute_B_matrix(rho)
    
    def _compute_E_matrix(self, rho: Optional[SchedulingParameters]) -> np.ndarray:
        """Compute residual injection matrix E(ρ) - delegates to centralized dynamics"""
        if rho is None:
            # Create default rho with delta=0 for initialization
            dummy_state = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            rho = self.dynamics.compute_scheduling_params(dummy_state, 0.0)
        return self.dynamics.compute_E_matrix(rho)
    
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
            # Store steering for training phase
            self._last_steering = steering
            
            # ========== PHASE 1: SENSOR PROCESSING ==========
            # IMU data (always available at high rate)
            vx_meas = motor_tach
            r_meas = gyro_z
            ay_meas = self._compute_ay(accel, vx_meas, r_meas)
            
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
            nn_input = self._prepare_nn_input(steering, throttle, accel)
            self.f_nn = self._get_nn_prediction(nn_input)
            
            # ========== PHASE 3: FIRST-LAYER OBSERVER (if enabled) ==========
            w_hat = self.f_nn.squeeze()  # Default: use NN prediction
            
            if self.use_first_layer and self.first_layer_observer is not None:
                # Build measurement for first-layer observer: [v_x, r, ψ, X, Y, a_y]
                measurement_1L = np.array([vx_meas, r_meas, psi_meas, X_meas, Y_meas, ay_meas])
                control = np.array([steering, throttle])
                
                # Update first-layer observer
                state_uio, w_uio = self.first_layer_observer.update(
                    measurement_1L, control, 
                    acceleration=accel
                )
                
                # Store first-layer state for composite loss calculation
                self.state_uio = state_uio
                
                # Use first-layer w estimate for observer dynamics
                w_hat = w_uio
            
            # ========== PHASE 4: COMPUTE SYSTEM MATRICES ==========
            rho = self._compute_scheduling_params(self.state_nn_6d, steering)
            A = self._compute_A_matrix(rho)
            B = self._compute_B_matrix(rho)
            E = self._compute_E_matrix(rho)
            C = self._compute_C_matrix()
            
            # Get scheduled observer gain
            vx_current = max(abs(self.state_nn_6d[self.IDX_VX]), self.min_vx)
            L = self.get_scheduled_gain(vx_current, steering)
            
            # ========== PHASE 5: STATE UPDATE (Predict + Correct) ==========
            u = np.array([steering, throttle])
            
            # Prediction: x̂_pred = x̂ + dt·(A·x̂ + B·u + E·w)
            x_dot = A @ self.state_nn_6d + B @ u + E @ w_hat
            state_pred = self.state_nn_6d + dt * x_dot
            
            # Correction (only when GPS valid)
            if gps_valid:
                # Build measurement vector [v_x, v_y, ψ, r, X, Y]
                measurement = np.array([
                    vx_meas,       # v_x from motor tach
                    0.0,           # v_y (not directly measured)
                    psi_meas,      # ψ from GPS
                    r_meas,        # r from gyro
                    X_meas,        # X from GPS
                    Y_meas         # Y from GPS
                ])
                
                # Innovation: y - C·x̂_pred
                innovation = measurement - C @ state_pred
                innovation[self.IDX_VY] = 0.0  # Don't correct v_y (unmeasured)
                
                # Correction: x̂ = x̂_pred + L·innovation
                self.state_nn_6d = state_pred + L @ innovation
            else:
                # Prediction only
                self.state_nn_6d = state_pred
            
            # Sync with base class state (4D)
            self.state = self._extract_4d_state()
            
            # ========== PHASE 6: SENSITIVITY UPDATE (matches Phase 5) ==========
            # Use Luenberger sensitivity that matches observer predict-correct structure
            self.dx_df = self.gradient_solver.gradient_solver_luenberger(
                self.dx_df, A, L, E, C, dt, gps_valid
            )
            
            # ========== PHASE 7: NEURAL NETWORK TRAINING ==========
            # Only train when we have ground truth (GPS valid)
            if gps_valid:
                self._train_network(nn_input, w_hat, gps_data, motor_tach)
            
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
                }
                # Get last loss from history if available
                last_loss = self.loss_history[-1] if self.loss_history else 0.0
                self.recorder.record(
                    t=t,
                    state_6d=self.state_nn_6d,
                    measurements=measurements,
                    nn_outputs=self.f_nn,
                    uio_state=self.state_uio if self.use_first_layer else None,
                    steering=steering,
                    throttle=throttle,
                    loss=last_loss,
                    gps_valid=gps_valid
                )
            
            self.last_update_time = time.time()
            self.update_count += 1
            
            return True
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Neural observer update error", e)
            return False
    
    def _compute_ay(self, accel: Optional[np.ndarray], vx: float, r: float) -> float:
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
            Lateral acceleration a_y
        """
        if accel is not None:
            # Direct IMU measurement (preferred)
            return float(accel[1])
        
        # Centripetal approximation: a_y ≈ r·v_x
        # Only valid for small slip angles, protect against small v_x
        MIN_VX_FOR_AY = 0.3  # Avoid numerical issues
        vx_safe = max(abs(vx), MIN_VX_FOR_AY)
        ay_raw = r * vx_safe
        
        # Clip to reasonable bounds (typical vehicle limits)
        MAX_AY = 5.0  # m/s²
        return float(np.clip(ay_raw, -MAX_AY, MAX_AY))
    
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
                       gps_data: Dict, motor_tach: float):
        """
        Train neural network with computed gradients.
        
        Supports two modes based on self.gradient_method:
            - 'autodiff': PyTorch automatic differentiation
            - 'analytical': Manual sensitivity propagation
        """
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
        
        # Get current matrices for autodiff
        steering = np.arctan2(
            self.state_nn_6d[self.IDX_VY],
            max(abs(self.state_nn_6d[self.IDX_VX]), self.min_vx)
        ) if hasattr(self, '_last_steering') else 0.0
        steering = getattr(self, '_last_steering', 0.0)
        
        rho = self._compute_scheduling_params(self.state_nn_6d, steering)
        A = self._compute_A_matrix(rho)
        B = self._compute_B_matrix(rho)
        E = self._compute_E_matrix(rho)
        vx_current = max(abs(self.state_nn_6d[self.IDX_VX]), self.min_vx)
        L = self.get_scheduled_gain(vx_current, steering)
        u = np.array([steering, 0.0])  # Control input
        dt = self.config.get('sample_time', 0.02)
        
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
        # Continuous-time dynamics
        x_dot = A @ state + B @ u + E @ f_nn
        
        # Prediction
        x_pred = state + dt * x_dot
        
        # Innovation and correction
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
            measurement: Current measurement  
            A, B, E, L: System matrices
            u: Control input
            dt: Sample time
            
        Returns:
            Differentiable loss tensor or None if failed
        """
        try:
            # Convert to tensors
            nn_input_t = torch.from_numpy(nn_input.astype(np.float32))
            state_t = torch.from_numpy(self.state_nn_6d.astype(np.float64))
            A_t = torch.from_numpy(A.astype(np.float64))
            B_t = torch.from_numpy(B.astype(np.float64))
            E_t = torch.from_numpy(E.astype(np.float64))
            L_t = torch.from_numpy(L.astype(np.float64))
            C_t = torch.eye(6, dtype=torch.float64)
            u_t = torch.from_numpy(u.astype(np.float64))
            y_t = torch.from_numpy(measurement.flatten().astype(np.float64))
            
            # Forward pass through neural network
            f_nn_t = self.model(nn_input_t).double()  # (output_dim,)
            
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
                        self.weight_matrices.get('T_y', np.eye(6)).astype(np.float64)
                    ),
                    'T_uio': torch.from_numpy(
                        self.weight_matrices.get('T_uio', np.eye(6)).astype(np.float64)
                    ),
                }
                
                loss = self.gradient_solver.compute_loss_composite_uio_autodiff(
                    x_new, x_uio_t, ref_t, y_t, C_t, weight_matrices_t,
                    f_nn_t, f_uk_t, self.config['lambda_regularization'],
                    self.ref_indices
                )
            else:
                # Standard measurement loss
                f_uk_t = torch.from_numpy(self.f_nn.flatten().astype(np.float64))
                loss = self.gradient_solver.compute_loss_measurement_autodiff(
                    x_new, y_t, W_t, f_nn_t, f_uk_t,
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
