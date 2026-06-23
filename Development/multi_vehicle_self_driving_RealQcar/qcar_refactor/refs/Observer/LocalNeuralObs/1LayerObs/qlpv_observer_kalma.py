"""
qLPV Augmented-State Observer with EKF-style Gain Computation

Implements a quasi-Linear Parameter-Varying (qLPV) augmented-state observer
with tire-residual estimation using Extended Kalman Filter (EKF) methodology
for dynamic observer gain computation.

State: x = [v_x, v_y, ψ, r, X, Y]ᵀ (6D)
Augmented state: x_a = [x; w_r; w_f]ᵀ (8D with tire residuals)
Measurements: y = [v_x, r, ψ, X, Y, a_y]ᵀ (6D with lateral acceleration)

Observer Equation:
    ẋ̂_a = A_a(ρ̂)·x̂_a + B_a(ρ̂)·u + L_a(ρ̂)·(y − C_a(ρ̂)·x̂_a − D(ρ̂)·u)
EKF Observer Equations:
    Predict:
        x̂_a⁻ = f(x̂_a, u)
        P⁻ = F·P·Fᵀ + Q

    Update:
        K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
        x̂_a = x̂_a⁻ + K·(y - h(x̂_a⁻, u))
        P = (I - K·H)·P⁻·(I - K·H)ᵀ + K·R·Kᵀ  (Joseph form)

where:
    - x̂_a = [x̂; ŵ_r; ŵ_f] is the augmented state estimate
    - ρ = {1/v_x, sin(δ), cos(δ), v_x, v_y, sin(ψ), cos(ψ)} scheduling parameters
    - w = [w_r, w_f] are tire force residuals (unknown inputs)
    - a_y provides algebraic constraint on w
    - P is the error covariance matrix
    - K is the Kalman gain (computed dynamically)
    - Q is process noise covariance
    - R is measurement noise covariance
    - F, H are Jacobians of f and h
References:
    - qLPV vehicle dynamics with tire-residual estimation
    - Extended Kalman Filter for nonlinear state estimation
    - UIO (Unknown Input Observer) for disturbance estimation
"""

import numpy as np
from typing import Optional, Dict, Tuple
from dataclasses import dataclass

# Import base class
import sys
from pathlib import Path
from scipy.linalg import expm
parent_dir = Path(__file__).parent
sys.path.insert(0, str(parent_dir))

try:
    from firstLayerObserverBase import FirstLayerObserverBase
except ImportError:
    # Fallback: define minimal base class
    from abc import ABC, abstractmethod
    class FirstLayerObserverBase(ABC):
        def __init__(self, state_dim: int = 4, unknown_input_dim: int = 2, sample_time: float = 0.02):
            self.state_dim = state_dim
            self.unknown_input_dim = unknown_input_dim
            self.Ts = sample_time
            self.state_hat = np.zeros(state_dim)
            self.f_uk_hat = np.zeros(unknown_input_dim)

# Import centralized qLPV vehicle dynamics
sys.path.insert(0, str(parent_dir.parent))
from qlpv_vehicle_dynamics_obs import (
    SchedulingParameters,
    get_default_vehicle_params,
    create_qlpv_dynamics,
    IDX_VX, IDX_VY, IDX_PSI, IDX_R, IDX_X, IDX_Y, STATE_DIM,
    MEAS_IDX_VX, MEAS_IDX_R, MEAS_IDX_PSI, MEAS_IDX_X, MEAS_IDX_Y, MEAS_IDX_AY, MEAS_DIM,
    IDX8_VX, IDX8_VY, IDX8_PSI, IDX8_R, IDX8_X, IDX8_Y, IDX8_AX, IDX8_AY, STATE_DIM_8D,
    MEAS8_IDX_VX, MEAS8_IDX_R, MEAS8_IDX_PSI, MEAS_IDX_AX, MEAS8_IDX_X, MEAS8_IDX_Y, MEAS8_IDX_AY, MEAS8_IDX_AX, MEAS_DIM_7D,
)

# Optional differentiator for r_dot pseudo-measurement
from differentiators import create_differentiator_from_config


# Note: SchedulingParameters is imported from qlpv_vehicle_dynamics_obs


class qLPVKalmanObserver(FirstLayerObserverBase):
    """
    qLPV Augmented-State Observer with EKF-style Kalman Gain Computation

    Estimates both vehicle states and unknown tire force residuals using
    an augmented-state qLPV observer structure with dynamically computed
    Kalman gains based on error covariance propagation.

    State vector: x = [v_x, v_y, ψ, r, X, Y]ᵀ
        - v_x: Longitudinal velocity (body frame)
        - v_y: Lateral velocity (body frame)
        - ψ: Yaw angle
        - r: Yaw rate
        - X: Global X position
        - Y: Global Y position

    Tire residuals: w = [w_r, w_f]ᵀ
        - w_r: Rear tire force residual = F_yr - C_r·α_r
        - w_f: Front tire force residual = F_yf - C_f·α_f

    Augmented state: x_a = [x; w]ᵀ (8-dimensional)

    Measurements: y = [v_x, r, ψ, X, Y, a_y]ᵀ
        - a_y: Lateral acceleration (gives algebraic handle on w)

    Key Feature: Kalman gain K is computed at each time step using the
    EKF predict-update cycle, providing optimal state estimation.
    """

    # State indices (using imported constants for consistency)
    # The class will rely on the imported constants where possible, 
    # ensuring consistency with qlpv_vehicle_dynamics_obs.py

    # We keep local references for convenience if needed, 
    # but primarily we should use the imported ones.


    def __init__(self, sample_time: float = 0.02, 
                 vehicle_params: Optional[Dict] = None,
                 Q: Optional[np.ndarray] = None,
                 R: Optional[np.ndarray] = None,
                 P0: Optional[np.ndarray] = None,
                 include_gyro_bias: bool = False,
                 use_8d_system: bool = False,
                 dynamics_model = None,
                 **kwargs):
        """
        Initialize qLPV Augmented-State Observer with EKF-style Gain Computation

        Args:
            sample_time: Sample time Ts [s]
            vehicle_params: Vehicle parameters dict
            Q: Process noise covariance
            R: Measurement noise covariance
            P0: Initial error covariance
            include_gyro_bias: Whether to include gyro bias state
            use_8d_system: Use 8D state system
            dynamics_model: Existing QLPVVehicleDynamicsObs instance (optional)
        """
        self.use_8d_system = use_8d_system
        self.disturbance_mode = kwargs.get('disturbance_mode', 'tire')
        self.udim = 3 if self.disturbance_mode == 'general' else 2

        if use_8d_system:
             self.state_dim = STATE_DIM_8D
             self.augmented_dim = STATE_DIM_8D + self.udim
             self.meas_dim = MEAS_DIM_7D
        else:
             self.state_dim = STATE_DIM
             self.augmented_dim = STATE_DIM + self.udim
             self.meas_dim = MEAS_DIM

        # Initialize base class
        super().__init__(
            state_dim=self.state_dim,
            unknown_input_dim=self.udim,
            sample_time=sample_time
        )

        # Vehicle parameters
        self.params = self._default_params()
        if vehicle_params is not None:
            self.params.update(vehicle_params)

        # Extract commonly used parameters
        self.lf = self.params['lf']
        self.lr = self.params['lr']
        self.m = self.params['m']
        self.Iz = self.params['Iz']
        self.Cf = self.params['Cf']
        self.Cr = self.params['Cr']
        self.mu = self.params.get('mu', 0.01) # Default small friction
        self.g = 9.81  # Gravity

        # Augmented state
        self.state_augmented = np.zeros(self.augmented_dim)

        # Initialize state estimate
        self.state_hat = np.zeros(self.state_dim)

        # Tire residual estimates
        self.w_hat = np.zeros(self.udim)  # [w_r, w_f] or [d_vx, d_vy, d_r]

        # =====================================================
        # Tire Residual Cross-Correlation (Observability Fix)
        # =====================================================
        # High correlation (0.9) encourages w_r and w_f to move together,
        # fixing the observability issue where they could diverge with opposite signs.
        # self.tire_correlation = kwargs.get('tire_correlation', 0.8)
        self.tire_correlation = kwargs.get('tire_correlation', 0.8)


        self.tire_info_layer_1 = {
            'Fyr_est': 0.0,
            'Fyf_est': 0.0,
            'Fyr_linear_only': 0.0,
            'Fyf_linear_only': 0.0,
            'alpha_r': 0.0,
            'alpha_f': 0.0
        }

        # =====================================================
        # EKF Covariance Matrices (Q, R, P)
        # =====================================================

        # Process noise covariance Q
        # Process noise covariance Q
        if Q is not None:
            self.Q_base = Q
        else:
            self.Q_base = self._default_Q() # Base Q per second (continuous approximation)

        # Measurement noise covariance R
        if R is not None:
            self.R = R
        else:
            self.R = self._default_R()

        # Error covariance matrix P
        if P0 is not None:
            self.P = P0.copy()
        else:
            self.P = self._default_P0()

        # Kalman gain K - computed dynamically
        self.K = np.zeros((self.augmented_dim, self.meas_dim))

        # Store last innovation for diagnostics
        self.innovation = np.zeros(self.meas_dim)

        # Gyro bias estimation (optional)
        self.include_gyro_bias = include_gyro_bias
        self.gyro_bias = 0.0

        # UIO residual for a_y constraint
        self.ay_innovation = 0.0
        self.w_constraint = 0.0  # m·ã_y ≈ w_r + cos(δ)·w_f

        # Minimum velocity threshold (from parameters_qcar.yaml)
        # Lower value allows estimation closer to zero when vehicle stops
        self.min_vx = self.params['vx_min']

        # Centralized vehicle dynamics
        if dynamics_model is not None:
            self.dynamics = dynamics_model
        else:
            self.dynamics = create_qlpv_dynamics(
                vehicle_params=self.params,
                min_vx=self.min_vx,
                use_8d_system=use_8d_system,
                disturbance_mode=self.disturbance_mode
            )

        # Numerical Jacobian step size
        self.epsilon = 1e-6

        # Internal state for control processing
        self.current_steering_angle = 0.0

        # =====================================================
        # Optional r_dot differentiator (pseudo-measurement)
        # =====================================================
        self.use_rdot_differentiator = kwargs.get('use_rdot_differentiator', False)
        self.rdot_diff_type = kwargs.get('rdot_diff_type', 'highgain')
        self.rdot_diff_config_path = kwargs.get('rdot_diff_config_path', None)
        self.rdot_diff_overrides = kwargs.get('rdot_diff_overrides', {})
        # Allow direct overrides via kwargs (rdot_diff_omega, rdot_diff_zeta, etc.)
        rdot_override = dict(self.rdot_diff_overrides) if isinstance(self.rdot_diff_overrides, dict) else {}
        for key in ('omega', 'zeta', 'ydot_max', 'tau', 'k1', 'k2', 'epsilon', 'smoothing', 'v_max'):
            kw_key = f'rdot_diff_{key}'
            if kw_key in kwargs:
                rdot_override[key] = kwargs[kw_key]
        self.rdot_diff_overrides = rdot_override
        self.rdot_meas_var = kwargs.get('rdot_meas_var', 0.25)  # (rad/s^2)^2
        self.rdot_diff = None
        self.rdot_meas = 0.0
        if self.use_rdot_differentiator:
            self.rdot_diff = create_differentiator_from_config(
                diff_type=self.rdot_diff_type,
                Ts=self.Ts,
                config_path=self.rdot_diff_config_path,
                **self.rdot_diff_overrides
            )

        # =====================================================
        # Excitation gating + decay for general disturbances (LEGACY)
        # =====================================================
        self.enable_excitation_gating = kwargs.get('enable_excitation_gating', False)
        self.ay_excitation_threshold = kwargs.get('ay_excitation_threshold', 0.1)
        self.rdot_excitation_threshold = kwargs.get('rdot_excitation_threshold', 0.1)
        self.r_excitation_threshold = kwargs.get('r_excitation_threshold', 0.05)

        self.enable_disturbance_decay = kwargs.get('enable_disturbance_decay', False)
        self.decay_only_when_unexcited = kwargs.get('decay_only_when_unexcited', True)
        self.decay_rate_dvx = kwargs.get('decay_rate_dvx', 0.5)
        self.decay_rate_dvy = kwargs.get('decay_rate_dvy', 0.5)
        self.decay_rate_dr = kwargs.get('decay_rate_dr', 0.5)

        # =====================================================
        # NEW: Observability-based gating (v_x, delta, yaw wrap)
        # =====================================================
        self.enable_observability_gating = kwargs.get('enable_observability_gating', False)
        
        # Velocity threshold: below this, lateral dynamics poorly observable
        self.vx_min_observable = kwargs.get('vx_min_observable', 0.3)  # m/s
        
        # Steering threshold: below this + low r, front/rear split unobservable
        self.delta_min_observable = kwargs.get('delta_min_observable', 0.02)  # rad (~1.1 deg)
        self.r_min_observable = kwargs.get('r_min_observable', 0.05)  # rad/s
        
        # Yaw wrap detection
        self.yaw_wrap_window = kwargs.get('yaw_wrap_window', 5)  # samples to gate after wrap
        self.yaw_wrap_threshold = kwargs.get('yaw_wrap_threshold', 2.5)  # rad jump to detect wrap
        self.yaw_wrap_counter = 0
        self.prev_psi_meas = 0.0
        
        # Per-component observability flags: [g_dvx, g_dvy, g_dr] or [g_wr, g_wf]
        self.observability_flags = np.ones(self.udim)
        
        # Decay rates when unobservable (faster decay when gated)
        self.gated_decay_rate_dvx = kwargs.get('gated_decay_rate_dvx', 2.0)
        self.gated_decay_rate_dvy = kwargs.get('gated_decay_rate_dvy', 3.0)
        self.gated_decay_rate_dr = kwargs.get('gated_decay_rate_dr', 5.0)





    def _safe_inverse(self, S: np.ndarray) -> np.ndarray:
        """Compute safe inverse of S"""
        try:
            return np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # Add small regularization
            S_reg = S + np.eye(S.shape[0]) * 1e-6
            return np.linalg.inv(S_reg)

    def _apply_decay(self, value: float, rate: float, dt: float) -> float:
        """Exponential decay helper for disturbance states."""
        # print("Debug: apply_decay called with value =", value, "rate =", rate, "dt =", dt)
        if rate is None or rate <= 0.0:
            return value
        return value * np.exp(-rate * dt)

    def _compute_observability_flags(self, vx: float, delta: float, r: float,
                                      psi_meas: float) -> np.ndarray:
        """
        Compute per-component observability flags based on driving conditions.
        
        Observability analysis:
        - d_vx: Generally observable from encoder (always g=1)
        - d_vy: Requires lateral motion → needs sufficient v_x for slip angles
        - d_r: Requires yaw dynamics → needs steering or rotation
        
        For tire mode [w_r, w_f]:
        - Both require slip angles → need v_x > v_min
        - Front/rear separation requires steering or rotation
        
        Args:
            vx: Longitudinal velocity [m/s]
            delta: Steering angle [rad]
            r: Yaw rate [rad/s]
            psi_meas: Measured heading [rad] (for wrap detection)
            
        Returns:
            Observability flags array (1=observable, 0=unobservable)
        """
        flags = np.ones(self.udim)
        
        # --- Yaw wrap detection ---
        # Detect large jump in heading measurement (wrap from +π to -π or vice versa)
        psi_jump = abs(psi_meas - self.prev_psi_meas)
        if psi_jump > self.yaw_wrap_threshold:
            self.yaw_wrap_counter = self.yaw_wrap_window
        
        yaw_wrapping = self.yaw_wrap_counter > 0
        
        # --- Low speed condition ---
        low_speed = abs(vx) < self.vx_min_observable
        
        # --- Straight driving condition ---
        
        # When steering and yaw rate are both small, front/rear separation is weak
        straight_driving = (abs(delta) < self.delta_min_observable and 
                           abs(r) < self.r_min_observable)
        
        if self.disturbance_mode == 'general':
            # [d_vx, d_vy, d_r]
            # d_vx (index 0): Always observable from encoder
            flags[0] = 1.0
            
            # d_vy (index 1): Needs lateral dynamics → requires v_x for slip angles
            if low_speed:
                flags[1] = 0.0
            
            # d_r (index 2): Needs yaw dynamics
            # - At low speed: yaw dynamics decouple from lateral
            # - During straight driving: weak excitation
            # - During yaw wrap: temporarily gate
            if low_speed or straight_driving or yaw_wrapping:
                flags[2] = 0.0
                
        else:
            # Tire mode: [w_r, w_f]
            # Both need slip angles which require forward motion
            if low_speed:
                flags[0] = 0.0  # w_r
                flags[1] = 0.0  # w_f
            
            # Front/rear separation needs steering
            if straight_driving:
                # Can still estimate sum, but not individual components
                # Reduce confidence in front tire (more sensitive to steering)
                flags[1] = 0.5  # Partial observability for w_f
        
        return flags

    def _apply_observability_gating(self, w_new: np.ndarray, w_prev: np.ndarray,
                                     flags: np.ndarray, dt: float) -> np.ndarray:
        """
        Apply observability gating to disturbance estimates.
        
        When a component is unobservable (flag=0), decay it toward zero
        instead of letting it drift from process noise.
        
        Args:
            w_new: Updated disturbance estimate from EKF
            w_prev: Previous disturbance estimate
            flags: Observability flags [0, 1] per component
            dt: Time step
            
        Returns:
            Gated disturbance estimate
        """
        w_gated = w_new.copy()
        
        if self.disturbance_mode == 'general' and self.udim >= 3:
            decay_rates = [self.gated_decay_rate_dvx, 
                          self.gated_decay_rate_dvy, 
                          self.gated_decay_rate_dr]
        else:
            # Tire mode
            decay_rates = [self.gated_decay_rate_dvy, self.gated_decay_rate_dvy]
        
        for i in range(min(len(flags), len(w_gated))):
            if flags[i] < 1.0:
                if flags[i] == 0.0:
                    # Fully unobservable: decay toward zero
                    w_gated[i] = self._apply_decay(w_prev[i], decay_rates[i], dt)
                else:
                    # Partially observable: blend between EKF update and decay
                    w_decayed = self._apply_decay(w_prev[i], decay_rates[i], dt)
                    w_gated[i] = flags[i] * w_new[i] + (1.0 - flags[i]) * w_decayed
        
        return w_gated

    def _unwrap_angle(self, angle: float, reference: float) -> float:
        """
        Unwrap angle to be continuous with reference.
        
        Args:
            angle: Angle to unwrap [rad]
            reference: Reference angle [rad]
            
        Returns:
            Unwrapped angle [rad]
        """
        diff = angle - reference
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return reference + diff

    def _discretize_augmented(self, A_a: np.ndarray, B_a: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Discretize continuous augmented system matrices using ZOH.

        Args:
            A_a: Continuous augmented state matrix
            B_a: Continuous augmented input matrix
            dt: Sample time

        Returns:
            Tuple of (A_ad, B_ad)
        """
        n_states = A_a.shape[0]
        n_inputs = B_a.shape[1]

        # Van Loan's matrix exponential method
        # M = [[A_a, B_a], [0, 0]]
        M = np.zeros((n_states + n_inputs, n_states + n_inputs))
        M[:n_states, :n_states] = A_a
        M[:n_states, n_states:] = B_a

        # Compute matrix exponential
        M_exp = expm(M * dt)

        # Extract discrete matrices
        A_ad = M_exp[:n_states, :n_states]
        B_ad = M_exp[:n_states, n_states:]

        return A_ad, B_ad

    def _default_params(self) -> Dict:
        """Default vehicle parameters - uses centralized defaults"""
        return get_default_vehicle_params()

    def _default_Q(self) -> np.ndarray:
        """
        Default process noise covariance Q (8×8 or 9×9)

        For 6D+tire mode: xa = [vx, vy, psi, r, X, Y, wr, wf] (8D)
        For 6D+general mode: xa = [vx, vy, psi, r, X, Y, d_vx, d_vy, d_r] (9D)

        TUNING GUIDE for 1/10 scale car:
        - Small for well-measured states (vx, psi, X, Y)
        - HIGH for v_y (weakly observable - only through a_y coupling)
        - HIGH for disturbances (random-walk, need to track changes)
        """
        if self.use_8d_system:
             # 8D state: [vx, vy, psi, r, X, Y, ax, ay] + disturbances
             Q_diag = [
                0.05,   # vx - encoder accurate
                1.0,    # vy - HIGH: weakly observable!
                0.001,  # psi - heading well-measured
                0.1,    # r - gyro good
                0.02,   # X - GPS
                0.02,   # Y - GPS
                0.1,    # ax (new state)
                0.1,    # ay (new state)
             ]
             Q_diag.extend([5.0] * self.udim)
             return np.diag(Q_diag)

        # 6D system: [vx, vy, psi, r, X, Y] + disturbances
        Q_diag = [
            0.05,   # vx - encoder is accurate
            2,    # vy - HIGH: weakly observable, must be loose so a_y innovation
                    # flows to tire residuals instead of being absorbed by v_y corrections.
                    # C[AY,VY] >> F[AY,w], so tight Q_vy starves residual estimation.
            0.001,  # psi - heading well-measured by GPS/IMU
            0.1,    # r - gyro is good but model has uncertainty
            0.02,   # X - GPS
            0.02,   # Y - GPS
        ]

        # Add disturbance noise (random-walk assumption)
        if self.disturbance_mode == 'general':
            # [d_vx, d_vy, d_r] - general additive disturbances
            # d_vy needs HIGH Q because it's key for v_y estimation
            Q_diag.extend([2.0, 2.0, 2.0])  # d_vx, d_vy, d_r
            return np.diag(Q_diag)
        else:
            # [wr, wf] - tire residuals with cross-correlation
            # High correlation encourages estimates to move together,
            # fixing the observability issue where they could diverge.
            q_wr = 5000.0
            q_wf = 5000.0
            corr = self.tire_correlation  # Default: 0.8
            
            # Build Q with correlation block for residuals
            Q_state = np.diag(Q_diag)  # 6x6 state part
            
            # Correlated residual block: [[q_wr, rho*sqrt(q_wr*q_wf)], [rho*sqrt(...), q_wf]]
            # Cross-correlation couples w_r and w_f process noise so they tend
            # to change together. This is physically motivated: similar tires on
            # a symmetric car produce similar residuals. Without this, the EKF
            # null-space (w_r - w_f direction) causes w_f to drift.
            off_diag = corr * np.sqrt(q_wr * q_wf)
            Q_w = np.array([
                [q_wr,     off_diag],
                [off_diag, q_wf]
            ])
            
            # Assemble full Q matrix
            Q = np.zeros((self.augmented_dim, self.augmented_dim))
            Q[:6, :6] = Q_state
            Q[6:8, 6:8] = Q_w
            
            return Q

    def _default_R(self) -> np.ndarray:
        """
        Default measurement noise covariance R (7×7)

        y = [vx, r, psi, X, Y, ay, ax]

        TUNING GUIDE for 1/10 scale QCar sensors:
        - Encoder: very accurate (~0.01 m/s std)
        - IMU gyro: very accurate (~0.01 rad/s std)
        - IMU accel: moderate (~0.1 m/s² std)
        - GPS: moderate (~0.2m position std)
        
        Smaller R = trust sensor more (use for accurate sensors)
        Larger R = trust model more (use for noisy sensors)
        """
        return np.diag([
            0.01,     # vx - encoder variance (m/s)² - very accurate
            0.0001,   # r - gyro variance (rad/s)² - IMU is very accurate!
            0.0004,   # psi - heading variance (rad)² - from GPS/IMU fusion
            0.04,     # X - GPS variance (m)² - ~0.2m std dev
            0.04,     # Y - GPS variance (m)²
            0.01,     # ay - lateral accel variance (m/s²)²
            0.01,     # ax - longitudinal accel variance (m/s²)²
        ])

    def _default_P0(self) -> np.ndarray:
        """
        Default initial error covariance P0

        Reflects initial uncertainty in state estimate.
        Higher values = more uncertainty = faster initial convergence
        """
        # 6D state base P0
        P0_diag = [
            0.05,    # vx - fairly certain
            1.0,     # vy - HIGH: uncertain initial lateral velocity.
                     # Must match high Q_vy to allow a_y innovation to flow to residuals.
            0.1,    # psi - fairly certain from GPS
            0.01,    # r - fairly certain from gyro
            0.01,    # X - uncertain position
            0.01,    # Y - uncertain position
        ]

        # Add disturbance initial uncertainty
        # Higher P0 for residuals allows faster initial convergence.
        # The Kalman gain for w depends on P_w cross-covariance, which
        # builds up from P0 through the E matrix coupling in the augmented dynamics.
        if self.disturbance_mode == 'general':
            P0_diag.extend([0.1, 0.5, 0.2])  # d_vx, d_vy, d_r (conservative start)
        else:
            P0_diag.extend([100.0, 100.0])  # wr, wf - large P0 for fast initial convergence

        return np.diag(P0_diag)



    # =========================================================
    # Numerical Jacobian Computation
    # =========================================================

    def _numerical_jacobian(self, func, x: np.ndarray) -> np.ndarray:
        """
        Compute numerical Jacobian of func at x using central differences

        Args:
            func: Function f(x) -> y
            x: Point to compute Jacobian at

        Returns:
            Jacobian matrix J[i,j] = df_i/dx_j
        """
        n = len(x)
        f0 = func(x)
        m = len(f0)
        J = np.zeros((m, n))

        for j in range(n):
            x_plus = x.copy()
            x_minus = x.copy()
            x_plus[j] += self.epsilon
            x_minus[j] -= self.epsilon
            J[:, j] = (func(x_plus) - func(x_minus)) / (2 * self.epsilon)

        return J

    def compute_scheduling_params(self, state: np.ndarray, delta: float) -> SchedulingParameters:
        """Compute scheduling parameters from current state and input"""
        return self.dynamics.compute_scheduling_params(state, delta)

    def compute_slip_angles(self, state: np.ndarray, delta: float) -> Tuple[float, float]:
        """Compute front and rear slip angles - delegates to centralized dynamics"""
        return self.dynamics.compute_slip_angles(state, delta)

    def compute_A_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """Compute state matrix A(ρ) - delegates to centralized dynamics"""
        return self.dynamics.compute_A_matrix(rho)

    def compute_B_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """Compute input matrix B(ρ) - delegates to centralized dynamics"""
        return self.dynamics.compute_B_matrix(rho)

    def compute_E_matrix(self, rho: SchedulingParameters) -> np.ndarray:
        """Compute residual injection matrix E(ρ) - delegates to centralized dynamics"""
        return self.dynamics.compute_E_matrix(rho)

    def compute_C_matrix(self, rho: SchedulingParameters, active_indices: Optional[list] = None) -> np.ndarray:
        """Compute output matrix C(ρ) - delegates to centralized dynamics"""
        return self.dynamics.compute_C_matrix(rho, active_indices=active_indices)

    def compute_D_matrix(self, rho: SchedulingParameters, active_indices: Optional[list] = None) -> np.ndarray:
        """Compute feedthrough matrix D(ρ) - delegates to centralized dynamics"""
        return self.dynamics.compute_D_matrix(rho, active_indices=active_indices)

    def compute_F_matrix(self, rho: SchedulingParameters, active_indices: Optional[list] = None) -> np.ndarray:
        """Compute residual-to-output matrix F(ρ) - delegates to centralized dynamics"""
        return self.dynamics.compute_F_matrix(rho, active_indices=active_indices)

    def compute_augmented_matrices(self, rho: SchedulingParameters, active_indices: Optional[list] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Compute augmented system matrices - delegates to centralized dynamics"""
        return self.dynamics.compute_augmented_matrices(rho, active_indices=active_indices)

    def compute_ay_innovation(self, measurement: np.ndarray, state_hat: np.ndarray,
                              control_input: np.ndarray, rho: SchedulingParameters) -> float:
        """
        Compute lateral acceleration innovation for UIO-style residual estimation

        ã_y = a_y - (C_ay·x̂ + D_ay·u)

        Constraint: m·ã_y ≈ w_r + cos(δ)·w_f

        Args:
            measurement: Measurement vector [v_x, r, ψ, X, Y, a_y]
            state_hat: State estimate [v_x, v_y, ψ, r, X, Y]
            control_input: Control [δ, a]
            rho: Scheduling parameters

        Returns:
            a_y innovation value
        """
        C = self.compute_C_matrix(rho)
        D = self.compute_D_matrix(rho)

        # Extract a_y row
        C_ay = C[MEAS_IDX_AY, :]
        D_ay = D[MEAS_IDX_AY, :]
        # Predicted a_y from linear model
        ay_predicted = C_ay @ state_hat + D_ay @ control_input

        # Actual a_y measurement
        ay_measured = measurement[MEAS_IDX_AY]
        # Innovation
        ay_innovation = ay_measured - ay_predicted

        return ay_innovation



    # =========================================================
    # EKF Predict + Update
    # =========================================================

    def ekf_predict(self, xa: np.ndarray, P: np.ndarray, 
                    u: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        EKF Prediction Step

        x̂⁻ = f(x̂, u)
        P⁻ = F·P·Fᵀ + Q

        Args:
            xa: Current augmented state estimate
            P: Current error covariance
            u: Control input
            dt: Sample time

        Returns:
            Tuple of (predicted state, predicted covariance)
        """
        # Calculate scheduling parameters
        # xa is augmented state, we need first 6 elements
        rho = self.compute_scheduling_params(xa[:self.state_dim], u[0])

        # Compute continuous augmented matrices
        A_a, B_a, _ = self.compute_augmented_matrices(rho)

        # Discretize using ZOH
        A_ad, B_ad = self._discretize_augmented(A_a, B_a, dt)

        # State prediction using discrete linear dynamics: x[k+1] = A_d·x[k] + B_d·u[k]
        xa_pred = A_ad @ xa + B_ad @ u

        # Jacobian F is just the discrete state matrix A_ad
        F = A_ad

        # Covariance prediction
        # Scale Q with dt if needed, but self.Q is already initialized via defaults * sample_time
        # Scale Q with dt
        Q_k = self.Q_base * dt
        P_pred = F @ P @ F.T + Q_k

        return xa_pred, P_pred

    def ekf_update(self, xa_pred: np.ndarray, P_pred: np.ndarray,
                   y_meas: np.ndarray, u: np.ndarray,
                   R_matrix: Optional[np.ndarray] = None,
                   active_indices: Optional[list] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        EKF Update Step

        K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
        x̂ = x̂⁻ + K·(y - h(x̂⁻, u))
        P = (I - K·H)·P⁻·(I - K·H)ᵀ + K·R·Kᵀ  (Joseph form)

        Args:
            xa_pred: Predicted augmented state
            P_pred: Predicted error covariance
            y_meas: Actual measurement vector (sliced to match active_indices)
            u: Control input
            R_matrix: Optional R matrix to use (must override self.R)
            active_indices: List of active measurement indices

        Returns:
            Tuple of (updated state, updated covariance, innovation, predicted measurement)
        """
        # Use provided R or default (must match active dimensions)
        R = R_matrix if R_matrix is not None else self.R


        # Calculate scheduling parameters from predicted state
        # xa_pred is augmented state, we need first 6 elements
        rho = self.compute_scheduling_params(xa_pred[:self.state_dim], u[0])

        # Compute analytical C matrix (qLPV)
        # We need augmented C matrix to account for tire residuals w
        # compute_augmented_matrices returns C_a (meas_dim x augmented_dim)
        _, _, C_a = self.compute_augmented_matrices(rho, active_indices=active_indices)

        # H is simply C_a in the linear/qLPV update
        H = C_a

        # Compute Feedthrough D matrix 
        # If 8D system, D assumption is usually 0 if measurements are states
        # dynamics.compute_D_matrix returns 6x2 (for 6D system) which might be wrong shape for 8D (7x8)
        # So we handle D dimension carefully

        D = self.compute_D_matrix(rho, active_indices=active_indices)

        # Measurement prediction using Linear/qLPV model: y = C_a·x_a + D·u
        y_pred = C_a @ xa_pred + D @ u

        # Innovation (measurement residual)
        innov = y_meas - y_pred

        # Wrap heading innovation to [-pi, pi]
        # Prevents instability when angle wraps from pi to -pi
        if self.use_8d_system:
            idx_psi_meas = MEAS8_IDX_PSI
        else:
            idx_psi_meas = MEAS_IDX_PSI

        # We need to find where PSI is in the active vector
        psi_in_meas = False
        idx_in_innov = -1

        if active_indices is not None:
            if idx_psi_meas in active_indices:
                idx_in_innov = active_indices.index(idx_psi_meas)
                psi_in_meas = True
        else:
            idx_in_innov = idx_psi_meas
            psi_in_meas = True

        if psi_in_meas and idx_in_innov < len(innov):
            innov[idx_in_innov] = (innov[idx_in_innov] + np.pi) % (2 * np.pi) - np.pi

        # Innovation covariance S = H·P⁻·Hᵀ + R
        S = H @ P_pred @ H.T + R

        # Kalman gain K = P⁻·Hᵀ·S⁻¹
        try:
            K = P_pred @ H.T @ self._safe_inverse(S)
        except np.linalg.LinAlgError:
            # Fallback: use pseudo-inverse
            K = P_pred @ H.T @ np.linalg.pinv(S)

        # State update
        xa_upd = xa_pred + K @ innov

        # Covariance update (Joseph form for numerical stability)
        I = np.eye(P_pred.shape[0])
        IKH = I - K @ H
        P_upd = IKH @ P_pred @ IKH.T + K @ R @ K.T

        # Enforce symmetry and PSD
        P_upd = 0.5 * (P_upd + P_upd.T)

        return xa_upd, P_upd, innov, y_pred

    def update(self, measurement: np.ndarray, control_input: np.ndarray,
               f_nn: Optional[np.ndarray] = None,
               acceleration: Optional[np.ndarray] = None,
               gps_available: bool = True,
               dt: Optional[float] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update qLPV augmented-state observer with EKF predict-update cycle

        Args:
            measurement: Measurement vector.
            control_input: Control [δ, a] OR [steering_cmd, throttle_cmd] if process_control=True
            f_nn: Neural network output (not used)
            acceleration: Full 3D acceleration [a_x, a_y, a_z] if a_y not in measurement
            gps_available: Whether GPS measurements [X, Y] (and possibly ψ) are valid
            dt: Sample time (if None, use self.Ts)

        Returns:
            Tuple of (state_estimate, tire_residual_estimate)
        """
        measurement = measurement.reshape(-1)
        y_full = self._process_measurement(measurement, acceleration)
        # Determine current time step
        current_dt = dt if dt is not None else self.Ts
        # Determine active indices and slice R and y
        idx_X = MEAS8_IDX_X 
        idx_Y = MEAS8_IDX_Y
        idx_PSI = MEAS8_IDX_PSI
        idx_R = MEAS8_IDX_R
        idx_VX = MEAS8_IDX_VX
        idx_AY = MEAS8_IDX_AY
        idx_AX = MEAS8_IDX_AX
        n_meas_in = len(measurement)
        gps_meas_available = False
        imu_ay_available = False
        imu_ax_available = False
        if n_meas_in >= 7:
            # Full 7D: [vx, r, psi, X, Y, ay, ax]
            gps_meas_available = True
            imu_ay_available = True
            imu_ax_available = True
        elif n_meas_in == 6:
            # Legacy 6D: [vx, r, psi, X, Y, ay]
            gps_meas_available = True
            imu_ay_available = True
        elif n_meas_in == 5:
            # Partial 5D: [vx, r, psi, X, Y]
            gps_meas_available = True
        elif n_meas_in == 4:
            # IMU-only 4D: [vx, r, ay, ax]
            imu_ay_available = True
            imu_ax_available = True
        elif n_meas_in == 2:
            # Minimal 2D: [vx, r]
            pass
        # Respect GPS availability flag even if measurements exist
        if not gps_available:
            gps_meas_available = False
        # Acceleration argument can provide IMU channels even if not in measurement vector
        if acceleration is not None:
            if len(acceleration) > 0:
                imu_ax_available = True
            if len(acceleration) > 1:
                imu_ay_available = True
        # Build active measurement indices in canonical order
        active_indices = [idx_VX, idx_R]
        if gps_meas_available:
            active_indices.extend([idx_PSI, idx_X, idx_Y])
        if imu_ay_available:
            active_indices.append(idx_AY)
        if imu_ax_available:
            active_indices.append(idx_AX)
        # Slice full measurement vector to active channels
        y = y_full[active_indices]
        # Slice R matrix
        R_effective = self.R[np.ix_(active_indices, active_indices)]
        # Process measurement handles [vx, r, psi, X, Y, ay, ax] mapping and IMU fallbacks

        # Cached measurements for gating / r_dot estimation
        r_meas = y_full[idx_R]
        ay_meas = y_full[idx_AY]
        psi_meas = y_full[idx_PSI]
        w_prev = self.state_augmented[self.state_dim:].copy()
        
        # Get current steering angle for observability check (will be updated below)
        delta_for_gating = self.current_steering_angle

        # Process Control Input
        u_raw = control_input.reshape(-1)
        # Input is [steering_cmd, throttle_cmd] (normalized -1 to 1)
        # Need to convert to physical [delta, accel] using current state

        # Construct state vector for dynamics module (needs correct format)
        # We use state_hat (estimated state) for this calculation
        current_state = self.state_hat.copy()

        throttle_cmd = u_raw[1] if len(u_raw) > 1 else 0.0
        steering_cmd = u_raw[0]

        # Use centralized dynamics to process control
        accel, _, new_steering_angle = self.dynamics.process_control_inputs(
            throttle_cmd, steering_cmd, current_state, 
            self.current_steering_angle, current_dt
        )

        # Update internal steering state
        self.current_steering_angle = new_steering_angle

        # Form physical control vector for EKF [delta, a]
        u = np.array([self.current_steering_angle, accel])

        # # CRITICAL FIX: Override integrated steering with measurement if available
        # # This prevents the open-loop servo model from drifting away from the true plant
        if len(u_raw) > 2:
            # print("direct u", u_raw)
            # If control_input passed as [steering_cmd, throttle_cmd, true_delta]
            self.current_steering_angle = u_raw[2]
            u[0] = self.current_steering_angle

        delta = u[0]

        # =====================================================
        # STILL CONDITION CHECK (BEFORE EKF)
        # =====================================================
        # Check if vehicle is stationary based on both:
        # 1. Measured velocity is very low
        # 2. Control commands are near zero
        # When stationary, qLPV model is unstable (1/vx singularity),
        # so we use a simplified kinematic fallback instead.
        
        # Determine measured vx from measurement vector
        measured_vx = 0.0
        if idx_VX in active_indices:
            idx_in_y = active_indices.index(idx_VX)
            measured_vx = y[idx_in_y]
        
        # Check control commands (throttle and steering)
        throttle_near_zero = abs(throttle_cmd) < 0.05
        steering_near_zero = abs(steering_cmd) < 0.05
        velocity_near_zero = abs(measured_vx) < 0.1  # Tight threshold: truly stationary
        
        # Smooth blending factor for transition region [V_STILL, V_NOMINAL]
        # 0.0 = fully kinematic (still), 1.0 = fully EKF (moving)
        V_STILL = 0.1   # Below this: pure kinematic
        V_NOMINAL = 0.2  # Above this: full EKF trust
        abs_vx = abs(measured_vx)
        if abs_vx <= V_STILL:
            velocity_blend = 0.0
        elif abs_vx >= V_NOMINAL:
            velocity_blend = 1.0
        else:
            # Smooth cubic interpolation (no derivative discontinuity)
            t = (abs_vx - V_STILL) / (V_NOMINAL - V_STILL)
            velocity_blend = 3.0 * t**2 - 2.0 * t**3
        
        # Vehicle is "still" if velocity is low AND (no throttle command OR no significant input)
        is_vehicle_still = velocity_near_zero and throttle_near_zero
        
        if is_vehicle_still:
            # =====================================================
            # KINEMATIC FALLBACK (for start/stop stability)
            # =====================================================
            # Skip qLPV EKF which has 1/vx singularity.
            # Use direct sensor measurements + simple kinematic update.
            
            # Keep current covariance (don't propagate with unstable model)
            P_upd = self.P.copy()
            
            # Use sensor measurements directly where available
            xa_upd = self.state_augmented.copy()
            
            # Update velocities: force to measured/zero (no lateral dynamics when still)
            xa_upd[IDX_VX] = measured_vx  # Use measured vx (should be ~0)
            xa_upd[IDX_VY] = 0.0  # No lateral velocity when stationary
            xa_upd[IDX_R] = y_full[idx_R]  # Use measured yaw rate (should be ~0)
            
            # Update position/heading from GPS if available
            if gps_meas_available:
                # Simple position update from GPS (no dynamics needed)
                xa_upd[IDX_X] = y_full[idx_X]
                xa_upd[IDX_Y] = y_full[idx_Y]
                xa_upd[IDX_PSI] = y_full[idx_PSI]
            
            # Zero out disturbances when stationary (no meaningful estimation)
            xa_upd[self.state_dim:] = 0.0
            
            # Set innovation to zero (no meaningful update)
            innov = np.zeros(len(y))
            y_pred = y.copy()
            
            # Reset r_dot differentiator to prevent transient on restart
            if self.rdot_diff is not None:
                self.rdot_meas = 0.0
            
        else:
            # =====================================================
            # FULL qLPV EKF (vehicle is moving)
            # =====================================================
            
            # --- Low-speed R scaling (prevents startup spike) ---
            # At low velocity, the qLPV model is poorly conditioned (1/vx terms).
            # Inflate R to make filter conservative, then smoothly reduce to nominal.
            if velocity_blend < 1.0:
                # Scale factor: 1.0 at full speed, up to 2x at transition start
                r_scale = 1.0 + 1 * (1.0 - velocity_blend)
                R_effective_scaled = R_effective 
                # NOTE: Previously reduced Q for disturbance states during transition:
                #   self.Q_base[i, i] *= velocity_blend
                # This was REMOVED because it made the filter extremely stiff for
                # tire residuals at low speed, preventing estimation from building up.
                # The R-scaling above is sufficient for conservative behavior.
                Q_save = None
            else:
                R_effective_scaled = R_effective
                Q_save = None
            
            # EKF PREDICT
            xa_pred, P_pred = self.ekf_predict(self.state_augmented, self.P, u, current_dt)

            # EKF UPDATE
            xa_upd, P_upd, innov, y_pred = self.ekf_update(xa_pred, P_pred, y, u, R_matrix=R_effective_scaled, active_indices=active_indices)
            
            # NOTE: Previously blended residuals with zero during transition:
            #   xa_upd[self.state_dim:] *= velocity_blend
            # This was REMOVED because it crushed tire residual estimates every cycle,
            # forcing the filter to re-estimate from scratch. The EKF's R-scaling
            # during transition already provides conservative behavior.
            # The residuals should be allowed to build up naturally.

            # =====================================================
            # TIRE RESIDUAL EQUALITY CONSTRAINT (Observability Fix)
            # =====================================================
            # The E matrix gives r_dot opposite signs for w_r and w_f:
            #   r_dot contribution: -lr/Iz * w_r + lf/Iz * cos(δ) * w_f
            # When w_r ≈ w_f (same tire type, symmetric car), these cancel
            # in the r dynamics → the r measurement cannot separate them.
            # The a_y measurement only sees their SUM (w_r + w_f).
            # This creates a null-space in (w_r - w_f), causing w_f to drift.
            #
            # Fix: Add pseudo-measurement y_eq = w_r - w_f ≈ 0
            # This directly constrains the null-space.
            if self.disturbance_mode == 'tire' and self.tire_correlation > 0.5:
                # H_eq selects (w_r - w_f): [0...0, 1, -1]
                H_eq = np.zeros((1, self.augmented_dim))
                H_eq[0, self.state_dim] = 1.0      # w_r coefficient
                H_eq[0, self.state_dim + 1] = -1.0  # -w_f coefficient
                
                # Predicted difference
                y_eq_pred = H_eq @ xa_upd
                
                # Measurement: we expect w_r - w_f ≈ 0
                y_eq_meas = 0.0
                
                # Measurement variance: lower = stronger constraint
                # With correlation=0.8: R_eq = 0.1*(1-0.8+0.1) = 0.03 (strong)
                # This is tight enough to keep w_r ≈ w_f while still allowing
                # small differences when the steering angle creates asymmetry.
                R_eq_base = 0.1
                R_eq = np.array([[R_eq_base * (1.0 - self.tire_correlation + 0.1)]])
                
                # Kalman update for equality constraint
                S_eq = H_eq @ P_upd @ H_eq.T + R_eq
                K_eq = P_upd @ H_eq.T @ self._safe_inverse(S_eq)
                
                # Update state with constraint
                xa_upd = xa_upd + (K_eq @ (y_eq_meas - y_eq_pred)).flatten()
                
                # Update covariance (Joseph form)
                I_eq = np.eye(self.augmented_dim)
                IKH_eq = I_eq - K_eq @ H_eq
                P_upd = IKH_eq @ P_upd @ IKH_eq.T + K_eq @ R_eq @ K_eq.T
                P_upd = 0.5 * (P_upd + P_upd.T)

            # Optional r_dot pseudo-measurement update (improves d_r observability)
            if self.use_rdot_differentiator and self.rdot_diff is not None and self.disturbance_mode == 'general':
                self.rdot_meas = self.rdot_diff.update(r_meas)
                use_rdot_update = True
                if self.enable_excitation_gating:
                    use_rdot_update = abs(self.rdot_meas) >= self.rdot_excitation_threshold
                if use_rdot_update:
                    rho_rdot = self.compute_scheduling_params(xa_upd[:self.state_dim], delta)
                    A_rdot = self.compute_A_matrix(rho_rdot)
                    B_rdot = self.compute_B_matrix(rho_rdot)
                    E_rdot = self.compute_E_matrix(rho_rdot)

                    H_rdot = np.zeros((1, self.augmented_dim))
                    H_rdot[0, :self.state_dim] = A_rdot[IDX_R, :]
                    H_rdot[0, self.state_dim:] = E_rdot[IDX_R, :]

                    y_rdot_pred = (H_rdot @ xa_upd + (B_rdot[IDX_R, :] @ u)).reshape(-1)
                    R_rdot = np.array([[self.rdot_meas_var]])
                    S_rdot = H_rdot @ P_upd @ H_rdot.T + R_rdot
                    K_rdot = P_upd @ H_rdot.T @ self._safe_inverse(S_rdot)

                    xa_upd = xa_upd + (K_rdot @ (self.rdot_meas - y_rdot_pred)).reshape(-1)
                    I = np.eye(P_upd.shape[0])
                    IKH = I - K_rdot @ H_rdot
                    P_upd = IKH @ P_upd @ IKH.T + K_rdot @ R_rdot @ K_rdot.T
                    P_upd = 0.5 * (P_upd + P_upd.T)
            else:
                self.rdot_meas = 0.0

        # Store updated state and covariance
        self.state_augmented = xa_upd
        self.P = P_upd

        # Wrap heading state to [-pi, pi]
        # Ensures estimated yaw stays in the same range as GPS sensors
        # self.IDX_PSI = 2 same for 6d and 8d
        self.state_augmented[IDX_PSI] = (self.state_augmented[IDX_PSI] + np.pi) % (2 * np.pi) - np.pi

        # =====================================================
        # ADDITIONAL STILL CONDITION (post-update safety)
        # =====================================================
        # Even after qLPV update, if velocity is very low, zero out velocities
        # to prevent small numerical drift when truly stationary.
        if abs(measured_vx) < V_STILL and throttle_near_zero:
            # Force velocities to zero
            self.state_augmented[IDX_VX] = 0.0
            self.state_augmented[IDX_VY] = 0.0
            self.state_augmented[IDX_R] = 0.0
            # Force disturbances to zero
            self.state_augmented[self.state_dim:] = 0.0


        # =====================================================
        # OBSERVABILITY-BASED GATING (new approach)
        # =====================================================
        if self.enable_observability_gating and self.udim >= 2:
            d_start = self.state_dim
            w_new = self.state_augmented[d_start:d_start + self.udim].copy()
            
            # Get current velocity estimate for observability check
            vx_est = self.state_augmented[IDX_VX]
            r_est = self.state_augmented[IDX_R]
            
            # Compute observability flags based on driving conditions
            self.observability_flags = self._compute_observability_flags(
                vx=vx_est,
                delta=delta,
                r=r_est,
                psi_meas=psi_meas
            )
            
            # Apply gating: decay unobservable components toward zero
            w_gated = self._apply_observability_gating(w_new, w_prev, 
                                                        self.observability_flags, current_dt)
            
            self.state_augmented[d_start:d_start + self.udim] = w_gated
            
            # Update yaw wrap counter
            if self.yaw_wrap_counter > 0:
                self.yaw_wrap_counter -= 1
            self.prev_psi_meas = psi_meas
            
        # =====================================================
        # LEGACY: Excitation gating + decay (if observability gating disabled)
        # =====================================================
        elif self.enable_excitation_gating and self.disturbance_mode == 'general' and self.udim >= 2:
            d_start = self.state_dim
            w_new = self.state_augmented[d_start:d_start + self.udim].copy()

            ay_excited = imu_ay_available and abs(ay_meas) >= self.ay_excitation_threshold
            if self.use_rdot_differentiator and self.rdot_diff is not None:
                r_excited = abs(self.rdot_meas) >= self.rdot_excitation_threshold
            else:
                r_excited = abs(r_meas) >= self.r_excitation_threshold

            # d_vy gating / decay
            if self.udim >= 2 and not ay_excited:
                base = w_prev[1]
                if self.enable_disturbance_decay or self.decay_only_when_unexcited:
                    w_new[1] = self._apply_decay(base, self.decay_rate_dvy, current_dt)
                else:
                    w_new[1] = base

            # d_r gating / decay
            if self.udim >= 3 and not r_excited:
                base = w_prev[2]
                if self.enable_disturbance_decay or self.decay_only_when_unexcited:
                    w_new[2] = self._apply_decay(base, self.decay_rate_dr, current_dt)
                else:
                    w_new[2] = base

            # Optional decay for d_vx
            if self.enable_disturbance_decay and not self.decay_only_when_unexcited and self.udim >= 1:
                w_new[0] = self._apply_decay(w_new[0], self.decay_rate_dvx, current_dt)

            self.state_augmented[d_start:d_start + self.udim] = w_new

        #  ---------------- JUST for DEBUG ----------------
        # Store Kalman gain for diagnostics - recompute using R_effective
        # Store Kalman gain for diagnostics - recompute using R_effective and analytical matrices
        try:
            rho = self.compute_scheduling_params(xa_pred[:self.state_dim], delta)
            _, _, C_a = self.compute_augmented_matrices(rho, active_indices=active_indices)
            H_curr = C_a
            S_curr = H_curr @ P_pred @ H_curr.T + R_effective
            self.K = P_pred @ H_curr.T @ np.linalg.pinv(S_curr)
        except Exception:
            self.K = np.zeros((self.augmented_dim, self.meas_dim))
        # Store innovation for diagnostics (padding to full size for compatibility)
        self.innovation = np.zeros(self.meas_dim)
        if active_indices is not None and len(innov) == len(active_indices):
            self.innovation[active_indices] = innov
        else:
            self.innovation[:len(innov)] = innov

        # Extract state and residual estimates
        self.state_hat = self.state_augmented[:self.state_dim].copy()
        self.w_hat = self.state_augmented[self.state_dim:].copy()
        self.tire_info_layer_1 =  self.dynamics._calculate_tire_info(self.state_augmented[IDX_VX], self.state_augmented[IDX_VY], self.state_augmented[IDX_R], delta, self.state_augmented[self.state_dim], self.state_augmented[self.state_dim + 1])

        # Update UIO-style residual constraint
        self.ay_innovation = 0.0
        self.w_constraint = 0.0


        # Copy tire residuals to base class attribute for interface compatibility
        self.f_uk_hat = self.w_hat.copy()

        # # Optional gyro bias update
        # if self.include_gyro_bias:
        #     if self.use_8d_system:
        #         idx_r_meas = MEAS8_IDX_R
        #         idx_r_state = IDX8_R
        #     else:
        #         idx_r_meas = MEAS_IDX_R
        #         idx_r_state = IDX_R

        #     r_error = y[idx_r_meas] - self.state_hat[idx_r_state]
        #     self.gyro_bias += 0.001 * r_error

        return self.state_hat.copy(), self.w_hat.copy()

    def _process_measurement(self, measurement: np.ndarray,
                             acceleration: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Process input measurement to get full measurement vector.

        Handles:
            - Full 7D: [v_x, r, psi, X, Y, a_y, a_x]
            - No GPS 4D: [v_x, r, a_y, a_x]
            - Legacy 6D: [v_x, r, psi, X, Y, a_y]
            - Legacy 2D: [v_x, r]

        Args:
            measurement: Input measurement (various formats)
            acceleration: Optional 3D acceleration [a_x, a_y, a_z]

        Returns:
            Full measurement vector y
        """
        measurement = measurement.reshape(-1)
        n_input = len(measurement)
        y = np.zeros(self.meas_dim)

        # 1. Basic Kinematics (always present)
        val_vx = measurement[0] if n_input > 0 else 0.0
        val_r = measurement[1] if n_input > 1 else 0.0

        if self.use_8d_system:
            y[MEAS8_IDX_VX] = val_vx
            y[MEAS8_IDX_R] = val_r
        else:
            y[MEAS_IDX_VX] = val_vx
            y[MEAS_IDX_R] = val_r

        # 2. Extract values based on input dimension
        val_psi = None
        val_X = None
        val_Y = None
        val_ay = None
        val_ax = None

        if n_input >= 7:
            # Full 7D: [vx, r, psi, X, Y, ay, ax]
            val_psi = measurement[2]
            val_X = measurement[3]
            val_Y = measurement[4]
            val_ay = measurement[5]
            val_ax = measurement[6]
        elif n_input == 6:
            # Legacy 6D: [vx, r, psi, X, Y, ay]
            val_psi = measurement[2]
            val_X = measurement[3]
            val_Y = measurement[4]
            val_ay = measurement[5]
        elif n_input == 4:
            # No GPS 4D: [vx, r, ay, ax]
            val_ay = measurement[2]
            val_ax = measurement[3]
        elif n_input == 5:
            # Partial 5D: [vx, r, psi, X, Y]
            val_psi = measurement[2]
            val_X = measurement[3]
            val_Y = measurement[4]

        # 3. Fill from Acceleration argument if missing
        if acceleration is not None:
             if val_ax is None and len(acceleration) > 0:
                 val_ax = acceleration[0]
             if val_ay is None and len(acceleration) > 1:
                 val_ay = acceleration[1]

        # 4. Fill missing GPS/State vars with current estimate
        if self.use_8d_system:
            idx_psi = IDX8_PSI
            idx_X = IDX8_X
            idx_Y = IDX8_Y
        else:
            idx_psi = IDX_PSI
            idx_X = IDX_X
            idx_Y = IDX_Y

        if val_psi is None: val_psi = self.state_hat[idx_psi]
        if val_X is None: val_X = self.state_hat[idx_X]
        if val_Y is None: val_Y = self.state_hat[idx_Y]

        # 5. Populate Result Vector
        if self.use_8d_system:
            y[MEAS8_IDX_PSI] = val_psi
            y[MEAS8_IDX_X] = val_X
            y[MEAS8_IDX_Y] = val_Y
            y[MEAS8_IDX_AY] = val_ay if val_ay is not None else 0.0
            y[MEAS8_IDX_AX] = val_ax if val_ax is not None else 0.0
        else:
            y[MEAS_IDX_PSI] = val_psi
            y[MEAS_IDX_X] = val_X
            y[MEAS_IDX_Y] = val_Y
            y[MEAS_IDX_AY] = val_ay if val_ay is not None else (val_r * val_vx)
            # Optional a_x channel if available in measurement space
            if val_ax is not None and self.meas_dim > MEAS_IDX_AX:
                y[MEAS_IDX_AX] = val_ax

        return y




    def get_state(self) -> np.ndarray:
        """Get current 6D state estimate [v_x, v_y, ψ, r, X, Y]"""
        return self.state_hat.copy()

    def get_tire_residuals(self) -> np.ndarray:
        """Get current tire residual estimates [w_r, w_f]"""
        return self.w_hat.copy()

    def get_augmented_state(self) -> np.ndarray:
        """Get full augmented state [x; w]"""
        return self.state_augmented.copy()

    def get_unknown_input(self) -> np.ndarray:
        """Get unknown input estimate (alias for tire residuals)"""
        return self.w_hat.copy()

    def get_ay_constraint(self) -> float:
        """
        Get the a_y-based constraint on tire residuals

        Returns m·ã_y ≈ w_r + cos(δ)·w_f
        """
        return self.w_constraint

    

    def check_uio_rank_condition(self, delta: float) -> bool:
        """
        Check UIO existence condition (rank condition)

        rank([C·E; F]) = rank(E) = 2

        This should be satisfied for normal driving conditions.

        Args:
            delta: Current steering angle

        Returns:
            True if rank condition is satisfied
        """
        rho = self.compute_scheduling_params(self.state_hat, delta)

        C = self.compute_C_matrix(rho)
        E = self.compute_E_matrix(rho)
        F = self.compute_F_matrix(rho)

        # Stack CE and F
        CE = C @ E
        stacked = np.vstack([CE, F])

        # Compute ranks
        rank_stacked = np.linalg.matrix_rank(stacked)
        rank_E = np.linalg.matrix_rank(E)

        return rank_stacked == rank_E == 2


    def reset(self, initial_state: Optional[np.ndarray] = None, 
              initial_position: Optional[np.ndarray] = None):
        """
        Reset observer state

        Args:
            initial_state: Initial state [v_x, v_y, ψ, r, X, Y] or [v_x, v_y, ψ, r]
            initial_position: Initial position [X, Y] if not in initial_state
        """
        if initial_state is not None:
            initial_state = initial_state.reshape(-1)
            if len(initial_state) >= self.state_dim:
                self.state_hat = initial_state[:self.state_dim].copy()
            else:
                self.state_hat = np.zeros(self.state_dim)
                self.state_hat[:len(initial_state)] = initial_state
                if initial_position is not None:
                    if self.use_8d_system:
                        self.state_hat[IDX8_X] = initial_position[0]
                        self.state_hat[IDX8_Y] = initial_position[1]
                    else:
                        self.state_hat[IDX_X] = initial_position[0]
                        self.state_hat[IDX_Y] = initial_position[1]
        else:
            self.state_hat = np.zeros(self.state_dim)
            if initial_position is not None:
                if self.use_8d_system:
                    self.state_hat[IDX8_X] = initial_position[0]
                    self.state_hat[IDX8_Y] = initial_position[1]
                else:
                    self.state_hat[IDX_X] = initial_position[0]
                    self.state_hat[IDX_Y] = initial_position[1]
        self.current_steering_angle = 0.0
        if self.rdot_diff is not None:
            self.rdot_diff.reset(0.0)
            self.rdot_meas = 0.0
        # Reset yaw wrap tracking
        self.yaw_wrap_counter = 0
        self.prev_psi_meas = 0.0
        self.observability_flags = np.ones(self.udim)
        # Reset augmented state
        self.state_augmented = np.zeros(self.augmented_dim)
        self.state_augmented[:self.state_dim] = self.state_hat

        # Reset tire residuals
        self.w_hat = np.zeros(self.udim)
        self.f_uk_hat = np.zeros(self.udim)

        # Reset EKF covariance to initial
        self.P = self._default_P0()

        # Reset Kalman gain
        self.K = np.zeros((self.augmented_dim, self.meas_dim))

        # Reset innovation
        self.innovation = np.zeros(self.meas_dim)

    def get_unknown_input_estimate(self) -> np.ndarray:
        """Get estimate of unknown inputs (tire residuals or disturbances)"""
        # Extract from augmented state
        start_idx = self.state_dim
        end_idx = self.state_dim + self.udim
        return self.state_augmented[start_idx:end_idx].copy()


    # =========================================================
    # EKF Diagnostic Getters
    # =========================================================

    def get_covariance(self) -> np.ndarray:
        """Get current error covariance matrix P (8×8)"""
        return self.P.copy()

    def get_kalman_gain(self) -> np.ndarray:
        """Get current Kalman gain matrix K (8×6)"""
        return self.K.copy()

    def get_innovation(self) -> np.ndarray:
        """Get last innovation (measurement residual) vector"""
        return self.innovation.copy()

    def get_state_uncertainty(self) -> np.ndarray:
        """
        Get state estimation uncertainty (standard deviations)
        Returns diagonal of sqrt(P) for state portion
        """
        return np.sqrt(np.diag(self.P)[:self.state_dim])
    def get_residual_uncertainty(self) -> np.ndarray:
        """
        Get tire residual estimation uncertainty (standard deviations)
        Returns diagonal of sqrt(P) for residual portion
        """
        return np.sqrt(np.diag(self.P)[self.state_dim:])

    def get_observability_flags(self) -> np.ndarray:
        """
        Get current observability flags for disturbance components.
        
        Returns:
            Array of flags [0, 1] per disturbance component:
            - 1.0 = fully observable
            - 0.0 = unobservable (being decayed)
            - 0.5 = partially observable
        """
        return self.observability_flags.copy()

    def get_yaw_wrap_status(self) -> Tuple[bool, int]:
        """
        Get yaw wrap gating status.
        
        Returns:
            Tuple of (is_wrapping, samples_remaining)
        """
        return (self.yaw_wrap_counter > 0, self.yaw_wrap_counter)

    # =========================================================
    # Tuning Methods
    # =========================================================

    def set_Q(self, Q: np.ndarray):
        """
        Set process noise covariance Q
        Args:
            Q: Process noise covariance (8×8)
        """
        if Q.shape != (self.augmented_dim, self.augmented_dim):
            raise ValueError(f"Q must be {self.augmented_dim}×{self.augmented_dim}")
        self.Q_base = Q.copy()
    def set_R(self, R: np.ndarray):
        """
        Set measurement noise covariance R
        Args:
            R: Measurement noise covariance (6×6)
        """
        if R.shape != (self.meas_dim, self.meas_dim):
            raise ValueError(f"R must be {self.meas_dim}×{self.meas_dim}")
        self.R = R.copy()
    def set_P(self, P: np.ndarray):
        """
        Set error covariance P (for re-initialization)
        Args:
            P: Error covariance (8×8)
        """
        if P.shape != (self.augmented_dim, self.augmented_dim):
            raise ValueError(f"P must be {self.augmented_dim}×{self.augmented_dim}")
        self.P = P.copy()

def create_qlpv_observer(sample_time: float = 0.02,
                         vehicle_params: Optional[Dict] = None,
                         **kwargs) -> qLPVKalmanObserver:
    """
    Factory function to create qLPV augmented-state observer

    Args:
        sample_time: Sample time [s]
        vehicle_params: Vehicle parameters dictionary
        **kwargs: Additional arguments passed to constructor

    Returns:
        Configured qLPVAugmentedObserver instance
    """
    return qLPVKalmanObserver(
        sample_time=sample_time,
        vehicle_params=vehicle_params,
        **kwargs
    )

