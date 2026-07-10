"""
Simplified Tire Model Estimator with EKF

A clean, focused implementation of tire force residual estimation using
an Extended Kalman Filter (EKF) based qLPV observer. This is a simplified
version of qlpv_observer_kalma.py, stripped down to only tire estimation.

=============================================================================
STATE & MEASUREMENT STRUCTURE
=============================================================================
State:           x = [v_x, v_y, ψ, r, X, Y]ᵀ           (6D vehicle state)
Tire residuals:  w = [w_r, w_f]ᵀ                       (2D unknown inputs)
Augmented state: x_a = [x; w]ᵀ                         (8D total)

Measurements:    y = [v_x, r, ψ, X, Y, a_y]ᵀ           (6D)

=============================================================================
TIRE MODEL
=============================================================================
Linear tire force:  F_y = C * α     (cornering stiffness × slip angle)
Actual tire force:  F_y = C * α + w (linear + residual)

Residuals:
    w_r = F_yr_actual - Cr * α_r    (rear tire residual)
    w_f = F_yf_actual - Cf * α_f    (front tire residual)

Slip angles:
    α_f = δ - (v_y + lf*r) / v_x    (front slip angle)
    α_r = -(v_y - lr*r) / v_x       (rear slip angle)

=============================================================================
EKF ALGORITHM
=============================================================================
Predict:
    x̂_a⁻ = A_d·x̂_a + B_d·u         (discrete state prediction)
    P⁻ = F·P·Fᵀ + Q                  (covariance prediction)

Update:
    K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹       (Kalman gain)
    x̂_a = x̂_a⁻ + K·(y - C·x̂_a⁻)    (state update)
    P = (I - K·H)·P⁻                 (covariance update)

=============================================================================
"""

import numpy as np
from typing import Optional, Dict, Tuple
from scipy.linalg import expm


# =============================================================================
# STATE INDICES (for clarity)
# =============================================================================
IDX_VX, IDX_VY, IDX_PSI, IDX_R, IDX_X, IDX_Y = 0, 1, 2, 3, 4, 5
MEAS_VX, MEAS_R, MEAS_PSI, MEAS_X, MEAS_Y, MEAS_AY = 0, 1, 2, 3, 4, 5


class TireEstimatorSimple:
    """
    Simplified qLPV EKF Observer for Tire Force Residual Estimation
    
    A clean, focused implementation that estimates:
    - Vehicle states: [v_x, v_y, ψ, r, X, Y]
    - Tire force residuals: [w_r, w_f]
    
    Usage:
        estimator = TireEstimatorSimple(sample_time=0.02)
        
        # In control loop:
        measurement = [vx, r, psi, X, Y, ay]
        control = [delta, accel]
        state_est, tire_residuals = estimator.update(measurement, control)
        tire_info = estimator.get_tire_info()
    """
    
    def __init__(self, 
                 sample_time: float = 0.02,
                 vehicle_params: Optional[Dict] = None):
        """
        Initialize the tire estimator.
        
        Args:
            sample_time: Sample time Ts [s]
            vehicle_params: Vehicle parameters dict with keys:
                - m: vehicle mass [kg]
                - Iz: yaw inertia [kg·m²]
                - lf: front axle distance [m]
                - lr: rear axle distance [m]
                - Cf: front cornering stiffness [N/rad]
                - Cr: rear cornering stiffness [N/rad]
        """
        # =====================================================
        # DIMENSIONS
        # =====================================================
        self.state_dim = 6          # [vx, vy, psi, r, X, Y]
        self.residual_dim = 2       # [w_r, w_f]
        self.augmented_dim = 8      # [state; residuals]
        self.meas_dim = 6           # [vx, r, psi, X, Y, ay]
        
        self.Ts = sample_time
        
        # =====================================================
        # VEHICLE PARAMETERS
        # =====================================================
        self.params = self._default_vehicle_params()
        if vehicle_params is not None:
            self.params.update(vehicle_params)
        
        # Extract for convenience
        self.m = self.params['m']
        self.Iz = self.params['Iz']
        self.lf = self.params['lf']
        self.lr = self.params['lr']
        self.Cf = self.params['Cf']
        self.Cr = self.params['Cr']
        self.min_vx = self.params.get('vx_min', 0.3)
        
        # =====================================================
        # STATE INITIALIZATION
        # =====================================================
        self.x_aug = np.zeros(self.augmented_dim)  # Augmented state [x; w]
        self.x_hat = np.zeros(self.state_dim)       # State estimate
        self.w_hat = np.zeros(self.residual_dim)    # Tire residual estimate
        
        # =====================================================
        # EKF COVARIANCE MATRICES
        # =====================================================
        self.Q = self._default_Q()   # Process noise
        self.R = self._default_R()   # Measurement noise
        self.P = self._default_P0()  # Error covariance
        
        # =====================================================
        # TIRE INFO STORAGE
        # =====================================================
        self.tire_info = {
            'Fyf_linear': 0.0,    # Front lateral force (linear model)
            'Fyr_linear': 0.0,    # Rear lateral force (linear model)
            'Fyf_total': 0.0,     # Front lateral force (with residual)
            'Fyr_total': 0.0,     # Rear lateral force (with residual)
            'alpha_f': 0.0,       # Front slip angle
            'alpha_r': 0.0,       # Rear slip angle
            'w_f': 0.0,           # Front tire residual
            'w_r': 0.0,           # Rear tire residual
        }
        
        # Store current steering for scheduling
        self.current_delta = 0.0
    
    # =========================================================
    # DEFAULT PARAMETERS
    # =========================================================
    
    def _default_vehicle_params(self) -> Dict:
        """Default vehicle parameters for 1/10 scale QCar"""
        return {
            'm': 3.5,        # Mass [kg]
            'Iz': 0.05,      # Yaw inertia [kg·m²]
            'lf': 0.128,     # Front axle distance [m]
            'lr': 0.128,     # Rear axle distance [m]
            'Cf': 120.0,     # Front cornering stiffness [N/rad]
            'Cr': 120.0,     # Rear cornering stiffness [N/rad]
            'vx_min': 0.3,   # Minimum velocity threshold [m/s]
        }
    
    def _default_Q(self) -> np.ndarray:
        """
        Process noise covariance Q (8×8)
        
        Tuning guide:
        - Small for well-measured states (vx, psi, position)
        - HIGH for weakly observable states (vy)
        - HIGH for tire residuals (random-walk model)
        """
        # State part [vx, vy, psi, r, X, Y]
        Q_state = np.diag([
            0.05,   # vx - encoder accurate
            0.5,    # vy - weakly observable
            0.001,  # psi - GPS/IMU heading good
            0.1,    # r - gyro good
            0.02,   # X - GPS
            0.02,   # Y - GPS
        ])
        
        # Tire residual part [w_r, w_f] with correlation
        # Correlation encourages similar residual estimates (physical prior)
        q_w = 1.0
        correlation = 0.8
        off_diag = correlation * q_w
        Q_w = np.array([
            [q_w, off_diag],
            [off_diag, q_w]
        ])
        
        # Assemble full Q
        Q = np.zeros((self.augmented_dim, self.augmented_dim))
        Q[:6, :6] = Q_state
        Q[6:8, 6:8] = Q_w
        return Q
    
    def _default_R(self) -> np.ndarray:
        """
        Measurement noise covariance R (6×6)
        
        y = [vx, r, psi, X, Y, ay]
        """
        return np.diag([
            0.01,    # vx - encoder very accurate
            0.0001,  # r - gyro very accurate
            0.0004,  # psi - heading from GPS/IMU
            0.04,    # X - GPS (~0.2m std)
            0.04,    # Y - GPS
            0.01,    # ay - IMU accelerometer
        ])
    
    def _default_P0(self) -> np.ndarray:
        """Initial error covariance P0"""
        return np.diag([
            0.5,    # vx
            0.5,    # vy - uncertain
            0.1,    # psi
            0.1,    # r
            1.0,    # X
            1.0,    # Y
            5.0,    # w_r
            5.0,    # w_f
        ])
    
    # =========================================================
    # qLPV SYSTEM MATRICES
    # =========================================================
    
    def compute_slip_angles(self, vx: float, vy: float, r: float, delta: float) -> Tuple[float, float]:
        """
        Compute tire slip angles.
        
        Args:
            vx: Longitudinal velocity [m/s]
            vy: Lateral velocity [m/s]  
            r: Yaw rate [rad/s]
            delta: Steering angle [rad]
            
        Returns:
            (alpha_f, alpha_r): Front and rear slip angles [rad]
        """
        # Avoid division by zero
        vx_safe = max(abs(vx), self.min_vx) * np.sign(vx) if vx != 0 else self.min_vx
        
        alpha_f = delta - (vy + self.lf * r) / vx_safe
        alpha_r = -(vy - self.lr * r) / vx_safe
        
        return alpha_f, alpha_r
    
    def compute_A_matrix(self, vx: float, delta: float) -> np.ndarray:
        """
        Compute state matrix A(ρ) for the qLPV model.
        
        The qLPV model captures bicycle dynamics with scheduling on velocity and steering.
        """
        vx_safe = max(abs(vx), self.min_vx)
        cos_d = np.cos(delta)
        sin_d = np.sin(delta)
        psi = self.x_hat[IDX_PSI]
        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)
        vy = self.x_hat[IDX_VY]
        r = self.x_hat[IDX_R]
        
        inv_vx = 1.0 / vx_safe
        
        A = np.zeros((6, 6))
        
        # v_x dynamics (longitudinal)
        # ẋ = ... + r·v_y (Coriolis)
        A[IDX_VX, IDX_VY] = r
        A[IDX_VX, IDX_R] = vy
        
        # v_y dynamics (lateral)
        # ẏ = -r·v_x + (1/m)·[Cf·αf + Cr·αr]
        # αf = δ - (vy + lf·r)/vx, αr = -(vy - lr·r)/vx
        A[IDX_VY, IDX_VX] = -r
        A[IDX_VY, IDX_VY] = -(self.Cf + self.Cr) / (self.m * vx_safe)
        A[IDX_VY, IDX_R] = -vx - (self.lf * self.Cf - self.lr * self.Cr) / (self.m * vx_safe)
        
        # ψ dynamics (heading)
        A[IDX_PSI, IDX_R] = 1.0
        
        # r dynamics (yaw rate)
        A[IDX_R, IDX_VY] = -(self.lf * self.Cf - self.lr * self.Cr) / (self.Iz * vx_safe)
        A[IDX_R, IDX_R] = -(self.lf**2 * self.Cf + self.lr**2 * self.Cr) / (self.Iz * vx_safe)
        
        # X dynamics (global X position)
        A[IDX_X, IDX_VX] = cos_psi
        A[IDX_X, IDX_VY] = -sin_psi
        
        # Y dynamics (global Y position)
        A[IDX_Y, IDX_VX] = sin_psi
        A[IDX_Y, IDX_VY] = cos_psi
        
        return A
    
    def compute_B_matrix(self, vx: float, delta: float) -> np.ndarray:
        """
        Compute input matrix B(ρ).
        
        Input u = [delta, accel]
        """
        vx_safe = max(abs(vx), self.min_vx)
        cos_d = np.cos(delta)
        
        B = np.zeros((6, 2))
        
        # v_x response to acceleration
        B[IDX_VX, 1] = 1.0
        
        # v_y response to steering (through front tire)
        B[IDX_VY, 0] = self.Cf / self.m * cos_d
        
        # r response to steering (yaw moment from front tire)
        B[IDX_R, 0] = self.lf * self.Cf / self.Iz * cos_d
        
        return B
    
    def compute_E_matrix(self, delta: float) -> np.ndarray:
        """
        Compute residual injection matrix E(ρ).
        
        Maps tire residuals [w_r, w_f] to state dynamics.
        """
        cos_d = np.cos(delta)
        
        E = np.zeros((6, 2))
        
        # Residuals affect v_y (lateral)
        E[IDX_VY, 0] = 1.0 / self.m           # w_r → v_y
        E[IDX_VY, 1] = cos_d / self.m         # w_f → v_y (scaled by cos(δ))
        
        # Residuals affect r (yaw)
        E[IDX_R, 0] = -self.lr / self.Iz      # w_r → r (rear moment arm)
        E[IDX_R, 1] = self.lf * cos_d / self.Iz  # w_f → r (front moment arm)
        
        return E
    
    def compute_C_matrix(self, vx: float, delta: float) -> np.ndarray:
        """
        Compute output matrix C(ρ).
        
        y = [vx, r, psi, X, Y, ay]
        """
        vx_safe = max(abs(vx), self.min_vx)
        cos_d = np.cos(delta)
        
        C = np.zeros((6, 6))
        
        # Direct measurements
        C[MEAS_VX, IDX_VX] = 1.0      # vx → vx
        C[MEAS_R, IDX_R] = 1.0        # r → r  
        C[MEAS_PSI, IDX_PSI] = 1.0    # psi → psi
        C[MEAS_X, IDX_X] = 1.0        # X → X
        C[MEAS_Y, IDX_Y] = 1.0        # Y → Y
        
        # a_y = r·vx + ẏ (lateral acceleration)
        # ẏ involves tire forces, captured through C_ay
        C[MEAS_AY, IDX_VX] = 0  # Will add through F matrix
        C[MEAS_AY, IDX_VY] = -(self.Cf + self.Cr) / (self.m * vx_safe)
        C[MEAS_AY, IDX_R] = vx - (self.lf * self.Cf - self.lr * self.Cr) / (self.m * vx_safe)
        
        return C
    
    def compute_F_matrix(self, delta: float) -> np.ndarray:
        """
        Compute residual-to-measurement matrix F(ρ).
        
        How tire residuals [w_r, w_f] appear in measurements (especially a_y).
        """
        cos_d = np.cos(delta)
        
        F = np.zeros((6, 2))
        
        # a_y measurement includes tire residuals
        F[MEAS_AY, 0] = 1.0 / self.m           # w_r → a_y
        F[MEAS_AY, 1] = cos_d / self.m         # w_f → a_y
        
        return F
    
    def compute_augmented_matrices(self, vx: float, delta: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute augmented system matrices for EKF.
        
        Augmented state: x_a = [x; w] where w follows random-walk model (ẇ = 0).
        
        Returns:
            A_a: Augmented state matrix (8×8)
            B_a: Augmented input matrix (8×2)
            C_a: Augmented output matrix (6×8)
        """
        A = self.compute_A_matrix(vx, delta)
        B = self.compute_B_matrix(vx, delta)
        E = self.compute_E_matrix(delta)
        C = self.compute_C_matrix(vx, delta)
        F = self.compute_F_matrix(delta)
        
        # Augmented A: [A, E; 0, 0]
        A_a = np.zeros((8, 8))
        A_a[:6, :6] = A
        A_a[:6, 6:8] = E
        # ẇ = 0 (random walk for residuals)
        
        # Augmented B: [B; 0]
        B_a = np.zeros((8, 2))
        B_a[:6, :] = B
        
        # Augmented C: [C, F]
        C_a = np.zeros((6, 8))
        C_a[:, :6] = C
        C_a[:, 6:8] = F
        
        return A_a, B_a, C_a
    
    def discretize(self, A: np.ndarray, B: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Discretize continuous system using matrix exponential (ZOH).
        
        Args:
            A: Continuous state matrix
            B: Continuous input matrix
            dt: Sample time
            
        Returns:
            (A_d, B_d): Discrete state and input matrices
        """
        n = A.shape[0]
        m = B.shape[1]
        
        # Van Loan method for ZOH discretization
        M = np.zeros((n + m, n + m))
        M[:n, :n] = A
        M[:n, n:] = B
        
        M_exp = expm(M * dt)
        
        A_d = M_exp[:n, :n]
        B_d = M_exp[:n, n:]
        
        return A_d, B_d
    
    # =========================================================
    # EKF PREDICT & UPDATE
    # =========================================================
    
    def ekf_predict(self, x_a: np.ndarray, P: np.ndarray, 
                    u: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        EKF Prediction Step.
        
        Args:
            x_a: Current augmented state [x; w]
            P: Current error covariance
            u: Control input [delta, accel]
            dt: Sample time
            
        Returns:
            (x_pred, P_pred): Predicted state and covariance
        """
        vx = x_a[IDX_VX]
        delta = u[0]
        
        # Get continuous augmented matrices
        A_a, B_a, _ = self.compute_augmented_matrices(vx, delta)
        
        # Discretize
        A_d, B_d = self.discretize(A_a, B_a, dt)
        
        # State prediction
        x_pred = A_d @ x_a + B_d @ u
        
        # Covariance prediction
        Q_k = self.Q * dt
        P_pred = A_d @ P @ A_d.T + Q_k
        
        return x_pred, P_pred
    
    def ekf_update(self, x_pred: np.ndarray, P_pred: np.ndarray,
                   y: np.ndarray, u: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        EKF Update Step.
        
        Args:
            x_pred: Predicted augmented state
            P_pred: Predicted covariance
            y: Measurement vector [vx, r, psi, X, Y, ay]
            u: Control input [delta, accel]
            
        Returns:
            (x_upd, P_upd, innov): Updated state, covariance, and innovation
        """
        vx = x_pred[IDX_VX]
        delta = u[0]
        
        # Get augmented output matrix
        _, _, C_a = self.compute_augmented_matrices(vx, delta)
        
        # Predicted measurement
        y_pred = C_a @ x_pred
        
        # Innovation (with heading angle wrapping)
        innov = y - y_pred
        innov[MEAS_PSI] = np.arctan2(np.sin(innov[MEAS_PSI]), np.cos(innov[MEAS_PSI]))
        
        # Innovation covariance
        S = C_a @ P_pred @ C_a.T + self.R
        
        # Kalman gain
        try:
            K = P_pred @ C_a.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            K = P_pred @ C_a.T @ np.linalg.pinv(S)
        
        # State update
        x_upd = x_pred + K @ innov
        
        # Covariance update (Joseph form for stability)
        I = np.eye(self.augmented_dim)
        IKC = I - K @ C_a
        P_upd = IKC @ P_pred @ IKC.T + K @ self.R @ K.T
        P_upd = 0.5 * (P_upd + P_upd.T)  # Enforce symmetry
        
        return x_upd, P_upd, innov
    
    # =========================================================
    # MAIN UPDATE
    # =========================================================
    
    def update(self, measurement: np.ndarray, control: np.ndarray, 
               dt: Optional[float] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Main update step - call this in your control loop.
        
        Args:
            measurement: [vx, r, psi, X, Y, ay] 
            control: [delta, accel] or [steering_cmd, throttle_cmd]
            dt: Sample time (uses self.Ts if None)
            
        Returns:
            (state_estimate, tire_residuals)
            - state_estimate: [vx, vy, psi, r, X, Y]
            - tire_residuals: [w_r, w_f]
        """
        dt = dt if dt is not None else self.Ts
        y = np.array(measurement).flatten()
        u = np.array(control).flatten()
        
        # Store steering for scheduling
        self.current_delta = u[0]
        
        # =====================================================
        # STILL CONDITION CHECK
        # =====================================================
        vx_meas = y[MEAS_VX]
        is_still = abs(vx_meas) < 0.1
        
        if is_still:
            # Vehicle is stationary - use kinematic fallback
            x_upd = self.x_aug.copy()
            x_upd[IDX_VX] = vx_meas
            x_upd[IDX_VY] = 0.0
            x_upd[IDX_R] = y[MEAS_R]
            x_upd[IDX_PSI] = y[MEAS_PSI]
            x_upd[IDX_X] = y[MEAS_X]
            x_upd[IDX_Y] = y[MEAS_Y]
            x_upd[6:] = 0.0  # Zero residuals when stationary
            
            P_upd = self.P.copy()
            
        else:
            # =====================================================
            # FULL EKF UPDATE
            # =====================================================
            
            # 1. Predict
            x_pred, P_pred = self.ekf_predict(self.x_aug, self.P, u, dt)
            
            # 2. Update
            x_upd, P_upd, _ = self.ekf_update(x_pred, P_pred, y, u)
        
        # =====================================================
        # STORE RESULTS
        # =====================================================
        self.x_aug = x_upd
        self.P = P_upd
        
        # Wrap heading to [-π, π]
        self.x_aug[IDX_PSI] = np.arctan2(np.sin(self.x_aug[IDX_PSI]), 
                                          np.cos(self.x_aug[IDX_PSI]))
        
        # Extract state and residuals
        self.x_hat = self.x_aug[:6].copy()
        self.w_hat = self.x_aug[6:].copy()
        
        # =====================================================
        # CALCULATE TIRE INFO
        # =====================================================
        self._update_tire_info()
        
        return self.x_hat.copy(), self.w_hat.copy()
    
    def _update_tire_info(self):
        """Calculate and store detailed tire information."""
        vx = self.x_hat[IDX_VX]
        vy = self.x_hat[IDX_VY]
        r = self.x_hat[IDX_R]
        delta = self.current_delta
        
        w_r = self.w_hat[0]
        w_f = self.w_hat[1]
        
        # Compute slip angles
        alpha_f, alpha_r = self.compute_slip_angles(vx, vy, r, delta)
        
        # Compute tire forces
        Fyf_linear = self.Cf * alpha_f
        Fyr_linear = self.Cr * alpha_r
        
        Fyf_total = Fyf_linear + w_f
        Fyr_total = Fyr_linear + w_r
        
        # Store
        self.tire_info = {
            'Fyf_linear': Fyf_linear,
            'Fyr_linear': Fyr_linear,
            'Fyf_total': Fyf_total,
            'Fyr_total': Fyr_total,
            'alpha_f': alpha_f,
            'alpha_r': alpha_r,
            'w_f': w_f,
            'w_r': w_r,
        }
    
    # =========================================================
    # GETTERS
    # =========================================================
    
    def get_state(self) -> np.ndarray:
        """Get current state estimate [vx, vy, psi, r, X, Y]"""
        return self.x_hat.copy()
    
    def get_tire_residuals(self) -> np.ndarray:
        """Get tire residual estimates [w_r, w_f]"""
        return self.w_hat.copy()
    
    def get_tire_info(self) -> Dict:
        """
        Get complete tire information.
        
        Returns dict with:
            - Fyf_linear: Front force from linear model
            - Fyr_linear: Rear force from linear model
            - Fyf_total: Front force with residual
            - Fyr_total: Rear force with residual
            - alpha_f: Front slip angle
            - alpha_r: Rear slip angle
            - w_f: Front tire residual
            - w_r: Rear tire residual
        """
        return self.tire_info.copy()
    
    def get_covariance(self) -> np.ndarray:
        """Get current error covariance P (8×8)"""
        return self.P.copy()
    
    def reset(self, initial_state: Optional[np.ndarray] = None):
        """
        Reset estimator to initial conditions.
        
        Args:
            initial_state: Optional initial state [vx, vy, psi, r, X, Y]
        """
        self.x_aug = np.zeros(self.augmented_dim)
        if initial_state is not None:
            self.x_aug[:min(len(initial_state), 6)] = initial_state[:6]
        
        self.x_hat = self.x_aug[:6].copy()
        self.w_hat = np.zeros(self.residual_dim)
        self.P = self._default_P0()
        self.current_delta = 0.0
        
        self._update_tire_info()


# =============================================================================
# FACTORY FUNCTION
# =============================================================================

def create_tire_estimator(sample_time: float = 0.02,
                          vehicle_params: Optional[Dict] = None) -> TireEstimatorSimple:
    """
    Create a tire estimator instance.
    
    Args:
        sample_time: Sample time [s]
        vehicle_params: Vehicle parameters dict
        
    Returns:
        Configured TireEstimatorSimple instance
    """
    return TireEstimatorSimple(sample_time=sample_time, 
                                vehicle_params=vehicle_params)


# =============================================================================
# EXAMPLE USAGE
# =============================================================================

if __name__ == "__main__":
    """Example demonstrating tire estimator usage."""
    
    # Create estimator
    estimator = TireEstimatorSimple(sample_time=0.02)
    
    # Initialize with known state
    estimator.reset(initial_state=np.array([1.0, 0, 0, 0, 0, 0]))
    
    # Simulate a few steps
    print("=== Tire Estimator Demo ===\n")
    
    for i in range(5):
        # Simulated measurements
        measurement = [1.0 + 0.1*i, 0.1, 0.05*i, 0.5*i, 0.1*i, 0.5]
        control = [0.1, 0.5]  # Slight steering, moderate acceleration
        
        # Update estimator
        state, residuals = estimator.update(measurement, control)
        tire_info = estimator.get_tire_info()
        
        print(f"Step {i+1}:")
        print(f"  State: vx={state[0]:.2f}, vy={state[1]:.3f}, r={state[3]:.3f}")
        print(f"  Residuals: w_r={residuals[0]:.3f}, w_f={residuals[1]:.3f}")
        print(f"  Slip angles: αf={tire_info['alpha_f']:.3f}, αr={tire_info['alpha_r']:.3f}")
        print(f"  Forces: Fyf={tire_info['Fyf_total']:.2f}N, Fyr={tire_info['Fyr_total']:.2f}N")
        print()
