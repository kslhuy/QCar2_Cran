"""
Gradient Solver for Neural Observer

Computes sensitivities and gradients for backpropagation through observer dynamics.
Implements the chain rule to propagate gradients from loss functions back to neural
network parameters.

Supports two gradient computation methods:
    1. Analytical: Manual sensitivity propagation (dx/df[k+1] = A_d·dx/df[k] + E_d)
    2. Autodiff: PyTorch automatic differentiation through observer dynamics
Available Loss Functions:
    - measurement_full: Standard measurement tracking ||x̂ - y||²_W
    - composite_uio: Two-layer UIO loss with trajectory reference
    - physics_tire: Physics-informed tire residual loss
      Adds direct IMU lateral acceleration constraint on (wr + wf·cos(δ)),
      temporal smoothness, and soft physical bounds for faster convergence.
    - prediction_error: Self-supervised 1-step prediction error (RECOMMENDED)
      No need for noisy physics target w*. Trains NN to minimize the observer's
      next-step measurement prediction error. The NN learns whatever correction
      makes the observer predict better.
Key Equations (Analytical):
    Sensitivity: dx/df[k+1] = (I + Ts·(A - L·C)) · dx/df[k] + Ts·E
    Chain Rule: dL/df = dL/dx · dx/df
"""

import numpy as np
from typing import Tuple, Optional, Callable

# Import PyTorch for autodiff (optional)
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


class GradientSolver:
    """
    Gradient solver for neural observer learning
    
    Computes sensitivities through observer dynamics and applies chain rule
    for gradient-based learning.
    """

    def __init__(self,sample_time: float, observer_matrices: dict):
        """
        Initialize gradient solver
        
        Args:
            observer_matrices: Dictionary containing:
                - 'A': System matrix (continuous or discrete)
                - 'C': Output matrix
                - 'D': Disturbance input matrix
                - 'K': Observer gain matrix
            sample_time: Sample time (Ts)
        """
        self.A = observer_matrices.get('A')
        self.C = observer_matrices.get('C')
        self.D = observer_matrices.get('D')
        self.Ts = sample_time
        
        # State dimension
        self.state_dim = self.A.shape[0] if self.A is not None else 4
        
        # Output dimension (unknown input dimension)
        # Try to infer from D matrix if available, otherwise default to 2
        if self.D is not None:
            self.output_dim = self.D.shape[1]
        else:
            self.output_dim = 2  # Default (tire force residuals)
        
        # Loss scaling factor
        self.K_loss = 1.0
    
    def _ensure_col_vector(self, v: np.ndarray) -> np.ndarray:
        """Ensure v is a column vector (n, 1) for consistent shape handling."""
        v = np.atleast_1d(np.squeeze(v))
        return v.reshape(-1, 1)
    
    def update_matrices(self, observer_matrices: dict):
        """
        Update observer matrices (for time-varying systems)
        
        Args:
            observer_matrices: Dictionary with updated matrices
        """
        if 'A' in observer_matrices:
            self.A = observer_matrices['A']
        if 'C' in observer_matrices:
            self.C = observer_matrices['C']
        if 'D' in observer_matrices:
            self.D = observer_matrices['D']

    

    
    def gradient_solver_discrete_lpv(self, sensitivity: np.ndarray,
                                      A: np.ndarray, L: np.ndarray, 
                                      E: np.ndarray) -> np.ndarray:
        """
        Update sensitivity for discrete-time LPV observer with time-varying matrices.
        
        For discretized Luenberger observer:
            x̂[k+1] = x̂[k] + Ts·(A(ρ)·x̂ + B·u + E(ρ)·f_nn + L·(y - C·x̂))
        
        The sensitivity equation becomes:
            dx/df[k+1] = (I + Ts·(A(ρ) - L·C)) · dx/df[k] + Ts·E(ρ)
        
        where A(ρ), E(ρ), and optionally L(ρ) vary with scheduling parameters ρ.
        
        Args:
            sensitivity: Current sensitivity matrix dx/df (state_dim × output_dim)
            A: Current state matrix A(ρ) (state_dim × state_dim)
            L: Current observer gain L(ρ) (state_dim × meas_dim)
            E: Current disturbance input matrix E(ρ) (state_dim × output_dim)
        
        Returns:
            Updated sensitivity matrix dx/df[k+1]
        """
        n = A.shape[0]
        
        # Discrete closed-loop dynamics: A_d = I + Ts·(A - L·C)
        A_cl_continuous = A - L @ self.C
        A_d = np.eye(n) + self.Ts * A_cl_continuous
        
        # Discrete input matrix: E_d = Ts·E
        E_d = self.Ts * E
        
        # LPV sensitivity update
        sensitivity_new = A_d @ sensitivity + E_d
        
        return sensitivity_new
    
    def gradient_solver_luenberger(self, sensitivity: np.ndarray,
                                   A: np.ndarray, L: np.ndarray, 
                                   E: np.ndarray, C: np.ndarray,
                                   dt: float, gps_valid: bool) -> np.ndarray:
        """
        Sensitivity for Luenberger observer with predict-correct structure.
        
        Observer dynamics:
            x_pred = x + dt*(A*x + B*u + E*w)      # Prediction
            x_new = x_pred + L*(y - C*x_pred)       # Correction (if GPS valid)
        
        Which expands to:
            x_new = (I + dt*A - L*C)*x + dt*B*u + dt*E*w + L*y   (when GPS valid)
            x_new = (I + dt*A)*x + dt*B*u + dt*E*w               (prediction only)
        
        Sensitivity equations:
            When GPS valid:  dx/df[k+1] = (I + dt*A - L*C) * dx/df[k] + dt*E
            When GPS invalid: dx/df[k+1] = (I + dt*A) * dx/df[k] + dt*E
        
        Args:
            sensitivity: Current sensitivity matrix dx/df (state_dim × output_dim)
            A: Current state matrix A(ρ) (state_dim × state_dim)
            L: Current observer gain L(ρ) (state_dim × meas_dim)
            E: Current disturbance input matrix E(ρ) (state_dim × output_dim)
            C: Output matrix (meas_dim × state_dim)
            dt: Sample time
            gps_valid: Whether GPS measurement is valid (correction applied)
        
        Returns:
            Updated sensitivity matrix dx/df[k+1]
        """
        n = A.shape[0]
        I = np.eye(n)
        
        if gps_valid:
            # Correction applied: A_cl = I + dt*A - L*C
            A_cl = I + dt * A - L @ C
        else:
            # Prediction only: A_cl = I + dt*A
            A_cl = I + dt * A
        
        E_d = dt * E
        return A_cl @ sensitivity + E_d
    
    def gradient_solver_luenberger_discrete(self, sensitivity: np.ndarray,
                                            A_d: np.ndarray, L: np.ndarray, 
                                            E_d: np.ndarray, C: np.ndarray,
                                            gps_valid: bool) -> np.ndarray:
        """
        Sensitivity for discrete-time Luenberger observer (ZOH discretized).
        
        This method uses pre-discretized matrices (A_d, E_d) from ZOH discretization,
        ensuring consistency with the observer update step which uses:
            x̂[k+1] = A_d·x̂[k] + B_d·u[k] + E_d·w[k] + L·(y[k] - C·x̂[k])
        
        Rearranging the observer equation:
            x̂[k+1] = (A_d - L·C)·x̂[k] + B_d·u[k] + E_d·w[k] + L·y[k]
        
        The closed-loop dynamics matrix is: A_cl = A_d - L·C
        
        Differentiating w.r.t. unknown input w (neural network output f_nn):
            ∂x̂[k+1]/∂f = (A_d - L·C) · ∂x̂[k]/∂f + E_d
        
        When GPS is invalid, no correction is applied (prediction only):
            x̂[k+1] = A_d·x̂[k] + B_d·u[k] + E_d·w[k]
            ∂x̂[k+1]/∂f = A_d · ∂x̂[k]/∂f + E_d
        
        Args:
            sensitivity: Current sensitivity matrix dx/df (state_dim × output_dim)
            A_d: Discrete-time state matrix A_d from ZOH (state_dim × state_dim)
            L: Observer gain L (state_dim × meas_dim)
            E_d: Discrete-time residual input matrix E_d from ZOH (state_dim × output_dim)
            C: Output matrix (meas_dim × state_dim)
            gps_valid: Whether GPS measurement is valid (correction applied)
        
        Returns:
            Updated sensitivity matrix dx/df[k+1]
        """
        if gps_valid:
            # Correction applied: A_cl = A_d - L·C
            A_cl = A_d - L @ C
        else:
            # Prediction only: A_cl = A_d
            A_cl = A_d
        
        return A_cl @ sensitivity + E_d
    
    def chain_rule_reference_tracking(self,
                                     reference: np.ndarray,
                                     state_hat: np.ndarray,
                                     dx_df: np.ndarray,
                                     f_hat: np.ndarray,
                                     f_nn: np.ndarray,
                                     weight_matrix: np.ndarray,
                                     lambda_reg: float = 0.0) -> Tuple[np.ndarray, float]:
        """
        Chain rule for reference tracking loss
        
        Loss: L = (x̂ - x_ref)ᵀ W (x̂ - x_ref) + λ ||f_nn - f̂||²
        
        Args:
            reference: Reference state (state_dim × 1)
            state_hat: Estimated state (state_dim × 1)
            dx_df: Sensitivity matrix (state_dim × output_dim)
            f_hat: Estimated force from observer (output_dim × 1)
            f_nn: Neural network output (output_dim × 1)
            weight_matrix: Weight matrix for state error (state_dim × state_dim)
            lambda_reg: Regularization weight for force error
        
        Returns:
            Tuple of (gradient dL/df, loss value)
        """
        # Reshape inputs
        state_hat = state_hat.reshape(-1, 1)
        reference = reference.reshape(-1, 1)
        
        # State error
        state_error = state_hat - reference
        
        # Loss 1: State tracking error
        loss1 = self.K_loss * ((state_error.T @ weight_matrix) @ state_error)[0, 0]
        
        # Loss 2: Force regularization (squared norm: λ||f_nn - f̂||²)
        f_nn_col = self._ensure_col_vector(f_nn)
        f_hat_col = self._ensure_col_vector(f_hat)
        f_error = f_nn_col - f_hat_col
        loss2 = lambda_reg * np.sum(f_error ** 2)
        
        # Total loss
        loss_total = loss1 + loss2
        
        # Gradient computation
        # dL/dx = W · (x̂ - x_ref)
        dL_dx = weight_matrix @ state_error
        
        # dL/df = dL/dx · dx/df
        dL_df_1 = dL_dx.T @ dx_df
        
        # dL/df from force regularization: d/df(λ||f||²) = 2λf
        dL_df_2 = 2 * lambda_reg * f_error.T
        
        # Total gradient
        dL_df = dL_df_1 + dL_df_2
        
        return dL_df, loss_total
    
    def chain_rule_measurement_tracking(self,
                                       measurement: np.ndarray,
                                       state_hat: np.ndarray,
                                       dx_df: np.ndarray,
                                       f_hat: np.ndarray,
                                       f_nn: np.ndarray,
                                       weight_matrix: np.ndarray,
                                       lambda_reg: float = 0.0) -> Tuple[np.ndarray, float]:
        """
        Chain rule for measurement tracking loss
        
        Loss: L = (ŷ - y)ᵀ W (ŷ - y) + λ ||f_nn - f̂||²
        where ŷ = C · x̂
        
        Args:
            measurement: Measurement vector (measurement_dim × 1)
            state_hat: Estimated state (state_dim × 1)
            dx_df: Sensitivity matrix (state_dim × output_dim)
            f_hat: Estimated force from observer (output_dim × 1)
            f_nn: Neural network output (output_dim × 1)
            weight_matrix: Weight matrix for measurement error
            lambda_reg: Regularization weight for force error
        
        Returns:
            Tuple of (gradient dL/df, loss value)
        """
        # Reshape inputs
        state_hat = state_hat.reshape(-1, 1)
        measurement = measurement.reshape(-1, 1)
        
        # Predicted measurement
        y_hat = self.C @ state_hat
        
        # Measurement error
        y_error = y_hat - measurement
        
        # Loss 1: Measurement tracking error
        loss1 = self.K_loss * ((y_error.T @ weight_matrix) @ y_error)[0, 0]
        
        # Loss 2: Force regularization (squared norm: λ||f_nn - f̂||²)
        f_nn_col = self._ensure_col_vector(f_nn)
        f_hat_col = self._ensure_col_vector(f_hat)
        f_error = f_nn_col - f_hat_col
        loss2 = lambda_reg * np.sum(f_error ** 2)
        
        # Total loss
        loss_total = loss1 + loss2
        
        # Gradient computation
        # dL/dy = W · (ŷ - y)
        dL_dy = weight_matrix @ y_error
        
        # dy/dx = C
        dy_dx = self.C
        
        # dL/dx = (dy/dx)ᵀ · dL/dy = Cᵀ · dL/dy
        dL_dx = dy_dx.T @ dL_dy
        
        # dL/df = dL/dx · dx/df
        dL_df_1 = dL_dx.T @ dx_df
        
        # dL/df from force regularization: d/df(λ||f||²) = 2λf
        dL_df_2 = 2 * lambda_reg * f_error.T
        
        # Total gradient
        dL_df = dL_df_1 + dL_df_2
        
        return dL_df, loss_total
    
    def chain_rule_full_measurement(self,
                                   measurement: np.ndarray,
                                   state_hat: np.ndarray,
                                   dx_df: np.ndarray,
                                   f_uk: np.ndarray,
                                   f_nn: np.ndarray,
                                   weight_matrix: np.ndarray,
                                   lambda_reg: float = 0.0) -> Tuple[np.ndarray, float]:
        """
        Chain rule for full state measurement tracking
        
        This version uses the true unknown force f_uk instead of f_hat
        for more accurate gradient computation.
        
        Loss: L = (x̂ - x)ᵀ W (x̂ - x) + λ ||f_nn - f_uk||²
        
        Args:
            measurement: Full state measurement (state_dim × 1)
            state_hat: Estimated state (state_dim × 1)
            dx_df: Sensitivity matrix (state_dim × output_dim)
            f_uk: True unknown force (output_dim × 1)
            f_nn: Neural network output (output_dim × 1)
            weight_matrix: Weight matrix for state error
            lambda_reg: Regularization weight for force error
        
        Returns:
            Tuple of (gradient dL/df, loss value)
        """
        # Reshape inputs
        state_hat = state_hat.reshape(-1, 1)
        measurement = measurement.reshape(-1, 1)
        
        # State error (direct comparison)
        state_error = state_hat - measurement
        
        # Loss 1: State tracking error
        loss1 = self.K_loss * ((state_error.T @ weight_matrix) @ state_error)[0, 0]
        
        # Loss 2: Force error (squared norm: λ||f_nn - f_uk||²)
        f_nn_col = self._ensure_col_vector(f_nn)
        f_uk_col = self._ensure_col_vector(f_uk)
        f_error = f_nn_col - f_uk_col
        loss2 = lambda_reg * np.sum(f_error ** 2)
        
        # Total loss
        loss_total = loss1 + loss2
        
        # Gradient computation
        # dL/dx = K_loss · W · (x̂ - x)
        dL_dy = self.K_loss * (weight_matrix @ state_error)
        
        # dL/df = dL/dx · dx/df
        dL_df_1 = dL_dy.T @ dx_df
        
        # dL/df from force error: d/df(λ||f||²) = 2λf
        dL_df_2 = 2 * lambda_reg * f_error.T
        
        # Total gradient
        dL_df = dL_df_1 + dL_df_2
        
        return dL_df, loss_total


    def chain_rule_composite_uio(self,
                                 reference: np.ndarray,
                                 measurement: np.ndarray,
                                 state_hat_nn: np.ndarray,
                                 state_hat_uio: np.ndarray,
                                 dx_df: np.ndarray,
                                 f_uk_uio: np.ndarray,
                                 f_nn: np.ndarray,
                                 weight_matrices: dict,
                                 lambda_reg: float = 0.0,
                                 ref_indices: Optional[np.ndarray] = None) -> Tuple[np.ndarray, float]:
        """
        Composite loss function for two-layer observer architecture with UIO
        
        Supports partial reference: only compares available reference states.
        Use ref_indices to specify which indices in x_nn correspond to x_ref.
        
        Loss: L = (x̂_nn[ref_indices] - x_ref)ᵀ T_ref (x̂_nn[ref_indices] - x_ref)  [Tracking]
                + (y - y_nn)ᵀ T_y (y - y_nn)                                        [Output Error]
                + (x̂_nn - x̂_uio)ᵀ T_uio (x̂_nn - x̂_uio)                           [UIO Consistency]
                + λ ||f_nn - f̂_uk||²                                                [Regularization]
        
        Args:
            reference: Reference trajectory x_ref (n_ref × 1), can be smaller than state_dim
            measurement: Actual measurement y (measurement_dim × 1)
            state_hat_nn: Neural observer state estimate x̂_nn (state_dim × 1)
            state_hat_uio: UIO state estimate x̂_uio (state_dim × 1)
            dx_df: Sensitivity matrix dx/df (state_dim × output_dim)
            f_uk_uio: UIO unknown input estimate f̂_uk (output_dim × 1)
            f_nn: Neural network output (output_dim × 1)
            weight_matrices: Dictionary with keys:
                - 'T_ref': Weight matrix for tracking error (n_ref × n_ref)
                - 'T_y': Weight matrix for output error
                - 'T_uio': Weight matrix for UIO consistency  
            lambda_reg: Regularization weight λ
            ref_indices: Indices in x_nn that correspond to x_ref.
                        Example: [4, 5, 2] means x_ref = [X, Y, ψ] maps to
                                 x_nn[4]=X, x_nn[5]=Y, x_nn[2]=ψ
                        If None, assumes first n elements of x_nn match x_ref.
        
        Returns:
            Tuple of (gradient dL/df, total loss value)
        """
        # Reshape inputs
        state_hat_nn = state_hat_nn.reshape(-1, 1)
        state_hat_uio = state_hat_uio.reshape(-1, 1)
        reference = reference.reshape(-1, 1)
        measurement = measurement.reshape(-1, 1)
        
        n_ref = reference.shape[0]
        
        # Determine which indices in x_nn correspond to x_ref
        if ref_indices is None:
            # Default: assume first n_ref elements of x_nn
            ref_indices = np.arange(n_ref)
        else:
            ref_indices = np.asarray(ref_indices)
        
        # Extract weight matrices
        T_ref = weight_matrices.get('T_ref', np.eye(n_ref))
        T_y = weight_matrices.get('T_y', np.eye(measurement.shape[0]))
        T_uio = weight_matrices.get('T_uio', np.eye(self.state_dim))
        
        # Ensure T_ref has correct dimensions
        if T_ref.shape[0] != n_ref:
            T_ref = np.eye(n_ref)  # Fallback to identity
        
        # ========== Term 1: Tracking Error (Reduced Dimension) ==========
        # Extract only the states that have reference values
        x_nn_reduced = state_hat_nn[ref_indices]  # (n_ref, 1)
        
        tracking_error = x_nn_reduced - reference
        loss_tracking = self.K_loss * ((tracking_error.T @ T_ref) @ tracking_error)[0, 0]
        
        # Gradient: dL_tracking/dx has sparse structure
        # dL/dx_reduced = T_ref · (x̂_nn_reduced - x_ref)
        dL_tracking_dx_reduced = T_ref @ tracking_error
        
        # Map back to full state space (state_dim × 1)
        dL_tracking_dx = np.zeros((self.state_dim, 1))
        for i, idx in enumerate(ref_indices):
            dL_tracking_dx[idx, 0] = dL_tracking_dx_reduced[i, 0]
        
        # ========== Term 2: Output Error ==========
        # (y - y_nn)ᵀ T_y (y - y_nn) where y_nn = C·x̂_nn
        y_nn = self.C @ state_hat_nn
        output_error = measurement - y_nn
        loss_output = self.K_loss * ((output_error.T @ T_y) @ output_error)[0, 0]
        
        # Gradient: dL_output/dx = -Cᵀ · T_y · (y - y_nn)
        dL_output_dx = -self.C.T @ T_y @ output_error
        
        # ========== Term 3: UIO Consistency ==========
        # (x̂_nn - x̂_uio)ᵀ T_uio (x̂_nn - x̂_uio)
        uio_error = state_hat_nn - state_hat_uio
        loss_uio = self.K_loss * ((uio_error.T @ T_uio) @ uio_error)[0, 0]
        
        # Gradient: dL_uio/dx = T_uio · (x̂_nn - x̂_uio)
        dL_uio_dx = T_uio @ uio_error
        
        # ========== Term 4: Regularization ==========
        # λ ||f_nn - f̂_uk||²
        f_nn_col = self._ensure_col_vector(f_nn)
        f_uk_col = self._ensure_col_vector(f_uk_uio)
        f_error = f_nn_col - f_uk_col
        loss_reg = lambda_reg * np.sum(f_error ** 2)
        
        # ========== Total Loss ==========
        loss_total = loss_tracking + loss_output + loss_uio + loss_reg
        
        # ========== Total Gradient ==========
        # Combine all gradient terms
        dL_dx_total = dL_tracking_dx + dL_output_dx + dL_uio_dx
        
        # Chain rule: dL/df = dL/dx · dx/df
        dL_df_state = dL_dx_total.T @ dx_df
        
        # Regularization gradient: dL_reg/df = 2λ(f_nn - f̂_uk)
        dL_df_reg = 2 * lambda_reg * f_error
        
        # Total gradient
        dL_df = dL_df_state + dL_df_reg
        
        return dL_df, loss_total

    def _solve_physics_target(self,
                              vehicle_params: dict,
                              steering: float,
                              ay_meas: float,
                              r_dot_meas: float,
                              alpha_f: float,
                              alpha_r: float) -> np.ndarray:
        """
        Solve the 2-equation system for unique tire residual target (w_r*, w_f*).
        
        Equations:
            Lateral force:  w_r + cos(δ)·w_f = m·a_y - Fyr_lin - Fyf_lin·cos(δ)
            Yaw moment:    -lr·w_r + lf·cos(δ)·w_f = Iz·ṙ + lr·Fyr_lin - lf·Fyf_lin·cos(δ)
        
        IMPORTANT: r_dot from finite difference is very noisy.
        We use a confidence-weighted formulation that reduces r_dot influence
        when it's likely unreliable (high magnitude = likely noise).
        
        Args:
            vehicle_params: Vehicle parameter dict
            steering: Steering angle δ [rad]
            ay_meas: Measured lateral acceleration [m/s²]
            r_dot_meas: Measured yaw acceleration [rad/s²]
            alpha_f: Front slip angle [rad]
            alpha_r: Rear slip angle [rad]
            
        Returns:
            w_star: Solved target [w_r*, w_f*] as (2,1) column vector
        """
        m = vehicle_params['m']
        Iz = vehicle_params['Iz']
        lf = vehicle_params['lf']
        lr = vehicle_params['lr']
        Cf = vehicle_params['Cf']
        Cr = vehicle_params['Cr']
        
        cos_d = np.cos(steering)
        
        Fyf_lin = Cf * alpha_f
        Fyr_lin = Cr * alpha_r
        
        # ========== Confidence weighting for noisy r_dot ==========
        # r_dot from finite difference is very noisy at 100 Hz.
        # We reduce its weight when |r_dot| is suspiciously large.
        # Typical real r_dot for QCar2: < 5 rad/s² during normal driving.
        r_dot_threshold = 5.0  # rad/s², above this is likely noise
        r_dot_confidence = np.exp(-0.5 * (r_dot_meas / r_dot_threshold) ** 2)
        # confidence: 1.0 at r_dot=0, ~0.6 at threshold, ~0.14 at 2*threshold
        
        # Weighted system: row 1 has weight 1.0 (a_y is reliable IMU),
        #                  row 2 has weight r_dot_confidence
        
        # System matrix
        A_sys = np.array([
            [1.0,   cos_d],
            [-lr * r_dot_confidence,   lf * cos_d * r_dot_confidence]
        ])
        
        # Right-hand side
        b_sys = np.array([
            [m * ay_meas - Fyr_lin - Fyf_lin * cos_d],
            [r_dot_confidence * (Iz * r_dot_meas + lr * Fyr_lin - lf * Fyf_lin * cos_d)]
        ])
        
        # Determinant = r_dot_confidence * cos(δ)·(lf + lr)
        det_A = r_dot_confidence * cos_d * (lf + lr)
        
        if abs(det_A) < 1e-6:
            # Near-singular (r_dot unreliable or δ≈π/2): use lateral-only with regularization
            # Fall back to rank-1 constraint: w_r + cos(δ)·w_f = b_lat
            # Add small regularization: minimize ||w||²
            b_lat = m * ay_meas - Fyr_lin - Fyf_lin * cos_d
            # Minimum norm solution of [1, cos_d] w = b_lat
            a_vec = np.array([[1.0], [cos_d]])
            a_norm_sq = 1.0 + cos_d ** 2
            w_star = a_vec * (b_lat / max(a_norm_sq, 1e-8))
        else:
            w_star = np.linalg.solve(A_sys, b_sys)
        
        return w_star
    
    def chain_rule_physics_informed_tire(self,
                                        measurement: np.ndarray,
                                        state_hat_nn: np.ndarray,
                                        state_hat_uio: np.ndarray,
                                        dx_df: np.ndarray,
                                        f_nn: np.ndarray,
                                        f_nn_prev: np.ndarray,
                                        vehicle_params: dict,
                                        steering: float,
                                        ay_meas: float,
                                        weight_matrices: dict,
                                        ref_indices: Optional[np.ndarray] = None,
                                        reference: Optional[np.ndarray] = None,
                                        r_dot_meas: float = 0.0,
                                        w_hat_L1: Optional[np.ndarray] = None,
                                        step_count: int = 0,
                                        ) -> Tuple[np.ndarray, float]:
        """
        Physics-Informed Tire Residual Loss V2 — Solved 2D Target.
        
        Key improvement over V1: Instead of a 1D lateral-acceleration-only
        constraint (rank-deficient for 2 unknowns), this version solves the
        2-equation system [lateral force + yaw moment] for a unique target
        (w_r*, w_f*), providing full 2D supervision.
        
        Equations used:
            Lateral:  m·a_y = (Cr·αr + w_r) + (Cf·αf + w_f)·cos(δ)
            Yaw:      Iz·ṙ  = -lr·(Cr·αr + w_r) + lf·(Cf·αf + w_f)·cos(δ)
        
        Loss = L_state + L_physics + L_smooth + L_bound + L_warmstart [+ L_ref]
        
        Terms:
            L_state     = (x̂_nn - x̂_uio)ᵀ T_uio (x̂_nn - x̂_uio)
            L_physics   = w_phys · ||f_nn - w*||²   (2D solved target)
            L_smooth    = w_smooth · ||f_nn(k) - f_nn(k-1)||²
            L_bound     = λ_bound · Σ max(0, |f_i| - f_max)²
            L_warmstart = λ_warm · exp(-step/τ) · ||f_nn - w_L1||²
            L_ref       = optional trajectory tracking
        
        Args:
            measurement: Full measurement [vx, vy, ψ, r, X, Y]ᵀ (6×1)
            state_hat_nn: Neural observer state estimate (6×1)
            state_hat_uio: First-layer observer state estimate (6×1)
            dx_df: Sensitivity matrix ∂x/∂f (6×2)
            f_nn: Current NN output [wr, wf]ᵀ (2×1)
            f_nn_prev: Previous step NN output [wr, wf]ᵀ (2×1)
            vehicle_params: Dict with keys: m, Iz, lf, lr, Cf, Cr, vx_min
            steering: Current steering angle δ [rad]
            ay_meas: Measured lateral acceleration from IMU [m/s²]
            weight_matrices: Dict with keys:
                - 'T_uio': State consistency weight (6×6)
                - 'w_physics_target': 2D physics target weight (scalar)
                - 'w_ay': Legacy 1D constraint weight (scalar, default 0)
                - 'w_smooth': Temporal smoothness weight (scalar)
                - 'lambda_bound': Soft bound weight (scalar)
                - 'f_max': Maximum expected residual magnitude [N]
                - 'lambda_warmstart': Warm start weight (scalar)
                - 'warmstart_decay': Decay time constant in steps (int)
                - 'T_ref': Reference tracking weight, optional
            ref_indices: Indices mapping reference to 6D state (optional)
            reference: Reference trajectory (optional)
            r_dot_meas: Measured yaw acceleration [rad/s²] (finite difference)
            w_hat_L1: Layer 1 residual estimate [w_r, w_f] for warm start
            step_count: Current training step (for warm start decay)
            
        Returns:
            Tuple of (gradient dL/df, total loss value)
        """
        # Reshape inputs
        state_hat_nn = state_hat_nn.reshape(-1, 1)
        state_hat_uio = state_hat_uio.reshape(-1, 1)
        f_nn_col = self._ensure_col_vector(f_nn)
        f_nn_prev_col = self._ensure_col_vector(f_nn_prev)
        
        # Extract vehicle parameters
        m = vehicle_params['m']
        Iz = vehicle_params['Iz']
        lf = vehicle_params['lf']
        lr = vehicle_params['lr']
        Cf = vehicle_params['Cf']
        Cr = vehicle_params['Cr']
        vx_min = vehicle_params.get('vx_min', 0.3)
        
        # Extract weight parameters
        T_uio = weight_matrices.get('T_uio', np.eye(self.state_dim))
        w_phys = weight_matrices.get('w_physics_target', 30.0)
        w_ay = weight_matrices.get('w_ay', 0.0)  # Legacy, default disabled
        w_smooth = weight_matrices.get('w_smooth', 2.0)
        lambda_bound = weight_matrices.get('lambda_bound', 0.1)
        f_max = weight_matrices.get('f_max', 50.0)
        lambda_warmstart = weight_matrices.get('lambda_warmstart', 5.0)
        warmstart_decay = weight_matrices.get('warmstart_decay', 500)
        
        cos_delta = np.cos(steering)
        
        # Current state estimates for slip angle computation
        vx = max(abs(state_hat_nn[0, 0]), vx_min)
        vy = state_hat_nn[1, 0]
        r = state_hat_nn[3, 0]
        
        # Slip angles
        alpha_f = steering - (vy + lf * r) / vx
        alpha_r = -(vy - lr * r) / vx
        
        # Linear tire forces (for reference, used by physics target solver)
        Fyf_lin = Cf * alpha_f
        Fyr_lin = Cr * alpha_r
        
        # Initialize total gradient and loss
        dL_df_total = np.zeros((1, self.output_dim))
        loss_total = 0.0
        
        # ========== Term 1: State Consistency with Layer 1 ==========
        uio_error = state_hat_nn - state_hat_uio
        loss_state = self.K_loss * ((uio_error.T @ T_uio) @ uio_error)[0, 0]
        dL_state_dx = T_uio @ uio_error
        dL_df_state = dL_state_dx.T @ dx_df  # (1 × output_dim)
        
        loss_total += loss_state
        dL_df_total += dL_df_state
        
        # ========== Term 2: Solved 2D Physics Target ==========
        # Solve [lateral force + yaw moment] for unique (w_r*, w_f*)
        if w_phys > 0.0:
            w_star = self._solve_physics_target(
                vehicle_params, steering, ay_meas, r_dot_meas,
                alpha_f, alpha_r
            )
            # Clamp w_star to reasonable bounds to avoid extreme targets
            w_star = np.clip(w_star, -f_max * 2, f_max * 2)
            
            f_phys_error = f_nn_col - w_star  # (2×1)
            loss_phys = w_phys * np.sum(f_phys_error ** 2)
            
            # ∂L_physics/∂f = 2·w_phys·(f_nn - w*)
            dL_df_phys = 2 * w_phys * f_phys_error.T  # (1 × 2)
            
            loss_total += loss_phys
            dL_df_total += dL_df_phys
        
        # # ========== Term 2b: Legacy 1D L_ay (backward compatible) ==========
        # if w_ay > 0.0:
        #     wr = f_nn_col[0, 0]
        #     wf = f_nn_col[1, 0]
        #     Fy_total_est = (Fyr_lin + wr) + (Fyf_lin + wf) * cos_delta
        #     e_ay = m * ay_meas - Fy_total_est
            
        #     loss_ay = w_ay * e_ay ** 2
        #     dL_df_ay = np.zeros((1, self.output_dim))
        #     dL_df_ay[0, 0] = 2 * w_ay * e_ay * (-1.0)
        #     dL_df_ay[0, 1] = 2 * w_ay * e_ay * (-cos_delta)
            
        #     loss_total += loss_ay
        #     dL_df_total += dL_df_ay
        
        # ========== Term 3: Temporal Smoothness ==========
        f_diff = f_nn_col - f_nn_prev_col
        loss_smooth = w_smooth * np.sum(f_diff ** 2)
        dL_df_smooth = 2 * w_smooth * f_diff.T
        
        loss_total += loss_smooth
        dL_df_total += dL_df_smooth
        
        # ========== Term 4: Soft Physical Bound ==========
        loss_bound = 0.0
        dL_df_bound = np.zeros((1, self.output_dim))
        for i in range(self.output_dim):
            fi = abs(f_nn_col[i, 0])
            if fi > f_max:
                excess = fi - f_max
                loss_bound += lambda_bound * excess ** 2
                sign_fi = np.sign(f_nn_col[i, 0])
                dL_df_bound[0, i] = 2 * lambda_bound * excess * sign_fi
        
        loss_total += loss_bound
        dL_df_total += dL_df_bound
        
        # ========== Term 5: Warm Start from Layer 1 ==========
        if w_hat_L1 is not None and lambda_warmstart > 0.0 and warmstart_decay > 0:
            decay_factor = np.exp(-step_count / warmstart_decay)
            w_L1_col = self._ensure_col_vector(w_hat_L1)
            f_warmstart_error = f_nn_col - w_L1_col
            effective_lambda = lambda_warmstart * decay_factor
            
            loss_warmstart = effective_lambda * np.sum(f_warmstart_error ** 2)
            dL_df_warmstart = 2 * effective_lambda * f_warmstart_error.T
            
            loss_total += loss_warmstart
            dL_df_total += dL_df_warmstart
        
        # ========== Term 6: Optional Reference Tracking ==========
        if reference is not None and ref_indices is not None:
            T_ref = weight_matrices.get('T_ref', np.eye(len(reference)))
            reference = reference.reshape(-1, 1)
            n_ref = reference.shape[0]
            
            if T_ref.shape[0] != n_ref:
                T_ref = np.eye(n_ref)
            
            x_nn_reduced = state_hat_nn[ref_indices]
            tracking_error = x_nn_reduced - reference
            loss_ref = self.K_loss * ((tracking_error.T @ T_ref) @ tracking_error)[0, 0]
            
            dL_ref_dx_reduced = T_ref @ tracking_error
            dL_ref_dx = np.zeros((self.state_dim, 1))
            for i, idx in enumerate(ref_indices):
                dL_ref_dx[idx, 0] = dL_ref_dx_reduced[i, 0]
            dL_df_ref = dL_ref_dx.T @ dx_df
            
            loss_total += loss_ref
            dL_df_total += dL_df_ref
        
        return dL_df_total, loss_total

    # =========================================================================
    # Autodiff Loss Functions (PyTorch)
    # =========================================================================
    
    def compute_loss_physics_informed_tire_autodiff(self,
                                                     x_hat_nn: 'torch.Tensor',
                                                     x_hat_uio: 'torch.Tensor',
                                                     f_nn: 'torch.Tensor',
                                                     f_nn_prev: 'torch.Tensor',
                                                     vehicle_params: dict,
                                                     steering: float,
                                                     ay_meas: float,
                                                     weight_matrices: dict,
                                                     reference: Optional['torch.Tensor'] = None,
                                                     ref_indices: Optional[np.ndarray] = None,
                                                     r_dot_meas: float = 0.0,
                                                     w_hat_L1: Optional['torch.Tensor'] = None,
                                                     step_count: int = 0,
                                                     ) -> 'torch.Tensor':
        """
        Physics-Informed Tire Residual Loss V2 (autodiff version).
        
        Uses solved 2D physics target from lateral force + yaw moment equations.
        
        Args:
            x_hat_nn: Neural observer state tensor (6,) 
            x_hat_uio: First-layer state tensor (6,)
            f_nn: NN tire residual tensor (2,) - requires_grad=True
            f_nn_prev: Previous NN output tensor (2,)
            vehicle_params: Dict with m, Iz, lf, lr, Cf, Cr, vx_min
            steering: Steering angle [rad]
            ay_meas: Lateral acceleration measurement [m/s²]
            weight_matrices: Dict with T_uio, w_physics_target, w_ay, w_smooth,
                            lambda_bound, f_max, lambda_warmstart, warmstart_decay, T_ref
            reference: Optional reference tensor
            ref_indices: Optional reference indices
            r_dot_meas: Measured yaw acceleration [rad/s²]
            w_hat_L1: Layer 1 residual estimate tensor [w_r, w_f] for warm start
            step_count: Current training step (for warm start decay)
            
        Returns:
            Differentiable loss tensor (scalar)
        """
        if not TORCH_AVAILABLE:
            raise RuntimeError("PyTorch not available for autodiff")
        
        x_hat_nn = x_hat_nn.flatten()
        x_hat_uio = x_hat_uio.flatten()
        f_nn = f_nn.flatten()
        f_nn_prev = f_nn_prev.flatten()
        
        m = vehicle_params['m']
        Iz = vehicle_params['Iz']
        lf = vehicle_params['lf']
        lr = vehicle_params['lr']
        Cf = vehicle_params['Cf']
        Cr = vehicle_params['Cr']
        vx_min = vehicle_params.get('vx_min', 0.3)
        
        T_uio = weight_matrices.get('T_uio', torch.eye(x_hat_nn.shape[0], dtype=x_hat_nn.dtype))
        w_phys = weight_matrices.get('w_physics_target', 30.0)
        w_ay = weight_matrices.get('w_ay', 0.0)
        w_smooth = weight_matrices.get('w_smooth', 2.0)
        lambda_bound = weight_matrices.get('lambda_bound', 0.1)
        f_max = weight_matrices.get('f_max', 50.0)
        lambda_warmstart = weight_matrices.get('lambda_warmstart', 5.0)
        warmstart_decay = weight_matrices.get('warmstart_decay', 500)
        
        cos_delta = np.cos(steering)
        
        # Current state estimates for slip angles
        vx = torch.clamp(torch.abs(x_hat_nn[0]), min=vx_min)
        vy = x_hat_nn[1]
        r = x_hat_nn[3]
        
        alpha_f = steering - (vy + lf * r) / vx
        alpha_r = -(vy - lr * r) / vx
        
        Fyf_lin = Cf * alpha_f
        Fyr_lin = Cr * alpha_r
        
        # Term 1: State consistency
        uio_error = x_hat_nn - x_hat_uio
        if isinstance(T_uio, torch.Tensor):
            loss_state = uio_error @ T_uio @ uio_error
        else:
            T_uio_t = torch.tensor(T_uio, dtype=x_hat_nn.dtype) if not isinstance(T_uio, torch.Tensor) else T_uio
            loss_state = uio_error @ T_uio_t @ uio_error
        
        # Term 2: Solved 2D physics target
        loss_phys = torch.tensor(0.0, dtype=x_hat_nn.dtype)
        if w_phys > 0.0:
            # Solve the 2-equation system using numpy (no gradient needed for w*)
            alpha_f_val = alpha_f.detach().item() if isinstance(alpha_f, torch.Tensor) else float(alpha_f)
            alpha_r_val = alpha_r.detach().item() if isinstance(alpha_r, torch.Tensor) else float(alpha_r)
            w_star_np = self._solve_physics_target(
                vehicle_params, steering, ay_meas, r_dot_meas,
                alpha_f_val, alpha_r_val
            )
            w_star_np = np.clip(w_star_np, -f_max * 2, f_max * 2)
            w_star = torch.tensor(w_star_np.flatten(), dtype=f_nn.dtype)
            
            f_phys_error = f_nn - w_star
            loss_phys = w_phys * torch.sum(f_phys_error ** 2)
        
        # Term 2b: Legacy 1D L_ay (backward compatible)
        loss_ay = torch.tensor(0.0, dtype=x_hat_nn.dtype)
        if w_ay > 0.0:
            wr = f_nn[0]
            wf = f_nn[1]
            Fy_total_est = (Fyr_lin + wr) + (Fyf_lin + wf) * cos_delta
            e_ay = m * ay_meas - Fy_total_est
            loss_ay = w_ay * e_ay ** 2
        
        # Term 3: Temporal smoothness
        f_diff = f_nn - f_nn_prev
        loss_smooth = w_smooth * torch.sum(f_diff ** 2)
        
        # Term 4: Soft bound
        loss_bound = torch.tensor(0.0, dtype=x_hat_nn.dtype)
        for i in range(f_nn.shape[0]):
            excess = torch.clamp(torch.abs(f_nn[i]) - f_max, min=0.0)
            loss_bound = loss_bound + lambda_bound * excess ** 2
        
        # Term 5: Warm start from Layer 1
        loss_warmstart = torch.tensor(0.0, dtype=x_hat_nn.dtype)
        if w_hat_L1 is not None and lambda_warmstart > 0.0 and warmstart_decay > 0:
            decay_factor = np.exp(-step_count / warmstart_decay)
            if not isinstance(w_hat_L1, torch.Tensor):
                w_hat_L1 = torch.tensor(w_hat_L1.flatten(), dtype=f_nn.dtype)
            w_hat_L1 = w_hat_L1.flatten()
            f_warmstart_error = f_nn - w_hat_L1
            effective_lambda = lambda_warmstart * decay_factor
            loss_warmstart = effective_lambda * torch.sum(f_warmstart_error ** 2)
        
        # Term 6: Optional reference tracking
        loss_ref = torch.tensor(0.0, dtype=x_hat_nn.dtype)
        if reference is not None and ref_indices is not None:
            T_ref = weight_matrices.get('T_ref', torch.eye(reference.shape[0], dtype=x_hat_nn.dtype))
            if not isinstance(T_ref, torch.Tensor):
                T_ref = torch.tensor(T_ref, dtype=x_hat_nn.dtype)
            reference = reference.flatten()
            x_nn_reduced = x_hat_nn[ref_indices]
            tracking_error = x_nn_reduced - reference
            loss_ref = tracking_error @ T_ref @ tracking_error
        
        return loss_state + loss_phys + loss_ay + loss_smooth + loss_bound + loss_warmstart + loss_ref

    # =========================================================================
    # Self-Supervised Prediction Error Loss
    # =========================================================================
    
    def chain_rule_prediction_error(self,
                                    y_next: np.ndarray,
                                    state_hat: np.ndarray,
                                    dx_df: np.ndarray,
                                    f_nn: np.ndarray,
                                    f_nn_prev: np.ndarray,
                                    A_d: np.ndarray,
                                    B_d: np.ndarray,
                                    E_d: np.ndarray,
                                    C: np.ndarray,
                                    D: np.ndarray,
                                    F: np.ndarray,
                                    u: np.ndarray,
                                    weight_matrices: dict,
                                    state_hat_uio: Optional[np.ndarray] = None,
                                    ) -> Tuple[np.ndarray, float]:
        """
        Self-Supervised 1-Step Prediction Error Loss (Analytical).
        
        No need to compute a noisy physics target w*. Instead, trains the NN
        to minimize how well the observer predicts the NEXT measurement.
        
        Loss = L_pred + L_smooth + L_bound [+ L_uio]
        
        Terms:
            L_pred  = ||y[k] - ŷ[k]||²_W  where ŷ = C·x̂ + D·u + F·w
            L_smooth = w_smooth · ||f_nn(k) - f_nn(k-1)||²
            L_bound  = λ_bound · Σ max(0, |f_i| - f_max)²
            L_uio    = w_uio · ||x̂_nn - x̂_uio||²  (optional consistency)
        
        The key advantage: the gradient dL/df flows through the observer
        dynamics (via dx_df sensitivity) — the NN learns whatever f_nn
        makes the next prediction match the measurement.
        
        Args:
            y_next: Current measurement vector (meas_dim,) or (meas_dim, 1)
            state_hat: Current observer state estimate (6,) or (6, 1)
            dx_df: Sensitivity matrix ∂x/∂f (6 × output_dim)
            f_nn: Current NN output [wr, wf] (output_dim,) or (output_dim, 1)
            f_nn_prev: Previous NN output (output_dim,)
            A_d, B_d, E_d: Discrete system matrices
            C: Measurement matrix (meas_dim × 6)
            D: Feedthrough D·u matrix (meas_dim × 2)
            F: Feedthrough F·w matrix (meas_dim × output_dim)
            u: Control input [δ, a] (2,)
            weight_matrices: Dict with:
                - 'W_pred': Measurement weight (meas_dim × meas_dim)
                - 'w_smooth': Temporal smoothness weight
                - 'lambda_bound': Soft bound weight
                - 'f_max': Max expected residual
                - 'T_uio': UIO consistency weight (optional)
                - 'w_uio_scale': Scalar scale for UIO term (optional)
            state_hat_uio: First-layer state (optional, for UIO consistency)
            
        Returns:
            Tuple of (gradient dL/df, total loss value)
        """
        state_hat = state_hat.reshape(-1, 1)
        y_next = y_next.reshape(-1, 1)
        u = u.reshape(-1, 1)
        f_nn_col = self._ensure_col_vector(f_nn)
        f_nn_prev_col = self._ensure_col_vector(f_nn_prev)
        
        W_pred = weight_matrices.get('W_pred', np.eye(y_next.shape[0]))
        w_smooth = weight_matrices.get('w_smooth', 0.5)
        lambda_bound = weight_matrices.get('lambda_bound', 0.1)
        f_max = weight_matrices.get('f_max', 50.0)
        
        dL_df_total = np.zeros((1, self.output_dim))
        loss_total = 0.0
        
        # ========== Term 1: Prediction Error ==========
        # ŷ = C·x̂ + D·u + F·w
        y_hat = C @ state_hat + D @ u + F @ f_nn_col
        pred_error = y_hat - y_next  # (meas_dim, 1)
        
        loss_pred = (pred_error.T @ W_pred @ pred_error)[0, 0]
        
        # Gradient: dL/df = dL/dy_hat · (dy_hat/dx · dx/df + dy_hat/df)
        # dy_hat/dx = C, dy_hat/df = F
        dL_dy_hat = 2.0 * W_pred @ pred_error  # (meas_dim, 1)
        dL_df_pred = dL_dy_hat.T @ C @ dx_df + dL_dy_hat.T @ F  # (1, output_dim)
        
        loss_total += loss_pred
        dL_df_total += dL_df_pred
        
        # ========== Term 2: Temporal Smoothness ==========
        f_diff = f_nn_col - f_nn_prev_col
        loss_smooth = w_smooth * np.sum(f_diff ** 2)
        dL_df_smooth = 2 * w_smooth * f_diff.T
        
        loss_total += loss_smooth
        dL_df_total += dL_df_smooth
        
        # ========== Term 3: Soft Bound ==========
        loss_bound = 0.0
        dL_df_bound = np.zeros((1, self.output_dim))
        for i in range(self.output_dim):
            fi = abs(f_nn_col[i, 0])
            if fi > f_max:
                excess = fi - f_max
                loss_bound += lambda_bound * excess ** 2
                sign_fi = np.sign(f_nn_col[i, 0])
                dL_df_bound[0, i] = 2 * lambda_bound * excess * sign_fi
        
        loss_total += loss_bound
        dL_df_total += dL_df_bound
        
        # ========== Term 4: L2 Magnitude Penalty ==========
        # Prevents NN from outputting unreasonably large residuals
        lambda_l2 = weight_matrices.get('lambda_l2', 0.0)
        if lambda_l2 > 0.0:
            loss_l2 = lambda_l2 * np.sum(f_nn_col ** 2)
            dL_df_l2 = 2 * lambda_l2 * f_nn_col.T  # (1, output_dim)
            loss_total += loss_l2
            dL_df_total += dL_df_l2
        
        # ========== Term 5: Warm Start from Layer 1 ==========
        w_hat_L1 = weight_matrices.get('w_hat_L1')
        lambda_warmstart = weight_matrices.get('lambda_warmstart', 0.0)
        warmstart_decay = weight_matrices.get('warmstart_decay', 300)
        step_count = weight_matrices.get('step_count', 0)
        if w_hat_L1 is not None and lambda_warmstart > 0.0 and warmstart_decay > 0:
            decay_factor = np.exp(-step_count / warmstart_decay)
            w_L1_col = self._ensure_col_vector(w_hat_L1)
            f_warmstart_error = f_nn_col - w_L1_col
            effective_lambda = lambda_warmstart * decay_factor
            loss_warmstart = effective_lambda * np.sum(f_warmstart_error ** 2)
            dL_df_warmstart = 2 * effective_lambda * f_warmstart_error.T
            loss_total += loss_warmstart
            dL_df_total += dL_df_warmstart
        
        # ========== Term 6: Optional UIO Consistency ==========
        if state_hat_uio is not None:
            T_uio = weight_matrices.get('T_uio', np.eye(self.state_dim))
            w_uio_scale = weight_matrices.get('w_uio_scale', 0.1)  # Low weight — advisory only
            state_hat_uio = state_hat_uio.reshape(-1, 1)
            uio_error = state_hat - state_hat_uio
            loss_uio = w_uio_scale * (uio_error.T @ T_uio @ uio_error)[0, 0]
            dL_uio_dx = w_uio_scale * T_uio @ uio_error
            dL_df_uio = dL_uio_dx.T @ dx_df
            
            loss_total += loss_uio
            dL_df_total += dL_df_uio
        
        # ========== Term 7: Tire Residual Correlation ==========
        # Penalize sign disagreement: L_corr = w_corr * (w_r - w_f)²
        # Both tires saturate together during cornering.
        w_corr = weight_matrices.get('w_corr', weight_matrices.get('pred_w_corr', 0.0))
        if w_corr > 0.0 and self.output_dim >= 2:
            diff_rf = f_nn_col[0, 0] - f_nn_col[1, 0]
            loss_corr = w_corr * diff_rf ** 2
            # Gradient: d/d(wr) = 2*w_corr*(wr-wf), d/d(wf) = -2*w_corr*(wr-wf)
            dL_df_corr = np.zeros((1, self.output_dim))
            dL_df_corr[0, 0] = 2 * w_corr * diff_rf
            dL_df_corr[0, 1] = -2 * w_corr * diff_rf
            loss_total += loss_corr
            dL_df_total += dL_df_corr
        
        return dL_df_total, loss_total
    
    def compute_loss_prediction_error_autodiff(self,
                                               x_hat_nn: 'torch.Tensor',
                                               f_nn: 'torch.Tensor',
                                               f_nn_prev: 'torch.Tensor',
                                               y_meas: 'torch.Tensor',
                                               C: 'torch.Tensor',
                                               D: 'torch.Tensor',
                                               F: 'torch.Tensor',
                                               u: 'torch.Tensor',
                                               weight_matrices: dict,
                                               x_hat_uio: Optional['torch.Tensor'] = None,
                                               ) -> 'torch.Tensor':
        """
        Self-Supervised 1-Step Prediction Error Loss (Autodiff).
        
        Trains the NN so that the observer's predicted measurement matches
        the actual measurement. No noisy w* target needed.
        
        Args:
            x_hat_nn: Observer state after update (6,) — differentiable w.r.t. f_nn
            f_nn: NN tire residual tensor (output_dim,) — requires_grad=True
            f_nn_prev: Previous NN output tensor (output_dim,)
            y_meas: Actual measurement tensor (meas_dim,)
            C: Measurement matrix tensor (meas_dim, 6)
            D: Feedthrough D·u tensor (meas_dim, 2)
            F: Feedthrough F·w tensor (meas_dim, output_dim)
            u: Control input tensor (2,)
            weight_matrices: Dict with W_pred, w_smooth, lambda_bound, f_max, T_uio
            x_hat_uio: Optional first-layer state tensor (6,)
            
        Returns:
            Differentiable loss tensor (scalar)
        """
        if not TORCH_AVAILABLE:
            raise RuntimeError("PyTorch not available for autodiff")
        
        x_hat_nn = x_hat_nn.flatten()
        f_nn = f_nn.flatten()
        f_nn_prev = f_nn_prev.flatten()
        y_meas = y_meas.flatten()
        u = u.flatten()
        
        W_pred_np = weight_matrices.get('W_pred', np.eye(y_meas.shape[0]))
        if isinstance(W_pred_np, np.ndarray):
            W_pred = torch.tensor(W_pred_np, dtype=x_hat_nn.dtype)
        else:
            W_pred = W_pred_np
        w_smooth = weight_matrices.get('w_smooth', 0.5)
        lambda_bound = weight_matrices.get('lambda_bound', 0.1)
        f_max = weight_matrices.get('f_max', 50.0)
        
        # Term 1: Prediction error
        y_hat = C @ x_hat_nn + D @ u + F @ f_nn
        pred_error = y_hat - y_meas
        loss_pred = pred_error @ W_pred @ pred_error
        
        # Term 2: Temporal smoothness
        f_diff = f_nn - f_nn_prev
        loss_smooth = w_smooth * torch.sum(f_diff ** 2)
        
        # Term 3: Soft bound
        loss_bound = torch.tensor(0.0, dtype=x_hat_nn.dtype)
        for i in range(f_nn.shape[0]):
            excess = torch.clamp(torch.abs(f_nn[i]) - f_max, min=0.0)
            loss_bound = loss_bound + lambda_bound * excess ** 2
        
        # Term 4: L2 magnitude penalty
        lambda_l2 = weight_matrices.get('lambda_l2', 0.0)
        loss_l2 = lambda_l2 * torch.sum(f_nn ** 2) if lambda_l2 > 0 else torch.tensor(0.0, dtype=x_hat_nn.dtype)
        
        # Term 5: Warm start from Layer 1
        loss_warmstart = torch.tensor(0.0, dtype=x_hat_nn.dtype)
        w_hat_L1 = weight_matrices.get('w_hat_L1')
        lambda_warmstart = weight_matrices.get('lambda_warmstart', 0.0)
        warmstart_decay = weight_matrices.get('warmstart_decay', 300)
        step_count = weight_matrices.get('step_count', 0)
        if w_hat_L1 is not None and lambda_warmstart > 0.0 and warmstart_decay > 0:
            import math
            decay_factor = math.exp(-step_count / warmstart_decay)
            if isinstance(w_hat_L1, np.ndarray):
                w_hat_L1_t = torch.tensor(w_hat_L1.flatten(), dtype=x_hat_nn.dtype)
            else:
                w_hat_L1_t = w_hat_L1.flatten()
            effective_lambda = lambda_warmstart * decay_factor
            warmstart_error = f_nn - w_hat_L1_t
            loss_warmstart = effective_lambda * torch.sum(warmstart_error ** 2)
        
        # Term 6: Optional UIO consistency (low weight)
        loss_uio = torch.tensor(0.0, dtype=x_hat_nn.dtype)
        if x_hat_uio is not None:
            T_uio_np = weight_matrices.get('T_uio', np.eye(x_hat_nn.shape[0]))
            T_uio = torch.tensor(T_uio_np, dtype=x_hat_nn.dtype) if isinstance(T_uio_np, np.ndarray) else T_uio_np
            w_uio_scale = weight_matrices.get('w_uio_scale', 0.1)
            x_hat_uio = x_hat_uio.flatten()
            uio_error = x_hat_nn - x_hat_uio
            loss_uio = w_uio_scale * (uio_error @ T_uio @ uio_error)
        
        return loss_pred + loss_smooth + loss_bound + loss_l2 + loss_warmstart + loss_uio

    def compute_loss_measurement_autodiff(self,
                                          x_hat: 'torch.Tensor',
                                          measurement: 'torch.Tensor',
                                          weight_matrix: 'torch.Tensor',
                                          f_nn: 'torch.Tensor',
                                          f_uk: 'torch.Tensor',
                                          lambda_reg: float = 0.0) -> 'torch.Tensor':
        """
        Compute measurement tracking loss with autodiff support.
        
        Loss: L = (x̂ - y)ᵀ W (x̂ - y) + λ ||f_nn - f_uk||²
        
        All inputs should be torch tensors with requires_grad where needed.
        
        Args:
            x_hat: Estimated state tensor (state_dim,) or (state_dim, 1)
            measurement: Measurement tensor (state_dim,)
            weight_matrix: Weight matrix tensor (state_dim, state_dim)
            f_nn: Neural network output tensor (output_dim,) - requires_grad=True
            f_uk: Unknown input estimate tensor (output_dim,)
            lambda_reg: Regularization weight
            
        Returns:
            loss: Differentiable loss tensor (scalar)
        """
        if not TORCH_AVAILABLE:
            raise RuntimeError("PyTorch not available for autodiff")
        
        # Ensure correct shapes
        x_hat = x_hat.flatten()
        measurement = measurement.flatten()
        f_nn = f_nn.flatten()
        f_uk = f_uk.flatten()
        
        # State tracking error
        state_error = x_hat - measurement
        loss_state = state_error @ weight_matrix @ state_error
        
        # Regularization term
        f_error = f_nn - f_uk
        loss_reg = lambda_reg * torch.sum(f_error ** 2)
        
        return loss_state + loss_reg
    
    def compute_loss_composite_uio_autodiff(self,
                                            x_hat_nn: 'torch.Tensor',
                                            x_hat_uio: 'torch.Tensor',
                                            reference: 'torch.Tensor',
                                            measurement: 'torch.Tensor',
                                            C: 'torch.Tensor',
                                            weight_matrices: dict,
                                            f_nn: 'torch.Tensor',
                                            f_uk_uio: 'torch.Tensor',
                                            lambda_reg: float = 0.0,
                                            ref_indices: Optional[np.ndarray] = None) -> 'torch.Tensor':
        """
        Compute composite UIO loss with autodiff support.
        
        Loss: L = (x̂_nn[ref_idx] - x_ref)ᵀ T_ref (x̂_nn[ref_idx] - x_ref)  [Tracking]
                + (y - C·x̂_nn)ᵀ T_y (y - C·x̂_nn)                         [Output Error]
                + (x̂_nn - x̂_uio)ᵀ T_uio (x̂_nn - x̂_uio)                  [UIO Consistency]
                + λ ||f_nn - f̂_uk||²                                       [Regularization]
        
        Args:
            x_hat_nn: Neural observer state estimate tensor (state_dim,)
            x_hat_uio: UIO state estimate tensor (state_dim,)
            reference: Reference trajectory tensor (n_ref,)
            measurement: Measurement tensor (meas_dim,)
            C: Output matrix tensor (meas_dim, state_dim)
            weight_matrices: Dict with tensors 'T_ref', 'T_y', 'T_uio'
            f_nn: Neural network output tensor - requires_grad=True
            f_uk_uio: UIO unknown input estimate tensor
            lambda_reg: Regularization weight
            ref_indices: Indices mapping reference to state
            
        Returns:
            loss: Differentiable loss tensor (scalar)
        """
        if not TORCH_AVAILABLE:
            raise RuntimeError("PyTorch not available for autodiff")
        
        # Ensure correct shapes
        x_hat_nn = x_hat_nn.flatten()
        x_hat_uio = x_hat_uio.flatten()
        reference = reference.flatten()
        measurement = measurement.flatten()
        f_nn = f_nn.flatten()
        f_uk_uio = f_uk_uio.flatten()
        
        n_ref = reference.shape[0]
        
        # Get weight matrices
        T_ref = weight_matrices.get('T_ref', torch.eye(n_ref, dtype=x_hat_nn.dtype))
        T_y = weight_matrices.get('T_y', torch.eye(measurement.shape[0], dtype=x_hat_nn.dtype))
        T_uio = weight_matrices.get('T_uio', torch.eye(x_hat_nn.shape[0], dtype=x_hat_nn.dtype))
        
        # Determine reference indices
        if ref_indices is None:
            ref_indices = list(range(n_ref))
        
        # Term 1: Tracking error (reduced dimension)
        x_nn_reduced = x_hat_nn[ref_indices]
        tracking_error = x_nn_reduced - reference
        loss_tracking = tracking_error @ T_ref @ tracking_error
        
        # Term 2: Output error
        y_nn = C @ x_hat_nn
        output_error = measurement - y_nn
        loss_output = output_error @ T_y @ output_error
        
        # Term 3: UIO consistency
        uio_error = x_hat_nn - x_hat_uio
        loss_uio = uio_error @ T_uio @ uio_error
        
        # Term 4: Regularization
        f_error = f_nn - f_uk_uio
        loss_reg = lambda_reg * torch.sum(f_error ** 2)
        
        return loss_tracking + loss_output + loss_uio + loss_reg


def create_weight_matrix(weights: dict) -> np.ndarray:
    """
    Create diagonal weight matrix from weight dictionary
    
    Args:
        weights: Dictionary with weight keys. Supports:
            - 4D: 'v_x', 'v_y', 'psi', 'psi_dot'
            - 6D: 'v_x', 'v_y', 'psi', 'psi_dot', 'X', 'Y' (or 'r' for psi_dot)
    
    Returns:
        Diagonal weight matrix (4×4 or 6×6 depending on keys present)
    """
    # Check if 6D weights are specified
    if 'X' in weights or 'Y' in weights:
        # 6D state: [v_x, v_y, ψ, r, X, Y]
        weight_array = np.array([
            weights.get('v_x', 1.0),
            weights.get('v_y', 1.0),
            weights.get('psi', 1.0),
            weights.get('psi_dot', weights.get('r', 1.0)),
            weights.get('X', 1.0),
            weights.get('Y', 1.0)
        ])
    else:
        # 4D state: [v_x, v_y, ψ, ψ_dot]
        weight_array = np.array([
            weights.get('v_x', 1.0),
            weights.get('v_y', 1.0),
            weights.get('psi', 1.0),
            weights.get('psi_dot', 1.0)
        ])
    
    return np.diag(weight_array)

