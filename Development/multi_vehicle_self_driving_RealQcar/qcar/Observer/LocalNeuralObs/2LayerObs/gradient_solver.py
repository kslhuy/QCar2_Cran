"""
Gradient Solver for Neural Observer

Computes sensitivities and gradients for backpropagation through observer dynamics.
Implements the chain rule to propagate gradients from loss functions back to neural
network parameters.

Supports two gradient computation methods:
    1. Analytical: Manual sensitivity propagation (dx/df[k+1] = A_d·dx/df[k] + E_d)
    2. Autodiff: PyTorch automatic differentiation through observer dynamics

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
    
    def __init__(self, observer_matrices: dict, sample_time: float):
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
        self.K = observer_matrices.get('K')
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
        if 'K' in observer_matrices:
            self.K = observer_matrices['K']
    
    def gradient_solver_continuous(self, sensitivity: np.ndarray) -> np.ndarray:
        """
        Update sensitivity for continuous-time observer
        
        Equation: dx/df[k+1] = ((A + K·C) · dx/df[k] + I) · Ts + dx/df[k]
        
        Args:
            sensitivity: Current sensitivity matrix dx/df (state_dim × output_dim)
        
        Returns:
            Updated sensitivity matrix
        """
        # Compute observer dynamics matrix
        A_obs = self.A + self.K @ self.C
        
        # Update sensitivity (Euler integration)
        sensitivity_new = (A_obs @ sensitivity + np.eye(self.state_dim, self.output_dim)) * self.Ts + sensitivity
        
        return sensitivity_new
    
    def gradient_solver_continuous_with_D(self, sensitivity: np.ndarray) -> np.ndarray:
        """
        Update sensitivity for continuous-time observer with disturbance matrix
        
        Equation: dx/df[k+1] = ((A + K·C) · dx/df[k] + D) · Ts + dx/df[k]
        
        Args:
            sensitivity: Current sensitivity matrix dx/df (state_dim × output_dim)
        
        Returns:
            Updated sensitivity matrix
        """
        # Compute observer dynamics matrix
        A_obs = self.A + self.K @ self.C
        
        # Update sensitivity with D matrix
        sensitivity_new = (A_obs @ sensitivity + self.D) * self.Ts + sensitivity
        
        return sensitivity_new
    
    def gradient_solver_discrete(self, sensitivity: np.ndarray) -> np.ndarray:
        """
        Update sensitivity for discrete-time observer (constant matrices version)
        
        Equation: dx/df[k+1] = (A + K·C) · dx/df[k] + D
        
        Note: This uses matrices set at initialization. For LPV systems,
              use gradient_solver_discrete_lpv() instead.
        
        Args:
            sensitivity: Current sensitivity matrix dx/df (state_dim × output_dim)
        
        Returns:
            Updated sensitivity matrix
        """
        # Compute observer dynamics matrix
        A_obs = self.A + self.K @ self.C
        
        # Update sensitivity (discrete)
        sensitivity_new = A_obs @ sensitivity + self.D
        
        return sensitivity_new
    
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

    # =========================================================================
    # Autodiff Loss Functions (PyTorch)
    # =========================================================================
    
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

