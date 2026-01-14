"""
Gradient Solver for Neural Observer

Computes sensitivities and gradients for backpropagation through observer dynamics.
Implements the chain rule to propagate gradients from loss functions back to neural
network parameters.

Key Equations:
    Sensitivity: dx/df[k+1] = (A + K·C) · dx/df[k] + D
    Chain Rule: dL/df = dL/dx · dx/df
"""

import numpy as np
from typing import Tuple, Optional


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
        self.output_dim = 2  # Tire force compensation dimension
        
        # Loss scaling factor
        self.K_loss = 1.0
    
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
        Update sensitivity for discrete-time observer
        
        Equation: dx/df[k+1] = (A + K·C) · dx/df[k] + D
        
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
        
        # Loss 2: Force regularization
        f_nn_reshaped = f_nn.T if f_nn.shape[0] == self.output_dim else f_nn
        f_hat_reshaped = f_hat.reshape(1, -1)
        f_error = f_nn_reshaped - f_hat_reshaped
        loss2 = lambda_reg * np.linalg.norm(f_error, ord=2)
        
        # Total loss
        loss_total = loss1 + loss2
        
        # Gradient computation
        # dL/dx = W · (x̂ - x_ref)
        dL_dx = weight_matrix @ state_error
        
        # dL/df = dL/dx · dx/df
        dL_df_1 = dL_dx.T @ dx_df
        
        # dL/df from force regularization
        dL_df_2 = lambda_reg * f_error
        
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
        
        # Loss 2: Force regularization
        f_error = f_nn - f_hat
        loss2 = lambda_reg * np.linalg.norm(f_error, ord=2)
        
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
        
        # dL/df from force regularization
        f_nn_reshaped = f_nn.T if f_nn.shape[0] == self.output_dim else f_nn
        f_hat_reshaped = f_hat.reshape(1, -1)
        dL_df_2 = lambda_reg * (f_nn_reshaped - f_hat_reshaped)
        
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
        
        # Loss 2: Force error
        f_nn_reshaped = f_nn.T if f_nn.shape[0] == self.output_dim else f_nn
        f_uk_reshaped = f_uk.reshape(1, -1)
        f_error = f_nn_reshaped - f_uk_reshaped
        loss2 = lambda_reg * np.linalg.norm(f_error, ord=2)
        
        # Total loss
        loss_total = loss1 + loss2
        
        # Gradient computation
        # dL/dx = K_loss · W · (x̂ - x)
        dL_dy = self.K_loss * (weight_matrix @ state_error)
        
        # dL/df = dL/dx · dx/df
        dL_df_1 = dL_dy.T @ dx_df
        
        # dL/df from force error
        dL_df_2 = lambda_reg * f_error
        
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
        f_nn_reshaped = f_nn.T if f_nn.shape[0] == self.output_dim else f_nn
        f_uk_reshaped = f_uk_uio.reshape(1, -1)
        f_error = f_nn_reshaped - f_uk_reshaped
        loss_reg = lambda_reg * np.linalg.norm(f_error, ord=2) ** 2
        
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

