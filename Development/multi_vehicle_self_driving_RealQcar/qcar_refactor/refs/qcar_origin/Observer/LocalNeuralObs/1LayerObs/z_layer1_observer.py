"""
Z-Style Sample-and-Hold Layer 1 Observer

Adapts the logic from sample_obs_z.py to the FirstLayerObserverBase interface,
using centralized QLPVVehicleDynamicsObs for system matrices.

This observer implements a sample-and-hold innovation scheme:
    dot_x = f(x) + B*u + D*d_hat + L_hold * ey_hold
    dot_z = -K*D*z - K*( f(x) + B*u + D*K*x )
    d_hat = z + K*x

where L_hold and ey_hold are updated only at measurement times.
"""

import numpy as np
import sys
from pathlib import Path
from typing import Optional, Tuple, Dict, List

# Add parent directory to path to import base class and dynamics
parent_dir = Path(__file__).parent
sys.path.insert(0, str(parent_dir))
sys.path.insert(0, str(parent_dir.parent))

from firstLayerObserverBase import FirstLayerObserverBase
from Development.multi_vehicle_self_driving_RealQcar.qcar_refactor.refs.qcar_origin.Observer.LocalNeuralObs.qlpv_vehicle_dynamics_obs import (
    QLPVVehicleDynamicsObs, 
    SchedulingParameters,
    get_vehicle_params,
    IDX_VX, IDX_VY, IDX_PSI, IDX_R, IDX_X, IDX_Y, STATE_DIM,
    MEAS_IDX_VX, MEAS_IDX_R, MEAS_IDX_PSI, MEAS_IDX_X, MEAS_IDX_Y, MEAS_IDX_AY
)

class ZLayer1Observer(FirstLayerObserverBase):
    """
    Z-Style Observer with Sample-and-Hold Innovation. 
    
    Ported from SampledDataObserver in sample_obs_z.py but adapted to 
    interface with QLPVVehicleDynamicsObs and FirstLayerObserverBase.
    
    The internal "Z-observer" uses a reduced state [vy, r, vx] (3D) + z (2D),
    but this wrapper manages the full 6D state [vx, vy, psi, r, X, Y].
    """
    
    def __init__(self, 
                 vehicle_params: Optional[Dict] = None, 
                 sample_time: float = 0.02,
                 observer_gains: Optional[Dict] = None):
        
        super().__init__(state_dim=6, unknown_input_dim=2, sample_time=sample_time)
        
        # 1. Setup Dynamics
        self.params = vehicle_params if vehicle_params is not None else get_vehicle_params()
        self.dynamics = QLPVVehicleDynamicsObs(self.params)
        
        # 2. Extract specific params needed for Z-logic (if any explicit usage remains)
        self.vx_min = self.params.get('vx_min', 0.5)
        self.vx_max = self.params.get('vx_max', 3.0) # Default if not present
        self.vy_min = -1.0 # Approximate bounds for scheduling
        self.vy_max = 1.0
        self.r_min = -1.5
        self.r_max = 1.5
        self.xi_min = -1.0
        self.xi_max = 1.0
        
        # Scheduling constant calculation (from sample_obs_z)
        # v0, v1 (eq. 16)
        self.v0 = 2 * self.vx_min * self.vx_max / (self.vx_min + self.vx_max)
        self.v1 = 2 * self.vx_min * self.vx_max / (self.vx_min - self.vx_max)
        
        # 3. Setup Gains
        # Expecting 'L_vertices' and 'tau' in observer_gains
        if observer_gains is None:
            observer_gains = {}
            
        self.L_vertices = observer_gains.get('L_vertices', None)
        self.tau = observer_gains.get('tau', 10.0) # Default tau
        
        if self.L_vertices is None:
             # Create dummy default gains if not provided (to prevent crash, but warn)
             print("Warning: ZLayer1Observer - L_vertices not provided in observer_gains! Using zeros.")
             self.L_vertices = [np.zeros((3, 2)) for _ in range(8)]
             
        # 4. Initialize Internal Z-State
        # xhat_z corresponds to [vy, r, vx]
        # But we will rely on self.state_hat (6D) as the primary state source.
        # We also need 'z' state for the disturbance estimation part.
        self.zhat = np.zeros(2) # 2D 'z' vector
        
        # Sample-and-Hold Storage
        self.L_hold = np.zeros((3, 2))
        self.ey_hold = np.zeros(2)
        self.has_sample = False
        
        # Precompute Constant Matrices if possible?
        # The sample_obs_z uses *constant* B and D constructed from params.
        # However, QLPV dynamics usually have state-dependent B (through steering).
        # sample_obs_z treats them as constant linear approximations?
        # Let's look at sample_obs_z again:
        # B depends on Coeffs, but NOT delta? sample_obs_z eq (5) suggests linear structure.
        # QLPV B(rho) has cos(delta) terms.
        # To strictly follow sample_obs_z "scheme", we might need the *linearized* constant matrices
        # for the observer structure injection, OR we can use the QLPV matrices evaluated at 
        # current operating point (Extended/Linear Param Varying approach).
        # Given the instruction "use the same model and matrix these new schem", we MUST use QLPV dynamics.
        
        # We will compute K = tau * D.T dynamically or establish a D substitute.
        # In sample_obs_z, D maps disturbance to state deriv. E matrix in QLPV does exactly that.
        # So D (sample_z) <-> E (QLPV).
        
    def _xi_from_vx(self, vx):
        vx = np.clip(vx, self.vx_min, self.vx_max)
        if abs(vx) < 1e-4: vx = 1e-4
        xi = self.v1 * (1.0/vx - 1.0/self.v0)
        return float(np.clip(xi, self.xi_min, self.xi_max))

    def _weights_h(self, vy_hat, r_meas, vx_hat):
        # 8-vertex scheduling weighting
        r1 = (np.clip(vy_hat, self.vy_min, self.vy_max) - self.vy_min) / (self.vy_max - self.vy_min)
        r2 = (np.clip(r_meas, self.r_min, self.r_max)   - self.r_min)  / (self.r_max  - self.r_min)
        xi_hat = self._xi_from_vx(vx_hat)
        r3 = (xi_hat - self.xi_min) / (self.xi_max - self.xi_min)

        r1 = float(np.clip(r1, 0.0, 1.0))
        r2 = float(np.clip(r2, 0.0, 1.0))
        r3 = float(np.clip(r3, 0.0, 1.0))

        a1, b1 = r1, 1-r1
        a2, b2 = r2, 1-r2
        a3, b3 = r3, 1-r3

        h = np.array([
            b1*b2*b3,
            b1*b2*a3,
            b1*a2*b3,
            b1*a2*a3,
            a1*b2*b3,
            a1*b2*a3,
            a1*a2*b3,
            a1*a2*a3
        ])
        return h

    def _L_from_h(self, h):
        L = np.zeros((3,2))
        for i in range(8):
            L += h[i] * self.L_vertices[i]
        return L

    def _get_dynamics_matrices_for_z_scheme(self, x, u):
        """
        Extract relevant 3x... submatrices for the [vy, r, vx] subset
        from the full 6D QLPV matrices.
        
        x: 6D state
        u: 2D input
        
        Returns:
            f_val (3D), B_sub (3x2), E_sub (3x2)
        """
        # 1. Update scheduling params
        rho = self.dynamics.compute_scheduling_params(x, u[0])
        
        # 2. Get full matrices
        # We need f(x) for the process model.
        # f_continuous returns full 6D x_dot.
        # But we need to separate f(x) + B*u structure vs raw f(x,u).
        # QLPV f_continuous includes B*u implicitely.
        # sample_obs_z separates them: xdot = f(x) + B*u + ...
        # Let's compute f(x) by calling f_continuous with u=0, w=0.
        
        x_dot_autonomous = self.dynamics.f_continuous(x, np.zeros(2), np.zeros(2))
        
        # Extract subset [vy, r, vx] <- Note order!
        # internal Z state is: 0:vy, 1:r, 2:vx
        indices = [IDX_VY, IDX_R, IDX_VX]
        
        f_sub = x_dot_autonomous[indices]
        
        # Get B matrix (control input map)
        B_full = self.dynamics.compute_B_matrix(rho)
        B_sub = B_full[indices, :] # 3x2
        
        # Get E matrix (disturbance map) -> Equivalent to D in sample_obs_z
        E_full = self.dynamics.compute_E_matrix(rho)
        E_sub = E_full[indices, :] # 3x2
        
        return f_sub, B_sub, E_sub

    def _rhs_z(self, xhat_6d, zhat, u):
        """
        Compute derivatives for xhat (subset) and zhat using Z-observer structure.
        """
        # internal state mapping:
        # xhat_z (implicit) = [xhat_6d[IDX_VY], xhat_6d[IDX_R], xhat_6d[IDX_VX]]
        
        # Get dynamics matrices for current state
        # f_sub: 3D, B_sub: 3x2, E_sub: 3x2
        f_sub, B_sub, E_sub = self._get_dynamics_matrices_for_z_scheme(xhat_6d, u)
        
        # K = tau * E.T (using E as D equivalent)
        K = self.tau * E_sub.T # 2x3
        
        # Construct subset vector [vy, r, vx]
        xhat_subset = xhat_6d[[IDX_VY, IDX_R, IDX_VX]]
        
        if self.has_sample:
            inj = self.L_hold @ self.ey_hold # 3D
        else:
            inj = np.zeros(3)
            
        d_hat = zhat + K @ xhat_subset
        
        # xdot_subset = f(x) + B*u + E*d_hat + inj
        # Note: f_sub is f(x) only (u=0).
        Bu = B_sub @ u
        Ed = E_sub @ d_hat
        
        xdot_subset = f_sub + Bu + Ed + inj
        
        # zdot = -K*E*z - K*( f(x) + B*u + E*K*x )
        #      = -K*E*z - K*( xdot_subset_nominal ) where nominal excludes 'd' and 'inj'??
        # Wait, sample_obs_z:
        # zdot = -K D z - K ( f(x) + B u + D K x )
        # This comes from differentiation of z = d - K x, assuming d_dot = 0.
        # d_dot = zdot + K xdot ? No.
        # If d_hat = z + K x => z = d_hat - K x
        # zdot = d_hat_dot - K xdot - K_dot x
        # Ideally d_hat_dot = 0.
        # zdot = - K xdot 
        #      = - K ( f(x) + B u + E d_hat + inj )   <- if we include injection in state observer
        #      = - K ( f(x) + B u + E (z + K x) + inj )
        #      = - K f(x) - K B u - K E z - K E K x - K inj
        #
        # sample_obs_z implementation:
        # zdot = -K @ D @ zhat - K @ ( f(xhat) + B @ u + D @ (K @ xhat) )
        # It seems sample_obs_z IGNORES 'inj' in the z-dot equation? Or it assumes observer error dynamics separation?
        # Eq (11) in sample_obs_z likely assumes d_dot=0 and designs z to converge.
        # I will strictly follow sample_obs_z implementation line 145:
        # zdot = -self.K @ self.D @ zhat - self.K @ ( self.f(xhat) + self.B @ u + self.D @ (self.K @ xhat) )
        
        term1 = -K @ E_sub @ zhat
        
        # f(xhat) + B u + D(K xhat)
        # Here f(xhat) corresponds to f_sub
        term2_inner = f_sub + Bu + E_sub @ (K @ xhat_subset)
        term2 = -K @ term2_inner
        
        zdot = term1 + term2
        
        return xdot_subset, zdot

    def update(self, measurement: np.ndarray, control_input: np.ndarray,
               f_nn: Optional[np.ndarray] = None,
               acceleration: Optional[np.ndarray] = None,
               gps_available: bool = True) -> Tuple[np.ndarray, np.ndarray]:
        
        # 1. Unpack Inputs
        # measurement: [vx, r, psi, X, Y, ay] (standard 6D meas) based on FirstLayerObserverBase doc (actually check qlpv logic)
        # qlpv_vehicle_dynamics defines MEAS_idxs.
        # measurement is typically provided by the 'measurements' array in standard order.
        
        # We need r and vx for the Z-observer sample-and-hold
        r_meas = measurement[MEAS_IDX_R]
        vx_meas = measurement[MEAS_IDX_VX]
        
        # u: [delta, a]
        u = control_input
        
        # 2. Measurement Update (Sample-and-Hold)
        # This logic runs every step, assuming 'update' is called at the sample rate.
        # See plan: "Logic Flow in update: 1. Call internal on_measurement..."
        
        # Scheduling state (from estimates)
        vy_hat = self.state_hat[IDX_VY]
        vx_hat = self.state_hat[IDX_VX]
        # xhat_3d = [vy, r, vx]
        
        h = self._weights_h(vy_hat, r_meas, vx_hat)
        self.L_hold = self._L_from_h(h)
        
        # Calculate innovation y - C xhat
        # Z-observer C matrix for [vy, r, vx] state:
        # y = [r, vx]
        # C = [[0, 1, 0], [0, 0, 1]] maps [vy, r, vx] -> [r, vx]
        # This matches sample_obs_z line 21.
        
        y_meas_z = np.array([r_meas, vx_meas])
        
        xhat_subset = np.array([self.state_hat[IDX_VY], self.state_hat[IDX_R], self.state_hat[IDX_VX]])
        
        # C_sub equivalent
        # r is index 1, vx is index 2 of subset
        y_hat_z = np.array([xhat_subset[1], xhat_subset[2]])
        
        self.ey_hold = y_meas_z - y_hat_z
        self.has_sample = True
        
        # 3. Integration Step (RK4)
        dt = self.Ts
        
        # We need to integrate full 6D state, but Z-observer logic only drives 3D subset+z.
        # Other states (X, Y, psi) are driven by kinematics.
        
        # Helper wrapper for RK4
        def dyn(x_full, z_val):
            dx_sub, dz = self._rhs_z(x_full, z_val, u)
            
            # Reconstruct full dx_6d
            # dx_sub corresponds to [vy, r, vx]
            # We need to compute [vx_dot, vy_dot, psi_dot, r_dot, X_dot, Y_dot]
            
            # Start with full QLPV dynamics for kinematic consistency
            # But OVERWRITE the physical states (vx, vy, r) with the observer's corrected values (dx_sub)
            
            # Wait, _rhs_z returns xdot_subset INCLUDING injection.
            # So dx_sub is effectively the closed-loop observer dynamics for those states.
            
            # We need kinematics for psi, X, Y using the current x_full.
            # Let's get "open loop" kinematics
            x_dot_kin = self.dynamics.f_continuous(x_full, u, np.zeros(2)) # w=0
            
            # Construct result
            dx_res = np.zeros(6)
            
            # Overwrite with observed dynamics for subset
            dx_res[IDX_VX] = dx_sub[2]
            dx_res[IDX_VY] = dx_sub[0]
            dx_res[IDX_R]  = dx_sub[1]
            
            # Keep kinematics for others from model (using current x state)
            dx_res[IDX_PSI] = x_dot_kin[IDX_PSI] 
            dx_res[IDX_X]   = x_dot_kin[IDX_X]
            dx_res[IDX_Y]   = x_dot_kin[IDX_Y]
            
            return dx_res, dz

        # RK4 Integration
        x0 = self.state_hat.copy()
        z0 = self.zhat.copy()
        
        k1x, k1z = dyn(x0, z0)
        k2x, k2z = dyn(x0 + 0.5*dt*k1x, z0 + 0.5*dt*k1z)
        k3x, k3z = dyn(x0 + 0.5*dt*k2x, z0 + 0.5*dt*k2z)
        k4x, k4z = dyn(x0 + dt*k3x,     z0 + dt*k3z)
        
        self.state_hat = x0 + (dt/6.0)*(k1x + 2*k2x + 2*k3x + k4x)
        self.zhat = z0 + (dt/6.0)*(k1z + 2*k2z + 2*k3z + k4z)
        
        # 4. Unknown Input Estimation Reconstruction
        # d_hat = z + K x
        # d_hat corresponds to E * w. 
        # In sample_obs_z, d_hat IS the disturbance in equation: dot_x = ... + D*d_hat.
        # But D in sample_obs_z was defined as mapping w -> xdot.
        # So d_hat is an estimate of w (vector dimension 2).
        
        # Recalculate K and E at new state for output? Or intersection?
        # Usually done at current state.
        
        f_sub, _, E_sub = self._get_dynamics_matrices_for_z_scheme(self.state_hat, u)
        K = self.tau * E_sub.T
        
        xhat_subset = self.state_hat[[IDX_VY, IDX_R, IDX_VX]]
        
        # Disturbance estimate (tire residuals)
        w_hat = self.zhat + K @ xhat_subset
        
        self.f_uk_hat = w_hat
        
        return self.state_hat.copy(), self.f_uk_hat.copy()

