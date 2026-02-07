"""
Model Predictive Controller (MPC) using CasADi for Vehicle Control

Provides combined longitudinal and lateral control using MPC optimization.
Adapted from MAIN_MPC_car_general and support_files_car_general.

This controller returns both throttle and steering angle in a single computation.
"""
import numpy as np
import casadi as ca
from abc import ABC, abstractmethod
from typing import Optional, Dict, Any, Tuple


def wrap_to_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi) range."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


class MPCControllerBase(ABC):
    """Base class for MPC controllers"""
    
    @abstractmethod
    def compute_control(self, follower_state: Dict[str, float], 
                       leader_state: Optional[Dict[str, float]], 
                       dt: float) -> Tuple[float, float]:
        """
        Compute both throttle and steering commands
        
        Args:
            follower_state: Dict with keys 'x', 'y', 'theta', 'velocity', etc.
            leader_state: Dict with keys 'x', 'y', 'theta', 'velocity' (or None)
            dt: Time step
            
        Returns:
            Tuple of (throttle, steering_angle)
        """
        pass
    
    @abstractmethod
    def reset(self):
        """Reset controller state"""
        pass


class CasADiMPCController(MPCControllerBase):
    """
    MPC Controller using CasADi for nonlinear optimization.
    
    Uses a kinematic bicycle model for prediction and optimizes both
    throttle (acceleration) and steering angle simultaneously.
    
    State: [x, y, psi, v] - position, heading, velocity
    Input: [delta, a] - steering angle, acceleration
    
    IMPORTANT: Must provide vehicle parameters. No defaults.
    """
    
    def __init__(self, 
                 params,  # REQUIRED: VehicleParameters object or dict
                 
                 # MPC configuration (can be overridden by config object)
                 horizon: int = 10,
                 dt_mpc: float = 0.05,
                 
                 # Cost weights - Q (tracking) >> R (input) to prevent corner cutting
                 Q_pos: float = 500.0,     # Position tracking (high = tight path following)
                 Q_heading: float = 300.0,  # Heading tracking (high = align with path)  
                 Q_vel: float = 10.0,      # Velocity tracking
                 R_delta: float = 1.0,     # Steering penalty (low = allow steering freely)
                 R_acc: float = 1.0,       # Acceleration penalty
                 R_delta_rate: float = 3.0,    # Steering rate penalty (low = responsive)
                 R_acc_rate: float = 3.0,      # Acceleration rate penalty
                 
                 # Terminal cost weights (typically higher for stability)
                 Qf_pos: float = 500.0,
                 Qf_heading: float = 500.0,
                 Qf_vel: float = 10.0,
                 
                 # Constraints
                 max_steering_rate: float = 3.0,   # Max steering rate (rad/s)
                 
                 # Path following tuning
                 path_lookahead_scale: float = 0.4, # Compress ref lookahead (0.3=tight like Stanley, 1.0=full)
                 
                 # Following parameters
                 desired_spacing: float = 0.5,     # Desired following distance (m)
                 time_headway: float = 0.5,        # Time headway (s)
                 
                 # Solver options
                 solver_print_level: int = 0,
                 solver_max_iter: int = 100,
                 
                 # Config and logger
                 config=None,
                 logger=None):
        """
        Initialize CasADi MPC Controller.
        
        Args:
            params: VehicleParameters object or dictionary containing:
                   - a (float): Distance from CG to front axle (mapped to lf)
                   - b (float): Distance from CG to rear axle (mapped to lr)
                   - steering.max (float): Max steering angle
                   - longitudinal.a_max (float): Max acceleration
                   - longitudinal.v_max (float): Max velocity
            horizon: Prediction horizon (steps)
            dt_mpc: MPC time step (s)
            Q_*: State tracking weights
            R_*: Input weights
            Qf_*: Terminal state weights
            config: Optional config object
            logger: Logger instance
        """
        self.logger = logger
        
        # Extract vehicle parameters strictly
        if hasattr(params, 'a'): # It's likely a VehicleParameters object
            self.lf = params.a
            self.lr = params.b
            self.wheelbase = self.lf + self.lr
            self.max_steering = params.steering.max
            self.max_acceleration = params.longitudinal.a_max
            # Approximating max deceleration as max acceleration if not specified
            self.max_deceleration = params.longitudinal.a_max 
            self.max_velocity = params.longitudinal.v_max
            self.min_velocity = params.longitudinal.v_min
        elif isinstance(params, dict):
            # Try to handle dict input (fallback for legacy dicts, but prefer object)
            # This logic assumes the dict follows the yaml structure or flat structure
            self.lf = params.get('a', params.get('lf'))
            self.lr = params.get('b', params.get('lr'))
            if self.lf is None or self.lr is None:
                raise ValueError("Parameters must contain 'a' (lf) and 'b' (lr)")
            self.wheelbase = self.lf + self.lr
            
            # Extract nested if present, else flat
            steering = params.get('steering', {})
            longitudinal = params.get('longitudinal', {})
            
            self.max_steering = steering.get('max', params.get('max_steering', 0.5))
            self.max_acceleration = longitudinal.get('a_max', params.get('max_acceleration', 2.0))
            self.max_deceleration = params.get('max_deceleration', self.max_acceleration)
            self.max_velocity = longitudinal.get('v_max', params.get('max_velocity', 2.0))
            self.min_velocity = longitudinal.get('v_min', params.get('min_velocity', 0.0))
        else:
             raise TypeError("params must be a VehicleParameters object or a dictionary")

        # Config overrides for MPC tuning (not vehicle params)
        if config and hasattr(config, 'get_mpc_params'):
            mpc_cfg = config.get_mpc_params()
            self.N = mpc_cfg.get('horizon', horizon)
            self.dt = mpc_cfg.get('dt_mpc', dt_mpc)
            self.Q_pos = mpc_cfg.get('Q_pos', Q_pos)
            self.Q_heading = mpc_cfg.get('Q_heading', Q_heading)
            self.Q_vel = mpc_cfg.get('Q_vel', Q_vel)
            self.R_delta = mpc_cfg.get('R_delta', R_delta)
            self.R_acc = mpc_cfg.get('R_acc', R_acc)
            self.R_delta_rate = mpc_cfg.get('R_delta_rate', R_delta_rate)
            self.R_acc_rate = mpc_cfg.get('R_acc_rate', R_acc_rate)
            self.Qf_pos = mpc_cfg.get('Qf_pos', Qf_pos)
            self.Qf_heading = mpc_cfg.get('Qf_heading', Qf_heading)
            self.Qf_vel = mpc_cfg.get('Qf_vel', Qf_vel)
            self.max_steering_rate = mpc_cfg.get('max_steering_rate', max_steering_rate)
            self.path_lookahead_scale = mpc_cfg.get('path_lookahead_scale', path_lookahead_scale)
            self.desired_spacing = mpc_cfg.get('desired_spacing', desired_spacing)
            self.time_headway = mpc_cfg.get('time_headway', time_headway)
        else:
            self.N = horizon
            self.dt = dt_mpc
            self.Q_pos = Q_pos
            self.Q_heading = Q_heading
            self.Q_vel = Q_vel
            self.R_delta = R_delta
            self.R_acc = R_acc
            self.R_delta_rate = R_delta_rate
            self.R_acc_rate = R_acc_rate
            self.Qf_pos = Qf_pos
            self.Qf_heading = Qf_heading
            self.Qf_vel = Qf_vel
            self.max_steering_rate = max_steering_rate
            self.path_lookahead_scale = path_lookahead_scale
            self.desired_spacing = desired_spacing
            self.time_headway = time_headway
        
        self.solver_print_level = solver_print_level
        self.solver_max_iter = solver_max_iter
        
        # State dimensions
        self.nx = 4  # [x, y, psi, v]
        self.nu = 2  # [delta, a]
        
        # Previous control inputs (for rate constraints)
        self.prev_delta = 0.0
        self.prev_acc = 0.0
        
        # Warm start solution
        self.prev_solution = None
        
        # Waypoint path following support
        # Waypoints shape: [2, N] (row 0 = x, row 1 = y), same as StanleyController
        self.waypoints = None
        self.n_waypoints = 0
        self.wpi = 0  # Current waypoint index (like Stanley's wpi)
        self.cyclic = True  # Loop around waypoints
        
        # Build the MPC solver
        self._build_solver()
        
    def _build_solver(self):
        """Build the CasADi NLP solver for MPC."""
        
        # Define symbolic variables
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        psi = ca.SX.sym('psi')
        v = ca.SX.sym('v')
        
        states = ca.vertcat(x, y, psi, v)
        
        delta = ca.SX.sym('delta')  # Steering angle
        a = ca.SX.sym('a')          # Acceleration
        
        controls = ca.vertcat(delta, a)
        
        # Kinematic bicycle model (continuous time)
        # Using rear axle as reference point
        beta = ca.atan(self.lr / self.wheelbase * ca.tan(delta))  # Slip angle
        
        x_dot = v * ca.cos(psi + beta)
        y_dot = v * ca.sin(psi + beta)
        psi_dot = v / self.lr * ca.sin(beta)
        v_dot = a
        
        rhs = ca.vertcat(x_dot, y_dot, psi_dot, v_dot)
        
        # Create function for state dynamics
        self.f = ca.Function('f', [states, controls], [rhs])
        
        # Decision variables
        # States over horizon: X = [x0, x1, ..., x_N]
        # Controls over horizon: U = [u0, u1, ..., u_{N-1}]
        
        X = ca.SX.sym('X', self.nx, self.N + 1)
        U = ca.SX.sym('U', self.nu, self.N)
        
        # Parameters: initial state + reference trajectory + previous inputs
        # P = [x0, ref_0, ref_1, ..., ref_N, prev_delta, prev_acc]
        n_ref = self.nx * (self.N + 1)  # Reference for all states over horizon
        P = ca.SX.sym('P', self.nx + n_ref + 2)
        
        # Extract initial state
        x0 = P[0:self.nx]
        
        # Extract previous inputs
        prev_delta = P[-2]
        prev_acc = P[-1]
        
        # Cost function
        cost = 0
        
        # Equality constraints (dynamics)
        g = []
        
        # Initial state constraint
        g.append(X[:, 0] - x0)
        
        # Build cost and constraints over horizon
        # KEY: Decaying position weight — step k=0 gets full Q_pos, later steps
        # get less. This makes MPC prioritize being ON the path NOW (like Stanley)
        # rather than optimally smoothing through future curves (corner cutting).
        for k in range(self.N):
            # Extract reference for this step
            ref_idx = self.nx + k * self.nx
            ref_k = P[ref_idx:ref_idx + self.nx]
            
            # State error
            state_error = X[:, k] - ref_k
            
            # Exponentially decaying position weight: step 0 = full, step N = ~30%
            decay = 0.92 ** k  # e.g. k=0→1.0, k=5→0.66, k=10→0.43, k=15→0.29
            
            # Stage cost with decaying position weight
            cost += self.Q_pos * decay * (state_error[0]**2 + state_error[1]**2)
            # Heading: use (1 - cos(dpsi))^2 + sin(dpsi)^2 to handle wrapping
            dpsi = X[2, k] - ref_k[2]
            cost += self.Q_heading * decay * ((1 - ca.cos(dpsi))**2 + ca.sin(dpsi)**2)
            cost += self.Q_vel * state_error[3]**2  # Velocity (no decay)
            
            # Input cost
            cost += self.R_delta * U[0, k]**2
            cost += self.R_acc * U[1, k]**2
            
            # Input rate cost
            if k == 0:
                cost += self.R_delta_rate * (U[0, k] - prev_delta)**2
                cost += self.R_acc_rate * (U[1, k] - prev_acc)**2
            else:
                cost += self.R_delta_rate * (U[0, k] - U[0, k-1])**2
                cost += self.R_acc_rate * (U[1, k] - U[1, k-1])**2
            
            # Dynamics constraint (RK4 integration)
            k1 = self.f(X[:, k], U[:, k])
            k2 = self.f(X[:, k] + self.dt/2 * k1, U[:, k])
            k3 = self.f(X[:, k] + self.dt/2 * k2, U[:, k])
            k4 = self.f(X[:, k] + self.dt * k3, U[:, k])
            x_next = X[:, k] + self.dt/6 * (k1 + 2*k2 + 2*k3 + k4)
            
            g.append(X[:, k+1] - x_next)
        
        # Terminal cost
        ref_terminal = P[self.nx + self.N * self.nx:self.nx + (self.N + 1) * self.nx]
        terminal_error = X[:, self.N] - ref_terminal
        cost += self.Qf_pos * (terminal_error[0]**2 + terminal_error[1]**2)
        # Terminal heading: same wrapping-safe formulation
        dpsi_f = X[2, self.N] - ref_terminal[2]
        cost += self.Qf_heading * ((1 - ca.cos(dpsi_f))**2 + ca.sin(dpsi_f)**2)
        cost += self.Qf_vel * terminal_error[3]**2
        
        # Flatten decision variables
        opt_variables = ca.vertcat(
            ca.reshape(X, -1, 1),
            ca.reshape(U, -1, 1)
        )
        
        # Flatten constraints
        g = ca.vertcat(*g)
        
        # Define NLP
        nlp = {
            'x': opt_variables,
            'f': cost,
            'g': g,
            'p': P
        }
        
        # Solver options
        opts = {
            'ipopt': {
                'print_level': self.solver_print_level,
                'max_iter': self.solver_max_iter,
                'acceptable_tol': 1e-6,
                'acceptable_obj_change_tol': 1e-4,
                'warm_start_init_point': 'yes',
            },
            'print_time': 0
        }
        
        self.solver = ca.nlpsol('mpc_solver', 'ipopt', nlp, opts)
        
        # Define bounds
        self._setup_bounds()
        
    def _setup_bounds(self):
        """Setup variable bounds for the optimization."""
        
        n_states = self.nx * (self.N + 1)
        n_controls = self.nu * self.N
        n_vars = n_states + n_controls
        
        # Variable bounds
        self.lbx = np.full(n_vars, -np.inf)
        self.ubx = np.full(n_vars, np.inf)
        
        # State bounds
        for k in range(self.N + 1):
            idx = k * self.nx
            # x, y: unbounded (handled by large values)
            self.lbx[idx] = -1e6      # x
            self.ubx[idx] = 1e6
            self.lbx[idx + 1] = -1e6  # y
            self.ubx[idx + 1] = 1e6
            self.lbx[idx + 2] = -1e6  # psi (unbounded)
            self.ubx[idx + 2] = 1e6
            self.lbx[idx + 3] = self.min_velocity  # v
            self.ubx[idx + 3] = self.max_velocity
        
        # Control bounds
        for k in range(self.N):
            idx = n_states + k * self.nu
            self.lbx[idx] = -self.max_steering      # delta
            self.ubx[idx] = self.max_steering
            self.lbx[idx + 1] = -self.max_deceleration  # a
            self.ubx[idx + 1] = self.max_acceleration
        
        # Constraint bounds (all equality constraints = 0)
        n_constraints = self.nx * (self.N + 1)
        self.lbg = np.zeros(n_constraints)
        self.ubg = np.zeros(n_constraints)
        
    # ======================== Waypoint Management ========================
    
    def set_waypoints(self, waypoints: np.ndarray, cyclic: bool = True):
        """
        Set waypoints for path following mode.
        
        Waypoints format matches StanleyController: shape [2, N]
        Row 0 = x coordinates, Row 1 = y coordinates.
        
        Args:
            waypoints: np.ndarray of shape [2, N]
            cyclic: Whether to loop around when reaching the end
        """
        self.waypoints = waypoints
        self.n_waypoints = waypoints.shape[1] if waypoints is not None else 0
        self.cyclic = cyclic
        self.wpi = 0
        
        if self.logger:
            self.logger.info(f"[MPC] Waypoints set: {self.n_waypoints} points, cyclic={cyclic}")
    
    def get_waypoint_index(self) -> int:
        """Get current waypoint index (same interface as StanleyController)."""
        return self.wpi
    
    def _find_closest_waypoint(self, x: float, y: float):
        """
        Advance waypoint index using LOCAL forward-walk (like Stanley).
        
        NEVER does global search — that causes path-cutting on oval/loop paths
        where a distant part of the path is geometrically close.
        
        Strategy: Walk forward from current wpi, advancing when we pass
        each segment. Checks multiple segments ahead to handle fast movement.
        Same logic as StanleyController.update() but checks a few more steps.
        """
        if self.waypoints is None or self.n_waypoints < 2:
            return
        
        p = np.array([x, y])
        
        # Forward-walk from current wpi (like Stanley, but check more segments)
        # At ~0.7 m/s with ~3ms waypoint spacing, we move ~5 waypoints per MPC call
        max_advance = 15  # Check up to 15 segments ahead
        
        for _ in range(max_advance):
            idx = self.wpi % (self.n_waypoints - 1) if self.cyclic else min(self.wpi, self.n_waypoints - 2)
            wp_1 = self.waypoints[:, idx]
            wp_2 = self.waypoints[:, (idx + 1) % self.n_waypoints]
            
            v = wp_2 - wp_1
            v_mag = np.linalg.norm(v)
            if v_mag < 1e-6:
                # Zero-length segment, skip it
                if self.cyclic or self.wpi < self.n_waypoints - 2:
                    self.wpi += 1
                else:
                    break
                continue
            
            v_uv = v / v_mag
            s = np.dot(p - wp_1, v_uv)
            
            # Only advance if we've PASSED this segment
            if s >= v_mag:
                if self.cyclic or self.wpi < self.n_waypoints - 2:
                    self.wpi += 1
                else:
                    break
            else:
                break
        
        # Handle cyclic wrap-around
        if self.cyclic and self.wpi >= self.n_waypoints - 1:
            self.wpi = self.wpi % (self.n_waypoints - 1)
    
    def _get_waypoint_reference(self, x: float, y: float, 
                                 target_velocity: float) -> np.ndarray:
        """
        Generate MPC reference trajectory from waypoints.
        
        Like Stanley, projects position onto the current segment to get the
        closest point on path. The first reference point IS that closest point
        (giving MPC a "come back to path" target, like Stanley's cross-track error).
        Then walks forward along waypoints, spacing reference points by
        (target_velocity * dt_mpc) meters.
        
        Args:
            x, y: Current vehicle position
            target_velocity: Desired speed along the path
            
        Returns:
            Reference trajectory [N+1, 4] with [x_ref, y_ref, psi_ref, v_ref]
        """
        ref = np.zeros((self.N + 1, self.nx))
        
        # Update closest waypoint (forward-walk only, never jumps)
        self._find_closest_waypoint(x, y)
        
        # Step size along the path per MPC time step
        # path_lookahead_scale < 1.0 compresses the lookahead: reference points
        # stay closer to the vehicle, so MPC sees less of upcoming curves
        # and doesn't try to cut corners. 0.4 = reference covers 40% of the
        # distance the vehicle actually travels per horizon step.
        step_dist = max(target_velocity, 0.1) * self.dt * self.path_lookahead_scale
        
        # Start walking from current segment
        wi = self.wpi % (self.n_waypoints - 1) if self.cyclic else min(self.wpi, self.n_waypoints - 2)
        wp_1 = self.waypoints[:, wi].copy()
        wp_2 = self.waypoints[:, (wi + 1) % self.n_waypoints].copy()
        
        seg_vec = wp_2 - wp_1
        seg_len = np.linalg.norm(seg_vec)
        if seg_len < 1e-6:
            seg_dir = np.array([np.cos(0.0), np.sin(0.0)])
            seg_len = 1e-6
        else:
            seg_dir = seg_vec / seg_len
        
        # Project current position onto segment (like Stanley does)
        p = np.array([x, y])
        s_on_seg = np.dot(p - wp_1, seg_dir)
        s_on_seg = np.clip(s_on_seg, 0.0, seg_len)
        
        # ref[0] = closest point on path (perpendicular projection)
        # This is KEY: it tells MPC "go to this point on the path first"
        # Same effect as Stanley's cross-track error
        closest_on_path = wp_1 + seg_dir * s_on_seg
        ref_heading = np.arctan2(seg_dir[1], seg_dir[0])
        
        ref[0, 0] = closest_on_path[0]
        ref[0, 1] = closest_on_path[1]
        ref[0, 2] = ref_heading
        ref[0, 3] = target_velocity
        
        # ref[1..N] = lookahead points along the path
        for k in range(1, self.N + 1):
            # Advance along path by step_dist
            s_on_seg += step_dist
            
            # Walk to next segments as needed
            while s_on_seg > seg_len:
                s_on_seg -= seg_len
                wi += 1
                
                if wi >= self.n_waypoints - 1:
                    if self.cyclic:
                        wi = 0
                    else:
                        # Clamp to last waypoint, set velocity=0 (stop)
                        ref_point = self.waypoints[:, -1]
                        for kk in range(k, self.N + 1):
                            ref[kk, 0] = ref_point[0]
                            ref[kk, 1] = ref_point[1]
                            ref[kk, 2] = ref_heading
                            ref[kk, 3] = 0.0
                        return ref
                
                wp_1 = self.waypoints[:, wi].copy()
                wp_2 = self.waypoints[:, (wi + 1) % self.n_waypoints].copy()
                seg_vec = wp_2 - wp_1
                seg_len = np.linalg.norm(seg_vec)
                if seg_len < 1e-6:
                    seg_len = 1e-6
                else:
                    seg_dir = seg_vec / seg_len
            
            # Compute reference point on this segment
            ref_point = wp_1 + seg_dir * s_on_seg
            ref_heading = np.arctan2(seg_dir[1], seg_dir[0])
            
            ref[k, 0] = ref_point[0]
            ref[k, 1] = ref_point[1]
            ref[k, 2] = ref_heading
            ref[k, 3] = target_velocity
        
        return ref
    
    # ======================== Reference Generation ========================
    
    def _generate_reference_trajectory(self, 
                                        current_state: np.ndarray,
                                        leader_state: Optional[Dict[str, float]],
                                        target_velocity: float) -> np.ndarray:
        """
        Generate reference trajectory for MPC.
        
        Modes:
        1. LEADER FOLLOWING: leader_state is provided → track behind leader
        2. PATH FOLLOWING:   waypoints are set → follow waypoint path
        3. STRAIGHT LINE:    no waypoints, no leader → maintain heading (fallback)
        
        Args:
            current_state: Current [x, y, psi, v]
            leader_state: Leader state dict (or None)
            target_velocity: Target velocity
            
        Returns:
            Reference trajectory [N+1, 4]
        """
        ref = np.zeros((self.N + 1, self.nx))
        
        if leader_state is not None:
            # ---- MODE 1: Leader Following ----
            leader_x = leader_state['x']
            leader_y = leader_state['y']
            leader_theta = leader_state['theta']
            leader_v = leader_state.get('velocity', target_velocity)
            
            # Desired following position: behind leader
            spacing = self.desired_spacing + self.time_headway * current_state[3]
            
            for k in range(self.N + 1):
                t_pred = k * self.dt
                
                # Predict leader position (constant velocity + heading)
                pred_leader_x = leader_x + leader_v * np.cos(leader_theta) * t_pred
                pred_leader_y = leader_y + leader_v * np.sin(leader_theta) * t_pred
                
                ref[k, 0] = pred_leader_x - spacing * np.cos(leader_theta)
                ref[k, 1] = pred_leader_y - spacing * np.sin(leader_theta)
                ref[k, 2] = leader_theta
                ref[k, 3] = leader_v
                
        elif self.waypoints is not None and self.n_waypoints >= 2:
            # ---- MODE 2: Waypoint Path Following ----
            ref = self._get_waypoint_reference(
                current_state[0], current_state[1], target_velocity
            )
            
        else:
            # ---- MODE 3: Straight line fallback ----
            x, y, psi, v = current_state
            for k in range(self.N + 1):
                t_pred = k * self.dt
                ref[k, 0] = x + target_velocity * np.cos(psi) * t_pred
                ref[k, 1] = y + target_velocity * np.sin(psi) * t_pred
                ref[k, 2] = psi
                ref[k, 3] = target_velocity
        
        return ref
    
    def compute_control(self, follower_state: Dict[str, float], 
                       leader_state: Optional[Dict[str, float]], 
                       dt: float) -> Tuple[float, float]:
        """
        Compute both throttle and steering using MPC.
        
        Args:
            follower_state: Dict with keys 'x', 'y', 'theta', 'velocity'
            leader_state: Dict with keys 'x', 'y', 'theta', 'velocity' (or None)
            dt: Time step (used for rate limiting if different from MPC dt)
            
        Returns:
            Tuple of (throttle, steering_angle)
        """
        # Extract current state
        x = follower_state['x']
        y = follower_state['y']
        psi = follower_state['theta']
        v = follower_state['velocity']
        
        current_state = np.array([x, y, psi, v])
        
        # Get target velocity
        target_velocity = follower_state.get('target_velocity', 0.5)
        
        # Generate reference trajectory
        ref = self._generate_reference_trajectory(current_state, leader_state, target_velocity)
        
        # Build parameter vector
        p = np.zeros(self.nx + self.nx * (self.N + 1) + 2)
        p[0:self.nx] = current_state
        p[self.nx:self.nx + self.nx * (self.N + 1)] = ref.flatten()
        p[-2] = self.prev_delta
        p[-1] = self.prev_acc
        
        # Initial guess (warm start)
        if self.prev_solution is not None:
            x0 = self.prev_solution
        else:
            # Cold start: straight line trajectory with zero inputs
            x0 = np.zeros(self.nx * (self.N + 1) + self.nu * self.N)
            for k in range(self.N + 1):
                x0[k * self.nx:(k + 1) * self.nx] = current_state
        
        # Solve NLP
        try:
            sol = self.solver(
                x0=x0,
                lbx=self.lbx,
                ubx=self.ubx,
                lbg=self.lbg,
                ubg=self.ubg,
                p=p
            )
            
            # Extract solution
            opt_sol = sol['x'].full().flatten()
            
            # Save for warm start
            self.prev_solution = opt_sol
            
            # Extract first control input
            n_states = self.nx * (self.N + 1)
            delta_opt = opt_sol[n_states]      # First steering
            acc_opt = opt_sol[n_states + 1]    # First acceleration
            
            # Apply rate limiting (soft: only clamp extreme jumps)
            # Use actual dt for rate calc, not MPC dt, since control runs faster
            effective_dt = max(dt, self.dt)
            delta_rate = (delta_opt - self.prev_delta) / effective_dt
            if abs(delta_rate) > self.max_steering_rate:
                delta_opt = self.prev_delta + np.sign(delta_rate) * self.max_steering_rate * effective_dt
            
            # Update previous inputs
            self.prev_delta = delta_opt
            self.prev_acc = acc_opt
            
            # Convert acceleration to throttle (simple linear mapping)
            # Throttle range: [-1, 1] where negative is braking
            throttle = self._acc_to_throttle(acc_opt)
            
            return throttle, delta_opt
            
        except Exception as e:
            if self.logger:
                self.logger.warning(f"MPC solver failed: {e}")
            # Return safe values
            return 0.0, 0.0
    
    def _acc_to_throttle(self, acc: float) -> float:
        """
        Convert acceleration command to throttle.
        
        Simple linear mapping:
        - Positive acceleration -> positive throttle
        - Negative acceleration -> negative throttle (braking)
        
        Args:
            acc: Acceleration command (m/s^2)
            
        Returns:
            Throttle command in range [-1, 1]
        """
        # Normalize by max acceleration/deceleration
        if acc >= 0:
            throttle = acc / self.max_acceleration
        else:
            throttle = acc / self.max_deceleration
        
        return np.clip(throttle, -1.0, 1.0)
    
    def reset(self):
        """Reset controller state."""
        self.prev_delta = 0.0
        self.prev_acc = 0.0
        self.prev_solution = None
        self.wpi = 0


class DynamicBicycleMPCController(CasADiMPCController):
    """
    MPC Controller using dynamic bicycle model.
    
    Extends the kinematic model with tire forces for better accuracy
    at higher speeds. Adapted from support_files_car_general.py.
    
    State: [x_dot, y_dot, psi, psi_dot, X, Y] - body velocities, heading, yaw rate, position
    Input: [delta, a] - steering angle, acceleration
    
    IMPORTANT: Must provide vehicle parameters. No defaults.
    """
    
    def __init__(self,
                 params, # REQUIRED: VehicleParameters object
                 
                 # MPC parameters
                 horizon: int = 10,
                 dt_mpc: float = 0.02,
                 
                 # Cost weights
                 Q_vx: float = 1.0,
                 Q_psi: float = 200.0,
                 Q_X: float = 50.0,
                 Q_Y: float = 50.0,
                 R_delta: float = 100.0,
                 R_acc: float = 1.0,
                 
                 config=None,
                 logger=None):
        """Initialize Dynamic Bicycle MPC Controller."""
        
        self.logger = logger
        
        # Extract vehicle parameters strictly
        if hasattr(params, 'm'):
            self.mass = params.m
            self.Iz = params.I_z
            self.lf = params.a
            self.lr = params.b
            self.mu = params.mu_road
            
            # Tire stiffness
            # Note: parameters_qcar might have normalized stiffness, check this
            # QCar params usually have Cf, Cr directly or Csf, Csr
            if hasattr(params, 'Cf') and params.Cf is not None:
                self.Cf = params.Cf
                self.Cr = params.Cr
            else:
                 # Fallback/Calculation based on Csf if available or raise error
                 # For now assuming straightforward availability as per parameters_qcar.yaml content seen earlier
                 raise ValueError("Vehicle parameters must include Cf and Cr")
                 
            self.max_steering = params.steering.max
            self.max_acceleration = params.longitudinal.a_max
            self.max_velocity = params.longitudinal.v_max
            
        else:
             raise TypeError("params must be a VehicleParameters object")

        self.g = 9.81
        
        self.N = horizon
        self.dt = dt_mpc
        
        self.Q_vx = Q_vx
        self.Q_psi = Q_psi
        self.Q_X = Q_X
        self.Q_Y = Q_Y
        self.R_delta = R_delta
        self.R_acc = R_acc
        
        # State dimensions
        self.nx = 6  # [x_dot, y_dot, psi, psi_dot, X, Y]
        self.nu = 2  # [delta, a]
        
        # Previous inputs
        self.prev_delta = 0.0
        self.prev_acc = 0.0
        self.prev_solution = None
        
        self._build_dynamic_solver()
        
    def _build_dynamic_solver(self):
        """Build MPC solver with dynamic bicycle model."""
        
        # State variables
        x_dot = ca.SX.sym('x_dot')  # Longitudinal velocity (body frame)
        y_dot = ca.SX.sym('y_dot')  # Lateral velocity (body frame)
        psi = ca.SX.sym('psi')       # Yaw angle
        psi_dot = ca.SX.sym('psi_dot')  # Yaw rate
        X = ca.SX.sym('X')           # Global X position
        Y = ca.SX.sym('Y')           # Global Y position
        
        states = ca.vertcat(x_dot, y_dot, psi, psi_dot, X, Y)
        
        # Control inputs
        delta = ca.SX.sym('delta')  # Steering angle
        a = ca.SX.sym('a')          # Acceleration
        
        controls = ca.vertcat(delta, a)
        
        # Dynamic bicycle model
        # Tire forces (linear tire model)
        alpha_f = delta - (y_dot + self.lf * psi_dot) / (x_dot + 1e-6)
        alpha_r = -(y_dot - self.lr * psi_dot) / (x_dot + 1e-6)
        
        Fyf = self.Cf * alpha_f
        Fyr = self.Cr * alpha_r
        
        # State derivatives
        x_dot_dot = a + (-Fyf * ca.sin(delta) - self.mu * self.mass * self.g) / self.mass + psi_dot * y_dot
        y_dot_dot = (Fyf * ca.cos(delta) + Fyr) / self.mass - psi_dot * x_dot
        psi_dot_out = psi_dot
        psi_dot_dot = (Fyf * self.lf * ca.cos(delta) - Fyr * self.lr) / self.Iz
        X_dot = x_dot * ca.cos(psi) - y_dot * ca.sin(psi)
        Y_dot = x_dot * ca.sin(psi) + y_dot * ca.cos(psi)
        
        rhs = ca.vertcat(x_dot_dot, y_dot_dot, psi_dot_out, psi_dot_dot, X_dot, Y_dot)
        
        self.f = ca.Function('f', [states, controls], [rhs])
        
        # Decision variables
        X_opt = ca.SX.sym('X', self.nx, self.N + 1)
        U_opt = ca.SX.sym('U', self.nu, self.N)
        
        # Parameters: initial state + reference (simplified: target velocity, heading, position)
        # P = [x0(6), ref_vx, ref_psi, ref_X, ref_Y]
        P = ca.SX.sym('P', self.nx + 4)
        
        x0 = P[0:self.nx]
        ref_vx = P[self.nx]
        ref_psi = P[self.nx + 1]
        ref_X = P[self.nx + 2]
        ref_Y = P[self.nx + 3]
        
        # Cost and constraints
        cost = 0
        g = [X_opt[:, 0] - x0]
        
        for k in range(self.N):
            # State error (tracking velocity, heading, position)
            cost += self.Q_vx * (X_opt[0, k] - ref_vx)**2
            cost += self.Q_psi * (X_opt[2, k] - ref_psi)**2
            cost += self.Q_X * (X_opt[4, k] - ref_X)**2
            cost += self.Q_Y * (X_opt[5, k] - ref_Y)**2
            
            # Input cost
            cost += self.R_delta * U_opt[0, k]**2
            cost += self.R_acc * U_opt[1, k]**2
            
            # Dynamics (Euler integration for simplicity)
            x_next = X_opt[:, k] + self.dt * self.f(X_opt[:, k], U_opt[:, k])
            g.append(X_opt[:, k+1] - x_next)
        
        # Terminal cost
        cost += self.Q_vx * (X_opt[0, self.N] - ref_vx)**2
        cost += self.Q_psi * (X_opt[2, self.N] - ref_psi)**2
        
        # Flatten
        opt_vars = ca.vertcat(ca.reshape(X_opt, -1, 1), ca.reshape(U_opt, -1, 1))
        g = ca.vertcat(*g)
        
        nlp = {'x': opt_vars, 'f': cost, 'g': g, 'p': P}
        
        opts = {
            'ipopt': {
                'print_level': 0,
                'max_iter': 100,
                'warm_start_init_point': 'yes',
            },
            'print_time': 0
        }
        
        self.solver = ca.nlpsol('mpc_dynamic', 'ipopt', nlp, opts)
        self._setup_dynamic_bounds()
        
    def _setup_dynamic_bounds(self):
        """Setup bounds for dynamic model."""
        
        n_states = self.nx * (self.N + 1)
        n_controls = self.nu * self.N
        n_vars = n_states + n_controls
        
        self.lbx = np.full(n_vars, -np.inf)
        self.ubx = np.full(n_vars, np.inf)
        
        # State bounds
        for k in range(self.N + 1):
            idx = k * self.nx
            self.lbx[idx] = 0.1  # x_dot (min velocity to avoid singularity)
            self.ubx[idx] = self.max_velocity
            self.lbx[idx + 1] = -10  # y_dot
            self.ubx[idx + 1] = 10
            # psi, psi_dot, X, Y: unbounded
        
        # Control bounds
        for k in range(self.N):
            idx = n_states + k * self.nu
            self.lbx[idx] = -self.max_steering
            self.ubx[idx] = self.max_steering
            self.lbx[idx + 1] = -self.max_acceleration
            self.ubx[idx + 1] = self.max_acceleration
        
        n_constraints = self.nx * (self.N + 1)
        self.lbg = np.zeros(n_constraints)
        self.ubg = np.zeros(n_constraints)
        
    def compute_control(self, follower_state: Dict[str, float], 
                       leader_state: Optional[Dict[str, float]], 
                       dt: float) -> Tuple[float, float]:
        """Compute control using dynamic bicycle model MPC."""
        
        # Extract state (convert from simple state to dynamic state)
        x = follower_state['x']
        y = follower_state['y']
        theta = follower_state['theta']
        v = max(follower_state['velocity'], 0.1)  # Avoid zero velocity
        
        # Assume y_dot and psi_dot are small if not provided
        y_dot = follower_state.get('y_dot', 0.0)
        psi_dot = follower_state.get('psi_dot', 0.0)
        
        current_state = np.array([v, y_dot, theta, psi_dot, x, y])
        
        # Reference
        target_velocity = follower_state.get('target_velocity', v)
        
        if leader_state is not None:
            ref_psi = leader_state['theta']
            ref_X = leader_state['x']
            ref_Y = leader_state['y']
        else:
            ref_psi = theta
            ref_X = x + target_velocity * np.cos(theta) * self.N * self.dt
            ref_Y = y + target_velocity * np.sin(theta) * self.N * self.dt
        
        # Parameters
        p = np.concatenate([current_state, [target_velocity, ref_psi, ref_X, ref_Y]])
        
        # Initial guess
        if self.prev_solution is not None:
            x0 = self.prev_solution
        else:
            x0 = np.zeros(self.nx * (self.N + 1) + self.nu * self.N)
            for k in range(self.N + 1):
                x0[k * self.nx:(k + 1) * self.nx] = current_state
        
        try:
            sol = self.solver(x0=x0, lbx=self.lbx, ubx=self.ubx,
                             lbg=self.lbg, ubg=self.ubg, p=p)
            
            opt_sol = sol['x'].full().flatten()
            self.prev_solution = opt_sol
            
            n_states = self.nx * (self.N + 1)
            delta_opt = opt_sol[n_states]
            acc_opt = opt_sol[n_states + 1]
            
            self.prev_delta = delta_opt
            self.prev_acc = acc_opt
            
            throttle = np.clip(acc_opt / self.max_acceleration, -1.0, 1.0)
            
            return throttle, delta_opt
            
        except Exception as e:
            if self.logger:
                self.logger.warning(f"Dynamic MPC solver failed: {e}")
            return 0.0, 0.0
    
    def reset(self):
        """Reset controller state."""
        self.prev_delta = 0.0
        self.prev_acc = 0.0
        self.prev_solution = None


# Robust import: try multiple paths for vehicle_parameters
try:
    from qcar.simulation.vehiclemodels.vehicle_parameters import get_qcar_parameters
except ImportError:
    try:
        import sys as _sys, os as _os
        _sim_dir = _os.path.join(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))), 'simulation')
        if _sim_dir not in _sys.path:
            _sys.path.insert(0, _sim_dir)
        from vehiclemodels.vehicle_parameters import get_qcar_parameters
    except ImportError:
        get_qcar_parameters = None


class MPCControllerFactory:
    """Factory to create MPC controllers by name."""
    
    CONTROLLER_TYPES = {
        'casadi_kinematic': CasADiMPCController,
        'casadi_dynamic': DynamicBicycleMPCController,
        'mpc': CasADiMPCController,  # Default alias
    }
    
    @staticmethod
    def create(controller_type: str, params: Dict[str, Any] = None, logger=None):
        """
        Create an MPC controller by type name.
        
        Args:
            controller_type: Type of controller ('casadi_kinematic', 'casadi_dynamic', 'mpc')
            params: MPC configuration dictionary (passed as kwargs to controller)
                   Should optionally contain 'params' key for vehicle parameters.
                   If 'params' key is missing, defaults to loading QCar parameters.
            logger: Logger instance
            
        Returns:
            MPC controller instance
        """
        if controller_type not in MPCControllerFactory.CONTROLLER_TYPES:
            raise ValueError(f"Unknown MPC controller type: {controller_type}. "
                           f"Available: {list(MPCControllerFactory.CONTROLLER_TYPES.keys())}")
        
        controller_class = MPCControllerFactory.CONTROLLER_TYPES[controller_type]
        mpc_config = params or {}
        
        # Check if vehicle parameters are provided in the config
        # The __init__ expects 'params' as the first argument (or named argument)
        if 'params' not in mpc_config:
            # Load default QCar parameters
            try:
                vehicle_params = get_qcar_parameters()
                # Inject vehicle parameters into the usage
                # We need to pass it explicitly to __init__
                # Construct arguments: pass 'params' as positional or keyword
                # Here we add it to kwargs
                mpc_config['params'] = vehicle_params
            except Exception as e:
                # If loading fails (e.g. file not found), we can't proceed
                if logger:
                    logger.error(f"Failed to load default QCar parameters: {e}")
                raise ValueError(f"Vehicle parameters not provided and failed to load defaults: {e}")
        
        return controller_class(logger=logger, **mpc_config)
