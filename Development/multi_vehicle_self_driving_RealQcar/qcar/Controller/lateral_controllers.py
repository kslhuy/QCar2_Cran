"""
Lateral Controllers for Vehicle Following

Provides different lateral control strategies with a common interface.
Easy to switch between different controllers.
"""
import numpy as np
import math
from abc import ABC, abstractmethod
from typing import Optional, Dict, Any
from threading import Lock


def wrap_to_pi(angle: float) -> float:
    """
    Wrap angle to [-pi, pi) range.
    
    This is a corrected version that properly handles all angles.
    Uses the standard normalization formula: (angle + pi) % (2*pi) - pi
    
    Args:
        angle: Angle in radians
        
    Returns:
        Wrapped angle in range [-pi, pi)
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


class LateralControllerBase(ABC):
    """Base class for all lateral controllers"""
    
    @abstractmethod
    def compute_steering(self, follower_state: Dict[str, float], 
                        leader_state: Optional[Dict[str, float]], 
                        dt: float) -> float:
        """
        Compute steering command
        
        Args:
            follower_state: Dict with keys 'x', 'y', 'theta', 'velocity'
            leader_state: Dict with keys 'x', 'y', 'theta', 'velocity' (or None)
            dt: Time step
            
        Returns:
            Steering angle command (radians)
        """
        pass
    
    @abstractmethod
    def reset(self):
        """Reset controller state"""
        pass


class PurePursuitController(LateralControllerBase):
    """
    Pure Pursuit lateral controller
    Tracks a point ahead of the leader vehicle
    """
    
    def __init__(self, lookahead_distance=1.0, k_steering=1.0, 
                 max_steering=0.55, adaptive_lookahead=True, config=None, logger=None):
        """
        Initialize Pure Pursuit controller
        
        Args:
            lookahead_distance: Base lookahead distance (meters)
            k_steering: Proportional steering gain
            max_steering: Maximum steering angle (radians)
            adaptive_lookahead: Enable adaptive lookahead based on distance to leader
            config: Optional config object (takes precedence)
            logger: Logger instance
        """
        self.logger = logger
        
        # Use config if provided
        if config and hasattr(config, 'get_lateral_params'):
            # ControllerConfig - get params from dictionary
            params = config.get_lateral_params('pure_pursuit')
            self.lookahead_distance = params.get('lookahead_distance', lookahead_distance)
            self.k_steering = params.get('k_steering', k_steering)
            self.max_steering = params.get('max_steering', max_steering)
            self.adaptive_lookahead = params.get('adaptive_lookahead', adaptive_lookahead)
        else:
            self.lookahead_distance = lookahead_distance
            self.k_steering = k_steering
            self.max_steering = max_steering
            self.adaptive_lookahead = adaptive_lookahead
        
    def compute_steering(self, follower_state: Dict[str, float], 
                        leader_state: Optional[Dict[str, float]], 
                        dt: float) -> float:
        """
        Compute steering using pure pursuit
        
        Pure pursuit computes steering to track a point ahead of the leader
        in the leader's heading direction.
        """
        if leader_state is None:
            # No leader - maintain current heading
            return 0.0
        
        # Extract states
        x = follower_state['x']
        y = follower_state['y']
        theta = follower_state['theta']
        
        x_j = leader_state['x']
        y_j = leader_state['y']
        theta_j = leader_state['theta']
        
        # Compute distance to leader
        dx_to_leader = x_j - x
        dy_to_leader = y_j - y
        distance_to_leader = math.sqrt(dx_to_leader**2 + dy_to_leader**2)
        
        # Adaptive lookahead: smaller when close to leader
        if self.adaptive_lookahead:
            lookahead = min(self.lookahead_distance, max(0.2, distance_to_leader * 0.5))
        else:
            lookahead = self.lookahead_distance

        
        # Target point ahead of leader in its heading direction
        # Use PLUS to place target ahead of leader 
        # Use MINUS to place target behind leader 
        target_x = x_j - lookahead * math.cos(theta_j)
        target_y = y_j - lookahead * math.sin(theta_j)
        
        # Compute heading error to target point
        dx = target_x - x
        dy = target_y - y
        target_angle = math.atan2(dy, dx)
        heading_error = wrap_to_pi(target_angle - theta)
        
        # Apply proportional control
        steering_cmd = self.k_steering * heading_error
        
        # Clamp to limits
        steering_cmd = np.clip(steering_cmd, -self.max_steering, self.max_steering)
        
        return steering_cmd
    
    def reset(self):
        """Reset controller state (no state to reset for pure pursuit)"""
        pass


class StanleyController(LateralControllerBase):
    """
    Stanley lateral controller for path following
    Combines heading error and cross-track error
    Compatible with both platoon following and standalone path following
    """
    
    def __init__(self, waypoints: np.ndarray = None, k_e=0.5, k_soft=1.0, 
                 max_steering=0.55, config=None, logger=None, cyclic: bool = True):
        """
        Initialize Stanley controller
        
        Args:
            waypoints: Waypoint array [2, N] for path following mode
            k_e: Cross-track error gain (if config not provided)
            k_soft: Softening gain to prevent division by zero
            max_steering: Maximum steering angle (radians)
            config: Optional config object (takes precedence)
            logger: Logger instance
            cyclic: Whether to cycle through waypoints
        """
        self.logger = logger
        
        # Use config if provided
        if config and hasattr(config, 'get_lateral_params'):
            # ControllerConfig - get params from dictionary
            params = config.get_lateral_params('stanley')
            self.k = params.get('k_e', k_e)
            self.max_steering_angle = params.get('max_steering', max_steering)
        else:
            self.k = k_e
            self.max_steering_angle = max_steering
        
        # Path following attributes
        self.wp = waypoints
        self.N = len(waypoints[0, :]) if waypoints is not None else 0
        self.wpi = 0
        self.cyclic = cyclic
        
        # Reference values for telemetry
        self.p_ref = np.array([0.0, 0.0])
        self.th_ref = 0.0
        
        # Errors
        self.cross_track_error = 0.0
        self.heading_error = 0.0
        
        # Softening constant for low speeds
        self.k_soft = k_soft
        
        # Thread safety
        self._lock = Lock()
    
    def update(self, p: np.ndarray, th: float, speed: float) -> float:
        """
        Update steering controller for path following
        
        Args:
            p: Current position [x, y]
            th: Current heading
            speed: Current speed
            
        Returns:
            Steering angle command
        """
        if self.wp is None:
            return 0.0
            
        with self._lock:
            # Get current waypoints
            wp_1 = self.wp[:, np.mod(self.wpi, self.N - 1)]
            wp_2 = self.wp[:, np.mod(self.wpi + 1, self.N - 1)]
            
            # Path vector
            v = wp_2 - wp_1
            v_mag = np.linalg.norm(v)
            
            # Handle zero-length segment (inspired by control.py)
            try:
                v_uv = v / v_mag
            except (ZeroDivisionError):
                return 0
            
            # Path tangent angle
            tangent = np.arctan2(v_uv[1], v_uv[0])
            
            # Progress along current segment
            s = np.dot(p - wp_1, v_uv)
            
            # Check if we should advance to next waypoint
            if s >= v_mag:
                if self.cyclic or self.wpi < self.N - 2:
                    self.wpi += 1
            
            # Closest point on path
            ep = wp_1 + v_uv * s
            
            # Cross-track error vector
            ct = ep - p
            
            # Direction of cross-track error
            dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)
            
            # Signed cross-track error
            ect = np.linalg.norm(ct) * np.sign(dir)
            
            # Heading error
            psi = wrap_to_pi(tangent - th)
            
            # Store for telemetry
            self.p_ref = ep
            self.th_ref = tangent
            self.cross_track_error = ect
            self.heading_error = psi
            
            # Stanley control law (matching control.py implementation)
            return np.clip(
                wrap_to_pi(psi + np.arctan2(self.k * ect, speed)),
                -self.max_steering_angle,
                self.max_steering_angle
            )
    
    def compute_steering(self, follower_state: Dict[str, float], 
                        leader_state: Optional[Dict[str, float]], 
                        dt: float) -> float:
        """
        Compute steering using Stanley control law (platoon following interface)
        
        Stanley law: delta = heading_error + arctan(k_e * crosstrack_error / (v + k_soft))
        """
        if leader_state is None:
            return 0.0
        
        # Extract states
        x = follower_state['x']
        y = follower_state['y']
        theta = follower_state['theta']
        v = follower_state['velocity']
        
        x_j = leader_state['x']
        y_j = leader_state['y']
        theta_j = leader_state['theta']
        
        # 1. Heading Error (Reference - Actual)
        # If Reference=0, Actual=10 (Left), Error = -10. Steer Right (-). Correct.
        # Heading error (align with leader heading)
        heading_error = wrap_to_pi(theta_j - theta)
        
        # 2. Cross-track error (Follower position relative to Leader Path)
        # Cross-track error (perpendicular distance to leader's path)
        # Project follower position onto leader's heading direction
        dx = x - x_j
        dy = y - y_j
        
        # Project vector onto Leader's lateral axis (Left is Positive)
        # Cross-track error: distance perpendicular to leader's heading
        crosstrack_error = -dx * math.sin(theta_j) + dy * math.cos(theta_j)
        
        # 3. Stanley Control Law
        # FIX: If crosstrack_error is positive (we are on the left), 
        # we need to steer Right (Negative).
        # We multiply k_e by crosstrack_error, so we must SUBTRACT the result 
        # (or add a negative).
        
        # Note: arctan2(y, x) -> arctan(y/x). 
        # We use a negative sign here to correct the direction.
        cte_steering = math.atan2(-self.k * crosstrack_error, v + self.k_soft)
        
        steering_cmd = heading_error + cte_steering
        
        # Store for telemetry
        self.cross_track_error = crosstrack_error
        self.heading_error = heading_error
        
        return np.clip(steering_cmd, -self.max_steering_angle, self.max_steering_angle)
    
    def get_reference_pose(self) -> tuple:
        """Get current reference pose"""
        with self._lock:
            return self.p_ref.copy(), self.th_ref
    
    def get_errors(self) -> tuple:
        """Get current control errors"""
        with self._lock:
            return self.cross_track_error, self.heading_error
    
    def get_waypoint_index(self) -> int:
        """Get current waypoint index"""
        with self._lock:
            return self.wpi
    
    def reset(self, waypoints: Optional[np.ndarray] = None):
        """Reset controller state"""
        with self._lock:
            if waypoints is not None:
                self.wp = waypoints
                self.N = len(waypoints[0, :])
            
            self.wpi = 0
            self.p_ref = np.array([0.0, 0.0])
            self.th_ref = 0.0
            self.cross_track_error = 0.0
            self.heading_error = 0.0
            
            # if self.logger:
            #     self.logger.log_control_event("steering_controller_reset", {})


class LookaheadController(LateralControllerBase):
    """
    Extended Lookahead Controller
    Uses curvature information and preview distance
    Based on MATLAB implementation
    """
    
    def __init__(self, ri=1.0, hi=0.3, l_r=0.141, l_f=0.115, 
                 k1=1.0, k2=1.0, max_steering=0.55, config=None, logger=None):
        """
        Initialize Lookahead controller
        
        Args:
            ri: Desired inter-vehicle spacing (meters)
            hi: Time headway (seconds)
            l_r: Distance from CG to rear axle (meters)
            l_f: Distance from CG to front axle (meters)
            k1: Lateral control gain
            k2: Yaw rate control gain
            max_steering: Maximum steering angle (radians)
            config: Optional config object (takes precedence)
            logger: Logger instance
        """
        self.logger = logger
        
        # Use config if provided
        if config and hasattr(config, 'get_lateral_params'):
            # ControllerConfig - get params from dictionary
            params = config.get_lateral_params('lookahead')
            self.ri = params.get('ri', ri)
            self.hi = params.get('hi', hi)
            self.l_r = params.get('l_r', l_r)
            self.l_f = params.get('l_f', l_f)
            self.k1 = params.get('k1', k1)
            self.k2 = params.get('k2', k2)
            self.max_steering = params.get('max_steering', max_steering)
        else:
            self.ri = ri
            self.hi = hi
            self.l_r = l_r
            self.l_f = l_f
            self.k1 = k1
            self.k2 = k2
            self.max_steering = max_steering
        
        # State variables
        self.prev_theta_lead = 0.0
        self.prev_yaw_rate_lead = 0.0
        
    def compute_steering(self, follower_state: Dict[str, float], 
                        leader_state: Optional[Dict[str, float]], 
                        dt: float) -> float:
        """
        Compute steering using lookahead control law
        
        Uses curvature estimation and preview distance
        """
        if leader_state is None:
            return 0.0
        
        # Extract states
        x = follower_state['x']
        y = follower_state['y']
        theta = follower_state['theta']
        v = follower_state['velocity']
        
        x_j = leader_state['x']
        y_j = leader_state['y']
        theta_j = leader_state['theta']
        v_j = leader_state['velocity']
        
        # Estimate leader's yaw rate
        dtheta = wrap_to_pi(theta_j - self.prev_theta_lead)
        yaw_rate_lead = dtheta / dt if dt > 0 else 0.0
        
        # Filter yaw rate
        alpha_filter = 0.3
        yaw_rate_lead_filtered = (alpha_filter * yaw_rate_lead + 
                                  (1 - alpha_filter) * self.prev_yaw_rate_lead)
        
        # Compute curvature
        kappa = self._compute_curvature(v_j, yaw_rate_lead_filtered, dt)
        
        # Compute effective spacing
        s_bar = self._compute_s_bar(kappa, self.ri, self.hi, v)
        
        # Lateral offset (Positive = Left of Leader)
        dx = x - x_j
        dy = y - y_j
        lateral_offset = -dx * math.sin(theta_j) + dy * math.cos(theta_j)
        
        # Heading error (Actual - Reference)
        # Positive = Facing Left relative to leader
        heading_error = wrap_to_pi(theta - theta_j)
        
        # FIX 1: Projection
        # If we are facing Left (heading_error > 0), our projected error 
        # at distance s_bar increases to the Left. We must ADD.
        delta_y = lateral_offset + s_bar * math.sin(heading_error)
        
        # FIX 2: Control Law Signs
        # Term 1 (Lateral): If delta_y is + (Left), steer Right (-). Correct.
        # Term 2 (Heading): If heading_error is + (Left), steer Right (-). Correct.
        # Term 3 (Curvature): If Curve is + (Left), steer Left (+). FIX: Change - to +
        
        steering_cmd = (-self.k1 * delta_y 
                        -self.k2 * heading_error 
                        + (self.l_r + self.l_f) * kappa) # Changed - to +
        
        # Update state
        self.prev_theta_lead = theta_j
        self.prev_yaw_rate_lead = yaw_rate_lead_filtered
        
        return np.clip(steering_cmd, -self.max_steering, self.max_steering)
    
    def _compute_curvature(self, v: float, yaw_rate: float, dt: float) -> float:
        """Compute path curvature from velocity and yaw rate"""
        if abs(v) < 0.01:
            return 0.0
        return yaw_rate / v
    
    def _compute_s_bar(self, kappa: float, ri: float, hi: float, v: float) -> float:
        """Compute effective spacing based on curvature"""
        # Simplified version
        s_bar = ri + hi * v
        return s_bar
    
    def reset(self):
        """Reset controller state"""
        self.prev_theta_lead = 0.0
        self.prev_yaw_rate_lead = 0.0


class HybridLateralController(LateralControllerBase):
    """
    Hybrid lateral controller that switches between controllers
    based on conditions (e.g., distance, velocity)
    """
    
    def __init__(self, primary_controller=None, secondary_controller=None, 
                 switch_distance=1.5, config=None, logger=None):
        """
        Initialize hybrid lateral controller
        
        Args:
            primary_controller: Primary lateral controller (used when close)
            secondary_controller: Secondary controller (used when far)
            switch_distance: Distance threshold for switching (meters)
            config: Optional config object (takes precedence)
            logger: Logger instance
        """
        self.logger = logger
        
        # Use config if provided to get switch_distance
        if config and hasattr(config, 'get_lateral_params'):
            params = config.get_lateral_params('hybrid')
            self.switch_distance = params.get('switch_distance', switch_distance)
            # Primary and secondary controllers should already be created by config loader
            self.primary = primary_controller or params.get('primary_controller') or PurePursuitController(config=config, logger=logger)
            self.secondary = secondary_controller or params.get('secondary_controller') or StanleyController(config=config, logger=logger)
        else:
            self.primary = primary_controller or PurePursuitController(logger=logger)
            self.secondary = secondary_controller or StanleyController(logger=logger)
            self.switch_distance = switch_distance
        
        self.last_mode = "unknown"
        
    def compute_steering(self, follower_state: Dict[str, float], 
                        leader_state: Optional[Dict[str, float]], 
                        dt: float) -> float:
        """Switch between controllers based on distance to leader"""
        
        if leader_state is None:
            return 0.0
        
        # Compute distance to leader
        x = follower_state['x']
        y = follower_state['y']
        x_j = leader_state['x']
        y_j = leader_state['y']
        
        distance = math.sqrt((x_j - x)**2 + (y_j - y)**2)
        
        # Switch logic with hysteresis
        if distance < self.switch_distance:
            # Close range - use primary (typically pure pursuit)
            if self.last_mode != "primary":
                self.last_mode = "primary"
                if self.logger:
                    self.logger.info("[HYBRID_LAT] Switching to primary controller")
            return self.primary.compute_steering(follower_state, leader_state, dt)
        else:
            # Far range - use secondary (typically Stanley)
            if self.last_mode != "secondary":
                self.last_mode = "secondary"
                if self.logger:
                    self.logger.info("[HYBRID_LAT] Switching to secondary controller")
            return self.secondary.compute_steering(follower_state, leader_state, dt)
    
    def reset(self):
        """Reset both controllers"""
        self.primary.reset()
        self.secondary.reset()
        self.last_mode = "unknown"


class LateralControllerFactory:
    """Factory to create lateral controllers by name"""
    
    CONTROLLER_TYPES = {
        'pure_pursuit': PurePursuitController,
        'stanley': StanleyController,
        'lookahead': LookaheadController,
        'hybrid': HybridLateralController,
    }
    
    @staticmethod
    def create(controller_type: str, params: Dict[str, Any] = None, logger=None):
        """
        Create a lateral controller
        
        Args:
            controller_type: One of 'pure_pursuit', 'stanley', 'lookahead', 'hybrid'
            params: Dictionary of controller-specific parameters
            logger: Logger instance
            
        Returns:
            Lateral controller instance
        """
        params = params or {}
        
        if controller_type not in LateralControllerFactory.CONTROLLER_TYPES:
            raise ValueError(
                f"Unknown controller type: {controller_type}. "
                f"Available: {list(LateralControllerFactory.CONTROLLER_TYPES.keys())}"
            )
        
        controller_class = LateralControllerFactory.CONTROLLER_TYPES[controller_type]
        return controller_class(logger=logger, **params)
