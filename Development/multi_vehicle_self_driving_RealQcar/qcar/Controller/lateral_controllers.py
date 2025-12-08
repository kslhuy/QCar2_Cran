"""
Lateral Controllers for Vehicle Following

Provides different lateral control strategies with a common interface.
Easy to switch between different controllers.
"""
import numpy as np
import math
from abc import ABC, abstractmethod
from typing import Optional, Dict, Any
from pal.utilities.math import wrap_to_pi


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
                 max_steering=0.55, adaptive_lookahead=True, logger=None):
        """
        Initialize Pure Pursuit controller
        
        Args:
            lookahead_distance: Base lookahead distance (meters)
            k_steering: Proportional steering gain
            max_steering: Maximum steering angle (radians)
            adaptive_lookahead: Enable adaptive lookahead based on distance to leader
            logger: Logger instance
        """
        self.lookahead_distance = lookahead_distance
        self.k_steering = k_steering
        self.max_steering = max_steering
        self.adaptive_lookahead = adaptive_lookahead
        self.logger = logger
        
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
        motion_direction = 1  # Assume forward motion
        target_x = x_j - motion_direction * lookahead * math.cos(theta_j)
        target_y = y_j - motion_direction * lookahead * math.sin(theta_j)
        
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
    Stanley lateral controller
    Combines heading error and cross-track error
    """
    
    def __init__(self, k_e=0.5, k_soft=1.0, max_steering=0.55, logger=None):
        """
        Initialize Stanley controller
        
        Args:
            k_e: Cross-track error gain
            k_soft: Softening gain to prevent division by zero
            max_steering: Maximum steering angle (radians)
            logger: Logger instance
        """
        self.k_e = k_e
        self.k_soft = k_soft
        self.max_steering = max_steering
        self.logger = logger
        
    def compute_steering(self, follower_state: Dict[str, float], 
                        leader_state: Optional[Dict[str, float]], 
                        dt: float) -> float:
        """
        Compute steering using Stanley control law
        
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
        cte_steering = math.atan2(-self.k_e * crosstrack_error, v + self.k_soft)
        
        steering_cmd = heading_error + cte_steering
        return steering_cmd
    
    def reset(self):
        """Reset controller state (no state to reset for Stanley)"""
        pass


class LookaheadController(LateralControllerBase):
    """
    Extended Lookahead Controller
    Uses curvature information and preview distance
    Based on MATLAB implementation
    """
    
    def __init__(self, ri=1.0, hi=0.3, l_r=0.141, l_f=0.115, 
                 k1=1.0, k2=1.0, max_steering=0.55, logger=None):
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
            logger: Logger instance
        """
        self.ri = ri
        self.hi = hi
        self.l_r = l_r
        self.l_f = l_f
        self.k1 = k1
        self.k2 = k2
        self.max_steering = max_steering
        self.logger = logger
        
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
                 switch_distance=1.5, logger=None):
        """
        Initialize hybrid lateral controller
        
        Args:
            primary_controller: Primary lateral controller (used when close)
            secondary_controller: Secondary controller (used when far)
            switch_distance: Distance threshold for switching (meters)
            logger: Logger instance
        """
        self.primary = primary_controller or PurePursuitController(logger=logger)
        self.secondary = secondary_controller or StanleyController(logger=logger)
        self.switch_distance = switch_distance
        self.logger = logger
        
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
