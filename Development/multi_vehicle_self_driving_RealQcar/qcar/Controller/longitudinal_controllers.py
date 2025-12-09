"""
Longitudinal Controllers for Vehicle Following

Provides different longitudinal control strategies with a common interface.
Easy to switch between different controllers.
"""
import numpy as np
from abc import ABC, abstractmethod
from typing import Optional, Dict, Any


class LongitudinalControllerBase(ABC):
    """Base class for all longitudinal controllers"""
    
    @abstractmethod
    def compute_throttle(self, follower_state: Dict[str, float], 
                        leader_state: Optional[Dict[str, float]], 
                        dt: float) -> float:
        """
        Compute throttle command
        
        Args:
            follower_state: Dict with keys 'x', 'y', 'theta', 'velocity'
            leader_state: Dict with keys 'x', 'y', 'theta', 'velocity' (or None)
            dt: Time step
            
        Returns:
            Throttle command (-1 to 1)
        """
        pass
    
    @abstractmethod
    def reset(self):
        """Reset controller state"""
        pass


class PIVelocityController(LongitudinalControllerBase):
    """
    Simple PI velocity controller
    Tracks a target velocity with PI feedback
    """
    
    def __init__(self, kp=0.1, ki=1.0, max_throttle=0.3, logger=None):
        self.kp = kp
        self.ki = ki
        self.max_throttle = max_throttle
        self.logger = logger
        
        self.ei = 0.0  # Integral error
        self.ei_max = 1.0  # Anti-windup limit
        
    def compute_throttle(self, follower_state: Dict[str, float], 
                        leader_state: Optional[Dict[str, float]], 
                        dt: float) -> float:
        """Compute PI control based on velocity error"""
        current_velocity = follower_state['velocity']
        target_velocity = follower_state.get('target_velocity', 0.0)
        
        # Calculate error
        e = target_velocity - current_velocity
        
        # Integral with anti-windup
        self.ei += dt * e
        self.ei = np.clip(self.ei, -self.ei_max, self.ei_max)
        
        # PI control
        u = self.kp * e + self.ki * self.ei
        
        # Clamp output
        u = np.clip(u, -self.max_throttle, self.max_throttle)
        
        return u
    
    def reset(self):
        """Reset integral state"""
        self.ei = 0.0


class CACCLongitudinalController(LongitudinalControllerBase):
    """
    CACC-based longitudinal controller
    Uses spacing error and velocity error to compute acceleration,
    then converts to throttle command
    """
    
    def __init__(self, s0=1.5, h=0.5, K=None, 
                 acc_to_throttle_gain=0.5,
                 max_throttle=0.3,
                 alpha_filter=0.3,
                 ki_velocity=0.1,
                 brake_smoothing=0.3,
                 logger=None):
        """
        Initialize CACC longitudinal controller
        
        Args:
            s0: Minimum spacing (meters)
            h: Time headway (seconds)
            K: Control gains [spacing_gain, velocity_gain]
            acc_to_throttle_gain: Gain to convert acceleration to throttle
            max_throttle: Maximum throttle output
            alpha_filter: Low-pass filter coefficient (0-1)
            ki_velocity: Velocity integral gain for additional stability
            brake_smoothing: Smoothing factor for negative throttle (0-1, higher = smoother)
            logger: Logger instance
        """
        self.s0 = s0
        self.h = h
        self.K = K if K is not None else np.array([[0.2, 0.05]])
        self.acc_to_throttle_gain = acc_to_throttle_gain
        self.max_throttle = max_throttle
        self.logger = logger
        
        # State for filtering
        self.prev_acc = 0.0
        self.alpha_filter = alpha_filter
        
        # Velocity integral for additional stability (optional)
        self.velocity_integral = 0.0
        self.ki_velocity = ki_velocity
        
        # Brake smoothing
        self.brake_smoothing = brake_smoothing
        self.prev_throttle = 0.0
        
    def compute_throttle(self, follower_state: Dict[str, float], 
                        leader_state: Optional[Dict[str, float]], 
                        dt: float) -> float:
        """
        Compute throttle using CACC law
        
        CACC computes desired acceleration based on:
        - Spacing error: (actual_spacing - desired_spacing)
        - Velocity error: (leader_velocity - follower_velocity)
        
        Then converts acceleration to throttle command
        """
        if leader_state is None:
            # No leader data - maintain current velocity
            return 0.0
        
        # Extract states
        x = follower_state['x']
        y = follower_state['y']
        v = follower_state['velocity']
        
        x_j = leader_state['x']
        y_j = leader_state['y']
        v_j = leader_state['velocity']
        
        # Calculate actual spacing
        spacing = np.hypot(x_j - x, y_j - y)
        
        # Calculate desired spacing (CTH policy: s_d = s0 + h*v)
        spacing_target = self.s0 + self.h * v
        
        # Calculate errors
        spacing_error = spacing - spacing_target
        velocity_error = v_j - v
        
        # CACC control law: acc = K[0] * e_spacing + K[1] * e_velocity
        error_vector = np.array([spacing_error, velocity_error])
        acc_desired = (self.K @ error_vector)[0]
        throttle = acc_desired
        
        # Clamp to limits
        throttle = np.clip(throttle, -self.max_throttle, self.max_throttle)
        
        # Apply smooth deceleration when throttle is negative
        if throttle < 0:
            # Smooth transition: blend between previous and current throttle
            throttle = (self.brake_smoothing * self.prev_throttle + 
                       (1 - self.brake_smoothing) * throttle)
        
        # Store for next iteration
        self.prev_throttle = throttle
        
        return throttle
    
    def reset(self):
        """Reset controller state"""
        self.prev_acc = 0.0
        self.velocity_integral = 0.0
        self.prev_throttle = 0.0


class HybridController(LongitudinalControllerBase):
    """
    Hybrid controller that switches between CACC and PI
    Uses CACC when leader is available, PI when not
    """
    
    def __init__(self, cacc_params=None, pi_params=None, logger=None):
        """
        Initialize hybrid controller
        
        Args:
            cacc_params: Dict of parameters for CACC controller
            pi_params: Dict of parameters for PI controller
            logger: Logger instance
        """
        cacc_params = cacc_params or {}
        pi_params = pi_params or {}
        
        self.cacc = CACCLongitudinalController(logger=logger, **cacc_params)
        self.pi = PIVelocityController(logger=logger, **pi_params)
        self.logger = logger
        
        self.last_mode = "unknown"
        
    def compute_throttle(self, follower_state: Dict[str, float], 
                        leader_state: Optional[Dict[str, float]], 
                        dt: float) -> float:
        """Switch between CACC and PI based on leader availability"""
        
        if leader_state is not None:
            # Leader available - use CACC
            if self.last_mode != "cacc":
                self.last_mode = "cacc"
                if self.logger:
                    self.logger.info("[HYBRID] Switching to CACC mode")
            return self.cacc.compute_throttle(follower_state, leader_state, dt)
        else:
            # No leader - use PI velocity tracking
            if self.last_mode != "pi":
                self.last_mode = "pi"
                if self.logger:
                    self.logger.info("[HYBRID] Switching to PI mode")
            return self.pi.compute_throttle(follower_state, leader_state, dt)
    
    def reset(self):
        """Reset both controllers"""
        self.cacc.reset()
        self.pi.reset()
        self.last_mode = "unknown"


class ControllerFactory:
    """Factory to create longitudinal controllers by name"""
    
    CONTROLLER_TYPES = {
        'pi': PIVelocityController,
        'cacc': CACCLongitudinalController,
        'hybrid': HybridController,
    }
    
    @staticmethod
    def create(controller_type: str, params: Dict[str, Any] = None, logger=None):
        """
        Create a longitudinal controller
        
        Args:
            controller_type: One of 'pi', 'cacc', 'hybrid'
            params: Dictionary of controller-specific parameters
            logger: Logger instance
            
        Returns:
            Longitudinal controller instance
        """
        params = params or {}
        
        if controller_type not in ControllerFactory.CONTROLLER_TYPES:
            raise ValueError(
                f"Unknown controller type: {controller_type}. "
                f"Available: {list(ControllerFactory.CONTROLLER_TYPES.keys())}"
            )
        
        controller_class = ControllerFactory.CONTROLLER_TYPES[controller_type]
        return controller_class(logger=logger, **params)
