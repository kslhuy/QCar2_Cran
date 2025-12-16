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


class PIDVelocityController(LongitudinalControllerBase):
    """
    PID velocity controller with anti-windup
    Compatible with both platoon following and standalone path following
    """
    
    def __init__(self, kp=0.1, ki=1.0, kd=0.01, max_throttle=0.3, config=None, logger=None):
        """
        Initialize PID velocity controller
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            max_throttle: Maximum throttle output
            config: Optional config object (takes precedence over individual params)
            logger: Logger instance
        """
        self.logger = logger
        
        # Use config if provided, otherwise use individual parameters
        if config and hasattr(config, 'get_longitudinal_params'):
            # ControllerConfig - get params from dictionary
            params = config.get_longitudinal_params('pid')
            self.kp = params.get('kp', kp)
            self.ki = params.get('ki', ki)
            self.kd = params.get('kd', kd)
            self.max_throttle = params.get('max_throttle', max_throttle)
        else:
            self.kp = kp
            self.ki = ki
            self.kd = kd
            self.max_throttle = max_throttle
        
        # print(f"[PIDVelocityController] Initialized with kp={self.kp}, ki={self.ki}, kd={self.kd}, max_throttle={self.max_throttle}")
        self.ei = 0.0  # Integral error
        self.prev_e = None  # Previous error for derivative
        self.last_error = 0.0
        
        # Anti-windup limit
        self.ei_max = 1.0
    
    def update(self, v: float, v_ref: float, dt: float) -> float:
        """
        Update speed controller (path following interface)
        
        Args:
            v: Current velocity
            v_ref: Reference velocity
            dt: Time step
            
        Returns:
            Throttle command
        """
        # Calculate error
        e = v_ref - v
        
        # Integral with anti-windup
        self.ei += dt * e
        self.ei = np.clip(self.ei, -self.ei_max, self.ei_max)
        
        # Calculate derivative of the error
        if self.prev_e is None or dt < 0.001:
            de = 0
        else:
            de = (e - self.prev_e) / dt
        self.prev_e = e
        
        # PID control
        u = self.kp * e + self.ki * self.ei + self.kd * de
        
        # Clamp output
        u = np.clip(u, -self.max_throttle, self.max_throttle)

        # print(f"[PIDVelocityController] v: {v:.2f}, v_ref: {v_ref:.2f}, e: {e:.2f}, ei: {self.ei:.2f}, de: {de:.2f}, throttle: {u:.2f}")
        
        self.last_error = e
        
        return u
        
    def compute_throttle(self, follower_state: Dict[str, float], 
                        leader_state: Optional[Dict[str, float]], 
                        dt: float) -> float:
        """
        Compute PID control based on velocity error (platoon following interface)
        
        Args:
            follower_state: Dict with keys 'velocity', 'target_velocity'
            leader_state: Not used for PID controller
            dt: Time step
            
        Returns:
            Throttle command
        """
        current_velocity = follower_state['velocity']
        target_velocity = follower_state.get('target_velocity', 0.0)
        
        # Use the update method to maintain state consistency
        return self.update(current_velocity, target_velocity, dt)
    
    def reset(self):
        """Reset controller state"""
        self.ei = 0.0
        self.prev_e = None
        self.last_error = 0.0
        
        if self.logger:
            self.logger.log_control_event("speed_controller_reset", {})


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
                 brake_smoothing=0.5,
                 max_acc_rate=2.0,
                 spacing_deadband=0.2,
                 velocity_deadband=0.05,
                 throttle_smoothing=0.7,
                 config=None,
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
            max_acc_rate: Maximum acceleration rate of change (m/s^3) to limit jerk
            spacing_deadband: Spacing error deadband to prevent oscillations (meters)
            velocity_deadband: Velocity error deadband to prevent oscillations (m/s)
            throttle_smoothing: Exponential smoothing factor for throttle (0-1, higher = smoother)
            config: Optional config object (takes precedence)
            logger: Logger instance
        """
        self.logger = logger
        
        # Use config if provided
        if config and hasattr(config, 'get_longitudinal_params'):
            # ControllerConfig - get params from dictionary
            params = config.get_longitudinal_params('cacc')
            self.s0 = params.get('s0', s0)
            self.h = params.get('h', h)
            self.K = params.get('K', K if K is not None else np.array([[0.2, 0.05]]))
            self.acc_to_throttle_gain = params.get('acc_to_throttle_gain', acc_to_throttle_gain)
            self.max_throttle = params.get('max_throttle', max_throttle)
            self.alpha_filter = params.get('alpha_filter', alpha_filter)
            self.ki_velocity = params.get('ki_velocity', ki_velocity)
            # self.spacing_deadband = params.get('spacing_deadband', spacing_deadband)
            # self.velocity_deadband = params.get('velocity_deadband', velocity_deadband)
            # self.throttle_smoothing = params.get('throttle_smoothing', throttle_smoothing)
        else:
            self.s0 = s0
            self.h = h
            self.K = K if K is not None else np.array([[0.2, 0.05]])
            self.acc_to_throttle_gain = acc_to_throttle_gain
            self.max_throttle = max_throttle
            self.alpha_filter = alpha_filter
            self.ki_velocity = ki_velocity
        self.spacing_deadband = spacing_deadband
        self.velocity_deadband = velocity_deadband
        self.throttle_smoothing = throttle_smoothing
        
        # State for filtering
        self.prev_acc = 0.0
        
        # Velocity integral for additional stability (optional)
        self.velocity_integral = 0.0
        
        # Brake smoothing
        self.brake_smoothing = brake_smoothing
        self.prev_throttle = 0.0
        
        # Acceleration rate limiter to prevent sudden jumps
        self.max_acc_rate = max_acc_rate  # Maximum change in acceleration per second
        self.prev_acc_desired = 0.0
        
        # Spacing error integral for steady-state accuracy
        self.spacing_integral = 0.0
        self.ki_spacing = 0.01  # Small integral gain for spacing
        
    def compute_throttle(self, follower_state: Dict[str, float], 
                        leader_state: Optional[Dict[str, float]], 
                        dt: float) -> float:
        """
        Compute throttle using CACC law with enhanced smoothing
        
        CACC computes desired acceleration based on:
        - Spacing error: (actual_spacing - desired_spacing)
        - Velocity error: (leader_velocity - follower_velocity)
        
        Enhanced with:
        - Deadband zones to prevent oscillations
        - Progressive control zones (comfort/normal/emergency)
        - Exponential throttle smoothing
        - Velocity-dependent gain scheduling
        """
        if leader_state is None:
            # No leader data - gradually reduce throttle to zero
            target_throttle = 0.0
            smoothed_throttle = (self.throttle_smoothing * self.prev_throttle + 
                               (1 - self.throttle_smoothing) * target_throttle)
            self.prev_throttle = smoothed_throttle
            return smoothed_throttle
        
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
        
        # Apply deadband to spacing error to prevent small oscillations
        if abs(spacing_error) < self.spacing_deadband:
            spacing_error = 0.0
        else:
            # Remove deadband offset for smoother transition
            spacing_error = spacing_error - np.sign(spacing_error) * self.spacing_deadband
        
        # Apply deadband to velocity error
        if abs(velocity_error) < self.velocity_deadband:
            velocity_error = 0.0
        else:
            velocity_error = velocity_error - np.sign(velocity_error) * self.velocity_deadband
        
        # Update spacing integral (with anti-windup)
        self.spacing_integral += spacing_error * dt
        self.spacing_integral = np.clip(self.spacing_integral, -2.0, 2.0)
        
        # CACC control law with integral term
        error_vector = np.array([spacing_error, velocity_error])
        acc_desired = (self.K @ error_vector)[0] + self.ki_spacing * self.spacing_integral
        
        # # Velocity-dependent gain scheduling (reduce gains at low speeds for stability)
        # speed_factor = np.clip(v / 0.5, 0.3, 1.0)  # Scale down gains below 0.5 m/s
        # acc_desired *= speed_factor
        
        # Apply acceleration rate limiter to prevent sudden jumps
        max_acc_change = self.max_acc_rate * dt
        acc_diff = acc_desired - self.prev_acc_desired
        
        # Limit the change in acceleration
        if abs(acc_diff) > max_acc_change:
            acc_desired = self.prev_acc_desired + np.sign(acc_diff) * max_acc_change
        
        # Store filtered acceleration for next iteration
        self.prev_acc_desired = acc_desired
        
        # Convert acceleration to throttle (simplified linear mapping)
        throttle_raw = acc_desired * self.acc_to_throttle_gain
        
        # Define control zones based on spacing error
        spacing_error_abs = abs(spacing - spacing_target)
        
        if spacing_error_abs < 0.3:  # Comfort zone - very gentle control
            throttle_raw *= 0.5
        elif spacing_error_abs < 0.8:  # Normal zone - standard control
            throttle_raw *= 0.8
        # else: Emergency zone - full control authority
        
        # Clamp to limits
        throttle_raw = np.clip(throttle_raw, -self.max_throttle, self.max_throttle)
        
        # Special handling for braking (negative throttle)
        if throttle_raw < 0:
            # More aggressive smoothing for braking to prevent jerky stops
            smoothing_factor = 0.85
            throttle_raw = (smoothing_factor * self.prev_throttle + 
                          (1 - smoothing_factor) * throttle_raw)
            throttle_raw = max(throttle_raw, 0.0)  # No negative throttle output
        
        # Apply exponential smoothing to final throttle command
        throttle = (self.throttle_smoothing * self.prev_throttle + 
                   (1 - self.throttle_smoothing) * throttle_raw)
        
        # Ensure throttle is non-negative
        throttle = max(throttle, 0.0)
        
        # Store for next iteration
        self.prev_throttle = throttle
        
        return throttle
    
    def reset(self):
        """Reset controller state"""
        self.prev_acc = 0.0
        self.velocity_integral = 0.0
        self.prev_throttle = 0.0
        self.prev_acc_desired = 0.0
        self.spacing_integral = 0.0


class HybridController(LongitudinalControllerBase):
    """
    Hybrid controller that switches between CACC and PI
    Uses CACC when leader is available, PI when not
    """
    
    def __init__(self, cacc_params=None, pi_params=None, config=None, logger=None):
        """
        Initialize hybrid controller
        
        Args:
            cacc_params: Dict of parameters for CACC controller
            pi_params: Dict of parameters for PI controller
            config: Optional config object (takes precedence)
            logger: Logger instance
        """
        self.logger = logger
        
        # Use config if provided
        if config:
            if hasattr(config, 'get_longitudinal_params'):
                # Get params for both controllers from config
                hybrid_params = config.get_longitudinal_params('hybrid')
                cacc_params = hybrid_params.get('cacc_params', cacc_params or {})
                pi_params = hybrid_params.get('pi_params', pi_params or {})
        else:
            cacc_params = cacc_params or {}
            pi_params = pi_params or {}
        
        self.cacc = CACCLongitudinalController(config=config, logger=logger, **cacc_params)
        self.pi = PIDVelocityController(config=config, logger=logger, **pi_params)
        
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
        'pi': PIDVelocityController,
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
