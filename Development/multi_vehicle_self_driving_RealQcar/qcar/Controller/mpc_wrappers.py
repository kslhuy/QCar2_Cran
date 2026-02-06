"""
MPC Controller Wrappers for Compatibility with Existing Controller Interfaces

Provides wrapper classes that allow MPC controllers to be used with the
existing LongitudinalControllerBase and LateralControllerBase interfaces.
"""
import numpy as np
from typing import Optional, Dict, Any, Tuple

from .mpc_controller import CasADiMPCController, DynamicBicycleMPCController, MPCControllerFactory
from .longitudinal_controllers import LongitudinalControllerBase
from .lateral_controllers import LateralControllerBase


class MPCLongitudinalWrapper(LongitudinalControllerBase):
    """
    Wrapper to use MPC controller with LongitudinalControllerBase interface.
    
    The MPC computes both throttle and steering, but this wrapper
    only returns the throttle component.
    """
    
    def __init__(self, mpc_controller: CasADiMPCController = None, 
                 mpc_type: str = 'mpc',
                 mpc_params: Dict[str, Any] = None,
                 logger=None):
        """
        Initialize wrapper.
        
        Args:
            mpc_controller: Pre-configured MPC controller (optional)
            mpc_type: Type of MPC to create if mpc_controller not provided
            mpc_params: Parameters for MPC creation
            logger: Logger instance
        """
        self.logger = logger
        
        if mpc_controller is not None:
            self.mpc = mpc_controller
        else:
            self.mpc = MPCControllerFactory.create(mpc_type, mpc_params, logger)
        
        # Store last steering for external access
        self.last_steering = 0.0
        
    def compute_throttle(self, follower_state: Dict[str, float], 
                        leader_state: Optional[Dict[str, float]], 
                        dt: float) -> float:
        """Compute throttle using MPC (steering is computed but stored separately)."""
        throttle, steering = self.mpc.compute_control(follower_state, leader_state, dt)
        self.last_steering = steering
        return throttle
    
    def get_last_steering(self) -> float:
        """Get the steering computed in the last control call."""
        return self.last_steering
    
    def reset(self):
        """Reset controller state."""
        self.mpc.reset()
        self.last_steering = 0.0


class MPCLateralWrapper(LateralControllerBase):
    """
    Wrapper to use MPC controller with LateralControllerBase interface.
    
    The MPC computes both throttle and steering, but this wrapper
    only returns the steering component.
    """
    
    def __init__(self, mpc_controller: CasADiMPCController = None,
                 mpc_type: str = 'mpc',
                 mpc_params: Dict[str, Any] = None,
                 logger=None):
        """
        Initialize wrapper.
        
        Args:
            mpc_controller: Pre-configured MPC controller (optional)
            mpc_type: Type of MPC to create if mpc_controller not provided
            mpc_params: Parameters for MPC creation
            logger: Logger instance
        """
        self.logger = logger
        
        if mpc_controller is not None:
            self.mpc = mpc_controller
        else:
            self.mpc = MPCControllerFactory.create(mpc_type, mpc_params, logger)
        
        # Store last throttle for external access
        self.last_throttle = 0.0
        
    def compute_steering(self, follower_state: Dict[str, float], 
                        leader_state: Optional[Dict[str, float]], 
                        dt: float) -> float:
        """Compute steering using MPC (throttle is computed but stored separately)."""
        throttle, steering = self.mpc.compute_control(follower_state, leader_state, dt)
        self.last_throttle = throttle
        return steering
    
    def get_last_throttle(self) -> float:
        """Get the throttle computed in the last control call."""
        return self.last_throttle
    
    def reset(self):
        """Reset controller state."""
        self.mpc.reset()
        self.last_throttle = 0.0


class MPCCombinedController:
    """
    Combined MPC controller that provides both throttle and steering
    in a single interface, suitable for vehicles that need unified control.
    
    This is the recommended way to use MPC when you want both outputs.
    """
    
    def __init__(self, 
                 mpc_type: str = 'mpc',
                 mpc_params: Dict[str, Any] = None,
                 config=None,
                 logger=None):
        """
        Initialize combined MPC controller.
        
        Args:
            mpc_type: Type of MPC ('mpc', 'casadi_kinematic', 'casadi_dynamic')
            mpc_params: MPC parameters
            config: Optional config object
            logger: Logger instance
        """
        self.logger = logger
        
        # If config provided, extract MPC params
        if config and hasattr(config, 'get_mpc_params'):
            mpc_params = config.get_mpc_params()
        
        self.mpc = MPCControllerFactory.create(mpc_type, mpc_params, logger)
        
        # Last computed values
        self.last_throttle = 0.0
        self.last_steering = 0.0
        
    def compute_control(self, follower_state: Dict[str, float], 
                       leader_state: Optional[Dict[str, float]], 
                       dt: float) -> Tuple[float, float]:
        """
        Compute both throttle and steering commands.
        
        Args:
            follower_state: Dict with keys 'x', 'y', 'theta', 'velocity', 
                           optionally 'target_velocity'
            leader_state: Dict with keys 'x', 'y', 'theta', 'velocity' (or None)
            dt: Time step
            
        Returns:
            Tuple of (throttle, steering_angle)
        """
        throttle, steering = self.mpc.compute_control(follower_state, leader_state, dt)
        self.last_throttle = throttle
        self.last_steering = steering
        return throttle, steering
    
    def compute_throttle(self, follower_state: Dict[str, float], 
                        leader_state: Optional[Dict[str, float]], 
                        dt: float) -> float:
        """Compute throttle (also computes steering internally)."""
        throttle, steering = self.compute_control(follower_state, leader_state, dt)
        return throttle
    
    def compute_steering(self, follower_state: Dict[str, float], 
                        leader_state: Optional[Dict[str, float]], 
                        dt: float) -> float:
        """Compute steering (also computes throttle internally)."""
        throttle, steering = self.compute_control(follower_state, leader_state, dt)
        return steering
    
    def get_last_control(self) -> Tuple[float, float]:
        """Get the last computed control values."""
        return self.last_throttle, self.last_steering
    
    def reset(self):
        """Reset controller state."""
        self.mpc.reset()
        self.last_throttle = 0.0
        self.last_steering = 0.0


# Add MPC to the factory registries in longitudinal and lateral controllers
def register_mpc_controllers():
    """
    Register MPC controller wrappers with existing controller factories.
    
    Call this function to make MPC available through the existing factory interfaces.
    """
    try:
        from .longitudinal_controllers import ControllerFactory as LongFactory
        from .lateral_controllers import LateralControllerFactory as LatFactory
        
        # Register MPC wrappers
        LongFactory.CONTROLLER_TYPES['mpc'] = MPCLongitudinalWrapper
        LatFactory.CONTROLLER_TYPES['mpc'] = MPCLateralWrapper
        
        return True
    except ImportError as e:
        print(f"Warning: Could not register MPC controllers: {e}")
        return False
