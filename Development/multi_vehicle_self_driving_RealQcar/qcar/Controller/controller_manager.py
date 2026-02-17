"""
Controller Manager - Centralized Controller Creation and Management

This module provides centralized management for vehicle controllers:
1. Creates and caches longitudinal and lateral controllers based on config
2. Provides easy access for states to get controllers
3. Reports active controller types to Ground Station
4. Enables runtime controller switching

Architecture:
- ControllerManager loads config and creates controllers on demand
- States request controllers via get_* methods (no factory imports needed)
- vehicle_logic uses this manager to report active types to GS
- GS can request controller switches through this manager
"""

from typing import Optional, Dict, Any
from dataclasses import dataclass
import os
import yaml

from .config_controller_loader import get_controller_config
from Controller.longitudinal_controllers import ControllerFactory



@dataclass
class ControllerInfo:
    """Information about an active controller."""
    controller: Any  # The actual controller instance
    type_name: str   # Type name (e.g., 'pid', 'cacc', 'pure_pursuit', 'stanley')
    category: str    # 'longitudinal' or 'lateral'


class ControllerManager:
    """
    Centralized Controller Manager
    
    Creates, tracks, and provides controllers for vehicle states.
    States simply call get_longitudinal_controller() or get_lateral_controller()
    to get the configured controller - no factory imports needed in states.
    
    Usage:
        # In vehicle_logic __init__:
        self.controller_manager = ControllerManager(logger=logger)
        
        # In state enter():
        self.longitudinal_controller = vehicle_logic.controller_manager.get_longitudinal_controller()
        self.lateral_controller = vehicle_logic.controller_manager.get_lateral_controller()
        
        # For telemetry:
        long_type = vehicle_logic.controller_manager.get_longitudinal_type()
    """
    
    def __init__(self, logger=None, config=None, vehicle_id: int = None):
        """
        Initialize ControllerManager.
        
        Args:
            logger: Logger instance
            config: Optional pre-loaded config (if None, will load from file)
            vehicle_id: Vehicle ID for per-vehicle config override
        """
        # Load config with vehicle-specific overrides if vehicle_id provided
        if config is None:
            self.config = get_controller_config(vehicle_id=vehicle_id)
        else:
            self.config = config
        
        self.logger = logger
        self.vehicle_id = vehicle_id
        
        # Active controllers
        self._longitudinal: Optional[ControllerInfo] = None
        self._lateral: Optional[ControllerInfo] = None
        
        # Cached steering controller for path-following modes
        self._steering_controller = None
        self._steering_controller_waypoints = None
        
        # Vehicle reference for waypoints (set later)
        self.vehicle_logic = None
        
        # Load default types from config file
        self._load_config()
        
        if self.logger:
            self.logger.logger.info(
                f"[ControllerManager] Initialized: long={self._longitudinal_type}, lat={self._lateral_type}"
            )
    
    def _load_config(self):
        """Load controller types and params from config."""
        
        try:
            if self.config:
                self._longitudinal_type = self.config.get_longitudinal_controller_type()
                self._lateral_type = self.config.get_lateral_controller_type()
        except Exception as e:
            if self.logger:
                self.logger.log_warning(f"Failed to load controller types from config: {e}")
    
    def set_vehicle_logic(self, vehicle_logic):
        """Set vehicle_logic reference for accessing waypoints."""
        self.vehicle_logic = vehicle_logic
    
    # =============== Controller Getters (called by states) ===============
    
    def get_longitudinal_controller(self, force_type: str = None):
        """
        Get or create longitudinal controller.
        
        Args:
            force_type: Override config type (e.g., 'cacc', 'pid', 'hybrid')
            
        Returns:
            Longitudinal controller instance
        """
        ctrl_type = force_type or self._longitudinal_type
        
        # Return cached if same type
        if self._longitudinal and self._longitudinal.type_name == ctrl_type:
            return self._longitudinal.controller
        
        # Create new controller
        try:
            
            params = {}
            if self.config:
                params = self.config.get_longitudinal_params(ctrl_type)
            
            # Special handling for state_feedback controllers - needs observer
            observer = None
            if ctrl_type in ['state_feedback', 'state_feedback_no_observer', 'classical_distributed']:
                if self.vehicle_logic and hasattr(self.vehicle_logic, 'vehicle_observer'):
                    # Get the fleet estimator (DistributedLuenbergerEstimator)
                    observer = getattr(self.vehicle_logic.vehicle_observer, 'fleet_estimator', None)
                    if observer is not None and self.logger:
                        self.logger.logger.info(
                            f"[ControllerManager] Passing observer to {ctrl_type} controller: "
                            f"type={type(observer).__name__}, vehicle_id={getattr(observer, 'vehicle_id', 'N/A')}"
                        )
                    elif self.logger:
                        self.logger.logger.warning(
                            f"[ControllerManager] {ctrl_type} controller requires observer, "
                            "but fleet_estimator not found in vehicle_observer"
                        )
            
            controller = ControllerFactory.create(
                ctrl_type,
                params,
                logger=self.logger.logger if self.logger else None,
                observer=observer  # Pass observer for state_feedback, state_feedback_no_observer, and classical_distributed controllers
            )
            
            self._longitudinal = ControllerInfo(
                controller=controller,
                type_name=ctrl_type,
                category='longitudinal'
            )
            
            if self.logger:
                self.logger.logger.info(f"[ControllerManager] Created longitudinal controller: {ctrl_type}")
            
            return controller
            
        except Exception as e:
            if self.logger:
                self.logger.log_error(f"Failed to create longitudinal controller {ctrl_type}", e)
            return None
    
    def get_lateral_controller(self, force_type: str = None, waypoints=None):
        """
        Get or create lateral controller.
        
        Args:
            force_type: Override config type (e.g., 'pure_pursuit', 'stanley', 'lookahead')
            waypoints: Waypoints for path-based controllers (optional)
            
        Returns:
            Lateral controller instance
        """
        ctrl_type = force_type or self._lateral_type
        
        # Handle 'path' mode separately - returns steering controller
        if ctrl_type == 'path':
            return self.get_steering_controller(waypoints)
        
        # Return cached if same type
        if self._lateral and self._lateral.type_name == ctrl_type:
            return self._lateral.controller
        
        # Create new controller
        try:
            from Controller.lateral_controllers import LateralControllerFactory
            
            params = {}
            if self.config:
                params = self.config.get_lateral_params(ctrl_type)
            
            controller = LateralControllerFactory.create(
                ctrl_type,
                params,
                logger=self.logger.logger if self.logger else None
            )
            
            self._lateral = ControllerInfo(
                controller=controller,
                type_name=ctrl_type,
                category='lateral'
            )
            
            if self.logger:
                self.logger.logger.info(f"[ControllerManager] Created lateral controller: {ctrl_type}")
            
            return controller
            
        except Exception as e:
            if self.logger:
                self.logger.log_error(f"Failed to create lateral controller {ctrl_type}", e)
            return None
    
    def get_steering_controller(self, waypoints=None):
        """
        Get or create steering controller for path following.
        
        Args:
            waypoints: Waypoint array (uses vehicle_logic.waypoint_sequence if None)
            
        Returns:
            SteeringController (StanleyController) instance
        """
        # Get waypoints from vehicle_logic if not provided
        if waypoints is None and self.vehicle_logic:
            waypoints = getattr(self.vehicle_logic, 'waypoint_sequence', None)
        
        if waypoints is None:
            if self.logger:
                self.logger.logger.warning("[ControllerManager] No waypoints for steering controller")
            return None
        
        # Return cached if waypoints unchanged
        if (self._steering_controller is not None and 
            self._steering_controller_waypoints is waypoints):
            return self._steering_controller
        
        # Create new steering controller using configured lateral controller type
        # PREVIOUS IMPLEMENTATION (hardcoded Stanley): 
        #   from Controller.lateral_controllers import StanleyController
        #   controller = StanleyController(waypoints=waypoints, config=self.config, logger=self.logger, cyclic=True)
        #   type_name='fix_lateral' (Just for Shengya; Default to "stanley")
        #
        # ISSUE: The hardcoded StanleyController ignored the lateral_controller_type setting in config.
        #        This caused the leader vehicle to use Stanley controller for path following even when
        #        fix_lateral (zero steering) was configured, resulting in unwanted turning behavior.
        #
        # NEW IMPLEMENTATION: Use get_lateral_controller() which respects the config file setting
        try:
            # Get the configured lateral controller type from config file
            ctrl_type = self.config.get_lateral_controller_type() if self.config else 'stanley'
            
            if self.logger:
                self.logger.logger.info(
                    f"[ControllerManager] Creating steering controller with type: {ctrl_type}"
                )
            
            # Use the lateral controller factory to create the controller based on config
            controller = self.get_lateral_controller(ctrl_type)
            
            if controller is None:
                if self.logger:
                    self.logger.logger.error(
                        f"[ControllerManager] Failed to create steering controller of type: {ctrl_type}"
                    )
                return None
            
            # For waypoint-based controllers (Stanley, PurePursuit), set the waypoints
            # Note: FixConstantLateralController does not need waypoints and will skip this
            if hasattr(controller, 'reset') and waypoints is not None:
                try:
                    controller.reset(waypoints)
                    if self.logger:
                        self.logger.logger.info(
                            f"[ControllerManager] Waypoints set for {ctrl_type} controller"
                        )
                except Exception as e:
                    if self.logger:
                        self.logger.logger.warning(
                            f"[ControllerManager] Controller {ctrl_type} does not support waypoints: {e}"
                        )
            
            self._steering_controller = controller
            self._steering_controller_waypoints = waypoints
            
            # Also track as lateral controller (for telemetry)
            self._lateral = ControllerInfo(
                controller=controller,
                type_name=ctrl_type,  # Now uses actual controller type from config
                category='lateral'
            )
            
            if self.logger:
                self.logger.logger.info(
                    f"[ControllerManager] Created steering controller ({ctrl_type})"
                )
            
            return controller
            
        except Exception as e:
            if self.logger:
                self.logger.log_error("Failed to create steering controller", e)
            return None
    
    def get_speed_controller(self):
        """
        Get speed controller for path following (FOLLOWING_PATH state).
        
        PREVIOUS IMPLEMENTATION (hardcoded PID):
            return self.get_longitudinal_controller('pid')
        
        ISSUE: The hardcoded 'pid' controller type ignored the longitudinal_controller_type 
               setting in config file. This caused the leader vehicle to use PID controller even 
               when 'fix' (constant throttle) was configured, resulting in no movement.
        
        NEW IMPLEMENTATION: Use the configured longitudinal controller type from config file.
        This respects the user's controller selection (e.g., 'fix', 'pid', 'cacc', etc.)
        
        Returns:
            Configured longitudinal controller instance (PIDVelocityController, 
            FixConstantController, etc. based on config)
        """
        # Get the configured longitudinal controller type
        ctrl_type = self.config.get_longitudinal_controller_type() if self.config else 'pid'
        
        if self.logger:
            self.logger.logger.info(
                f"[ControllerManager] Creating speed controller with type: {ctrl_type}"
            )
        
        # Use the configured controller type instead of hardcoded 'pid'
        return self.get_longitudinal_controller(ctrl_type)
    
    # =============== Type Getters (for telemetry) ===============
    
    def get_longitudinal_type(self) -> str:
        """Get active longitudinal controller type name."""
        if self._longitudinal:
            return self._longitudinal.type_name
        return self._longitudinal_type  # Default from config
    
    def get_lateral_type(self) -> str:
        """Get active lateral controller type name."""
        if self._lateral:
            return self._lateral.type_name
        return self._lateral_type  # Default from config
    
    def get_status(self) -> Dict[str, str]:
        """Get current controller status for telemetry."""
        return {
            'longitudinal_ctrl_type': self.get_longitudinal_type(),
            'lateral_ctrl_type': self.get_lateral_type(),
        }
    
    # =============== Runtime Switching ===============
    
    def switch_longitudinal(self, new_type: str) -> bool:
        """
        Switch longitudinal controller at runtime.
        
        Args:
            new_type: New controller type ('cacc', 'pid', 'hybrid')
            
        Returns:
            bool: True if switch successful
        """
        try:
            # Clear cached to force recreation
            self._longitudinal = None
            self._longitudinal_type = new_type
            
            # Create new controller
            controller = self.get_longitudinal_controller(new_type)
            
            # Update config for persistence
            if self.config and controller:
                self.config.config['longitudinal_controller_type'] = new_type
            
            return controller is not None
            
        except Exception as e:
            if self.logger:
                self.logger.log_error(f"Failed to switch longitudinal controller to {new_type}", e)
            return False
    
    def switch_lateral(self, new_type: str) -> bool:
        """
        Switch lateral controller at runtime.
        
        Args:
            new_type: New controller type ('pure_pursuit', 'stanley', 'lookahead', 'hybrid', 'path')
            
        Returns:
            bool: True if switch successful
        """
        try:
            # Clear cached to force recreation
            self._lateral = None
            self._lateral_type = new_type
            
            # Create new controller
            controller = self.get_lateral_controller(new_type)
            
            # Update config for persistence
            if self.config and controller:
                self.config.config['lateral_controller_type'] = new_type
            
            return controller is not None
            
        except Exception as e:
            if self.logger:
                self.logger.log_error(f"Failed to switch lateral controller to {new_type}", e)
            return False
    
    def update_steering_waypoints(self, new_waypoints):
        """
        Update waypoints for steering controller.
        
        Args:
            new_waypoints: New waypoint array
        """
        if self._steering_controller and hasattr(self._steering_controller, 'reset'):
            self._steering_controller.reset(new_waypoints)
            self._steering_controller_waypoints = new_waypoints
            if self.logger:
                self.logger.logger.info("[ControllerManager] Steering controller waypoints updated")
    
    # =============== Utility ===============
    
    def reset_controllers(self):
        """Reset all controller states (e.g., integral terms)."""
        if self._longitudinal and hasattr(self._longitudinal.controller, 'reset'):
            self._longitudinal.controller.reset()
        if self._lateral and hasattr(self._lateral.controller, 'reset'):
            self._lateral.controller.reset()
        if self._steering_controller and hasattr(self._steering_controller, 'reset'):
            # Don't clear waypoints, just reset state
            pass
        if self.logger:
            self.logger.logger.debug("[ControllerManager] Controllers reset")
    
    def clear(self):
        """Clear all cached controllers (called on shutdown)."""
        self._longitudinal = None
        self._lateral = None
        self._steering_controller = None
        self._steering_controller_waypoints = None
        if self.logger:
            self.logger.logger.debug("[ControllerManager] Controllers cleared")
    
    def __repr__(self):
        return (
            f"ControllerManager(\n"
            f"  longitudinal: {self.get_longitudinal_type()}\n"
            f"  lateral: {self.get_lateral_type()}\n"
            f")"
        )


