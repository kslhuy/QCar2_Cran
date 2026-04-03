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
    type_name: str  # Type name (e.g., 'pid', 'cacc', 'pure_pursuit', 'stanley')
    category: str  # 'longitudinal' or 'lateral'


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

    # Controller types that compute BOTH longitudinal and lateral outputs in a
    # single call. When switching only one axis (e.g. lateral), the state must
    # also handle the other axis to avoid leaving stale control logic active.
    COUPLED_CONTROLLERS = {"pp_map", "mpc"}

    @classmethod
    def is_coupled(cls, ctrl_type: str) -> bool:
        """Return True if *ctrl_type* handles both lon and lat jointly."""
        return ctrl_type in cls.COUPLED_CONTROLLERS

    def __init__(
        self,
        logger=None,
        config=None,
        vehicle_type="QCar",
        vehicle_geometry=None,
    ):
        """
        Initialize ControllerManager.

        Args:
            logger: Logger instance
            vehicle_type: Type of the vehicle (e.g., 'QCar' or 'Limo')
        """
        self.config = config if config else get_controller_config()
        self.logger = logger
        self.vehicle_type = vehicle_type
        self.vehicle_geometry = self._normalize_vehicle_geometry(vehicle_geometry)
        if (
            self.vehicle_geometry
            and hasattr(self.config, "set_vehicle_params_override")
        ):
            self.config.set_vehicle_params_override(self.vehicle_geometry)

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
                f"[ControllerManager] Initialized: "
                f"path[long={self._path_longitudinal_type}, lat={self._path_lateral_type}], "
                f"leader[long={self._leader_longitudinal_type}, lat={self._leader_lateral_type}]"
            )

    @staticmethod
    def _normalize_vehicle_geometry(vehicle_geometry) -> Dict[str, float]:
        """Convert runtime geometry data into a plain numeric dictionary."""
        if isinstance(vehicle_geometry, dict):
            source = vehicle_geometry
        else:
            source = getattr(vehicle_geometry, "__dict__", {}) or {}

        normalized: Dict[str, float] = {}
        for key in ("wheelbase", "l_r", "l_f", "track"):
            value = source.get(key)
            if value is not None:
                normalized[key] = float(value)
        return normalized

    def _load_config(self):
        """Load controller types and params from config."""
        # Legacy single-type (kept for backward compat)
        self._longitudinal_type = "cacc"
        self._lateral_type = "pure_pursuit"

        # State-specific types
        self._path_longitudinal_type = "pid"
        self._path_lateral_type = "pp_map"
        self._leader_longitudinal_type = "cacc"
        self._leader_lateral_type = "pure_pursuit"

        try:
            if self.config:
                self._longitudinal_type = self.config.get_longitudinal_controller_type()
                self._lateral_type = self.config.get_lateral_controller_type()

                # State-specific overrides
                self._path_longitudinal_type = self.config.get_path_longitudinal_type()
                self._path_lateral_type = self.config.get_path_lateral_type()
                self._leader_longitudinal_type = (
                    self.config.get_leader_longitudinal_type()
                )
                self._leader_lateral_type = self.config.get_leader_lateral_type()
        except Exception as e:
            if self.logger:
                self.logger.log_warning(
                    f"Failed to load controller types from config: {e}"
                )

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
                
            params['vehicle_type'] = self.vehicle_type

            controller = ControllerFactory.create(
                ctrl_type,
                params,
                logger=self.logger.logger if self.logger else None,
                config=self.config,
            )

            self._longitudinal = ControllerInfo(
                controller=controller, type_name=ctrl_type, category="longitudinal"
            )

            if self.logger:
                self.logger.logger.info(
                    f"[ControllerManager] Created longitudinal controller: {ctrl_type}"
                )

            return controller

        except Exception as e:
            if self.logger:
                self.logger.log_error(
                    f"Failed to create longitudinal controller {ctrl_type}", e
                )
            return None

    def get_lateral_controller(self, force_type: str = None, waypoints=None):
        """
        Get or create lateral controller.

        Args:
            force_type: Override config type (e.g., 'pure_pursuit', 'stanley', 'lookahead', 'pp_map')
            waypoints: Waypoints for path-based controllers (optional)

        Returns:
            Lateral controller instance
        """
        ctrl_type = force_type or self._lateral_type

        # Handle 'path' mode separately - returns steering controller
        if ctrl_type == "path":
            return self.get_steering_controller(waypoints)
        if ctrl_type == "pp_map":
            controller = self.get_steering_controller(waypoints)
            if controller is not None:
                self._lateral = ControllerInfo(
                    controller=controller, type_name="pp_map", category="lateral"
                )
            return controller

        # Return cached if same type
        if self._lateral and self._lateral.type_name == ctrl_type:
            return self._lateral.controller

        # Create new controller
        try:
            from Controller.lateral_controllers import LateralControllerFactory

            params = {}
            if self.config:
                params = self.config.get_lateral_params(ctrl_type)

            params['vehicle_type'] = self.vehicle_type

            controller = LateralControllerFactory.create(
                ctrl_type,
                params,
                logger=self.logger.logger if self.logger else None,
                config=self.config,
            )

            self._lateral = ControllerInfo(
                controller=controller, type_name=ctrl_type, category="lateral"
            )

            if self.logger:
                self.logger.logger.info(
                    f"[ControllerManager] Created lateral controller: {ctrl_type}"
                )

            return controller

        except Exception as e:
            if self.logger:
                self.logger.log_error(
                    f"Failed to create lateral controller {ctrl_type}", e
                )
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
            waypoints = getattr(self.vehicle_logic, "waypoint_sequence", None)

        if waypoints is None:
            if self.logger:
                self.logger.logger.warning(
                    "[ControllerManager] No waypoints for steering controller"
                )
            return None

        # Return cached if waypoints unchanged
        if (
            self._steering_controller is not None
            and self._steering_controller_waypoints is waypoints
        ):
            return self._steering_controller

        # Create new steering controller
        try:
            from Controller.lateral_controllers import StanleyController

            controller = StanleyController(
                waypoints=waypoints, config=self.config, logger=self.logger, cyclic=True
            )

            self._steering_controller = controller
            self._steering_controller_waypoints = waypoints

            # Also track as lateral controller (for telemetry)
            self._lateral = ControllerInfo(
                controller=controller, type_name="stanley", category="lateral"
            )

            if self.logger:
                self.logger.logger.info(
                    "[ControllerManager] Created steering controller (Stanley)"
                )

            return controller

        except Exception as e:
            if self.logger:
                self.logger.log_error("Failed to create steering controller", e)
            return None

    def get_speed_controller(self):
        """
        Get PID speed controller (for FOLLOWING_PATH state).
        Convenience wrapper around get_longitudinal_controller('pid').

        Returns:
            PIDVelocityController instance
        """
        return self.get_longitudinal_controller("pid")

    # =============== Type Getters (for telemetry) ===============

    def get_longitudinal_type(self, state: str = None) -> str:
        """Get longitudinal controller type for a given state.

        Args:
            state: 'path' or 'leader'. None returns currently active type.
        """
        if state == "path":
            return self._path_longitudinal_type
        elif state == "leader":
            return self._leader_longitudinal_type
        # Fallback: active controller or legacy default
        if self._longitudinal:
            return self._longitudinal.type_name
        return self._longitudinal_type

    def get_lateral_type(self, state: str = None) -> str:
        """Get lateral controller type for a given state.

        Args:
            state: 'path' or 'leader'. None returns currently active type.
        """
        if state == "path":
            return self._path_lateral_type
        elif state == "leader":
            return self._leader_lateral_type
        # Fallback: active controller or legacy default
        if self._lateral:
            return self._lateral.type_name
        return self._lateral_type

    def get_status(self) -> Dict[str, str]:
        """Get current controller status for telemetry."""
        return {
            "longitudinal_ctrl_type": self.get_longitudinal_type(),
            "lateral_ctrl_type": self.get_lateral_type(),
            "path_long_type": self._path_longitudinal_type,
            "path_lat_type": self._path_lateral_type,
            "leader_long_type": self._leader_longitudinal_type,
            "leader_lat_type": self._leader_lateral_type,
        }

    # =============== Runtime Switching ===============

    def switch_longitudinal(self, new_type: str, state: str = "path") -> bool:
        """
        Switch longitudinal controller at runtime for a specific state.

        Args:
            new_type: New controller type ('cacc', 'pid', 'hybrid')
            state: 'path' or 'leader'

        Returns:
            bool: True if switch successful
        """
        try:
            # Update the state-specific type
            if state == "leader":
                self._leader_longitudinal_type = new_type
            else:
                self._path_longitudinal_type = new_type

            # Clear cached to force recreation
            self._longitudinal = None
            self._longitudinal_type = new_type

            # Create new controller
            controller = self.get_longitudinal_controller(new_type)

            # Update config for persistence
            if self.config and controller:
                config_key = f"{state}_longitudinal_controller_type"
                self.config.config[config_key] = new_type
                self.config.config["longitudinal_controller_type"] = new_type

            if self.logger:
                self.logger.logger.info(
                    f"[ControllerManager] Switched {state} longitudinal → {new_type}"
                )

            return controller is not None

        except Exception as e:
            if self.logger:
                self.logger.log_error(
                    f"Failed to switch longitudinal controller to {new_type}", e
                )
            return False

    def switch_lateral(self, new_type: str, state: str = "path") -> bool:
        """
        Switch lateral controller at runtime for a specific state.

        Args:
            new_type: New controller type ('pure_pursuit', 'stanley', 'lookahead', 'hybrid', 'path', 'pp_map')
            state: 'path' or 'leader'

        Returns:
            bool: True if switch successful
        """
        try:
            # Update the state-specific type
            if state == "leader":
                self._leader_lateral_type = new_type
            else:
                self._path_lateral_type = new_type

            # Clear cached to force recreation
            self._lateral = None
            self._lateral_type = new_type

            # Create new controller
            controller = self.get_lateral_controller(new_type)

            # Update config for persistence
            # coupled types such as pp_map and mpc may not immediately produce a
            # controller instance, but we still want to remember the choice
            if self.config and (controller is not None or new_type in ("pp_map", "mpc")):
                config_key = f"{state}_lateral_controller_type"
                self.config.config[config_key] = new_type
                self.config.config["lateral_controller_type"] = new_type

            if self.logger:
                self.logger.logger.info(
                    f"[ControllerManager] Switched {state} lateral → {new_type}"
                )

            return controller is not None or new_type in ("pp_map", "mpc")

        except Exception as e:
            if self.logger:
                self.logger.log_error(
                    f"Failed to switch lateral controller to {new_type}", e
                )
            return False

    def update_steering_waypoints(self, new_waypoints):
        """
        Update waypoints for steering controller.

        Args:
            new_waypoints: New waypoint array
        """
        if self._steering_controller and hasattr(self._steering_controller, "reset"):
            self._steering_controller.reset(new_waypoints)
            self._steering_controller_waypoints = new_waypoints
            if self.logger:
                self.logger.logger.info(
                    "[ControllerManager] Steering controller waypoints updated"
                )

    def update_controller_params(
        self, category: str, params: dict, state_context: str = "path"
    ) -> bool:
        """
        Update controller parameters dynamically at runtime and persist to config.

        Args:
            category: 'longitudinal' or 'lateral'
            params: Dictionary of parameters to update
            state_context: 'path' or 'leader'

        Returns:
            bool: True if parameters were successfully updated, False otherwise
        """
        success = False

        # 1. Update active controller if it matches the category and state_context
        # We need to know which controller is active to avoid updating the wrong one
        active_type = None
        controller_info = None

        if category == "longitudinal":
            active_type = (
                self._path_longitudinal_type
                if state_context == "path"
                else self._leader_longitudinal_type
            )
            if self._longitudinal and self._longitudinal.type_name == active_type:
                controller_info = self._longitudinal
        elif category == "lateral":
            active_type = (
                self._path_lateral_type
                if state_context == "path"
                else self._leader_lateral_type
            )
            if self._lateral and self._lateral.type_name == active_type:
                controller_info = self._lateral
            # also handle the case where path lateral is active and it's stanley steering_controller
            elif active_type == "path" and self._steering_controller:
                # the steering controller is stanley
                try:
                    if hasattr(self._steering_controller, "update_params"):
                        self._steering_controller.update_params(params)
                        success = True
                except Exception as e:
                    if self.logger:
                        self.logger.log_error(
                            "Failed to update params on active steering controller", e
                        )

        if controller_info and controller_info.controller:
            try:
                if hasattr(controller_info.controller, "update_params"):
                    controller_info.controller.update_params(params)
                    success = True
                else:
                    if self.logger:
                        self.logger.log_warning(
                            f"Controller {active_type} lacks update_params method"
                        )
            except Exception as e:
                if self.logger:
                    self.logger.log_error(
                        f"Failed to update params on active controller {active_type}", e
                    )

        # Special case for pp_map and MPC since these coupled controllers are
        # managed inside the state machine rather than by ControllerManager.
        if category == "lateral" and active_type in ("pp_map", "mpc"):
            # The state machine (FollowingPathState) holds the controller instance.
            # We attempt to locate it via vehicle_logic and then update it.
            if self.vehicle_logic and hasattr(
                self.vehicle_logic, "current_state_handler"
            ):
                state = self.vehicle_logic.current_state_handler
                ctrl_attr = "pp_controller" if active_type == "pp_map" else "mpc_controller"
                if hasattr(state, ctrl_attr) and getattr(state, ctrl_attr) is not None:
                    controller_obj = getattr(state, ctrl_attr)
                    try:
                        if hasattr(controller_obj, "update_params"):
                            controller_obj.update_params(params)
                            success = True
                    except Exception as e:
                        if self.logger:
                            self.logger.log_error(
                                f"Failed to update params on {ctrl_attr}", e
                            )

        # 2. Update ControllerConfig to persist changes
        if self.config and self.config.config:
            if active_type:
                if active_type not in self.config.config:
                    self.config.config[active_type] = {}
                # Update the config dictionary
                for k, v in params.items():
                    self.config.config[active_type][k] = v
                success = True
                if self.logger:
                    self.logger.logger.info(
                        f"Updated ControllerConfig for {active_type} with params: {params}"
                    )

        return success

    # =============== Utility ===============

    def reset_controllers(self):
        """Reset all controller states (e.g., integral terms)."""
        if self._longitudinal and hasattr(self._longitudinal.controller, "reset"):
            self._longitudinal.controller.reset()
        if self._lateral and hasattr(self._lateral.controller, "reset"):
            self._lateral.controller.reset()
        if self._steering_controller and hasattr(self._steering_controller, "reset"):
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
            f"  path[long={self._path_longitudinal_type}, lat={self._path_lateral_type}]\n"
            f"  leader[long={self._leader_longitudinal_type}, lat={self._leader_lateral_type}]\n"
            f")"
        )
