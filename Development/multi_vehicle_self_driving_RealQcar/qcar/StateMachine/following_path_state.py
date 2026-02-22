"""
Following Path State - Simplified Event-Driven Implementation

Handles autonomous path following using predefined waypoints.
Uses single handle_event method for command processing.
"""

import time
import numpy as np
from typing import Dict, Any, Tuple, Optional
from .state_base import StateBase
from .vehicle_state import VehicleState, StateTransitionReason
from .following_path import (
    build_pp_waypoint_array,
    extract_lane_data,
    project_to_route_frenet,
    update_pp_runtime_speed_profile,
)

# Import CommandType once at module level
import sys
import os

# Add parent directory to sys.path for imports
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

try:
    from command_types import CommandType

    COMMAND_TYPE_AVAILABLE = True
except ImportError as e:
    print(f"ERROR: Cannot import CommandType: {e}")
    COMMAND_TYPE_AVAILABLE = False
    CommandType = None


# Import LaneFusion for modular lane-assisted path following
try:
    from Controller.LaneFusion import LaneFusion, LaneFusionConfig, FusionStrategy
    from Controller.LaneFusion.config_loader_lane_fusion import get_lane_fusion_config

    LANE_FUSION_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: LaneFusion not available: {e}")
    LANE_FUSION_AVAILABLE = False
    LaneFusion = None
    LaneFusionConfig = None
    FusionStrategy = None
    get_lane_fusion_config = None


# Import MPC controller directly (standalone, no wrappers needed)
try:
    from Controller.mpc_controller import CasADiMPCController, MPCControllerFactory

    MPC_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: MPC controller not available: {e}")
    MPC_AVAILABLE = False
    CasADiMPCController = None
    MPCControllerFactory = None

# Import map-based PP controller and rich path planner (optional)
try:
    from Controller.PP_controller import PP_Controller

    PP_MAP_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: PP controller not available: {e}")
    PP_MAP_AVAILABLE = False
    PP_Controller = None

try:
    from PathPlanner.path_rich import RichSDCSPlanner

    PATH_RICH_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: path_rich planner not available: {e}")
    PATH_RICH_AVAILABLE = False
    RichSDCSPlanner = None


class FollowingPathState(StateBase):
    """Handler for FOLLOWING_PATH state with simplified event handling"""

    def __init__(self, vehicle_logic):
        """Initialize the following path state"""
        super().__init__(vehicle_logic)

        # State-specific controllers (initialized when needed)
        self.steering_controller = None
        self.speed_controller = None
        self._controllers_initialized = False

        # Lane fusion system for combining waypoint steering with lane detection
        # Lane detection runs in yolo_server, results received via yolo_data dict
        self.lane_fusion = None
        self._lane_fusion_enabled = False

        # MPC controller for combined throttle + steering (test mode)
        self.mpc_controller = None
        self._use_mpc = False  # Set to False to revert to original PID+Stanley control

        # Optional map-based PP controller mode
        self._use_pp_map = False
        self.pp_controller = None
        self.pp_map_params = {}
        self.rich_planner = None
        self.pp_waypoint_array = None  # Nx7 with PP expected columns
        self.pp_s_axis = None  # Route distance axis [m]
        self.pp_track_length = 0.0
        self.pp_speed_profile_base = None  # Base speed profile from path_rich
        self.pp_profile_design_speed = 0.0  # desired_speed used to build base profile
        self.pp_last_v_ref = None  # For lightweight change logging

    def _init_controllers(self, force: bool = False):
        """
        Initialize controllers for this state using ControllerManager.
        Controllers are created and cached centrally in controller_manager.
        """
        # self.logger.logger.info("[PATH] Initializing controllers...")
        if self._controllers_initialized and not force:
            return

        # Get controllers from ControllerManager (creates them if needed)
        if hasattr(self.vehicle_logic, "controller_manager"):
            cm = self.vehicle_logic.controller_manager
            lateral_type = cm.get_lateral_type(state="path")
            self._use_pp_map = lateral_type == "pp_map"

            # Speed controller (PID for path following)
            # TODO : Instead of get_speed_controller(), use get_longitudinal_controller()
            # Currently only PID is available for path following
            self.speed_controller = cm.get_speed_controller()
            if self.speed_controller:
                self.vehicle_logic.speed_controller = (
                    self.speed_controller
                )  # Backward compatibility
                self.logger.logger.info(
                    "[PATH] Speed controller obtained from ControllerManager"
                )

            if self._use_pp_map:
                self._init_pp_mode()
                # Keep a conventional steering controller cached for fallback/recovery.
                self.steering_controller = cm.get_steering_controller()
                self.vehicle_logic.steering_controller = self.steering_controller
                if self.pp_controller is not None:
                    self.logger.logger.info("[PATH] PP map controller initialized")
                else:
                    self.logger.logger.warning(
                        "[PATH] PP map requested but unavailable - using steering controller fallback"
                    )
                    self._use_pp_map = False
            else:
                # Steering controller (uses waypoints from vehicle_logic)
                self.steering_controller = cm.get_steering_controller()
                if self.steering_controller:
                    self.vehicle_logic.steering_controller = (
                        self.steering_controller
                    )  # Backward compatibility
                    self.logger.logger.info(
                        "[PATH] Steering controller obtained from ControllerManager"
                    )
        else:
            self.logger.logger.warning("[PATH] ControllerManager not available")

        # Initialize MPC controller for combined throttle+steering (test mode)
        if self._use_mpc and MPC_AVAILABLE:
            try:
                mpc_params = {
                    "horizon": 15,  # 15×0.05=0.75s lookahead (enough to see curves)
                    "dt_mpc": 0.05,
                    "Q_pos": 200.0,  # Position tracking (moderate to avoid over-correction)
                    "Q_heading": 100.0,  # Heading tracking (align with path tangent)
                    "Q_vel": 50.0,  # Higher velocity weight: maintain forward speed!
                    "R_delta": 5.0,  # Moderate: smooth steering
                    "R_acc": 5.0,  # Moderate: smooth throttle
                    "R_delta_rate": 15.0,  # Penalize jerky steering
                    "R_acc_rate": 15.0,  # Penalize jerky throttle (prevents oscillation)
                    "Qf_pos": 200.0,  # Terminal position
                    "Qf_heading": 200.0,  # Terminal heading
                    "Qf_vel": 50.0,  # Terminal velocity (keep moving!)
                    "path_lookahead_scale": 1,  # More lookahead for smoother curves
                    "desired_spacing": 0.5,
                    "max_steering_rate": 1.0,  # Reasonable steering rate
                }
                # Use factory to create standalone MPC (auto-loads QCar vehicle params)
                self.mpc_controller = MPCControllerFactory.create(
                    "mpc",
                    params=mpc_params,
                    logger=self.logger.logger if self.logger else None,
                )

                # Pass waypoints to MPC so it follows the path (like Stanley does)
                if (
                    hasattr(self.vehicle_logic, "waypoint_sequence")
                    and self.vehicle_logic.waypoint_sequence is not None
                ):
                    self.mpc_controller.set_waypoints(
                        self.vehicle_logic.waypoint_sequence, cyclic=True
                    )
                    self.logger.logger.info(
                        f"[PATH] MPC loaded {self.vehicle_logic.waypoint_sequence.shape[1]} waypoints"
                    )
                else:
                    self.logger.logger.warning(
                        "[PATH] MPC initialized but no waypoints available yet"
                    )

                self.logger.logger.info(
                    "[PATH] MPC controller initialized for path following (standalone)"
                )
            except Exception as e:
                self.logger.log_error(
                    "Failed to initialize MPC controller, falling back to PID+Stanley",
                    e,
                )
                self.mpc_controller = None
                self._use_mpc = False
        elif self._use_mpc and not MPC_AVAILABLE:
            self.logger.logger.warning(
                "[PATH] MPC requested but not available (casadi not installed?)"
            )
            self._use_mpc = False

        # Initialize Lane Fusion system for lane-assisted path following
        self._init_lane_fusion()

        self._controllers_initialized = True

    # ------------------------------------------------------------------
    # Runtime controller switch hook (overrides StateBase default)
    # ------------------------------------------------------------------
    def _on_controller_switched(
        self, category: str, controller_type: str, state_context: str
    ) -> None:
        """
        React to a runtime controller switch from the GS GUI.

        For coupled controllers (pp_map, mpc) that compute both throttle and
        steering in a single call, switching only the *lateral* axis must
        also update the longitudinal pipeline (and vice-versa).
        """
        cm = self.vehicle_logic.controller_manager

        if state_context != "path":
            # Not our state — fall back to the default full re-init.
            super()._on_controller_switched(category, controller_type, state_context)
            return

        # --- Lateral switch in path-following mode ---
        if category == "lateral":
            was_pp_map = self._use_pp_map
            now_pp_map = controller_type == "pp_map"

            if was_pp_map and not now_pp_map:
                # ---- Leaving pp_map → switch to PID speed + steering ----
                self._use_pp_map = False
                self._use_mpc = False
                self.pp_controller = None  # deactivate PP pipeline

                # Ensure PID speed controller is active
                self.speed_controller = cm.get_speed_controller()
                if self.speed_controller:
                    self.vehicle_logic.speed_controller = self.speed_controller

                # Activate conventional steering controller
                self.steering_controller = cm.get_steering_controller()
                if self.steering_controller:
                    self.vehicle_logic.steering_controller = self.steering_controller

                self.logger.logger.info(
                    f"[PATH] PP map deactivated → PID speed + {controller_type} steering"
                )

            elif not was_pp_map and now_pp_map:
                # ---- Entering pp_map → initialize PP pipeline ----
                self._use_pp_map = True
                self._use_mpc = False
                self._init_pp_mode()

                # Keep a conventional steering controller as fallback
                self.steering_controller = cm.get_steering_controller()
                if self.steering_controller:
                    self.vehicle_logic.steering_controller = self.steering_controller

                if self.pp_controller is not None:
                    self.logger.logger.info("[PATH] PP map controller activated")
                else:
                    self.logger.logger.warning(
                        "[PATH] PP map requested but unavailable — falling back"
                    )
                    self._use_pp_map = False

            else:
                # Same regime (non-coupled → non-coupled), just refresh steering
                self.steering_controller = cm.get_steering_controller()
                if self.steering_controller:
                    self.vehicle_logic.steering_controller = self.steering_controller
                self.logger.logger.info(
                    f"[PATH] Lateral controller switched to {controller_type}"
                )

        elif category == "longitudinal":
            # Longitudinal switch — re-fetch speed controller
            self.speed_controller = cm.get_speed_controller()
            if self.speed_controller:
                self.vehicle_logic.speed_controller = self.speed_controller
            self.logger.logger.info(
                f"[PATH] Longitudinal controller switched to {controller_type}"
            )

        else:
            # Unknown category — fall back to base
            super()._on_controller_switched(category, controller_type, state_context)

    def _init_pp_mode(self):
        """Initialize map-based PP controller and rich waypoint representation."""
        self.pp_controller = None
        self.pp_waypoint_array = None
        self.pp_s_axis = None
        self.pp_track_length = 0.0
        self.pp_speed_profile_base = None
        self.pp_profile_design_speed = 0.0
        self.pp_last_v_ref = None

        if not PP_MAP_AVAILABLE:
            self.logger.logger.warning("[PATH] PP controller module is not available")
            return
        if not PATH_RICH_AVAILABLE:
            self.logger.logger.warning("[PATH] path_rich module is not available")
            return
        if not hasattr(self.vehicle_logic, "controller_manager"):
            return

        cm = self.vehicle_logic.controller_manager
        params = {}
        if cm.config:
            params = cm.config.get_lateral_params("pp_map")
        self.pp_map_params = params

        wheelbase = 0.256
        if cm.config:
            wheelbase = cm.config.get_vehicle_params().get("wheelbase", wheelbase)

        loop_rate = float(getattr(self.vehicle_logic, "controller_rate", 100.0))
        state_machine_rate = loop_rate

        self.pp_controller = PP_Controller(
            t_clip_min=params.get("t_clip_min", 0.4),
            t_clip_max=params.get("t_clip_max", 1.8),
            m_l1=params.get("m_l1", 0.35),
            q_l1=params.get("q_l1", 0.15),
            speed_lookahead=params.get("speed_lookahead", 0.15),
            lat_err_coeff=params.get("lat_err_coeff", 0.8),
            acc_scaler_for_steer=params.get("acc_scaler_for_steer", 1.0),
            dec_scaler_for_steer=params.get("dec_scaler_for_steer", 1.0),
            start_scale_speed=params.get("start_scale_speed", 0.2),
            end_scale_speed=params.get("end_scale_speed", 1.0),
            downscale_factor=params.get("downscale_factor", 0.35),
            speed_lookahead_for_steer=params.get("speed_lookahead_for_steer", 0.1),
            prioritize_dyn=params.get("prioritize_dyn", False),
            trailing_gap=params.get("trailing_gap", 0.8),
            trailing_p_gain=params.get("trailing_p_gain", 0.6),
            trailing_i_gain=params.get("trailing_i_gain", 0.0),
            trailing_d_gain=params.get("trailing_d_gain", 0.1),
            blind_trailing_speed=params.get("blind_trailing_speed", 0.2),
            loop_rate=loop_rate,
            wheelbase=wheelbase,
            state_machine_rate=state_machine_rate,
            logger_info=self.logger.logger.info,
            logger_warn=self.logger.logger.warning,
        )

        sample_ds = float(params.get("sample_ds", 0.02))
        path_cfg = getattr(self.config, "path", None)
        self.rich_planner = RichSDCSPlanner(
            leftHandTraffic=getattr(path_cfg, "left_hand_traffic", False),
            useSmallMap=True,
            sample_ds=sample_ds,
            is_cyclic=True,
        )

        if (
            hasattr(self.vehicle_logic, "waypoint_sequence")
            and self.vehicle_logic.waypoint_sequence is not None
        ):
            self._build_pp_waypoint_array(self.vehicle_logic.waypoint_sequence)

    def _build_pp_waypoint_array(self, waypoint_sequence: np.ndarray) -> bool:
        """
        Build PP waypoint map from [2, N] path using rich trajectory conversion.

        PP_Controller expects columns where:
          0:x, 1:y, 2:ref_speed, 5:curvature, 6:heading.
        """
        result = build_pp_waypoint_array(
            rich_planner=self.rich_planner,
            waypoint_sequence=waypoint_sequence,
            params=self.pp_map_params,
            logger=self.logger.logger if self.logger else None,
        )
        if result is None:
            return False

        (
            self.pp_waypoint_array,
            self.pp_speed_profile_base,
            self.pp_s_axis,
            self.pp_track_length,
            design_speed,
        ) = result
        self.pp_profile_design_speed = max(float(design_speed), 1e-3)
        self.pp_last_v_ref = None
        self._update_pp_runtime_speed_profile()
        return True

    def _update_pp_runtime_speed_profile(self) -> None:
        """
        Scale PP waypoint speed profile online using current vehicle v_ref.

        This keeps curvature-based speed shaping from path_rich, while allowing
        runtime speed commands (SET_VELOCITY) to speed up/slow down PP behavior.
        """
        v_ref_runtime = float(max(getattr(self.vehicle_logic, "v_ref", 0.0), 0.0))
        speed_scale = update_pp_runtime_speed_profile(
            pp_waypoint_array=self.pp_waypoint_array,
            pp_speed_profile_base=self.pp_speed_profile_base,
            profile_design_speed=self.pp_profile_design_speed,
            v_ref_runtime=v_ref_runtime,
        )
        if speed_scale is None:
            return

        if self.pp_last_v_ref is None or abs(v_ref_runtime - self.pp_last_v_ref) > 0.03:
            self.pp_last_v_ref = v_ref_runtime
            if (
                hasattr(self.vehicle_logic, "loop_counter")
                and self.vehicle_logic.loop_counter % 10 == 0
            ):
                self.logger.logger.info(
                    f"[PP-PATH] Applied runtime v_ref scaling: v_ref={v_ref_runtime:.2f}, "
                    f"scale={speed_scale:.2f}"
                )

    def _project_to_route_frenet(self, x: float, y: float) -> Tuple[float, float]:
        """Project cartesian pose onto current PP route and return (s, d)."""
        return project_to_route_frenet(
            pp_waypoint_array=self.pp_waypoint_array,
            pp_s_axis=self.pp_s_axis,
            pp_track_length=self.pp_track_length,
            x=x,
            y=y,
        )

    def _compute_pp_control(
        self, dt: float, sensor_data: Dict[str, Any]
    ) -> Tuple[float, float]:
        """Compute control using PP_Controller + PID throttle tracking."""
        if not self.vehicle_logic.controller_manager.config.enable_steering_control:
            return self._compute_speed_control(sensor_data["velocity"], dt), 0.0

        if self.pp_controller is None:
            return self._compute_speed_control(sensor_data["velocity"], dt), 0.0

        if self.pp_waypoint_array is None and hasattr(
            self.vehicle_logic, "waypoint_sequence"
        ):
            self._build_pp_waypoint_array(self.vehicle_logic.waypoint_sequence)
        if self.pp_waypoint_array is None:
            return self._compute_speed_control(sensor_data["velocity"], dt), 0.0
        self._update_pp_runtime_speed_profile()

        x = float(sensor_data["x"])
        y = float(sensor_data["y"])
        theta = float(sensor_data["theta"])
        velocity = float(sensor_data["velocity"])
        acceleration = float(sensor_data.get("acceleration", 0.0))

        s, d = self._project_to_route_frenet(x, y)
        position_in_map = np.array([[x, y, theta]], dtype=float)
        position_in_map_frenet = np.array([s, d, velocity], dtype=float)
        acc_now = np.array([acceleration], dtype=float)

        speed_target = None
        steering = 0.0
        try:
            speed_target, _, _, steering, _, _, _ = self.pp_controller.main_loop(
                state="FOLLOWING_PATH",
                position_in_map=position_in_map,
                waypoint_array_in_map=self.pp_waypoint_array,
                speed_now=velocity,
                opponent=None,
                position_in_map_frenet=position_in_map_frenet,
                acc_now=acc_now,
                track_length=self.pp_track_length,
            )
        except Exception as e:
            self.logger.log_error("PP control step failed", e)
            return self._compute_speed_control(velocity, dt), 0.0

        if speed_target is None or not np.isfinite(speed_target):
            speed_target = self.vehicle_logic.v_ref
        # PP target speed is map/curvature based and may jump near lap wrap
        # or when entering/leaving a hard-turn segment.
        speed_target = max(float(speed_target), 0.0)

        # dt_safe = max(float(dt), 1e-3)
        dt_safe = max(float(dt), 1e-3)

        if self.speed_controller:
            # Closed-loop tracking of PP target speed (supports braking without reversing).
            u = self.speed_controller.update(velocity, speed_target, dt_safe)
        else:
            # Safe fallback if speed controller is missing.
            u = speed_target

        if steering is None or not np.isfinite(steering):
            steering = 0.0
        delta = float(np.clip(steering, -0.5, 0.5))

        if (
            hasattr(self.vehicle_logic, "loop_counter")
            and self.vehicle_logic.loop_counter % 200 == 0
        ):
            idx = self.pp_controller.idx_nearest_waypoint if self.pp_controller else -1
            # NOTE: this throttle is pre-actuator clamp. Final clamp is applied in
            # vehicle_logic using current Gear (DRIVE_1/2/3 max throttle).
            self.logger.logger.info(
                f"[PP-PATH] v={velocity:.2f}, v_cmd={speed_target:.2f}, throttle={u:.3f}, "
                f"steer={delta:.3f}, s={s:.2f}, d={d:.2f}, wp={idx}"
            )

        return u, delta

    def _init_lane_fusion(self, config: dict = None):
        """
        Initialize the modular lane fusion system from YAML config.

        The lane fusion system combines waypoint-based steering with lane detection
        corrections to improve path following, especially with poor GPS.

        Configuration is loaded from: Controller/LaneFusion/config_lane_fusion.yaml

        Args:
            config: Optional override configuration dict
        """
        if not LANE_FUSION_AVAILABLE:
            self.logger.logger.warning(
                "[PATH] LaneFusion not available - lane assist disabled"
            )
            return

        try:
            # Load configuration from YAML file
            yaml_config = get_lane_fusion_config()
            settings = yaml_config.get_fusion_settings()

            # Check if enabled in config
            if not settings.enabled:
                self.logger.logger.info("[PATH] Lane Fusion disabled in config")
                self._lane_fusion_enabled = False
                return

            # Build config dict from YAML settings
            fusion_settings = {
                "strategy": settings.strategy,
                "max_lane_weight": settings.max_lane_weight,
                "min_confidence": settings.min_confidence,
                "lane_gain": settings.lane_gain,
                "smoothing_factor": settings.smoothing_factor,
                "max_steering": settings.max_steering,
                "deadband": settings.deadband,
                "switch_threshold": settings.switch_threshold,
                "enable_curvature_compensation": settings.enable_curvature_compensation,
                "debug_logging": settings.debug_logging,
            }

            # Override with provided config (for programmatic changes)
            if config:
                fusion_settings.update(config)

            # Create fusion config
            fusion_config = LaneFusionConfig(
                strategy=FusionStrategy(fusion_settings["strategy"]),
                max_lane_weight=fusion_settings["max_lane_weight"],
                min_confidence=fusion_settings["min_confidence"],
                lane_gain=fusion_settings["lane_gain"],
                smoothing_factor=fusion_settings["smoothing_factor"],
                max_steering=fusion_settings["max_steering"],
                deadband=fusion_settings["deadband"],
                switch_threshold=fusion_settings["switch_threshold"],
                enable_curvature_compensation=fusion_settings[
                    "enable_curvature_compensation"
                ],
                debug_logging=fusion_settings["debug_logging"],
            )

            # Create lane fusion instance
            self.lane_fusion = LaneFusion(config=fusion_config, logger=self.logger)
            self._lane_fusion_enabled = True

            self.logger.logger.info(
                f"[PATH] Lane Fusion initialized from YAML: strategy={fusion_settings['strategy']}, "
                f"max_weight={fusion_settings['max_lane_weight']}, min_conf={fusion_settings['min_confidence']}"
            )

        except Exception as e:
            self.logger.log_error("Failed to initialize Lane Fusion", e)
            self._lane_fusion_enabled = False

    def configure_lane_fusion(self, **kwargs):
        """
        Configure lane fusion parameters at runtime.

        Args:
            strategy: 'adaptive', 'weighted', 'cascaded', 'switch', 'lane_priority'
            max_lane_weight: Maximum lane steering influence (0.0-1.0)
            min_confidence: Minimum lane confidence to use (0.0-1.0)
            lane_gain: Gain applied to lane steering correction
            enabled: Enable/disable lane fusion

        Example:
            following_state.configure_lane_fusion(
                strategy='cascaded',
                max_lane_weight=0.5,
                enabled=True
            )
        """
        if "enabled" in kwargs:
            self._lane_fusion_enabled = kwargs.pop("enabled")
            self.logger.logger.info(
                f"[PATH] Lane fusion enabled: {self._lane_fusion_enabled}"
            )

        if self.lane_fusion is None:
            return

        # Update individual parameters
        for key, value in kwargs.items():
            if key == "strategy":
                self.lane_fusion.config.strategy = FusionStrategy(value)
            elif hasattr(self.lane_fusion.config, key):
                setattr(self.lane_fusion.config, key, value)

        self.logger.logger.info(f"[PATH] Lane fusion config updated: {kwargs}")

    def update_path(self, new_waypoint_sequence: np.ndarray):
        """
        Update the waypoint sequence and reset the steering controller

        Args:
            new_waypoint_sequence: New waypoint sequence to follow
        """
        try:
            # Update vehicle logic waypoint sequence
            self.vehicle_logic.waypoint_sequence = new_waypoint_sequence

            if self._use_pp_map:
                if self._build_pp_waypoint_array(new_waypoint_sequence):
                    self.logger.logger.info(
                        "[PATH] PP map waypoints rebuilt from new path"
                    )
                else:
                    self.logger.logger.warning(
                        "[PATH] Failed to rebuild PP map waypoints from new path"
                    )

            # Reset steering controller with new waypoints
            if self.steering_controller:
                self.steering_controller.reset(new_waypoint_sequence)
                self.logger.logger.info(
                    "[PATH] Steering controller updated with new path"
                )

            # Also update MPC controller waypoints
            if self.mpc_controller is not None and hasattr(
                self.mpc_controller, "set_waypoints"
            ):
                self.mpc_controller.set_waypoints(new_waypoint_sequence, cyclic=True)
                self.logger.logger.info(
                    f"[PATH] MPC updated with {new_waypoint_sequence.shape[1]} waypoints"
                )

            if not self.steering_controller and self.mpc_controller is None:
                # Try to initialize if not already done
                self._init_controllers()

        except Exception as e:
            self.logger.log_error("Failed to update steering controller path", e)

    def enter(self) -> bool:
        """Initialize path following mode"""
        super().enter()
        self.logger.logger.info("[PATH] Entering FOLLOWING_PATH state")

        # Initialize controllers if not already done
        self._init_controllers()

        # Initialize state data
        self.state_data = {
            "session_start_time": time.time(),
            "lap_count": 0,
            "last_waypoint_index": 0,
            "navigation_to_start_completed": False,
            "path_following_active": False,
        }

        # Reset speed controller integral to prevent windup
        if self.speed_controller:
            if hasattr(self.speed_controller, "ei"):
                self.speed_controller.ei = 0
            elif hasattr(self.speed_controller, "reset"):
                self.speed_controller.reset()
            self.logger.logger.info("Speed controller reset")
        elif hasattr(self.vehicle_logic, "speed_controller"):
            # Fallback to vehicle_logic controller if state controller not available
            if hasattr(self.vehicle_logic.speed_controller, "ei"):
                self.vehicle_logic.speed_controller.ei = 0
            elif hasattr(self.vehicle_logic.speed_controller, "reset"):
                self.vehicle_logic.speed_controller.reset()
            self.logger.logger.info("Speed controller reset (fallback)")

        # Get current waypoint index for continuity
        if self.steering_controller:
            self.state_data["last_waypoint_index"] = (
                self.steering_controller.get_waypoint_index()
            )
            self.logger.logger.info(
                f"Continuing from waypoint index: {self.state_data['last_waypoint_index']}"
            )
        elif self._use_pp_map and self.pp_controller is not None:
            idx = self.pp_controller.idx_nearest_waypoint
            self.state_data["last_waypoint_index"] = int(idx) if idx is not None else 0
            self.logger.logger.info(
                f"Continuing from PP waypoint index: {self.state_data['last_waypoint_index']}"
            )
        else:
            self.logger.logger.warning("[PATH] No steering controller available")

        return True

    def update(
        self, dt: float, sensor_data: Dict[str, Any]
    ) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        """Update path following control"""

        # print(sensor_data)
        # Extract sensor data
        x = sensor_data["x"]
        y = sensor_data["y"]
        theta = sensor_data["theta"]
        velocity = sensor_data["velocity"]

        # Handle startup delay
        if self.vehicle_logic.elapsed_time() < self.config.timing.start_delay:
            return 0.0, 0.0, None

        # === CONTROL COMPUTATION ===

        if self._use_mpc and self.mpc_controller is not None:
            # --- MPC path following (waypoints loaded in _init_controllers) ---
            follower_state = {
                "x": x,
                "y": y,
                "theta": theta,
                "velocity": velocity,
                "target_velocity": self.vehicle_logic.v_ref,
            }
            # leader_state=None -> MPC uses waypoints for reference trajectory
            u, delta = self.mpc_controller.compute_control(
                follower_state, leader_state=None, dt=dt
            )
            # u = np.clip(u, -0.1, 0.1)

            # Periodic MPC logging
            if (
                hasattr(self.vehicle_logic, "loop_counter")
                and self.vehicle_logic.loop_counter % 200 == 0
            ):
                wpi = self.mpc_controller.get_waypoint_index()
                n_wp = self.mpc_controller.n_waypoints
                self.logger.logger.info(
                    f"[MPC-PATH] throttle={u:.3f}, steer={delta:.3f}rad "
                    f"({np.rad2deg(delta):.1f}°), v={velocity:.2f}, "
                    f"v_ref={self.vehicle_logic.v_ref:.2f}, wp={wpi}/{n_wp}"
                )
        elif self._use_pp_map and self.pp_controller is not None:
            # --- Map-based PP controller + PID throttle tracking ---
            u, delta = self._compute_pp_control(dt, sensor_data)
        else:
            # --- Original PID + Stanley control ---
            # # Speed control
            # u = self._compute_speed_control(velocity, dt)
            #
            # # Steering control with lane fusion
            # yolo_data = sensor_data.get('yolo_data', None)
            # if yolo_data and not yolo_data.get('is_valid', True):
            #     yolo_data = None
            # delta = self._compute_steering_control(x, y, theta, velocity, yolo_data)

            # Fallback: original controllers (uncomment above to re-enable)
            u = self._compute_speed_control(velocity, dt)
            yolo_data = sensor_data.get("yolo_data", None)
            if yolo_data and not yolo_data.get("is_valid", True):
                yolo_data = None
            delta = self._compute_steering_control(x, y, theta, velocity, yolo_data)

        gear = getattr(self.vehicle_logic, "gear", None)
        max_throttle = float(getattr(gear, "value", 0.1))
        if abs(u) > max_throttle:
            u = np.clip(u, -max_throttle, max_throttle)

        # Monitor progress
        self._monitor_progress()

        # Periodic logging
        self._periodic_logging(x, y, theta, velocity)

        return u, delta, None

    def handle_event(
        self, command_type, data: Dict[str, Any] = None
    ) -> Optional[Tuple[VehicleState, StateTransitionReason]]:
        """
        Handle events while path following.

        Args:
            command_type: CommandType enum (e.g., CommandType.STOP, CommandType.START_PLATOON)
            data: Optional event data

        Returns:
            Optional state transition.
        """
        data = data or {}

        if not COMMAND_TYPE_AVAILABLE:
            self.logger.logger.warning(
                f"CommandType not available in FollowingPathState - using base handler for {command_type}"
            )
            return super().handle_event(command_type, data)

        if command_type == CommandType.START_PLATOON:
            return self._handle_start_platoon_event(data)

        if command_type == CommandType.SET_PATH:
            self._handle_set_path_event(data)
            return None

        # Common events (stop, emergency_stop, set_velocity, ...) are handled by base class.
        return super().handle_event(command_type, data)

    def _handle_start_platoon_event(
        self,
        data: Dict[str, Any],
    ) -> Optional[Tuple[VehicleState, StateTransitionReason]]:
        """Handle START_PLATOON while in FOLLOWING_PATH."""
        self.logger.logger.info(f"[PLATOON] START_PLATOON received with data: {data}")

        if not self.validate_event_data(data, ["leader_id"]):
            self.logger.logger.error("[PLATOON] Missing 'leader_id' in command data!")
            return None

        if not (
            hasattr(self.vehicle_logic, "platoon_controller")
            and self.vehicle_logic.platoon_controller
            and self.vehicle_logic.platoon_controller.setup_complete
        ):
            self.logger.logger.warning(
                "[PLATOON] START_PLATOON rejected - SETUP_PLATOON_FORMATION has not been received yet! "
                "Please send SETUP_PLATOON_FORMATION command first before starting platoon."
            )
            return None

        leader_id = data.get("leader_id")
        is_leader = self.vehicle_logic.platoon_controller.is_leader
        my_position = getattr(
            self.vehicle_logic.platoon_controller, "my_position", None
        )

        self.logger.logger.info(
            f"[PLATOON] START_PLATOON received (leader_id={leader_id})"
        )
        self.logger.logger.info(
            f"[PLATOON] My formation: is_leader={is_leader}, position={my_position}"
        )

        if is_leader:
            self.logger.logger.info(
                "[PLATOON] I am LEADER - staying in FOLLOWING_PATH state"
            )
            self.vehicle_logic.platoon_controller.enabled = True
            return None

        self.logger.logger.info(
            f"[PLATOON] I am FOLLOWER-{my_position} - transitioning to FOLLOWING_LEADER state "
            f"(following vehicle {leader_id})"
        )
        self.vehicle_logic.platoon_controller.enable_as_follower()
        self.vehicle_logic.platoon_controller.leader_car_id = leader_id
        self.vehicle_logic.platoon_controller.enabled = True
        return (VehicleState.FOLLOWING_LEADER, StateTransitionReason.START_COMMAND)

    def _handle_set_path_event(self, data: Dict[str, Any]) -> None:
        """Handle SET_PATH while in FOLLOWING_PATH."""
        node_sequence = data.get("node_sequence")
        if not (node_sequence and isinstance(node_sequence, list)):
            self.logger.logger.warning(f"[!] Invalid path update data: {data}")
            return

        if not (hasattr(self.vehicle_logic, "roadmap") and self.vehicle_logic.roadmap):
            self.logger.logger.warning("[!] No roadmap available for path generation")
            return

        try:
            new_waypoints = self.vehicle_logic.roadmap.generate_path(node_sequence)
            self.update_path(new_waypoints)
            self.logger.logger.info(f"[OK] Path updated in {self.__class__.__name__}")
        except Exception as e:
            self.logger.log_error("Failed to generate path from nodes", e)

    def _should_follow_leader(self, sensor_data: Dict[str, Any]) -> bool:
        """Check if we should transition to following a leader"""
        yolo_data = sensor_data.get("yolo_data", {})

        # Check for cars ahead and platoon mode
        if (
            yolo_data.get("cars", False)
            and hasattr(self.vehicle_logic, "platoon_controller")
            and self.vehicle_logic.platoon_controller
        ):
            car_distance = yolo_data.get("car_dist")
            if car_distance and car_distance < 3.0:  # Car within 3 meters
                # Check if platoon mode should be activated
                if getattr(
                    self.vehicle_logic.platoon_controller,
                    "should_activate",
                    lambda: False,
                )():
                    return True

        return False

    def _check_navigation_to_start(self, x: float, y: float) -> bool:
        """Check if navigation to start position is completed"""
        # If we don't have a steering controller, try to initialize first
        if not self.steering_controller:
            self._init_controllers()

        # If still no steering controller, assume we're ready
        if not self.steering_controller:
            return True

        # Check distance to first waypoint of the planned path
        try:
            target_wp = self.steering_controller.wp[:, 0]
            current_pos = np.array([x, y])
            dist_to_target = np.linalg.norm(target_wp - current_pos)

            if dist_to_target < 0.5:  # Within 50cm of start
                return True
        except:
            # If we can't check, assume navigation is complete
            return True

        return False

    def _compute_speed_control(self, velocity: float, dt: float) -> float:
        """Compute speed control command"""
        if not self.speed_controller:
            return 0.0

        # Apply YOLO adjustments to reference velocity
        yolo_gain = getattr(self.vehicle_logic, "yolo_gain", 1.0)
        v_ref_adjusted = self.vehicle_logic.v_ref * yolo_gain
        # print (f"Adjusted v_ref: {v_ref_adjusted:.2f} m/s (YOLO gain: {yolo_gain:.2f})")
        return self.speed_controller.update(velocity, v_ref_adjusted, dt)

    def _extract_lane_data(self, yolo_data: dict) -> dict:
        """
        Extract and validate lane detection data from YOLO packet.

        Kept as a thin wrapper for backward compatibility.
        """
        return extract_lane_data(yolo_data, min_confidence=0.1)

    def _compute_steering_control(
        self, x: float, y: float, theta: float, velocity: float, yolo_data: dict = None
    ) -> float:
        """
        Compute steering control command with optional lane fusion.

        Lane detection runs in yolo_server_virtual.py (where the camera is).
        Results are sent here via UDP as yolo_data dict.
        LaneFusion converts this dict to standardized format for fusion.

        Args:
            x, y: Vehicle position
            theta: Vehicle heading
            velocity: Current velocity
            yolo_data: Dict with lane data from YOLO server
                      (keys: lane_confidence, lane_steering, lane_slope, lane_intercept,
                             lane_left_detected, lane_right_detected)

        Returns:
            Steering command in radians
        """
        if (
            not self.vehicle_logic.controller_manager.config.enable_steering_control
            or not self.steering_controller
        ):
            return 0.0

        # Primary: Waypoint-based steering (global path following)
        # Add lookahead point slightly ahead of vehicle
        lookahead_offset = 0.2  # meters
        p = (
            np.array([x, y])
            + np.array([np.cos(theta), np.sin(theta)]) * lookahead_offset
        )
        waypoint_steering = self.steering_controller.update(
            p, theta, max(velocity, 0.1)
        )

        # Secondary: Lane-based correction using modular LaneFusion system
        if self._lane_fusion_enabled and self.lane_fusion is not None:
            # LaneFusion internally converts yolo_data to LaneDetectionResult
            fusion_result = self.lane_fusion.compute_steering(
                waypoint_steering=waypoint_steering,
                yolo_data=yolo_data,
                velocity=velocity,
            )
            final_steering = fusion_result.final_steering

            # Log lane fusion activity periodically (every ~0.5 sec at 200Hz)
            if fusion_result.lane_valid and self.vehicle_logic.loop_counter % 100 == 0:
                lane_info = self._extract_lane_data(yolo_data)
                lane_status = "L" if lane_info["left_detected"] else "-"
                lane_status += "R" if lane_info["right_detected"] else "-"
                self.logger.logger.info(
                    f"[LANE] [{lane_status}] conf={fusion_result.lane_confidence:.2f}, "
                    f"wp={waypoint_steering:.3f}, lane={fusion_result.lane_steering:.3f}, "
                    f"w={fusion_result.lane_weight_used:.2f}, final={final_steering:.3f}"
                )
        else:
            # Lane fusion disabled or not available - use waypoint steering only
            final_steering = waypoint_steering

        return np.clip(final_steering, -0.5, 0.5)

    def get_lane_fusion_stats(self) -> dict:
        """
        Get lane fusion statistics for monitoring and debugging.

        Returns:
            Dict with fusion statistics including lane usage rate
        """
        if self.lane_fusion is None:
            return {"enabled": False, "lane_fusion_available": False}

        stats = self.lane_fusion.get_statistics()
        stats["enabled"] = self._lane_fusion_enabled
        stats["lane_fusion_available"] = True
        return stats

    def _monitor_progress(self):
        """Monitor waypoint progress and lap completion"""
        if self._use_pp_map and self.pp_controller is not None:
            idx = self.pp_controller.idx_nearest_waypoint
            current_waypoint_index = int(idx) if idx is not None else 0
            total_waypoints = (
                self.pp_waypoint_array.shape[0]
                if self.pp_waypoint_array is not None
                else 0
            )
        elif self.steering_controller:
            current_waypoint_index = self.steering_controller.get_waypoint_index()
            total_waypoints = (
                self.vehicle_logic.waypoint_sequence.shape[1]
                if hasattr(self.vehicle_logic, "waypoint_sequence")
                else 0
            )
        else:
            return

        prev_waypoint_index = self.state_data["last_waypoint_index"]
        if current_waypoint_index != prev_waypoint_index:
            self.state_data["last_waypoint_index"] = current_waypoint_index

            # Check if we've completed a lap
            if total_waypoints > 0:
                if (
                    current_waypoint_index == 0
                    and prev_waypoint_index > total_waypoints * 0.8
                ):
                    self.state_data["lap_count"] += 1
                    lap_time = time.time() - self.state_data["session_start_time"]
                    self.logger.logger.info(
                        f"🏁 Completed lap {self.state_data['lap_count']} in {lap_time:.1f}s"
                    )
                    self.state_data["session_start_time"] = (
                        time.time()
                    )  # Reset for next lap

    def _periodic_logging(self, x: float, y: float, theta: float, velocity: float):
        """Log performance metrics periodically"""
        if not hasattr(self.vehicle_logic, "loop_counter"):
            return

        if self.vehicle_logic.loop_counter % 200 != 0:
            return

        if self._use_pp_map and self.pp_controller is not None:
            idx = (
                self.pp_controller.idx_nearest_waypoint
                if self.pp_controller.idx_nearest_waypoint is not None
                else -1
            )
            self.logger.logger.debug(
                f"Path following (PP) - WP: {idx}, V: {velocity:.2f}m/s"
            )
            return

        if self.steering_controller:
            errors = self.steering_controller.get_errors()
            cross_track_error = errors[0]
            heading_error = errors[1]

            self.logger.logger.debug(
                f"Path following - CTE: {cross_track_error:.3f}m, "
                f"HE: {heading_error:.3f}rad, V: {velocity:.2f}m/s"
            )

    def exit(self):
        """Clean up when leaving path following state"""
        self.logger.logger.info("[PATH] Exiting FOLLOWING_PATH state")

        # Log final statistics
        session_time = self.get_time_in_state()
        self.logger.logger.info(f"Path following session duration: {session_time:.1f}s")

        if self.state_data["lap_count"] > 0:
            self.logger.logger.info(
                f"Completed {self.state_data['lap_count']} laps in this session"
            )

        # Log final waypoint position
        if self.steering_controller:
            final_waypoint_index = self.steering_controller.get_waypoint_index()
            self.logger.logger.info(f"Final waypoint index: {final_waypoint_index}")
        elif self._use_pp_map and self.pp_controller is not None:
            idx = self.pp_controller.idx_nearest_waypoint
            final_waypoint_index = int(idx) if idx is not None else -1
            self.logger.logger.info(f"Final PP waypoint index: {final_waypoint_index}")

        super().exit()

    def _on_params_updated(
        self, category: str, params: dict, state_context: str
    ) -> None:
        """
        React to parameter updates from the GUI.
        If path parameters change for pp_map, regenerate the waypoint array.
        """
        # We only care about lateral parameters for the path state when using pp_map
        if category == "lateral" and state_context == "path" and self._use_pp_map:
            if self.pp_map_params is not None:
                self.pp_map_params.update(params)
            else:
                self.pp_map_params = params

            if (
                hasattr(self.vehicle_logic, "waypoint_sequence")
                and self.vehicle_logic.waypoint_sequence is not None
            ):
                self._build_pp_waypoint_array(self.vehicle_logic.waypoint_sequence)
                if self.logger:
                    self.logger.logger.info(
                        "[PP-PATH] Waypoint array rebuilt to reflect dynamic parameter changes"
                    )
