"""
Following Leader State - Clean and Simple Implementation

Handles following another vehicle (platoon/convoy mode).
Delegates all platoon logic to PlatoonController.

LATERAL CONTROL MODES:
  - 'pure_pursuit', 'stanley', 'lookahead', 'hybrid': Follow leader's position directly
  - 'path', 'pp_map': Follow predefined waypoints (like FOLLOWING_PATH state) while maintaining
            longitudinal spacing with leader

USAGE:
  Set lateral_controller_type in config:
    - For leader tracking: lateral_controller_type = 'pure_pursuit' (or other)
    - For path following: lateral_controller_type = 'path' or 'pp_map'
"""

import time
import numpy as np
from typing import Dict, Any, Tuple, Optional
from .state_base import StateBase
from .vehicle_state import VehicleState, StateTransitionReason

# Import CommandType
import sys
import os

parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

try:
    from command_handler import CommandType
    from Controller.longitudinal_controllers import ControllerFactory

    COMMAND_TYPE_AVAILABLE = True
except ImportError as e:
    print(f"ERROR: Cannot import CommandType: {e}")
    COMMAND_TYPE_AVAILABLE = False
    CommandType = None
    ControllerFactory = None


class FollowingLeaderState(StateBase):
    """Handler for FOLLOWING_LEADER state - uses centralized ControllerManager"""

    def __init__(self, vehicle_logic):
        """Initialize the following leader state"""
        super().__init__(vehicle_logic)

        # Controllers obtained from ControllerManager
        self.longitudinal_controller = None
        self.lateral_controller = None
        self.steering_controller = None  # For path-following lateral mode
        self.lateral_controller_type = (
            None  # 'path', 'fusion', or other (pure_pursuit, stanley, etc.)
        )

        # Cached route projection data for along-track gap estimation
        self._route_waypoint_ref = None
        self._route_xy = None
        self._route_s_axis = None
        self._route_track_length = 0.0
        self.reverse_follow_config: Dict[str, Any] = {}
        self.sensor_acc_config: Dict[str, Any] = {}
        self.leader_source_config: Dict[str, Any] = {}
        self.trust_longitudinal_fusion_config: Dict[str, Any] = {}
        self.multi_predecessor_cacc_config: Dict[str, Any] = {}
        self.sensor_acc_controller = None
        self._reverse_follow_active = False
        self._reverse_heading_limit_rad = np.deg2rad(20.0)
        self._sensor_acc_distance_filtered: Optional[float] = None

        self._prev_u = 0.0
        self._prev_delta = 0.0

    def _smooth_cmd(self, raw: float, prev: float, dt: float, alpha: float, rise_rate: float, fall_rate: float) -> float:
        if prev is None or dt <= 1e-6:
            return raw

        if raw > prev:
            limited = min(raw, prev + rise_rate * dt)
        else:
            limited = max(raw, prev - fall_rate * dt)

        return alpha * prev + (1.0 - alpha) * limited

    def enter(self) -> bool:
        """Initialize leader following mode"""
        super().enter()
        print("[DEBUG] ===============================")
        print("[DEBUG] ENTERING FOLLOWING_LEADER STATE")
        print("[DEBUG] ===============================")
        self.logger.logger.info("[FOLLOW] Entering FOLLOWING_LEADER state")

        # Force trailing mode — never overtake the leader
        try:
            if (
                hasattr(self.vehicle_logic, "yolo_manager")
                and self.vehicle_logic.yolo_manager
                and self.vehicle_logic.yolo_manager.yolo_drive is not None
            ):
                self.vehicle_logic.yolo_manager.yolo_drive.car_overtake_mode = False
                self.logger.logger.info(
                    "[FOLLOW] Car overtake mode forced OFF (trailing leader)"
                )
        except Exception:
            pass

        # Check if formation data exists and set up if missing
        if hasattr(self.vehicle_logic, "platoon_controller"):
            self.vehicle_logic.platoon_controller.enable_as_follower()
            self.logger.logger.info("Platoon controller configured as follower")

        # Get controllers from ControllerManager
        self._init_controllers()
        self._load_reverse_follow_config()
        self._load_trust_longitudinal_fusion_config()
        self._load_sensor_acc_config()
        self._load_leader_source_config()
        self._load_multi_predecessor_cacc_config()

        return True

    def _init_controllers(self, force: bool = False):
        """Get controllers from ControllerManager (creates them if needed)."""
        if not hasattr(self.vehicle_logic, "controller_manager"):
            self.logger.logger.error("[FOLLOW] ControllerManager not available")
            return

        cm = self.vehicle_logic.controller_manager
        longitudinal_type = cm.get_longitudinal_type(state="leader")
        lateral_type = cm.get_lateral_type(state="leader")
        print(f"[FOLLOW] Lateral controller type: {lateral_type}")
        self.lateral_controller_type = lateral_type  # Store for use in _compute_control

        # Get longitudinal controller using leader-specific type
        self.longitudinal_controller = cm.get_longitudinal_controller(
            force_type=longitudinal_type
        )
        if self.longitudinal_controller:
            self.logger.logger.info(
                f"[FOLLOW] Longitudinal controller: {longitudinal_type}"
            )
        else:
            self.logger.logger.error(
                f"[FOLLOW] Longitudinal controller unavailable: {longitudinal_type}"
            )

        # Get lateral controller based on type
        if lateral_type in ("path", "pp_map", "fusion", "fusion_lateral"):
            # Path-following mode (and fusion): use steering controller
            self.steering_controller = cm.get_steering_controller()
            if self.steering_controller:
                self.logger.logger.info(
                    "[FOLLOW] Enabled path-following lateral mode (SteeringController)"
                )

        if lateral_type not in ("path", "pp_map"):
            # Leader-tracking mode: use lateral controller factory
            self.lateral_controller = cm.get_lateral_controller(force_type=lateral_type)
            if self.lateral_controller:
                self.logger.logger.info(f"[FOLLOW] Lateral controller: {lateral_type}")

        # Build projection cache once on entry (refreshes automatically if path object changes)
        self._refresh_route_projection_cache()

    def _load_reverse_follow_config(self):
        """Load conservative reverse-follow safety limits from controller config."""
        reverse_cfg = {
            "enabled": False,
            "trigger_velocity_threshold": -0.03,
            "max_reverse_throttle": 0.08,
            "max_reverse_speed": 0.12,
            "min_gap": 0.20,
            "max_gap": 0.90,
            "max_heading_error_deg": 20.0,
            "reverse_steering_gain": 0.5,
            "stop_on_v2v_loss": True,
        }

        controller_config = getattr(
            getattr(self.vehicle_logic, "controller_manager", None), "config", None
        )
        if controller_config and hasattr(
            controller_config, "get_leader_reverse_follow_config"
        ):
            reverse_cfg.update(controller_config.get_leader_reverse_follow_config())

        self.reverse_follow_config = reverse_cfg
        self._reverse_heading_limit_rad = np.deg2rad(
            max(float(reverse_cfg.get("max_heading_error_deg", 20.0)), 0.0)
        )

    def _load_trust_longitudinal_fusion_config(self):
        """Load trust-aware longitudinal command-fusion settings."""
        fusion_cfg = {
            "enabled": False,
            "trust_low": 0.50,
            "trust_high": 0.80,
            "unavailable_policy": "legacy_cacc",
            "low_trust_policy": "sensor_acc_or_stop",
        }

        controller_manager = getattr(self.vehicle_logic, "controller_manager", None)
        controller_config = getattr(controller_manager, "config", None)
        if controller_config and hasattr(
            controller_config, "get_trust_longitudinal_fusion_config"
        ):
            fusion_cfg.update(controller_config.get_trust_longitudinal_fusion_config())

        enabled_value = fusion_cfg.get("enabled", False)
        if isinstance(enabled_value, str):
            fusion_enabled = enabled_value.strip().lower() in {
                "1",
                "true",
                "yes",
                "on",
            }
        else:
            fusion_enabled = bool(enabled_value)

        try:
            trust_low = float(np.clip(fusion_cfg.get("trust_low", 0.50), 0.0, 1.0))
            trust_high = float(np.clip(fusion_cfg.get("trust_high", 0.80), 0.0, 1.0))
        except (TypeError, ValueError):
            trust_low, trust_high = 0.50, 0.80
        if trust_high <= trust_low:
            trust_low, trust_high = 0.50, 0.80

        unavailable_policy = str(
            fusion_cfg.get("unavailable_policy", "legacy_cacc")
        ).strip().lower()
        if unavailable_policy not in {"legacy_cacc", "sensor_acc_or_stop"}:
            unavailable_policy = "legacy_cacc"

        low_trust_policy = str(
            fusion_cfg.get("low_trust_policy", "sensor_acc_or_stop")
        ).strip().lower()
        if low_trust_policy not in {
            "sensor_acc_or_stop",
            "sensor_acc_or_cacc",
            "legacy_cacc",
        }:
            low_trust_policy = "sensor_acc_or_stop"

        alpha_mode = str(
            fusion_cfg.get("alpha_mode", "threshold_map")
        ).strip().lower()
        alpha_mode_aliases = {
            "threshold": "threshold_map",
            "threshold_mapping": "threshold_map",
            "trust_threshold": "threshold_map",
            "opinion": "direct_opinion",
            "raw_opinion": "direct_opinion",
            "direct": "direct_opinion",
        }
        alpha_mode = alpha_mode_aliases.get(alpha_mode, alpha_mode)
        if alpha_mode not in {"threshold_map", "direct_opinion"}:
            alpha_mode = "threshold_map"

        opinion_scope = str(
            fusion_cfg.get("opinion_scope", "direct_leader")
        ).strip().lower()
        opinion_scope_aliases = {
            "leader": "direct_leader",
            "direct": "direct_leader",
            "leader_only": "direct_leader",
            "predecessors": "used_predecessors",
            "all_predecessors": "used_predecessors",
            "paper": "used_predecessors",
        }
        opinion_scope = opinion_scope_aliases.get(opinion_scope, opinion_scope)
        if opinion_scope not in {"direct_leader", "used_predecessors"}:
            opinion_scope = "direct_leader"

        self.trust_longitudinal_fusion_config = {
            "enabled": fusion_enabled,
            "trust_low": trust_low,
            "trust_high": trust_high,
            "unavailable_policy": unavailable_policy,
            "low_trust_policy": low_trust_policy,
            "alpha_mode": alpha_mode,
            "opinion_scope": opinion_scope,
        }
        self.logger.logger.info(
            "[FOLLOW] Trust longitudinal fusion: "
            f"{'enabled' if self.trust_longitudinal_fusion_config['enabled'] else 'disabled'}"
        )

    def _load_sensor_acc_config(self):
        """Load optional local sensor ACC branch for close-range stop control."""
        sensor_cfg = {
            "enabled": False,
            "blend_alpha": 0.7,
            "desired_distance": 0.35,
            "time_headway": 0.0,
            "distance_gain": 1.0,
            "min_target_velocity": 0.0,
            "max_target_velocity": 0.8,
            "stop_distance": 0.20,
            "max_distance": 2.0,
            "max_offset": 0.75,
            "distance_smoothing": 0.6,
        }

        controller_manager = getattr(self.vehicle_logic, "controller_manager", None)
        controller_config = getattr(controller_manager, "config", None)
        if controller_config and hasattr(controller_config, "get_leader_sensor_acc_config"):
            sensor_cfg.update(controller_config.get_leader_sensor_acc_config())

        self.sensor_acc_config = sensor_cfg
        self.sensor_acc_controller = None
        self._sensor_acc_distance_filtered = None

        trust_fusion_enabled = bool(
            self.trust_longitudinal_fusion_config.get("enabled", False)
        )
        if (
            not sensor_cfg.get("enabled", False)
            and not trust_fusion_enabled
        ) or ControllerFactory is None:
            return

        try:
            params = {"vehicle_type": getattr(controller_manager, "vehicle_type", "QCar")}
            self.sensor_acc_controller = ControllerFactory.create(
                "sensor_acc",
                params,
                logger=self.logger,
                config=controller_config,
            )
        except Exception as exc:
            self.logger.logger.warning(
                f"[FOLLOW] Failed to initialize sensor ACC helper: {exc}"
            )
            self.sensor_acc_controller = None

    def _load_leader_source_config(self):
        """Load the single leader source used by longitudinal and lateral control."""
        source_cfg = {"mode": "direct_v2v_attacked"}

        controller_manager = getattr(self.vehicle_logic, "controller_manager", None)
        controller_config = getattr(controller_manager, "config", None)
        if controller_config and hasattr(controller_config, "get_leader_source_config"):
            source_cfg.update(controller_config.get_leader_source_config())

        self.leader_source_config = source_cfg
        self.logger.logger.info(
            "[FOLLOW] Leader source: "
            f"{source_cfg.get('mode', 'direct_v2v_attacked')}"
        )

    def _load_multi_predecessor_cacc_config(self):
        """Load optional multi-predecessor CACC feedforward settings."""
        cfg = {
            "enabled": False,
            "include_direct_leader": False,
            "max_predecessors": 3,
        }

        controller_manager = getattr(self.vehicle_logic, "controller_manager", None)
        controller_config = getattr(controller_manager, "config", None)
        if controller_config and hasattr(
            controller_config, "get_multi_predecessor_cacc_config"
        ):
            cfg.update(controller_config.get_multi_predecessor_cacc_config())

        selected_longitudinal = ""
        if controller_manager and hasattr(controller_manager, "get_longitudinal_type"):
            selected_longitudinal = str(
                controller_manager.get_longitudinal_type(state="leader") or ""
            ).strip().lower()
        if selected_longitudinal == "multi_predecessor_cacc":
            cfg["enabled"] = True

        enabled = cfg.get("enabled", False)
        if isinstance(enabled, str):
            enabled = enabled.strip().lower() in {"1", "true", "yes", "on"}
        else:
            enabled = bool(enabled)

        include_direct = cfg.get("include_direct_leader", False)
        if isinstance(include_direct, str):
            include_direct = include_direct.strip().lower() in {
                "1",
                "true",
                "yes",
                "on",
            }
        else:
            include_direct = bool(include_direct)

        try:
            max_predecessors = max(int(cfg.get("max_predecessors", 3)), 0)
        except (TypeError, ValueError):
            max_predecessors = 3

        cfg["enabled"] = enabled
        cfg["include_direct_leader"] = include_direct
        cfg["max_predecessors"] = max_predecessors
        self.multi_predecessor_cacc_config = cfg
        self.logger.logger.info(
            "[FOLLOW] Multi-predecessor CACC: "
            f"{'enabled' if enabled else 'disabled'}"
        )

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        """Wrap angle to [-pi, pi)."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def _compute_leader_gap(
        self, follower_state: Dict[str, float], leader_state: Dict[str, float]
    ) -> float:
        """Use Euclidean gap for reverse safety gating."""
        dx = float(leader_state["x"]) - float(follower_state["x"])
        dy = float(leader_state["y"]) - float(follower_state["y"])
        return float(np.hypot(dx, dy))

    def _get_reverse_gate_status(
        self,
        follower_state: Dict[str, float],
        leader_state: Dict[str, float],
        v2v_data: Optional[Dict[str, Any]],
    ) -> Tuple[bool, str]:
        """Evaluate reverse-follow intent and local safety gates."""
        cfg = self.reverse_follow_config
        if not cfg.get("enabled", False):
            return False, "disabled"

        if v2v_data is None:
            return False, "v2v_lost"

        leader_velocity = float(leader_state.get("velocity", 0.0))
        if leader_velocity >= float(cfg.get("trigger_velocity_threshold", -0.03)):
            return False, "leader_not_reversing"

        gap = self._compute_leader_gap(follower_state, leader_state)
        if gap < float(cfg.get("min_gap", 0.20)):
            return False, "gap_too_small"
        if gap > float(cfg.get("max_gap", 0.90)):
            return False, "gap_too_large"

        heading_error = abs(
            self._wrap_to_pi(
                float(leader_state.get("theta", 0.0))
                - float(follower_state.get("theta", 0.0))
            )
        )
        if heading_error > self._reverse_heading_limit_rad:
            return False, "heading_mismatch"

        return True, "ok"

    def _should_activate_reverse_follow(
        self,
        follower_state: Dict[str, float],
        leader_state: Dict[str, float],
        v2v_data: Optional[Dict[str, Any]],
    ) -> bool:
        """Reverse-follow is allowed only when reverse intent and safety both pass."""
        reverse_ok, _ = self._get_reverse_gate_status(
            follower_state, leader_state, v2v_data
        )
        return reverse_ok

    def _should_force_reverse_stop(
        self,
        follower_state: Dict[str, float],
        leader_state: Dict[str, float],
        v2v_data: Optional[Dict[str, Any]],
    ) -> bool:
        """Hold stop if leader is trying to reverse but local safety gates fail."""
        cfg = self.reverse_follow_config
        if not cfg.get("enabled", False):
            return False

        if v2v_data is None:
            return self._reverse_follow_active and bool(cfg.get("stop_on_v2v_loss", True))

        leader_velocity = float(leader_state.get("velocity", 0.0))
        if leader_velocity >= float(cfg.get("trigger_velocity_threshold", -0.03)):
            return False

        reverse_ok, _ = self._get_reverse_gate_status(
            follower_state, leader_state, v2v_data
        )
        return not reverse_ok

    def _get_reverse_steering_limit(self) -> float:
        """Extract the steering clamp from active controllers when available."""
        for controller in (self.lateral_controller, self.steering_controller):
            if controller is None:
                continue
            for attr in ("max_steering", "max_steering_angle"):
                value = getattr(controller, attr, None)
                if value is not None:
                    return abs(float(value))
        return 0.5

    def _compute_reverse_steering(
        self,
        follower_state: Dict[str, float],
        leader_state: Dict[str, float],
    ) -> float:
        """Track a point behind the leader and invert steering for reverse motion."""
        desired_gap = float(
            np.clip(
                max(
                    float(self.reverse_follow_config.get("min_gap", 0.20)),
                    float(getattr(self.longitudinal_controller, "s0", 0.20)),
                ),
                float(self.reverse_follow_config.get("min_gap", 0.20)),
                float(self.reverse_follow_config.get("max_gap", 0.90)),
            )
        )

        leader_theta = float(leader_state.get("theta", 0.0))
        target_x = float(leader_state["x"]) - desired_gap * np.cos(leader_theta)
        target_y = float(leader_state["y"]) - desired_gap * np.sin(leader_theta)

        dx = target_x - float(follower_state["x"])
        dy = target_y - float(follower_state["y"])
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return 0.0

        target_angle = float(np.arctan2(dy, dx))
        heading_error = self._wrap_to_pi(target_angle - float(follower_state["theta"]))
        steering_cmd = -float(
            self.reverse_follow_config.get("reverse_steering_gain", 0.5)
        ) * heading_error

        steering_limit = self._get_reverse_steering_limit()
        return float(np.clip(steering_cmd, -steering_limit, steering_limit))

    def update(
        self, dt: float, sensor_data: Dict[str, Any]
    ) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        """Update leader following control using PlatoonController"""

        # Debug to see if this state is being called
        if (
            hasattr(self.vehicle_logic, "loop_counter")
            and self.vehicle_logic.loop_counter % 1000 == 0
        ):
            print(
                f"[DEBUG] FollowingLeaderState.update called - loop_counter: {getattr(self.vehicle_logic, 'loop_counter', 'N/A')}"
            )

        # Check startup delay
        if self.vehicle_logic.elapsed_time() < self.config.timing.start_delay:
            return 0.0, 0.0, None

        # Extract basic sensor data
        x, y, theta = sensor_data["x"], sensor_data["y"], sensor_data["theta"]
        velocity = sensor_data["velocity"]
        acceleration = float(sensor_data.get("acceleration", 0.0))
        yolo_data = sensor_data.get("yolo_data", {})

        # Update Data for PlatoonController (perception and V2V data)
        self._update_platoon_controller(yolo_data)

        # Get V2V leader data
        v2v_data = self._get_v2v_leader_data()

        # Compute control using PlatoonController (pass yolo_data for turning detection)
        u, delta = self._compute_control(
            dt, x, y, theta, velocity, acceleration, v2v_data, yolo_data
        )

        # Enhanced periodic logging for debugging
        # self._log_status(velocity, u, v2v_data)

        return u, delta, None

    # ===== SIMPLIFIED HELPER METHODS =====

    def _update_platoon_controller(self, yolo_data: Dict[str, Any]):
        """Update PlatoonController with perception data"""
        if not hasattr(self.vehicle_logic, "platoon_controller"):
            return

        pc = self.vehicle_logic.platoon_controller

        # Update with YOLO perception data
        if self.vehicle_logic.yolo_manager.yolo_enabled:
            cars_detected = yolo_data.get("cars", [])
            car_distance = yolo_data.get("car_dist")
            # Check if any car is detected - handle numpy arrays, lists, and scalars
            if isinstance(cars_detected, np.ndarray):
                has_car = cars_detected.any() if cars_detected.size > 0 else False
            elif isinstance(cars_detected, (list, tuple)):
                has_car = any(cars_detected)
            else:
                has_car = bool(cars_detected) if cars_detected is not None else False
            pc.update_leader_info(
                detected=has_car and car_distance is not None,
                distance=car_distance,
                velocity=None,  # Velocity comes from V2V
            )

        # # Update with V2V velocity data
        # if hasattr(self.vehicle_logic, 'v2v_manager'):
        #     pc.update_leader_velocity_from_v2v(
        #         self.vehicle_logic.v2v_manager,
        #         self.vehicle_logic.vehicle_id
        #     )

    def _extract_lane_data(self, yolo_data: dict) -> dict:
        """
        Extract and validate lane detection data from YOLO packet.

        This helper provides a clean interface to the lane data transmitted
        from yolo_server, handling missing/invalid data gracefully.

        Args:
            yolo_data: Dict from YOLO receiver with lane_* keys

        Returns:
            Structured dict with validated lane data:
            - valid: bool - Whether lane data is usable
            - confidence: float - Detection confidence [0-1]
            - steering: float - Suggested steering correction
            - curvature: float - Lane curvature proxy
            - offset: float - Lateral offset from center
            - left_detected: bool - Left lane marker visible
            - right_detected: bool - Right lane marker visible
        """
        if not yolo_data:
            return {
                "valid": False,
                "confidence": 0.0,
                "steering": 0.0,
                "curvature": 0.0,
                "offset": 0.0,
                "left_detected": False,
                "right_detected": False,
            }

        confidence = yolo_data.get("lane_confidence", 0.0)
        is_valid = confidence > 0.1  # Minimum threshold for usable lane data

        return {
            "valid": is_valid,
            "confidence": confidence,
            "steering": yolo_data.get("lane_steering", 0.0),
            "curvature": yolo_data.get(
                "lane_slope", 0.0
            ),  # slope used as curvature proxy
            "offset": yolo_data.get("lane_intercept", 0.0),
            "left_detected": yolo_data.get("lane_left_detected", False),
            "right_detected": yolo_data.get("lane_right_detected", False),
        }

    def _detect_turning_section(
        self, yolo_data: dict, follower_theta: float, leader_theta: float
    ) -> Tuple[bool, str, float]:
        """
        Detect if vehicle is in a turning section using lane curvature
        and leader orientation.

        Uses two signals:
        1. Lane curvature from YOLO detection
        2. Heading difference between follower and leader (global orientation)

        Args:
            yolo_data: YOLO lane detection data
            follower_theta: Follower's heading (radians)
            leader_theta: Leader's heading from V2V (radians)

        Returns:
            Tuple of (is_turning, direction, curvature):
            - is_turning: bool - True if in a turning section
            - direction: str - 'left', 'right', or 'straight'
            - curvature: float - Combined curvature estimate
        """
        # Default values
        is_turning = False
        direction = "straight"
        curvature = 0.0

        # Thresholds (can be made configurable via YAML later)
        CURVATURE_THRESHOLD = 0.3  # Lane curvature threshold
        HEADING_DIFF_THRESHOLD = 0.15  # ~8.5 degrees

        # Extract lane data
        lane_data = self._extract_lane_data(yolo_data)
        lane_curvature = lane_data.get("curvature", 0.0)

        # Compute heading difference (leader orientation from V2V)
        # Normalize to [-pi, pi]
        heading_diff = (leader_theta - follower_theta + np.pi) % (2 * np.pi) - np.pi

        # Combine both signals for turning detection
        # Lane curvature provides visual cue, heading diff provides trajectory cue
        if lane_data["valid"]:
            curvature = lane_curvature
        else:
            # Fallback to heading difference as curvature proxy
            curvature = heading_diff

        # Detect turning based on combined signals
        if (
            abs(lane_curvature) > CURVATURE_THRESHOLD
            or abs(heading_diff) > HEADING_DIFF_THRESHOLD
        ):
            is_turning = True

            # Determine direction from the stronger signal
            if lane_data["valid"] and abs(lane_curvature) > abs(heading_diff):
                direction = "left" if lane_curvature > 0 else "right"
            else:
                direction = "left" if heading_diff > 0 else "right"

        # Log turning detection periodically
        if (
            is_turning
            and hasattr(self.vehicle_logic, "loop_counter")
            and self.vehicle_logic.loop_counter % 100 == 0
        ):
            self.logger.logger.debug(
                f"[FOLLOW] Turning detected: {direction}, "
                f"lane_curv={lane_curvature:.3f}, heading_diff={heading_diff:.3f}"
            )

        return is_turning, direction, curvature

    def _update_sensor_acc_distance(self, yolo_data: Optional[Dict[str, Any]]) -> Optional[float]:
        """Filter YOLO leader distance for close-range stop protection."""
        trust_fusion_enabled = bool(
            self.trust_longitudinal_fusion_config.get("enabled", False)
        )
        if not self.sensor_acc_config.get("enabled", False) and not trust_fusion_enabled:
            self._sensor_acc_distance_filtered = None
            return None

        yolo_data = yolo_data or {}
        cars_detected = yolo_data.get("cars", [])
        if isinstance(cars_detected, np.ndarray):
            has_car = cars_detected.any() if cars_detected.size > 0 else False
        elif isinstance(cars_detected, (list, tuple)):
            has_car = any(cars_detected)
        else:
            has_car = bool(cars_detected) if cars_detected is not None else False

        measured_distance = yolo_data.get("car_dist", None)
        measured_offset = yolo_data.get("car_offset", None)

        if not has_car or measured_distance is None:
            self._sensor_acc_distance_filtered = None
            return None

        try:
            distance_value = float(measured_distance)
        except (TypeError, ValueError):
            self._sensor_acc_distance_filtered = None
            return None

        if not np.isfinite(distance_value) or distance_value <= 0.0:
            self._sensor_acc_distance_filtered = None
            return None

        max_distance = float(self.sensor_acc_config.get("max_distance", 2.0))
        if distance_value > max_distance:
            self._sensor_acc_distance_filtered = None
            return None

        if measured_offset is not None:
            try:
                offset_value = abs(float(measured_offset))
            except (TypeError, ValueError):
                offset_value = 0.0
            if offset_value > float(self.sensor_acc_config.get("max_offset", 0.75)):
                self._sensor_acc_distance_filtered = None
                return None

        alpha = float(np.clip(self.sensor_acc_config.get("distance_smoothing", 0.6), 0.0, 0.99))
        if self._sensor_acc_distance_filtered is None:
            self._sensor_acc_distance_filtered = distance_value
        else:
            self._sensor_acc_distance_filtered = (
                alpha * self._sensor_acc_distance_filtered
                + (1.0 - alpha) * distance_value
            )

        return float(self._sensor_acc_distance_filtered)

    def _fill_sensor_acc_distance_from_clean_v2v(
        self,
        follower_state: Dict[str, Any],
        leader_state: Dict[str, Any],
    ) -> Optional[float]:
        """Use the clean V2V channel as a YOLO-distance substitute in simulation."""
        if follower_state.get("sensor_leader_distance_filtered") is not None:
            return float(follower_state["sensor_leader_distance_filtered"])

        v2v_manager = getattr(self.vehicle_logic, "v2v_manager", None)
        if v2v_manager is None or not hasattr(v2v_manager, "get_latest_local_state_raw"):
            return None

        leader_id = leader_state.get("vehicle_id")
        if leader_id is None:
            leader_id = self._get_direct_leader_vehicle_id()
        try:
            leader_id = int(leader_id)
        except (TypeError, ValueError):
            return None

        try:
            clean_data = v2v_manager.get_latest_local_state_raw(
                leader_id, channel="clean"
            )
        except Exception as exc:
            self.logger.logger.debug(
                f"[FOLLOW] Clean V2V gap fallback unavailable: {exc}"
            )
            return None

        if not clean_data:
            return None

        try:
            follower_x = float(follower_state["x"])
            follower_y = float(follower_state["y"])
            leader_x = float(clean_data["x"])
            leader_y = float(clean_data["y"])
        except (KeyError, TypeError, ValueError):
            return None

        clean_gap = None
        along_track = self._compute_along_track_gap(
            follower_x, follower_y, leader_x, leader_y
        )
        if along_track is not None:
            clean_gap = float(along_track[0])

        if clean_gap is None or clean_gap <= 0.0:
            clean_gap = float(np.hypot(leader_x - follower_x, leader_y - follower_y))

        if not np.isfinite(clean_gap) or clean_gap <= 0.0:
            return None

        max_distance = max(float(self.sensor_acc_config.get("max_distance", 2.0)), 1e-3)
        clean_gap = float(np.clip(clean_gap, 1e-3, max_distance))

        follower_state["sensor_leader_distance"] = clean_gap
        follower_state["sensor_leader_distance_filtered"] = clean_gap
        follower_state["sensor_leader_distance_source"] = "clean_v2v"
        return clean_gap

    def _attach_clean_v2v_leader_baseline(
        self,
        follower_state: Dict[str, Any],
        leader_state: Dict[str, Any],
    ) -> None:
        """Attach clean V2V leader state as a baseline for controller diagnostics."""
        v2v_manager = getattr(self.vehicle_logic, "v2v_manager", None)
        if v2v_manager is None or not hasattr(v2v_manager, "get_latest_local_state_raw"):
            return

        leader_id = leader_state.get("vehicle_id")
        if leader_id is None:
            leader_id = self._get_direct_leader_vehicle_id()
        try:
            leader_id = int(leader_id)
        except (TypeError, ValueError):
            return

        try:
            clean_data = v2v_manager.get_latest_local_state_raw(
                leader_id, channel="clean"
            )
        except Exception as exc:
            self.logger.logger.debug(
                f"[FOLLOW] Clean V2V baseline unavailable: {exc}"
            )
            return

        if not clean_data:
            return

        try:
            follower_x = float(follower_state["x"])
            follower_y = float(follower_state["y"])
            clean_x = float(clean_data["x"])
            clean_y = float(clean_data["y"])
            clean_theta = float(clean_data.get("theta", 0.0))
            clean_velocity = float(clean_data.get("velocity", 0.0))
            clean_acceleration = float(clean_data.get("acceleration", 0.0))
        except (KeyError, TypeError, ValueError):
            return

        clean_distance = float(np.hypot(clean_x - follower_x, clean_y - follower_y))
        clean_along_track_gap = float("nan")
        along_track = self._compute_along_track_gap(
            follower_x, follower_y, clean_x, clean_y
        )
        if along_track is not None:
            clean_along_track_gap = float(along_track[0])

        leader_state["leader_clean_available"] = True
        leader_state["leader_clean_x"] = clean_x
        leader_state["leader_clean_y"] = clean_y
        leader_state["leader_clean_theta"] = clean_theta
        leader_state["leader_clean_velocity"] = clean_velocity
        leader_state["leader_clean_acceleration"] = clean_acceleration
        leader_state["leader_clean_distance"] = clean_distance
        leader_state["leader_clean_along_track_gap"] = clean_along_track_gap

    @staticmethod
    def _compute_trust_fusion_alpha(
        trust_value: Optional[float], trust_low: float, trust_high: float
    ) -> Optional[float]:
        """Map trust into the CACC command weight used by longitudinal fusion."""
        if trust_value is None:
            return None
        try:
            trust = float(trust_value)
            low = float(trust_low)
            high = float(trust_high)
        except (TypeError, ValueError):
            return None
        if not np.isfinite(trust) or not np.isfinite(low) or not np.isfinite(high):
            return None
        if high <= low:
            return None
        return float(np.clip((trust - low) / (high - low), 0.0, 1.0))

    @staticmethod
    def _coerce_trust_opinion(value: Any) -> Optional[float]:
        """Return a finite opinion in [0, 1], or None when unavailable."""
        if value is None:
            return None
        try:
            opinion = float(value)
        except (TypeError, ValueError):
            return None
        if not np.isfinite(opinion):
            return None
        return float(np.clip(opinion, 0.0, 1.0))

    def _get_state_trust_opinion(self, state: Dict[str, Any]) -> Optional[float]:
        """Read the controller opinion O_i(j), preferring generalized trust."""
        if not isinstance(state, dict):
            return None
        for key in (
            "leader_generalized_trust",
            "generalized_trust",
            "gtrust",
            "leader_trust",
            "leader_direct_trust",
            "direct_trust",
            "trust",
        ):
            opinion = self._coerce_trust_opinion(state.get(key))
            if opinion is not None:
                return opinion
        return None

    def _predecessor_used_by_cacc(self, pred_state: Dict[str, Any]) -> bool:
        """Mirror the trust gate used by multi-predecessor CACC feedforward."""
        cfg = self.multi_predecessor_cacc_config or {}
        opinion = self._get_state_trust_opinion(pred_state)
        if opinion is None:
            try:
                missing_weight = float(cfg.get("missing_trust_weight", 0.0))
            except (TypeError, ValueError):
                missing_weight = 0.0
            return missing_weight > 0.0

        require_trust = bool(cfg.get("require_trust", True))
        try:
            min_trust = float(cfg.get("min_trust", 0.50))
        except (TypeError, ValueError):
            min_trust = 0.50
        return (not require_trust) or opinion >= float(np.clip(min_trust, 0.0, 1.0))

    def _get_trust_fusion_opinion(
        self,
        follower_state: Dict[str, Any],
        leader_state: Dict[str, Any],
    ) -> Optional[float]:
        """Select the opinion used by the local/coop command fusion."""
        cfg = self.trust_longitudinal_fusion_config
        scope = str(cfg.get("opinion_scope", "direct_leader")).strip().lower()

        candidate_states = [leader_state]
        seen_ids = set()
        leader_id = leader_state.get("vehicle_id")
        try:
            seen_ids.add(int(leader_id))
        except (TypeError, ValueError):
            pass

        if scope == "used_predecessors":
            predecessor_states = follower_state.get("multi_predecessor_states", [])
            if isinstance(predecessor_states, (list, tuple)):
                for pred_state in predecessor_states:
                    if not isinstance(pred_state, dict):
                        continue
                    pred_id = pred_state.get("vehicle_id")
                    try:
                        pred_id_int = int(pred_id)
                    except (TypeError, ValueError):
                        pred_id_int = None
                    if pred_id_int is not None and pred_id_int in seen_ids:
                        continue
                    if not self._predecessor_used_by_cacc(pred_state):
                        continue
                    if pred_id_int is not None:
                        seen_ids.add(pred_id_int)
                    candidate_states.append(pred_state)

        opinions = []
        for state in candidate_states:
            opinion = self._get_state_trust_opinion(state)
            if opinion is None:
                return None
            opinions.append(opinion)

        if not opinions:
            return None
        return float(min(opinions))

    def _compute_trust_fusion_alpha_for_mode(
        self,
        opinion: Optional[float],
        trust_low: float,
        trust_high: float,
    ) -> Optional[float]:
        """Map the selected opinion into the cooperative-command weight."""
        if opinion is None:
            return None
        alpha_mode = str(
            self.trust_longitudinal_fusion_config.get("alpha_mode", "threshold_map")
        ).strip().lower()
        if alpha_mode == "direct_opinion":
            return self._coerce_trust_opinion(opinion)
        return self._compute_trust_fusion_alpha(opinion, trust_low, trust_high)

    def _get_direct_leader_vehicle_id(self) -> Optional[int]:
        """Resolve the direct platoon leader id when formation metadata is available."""
        platoon_controller = getattr(self.vehicle_logic, "platoon_controller", None)
        if platoon_controller is None or not hasattr(
            platoon_controller, "get_direct_leader_vehicle_id"
        ):
            return None
        try:
            leader_id = platoon_controller.get_direct_leader_vehicle_id()
            return int(leader_id) if leader_id is not None else None
        except Exception:
            return None

    def _get_leader_trust_context(
        self, leader_state: Dict[str, Any]
    ) -> Optional[Dict[str, Any]]:
        """Fetch trust context for the current direct leader."""
        leader_id = leader_state.get("vehicle_id")
        if leader_id is None:
            leader_id = self._get_direct_leader_vehicle_id()
        return self._get_vehicle_trust_context(leader_id)

    def _get_vehicle_trust_context(
        self, vehicle_id: Optional[int]
    ) -> Optional[Dict[str, Any]]:
        """Fetch trust context for a specific vehicle id."""
        try:
            vehicle_id = int(vehicle_id)
        except (TypeError, ValueError):
            return None

        observer = getattr(self.vehicle_logic, "vehicle_observer", None)
        if observer is None or not hasattr(observer, "get_vehicle_trust_context"):
            return None

        try:
            context = observer.get_vehicle_trust_context(vehicle_id)
        except Exception as exc:
            self.logger.logger.debug(
                f"[FOLLOW] Vehicle trust context unavailable: {exc}"
            )
            return None
        return context if isinstance(context, dict) else None

    def _attach_leader_trust_context(self, leader_state: Dict[str, Any]) -> None:
        """Attach trust fields used by command fusion and following logs."""
        context = self._get_leader_trust_context(leader_state)
        leader_state["leader_trust"] = None
        leader_state["leader_trust_source"] = "unavailable"
        leader_state["leader_direct_trust"] = None
        leader_state["leader_generalized_trust"] = None
        leader_state["leader_trusted"] = None

        if context is None:
            return

        direct_trust = context.get("direct_trust")
        generalized_trust = context.get("generalized_trust")
        leader_state["leader_direct_trust"] = direct_trust
        leader_state["leader_generalized_trust"] = generalized_trust
        leader_state["leader_trusted"] = context.get("trusted")
        leader_state["leader_attack_flags"] = context.get("attack_flags", {})

        if generalized_trust is not None:
            leader_state["leader_trust"] = generalized_trust
            leader_state["leader_trust_source"] = "generalized"
        elif direct_trust is not None:
            leader_state["leader_trust"] = direct_trust
            leader_state["leader_trust_source"] = "direct"

    def _get_predecessor_vehicle_ids(self) -> list:
        """Return vehicles ahead of the host, sorted nearest first."""
        platoon_controller = getattr(self.vehicle_logic, "platoon_controller", None)
        if platoon_controller is None:
            return []

        formation = getattr(platoon_controller, "formation_data", {}) or {}
        my_position = getattr(platoon_controller, "my_position", None)
        try:
            my_position = int(my_position)
        except (TypeError, ValueError):
            return []

        predecessors = []
        for vehicle_id, position in formation.items():
            try:
                vehicle_id_int = int(vehicle_id)
                position_int = int(position)
            except (TypeError, ValueError):
                continue
            if position_int < my_position:
                predecessors.append((my_position - position_int, vehicle_id_int))

        predecessors.sort(key=lambda item: item[0])
        return predecessors

    def _get_vehicle_data_from_fleet_estimator(
        self, vehicle_id: int
    ) -> Optional[Dict[str, Any]]:
        """Get one vehicle state from the fleet estimator output."""
        observer = getattr(self.vehicle_logic, "vehicle_observer", None)
        if observer is None or not hasattr(observer, "get_vehicle_state"):
            return None

        leader_state = observer.get_vehicle_state(vehicle_id)
        if leader_state is None or len(leader_state) < 4:
            return None

        acceleration = float(leader_state[4]) if len(leader_state) > 4 else 0.0
        return {
            "vehicle_id": int(vehicle_id),
            "x": float(leader_state[0]),
            "y": float(leader_state[1]),
            "theta": float(leader_state[2]),
            "velocity": float(leader_state[3]),
            "acceleration": acceleration,
            "source": "fleet_estimator",
        }

    def _get_vehicle_data_for_longitudinal_source(
        self, vehicle_id: int, source_mode: str
    ) -> Optional[Dict[str, Any]]:
        """Read a predecessor state from the same source family as direct CACC."""
        source_mode = str(source_mode or "direct_v2v_attacked").strip().lower()
        if source_mode == "fleet_estimator":
            return self._get_vehicle_data_from_fleet_estimator(vehicle_id)
        v2v_manager = getattr(self.vehicle_logic, "v2v_manager", None)
        if v2v_manager is None or not hasattr(v2v_manager, "get_latest_local_state_raw"):
            return None

        channel = "clean" if source_mode == "direct_v2v_clean" else "attacked"
        data = v2v_manager.get_latest_local_state_raw(vehicle_id, channel=channel)
        if data is None:
            return None

        data = dict(data)
        data.setdefault("vehicle_id", int(vehicle_id))
        data.setdefault("source", source_mode)
        return data

    def _collect_multi_predecessor_states(
        self,
        follower_state: Dict[str, Any],
        direct_leader_state: Dict[str, Any],
    ) -> list:
        """Collect trusted-feedforward candidates ahead of this vehicle."""
        cfg = self.multi_predecessor_cacc_config
        if not cfg.get("enabled", False):
            return []

        max_predecessors = int(cfg.get("max_predecessors", 0))
        if max_predecessors <= 0:
            return []

        source_mode = self.leader_source_config.get(
            "mode", "direct_v2v_attacked"
        )
        include_direct = bool(cfg.get("include_direct_leader", False))
        direct_id = direct_leader_state.get("vehicle_id")
        try:
            direct_id = int(direct_id)
        except (TypeError, ValueError):
            direct_id = None

        predecessor_states = []
        for order_index, vehicle_id in self._get_predecessor_vehicle_ids():
            if not include_direct and direct_id is not None and vehicle_id == direct_id:
                continue

            pred = self._get_vehicle_data_for_longitudinal_source(
                vehicle_id, source_mode
            )
            if pred is None:
                continue

            pred_state = {
                "vehicle_id": int(vehicle_id),
                "x": float(pred.get("x", 0.0)),
                "y": float(pred.get("y", 0.0)),
                "theta": float(pred.get("theta", 0.0)),
                "velocity": float(pred.get("velocity", pred.get("v", 0.0))),
                "acceleration": float(pred.get("acceleration", 0.0)),
                "source": pred.get("source", source_mode),
                "order_index": int(order_index),
            }
            self._attach_leader_trust_context(pred_state)

            along_track = self._compute_along_track_gap(
                float(follower_state["x"]),
                float(follower_state["y"]),
                pred_state["x"],
                pred_state["y"],
            )
            if along_track is not None:
                gap_s, _, pred_frenet = along_track
                pred_state["distance_ahead"] = max(float(gap_s), 0.0)
                pred_state["path_s"] = pred_frenet[0]
                pred_state["path_d"] = pred_frenet[1]
            else:
                pred_state["distance_ahead"] = float(
                    np.hypot(
                        pred_state["x"] - float(follower_state["x"]),
                        pred_state["y"] - float(follower_state["y"]),
                    )
                )

            predecessor_states.append(pred_state)
            if len(predecessor_states) >= max_predecessors:
                break

        return predecessor_states

    def _compute_sensor_acc_command(
        self,
        follower_state: Dict[str, Any],
        leader_state: Dict[str, Any],
        dt: float,
    ) -> Optional[float]:
        """Compute the local sensor ACC fallback command when a valid gap exists."""
        if self.sensor_acc_controller is None:
            return None
        if follower_state.get("sensor_leader_distance_filtered") is None:
            self._fill_sensor_acc_distance_from_clean_v2v(
                follower_state, leader_state
            )
        if follower_state.get("sensor_leader_distance_filtered") is None:
            return None
        return float(
            self.sensor_acc_controller.compute_throttle(
                follower_state, leader_state, dt
            )
        )

    def _set_longitudinal_fusion_log_fields(
        self,
        follower_state: Dict[str, Any],
        u_cacc: float,
        u_sensor: Optional[float],
        alpha: Optional[float],
        policy: str,
    ) -> None:
        """Store command-fusion details in follower_state for CSV logging."""
        follower_state["u_cacc"] = float(u_cacc)
        follower_state["u_sensor"] = (
            float(u_sensor) if u_sensor is not None else float("nan")
        )
        follower_state["trust_fusion_alpha"] = (
            float(alpha) if alpha is not None else float("nan")
        )
        follower_state["trust_fusion_policy"] = policy

    def _publish_controller_debug_snapshot(
        self,
        follower_state: Dict[str, Any],
        leader_state: Optional[Dict[str, Any]],
        u: float,
        delta: float,
    ) -> None:
        """Publish controller diagnostics into the observer trust-log path."""
        leader_state = leader_state or {}
        controller_manager = getattr(self.vehicle_logic, "controller_manager", None)
        longitudinal_type = ""
        lateral_type = str(self.lateral_controller_type or "")

        if controller_manager is not None:
            try:
                if hasattr(controller_manager, "get_longitudinal_type"):
                    longitudinal_type = str(
                        controller_manager.get_longitudinal_type(state="leader") or ""
                    )
            except Exception:
                longitudinal_type = ""
            if not lateral_type:
                try:
                    if hasattr(controller_manager, "get_lateral_type"):
                        lateral_type = str(
                            controller_manager.get_lateral_type(state="leader") or ""
                        )
                except Exception:
                    lateral_type = ""

        if not longitudinal_type and self.longitudinal_controller is not None:
            longitudinal_type = type(self.longitudinal_controller).__name__
        if not lateral_type and self.lateral_controller is not None:
            lateral_type = type(self.lateral_controller).__name__

        distance_to_leader = float("nan")
        velocity_difference = float("nan")
        if leader_state:
            try:
                distance_to_leader = float(
                    np.hypot(
                        float(leader_state.get("x", 0.0))
                        - float(follower_state.get("x", 0.0)),
                        float(leader_state.get("y", 0.0))
                        - float(follower_state.get("y", 0.0)),
                    )
                )
                velocity_difference = float(leader_state.get("velocity", 0.0)) - float(
                    follower_state.get("velocity", 0.0)
                )
            except (TypeError, ValueError):
                distance_to_leader = float("nan")
                velocity_difference = float("nan")

        snapshot = {
            "state": "following_leader",
            "longitudinal_type": longitudinal_type,
            "lateral_type": lateral_type,
            "leader_source": leader_state.get("source", ""),
            "leader_id": leader_state.get("vehicle_id"),
            "leader_trust": leader_state.get("leader_trust"),
            "leader_trust_source": leader_state.get("leader_trust_source", ""),
            "policy": follower_state.get("trust_fusion_policy", ""),
            "alpha": follower_state.get("trust_fusion_alpha"),
            "u_final": float(u),
            "delta_final": float(delta),
            "u_raw": follower_state.get("raw_throttle_u"),
            "delta_raw": follower_state.get("raw_steering_delta"),
            "u_cacc": follower_state.get("u_cacc"),
            "u_sensor": follower_state.get("u_sensor"),
            "sensor_gap": follower_state.get(
                "sensor_leader_distance_filtered",
                follower_state.get("sensor_leader_distance"),
            ),
            "along_track_gap": follower_state.get("along_track_gap"),
            "distance_to_leader": distance_to_leader,
            "velocity_difference": velocity_difference,
            "reverse_follow_active": bool(
                follower_state.get("reverse_follow_active", False)
            ),
            "reverse_follow_blocked": bool(
                follower_state.get("reverse_follow_blocked", False)
            ),
            "multi_predecessor_count": follower_state.get(
                "multi_predecessor_count"
            ),
            "multi_predecessor_weight_sum": follower_state.get(
                "multi_predecessor_weight_sum"
            ),
            "multi_predecessor_spacing_term": follower_state.get(
                "multi_predecessor_spacing_term"
            ),
            "multi_predecessor_velocity_term": follower_state.get(
                "multi_predecessor_velocity_term"
            ),
            "multi_predecessor_acceleration_term": follower_state.get(
                "multi_predecessor_acceleration_term"
            ),
        }

        observer = getattr(self.vehicle_logic, "vehicle_observer", None)
        if observer is None or not hasattr(observer, "set_controller_debug_snapshot"):
            return
        try:
            observer.set_controller_debug_snapshot(snapshot)
        except Exception:
            pass


    def _apply_trust_longitudinal_fusion(
        self,
        u_cacc: float,
        follower_state: Dict[str, Any],
        leader_state: Dict[str, Any],
        dt: float,
    ) -> float:
        """Blend CACC with local sensor ACC according to leader trust."""
        cfg = self.trust_longitudinal_fusion_config
        trust_low = float(cfg.get("trust_low", 0.50))
        trust_high = float(cfg.get("trust_high", 0.80))
        fusion_opinion = self._get_trust_fusion_opinion(
            follower_state, leader_state
        )
        alpha = self._compute_trust_fusion_alpha_for_mode(
            fusion_opinion, trust_low, trust_high
        )

        u_sensor = self._compute_sensor_acc_command(
            follower_state, leader_state, dt
        )

        if alpha is None:
            unavailable_policy = str(
                cfg.get("unavailable_policy", "legacy_cacc")
            ).strip().lower()
            if unavailable_policy == "sensor_acc_or_stop":
                if u_sensor is not None:
                    self._set_longitudinal_fusion_log_fields(
                        follower_state,
                        u_cacc,
                        u_sensor,
                        0.0,
                        "trust_unavailable_sensor_acc",
                    )
                    return float(u_sensor)
                self._set_longitudinal_fusion_log_fields(
                    follower_state,
                    u_cacc,
                    None,
                    0.0,
                    "trust_unavailable_stop_no_sensor",
                )
                return 0.0

            self._set_longitudinal_fusion_log_fields(
                follower_state,
                u_cacc,
                u_sensor,
                1.0,
                "trust_unavailable_legacy_cacc",
            )
            return float(u_cacc)

        if (
            float(fusion_opinion) < trust_low
            and str(cfg.get("low_trust_policy", "sensor_acc_or_stop")).strip().lower()
            == "legacy_cacc"
        ):
            self._set_longitudinal_fusion_log_fields(
                follower_state,
                u_cacc,
                u_sensor,
                1.0,
                "low_trust_legacy_cacc",
            )
            return float(u_cacc)

        if u_sensor is None:
            if float(fusion_opinion) < trust_low:
                if (
                    str(cfg.get("low_trust_policy", "sensor_acc_or_stop"))
                    .strip()
                    .lower()
                    == "sensor_acc_or_cacc"
                ):
                    self._set_longitudinal_fusion_log_fields(
                        follower_state,
                        u_cacc,
                        None,
                        1.0,
                        "low_trust_no_sensor_cacc",
                    )
                    return float(u_cacc)

                self._set_longitudinal_fusion_log_fields(
                    follower_state,
                    u_cacc,
                    None,
                    0.0,
                    "low_trust_stop_no_sensor",
                )
                return 0.0

            self._set_longitudinal_fusion_log_fields(
                follower_state,
                u_cacc,
                None,
                1.0,
                "no_sensor_legacy_cacc",
            )
            return float(u_cacc)

        fused_u = float(alpha * u_cacc + (1.0 - alpha) * u_sensor)
        if alpha <= 1e-6:
            policy = "low_trust_sensor_acc"
        elif alpha >= 1.0 - 1e-6:
            policy = "high_trust_cacc"
        else:
            policy = "trust_blend"

        self._set_longitudinal_fusion_log_fields(
            follower_state, u_cacc, u_sensor, alpha, policy
        )
        return fused_u

    def _blend_sensor_acc_command(
        self,
        base_u: float,
        follower_state: Dict[str, Any],
        leader_state: Dict[str, Any],
        dt: float,
        sensor_u: Optional[float] = None,
    ) -> float:
        """Blend local close-range ACC with V2V CACC near the leader bumper."""
        if self.sensor_acc_controller is None:
            return base_u

        measured_distance = follower_state.get("sensor_leader_distance_filtered")
        if measured_distance is None:
            return base_u

        if sensor_u is None:
            sensor_u = self.sensor_acc_controller.compute_throttle(
                follower_state, leader_state, dt
            )

        stop_distance = float(self.sensor_acc_config.get("stop_distance", 0.20))
        desired_distance = max(
            float(self.sensor_acc_config.get("desired_distance", stop_distance)),
            stop_distance + 1e-3,
        )
        proximity_ratio = float(
            np.clip(
                (float(measured_distance) - stop_distance)
                / max(desired_distance - stop_distance, 1e-3),
                0.0,
                1.0,
            )
        )
        blend_alpha = float(
            np.clip(self.sensor_acc_config.get("blend_alpha", 0.7), 0.0, 1.0)
        )
        effective_alpha = blend_alpha * proximity_ratio
        return float(effective_alpha * base_u + (1.0 - effective_alpha) * sensor_u)

    def _should_hold_stop(
        self,
        follower_state: Dict[str, Any],
        leader_state: Dict[str, Any],
    ) -> bool:
        """Hold zero command once the follower reaches the stand-off zone."""
        leader_speed = abs(float(leader_state.get("velocity", 0.0)))
        follower_speed = abs(float(follower_state.get("velocity", 0.0)))
        if leader_speed > 0.05 or follower_speed > 0.12:
            return False

        measured_gap = follower_state.get("sensor_leader_distance_filtered")
        if measured_gap is not None:
            stop_target = float(
                self.sensor_acc_config.get(
                    "desired_distance",
                    getattr(self.longitudinal_controller, "s0", 0.2),
                )
            )
            stop_margin = 0.03
            return float(measured_gap) <= stop_target + stop_margin

        along_track_gap = follower_state.get("along_track_gap")
        if along_track_gap is None:
            return False

        stop_target = float(getattr(self.longitudinal_controller, "s0", 0.2))
        return float(along_track_gap) <= stop_target + 0.02

    def _get_v2v_leader_data(self) -> Optional[Dict[str, Any]]:
        """Get leader data for control from the configured source."""
        if not hasattr(self.vehicle_logic, "platoon_controller"):
            return None

        # Check if platoon controller is properly configured
        pc = self.vehicle_logic.platoon_controller

        # If this vehicle is actually the leader, it shouldn't be following anyone
        if hasattr(pc, "is_leader") and pc.is_leader:
            self.logger.logger.warning("Leader vehicle trying to get V2V leader data")
            # TODO : Consider transitioning back to FOLLOWING_PATH state
            return None

        source_mode = str(
            self.leader_source_config.get("mode", "direct_v2v_attacked")
        ).strip().lower()
        if source_mode == "fleet_estimator":
            return self._get_leader_data_from_fleet_estimator()
        v2v_channel = "clean" if source_mode == "direct_v2v_clean" else "attacked"
        v2v_data = self.vehicle_logic.platoon_controller.get_direct_leader_data_from_v2v(
            self.vehicle_logic.v2v_manager,
            self.vehicle_logic.vehicle_id,
            channel=v2v_channel,
        )
        if v2v_data is not None:
            v2v_data = dict(v2v_data)
            leader_vehicle_id = self._get_direct_leader_vehicle_id()
            if leader_vehicle_id is not None:
                v2v_data.setdefault("vehicle_id", leader_vehicle_id)
            v2v_data.setdefault("source", source_mode)

        # Log V2V data status occasionally for debugging
        if (
            hasattr(self.vehicle_logic, "loop_counter")
            and self.vehicle_logic.loop_counter % 1000 == 0
        ):
            if v2v_data is not None:
                self.logger.logger.debug(
                    f"[FOLLOW] Leader data available from {source_mode}"
                )
            else:
                self.logger.logger.debug(
                    f"[FOLLOW] No leader data received from {source_mode}"
                )

        return v2v_data

    def _get_leader_data_from_fleet_estimator(self) -> Optional[Dict[str, Any]]:
        """Get direct leader state from the fleet estimator output."""
        platoon_controller = getattr(self.vehicle_logic, "platoon_controller", None)
        if platoon_controller is None:
            return None

        if not hasattr(platoon_controller, "get_direct_leader_vehicle_id"):
            return None

        leader_vehicle_id = platoon_controller.get_direct_leader_vehicle_id()
        if leader_vehicle_id is None:
            return None

        data = self._get_vehicle_data_from_fleet_estimator(leader_vehicle_id)
        if data is None:
            return None

        if (
            hasattr(self.vehicle_logic, "loop_counter")
            and self.vehicle_logic.loop_counter % 1000 == 0
        ):
            self.logger.logger.debug(
                f"[FOLLOW] Using fleet estimator leader state from vehicle {leader_vehicle_id}"
            )
        return data

    def _normalize_waypoint_sequence(self, waypoint_sequence) -> Optional[np.ndarray]:
        """Normalize waypoint sequence to shape [N, 2]."""
        if waypoint_sequence is None:
            return None

        try:
            wp = np.asarray(waypoint_sequence, dtype=float)
        except Exception:
            return None

        if wp.ndim != 2:
            return None

        # Common format in this project is [2, N]
        if wp.shape[0] >= 2 and wp.shape[1] >= 2:
            if wp.shape[0] == 2:
                xy = wp[:2, :].T
            elif wp.shape[1] == 2:
                xy = wp[:, :2]
            else:
                return None
        else:
            return None

        finite_mask = np.isfinite(xy).all(axis=1)
        xy = xy[finite_mask]
        if xy.shape[0] < 2:
            return None

        # Drop consecutive duplicates to avoid zero-length segments in projection
        seg_len = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        keep = np.ones(xy.shape[0], dtype=bool)
        keep[1:] = seg_len > 1e-6
        xy = xy[keep]
        if xy.shape[0] < 2:
            return None

        return xy

    def _refresh_route_projection_cache(self) -> None:
        """Refresh cached route projection data from current waypoints if needed."""
        waypoint_sequence = getattr(self.vehicle_logic, "waypoint_sequence", None)
        if waypoint_sequence is None:
            self._route_waypoint_ref = None
            self._route_xy = None
            self._route_s_axis = None
            self._route_track_length = 0.0
            return

        # Fast-path: same object already cached
        if waypoint_sequence is self._route_waypoint_ref and self._route_xy is not None:
            return

        xy = self._normalize_waypoint_sequence(waypoint_sequence)
        if xy is None:
            self._route_waypoint_ref = waypoint_sequence
            self._route_xy = None
            self._route_s_axis = None
            self._route_track_length = 0.0
            return

        seg_len = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        s_axis = np.zeros(xy.shape[0], dtype=float)
        s_axis[1:] = np.cumsum(seg_len)

        self._route_waypoint_ref = waypoint_sequence
        self._route_xy = xy
        self._route_s_axis = s_axis
        self._route_track_length = float(max(s_axis[-1], 1e-6))

    def _project_to_route_frenet(
        self, x: float, y: float
    ) -> Optional[Tuple[float, float]]:
        """
        Project Cartesian point onto cached route and return Frenet-like (s, d).

        Returns:
            (s, d) or None if route cache is unavailable.
        """
        if self._route_xy is None or self._route_s_axis is None:
            return None

        path_xy = self._route_xy
        p = np.array([float(x), float(y)], dtype=float)
        seg_start = path_xy[:-1, :]
        seg_end = path_xy[1:, :]
        seg_vec = seg_end - seg_start
        seg_len_sq = np.sum(seg_vec * seg_vec, axis=1)

        with np.errstate(divide="ignore", invalid="ignore"):
            rel = p[None, :] - seg_start
            t = np.sum(rel * seg_vec, axis=1) / np.maximum(seg_len_sq, 1e-9)
        t = np.clip(t, 0.0, 1.0)

        proj = seg_start + seg_vec * t[:, None]
        diff = p[None, :] - proj
        dist_sq = np.sum(diff * diff, axis=1)
        i = int(np.argmin(dist_sq))

        tangent = seg_vec[i]
        tangent_norm = float(np.linalg.norm(tangent))
        if tangent_norm < 1e-9:
            return float(self._route_s_axis[i]), 0.0

        tangent_unit = tangent / tangent_norm
        normal_unit = np.array([-tangent_unit[1], tangent_unit[0]], dtype=float)
        d = float(np.dot(p - proj[i], normal_unit))
        s = float(self._route_s_axis[i] + t[i] * tangent_norm)

        if self._route_track_length > 1e-6:
            s = s % self._route_track_length

        return s, d

    def _compute_along_track_gap(
        self, follower_x: float, follower_y: float, leader_x: float, leader_y: float
    ) -> Optional[Tuple[float, Tuple[float, float], Tuple[float, float]]]:
        """Compute signed along-track gap (leader_s - follower_s) on shared route."""
        self._refresh_route_projection_cache()
        follower_frenet = self._project_to_route_frenet(follower_x, follower_y)
        leader_frenet = self._project_to_route_frenet(leader_x, leader_y)

        if follower_frenet is None or leader_frenet is None:
            return None

        follower_s, follower_d = follower_frenet
        leader_s, leader_d = leader_frenet
        gap = leader_s - follower_s

        # Signed shortest gap on cyclic route to avoid wrap jumps.
        if self._route_track_length > 1e-6:
            half_track = 0.5 * self._route_track_length
            if gap > half_track:
                gap -= self._route_track_length
            elif gap < -half_track:
                gap += self._route_track_length

        return gap, (follower_s, follower_d), (leader_s, leader_d)

    def _compute_control(
        self,
        dt: float,
        x: float,
        y: float,
        theta: float,
        velocity: float,
        acceleration: float,
        v2v_data: Optional[Dict[str, Any]],
        yolo_data: Dict[str, Any] = None,
    ) -> Tuple[float, float]:
        """Compute control commands using modular longitudinal and lateral controllers"""
        base_velocity = self.vehicle_logic.v_ref

        # No V2V data - stop
        if v2v_data is None:
            if (
                self._reverse_follow_active
                and self.reverse_follow_config.get("stop_on_v2v_loss", True)
            ):
                self.logger.logger.warning(
                    "[FOLLOW] Reverse follow halted: V2V leader data lost"
                )
            self._reverse_follow_active = False
            if (
                hasattr(self.vehicle_logic, "loop_counter")
                and self.vehicle_logic.loop_counter % 200 == 0
            ):
                self.logger.logger.warning("[FOLLOW] V2V data is None - stopping")
            self._publish_controller_debug_snapshot(
                {
                    "x": x,
                    "y": y,
                    "theta": theta,
                    "velocity": velocity,
                    "acceleration": acceleration,
                    "target_velocity": 0.0,
                    "trust_fusion_policy": "no_v2v_stop",
                    "trust_fusion_alpha": 0.0,
                    "raw_throttle_u": 0.0,
                    "raw_steering_delta": 0.0,
                },
                {},
                0.0,
                0.0,
            )
            return 0.0, 0.0

        if self.longitudinal_controller is None:
            if (
                hasattr(self.vehicle_logic, "loop_counter")
                and self.vehicle_logic.loop_counter % 200 == 0
            ):
                self.logger.logger.error(
                    "[FOLLOW] No longitudinal controller available - holding stop"
                )
            self._publish_controller_debug_snapshot(
                {
                    "x": x,
                    "y": y,
                    "theta": theta,
                    "velocity": velocity,
                    "acceleration": acceleration,
                    "target_velocity": 0.0,
                    "trust_fusion_policy": "no_controller_stop",
                    "trust_fusion_alpha": 0.0,
                    "raw_throttle_u": 0.0,
                    "raw_steering_delta": 0.0,
                },
                {},
                0.0,
                0.0,
            )
            return 0.0, 0.0

        # Get leader theta for turning detection
        leader_theta = v2v_data.get("theta", 0.0)

        # Detect turning section using YOLO lane data + leader orientation
        is_turning, turn_direction, curvature = self._detect_turning_section(
            yolo_data or {}, theta, leader_theta
        )
        sensor_gap = self._update_sensor_acc_distance(yolo_data)

        # Prepare follower state with turning context
        follower_state = {
            "x": x,
            "y": y,
            "theta": theta,
            "velocity": velocity,
            "acceleration": acceleration,
            "target_velocity": base_velocity,
            # "motor_tach": float(sensor_data.get("motor_tach", velocity)),
            # "battery_voltage": float(sensor_data.get("battery_voltage", 0.0)),
            # Turning context for dynamic lookahead
            "is_turning": is_turning,
            "turn_direction": turn_direction,
            "curvature": curvature,
        }
        if sensor_gap is not None:
            follower_state["sensor_leader_distance"] = float(sensor_gap)
            follower_state["sensor_leader_distance_filtered"] = float(sensor_gap)

        # Prepare leader state from V2V data
        leader_x = v2v_data.get("x", 0.0)
        leader_y = v2v_data.get("y", 0.0)
        leader_state = {
            "vehicle_id": v2v_data.get("vehicle_id"),
            "x": leader_x,
            "y": leader_y,
            "theta": leader_theta,
            "velocity": v2v_data.get("velocity", 0.0),
            "acceleration": v2v_data.get("acceleration", 0.0),
            "source": v2v_data.get("source", "v2v"),
        }
        self._attach_leader_trust_context(leader_state)

        # Compute path-based along-track gap when route waypoints are available.
        along_track = self._compute_along_track_gap(x, y, leader_x, leader_y)
        # leader_state.get("path_d"), it is looking at a value that was entirely computed locally by the follower car based purely on the raw 
        # (x, y) coordinates the leader sent over the network.
        if along_track is not None:
            gap_s, follower_frenet, leader_frenet = along_track
            follower_state["along_track_gap"] = gap_s
            follower_state["path_s"] = follower_frenet[0]
            follower_state["path_d"] = follower_frenet[1]
            leader_state["path_s"] = leader_frenet[0]
            leader_state["path_d"] = leader_frenet[1]

        self._attach_clean_v2v_leader_baseline(follower_state, leader_state)

        predecessor_states = self._collect_multi_predecessor_states(
            follower_state, leader_state
        )
        if predecessor_states:
            follower_state["multi_predecessor_states"] = predecessor_states

        reverse_follow_active = self._should_activate_reverse_follow(
            follower_state, leader_state, v2v_data
        )
        reverse_follow_blocked = self._should_force_reverse_stop(
            follower_state, leader_state, v2v_data
        )

        if reverse_follow_active:
            reverse_speed_target = min(
                abs(float(leader_state.get("velocity", 0.0))),
                float(self.reverse_follow_config.get("max_reverse_speed", 0.12)),
            )
            follower_state["reverse_follow_active"] = True
            follower_state["reverse_follow_config"] = dict(self.reverse_follow_config)
            follower_state["target_velocity"] = -reverse_speed_target
            follower_state["trust_fusion_policy"] = "reverse_follow"

            u = self.longitudinal_controller.compute_throttle(
                follower_state, leader_state, dt
            )
            delta = self._compute_reverse_steering(follower_state, leader_state)
            self.vehicle_logic.v_ref_actual = -reverse_speed_target
            self._reverse_follow_active = True
        elif reverse_follow_blocked:
            follower_state["reverse_follow_active"] = False
            follower_state["reverse_follow_blocked"] = True
            follower_state["trust_fusion_policy"] = "reverse_blocked"
            follower_state["target_velocity"] = 0.0
            u = 0.0
            delta = 0.0
            self.vehicle_logic.v_ref_actual = 0.0
            self._reverse_follow_active = False
        else:
            self._reverse_follow_active = False

            # Compute throttle using modular longitudinal controller
            u_cacc = self.longitudinal_controller.compute_throttle(
                follower_state, leader_state, dt
            )
            if self.trust_longitudinal_fusion_config.get("enabled", False):
                u = self._apply_trust_longitudinal_fusion(
                    u_cacc, follower_state, leader_state, dt
                )
            else:
                u_sensor = self._compute_sensor_acc_command(
                    follower_state, leader_state, dt
                )
                u = self._blend_sensor_acc_command(
                    u_cacc, follower_state, leader_state, dt, sensor_u=u_sensor
                )
                self._set_longitudinal_fusion_log_fields(
                    follower_state,
                    u_cacc,
                    u_sensor,
                    1.0,
                    "legacy_sensor_acc_blend"
                    if u_sensor is not None
                    else "legacy_cacc",
                )

            delta = self._compute_lateral_control(
                follower_state=follower_state,
                leader_state=leader_state,
                dt=dt,
            )

            # Record actual target speed for scope display
            self.vehicle_logic.v_ref_actual = base_velocity

            if self._should_hold_stop(follower_state, leader_state):
                u = 0.0
                self.vehicle_logic.v_ref_actual = 0.0
                previous_policy = follower_state.get("trust_fusion_policy")
                follower_state["trust_fusion_policy"] = (
                    f"{previous_policy}+hold_stop"
                    if previous_policy
                    else "hold_stop"
                )

        raw_u = u
        raw_delta = delta
        follower_state["raw_throttle_u"] = float(raw_u)
        follower_state["raw_steering_delta"] = float(raw_delta)

        # Apply unified command smoothing
        if hasattr(self.vehicle_logic, "controller_manager"):
            cm = self.vehicle_logic.controller_manager
            if hasattr(cm, "config") and hasattr(cm.config, "get_command_smoothing_config"):
                smooth_cfg = cm.config.get_command_smoothing_config()
                long_cfg = smooth_cfg.get("longitudinal", {})
                lat_cfg = smooth_cfg.get("lateral", {})
                u = self._smooth_cmd(u, self._prev_u, dt, 
                                     long_cfg.get("alpha", 0.7),
                                     long_cfg.get("rise_rate", 0.25),
                                     long_cfg.get("fall_rate", 0.40))
                delta = self._smooth_cmd(delta, self._prev_delta, dt,
                                          lat_cfg.get("alpha", 0.8),
                                         lat_cfg.get("rise_rate", 1.0),
                                         lat_cfg.get("fall_rate", 1.0))

        # Log final command after smoothing, plus raw pre-smoothing values.
        self._publish_controller_debug_snapshot(follower_state, leader_state, u, delta)
        self.logger.log_following_leader_control(
            follower_state=follower_state, leader_state=leader_state, u=u, delta=delta
        )

        self._prev_u = u
        self._prev_delta = delta

        # u = 0.05
        # delta = 0
        return u, delta

    def _compute_lateral_control(
        self,
        follower_state: Dict[str, Any],
        leader_state: Optional[Dict[str, Any]],
        dt: float,
    ) -> float:
        """Compute steering from the same leader state used by longitudinal control."""
        x = float(follower_state["x"])
        y = float(follower_state["y"])
        theta = float(follower_state["theta"])
        velocity = float(follower_state["velocity"])

        if self.lateral_controller_type in ("path", "pp_map"):
            return self._compute_path_steering(x, y, theta, velocity)

        if self.lateral_controller_type in ("fusion", "fusion_lateral"):
            return self._compute_fusion_steering(follower_state, leader_state, dt)

        if leader_state is None or self.lateral_controller is None:
            return self._compute_path_steering(x, y, theta, velocity)

        return self.lateral_controller.compute_steering(
            follower_state, leader_state, dt
        )

    def _compute_path_steering(
        self, x: float, y: float, theta: float, velocity: float
    ) -> float:
        """Compute steering control using path-following (like in FOLLOWING_PATH state)"""
        if (
            not self.vehicle_logic.controller_manager.config.enable_steering_control
            or not self.steering_controller
        ):
            return 0.0

        p = np.array([x, y])
        return self.steering_controller.update(p, theta, max(velocity, 0.1))

    def _compute_fusion_steering(
        self,
        follower_state: Dict[str, Any],
        leader_state: Optional[Dict[str, float]],
        dt: float,
    ) -> float:
        """
        Compute steering using fusion of path following and leader tracking.

        This mode combines the smoothness of path-based steering with the
        adaptability of leader position tracking. The path provides the baseline
        trajectory while the leader position provides deviation corrections.

        Args:
            follower_state: Current vehicle state, including turn/path context
            leader_state: Leader position and heading from V2V
            dt: Time step

        Returns:
            Fused steering command in radians
        """
        # Get path-based steering (primary reference)
        path_steering = self._compute_path_steering(
            follower_state["x"],
            follower_state["y"],
            follower_state["theta"],
            follower_state["velocity"],
        )

        if leader_state is None:
            return path_steering

        if self.lateral_controller is not None:
            return self.lateral_controller.compute_steering(
                follower_state, leader_state, dt, path_steering=path_steering
            )

        return path_steering

    def _log_status(
        self,
        velocity: float,
        throttle_cmd: float = None,
        v2v_data: Optional[Dict[str, Any]] = None,
    ):
        """Enhanced periodic logging for debugging"""
        if (
            hasattr(self.vehicle_logic, "loop_counter")
            and self.vehicle_logic.loop_counter % 200 == 0
        ):  # Every second at 200Hz
            status = "unknown"
            leader_info = ""
            spacing_info = ""

            if hasattr(self.vehicle_logic, "platoon_controller"):
                pc = self.vehicle_logic.platoon_controller

                # Check for V2V leader data first (preferred for followers)
                if v2v_data is not None:
                    leader_velocity = v2v_data.get("velocity", 0.0)
                    status = "following_v2v"
                    leader_info = f"L_vel: {leader_velocity:.2f}m/s"
                elif pc.is_spacing_stable():
                    status = "stable"
                elif pc.leader_detected:
                    status = f"following_yolo (dist: {pc.leader_distance:.2f}m)"
                else:
                    status = "no_leader"

            # Enhanced logging with control command details
            if throttle_cmd is not None:
                self.logger.logger.info(
                    f"[FOLLOW] {status} | V: {velocity:.2f}m/s | "
                    f"Throttle: {throttle_cmd:.3f} | {leader_info} | {spacing_info}"
                )
            else:
                self.logger.logger.info(
                    f"[FOLLOW] Status: {status}, velocity: {velocity:.2f}m/s"
                )

    def handle_event(
        self, command_type, data: Dict[str, Any] = None
    ) -> Optional[Tuple[VehicleState, StateTransitionReason]]:
        """Handle events while following leader"""
        data = data or {}

        # Check if CommandType import was successful
        if not COMMAND_TYPE_AVAILABLE:
            return super().handle_event(command_type, data)

        # Handle platoon disable command
        if command_type == CommandType.DISABLE_PLATOON:
            self.logger.logger.info(
                "Disabling platoon mode - returning to path following"
            )
            if hasattr(self.vehicle_logic, "platoon_controller"):
                self.vehicle_logic.platoon_controller.disable()
            return (VehicleState.FOLLOWING_PATH, StateTransitionReason.PLATOON_COMMAND)

        # Let base class handle common events (stop, emergency_stop, set_velocity, etc.)
        return super().handle_event(command_type, data)

    def exit(self):
        """Clean up when leaving leader following state"""
        self.logger.logger.info("[FOLLOW] Exiting FOLLOWING_LEADER state")

        # Log session statistics
        session_time = self.get_time_in_state()
        self.logger.logger.info(
            f"Leader following session duration: {session_time:.1f}s"
        )

        # Disable platoon controller if available
        if hasattr(self.vehicle_logic, "platoon_controller"):
            if self.vehicle_logic.platoon_controller.is_spacing_stable():
                self.logger.logger.info("Formation was established successfully")
            else:
                self.logger.logger.info("Formation was not fully established")
            self.vehicle_logic.platoon_controller.disable()

        # Reset controllers
        self._reverse_follow_active = False
        self._sensor_acc_distance_filtered = None
        if self.longitudinal_controller:
            self.longitudinal_controller.reset()
        if self.lateral_controller:
            self.lateral_controller.reset()
        if self.sensor_acc_controller:
            self.sensor_acc_controller.reset()
        self.logger.logger.info("Controllers reset")

        super().exit()
