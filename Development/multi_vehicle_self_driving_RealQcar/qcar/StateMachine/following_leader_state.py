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

    COMMAND_TYPE_AVAILABLE = True
except ImportError as e:
    print(f"ERROR: Cannot import CommandType: {e}")
    COMMAND_TYPE_AVAILABLE = False
    CommandType = None


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

    def _get_v2v_leader_data(self) -> Optional[Dict[str, Any]]:
        """Get leader data from V2V"""
        if not hasattr(self.vehicle_logic, "platoon_controller"):
            return None

        # Check if platoon controller is properly configured
        pc = self.vehicle_logic.platoon_controller

        # If this vehicle is actually the leader, it shouldn't be following anyone
        if hasattr(pc, "is_leader") and pc.is_leader:
            self.logger.logger.warning("Leader vehicle trying to get V2V leader data")
            # TODO : Consider transitioning back to FOLLOWING_PATH state
            return None

        # TODO: Should have the choice to use data direct from v2v_manager or Estimated by Observer
        v2v_data = (
            self.vehicle_logic.platoon_controller.get_direct_leader_data_from_v2v(
                self.vehicle_logic.v2v_manager, self.vehicle_logic.vehicle_id
            )
        )

        # Log V2V data status occasionally for debugging
        if (
            hasattr(self.vehicle_logic, "loop_counter")
            and self.vehicle_logic.loop_counter % 1000 == 0
        ):
            if v2v_data is not None:
                self.logger.logger.debug(f"[FOLLOW] V2V Leader data available")
            else:
                self.logger.logger.debug(f"[FOLLOW] No V2V leader data received")

        return v2v_data

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
                hasattr(self.vehicle_logic, "loop_counter")
                and self.vehicle_logic.loop_counter % 200 == 0
            ):
                self.logger.logger.warning("[FOLLOW] V2V data is None - stopping")
            return 0.0, 0.0

        # Get leader theta for turning detection
        leader_theta = v2v_data.get("theta", 0.0)

        # Detect turning section using YOLO lane data + leader orientation
        is_turning, turn_direction, curvature = self._detect_turning_section(
            yolo_data or {}, theta, leader_theta
        )

        # Prepare follower state with turning context
        follower_state = {
            "x": x,
            "y": y,
            "theta": theta,
            "velocity": velocity,
            "acceleration": acceleration,
            "target_velocity": base_velocity,
            # Turning context for dynamic lookahead
            "is_turning": is_turning,
            "turn_direction": turn_direction,
            "curvature": curvature,
        }

        # Prepare leader state from V2V data
        leader_x = v2v_data.get("x", 0.0)
        leader_y = v2v_data.get("y", 0.0)
        leader_state = {
            "x": leader_x,
            "y": leader_y,
            "theta": leader_theta,
            "velocity": v2v_data.get("velocity", 0.0),
            "acceleration": v2v_data.get("acceleration", 0.0),
        }

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

        # Compute throttle using modular longitudinal controller
        u = self.longitudinal_controller.compute_throttle(
            follower_state, leader_state, dt
        )

        # Compute steering based on lateral control mode
        if self.lateral_controller_type in ("path", "pp_map"):
            # Path-following mode: use steering controller with waypoints
            delta = self._compute_path_steering(x, y, theta, velocity)
        elif self.lateral_controller_type in ("fusion", "fusion_lateral"):
            # Fusion mode: combines path steering with leader tracking
            delta = self._compute_fusion_steering(
                x, y, theta, velocity, leader_state, dt
            )
        else:
            # Leader-following mode: use lateral controller to follow leader position
            delta = self.lateral_controller.compute_steering(
                follower_state, leader_state, dt
            )

        # Log following leader control data
        self.logger.log_following_leader_control(
            follower_state=follower_state, leader_state=leader_state, u=u, delta=delta
        )

        # Record actual target speed for scope display
        self.vehicle_logic.v_ref_actual = base_velocity

        # u = 0.05
        # delta = 0
        return u, delta

    def _compute_path_steering(
        self, x: float, y: float, theta: float, velocity: float
    ) -> float:
        """Compute steering control using path-following (like in FOLLOWING_PATH state)"""
        if (
            not self.vehicle_logic.controller_manager.config.enable_steering_control
            or not self.steering_controller
        ):
            return 0.0

        # Use look-ahead point (0.2m forward from vehicle center)
        p = np.array([x, y]) + np.array([np.cos(theta), np.sin(theta)]) * 0.2
        return self.steering_controller.update(p, theta, max(velocity, 0.1))

    def _compute_fusion_steering(
        self,
        x: float,
        y: float,
        theta: float,
        velocity: float,
        leader_state: Dict[str, float],
        dt: float,
    ) -> float:
        """
        Compute steering using fusion of path following and leader tracking.

        This mode combines the smoothness of path-based steering with the
        adaptability of leader position tracking. The path provides the baseline
        trajectory while the leader position provides deviation corrections.

        Args:
            x, y: Current position
            theta: Current heading
            velocity: Current velocity
            leader_state: Leader position and heading from V2V
            dt: Time step

        Returns:
            Fused steering command in radians
        """
        # Get path-based steering (primary reference)
        path_steering = self._compute_path_steering(x, y, theta, velocity)

        if self.lateral_controller is not None:
            follower_state = {"x": x, "y": y, "theta": theta, "velocity": velocity}
            if hasattr(self.vehicle_logic, "current_state_data"):
                follower_state.update(self.vehicle_logic.current_state_data)

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
        self.longitudinal_controller.reset()
        if self.lateral_controller:
            self.lateral_controller.reset()
        self.logger.logger.info("Controllers reset")

        super().exit()
