"""
Taxi Mode State - Event-Driven Autonomous Taxi
Inherits from FollowingPathState to reuse waypoint following functionality,
but adds logic to interface with the TaxiManager for multi-stop trips.

Quanser Competition Scenario (without LED — LED added later):
  Hub → pick-up (full stop 3 s) → drop-off (full stop 3 s) → Hub

YOLO Integration:
- Stop signs:    yolo_gain -> 0.0  => vehicle stops completely
- Red traffic:   yolo_gain -> 0.0  => vehicle stops until green
- Yield signs:   yolo_gain -> 0.5  => vehicle slows to half speed
- Cars/persons:  yolo_gain -> proportional braking by distance
All handled via the inherited FollowingPathState which applies yolo_gain
to the throttle output. TaxiModeState adds taxi-specific logging.
"""

import time
import numpy as np
from typing import Dict, Any, Tuple, Optional

from command_types import CommandType
from .vehicle_state import VehicleState, StateTransitionReason
from .following_path_state import FollowingPathState
from Taxi.taxi_manager import TripSuperState, TripState


class TaxiModeState(FollowingPathState):
    """
    Handler for TAXI_MODE state.
    Uses TaxiManager to get sequence of nodes, generates path for each segment,
    drives there, waits, and then goes to the next segment.

    YOLO-aware: Reacts to stop signs and red traffic lights by zeroing throttle.
    """

    def __init__(self, vehicle_logic):
        super().__init__(vehicle_logic)
        self.state_name = "TAXI_MODE"
        self.taxi_manager = None
        self.dist_threshold = 0.5  # Distance to consider "arrived" at destination

        # Taxi Hub coordinates (Node 10)
        self.hub_x = -1.28205
        self.hub_y = -0.45991

        # YOLO reaction tracking (for logging, no duplicate spam)
        self._yolo_stopped = False
        self._yolo_stop_reason = ""
        self._yolo_log_cooldown = 0.0

        self.scale_factor = 1.0

    def _set_stop_speed_reference(self) -> None:
        """Ensure scope/telemetry reflects a full stop during taxi waits/stops."""
        self.vehicle_logic.v_ref_actual = 0.0

    def enter(self) -> bool:
        """Initialize taxi mode"""
        success = super().enter()
        if not success:
            return False

        # Taxi segments are A→B (non-cyclic), override the default is_cyclic=True
        if self.rich_planner is not None:
            self.rich_planner.is_cyclic = False

        self.logger.logger.info("[TAXI] Entering TAXI_MODE state")

        # Need access to TaxiManager
        if (
            not hasattr(self.vehicle_logic, "taxi_manager")
            or self.vehicle_logic.taxi_manager is None
        ):
            self.logger.logger.error("[TAXI] TaxiManager not found in VehicleLogic!")
            return False

        self.taxi_manager = self.vehicle_logic.taxi_manager

        # Check if already at hub physically. If yes, this will cleanly set the state to 2.
        # Handle sensor_data via vehicle_observer because vehicle_logic doesn't store it
        sensor_data = {}
        if (
            hasattr(self.vehicle_logic, "vehicle_observer")
            and self.vehicle_logic.vehicle_observer
        ):
            sensor_data = (
                self.vehicle_logic.vehicle_observer.get_estimated_state_for_control()
            )

        curr_x = sensor_data.get("x", 0.0)
        curr_y = sensor_data.get("y", 0.0)

        self.taxi_manager.check_hub_arrival(
            curr_x, curr_y, self.hub_x, self.hub_y, self.dist_threshold
        )

        if self.taxi_manager.trip_super_state == TripSuperState.INITIALIZING:
            if not getattr(self, "notified_not_at_hub", False):
                self.logger.logger.warning(
                    "[TAXI] Vehicle is NOT at the taxi hub! Please send ENABLE_TAXI_MODE command again to automatically route to the hub."
                )
                self.notified_not_at_hub = True
                return True
            else:
                self.logger.logger.info("[TAXI] Auto-routing to Taxi Hub...")

                # Try to get the current node based on position
                current_node = None
                if (
                    hasattr(self.vehicle_logic, "roadmap")
                    and self.vehicle_logic.roadmap
                ):
                    # Find the nearest node to the vehicle's current position
                    nearest_dist = float("inf")
                    for node_idx, node in enumerate(self.vehicle_logic.roadmap.nodes):
                        dist = (
                            (curr_x - node.pose[0, 0]) ** 2
                            + (curr_y - node.pose[1, 0]) ** 2
                        ) ** 0.5
                        if dist < nearest_dist:
                            nearest_dist = dist
                            current_node = node_idx

                self.taxi_manager.dispatch_to_hub(current_node=current_node)
                self.notified_not_at_hub = False  # Reset for future toggles

        # Request path for the current segment if driving
        self._update_taxi_segment()
        return True

    def _update_taxi_segment(self) -> bool:
        """
        Fetches current segment from TaxiManager and sets up the path.
        Returns True if path generated successfully, False otherwise.
        """
        segment = self.taxi_manager.get_current_segment()
        if not segment or len(segment) < 2:
            self.logger.logger.info("[TAXI] No active segment available.")
            return False

        self.logger.logger.info(f"[TAXI] Generating path for segment: {segment}")

        if not (hasattr(self.vehicle_logic, "roadmap") and self.vehicle_logic.roadmap):
            self.logger.logger.error("[TAXI] No roadmap available for path generation!")
            return False

        try:
            new_waypoints = self.vehicle_logic.roadmap.generate_path(segment)

            # # Use RichSDCSPlanner for unified path generation when available,
            # # falling back to the raw roadmap otherwise.
            # if self.rich_planner is not None:
            #     self.rich_planner.is_cyclic = False  # Taxi segments are A→B
            #     new_waypoints = self.rich_planner.generate_path(segment)
            # else:
            #     new_waypoints = self.vehicle_logic.roadmap.generate_path(segment)

            if not self.vehicle_logic.is_physical_qcar and new_waypoints is not None:
                new_waypoints = new_waypoints * 0.975

            self.update_path(new_waypoints)

            # Update the vehicle logic's node_sequence so that it gets broadcasted
            # to the Ground Station for scope mode visualization.
            if (
                hasattr(self.taxi_manager, "path_nodes")
                and self.taxi_manager.path_nodes
            ):
                self.vehicle_logic.node_sequence = self.taxi_manager.path_nodes
            # ----------------

            self.logger.logger.info("[TAXI] Successfully updated path for new segment.")
            return True
        except Exception as e:
            self.logger.log_error("Failed to generate path from taxi nodes", e)
            return False

    def update(
        self, dt: float, sensor_data: Dict[str, Any]
    ) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        """Update taxi mode control"""

        # 1. State machine tick for taxi logic
        # If no active trip, just stop.
        if not self.taxi_manager.current_trip_status:
            self._set_stop_speed_reference()
            return 0.0, 0.0, None

        if self.taxi_manager.current_trip_state == TripState.WAITING:
            # We are waiting at a stop (pickup / dropoff / intermediate)
            if self.taxi_manager.check_wait_time():
                # Wait time elapsed — generate path for the next segment
                stop_type = self.taxi_manager.get_current_stop_type()
                self.logger.logger.info(
                    f"[TAXI] Wait over (was {stop_type}). Driving to next segment."
                )
                segment_ready = self._update_taxi_segment()
                if not segment_ready:
                    if not self.taxi_manager.current_trip_status:
                        self.logger.logger.info("[TAXI] Trip finished. Staying idle.")
                        self._set_stop_speed_reference()
                        return 0.0, 0.0, None
            else:
                # Still waiting at stop
                self._set_stop_speed_reference()
                return 0.0, 0.0, None

        # 2. We are driving (state == 1). Use FollowingPathState to compute control
        #    FollowingPathState.update() already applies yolo_gain to throttle,
        #    so stop signs / red lights / yield / cars / persons are handled.
        u, delta, transition = super().update(dt, sensor_data)

        # 2b. YOLO-aware taxi behaviour: log and enforce full stop
        yolo_data = sensor_data.get("yolo_data", None)
        if yolo_data:
            yolo_gain = yolo_data.get("yolo_gain", 1.0)

            if yolo_gain <= 0.01:
                u = 0.0
                reason = self._identify_stop_reason(yolo_data)
                if not self._yolo_stopped or reason != self._yolo_stop_reason:
                    self._yolo_stopped = True
                    self._yolo_stop_reason = reason
                    self.logger.logger.info(f"[TAXI-YOLO] Stopping: {reason}")
            elif yolo_gain < 1.0:
                if self._yolo_stopped:
                    self._yolo_stopped = False
                    self._yolo_stop_reason = ""
                    self.logger.logger.info(
                        "[TAXI-YOLO] Resuming movement (reduced speed)"
                    )
            else:
                if self._yolo_stopped:
                    self._yolo_stopped = False
                    self._yolo_stop_reason = ""
                    self.logger.logger.info("[TAXI-YOLO] Clear — resuming normal speed")

        # 3. Check if we arrived at the current segment's destination
        curr_x = sensor_data.get("x", None)
        curr_y = sensor_data.get("y", None)
        if curr_x is not None and curr_y is not None:
            dest_reached = self._check_arrival(curr_x, curr_y)
            if dest_reached:
                stop_type = self.taxi_manager._classify_stop(
                    self.taxi_manager.stop_index + 1
                )
                self.logger.logger.info(f"[TAXI] Reached stop! type={stop_type}")
                trip_done = self.taxi_manager.handle_stop_arrival()
                if trip_done:
                    self.logger.logger.info("[TAXI] Entire trip completed.")
                u = 0.0  # full stop
                self._set_stop_speed_reference()

        return u, delta, transition

    # ------------------------------------------------------------------
    # Arrival detection
    # ------------------------------------------------------------------

    def _check_arrival(self, curr_x: float, curr_y: float) -> bool:
        """
        Check if the vehicle has arrived at the last waypoint of the current
        segment.  Works with both PurePursuit and Stanley waypoint arrays.
        """
        # Try PurePursuit waypoint array first
        if hasattr(self, "pp_waypoint_array") and self.pp_waypoint_array is not None:
            last_x = self.pp_waypoint_array[-1, 0]
            last_y = self.pp_waypoint_array[-1, 1]
        else:
            # Stanley / generic waypoint_sequence (shape [>=2, N])
            ways = getattr(self.vehicle_logic, "waypoint_sequence", None)
            if ways is None or ways.shape[1] == 0:
                return False
            last_x, last_y = ways[0, -1], ways[1, -1]

        dist = np.sqrt((curr_x - last_x) ** 2 + (curr_y - last_y) ** 2)
        return dist < self.dist_threshold

    # ------------------------------------------------------------------
    # Event handling
    # ------------------------------------------------------------------

    def handle_event(
        self, command_type, data: Dict[str, Any] = None
    ) -> Optional[Tuple[VehicleState, StateTransitionReason]]:
        """
        Handle taxi specific events (e.g. SET_TAXI_TRIP, DISABLE_TAXI_MODE, ENABLE_TAXI_MODE).
        """
        if command_type == CommandType.ENABLE_TAXI_MODE:
            if self.taxi_manager.trip_super_state == TripSuperState.INITIALIZING:
                if getattr(self, "notified_not_at_hub", False):
                    self.logger.logger.info(
                        "[TAXI] Auto-routing to Taxi Hub (via supplementary command)..."
                    )

                    # Try to get the current node based on position
                    current_node = None
                    if (
                        hasattr(self.vehicle_logic, "roadmap")
                        and self.vehicle_logic.roadmap
                    ):
                        # Find the nearest node to the vehicle's current position
                        sensor_data = {}
                        if (
                            hasattr(self.vehicle_logic, "vehicle_observer")
                            and self.vehicle_logic.vehicle_observer
                        ):
                            sensor_data = self.vehicle_logic.vehicle_observer.get_estimated_state_for_control()

                        curr_x = sensor_data.get("x", 0.0)
                        curr_y = sensor_data.get("y", 0.0)

                        nearest_dist = float("inf")
                        for node_idx, node in enumerate(
                            self.vehicle_logic.roadmap.nodes
                        ):
                            dist = (
                                (curr_x - node.pose[0, 0]) ** 2
                                + (curr_y - node.pose[1, 0]) ** 2
                            ) ** 0.5
                            if dist < nearest_dist:
                                nearest_dist = dist
                                current_node = node_idx

                    self.taxi_manager.dispatch_to_hub(current_node=current_node)
                    self.notified_not_at_hub = False
                    self._update_taxi_segment()
                else:
                    self.logger.logger.warning(
                        "[TAXI] Vehicle is NOT at the taxi hub! Please send ENABLE_TAXI_MODE command again to automatically route to the hub."
                    )
                    self.notified_not_at_hub = True
            else:
                self.logger.logger.info("[TAXI] Taxi Mode is already active and ready.")
            return None

        if command_type == CommandType.DISABLE_TAXI_MODE:
            self.logger.logger.info("[TAXI] Disable taxi mode requested.")
            self.taxi_manager.cancel_current_trip()
            return (VehicleState.STOPPED, StateTransitionReason.STOP_COMMAND)

        if (
            command_type == CommandType.SET_PATH
            or command_type == CommandType.SET_TAXI_TRIP
        ):
            nodes = data.get("node_sequence", data.get("trip_nodes", []))
            success = self.taxi_manager.request_new_trip(nodes)
            if success:
                self._update_taxi_segment()
            return None

        # Let base class handle STOP, EMERGENCY_STOP, etc.
        return super().handle_event(command_type, data)

    # ------------------------------------------------------------------
    # YOLO helpers
    # ------------------------------------------------------------------
    def _identify_stop_reason(self, yolo_data: dict) -> str:
        """Identify which YOLO detection caused a full stop (gain ≈ 0)."""
        stop_sign = yolo_data.get("stop_sign", None)
        traffic = yolo_data.get("traffic_light", None)
        cars = yolo_data.get("cars", None)
        person = yolo_data.get("person", None)

        reasons = []

        # stop_sign[0] > 0 means at least one stop sign detected
        if stop_sign is not None and stop_sign[0] > 0:
            reasons.append("Stop sign")

        # traffic_light[0] > 0 means traffic light detected (red if gain → 0)
        if traffic is not None and traffic[0] > 0:
            reasons.append("Red traffic light")

        # car/person close enough to force full stop
        if cars is not None and cars[0] > 0:
            dist = yolo_data.get("car_dist", None)
            reasons.append(f"Car ahead ({dist:.2f}m)" if dist else "Car ahead")

        if person is not None and person[0] > 0:
            dist = yolo_data.get("person_dist", None)
            reasons.append(f"Pedestrian ({dist:.2f}m)" if dist else "Pedestrian")

        # Center-box obstacle (person or cone directly in path)
        if yolo_data.get("obstacle_in_path", False):
            obs_type = yolo_data.get("obstacle_type", 0.0)
            obs_dist = yolo_data.get("obstacle_dist", None)
            if obs_type == 1.0:
                reasons.append(
                    f"Person blocking path ({obs_dist:.2f}m)"
                    if obs_dist
                    else "Person blocking path"
                )
            elif obs_type == 2.0:
                reasons.append(
                    f"Cone/obstacle in path ({obs_dist:.2f}m)"
                    if obs_dist
                    else "Cone in path"
                )
            elif obs_type == 3.0:
                reasons.append(
                    f"Car obstacle — overtaking ({obs_dist:.2f}m)"
                    if obs_dist
                    else "Car obstacle — overtaking"
                )

        return " + ".join(reasons) if reasons else "Unknown YOLO trigger"
