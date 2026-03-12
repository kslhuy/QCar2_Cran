"""
Waiting for Start State - Event-Driven Implementation

Handles the ready state where vehicle waits for start command.
Uses single handle_event method with direct transitions.
Much simpler than the previous approach!
"""

from typing import Dict, Any, Tuple, Optional
from .state_base import StateBase
from .vehicle_state import VehicleState, StateTransitionReason

# Import CommandType once at module level
import sys
import os

# Add parent directory to sys.path for imports
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


class WaitingForStartState(StateBase):
    """Handler for WAITING_FOR_START state with direct event transitions"""

    def enter(self) -> bool:
        """Initialize waiting for start state"""
        super().enter()
        self.logger.logger.info("Entering WAITING_FOR_START state")

        # Initialize state data
        self.state_data = {
            "ready_for_commands": True,
            "auto_start_delay": self.config.timing.start_delay
            if hasattr(self.config, "timing")
            else 1.0,
            "auto_start_enabled": False,  # Can be enabled for testing
        }

        self.logger.logger.info("Ready for start command")
        return True

    def update(
        self, dt: float, sensor_data: Dict[str, Any]
    ) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        """Handle waiting for start commands"""

        # No movement while waiting
        throttle, steering = 0.0, 0.0

        # Check for emergency stop conditions first (handled by base class)
        # emergency_transition = super().update(dt, sensor_data)
        # if emergency_transition[2]:
        #     return emergency_transition

        # Optional: Auto-start logic (disabled by default)
        if self.state_data["auto_start_enabled"]:
            wait_time = self.get_time_in_state()
            if wait_time > self.state_data["auto_start_delay"]:
                self.logger.logger.info(" Auto-start triggered")
                return (
                    throttle,
                    steering,
                    (VehicleState.FOLLOWING_PATH, StateTransitionReason.START_COMMAND),
                )

        # Stay in waiting state
        return throttle, steering, None

    def handle_event(
        self, command_type, data: Dict[str, Any] = None
    ) -> Optional[Tuple[VehicleState, StateTransitionReason]]:
        """
        Handle command types with direct transitions

        Args:
            command_type: CommandType enum (e.g., CommandType.START, CommandType.STOP)
            data: Optional event data

        Returns:
            Optional[Tuple[VehicleState, StateTransitionReason]]: Transition if needed
        """
        data = data or {}

        # Check if CommandType import was successful
        if not COMMAND_TYPE_AVAILABLE:
            # Fallback to base class if CommandType not available
            return super().handle_event(command_type, data)

        if not self.state_data["ready_for_commands"]:
            if self.logger:
                self.logger.logger.warning(
                    f"[!] Ignoring '{command_type}' command - state not ready"
                )
            return None

        # Handle start command - DIRECT TRANSITION!
        # if command_type == CommandType.ACTIVATE_V2V:

        # Handle calibrate command - GPS-only recalibration without full reinitialization
        if command_type == CommandType.CALIBRATE:
            self.logger.logger.info(
                "GPS Calibration command received - performing GPS-only recalibration"
            )
            if self._recalibrate_gps():
                self.logger.logger.info("GPS recalibration completed successfully")
            else:
                self.logger.log_error("GPS recalibration failed")
            # Stay in WAITING_FOR_START state - no transition needed
            return None

        # Handle active calibration mode - DIRECT TRANSITION to CALIBRATING!
        if command_type == CommandType.ENABLE_CALIBRATION_MODE:
            cal_type = data.get("calibration_type", "throttle_velocity")
            cal_params = data.get("params", {})
            self.logger.logger.info(
                f"[CAL] Calibration mode requested: {cal_type}"
            )
            # Store calibration request for CalibratingState.enter() to read
            self.vehicle_logic._calibration_request = {
                "calibration_type": cal_type,
                "params": cal_params,
            }
            return (VehicleState.CALIBRATING, StateTransitionReason.START_COMMAND)

        # Handle start command - DIRECT TRANSITION (Normal start only)!
        if command_type == CommandType.START:
            self.logger.logger.info(
                " Normal start command received - transitioning to FOLLOWING_PATH state!"
            )
            return (VehicleState.FOLLOWING_PATH, StateTransitionReason.START_COMMAND)

        # Handle start platoon command - DIRECT TRANSITION BASED ON FORMATION ROLE!
        elif command_type == CommandType.START_PLATOON:
            # Debug: Log received data
            self.logger.logger.info(f"[START_PLATOON] Received data: {data}")

            if not self.validate_event_data(data, ["leader_id"]):
                self.logger.logger.error(
                    "[START_PLATOON] Missing 'leader_id' in command data!"
                )
                return None

            # ✅ CRITICAL: Check if platoon setup was completed first
            if not (
                hasattr(self.vehicle_logic, "platoon_controller")
                and self.vehicle_logic.platoon_controller
                and self.vehicle_logic.platoon_controller.setup_complete
            ):
                self.logger.logger.warning(
                    "START_PLATOON rejected - SETUP_PLATOON_FORMATION has not been received yet! "
                    "Please send SETUP_PLATOON_FORMATION command first before starting platoon."
                )
                return None

            leader_id = data.get("leader_id")

            # ✅ Check formation to determine role and state transition
            is_leader = self.vehicle_logic.platoon_controller.is_leader
            my_position = getattr(
                self.vehicle_logic.platoon_controller, "my_position", None
            )

            self.logger.logger.info(
                f"[START_PLATOON] Start platoon command received (setup_complete=True)"
            )
            self.logger.logger.info(
                f"[START_PLATOON] My formation: is_leader={is_leader}, position={my_position}, target_leader={leader_id}"
            )

            if is_leader:
                # Leaders go to FOLLOWING_PATH state and follow their predefined path
                self.logger.logger.info(
                    "[START_PLATOON] I am LEADER - transitioning to FOLLOWING_PATH state (path following as leader)"
                )
                # Enable platoon mode for leader
                self.vehicle_logic.platoon_controller.enabled = True
                return (
                    VehicleState.FOLLOWING_PATH,
                    StateTransitionReason.START_COMMAND,
                )
            else:
                # Followers transition to FOLLOWING_LEADER state
                self.logger.logger.info(
                    f"[START_PLATOON] I am FOLLOWER-{my_position} - transitioning to FOLLOWING_LEADER state (following vehicle {leader_id})"
                )

                # Enable platoon controller for follower mode
                self.vehicle_logic.platoon_controller.enable_as_follower()
                self.vehicle_logic.platoon_controller.leader_car_id = leader_id
                self.vehicle_logic.platoon_controller.enabled = True

                # Direct transition to following leader state
                return (
                    VehicleState.FOLLOWING_LEADER,
                    StateTransitionReason.START_COMMAND,
                )

        # Handle velocity updates - store for when we start
        elif command_type == CommandType.SET_VELOCITY:
            v_ref = data.get("v_ref")
            if v_ref is not None and 0 <= v_ref <= 2.0:
                if hasattr(self.vehicle_logic, "v_ref"):
                    self.vehicle_logic.v_ref = v_ref
                    self.logger.logger.info(
                        f"[OK] Velocity preset to {v_ref} m/s for when movement starts"
                    )
                    return None
            self.logger.logger.warning(
                f"[WARN] Invalid velocity update while waiting: {v_ref}"
            )
            return None

        # Handle manual mode activation - DIRECT TRANSITION!
        elif command_type == CommandType.ENABLE_MANUAL_MODE:
            control_type = data.get("control_type", "unknown")
            self.logger.logger.info(
                f" Manual mode activated with control_type: {control_type}"
            )
            return (
                VehicleState.MANUAL_MODE,
                StateTransitionReason.MANUAL_MODE_ACTIVATED,
            )

        # Handle taxi mode activation - DIRECT TRANSITION!
        elif command_type == CommandType.ENABLE_TAXI_MODE:
            self.logger.logger.info(
                "Taxi mode enabled - transitioning to TAXI_MODE state"
            )
            return (VehicleState.TAXI_MODE, StateTransitionReason.TAXI_MODE_ACTIVATED)

        # Handle path updates - store for when we start
        elif command_type == CommandType.SET_PATH:
            node_sequence = data.get("node_sequence")
            self.logger.logger.info(
                f"Received SET_PATH command with node_sequence: {node_sequence}"
            )

            if node_sequence and isinstance(node_sequence, list):
                # Generate waypoints from node sequence using roadmap
                if (
                    hasattr(self.vehicle_logic, "roadmap")
                    and self.vehicle_logic.roadmap
                ):
                    try:
                        new_waypoints = self.vehicle_logic.roadmap.generate_path(
                            node_sequence
                        )
                        self.vehicle_logic.waypoint_sequence = new_waypoints

                        # Update steering controller if it exists (though it shouldn't in waiting state)
                        if (
                            hasattr(self.vehicle_logic, "steering_controller")
                            and self.vehicle_logic.steering_controller
                        ):
                            self.vehicle_logic.steering_controller.reset(new_waypoints)

                        self.logger.logger.info(
                            f"Path updated with {len(node_sequence)} nodes, generated {new_waypoints.shape[1]} waypoints"
                        )
                        return None
                    except Exception as e:
                        self.logger.log_error("Failed to generate path from nodes", e)
                else:
                    self.logger.logger.warning(
                        "No roadmap available for path generation"
                    )
            else:
                self.logger.logger.warning(f"Invalid path data: {node_sequence}")
            return None

        # Handle initial position updates - set the vehicle's starting position (with optional GPS recalibration)
        elif command_type == CommandType.SET_INITIAL_POSITION:
            x = data.get("x")
            y = data.get("y")
            theta = data.get("theta", 0.0)
            calibrate = data.get(
                "calibrate", False
            )  # Default to False for backward compatibility

            if x is not None and y is not None:
                try:
                    # Create initial pose array [x, y, theta]
                    import numpy as np

                    initial_pose = np.array([float(x), float(y), float(theta)])

                    if calibrate:
                        # Full GPS recalibration with new position
                        self.logger.logger.info(
                            f"Setting initial position WITH GPS recalibration: ({x:.2f}, {y:.2f}, theta={theta:.2f})"
                        )

                        if self._recalibrate_gps(initial_pose):
                            self.logger.logger.info(
                                "✓ GPS recalibrated and observer reset successfully"
                            )
                        else:
                            self.logger.log_error(
                                "GPS recalibration with initial position failed"
                            )
                            return None
                    else:
                        # Just reset observer without GPS recalibration
                        self.logger.logger.info(
                            f"Setting initial position WITHOUT GPS recalibration: ({x:.2f}, {y:.2f}, theta={theta:.2f})"
                        )

                        if (
                            hasattr(self.vehicle_logic, "vehicle_observer")
                            and self.vehicle_logic.vehicle_observer
                        ):
                            self.vehicle_logic.vehicle_observer.reset_observer(
                                initial_pose
                            )
                            self.logger.logger.info(
                                "✓ Observer reset successfully (no GPS recalibration)"
                            )

                            # For fake vehicles: Update mock hardware positions even without GPS recalibration
                            self._update_fake_vehicle_position(initial_pose)
                        else:
                            self.logger.logger.warning(
                                "Vehicle observer not available for position update"
                            )

                    return None

                except Exception as e:
                    self.logger.log_error(
                        f"Failed to set initial position ({x}, {y}, {theta})", e
                    )
                    return None
            else:
                self.logger.logger.warning(
                    f"Invalid initial position data: x={x}, y={y}"
                )
                return None

        # Let base class handle common events (stop, emergency_stop)
        return super().handle_event(command_type, data)

    def exit(self):
        """Clean up waiting state"""
        self.logger.logger.info("Exiting WAITING_FOR_START state")

        wait_time = self.get_time_in_state()
        self.logger.logger.info(f"Waited for {wait_time:.1f}s before starting")

        super().exit()
