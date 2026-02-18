"""
Manual Mode State - Direct Control from Ground Station

Allows the vehicle to be controlled directly from the Ground Station GUI
with direct throttle and steering commands. Supports keyboard, joystick,
or any other input method configured on the Ground Station side.
"""

import time
import numpy as np
from typing import Dict, Any, Tuple, Optional
from .state_base import StateBase
from .vehicle_state import VehicleState, StateTransitionReason, Gear
from pal.products.qcar import IS_PHYSICAL_QCAR

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


class ManualModeState(StateBase):
    """Handler for MANUAL_MODE state with direct control from Ground Station"""

    def enter(self) -> bool:
        """Initialize manual mode state"""
        super().enter()
        self.logger.logger.info("[MANUAL] Entering MANUAL_MODE state")

        # Initialize state data
        self.state_data = {
            "control_type": "unknown",  # 'keyboard', 'joystick', etc.
            "last_command_time": 0.0,
            "command_timeout": 1.0,  # Timeout in seconds (stop if no commands received)
            "current_throttle": 0.0,
            "current_steering": 0.0,
            "last_sent_throttle": None,  # Track last sent values to hardware
            "last_sent_steering": None,
            # 'last_hardware_update_time': 0.0,  # Track time of last hardware write
            "command_count": 0,
            "timeout_warnings": 0,
            "hardware_update_count": 0,
        }

        self.logger.logger.info(
            "[MANUAL] Manual mode activated - waiting for control commands"
        )
        self.logger.logger.info(
            "[MANUAL] Command timeout: {:.1f}s".format(
                self.state_data["command_timeout"]
            )
        )

        return True

    def update(
        self, dt: float, sensor_data: Dict[str, Any]
    ) -> Tuple[float, float, Optional[Tuple[VehicleState, StateTransitionReason]]]:
        """Handle manual mode - apply direct control commands from Ground Station"""

        # Get current control commands from state data
        throttle = self.state_data.get("current_throttle", 0.0)
        steering = self.state_data.get("current_steering", 0.0)

        # Check for command timeout (stop if no recent commands)
        current_time = time.time()
        last_cmd_time = self.state_data.get("last_command_time", 0.0)
        timeout = self.state_data.get("command_timeout", 1.0)

        if last_cmd_time > 0 and (current_time - last_cmd_time) > timeout:
            # No commands received recently - safety stop
            self.state_data["timeout_warnings"] += 1

            # Log timeout warning periodically
            if (
                self.state_data["timeout_warnings"] % 100 == 1
            ):  # First warning and every 100 cycles
                self.logger.log_warning(
                    f"[MANUAL] Command timeout - no commands for {current_time - last_cmd_time:.2f}s, stopping vehicle"
                )

            # Stop the vehicle but stay in manual mode
            throttle = 0.0
            steering = 0.0

        # Only send to hardware if commands changed (avoid blocking on every loop)
        # if self._commands_changed(throttle, steering):
        self._send_to_hardware(throttle, steering)

        # Periodic status logging
        self._periodic_status_logging()

        # Stay in manual mode (No need to send throttle/steering here, done in _send_to_hardware)
        return None, None, None

    def handle_event(
        self, command_type, data: Dict[str, Any] = None
    ) -> Optional[Tuple[VehicleState, StateTransitionReason]]:
        """
        Handle commands while in manual mode

        Args:
            command_type: CommandType enum
            data: Optional event data

        Returns:
            Optional state transition
        """
        data = data or {}

        # Check if CommandType import was successful
        if not COMMAND_TYPE_AVAILABLE:
            return super().handle_event(command_type, data)

        # Handle manual control commands (throttle/steering updates)
        if command_type == CommandType.MANUAL_CONTROL:
            throttle = data.get("throttle", 0.0)
            steering = data.get("steering", 0.0)

            if not IS_PHYSICAL_QCAR:
                throttle *= 0.7  # Scale down for simulation (to sensible speeds)

            # Apply Gear-based throttle limiting
            if hasattr(self.vehicle_logic, "gear"):
                current_gear = self.vehicle_logic.gear
                limit = 0.2  # Default full power

                if current_gear == Gear.DRIVE_1:
                    limit = 0.2  # Limit to 50% power in Gear 1
                elif current_gear == Gear.DRIVE_2:
                    limit = 0.4  # Limit to 75% power in Gear 2
                elif current_gear == Gear.DRIVE_3:
                    limit = 0.6  # Full power in Gear 3

                # # Apply limit to throttle magnitude
                # throttle = min(throttle, limit)

            # Validate and clamp control inputs
            throttle = max(-limit, min(limit, throttle))
            if self.vehicle_logic.vehicle_type == "Limo":
                steering *= 2.0  # Map -0.5 -> -1.0, 0.5 -> 1.0
            steering = max(-1.0, min(1.0, steering))

            # Update state data
            self.state_data["current_throttle"] = throttle
            self.state_data["current_steering"] = steering
            self.state_data["last_command_time"] = time.time()
            self.state_data["command_count"] += 1
            self.state_data["timeout_warnings"] = 0  # Reset timeout warnings

            # Log first command and periodically
            if self.state_data["command_count"] == 1:
                self.logger.logger.info(
                    f"[MANUAL] First control command received: throttle={throttle:.2f}, steering={steering:.2f}"
                )
            elif self.state_data["command_count"] % 500 == 0:
                self.logger.logger.info(
                    f"[MANUAL] Command #{self.state_data['command_count']}: throttle={throttle:.2f}, steering={steering:.2f}"
                )

            return None  # Stay in manual mode

        # Handle disable manual mode command
        elif command_type == CommandType.DISABLE_MANUAL_MODE:
            self.logger.logger.info(
                "[MANUAL] Manual mode disabled - transitioning to STOPPED"
            )
            return (VehicleState.STOPPED, StateTransitionReason.STOP_COMMAND)

        # Handle stop command (transition to STOPPED state)
        elif command_type in [CommandType.STOP, CommandType.EMERGENCY_STOP]:
            self.logger.logger.info(f"[MANUAL] Stop command received: {command_type}")
            return (VehicleState.STOPPED, StateTransitionReason.STOP_COMMAND)

        # Handle start command (transition to autonomous mode)
        elif command_type == CommandType.START:
            self.logger.logger.info(
                "[MANUAL] Start command - transitioning to autonomous mode"
            )
            return (VehicleState.WAITING_FOR_START, StateTransitionReason.START_COMMAND)

        # Use base class handler for other commands
        else:
            return super().handle_event(command_type, data)

    def _commands_changed(self, throttle: float, steering: float) -> bool:
        """Check if commands have changed since last hardware update"""
        last_throttle = self.state_data.get("last_sent_throttle")
        last_steering = self.state_data.get("last_sent_steering")

        # Always send first command
        if last_throttle is None or last_steering is None:
            return True

        # Check for significant change (0.01 threshold to avoid noise)
        throttle_changed = abs(throttle - last_throttle) > 0.01
        steering_changed = abs(steering - last_steering) > 0.01

        return throttle_changed or steering_changed

    def _send_to_hardware(self, throttle: float, steering: float):
        """Send commands to hardware with LED indicators"""
        try:
            if (
                not hasattr(self.vehicle_logic, "qcar")
                or self.vehicle_logic.qcar is None
            ):
                return

            # Set LED indicators
            LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])  # Default: rear lights on

            # Adjust LED indicators based on steering
            if steering > 0.1:
                # Left turn indicators
                LEDs[0] = 1
                LEDs[2] = 1
            elif steering < -0.1:
                # Right turn indicators
                LEDs[1] = 1
                LEDs[3] = 1

            # Reverse/brake lights
            if throttle < 0:
                LEDs[5] = 1

            # Send commands to QCar hardware
            self.vehicle_logic.qcar.read_write_std(
                throttle=throttle, steering=steering, LEDs=LEDs
            )

            # Update tracking
            self.state_data["last_sent_throttle"] = throttle
            self.state_data["last_sent_steering"] = steering
            self.state_data["hardware_update_count"] += 1

        except Exception as e:
            # Don't crash on hardware errors
            if self.state_data.get("hardware_update_count", 0) % 50 == 1:
                self.logger.log_error("[MANUAL] Hardware update error", e)

    def _update_led_indicators(self, throttle: float, steering: float):
        """Deprecated: Use _send_to_hardware instead"""
        pass

    def _periodic_status_logging(self):
        """Log manual mode status periodically"""
        if (
            hasattr(self.vehicle_logic, "loop_counter")
            and self.vehicle_logic.loop_counter % 1000 == 0
        ):  # Every ~5 seconds at 200Hz
            time_in_mode = self.get_time_in_state()
            cmd_count = self.state_data.get("command_count", 0)
            hw_count = self.state_data.get("hardware_update_count", 0)
            avg_cmd_rate = cmd_count / time_in_mode if time_in_mode > 0 else 0
            avg_hw_rate = hw_count / time_in_mode if time_in_mode > 0 else 0

            self.logger.logger.info(
                f"[MANUAL] Manual mode active for {time_in_mode:.1f}s - "
                f"Commands: {cmd_count} ({avg_cmd_rate:.1f} Hz), "
                f"Hardware updates: {hw_count} ({avg_hw_rate:.1f} Hz)"
            )

    def exit(self):
        """Called when exiting manual mode state"""
        cmd_count = self.state_data.get("command_count", 0)
        time_in_mode = self.get_time_in_state()

        self.logger.logger.info(
            f"[MANUAL] Exiting MANUAL_MODE state - "
            f"Duration: {time_in_mode:.1f}s, Commands: {cmd_count}"
        )

        # Ensure vehicle is stopped when leaving manual mode
        if hasattr(self.vehicle_logic, "qcar") and self.vehicle_logic.qcar is not None:
            try:
                self.vehicle_logic.qcar.write(throttle=0, steering=0)
            except:
                pass
