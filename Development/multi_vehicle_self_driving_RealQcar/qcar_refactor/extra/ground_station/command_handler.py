"""Terminal command parsing and server routing for the ground station."""

from __future__ import annotations

import shlex
from dataclasses import dataclass
from typing import TYPE_CHECKING

from core.commands import CommandSource, CommandType, VehicleCommand

if TYPE_CHECKING:
    from .server import CommandDelivery, GroundStationServer


@dataclass(frozen=True)
class GroundStationCommandRequest:
    """One terminal action, optionally containing a typed vehicle command."""

    action: str
    vehicle_id: int | None = None
    command: VehicleCommand | None = None


class GroundStationCommandHandler:
    """Convert CLI text into a typed command and route it through the server.

    This class owns terminal syntax and target selection only. It does not
    access a vehicle runtime, state machine, fleet manager, or actuator.
    """

    _SIMPLE_COMMANDS = {
        "start": CommandType.START,
        "stop": CommandType.STOP,
        "emergency-stop": CommandType.EMERGENCY_STOP,
        "reset": CommandType.RESET,
        "build-fleet": CommandType.BUILD_FLEET,
        "cancel-fleet": CommandType.CANCEL_FLEET,
        "enable-manual": CommandType.ENABLE_MANUAL,
        "disable-manual": CommandType.DISABLE_MANUAL,
        "disable-sdcs-map": CommandType.DISABLE_SDCS_MAP,
    }

    def parse(self, text: str) -> GroundStationCommandRequest:
        """Parse one terminal line without contacting the server."""
        parts = shlex.split(text)
        if not parts:
            return GroundStationCommandRequest("")
        action = parts[0].lower()
        if action in {"help", "list", "quit", "exit"}:
            if len(parts) != 1:
                raise ValueError(f"{action} takes no arguments")
            return GroundStationCommandRequest(action)
        if action == "status":
            if len(parts) != 2:
                raise ValueError("status requires a vehicle ID")
            return GroundStationCommandRequest(action, vehicle_id=self._vehicle_id(parts[1]))

        vehicle_id = self._required_vehicle_id(parts)
        if action in self._SIMPLE_COMMANDS:
            if len(parts) != 2:
                raise ValueError(f"{action} takes only a vehicle ID")
            return GroundStationCommandRequest(
                action,
                vehicle_id,
                VehicleCommand(self._SIMPLE_COMMANDS[action], source=CommandSource.LOCAL),
            )
        if action == "set-velocity":
            if len(parts) != 3:
                raise ValueError("set-velocity requires a vehicle ID and velocity in m/s")
            try:
                velocity = float(parts[2])
            except ValueError as exc:
                raise ValueError("velocity must be numeric") from exc
            return GroundStationCommandRequest(
                action,
                vehicle_id,
                VehicleCommand(CommandType.SET_VELOCITY, {"velocity": velocity}, source=CommandSource.LOCAL),
            )
        if action == "set-path":
            if len(parts) != 3:
                raise ValueError("set-path requires a vehicle ID and CSV path")
            return GroundStationCommandRequest(
                action,
                vehicle_id,
                VehicleCommand(CommandType.SET_PATH, {"path": parts[2]}, source=CommandSource.LOCAL),
            )
        if action == "enable-sdcs-map":
            if len(parts) < 5:
                raise ValueError("enable-sdcs-map requires a vehicle ID, loop, and at least two node IDs")
            loop = self._loop(parts[2])
            try:
                nodes = [int(value) for value in parts[3:]]
            except ValueError as exc:
                raise ValueError("SDCS map node IDs must be integers") from exc
            return GroundStationCommandRequest(
                action,
                vehicle_id,
                VehicleCommand(
                    CommandType.ENABLE_SDCS_MAP,
                    {"nodes": nodes, "loop": loop},
                    source=CommandSource.LOCAL,
                ),
            )
        if action == "manual":
            if len(parts) != 4:
                raise ValueError("manual requires a vehicle ID, throttle, and steering in radians")
            try:
                throttle = float(parts[2])
                steering = float(parts[3])
            except ValueError as exc:
                raise ValueError("manual throttle and steering must be numeric") from exc
            return GroundStationCommandRequest(
                action,
                vehicle_id,
                VehicleCommand(
                    CommandType.MANUAL_INPUT,
                    {"throttle": throttle, "steering": steering},
                    source=CommandSource.LOCAL,
                ),
            )
        if action == "manual-drive":
            if len(parts) != 2:
                raise ValueError("manual-drive requires a vehicle ID")
            return GroundStationCommandRequest(action, vehicle_id=vehicle_id)
        raise ValueError(f"unsupported command: {action}")

    def route(self, server: "GroundStationServer", request: GroundStationCommandRequest) -> "CommandDelivery":
        """Send a parsed vehicle command through the registered TCP session."""
        if request.vehicle_id is None or request.command is None:
            raise ValueError("request does not contain a vehicle command")
        return server.send_command(request.vehicle_id, request.command)

    @staticmethod
    def _required_vehicle_id(parts: list[str]) -> int:
        if len(parts) < 2:
            raise ValueError("a vehicle ID is required")
        return GroundStationCommandHandler._vehicle_id(parts[1])

    @staticmethod
    def _vehicle_id(value: str) -> int:
        try:
            vehicle_id = int(value)
        except ValueError as exc:
            raise ValueError("vehicle ID must be an integer") from exc
        if vehicle_id < 0:
            raise ValueError("vehicle ID must be non-negative")
        return vehicle_id

    @staticmethod
    def _loop(value: str) -> int | str:
        if value == "inf":
            return value
        try:
            loop = int(value)
        except ValueError as exc:
            raise ValueError("loop must be 0, 1, 2, or inf") from exc
        if loop not in (0, 1, 2):
            raise ValueError("loop must be 0, 1, 2, or inf")
        return loop
