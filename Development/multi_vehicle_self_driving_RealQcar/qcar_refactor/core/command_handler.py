"""Transport-independent command dispatch for one vehicle runtime."""

from __future__ import annotations

import time
from dataclasses import dataclass

from core.commands import CommandOutcome, CommandResult, CommandType, VehicleCommand
from core.vehicle_state_machine import State, StateMachine


@dataclass(frozen=True)
class CommandHandling:
    """Command result plus runtime-owned controller and safety actions."""

    result: CommandResult
    reset_control: bool = False
    require_safe_stop: bool = False
    safe_stop_reason: str = ""
    controller_profile: str | None = None
    manual_input: tuple[float, float] | None = None
    planner_profile: str | None = None
    sdcs_node_sequence: tuple[int, ...] | None = None
    sdcs_loop: int | str | None = None
    lidar_diagnostic_enabled: bool | None = None


class VehicleCommandHandler:
    """Interpret validated :class:`VehicleCommand` values for one vehicle."""

    def __init__(
        self,
        vehicle_id: int,
        state_machine: StateMachine,
        planner,
        fleet=None,
        *,
        manual_controller_available: bool = False,
    ) -> None:
        self._vehicle_id = int(vehicle_id)
        self._state_machine = state_machine
        self._planner = planner
        self._fleet = fleet
        self._manual_controller_available = bool(manual_controller_available)
        self._manual_enabled = False

    def reset(self) -> None:
        """Clear command-mode state when the vehicle runtime is restarted."""
        self._manual_enabled = False

    def handle(self, command: VehicleCommand, *, now_monotonic: float | None = None) -> CommandHandling:
        """Apply command semantics and return required runtime safety work."""
        if command.target_vehicle_id is not None and command.target_vehicle_id != self._vehicle_id:
            return self._result(
                command,
                CommandOutcome.REJECTED,
                "vehicle_id_mismatch",
                "Command targets a different vehicle",
            )

        manual_handling = self._handle_manual_command(command)
        if manual_handling is not None:
            return manual_handling

        lidar_handling = self._handle_lidar_diagnostic_command(command)
        if lidar_handling is not None:
            return lidar_handling

        if self._fleet is None and command.command_type in {
            CommandType.BUILD_FLEET,
            CommandType.CANCEL_FLEET,
        }:
            return self._result(
                command,
                CommandOutcome.REJECTED,
                "fleet_manager_unavailable",
                "This vehicle was not launched from a fleet-enabled scenario",
            )

        fleet_handling = self._handle_fleet_command(command, now_monotonic)
        if fleet_handling is not None:
            return fleet_handling

        sdcs_handling = self._handle_sdcs_map_command(command)
        if sdcs_handling is not None:
            return sdcs_handling

        if command.command_type == CommandType.SET_VELOCITY:
            self._planner.set_target_velocity(float(command.payload["velocity"]))
            return self._result(command, CommandOutcome.APPLIED)
        if command.command_type == CommandType.SET_PATH:
            try:
                self._planner.load_path(command.payload["path"])
            except (OSError, ValueError) as exc:
                return self._result(command, CommandOutcome.REJECTED, "invalid_path", str(exc))
            return self._result(command, CommandOutcome.APPLIED)

        previous_state = self._state_machine.state
        applied = self._state_machine.handle_command(command)
        current_state = self._state_machine.state
        if not applied:
            return self._result(
                command,
                CommandOutcome.REJECTED,
                "invalid_state_transition",
                f"{command.command_type.value} is not valid while runtime is {current_state.name}",
            )

        if current_state == State.RUNNING:
            fleet_started = self._fleet is not None and self._fleet.start_for_vehicle(
                now_monotonic=time.monotonic() if now_monotonic is None else float(now_monotonic)
            )
            return CommandHandling(
                self._command_result(command, CommandOutcome.APPLIED),
                reset_control=previous_state != current_state,
                controller_profile=(
                    self._disable_manual()
                    if fleet_started and self._fleet.is_follower()
                    else None
                ),
            )

        reason = f"state_{current_state.name.lower()}"
        if self._fleet is not None:
            self._fleet.stop_for_vehicle(reason)
        return CommandHandling(
            self._command_result(command, CommandOutcome.APPLIED),
            require_safe_stop=True,
            safe_stop_reason=reason,
            controller_profile=self._disable_manual(),
        )

    def _handle_lidar_diagnostic_command(self, command: VehicleCommand) -> CommandHandling | None:
        if command.command_type not in {
            CommandType.ENABLE_LIDAR_DIAGNOSTIC,
            CommandType.DISABLE_LIDAR_DIAGNOSTIC,
        }:
            return None
        if self._state_machine.state not in (State.READY, State.RUNNING, State.STOPPED):
            return self._result(
                command,
                CommandOutcome.REJECTED,
                "lidar_diagnostic_requires_started_vehicle",
                "LiDAR diagnostics require a READY, RUNNING, or STOPPED vehicle runtime",
            )
        return CommandHandling(
            self._command_result(command, CommandOutcome.APPLIED),
            lidar_diagnostic_enabled=(command.command_type == CommandType.ENABLE_LIDAR_DIAGNOSTIC),
        )

    def _handle_sdcs_map_command(self, command: VehicleCommand) -> CommandHandling | None:
        if command.command_type not in {
            CommandType.ENABLE_SDCS_MAP,
            CommandType.DISABLE_SDCS_MAP,
        }:
            return None
        if self._fleet_blocks_sdcs_map():
            return self._result(
                command,
                CommandOutcome.REJECTED,
                "sdcs_map_rejected_for_fleet_follower",
                "SDCS map selection is owned by the fleet leader while fleet operation is active",
            )
        if command.command_type == CommandType.ENABLE_SDCS_MAP:
            if self._state_machine.state not in (State.READY, State.RUNNING):
                return self._result(
                    command,
                    CommandOutcome.REJECTED,
                    "sdcs_map_requires_ready_or_running_vehicle",
                    "Select an SDCS map route only while the vehicle runtime is READY or RUNNING",
                )
            has_profile = getattr(self._planner, "has_profile", None)
            if not callable(has_profile) or not has_profile("sdcs_map"):
                return self._result(
                    command,
                    CommandOutcome.REJECTED,
                    "sdcs_map_profile_unavailable",
                    "This vehicle has no configured SDCS map planner profile",
                )
            return CommandHandling(
                self._command_result(command, CommandOutcome.APPLIED),
                reset_control=True,
                require_safe_stop=self._state_machine.should_drive(),
                safe_stop_reason="sdcs_map_route_change",
                planner_profile="sdcs_map",
                sdcs_node_sequence=tuple(command.payload["nodes"]),
                sdcs_loop=command.payload["loop"],
            )

        if self._fleet_is_active():
            # The leader remains the fleet's motion source.  Restore its
            # configured route without stopping followers; the runtime writes
            # one zero command before calculating the next reference.
            return CommandHandling(
                self._command_result(command, CommandOutcome.APPLIED),
                reset_control=True,
                require_safe_stop=True,
                safe_stop_reason="sdcs_map_disabled",
                controller_profile=self._disable_manual(),
                planner_profile="configured",
            )

        # Outside a fleet, disabling a map route is a safety event: restore
        # the configured planner, return to STOPPED, then require START.
        self._state_machine.handle_command(
            VehicleCommand(CommandType.STOP, {"reason": "sdcs_map_disabled"}, source=command.source)
        )
        return CommandHandling(
            self._command_result(command, CommandOutcome.APPLIED),
            reset_control=True,
            require_safe_stop=True,
            safe_stop_reason="sdcs_map_disabled",
            controller_profile=self._disable_manual(),
            planner_profile="configured",
        )

    def _handle_manual_command(self, command: VehicleCommand) -> CommandHandling | None:
        if command.command_type not in {
            CommandType.ENABLE_MANUAL,
            CommandType.DISABLE_MANUAL,
            CommandType.MANUAL_INPUT,
        }:
            return None
        if not self._manual_controller_available:
            return self._result(
                command,
                CommandOutcome.REJECTED,
                "manual_controller_unavailable",
                "This vehicle has no configured manual controller",
            )
        if command.command_type == CommandType.ENABLE_MANUAL:
            if self._state_machine.state not in (State.READY, State.RUNNING):
                return self._result(
                    command,
                    CommandOutcome.REJECTED,
                    "manual_requires_ready_or_running_vehicle",
                    "Manual control requires the vehicle runtime to be READY or RUNNING",
                )
            if self._fleet_blocks_manual():
                return self._result(
                    command,
                    CommandOutcome.REJECTED,
                    "manual_rejected_while_fleet_active",
                    "Manual control is unavailable while fleet operation is active",
                )
            self._manual_enabled = True
            return CommandHandling(
                self._command_result(command, CommandOutcome.APPLIED),
                controller_profile="manual",
            )
        if command.command_type == CommandType.DISABLE_MANUAL:
            return CommandHandling(
                self._command_result(command, CommandOutcome.APPLIED),
                controller_profile=self._disable_manual(),
            )
        if not self._manual_enabled:
            return self._result(
                command,
                CommandOutcome.REJECTED,
                "manual_not_active",
                "Enable manual control before sending manual input",
            )
        if not self._state_machine.should_drive():
            return self._result(
                command,
                CommandOutcome.REJECTED,
                "manual_input_requires_running_vehicle",
                "Manual input requires the vehicle runtime to be RUNNING",
            )
        return CommandHandling(
            self._command_result(command, CommandOutcome.APPLIED),
            manual_input=(
                float(command.payload["throttle"]),
                float(command.payload["steering"]),
            ),
        )

    def _disable_manual(self) -> str | None:
        if not self._manual_enabled:
            return None
        self._manual_enabled = False
        return "configured"

    def _fleet_blocks_manual(self) -> bool:
        """Followers retain fleet control; leaders may use their manual profile."""
        if self._fleet is None:
            return False
        phase = getattr(self._fleet, "phase", None)
        value = getattr(phase, "value", phase)
        return value in ("building", "active") and self._fleet.is_follower()

    def _fleet_is_active(self) -> bool:
        if self._fleet is None:
            return False
        phase = getattr(self._fleet, "phase", None)
        value = getattr(phase, "value", phase)
        return value in ("building", "active")

    def _fleet_blocks_sdcs_map(self) -> bool:
        return self._fleet_is_active() and self._fleet.is_follower()

    def _handle_fleet_command(
        self,
        command: VehicleCommand,
        now_monotonic: float | None,
    ) -> CommandHandling | None:
        if self._fleet is None:
            return None
        fleet_command = self._fleet.handle_command(
            command,
            vehicle_running=self._state_machine.should_drive(),
            now_monotonic=time.monotonic() if now_monotonic is None else float(now_monotonic),
        )
        if not fleet_command.handled:
            return None
        if not fleet_command.accepted:
            return self._result(
                command,
                CommandOutcome.REJECTED,
                fleet_command.reason or "fleet_command_rejected",
                "Fleet manager rejected the lifecycle command",
            )
        if fleet_command.stop_vehicle:
            safe_stop_reason = fleet_command.reason or "fleet_cancel"
            self._state_machine.handle_command(
                VehicleCommand(
                    CommandType.STOP,
                    {"reason": safe_stop_reason},
                    source=command.source,
                )
            )
            return CommandHandling(
                self._command_result(command, CommandOutcome.APPLIED),
                require_safe_stop=True,
                safe_stop_reason=safe_stop_reason,
                controller_profile=self._disable_manual(),
            )
        return CommandHandling(
            self._command_result(command, CommandOutcome.APPLIED),
            controller_profile=(
                self._disable_manual()
                if self._state_machine.should_drive() and self._fleet.is_follower()
                else None
            ),
        )

    def _result(
        self,
        command: VehicleCommand,
        outcome: CommandOutcome,
        reason_code: str = "",
        reason: str = "",
    ) -> CommandHandling:
        return CommandHandling(self._command_result(command, outcome, reason_code, reason))

    def _command_result(
        self,
        command: VehicleCommand,
        outcome: CommandOutcome,
        reason_code: str = "",
        reason: str = "",
    ) -> CommandResult:
        return CommandResult(
            command_id=command.command_id,
            vehicle_id=self._vehicle_id,
            outcome=outcome,
            runtime_state=self._state_machine.state.name,
            reason_code=reason_code,
            reason=reason,
        )
