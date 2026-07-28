"""Minimal, state-machine-gated runtime for one vehicle actor."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any

from core.command_handler import VehicleCommandHandler
from core.commands import CommandResult, CommandSource, CommandType, VehicleCommand
from core.vehicle_types import ControlInput, ControllerReference, SensorData, VehicleStateEstimate
from core.vehicle_config import ConfigVehicle
from core.vehicle_state_machine import State, StateMachine
from utils.control.managers import ControllerCapabilityError, ControllerManager
from utils.ground_station.runtime_facade import GroundStationRuntimeFacade


@dataclass(frozen=True)
class RuntimeTelemetry:
    """One completed runtime iteration."""

    state: State
    dt: float
    sensor_data: SensorData
    estimate: VehicleStateEstimate
    target: ControllerReference
    command: ControlInput


class VehicleRuntime:
    """Own the control loop and safety lifecycle for one vehicle actor."""

    def __init__(
        self,
        config: ConfigVehicle,
        io,
        observer,
        planner,
        controller_manager,
        v2v,
        ground_station: GroundStationRuntimeFacade,
        simulation=None,
        fleet=None,
        logger=None,
    ):
        self.config = config
        self.io = io
        self.observer = observer
        self.planner = planner
        self.controller_manager = controller_manager
        self.v2v = v2v
        self.simulation = simulation
        self._fleet = fleet
        self._ground_station = ground_station
        self.state_machine = StateMachine()
        self._command_handler = VehicleCommandHandler(
            config.vehicle_id,
            self.state_machine,
            planner,
            fleet,
            manual_controller_available=self.controller_manager.has_profile("manual"),
        )
        self._logger = logger or logging.getLogger(self.__class__.__name__)
        self._started = False
        self._last_step_time: float | None = None

    @property
    def fleet(self):
        """Return the constructor-injected fleet interface, when configured."""
        return self._fleet

    @property
    def ground_station(self):
        """Return the runtime-facing ground-station facade."""
        return self._ground_station

    def start(self) -> None:
        """Start dependencies and leave the actor in READY on success."""
        if self._started:
            return
        started = []
        try:
            self._command_handler.reset()
            if self.simulation is not None:
                self.simulation.start()
                started.append(self.simulation)
            self.observer.start()
            started.append(self.observer)
            self.planner.reset()
            self.controller_manager.restore_configured()
            self.v2v.start()
            started.append(self.v2v)
            self.ground_station.start()
            started.append(self.ground_station)
        except Exception as exc:
            self._rollback_startup(started)
            self.state_machine.mark_error(f"startup failed: {exc}")
            self._write_zero("startup_failure")
            raise RuntimeError("Vehicle runtime startup failed") from exc

        self._started = True
        self._last_step_time = None
        self.state_machine.mark_ready()

    def handle_command(self, command: VehicleCommand) -> CommandResult:
        """Apply one validated command through the safety-owned runtime path."""
        handling = self._command_handler.handle(command, now_monotonic=time.monotonic())
        if handling.controller_profile is not None:
            self.controller_manager.select(handling.controller_profile)
        if handling.manual_input is not None:
            self.controller_manager.set_input(*handling.manual_input)
        if handling.reset_control:
            self.planner.reset()
            self.controller_manager.reset()
        if handling.require_safe_stop:
            self._write_zero(handling.safe_stop_reason)
        return self.ground_station.record_command_result(command, handling.result)

    def step(self, dt: float | None = None) -> RuntimeTelemetry:
        """Run one ordered control-loop iteration.

        The caller supplies ``dt`` in deterministic tests. Production callers
        omit it and use the monotonic clock.
        """
        if not self._started:
            raise RuntimeError("Vehicle runtime has not started")
        resolved_dt = self._resolve_dt(dt)

        try:
            self.ground_station.process_pending(self.handle_command)
            if self.simulation is not None:
                self.simulation.tick()
            self.io.read_to_cache()
            sensor_data = self.io.read()
            estimate = self.observer.update(sensor_data, resolved_dt, self.io.get_last_command())
            fleet_step = self.fleet.run_cycle(estimate, dt=resolved_dt) if self.fleet is not None else None
            if fleet_step is not None and fleet_step.fault_reason is not None:
                self.handle_command(
                    VehicleCommand(CommandType.STOP, {"reason": fleet_step.fault_reason}, source=CommandSource.RUNTIME)
                )
            if fleet_step is not None and fleet_step.controller_profile is not None:
                if not self.controller_manager.is_selected(fleet_step.controller_profile):
                    self.controller_manager.select(fleet_step.controller_profile)
            target = (
                fleet_step.target
                if fleet_step is not None and fleet_step.target is not None
                else self.planner.update(estimate)
            )

            if self.state_machine.should_drive() and self.controller_manager.uses_planner_completion and target.is_finished:
                self.handle_command(
                    VehicleCommand(CommandType.STOP, {"reason": "path_finished"}, source=CommandSource.RUNTIME)
                )

            if self.state_machine.should_drive():
                command = self._compute_control_input(estimate, target, resolved_dt, fleet_step)
                self.io.write(command)
            else:
                command = self._write_zero(f"state_{self.state_machine.state.name.lower()}")

            telemetry = RuntimeTelemetry(self.state_machine.state, resolved_dt, sensor_data, estimate, target, command)
            manual_mode = self.controller_manager.is_selected("manual")
            self.ground_station.publish_monitoring(
                vehicle_id=self.config.vehicle_id,
                runtime_state=telemetry.state.name,
                estimate=telemetry.estimate,
                control_reference=telemetry.target,
                fleet=self.fleet,
                v2v=self.v2v,
                control_mode="manual" if manual_mode else "auto",
                manual_input_age_s=(
                    self.controller_manager.input_age_s() if manual_mode else None
                ),
            )
            return telemetry
        except Exception as exc:
            if self.fleet is not None:
                self.fleet.abort(f"control loop failed: {exc}")
            self.state_machine.mark_error(f"control loop failed: {exc}")
            self._write_zero("runtime_error")
            raise RuntimeError("Vehicle runtime control loop failed") from exc

    def shutdown(self) -> None:
        """Write zero, then release modules in reverse dependency order."""
        if self.fleet is not None:
            self.fleet.stop_for_vehicle("shutdown")
        self._command_handler.reset()
        self.controller_manager.restore_configured()
        self._write_zero("shutdown")
        for module in (self.ground_station, self.v2v, self.observer):
            try:
                module.stop()
            except Exception as exc:
                self._logger.warning("Failed to stop %s: %s", type(module).__name__, exc)
        try:
            self.io.close()
        except Exception as exc:
            self._logger.warning("Failed to close IO: %s", exc)
        if self.simulation is not None:
            try:
                self.simulation.close()
            except Exception as exc:
                self._logger.warning("Failed to close simulation: %s", exc)
        if self.fleet is not None:
            try:
                self.fleet.shutdown()
            except Exception as exc:
                self._logger.warning("Failed to close fleet manager: %s", exc)
        self._started = False

    def _resolve_dt(self, dt: float | None) -> float:
        if dt is None:
            now = time.monotonic()
            if self._last_step_time is None:
                dt = 1.0 / float(self.config.runtime["loop_rate_hz"])
            else:
                dt = now - self._last_step_time
                if dt <= 0.0:
                    dt = 1.0 / float(self.config.runtime["loop_rate_hz"])
            self._last_step_time = now
        if not isinstance(dt, (int, float)) or isinstance(dt, bool) or dt <= 0.0:
            raise ValueError("Control-loop dt must be a positive number")
        return float(dt)

    def _write_zero(self, source: str) -> ControlInput:
        command = ControlInput(0.0, 0.0, 0.0, source)
        try:
            self.io.write(command)
        except RuntimeError:
            # A closed adapter cannot accept a command during repeated shutdown.
            pass
        return command

    def _compute_control_input(
        self,
        estimate: VehicleStateEstimate,
        target: ControllerReference,
        dt: float,
        fleet_step=None,
    ) -> ControlInput:
        if fleet_step is not None and self.fleet is not None:
            if fleet_step.hold_command:
                return ControlInput(0.0, 0.0, 0.0, "fleet_building_hold")
            if fleet_step.target is not None:
                try:
                    return self.controller_manager.compute_fleet(estimate, target, dt)
                except ControllerCapabilityError as exc:
                    reason = str(exc)
                    self.fleet.abort(reason)
                    self.handle_command(
                        VehicleCommand(CommandType.STOP, {"reason": reason}, source=CommandSource.RUNTIME)
                    )
                    return ControlInput(0.0, 0.0, 0.0, "fleet_fault")
        return self.controller_manager.compute(estimate, target, dt)

    def _rollback_startup(self, started: list[Any]) -> None:
        for module in reversed(started):
            try:
                close = getattr(module, "close", None)
                if callable(close):
                    close()
                else:
                    module.stop()
            except Exception as exc:
                self._logger.warning("Startup rollback failed for %s: %s", type(module).__name__, exc)
        try:
            self.io.close()
        except Exception as exc:
            self._logger.warning("Startup rollback failed for IO: %s", exc)
