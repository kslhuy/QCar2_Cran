"""Minimal, state-machine-gated runtime for one vehicle actor."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any

from core.types import ControlCommand, GuiCommand, ControllerReference, SensorData, VehicleStateEstimate
from core.vehicle_config import ConfigVehicle
from core.vehicle_state_machine import State, StateMachine


@dataclass(frozen=True)
class RuntimeTelemetry:
    """One completed runtime iteration."""

    state: State
    dt: float
    sensor_data: SensorData
    estimate: VehicleStateEstimate
    target: ControllerReference
    command: ControlCommand


class VehicleRuntime:
    """Own the control loop and safety lifecycle for one vehicle actor."""

    def __init__(
        self,
        config: ConfigVehicle,
        io,
        observer,
        planner,
        controller,
        v2v,
        simulation=None,
        fleet=None,
        logger=None,
    ):
        self.config = config
        self.io = io
        self.observer = observer
        self.planner = planner
        self.controller = controller
        self.v2v = v2v
        self.simulation = simulation
        self._fleet = fleet
        self.state_machine = StateMachine()
        self._logger = logger or logging.getLogger(self.__class__.__name__)
        self._started = False
        self._last_step_time: float | None = None

    @property
    def fleet(self):
        """Return the constructor-injected fleet interface, when configured."""
        return self._fleet

    def start(self) -> None:
        """Start dependencies and leave the actor in READY on success."""
        if self._started:
            return
        started = []
        try:
            if self.simulation is not None:
                self.simulation.start()
                started.append(self.simulation)
            self.observer.start()
            started.append(self.observer)
            self.planner.reset()
            self.controller.reset()
            self.v2v.start()
            started.append(self.v2v)
        except Exception as exc:
            self._rollback_startup(started)
            self.state_machine.mark_error(f"startup failed: {exc}")
            self._write_zero("startup_failure")
            raise RuntimeError("Vehicle runtime startup failed") from exc

        self._started = True
        self._last_step_time = None
        self.state_machine.mark_ready()

    def handle_command(self, command: GuiCommand) -> State:
        """Apply a supported external command and enforce immediate safe stop."""
        command_name = command.command.upper()
        if self.fleet is not None:
            fleet_command = self.fleet.handle_command(
                command_name,
                vehicle_running=self.state_machine.should_drive(),
                now_monotonic=time.monotonic(),
            )
            if fleet_command.handled:
                if fleet_command.stop_vehicle:
                    self.state_machine.handle_command(GuiCommand("STOP", command.payload))
                    self._write_zero(fleet_command.reason)
                    self._complete_fleet_cancellation()
                return self.state_machine.state

        previous_state = self.state_machine.state
        self.state_machine.handle_command(command)
        current_state = self.state_machine.state

        if current_state == State.RUNNING and previous_state != State.RUNNING:
            self.planner.reset()
            self.controller.reset()
        if current_state != State.RUNNING:
            self._cancel_fleet(f"state_{current_state.name.lower()}")
            self._write_zero(f"state_{current_state.name.lower()}")
            self._complete_fleet_cancellation()
        return current_state

    def step(self, dt: float | None = None) -> RuntimeTelemetry:
        """Run one ordered control-loop iteration.

        The caller supplies ``dt`` in deterministic tests. Production callers
        omit it and use the monotonic clock.
        """
        if not self._started:
            raise RuntimeError("Vehicle runtime has not started")
        resolved_dt = self._resolve_dt(dt)

        try:
            if self.simulation is not None:
                self.simulation.tick()
            self.io.read_to_cache()
            sensor_data = self.io.read()
            estimate = self.observer.update(sensor_data, resolved_dt, self.io.get_last_command())
            fleet_step = self._run_fleet_exchange(estimate, resolved_dt)
            target = (
                fleet_step.target
                if fleet_step is not None and fleet_step.target is not None
                else self.planner.update(estimate)
            )

            if self.state_machine.should_drive() and target.is_finished:
                self.handle_command(GuiCommand("STOP", {"reason": "path_finished"}))

            if self.state_machine.should_drive():
                command = self._compute_command(estimate, target, resolved_dt, fleet_step)
                self.io.write(command)
            else:
                command = self._write_zero(f"state_{self.state_machine.state.name.lower()}")

            return RuntimeTelemetry(self.state_machine.state, resolved_dt, sensor_data, estimate, target, command)
        except Exception as exc:
            self._fault_fleet(f"control loop failed: {exc}")
            self.state_machine.mark_error(f"control loop failed: {exc}")
            self._write_zero("runtime_error")
            self._complete_fleet_cancellation()
            raise RuntimeError("Vehicle runtime control loop failed") from exc

    def shutdown(self) -> None:
        """Write zero, then release modules in reverse dependency order."""
        self._cancel_fleet("shutdown")
        self._write_zero("shutdown")
        self._complete_fleet_cancellation()
        for module in (self.v2v, self.observer):
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

    def _write_zero(self, source: str) -> ControlCommand:
        command = ControlCommand(0.0, 0.0, 0.0, source)
        try:
            self.io.write(command)
        except RuntimeError:
            # A closed adapter cannot accept a command during repeated shutdown.
            pass
        return command

    def _cancel_fleet(self, reason: str) -> None:
        if self.fleet is not None:
            self.fleet.request_cancel(reason)

    def _fault_fleet(self, reason: str) -> None:
        if self.fleet is not None:
            self.fleet.fault(reason)

    def _run_fleet_exchange(self, estimate: VehicleStateEstimate, dt: float):
        """Run the fleet decision after the local observer has updated."""
        if self.fleet is None:
            return None
        now = time.monotonic()
        result = self.fleet.step(estimate, self.v2v.drain_received(), now, dt=dt)
        if result.fault_reason is not None:
            self._fault_fleet(result.fault_reason)
            self.state_machine.handle_command(GuiCommand("STOP", {"reason": result.fault_reason}))
            self._write_zero("fleet_fault")
            self._complete_fleet_cancellation()
        if result.publication is not None:
            self.v2v.publish(
                result.publication.message_type,
                result.publication.payload,
                list(result.publication.target_vehicle_ids),
            )
        return result

    def _compute_command(
        self,
        estimate: VehicleStateEstimate,
        target: ControllerReference,
        dt: float,
        fleet_step=None,
    ) -> ControlCommand:
        if fleet_step is not None and self.fleet is not None:
            if fleet_step.hold_command:
                return ControlCommand(0.0, 0.0, 0.0, "fleet_building_hold")
            if fleet_step.target is not None:
                if not self.controller.supports_fleet_reference:
                    reason = "active fleet follower requires a fleet controller"
                    self._fault_fleet(reason)
                    self.state_machine.handle_command(GuiCommand("STOP", {"reason": reason}))
                    self._write_zero("fleet_fault")
                    self._complete_fleet_cancellation()
                    return ControlCommand(0.0, 0.0, 0.0, "fleet_fault")
                return self.controller.compute(estimate, target, dt)
        return self.controller.compute(estimate, target, dt)

    def _complete_fleet_cancellation(self) -> None:
        if self.fleet is not None:
            self.fleet.complete_cancellation()

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
