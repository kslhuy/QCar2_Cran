"""Minimal, state-machine-gated runtime for one vehicle actor."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any

from core.types import ControlCommand, GuiCommand, PlannerTarget, SensorData, VehicleStateEstimate
from core.vehicle_config import ConfigVehicle
from core.vehicle_state_machine import State, StateMachine


@dataclass(frozen=True)
class RuntimeTelemetry:
    """One completed runtime iteration."""

    state: State
    dt: float
    sensor_data: SensorData
    estimate: VehicleStateEstimate
    target: PlannerTarget
    command: ControlCommand


class VehicleRuntime:
    """Own the control loop and safety lifecycle for one vehicle actor."""

    def __init__(self, config: ConfigVehicle, io, observer, planner, controller, v2v, simulation=None, logger=None):
        self.config = config
        self.io = io
        self.observer = observer
        self.planner = planner
        self.controller = controller
        self.v2v = v2v
        self.simulation = simulation
        self.state_machine = StateMachine()
        self._logger = logger or logging.getLogger(self.__class__.__name__)
        self._started = False
        self._last_step_time: float | None = None

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
        previous_state = self.state_machine.state
        self.state_machine.handle_command(command)
        current_state = self.state_machine.state

        if current_state == State.RUNNING and previous_state != State.RUNNING:
            self.planner.reset()
            self.controller.reset()
        if current_state != State.RUNNING:
            self._write_zero(f"state_{current_state.name.lower()}")
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
            self.v2v.process_received_messages()
            target = self.planner.update(estimate)

            if self.state_machine.should_drive() and target.is_finished:
                self.handle_command(GuiCommand("STOP", {"reason": "path_finished"}))

            if self.state_machine.should_drive():
                command = self.controller.compute(estimate, target, resolved_dt)
                self.io.write(command)
            else:
                command = self._write_zero(f"state_{self.state_machine.state.name.lower()}")

            self.v2v.broadcast_local_state(estimate)
            return RuntimeTelemetry(self.state_machine.state, resolved_dt, sensor_data, estimate, target, command)
        except Exception as exc:
            self.state_machine.mark_error(f"control loop failed: {exc}")
            self._write_zero("runtime_error")
            raise RuntimeError("Vehicle runtime control loop failed") from exc

    def shutdown(self) -> None:
        """Write zero, then release modules in reverse dependency order."""
        self._write_zero("shutdown")
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
