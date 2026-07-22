"""Platform-launch base class for one independently run vehicle process."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
import time
from typing import Any, Callable, Protocol

from core.vehicle_process import VehicleProcessSpec, build_vehicle_process_runtime, run_vehicle_process


class SimulatorSetup(Protocol):
    """Minimum scenario interface required by a platform process manager."""

    @property
    def vehicles(self) -> tuple[Any, ...]:
        """Return the scenario-owned vehicle definitions."""
        ...


@dataclass(frozen=True)
class SimulatorProcessContext:
    """Resolved launch inputs for one platform-managed vehicle process."""

    setup: Any
    vehicle: Any
    spec: VehicleProcessSpec


class BaseSimulatorProcessManager(ABC):
    """Share launch orchestration while platform subclasses own their setup format.

    This belongs in ``extra`` because it coordinates executable platform
    launchers. The platform-neutral runtime lifecycle remains in
    ``core.vehicle_process``.
    """

    platform_name = "vehicle"

    def __init__(self, setup_file: str | Path, vehicle_id: int) -> None:
        self._setup_file = str(setup_file)
        self._vehicle_id = int(vehicle_id)

    @abstractmethod
    def load_setup(self, setup_file: str) -> SimulatorSetup:
        """Load one platform scenario without starting a runtime."""

    @abstractmethod
    def build_process_spec(self, setup: Any, vehicle: Any) -> VehicleProcessSpec:
        """Convert one platform vehicle entry into the common process spec."""

    def prepare(self) -> SimulatorProcessContext:
        """Resolve the selected vehicle and its platform-neutral process spec."""
        setup = self.load_setup(self._setup_file)
        vehicle = next((item for item in setup.vehicles if item.vehicle_id == self._vehicle_id), None)
        if vehicle is None:
            raise ValueError(f"Vehicle {self._vehicle_id} is not in the {self.platform_name} setup")
        return SimulatorProcessContext(setup=setup, vehicle=vehicle, spec=self.build_process_spec(setup, vehicle))

    @staticmethod
    def build_runtime(context: SimulatorProcessContext):
        """Build one runtime with its optional process-local fleet interface."""
        fleet_setup = getattr(context.setup, "fleet", None)
        fleet = None
        if fleet_setup is not None:
            from utils.fleet import FleetManager

            fleet = FleetManager(fleet_setup.registry, context.vehicle.vehicle_id)
        return build_vehicle_process_runtime(context.spec, fleet=fleet)

    def run(
        self,
        cycles: int | None,
        dt: float | Callable[[object, SimulatorProcessContext], float | None],
        on_ready: Callable[[object, SimulatorProcessContext], None] | None = None,
        on_running: Callable[[object, SimulatorProcessContext], None] | None = None,
        on_step: Callable[[object, object, SimulatorProcessContext], None] | None = None,
        collect_telemetry: bool = True,
        should_stop: Callable[[], bool] | None = None,
    ) -> tuple[SimulatorProcessContext, list[object]]:
        """Build and run one process using platform callbacks only at hooks."""
        context = self.prepare()
        runtime = self.build_runtime(context)
        resolved_dt = dt(runtime, context) if callable(dt) else dt

        def ready_callback(active_runtime) -> None:
            if on_ready is not None:
                on_ready(active_runtime, context)

        def step_callback(telemetry) -> None:
            if on_step is not None:
                on_step(runtime, telemetry, context)

        def running_callback(active_runtime) -> None:
            if on_running is not None:
                on_running(active_runtime, context)

        telemetry = run_vehicle_process(
            runtime,
            cycles=cycles,
            dt=resolved_dt,
            on_ready=ready_callback if on_ready is not None else None,
            on_running=running_callback if on_running is not None else None,
            on_step=step_callback if on_step is not None else None,
            collect_telemetry=collect_telemetry,
            should_stop=should_stop,
        )
        return context, telemetry

    @staticmethod
    def wait_for_start_signal(path: Path, timeout_s: float = 30.0) -> None:
        """Wait for a parent launcher to release independently started workers."""
        deadline = time.monotonic() + timeout_s
        while not path.exists():
            if time.monotonic() >= deadline:
                raise TimeoutError("Timed out waiting for multi-vehicle start signal")
            time.sleep(0.02)

    @staticmethod
    def serialize_telemetry(telemetry, fleet=None) -> dict:
        """Return the common JSON-safe row used by platform process runners."""
        sensor = telemetry.sensor_data
        estimate = telemetry.estimate
        row = {
            "time_s": float(sensor.sensor_timestamp),
            "sensor_time_s": float(sensor.sensor_timestamp),
            "control_dt_s": float(telemetry.dt),
            "gps_x_m": float(sensor.gps_position[0]),
            "gps_y_m": float(sensor.gps_position[1]),
            "gps_valid": bool(sensor.gps_valid),
            "estimate_x_m": float(estimate.x),
            "estimate_y_m": float(estimate.y),
            "estimate_valid": bool(estimate.valid),
            "speed_mps": float(sensor.motor_tach),
            "throttle": float(telemetry.command.throttle),
            "steering_rad": float(telemetry.command.steering),
            "command_source": telemetry.command.source,
        }
        if fleet is not None:
            status = fleet.status()
            row["fleet_phase"] = status.phase.value
            row["fleet_role"] = status.role.value
            row["fleet_membership_revision"] = status.membership_revision
            row["fleet_reason"] = status.reason
            row["predecessor_age_s"] = fleet.predecessor_age_s(time.monotonic())
            row["predecessor_gap_m"] = fleet.predecessor_gap_m(estimate)
            row.update({f"v2v_{name}": value for name, value in fleet.counters().items()})
            distributed = fleet.distributed_estimate()
            row["distributed_estimate_count"] = 0 if distributed is None else len(distributed.estimates)
        return row
