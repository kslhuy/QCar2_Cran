"""Shared lifecycle for local simulation and physical-platform vehicle processes.

Platform code owns external resources (a CARLA session, PAL QCar, or future
ROS 2 node).  It supplies them as ``VehicleProcessSpec.resources``; the core
runtime continues to own vehicle safety, the control loop, and all IO calls.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from contextlib import nullcontext
from dataclasses import dataclass, replace
from enum import Enum
from pathlib import Path
import time
from typing import Any, Callable, ContextManager, Mapping, Protocol

from core.vehicle_process import VehicleProcessSpec, build_vehicle_process_runtime, run_vehicle_process


class PlatformSetup(Protocol):
    """Minimum scenario interface required by a platform process manager."""

    @property
    def vehicles(self) -> tuple[Any, ...]:
        """Return the scenario-owned vehicle definitions."""


class StartPolicy(str, Enum):
    """Who may move a runtime out of the state-machine READY state."""

    SIMULATOR_BARRIER = "simulator_barrier"
    OPERATOR_COMMAND = "operator_command"

    @property
    def auto_start(self) -> bool:
        """Whether the shared runner sends its simulator-only START command."""

        return self is StartPolicy.SIMULATOR_BARRIER


@dataclass(frozen=True)
class PlatformProcessContext:
    """Resolved launch inputs for one independently owned vehicle process."""

    setup: Any
    vehicle: Any
    spec: VehicleProcessSpec


# ``contextlib.AbstractContextManager`` was not subscriptable until after the
# Python 3.8 runtime installed on current QCar images.  The typing equivalent
# is generic across the supported deployment versions.
ResourceContext = ContextManager[Mapping[str, Any]]
RuntimeBuilder = Callable[[VehicleProcessSpec], Any]


def run_platform_process(
    spec: VehicleProcessSpec,
    *,
    cycles: int | None,
    dt: float | Callable[[object], float | None] | None = None,
    start_policy: StartPolicy = StartPolicy.SIMULATOR_BARRIER,
    resource_context: ResourceContext | None = None,
    runtime_builder: RuntimeBuilder = build_vehicle_process_runtime,
    on_ready: Callable[[object], None] | None = None,
    on_running: Callable[[object], None] | None = None,
    on_step: Callable[[object], None] | None = None,
    collect_telemetry: bool = True,
    should_stop: Callable[[], bool] | None = None,
    pace: bool = False,
    sleep: Callable[[float], None] = time.sleep,
    monotonic: Callable[[], float] = time.monotonic,
) -> list[object]:
    """Run one actor while a platform resource context remains its sole owner."""

    context = resource_context or nullcontext({})
    with context as platform_resources:
        if not isinstance(platform_resources, Mapping):
            raise TypeError("Platform resource context must yield a mapping")
        resources = dict(spec.resources or {})
        overlap = set(resources).intersection(platform_resources)
        if overlap:
            names = ", ".join(sorted(overlap))
            raise ValueError(f"Platform resources conflict with VehicleProcessSpec.resources: {names}")
        resources.update(platform_resources)
        runtime = runtime_builder(replace(spec, resources=resources or None))
        resolved_dt = dt(runtime) if callable(dt) else dt
        return run_vehicle_process(
            runtime,
            cycles=cycles,
            dt=resolved_dt,
            on_ready=on_ready,
            on_running=on_running,
            on_step=on_step,
            collect_telemetry=collect_telemetry,
            should_stop=should_stop,
            auto_start=start_policy.auto_start,
            pace=pace,
            sleep=sleep,
            monotonic=monotonic,
        )


class BasePlatformProcessManager(ABC):
    """Shared lifecycle contract for one virtual, simulated, or physical vehicle."""

    platform_name = "vehicle"
    start_policy = StartPolicy.SIMULATOR_BARRIER
    pace = False

    @abstractmethod
    def prepare(self) -> PlatformProcessContext:
        """Resolve this platform's one vehicle and its platform-neutral spec."""

    def resource_context(self, context: PlatformProcessContext) -> ResourceContext:
        """Open resources owned by this platform; resource-free platforms use none."""

        del context
        return nullcontext({})

    @staticmethod
    def build_runtime(context: PlatformProcessContext):
        """Compatibility helper for callers that only need a resource-free runtime."""

        return build_vehicle_process_runtime(context.spec)

    def run(
        self,
        cycles: int | None,
        dt: float | Callable[[object, PlatformProcessContext], float | None] | None = None,
        on_ready: Callable[[object, PlatformProcessContext], None] | None = None,
        on_running: Callable[[object, PlatformProcessContext], None] | None = None,
        on_step: Callable[[object, object, PlatformProcessContext], None] | None = None,
        collect_telemetry: bool = True,
        should_stop: Callable[[], bool] | None = None,
        pace: bool | None = None,
        runtime_builder: RuntimeBuilder = build_vehicle_process_runtime,
        sleep: Callable[[float], None] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> tuple[PlatformProcessContext, list[object]]:
        """Run one process, keeping platform resources alive through shutdown."""

        context = self.prepare()
        def ready_callback(runtime) -> None:
            if on_ready is not None:
                on_ready(runtime, context)

        def running_callback(runtime) -> None:
            if on_running is not None:
                on_running(runtime, context)

        def step_callback(telemetry) -> None:
            if on_step is not None:
                # The runtime is available through the closure once built below.
                on_step(runtime_holder["runtime"], telemetry, context)

        runtime_holder: dict[str, object] = {}

        def build_runtime(spec: VehicleProcessSpec):
            runtime = runtime_builder(spec)
            runtime_holder["runtime"] = runtime
            return runtime

        telemetry = run_platform_process(
            context.spec,
            cycles=cycles,
            dt=(lambda runtime: dt(runtime, context)) if callable(dt) else dt,
            start_policy=self.start_policy,
            resource_context=self.resource_context(context),
            runtime_builder=build_runtime,
            on_ready=ready_callback if on_ready is not None else None,
            on_running=running_callback if on_running is not None else None,
            on_step=step_callback if on_step is not None else None,
            collect_telemetry=collect_telemetry,
            should_stop=should_stop,
            pace=self.pace if pace is None else pace,
            sleep=sleep,
            monotonic=monotonic,
        )
        return context, telemetry

    @staticmethod
    def wait_for_start_signal(path: Path, timeout_s: float = 30.0) -> None:
        """Wait for a local launcher to release a simulator worker barrier."""

        deadline = time.monotonic() + timeout_s
        while not path.exists():
            if time.monotonic() >= deadline:
                raise TimeoutError("Timed out waiting for multi-vehicle start signal")
            time.sleep(0.02)

    @staticmethod
    def serialize_telemetry(telemetry, fleet=None) -> dict:
        """Return the common JSON-safe telemetry row used by platform workers."""

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


class ScenarioPlatformProcessManager(BasePlatformProcessManager):
    """Resolve one process from a setup source with one or more vehicle entries."""

    def __init__(self, setup_file: str | Path, vehicle_id: int | None) -> None:
        self._setup_file = str(setup_file)
        self._vehicle_id = None if vehicle_id is None else int(vehicle_id)

    @abstractmethod
    def load_setup(self, setup_file: str) -> PlatformSetup:
        """Load one scenario without opening its external resources."""

    @abstractmethod
    def build_process_spec(self, setup: Any, vehicle: Any) -> VehicleProcessSpec:
        """Convert a selected scenario entry into one platform-neutral spec."""

    def prepare(self) -> PlatformProcessContext:
        """Resolve the selected scenario vehicle and process spec."""

        setup = self.load_setup(self._setup_file)
        vehicles = tuple(setup.vehicles)
        if self._vehicle_id is None:
            if len(vehicles) != 1:
                raise ValueError(f"A vehicle ID is required for the {self.platform_name} setup")
            vehicle = vehicles[0]
        else:
            vehicle = next((item for item in vehicles if item.vehicle_id == self._vehicle_id), None)
        if vehicle is None:
            raise ValueError(f"Vehicle {self._vehicle_id} is not in the {self.platform_name} setup")
        return PlatformProcessContext(setup=setup, vehicle=vehicle, spec=self.build_process_spec(setup, vehicle))
