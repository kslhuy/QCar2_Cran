"""Platform-neutral building blocks for one independently run vehicle process."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import time
from typing import TYPE_CHECKING, Any, Callable, Mapping

from core.module_factory import build_vehicle_modules
from core.commands import CommandSource, CommandType, VehicleCommand
from core.vehicle_config import ConfigError, load_config
from core.vehicle_logic import VehicleRuntime

if TYPE_CHECKING:
    from utils.fleet import FleetRuntimeSpec


@dataclass(frozen=True)
class VehicleProcessSpec:
    """Configuration inputs for one independently run vehicle process.

    ``vehicle_id=None`` preserves the ID declared by the selected vehicle
    configuration. Scenario workers still provide an explicit ID.
    """

    vehicle_id: int | None
    vehicle_config_file: str
    selection_overrides: Mapping[str, str] | None = None
    value_overrides: Mapping[str, Any] | None = None
    resources: Mapping[str, Any] | None = None
    fleet_spec: FleetRuntimeSpec | None = None


def build_vehicle_process_runtime(spec: VehicleProcessSpec, logger=None) -> VehicleRuntime:
    """Build one runtime without assuming an IO or simulation backend."""
    overrides = deepcopy(dict(spec.value_overrides or {}))
    if spec.vehicle_id is not None:
        configured_id = overrides.get("vehicle_id")
        if configured_id is not None and configured_id != spec.vehicle_id:
            raise ConfigError("VehicleProcessSpec.vehicle_id conflicts with value_overrides['vehicle_id']")
        overrides["vehicle_id"] = spec.vehicle_id
    config = load_config(
        vehicle_config_file=spec.vehicle_config_file,
        selection_overrides=spec.selection_overrides,
        value_overrides=overrides,
    )
    modules = build_vehicle_modules(config, logger=logger, resources=dict(spec.resources or {}))
    fleet = None
    if spec.fleet_spec is not None:
        from utils.fleet import build_fleet_manager

        fleet = build_fleet_manager(spec.fleet_spec, config.vehicle_id, logger)
        fleet.attach_transport(modules.v2v)
    return VehicleRuntime(
        config, modules.io, modules.observer, modules.planner,
        modules.controller_manager, modules.v2v, simulation=modules.simulation, fleet=fleet,
        ground_station=modules.ground_station, logger=logger,
    )


def run_vehicle_process(
    runtime: VehicleRuntime,
    cycles: int | None,
    dt: float | None = None,
    on_ready: Callable[[VehicleRuntime], None] | None = None,
    on_running: Callable[[VehicleRuntime], None] | None = None,
    on_step: Callable[[object], None] | None = None,
    collect_telemetry: bool = True,
    should_stop: Callable[[], bool] | None = None,
    auto_start: bool = True,
    pace: bool = False,
    sleep: Callable[[float], None] = time.sleep,
    monotonic: Callable[[], float] = time.monotonic,
) -> list[object]:
    """Start, optionally command, step, and safely shut down one runtime.

    Hooks let optional platform runners coordinate process barriers and pacing
    without adding platform behavior to the shared vehicle runtime.  Simulator
    workers use the historical ``auto_start=True`` behaviour.  Physical
    workers set it false, leaving the runtime in ``READY`` until its configured
    ground-station path delivers a valid ``START`` command.
    """
    if cycles is not None and cycles <= 0:
        raise ValueError("cycles must be positive")
    telemetry = []
    try:
        runtime.start()
        if on_ready is not None:
            on_ready(runtime)
        if auto_start:
            runtime.handle_command(VehicleCommand(CommandType.START, source=CommandSource.SIMULATOR))
            if on_running is not None:
                on_running(runtime)
        completed_cycles = 0
        while cycles is None or completed_cycles < cycles:
            if should_stop is not None and should_stop():
                break
            started_at = monotonic() if pace else None
            sample = runtime.step(dt=dt)
            completed_cycles += 1
            if collect_telemetry:
                telemetry.append(sample)
            if on_step is not None:
                on_step(sample)
            if pace and started_at is not None:
                period_s = 1.0 / float(runtime.config.runtime["loop_rate_hz"])
                sleep(max(0.0, period_s - (monotonic() - started_at)))
    finally:
        runtime.shutdown()
    return telemetry
