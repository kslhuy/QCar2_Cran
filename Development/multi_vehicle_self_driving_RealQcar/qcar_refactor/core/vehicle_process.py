"""Platform-neutral building blocks for one independently run vehicle process."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Callable, Mapping

from core.module_factory import build_vehicle_modules
from core.types import GuiCommand
from core.vehicle_config import ConfigError, load_config
from core.vehicle_logic import VehicleRuntime


@dataclass(frozen=True)
class VehicleProcessSpec:
    """Configuration inputs for one independently run vehicle process."""

    vehicle_id: int
    vehicle_config_file: str
    selection_overrides: Mapping[str, str] | None = None
    value_overrides: Mapping[str, Any] | None = None
    resources: Mapping[str, Any] | None = None


def build_vehicle_process_runtime(spec: VehicleProcessSpec, logger=None, fleet=None) -> VehicleRuntime:
    """Build one runtime without assuming an IO or simulation backend."""
    overrides = deepcopy(dict(spec.value_overrides or {}))
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
    return VehicleRuntime(
        config, modules.io, modules.observer, modules.planner,
        modules.controller, modules.v2v, simulation=modules.simulation, fleet=fleet, logger=logger,
    )


def run_vehicle_process(
    runtime: VehicleRuntime,
    cycles: int,
    dt: float | None = None,
    on_ready: Callable[[VehicleRuntime], None] | None = None,
    on_running: Callable[[VehicleRuntime], None] | None = None,
    on_step: Callable[[object], None] | None = None,
) -> list[object]:
    """Start, command, step, and safely shut down one runtime.

    Hooks let optional platform runners coordinate process barriers and pacing
    without adding platform behavior to the shared vehicle runtime.
    """
    if cycles <= 0:
        raise ValueError("cycles must be positive")
    telemetry = []
    try:
        runtime.start()
        if on_ready is not None:
            on_ready(runtime)
        runtime.handle_command(GuiCommand("START", {}))
        if on_running is not None:
            on_running(runtime)
        for _ in range(cycles):
            sample = runtime.step(dt=dt)
            telemetry.append(sample)
            if on_step is not None:
                on_step(sample)
    finally:
        runtime.shutdown()
    return telemetry
