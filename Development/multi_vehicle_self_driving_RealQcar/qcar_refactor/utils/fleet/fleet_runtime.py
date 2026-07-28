"""Build the process-local fleet runtime from platform-neutral fleet inputs."""

from __future__ import annotations

from dataclasses import dataclass

from .fleet_manager import FleetManager
from .fleet_registry import FleetRegistry
from .fleet_utils.distributed_observer import build_distributed_observer


@dataclass(frozen=True)
class FleetRuntimeSpec:
    """Fleet membership and policy selected for one vehicle process."""

    registry: FleetRegistry


def build_fleet_manager(spec: FleetRuntimeSpec, vehicle_id: int, logger=None) -> FleetManager:
    """Construct one local fleet manager and its configured observer."""
    observer = build_distributed_observer(
        spec.registry.snapshot().policy.distributed_observer,
        vehicle_id,
        logger,
    )
    return FleetManager(spec.registry, vehicle_id, distributed_observer=observer)
