"""Construct process-local advisory distributed observers from fleet policy."""

from __future__ import annotations

from ...fleet_types import DistributedObserverConfig, FleetError
from .distributed_observer_base import DistributedObserverBase
from .distributed_observer_fake import DistributedObserverFake
from .distributed_observer_luenberger import DistributedObserverLuenberger


def build_distributed_observer(
    config: DistributedObserverConfig,
    vehicle_id: int,
    logger=None,
) -> DistributedObserverBase:
    """Build the selected observer without coupling it to scenario execution."""
    if config.implementation == "fake":
        return DistributedObserverFake(dict(config.parameters), vehicle_id, logger)
    if config.implementation == "luenberger":
        return DistributedObserverLuenberger(dict(config.parameters), vehicle_id, logger)
    raise FleetError(f"Unsupported distributed observer implementation: {config.implementation}")
