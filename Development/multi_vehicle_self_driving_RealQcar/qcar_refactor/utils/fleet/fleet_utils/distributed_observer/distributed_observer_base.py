"""Base contract for advisory distributed fleet observers."""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from collections.abc import Mapping, Sequence

from core.types import VehicleStateEstimate

from ...fleet_types import DistributedFleetEstimate, FleetPeerSnapshot


class DistributedObserverBase(ABC):
    """Build a local estimate of fleet state without owning V2V or control.

    The caller supplies already-validated peer snapshots. Implementations are
    advisory: a fleet controller must not use their output until the specific
    algorithm has independent safety and performance validation.
    """

    def __init__(self, config: Mapping[str, object], vehicle_id: int, logger=None) -> None:
        self._config = dict(config)
        self._vehicle_id = int(vehicle_id)
        self._logger = logger or logging.getLogger(self.__class__.__name__)
        self._latest: DistributedFleetEstimate | None = None
        self._started = False

    def start(self) -> None:
        """Start the local observer lifecycle; repeated calls are safe."""
        self._started = True

    @abstractmethod
    def update(
        self,
        *,
        local_estimate: VehicleStateEstimate,
        peer_snapshots: Sequence[FleetPeerSnapshot],
        membership_revision: int,
        measurements: Mapping[int, VehicleStateEstimate] | None = None,
        dt: float = 0.0,
    ) -> DistributedFleetEstimate:
        """Return one immutable fleet-state estimate from local input sources."""

    def get_latest(self) -> DistributedFleetEstimate | None:
        """Return the latest result without updating the observer."""
        return self._latest

    def stop(self) -> None:
        """Stop the observer lifecycle; repeated calls are safe."""
        self._started = False
