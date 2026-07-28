"""Pass-through distributed observer used while algorithm development is pending."""

from __future__ import annotations

from collections.abc import Mapping, Sequence

from core.vehicle_types import VehicleStateEstimate

from .distributed_observer_base import DistributedObserverBase
from ...fleet_types import (
    DistributedEstimateSource,
    DistributedFleetEstimate,
    DistributedVehicleEstimate,
    FleetPeerSnapshot,
)


class DistributedObserverFake(DistributedObserverBase):
    """Expose available local, measured, and V2V states without calculation.

    Direct measurements take precedence over local or remote estimates. The
    vehicle's own local observer is used next, followed by validated V2V peer
    snapshots. This selection rule is deterministic and intentionally contains
    no filtering, fusion, or prediction.
    """

    def update(
        self,
        *,
        local_estimate: VehicleStateEstimate,
        peer_snapshots: Sequence[FleetPeerSnapshot],
        membership_revision: int,
        measurements: Mapping[int, VehicleStateEstimate] | None = None,
        dt: float = 0.0,
    ) -> DistributedFleetEstimate:
        del dt
        if not self._started:
            raise RuntimeError("Distributed observer has not started")

        selected: dict[int, DistributedVehicleEstimate] = {}
        for vehicle_id, estimate in (measurements or {}).items():
            selected[int(vehicle_id)] = DistributedVehicleEstimate(
                int(vehicle_id), estimate, DistributedEstimateSource.MEASUREMENT
            )
        selected.setdefault(
            self._vehicle_id,
            DistributedVehicleEstimate(
                self._vehicle_id, local_estimate, DistributedEstimateSource.LOCAL_OBSERVER
            ),
        )
        for snapshot in peer_snapshots:
            selected.setdefault(
                snapshot.source_vehicle_id,
                DistributedVehicleEstimate(
                    snapshot.source_vehicle_id, snapshot.estimate, DistributedEstimateSource.V2V
                ),
            )

        self._latest = DistributedFleetEstimate(
            observer_vehicle_id=self._vehicle_id,
            membership_revision=int(membership_revision),
            estimates=tuple(selected[vehicle_id] for vehicle_id in sorted(selected)),
        )
        return self._latest
