"""Convert validated predecessor data into a fleet controller reference."""

from __future__ import annotations

from core.vehicle_types import ControllerReference, VehicleStateEstimate

from .fleet_state_machine import FleetPhase
from .fleet_types import FleetPeerSnapshot, FleetStatus


class FleetFollowingPolicy:
    """Keep fleet eligibility and predecessor lookup outside controller algorithms."""

    def make_reference(
        self,
        ego_estimate: VehicleStateEstimate,
        predecessor: FleetPeerSnapshot | None,
        status: FleetStatus,
    ) -> ControllerReference | None:
        del ego_estimate
        if status.phase != FleetPhase.ACTIVE or predecessor is None or not predecessor.valid:
            return None
        front = predecessor.estimate
        return ControllerReference(front.x, front.y, front.theta, max(0.0, front.velocity), False)
