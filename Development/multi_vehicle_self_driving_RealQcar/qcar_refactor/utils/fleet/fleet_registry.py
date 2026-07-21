"""Revisioned mutable fleet membership registry."""

from __future__ import annotations

from threading import RLock

from .fleet_base import FleetFormationBuilder
from .fleet_types import FleetError, FleetFormation, FleetMember


class FleetRegistry:
    """Own controlled membership updates and expose immutable formations."""

    def __init__(self, formation: FleetFormation) -> None:
        self._builder = FleetFormationBuilder()
        self._formation = formation
        self._lock = RLock()

    def snapshot(self) -> FleetFormation:
        with self._lock:
            return self._formation

    def join(self, member: FleetMember) -> FleetFormation:
        with self._lock:
            if any(existing.vehicle_id == member.vehicle_id for existing in self._formation.members):
                raise FleetError(f"Vehicle {member.vehicle_id} is already a fleet member")
            return self._replace((*self._formation.members, member))

    def leave(self, vehicle_id: int) -> FleetFormation:
        with self._lock:
            if not any(member.vehicle_id == vehicle_id for member in self._formation.members):
                raise FleetError(f"Vehicle {vehicle_id} is not a fleet member")
            return self._replace(tuple(member for member in self._formation.members if member.vehicle_id != vehicle_id))

    def update_member(self, member: FleetMember) -> FleetFormation:
        with self._lock:
            if not any(existing.vehicle_id == member.vehicle_id for existing in self._formation.members):
                raise FleetError(f"Vehicle {member.vehicle_id} is not a fleet member")
            return self._replace(
                tuple(
                    member if existing.vehicle_id == member.vehicle_id else existing
                    for existing in self._formation.members
                )
            )

    def _replace(self, members: tuple[FleetMember, ...]) -> FleetFormation:
        self._formation = self._builder.build(
            self._formation.formation_id,
            members,
            self._formation.policy,
            membership_revision=self._formation.membership_revision + 1,
        )
        return self._formation
