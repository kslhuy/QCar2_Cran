"""Local fleet operating state machine."""

from __future__ import annotations

from enum import Enum

from .fleet_registry import FleetRegistry
from .fleet_types import FleetError, FleetStatus


class FleetPhase(str, Enum):
    DISABLED = "disabled"
    BUILDING = "building"
    ACTIVE = "active"
    CANCELLING = "cancelling"
    FAULT = "fault"


class FleetStateMachine:
    """Own one vehicle's fleet phase without replacing vehicle safety state."""

    def __init__(self, registry: FleetRegistry, vehicle_id: int) -> None:
        self._registry = registry
        self._vehicle_id = int(vehicle_id)
        self._registry.snapshot().member(self._vehicle_id)
        self._phase = FleetPhase.DISABLED
        self._reason = ""

    @property
    def phase(self) -> FleetPhase:
        return self._phase

    def status(self, peer_health: tuple[tuple[int, bool], ...] = ()) -> FleetStatus:
        formation = self._registry.snapshot()
        member = formation.member(self._vehicle_id)
        return FleetStatus(
            formation_id=formation.formation_id,
            membership_revision=formation.membership_revision,
            source_vehicle_id=member.vehicle_id,
            member_order=member.member_order,
            role=member.role,
            phase=self._phase,
            peer_health=peer_health,
            reason=self._reason,
        )

    def request_build(self, *, vehicle_running: bool) -> bool:
        if not vehicle_running:
            return False
        if self._phase == FleetPhase.DISABLED:
            self._phase = FleetPhase.BUILDING
            self._reason = ""
            return True
        return False

    def activate(self) -> bool:
        if self._phase != FleetPhase.BUILDING:
            return False
        self._phase = FleetPhase.ACTIVE
        return True

    def request_cancel(self, reason: str = "") -> bool:
        if self._phase == FleetPhase.DISABLED:
            return False
        if self._phase != FleetPhase.CANCELLING:
            self._phase = FleetPhase.CANCELLING
            self._reason = reason
        return True

    def fault(self, reason: str) -> bool:
        if not reason:
            raise FleetError("Fleet fault reason must be non-empty")
        if self._phase == FleetPhase.DISABLED:
            return False
        self._phase = FleetPhase.FAULT
        self._reason = reason
        self._phase = FleetPhase.CANCELLING
        return True

    def complete_cancellation(self) -> bool:
        if self._phase != FleetPhase.CANCELLING:
            return False
        self._phase = FleetPhase.DISABLED
        return True
