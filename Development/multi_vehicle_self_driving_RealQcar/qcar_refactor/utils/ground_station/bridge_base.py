"""Vehicle-side ground-station bridge interface."""

from __future__ import annotations

from abc import ABC, abstractmethod

from core.commands import CommandResult, VehicleCommand
from .monitoring import MonitoringSnapshot


class GroundStationBridgeBase(ABC):
    """One runtime-facing facade for command and monitoring transport."""

    @abstractmethod
    def start(self) -> None:
        """Start background transport work without blocking vehicle startup."""
        ...

    @abstractmethod
    def stop(self) -> None:
        """Stop background transport work; repeated calls are harmless."""
        ...

    @abstractmethod
    def drain_commands(self, limit: int) -> list[VehicleCommand]:
        """Return at most ``limit`` commands queued by the transport."""
        ...

    @abstractmethod
    def publish_snapshot(self, snapshot: MonitoringSnapshot) -> None:
        """Offer the latest monitoring state for asynchronous transport."""
        ...

    @abstractmethod
    def publish_ack(self, result: CommandResult) -> None:
        """Offer a command acknowledgement for asynchronous transport."""
        ...

    @abstractmethod
    def get_status(self) -> dict[str, object]:
        """Return local bridge connection status for diagnostics."""
        ...


class NullGroundStationBridge(GroundStationBridgeBase):
    """No-op bridge used when no operator connection is configured."""

    def __init__(self, config: dict | None = None, vehicle_id: int = 0, logger=None) -> None:
        self._vehicle_id = int(vehicle_id)

    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None

    def drain_commands(self, limit: int) -> list[VehicleCommand]:
        return []

    def publish_snapshot(self, snapshot: MonitoringSnapshot) -> None:
        return None

    def publish_ack(self, result: CommandResult) -> None:
        return None

    def get_status(self) -> dict[str, object]:
        return {"enabled": False, "connected": False, "registered": False}
