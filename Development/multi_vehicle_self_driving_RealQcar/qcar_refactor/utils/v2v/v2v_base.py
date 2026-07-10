import logging
from abc import ABC, abstractmethod

from core.types import V2VState, VehicleStateEstimate


class V2VBase(ABC):
    """Minimal V2V interface used by the refactored runtime."""

    @abstractmethod
    def start(self) -> None:
        ...

    @abstractmethod
    def process_received_messages(self) -> None:
        ...

    @abstractmethod
    def broadcast_local_state(self, state: VehicleStateEstimate) -> bool:
        ...

    @abstractmethod
    def get_peer_states(self) -> dict[int, V2VState]:
        ...

    @abstractmethod
    def get_status(self) -> dict:
        ...

    @abstractmethod
    def stop(self) -> None:
        ...


class V2VNull(V2VBase):
    """Safe no-op V2V implementation for tests and single-vehicle runs."""

    def __init__(self, vehicle_id: int = 0, logger: logging.Logger | None = None) -> None:
        self._vehicle_id = vehicle_id
        self._logger = logger or logging.getLogger(self.__class__.__name__)

    def start(self) -> None:
        return None

    def process_received_messages(self) -> None:
        return None

    def broadcast_local_state(self, state: VehicleStateEstimate) -> bool:
        return False

    def get_peer_states(self) -> dict[int, V2VState]:
        return {}

    def get_status(self) -> dict:
        return {"enabled": False, "active": False, "peer_count": 0}

    def stop(self) -> None:
        return None
