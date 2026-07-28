import logging
from abc import ABC, abstractmethod

from core.vehicle_types import V2VMessage


class V2VBase(ABC):
    """Minimal V2V interface used by the refactored runtime."""

    def __init__(self, config: dict, vehicle_id: int = 0, logger: logging.Logger | None = None) -> None:
        self._config = dict(config)
        self._vehicle_id = int(vehicle_id)
        self._logger = logger or logging.getLogger(self.__class__.__name__)

    @abstractmethod
    def start(self) -> None:
        ...

    @abstractmethod
    def publish(self, message_type: str, payload: dict, target_vehicle_ids: list[int] | None = None) -> bool:
        ...

    @abstractmethod
    def drain_received(self) -> list[V2VMessage]:
        ...

    @abstractmethod
    def get_status(self) -> dict:
        ...

    @abstractmethod
    def stop(self) -> None:
        ...


class V2VNull(V2VBase):
    """Safe no-op V2V implementation for tests and single-vehicle runs."""

    def __init__(self, config: dict, vehicle_id: int = 0, logger: logging.Logger | None = None) -> None:
        super().__init__(config=config, vehicle_id=vehicle_id, logger=logger)

    def start(self) -> None:
        return None

    def publish(self, message_type: str, payload: dict, target_vehicle_ids: list[int] | None = None) -> bool:
        return False

    def drain_received(self) -> list[V2VMessage]:
        return []

    def get_status(self) -> dict:
        return {"enabled": False, "active": False, "peer_count": 0}

    def stop(self) -> None:
        return None
