"""Runtime delegation for observer implementations."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from core.vehicle_types import ControlInput, SensorData, VehicleStateEstimate
from utils.control.observer.observer_base import ObserverBase

from .manager_base import ManagerBase


class ObserverManager(ManagerBase[ObserverBase]):
    """Expose one selected observer through a stable runtime interface."""

    def __init__(
        self,
        configured_observer: ObserverBase,
        builders: Mapping[str, Callable[[], ObserverBase]] | None = None,
    ) -> None:
        super().__init__(configured_observer, builders)

    def start(self, initial_pose: list[float] | None = None) -> None:
        self._active.start(initial_pose)

    def update(
        self,
        sensor_data: SensorData,
        dt: float,
        last_command: ControlInput | None = None,
    ) -> VehicleStateEstimate:
        return self._active.update(sensor_data, dt, last_command)

    def get_latest(self) -> VehicleStateEstimate:
        return self._active.get_latest()

    def stop(self) -> None:
        self._active.stop()

    def _validate_utility(self, utility: object, name: str) -> None:
        for method in ("start", "update", "get_latest", "stop"):
            if not callable(getattr(utility, method, None)):
                raise TypeError(f"{name} must provide {method}()")
