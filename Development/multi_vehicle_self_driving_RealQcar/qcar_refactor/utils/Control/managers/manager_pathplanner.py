"""Runtime delegation for path-planner implementations."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from typing import Any

from core.vehicle_types import ControllerReference, VehicleStateEstimate
from utils.control.path_planner.path_planner_base import PathPlannerBase

from .manager_base import ManagerBase


class PathPlannerManager(ManagerBase[PathPlannerBase]):
    """Expose one selected path planner through a stable runtime interface."""

    def __init__(
        self,
        configured_planner: PathPlannerBase,
        builders: Mapping[str, Callable[[], PathPlannerBase]] | None = None,
    ) -> None:
        super().__init__(configured_planner, builders)

    def load_path(self, path_source: Any) -> None:
        self._active.load_path(path_source)

    def reset(self) -> None:
        self._active.reset()

    def update(self, state: VehicleStateEstimate) -> ControllerReference:
        return self._active.update(state)

    def set_target_velocity(self, target_velocity: float) -> None:
        self._active.set_target_velocity(target_velocity)

    def is_finished(self) -> bool:
        return self._active.is_finished()

    def _validate_utility(self, utility: object, name: str) -> None:
        for method in ("load_path", "reset", "update", "set_target_velocity", "is_finished"):
            if not callable(getattr(utility, method, None)):
                raise TypeError(f"{name} must provide {method}()")
