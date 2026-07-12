"""
Base path planner interface.

Path planners convert the current vehicle state into a planner target. They
should not know about vehicle hardware, observers, controllers, GUI, or V2V.
"""

import logging
from abc import ABC, abstractmethod

from core.types import PlannerTarget, VehicleStateEstimate


class PathPlannerBase(ABC):
    """Interface every path planner implementation should follow."""

    def __init__(self, config: dict, vehicle_id: int = 0, logger=None) -> None:
        self._config = dict(config)
        self._vehicle_id = int(vehicle_id)
        self._logger = logger or logging.getLogger(self.__class__.__name__)

    @abstractmethod
    def load_path(self, path_source) -> None:
        """Load waypoints from a file path or iterable of points."""
        ...

    @abstractmethod
    def reset(self) -> None:
        """Reset internal progress to the beginning of the path."""
        ...

    @abstractmethod
    def update(self, state: VehicleStateEstimate) -> PlannerTarget:
        """Return the current target for the controller."""
        ...

    @abstractmethod
    def set_target_velocity(self, target_velocity: float) -> None:
        """Set the desired path-following speed."""
        ...

    @abstractmethod
    def is_finished(self) -> bool:
        """Return True after the final waypoint has been reached."""
        ...


class PathPlannerNull(PathPlannerBase):
    """Safe no-op planner for tests and bring-up."""

    def __init__(self, config: dict, vehicle_id: int = 0, logger=None) -> None:
        super().__init__(config, vehicle_id, logger)
        self._target_velocity = float(self._config.get("target_velocity", 0.0))
        self._finished = True

    def load_path(self, path_source) -> None:
        self._finished = True

    def reset(self) -> None:
        self._finished = True

    def update(self, state: VehicleStateEstimate) -> PlannerTarget:
        return PlannerTarget(
            target_x=float(state.x),
            target_y=float(state.y),
            target_theta=float(state.theta),
            target_velocity=0.0,
            is_finished=True,
        )

    def set_target_velocity(self, target_velocity: float) -> None:
        self._target_velocity = float(target_velocity)

    def is_finished(self) -> bool:
        return self._finished
