"""
Base controller interface.

Controllers convert an estimated vehicle state plus a planner target into a
vehicle command. They should not read sensors, load paths, write hardware, or
know about GUI/V2V.
"""

import logging
from abc import ABC, abstractmethod

from core.types import ControlCommand, ControllerReference, VehicleStateEstimate


class ControllerBase(ABC):
    """Interface every controller implementation should follow."""

    def __init__(self, config: dict, vehicle_id: int = 0, logger=None) -> None:
        self._config = dict(config)
        self._vehicle_id = int(vehicle_id)
        self._logger = logger or logging.getLogger(self.__class__.__name__)

    @property
    def supports_fleet_reference(self) -> bool:
        """Whether this controller accepts a predecessor-derived reference."""
        return False

    @abstractmethod
    def reset(self) -> None:
        """Reset internal controller memory, such as PID integral error."""
        ...

    @abstractmethod
    def compute(
        self,
        state: VehicleStateEstimate,
        target: ControllerReference,
        dt: float,
    ) -> ControlCommand:
        """Compute one throttle/steering command."""
        ...


class ControllerNull(ControllerBase):
    """Safe no-op controller for tests and startup."""

    def __init__(self, config: dict, vehicle_id: int = 0, logger=None) -> None:
        super().__init__(config, vehicle_id, logger)

    def reset(self) -> None:
        pass

    def compute(
        self,
        state: VehicleStateEstimate,
        target: ControllerReference,
        dt: float,
    ) -> ControlCommand:
        return ControlCommand(
            throttle=0.0,
            steering=0.0,
            target_velocity=0.0,
            source="null_controller",
        )
