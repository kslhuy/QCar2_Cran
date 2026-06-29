# utils/control_utils/observer/base_observer.py

"""
Base Observer — defines the interface every observer must implement.
Concrete implementations live in their own files (kalman_observer.py, etc.).
"""

import logging
import time
from abc import ABC, abstractmethod
from typing import Optional

from core.Types import VehicleStateEstimate, ControlCommand


class BaseVehicleObserver(ABC):
    """
    Abstract interface for all vehicle observers.

    Every observer implementation must provide:
        start()   — initialize state
        update()  — one estimation step, returns VehicleStateEstimate
        get_latest() — return last estimate without recomputing
        stop()    — cleanup
        
    """
    __slots__ = ('_logger',)
    def __init__(self, logger=None) -> None:
        self._logger = logger or logging.getLogger(self.__class__.__name__)

    @abstractmethod
    def start(self, initial_pose: Optional[list] = None) -> None:
        """Initialize the estimator with an optional starting pose [x, y, theta]."""
        ...

    @abstractmethod
    def update(
        self,
        sensor_data: dict,
        dt: float,
        last_command: Optional[ControlCommand] = None,
    ) -> VehicleStateEstimate:
        """
        Run one estimation step.

        Args:
            sensor_data: dict from vehicle_io.read() with keys:
                motor_tach, gyro_z, accelerometer, gps_valid, gps_position, sensor_timestamp
            dt: time delta since last update (seconds)
            last_command: most recent ControlCommand written to vehicle (optional)

        Returns:
            VehicleStateEstimate with current [x, y, theta, velocity, acceleration]
        """
        ...

    @abstractmethod
    def get_latest(self) -> VehicleStateEstimate:
        """
        Return the last computed state estimate without recomputing.
        Safe to call at any time — returns zero state if not started.
        """
        ...

    @abstractmethod
    def stop(self) -> None:
        """Release resources, stop threads, close files."""
        ...


class NullObserver(BaseVehicleObserver):
    """
    No-op observer for testing and offline simulation.
    Always returns zero state or given position with safe defaults.
    """

    __slots__ = ('_logger', '_last')

    def __init__(self, logger=None) -> None:
        super().__init__(logger)
        self._last = VehicleStateEstimate(
            timestamp=0.0, x=0.0, y=0.0, theta=0.0,
            velocity=0.0, acceleration=0.0, gps_valid=False,
        )

    def start(self, initial_pose: Optional[list] = None) -> None:
        now = time.time()
        self._last.timestamp = now
        self._last.x = initial_pose[0] if initial_pose else 0.0
        self._last.y = initial_pose[1] if initial_pose else 0.0
        self._last.theta = initial_pose[2] if initial_pose else 0.0
        self._logger.info("NullObserver started")

    def update(
        self,
        sensor_data: dict,
        dt: float,
        last_command: Optional[ControlCommand] = None,
    ) -> VehicleStateEstimate:
        now = sensor_data.get("sensor_timestamp", time.time())
        self._last = VehicleStateEstimate(
            timestamp=now,
            x=0.0, y=0.0, theta=0.0,
            velocity=0.0, acceleration=0.0,
            gps_valid=sensor_data.get("gps_valid", False),
        )
        return self._last

    def get_latest(self) -> VehicleStateEstimate:
        return self._last

    def stop(self) -> None:
        self._logger.info("NullObserver stopped")