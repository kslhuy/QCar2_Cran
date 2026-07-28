"""CARLA ego-actor adapter; session lifecycle remains external."""

from __future__ import annotations

import math
import numpy as np

from core.vehicle_types import ControlInput
from .io_base import IOBase


def _wrap_radians(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


class IOCarla(IOBase):
    """Read one session-owned ego actor and apply normalized CARLA controls."""

    def __init__(self, config: dict, session, vehicle_id: int = 0, logger=None, control_factory=None):
        super().__init__(config, vehicle_id, logger)
        self._session = session
        self._control_factory = control_factory
        self._steering_scale = float(self._config.get("steering_scale", 1.0))
        self._steering_sign = float(self._config.get("steering_sign", 1.0))

    def read_to_cache(self) -> None:
        """Refresh exactly once after the runtime has completed a CARLA tick."""
        with self._cache_lock:
            self._poll_sensors()
            self._poll_gps()

    def _poll_sensors(self) -> None:
        actor = self._session.ego_actor
        snapshot = self._session.get_snapshot()
        transform = actor.get_transform()
        velocity = actor.get_velocity()
        forward = transform.get_forward_vector()
        signed_speed = float(velocity.x * forward.x + velocity.y * forward.y + velocity.z * forward.z)
        self._sensor_data_cache.motor_tach = signed_speed
        self._sensor_data_cache.gyro_z = float(snapshot.gyro_z)
        self._sensor_data_cache.accelerometer = np.asarray(snapshot.accelerometer, dtype=float)
        self._sensor_data_cache.sensor_timestamp = float(snapshot.timestamp)

    def _poll_gps(self) -> None:
        transform = self._session.ego_actor.get_transform()
        location = transform.location
        self._sensor_data_cache.gps_valid = True
        self._sensor_data_cache.gps_position = np.array(
            [float(location.x), -float(location.y), _wrap_radians(-math.radians(float(transform.rotation.yaw)))],
            dtype=float,
        )
        self._sensor_data_cache.gps_timestamp = float(self._session.get_snapshot().timestamp)

    def _hardware_write(self, throttle: float, steering: float) -> None:
        try:
            actor = self._session.ego_actor
        except RuntimeError:
            # Startup may fail before the externally owned session spawns an ego.
            return
        normalized_steer = self._clip(steering * self._steering_sign * self._steering_scale, -1.0, 1.0)
        values = {
            "throttle": max(0.0, float(throttle)),
            "steer": normalized_steer,
            "brake": max(0.0, -float(throttle)),
        }
        control = self._control_factory(**values) if self._control_factory else self._session.make_vehicle_control(**values)
        actor.apply_control(control)
