"""CARLA ego-actor adapter; session lifecycle remains external."""

from __future__ import annotations

import math
from typing import Any
import numpy as np

from core.vehicle_types import ControlInput, LaserScanSample
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
        sensors = self._config.get("sensors", {})
        lidar = sensors.get("lidar", {}) if isinstance(sensors, dict) else {}
        self._lidar_enabled = bool(lidar.get("enabled", True)) if isinstance(lidar, dict) else True

    def read_to_cache(self) -> None:
        """Refresh exactly once after the runtime has completed a CARLA tick."""
        with self._cache_lock:
            self._poll_sensors()
            self._poll_gps()
        self._read_lidar_to_local_buffer()

    def set_lidar_enabled(self, enabled: bool) -> bool:
        """Switch local conversion of the session-owned CARLA LiDAR actor."""

        if not isinstance(enabled, bool):
            raise ValueError("LiDAR enabled state must be boolean")
        if not callable(getattr(self._session, "drain_lidar_measurements", None)):
            return False
        self._lidar_enabled = enabled
        return True

    def lidar_status(self) -> dict:
        status = super().lidar_status()
        status.update(
            {
                "source": "carla_session",
                "available": callable(getattr(self._session, "drain_lidar_measurements", None)),
                "enabled": self._lidar_enabled,
            }
        )
        return status

    def _read_lidar_to_local_buffer(self) -> None:
        """Convert session-owned raw LiDAR callbacks in this local IO process."""

        if not self._lidar_enabled:
            return
        drain = getattr(self._session, "drain_lidar_measurements", None)
        if not callable(drain):
            return
        scan_config = _session_lidar_scan_config(self._session)
        for measurement in drain():
            try:
                self._publish_lidar_scan(carla_lidar_to_laser_scan(measurement, scan_config))
            except Exception as exc:
                self._logger.warning("Failed to convert CARLA LiDAR scan: %s", exc)

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


def carla_lidar_to_laser_scan(data: Any, scan_config: dict[str, Any] | None = None) -> LaserScanSample:
    """Project one local CARLA ray-cast payload into a planar ``LaserScanSample``.

    CARLA uses positive Y to the right; the common scan convention follows
    ROS with positive Y to the left.  This conversion belongs to ``IOCarla``
    because it is the local data-plane adapter, not CARLA session lifecycle.
    """

    config = dict(scan_config or {})
    bin_count = _positive_int(config.get("bin_count", 360), "bin_count")
    angle_min = float(config.get("angle_min_rad", -math.pi))
    angle_increment = float(config.get("angle_increment_rad", (2.0 * math.pi) / bin_count))
    angle_max = angle_min + (bin_count - 1) * angle_increment
    range_min = float(config.get("range_min_m", 0.05))
    range_max = float(config.get("range_max_m", 20.0))
    planar_z_tolerance_value = config.get("planar_z_tolerance_m")
    planar_z_tolerance = None if planar_z_tolerance_value is None else float(planar_z_tolerance_value)
    if not math.isfinite(angle_min) or not math.isfinite(angle_increment) or angle_increment <= 0.0:
        raise ValueError("CARLA LiDAR scan angles must be finite with a positive increment")
    if not math.isfinite(range_min) or not math.isfinite(range_max) or range_min < 0.0 or range_max <= range_min:
        raise ValueError("CARLA LiDAR scan ranges must satisfy 0 <= range_min_m < range_max_m")
    if planar_z_tolerance is not None and (
        not math.isfinite(planar_z_tolerance) or planar_z_tolerance < 0.0
    ):
        raise ValueError("CARLA LiDAR planar_z_tolerance_m must be non-negative and finite")

    timestamp_s = float(getattr(data, "timestamp_s", getattr(data, "timestamp", 0.0)))
    if not math.isfinite(timestamp_s) or timestamp_s < 0.0:
        raise ValueError("CARLA LiDAR measurement timestamp must be non-negative and finite")
    raw_data = getattr(data, "raw_data", None)
    if raw_data is None:
        raise ValueError("CARLA LiDAR measurement has no raw_data point cloud")
    points = np.frombuffer(raw_data, dtype=np.float32)
    if points.size % 4 != 0:
        raise ValueError("CARLA LiDAR raw_data must contain x, y, z, intensity float32 tuples")
    points = points.reshape((-1, 4))
    ranges = np.full(bin_count, np.inf, dtype=float)
    if points.size:
        x = points[:, 0].astype(float, copy=False)
        y = -points[:, 1].astype(float, copy=False)
        z = points[:, 2].astype(float, copy=False)
        distance = np.hypot(x, y)
        angle = np.arctan2(y, x)
        valid = (
            np.isfinite(distance)
            & np.isfinite(angle)
            & (distance >= range_min)
            & (distance <= range_max)
            & (angle >= angle_min)
            & (angle <= angle_max)
        )
        if planar_z_tolerance is not None:
            valid &= np.abs(z) <= planar_z_tolerance
        if np.any(valid):
            indices = np.floor((angle[valid] - angle_min) / angle_increment).astype(int)
            indices = np.clip(indices, 0, bin_count - 1)
            np.minimum.at(ranges, indices, distance[valid])

    scan_time_s = float(config.get("scan_time_s", 0.0))
    time_increment_s = float(config.get("time_increment_s", 0.0))
    return LaserScanSample(
        timestamp_ns=int(round(timestamp_s * 1_000_000_000)),
        frame_id=str(config.get("frame_id", "laser")),
        angle_min_rad=angle_min,
        angle_max_rad=angle_max,
        angle_increment_rad=angle_increment,
        time_increment_s=time_increment_s,
        scan_time_s=scan_time_s,
        range_min_m=range_min,
        range_max_m=range_max,
        ranges_m=tuple(float(value) for value in ranges),
    )


def _session_lidar_scan_config(session: Any) -> dict[str, Any]:
    getter = getattr(session, "lidar_scan_config", None)
    if not callable(getter):
        return {}
    config = getter()
    return dict(config) if isinstance(config, dict) else {}


def _positive_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"CARLA LiDAR {name} must be a positive integer")
    return value
