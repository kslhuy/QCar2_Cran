"""QCar sensor and actuator adapter over externally owned device objects."""

from __future__ import annotations

import time

import numpy as np

from .io_base import IOBase


class IOQCar2(IOBase):
    """Adapt an already-created QCar and optional QCarGPS source.

    This class does not create QCar, QCarGPS, QLabs, or real-time model
    resources. A physical-device bootstrap or future QLabs session owns those
    resources and passes the resulting objects to the adapter.
    """

    def __init__(self, config: dict, qcar, gps=None, vehicle_id: int = 0, logger=None):
        if qcar is None:
            raise ValueError("IOQCar2 requires an already-created qcar object")
        super().__init__(config, vehicle_id, logger)
        self._qcar = qcar
        self._gps = gps

    def _poll_sensors(self) -> None:
        now = time.time()
        try:
            self._qcar.read()
            self._sensor_data_cache.motor_tach = float(getattr(self._qcar, "motorTach", 0.0))
            self._sensor_data_cache.gyro_z = float(np.asarray(self._qcar.gyroscope, dtype=float).reshape(-1)[2])
            self._sensor_data_cache.accelerometer = np.asarray(
                self._qcar.accelerometer, dtype=float
            ).reshape(-1)[:3].copy()
            self._sensor_data_cache.sensor_timestamp = now
        except Exception as exc:
            self._logger.warning("Failed to read QCar sensors: %s", exc)

    def _poll_gps(self) -> None:
        now = time.time()
        gps = self._gps
        if gps is None:
            self._sensor_data_cache.gps_valid = False
            self._sensor_data_cache.gps_timestamp = now
            return
        try:
            if gps.readGPS():
                self._sensor_data_cache.gps_valid = True
                self._sensor_data_cache.gps_position = np.array(
                    [gps.position[0], gps.position[1], gps.orientation[2]], dtype=float
                )
                self._sensor_data_cache.gps_timestamp = now
                return
        except Exception as exc:
            self._logger.warning("Failed to read QCar GPS: %s", exc)
        self._sensor_data_cache.gps_valid = False
        self._sensor_data_cache.gps_timestamp = now

    def _hardware_write(self, throttle: float, steering: float) -> None:
        self._qcar.write(throttle=throttle, steering=steering)

    def _close_impl(self) -> None:
        """Release optional adapter-local GPS stream clients only."""
        gps = self._gps
        for stream_name in ("_gps_client", "_lidar_client"):
            stream = getattr(gps, stream_name, None)
            terminate = getattr(stream, "terminate", None)
            if callable(terminate):
                try:
                    terminate()
                except Exception as exc:
                    self._logger.debug("Failed to terminate QCar GPS stream %s: %s", stream_name, exc)
        lidar = getattr(gps, "lidar", None)
        terminate = getattr(lidar, "terminate", None)
        if callable(terminate):
            try:
                terminate()
            except Exception as exc:
                self._logger.debug("Failed to terminate QCar lidar client: %s", exc)
