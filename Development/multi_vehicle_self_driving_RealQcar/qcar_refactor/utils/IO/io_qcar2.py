"""QCar sensor and actuator adapter over externally owned device objects."""

from __future__ import annotations

import time
import math
import threading

import numpy as np

from core.vehicle_types import LaserScanSample
from .io_base import IOBase


class IOQCar2(IOBase):
    """Adapt an already-created QCar and optional QCarGPS source.

    This class does not create QCar, QCarGPS, QLabs, or real-time model
    resources. A physical-device bootstrap or future QLabs session owns those
    resources and passes the resulting objects to the adapter.
    """

    def __init__(
        self,
        config: dict,
        qcar,
        gps=None,
        lidar=None,
        lidar_manager=None,
        vehicle_id: int = 0,
        logger=None,
    ):
        if qcar is None:
            raise ValueError("IOQCar2 requires an already-created qcar object")
        super().__init__(config, vehicle_id, logger)
        self._qcar = qcar
        self._gps = gps
        self._lidar = lidar
        self._lidar_manager = lidar_manager
        self._lidar_settings = _qcar_lidar_settings(self._config)
        if lidar is not None and lidar_manager is not None:
            raise ValueError("IOQCar2 accepts either a direct LiDAR or a LiDAR resource manager, not both")
        self._lidar_worker_lock = threading.Lock()
        self._lidar_worker_stop = threading.Event()
        self._lidar_worker: threading.Thread | None = None
        self._lidar_enabled = bool(lidar is not None and self._lidar_settings["enabled"])
        self._lidar_read_failures = 0
        self._lidar_last_timestamp_ns: int | None = None

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

    def read_to_cache(self) -> None:
        """Refresh required QCar data and lazily start local LiDAR acquisition.

        The worker runs independently after this first refresh, so LiDAR
        polling is not tied to the control-loop rate. The later IO template
        method will make this startup hook common across all optional sources.
        """

        self._ensure_lidar_worker()
        super().read_to_cache()

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

    def set_lidar_enabled(self, enabled: bool) -> bool:
        """Start or stop the local LiDAR worker and its optional PAL session.

        Direct sources are owned for the enclosing platform context.  For the
        normal physical-QCar runtime, an injected platform manager creates PAL
        only when an operator enables LiDAR and releases it after disable.
        """

        if not isinstance(enabled, bool):
            raise ValueError("LiDAR enabled state must be boolean")
        if enabled:
            if self._lidar is None and self._lidar_manager is None:
                return False
            if self._lidar is None:
                try:
                    lidar = self._lidar_manager.acquire()
                except Exception as error:
                    raise RuntimeError(f"Unable to acquire QCar LiDAR: {error}") from error
                if lidar is None:
                    raise RuntimeError("QCar LiDAR manager returned no resource")
                with self._lidar_worker_lock:
                    self._lidar = lidar
            with self._lidar_worker_lock:
                self._lidar_enabled = True
                self._lidar_worker_stop.clear()
            return True

        release_manager = False
        with self._lidar_worker_lock:
            self._lidar_enabled = False
            self._lidar_worker_stop.set()
            worker = self._lidar_worker
            if (worker is None or not worker.is_alive()) and self._lidar_manager is not None and self._lidar is not None:
                self._lidar = None
                release_manager = True
        if release_manager:
            self._lidar_manager.release()
        return True

    def lidar_status(self) -> dict:
        """Return common queue counters plus QCar-worker health."""

        status = super().lidar_status()
        with self._lidar_worker_lock:
            worker = self._lidar_worker
            status.update(
                {
                    "source": self._lidar_settings["source"],
                    "available": self._lidar is not None or self._lidar_manager is not None,
                    "resource_open": self._lidar is not None,
                    "enabled": self._lidar_enabled,
                    "worker_running": bool(worker is not None and worker.is_alive()),
                    "read_failures": self._lidar_read_failures,
                    "last_timestamp_ns": self._lidar_last_timestamp_ns,
                }
            )
        return status

    def _close_impl(self) -> None:
        """Stop only the adapter-owned polling thread.

        The platform resource context terminates a direct PAL LiDAR object
        after this adapter has stopped using it.  A managed LiDAR is released
        here if the normal stop command has not already done so.
        """

        with self._lidar_worker_lock:
            self._lidar_enabled = False
            self._lidar_worker_stop.set()
        with self._lidar_worker_lock:
            worker = self._lidar_worker
        if worker is not None and worker.is_alive():
            worker.join(timeout=2.0)
            if worker.is_alive():
                self._logger.warning("QCar LiDAR worker did not stop within two seconds")
                return
        release_manager = False
        with self._lidar_worker_lock:
            self._lidar_worker = None
            if self._lidar_manager is not None and self._lidar is not None:
                self._lidar = None
                release_manager = True
        if release_manager:
            self._lidar_manager.release()

    def _ensure_lidar_worker(self) -> None:
        with self._lidar_worker_lock:
            if self._lidar_worker is not None and self._lidar_worker.is_alive():
                return
            self._lidar_worker = None
            if not self._lidar_enabled or self._lidar is None:
                return
            self._lidar_worker_stop.clear()
            self._lidar_worker = threading.Thread(
                target=self._run_lidar_worker,
                name=f"QCarLidar-{self._vehicle_id}",
                daemon=True,
            )
            self._lidar_worker.start()

    def _run_lidar_worker(self) -> None:
        period_s = 1.0 / self._lidar_settings["poll_rate_hz"]
        next_poll_at = time.monotonic()
        try:
            while not self._lidar_worker_stop.is_set():
                with self._lidar_worker_lock:
                    lidar = self._lidar
                    enabled = self._lidar_enabled
                if not enabled or lidar is None:
                    break
                try:
                    if bool(lidar.read()):
                        timestamp_ns = time.time_ns()
                        scan = qcar_lidar_to_laser_scan(
                            lidar.angles,
                            lidar.distances,
                            timestamp_ns=timestamp_ns,
                            scan_config=self._lidar_settings,
                        )
                        self._publish_lidar_scan(scan)
                        with self._lidar_worker_lock:
                            self._lidar_last_timestamp_ns = timestamp_ns
                except Exception as exc:
                    with self._lidar_worker_lock:
                        self._lidar_read_failures += 1
                    self._logger.warning("Failed to read QCar LiDAR scan: %s", exc)
                next_poll_at += period_s
                self._lidar_worker_stop.wait(max(0.0, next_poll_at - time.monotonic()))
        finally:
            release_manager = False
            with self._lidar_worker_lock:
                self._lidar_worker = None
                if not self._lidar_enabled and self._lidar_manager is not None and self._lidar is not None:
                    self._lidar = None
                    release_manager = True
            if release_manager:
                self._lidar_manager.release()


def qcar_lidar_to_laser_scan(
    angles, distances, *, timestamp_ns: int, scan_config: dict | None = None
) -> LaserScanSample:
    """Bin one PAL ``QCarLidar`` frame into the common planar scan contract.

    PAL returns matching heading and distance arrays. The selected QCar IO
    profile explicitly owns the provisional sensor-frame sign and offset;
    they remain calibration values until the deployed stationary probe has
    validated physical orientation.
    """

    config = _qcar_lidar_settings({"sensors": {"lidar": dict(scan_config or {})}})
    raw_angles = np.asarray(angles, dtype=float).reshape(-1)
    raw_distances = np.asarray(distances, dtype=float).reshape(-1)
    if raw_angles.size == 0 or raw_angles.size != raw_distances.size:
        raise ValueError(
            f"QCar LiDAR requires equally sized non-empty arrays; got {raw_angles.size} and {raw_distances.size}"
        )
    if not np.all(np.isfinite(raw_angles)):
        raise ValueError("QCar LiDAR angles must be finite")

    bin_count = config["bin_count"]
    angle_min = config["angle_min_rad"]
    angle_increment = (2.0 * math.pi) / bin_count
    angle_max = angle_min + (bin_count - 1) * angle_increment
    angles_projected = np.mod(
        config["angle_sign"] * raw_angles + config["angle_offset_rad"] - angle_min,
        2.0 * math.pi,
    ) + angle_min
    ranges = np.full(bin_count, np.inf, dtype=float)
    valid = (
        np.isfinite(raw_distances)
        & (raw_distances >= config["range_min_m"])
        & (raw_distances <= config["range_max_m"])
    )
    if np.any(valid):
        indices = np.floor((angles_projected[valid] - angle_min) / angle_increment).astype(int)
        indices = np.clip(indices, 0, bin_count - 1)
        np.minimum.at(ranges, indices, raw_distances[valid])
    return LaserScanSample(
        timestamp_ns=timestamp_ns,
        frame_id=config["frame_id"],
        angle_min_rad=angle_min,
        angle_max_rad=angle_max,
        angle_increment_rad=angle_increment,
        time_increment_s=0.0,
        scan_time_s=1.0 / config["poll_rate_hz"],
        range_min_m=config["range_min_m"],
        range_max_m=config["range_max_m"],
        ranges_m=tuple(float(value) for value in ranges),
    )


def _qcar_lidar_settings(config: dict) -> dict:
    sensors = config.get("sensors", {}) if isinstance(config, dict) else {}
    lidar = sensors.get("lidar", {}) if isinstance(sensors, dict) else {}
    if not isinstance(lidar, dict):
        raise ValueError("QCar sensors.lidar must be a mapping")
    enabled = bool(lidar.get("enabled", False))
    source = str(lidar.get("source", "qcar_hardware"))
    if source != "qcar_hardware":
        raise ValueError("IOQCar2 supports only sensors.lidar.source: qcar_hardware")
    poll_rate_hz = _positive_float(lidar.get("poll_rate_hz", 20.0), "poll_rate_hz")
    bin_count = _positive_int(lidar.get("bin_count", 360), "bin_count")
    range_min_m = _non_negative_float(lidar.get("range_min_m", 0.05), "range_min_m")
    range_max_m = _positive_float(lidar.get("range_max_m", 12.0), "range_max_m")
    if range_max_m <= range_min_m:
        raise ValueError("QCar LiDAR range_max_m must exceed range_min_m")
    angle_min_rad = _finite_float(lidar.get("angle_min_rad", -math.pi), "angle_min_rad")
    angle_offset_rad = _finite_float(lidar.get("angle_offset_rad", 0.0), "angle_offset_rad")
    angle_sign = _finite_float(lidar.get("angle_sign", 1.0), "angle_sign")
    if angle_sign not in (-1.0, 1.0):
        raise ValueError("QCar LiDAR angle_sign must be either -1.0 or 1.0")
    frame_id = lidar.get("frame_id", "qcar_lidar")
    if not isinstance(frame_id, str) or not frame_id:
        raise ValueError("QCar LiDAR frame_id must be a non-empty string")
    return {
        "enabled": enabled,
        "source": source,
        "poll_rate_hz": poll_rate_hz,
        "bin_count": bin_count,
        "range_min_m": range_min_m,
        "range_max_m": range_max_m,
        "angle_min_rad": angle_min_rad,
        "angle_offset_rad": angle_offset_rad,
        "angle_sign": angle_sign,
        "frame_id": frame_id,
    }


def _finite_float(value, name: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
        raise ValueError(f"QCar LiDAR {name} must be finite")
    return float(value)


def _non_negative_float(value, name: str) -> float:
    result = _finite_float(value, name)
    if result < 0.0:
        raise ValueError(f"QCar LiDAR {name} must be non-negative")
    return result


def _positive_float(value, name: str) -> float:
    result = _finite_float(value, name)
    if result <= 0.0:
        raise ValueError(f"QCar LiDAR {name} must be positive")
    return result


def _positive_int(value, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"QCar LiDAR {name} must be a positive integer")
    return value
