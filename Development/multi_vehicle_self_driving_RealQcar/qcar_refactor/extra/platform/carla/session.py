"""Synchronous CARLA platform world, actor, sensor, and cleanup ownership.

This optional platform integration belongs in ``extra`` because it starts and
owns a simulator session; it is not a reusable vehicle utility.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
import time
from typing import Any


@dataclass(frozen=True)
class CarlaSensorSnapshot:
    """Latest session-owned CARLA sensor values in project-facing units."""

    frame: int
    timestamp: float
    accelerometer: tuple[float, float, float] = (0.0, 0.0, 0.0)
    gyro_z: float = 0.0


@dataclass(frozen=True)
class _CarlaLidarRawFrame:
    """One copied native CARLA LiDAR callback payload.

    The session owns the CARLA actor and stores only this raw payload.  The
    local ``IOCarla`` adapter later converts it to the common scan contract.
    """

    frame: int
    timestamp_s: float
    raw_data: bytes


class CarlaSession:
    """Own one CARLA client/world configuration and its spawned actors.

    The session owns all CARLA API lifecycle work. Consumers receive only the
    ego actor and immutable latest sensor snapshot.
    """

    _WORLD_SETTING_FIELDS = (
        "synchronous_mode",
        "no_rendering_mode",
        "fixed_delta_seconds",
        "substepping",
        "max_substep_delta_time",
        "max_substeps",
        "max_culling_distance",
        "deterministic_ragdolls",
        "tile_stream_distance",
        "actor_active_distance",
        "spectator_as_ego",
    )

    def __init__(self, config: dict[str, Any], client=None, carla_api=None, logger=None):
        self._config = dict(config)
        self._client = client
        self._carla = carla_api
        self._logger = logger
        self._tick_owner = bool(self._config.get("tick_owner", True))
        self._world = None
        self._original_settings = None
        self._ego_actor = None
        self._sensor_actors: list[Any] = []
        self._snapshot_lock = threading.Lock()
        self._snapshot = CarlaSensorSnapshot(frame=0, timestamp=0.0)
        self._lidar_measurements: deque[_CarlaLidarRawFrame] = deque()
        self._lidar_measurement_capacity = self._resolve_lidar_queue_capacity()
        self._started = False
        self._closed = False

    @property
    def ego_actor(self):
        if self._ego_actor is None:
            raise RuntimeError("CARLA session has not spawned an ego actor")
        return self._ego_actor

    def get_snapshot(self) -> CarlaSensorSnapshot:
        with self._snapshot_lock:
            return self._snapshot

    def drain_lidar_measurements(self) -> tuple[_CarlaLidarRawFrame, ...]:
        """Return each raw LiDAR callback payload once in frame order.

        This is a process-local hand-off to ``IOCarla``.  It is deliberately
        not a normalized scan API and never forwards data to the ground
        station.
        """

        with self._snapshot_lock:
            measurements = tuple(self._lidar_measurements)
            self._lidar_measurements.clear()
            return measurements

    def lidar_scan_config(self) -> dict[str, Any]:
        """Return CARLA-only projection settings for the local IO adapter."""

        sensor = self._config.get("sensors", {}).get("lidar", {})
        scan = sensor.get("scan", {}) if isinstance(sensor, dict) else {}
        return dict(scan) if isinstance(scan, dict) else {}

    def make_vehicle_control(self, throttle: float, steer: float, brake: float):
        """Construct a CARLA control object without exposing the CARLA API handle."""
        if self._carla is None:
            raise RuntimeError("CARLA session has not initialized its API")
        return self._carla.VehicleControl(throttle=throttle, steer=steer, brake=brake)

    def start(self) -> None:
        if self._started:
            return
        if self._closed:
            raise RuntimeError("Cannot restart a closed CARLA session")
        try:
            self._ensure_client()
            self._world = self._client.get_world()
            if self._tick_owner:
                self._configure_synchronous_world()
            self._spawn_ego()
            self._spawn_sensors()
            if self._tick_owner:
                for _ in range(max(0, int(self._config.get("warmup_ticks", 0)))):
                    self.tick()
            self._started = True
        except Exception:
            self.close()
            raise

    def tick(self) -> CarlaSensorSnapshot:
        if not self._started and self._ego_actor is None:
            raise RuntimeError("CARLA session has not started")
        if self._tick_owner:
            frame = int(self._world.tick())
            timestamp = self._world_timestamp()
        else:
            wait_for_tick = getattr(self._world, "wait_for_tick", None)
            if not callable(wait_for_tick):
                raise RuntimeError("CARLA follower session requires world.wait_for_tick()")
            snapshot = wait_for_tick(float(self._config.get("timeout_s", 10.0)))
            frame = int(getattr(snapshot, "frame", self.get_snapshot().frame))
            timestamp = self._snapshot_timestamp(snapshot)
        with self._snapshot_lock:
            self._snapshot = CarlaSensorSnapshot(
                frame=frame,
                timestamp=timestamp,
                accelerometer=self._snapshot.accelerometer,
                gyro_z=self._snapshot.gyro_z,
            )
            return self._snapshot

    def stop(self) -> None:
        if self._ego_actor is None:
            return
        apply_control = getattr(self._ego_actor, "apply_control", None)
        if callable(apply_control) and self._carla is not None:
            try:
                apply_control(self._carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
            except Exception as exc:
                self._log("debug", "Failed to apply CARLA safe stop: %s", exc)

    def close(self) -> None:
        if self._closed:
            return
        try:
            self.stop()
            for sensor in reversed(self._sensor_actors):
                stop = getattr(sensor, "stop", None)
                if callable(stop):
                    try:
                        stop()
                    except Exception as exc:
                        self._log("debug", "Failed to stop CARLA sensor: %s", exc)
                self._destroy_actor(sensor)
            self._sensor_actors.clear()
            self._destroy_actor(self._ego_actor)
            self._ego_actor = None
            if self._world is not None and self._original_settings is not None:
                self._restore_world_settings()
        finally:
            self._started = False
            self._closed = True

    def _ensure_client(self) -> None:
        if self._carla is None:
            import carla  # Optional dependency: imported only for live CARLA sessions.

            self._carla = carla
        if self._client is None:
            self._client = self._carla.Client(str(self._config.get("host", "127.0.0.1")), int(self._config.get("port", 2000)))
        set_timeout = getattr(self._client, "set_timeout", None)
        if callable(set_timeout):
            set_timeout(float(self._config.get("timeout_s", 10.0)))

    def _configure_synchronous_world(self) -> None:
        settings = self._world.get_settings()
        self._original_settings = {
            field: getattr(settings, field)
            for field in self._WORLD_SETTING_FIELDS
            if hasattr(settings, field)
        }
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = float(self._config.get("fixed_delta_seconds", 0.05))
        self._world.apply_settings(settings)

    def _restore_world_settings(self) -> None:
        """Restore settings without copying CARLA's non-pickleable wrapper."""
        settings = self._world.get_settings()
        for field, value in self._original_settings.items():
            setattr(settings, field, value)
        self._world.apply_settings(settings)

    def _spawn_ego(self) -> None:
        library = self._world.get_blueprint_library()
        blueprint_id = str(self._config.get("ego_blueprint", "vehicle.tesla.model3"))
        blueprint = library.find(blueprint_id)
        transform = self._spawn_transform()
        spawn = getattr(self._world, "try_spawn_actor", None) or self._world.spawn_actor
        self._ego_actor = spawn(blueprint, transform)
        if self._ego_actor is None:
            location = transform.location
            raise RuntimeError(
                f"CARLA could not spawn '{blueprint_id}' at "
                f"({location.x:.2f}, {location.y:.2f}, {location.z:.2f})"
            )

    def _spawn_transform(self):
        """Use an explicit transform when a custom world has no parsed map."""
        configured = self._config.get("spawn_transform", self._config.get("default_spawn_transform"))
        if isinstance(configured, dict):
            return self._make_transform(configured)
        spawn_points = self._world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("CARLA map has no spawn points; configure a scenario spawn_transform")
        index = int(self._config.get("spawn_point_index", 0))
        return spawn_points[index % len(spawn_points)]

    def _spawn_sensors(self) -> None:
        for name, sensor_config in self._config.get("sensors", {}).items():
            if not sensor_config.get("enabled", False):
                continue
            blueprint = self._world.get_blueprint_library().find(sensor_config["blueprint"])
            self._configure_sensor_attributes(blueprint, sensor_config)
            transform = self._make_transform(sensor_config.get("transform", {}))
            actor = self._world.spawn_actor(blueprint, transform, attach_to=self.ego_actor)
            actor.listen(self._sensor_callback(name, sensor_config))
            self._sensor_actors.append(actor)

    @staticmethod
    def _configure_sensor_attributes(blueprint, sensor_config: dict[str, Any]) -> None:
        """Apply optional CARLA blueprint attributes without leaking them to callers."""

        set_attribute = getattr(blueprint, "set_attribute", None)
        attributes = sensor_config.get("attributes", {})
        if not callable(set_attribute) or not isinstance(attributes, dict):
            return
        for key, value in attributes.items():
            set_attribute(str(key), str(value))

    def _make_transform(self, values: dict[str, Any]):
        location = self._carla.Location(
            x=float(values.get("x", 0.0)), y=float(values.get("y", 0.0)), z=float(values.get("z", 0.0))
        )
        rotation = self._carla.Rotation(
            pitch=float(values.get("pitch", 0.0)), yaw=float(values.get("yaw", 0.0)), roll=float(values.get("roll", 0.0))
        )
        return self._carla.Transform(location, rotation)

    def _sensor_callback(self, name: str, sensor_config: dict[str, Any]):
        def callback(data) -> None:
            timestamp = float(getattr(data, "timestamp", self._world_timestamp()))
            frame = int(getattr(data, "frame", self.get_snapshot().frame))
            with self._snapshot_lock:
                previous = self._snapshot
                accelerometer = previous.accelerometer
                gyro_z = previous.gyro_z
                if name == "imu":
                    accel = data.accelerometer
                    gyro = data.gyroscope
                    accelerometer = (float(accel.x), -float(accel.y), float(accel.z))
                    gyro_z = -float(gyro.z)
                elif name == "lidar":
                    try:
                        measurement = _CarlaLidarRawFrame(
                            frame=frame,
                            timestamp_s=timestamp,
                            raw_data=bytes(data.raw_data),
                        )
                        if len(self._lidar_measurements) >= self._lidar_measurement_capacity:
                            self._lidar_measurements.popleft()
                        self._lidar_measurements.append(measurement)
                    except Exception as exc:
                        self._log("warning", "Failed to copy CARLA LiDAR callback: %s", exc)
                self._snapshot = CarlaSensorSnapshot(frame, timestamp, accelerometer, gyro_z)

        return callback

    def _world_timestamp(self) -> float:
        snapshot = getattr(self._world, "get_snapshot", lambda: None)()
        return self._snapshot_timestamp(snapshot)

    @staticmethod
    def _snapshot_timestamp(snapshot) -> float:
        timestamp = getattr(snapshot, "timestamp", None)
        return float(getattr(timestamp, "elapsed_seconds", time.monotonic()))

    def _resolve_lidar_queue_capacity(self) -> int:
        sensor = self._config.get("sensors", {}).get("lidar", {})
        value = sensor.get("callback_queue_capacity", 8) if isinstance(sensor, dict) else 8
        try:
            capacity = int(value)
        except (TypeError, ValueError) as exc:
            raise ValueError("CARLA LiDAR callback_queue_capacity must be a positive integer") from exc
        if capacity <= 0:
            raise ValueError("CARLA LiDAR callback_queue_capacity must be a positive integer")
        return capacity

    def _destroy_actor(self, actor) -> None:
        destroy = getattr(actor, "destroy", None)
        if callable(destroy):
            try:
                destroy()
            except Exception as exc:
                self._log("debug", "Failed to destroy CARLA actor: %s", exc)

    def _log(self, level: str, message: str, *args) -> None:
        if self._logger is not None:
            getattr(self._logger, level)(message, *args)
