"""
Pure-Python CARLA bridge for the existing QCar vehicle stack.

This module deliberately avoids ROS dependencies. It connects to CARLA through
the Python API, exposes QCar-like hardware/GPS adapters, and keeps CARLA
synchronous ticking under the control of the main vehicle loop.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
from typing import Any, List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class CarlaBridgeConfig:
    """Runtime configuration for a single CARLA ego vehicle."""

    host: str = "127.0.0.1"
    port: int = 2000
    timeout: float = 10.0
    fixed_delta_seconds: float = 0.05
    synchronous_mode: bool = True

    ego_role_name: str = "ego"
    vehicle_blueprint: str = "vehicle.lincoln.mkz_2020"
    spawn_point_index: int = 0
    goal_spawn_indices: List[int] = field(default_factory=lambda: [10])
    use_existing_ego: bool = False
    destroy_spawned_ego: bool = True

    route_sampling_resolution: float = 2.0
    fallback_route_points: int = 80
    draw_route: bool = False

    camera_width: int = 640
    camera_height: int = 360
    camera_fov: float = 90.0
    lidar_range: float = 30.0
    lidar_channels: int = 32
    lidar_points_per_second: int = 56000
    lidar_rotation_frequency: float = 20.0

    max_steer_rad: float = 0.55
    steer_sign: float = 1.0
    command_to_speed_gain: float = 6.0
    max_target_speed: float = 2.5
    speed_kp: float = 0.55
    speed_ki: float = 0.02
    speed_kd: float = 0.04
    max_throttle: float = 0.65
    max_brake: float = 0.8
    brake_gain: float = 1.2
    stop_brake: float = 0.35

    obstacle_min_distance: float = 0.35
    obstacle_max_distance: float = 12.0
    obstacle_lateral_window: float = 2.5


class CarlaGPSAdapter:
    """QCarGPS-compatible adapter backed by a CARLA vehicle actor."""

    def __init__(self, ego_vehicle: Any):
        self.ego_vehicle = ego_vehicle
        self.position = np.zeros(3, dtype=float)
        self.orientation = np.zeros(3, dtype=float)
        self._last_update = 0.0
        self._last_read_time = 0.0
        self.update_from_actor()

    def update_from_actor(self) -> None:
        transform = self.ego_vehicle.get_transform()
        self.position[:] = [
            float(transform.location.x),
            float(transform.location.y),
            float(transform.location.z),
        ]
        self.orientation[:] = [
            math.radians(float(transform.rotation.roll)),
            math.radians(float(transform.rotation.pitch)),
            math.radians(float(transform.rotation.yaw)),
        ]
        self._last_update = time.time()

    def readGPS(self) -> bool:
        self.update_from_actor()
        if self._last_update > self._last_read_time:
            self._last_read_time = self._last_update
            return True
        return False

    def read(self) -> List[float]:
        self.update_from_actor()
        return [
            float(self.position[0]),
            float(self.position[1]),
            float(self.orientation[2]),
        ]


class CarlaQCarAdapter:
    """QCar-compatible command/sensor adapter for one CARLA vehicle."""

    def __init__(self, carla_module: Any, ego_vehicle: Any, config: CarlaBridgeConfig):
        self.carla = carla_module
        self.ego_vehicle = ego_vehicle
        self.config = config

        self.motorCurrent = 0.0
        self.batteryVoltage = 12.0
        self.motorEncoder = []

        self._motor_tach = 0.0
        self._gyroscope = np.zeros(3, dtype=float)
        self._accelerometer = np.zeros(3, dtype=float)
        self._last_update = time.time()

        self._speed_error_i = 0.0
        self._prev_speed_error: Optional[float] = None
        self._last_control_time = time.time()
        self._last_control = None
        self._last_target_speed = 0.0
        self._last_requested_command = 0.0

        self.update_from_actor()

    @property
    def motorTach(self) -> float:
        return float(self._motor_tach)

    @property
    def gyroscope(self) -> np.ndarray:
        return self._gyroscope

    @property
    def accelerometer(self) -> np.ndarray:
        return self._accelerometer

    def update_imu(self, imu_measurement: Any) -> None:
        """Update accelerometer and gyro from a CARLA IMU callback."""
        try:
            self._accelerometer[:] = [
                float(imu_measurement.accelerometer.x),
                float(imu_measurement.accelerometer.y),
                float(imu_measurement.accelerometer.z),
            ]
            self._gyroscope[:] = [
                float(imu_measurement.gyroscope.x),
                float(imu_measurement.gyroscope.y),
                float(imu_measurement.gyroscope.z),
            ]
            self._last_update = time.time()
        except Exception:
            self.update_from_actor()

    def update_from_actor(self) -> None:
        velocity = self.ego_vehicle.get_velocity()
        acceleration = self.ego_vehicle.get_acceleration()
        angular_velocity = self.ego_vehicle.get_angular_velocity()
        transform = self.ego_vehicle.get_transform()
        forward = transform.get_forward_vector()

        signed_speed = (
            float(velocity.x) * float(forward.x)
            + float(velocity.y) * float(forward.y)
            + float(velocity.z) * float(forward.z)
        )
        self._motor_tach = signed_speed
        self._accelerometer[:] = [
            float(acceleration.x),
            float(acceleration.y),
            float(acceleration.z),
        ]
        # CARLA angular velocity is reported in deg/s by the Python API.
        self._gyroscope[:] = [
            math.radians(float(angular_velocity.x)),
            math.radians(float(angular_velocity.y)),
            math.radians(float(angular_velocity.z)),
        ]
        self._last_update = time.time()

    def read(self) -> List[float]:
        self.update_from_actor()
        return [
            float(self._motor_tach),
            0.0,
            float(self._gyroscope[0]),
            float(self._gyroscope[1]),
            float(self._gyroscope[2]),
        ]

    def read_write_std(self, throttle: float = 0.0, steering: float = 0.0, LEDs=None):
        self.write(throttle=throttle, steering=steering)

    def write(self, throttle: float = 0.0, steering: float = 0.0):
        """Convert QCar-style command into CARLA VehicleControl."""
        self.update_from_actor()
        command = float(throttle) if np.isfinite(throttle) else 0.0
        steering = float(steering) if np.isfinite(steering) else 0.0

        target_speed = self._command_to_target_speed(command)
        control = self._target_speed_to_control(target_speed, steering)
        self.ego_vehicle.apply_control(control)

        self._last_control = control
        self._last_target_speed = target_speed
        self._last_requested_command = command

    def emergency_stop(self):
        control = self.carla.VehicleControl(
            throttle=0.0,
            steer=0.0,
            brake=1.0,
            hand_brake=False,
            reverse=False,
        )
        self.ego_vehicle.apply_control(control)
        self._last_control = control
        self._last_target_speed = 0.0
        self._speed_error_i = 0.0
        self._prev_speed_error = None

    def _command_to_target_speed(self, command: float) -> float:
        if command <= 0.0:
            return 0.0
        return float(
            np.clip(
                command * float(self.config.command_to_speed_gain),
                0.0,
                float(self.config.max_target_speed),
            )
        )

    def _target_speed_to_control(self, target_speed: float, steering: float):
        current_speed = max(float(self._motor_tach), 0.0)
        error = float(target_speed) - current_speed
        now = time.time()
        dt = max(now - self._last_control_time, 1e-3)
        self._last_control_time = now

        self._speed_error_i = float(np.clip(self._speed_error_i + error * dt, -5.0, 5.0))
        if self._prev_speed_error is None:
            d_error = 0.0
        else:
            d_error = (error - self._prev_speed_error) / dt
        self._prev_speed_error = error

        pedal = (
            float(self.config.speed_kp) * error
            + float(self.config.speed_ki) * self._speed_error_i
            + float(self.config.speed_kd) * d_error
        )

        if target_speed <= 1e-3:
            throttle = 0.0
            brake = float(self.config.stop_brake) if current_speed > 0.05 else 0.0
            self._speed_error_i = 0.0
        elif pedal >= 0.0:
            throttle = float(np.clip(pedal, 0.0, float(self.config.max_throttle)))
            brake = 0.0
        else:
            throttle = 0.0
            brake = float(
                np.clip(
                    -pedal * float(self.config.brake_gain),
                    0.0,
                    float(self.config.max_brake),
                )
            )

        steer = float(
            np.clip(
                float(self.config.steer_sign)
                * steering
                / max(float(self.config.max_steer_rad), 1e-6),
                -1.0,
                1.0,
            )
        )

        return self.carla.VehicleControl(
            throttle=throttle,
            steer=steer,
            brake=brake,
            hand_brake=False,
            reverse=False,
        )


class CarlaSimulationBridge:
    """Owns CARLA connection, actors, sensors, route, and per-tick updates."""

    def __init__(self, config: CarlaBridgeConfig, logger=None):
        self.config = config
        self.logger = logger
        self.carla = self._import_carla()

        self.client = None
        self.world = None
        self.original_settings = None
        self.ego_vehicle = None
        self.spawned_ego = False
        self.sensors: List[Any] = []

        self.qcar_adapter: Optional[CarlaQCarAdapter] = None
        self.gps_adapter: Optional[CarlaGPSAdapter] = None
        self.waypoint_sequence: Optional[np.ndarray] = None

        self.latest_camera = None
        self.latest_lidar_points = None
        self.latest_collision = None
        self._collision_reported = False
        self._last_opponent_payload: List[float] = []

    @staticmethod
    def _import_carla():
        try:
            import carla  # type: ignore

            return carla
        except Exception as exc:
            raise RuntimeError(
                "CARLA Python API is not importable. Install the CARLA UE5 Python "
                "package for this Python environment before running vehicle_main_carla.py."
            ) from exc

    def setup(self) -> None:
        self.client = self.carla.Client(self.config.host, int(self.config.port))
        self.client.set_timeout(float(self.config.timeout))
        self.world = self.client.get_world()

        self.original_settings = self.world.get_settings()
        if self.config.synchronous_mode:
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = float(self.config.fixed_delta_seconds)
            self.world.apply_settings(settings)

        self.ego_vehicle = self._select_or_spawn_ego()
        self.ego_vehicle.set_autopilot(False)

        self.qcar_adapter = CarlaQCarAdapter(self.carla, self.ego_vehicle, self.config)
        self.gps_adapter = CarlaGPSAdapter(self.ego_vehicle)

        self._spawn_sensors()
        self.waypoint_sequence = self.generate_route_waypoints()
        if self.config.draw_route and self.waypoint_sequence is not None:
            self._draw_route(self.waypoint_sequence)

        self._log(
            "CARLA bridge ready: "
            f"ego_id={self.ego_vehicle.id}, route_points={self.waypoint_sequence.shape[1]}"
        )

    def tick(self) -> float:
        if self.world is None:
            raise RuntimeError("CARLA bridge is not set up")
        frame = self.world.tick()
        if self.qcar_adapter is not None:
            self.qcar_adapter.update_from_actor()
        if self.gps_adapter is not None:
            self.gps_adapter.update_from_actor()
        return float(frame)

    def destroy(self) -> None:
        if self.qcar_adapter is not None:
            try:
                self.qcar_adapter.emergency_stop()
            except Exception:
                pass

        for sensor in list(self.sensors):
            try:
                sensor.stop()
            except Exception:
                pass
            try:
                sensor.destroy()
            except Exception:
                pass
        self.sensors.clear()

        if (
            self.spawned_ego
            and self.config.destroy_spawned_ego
            and self.ego_vehicle is not None
        ):
            try:
                self.ego_vehicle.destroy()
            except Exception:
                pass

        if (
            self.world is not None
            and self.original_settings is not None
            and self.config.synchronous_mode
        ):
            try:
                self.world.apply_settings(self.original_settings)
            except Exception:
                pass

    def has_collision(self) -> bool:
        return self.latest_collision is not None

    def consume_collision_report(self) -> Optional[Any]:
        if self.latest_collision is None or self._collision_reported:
            return None
        self._collision_reported = True
        return self.latest_collision

    def get_opponent_payload(self) -> List[float]:
        return list(self._last_opponent_payload)

    def _select_or_spawn_ego(self):
        if self.config.use_existing_ego:
            ego = self._find_existing_ego()
            if ego is not None:
                self._log(f"Using existing CARLA ego vehicle id={ego.id}")
                return ego

        bp_lib = self.world.get_blueprint_library()
        try:
            vehicle_bp = bp_lib.find(self.config.vehicle_blueprint)
        except Exception:
            matches = list(bp_lib.filter("vehicle.*"))
            if not matches:
                raise RuntimeError("No CARLA vehicle blueprints available")
            vehicle_bp = matches[0]
            self._log(
                f"Vehicle blueprint '{self.config.vehicle_blueprint}' not found; "
                f"using '{vehicle_bp.id}'"
            )

        self._set_blueprint_attribute(vehicle_bp, "role_name", self.config.ego_role_name)
        self._set_blueprint_attribute(vehicle_bp, "ros_name", self.config.ego_role_name)

        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("CARLA map has no spawn points")

        start = int(self.config.spawn_point_index) % len(spawn_points)
        ordered_indices = list(range(start, len(spawn_points))) + list(range(0, start))
        for idx in ordered_indices:
            vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_points[idx])
            if vehicle is not None:
                self.spawned_ego = True
                self.config.spawn_point_index = idx
                self._log(f"Spawned CARLA ego vehicle id={vehicle.id} at spawn {idx}")
                return vehicle

        raise RuntimeError("Failed to spawn CARLA ego vehicle at any map spawn point")

    def _find_existing_ego(self):
        role = str(self.config.ego_role_name)
        for actor in self.world.get_actors().filter("vehicle.*"):
            attrs = getattr(actor, "attributes", {}) or {}
            if attrs.get("role_name") == role or attrs.get("ros_name") == role:
                return actor
        return None

    @staticmethod
    def _set_blueprint_attribute(blueprint: Any, name: str, value: Any) -> None:
        try:
            if blueprint.has_attribute(name):
                blueprint.set_attribute(name, str(value))
        except Exception:
            pass

    def _spawn_sensors(self) -> None:
        bp_lib = self.world.get_blueprint_library()

        self._spawn_sensor(
            bp_lib.find("sensor.camera.rgb"),
            self.carla.Transform(
                self.carla.Location(x=1.6, z=1.4),
                self.carla.Rotation(pitch=-8.0),
            ),
            self._on_camera,
            {
                "image_size_x": self.config.camera_width,
                "image_size_y": self.config.camera_height,
                "fov": self.config.camera_fov,
                "sensor_tick": self.config.fixed_delta_seconds,
            },
        )

        self._spawn_sensor(
            bp_lib.find("sensor.lidar.ray_cast"),
            self.carla.Transform(self.carla.Location(x=0.0, z=1.8)),
            self._on_lidar,
            {
                "range": self.config.lidar_range,
                "channels": self.config.lidar_channels,
                "points_per_second": self.config.lidar_points_per_second,
                "rotation_frequency": self.config.lidar_rotation_frequency,
                "sensor_tick": self.config.fixed_delta_seconds,
            },
        )

        self._spawn_sensor(
            bp_lib.find("sensor.other.imu"),
            self.carla.Transform(self.carla.Location(x=0.0, z=0.5)),
            self._on_imu,
            {"sensor_tick": self.config.fixed_delta_seconds},
        )

        self._spawn_sensor(
            bp_lib.find("sensor.other.collision"),
            self.carla.Transform(),
            self._on_collision,
            {},
        )

    def _spawn_sensor(
        self,
        blueprint: Any,
        transform: Any,
        callback,
        attributes: dict,
    ) -> None:
        for name, value in attributes.items():
            self._set_blueprint_attribute(blueprint, name, value)
        sensor = self.world.spawn_actor(blueprint, transform, attach_to=self.ego_vehicle)
        sensor.listen(callback)
        self.sensors.append(sensor)

    def _on_camera(self, image: Any) -> None:
        try:
            array = np.frombuffer(image.raw_data, dtype=np.uint8)
            array = array.reshape((image.height, image.width, 4))
            self.latest_camera = array[:, :, :3].copy()
        except Exception:
            self.latest_camera = image

    def _on_lidar(self, lidar_data: Any) -> None:
        try:
            points = np.frombuffer(lidar_data.raw_data, dtype=np.float32).reshape((-1, 4))
            self.latest_lidar_points = points
            self._last_opponent_payload = self._build_obstacle_payload(points)
        except Exception:
            self.latest_lidar_points = None
            self._last_opponent_payload = []

    def _on_imu(self, imu_data: Any) -> None:
        if self.qcar_adapter is not None:
            self.qcar_adapter.update_imu(imu_data)

    def _on_collision(self, event: Any) -> None:
        self.latest_collision = event

    def _build_obstacle_payload(self, points: np.ndarray) -> List[float]:
        if points is None or points.size == 0:
            return []

        x = points[:, 0]
        y = points[:, 1]
        distances = np.hypot(x, y)
        mask = (
            (x > 0.0)
            & (np.abs(y) <= float(self.config.obstacle_lateral_window))
            & (distances >= float(self.config.obstacle_min_distance))
            & (distances <= float(self.config.obstacle_max_distance))
        )
        if not np.any(mask):
            return []

        selected_indices = np.where(mask)[0]
        nearest_idx = int(selected_indices[np.argmin(distances[selected_indices])])
        local_x = float(points[nearest_idx, 0])
        local_y = float(points[nearest_idx, 1])
        local_z = float(points[nearest_idx, 2])
        distance = float(distances[nearest_idx])

        try:
            lidar_transform = self.sensors[1].get_transform()
            world_loc = lidar_transform.transform(
                self.carla.Location(x=local_x, y=local_y, z=local_z)
            )
            obs_x = float(world_loc.x)
            obs_y = float(world_loc.y)
        except Exception:
            ego_transform = self.ego_vehicle.get_transform()
            yaw = math.radians(float(ego_transform.rotation.yaw))
            obs_x = float(ego_transform.location.x) + local_x * math.cos(yaw) - local_y * math.sin(yaw)
            obs_y = float(ego_transform.location.y) + local_x * math.sin(yaw) + local_y * math.cos(yaw)

        return [
            0.0,  # id
            obs_x,
            obs_y,
            0.4,  # size
            0.0,  # vx
            0.0,  # vy
            0.0,  # is_static
            1.0,  # is_visible
            distance,
            0.5,  # confidence
        ]

    def generate_route_waypoints(self) -> np.ndarray:
        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("Cannot generate route: CARLA map has no spawn points")

        current_location = self.ego_vehicle.get_location()
        goals = self._resolve_goal_transforms(spawn_points)

        points: List[Tuple[float, float]] = []
        start_location = current_location
        for goal_transform in goals:
            segment = self._trace_route(start_location, goal_transform.location)
            if points and segment:
                segment = segment[1:]
            points.extend(segment)
            start_location = goal_transform.location

        if len(points) < 2:
            points = self._straight_line_points(current_location, goals[-1].location)

        deduped = self._dedupe_points(points)
        if len(deduped) < 2:
            raise RuntimeError("Generated CARLA route has fewer than two waypoints")

        return np.asarray(deduped, dtype=float).T

    def _resolve_goal_transforms(self, spawn_points: Sequence[Any]) -> List[Any]:
        if self.config.goal_spawn_indices:
            indices = self.config.goal_spawn_indices
        else:
            indices = [int(self.config.spawn_point_index) + 10]
        return [spawn_points[int(idx) % len(spawn_points)] for idx in indices]

    def _trace_route(self, start_location: Any, end_location: Any) -> List[Tuple[float, float]]:
        try:
            from agents.navigation.global_route_planner import GlobalRoutePlanner

            planner = GlobalRoutePlanner(
                self.world.get_map(),
                float(self.config.route_sampling_resolution),
            )
            route = planner.trace_route(start_location, end_location)
            points = []
            for waypoint, _road_option in route:
                loc = waypoint.transform.location
                points.append((float(loc.x), float(loc.y)))
            if len(points) >= 2:
                return points
        except Exception as exc:
            self._log(f"GlobalRoutePlanner unavailable, using straight route fallback: {exc}")

        return self._straight_line_points(start_location, end_location)

    def _straight_line_points(self, start_location: Any, end_location: Any) -> List[Tuple[float, float]]:
        count = max(int(self.config.fallback_route_points), 2)
        xs = np.linspace(float(start_location.x), float(end_location.x), count)
        ys = np.linspace(float(start_location.y), float(end_location.y), count)
        return list(zip(xs.tolist(), ys.tolist()))

    @staticmethod
    def _dedupe_points(points: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
        deduped: List[Tuple[float, float]] = []
        for x, y in points:
            if not deduped:
                deduped.append((float(x), float(y)))
                continue
            prev_x, prev_y = deduped[-1]
            if math.hypot(float(x) - prev_x, float(y) - prev_y) > 1e-3:
                deduped.append((float(x), float(y)))
        return deduped

    def _draw_route(self, waypoint_sequence: np.ndarray) -> None:
        try:
            debug = self.world.debug
            z = float(self.ego_vehicle.get_location().z) + 0.25
            for idx in range(waypoint_sequence.shape[1] - 1):
                p0 = self.carla.Location(
                    x=float(waypoint_sequence[0, idx]),
                    y=float(waypoint_sequence[1, idx]),
                    z=z,
                )
                p1 = self.carla.Location(
                    x=float(waypoint_sequence[0, idx + 1]),
                    y=float(waypoint_sequence[1, idx + 1]),
                    z=z,
                )
                debug.draw_line(
                    p0,
                    p1,
                    thickness=0.08,
                    color=self.carla.Color(255, 80, 0),
                    life_time=30.0,
                )
        except Exception:
            pass

    def _log(self, message: str) -> None:
        if self.logger is not None:
            try:
                self.logger.logger.info(message)
                return
            except Exception:
                pass
        print(f"[CARLA] {message}")
