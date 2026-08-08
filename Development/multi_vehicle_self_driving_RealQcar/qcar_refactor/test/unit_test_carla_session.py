"""Fake-CARLA tests for synchronous session ownership and cleanup."""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from extra.platform.carla.session import CarlaSession


class _Settings:
    def __init__(self):
        self.synchronous_mode = False
        self.fixed_delta_seconds = None


class _Vector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Rotation:
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
        self.pitch, self.yaw, self.roll = pitch, yaw, roll


class _Transform:
    def __init__(self, location=None, rotation=None):
        self.location = location or _Vector()
        self.rotation = rotation or _Rotation()


class _VehicleControl:
    def __init__(self, throttle=0.0, steer=0.0, brake=0.0):
        self.throttle, self.steer, self.brake = throttle, steer, brake


class _Actor:
    def __init__(self, sensor=False):
        self.sensor = sensor
        self.destroyed = False
        self.stopped = False
        self.callback = None
        self.controls = []

    def listen(self, callback):
        self.callback = callback

    def stop(self):
        self.stopped = True

    def destroy(self):
        self.destroyed = True

    def apply_control(self, control):
        self.controls.append(control)

    def get_transform(self):
        return _Transform(_Vector(2.0, 3.0, 0.0), _Rotation(yaw=10.0))

    def get_velocity(self):
        return _Vector(1.0, 0.0, 0.0)


class _World:
    def __init__(self):
        self.settings = _Settings()
        self.applied_settings = []
        self.ego = None
        self.last_spawn_transform = None
        self.sensors = []
        self.frame = 0

    def get_settings(self):
        return self.settings

    def apply_settings(self, settings):
        self.settings = settings
        self.applied_settings.append((settings.synchronous_mode, settings.fixed_delta_seconds))

    def get_blueprint_library(self):
        return self

    def find(self, blueprint):
        return blueprint

    def get_map(self):
        return self

    def get_spawn_points(self):
        return [_Transform()]

    def try_spawn_actor(self, blueprint, transform):
        self.last_spawn_transform = transform
        self.ego = _Actor()
        return self.ego

    def spawn_actor(self, blueprint, transform, attach_to=None):
        sensor = _Actor(sensor=True)
        self.sensors.append(sensor)
        return sensor

    def tick(self):
        self.frame += 1
        return self.frame

    def get_snapshot(self):
        return type("Snapshot", (), {"timestamp": type("Timestamp", (), {"elapsed_seconds": self.frame * 0.05})()})()

    def wait_for_tick(self, timeout):
        return type(
            "Snapshot",
            (),
            {"frame": self.frame, "timestamp": type("Timestamp", (), {"elapsed_seconds": self.frame * 0.05})()},
        )()


class _Client:
    def __init__(self, world):
        self.world = world
        self.timeout = None

    def set_timeout(self, timeout):
        self.timeout = timeout

    def get_world(self):
        return self.world


class _Carla:
    Location = _Vector
    Rotation = _Rotation
    Transform = _Transform
    VehicleControl = _VehicleControl


_CONFIG = {
    "implementation": "carla",
    "fixed_delta_seconds": 0.05,
    "warmup_ticks": 1,
    "ego_blueprint": "vehicle.test",
    "sensors": {"imu": {"enabled": True, "blueprint": "sensor.other.imu", "transform": {}}},
}


class TestCarlaSession(unittest.TestCase):
    def setUp(self):
        self.world = _World()
        self.session = CarlaSession(_CONFIG, client=_Client(self.world), carla_api=_Carla)

    def test_start_tick_callback_and_close_own_resources(self):
        self.session.start()
        self.assertTrue(self.world.settings.synchronous_mode)
        self.assertEqual(self.world.frame, 1)

        imu = type(
            "Imu", (),
            {"timestamp": 0.1, "frame": 2, "accelerometer": _Vector(1.0, 2.0, 3.0), "gyroscope": _Vector(0.0, 0.0, 0.4)},
        )()
        self.world.sensors[0].callback(imu)
        snapshot = self.session.tick()
        self.assertEqual(snapshot.accelerometer, (1.0, -2.0, 3.0))
        self.assertAlmostEqual(snapshot.gyro_z, -0.4)

        self.session.close()
        self.session.close()
        self.assertTrue(self.world.sensors[0].stopped)
        self.assertTrue(self.world.sensors[0].destroyed)
        self.assertTrue(self.world.ego.destroyed)
        self.assertFalse(self.world.settings.synchronous_mode)
        self.assertEqual(self.world.ego.controls[-1].brake, 1.0)

    def test_explicit_spawn_transform_does_not_require_map(self):
        config = {
            **_CONFIG,
            "spawn_transform": {"x": 4.0, "y": -2.0, "z": 0.5, "yaw": 90.0},
        }
        self.world.get_map = lambda: (_ for _ in ()).throw(RuntimeError("map unavailable"))
        session = CarlaSession(config, client=_Client(self.world), carla_api=_Carla)
        try:
            session.start()
            transform = self.world.last_spawn_transform
            self.assertEqual((transform.location.x, transform.location.y, transform.location.z), (4.0, -2.0, 0.5))
            self.assertEqual(transform.rotation.yaw, 90.0)
        finally:
            session.close()

    def test_follower_waits_for_frames_without_owning_world_settings(self):
        config = {**_CONFIG, "tick_owner": False, "warmup_ticks": 0}
        session = CarlaSession(config, client=_Client(self.world), carla_api=_Carla)
        try:
            session.start()
            self.assertFalse(self.world.settings.synchronous_mode)
            self.world.frame = 4
            snapshot = session.tick()
            self.assertEqual(snapshot.frame, 4)
            self.assertEqual(snapshot.timestamp, 0.2)
        finally:
            session.close()
        self.assertEqual(self.world.applied_settings, [])

    def test_lidar_callback_queues_a_copied_raw_measurement(self):
        config = {
            **_CONFIG,
            "sensors": {
                **_CONFIG["sensors"],
                "lidar": {
                    "enabled": True,
                    "blueprint": "sensor.lidar.ray_cast",
                    "transform": {},
                    "scan": {"frame_id": "laser", "bin_count": 8, "range_min_m": 0.05, "range_max_m": 20.0},
                },
            },
        }
        session = CarlaSession(config, client=_Client(self.world), carla_api=_Carla)
        try:
            session.start()
            lidar = self.world.sensors[1]
            points = np.array([3.0, 0.0, 0.0, 1.0], dtype=np.float32)
            lidar.callback(type("Lidar", (), {"timestamp": 0.15, "frame": 3, "raw_data": points.tobytes()})())

            measurements = session.drain_lidar_measurements()

            self.assertEqual(len(measurements), 1)
            self.assertEqual(measurements[0].frame, 3)
            self.assertEqual(measurements[0].timestamp_s, 0.15)
            self.assertEqual(measurements[0].raw_data, points.tobytes())
            self.assertEqual(session.drain_lidar_measurements(), ())
        finally:
            session.close()


if __name__ == "__main__":
    unittest.main()
