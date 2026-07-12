"""Fake-actor tests for the narrow CARLA IO adapter."""

import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.types import ControlCommand
from utils.io.io_carla import IOCarla
from extra.simulation.carla.session import CarlaSensorSnapshot


class _Vector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Transform:
    def __init__(self):
        self.location = _Vector(4.0, 2.0, 0.0)
        self.rotation = type("Rotation", (), {"yaw": 90.0})()

    def get_forward_vector(self):
        return _Vector(0.0, 1.0, 0.0)


class _Actor:
    def __init__(self):
        self.controls = []

    def get_transform(self):
        return _Transform()

    def get_velocity(self):
        return _Vector(0.0, 3.0, 0.0)

    def apply_control(self, control):
        self.controls.append(control)


class _Session:
    def __init__(self):
        self.ego_actor = _Actor()

    def get_snapshot(self):
        return CarlaSensorSnapshot(5, 1.25, (1.0, -2.0, 3.0), -0.4)

    def make_vehicle_control(self, **values):
        return values


_CONFIG = {
    "write": {"max_throttle": 1.0, "max_steering": 0.48},
    "read": {"sensor_rate_hz": 20, "gps_rate_hz": 20},
    "steering_scale": 2.0,
    "steering_sign": -1.0,
}


class TestIOCarla(unittest.TestCase):
    def setUp(self):
        self.session = _Session()
        self.io = IOCarla(_CONFIG, self.session)

    def test_reads_session_snapshot_and_actor_pose(self):
        self.io.read_to_cache()
        data = self.io.read()

        self.assertAlmostEqual(data.motor_tach, 3.0)
        self.assertAlmostEqual(data.gyro_z, -0.4)
        np.testing.assert_array_equal(data.accelerometer, [1.0, -2.0, 3.0])
        np.testing.assert_allclose(data.gps_position, [4.0, -2.0, -math.pi / 2.0])
        self.assertEqual(data.sensor_timestamp, 1.25)
        self.assertTrue(data.gps_valid)

    def test_maps_throttle_brake_and_normalized_steering(self):
        self.io.write(ControlCommand(0.5, 0.3, 0.0, "test"))
        self.io.write(ControlCommand(-0.4, -0.3, 0.0, "test"))

        forward, braking = self.session.ego_actor.controls
        self.assertEqual(forward, {"throttle": 0.5, "steer": -0.6, "brake": 0.0})
        self.assertEqual(braking, {"throttle": 0.0, "steer": 0.6, "brake": 0.4})


if __name__ == "__main__":
    unittest.main()
