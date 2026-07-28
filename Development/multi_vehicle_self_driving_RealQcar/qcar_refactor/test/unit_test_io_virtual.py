"""Unit tests for the deterministic virtual vehicle IO adapter."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.vehicle_types import ControlInput
from utils.io.io_virtual import IOVirtual


_CONFIG = {
    "write": {"max_throttle": 1.0, "max_steering": 0.48},
    "read": {"sensor_rate_hz": 50, "gps_rate_hz": 10},
    "dt": 0.02,
    "wheelbase": 0.3,
    "gps_period_steps": 1,
    "motor_tach_noise_std": 0.0,
    "gyro_noise_std": 0.0,
    "accel_noise_std": 0.0,
    "gps_xy_noise_std": 0.0,
    "gps_theta_noise_std": 0.0,
}


class TestIOVirtual(unittest.TestCase):
    def setUp(self):
        self.io = IOVirtual(_CONFIG)

    def test_throttle_advances_longitudinal_state(self):
        self.io.write(ControlInput(0.5, 0.0, 0.5, "test"))
        for _ in range(50):
            self.io.read_to_cache()

        x, y, theta, velocity, _ = self.io.true_state()
        self.assertGreater(x, 0.0)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(theta, 0.0, places=6)
        self.assertGreater(velocity, 0.0)

    def test_steering_produces_lateral_motion_and_yaw(self):
        self.io.write(ControlInput(0.6, 0.25, 0.6, "test"))
        for _ in range(100):
            self.io.read_to_cache()

        _, y, theta, velocity, _ = self.io.true_state()
        self.assertGreater(velocity, 0.0)
        self.assertGreater(y, 0.0)
        self.assertGreater(theta, 0.0)

    def test_sensor_snapshot_matches_noise_free_state(self):
        self.io.write(ControlInput(0.5, 0.1, 0.5, "test"))
        self.io.read_to_cache()
        sensor = self.io.read()
        x, y, theta, velocity, _ = self.io.true_state()

        self.assertAlmostEqual(sensor.motor_tach, velocity)
        self.assertTrue(sensor.gps_valid)
        self.assertAlmostEqual(sensor.gps_position[0], x)
        self.assertAlmostEqual(sensor.gps_position[1], y)
        self.assertAlmostEqual(sensor.gps_position[2], theta)

    def test_stop_sets_next_actuator_request_to_zero(self):
        self.io.write(ControlInput(0.5, 0.2, 0.5, "test"))
        self.io.stop()
        command = self.io.get_last_command()

        self.assertEqual(command.throttle, 0.0)
        self.assertEqual(command.steering, 0.0)


if __name__ == "__main__":
    unittest.main()
