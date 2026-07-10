"""
Unit tests for BaseVehicleObserver and NullObserver.

Run from the qcar_refactor directory:
    python -m unittest test.unit_test_observer
"""
import sys
import os
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.types import VehicleStateEstimate, ControlCommand
from utils.control.observer.observer_base import ObserverBase, ObserverNull


class TestBaseVehicleObserverAbstract(unittest.TestCase):
    """Verify BaseVehicleObserver cannot be instantiated directly."""

    def test_cannot_instantiate_abstract(self):
        with self.assertRaises(TypeError):
            ObserverBase()  # pylint: disable=abstract-class-instantiated

    def test_concrete_subclass_must_implement_all(self):
        class Incomplete(ObserverBase):
            pass
        with self.assertRaises(TypeError):
            Incomplete()


class TestNullObserver(unittest.TestCase):
    """Tests for NullObserver — no-op observer for testing."""

    def setUp(self):
        self.obs = ObserverNull()

    def test_start_default_pose(self):
        self.obs.start()
        state = self.obs.get_latest()
        self.assertIsInstance(state, VehicleStateEstimate)
        self.assertEqual(state.x, 0.0)
        self.assertEqual(state.y, 0.0)
        self.assertEqual(state.theta, 0.0)
        self.assertGreater(state.timestamp, 0.0)

    def test_start_with_pose(self):
        self.obs.start(initial_pose=[1.0, 2.0, 0.5])
        state = self.obs.get_latest()
        self.assertEqual(state.x, 1.0)
        self.assertEqual(state.y, 2.0)
        self.assertEqual(state.theta, 0.5)

    def test_update_returns_zero_state(self):
        self.obs.start()
        sensor_data = {
            "motor_tach": 100.0,
            "gyro_z": 0.1,
            "accelerometer": None,
            "gps_valid": True,
            "gps_position": None,
            "sensor_timestamp": time.time(),
        }
        cmd = ControlCommand(throttle=0.5, steering=0.1, target_velocity=0.0)
        state = self.obs.update(sensor_data, dt=0.01, last_command=cmd)

        # NullObserver always returns zero position / velocity
        self.assertEqual(state.x, 0.0)
        self.assertEqual(state.y, 0.0)
        self.assertEqual(state.theta, 0.0)
        self.assertEqual(state.velocity, 0.0)
        self.assertEqual(state.acceleration, 0.0)
        # But it does forward gps_valid from sensor_data
        self.assertTrue(state.gps_valid)

    def test_update_preserves_gps_invalid(self):
        self.obs.start()
        sensor_data = {
            "motor_tach": 0.0,
            "gyro_z": 0.0,
            "accelerometer": None,
            "gps_valid": False,
            "gps_position": None,
            "sensor_timestamp": time.time(),
        }
        state = self.obs.update(sensor_data, dt=0.01)
        self.assertFalse(state.gps_valid)

    def test_get_latest_before_start_returns_zero(self):
        state = self.obs.get_latest()
        self.assertIsInstance(state, VehicleStateEstimate)
        self.assertEqual(state.x, 0.0)
        self.assertEqual(state.y, 0.0)
        self.assertEqual(state.theta, 0.0)
        self.assertEqual(state.velocity, 0.0)

    def test_get_latest_returns_last_estimate_without_recomputing(self):
        self.obs.start(initial_pose=[3.0, 4.0, 0.8])
        sensor_data = {
            "motor_tach": 0.0,
            "gyro_z": 0.0,
            "accelerometer": None,
            "gps_valid": True,
            "gps_position": None,
            "sensor_timestamp": time.time(),
        }
        self.obs.update(sensor_data, dt=0.01)

        # get_latest should now reflect the last update (zero state + gps_valid=True)
        state = self.obs.get_latest()
        self.assertEqual(state.x, 0.0)
        self.assertTrue(state.gps_valid)

    def test_stop_does_not_raise(self):
        self.obs.start()
        self.obs.update(
            {"motor_tach": 0.0, "gyro_z": 0.0, "accelerometer": None,
             "gps_valid": False, "gps_position": None,
             "sensor_timestamp": time.time()},
            dt=0.01,
        )
        self.obs.stop()  # should not raise

    def test_stop_then_get_latest_still_works(self):
        self.obs.start(initial_pose=[5.0, 6.0, 1.0])
        self.obs.stop()
        state = self.obs.get_latest()
        self.assertIsInstance(state, VehicleStateEstimate)
        self.assertEqual(state.x, 5.0)

    def test_update_with_missing_keys_defaults(self):
        """NullObserver uses .get() — missing keys should not crash."""
        self.obs.start()
        state = self.obs.update({}, dt=0.01)
        self.assertIsInstance(state, VehicleStateEstimate)
        self.assertFalse(state.gps_valid)  # defaults to False


if __name__ == "__main__":
    unittest.main()
