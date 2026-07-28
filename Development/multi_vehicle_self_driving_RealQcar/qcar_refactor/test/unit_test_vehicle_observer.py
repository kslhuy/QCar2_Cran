"""Unit tests for the observer base contract and null observer."""

import os
import sys
import time
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.vehicle_types import ControlInput, SensorData, VehicleStateEstimate
from utils.control.observer.observer_base import ObserverBase, ObserverNull


def _sensor_data(gps_valid=False):
    return SensorData(
        motor_tach=0.0,
        gyro_z=0.0,
        accelerometer=np.zeros(3),
        sensor_timestamp=time.time(),
        gps_valid=gps_valid,
        gps_position=np.zeros(3),
        gps_timestamp=time.time(),
    )


class TestBaseVehicleObserverAbstract(unittest.TestCase):
    def test_cannot_instantiate_abstract(self):
        with self.assertRaises(TypeError):
            ObserverBase({})

    def test_concrete_subclass_must_implement_all_methods(self):
        class Incomplete(ObserverBase):
            pass

        with self.assertRaises(TypeError):
            Incomplete({})


class TestNullObserver(unittest.TestCase):
    def setUp(self):
        self.observer = ObserverNull({}, vehicle_id=2)

    def test_init_stores_common_constructor_values(self):
        self.assertEqual(self.observer._config, {})
        self.assertEqual(self.observer._vehicle_id, 2)

    def test_start_uses_default_or_supplied_pose(self):
        self.observer.start()
        self.assertEqual(self.observer.get_latest().x, 0.0)

        self.observer.start(initial_pose=[1.0, 2.0, 0.5])
        state = self.observer.get_latest()
        self.assertEqual((state.x, state.y, state.theta), (1.0, 2.0, 0.5))

    def test_update_accepts_sensor_data_and_returns_safe_state(self):
        self.observer.start()
        command = ControlInput(throttle=0.5, steering=0.1, target_velocity=0.0)
        state = self.observer.update(_sensor_data(gps_valid=True), dt=0.01, last_command=command)

        self.assertIsInstance(state, VehicleStateEstimate)
        self.assertEqual((state.x, state.y, state.theta, state.velocity), (0.0, 0.0, 0.0, 0.0))
        self.assertTrue(state.gps_valid)

    def test_update_requires_sensor_data(self):
        self.observer.start()
        with self.assertRaises(AttributeError):
            self.observer.update({}, dt=0.01)

    def test_stop_preserves_latest_estimate(self):
        self.observer.start(initial_pose=[5.0, 6.0, 1.0])
        self.observer.stop()
        self.assertEqual(self.observer.get_latest().x, 5.0)


if __name__ == "__main__":
    unittest.main()
