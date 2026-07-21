"""
Unit tests for the minimal controller.

Run from the qcar_refactor directory:
    python -m unittest test.unit_test_controller
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.types import ControlCommand, ControllerReference, VehicleStateEstimate
from utils.control.controller import (
    ControllerBase,
    ControllerFleet2D,
    ControllerFleetBase,
    ControllerFleetLongitudinal,
    ControllerNull,
    ControllerSimple,
)


SIMPLE_CONFIG = {
    "kp_velocity": 0.2,
    "ki_velocity": 0.0,
    "kd_velocity": 0.0,
    "feedforward_gain": 0.0,
    "steering_gain": 1.0,
    "max_throttle": 0.10,
    "min_throttle": -0.10,
    "max_steering": 0.48,
    "integral_limit": 1.0,
}


def _state(x=0.0, y=0.0, theta=0.0, velocity=0.0):
    return VehicleStateEstimate(
        timestamp=1.0,
        x=float(x),
        y=float(y),
        theta=float(theta),
        velocity=float(velocity),
        acceleration=0.0,
        gps_valid=True,
    )


def _target(x=1.0, y=0.0, theta=0.0, velocity=0.6, finished=False):
    return ControllerReference(
        target_x=float(x),
        target_y=float(y),
        target_theta=float(theta),
        target_velocity=float(velocity),
        is_finished=bool(finished),
    )


class TestBaseController(unittest.TestCase):
    def test_cannot_instantiate_base_controller(self):
        with self.assertRaises(TypeError):
            ControllerBase({})

    def test_null_controller_returns_safe_zero_command(self):
        controller = ControllerNull({}, vehicle_id=3)
        command = controller.compute(_state(), _target(), dt=0.01)

        self.assertEqual(controller._config, {})
        self.assertEqual(controller._vehicle_id, 3)
        self.assertIsInstance(command, ControlCommand)
        self.assertEqual(command.throttle, 0.0)
        self.assertEqual(command.steering, 0.0)
        self.assertEqual(command.target_velocity, 0.0)
        self.assertFalse(controller.supports_fleet_reference)

    def test_cannot_instantiate_base_fleet_controller(self):
        with self.assertRaises(TypeError):
            ControllerFleetBase({})


class TestSimplePathController(unittest.TestCase):
    def test_forward_target_produces_positive_throttle_and_zero_steering(self):
        controller = ControllerSimple(SIMPLE_CONFIG)

        command = controller.compute(
            _state(x=0.0, y=0.0, theta=0.0, velocity=0.0),
            _target(x=2.0, y=0.0, velocity=0.5),
            dt=0.1,
        )

        self.assertGreater(command.throttle, 0.0)
        self.assertAlmostEqual(command.steering, 0.0)
        self.assertAlmostEqual(command.target_velocity, 0.5)
        self.assertEqual(command.source, "simple_path_controller")

    def test_left_target_produces_positive_steering(self):
        controller = ControllerSimple(SIMPLE_CONFIG)

        command = controller.compute(
            _state(x=0.0, y=0.0, theta=0.0),
            _target(x=1.0, y=1.0),
            dt=0.1,
        )

        self.assertGreater(command.steering, 0.0)

    def test_right_target_produces_negative_steering(self):
        controller = ControllerSimple(SIMPLE_CONFIG)

        command = controller.compute(
            _state(x=0.0, y=0.0, theta=0.0),
            _target(x=1.0, y=-1.0),
            dt=0.1,
        )

        self.assertLess(command.steering, 0.0)

    def test_finished_target_returns_zero_command_and_resets_integral(self):
        controller = ControllerSimple({**SIMPLE_CONFIG, "ki_velocity": 1.0})
        controller.compute(_state(velocity=0.0), _target(velocity=1.0), dt=0.5)
        self.assertNotEqual(controller.integral_error, 0.0)

        command = controller.compute(
            _state(),
            _target(finished=True),
            dt=0.1,
        )

        self.assertEqual(command.throttle, 0.0)
        self.assertEqual(command.steering, 0.0)
        self.assertEqual(command.target_velocity, 0.0)
        self.assertEqual(command.source, "path_finished")
        self.assertEqual(controller.integral_error, 0.0)

    def test_output_is_clipped_to_limits(self):
        controller = ControllerSimple({
            **SIMPLE_CONFIG,
            "kp_velocity": 10.0,
            "steering_gain": 10.0,
        })

        command = controller.compute(
            _state(x=0.0, y=0.0, theta=0.0, velocity=-10.0),
            _target(x=0.0, y=10.0, velocity=10.0),
            dt=0.1,
        )

        self.assertLessEqual(command.throttle, 0.1)
        self.assertGreaterEqual(command.throttle, -0.1)
        self.assertLessEqual(command.steering, 0.48)
        self.assertGreaterEqual(command.steering, -0.48)

    def test_negative_velocity_target_is_treated_as_stop(self):
        controller = ControllerSimple(SIMPLE_CONFIG)

        command = controller.compute(
            _state(velocity=0.5),
            _target(velocity=-1.0),
            dt=0.1,
        )

        self.assertEqual(command.target_velocity, 0.0)
        self.assertLessEqual(command.throttle, 0.0)


class TestFleet2DController(unittest.TestCase):
    def test_follower_accelerates_when_behind_and_uses_virtual_target(self):
        controller = ControllerFleet2D({
            "desired_gap_m": 1.0, "time_headway_s": 0.0, "gap_gain": 0.5,
            "kp_velocity": 1.0, "max_velocity": 2.0,
            "max_throttle": 0.2, "min_throttle": -0.2, "max_steering": 0.4,
        })
        command = controller.compute(_state(x=0.0, velocity=0.0), _target(x=3.0, velocity=0.5), 0.1)

        self.assertGreater(command.throttle, 0.0)
        self.assertAlmostEqual(command.steering, 0.0)
        self.assertEqual(command.source, "fleet_2d_controller")
        self.assertTrue(controller.supports_fleet_reference)

    def test_follower_command_is_bounded(self):
        controller = ControllerFleet2D({
            "desired_gap_m": 0.5, "gap_gain": 10.0, "kp_velocity": 10.0,
            "max_velocity": 10.0, "max_throttle": 0.1, "min_throttle": -0.1,
            "max_steering": 0.2, "steering_gain": 10.0,
        })
        command = controller.compute(_state(x=0.0, y=0.0), _target(x=10.0, y=10.0, velocity=10.0), 0.1)

        self.assertLessEqual(command.throttle, 0.1)
        self.assertGreaterEqual(command.throttle, -0.1)
        self.assertLessEqual(abs(command.steering), 0.2)


class TestFleetLongitudinalController(unittest.TestCase):
    def test_follower_uses_gap_response_without_steering(self):
        controller = ControllerFleetLongitudinal({
            "desired_gap_m": 1.0, "time_headway_s": 0.0, "gap_gain": 0.5,
            "kp_velocity": 1.0, "max_velocity": 2.0,
            "max_throttle": 0.2, "min_throttle": -0.2, "max_steering": 0.4,
        })

        command = controller.compute(_state(x=0.0, velocity=0.0), _target(x=3.0, velocity=0.5), 0.1)

        self.assertGreater(command.throttle, 0.0)
        self.assertEqual(command.steering, 0.0)
        self.assertEqual(command.source, "fleet_longitudinal_controller")
        self.assertTrue(controller.supports_fleet_reference)


if __name__ == "__main__":
    unittest.main()
