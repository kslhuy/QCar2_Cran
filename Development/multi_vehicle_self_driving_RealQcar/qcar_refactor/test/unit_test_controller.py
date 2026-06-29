"""
Unit tests for the minimal controller.

Run from the qcar_refactor directory:
    python -m unittest test.unit_test_controller
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.Types import ControlCommand, PlannerTarget, VehicleStateEstimate
from utils.Control.Controller import BaseController, NullController, SimplePathController


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
    return PlannerTarget(
        target_x=float(x),
        target_y=float(y),
        target_theta=float(theta),
        target_velocity=float(velocity),
        is_finished=bool(finished),
    )


class TestBaseController(unittest.TestCase):
    def test_cannot_instantiate_base_controller(self):
        with self.assertRaises(TypeError):
            BaseController()

    def test_null_controller_returns_safe_zero_command(self):
        controller = NullController()
        command = controller.compute(_state(), _target(), dt=0.01)

        self.assertIsInstance(command, ControlCommand)
        self.assertEqual(command.throttle, 0.0)
        self.assertEqual(command.steering, 0.0)
        self.assertEqual(command.target_velocity, 0.0)


class TestSimplePathController(unittest.TestCase):
    def test_forward_target_produces_positive_throttle_and_zero_steering(self):
        controller = SimplePathController(
            kp_velocity=0.2,
            max_throttle=0.1,
            max_steering=0.48,
        )

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
        controller = SimplePathController(steering_gain=1.0, max_steering=0.48)

        command = controller.compute(
            _state(x=0.0, y=0.0, theta=0.0),
            _target(x=1.0, y=1.0),
            dt=0.1,
        )

        self.assertGreater(command.steering, 0.0)

    def test_right_target_produces_negative_steering(self):
        controller = SimplePathController(steering_gain=1.0, max_steering=0.48)

        command = controller.compute(
            _state(x=0.0, y=0.0, theta=0.0),
            _target(x=1.0, y=-1.0),
            dt=0.1,
        )

        self.assertLess(command.steering, 0.0)

    def test_finished_target_returns_zero_command_and_resets_integral(self):
        controller = SimplePathController(ki_velocity=1.0, integral_limit=1.0)
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
        controller = SimplePathController(
            kp_velocity=10.0,
            steering_gain=10.0,
            max_throttle=0.1,
            min_throttle=-0.1,
            max_steering=0.48,
        )

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
        controller = SimplePathController(kp_velocity=0.2)

        command = controller.compute(
            _state(velocity=0.5),
            _target(velocity=-1.0),
            dt=0.1,
        )

        self.assertEqual(command.target_velocity, 0.0)
        self.assertLessEqual(command.throttle, 0.0)


if __name__ == "__main__":
    unittest.main()
