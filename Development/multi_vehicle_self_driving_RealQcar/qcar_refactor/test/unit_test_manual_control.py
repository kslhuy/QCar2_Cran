"""Unit tests for the vehicle-side manual controller."""

from __future__ import annotations

import os
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.command_handler import VehicleCommandHandler
from core.commands import CommandOutcome, CommandType, VehicleCommand
from core.vehicle_state_machine import StateMachine
from core.types import ControllerReference, VehicleStateEstimate
from utils.control.controller.controller_manual import ControllerManual


def _controller() -> ControllerManual:
    return ControllerManual(
        {"command_timeout_s": 0.25, "max_throttle": 0.35, "max_steering": 0.30},
        vehicle_id=2,
    )


class _Planner:
    def set_target_velocity(self, velocity):
        return None

    def load_path(self, path):
        return None


class TestControllerManual(unittest.TestCase):
    def test_clips_fresh_input_and_times_out_using_monotonic_time(self):
        controller = _controller()
        controller.set_input(0.9, 0.9, now_monotonic=10.0)

        state = VehicleStateEstimate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, True)
        target = ControllerReference(0.0, 0.0, 0.0, 0.0)
        with patch("utils.control.controller.controller_manual.time.monotonic", return_value=10.20):
            fresh = controller.compute(state, target, 0.01)
        self.assertAlmostEqual(controller.input_age_s(now_monotonic=10.20), 0.20)
        with patch("utils.control.controller.controller_manual.time.monotonic", return_value=10.26):
            timed_out = controller.compute(state, target, 0.01)

        self.assertAlmostEqual(fresh.throttle, 0.35)
        self.assertAlmostEqual(fresh.steering, 0.30)
        self.assertEqual(timed_out.throttle, 0.0)

    def test_handler_requires_running_state_before_manual_activation(self):
        state_machine = StateMachine()
        handler = VehicleCommandHandler(2, state_machine, _Planner(), manual_controller_available=True)

        rejected = handler.handle(VehicleCommand(CommandType.ENABLE_MANUAL), now_monotonic=1.0)
        state_machine.mark_ready()
        handler.handle(VehicleCommand(CommandType.START), now_monotonic=1.0)
        accepted = handler.handle(VehicleCommand(CommandType.ENABLE_MANUAL), now_monotonic=2.0)
        input_result = handler.handle(
            VehicleCommand(CommandType.MANUAL_INPUT, {"throttle": 0.2, "steering": 0.1}),
            now_monotonic=2.1,
        )

        self.assertEqual(rejected.result.outcome, CommandOutcome.REJECTED)
        self.assertEqual(rejected.result.reason_code, "manual_requires_running_vehicle")
        self.assertEqual(accepted.result.outcome, CommandOutcome.APPLIED)
        self.assertEqual(accepted.controller_profile, "manual")
        self.assertEqual(input_result.result.outcome, CommandOutcome.APPLIED)
        self.assertEqual(input_result.manual_input, (0.2, 0.1))


if __name__ == "__main__":
    unittest.main()
