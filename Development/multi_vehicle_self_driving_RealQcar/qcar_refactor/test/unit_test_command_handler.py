"""Unit tests for the core transport-independent command handler."""

from __future__ import annotations

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.command_handler import VehicleCommandHandler
from core.commands import CommandOutcome, CommandType, VehicleCommand
from core.vehicle_state_machine import State, StateMachine
from utils.fleet.fleet_types import FleetCommandResult


class _Planner:
    def __init__(self) -> None:
        self.velocity = None
        self.path = None

    def set_target_velocity(self, velocity: float) -> None:
        self.velocity = velocity

    def load_path(self, path: str) -> None:
        self.path = path


class _SDCSPlanner(_Planner):
    def has_profile(self, name: str) -> bool:
        return name == "sdcs_map"


class _Fleet:
    def __init__(self, result: FleetCommandResult) -> None:
        self.result = result
        self.commands = []

    def handle_command(self, command, *, vehicle_running, now_monotonic):
        self.commands.append((command, vehicle_running, now_monotonic))
        return self.result

    def is_follower(self):
        return False


class TestVehicleCommandHandler(unittest.TestCase):
    def setUp(self) -> None:
        self.state_machine = StateMachine()
        self.state_machine.mark_ready()
        self.planner = _Planner()

    def test_start_changes_state_and_requests_control_reset(self):
        handler = VehicleCommandHandler(3, self.state_machine, self.planner)

        handling = handler.handle(VehicleCommand(CommandType.START), now_monotonic=1.0)

        self.assertEqual(handling.result.outcome, CommandOutcome.APPLIED)
        self.assertEqual(handling.result.runtime_state, "RUNNING")
        self.assertTrue(handling.reset_control)
        self.assertEqual(self.state_machine.state, State.RUNNING)

    def test_set_velocity_changes_planner_without_runtime_safety_action(self):
        handler = VehicleCommandHandler(3, self.state_machine, self.planner)

        handling = handler.handle(VehicleCommand(CommandType.SET_VELOCITY, {"velocity": 0.8}))

        self.assertEqual(handling.result.outcome, CommandOutcome.APPLIED)
        self.assertEqual(self.planner.velocity, 0.8)
        self.assertFalse(handling.require_safe_stop)

    def test_target_mismatch_is_rejected_without_dispatch(self):
        handler = VehicleCommandHandler(3, self.state_machine, self.planner)

        handling = handler.handle(VehicleCommand(CommandType.START, target_vehicle_id=4))

        self.assertEqual(handling.result.outcome, CommandOutcome.REJECTED)
        self.assertEqual(handling.result.reason_code, "vehicle_id_mismatch")
        self.assertEqual(self.state_machine.state, State.READY)

    def test_fleet_lifecycle_command_is_rejected_without_a_fleet_manager(self):
        handler = VehicleCommandHandler(3, self.state_machine, self.planner)

        handling = handler.handle(VehicleCommand(CommandType.BUILD_FLEET))

        self.assertEqual(handling.result.outcome, CommandOutcome.REJECTED)
        self.assertEqual(handling.result.reason_code, "fleet_manager_unavailable")

    def test_fleet_cancel_requests_runtime_safe_stop_after_state_transition(self):
        self.state_machine.handle_command(VehicleCommand(CommandType.START))
        fleet = _Fleet(FleetCommandResult(handled=True, stop_vehicle=True, reason="fleet_cancel"))
        handler = VehicleCommandHandler(3, self.state_machine, self.planner, fleet)

        handling = handler.handle(VehicleCommand(CommandType.CANCEL_FLEET), now_monotonic=2.0)

        self.assertTrue(handling.require_safe_stop)
        self.assertEqual(handling.safe_stop_reason, "fleet_cancel")
        self.assertEqual(self.state_machine.state, State.STOPPED)
        self.assertTrue(fleet.commands[0][1])

    def test_enable_sdcs_map_requires_ready_and_returns_runtime_planner_action(self):
        handler = VehicleCommandHandler(3, self.state_machine, _SDCSPlanner())

        handling = handler.handle(
            VehicleCommand(CommandType.ENABLE_SDCS_MAP, {"nodes": [0, 2, 4], "loop": 2})
        )

        self.assertEqual(handling.result.outcome, CommandOutcome.APPLIED)
        self.assertEqual(handling.planner_profile, "sdcs_map")
        self.assertEqual(handling.sdcs_node_sequence, (0, 2, 4))
        self.assertEqual(handling.sdcs_loop, 2)
        self.assertTrue(handling.reset_control)

    def test_disable_sdcs_map_stops_and_restores_configured_planner(self):
        handler = VehicleCommandHandler(3, self.state_machine, _SDCSPlanner())
        self.state_machine.handle_command(VehicleCommand(CommandType.START))

        handling = handler.handle(VehicleCommand(CommandType.DISABLE_SDCS_MAP))

        self.assertEqual(handling.result.outcome, CommandOutcome.APPLIED)
        self.assertEqual(handling.planner_profile, "configured")
        self.assertTrue(handling.require_safe_stop)
        self.assertEqual(self.state_machine.state, State.STOPPED)

    def test_running_fleet_leader_may_change_sdcs_route_but_follower_cannot(self):
        self.state_machine.handle_command(VehicleCommand(CommandType.START))
        fleet = _Fleet(FleetCommandResult(handled=False))
        fleet.phase = "active"
        handler = VehicleCommandHandler(3, self.state_machine, _SDCSPlanner(), fleet)
        command = VehicleCommand(CommandType.ENABLE_SDCS_MAP, {"nodes": [0, 2, 4], "loop": "inf"})

        leader = handler.handle(command)

        self.assertEqual(leader.result.outcome, CommandOutcome.APPLIED)
        self.assertTrue(leader.require_safe_stop)
        fleet.is_follower = lambda: True
        follower = handler.handle(command)
        self.assertEqual(follower.result.outcome, CommandOutcome.REJECTED)
        self.assertEqual(follower.result.reason_code, "sdcs_map_rejected_for_fleet_follower")


if __name__ == "__main__":
    unittest.main()
