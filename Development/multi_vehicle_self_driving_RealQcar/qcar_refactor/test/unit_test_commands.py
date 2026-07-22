"""Unit tests for the transport-independent vehicle command contract."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.commands import (
    CommandError,
    CommandOutcome,
    CommandResult,
    CommandSource,
    CommandType,
    VehicleCommand,
)


class TestVehicleCommand(unittest.TestCase):
    def test_round_trip_preserves_typed_command_fields(self):
        command = VehicleCommand(
            CommandType.SET_VELOCITY,
            {"velocity": 0.6},
            command_id="velocity-1",
            source=CommandSource.GROUND_STATION,
            issued_at_epoch_s=123.0,
            target_vehicle_id=2,
        )

        restored = VehicleCommand.from_mapping(command.to_mapping())

        self.assertEqual(restored, command)
        with self.assertRaises(TypeError):
            restored.payload["velocity"] = 0.0

    def test_rejects_unknown_commands_and_invalid_payloads(self):
        with self.assertRaisesRegex(CommandError, "Unsupported command type"):
            VehicleCommand.from_mapping({"command_type": "MANUAL"})
        with self.assertRaisesRegex(CommandError, "non-negative"):
            VehicleCommand(CommandType.SET_VELOCITY, {"velocity": -0.1})
        with self.assertRaisesRegex(CommandError, "requires a non-empty string"):
            VehicleCommand(CommandType.SET_PATH, {"path": ""})
        with self.assertRaisesRegex(CommandError, "Unsupported payload fields"):
            VehicleCommand(CommandType.START, {"velocity": 0.1})

    def test_result_round_trip_retains_acknowledgement_identity(self):
        result = CommandResult(
            command_id="start-1",
            vehicle_id=3,
            outcome=CommandOutcome.APPLIED,
            runtime_state="RUNNING",
            reason_code="",
            reason="",
        )

        self.assertEqual(CommandResult.from_mapping(result.to_mapping()), result)


if __name__ == "__main__":
    unittest.main()
