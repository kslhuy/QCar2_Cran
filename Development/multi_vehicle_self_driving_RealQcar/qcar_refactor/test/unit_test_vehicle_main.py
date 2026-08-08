"""Tests for the standalone vehicle command-line entry point."""

import os
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core import vehicle_main


class _Runtime:
    def __init__(self):
        self.config = SimpleNamespace(runtime={"loop_rate_hz": 100.0})
        self.calls = []

    def start(self):
        self.calls.append("start")

    def step(self):
        self.calls.append("step")

    def shutdown(self):
        self.calls.append("shutdown")


class TestVehicleMain(unittest.TestCase):
    def test_builds_runtime_through_a_process_spec_and_preserves_operator_startup(self):
        runtime = _Runtime()
        with (
            patch("core.vehicle_main.build_vehicle_process_runtime", return_value=runtime) as build_runtime,
            patch("core.vehicle_main.time.sleep"),
        ):
            result = vehicle_main.main(
                [
                    "--vehicle-config", "config_vehicle_virtual.yaml",
                    "--vehicle-id", "7",
                    "--ground-station", "null",
                    "--ground-station-host", "10.0.0.2",
                    "--ground-station-port", "6000",
                    "--cycles", "1",
                ]
            )

        self.assertEqual(result, 0)
        spec = build_runtime.call_args.args[0]
        self.assertEqual(spec.vehicle_id, 7)
        self.assertEqual(spec.vehicle_config_file, "config_vehicle_virtual.yaml")
        self.assertEqual(spec.selection_overrides, {"ground_station": "null"})
        self.assertEqual(
            spec.value_overrides,
            {"modules": {"ground_station": {"server_host": "10.0.0.2", "server_port": 6000}}},
        )
        self.assertEqual(runtime.calls, ["start", "step", "shutdown"])

    def test_keeps_the_profile_vehicle_id_when_no_cli_override_is_given(self):
        runtime = _Runtime()
        with (
            patch("core.vehicle_main.build_vehicle_process_runtime", return_value=runtime) as build_runtime,
            patch("core.vehicle_main.time.sleep"),
        ):
            vehicle_main.main(["--headless", "--cycles", "1"])

        spec = build_runtime.call_args.args[0]
        self.assertIsNone(spec.vehicle_id)
        self.assertEqual(spec.vehicle_config_file, "config_vehicle_headless.yaml")
        self.assertEqual(runtime.calls, ["start", "step", "shutdown"])


if __name__ == "__main__":
    unittest.main()
