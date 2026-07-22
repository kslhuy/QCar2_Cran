"""Tests for the platform-neutral single-vehicle process building blocks."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.commands import VehicleCommand
from core.vehicle_config import ConfigError
from core.vehicle_process import VehicleProcessSpec, build_vehicle_process_runtime, run_vehicle_process


class TestVehicleProcess(unittest.TestCase):
    def test_builds_virtual_io_runtime_from_one_process_spec(self):
        route = [[0.0, 0.0], [1.0, 0.1], [2.0, 0.0]]
        runtime = build_vehicle_process_runtime(
            VehicleProcessSpec(
                vehicle_id=3,
                vehicle_config_file="config_vehicle_virtual.yaml",
                value_overrides={
                    "mission": {"path": route},
                    "modules": {"planner": {"path_source": route}},
                },
            )
        )

        self.assertEqual(runtime.config.vehicle_id, 3)
        self.assertEqual(runtime.io.__class__.__name__, "IOVirtual")

    def test_rejects_conflicting_vehicle_id_override(self):
        spec = VehicleProcessSpec(
            vehicle_id=3,
            vehicle_config_file="config_vehicle_virtual.yaml",
            value_overrides={"vehicle_id": 4},
        )

        with self.assertRaisesRegex(ConfigError, "conflicts"):
            build_vehicle_process_runtime(spec)

    def test_runs_start_step_and_shutdown_in_order(self):
        runtime = _FakeRuntime()
        samples = run_vehicle_process(runtime, cycles=2, dt=0.05)

        self.assertEqual(samples, ["sample-1", "sample-2"])
        self.assertEqual(runtime.calls, ["start", ("command", "START"), ("step", 0.05), ("step", 0.05), "shutdown"])


class _FakeRuntime:
    def __init__(self):
        self.calls = []
        self._steps = 0

    def start(self):
        self.calls.append("start")

    def handle_command(self, command: VehicleCommand):
        self.calls.append(("command", command.command_type.value))

    def step(self, dt):
        self._steps += 1
        self.calls.append(("step", dt))
        return f"sample-{self._steps}"

    def shutdown(self):
        self.calls.append("shutdown")


if __name__ == "__main__":
    unittest.main()
