"""Tests for the shared platform runner and launcher contracts."""

from contextlib import contextmanager
from dataclasses import dataclass
import os
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.vehicle_process import VehicleProcessSpec
from extra.platform.base import StartPolicy, run_platform_process
from extra.platform.launcher import (
    LocalProcessEndpoint,
    PlatformLauncher,
    RemoteProcessEndpoint,
)
from extra.platform.ros2.resources import ros2_resource_context
from extra.deployment.deployment_type import CommandResult, PreflightReport


class _Runtime:
    class Config:
        runtime = {"loop_rate_hz": 100}

    config = Config()

    def __init__(self, calls):
        self.calls = calls

    def start(self):
        self.calls.append("start")

    def handle_command(self, command):
        self.calls.append(("command", command.command_type.value))

    def step(self, dt=None):
        self.calls.append(("step", dt))
        return "sample"

    def shutdown(self):
        self.calls.append("shutdown")


class TestPlatformProcess(unittest.TestCase):
    def test_operator_policy_keeps_the_shared_runtime_ready_and_releases_resources_last(self):
        calls = []
        received_specs = []

        @contextmanager
        def resources():
            calls.append("resource_open")
            try:
                yield {"device": object()}
            finally:
                calls.append("resource_close")

        def build(spec):
            received_specs.append(spec)
            return _Runtime(calls)

        samples = run_platform_process(
            VehicleProcessSpec(4, "config_vehicle_virtual.yaml"),
            cycles=1,
            dt=0.1,
            start_policy=StartPolicy.OPERATOR_COMMAND,
            resource_context=resources(),
            runtime_builder=build,
        )

        self.assertEqual(samples, ["sample"])
        self.assertIn("device", received_specs[0].resources)
        self.assertEqual(
            calls,
            ["resource_open", "start", ("step", 0.1), "shutdown", "resource_close"],
        )

    def test_simulator_policy_preserves_the_existing_automatic_start(self):
        calls = []

        run_platform_process(
            VehicleProcessSpec(4, "config_vehicle_virtual.yaml"),
            cycles=1,
            dt=0.1,
            runtime_builder=lambda _spec: _Runtime(calls),
        )

        self.assertEqual(calls, ["start", ("command", "START"), ("step", 0.1), "shutdown"])


class TestRemotePlatformLauncher(unittest.TestCase):
    def test_preflights_all_endpoints_before_starting_any(self):
        calls = []

        @dataclass
        class Configuration:
            name: str

        class Client:
            def __init__(self, configuration):
                self.configuration = configuration

            def __enter__(self):
                return self

            def __exit__(self, *_):
                return None

            def preflight(self):
                calls.append(("preflight", self.configuration.name))
                return PreflightReport(True, self.configuration.name, "aarch64", "/dev/root", (), ())

            def start(self):
                calls.append(("start", self.configuration.name))
                return CommandResult("start", 0, self.configuration.name, "")

            def tail_logs(self, _lines):
                return CommandResult("logs", 0, f"{self.configuration.name} READY", "")

        report = PlatformLauncher(client_factory=Client).launch(
            [
                RemoteProcessEndpoint(0, "qcar-a", Configuration("qcar-a")),
                RemoteProcessEndpoint(1, "qcar-b", Configuration("qcar-b")),
            ],
            operation="start",
        )

        self.assertEqual(
            calls,
            [("preflight", "qcar-a"), ("preflight", "qcar-b"), ("start", "qcar-a"), ("start", "qcar-b")],
        )
        self.assertTrue(report.ok)
        self.assertEqual([item.endpoint for item in report.endpoints], ["qcar-a", "qcar-b"])
        self.assertEqual([item.logs for item in report.endpoints], ["qcar-a READY", "qcar-b READY"])
        self.assertEqual([item.shutdown for item in report.endpoints], ["not_requested", "not_requested"])

    def test_preflight_failure_is_reported_without_starting_any_endpoint(self):
        calls = []

        @dataclass
        class Configuration:
            name: str
            allowed: bool

        class Client:
            def __init__(self, configuration):
                self.configuration = configuration

            def __enter__(self):
                return self

            def __exit__(self, *_):
                return None

            def preflight(self):
                calls.append(("preflight", self.configuration.name))
                return PreflightReport(
                    self.configuration.allowed,
                    self.configuration.name,
                    "aarch64",
                    "/dev/root",
                    (),
                    (),
                )

            def start(self):
                calls.append(("start", self.configuration.name))
                return CommandResult("start", 0, "", "")

        report = PlatformLauncher(client_factory=Client).launch(
            [
                RemoteProcessEndpoint(0, "qcar-a", Configuration("qcar-a", True)),
                RemoteProcessEndpoint(1, "qcar-b", Configuration("qcar-b", False)),
            ],
            operation="start",
        )

        self.assertFalse(report.ok)
        self.assertEqual(calls, [("preflight", "qcar-a"), ("preflight", "qcar-b")])
        self.assertIn("another remote endpoint", report.endpoints[0].failure)
        self.assertEqual(report.endpoints[1].failure, "Remote preflight failed")


class TestLocalPlatformLauncher(unittest.TestCase):
    def test_local_worker_returns_the_common_lifecycle_result(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            ready_file = root / "vehicle-4.ready"
            start_file = root / "start.signal"
            stop_file = root / "stop.signal"
            script = (
                "from pathlib import Path; import sys; "
                "Path(sys.argv[1]).touch(); "
                "print('{\"vehicle_id\": 4, \"rows\": []}')"
            )
            report = PlatformLauncher(working_directory=root).launch(
                [
                    LocalProcessEndpoint(
                        vehicle_id=4,
                        endpoint="local:4",
                        command=(sys.executable, "-c", script, str(ready_file)),
                        ready_file=ready_file,
                    )
                ],
                operation="run",
                start_file=start_file,
                stop_file=stop_file,
                ready_timeout_s=2.0,
                stop_requested=lambda: False,
            )

        self.assertTrue(report.ok)
        result = report.endpoints[0]
        self.assertTrue(result.ready)
        self.assertTrue(result.healthy)
        self.assertEqual(result.shutdown, "completed")
        self.assertIn('"vehicle_id": 4', result.logs)


class TestRos2ResourceContext(unittest.TestCase):
    def test_ros2_context_owns_node_and_runtime_lifecycle_without_importing_ros2(self):
        calls = []

        class Node:
            def destroy_node(self):
                calls.append("destroy_node")

        class FakeRos2:
            def init(self):
                calls.append("init")

            def shutdown(self):
                calls.append("shutdown")

        with ros2_resource_context(
            node_name="vehicle_2",
            rclpy_module=FakeRos2(),
            node_factory=lambda name: calls.append(("create_node", name)) or Node(),
        ) as resources:
            self.assertIn("ros_node", resources)
            self.assertEqual(calls, ["init", ("create_node", "vehicle_2")])
        self.assertEqual(calls, ["init", ("create_node", "vehicle_2"), "destroy_node", "shutdown"])


if __name__ == "__main__":
    unittest.main()
