"""Localhost integration coverage for TCP bridge -> runtime -> acknowledgement flow."""

import os
import sys
import time
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.commands import CommandType, VehicleCommand
from core.types import ControlInput, ControllerReference, SensorData, VehicleStateEstimate
from core.vehicle_config import ConfigVehicle
from core.vehicle_logic import VehicleRuntime
from extra.ground_station.server import GroundStationServer
from utils.fleet import FleetFormationBuilder, FleetManager, FleetMember, FleetPolicy, FleetRegistry
from utils.ground_station.bridge_tcp import GroundStationClientBridge
from utils.ground_station.runtime_facade import GroundStationRuntimeFacade
from utils.io.io_base import IONull
from utils.v2v.v2v_base import V2VNull
from utils.control.controller.controller_manual import ControllerManual
from utils.control.controller.controller_manager import ControllerManager


def _wait_for(predicate, timeout_s: float = 3.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


class _Observer:
    def start(self):
        return None

    def stop(self):
        return None

    def update(self, sensor_data: SensorData, dt: float, last_command: ControlInput) -> VehicleStateEstimate:
        return VehicleStateEstimate(
            sensor_data.sensor_timestamp,
            1.0,
            2.0,
            0.1,
            0.4,
            0.0,
            True,
            valid=True,
        )


class _Planner:
    def __init__(self):
        self.target_velocity = 0.4

    def reset(self):
        return None

    def update(self, state: VehicleStateEstimate) -> ControllerReference:
        return ControllerReference(2.0, 2.0, 0.0, self.target_velocity, False)

    def set_target_velocity(self, target_velocity: float) -> None:
        self.target_velocity = float(target_velocity)

    def load_path(self, path_source) -> None:
        raise AssertionError("SET_PATH is not part of this localhost smoke test")


class _Controller:
    supports_fleet_reference = False

    def reset(self):
        return None

    def compute(self, state: VehicleStateEstimate, target: ControllerReference, dt: float) -> ControlInput:
        return ControlInput(0.2, 0.0, target.target_velocity, "integration_controller")


class TestGroundStationRuntimeIntegration(unittest.TestCase):
    def setUp(self):
        self.server = GroundStationServer("127.0.0.1", 0, max_frame_bytes=4096)
        self.server.start()
        self.config = ConfigVehicle(
            vehicle_id=4,
            runtime={"loop_rate_hz": 20},
            mission={},
            modules={
                "model": {"wheelbase": 0.3},
                "io": {
                    "implementation": "null",
                    "write": {"max_throttle": 1.0, "max_steering": 1.0},
                    "read": {"sensor_rate_hz": 20, "gps_rate_hz": 20},
                    "timing": {"loop_rate_hz": 20},
                },
                "observer": {"implementation": "null"},
                "planner": {"implementation": "null"},
                "controller": {"implementation": "null"},
                "v2v": {"implementation": "null", "enabled": False},
                "simulation": {"implementation": "null"},
                "ground_station": {
                    "implementation": "tcp_client",
                    "enabled": True,
                    "command_batch_size": 4,
                },
            },
        )
        bridge_config = {
            "implementation": "tcp_client",
            "enabled": True,
            "server_host": "127.0.0.1",
            "server_port": self.server.port,
            "connect_timeout_s": 0.5,
            "reconnect_interval_s": 0.05,
            "command_queue_size": 8,
            "outbound_queue_size": 8,
            "max_frame_bytes": 4096,
            "monitoring_rate_hz": 100.0,
            "command_batch_size": 4,
        }
        self.bridge = GroundStationClientBridge(bridge_config, vehicle_id=4)
        self.planner = _Planner()
        policy = FleetPolicy.from_mapping(
            {
                "following_policy": "direct_predecessor",
                "communication": {
                    "topology": "predecessor_chain",
                    "edge_direction": "directed",
                    "ego_estimate_rate_hz": 10.0,
                    "peer_timeout_s": 1.0,
                },
            }
        )
        formation = FleetFormationBuilder().build(
            "ground_station_test_fleet",
            (
                FleetMember.from_mapping({"vehicle_id": 4, "role": "leader", "member_order": 0}),
                FleetMember.from_mapping({"vehicle_id": 5, "role": "follower", "member_order": 1}),
            ),
            policy,
        )
        v2v = V2VNull({}, vehicle_id=4)
        fleet = FleetManager(FleetRegistry(formation), vehicle_id=4)
        fleet.attach_transport(v2v)
        self.runtime = VehicleRuntime(
            self.config,
            IONull(self.config.module("io"), vehicle_id=4),
            _Observer(),
            self.planner,
            ControllerManager(
                _Controller(),
                {
                    "manual": lambda: ControllerManual(
                        {"command_timeout_s": 0.5, "max_throttle": 0.35, "max_steering": 0.30},
                        vehicle_id=4,
                    )
                },
            ),
            v2v,
            GroundStationRuntimeFacade(self.bridge, command_batch_size=4),
            fleet=fleet,
        )

    def tearDown(self):
        self.runtime.shutdown()
        self.server.stop()

    def test_server_command_is_applied_and_acknowledged_by_runtime(self):
        self.runtime.start()
        self.assertTrue(_wait_for(lambda: len(self.server.session_rows()) == 1))

        delivery = self.server.send_command(4, VehicleCommand(CommandType.START, command_id="start-4"))
        self.assertTrue(delivery.accepted, delivery.reason)
        for _ in range(30):
            self.runtime.step(dt=0.05)
            rows = self.server.session_rows()
            if rows and rows[0]["last_command_result"].get("command_id") == "start-4":
                break
            time.sleep(0.01)

        row = self.server.session_rows()[0]
        self.assertEqual(row["last_command_result"]["outcome"], "applied")
        self.assertEqual(row["last_command_result"]["runtime_state"], "RUNNING")
        self.assertTrue(
            _wait_for(
                lambda: (
                    self.server.session_rows()[0]["snapshot"] is not None
                    and self.server.session_rows()[0]["snapshot"]["runtime_state"] == "RUNNING"
                )
            ),
            "monitoring snapshot did not arrive after the command acknowledgement",
        )
        row = self.server.session_rows()[0]
        self.assertEqual(row["snapshot"]["runtime_state"], "RUNNING")
        self.assertAlmostEqual(row["snapshot"]["velocity_mps"], 0.4)

        delivery = self.server.send_command(
            4,
            VehicleCommand(CommandType.SET_VELOCITY, {"velocity": 0.7}, command_id="velocity-4"),
        )
        self.assertTrue(delivery.accepted, delivery.reason)
        for _ in range(30):
            self.runtime.step(dt=0.05)
            rows = self.server.session_rows()
            if rows and rows[0]["last_command_result"].get("command_id") == "velocity-4":
                break
            time.sleep(0.01)

        self.assertAlmostEqual(self.planner.target_velocity, 0.7)
        self.assertEqual(self.server.session_rows()[0]["last_command_result"]["outcome"], "applied")

    def test_server_can_build_and_cancel_a_configured_fleet(self):
        self.runtime.start()
        self.assertTrue(_wait_for(lambda: len(self.server.session_rows()) == 1))
        self.assertTrue(self.server.send_command(4, VehicleCommand(CommandType.START, command_id="start-fleet-4")).accepted)
        self._run_until_ack("start-fleet-4")

        self.assertTrue(
            self.server.send_command(4, VehicleCommand(CommandType.BUILD_FLEET, command_id="build-fleet-4")).accepted
        )
        build = self._run_until_ack("build-fleet-4")
        self.assertEqual(build["outcome"], "applied")
        self._run_until_fleet_phase({"building", "active"})

        self.assertTrue(
            self.server.send_command(4, VehicleCommand(CommandType.CANCEL_FLEET, command_id="cancel-fleet-4")).accepted
        )
        cancel = self._run_until_ack("cancel-fleet-4")
        self.assertEqual(cancel["outcome"], "applied")
        self.assertEqual(cancel["runtime_state"], "STOPPED")

    def test_server_manual_input_drives_the_vehicle_without_direct_runtime_access(self):
        self.runtime.start()
        self.assertTrue(_wait_for(lambda: len(self.server.session_rows()) == 1))
        self.assertTrue(self.server.send_command(4, VehicleCommand(CommandType.START, command_id="start-manual-4")).accepted)
        self._run_until_ack("start-manual-4")
        self.assertTrue(
            self.server.send_command(4, VehicleCommand(CommandType.ENABLE_MANUAL, command_id="enable-manual-4")).accepted
        )
        self._run_until_ack("enable-manual-4")
        self.assertTrue(
            self.server.send_command(
                4,
                VehicleCommand(
                    CommandType.MANUAL_INPUT,
                    {"throttle": 0.2, "steering": 0.1},
                    command_id="manual-input-4",
                ),
            ).accepted
        )

        for _ in range(30):
            telemetry = self.runtime.step(dt=0.05)
            if telemetry.command.source == "manual_controller" and telemetry.command.throttle > 0.0:
                break
            time.sleep(0.01)
        else:
            self.fail("manual input did not reach the vehicle control loop")

        self.assertAlmostEqual(telemetry.command.throttle, 0.2)
        self.assertAlmostEqual(telemetry.command.steering, 0.1)
        # Publish a second manual-mode sample after the bridge's rate limiter
        # has elapsed; the preceding automatic sample may have been recent.
        time.sleep(0.02)
        self.runtime.step(dt=0.05)
        self.assertTrue(
            _wait_for(
                lambda: self.server.session_rows()[0]["snapshot"] is not None
                and self.server.session_rows()[0]["snapshot"]["control_mode"] == "manual"
            )
        )

    def _run_until_ack(self, command_id: str) -> dict:
        for _ in range(30):
            self.runtime.step(dt=0.05)
            rows = self.server.session_rows()
            if rows and rows[0]["last_command_result"].get("command_id") == command_id:
                return rows[0]["last_command_result"]
            time.sleep(0.01)
        self.fail(f"No acknowledgement for {command_id}")

    def _run_until_fleet_phase(self, expected_phases: set[str]) -> None:
        for _ in range(20):
            self.runtime.step(dt=0.05)
            rows = self.server.session_rows()
            if rows and rows[0]["snapshot"] is not None and rows[0]["snapshot"]["fleet_phase"] in expected_phases:
                return
            time.sleep(0.01)
        self.fail(f"No monitoring snapshot with fleet phase in {expected_phases}")


if __name__ == "__main__":
    unittest.main()
