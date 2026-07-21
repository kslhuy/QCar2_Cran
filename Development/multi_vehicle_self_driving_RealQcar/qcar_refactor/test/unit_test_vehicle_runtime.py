"""Headless tests for the minimal one-vehicle runtime."""

import os
import sys
import time
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.types import ControlCommand, GuiCommand, ControllerReference, SensorData, V2VMessage, VehicleStateEstimate
from core.vehicle_config import ConfigVehicle
from core.module_factory import build_vehicle_modules
from core.vehicle_logic import VehicleRuntime
from core.vehicle_state_machine import State
from utils.fleet import (
    FleetFormationBuilder,
    FleetManager,
    FleetMember,
    FleetPolicy,
    FleetRegistry,
    encode_vehicle_state_estimate,
)
from utils.io.io_base import IONull


def _config():
    return ConfigVehicle(
        vehicle_id=0,
        runtime={"loop_rate_hz": 100},
        mission={},
        modules={
            "model": {"wheelbase": 0.3},
            "io": {
                "implementation": "null",
                "write": {"max_throttle": 1.0, "max_steering": 1.0},
                "read": {"sensor_rate_hz": 100, "gps_rate_hz": 10},
                "timing": {"loop_rate_hz": 100},
            },
            "observer": {"implementation": "null", "wheelbase": 0.3},
            "planner": {"implementation": "null"},
            "controller": {"implementation": "null"},
            "v2v": {"implementation": "null", "enabled": False},
            "simulation": {"implementation": "null"},
            "ground_station": {"implementation": "null"},
        },
    )


def _fleet_manager(vehicle_id: int = 0):
    policy = FleetPolicy.from_mapping(
        {
            "following_policy": "direct_predecessor",
            "communication": {
                "topology": "loop",
                "edge_direction": "directed",
                "ego_estimate_rate_hz": 20,
                "peer_timeout_s": 0.5,
            },
        }
    )
    formation = FleetFormationBuilder().build(
        "runtime_test_fleet",
        (
            FleetMember.from_mapping({"vehicle_id": 0, "role": "leader", "member_order": 0}),
            FleetMember.from_mapping({"vehicle_id": 1, "role": "follower", "member_order": 1}),
        ),
        policy,
    )
    return FleetManager(FleetRegistry(formation), vehicle_id=vehicle_id)


class _Observer:
    def __init__(self, events, fail=False):
        self.events = events
        self.fail = fail

    def start(self):
        self.events.append("observer.start")

    def update(self, sensor_data, dt, last_command):
        self.events.append("observer.update")
        if self.fail:
            raise RuntimeError("observer failed")
        return VehicleStateEstimate(sensor_data.sensor_timestamp, 0.0, 0.0, 0.0, 0.0, 0.0, False)

    def stop(self):
        self.events.append("observer.stop")


class _Planner:
    def __init__(self, events, finished=False):
        self.events = events
        self.finished = finished

    def reset(self):
        self.events.append("planner.reset")

    def update(self, state):
        self.events.append("planner.update")
        return ControllerReference(1.0, 0.0, 0.0, 0.5, self.finished)


class _Controller:
    supports_fleet_reference = False

    def __init__(self, events):
        self.events = events

    def reset(self):
        self.events.append("controller.reset")

    def compute(self, state, target, dt):
        self.events.append("controller.compute")
        return ControlCommand(0.4, 0.1, target.target_velocity, "test_controller")


class _FleetController(_Controller):
    supports_fleet_reference = True

    def compute(self, state, target, dt):
        self.events.append("controller.compute_fleet_reference")
        return ControlCommand(0.3, 0.2, target.target_velocity, "test_fleet_controller")


class _V2V:
    def __init__(self, events, fail_start=False):
        self.events = events
        self.fail_start = fail_start
        self.received = []
        self.publications = []

    def start(self):
        self.events.append("v2v.start")
        if self.fail_start:
            raise RuntimeError("v2v failed")

    def publish(self, message_type, payload, target_vehicle_ids=None):
        self.events.append("v2v.publish")
        self.publications.append((message_type, payload, target_vehicle_ids))
        return False

    def drain_received(self):
        self.events.append("v2v.drain")
        received = self.received
        self.received = []
        return received

    def stop(self):
        self.events.append("v2v.stop")


class _RecordingNullIO(IONull):
    def __init__(self, config, events):
        super().__init__(config)
        self.events = events
        self.commands = []

    def _hardware_write(self, throttle, steering):
        self.events.append("io.write")
        self.commands.append((throttle, steering))

    def read_to_cache(self):
        self.events.append("io.read")
        super().read_to_cache()

    def close(self):
        self.events.append("io.close")
        super().close()


class TestVehicleRuntime(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.config = _config()
        self.io = _RecordingNullIO(self.config.module("io"), self.events)
        self.observer = _Observer(self.events)
        self.planner = _Planner(self.events)
        self.controller = _Controller(self.events)
        self.v2v = _V2V(self.events)
        self.runtime = VehicleRuntime(self.config, self.io, self.observer, self.planner, self.controller, self.v2v)

    def _rebuild_runtime(self, *, fleet, controller=None):
        self.controller = controller or self.controller
        self.runtime = VehicleRuntime(
            self.config,
            self.io,
            self.observer,
            self.planner,
            self.controller,
            self.v2v,
            fleet=fleet,
        )

    def test_factory_builds_headless_runtime(self):
        config = _config()
        modules = build_vehicle_modules(config)
        runtime = VehicleRuntime(
            config,
            modules.io,
            modules.observer,
            modules.planner,
            modules.controller,
            modules.v2v,
        )
        runtime.start()
        self.assertEqual(runtime.state_machine.state, State.READY)
        runtime.shutdown()

    def test_start_enters_ready_and_blocks_drive_until_start(self):
        self.runtime.start()
        telemetry = self.runtime.step(dt=0.01)

        self.assertEqual(telemetry.state, State.READY)
        self.assertEqual(telemetry.command.throttle, 0.0)
        self.assertNotIn("controller.compute", self.events)

    def test_start_allows_controller_command(self):
        self.runtime.start()
        self.runtime.handle_command(GuiCommand("START", {}))
        telemetry = self.runtime.step(dt=0.01)

        self.assertEqual(telemetry.state, State.RUNNING)
        self.assertEqual(telemetry.command.throttle, 0.4)
        self.assertEqual(self.io.commands[-1], (0.4, 0.1))
        self.assertLess(self.events.index("observer.update"), self.events.index("planner.update"))
        self.assertLess(self.events.index("planner.update"), self.events.index("controller.compute"))

    def test_stop_forces_zero_command(self):
        self.runtime.start()
        self.runtime.handle_command(GuiCommand("START", {}))
        self.runtime.step(dt=0.01)
        self.runtime.handle_command(GuiCommand("STOP", {}))

        self.assertEqual(self.runtime.state_machine.state, State.STOPPED)
        self.assertEqual(self.io.commands[-1], (0.0, 0.0))

    def test_fleet_build_and_cancel_use_existing_safe_stop_path(self):
        self._rebuild_runtime(fleet=_fleet_manager())
        self.runtime.start()
        self.runtime.handle_command(GuiCommand("START", {}))

        self.runtime.handle_command(GuiCommand("BUILD_FLEET", {}))
        self.assertEqual(self.runtime.fleet.status().phase.value, "building")

        self.runtime.handle_command(GuiCommand("CANCEL_FLEET", {}))
        self.assertEqual(self.runtime.state_machine.state, State.STOPPED)
        self.assertEqual(self.runtime.fleet.status().phase.value, "disabled")
        self.assertEqual(self.io.commands[-1], (0.0, 0.0))

    def test_fleet_build_is_rejected_when_vehicle_is_not_running(self):
        self._rebuild_runtime(fleet=_fleet_manager())
        self.runtime.start()
        self.runtime.handle_command(GuiCommand("BUILD_FLEET", {}))

        self.assertEqual(self.runtime.fleet.status().phase.value, "disabled")

    def test_fleet_runtime_consumes_generic_messages_and_publishes_to_peers(self):
        self._rebuild_runtime(fleet=_fleet_manager())
        peer = _fleet_manager(vehicle_id=1)
        self.runtime.start()
        self.runtime.handle_command(GuiCommand("START", {}))
        self.runtime.handle_command(GuiCommand("BUILD_FLEET", {}))
        payload = encode_vehicle_state_estimate(
            VehicleStateEstimate(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, True),
            peer.status(),
        )
        self.v2v.received.append(
            V2VMessage(1, "VEHICLE_STATE_ESTIMATE", payload, 1, 0.0, 0, time.monotonic(), 0)
        )

        telemetry = self.runtime.step(dt=0.01)

        self.assertEqual(telemetry.state, State.RUNNING)
        self.assertEqual(self.runtime.fleet.status().phase.value, "active")
        self.assertEqual(self.v2v.publications[-1][0], "VEHICLE_STATE_ESTIMATE")
        self.assertEqual(self.v2v.publications[-1][2], [1])

    def test_runtime_selects_fleet_controller_from_manager_reference(self):
        self._rebuild_runtime(fleet=_fleet_manager(vehicle_id=1), controller=_FleetController(self.events))
        leader = _fleet_manager(vehicle_id=0)
        self.runtime.start()
        self.runtime.handle_command(GuiCommand("START", {}))
        self.runtime.handle_command(GuiCommand("BUILD_FLEET", {}))
        self.assertTrue(leader.request_build(vehicle_running=True, now_monotonic=time.monotonic()))
        payload = encode_vehicle_state_estimate(
            VehicleStateEstimate(1.0, 1.0, 0.0, 0.0, 0.4, 0.0, True),
            leader.status(),
        )
        self.v2v.received.append(
            V2VMessage(0, "VEHICLE_STATE_ESTIMATE", payload, 1, 0.0, 0, time.monotonic(), 0)
        )

        telemetry = self.runtime.step(dt=0.01)

        self.assertEqual(telemetry.state, State.RUNNING)
        self.assertEqual(telemetry.command.source, "test_fleet_controller")
        self.assertIn("controller.compute_fleet_reference", self.events)
        self.assertNotIn("controller.compute", self.events)

    def test_emergency_stop_cancels_an_active_fleet_before_writing_zero(self):
        self._rebuild_runtime(fleet=_fleet_manager())
        peer = _fleet_manager(vehicle_id=1)
        self.runtime.start()
        self.runtime.handle_command(GuiCommand("START", {}))
        self.runtime.handle_command(GuiCommand("BUILD_FLEET", {}))
        payload = encode_vehicle_state_estimate(
            VehicleStateEstimate(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, True), peer.status()
        )
        self.v2v.received.append(
            V2VMessage(1, "VEHICLE_STATE_ESTIMATE", payload, 1, 0.0, 0, time.monotonic(), 0)
        )
        self.runtime.step(dt=0.01)
        self.assertEqual(self.runtime.fleet.status().phase.value, "active")

        self.runtime.handle_command(GuiCommand("EMERGENCY_STOP", {}))

        self.assertEqual(self.runtime.state_machine.state, State.STOPPED)
        self.assertEqual(self.runtime.fleet.status().phase.value, "disabled")
        self.assertEqual(self.io.commands[-1], (0.0, 0.0))

    def test_shutdown_cancels_an_active_fleet_before_releasing_modules(self):
        self._rebuild_runtime(fleet=_fleet_manager())
        peer = _fleet_manager(vehicle_id=1)
        self.runtime.start()
        self.runtime.handle_command(GuiCommand("START", {}))
        self.runtime.handle_command(GuiCommand("BUILD_FLEET", {}))
        payload = encode_vehicle_state_estimate(
            VehicleStateEstimate(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, True), peer.status()
        )
        self.v2v.received.append(
            V2VMessage(1, "VEHICLE_STATE_ESTIMATE", payload, 1, 0.0, 0, time.monotonic(), 0)
        )
        self.runtime.step(dt=0.01)
        self.assertEqual(self.runtime.fleet.status().phase.value, "active")

        self.runtime.shutdown()

        self.assertEqual(self.runtime.fleet.status().phase.value, "disabled")
        self.assertEqual(self.io.commands[-1], (0.0, 0.0))

    def test_emergency_stop_forces_zero_command(self):
        self.runtime.start()
        self.runtime.handle_command(GuiCommand("START", {}))
        self.runtime.step(dt=0.01)
        self.runtime.handle_command(GuiCommand("EMERGENCY_STOP", {}))

        self.assertEqual(self.runtime.state_machine.state, State.STOPPED)
        self.assertEqual(self.io.commands[-1], (0.0, 0.0))

    def test_path_completion_stops_and_writes_zero(self):
        self.planner.finished = True
        self.runtime.start()
        self.runtime.handle_command(GuiCommand("START", {}))
        telemetry = self.runtime.step(dt=0.01)

        self.assertEqual(telemetry.state, State.STOPPED)
        self.assertEqual(telemetry.command.throttle, 0.0)
        self.assertNotIn("controller.compute", self.events)

    def test_module_failure_marks_error_and_writes_zero(self):
        self.observer.fail = True
        self.runtime.start()
        self.runtime.handle_command(GuiCommand("START", {}))

        with self.assertRaisesRegex(RuntimeError, "control loop failed"):
            self.runtime.step(dt=0.01)
        self.assertEqual(self.runtime.state_machine.state, State.ERROR)
        self.assertEqual(self.io.commands[-1], (0.0, 0.0))

    def test_startup_failure_rolls_back_started_dependencies(self):
        config = _config()
        events = []
        io = _RecordingNullIO(config.module("io"), events)
        runtime = VehicleRuntime(
            config,
            io,
            _Observer(events),
            _Planner(events),
            _Controller(events),
            _V2V(events, fail_start=True),
        )

        with self.assertRaisesRegex(RuntimeError, "startup failed"):
            runtime.start()
        self.assertEqual(runtime.state_machine.state, State.ERROR)
        self.assertIn("observer.stop", events)
        self.assertIn("io.close", events)

    def test_shutdown_stops_modules_before_closing_io(self):
        self.runtime.start()
        self.runtime.shutdown()

        self.assertLess(self.events.index("v2v.stop"), self.events.index("io.close"))
        self.assertLess(self.events.index("observer.stop"), self.events.index("io.close"))

    def test_invalid_dt_is_rejected(self):
        self.runtime.start()
        with self.assertRaisesRegex(ValueError, "dt"):
            self.runtime.step(dt=0.0)

    def test_simulation_starts_ticks_before_io_and_closes_last(self):
        class _Simulation:
            def start(inner_self):
                self.events.append("simulation.start")

            def tick(inner_self):
                self.events.append("simulation.tick")

            def close(inner_self):
                self.events.append("simulation.close")

        self.runtime.simulation = _Simulation()
        self.runtime.start()
        self.runtime.step(dt=0.01)
        self.runtime.shutdown()

        self.assertLess(self.events.index("simulation.start"), self.events.index("observer.start"))
        self.assertLess(self.events.index("simulation.tick"), self.events.index("io.read"))
        self.assertLess(self.events.index("io.close"), self.events.index("simulation.close"))


if __name__ == "__main__":
    unittest.main()
