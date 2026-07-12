"""Headless tests for the minimal one-vehicle runtime."""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.types import ControlCommand, GuiCommand, PlannerTarget, SensorData, VehicleStateEstimate
from core.vehicle_config import ConfigVehicle
from core.module_factory import build_vehicle_modules
from core.vehicle_logic import VehicleRuntime
from core.vehicle_state_machine import State
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
        return PlannerTarget(1.0, 0.0, 0.0, 0.5, self.finished)


class _Controller:
    def __init__(self, events):
        self.events = events

    def reset(self):
        self.events.append("controller.reset")

    def compute(self, state, target, dt):
        self.events.append("controller.compute")
        return ControlCommand(0.4, 0.1, target.target_velocity, "test_controller")


class _V2V:
    def __init__(self, events, fail_start=False):
        self.events = events
        self.fail_start = fail_start

    def start(self):
        self.events.append("v2v.start")
        if self.fail_start:
            raise RuntimeError("v2v failed")

    def process_received_messages(self):
        self.events.append("v2v.receive")

    def broadcast_local_state(self, state):
        self.events.append("v2v.broadcast")
        return False

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
        config = _config()
        self.io = _RecordingNullIO(config.module("io"), self.events)
        self.observer = _Observer(self.events)
        self.planner = _Planner(self.events)
        self.controller = _Controller(self.events)
        self.v2v = _V2V(self.events)
        self.runtime = VehicleRuntime(config, self.io, self.observer, self.planner, self.controller, self.v2v)

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
        self.assertLess(self.events.index("observer.update"), self.events.index("v2v.receive"))
        self.assertLess(self.events.index("v2v.receive"), self.events.index("planner.update"))
        self.assertLess(self.events.index("planner.update"), self.events.index("controller.compute"))

    def test_stop_forces_zero_command(self):
        self.runtime.start()
        self.runtime.handle_command(GuiCommand("START", {}))
        self.runtime.step(dt=0.01)
        self.runtime.handle_command(GuiCommand("STOP", {}))

        self.assertEqual(self.runtime.state_machine.state, State.STOPPED)
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
