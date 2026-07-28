"""Unit tests for the stable control-utility manager interfaces."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.vehicle_types import ControllerReference, SensorData, VehicleStateEstimate
from utils.control.managers import ObserverManager, PathPlannerManager


class _Observer:
    def __init__(self):
        self.started = False
        self.stopped = False
        self.latest = VehicleStateEstimate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False)

    def start(self, initial_pose=None):
        self.started = True

    def update(self, sensor_data, dt, last_command=None):
        self.latest = VehicleStateEstimate(sensor_data.sensor_timestamp, 1.0, 2.0, 0.0, 0.3, 0.0, True)
        return self.latest

    def get_latest(self):
        return self.latest

    def stop(self):
        self.stopped = True


class _Planner:
    def __init__(self):
        self.path = None
        self.target_velocity = 0.0
        self.reset_count = 0

    def load_path(self, path_source):
        self.path = path_source

    def reset(self):
        self.reset_count += 1

    def update(self, state):
        return ControllerReference(state.x, state.y, state.theta, self.target_velocity)

    def set_target_velocity(self, target_velocity):
        self.target_velocity = float(target_velocity)

    def is_finished(self):
        return False


class _SDCSPlanner(_Planner):
    def __init__(self):
        super().__init__()
        self.nodes = None
        self.loop = None

    def set_node_sequence(self, nodes, *, loop=0):
        self.nodes = tuple(nodes)
        self.loop = loop


def _sensor_data():
    return SensorData(0.0, 0.0, [0.0, 0.0, 0.0], 2.0, True, [1.0, 2.0, 0.0], 2.0)


class TestControlUtilityManagers(unittest.TestCase):
    def test_observer_manager_delegates_lifecycle_and_update(self):
        observer = _Observer()
        manager = ObserverManager(observer)

        manager.start()
        estimate = manager.update(_sensor_data(), 0.02)
        manager.stop()

        self.assertTrue(observer.started)
        self.assertTrue(observer.stopped)
        self.assertEqual(estimate, manager.get_latest())

    def test_path_planner_manager_delegates_and_selects_lazy_profile(self):
        configured = _Planner()
        alternate = _Planner()
        manager = PathPlannerManager(configured, {"alternate": lambda: alternate})

        manager.load_path([[0.0, 0.0]])
        manager.set_target_velocity(0.4)
        manager.reset()
        self.assertEqual(configured.path, [[0.0, 0.0]])
        self.assertEqual(configured.target_velocity, 0.4)
        self.assertEqual(configured.reset_count, 1)

        manager.select("alternate")
        self.assertEqual(manager.active_name, "alternate")
        self.assertIs(manager.restore_configured(), configured)

    def test_path_planner_manager_exposes_sdcs_routes_only_for_supporting_profile(self):
        configured = _Planner()
        sdcs = _SDCSPlanner()
        manager = PathPlannerManager(configured, {"sdcs_map": lambda: sdcs})

        with self.assertRaisesRegex(ValueError, "does not support"):
            manager.set_node_sequence([0, 2], loop=1)
        manager.select("sdcs_map")
        manager.set_node_sequence([0, 2, 4], loop="inf")

        self.assertEqual(sdcs.nodes, (0, 2, 4))
        self.assertEqual(sdcs.loop, "inf")
