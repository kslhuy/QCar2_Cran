"""
Unit tests for the minimal path planner.

Run from the qcar_refactor directory:
    python -m unittest test.unit_test_path_planner
"""

import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from utils.control.path_planner import PathPlannerBase, PathPlannerNull, PathPlannerStatic
from core.types import PlannerTarget, VehicleStateEstimate


class _SilentLogger:
    def info(self, message: str) -> None:
        pass
    def warning(self, message: str) -> None:
        pass


def _state(x=0.0, y=0.0, theta=0.0, velocity=0.0):
    return VehicleStateEstimate(
        timestamp=1.0,
        x=float(x),
        y=float(y),
        theta=float(theta),
        velocity=float(velocity),
        acceleration=0.0,
        gps_valid=True,
    )


class TestBasePathPlanner(unittest.TestCase):
    def test_cannot_instantiate_base_planner(self):
        with self.assertRaises(TypeError):
            PathPlannerBase({})

    def test_null_planner_returns_safe_finished_target(self):
        planner = PathPlannerNull({"target_velocity": 0.0}, vehicle_id=4)
        target = planner.update(_state(x=2.0, y=3.0, theta=0.5))

        self.assertEqual(planner._vehicle_id, 4)
        self.assertIsInstance(target, PlannerTarget)
        self.assertTrue(target.is_finished)
        self.assertEqual(target.target_velocity, 0.0)
        self.assertEqual(target.target_x, 2.0)
        self.assertEqual(target.target_y, 3.0)


class TestStaticWaypointPlanner(unittest.TestCase):
    def test_loads_waypoints_from_iterable(self):
        planner = PathPlannerStatic({
            "path_source": [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
            "lookahead_distance": 0.5,
            "target_velocity": 0.6,
        })

        self.assertFalse(planner.is_finished())
        self.assertEqual(planner.waypoints.shape, (3, 3))
        self.assertAlmostEqual(planner.waypoints[0, 2], 0.0)

    def test_returns_lookahead_target(self):
        planner = PathPlannerStatic({
            "path_source": [(0.0, 0.0), (0.25, 0.0), (1.0, 0.0)],
            "lookahead_distance": 0.5,
            "target_velocity": 0.7,
        })

        target = planner.update(_state(x=0.0, y=0.0))

        self.assertIsInstance(target, PlannerTarget)
        self.assertFalse(target.is_finished)
        self.assertAlmostEqual(target.target_x, 1.0)
        self.assertAlmostEqual(target.target_y, 0.0)
        self.assertAlmostEqual(target.target_velocity, 0.7)

    def test_marks_finished_near_final_waypoint(self):
        planner = PathPlannerStatic({
            "path_source": [(0.0, 0.0), (1.0, 0.0)],
            "finish_tolerance": 0.2,
            "target_velocity": 0.6,
        })

        target = planner.update(_state(x=0.95, y=0.0))

        self.assertTrue(target.is_finished)
        self.assertTrue(planner.is_finished())
        self.assertEqual(target.target_velocity, 0.0)
        self.assertAlmostEqual(target.target_x, 1.0)

    def test_set_target_velocity_updates_future_targets(self):
        planner = PathPlannerStatic({
            "path_source": [(0.0, 0.0), (2.0, 0.0)],
            "lookahead_distance": 0.5,
            "target_velocity": 0.4,
        })
        planner.set_target_velocity(0.9)

        target = planner.update(_state(x=0.0, y=0.0))

        self.assertFalse(target.is_finished)
        self.assertAlmostEqual(target.target_velocity, 0.9)

    def test_empty_path_returns_finished_current_state_target(self):
        planner = PathPlannerStatic({"path_source": []}, logger=_SilentLogger())
        target = planner.update(_state(x=4.0, y=-2.0, theta=1.2))

        self.assertTrue(target.is_finished)
        self.assertEqual(target.target_velocity, 0.0)
        self.assertEqual(target.target_x, 4.0)
        self.assertEqual(target.target_y, -2.0)
        self.assertEqual(target.target_theta, 1.2)

    def test_loads_existing_path_rich_csv_column_layout(self):
        with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as file_obj:
            path = file_obj.name
            file_obj.write("t,x,y,c3,c4,c5,theta\n")
            file_obj.write("0.0,0.0,0.0,0.0,0.0,0.0,0.1\n")
            file_obj.write("0.1,1.0,0.0,0.0,0.0,0.0,0.2\n")

        try:
            planner = PathPlannerStatic({"path_source": path})
            waypoints = planner.waypoints
        finally:
            os.remove(path)

        self.assertEqual(waypoints.shape, (2, 3))
        np.testing.assert_allclose(waypoints[0], [0.0, 0.0, 0.1])
        np.testing.assert_allclose(waypoints[1], [1.0, 0.0, 0.2])


if __name__ == "__main__":
    unittest.main()
