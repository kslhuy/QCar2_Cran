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

from utils.control.path_planner import (
    PathPlannerBase,
    PathPlannerNull,
    PathPlannerSDCSSmallMap,
    PathPlannerStatic,
    SDCSSmallMapRoadMap,
)
from core.vehicle_types import ControllerReference, VehicleStateEstimate


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
    def test_null_planner_returns_safe_finished_target(self):
        planner = PathPlannerNull({"target_velocity": 0.0}, vehicle_id=4)
        target = planner.update(_state(x=2.0, y=3.0, theta=0.5))

        self.assertEqual(planner._vehicle_id, 4)
        self.assertIsInstance(target, ControllerReference)
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

        self.assertIsInstance(target, ControllerReference)
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


class TestSDCSSmallMapPlanner(unittest.TestCase):
    def test_uses_right_hand_small_map_nodes_in_carla_plane_metres(self):
        roadmap = SDCSSmallMapRoadMap(sample_distance_m=0.02)

        self.assertEqual(roadmap.node_ids, tuple(range(11)))
        x_m, y_m, heading_rad = roadmap.node_pose(0)
        self.assertAlmostEqual(x_m, -1.14)
        self.assertAlmostEqual(y_m, 1.053177)
        self.assertAlmostEqual(heading_rad, -np.pi / 2.0)

    def test_generates_directed_route_and_standard_planner_targets(self):
        planner = PathPlannerSDCSSmallMap(
            {
                "node_sequence": [0, 2, 4, 6, 0],
                "sample_distance_m": 0.02,
                "lookahead_distance": 0.25,
                "target_velocity": 0.5,
            }
        )

        self.assertEqual(planner.node_sequence, (0, 2, 4, 6, 0))
        self.assertEqual(planner.available_node_ids, tuple(range(11)))
        self.assertGreater(len(planner.waypoints), 100)
        np.testing.assert_allclose(planner.waypoints[0, :2], planner.node_pose(0)[:2])
        np.testing.assert_allclose(planner.waypoints[-1, :2], planner.node_pose(0)[:2])

        target = planner.update(_state(*planner.node_pose(0)[:2]))
        self.assertFalse(target.is_finished)
        self.assertAlmostEqual(target.target_velocity, 0.5)

    def test_rejects_invalid_node_ids_and_unusable_sequences(self):
        planner = PathPlannerSDCSSmallMap({"sample_distance_m": 0.02})

        with self.assertRaisesRegex(ValueError, r"\[0, 10\]"):
            planner.set_node_sequence([0, 11])
        with self.assertRaisesRegex(ValueError, "at least two"):
            planner.set_node_sequence([0])
        with self.assertRaisesRegex(ValueError, "adjacent duplicate"):
            planner.set_node_sequence([0, 0])

    def test_loop_policy_expands_closed_circuits_and_preserves_open_route(self):
        planner = PathPlannerSDCSSmallMap({"sample_distance_m": 0.02})

        planner.set_node_sequence([0, 2, 4], loop=0)
        self.assertEqual(planner.route_node_sequence, (0, 2, 4))
        planner.set_node_sequence([0, 2, 4], loop=1)
        self.assertEqual(planner.route_node_sequence, (0, 2, 4, 0))
        planner.set_node_sequence([0, 2, 4], loop=2)
        self.assertEqual(planner.route_node_sequence, (0, 2, 4, 0, 2, 4, 0))

    def test_infinite_loop_restarts_at_origin_instead_of_finishing(self):
        planner = PathPlannerSDCSSmallMap(
            {"node_sequence": [0, 2, 4], "loop": "inf", "finish_tolerance": 0.2}
        )
        final_x, final_y, _ = planner.node_pose(0)

        target = planner.update(_state(final_x, final_y))

        self.assertFalse(target.is_finished)
        self.assertFalse(planner.is_finished())
        self.assertEqual(planner.loop, "inf")


if __name__ == "__main__":
    unittest.main()
