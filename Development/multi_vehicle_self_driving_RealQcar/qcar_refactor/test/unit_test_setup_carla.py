"""Tests for parsed multi-vehicle CARLA setup manifests."""

import os
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from extra.platform.carla.scenario import CarlaSetupError, load_carla_setup, parse_carla_setup


def _manifest():
    return {
        "simulation_profile": "carla_sync",
        "vehicles": [
            {
                "vehicle_id": 1,
                "spawn_transform": {"x": -13.1, "y": 0.8, "z": 0.5, "yaw": 0.0},
                "mission": {"path": [[-13.1, -0.8], [-12.0, -0.5]], "target_velocity": 0.4},
                "tick_owner": True,
                "v2v": {"local_port": 8001, "peers": [{"vehicle_id": 2, "port": 8002}]},
            },
            {
                "vehicle_id": 2,
                "spawn_transform": {"x": -13.1, "y": -2.0, "z": 0.5, "yaw": 0.0},
                "mission": {"path": [[-13.1, 2.0], [-12.0, 2.3]]},
                "v2v": {"local_port": 8002, "peers": [{"vehicle_id": 1, "port": 8001}]},
            },
        ],
    }


class TestCarlaSetup(unittest.TestCase):
    def _write_manifest(self, manifest):
        temporary = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False, encoding="ascii")
        with temporary as file:
            yaml.safe_dump(manifest, file)
        self.addCleanup(lambda: os.path.exists(temporary.name) and os.unlink(temporary.name))
        return temporary.name

    def test_loads_two_vehicle_setup_and_cli_parser(self):
        path = self._write_manifest(_manifest())
        setup = load_carla_setup(path)
        parsed = parse_carla_setup(["--setup-file", path])

        self.assertEqual(setup, parsed)
        self.assertEqual(setup.simulation_profile, "carla_sync")
        self.assertEqual(setup.host, "127.0.0.1")
        self.assertEqual(setup.port, 2000)
        self.assertEqual([vehicle.vehicle_id for vehicle in setup.vehicles], [1, 2])
        self.assertTrue(setup.vehicles[0].tick_owner)
        self.assertEqual(setup.vehicles[1].route[-1], (-12.0, 2.3))
        self.assertEqual(setup.vehicles[0].v2v["local_port"], 8001)
        self.assertEqual(setup.vehicles[0].v2v_profile, "udp_default")
        spec = setup.vehicles[0].to_process_spec(setup.simulation_profile)
        self.assertEqual(spec.selection_overrides["v2v"], "udp_default")
        self.assertEqual(spec.value_overrides["modules"]["v2v"]["local_port"], 8001)
        self.assertEqual(
            spec.value_overrides["modules"]["observer"]["initial_pose"],
            {"x": -13.1, "y": -0.8, "theta": 0.0},
        )

    def test_rejects_duplicate_ids_and_invalid_tick_ownership(self):
        duplicate = _manifest()
        duplicate["vehicles"][1]["vehicle_id"] = 1
        with self.assertRaisesRegex(CarlaSetupError, "must be unique"):
            load_carla_setup(self._write_manifest(duplicate))

        no_owner = _manifest()
        no_owner["vehicles"][0]["tick_owner"] = False
        with self.assertRaisesRegex(CarlaSetupError, "Exactly one"):
            load_carla_setup(self._write_manifest(no_owner))

    def test_rejects_colliding_v2v_ports_and_unsupported_ground_station(self):
        manifest = _manifest()
        manifest["vehicles"][1]["v2v"]["local_port"] = 8001
        with self.assertRaisesRegex(CarlaSetupError, "V2V local ports must be unique"):
            load_carla_setup(self._write_manifest(manifest))

        manifest = _manifest()
        manifest["vehicles"][0]["ground_station"] = {"host": "127.0.0.1", "port": 5001}
        with self.assertRaisesRegex(CarlaSetupError, "not supported"):
            load_carla_setup(self._write_manifest(manifest))

        null_v2v = _manifest()
        null_v2v["vehicles"][0]["v2v_profile"] = "null"
        with self.assertRaisesRegex(CarlaSetupError, "cannot be 'null'"):
            load_carla_setup(self._write_manifest(null_v2v))

    def test_default_sdcs_scenario_selects_node_route_and_loop_policy(self):
        scenario_path = Path(__file__).resolve().parents[1] / "config" / "scenarios" / "test" /"carla_sdcs_small_map.yaml"

        setup = load_carla_setup(scenario_path)
        vehicle = setup.vehicles[0]
        spec = vehicle.to_process_spec(setup.simulation_profile)

        self.assertEqual(setup.simulation_profile, "carla_sync")
        self.assertIsNone(vehicle.route)
        self.assertEqual(vehicle.node_sequence, (0, 2, 4, 6, 10))
        self.assertEqual(vehicle.loop, 1)
        self.assertEqual(spec.value_overrides["modules"]["planner"]["node_sequence"], [0, 2, 4, 6, 10])
        self.assertEqual(spec.value_overrides["modules"]["planner"]["loop"], 1)


if __name__ == "__main__":
    unittest.main()
