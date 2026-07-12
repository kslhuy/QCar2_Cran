"""Tests for parsed multi-vehicle simulation setup manifests."""

import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from extra.launch.carla_scenario import SimulationSetupError, load_simulation_setup, parse_simulation_setup


def _manifest():
    return {
        "host": "127.0.0.1",
        "port": 2000,
        "vehicles": [
            {
                "vehicle_id": 1,
                "spawn_transform": {"x": -13.1, "y": 0.8, "z": 0.5, "yaw": 0.0},
                "route": [[-13.1, -0.8], [-12.0, -0.5]],
                "tick_owner": True,
            },
            {
                "vehicle_id": 2,
                "spawn_transform": {"x": -13.1, "y": -2.0, "z": 0.5, "yaw": 0.0},
                "route": [[-13.1, 2.0], [-12.0, 2.3]],
            },
        ],
    }


class TestSimulationSetup(unittest.TestCase):
    def _write_manifest(self, manifest):
        temporary = tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False, encoding="ascii")
        with temporary as file:
            json.dump(manifest, file)
        self.addCleanup(lambda: os.path.exists(temporary.name) and os.unlink(temporary.name))
        return temporary.name

    def test_loads_two_vehicle_setup_and_cli_parser(self):
        path = self._write_manifest(_manifest())
        setup = load_simulation_setup(path)
        parsed = parse_simulation_setup(["--setup-file", path])

        self.assertEqual(setup, parsed)
        self.assertEqual([vehicle.vehicle_id for vehicle in setup.vehicles], [1, 2])
        self.assertTrue(setup.vehicles[0].tick_owner)
        self.assertEqual(setup.vehicles[1].route[-1], (-12.0, 2.3))

    def test_rejects_duplicate_ids_and_invalid_tick_ownership(self):
        duplicate = _manifest()
        duplicate["vehicles"][1]["vehicle_id"] = 1
        with self.assertRaisesRegex(SimulationSetupError, "must be unique"):
            load_simulation_setup(self._write_manifest(duplicate))

        no_owner = _manifest()
        no_owner["vehicles"][0]["tick_owner"] = False
        with self.assertRaisesRegex(SimulationSetupError, "Exactly one"):
            load_simulation_setup(self._write_manifest(no_owner))


if __name__ == "__main__":
    unittest.main()
