"""Tests for parsed multi-vehicle virtual setup manifests."""

import os
import sys
import tempfile
import unittest

import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from extra.platform.virtual.scenario import VirtualSetupError, load_virtual_setup, parse_virtual_setup


def _manifest():
    return {
        "simulation_profile": "null",
        "vehicles": [
            {
                "vehicle_id": 1,
                "initial_pose": [1.5, -0.5, 0.2],
                "v2v": {"local_port": 8101, "peers": [{"vehicle_id": 2, "port": 8102}]},
                "mission": {"path": [[0.0, 0.0], [1.0, 0.3]], "target_velocity": 0.4},
            },
            {
                "vehicle_id": 2,
                "v2v": {"local_port": 8102, "peers": [{"vehicle_id": 1, "port": 8101}]},
                "mission": {"path": [[0.0, 1.0], [1.0, 0.7]]},
            },
        ],
    }


class TestVirtualSetup(unittest.TestCase):
    def _write_manifest(self, manifest):
        temporary = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False, encoding="ascii")
        with temporary as file:
            yaml.safe_dump(manifest, file)
        self.addCleanup(lambda: os.path.exists(temporary.name) and os.unlink(temporary.name))
        return temporary.name

    def test_loads_two_vehicle_setup_and_builds_process_spec(self):
        path = self._write_manifest(_manifest())
        setup = load_virtual_setup(path)
        parsed = parse_virtual_setup(["--setup-file", path])

        self.assertEqual(setup, parsed)
        self.assertEqual(setup.simulation_profile, "null")
        self.assertEqual([vehicle.vehicle_id for vehicle in setup.vehicles], [1, 2])
        self.assertEqual(setup.vehicles[1].target_velocity, 0.30)
        spec = setup.vehicles[0].to_process_spec(setup.simulation_profile)
        self.assertEqual(spec.vehicle_id, 1)
        self.assertEqual(spec.value_overrides["mission"]["path"][-1], [1.0, 0.3])
        self.assertEqual(spec.selection_overrides, {"simulation": "null", "v2v": "udp_default"})
        self.assertEqual(spec.value_overrides["modules"]["v2v"]["local_port"], 8101)
        self.assertEqual(spec.value_overrides["modules"]["io"]["initial_x"], 1.5)
        self.assertEqual(
            spec.value_overrides["modules"]["observer"]["initial_pose"],
            {"x": 1.5, "y": -0.5, "theta": 0.2},
        )

    def test_rejects_duplicate_vehicle_ids_and_invalid_missions(self):
        duplicate = _manifest()
        duplicate["vehicles"][1]["vehicle_id"] = 1
        with self.assertRaisesRegex(VirtualSetupError, "must be unique"):
            load_virtual_setup(self._write_manifest(duplicate))

        invalid_path = _manifest()
        invalid_path["vehicles"][0]["mission"]["path"] = [[0.0, 0.0]]
        with self.assertRaisesRegex(VirtualSetupError, "at least two"):
            load_virtual_setup(self._write_manifest(invalid_path))

        unsupported = _manifest()
        unsupported["vehicles"][0]["ground_station"] = {"host": "127.0.0.1", "port": 5001}
        with self.assertRaisesRegex(VirtualSetupError, "not supported"):
            load_virtual_setup(self._write_manifest(unsupported))

        null_v2v = _manifest()
        null_v2v["vehicles"][0]["v2v_profile"] = "null"
        with self.assertRaisesRegex(VirtualSetupError, "cannot be 'null'"):
            load_virtual_setup(self._write_manifest(null_v2v))


if __name__ == "__main__":
    unittest.main()
