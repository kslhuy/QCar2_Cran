"""Unit tests for selected-profile vehicle configuration loading."""

import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.module_factory import build_vehicle_modules
from core.vehicle_config import ConfigError, load_config


_CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


class TestVehicleConfig(unittest.TestCase):
    def setUp(self):
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.config_dir = Path(self._temporary_directory.name) / "config"
        shutil.copytree(_CONFIG_DIR, self.config_dir)

    def tearDown(self):
        self._temporary_directory.cleanup()

    def _edit_vehicle(self, edit):
        path = self.config_dir / "config_vehicle.yaml"
        data = yaml.safe_load(path.read_text(encoding="ascii"))
        edit(data)
        path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="ascii")

    def test_loads_selected_profiles_without_optional_backends(self):
        config = load_config(self.config_dir)

        self.assertEqual(config.vehicle_id, 0)
        self.assertEqual(config.module("io")["implementation"], "qcar")
        self.assertEqual(config.module("observer")["implementation"], "ekf")
        self.assertEqual(config.module("planner")["implementation"], "static")
        self.assertEqual(config.module("controller")["implementation"], "simple")
        self.assertEqual(config.module("simulation")["implementation"], "null")
        self.assertEqual(config.module("io")["timing"]["loop_rate_hz"], 100)
        self.assertEqual(config.module("observer")["wheelbase"], 0.3)

    def test_cli_overrides_selected_and_runtime_values(self):
        config = load_config(
            self.config_dir,
            value_overrides={
                "vehicle_id": 4,
                "runtime": {"loop_rate_hz": 50},
                "modules": {"controller": {"kp_velocity": 0.7}},
            },
        )

        self.assertEqual(config.vehicle_id, 4)
        self.assertEqual(config.runtime["loop_rate_hz"], 50)
        self.assertEqual(config.module("io")["timing"]["loop_rate_hz"], 50)
        self.assertEqual(config.module("controller")["kp_velocity"], 0.7)

    def test_missing_profile_is_rejected(self):
        self._edit_vehicle(lambda data: data["modules"].update({"planner": "missing"}))
        with self.assertRaisesRegex(ConfigError, "Unknown planner profile"):
            load_config(self.config_dir)

    def test_missing_required_module_is_rejected(self):
        self._edit_vehicle(lambda data: data["modules"].pop("io"))
        with self.assertRaisesRegex(ConfigError, "'io'"):
            load_config(self.config_dir)

    def test_invalid_safety_value_is_rejected(self):
        with self.assertRaisesRegex(ConfigError, "max_throttle"):
            load_config(
                self.config_dir,
                value_overrides={"modules": {"io": {"write": {"max_throttle": 2.0}}}},
            )

    def test_unavailable_backend_selection_is_rejected(self):
        self._edit_vehicle(lambda data: data["modules"].update({"io": "not_installed"}))
        with self.assertRaisesRegex(ConfigError, "Unknown io profile"):
            load_config(self.config_dir)

    def test_headless_selection_loads_without_hardware_imports(self):
        config = load_config(
            self.config_dir,
            vehicle_config_file="config_vehicle_headless.yaml",
        )
        self.assertEqual(config.module("io")["implementation"], "null")
        self.assertFalse(config.module("v2v")["enabled"])

    def test_sdcs_small_map_profile_passes_mission_node_sequence_to_planner(self):
        config = load_config(
            self.config_dir,
            vehicle_config_file="config_vehicle_sdcs_small_map.yaml",
        )

        self.assertEqual(config.module("planner")["implementation"], "sdcs_small_map")
        self.assertEqual(config.module("planner")["node_sequence"], config.mission["node_sequence"])

    def test_carla_default_uses_the_sdcs_planner_and_one_closed_circuit(self):
        config = load_config(self.config_dir, vehicle_config_file="config_vehicle_carla.yaml")

        self.assertEqual(config.module("planner")["implementation"], "sdcs_small_map")
        self.assertEqual(config.module("planner")["node_sequence"], [0, 2, 4, 6, 10])
        self.assertEqual(config.module("planner")["loop"], 1)
        self.assertEqual(config.module("simulation")["implementation"], "carla")

    def test_selection_override_is_applied_before_profiles_are_loaded(self):
        config = load_config(
            self.config_dir,
            selection_overrides={"io": "null", "observer": "null"},
        )
        self.assertEqual(config.module("io")["implementation"], "null")
        self.assertEqual(config.module("observer")["implementation"], "null")

    def test_tcp_ground_station_profile_builds_the_new_vehicle_bridge(self):
        config = load_config(
            self.config_dir,
            selection_overrides={
                "ground_station": "tcp_client",
                "io": "null",
                "observer": "null",
                "planner": "null",
                "controller": "null",
                "v2v": "null",
            },
        )
        modules = build_vehicle_modules(config)

        self.assertEqual(modules.ground_station.__class__.__name__, "GroundStationRuntimeFacade")

    def test_unknown_ground_station_profile_is_rejected_by_config_loading(self):
        with self.assertRaisesRegex(ConfigError, "Unknown ground_station profile"):
            load_config(self.config_dir, selection_overrides={"ground_station": "legacy"})

    def test_loads_a_new_module_from_the_vehicle_selection(self):
        (self.config_dir / "config_perception.yaml").write_text(
            "mock_camera:\n  implementation: mock_camera\n  frame_rate_hz: 20\n",
            encoding="ascii",
        )
        self._edit_vehicle(lambda data: data["modules"].update({"perception": "mock_camera"}))

        config = load_config(self.config_dir)

        self.assertEqual(config.module("perception")["implementation"], "mock_camera")
        self.assertEqual(config.module("perception")["frame_rate_hz"], 20)


if __name__ == "__main__":
    unittest.main()
