"""Unit tests for the standard generated-test artifact layout."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from test.helper_artifacts import create_artifact_run


class TestArtifactRun(unittest.TestCase):
    def test_creates_classified_directories_and_manifest(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = create_artifact_run(
                category="integration",
                platform="carla",
                test_name="lidar_route",
                root=temporary,
                metadata={"vehicle_ids": [0], "scenario": "sdcs_small_map"},
                run_id="20260806t120000000000z-test",
            )

            self.assertEqual(
                run.directory,
                Path(temporary) / "integration" / "carla" / "lidar_route" / "20260806t120000000000z-test",
            )
            for directory in (run.raw_directory, run.derived_directory, run.figures_directory, run.logs_directory):
                self.assertTrue(directory.is_dir())
            manifest = json.loads(run.manifest_path.read_text(encoding="utf-8"))
            self.assertEqual(manifest["platform"], "carla")
            self.assertEqual(manifest["test_name"], "lidar_route")
            self.assertEqual(manifest["metadata"]["vehicle_ids"], [0])

    def test_creates_unique_run_ids_when_none_is_supplied(self):
        with tempfile.TemporaryDirectory() as temporary:
            first = create_artifact_run(category="diagnostic", platform="qcar", test_name="io", root=temporary)
            second = create_artifact_run(category="diagnostic", platform="qcar", test_name="io", root=temporary)

            self.assertNotEqual(first.run_id, second.run_id)
            self.assertNotEqual(first.directory, second.directory)

    def test_rejects_unsafe_or_ambiguous_identifiers(self):
        with self.assertRaisesRegex(ValueError, "test_name"):
            create_artifact_run(category="integration", platform="carla", test_name="../route")


if __name__ == "__main__":
    unittest.main()
