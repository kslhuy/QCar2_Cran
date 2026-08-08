"""Manual local-CARLA smoke test for the shared LiDAR scan contract.

Run directly only after a local CARLA server is ready:
    python test/test_integration_carla_localization.py
"""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.vehicle_config import load_config
from extra.platform.carla.session import CarlaSession
from test.helper_artifacts import create_artifact_run
from utils.io.io_carla import IOCarla
from utils.localization import plot_laser_scan_artifact, write_laser_scan_artifact


@unittest.skipUnless(__name__ == "__main__", "run this local CARLA integration test file directly")
class TestCarlaLocalizationIntegration(unittest.TestCase):
    """Verify a live ray-cast scan can enter the cross-platform contract."""

    def test_carla_lidar_publishes_a_nonempty_ros_shaped_scan(self):
        config = load_config(vehicle_config_file="config_vehicle_carla.yaml")
        session = CarlaSession(config.module("simulation"))
        io = IOCarla(config.module("io"), session)
        try:
            try:
                session.start()
            except (ImportError, RuntimeError) as error:
                self.skipTest(f"Local CARLA server is unavailable: {error}")
            scan = None
            for _ in range(40):
                session.tick()
                io.read_to_cache()
                scans = io.drain_lidar_scans()
                scan = scans[-1] if scans else None
                if scan is not None:
                    break

            self.assertIsNotNone(scan, "CARLA LiDAR produced no scan after 40 synchronous ticks")
            self.assertEqual(scan.frame_id, "laser")
            self.assertGreater(len(scan.ranges_m), 0)
            self.assertTrue(any(math.isfinite(value) for value in scan.ranges_m))
            self.assertTrue(all(value >= scan.range_min_m or math.isinf(value) for value in scan.ranges_m))
            artifacts = create_artifact_run(
                category="integration",
                platform="carla",
                test_name="lidar_scan_smoke",
                metadata={"frame_id": scan.frame_id, "range_count": len(scan.ranges_m)},
            )
            data_path, metadata_path = write_laser_scan_artifact(scan, artifacts.raw_directory)
            plot_path = plot_laser_scan_artifact(metadata_path, output_path=artifacts.figures_directory / "first_scan.png")
            print(f"Stored first CARLA reference scan: {data_path}")
            print(f"Stored first CARLA scan metadata: {metadata_path}")
            print(f"Stored first CARLA scan plot: {plot_path}")
        finally:
            io.close()
            session.close()


if __name__ == "__main__":
    unittest.main()
