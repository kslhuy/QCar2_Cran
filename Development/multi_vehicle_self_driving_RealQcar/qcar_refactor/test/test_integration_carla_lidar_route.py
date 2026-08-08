"""Run the CARLA SDCS route with live LiDAR and localisation comparison.

Run this file directly after starting the local CARLA SDCS world::

    python test/test_integration_carla_lidar_route.py

It drives the planned route through nodes ``0 -> 1 -> 4 -> 0`` in paced
simulation time. A stationary scan captured at the confirmed node-0 pose is
the fixed reference map. The optional Qt view shows localisation/path on the
left and current plus reference 2-D LiDAR scans on the right. The default TCP
bridge also registers vehicle 0 with a locally running CLI ground station.
Results are saved under ``test/artifacts/integration/carla/lidar_route/<run-id>/``.

The LiDAR trace is *reference-scan map localization initialized at node 0*.
It is comparison-only: the route controller continues to use ``IOCarla``'s
CARLA pose, while the test compares the LiDAR estimate with the unchanged EKF
estimate.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.commands import CommandType, VehicleCommand
from core.module_factory import build_vehicle_modules
from core.vehicle_config import load_config
from core.vehicle_logic import VehicleRuntime
from core.vehicle_state_machine import State
from extra.ground_station.utils.live_plotting import LiveLidarLocalizationViewer
from test.helper_artifacts import ArtifactRun, create_artifact_run
from utils.control.path_planner import SDCSSmallMapRoadMap
from core.vehicle_types import LaserScanSample, PoseMeasurement
from utils.localization import (
    ReferenceScanLidarLocalizer,
    plot_laser_scan_artifact,
    write_laser_scan_artifact,
)


def main(argv: list[str] | None = None) -> int:
    arguments = _parse_arguments(argv)
    result = run_lidar_route_test(
        duration_s=arguments.duration_s,
        show_live_plot=not arguments.no_window,
        realtime=not arguments.as_fast_as_possible,
        ground_station_profile=arguments.ground_station,
        ground_station_host=arguments.ground_station_host,
        ground_station_port=arguments.ground_station_port,
    )
    print("CARLA LiDAR route test completed.")
    for label, path in result.items():
        print(f"{label}: {path}")
    return 0


def run_lidar_route_test(
    *,
    duration_s: float = 120.0,
    show_live_plot: bool = True,
    realtime: bool = True,
    ground_station_profile: str = "tcp_client",
    ground_station_host: str | None = None,
    ground_station_port: int | None = None,
) -> dict[str, Path]:
    """Drive the selected route, update live plots, and write reproducible artifacts."""

    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("duration_s must be positive and finite")
    if ground_station_profile not in {"null", "tcp_client"}:
        raise ValueError("ground_station_profile must be 'null' or 'tcp_client'")
    if ground_station_profile == "null" and (
        ground_station_host is not None or ground_station_port is not None
    ):
        raise ValueError("ground-station host/port overrides require ground_station_profile='tcp_client'")

    node_sequence = (0, 1, 4, 0)
    artifact_run = create_artifact_run(
        category="integration",
        platform="carla",
        test_name="lidar_route",
        metadata={
            "node_sequence": list(node_sequence),
            "duration_limit_s": duration_s,
            "realtime": realtime,
            "ground_station_profile": ground_station_profile,
        },
    )
    ground_station_values: dict[str, object] = {}
    if ground_station_host is not None:
        ground_station_values["server_host"] = ground_station_host
    if ground_station_port is not None:
        ground_station_values["server_port"] = ground_station_port
    config = load_config(
        vehicle_config_file="config_vehicle_carla.yaml",
        selection_overrides={"ground_station": ground_station_profile},
        value_overrides={
            "mission": {"node_sequence": list(node_sequence), "loop": 0, "target_velocity": 3.0},
            "modules": {
                "observer": {
                    "initial_pose": {"x": -1.14, "y": 1.053177, "theta": -math.pi / 2.0}
                },
                "planner": {"node_sequence": list(node_sequence), "loop": 0, "target_velocity": 3.0},
                "simulation": {
                    "spawn_transform": {"x": -1.14, "y": -1.053177, "z": 0.5, "yaw": 90.0},
                },
                **({"ground_station": ground_station_values} if ground_station_values else {}),
            },
        },
    )
    roadmap = SDCSSmallMapRoadMap()
    route = roadmap.generate_path(node_sequence)
    modules = build_vehicle_modules(config)
    runtime = VehicleRuntime(
        config,
        modules.io,
        modules.observer,
        modules.planner,
        modules.controller_manager,
        modules.v2v,
        modules.ground_station,
        simulation=modules.simulation,
    )
    fixed_delta_s = float(config.module("simulation")["fixed_delta_seconds"])
    initial_x, initial_y, initial_yaw = roadmap.node_pose(node_sequence[0])
    viewer: LiveLidarLocalizationViewer | None = None
    rows: list[dict[str, Any]] = []
    last_scan: LaserScanSample | None = None
    reference_scan: LaserScanSample | None = None
    latest_lidar_pose = PoseMeasurement(0, "map", initial_x, initial_y, initial_yaw, False)
    localizer: ReferenceScanLidarLocalizer | None = None
    finished = False

    try:
        runtime.start()
        # Let asynchronous sensor callbacks deliver the first stationary scan
        # before movement begins.  Its known map pose is node 0.
        for _ in range(40):
            runtime.step(dt=fixed_delta_s)
            last_scan = _latest_lidar_scan(runtime.io)
            if last_scan is not None:
                break
        if last_scan is None:
            raise RuntimeError("CARLA LiDAR produced no scan after 40 synchronous warm-up ticks")
        if not any(math.isfinite(value) for value in last_scan.ranges_m):
            raise RuntimeError(
                "CARLA LiDAR scan contains no collision returns; verify the SDCS map has collidable walls at z=0.5 m"
            )
        reference_scan = last_scan
        reference_pose = PoseMeasurement(reference_scan.timestamp_ns, "map", initial_x, initial_y, initial_yaw, True)
        localizer = ReferenceScanLidarLocalizer(
            reference_scan,
            reference_pose,
            minimum_range_m=2.0,
            max_correspondence_distance_m=1.0,
            min_correspondences=12,
            max_iterations=16,
        )
        localizer.start(reference_pose)
        latest_lidar_pose = localizer.update(reference_scan)
        # Start CARLA before Qt so the Windows native-library load order stays
        # compatible, as in the standalone CARLA viewer diagnostic.
        if show_live_plot:
            viewer = LiveLidarLocalizationViewer(route)

        start_result = runtime.handle_command(VehicleCommand(CommandType.START))
        if start_result.runtime_state != State.RUNNING.name:
            raise RuntimeError(f"Route start was rejected: {start_result}")

        deadline = time.monotonic() + duration_s
        next_wall_tick = time.monotonic()
        while time.monotonic() < deadline:
            telemetry = runtime.step(dt=fixed_delta_s)
            scan = _latest_lidar_scan(runtime.io)
            if scan is not None:
                last_scan = scan
                latest_lidar_pose = localizer.update(scan)

            row = _make_row(telemetry, latest_lidar_pose, localizer)
            rows.append(row)
            if viewer is not None:
                viewer.show_state(
                    current_scan=last_scan,
                    reference_scan=reference_scan,
                    carla_position=(row["gps_x_m"], row["gps_y_m"]),
                    ekf_position=(row["ekf_x_m"], row["ekf_y_m"]),
                    lidar_position=latest_lidar_pose,
                    reference_position=reference_pose,
                )
                if not viewer.process_events():
                    break
            if telemetry.state is State.STOPPED and telemetry.target.is_finished:
                finished = True
                break
            if realtime:
                next_wall_tick += fixed_delta_s
                time.sleep(max(0.0, next_wall_tick - time.monotonic()))
    finally:
        runtime.shutdown()
        if viewer is not None:
            viewer.close()
        if localizer is not None:
            localizer.close()

    if not rows:
        raise RuntimeError("CARLA route produced no telemetry rows")
    artifacts = _write_artifacts(rows, route, reference_scan, last_scan, artifact_run)
    if not finished:
        raise RuntimeError(
            f"Route 0 -> 1 -> 4 -> 0 did not finish within {duration_s:.1f} s; "
            f"partial artifacts were written to {artifact_run.directory}"
        )
    return artifacts


def _latest_lidar_scan(io) -> LaserScanSample | None:
    """Consume local IOCarla scans once and keep the newest for this test."""

    scans = io.drain_lidar_scans()
    return scans[-1] if scans else None


def _make_row(telemetry, lidar_pose: PoseMeasurement, localizer: ReferenceScanLidarLocalizer) -> dict[str, Any]:
    gps = telemetry.sensor_data.gps_position
    return {
        "time_s": float(telemetry.sensor_data.sensor_timestamp),
        "gps_x_m": float(gps[0]),
        "gps_y_m": float(gps[1]),
        "gps_yaw_rad": float(gps[2]),
        "ekf_x_m": float(telemetry.estimate.x),
        "ekf_y_m": float(telemetry.estimate.y),
        "ekf_yaw_rad": float(telemetry.estimate.theta),
        "lidar_x_m": float(lidar_pose.x_m),
        "lidar_y_m": float(lidar_pose.y_m),
        "lidar_yaw_rad": float(lidar_pose.yaw_rad),
        "lidar_valid": bool(lidar_pose.valid),
        "lidar_correspondences": int(localizer.last_correspondence_count),
        "lidar_rmse_m": localizer.last_rmse_m,
        "speed_mps": float(telemetry.sensor_data.motor_tach),
        "target_x_m": float(telemetry.target.target_x),
        "target_y_m": float(telemetry.target.target_y),
        "target_speed_mps": float(telemetry.target.target_velocity),
        "throttle": float(telemetry.command.throttle),
        "steering_rad": float(telemetry.command.steering),
        "state": telemetry.state.name,
    }


class _LiveDashboard:
    """Update one non-blocking figure without coupling the runtime to pyplot."""

    def __init__(self, route: np.ndarray, *, show: bool) -> None:
        import matplotlib

        if not show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as pyplot

        self._pyplot = pyplot
        self._show = show
        if show:
            pyplot.ion()
        self._figure, (self._scan_axis, self._route_axis) = pyplot.subplots(1, 2, figsize=(14, 6), constrained_layout=True)
        self._scan_points = self._scan_axis.scatter([], [], s=8, color="#1769aa", label="LiDAR returns")
        self._scan_axis.scatter([0.0], [0.0], s=55, color="#c62828", marker="x", label="sensor")
        self._scan_axis.set_aspect("equal", adjustable="box")
        self._scan_axis.set_xlabel("forward x (m)")
        self._scan_axis.set_ylabel("left y (m)")
        self._scan_axis.grid(True, alpha=0.3)
        self._scan_axis.legend(loc="upper right")
        self._route_axis.plot(route[:, 0], route[:, 1], "--", color="0.35", label="planner route 0→1→4→0")
        (self._gps_line,) = self._route_axis.plot([], [], color="#2e7d32", label="CARLA pose (control input)")
        (self._ekf_line,) = self._route_axis.plot([], [], color="#1565c0", label="EKF")
        (self._lidar_line,) = self._route_axis.plot([], [], color="#ef6c00", label="LiDAR reference-scan localization")
        self._route_axis.set_aspect("equal", adjustable="box")
        self._route_axis.set_xlabel("map x (m)")
        self._route_axis.set_ylabel("map y (m)")
        self._route_axis.grid(True, alpha=0.3)
        self._route_axis.legend(loc="best")
        if show:
            pyplot.show(block=False)

    def update(self, scan: LaserScanSample | None, rows: list[dict[str, Any]]) -> None:
        if scan is not None:
            points = _scan_xy(scan)
            self._scan_points.set_offsets(points if len(points) else np.empty((0, 2)))
            self._scan_axis.set_xlim(-scan.range_max_m, scan.range_max_m)
            self._scan_axis.set_ylim(-scan.range_max_m, scan.range_max_m)
            self._scan_axis.set_title(f"Live 2-D LiDAR: {len(points)}/{len(scan.ranges_m)} finite bins")
        records = np.asarray(
            [
                (row["gps_x_m"], row["gps_y_m"], row["ekf_x_m"], row["ekf_y_m"], row["lidar_x_m"], row["lidar_y_m"], row["lidar_valid"])
                for row in rows
            ],
            dtype=float,
        )
        self._gps_line.set_data(records[:, 0], records[:, 1])
        self._ekf_line.set_data(records[:, 2], records[:, 3])
        valid = records[:, 6].astype(bool)
        self._lidar_line.set_data(records[valid, 4], records[valid, 5])
        self._route_axis.set_title("Live route: CARLA pose, EKF, and LiDAR reference-scan localization")
        if self._show:
            self._figure.canvas.draw_idle()
            self._figure.canvas.flush_events()
            self._pyplot.pause(0.001)

    def close(self) -> None:
        if self._show:
            self._pyplot.ioff()
        self._pyplot.close(self._figure)


def _write_artifacts(
    rows: list[dict[str, Any]],
    route: np.ndarray,
    reference_scan: LaserScanSample | None,
    last_scan: LaserScanSample | None,
    artifact_run: ArtifactRun,
) -> dict[str, Path]:
    csv_path = artifact_run.derived_directory / "route_trace.csv"
    with csv_path.open("w", newline="", encoding="ascii") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    import matplotlib.pyplot as pyplot

    time_s = np.asarray([row["time_s"] for row in rows], dtype=float)
    time_s -= time_s[0]
    gps = np.asarray([(row["gps_x_m"], row["gps_y_m"]) for row in rows], dtype=float)
    ekf = np.asarray([(row["ekf_x_m"], row["ekf_y_m"]) for row in rows], dtype=float)
    lidar = np.asarray([(row["lidar_x_m"], row["lidar_y_m"]) for row in rows], dtype=float)
    valid = np.asarray([row["lidar_valid"] for row in rows], dtype=bool)
    lidar_to_ekf = np.linalg.norm(lidar - ekf, axis=1)
    lidar_to_gps = np.linalg.norm(lidar - gps, axis=1)

    figure, axes = pyplot.subplots(2, 2, figsize=(14, 10), constrained_layout=True)
    axes[0, 0].plot(route[:, 0], route[:, 1], "--", color="0.35", label="planner route 0→1→4→0")
    axes[0, 0].plot(gps[:, 0], gps[:, 1], label="CARLA pose (control input)")
    axes[0, 0].plot(ekf[:, 0], ekf[:, 1], label="EKF")
    axes[0, 0].plot(lidar[valid, 0], lidar[valid, 1], label="LiDAR reference-scan localization")
    axes[0, 0].set_title("Trajectory comparison")
    axes[0, 0].set_xlabel("map x (m)")
    axes[0, 0].set_ylabel("map y (m)")
    axes[0, 0].axis("equal")
    axes[0, 0].grid(True)
    axes[0, 0].legend()

    axes[0, 1].plot(time_s[valid], lidar_to_ekf[valid], label="LiDAR − EKF")
    axes[0, 1].plot(time_s[valid], lidar_to_gps[valid], label="LiDAR − CARLA pose", alpha=0.75)
    axes[0, 1].set_title("LiDAR position disagreement")
    axes[0, 1].set_xlabel("simulation time (s)")
    axes[0, 1].set_ylabel("position error (m)")
    axes[0, 1].grid(True)
    axes[0, 1].legend()

    axes[1, 0].plot(time_s, [row["speed_mps"] for row in rows], label="speed")
    axes[1, 0].plot(time_s, [row["target_speed_mps"] for row in rows], label="target speed")
    axes[1, 0].plot(time_s, [row["throttle"] for row in rows], label="throttle")
    axes[1, 0].set_title("Route-following control")
    axes[1, 0].set_xlabel("simulation time (s)")
    axes[1, 0].grid(True)
    axes[1, 0].legend()

    rmse = [float("nan") if row["lidar_rmse_m"] is None else row["lidar_rmse_m"] for row in rows]
    quality_axis = axes[1, 1]
    rmse_axis = quality_axis.twinx()
    quality_line = quality_axis.plot(time_s, [row["lidar_correspondences"] for row in rows], label="ICP correspondences")
    rmse_line = rmse_axis.plot(time_s, rmse, color="#ef6c00", label="ICP RMSE (m)")
    quality_axis.set_title("LiDAR scan-matching quality")
    quality_axis.set_xlabel("simulation time (s)")
    quality_axis.set_ylabel("correspondences")
    rmse_axis.set_ylabel("RMSE (m)")
    quality_axis.grid(True)
    quality_axis.legend(quality_line + rmse_line, [line.get_label() for line in quality_line + rmse_line])

    comparison_path = artifact_run.figures_directory / "trajectory_comparison.png"
    figure.savefig(comparison_path, dpi=160)
    pyplot.close(figure)
    artifacts = {"manifest": artifact_run.manifest_path, "trace_csv": csv_path, "trajectory_plot": comparison_path}
    if reference_scan is not None:
        _, reference_metadata_path = write_laser_scan_artifact(
            reference_scan, artifact_run.raw_directory, basename="reference_scan"
        )
        artifacts["reference_scan_metadata"] = reference_metadata_path
        artifacts["reference_scan_plot"] = plot_laser_scan_artifact(
            reference_metadata_path, output_path=artifact_run.figures_directory / "reference_scan.png"
        )
    if last_scan is not None:
        _, metadata_path = write_laser_scan_artifact(last_scan, artifact_run.raw_directory, basename="last_scan")
        artifacts["last_scan_metadata"] = metadata_path
        artifacts["last_scan_plot"] = plot_laser_scan_artifact(
            metadata_path, output_path=artifact_run.figures_directory / "last_scan.png"
        )
    return artifacts


def _scan_xy(scan: LaserScanSample) -> np.ndarray:
    ranges = np.asarray(scan.ranges_m, dtype=float)
    angles = scan.angle_min_rad + np.arange(len(ranges), dtype=float) * scan.angle_increment_rad
    valid = np.isfinite(ranges) & (ranges >= scan.range_min_m) & (ranges <= scan.range_max_m)
    return np.column_stack((ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid])))


def _parse_arguments(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the live CARLA LiDAR route comparison test.")
    parser.add_argument("--duration-s", type=float, default=120.0, help="wall-clock timeout in seconds (default: 120)")
    parser.add_argument("--no-window", action="store_true", help="save artifacts without opening the live Qt view")
    parser.add_argument("--as-fast-as-possible", action="store_true", help="do not pace CARLA ticks to real time")
    parser.add_argument("--ground-station", choices=("null", "tcp_client"), default="tcp_client")
    parser.add_argument("--ground-station-host", help="override the TCP ground-station host")
    parser.add_argument("--ground-station-port", type=int, help="override the TCP ground-station port")
    return parser.parse_args(argv)


if __name__ == "__main__":
    raise SystemExit(main())
