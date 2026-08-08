"""Portable recording helpers for ROS-shaped LiDAR localization artifacts."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from core.vehicle_types import LaserScanSample


def write_laser_scan_artifact(
    scan: LaserScanSample, output_directory: str | Path, *, basename: str = "first_scan"
) -> tuple[Path, Path]:
    """Store one scan as compact arrays plus JSON ROS-compatible metadata."""

    if not isinstance(scan, LaserScanSample):
        raise TypeError("write_laser_scan_artifact requires a LaserScanSample")
    if not isinstance(basename, str) or not basename or Path(basename).name != basename:
        raise ValueError("basename must be a simple non-empty file name")
    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    data_path = destination / f"{basename}.npz"
    metadata_path = destination / f"{basename}.json"
    np.savez_compressed(
        data_path,
        ranges_m=np.asarray(scan.ranges_m, dtype=np.float32),
        intensities=np.asarray(scan.intensities, dtype=np.float32),
    )
    metadata_path.write_text(
        json.dumps(
            {
                "schema": "sensor_msgs/msg/LaserScan-compatible",
                "data_file": data_path.name,
                "timestamp_ns": scan.timestamp_ns,
                "frame_id": scan.frame_id,
                "angle_min_rad": scan.angle_min_rad,
                "angle_max_rad": scan.angle_max_rad,
                "angle_increment_rad": scan.angle_increment_rad,
                "time_increment_s": scan.time_increment_s,
                "scan_time_s": scan.scan_time_s,
                "range_min_m": scan.range_min_m,
                "range_max_m": scan.range_max_m,
                "range_count": len(scan.ranges_m),
                "intensity_count": len(scan.intensities),
                "coordinate_convention": "ROS sensor frame: x forward, y left, z up; angles CCW-positive",
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return data_path, metadata_path


def plot_laser_scan_artifact(
    metadata_path: str | Path, *, output_path: str | Path | None = None
) -> Path:
    """Render a stored scan as a top-down ROS sensor-frame PNG plot."""

    metadata_file = Path(metadata_path)
    try:
        metadata = json.loads(metadata_file.read_text(encoding="utf-8"))
        data_file = metadata_file.with_name(str(metadata["data_file"]))
        with np.load(data_file) as arrays:
            ranges = np.asarray(arrays["ranges_m"], dtype=float)
        angle_min = float(metadata["angle_min_rad"])
        angle_increment = float(metadata["angle_increment_rad"])
        range_min = float(metadata["range_min_m"])
        range_max = float(metadata["range_max_m"])
        frame_id = str(metadata["frame_id"])
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise ValueError(f"Cannot read LiDAR scan artifact metadata {metadata_file}: {error}") from error
    angles = angle_min + np.arange(ranges.size, dtype=float) * angle_increment
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    if not np.any(valid):
        raise ValueError(f"LiDAR scan artifact contains no valid ranges: {metadata_file}")
    x_forward = ranges[valid] * np.cos(angles[valid])
    y_left = ranges[valid] * np.sin(angles[valid])
    destination = Path(output_path) if output_path is not None else metadata_file.with_suffix(".png")
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as pyplot
    except ImportError as error:
        raise RuntimeError("Plotting a LiDAR scan requires matplotlib") from error
    figure, axis = pyplot.subplots(figsize=(8, 8), constrained_layout=True)
    axis.scatter(x_forward, y_left, s=5, color="#1769aa", label=f"{valid.sum()} LiDAR returns")
    axis.scatter([0.0], [0.0], s=50, color="#c62828", marker="x", label="sensor origin")
    axis.arrow(0.0, 0.0, min(1.0, range_max * 0.1), 0.0, color="#c62828", width=0.02, length_includes_head=True)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("forward x (m)")
    axis.set_ylabel("left y (m)")
    axis.set_title(f"LiDAR scan: {frame_id}")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=150)
    pyplot.close(figure)
    return destination
