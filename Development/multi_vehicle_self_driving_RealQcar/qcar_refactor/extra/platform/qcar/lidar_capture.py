"""Read-only physical-QCar LiDAR capture and ground-station plotting tool.

The capture command runs on the QCar and opens only PAL's ``QCarLidar``
resource. It calls ``QCarLidar.read()``, copies its ``angles`` and
``distances`` arrays, then closes that LiDAR resource. It does *not* create a
``QCar`` actuator object or ``QCarGPS``; in particular, it does not start,
stop, or calibrate the separate LiDAR-to-GPS service.

The plot command runs after the capture has been fetched to the ground
station. It produces a static first diagnostic view; an explicit Qt live
preview remains a separate future diagnostic feature.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import time
from typing import Any, Callable, Sequence

import numpy as np

from extra.platform.qcar.process_runner import QCarLidarFactory, qcar_lidar_resource_context


_CSV_NAME = "real_qcar_lidar.csv"
_METADATA_NAME = "real_qcar_lidar.json"
_PLOT_NAME = "real_qcar_lidar.png"
_FIELDS = (
    "elapsed_s",
    "scan_timestamp_s",
    "scan_index",
    "point_index",
    "angle_rad",
    "distance_m",
    "valid",
)

class QCarLidarCaptureError(RuntimeError):
    """Raised when the explicit, read-only LiDAR probe cannot complete."""


@dataclass(frozen=True)
class QCarLidarCaptureResult:
    """Paths and summary data produced by one stationary LiDAR capture."""

    csv_path: Path
    metadata_path: Path
    scan_count: int
    point_count: int
    duration_s: float
    poll_rate_hz: float

    def to_mapping(self) -> dict[str, object]:
        values = asdict(self)
        values["csv_path"] = str(self.csv_path)
        values["metadata_path"] = str(self.metadata_path)
        return values


def capture_qcar_lidar(
    *,
    output_directory: str | Path,
    duration_s: float = 10.0,
    poll_rate_hz: float = 20.0,
    num_measurements: int = 384,
    enable_filtering: bool = True,
    lidar_factory: QCarLidarFactory | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> QCarLidarCaptureResult:
    """Capture stationary physical-QCar LiDAR frames without driving the car.

    ``QCarLidar`` is the direct PAL RPLidar client. Its current PAL API
    returns a boolean from ``read()`` and updates ``angles`` (radians) and
    ``distances`` (metres). Zeros are retained as invalid raw readings in the
    CSV rather than discarded so the capture can validate the later IO
    conversion policy.

    Call only while no vehicle runtime, calibration tool, or other LiDAR
    client owns the device. The vehicle's fixed position makes this a useful
    acquisition and geometry probe; it does not infer a map pose.
    """

    _validate_capture_arguments(duration_s, poll_rate_hz, num_measurements)
    period_s = 1.0 / poll_rate_hz
    started_at = monotonic()
    next_poll_at = started_at
    rows: list[dict[str, object]] = []
    scan_count = 0

    try:
        with qcar_lidar_resource_context(
            lidar_factory=lidar_factory,
            num_measurements=num_measurements,
            enable_filtering=enable_filtering,
        ) as lidar:
            while monotonic() - started_at < duration_s:
                try:
                    received = bool(lidar.read())
                except Exception as error:
                    raise QCarLidarCaptureError(f"QCarLidar.read() failed: {error}") from error

                captured_at = monotonic()
                if received:
                    angles, distances = _copy_scan_arrays(lidar)
                    rows.extend(
                        _scan_rows(
                            elapsed_s=captured_at - started_at,
                            scan_index=scan_count,
                            angles=angles,
                            distances=distances,
                        )
                    )
                    scan_count += 1

                next_poll_at += period_s
                sleep(max(0.0, next_poll_at - monotonic()))
    except QCarLidarCaptureError:
        raise
    except Exception as error:
        raise QCarLidarCaptureError(f"Unable to open or release QCarLidar: {error}") from error

    valid_point_count = sum(1 for row in rows if bool(row["valid"]))
    if scan_count < 2:
        raise QCarLidarCaptureError(
            "Physical QCar LiDAR capture produced fewer than two fresh scans; "
            "verify that no other process owns the sensor."
        )
    if valid_point_count == 0:
        raise QCarLidarCaptureError("Physical QCar LiDAR capture contained no positive finite distances")

    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    csv_path = destination / _CSV_NAME
    metadata_path = destination / _METADATA_NAME
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=_FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    result = QCarLidarCaptureResult(
        csv_path=csv_path,
        metadata_path=metadata_path,
        scan_count=scan_count,
        point_count=len(rows),
        duration_s=duration_s,
        poll_rate_hz=poll_rate_hz,
    )
    metadata_path.write_text(
        json.dumps(
            {
                **result.to_mapping(),
                "pal_api": {
                    "class": "pal.products.qcar.QCarLidar",
                    "read_method": "read",
                    "angle_unit": "rad",
                    "distance_unit": "m",
                    "arrays": ["angles", "distances"],
                    "filtering_enabled": bool(enable_filtering),
                    "num_measurements": num_measurements,
                },
                "capture_mode": "stationary_read_only",
                "actuator_writes": 0,
                "qcar_actuator_created": False,
                "qcar_gps_created": False,
                "lidar_to_gps_service_changed": False,
                "valid_point_count": valid_point_count,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return result


def plot_qcar_lidar_capture(
    csv_path: str | Path, *, output_path: str | Path | None = None
) -> Path:
    """Render the newest captured LiDAR scan as a local Cartesian PNG plot."""

    source = Path(csv_path)
    rows = _read_capture_csv(source)
    newest_index = max(int(row["scan_index"]) for row in rows)
    newest = [row for row in rows if int(row["scan_index"]) == newest_index and bool(row["valid"])]
    if not newest:
        raise QCarLidarCaptureError("Newest LiDAR scan has no valid points to plot")
    destination = Path(output_path) if output_path is not None else source.with_name(_PLOT_NAME)
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as pyplot
    except ImportError as error:
        raise QCarLidarCaptureError(
            "Plotting requires matplotlib on the ground-station Python environment"
        ) from error

    angles = np.asarray([float(row["angle_rad"]) for row in newest], dtype=float)
    distances = np.asarray([float(row["distance_m"]) for row in newest], dtype=float)
    figure, axis = pyplot.subplots(figsize=(8, 8), constrained_layout=True)
    axis.scatter(distances * np.cos(angles), distances * np.sin(angles), s=5, label=f"scan {newest_index}")
    axis.scatter([0.0], [0.0], marker="+", s=80, label="QCar LiDAR origin")
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.set_title("Physical QCar LiDAR capture")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=150)
    pyplot.close(figure)
    return destination


def _copy_scan_arrays(lidar: Any) -> tuple[np.ndarray, np.ndarray]:
    try:
        angles = np.asarray(lidar.angles, dtype=float).reshape(-1).copy()
        distances = np.asarray(lidar.distances, dtype=float).reshape(-1).copy()
    except (AttributeError, TypeError, ValueError) as error:
        raise QCarLidarCaptureError("QCarLidar did not expose numeric angles and distances arrays") from error
    if angles.size == 0 or angles.size != distances.size:
        raise QCarLidarCaptureError(
            f"QCarLidar returned incompatible scan arrays: angles={angles.size}, distances={distances.size}"
        )
    if not np.all(np.isfinite(angles)):
        raise QCarLidarCaptureError("QCarLidar returned non-finite scan angles")
    return angles, distances


def _scan_rows(
    *, elapsed_s: float, scan_index: int, angles: np.ndarray, distances: np.ndarray
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for point_index, (angle, distance) in enumerate(zip(angles, distances)):
        distance_value = float(distance)
        rows.append(
            {
                "elapsed_s": float(elapsed_s),
                "scan_timestamp_s": float(elapsed_s),
                "scan_index": scan_index,
                "point_index": point_index,
                "angle_rad": float(angle),
                "distance_m": distance_value,
                "valid": bool(math.isfinite(distance_value) and distance_value > 0.0),
            }
        )
    return rows


def _read_capture_csv(path: Path) -> list[dict[str, object]]:
    try:
        with path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            if tuple(reader.fieldnames or ()) != _FIELDS:
                raise QCarLidarCaptureError(f"Unexpected physical-QCar LiDAR CSV columns in {path}")
            rows = [
                {
                    "elapsed_s": float(row["elapsed_s"]),
                    "scan_timestamp_s": float(row["scan_timestamp_s"]),
                    "scan_index": int(row["scan_index"]),
                    "point_index": int(row["point_index"]),
                    "angle_rad": float(row["angle_rad"]),
                    "distance_m": float(row["distance_m"]),
                    "valid": row["valid"].strip().lower() == "true",
                }
                for row in reader
            ]
    except OSError as error:
        raise QCarLidarCaptureError(f"Cannot read physical-QCar LiDAR CSV {path}: {error}") from error
    except (KeyError, TypeError, ValueError) as error:
        raise QCarLidarCaptureError(f"Invalid physical-QCar LiDAR CSV {path}: {error}") from error
    if not rows:
        raise QCarLidarCaptureError("Physical-QCar LiDAR CSV has no scan points")
    return rows


def _validate_capture_arguments(duration_s: float, poll_rate_hz: float, num_measurements: int) -> None:
    if not 1.0 <= duration_s <= 300.0:
        raise QCarLidarCaptureError("--duration-s must be between 1 and 300 seconds")
    if not 1.0 <= poll_rate_hz <= 100.0:
        raise QCarLidarCaptureError("--poll-rate-hz must be between 1 and 100 Hz")
    if not isinstance(num_measurements, int) or isinstance(num_measurements, bool) or num_measurements <= 0:
        raise QCarLidarCaptureError("--num-measurements must be a positive integer")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Capture and plot read-only physical-QCar LiDAR diagnostics")
    commands = parser.add_subparsers(dest="command", required=True)
    capture = commands.add_parser("capture", help="run a stationary read-only LiDAR capture on the QCar")
    capture.add_argument("--output-dir", default="../artifacts/real_qcar_lidar")
    capture.add_argument("--duration-s", type=float, default=10.0)
    capture.add_argument("--poll-rate-hz", type=float, default=20.0)
    capture.add_argument("--num-measurements", type=int, default=384)
    capture.add_argument("--no-filtering", action="store_true")
    plot = commands.add_parser("plot", help="plot a fetched physical-QCar LiDAR CSV on the ground station")
    plot.add_argument("--input", required=True)
    plot.add_argument("--output")
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "capture":
            result = capture_qcar_lidar(
                output_directory=arguments.output_dir,
                duration_s=arguments.duration_s,
                poll_rate_hz=arguments.poll_rate_hz,
                num_measurements=arguments.num_measurements,
                enable_filtering=not arguments.no_filtering,
            )
            print(json.dumps(result.to_mapping(), indent=2, sort_keys=True))
            return 0
        print(plot_qcar_lidar_capture(arguments.input, output_path=arguments.output))
        return 0
    except QCarLidarCaptureError as error:
        parser.exit(2, f"error: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
