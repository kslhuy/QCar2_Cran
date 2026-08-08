"""Stationary physical-QCar IO capture and local plotting utility.

The ``capture`` command runs on the QCar. It opens PAL's ``QCar`` device and,
when explicitly requested, its ``QCarLidar`` device. It exercises the
sensor-read path through :class:`utils.io.io_qcar2.IOQCar2` and writes CSV
artifacts plus metadata. It never starts ``VehicleRuntime``, creates a route,
or calls ``QCar.write``/``IOQCar2.stop``. PAL termination still performs its
own final neutral cleanup when the device session closes.

The ``plot`` command can run on the ground station after the CSV is fetched
from the active release. Keeping plotting local means the vehicle does not
need a GUI or matplotlib installation.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import time
from typing import Any, Callable, Mapping, Sequence

from core.vehicle_types import LaserScanSample
from core.vehicle_config import ConfigError, load_config
from extra.platform.qcar.process_runner import QCarFactory, QCarLidarFactory, qcar_resource_context
from utils.io.io_qcar2 import IOQCar2


_CSV_NAME = "real_qcar_io.csv"
_METADATA_NAME = "real_qcar_io.json"
_PLOT_NAME = "real_qcar_io.png"
_LIDAR_CSV_NAME = "real_qcar_lidar_normalized.csv"
_LIDAR_PLOT_NAME = "real_qcar_lidar_normalized.png"
_FIELDS = (
    "elapsed_s",
    "sensor_timestamp_s",
    "motor_tach",
    "gyro_z_rad_s",
    "accel_x_m_s2",
    "accel_y_m_s2",
    "accel_z_m_s2",
)
_LIDAR_FIELDS = (
    "scan_timestamp_ns",
    "scan_index",
    "bin_index",
    "angle_rad",
    "range_m",
    "valid",
)


class QCarIOCaptureError(RuntimeError):
    """Raised when a read-only physical-QCar capture cannot be completed."""


@dataclass(frozen=True)
class QCarIOCaptureResult:
    """Paths and summary data produced by one physical IO capture."""

    csv_path: Path
    metadata_path: Path
    sample_count: int
    duration_s: float
    sample_rate_hz: float
    lidar_csv_path: Path | None = None
    lidar_scan_count: int = 0
    lidar_point_count: int = 0
    lidar_status: Mapping[str, object] | None = None

    def to_mapping(self) -> dict[str, object]:
        values = asdict(self)
        values["csv_path"] = str(self.csv_path)
        values["metadata_path"] = str(self.metadata_path)
        values["lidar_csv_path"] = (
            str(self.lidar_csv_path) if self.lidar_csv_path is not None else None
        )
        values["lidar_status"] = dict(self.lidar_status or {})
        return values


def capture_qcar_io(
    *,
    output_directory: str | Path,
    vehicle_config_file: str = "config_vehicle_deployment_test.yaml",
    duration_s: float = 10.0,
    sample_rate_hz: float = 50.0,
    read_mode: int = 1,
    qcar_factory: QCarFactory | None = None,
    lidar_factory: QCarLidarFactory | None = None,
    include_lidar: bool = False,
    lidar_scan_observer: Callable[[LaserScanSample], None] | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> QCarIOCaptureResult:
    """Capture stationary physical-QCar IO samples without actuator writes.

    Call this only while no ``targets-start`` vehicle runtime owns the QCar.
    The vehicle must be physically supported with its wheels clear. The
    capture does not command motion, but PAL performs a final neutral cleanup
    when its device session is released. ``include_lidar`` makes the capture
    instantiate the selected direct PAL LiDAR resource, start the local
    ``IOQCar2`` worker, and save its *normalized* `LaserScanSample` frames.
    It does not consume the normal runtime/localisation queue. An optional
    observer receives each drained scan only for an explicit diagnostic
    capture, such as the Qt ground-station viewer.
    """

    _validate_capture_arguments(duration_s, sample_rate_hz, read_mode)
    config = load_config(vehicle_config_file=vehicle_config_file)
    io_config = config.module("io")
    if io_config.get("implementation") != "qcar":
        raise ConfigError("Physical IO capture requires a vehicle configuration with modules.io: qcar")
    io_config, lidar_config = _capture_io_config(io_config, include_lidar=include_lidar)

    period_s = 1.0 / sample_rate_hz
    started_at = monotonic()
    samples: list[dict[str, float]] = []
    lidar_scans: list[LaserScanSample] = []
    next_sample_at = started_at
    lidar_status: dict[str, object] = {}
    with qcar_resource_context(
        qcar_factory=qcar_factory,
        lidar_factory=lidar_factory,
        lidar_config=lidar_config,
        read_mode=read_mode,
    ) as resources:
        io = IOQCar2(
            io_config,
            qcar=resources["qcar"],
            lidar=resources.get("lidar"),
            vehicle_id=config.vehicle_id,
        )
        try:
            while monotonic() - started_at < duration_s:
                # The plain sensor capture directly polls only required QCar
                # state. The explicit LiDAR mode instead calls the regular IO
                # refresh once, which starts the independent local worker.
                if include_lidar:
                    io.read_to_cache()
                    _collect_lidar_scans(io, lidar_scans, lidar_scan_observer)
                else:
                    # Do not call IOQCar2.stop()/close() in this diagnostic:
                    # they deliberately write a neutral command.
                    io._poll_sensors()
                sensor = io.read()
                samples.append(
                    {
                        "elapsed_s": monotonic() - started_at,
                        "sensor_timestamp_s": float(sensor.sensor_timestamp),
                        "motor_tach": float(sensor.motor_tach),
                        "gyro_z_rad_s": float(sensor.gyro_z),
                        "accel_x_m_s2": float(sensor.accelerometer[0]),
                        "accel_y_m_s2": float(sensor.accelerometer[1]),
                        "accel_z_m_s2": float(sensor.accelerometer[2]),
                    }
                )
                next_sample_at += period_s
                sleep(max(0.0, next_sample_at - monotonic()))
            if include_lidar:
                _collect_lidar_scans(io, lidar_scans, lidar_scan_observer)
                lidar_status = dict(io.lidar_status())
        finally:
            # Stop the adapter-owned worker before the platform context
            # releases PAL resources. Do not use close(), which writes zero.
            io._close_impl()

    if len(samples) < 2:
        raise QCarIOCaptureError("Physical QCar IO capture produced fewer than two samples")
    if len({sample["sensor_timestamp_s"] for sample in samples}) < 2:
        raise QCarIOCaptureError("Physical QCar sensor timestamps did not advance")
    if include_lidar and len(lidar_scans) < 2:
        raise QCarIOCaptureError(
            "Physical QCar LiDAR IO capture produced fewer than two normalized scans; "
            "verify that no other process owns the sensor"
        )

    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    csv_path = destination / _CSV_NAME
    metadata_path = destination / _METADATA_NAME
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=_FIELDS)
        writer.writeheader()
        writer.writerows(samples)
    lidar_csv_path = None
    if include_lidar:
        lidar_csv_path = destination / _LIDAR_CSV_NAME
        _write_lidar_capture_csv(lidar_csv_path, lidar_scans)
    result = QCarIOCaptureResult(
        csv_path=csv_path,
        metadata_path=metadata_path,
        sample_count=len(samples),
        duration_s=duration_s,
        sample_rate_hz=sample_rate_hz,
        lidar_csv_path=lidar_csv_path,
        lidar_scan_count=len(lidar_scans),
        lidar_point_count=sum(len(scan.ranges_m) for scan in lidar_scans),
        lidar_status=lidar_status or None,
    )
    metadata_path.write_text(
        json.dumps(
            {
                **result.to_mapping(),
                "vehicle_id": config.vehicle_id,
                "vehicle_config_file": vehicle_config_file,
                "read_mode": read_mode,
                "actuator_writes": 0,
                "lidar_enabled": include_lidar,
                "lidar_capture_contract": "normalized_laser_scan" if include_lidar else None,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return result


def plot_qcar_io_lidar_capture(
    csv_path: str | Path, *, output_path: str | Path | None = None
) -> Path:
    """Render the newest normalized `IOQCar2` scan as a Cartesian PNG."""

    source = Path(csv_path)
    rows = _read_lidar_capture_csv(source)
    newest_index = max(int(row["scan_index"]) for row in rows)
    newest = [
        row for row in rows if int(row["scan_index"]) == newest_index and bool(row["valid"])
    ]
    if not newest:
        raise QCarIOCaptureError("Newest normalized QCar LiDAR scan has no valid ranges")
    destination = Path(output_path) if output_path is not None else source.with_name(_LIDAR_PLOT_NAME)
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as pyplot
    except ImportError as error:
        raise QCarIOCaptureError(
            "Plotting requires matplotlib on the ground-station Python environment"
        ) from error
    angles = [float(row["angle_rad"]) for row in newest]
    ranges = [float(row["range_m"]) for row in newest]
    figure, axis = pyplot.subplots(figsize=(8, 8), constrained_layout=True)
    axis.scatter(
        [distance * math.cos(angle) for distance, angle in zip(ranges, angles)],
        [distance * math.sin(angle) for distance, angle in zip(ranges, angles)],
        s=5,
        label=f"normalized scan {newest_index}",
    )
    axis.scatter([0.0], [0.0], marker="+", s=80, label="QCar LiDAR origin")
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.set_title("Physical QCar IO normalized LiDAR capture")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=150)
    pyplot.close(figure)
    return destination


def plot_qcar_io_capture(
    csv_path: str | Path, *, output_path: str | Path | None = None
) -> Path:
    """Render one locally fetched physical-QCar IO CSV as a PNG plot."""

    source = Path(csv_path)
    rows = _read_capture_csv(source)
    destination = Path(output_path) if output_path is not None else source.with_name(_PLOT_NAME)
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as pyplot
    except ImportError as error:
        raise QCarIOCaptureError(
            "Plotting requires matplotlib on the ground-station Python environment"
        ) from error

    elapsed = [row["elapsed_s"] for row in rows]
    figure, axes = pyplot.subplots(3, 1, figsize=(10, 9), sharex=True, constrained_layout=True)
    axes[0].plot(elapsed, [row["motor_tach"] for row in rows], label="motor tach")
    axes[0].plot(elapsed, [row["gyro_z_rad_s"] for row in rows], label="gyro z")
    axes[0].set_ylabel("sensor value")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(elapsed, [row["accel_x_m_s2"] for row in rows], label="accel x")
    axes[1].plot(elapsed, [row["accel_y_m_s2"] for row in rows], label="accel y")
    axes[1].plot(elapsed, [row["accel_z_m_s2"] for row in rows], label="accel z")
    axes[1].set_ylabel("acceleration (m/s²)")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    sensor_time = [row["sensor_timestamp_s"] - rows[0]["sensor_timestamp_s"] for row in rows]
    axes[2].plot(elapsed, sensor_time, label="sensor timestamp")
    axes[2].plot(elapsed, elapsed, linestyle="--", label="capture time")
    axes[2].set_xlabel("capture time (s)")
    axes[2].set_ylabel("relative time (s)")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=150)
    pyplot.close(figure)
    return destination


def _read_capture_csv(path: Path) -> list[dict[str, float]]:
    try:
        with path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            if tuple(reader.fieldnames or ()) != _FIELDS:
                raise QCarIOCaptureError(f"Unexpected physical-QCar CSV columns in {path}")
            rows = [{field: float(row[field]) for field in _FIELDS} for row in reader]
    except OSError as error:
        raise QCarIOCaptureError(f"Cannot read physical-QCar IO CSV {path}: {error}") from error
    except (KeyError, TypeError, ValueError) as error:
        raise QCarIOCaptureError(f"Invalid physical-QCar IO CSV {path}: {error}") from error
    if len(rows) < 2:
        raise QCarIOCaptureError("Physical-QCar IO CSV must contain at least two samples")
    return rows


def _capture_io_config(
    io_config: Mapping[str, object], *, include_lidar: bool
) -> tuple[dict[str, object], dict[str, object] | None]:
    """Copy an IO profile and enable LiDAR only for this explicit diagnostic."""

    copied = dict(io_config)
    if not include_lidar:
        return copied, None
    configured_sensors = io_config.get("sensors", {})
    sensors = dict(configured_sensors) if isinstance(configured_sensors, Mapping) else {}
    configured_lidar = sensors.get("lidar", {})
    lidar = dict(configured_lidar) if isinstance(configured_lidar, Mapping) else {}
    lidar["enabled"] = True
    lidar.setdefault("source", "qcar_hardware")
    sensors["lidar"] = lidar
    copied["sensors"] = sensors
    return copied, lidar


def _write_lidar_capture_csv(path: Path, scans: Sequence[LaserScanSample]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=_LIDAR_FIELDS)
        writer.writeheader()
        for scan_index, scan in enumerate(scans):
            for bin_index, distance in enumerate(scan.ranges_m):
                value = float(distance)
                writer.writerow(
                    {
                        "scan_timestamp_ns": scan.timestamp_ns,
                        "scan_index": scan_index,
                        "bin_index": bin_index,
                        "angle_rad": scan.angle_min_rad + bin_index * scan.angle_increment_rad,
                        "range_m": value,
                        "valid": math.isfinite(value)
                        and scan.range_min_m <= value <= scan.range_max_m,
                    }
                )


def _collect_lidar_scans(
    io: IOQCar2,
    destination: list[LaserScanSample],
    observer: Callable[[LaserScanSample], None] | None,
) -> None:
    scans = io.drain_lidar_scans()
    destination.extend(scans)
    if observer is not None:
        for scan in scans:
            observer(scan)


def _read_lidar_capture_csv(path: Path) -> list[dict[str, object]]:
    try:
        with path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            if tuple(reader.fieldnames or ()) != _LIDAR_FIELDS:
                raise QCarIOCaptureError(f"Unexpected normalized QCar LiDAR CSV columns in {path}")
            rows = [
                {
                    "scan_timestamp_ns": int(row["scan_timestamp_ns"]),
                    "scan_index": int(row["scan_index"]),
                    "bin_index": int(row["bin_index"]),
                    "angle_rad": float(row["angle_rad"]),
                    "range_m": float(row["range_m"]),
                    "valid": row["valid"].strip().lower() == "true",
                }
                for row in reader
            ]
    except OSError as error:
        raise QCarIOCaptureError(f"Cannot read normalized QCar LiDAR CSV {path}: {error}") from error
    except (KeyError, TypeError, ValueError) as error:
        raise QCarIOCaptureError(f"Invalid normalized QCar LiDAR CSV {path}: {error}") from error
    if not rows:
        raise QCarIOCaptureError("Normalized QCar LiDAR CSV has no scan points")
    return rows


def _validate_capture_arguments(duration_s: float, sample_rate_hz: float, read_mode: int) -> None:
    if not 1.0 <= duration_s <= 300.0:
        raise QCarIOCaptureError("--duration-s must be between 1 and 300 seconds")
    if not 1.0 <= sample_rate_hz <= 100.0:
        raise QCarIOCaptureError("--sample-rate-hz must be between 1 and 100 Hz")
    if not isinstance(read_mode, int) or isinstance(read_mode, bool) or read_mode < 0:
        raise QCarIOCaptureError("--read-mode must be a non-negative integer")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Capture and plot read-only physical-QCar IO diagnostics")
    commands = parser.add_subparsers(dest="command", required=True)
    capture = commands.add_parser("capture", help="run a stationary QCar IO capture")
    # The documented command runs from ``current/payload``. Put artifacts one
    # level above it so ``fetch-artifact`` can address them relative to the
    # active release root (``current/artifacts/...``).
    capture.add_argument("--output-dir", default="../artifacts/real_qcar_io")
    capture.add_argument("--vehicle-config", default="config_vehicle_deployment_test.yaml")
    capture.add_argument("--duration-s", type=float, default=10.0)
    capture.add_argument("--sample-rate-hz", type=float, default=50.0)
    capture.add_argument("--read-mode", type=int, default=1)
    capture.add_argument(
        "--with-lidar",
        action="store_true",
        help="also verify the local IOQCar2 LiDAR worker and save normalized scans",
    )
    capture.add_argument(
        "--diagnostic-host",
        help="optional Qt diagnostic-viewer host; allowed only with --with-lidar",
    )
    capture.add_argument("--diagnostic-port", type=int, default=5001)
    plot = commands.add_parser("plot", help="plot a fetched physical-QCar CSV on the ground station")
    plot.add_argument("--input", required=True)
    plot.add_argument("--output")
    plot_lidar = commands.add_parser(
        "plot-lidar", help="plot a fetched normalized IOQCar2 LiDAR scan on the ground station"
    )
    plot_lidar.add_argument("--input", required=True)
    plot_lidar.add_argument("--output")
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "capture":
            if arguments.diagnostic_host and not arguments.with_lidar:
                parser.error("--diagnostic-host requires --with-lidar")
            if not 1 <= arguments.diagnostic_port <= 65535:
                parser.error("--diagnostic-port must be in [1, 65535]")
            publisher = None
            if arguments.diagnostic_host:
                from utils.ground_station.sensor_diagnostic import UdpLidarDiagnosticPublisher

                publisher = UdpLidarDiagnosticPublisher(
                    arguments.diagnostic_host,
                    arguments.diagnostic_port,
                    vehicle_id=load_config(vehicle_config_file=arguments.vehicle_config).vehicle_id,
                )
            try:
                print(json.dumps(capture_qcar_io(
                    output_directory=arguments.output_dir,
                    vehicle_config_file=arguments.vehicle_config,
                    duration_s=arguments.duration_s,
                    sample_rate_hz=arguments.sample_rate_hz,
                    read_mode=arguments.read_mode,
                    include_lidar=arguments.with_lidar,
                    lidar_scan_observer=publisher.publish if publisher is not None else None,
                ).to_mapping(), indent=2, sort_keys=True))
            finally:
                if publisher is not None:
                    publisher.close()
            return 0
        if arguments.command == "plot":
            print(plot_qcar_io_capture(arguments.input, output_path=arguments.output))
        else:
            print(plot_qcar_io_lidar_capture(arguments.input, output_path=arguments.output))
        return 0
    except KeyboardInterrupt:
        # The ground-station diagnostic manager uses SIGINT through its
        # remote wrapper so the capture's resource-context ``finally`` blocks
        # run.  That normal operator stop should not leave a traceback in the
        # vehicle diagnostic log.
        return 0
    except (ConfigError, QCarIOCaptureError) as error:
        parser.exit(2, f"error: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
