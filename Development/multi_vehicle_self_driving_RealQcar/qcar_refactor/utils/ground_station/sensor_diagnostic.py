"""Explicit, bounded vehicle-to-ground-station sensor diagnostic packets.

This transport helper belongs on the vehicle side. It is used only by a
requested capture/diagnostic session, so normal vehicle runtimes never forward
continuous raw sensor data to the ground station.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import socket
from typing import Any, Mapping

from core.vehicle_types import LaserScanSample


DIAGNOSTIC_SCHEMA = "qcar.sensor_diagnostic.v1"
MAX_DIAGNOSTIC_PACKET_BYTES = 60_000


class SensorDiagnosticError(ValueError):
    """Raised when an explicit diagnostic packet is malformed or unsafe."""


@dataclass(frozen=True)
class LidarDiagnosticFrame:
    """One decoded normalized LiDAR frame from a requested diagnostic session."""

    vehicle_id: int
    scan: LaserScanSample


class UdpLidarDiagnosticPublisher:
    """Send normalized scans to one requested ground-station viewer endpoint."""

    def __init__(self, host: str, port: int, *, vehicle_id: int = 0) -> None:
        if not isinstance(host, str) or not host:
            raise SensorDiagnosticError("Diagnostic host must be a non-empty string")
        _validate_port(port)
        _validate_vehicle_id(vehicle_id)
        self._endpoint = (host, port)
        self._vehicle_id = vehicle_id
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def publish(self, scan: LaserScanSample) -> None:
        self._socket.sendto(
            encode_lidar_diagnostic(scan, vehicle_id=self._vehicle_id), self._endpoint
        )

    def close(self) -> None:
        self._socket.close()


def encode_lidar_diagnostic(scan: LaserScanSample, *, vehicle_id: int) -> bytes:
    """Encode one common scan in a compact JSON/UDP diagnostic envelope."""

    if not isinstance(scan, LaserScanSample):
        raise SensorDiagnosticError("Diagnostic publisher requires a LaserScanSample")
    _validate_vehicle_id(vehicle_id)
    payload = {
        "schema": DIAGNOSTIC_SCHEMA,
        "type": "lidar",
        "vehicle_id": vehicle_id,
        "scan": {
            "timestamp_ns": scan.timestamp_ns,
            "frame_id": scan.frame_id,
            "angle_min_rad": scan.angle_min_rad,
            "angle_max_rad": scan.angle_max_rad,
            "angle_increment_rad": scan.angle_increment_rad,
            "time_increment_s": scan.time_increment_s,
            "scan_time_s": scan.scan_time_s,
            "range_min_m": scan.range_min_m,
            "range_max_m": scan.range_max_m,
            # JSON has no portable infinity. Null retains the ROS-shaped
            # invalid/out-of-range convention without a Python-specific token.
            "ranges_m": [float(value) if math.isfinite(value) else None for value in scan.ranges_m],
            "intensities": [float(value) if math.isfinite(value) else None for value in scan.intensities],
        },
    }
    encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")
    if len(encoded) > MAX_DIAGNOSTIC_PACKET_BYTES:
        raise SensorDiagnosticError(
            f"LiDAR diagnostic frame is {len(encoded)} bytes; limit is {MAX_DIAGNOSTIC_PACKET_BYTES}"
        )
    return encoded


def decode_lidar_diagnostic(packet: bytes) -> LidarDiagnosticFrame:
    """Decode and validate one normalized LiDAR diagnostic envelope."""

    if not isinstance(packet, bytes) or not packet:
        raise SensorDiagnosticError("Diagnostic packet must be non-empty bytes")
    if len(packet) > MAX_DIAGNOSTIC_PACKET_BYTES:
        raise SensorDiagnosticError("Diagnostic packet exceeds the configured byte limit")
    try:
        payload = json.loads(packet.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise SensorDiagnosticError("Diagnostic packet is not valid UTF-8 JSON") from error
    if not isinstance(payload, Mapping):
        raise SensorDiagnosticError("Diagnostic packet root must be a mapping")
    if payload.get("schema") != DIAGNOSTIC_SCHEMA or payload.get("type") != "lidar":
        raise SensorDiagnosticError("Unsupported diagnostic packet schema or type")
    vehicle_id = payload.get("vehicle_id")
    _validate_vehicle_id(vehicle_id)
    scan_values = payload.get("scan")
    if not isinstance(scan_values, Mapping):
        raise SensorDiagnosticError("LiDAR diagnostic packet has no scan mapping")
    ranges = _decode_float_sequence(scan_values.get("ranges_m"), "ranges_m", allow_null=True)
    intensities = _decode_float_sequence(scan_values.get("intensities", ()), "intensities", allow_null=True)
    if len(ranges) > 4096 or len(intensities) > 4096:
        raise SensorDiagnosticError("Diagnostic LiDAR scan exceeds 4096 bins")
    try:
        scan = LaserScanSample(
            timestamp_ns=scan_values["timestamp_ns"],
            frame_id=scan_values["frame_id"],
            angle_min_rad=scan_values["angle_min_rad"],
            angle_max_rad=scan_values["angle_max_rad"],
            angle_increment_rad=scan_values["angle_increment_rad"],
            time_increment_s=scan_values["time_increment_s"],
            scan_time_s=scan_values["scan_time_s"],
            range_min_m=scan_values["range_min_m"],
            range_max_m=scan_values["range_max_m"],
            ranges_m=ranges,
            intensities=intensities,
        )
    except (KeyError, TypeError, ValueError) as error:
        raise SensorDiagnosticError(f"Invalid normalized LiDAR scan: {error}") from error
    return LidarDiagnosticFrame(vehicle_id=vehicle_id, scan=scan)


def _decode_float_sequence(value: Any, name: str, *, allow_null: bool) -> tuple[float, ...]:
    if not isinstance(value, list):
        raise SensorDiagnosticError(f"Diagnostic {name} must be a JSON list")
    decoded: list[float] = []
    for item in value:
        if item is None and allow_null:
            decoded.append(math.inf)
            continue
        if not isinstance(item, (int, float)) or isinstance(item, bool) or not math.isfinite(item):
            raise SensorDiagnosticError(f"Diagnostic {name} values must be finite numbers or null")
        decoded.append(float(item))
    return tuple(decoded)


def _validate_vehicle_id(value: Any) -> None:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise SensorDiagnosticError("Diagnostic vehicle_id must be a non-negative integer")


def _validate_port(value: Any) -> None:
    if not isinstance(value, int) or isinstance(value, bool) or not 1 <= value <= 65535:
        raise SensorDiagnosticError("Diagnostic port must be an integer in [1, 65535]")
