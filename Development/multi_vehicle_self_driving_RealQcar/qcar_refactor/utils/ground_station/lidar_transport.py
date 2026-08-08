"""Bounded normalized LiDAR payload for the ground-station TCP protocol."""

from __future__ import annotations

import math
from typing import Any, Mapping

from core.vehicle_types import LaserScanSample


MAX_LIDAR_SCAN_BINS = 4096


def lidar_scan_to_mapping(scan: LaserScanSample, *, vehicle_id: int) -> dict[str, object]:
    """Encode one common scan without exposing platform-native sensor data."""

    if not isinstance(scan, LaserScanSample):
        raise TypeError("LiDAR transport requires a LaserScanSample")
    _vehicle_id(vehicle_id)
    _validate_scan_size(scan)
    return {
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
            # Null preserves ROS-shaped invalid/out-of-range values across
            # MessagePack implementations without relying on NaN/Infinity.
            "ranges_m": [_finite_or_none(value) for value in scan.ranges_m],
            "intensities": [_finite_or_none(value) for value in scan.intensities],
        },
    }


def lidar_scan_from_mapping(payload: Mapping[str, Any]) -> tuple[int, LaserScanSample]:
    """Decode one bounded scan sent by the matching registered vehicle."""

    if not isinstance(payload, Mapping):
        raise ValueError("LiDAR frame payload must be a mapping")
    vehicle_id = _vehicle_id(payload.get("vehicle_id"))
    values = payload.get("scan")
    if not isinstance(values, Mapping):
        raise ValueError("LiDAR frame requires a scan mapping")
    ranges = _float_sequence(values.get("ranges_m"), "ranges_m")
    intensities = _float_sequence(values.get("intensities", ()), "intensities")
    if len(ranges) > MAX_LIDAR_SCAN_BINS or len(intensities) > MAX_LIDAR_SCAN_BINS:
        raise ValueError(f"LiDAR frame exceeds {MAX_LIDAR_SCAN_BINS} bins")
    try:
        scan = LaserScanSample(
            timestamp_ns=values["timestamp_ns"],
            frame_id=values["frame_id"],
            angle_min_rad=values["angle_min_rad"],
            angle_max_rad=values["angle_max_rad"],
            angle_increment_rad=values["angle_increment_rad"],
            time_increment_s=values["time_increment_s"],
            scan_time_s=values["scan_time_s"],
            range_min_m=values["range_min_m"],
            range_max_m=values["range_max_m"],
            ranges_m=ranges,
            intensities=intensities,
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"Invalid normalized LiDAR scan: {error}") from error
    return vehicle_id, scan


def _finite_or_none(value: float) -> float | None:
    value = float(value)
    return value if math.isfinite(value) else None


def _float_sequence(value: Any, name: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)):
        raise ValueError(f"LiDAR {name} must be a sequence")
    result = []
    for item in value:
        if item is None:
            result.append(math.inf)
        elif isinstance(item, (int, float)) and not isinstance(item, bool) and math.isfinite(float(item)):
            result.append(float(item))
        else:
            raise ValueError(f"LiDAR {name} values must be finite numbers or null")
    return tuple(result)


def _validate_scan_size(scan: LaserScanSample) -> None:
    if len(scan.ranges_m) > MAX_LIDAR_SCAN_BINS or len(scan.intensities) > MAX_LIDAR_SCAN_BINS:
        raise ValueError(f"LiDAR scan exceeds {MAX_LIDAR_SCAN_BINS} bins")


def _vehicle_id(value: Any) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError("LiDAR vehicle_id must be a non-negative integer")
    return value
