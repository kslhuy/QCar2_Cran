"""Shared lifecycle contract for LiDAR scan-matching algorithms."""

from __future__ import annotations

from abc import ABC, abstractmethod

from core.vehicle_types import LaserScanSample, PoseMeasurement


class ScanMatchAlgorithm(ABC):
    """Convert LiDAR scans to map poses without owning a sensor or runtime.

    Concrete implementations own only their matcher, map, and calibration
    state.  Platform contexts own native LiDAR acquisition; a future runtime
    manager may select algorithms or check freshness through composition.
    """

    def __init__(self) -> None:
        self._started = False
        self._closed = False

    def start(self, initial_pose: PoseMeasurement | None = None) -> None:
        if self._closed:
            raise RuntimeError("Cannot start a closed ScanMatchAlgorithm")
        self._on_start(initial_pose)
        self._started = True

    def update(self, scan: LaserScanSample) -> PoseMeasurement:
        if not self._started:
            raise RuntimeError("ScanMatchAlgorithm must be started before update")
        if self._closed:
            raise RuntimeError("Cannot update a closed ScanMatchAlgorithm")
        if not isinstance(scan, LaserScanSample):
            raise TypeError("ScanMatchAlgorithm.update requires a LaserScanSample")
        pose = self.localize(scan)
        if not isinstance(pose, PoseMeasurement):
            raise TypeError("ScanMatchAlgorithm.localize must return a PoseMeasurement")
        return pose

    def reset(self, initial_pose: PoseMeasurement | None = None) -> None:
        if self._closed:
            raise RuntimeError("Cannot reset a closed ScanMatchAlgorithm")
        self._on_reset(initial_pose)
        self._started = True

    def close(self) -> None:
        if self._closed:
            return
        try:
            self._on_close()
        finally:
            self._started = False
            self._closed = True

    def _on_start(self, initial_pose: PoseMeasurement | None) -> None:
        del initial_pose

    def _on_reset(self, initial_pose: PoseMeasurement | None) -> None:
        del initial_pose

    def _on_close(self) -> None:
        """Optional cleanup hook for algorithm-owned resources."""

    @abstractmethod
    def localize(self, scan: LaserScanSample) -> PoseMeasurement:
        """Return one map-frame pose from a LiDAR scan."""
