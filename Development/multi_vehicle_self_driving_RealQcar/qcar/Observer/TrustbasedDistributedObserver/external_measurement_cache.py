"""
External Measurement Cache

Stores and retrieves externally measured host-target relative states
(e.g. from YOLO detection or radar) for use in trust evaluation.
"""

import time
from threading import RLock
from typing import Any, Dict, Optional

import numpy as np


class ExternalMeasurementCache:
    """Thread-safe cache for external relative measurements (e.g. YOLO / radar)."""

    def __init__(self, max_age_s: float = 1.0):
        max_age_s = float(max_age_s)
        if not np.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError("Measurement max age must be positive and finite")
        self._data: Dict[int, Dict[str, Any]] = {}
        self._max_age_ns = int(max_age_s * 1e9)
        self._lock = RLock()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set(
        self,
        target_id: int,
        distance: float,
        relative_velocity: Optional[float] = None,
        timestamp_ns: Optional[int] = None,
        source: str = "external_sensor",
        measurement_confidence: Optional[float] = None,
        relative_bearing: Optional[float] = None,
    ) -> bool:
        """Store externally measured host-target relative states."""
        try:
            tid = int(target_id)
            dist = float(distance)
            if tid < 0 or not np.isfinite(dist) or dist <= 0.0:
                return False

            rel_vel = float(relative_velocity) if relative_velocity is not None else float("nan")
            if not np.isfinite(rel_vel):
                rel_vel = float("nan")

            rel_conf = (
                float(measurement_confidence)
                if measurement_confidence is not None
                else float("nan")
            )
            if np.isfinite(rel_conf):
                rel_conf = float(np.clip(rel_conf, 0.0, 1.0))
            else:
                rel_conf = float("nan")

            rel_bearing = (
                float(relative_bearing)
                if relative_bearing is not None
                else float("nan")
            )
            if np.isfinite(rel_bearing):
                rel_bearing = float(np.arctan2(np.sin(rel_bearing), np.cos(rel_bearing)))
            else:
                rel_bearing = float("nan")

            ts_ns = int(timestamp_ns) if timestamp_ns is not None else int(time.time_ns())
            if ts_ns <= 0:
                return False
            entry = {
                "distance": dist,
                "relative_velocity": rel_vel,
                "relative_bearing": rel_bearing,
                "confidence": rel_conf,
                "timestamp_ns": ts_ns,
                "source": str(source),
            }
            with self._lock:
                previous = self._data.get(tid)
                if previous is not None and ts_ns < previous["timestamp_ns"]:
                    return False
                self._data[tid] = entry
            return True
        except (TypeError, ValueError, OverflowError):
            return False

    def clear(self, target_id: Optional[int] = None) -> None:
        """Clear cached external relative measurement(s)."""
        with self._lock:
            if target_id is None:
                self._data.clear()
            else:
                self._data.pop(int(target_id), None)

    def get(self, target_id: int, current_time_ns: int) -> Optional[Dict[str, Any]]:
        """Return fresh external relative measurement for the target, if available."""
        with self._lock:
            stored = self._data.get(int(target_id))
            entry = dict(stored) if stored is not None else None
        if not entry:
            return None
        ts_ns = int(entry.get("timestamp_ns", 0.0))
        if ts_ns <= 0:
            return None
        age_ns = int(current_time_ns) - ts_ns
        if age_ns < 0 or age_ns > self._max_age_ns:
            return None
        return entry
