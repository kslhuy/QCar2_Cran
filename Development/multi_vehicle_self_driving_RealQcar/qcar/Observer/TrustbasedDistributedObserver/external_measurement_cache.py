"""
External Measurement Cache

Stores and retrieves externally measured host-target relative states
(e.g. from YOLO detection or radar) for use in trust evaluation.
"""

import time
import numpy as np
from typing import Dict, Optional


class ExternalMeasurementCache:
    """Thread-safe cache for external relative measurements (e.g. YOLO / radar)."""

    def __init__(self, max_age_s: float = 1.0):
        self._data: Dict[int, Dict[str, float]] = {}
        self._max_age_ns = int(max(0.05, float(max_age_s)) * 1e9)

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
            if not np.isfinite(dist) or dist <= 0.0:
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
            self._data[tid] = {
                "distance": dist,
                "relative_velocity": rel_vel,
                "relative_bearing": rel_bearing,
                "confidence": rel_conf,
                "timestamp_ns": float(ts_ns),
                "source": str(source),
            }
            return True
        except Exception:
            return False

    def clear(self, target_id: Optional[int] = None) -> None:
        """Clear cached external relative measurement(s)."""
        if target_id is None:
            self._data.clear()
            return
        try:
            self._data.pop(int(target_id), None)
        except Exception:
            pass

    def get(self, target_id: int, current_time_ns: int) -> Optional[Dict[str, float]]:
        """Return fresh external relative measurement for the target, if available."""
        entry = self._data.get(int(target_id))
        if not entry:
            return None
        ts_ns = int(entry.get("timestamp_ns", 0.0))
        if ts_ns <= 0:
            return None
        if (int(current_time_ns) - ts_ns) > self._max_age_ns:
            return None
        return entry
