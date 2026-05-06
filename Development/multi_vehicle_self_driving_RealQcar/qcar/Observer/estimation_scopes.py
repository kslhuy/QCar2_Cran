"""
CSV recording helpers for observer estimation data.

This module was reduced to the only runtime feature still used by the
observer: recording local and fleet estimator data to CSV.
"""

import csv
import os
from datetime import datetime
from numbers import Real
from typing import Dict, Iterable, List, Optional


LOCAL_RECORD_COLUMNS = [
    "x",
    "y",
    "theta",
    "velocity",
    "acceleration",
    "x_gps",
    "y_gps",
    "theta_gps",
    "steering",
    "throttle",
    "v_ref",
    "gps_valid",
]


def build_fleet_record_columns(max_vehicles: int = 5) -> List[str]:
    """Return flattened fleet CSV columns for the requested fleet size."""
    columns = ["consensus_error"]
    state_names = ["x", "y", "theta", "v", "a"]

    for vehicle_idx in range(max_vehicles):
        for state_name in state_names:
            columns.append(f"fleet_{state_name}_{vehicle_idx}")

    for vehicle_idx in range(max_vehicles):
        columns.append(f"trust_{vehicle_idx}")

    return columns


class ScopeDataRecorder:
    """Record local or fleet estimator snapshots to CSV."""

    def __init__(self, output_dir: str = "scope_recordings", max_vehicles: int = 5):
        self.output_dir = output_dir
        self.max_vehicles = max_vehicles
        self.file = None
        self.writer = None
        self.recording = False
        self.columns: List[str] = []

    def start(
        self,
        columns: Iterable[str],
        name: str = "scope",
        overwrite: bool = False,
    ) -> str:
        """
        Open a CSV file and write the header.

        Args:
            columns: Data columns excluding the mandatory ``time`` column.
            name: File prefix such as ``local_V0`` or ``fleet_V0``.
            overwrite: If True, use ``{name}.csv``. Otherwise append a time tag.
        """
        os.makedirs(self.output_dir, exist_ok=True)

        if overwrite:
            filepath = os.path.join(self.output_dir, f"{name}.csv")
        else:
            timestamp = datetime.now().strftime("%H%M%S")
            filepath = os.path.join(self.output_dir, f"{name}_{timestamp}.csv")

        self.columns = ["time"] + list(columns)
        self.file = open(filepath, "w", newline="", buffering=8192)
        self.writer = csv.DictWriter(self.file, fieldnames=self.columns)
        self.writer.writeheader()
        self.recording = True
        return filepath

    def _flatten_data(self, data: Dict[str, object]) -> Dict[str, float]:
        """Flatten fleet arrays and trust dictionaries into scalar CSV columns."""
        flattened: Dict[str, float] = {}

        for key, value in data.items():
            if key == "fleet_states" and hasattr(value, "shape"):
                state_names = ["x", "y", "theta", "v", "a"]
                num_states = min(value.shape[0], len(state_names))
                num_vehicles = min(value.shape[1], self.max_vehicles)

                for vehicle_idx in range(num_vehicles):
                    for state_idx in range(num_states):
                        column = f"fleet_{state_names[state_idx]}_{vehicle_idx}"
                        flattened[column] = float(value[state_idx, vehicle_idx])
            elif key == "trust_scores" and isinstance(value, dict):
                for vehicle_idx in range(self.max_vehicles):
                    column = f"trust_{vehicle_idx}"
                    flattened[column] = float(value.get(vehicle_idx, 0.5))
            else:
                scalar = self._to_float(value)
                if scalar is not None:
                    flattened[key] = scalar

        return flattened

    @staticmethod
    def _to_float(value: object) -> Optional[float]:
        """Convert scalar-like values to float, otherwise return None."""
        if isinstance(value, Real):
            return float(value)

        try:
            item = value.item()
        except AttributeError:
            return None
        except Exception:
            return None

        return float(item) if isinstance(item, Real) else None

    def record(self, t: float, data: Dict[str, object]) -> None:
        """Append one sample to the CSV file."""
        if not self.recording or self.writer is None:
            return

        try:
            flattened = self._flatten_data(data)
            row = {"time": float(t)}

            for column in self.columns[1:]:
                row[column] = flattened.get(column, 0.0)

            self.writer.writerow(row)
        except Exception:
            pass

    def stop(self) -> None:
        """Flush and close the CSV file."""
        self.recording = False

        if self.file is None:
            return

        try:
            self.file.flush()
            self.file.close()
        except Exception:
            pass
        finally:
            self.file = None
            self.writer = None


__all__ = [
    "LOCAL_RECORD_COLUMNS",
    "build_fleet_record_columns",
    "ScopeDataRecorder",
]
