"""
Relative UIO Logger

Non-blocking CSV logger for the relative Unknown Input Observer (UIO).
This intentionally stays separate from the trust/weight logger so relative
observer diagnostics can be plotted independently.
"""

import csv
import os
import queue
import threading
from typing import Any, Dict, Iterable, List
# cd Development\multi_vehicle_self_driving_RealQcar\qcar\Observer\TrustbasedDistributedObserver
# python plot_trust_data.py --file ".\trust_weight_log_V1.csv" --relative-file "..\Relative_obs\relative_uio_log_V1.csv"


class RelativeUIOLogger:
    """Background CSV logger for relative observer samples."""

    def __init__(self, output_dir: str = None, max_queue_size: int = 10000):
        if output_dir is None:
            output_dir = os.path.dirname(os.path.abspath(__file__))

        self.output_dir = output_dir
        self.queue = queue.Queue(maxsize=max_queue_size)
        self.thread = None
        self.file = None
        self.writer = None
        self.recording = False

        self.columns = self._build_columns()

    @staticmethod
    def _build_columns() -> List[str]:
        base_columns = [
            "time",
            "timestamp_ns",
            "target_id",
            "dt",
            "measurement_used",
            "measurement_source",
            "measurement_confidence",
            "measurement_age_s",
            "measurement_distance",
            "measurement_relative_velocity",
            "v2v_fallback_distance",
            "v2v_fallback_relative_velocity",
            "v2v_fallback_relative_acceleration",
            "distance_error_vs_v2v",
            "relative_velocity_error_vs_v2v",
            "relative_acceleration_error_vs_v2v",
            "host_velocity_input",
            "target_acceleration_input",
            "delta_hat",
            "delta_dot_hat",
            "delta_ddot_hat",
            "f_c_hat",
        ]

        vector_columns = []
        for prefix in ("w", "host_state", "target_state"):
            width = 4 if prefix == "w" else 5
            for i in range(width):
                vector_columns.append(f"{prefix}_{i}")

        matrix_columns = []
        for prefix, rows, cols in (
            ("gain_K", 4, 2),
            ("gain_L", 4, 2),
            ("gain_N", 4, 4),
        ):
            for r in range(rows):
                for c in range(cols):
                    matrix_columns.append(f"{prefix}_{r}_{c}")

        return base_columns + vector_columns + matrix_columns

    @staticmethod
    def _to_float_or_nan(value: Any) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return float("nan")

    @classmethod
    def _add_vector(
        cls, row: Dict[str, Any], prefix: str, values: Iterable[Any], width: int
    ) -> None:
        clean_values = list(values) if values is not None else []
        for idx in range(width):
            value = clean_values[idx] if idx < len(clean_values) else float("nan")
            row[f"{prefix}_{idx}"] = cls._to_float_or_nan(value)

    @classmethod
    def _add_matrix(
        cls, row: Dict[str, Any], prefix: str, values: Any, rows: int, cols: int
    ) -> None:
        for r in range(rows):
            for c in range(cols):
                value = float("nan")
                try:
                    value = values[r][c]
                except (TypeError, IndexError, KeyError):
                    pass
                row[f"{prefix}_{r}_{c}"] = cls._to_float_or_nan(value)

    def start(self, vehicle_id: int, overwrite: bool = True) -> str:
        """Open the CSV and start the background writer thread."""
        if self.recording:
            return ""

        os.makedirs(self.output_dir, exist_ok=True)
        if overwrite:
            filename = f"relative_uio_log_V{vehicle_id}.csv"
        else:
            filename = f"relative_uio_log_V{vehicle_id}_{os.getpid()}.csv"
        filepath = os.path.join(self.output_dir, filename)

        try:
            self.file = open(filepath, "w", newline="", buffering=8192)
            self.writer = csv.DictWriter(self.file, fieldnames=self.columns)
            self.writer.writeheader()
            self.recording = True
            self.thread = threading.Thread(target=self._write_loop, daemon=True)
            self.thread.start()
            return filepath
        except Exception as e:
            print(f"[RelativeUIOLogger] Failed to open log file: {e}")
            self.recording = False
            self.file = None
            self.writer = None
            return ""

    def record(self, data: Dict[str, Any]) -> None:
        """Queue one relative UIO sample without blocking the observer loop."""
        if not self.recording:
            return

        row = {}
        for col in self.columns:
            row[col] = float("nan")

        row["time"] = self._to_float_or_nan(data.get("time", float("nan")))
        row["timestamp_ns"] = self._to_float_or_nan(
            data.get("timestamp_ns", float("nan"))
        )
        row["target_id"] = self._to_float_or_nan(data.get("target_id", float("nan")))
        row["dt"] = self._to_float_or_nan(data.get("dt", float("nan")))
        row["measurement_used"] = int(bool(data.get("measurement_used", False)))
        row["measurement_source"] = str(data.get("measurement_source", ""))
        row["measurement_confidence"] = self._to_float_or_nan(
            data.get("measurement_confidence", float("nan"))
        )
        row["measurement_age_s"] = self._to_float_or_nan(
            data.get("measurement_age_s", float("nan"))
        )
        row["measurement_distance"] = self._to_float_or_nan(
            data.get("measurement_distance", float("nan"))
        )
        row["measurement_relative_velocity"] = self._to_float_or_nan(
            data.get("measurement_relative_velocity", float("nan"))
        )
        row["v2v_fallback_distance"] = self._to_float_or_nan(
            data.get("v2v_fallback_distance", float("nan"))
        )
        row["v2v_fallback_relative_velocity"] = self._to_float_or_nan(
            data.get("v2v_fallback_relative_velocity", float("nan"))
        )
        row["v2v_fallback_relative_acceleration"] = self._to_float_or_nan(
            data.get("v2v_fallback_relative_acceleration", float("nan"))
        )
        row["distance_error_vs_v2v"] = self._to_float_or_nan(
            data.get("distance_error_vs_v2v", float("nan"))
        )
        row["relative_velocity_error_vs_v2v"] = self._to_float_or_nan(
            data.get("relative_velocity_error_vs_v2v", float("nan"))
        )
        row["relative_acceleration_error_vs_v2v"] = self._to_float_or_nan(
            data.get("relative_acceleration_error_vs_v2v", float("nan"))
        )
        row["host_velocity_input"] = self._to_float_or_nan(
            data.get("host_velocity_input", float("nan"))
        )
        row["target_acceleration_input"] = self._to_float_or_nan(
            data.get("target_acceleration_input", float("nan"))
        )

        state = data.get("state", [])
        row["delta_hat"] = self._to_float_or_nan(state[0] if len(state) > 0 else None)
        row["delta_dot_hat"] = self._to_float_or_nan(
            state[1] if len(state) > 1 else None
        )
        row["delta_ddot_hat"] = self._to_float_or_nan(
            state[2] if len(state) > 2 else None
        )
        row["f_c_hat"] = self._to_float_or_nan(state[3] if len(state) > 3 else None)

        self._add_vector(row, "w", data.get("w", []), 4)
        self._add_vector(row, "host_state", data.get("host_state", []), 5)
        self._add_vector(row, "target_state", data.get("target_state", []), 5)

        gains = data.get("gains", {})
        if not isinstance(gains, dict):
            gains = {}
        self._add_matrix(row, "gain_K", gains.get("K", []), 4, 2)
        self._add_matrix(row, "gain_L", gains.get("L", []), 4, 2)
        self._add_matrix(row, "gain_N", gains.get("N", []), 4, 4)

        try:
            self.queue.put_nowait(row)
        except queue.Full:
            pass

    def _write_loop(self) -> None:
        writes_since_flush = 0
        while self.recording or not self.queue.empty():
            try:
                row = self.queue.get(timeout=0.1)
                if self.writer is not None:
                    self.writer.writerow(row)
                self.queue.task_done()

                writes_since_flush += 1
                if writes_since_flush >= 50 and self.file:
                    self.file.flush()
                    writes_since_flush = 0
            except queue.Empty:
                continue
            except Exception:
                continue

    def stop(self) -> None:
        """Stop the writer thread and close the CSV."""
        self.recording = False
        if self.thread:
            self.thread.join(timeout=2.0)
            self.thread = None
        if self.file:
            try:
                self.file.flush()
                self.file.close()
            except Exception:
                pass
        self.file = None
        self.writer = None
