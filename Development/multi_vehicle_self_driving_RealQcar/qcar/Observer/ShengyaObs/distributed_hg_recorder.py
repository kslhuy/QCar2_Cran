"""
Distributed high-gain observer data recorder.

This recorder keeps the same non-blocking write path as the distributed
Luenberger recorder, but uses HG-specific filenames and 0-based vehicle
state columns that match the HG observer state:

    [x0, v0, a0, x1, v1, a1, ...]
"""
import csv
import os
import queue
import re
import threading
import time
from datetime import datetime
from typing import Dict, List

import numpy as np


class DistributedHGRecorder:
    """Non-blocking recorder for DistributedHGEstimator debug data."""

    _STOP_SENTINEL = object()
    _session_lock = threading.Lock()
    _active_sessions = {}

    def __init__(self, output_dir: str = "observer_recordings",
                 vehicle_id: int = 0,
                 observer_size: int = 4,
                 fleet_size: int = 4,
                 queue_size: int = 1000,
                 flush_interval: float = 5.0,
                 run_id: str = None):
        self.output_dir = output_dir
        self.vehicle_id = vehicle_id
        self.observer_size = observer_size
        self.fleet_size = fleet_size
        self.queue_size = queue_size
        self.flush_interval = flush_interval
        self.run_id = run_id
        self.run_dir = None

        self.file = None
        self.writer = None
        self.recording = False
        self.filepath = None
        self.columns = []

        self._write_queue = None
        self._writer_thread = None
        self._last_flush_time = 0.0
        self._record_count = 0
        self._dropped_count = 0

    def start(self) -> str:
        """Start recording in a run directory shared by all fleet vehicles."""
        os.makedirs(self.output_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = self._get_run_directory(timestamp)
        os.makedirs(self.run_dir, exist_ok=True)

        filename = f"dist_hg_v{self.vehicle_id}_{timestamp}.csv"
        self.filepath = os.path.join(self.run_dir, filename)

        self.columns = self._build_columns()
        self.file = open(self.filepath, "w", newline="", buffering=8192)
        self.writer = csv.DictWriter(self.file, fieldnames=self.columns)
        self.writer.writeheader()
        self.file.flush()

        self._write_queue = queue.Queue(maxsize=self.queue_size)
        self._last_flush_time = time.time()
        self._record_count = 0
        self._dropped_count = 0
        self.recording = True

        self._writer_thread = threading.Thread(
            target=self._writer_loop,
            name=f"HGRecorder_V{self.vehicle_id}",
            daemon=True,
        )
        self._writer_thread.start()
        return self.filepath

    def _get_run_directory(self, timestamp: str) -> str:
        """Return one run directory for the recorders of the current fleet.

        Estimators for a simulated fleet are normally constructed sequentially
        in one process.  The process-local registry keeps their run directory
        stable even when construction crosses a one-second timestamp boundary.
        An explicit ``run_id`` can be used when recorders run in separate
        processes or on separate machines with a shared output directory.
        """
        if self.run_id is not None:
            safe_run_id = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(self.run_id)).strip("._")
            if not safe_run_id:
                raise ValueError("run_id must contain at least one valid character")
            return os.path.join(self.output_dir, f"dist_hg_run_{safe_run_id}")

        session_key = (os.path.abspath(self.output_dir), self.fleet_size)
        with self._session_lock:
            session = self._active_sessions.get(session_key)
            if session is None or self.vehicle_id in session["vehicle_ids"]:
                session = {
                    "run_dir": self._new_run_directory(timestamp),
                    "vehicle_ids": set(),
                }
                self._active_sessions[session_key] = session

            session["vehicle_ids"].add(self.vehicle_id)
            run_dir = session["run_dir"]

            if len(session["vehicle_ids"]) >= self.fleet_size:
                self._active_sessions.pop(session_key, None)

            return run_dir

    def _new_run_directory(self, timestamp: str) -> str:
        """Choose a fresh directory without overwriting an earlier run."""
        base_dir = os.path.join(self.output_dir, f"dist_hg_run_{timestamp}")
        candidate = base_dir
        suffix = 1
        vehicle_prefix = f"dist_hg_v{self.vehicle_id}_"

        while os.path.isdir(candidate):
            # Another process may already have created this run in the same
            # second. Join it if this vehicle has not claimed a file there.
            if not any(
                name.startswith(vehicle_prefix) and name.endswith(".csv")
                for name in os.listdir(candidate)
            ):
                return candidate
            candidate = f"{base_dir}_{suffix:02d}"
            suffix += 1

        return candidate

    def _build_columns(self) -> List[str]:
        columns = ["time"]

        for prefix in ["x_vec"]:
            for vid in range(self.observer_size):
                columns.extend([
                    f"{prefix}_x{vid}",
                    f"{prefix}_v{vid}",
                    f"{prefix}_a{vid}",
                ])

        for term in ["dynamics", "measurement", "consensus"]:
            for vid in range(self.observer_size):
                columns.extend([
                    f"{term}_x{vid}",
                    f"{term}_v{vid}",
                    f"{term}_a{vid}",
                ])

        columns.extend([
            "local_meas_x",
            "est_meas_x",
            "meas_err_x",
            "neighbor_count",
            "consensus_norm",
        ])

        for vid in range(self.observer_size):
            columns.append(f"collective_control_{vid}")

        for vid in range(self.fleet_size):
            columns.extend([
                f"fleet_x_{vid}",
                f"fleet_y_{vid}",
                f"fleet_theta_{vid}",
                f"fleet_v_{vid}",
                f"fleet_a_{vid}",
            ])

        columns.extend([
            "dt",
            "position",
            "velocity",
            "acceleration",
            "control_input",
            "local_measurement_p",
            "local_measurement_v",
        ])

        for vid in range(self.fleet_size):
            columns.extend([
                f"true_position_{vid}",
                f"true_velocity_{vid}",
                f"true_acceleration_{vid}",
                f"true_throttle_{vid}",
            ])

        return columns

    def record(self, t: float, debug_data: Dict) -> bool:
        if not self.recording or self._write_queue is None or not debug_data:
            return False

        try:
            row = self._build_row(t, debug_data)
            try:
                self._write_queue.put_nowait(row)
                return True
            except queue.Full:
                self._dropped_count += 1
                return False
        except Exception:
            return False

    def _build_row(self, t: float, debug_data: Dict) -> Dict:
        row = {"time": t}

        data = debug_data.get("x_vec", None)
        self._write_vector_blocks(row, "x_vec", data)

        for term_key, term_name in [
            ("dynamics_term", "dynamics"),
            ("measurement_term", "measurement"),
            ("consensus_term", "consensus"),
        ]:
            self._write_vector_blocks(row, term_name, debug_data.get(term_key, None))

        row["local_meas_x"] = self._first(debug_data.get("local_measurement", None))
        row["est_meas_x"] = self._first(debug_data.get("estimated_measurement", None))
        row["meas_err_x"] = self._first(debug_data.get("measurement_error", None))
        row["neighbor_count"] = debug_data.get("neighbor_count", 0)
        row["consensus_norm"] = debug_data.get("consensus_norm", 0.0)

        collective_control = debug_data.get("collective_control", None)
        if collective_control is not None:
            flat_control = np.asarray(collective_control).reshape(-1)
            for vid in range(self.observer_size):
                row[f"collective_control_{vid}"] = (
                    flat_control[vid] if vid < len(flat_control) else np.nan
                )

        fleet = debug_data.get("fleet_states", None)
        if fleet is not None:
            fleet = np.asarray(fleet)
            for vid in range(self.fleet_size):
                if vid < fleet.shape[1]:
                    row[f"fleet_x_{vid}"] = fleet[0, vid] if fleet.shape[0] > 0 else np.nan
                    row[f"fleet_y_{vid}"] = fleet[1, vid] if fleet.shape[0] > 1 else np.nan
                    row[f"fleet_theta_{vid}"] = fleet[2, vid] if fleet.shape[0] > 2 else np.nan
                    row[f"fleet_v_{vid}"] = fleet[3, vid] if fleet.shape[0] > 3 else np.nan
                    row[f"fleet_a_{vid}"] = fleet[4, vid] if fleet.shape[0] > 4 else np.nan

        for key in [
            "dt",
            "position",
            "velocity",
            "acceleration",
            "control_input",
            "local_measurement_p",
            "local_measurement_v",
        ]:
            row[key] = debug_data.get(key, np.nan)

        for vid in range(self.fleet_size):
            row[f"true_position_{vid}"] = debug_data.get(f"true_position_{vid}", np.nan)
            row[f"true_velocity_{vid}"] = debug_data.get(f"true_velocity_{vid}", np.nan)
            row[f"true_acceleration_{vid}"] = debug_data.get(f"true_acceleration_{vid}", np.nan)
            row[f"true_throttle_{vid}"] = debug_data.get(f"true_throttle_{vid}", np.nan)

        return row

    def _write_vector_blocks(self, row: Dict, prefix: str, data) -> None:
        if data is None:
            return

        flat = np.asarray(data).reshape(-1)
        for vid in range(self.observer_size):
            base = vid * 3
            row[f"{prefix}_x{vid}"] = flat[base] if base < len(flat) else np.nan
            row[f"{prefix}_v{vid}"] = flat[base + 1] if base + 1 < len(flat) else np.nan
            row[f"{prefix}_a{vid}"] = flat[base + 2] if base + 2 < len(flat) else np.nan

    @staticmethod
    def _first(data):
        if data is None:
            return np.nan
        flat = np.asarray(data).reshape(-1)
        return flat[0] if len(flat) else np.nan

    def _writer_loop(self):
        while True:
            try:
                try:
                    item = self._write_queue.get(timeout=0.5)
                except queue.Empty:
                    if time.time() - self._last_flush_time > self.flush_interval:
                        self._flush()
                    continue

                if item is self._STOP_SENTINEL:
                    break

                if self.writer is not None:
                    self.writer.writerow(item)
                    self._record_count += 1

                if time.time() - self._last_flush_time > self.flush_interval:
                    self._flush()
            except Exception:
                pass

        self._flush()

    def _flush(self):
        if self.file is not None:
            try:
                self.file.flush()
                self._last_flush_time = time.time()
            except Exception:
                pass

    def stop(self) -> Dict:
        self.recording = False

        if self._write_queue is not None:
            try:
                self._write_queue.put(self._STOP_SENTINEL, timeout=1.0)
            except Exception:
                pass

        if self._writer_thread is not None and self._writer_thread.is_alive():
            self._writer_thread.join(timeout=2.0)

        if self.file is not None:
            try:
                self.file.flush()
                self.file.close()
            except Exception:
                pass
            self.file = None
            self.writer = None

        self._write_queue = None
        self._writer_thread = None
        return {
            "record_count": self._record_count,
            "dropped_count": self._dropped_count,
            "filepath": self.filepath,
        }

    def is_recording(self) -> bool:
        return self.recording

    def get_filepath(self):
        return self.filepath

    def get_stats(self) -> Dict:
        return {
            "record_count": self._record_count,
            "dropped_count": self._dropped_count,
            "queue_size": self._write_queue.qsize() if self._write_queue else 0,
            "filepath": self.filepath,
        }
