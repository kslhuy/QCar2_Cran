"""
Trust and Weight Logger

Non-blocking I/O logger specifically for logging trust components,
final trust scores, and consensus weights for the TrustBasedFleetEstimator.
Plots can be visualized later. This file lives in TrustbasedDistributedObserver.
"""

import os
import csv
import threading
import queue
from typing import Dict, Any


class TrustWeightLogger:
    def __init__(self, output_dir: str = None, max_vehicles: int = 10):
        if output_dir is None:
            output_dir = os.path.dirname(os.path.abspath(__file__))
        self.output_dir = output_dir
        self.max_vehicles = max_vehicles
        self.recording = False

        self.queue = queue.Queue(maxsize=10000)
        self.thread = None
        self.file = None
        self.writer = None

        # Define all expected columns
        self.columns = ["time", "w0", "w_self"]
        for i in range(max_vehicles):
            self.columns.extend(
                [
                    f"trust_{i}",
                    f"v_score_{i}",
                    f"d_score_{i}",
                    f"a_score_{i}",
                    f"h_score_{i}",
                    f"b_score_{i}",
                    f"q_factor_{i}",
                    f"w_neighbor_{i}",
                    f"flag_attack_{i}",
                    f"flag_local_{i}",
                    f"flag_global_{i}",
                ]
            )

    def start(self, vehicle_id: int):
        if self.recording:
            return

        os.makedirs(self.output_dir, exist_ok=True)
        # Always overwrite the file for this vehicle
        filepath = os.path.join(self.output_dir, f"trust_weight_log_V{vehicle_id}.csv")

        try:
            self.file = open(filepath, "w", newline="", buffering=8192)
            self.writer = csv.DictWriter(self.file, fieldnames=self.columns)
            self.writer.writeheader()

            self.recording = True
            self.thread = threading.Thread(target=self._write_loop, daemon=True)
            self.thread.start()
        except Exception as e:
            print(f"[TrustLogger] Failed to open log file: {e}")
            self.recording = False

    def record(self, t: float, data: Dict[str, Any]):
        """
        Record a data sample without blocking.
        """
        if not self.recording:
            return

        # Flatten data for CSV
        row = {"time": t, "w0": data.get("w0", 0.0), "w_self": data.get("w_self", 0.0)}

        neighbors = data.get("neighbors", {})
        for i in range(self.max_vehicles):
            if i in neighbors:
                ndata = neighbors[i]
                row[f"trust_{i}"] = ndata.get("trust_score", 0.5)
                row[f"v_score_{i}"] = ndata.get("velocity_score", 1.0)
                row[f"d_score_{i}"] = ndata.get("distance_score", 1.0)
                row[f"a_score_{i}"] = ndata.get("acceleration_score", 1.0)
                row[f"h_score_{i}"] = ndata.get("heading_score", 1.0)
                row[f"b_score_{i}"] = ndata.get("beacon_score", 1.0)
                row[f"q_factor_{i}"] = ndata.get("quality_factor", 1.0)
                row[f"w_neighbor_{i}"] = ndata.get("w_neighbor", 0.0)
                row[f"flag_attack_{i}"] = (
                    1 if ndata.get("flag_target_attack", False) else 0
                )
                row[f"flag_local_{i}"] = (
                    1 if ndata.get("flag_local_est_check", False) else 0
                )
                row[f"flag_global_{i}"] = (
                    1 if ndata.get("flag_global_est_check", False) else 0
                )
            else:
                # Default/empty values: use neutral/safe defaults for plots
                row[f"trust_{i}"] = 0.5
                row[f"v_score_{i}"] = 1.0
                row[f"d_score_{i}"] = 1.0
                row[f"a_score_{i}"] = 1.0
                row[f"h_score_{i}"] = 1.0
                row[f"b_score_{i}"] = 0.0
                row[f"q_factor_{i}"] = 0.0
                row[f"w_neighbor_{i}"] = 0.0
                row[f"flag_attack_{i}"] = 0
                row[f"flag_local_{i}"] = 0
                row[f"flag_global_{i}"] = 0

        try:
            self.queue.put_nowait(row)
        except queue.Full:
            pass  # Drop if queue is full

    def _write_loop(self):
        while self.recording or not self.queue.empty():
            try:
                row = self.queue.get(timeout=0.1)
                self.writer.writerow(row)
                self.queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                pass

    def stop(self):
        self.recording = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.file:
            try:
                self.file.flush()
                self.file.close()
            except Exception:
                pass
