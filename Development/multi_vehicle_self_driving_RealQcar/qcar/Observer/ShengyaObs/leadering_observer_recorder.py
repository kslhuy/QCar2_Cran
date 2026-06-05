"""
Recorder for LeaderingObserverEstimator.

Writes observer estimates, the computed communication delay estimate, and the
latest available leader ground truth for offline plotting.
"""
import csv
import os
import queue
import threading
import time
from datetime import datetime
from typing import Dict, Optional


class LeaderingObserverRecorder:
    """Small non-blocking CSV recorder for leadering observer debug data."""

    _STOP_SENTINEL = object()

    def __init__(self, output_dir: str = "observer_recordings",
                 vehicle_id: int = 0,
                 observer_name: str = "leadering_observer",
                 queue_size: int = 1000,
                 flush_interval: float = 5.0):
        self.output_dir = output_dir
        self.vehicle_id = vehicle_id
        self.observer_name = observer_name
        self.queue_size = queue_size
        self.flush_interval = flush_interval

        self.file = None
        self.writer = None
        self.filepath: Optional[str] = None
        self.recording = False
        self._write_queue = None
        self._writer_thread = None
        self._last_flush_time = 0.0
        self._record_count = 0
        self._dropped_count = 0

        self.columns = [
            "time",
            "timestamp",
            "dt",
            "hat_tau",
            "raw_hat_tau",
            "hat_tau_attack_active",
            "hat_tau_attack_bias",
            "u_leader",
            "y_zeta",
            "z_filter",
            "v2v_measurement_delay",
            "v2v_measurement_age",
            "v2v_position_noise",
            "innovation",
            "integral_g",
            "g_value",
            "zeta_hat_0",
            "zeta_hat_x",
            "zeta_hat_v",
            "zeta_hat_a",
            "true_leader_x",
            "true_leader_v",
            "true_leader_a",
            "true_leader_u",
            "err_x",
            "err_v",
            "err_a",
        ]

    def start(self) -> str:
        os.makedirs(self.output_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.observer_name}_v{self.vehicle_id}_{timestamp}.csv"
        self.filepath = os.path.join(self.output_dir, filename)

        self.file = open(self.filepath, "w", newline="", buffering=8192)
        self.writer = csv.DictWriter(self.file, fieldnames=self.columns, extrasaction="ignore")
        self.writer.writeheader()
        self.file.flush()

        self._write_queue = queue.Queue(maxsize=self.queue_size)
        self._last_flush_time = time.time()
        self._record_count = 0
        self._dropped_count = 0
        self.recording = True

        self._writer_thread = threading.Thread(
            target=self._writer_loop,
            name=f"LeaderingObserverRecorder_V{self.vehicle_id}",
            daemon=True,
        )
        self._writer_thread.start()
        return self.filepath

    def record(self, row: Dict) -> bool:
        if not self.recording or self._write_queue is None:
            return False
        try:
            self._write_queue.put_nowait(row)
            return True
        except queue.Full:
            self._dropped_count += 1
            return False

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
        stats = {
            "record_count": self._record_count,
            "dropped_count": self._dropped_count,
            "filepath": self.filepath,
        }

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
        return stats

    def get_filepath(self) -> Optional[str]:
        return self.filepath

    def get_stats(self) -> Dict:
        return {
            "record_count": self._record_count,
            "dropped_count": self._dropped_count,
            "queue_size": self._write_queue.qsize() if self._write_queue else 0,
            "filepath": self.filepath,
        }
