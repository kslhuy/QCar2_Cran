"""
ROS-free online SysID service.

This module provides a queue + worker-thread pipeline that can run continuously
inside vehicle_logic:
1) submit samples [v_x, v_y, omega, delta] from the control loop,
2) keep a filtered rolling buffer,
3) trigger background training with existing On_Track_SysID helpers.

Optional ZeroMQ PUB can broadcast status/training events for external monitors.
"""

from __future__ import annotations

import os
import sys
import time
import queue
import threading
from collections import deque
from typing import Any, Dict, Optional, Tuple

import numpy as np

try:
    import zmq

    ZMQ_AVAILABLE = True
except Exception:
    ZMQ_AVAILABLE = False


SRC_DIR = os.path.dirname(os.path.abspath(__file__))
if SRC_DIR not in sys.path:
    sys.path.append(SRC_DIR)


class OnlineSysIDService:
    """
    Threaded online learning service for On_Track_SysID.

    Public API:
    - start(collect=False)
    - shutdown()
    - start_collection()
    - stop_collection()
    - clear_buffer()
    - submit_sample(sample)
    - trigger_train(train_options=None)
    - update_runtime_config(config_updates)
    - get_status()
    """

    def __init__(
        self,
        logger,
        vehicle_id: int,
        racecar_version: str = "SIM",
        sample_dt: float = 0.02,
        min_speed_threshold: float = 1.0,
        min_samples: int = 1500,
        sample_queue_size: int = 2048,
        buffer_size: int = 20000,
        save_lut_name: str = "online_sysid",
        generate_lut: bool = False,
        plot_model: bool = False,
        package_path: Optional[str] = None,
        zmq_pub_enabled: bool = False,
        zmq_pub_port: int = 18777,
    ):
        self.logger = logger
        self.vehicle_id = int(vehicle_id)

        # Runtime config (can be updated via update_runtime_config)
        self.racecar_version = str(racecar_version)
        self.sample_dt = float(sample_dt)
        self.min_speed_threshold = float(min_speed_threshold)
        self.min_samples = int(min_samples)
        self.sample_queue_size = int(sample_queue_size)
        self.buffer_size = int(buffer_size)
        self.save_lut_name = str(save_lut_name)
        self.generate_lut = bool(generate_lut)
        self.plot_model = bool(plot_model)
        self.package_path = (
            os.path.abspath(package_path)
            if package_path
            else os.path.abspath(os.path.join(SRC_DIR, ".."))
        )

        # Worker communication
        self._sample_queue: queue.Queue = queue.Queue(maxsize=self.sample_queue_size)
        self._command_queue: queue.Queue = queue.Queue(maxsize=64)
        self._sample_buffer: deque = deque(maxlen=self.buffer_size)

        # State
        self._running = False
        self._collecting = False
        self._training = False
        self._worker: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._buffer_lock = threading.RLock()
        self._state_lock = threading.RLock()

        # Counters + status
        self._samples_received = 0
        self._samples_accepted = 0
        self._samples_dropped = 0
        self._last_train_time = 0.0
        self._last_train_error = ""
        self._last_train_result: Dict[str, Any] = {}

        # Optional ZMQ status publisher
        self._zmq_pub_enabled = bool(zmq_pub_enabled)
        self._zmq_pub_port = int(zmq_pub_port)
        self._zmq_ctx = None
        self._zmq_pub = None
        self._init_zmq_pub()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start(self, collect: bool = False) -> bool:
        with self._state_lock:
            if self._running:
                self._collecting = bool(collect) or self._collecting
                return True

            self._stop_event.clear()
            self._collecting = bool(collect)
            self._worker = threading.Thread(
                target=self._worker_loop,
                name=f"online_sysid_worker_v{self.vehicle_id}",
                daemon=True,
            )
            self._worker.start()
            self._running = True

        self._log_info(
            f"[OnlineSysID] Worker started (collecting={self._collecting}, racecar_version={self.racecar_version})"
        )
        self._publish_event("worker_started", self.get_status())
        return True

    def shutdown(self) -> None:
        with self._state_lock:
            if not self._running:
                self._close_zmq_pub()
                return
            self._running = False
            self._collecting = False
            self._stop_event.set()
            try:
                self._command_queue.put_nowait({"type": "shutdown"})
            except queue.Full:
                pass
            worker = self._worker

        if worker and worker.is_alive():
            worker.join(timeout=5.0)

        self._close_zmq_pub()
        self._log_info("[OnlineSysID] Worker stopped")

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------
    def start_collection(self) -> bool:
        with self._state_lock:
            if not self._running:
                self.start(collect=True)
            self._collecting = True
        self._publish_event("collection_started", self.get_status())
        return True

    def stop_collection(self) -> bool:
        with self._state_lock:
            self._collecting = False
        self._publish_event("collection_stopped", self.get_status())
        return True

    def clear_buffer(self) -> None:
        with self._buffer_lock:
            self._sample_buffer.clear()
        self._publish_event("buffer_cleared", self.get_status())

    def trigger_train(
        self, train_options: Optional[Dict[str, Any]] = None
    ) -> Tuple[bool, str]:
        with self._state_lock:
            if not self._running:
                return False, "OnlineSysID worker is not running"
            if self._training:
                return False, "Training already in progress"

        cmd = {"type": "train", "options": train_options or {}}
        try:
            self._command_queue.put_nowait(cmd)
            return True, "Training request queued"
        except queue.Full:
            return False, "Training queue is full"

    def update_runtime_config(self, config_updates: Dict[str, Any]) -> Dict[str, Any]:
        """
        Update runtime config fields that are safe to change online.
        Returns applied keys + current status.
        """
        if not isinstance(config_updates, dict):
            return {"applied": [], "status": self.get_status()}

        applied = []
        with self._state_lock:
            if "racecar_version" in config_updates:
                self.racecar_version = str(config_updates["racecar_version"])
                applied.append("racecar_version")
            if "sample_dt" in config_updates:
                self.sample_dt = max(1e-4, float(config_updates["sample_dt"]))
                applied.append("sample_dt")
            if "min_speed_threshold" in config_updates:
                self.min_speed_threshold = float(config_updates["min_speed_threshold"])
                applied.append("min_speed_threshold")
            if "min_samples" in config_updates:
                self.min_samples = max(20, int(config_updates["min_samples"]))
                applied.append("min_samples")
            if "save_lut_name" in config_updates:
                self.save_lut_name = str(config_updates["save_lut_name"])
                applied.append("save_lut_name")
            if "generate_lut" in config_updates:
                self.generate_lut = bool(config_updates["generate_lut"])
                applied.append("generate_lut")
            if "plot_model" in config_updates:
                self.plot_model = bool(config_updates["plot_model"])
                applied.append("plot_model")

        status = self.get_status()
        if applied:
            self._publish_event("config_updated", {"applied": applied, "status": status})
        return {"applied": applied, "status": status}

    # ------------------------------------------------------------------
    # Data path
    # ------------------------------------------------------------------
    def submit_sample(self, sample: np.ndarray, timestamp: Optional[float] = None) -> bool:
        """
        Submit one sample [v_x, v_y, omega, delta].

        The real work happens in the worker thread. This method is non-blocking
        and drops old samples when queue is full.
        """
        arr = np.asarray(sample, dtype=np.float32).reshape(-1)
        if arr.size != 4 or not np.all(np.isfinite(arr)):
            return False

        with self._state_lock:
            if not self._running or not self._collecting:
                return False
            self._samples_received += 1

        item = (float(timestamp) if timestamp is not None else time.time(), arr.copy())
        try:
            self._sample_queue.put_nowait(item)
            return True
        except queue.Full:
            # Drop oldest queued sample to prioritize newest (online adaptation)
            try:
                self._sample_queue.get_nowait()
            except queue.Empty:
                pass

            try:
                self._sample_queue.put_nowait(item)
                with self._state_lock:
                    self._samples_dropped += 1
                return True
            except queue.Full:
                with self._state_lock:
                    self._samples_dropped += 1
                return False

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------
    def get_status(self) -> Dict[str, Any]:
        with self._state_lock:
            running = self._running
            collecting = self._collecting
            training = self._training
            samples_received = self._samples_received
            samples_accepted = self._samples_accepted
            samples_dropped = self._samples_dropped
            last_train_time = self._last_train_time
            last_train_error = self._last_train_error
            last_train_result = dict(self._last_train_result)
            racecar_version = self.racecar_version
            sample_dt = self.sample_dt
            min_speed_threshold = self.min_speed_threshold
            min_samples = self.min_samples
            save_lut_name = self.save_lut_name
            generate_lut = self.generate_lut
            plot_model = self.plot_model

        with self._buffer_lock:
            buffered_samples = len(self._sample_buffer)

        return {
            "worker_running": running,
            "collecting": collecting,
            "training": training,
            "racecar_version": racecar_version,
            "sample_dt": sample_dt,
            "min_speed_threshold": min_speed_threshold,
            "min_samples": min_samples,
            "buffered_samples": buffered_samples,
            "buffer_capacity": self.buffer_size,
            "samples_received": samples_received,
            "samples_accepted": samples_accepted,
            "samples_dropped": samples_dropped,
            "save_lut_name": save_lut_name,
            "generate_lut": generate_lut,
            "plot_model": plot_model,
            "last_train_time": last_train_time,
            "last_train_error": last_train_error,
            "last_train_result": last_train_result,
            "zmq_pub_enabled": self._zmq_pub is not None,
            "zmq_pub_port": self._zmq_pub_port if self._zmq_pub is not None else None,
        }

    # ------------------------------------------------------------------
    # Internal worker
    # ------------------------------------------------------------------
    def _worker_loop(self) -> None:
        while not self._stop_event.is_set():
            self._process_one_command(timeout=0.02)
            self._drain_sample_queue(max_batch=256)

    def _process_one_command(self, timeout: float = 0.0) -> None:
        try:
            cmd = self._command_queue.get(timeout=timeout)
        except queue.Empty:
            return

        cmd_type = cmd.get("type")
        if cmd_type == "shutdown":
            return
        if cmd_type == "train":
            options = cmd.get("options", {})
            self._run_training_job(options)

    def _drain_sample_queue(self, max_batch: int = 256) -> None:
        for _ in range(max_batch):
            try:
                _ts, sample = self._sample_queue.get_nowait()
            except queue.Empty:
                return

            with self._state_lock:
                speed_threshold = self.min_speed_threshold

            vx = float(sample[0])
            if abs(vx) <= speed_threshold:
                continue

            with self._buffer_lock:
                self._sample_buffer.append(sample.copy())

            with self._state_lock:
                self._samples_accepted += 1

    def _run_training_job(self, options: Dict[str, Any]) -> None:
        with self._state_lock:
            if self._training:
                return
            self._training = True
            self._last_train_error = ""

            racecar_version = str(options.get("racecar_version", self.racecar_version))
            sample_dt = float(options.get("sample_dt", self.sample_dt))
            min_samples = int(options.get("min_samples", self.min_samples))
            save_lut_name = str(options.get("save_lut_name", self.save_lut_name))
            generate_lut = bool(options.get("generate_lut", self.generate_lut))
            plot_model = bool(options.get("plot_model", self.plot_model))

        with self._buffer_lock:
            if len(self._sample_buffer) == 0:
                training_data = np.zeros((0, 4), dtype=np.float32)
            else:
                training_data = np.asarray(self._sample_buffer, dtype=np.float32)

        if training_data.shape[0] < min_samples:
            msg = (
                f"Not enough samples for training ({training_data.shape[0]} < {min_samples})"
            )
            with self._state_lock:
                self._last_train_error = msg
                self._training = False
            self._log_warn(f"[OnlineSysID] {msg}")
            self._publish_event("train_skipped", {"reason": msg, "status": self.get_status()})
            return

        started_at = time.time()
        self._publish_event(
            "train_started",
            {
                "racecar_version": racecar_version,
                "sample_count": int(training_data.shape[0]),
                "sample_dt": sample_dt,
            },
        )

        try:
            from helpers.train_model import nn_train

            c_pf_identified, c_pr_identified = nn_train(
                training_data=training_data,
                racecar_version=racecar_version,
                save_LUT_name=save_lut_name,
                plot_model=plot_model,
                dt=sample_dt,
                generate_lut=generate_lut,
                package_path=self.package_path,
                always_plot_last_iteration=False,
            )

            elapsed = time.time() - started_at
            result = {
                "racecar_version": racecar_version,
                "sample_count": int(training_data.shape[0]),
                "elapsed_s": float(elapsed),
                "c_pf_identified": np.asarray(c_pf_identified, dtype=float).tolist(),
                "c_pr_identified": np.asarray(c_pr_identified, dtype=float).tolist(),
                "save_lut_name": save_lut_name,
                "generate_lut": generate_lut,
                "plot_model": plot_model,
            }

            with self._state_lock:
                self._last_train_time = time.time()
                self._last_train_result = result
                self._last_train_error = ""
                self._training = False

            self._log_info(
                f"[OnlineSysID] Training completed in {elapsed:.2f}s with {training_data.shape[0]} samples"
            )
            self._publish_event("train_completed", result)
        except Exception as exc:
            with self._state_lock:
                self._last_train_error = str(exc)
                self._training = False
            self._log_error("[OnlineSysID] Training failed", exc)
            self._publish_event("train_failed", {"error": str(exc), "status": self.get_status()})

    # ------------------------------------------------------------------
    # ZMQ publisher (optional)
    # ------------------------------------------------------------------
    def _init_zmq_pub(self) -> None:
        if not self._zmq_pub_enabled:
            return
        if not ZMQ_AVAILABLE:
            self._log_warn("[OnlineSysID] pyzmq not available, ZMQ status publisher disabled")
            return

        try:
            self._zmq_ctx = zmq.Context.instance()
            self._zmq_pub = self._zmq_ctx.socket(zmq.PUB)
            self._zmq_pub.setsockopt(zmq.SNDHWM, 64)
            self._zmq_pub.setsockopt(zmq.LINGER, 0)
            self._zmq_pub.bind(f"tcp://*:{self._zmq_pub_port}")
            time.sleep(0.2)  # allow subscribers to connect
            self._log_info(
                f"[OnlineSysID] ZMQ PUB ready on tcp://*:{self._zmq_pub_port}"
            )
        except Exception as exc:
            self._zmq_pub = None
            self._log_error("[OnlineSysID] Failed to initialize ZMQ PUB", exc)

    def _close_zmq_pub(self) -> None:
        if self._zmq_pub is not None:
            try:
                self._zmq_pub.close(linger=0)
            except Exception:
                pass
            self._zmq_pub = None

    def _publish_event(self, event: str, data: Dict[str, Any]) -> None:
        if self._zmq_pub is None:
            return
        payload = {
            "type": "online_sysid",
            "event": event,
            "vehicle_id": self.vehicle_id,
            "timestamp": time.time(),
            "data": data,
        }
        try:
            self._zmq_pub.send_json(payload, flags=zmq.NOBLOCK)
        except Exception:
            # Keep non-blocking behavior and avoid polluting logs on subscriber gaps.
            pass

    # ------------------------------------------------------------------
    # Logging helpers
    # ------------------------------------------------------------------
    def _log_info(self, msg: str) -> None:
        if self.logger is None:
            print(msg)
            return
        if hasattr(self.logger, "logger"):
            self.logger.logger.info(msg)
        else:
            print(msg)

    def _log_warn(self, msg: str) -> None:
        if self.logger is None:
            print(msg)
            return
        if hasattr(self.logger, "log_warning"):
            self.logger.log_warning(msg)
        elif hasattr(self.logger, "logger"):
            self.logger.logger.warning(msg)
        else:
            print(msg)

    def _log_error(self, msg: str, exc: Optional[Exception] = None) -> None:
        if self.logger is None:
            if exc is not None:
                print(f"{msg}: {exc}")
            else:
                print(msg)
            return
        if hasattr(self.logger, "log_error"):
            self.logger.log_error(msg, exc)
        elif hasattr(self.logger, "logger"):
            if exc is None:
                self.logger.logger.error(msg)
            else:
                self.logger.logger.error(f"{msg}: {exc}")
        else:
            if exc is not None:
                print(f"{msg}: {exc}")
            else:
                print(msg)
