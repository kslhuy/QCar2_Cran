"""
Vehicle-side ZMQ client for online SysID.

This module is intentionally lightweight:
- publishes samples [v_x, v_y, omega, delta] to an external worker,
- publishes control commands (start/stop/train/status/config/clear),
- subscribes to worker status/ack messages.

It does NOT run training locally. Training is handled by a separate worker
process (see online_sysid_zmq_worker.py).
"""

from __future__ import annotations

import time
from typing import Any, Dict, Optional

import numpy as np

try:
    import zmq

    ZMQ_AVAILABLE = True
except Exception:
    ZMQ_AVAILABLE = False


SAMPLE_SNDHWM = 4096
CONTROL_SNDHWM = 128
STATUS_RCVHWM = 64


class OnlineSysIDZMQClient:
    """Vehicle-side ZMQ transport for external online SysID worker."""

    def __init__(
        self,
        logger,
        vehicle_id: int,
        sample_port: int = 18880,
        control_port: int = 18881,
        status_host: str = "127.0.0.1",
        status_port: int = 18882,
        bind_ip: str = "*",
    ):
        self.logger = logger
        self.vehicle_id = int(vehicle_id)

        self.sample_port = int(sample_port)
        self.control_port = int(control_port)
        self.status_host = str(status_host)
        self.status_port = int(status_port)
        self.bind_ip = str(bind_ip)

        self._ctx = None
        self._sample_pub = None
        self._control_pub = None
        self._status_sub = None

        self._running = False
        self._collecting = False
        self._samples_sent = 0
        self._samples_dropped = 0
        self._commands_sent = 0
        self._last_error = ""
        self._last_remote_status: Dict[str, Any] = {}
        self._last_remote_status_time = 0.0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start(self) -> bool:
        if self._running:
            return True
        if not ZMQ_AVAILABLE:
            self._last_error = "pyzmq is not available"
            self._log_warn("[OnlineSysID-ZMQ] pyzmq not available")
            return False

        try:
            self._ctx = zmq.Context.instance()

            self._sample_pub = self._ctx.socket(zmq.PUB)
            self._sample_pub.setsockopt(zmq.SNDHWM, SAMPLE_SNDHWM)
            self._sample_pub.setsockopt(zmq.LINGER, 0)
            self._sample_pub.bind(f"tcp://{self.bind_ip}:{self.sample_port}")

            self._control_pub = self._ctx.socket(zmq.PUB)
            self._control_pub.setsockopt(zmq.SNDHWM, CONTROL_SNDHWM)
            self._control_pub.setsockopt(zmq.LINGER, 0)
            self._control_pub.bind(f"tcp://{self.bind_ip}:{self.control_port}")

            self._status_sub = self._ctx.socket(zmq.SUB)
            self._status_sub.setsockopt(zmq.SUBSCRIBE, b"")
            self._status_sub.setsockopt(zmq.CONFLATE, 1)
            self._status_sub.setsockopt(zmq.RCVHWM, STATUS_RCVHWM)
            self._status_sub.setsockopt(zmq.RCVTIMEO, 0)
            self._status_sub.setsockopt(zmq.LINGER, 0)
            self._status_sub.connect(f"tcp://{self.status_host}:{self.status_port}")

            # Give remote subscribers a short time to connect (slow joiner).
            time.sleep(0.2)

            self._running = True
            self._last_error = ""
            self._log_info(
                f"[OnlineSysID-ZMQ] Started: sample=tcp://{self.bind_ip}:{self.sample_port}, "
                f"control=tcp://{self.bind_ip}:{self.control_port}, "
                f"status=tcp://{self.status_host}:{self.status_port}"
            )
            return True
        except Exception as exc:
            self._last_error = str(exc)
            self._log_error("[OnlineSysID-ZMQ] Failed to start", exc)
            self.stop()
            return False

    def stop(self) -> None:
        self._collecting = False
        self._running = False
        try:
            if self._sample_pub is not None:
                self._sample_pub.close(linger=0)
        except Exception:
            pass
        try:
            if self._control_pub is not None:
                self._control_pub.close(linger=0)
        except Exception:
            pass
        try:
            if self._status_sub is not None:
                self._status_sub.close(linger=0)
        except Exception:
            pass
        self._sample_pub = None
        self._control_pub = None
        self._status_sub = None

    # ------------------------------------------------------------------
    # High-level controls
    # ------------------------------------------------------------------
    def start_collection(self) -> bool:
        if not self.start():
            return False
        self._collecting = True
        return self.send_command("start")

    def stop_collection(self) -> bool:
        self._collecting = False
        if not self._running:
            return False
        return self.send_command("stop")

    def clear_buffer(self) -> bool:
        return self.send_command("clear")

    def trigger_train(self, train_options: Optional[Dict[str, Any]] = None) -> bool:
        return self.send_command("train", train_options=train_options or {})

    def update_config(self, config: Dict[str, Any]) -> bool:
        return self.send_command("set_config", config=config or {})

    def request_status(self) -> bool:
        return self.send_command("status")

    # ------------------------------------------------------------------
    # Data path
    # ------------------------------------------------------------------
    def submit_sample(self, sample: np.ndarray, timestamp: Optional[float] = None) -> bool:
        if not self._running or not self._collecting or self._sample_pub is None:
            return False

        arr = np.asarray(sample, dtype=np.float32).reshape(-1)
        if arr.size != 4 or not np.all(np.isfinite(arr)):
            return False

        payload = {
            "type": "sample",
            "vehicle_id": self.vehicle_id,
            "timestamp": float(timestamp) if timestamp is not None else time.time(),
            "sample": arr.tolist(),
        }
        try:
            self._sample_pub.send_json(payload, flags=zmq.NOBLOCK)
            self._samples_sent += 1
            return True
        except Exception:
            self._samples_dropped += 1
            return False

    def send_command(self, action: str, **kwargs) -> bool:
        if not self.start() or self._control_pub is None:
            return False
        payload = {
            "type": "cmd",
            "action": str(action),
            "vehicle_id": self.vehicle_id,
            "timestamp": time.time(),
        }
        payload.update(kwargs)
        try:
            self._control_pub.send_json(payload, flags=zmq.NOBLOCK)
            self._commands_sent += 1
            return True
        except Exception as exc:
            self._last_error = str(exc)
            return False

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------
    def poll_status(self, max_messages: int = 10) -> None:
        if self._status_sub is None:
            return
        for _ in range(max_messages):
            try:
                msg = self._status_sub.recv_json(flags=zmq.NOBLOCK)
            except Exception:
                return
            self._last_remote_status = msg
            self._last_remote_status_time = time.time()

    def get_status(self) -> Dict[str, Any]:
        self.poll_status(max_messages=10)
        return {
            "running": self._running,
            "collecting": self._collecting,
            "sample_port": self.sample_port,
            "control_port": self.control_port,
            "status_host": self.status_host,
            "status_port": self.status_port,
            "samples_sent": self._samples_sent,
            "samples_dropped": self._samples_dropped,
            "commands_sent": self._commands_sent,
            "last_error": self._last_error,
            "last_remote_status_time": self._last_remote_status_time,
            "last_remote_status": dict(self._last_remote_status)
            if isinstance(self._last_remote_status, dict)
            else {},
        }

    def is_collecting(self) -> bool:
        return self._running and self._collecting

    # ------------------------------------------------------------------
    # Logging helpers
    # ------------------------------------------------------------------
    def _log_info(self, msg: str) -> None:
        if self.logger is None:
            return
        if hasattr(self.logger, "logger"):
            self.logger.logger.info(msg)
        else:
            print(msg)

    def _log_warn(self, msg: str) -> None:
        if self.logger is None:
            return
        if hasattr(self.logger, "log_warning"):
            self.logger.log_warning(msg)
        elif hasattr(self.logger, "logger"):
            self.logger.logger.warning(msg)
        else:
            print(msg)

    def _log_error(self, msg: str, exc: Optional[Exception] = None) -> None:
        if self.logger is None:
            return
        if hasattr(self.logger, "log_error"):
            self.logger.log_error(msg, exc)
        elif hasattr(self.logger, "logger"):
            if exc is None:
                self.logger.logger.error(msg)
            else:
                self.logger.logger.error(f"{msg}: {exc}")
        else:
            print(msg if exc is None else f"{msg}: {exc}")
