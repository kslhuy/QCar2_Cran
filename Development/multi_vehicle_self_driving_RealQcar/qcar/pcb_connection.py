"""Passive PCB UART status, independent of the vehicle plant and digital twin.

Reads the existing COM H753 debug console. It never sends commands or treats
an open serial port (or simulated UART traffic) as proof of a connected PCB.
"""

import os
import re
import socket
import time
from threading import Event, Lock, Thread


_BOARD_LINE = re.compile(
    rb"^(?:COM H753 - |\[\d+\] IMU\s+status=0x[0-9A-Fa-f]{2}\b|"
    rb"\[UART \d+\] st=0x[0-9A-Fa-f]{2}\b|"
    rb"\[WiFi\] RDSR0=0x[0-9A-Fa-f]{2} RDSR1=0x[0-9A-Fa-f]{2} "
    rb"RDSR2=0x[0-9A-Fa-f]{2} HOST_IRQ=[01]$|"
    rb"\[WiFi\] SPI4 status-register read failed$)"
)


class PcbConnectionMonitor:
    """Read a configured host UART in a worker, outside the control loop."""

    def __init__(self, port="", baud_rate=115200, *, serial_factory=None,
                 clock=time.monotonic, timeout_s=3.0, retry_s=2.0):
        self.port = str(port).strip()
        self.baud_rate = int(baud_rate)
        self.timeout_s = max(0.1, float(timeout_s))
        self._retry_s = max(0.01, float(retry_s))
        self._serial_factory = serial_factory
        self._clock = clock
        self._lock = Lock()
        self._stop = Event()
        self._opened = False
        self._last_receive = None
        self._messages = 0
        self._error = ""
        self._thread = None
        if self.port:
            self._thread = Thread(target=self._run, name="pcb-uart", daemon=True)
            self._thread.start()

    @classmethod
    def from_environment(cls, vehicle_id):
        # Per-car settings prevent several simulated cars claiming one port.
        port = os.environ.get(f"QCAR_PCB_PORT_{vehicle_id}", "")
        baud = os.environ.get(f"QCAR_PCB_BAUD_{vehicle_id}", "115200")
        try:
            baud_rate = int(baud)
            if baud_rate <= 0:
                raise ValueError
        except ValueError:
            monitor = cls()
            monitor.port = port.strip()
            monitor._error = f"Invalid QCAR_PCB_BAUD_{vehicle_id}: {baud}"
            return monitor
        return cls(port, baud_rate)

    def _run(self):
        while not self._stop.is_set():
            connection = None
            try:
                factory = self._serial_factory
                if factory is None:
                    try:
                        from serial import Serial
                    except ImportError as exc:
                        raise RuntimeError("Install pyserial in the vehicle Python environment") from exc
                    factory = Serial
                connection = factory(port=self.port, baudrate=self.baud_rate, timeout=0.2)
                with self._lock:
                    self._opened = True
                    self._error = ""
                    self._last_receive = None
                pending = bytearray()
                while not self._stop.is_set():
                    chunk = connection.read(min(4096, max(1, connection.in_waiting)))
                    pending.extend(chunk)
                    while b"\n" in pending:
                        line, _, remainder = pending.partition(b"\n")
                        pending = bytearray(remainder)
                        if _BOARD_LINE.match(line.strip()):
                            with self._lock:
                                self._last_receive = self._clock()
                                self._messages += 1
                    # Bound memory even for noise or an incompatible protocol.
                    if len(pending) > 4096:
                        pending.clear()
            except Exception as exc:
                with self._lock:
                    self._error = str(exc)
            finally:
                with self._lock:
                    self._opened = False
                    self._last_receive = None
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass
            self._stop.wait(self._retry_s)

    def get_status(self):
        with self._lock:
            age = None if self._last_receive is None else max(0.0, self._clock() - self._last_receive)
            if self._error:
                state = "error"
            elif not self.port:
                state = "not_configured"
            elif not self._opened:
                state = "disconnected"
            elif age is None:
                state = "waiting"
            elif age > self.timeout_s:
                state = "stale"
            else:
                state = "connected"
            return {
                "state": state,
                "transport": "uart",
                "protocol": "cran_debug_uart",
                "host": socket.gethostname(),
                "port": self.port or None,
                "baud_rate": self.baud_rate,
                "last_receive_age_s": age,
                "messages_received": self._messages,
                "last_error": self._error or None,
            }

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
