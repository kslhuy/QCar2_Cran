"""Reconnecting vehicle-side TCP bridge for the ground station."""

from __future__ import annotations

import logging
from queue import Empty, Full, Queue
import select
import socket
import threading
import time
from typing import Any, Mapping
from uuid import uuid4

from core.commands import CommandError, CommandOutcome, CommandResult, CommandSource, CommandType, VehicleCommand
from .bridge_base import GroundStationBridgeBase
from .monitoring import MonitoringSnapshot
from .protocol import FrameDecoder, FrameType, ProtocolError, encode_frame, error_payload


class GroundStationClientBridge(GroundStationBridgeBase):
    """Queue-based TCP client which never invokes vehicle runtime code itself."""

    def __init__(self, config: dict, vehicle_id: int = 0, logger=None) -> None:
        self._config = dict(config)
        self._vehicle_id = int(vehicle_id)
        self._logger = logger or logging.getLogger(self.__class__.__name__)
        self._host = _required_string(self._config, "server_host")
        self._port = _port(self._config.get("server_port"))
        self._connect_timeout_s = _positive_float(self._config.get("connect_timeout_s", 1.0), "connect_timeout_s")
        self._reconnect_interval_s = _positive_float(
            self._config.get("reconnect_interval_s", 1.0), "reconnect_interval_s"
        )
        self._max_frame_bytes = _positive_int(self._config.get("max_frame_bytes", 65536), "max_frame_bytes")
        self._monitoring_rate_hz = _positive_float(
            self._config.get("monitoring_rate_hz", 10.0), "monitoring_rate_hz"
        )
        command_queue_size = _positive_int(self._config.get("command_queue_size", 32), "command_queue_size")
        outbound_queue_size = _positive_int(self._config.get("outbound_queue_size", 64), "outbound_queue_size")
        self._commands: Queue[VehicleCommand] = Queue(maxsize=command_queue_size)
        self._latest_manual_input: VehicleCommand | None = None
        self._manual_input_lock = threading.Lock()
        self._outbound_control: Queue[bytes] = Queue(maxsize=outbound_queue_size)
        self._latest_snapshot: MonitoringSnapshot | None = None
        self._snapshot_lock = threading.Lock()
        self._last_snapshot_offer_at = 0.0
        self._running = threading.Event()
        self._connected = threading.Event()
        self._registered = threading.Event()
        self._thread: threading.Thread | None = None
        self._socket: socket.socket | None = None
        self._socket_lock = threading.Lock()
        self._session_id = uuid4().hex
        self._last_error = ""
        self._stats = {
            "commands_received": 0,
            "commands_rejected": 0,
            "manual_inputs_coalesced": 0,
            "acks_sent": 0,
            "snapshots_sent": 0,
            "reconnects": 0,
        }

    def start(self) -> None:
        if self._running.is_set():
            return
        self._running.set()
        self._thread = threading.Thread(target=self._run, name=f"GroundStationBridge-{self._vehicle_id}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if not self._running.is_set():
            return
        self._running.clear()
        with self._socket_lock:
            if self._socket is not None:
                try:
                    self._socket.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    self._socket.close()
                except OSError:
                    pass
                self._socket = None
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, self._connect_timeout_s + 0.5))
        self._thread = None
        self._connected.clear()
        self._registered.clear()

    def drain_commands(self, limit: int) -> list[VehicleCommand]:
        if not isinstance(limit, int) or isinstance(limit, bool) or limit <= 0:
            raise ValueError("command drain limit must be a positive integer")
        commands = []
        for _ in range(limit):
            try:
                commands.append(self._commands.get_nowait())
            except Empty:
                break
        if len(commands) < limit:
            with self._manual_input_lock:
                manual_input, self._latest_manual_input = self._latest_manual_input, None
            if manual_input is not None:
                commands.append(manual_input)
        return commands

    def publish_snapshot(self, snapshot: MonitoringSnapshot) -> None:
        if not isinstance(snapshot, MonitoringSnapshot):
            raise TypeError("ground-station snapshots must use MonitoringSnapshot")
        now = time.monotonic()
        if now - self._last_snapshot_offer_at < 1.0 / self._monitoring_rate_hz:
            return
        self._last_snapshot_offer_at = now
        with self._snapshot_lock:
            self._latest_snapshot = snapshot

    def publish_ack(self, result: CommandResult) -> None:
        if not isinstance(result, CommandResult):
            raise TypeError("ground-station acknowledgements must use CommandResult")
        self._offer_control(FrameType.COMMAND_ACK, result.to_mapping())

    def get_status(self) -> dict[str, object]:
        return {
            "enabled": True,
            "connected": self._connected.is_set(),
            "registered": self._registered.is_set(),
            "last_error": self._last_error,
            **dict(self._stats),
        }

    def _run(self) -> None:
        while self._running.is_set():
            try:
                self._run_connection()
            except OSError as exc:
                self._last_error = str(exc)
            except ProtocolError as exc:
                self._last_error = str(exc)
            finally:
                self._close_connection()
            if self._running.is_set():
                self._stats["reconnects"] += 1
                time.sleep(self._reconnect_interval_s)

    def _run_connection(self) -> None:
        connection = socket.create_connection((self._host, self._port), timeout=self._connect_timeout_s)
        connection.setblocking(False)
        with self._socket_lock:
            self._socket = connection
        self._connected.set()
        decoder = FrameDecoder(max_frame_bytes=self._max_frame_bytes)
        self._send_direct(
            connection,
            FrameType.REGISTER,
            {
                "vehicle_id": self._vehicle_id,
                "session_id": self._session_id,
                "capabilities": {"commands": True, "monitoring": True},
            },
        )
        registration_deadline = time.monotonic() + self._connect_timeout_s
        while self._running.is_set() and not self._registered.is_set():
            if time.monotonic() >= registration_deadline:
                raise ProtocolError("ground-station registration timed out")
            self._receive_available(connection, decoder)
            time.sleep(0.01)
        while self._running.is_set() and self._registered.is_set():
            self._send_queued_control(connection)
            self._send_latest_snapshot(connection)
            self._receive_available(connection, decoder)
            time.sleep(0.005)

    def _receive_available(self, connection: socket.socket, decoder: FrameDecoder) -> None:
        readable, _, _ = select.select([connection], [], [], 0.05)
        if not readable:
            return
        data = connection.recv(65536)
        if not data:
            raise OSError("ground-station server closed the connection")
        for frame in decoder.feed(data):
            self._handle_frame(frame.frame_type, frame.payload)

    def _handle_frame(self, frame_type: FrameType, payload: Mapping[str, Any]) -> None:
        if frame_type == FrameType.REGISTER_ACK:
            if payload.get("accepted") is True and payload.get("vehicle_id") == self._vehicle_id:
                self._registered.set()
                self._last_error = ""
                return
            raise ProtocolError(str(payload.get("reason", "ground-station registration rejected")))
        if not self._registered.is_set():
            raise ProtocolError("received non-registration frame before REGISTER_ACK")
        if frame_type == FrameType.COMMAND_REQUEST:
            self._queue_command(payload)
            return
        if frame_type == FrameType.ERROR:
            self._last_error = str(payload.get("message", "ground-station protocol error"))
            return
        raise ProtocolError(f"unexpected frame from ground station: {frame_type.value}")

    def _queue_command(self, payload: Mapping[str, Any]) -> None:
        try:
            command = VehicleCommand.from_mapping(payload)
            if command.source != CommandSource.GROUND_STATION:
                raise CommandError("ground-station commands must use ground_station source")
            if command.target_vehicle_id not in (None, self._vehicle_id):
                raise CommandError("command targets a different vehicle")
        except CommandError as exc:
            self._stats["commands_rejected"] += 1
            command_id = payload.get("command_id")
            if not isinstance(command_id, str) or not command_id.strip():
                command_id = "invalid_command"
            self._offer_control(
                FrameType.COMMAND_ACK,
                CommandResult(
                    command_id=command_id,
                    vehicle_id=self._vehicle_id,
                    outcome=CommandOutcome.REJECTED,
                    runtime_state="UNKNOWN",
                    reason_code="invalid_command",
                    reason=str(exc),
                ).to_mapping(),
            )
            return
        try:
            if command.command_type == CommandType.MANUAL_INPUT:
                with self._manual_input_lock:
                    if self._latest_manual_input is not None:
                        self._stats["manual_inputs_coalesced"] += 1
                    self._latest_manual_input = command
                self._stats["commands_received"] += 1
                return
            self._commands.put_nowait(command)
            self._stats["commands_received"] += 1
        except Full:
            self._stats["commands_rejected"] += 1
            self._offer_control(
                FrameType.COMMAND_ACK,
                CommandResult(
                    command_id=command.command_id,
                    vehicle_id=self._vehicle_id,
                    outcome=CommandOutcome.REJECTED,
                    runtime_state="UNKNOWN",
                    reason_code="command_queue_full",
                    reason="Vehicle command queue is full",
                ).to_mapping(),
            )

    def _offer_control(self, frame_type: FrameType, payload: Mapping[str, Any]) -> None:
        encoded = encode_frame(frame_type, payload, max_frame_bytes=self._max_frame_bytes)
        try:
            self._outbound_control.put_nowait(encoded)
        except Full:
            self._last_error = "ground-station control queue is full"

    def _send_queued_control(self, connection: socket.socket) -> None:
        while True:
            try:
                connection.sendall(self._outbound_control.get_nowait())
                self._stats["acks_sent"] += 1
            except Empty:
                return

    def _send_latest_snapshot(self, connection: socket.socket) -> None:
        with self._snapshot_lock:
            snapshot = self._latest_snapshot
            self._latest_snapshot = None
        if snapshot is None:
            return
        self._send_direct(connection, FrameType.MONITORING_SNAPSHOT, snapshot.to_mapping())
        self._stats["snapshots_sent"] += 1

    def _send_direct(self, connection: socket.socket, frame_type: FrameType, payload: Mapping[str, Any]) -> None:
        connection.sendall(encode_frame(frame_type, payload, max_frame_bytes=self._max_frame_bytes))

    def _close_connection(self) -> None:
        self._registered.clear()
        self._connected.clear()
        with self._socket_lock:
            connection, self._socket = self._socket, None
        if connection is not None:
            try:
                connection.close()
            except OSError:
                pass


def _required_string(config: Mapping[str, Any], key: str) -> str:
    value = config.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"ground_station.{key} must be a non-empty string")
    return value


def _port(value: Any) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or not 1 <= value <= 65535:
        raise ValueError("ground_station.server_port must be an integer in [1, 65535]")
    return value


def _positive_float(value: Any, name: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or float(value) <= 0.0:
        raise ValueError(f"ground_station.{name} must be positive")
    return float(value)


def _positive_int(value: Any, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"ground_station.{name} must be a positive integer")
    return value
