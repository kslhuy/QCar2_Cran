"""Single-listener ground-station server and registered vehicle session registry."""

from __future__ import annotations

import socket
import threading
import time
from typing import Any, Mapping

from core.commands import CommandSource, VehicleCommand
from core.vehicle_types import LaserScanSample
from utils.ground_station.lidar_transport import lidar_scan_from_mapping
from utils.ground_station.monitoring import MonitoringSnapshot
from utils.ground_station.protocol import (
    DEFAULT_MAX_FRAME_BYTES,
    FrameDecoder,
    FrameType,
    ProtocolError,
    encode_frame,
    error_payload,
)

from ..utils.logging import get_ground_station_logger
from ..ground_station_type import CommandDelivery, DisconnectedVehicle, VehicleSession


class GroundStationServer:
    """TCP server that registers vehicles and routes typed commands by vehicle ID."""

    _MAX_RECENT_DISCONNECTIONS = 16

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 5000,
        *,
        max_frame_bytes: int = DEFAULT_MAX_FRAME_BYTES,
        logger=None,
    ) -> None:
        if not isinstance(host, str) or not host:
            raise ValueError("host must be a non-empty string")
        if not isinstance(port, int) or isinstance(port, bool) or not 0 <= port <= 65535:
            raise ValueError("port must be an integer in [0, 65535]")
        if not isinstance(max_frame_bytes, int) or isinstance(max_frame_bytes, bool) or max_frame_bytes < 64:
            raise ValueError("max_frame_bytes must be an integer of at least 64")
        self._host = host
        self._port = port
        self._max_frame_bytes = max_frame_bytes
        self._logger = logger or get_ground_station_logger("server")
        self._listener: socket.socket | None = None
        self._accept_thread: threading.Thread | None = None
        self._running = threading.Event()
        self._sessions: dict[int, VehicleSession] = {}
        self._recent_disconnections: dict[int, DisconnectedVehicle] = {}
        self._sessions_lock = threading.RLock()

    @property
    def port(self) -> int:
        """Return the configured or OS-assigned listener port."""
        if self._listener is None:
            return self._port
        return int(self._listener.getsockname()[1])

    def start(self) -> None:
        if self._running.is_set():
            return
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self._host, self._port))
        listener.listen()
        listener.settimeout(0.2)
        self._listener = listener
        self._running.set()
        self._accept_thread = threading.Thread(target=self._accept_loop, name="GroundStationAccept", daemon=True)
        self._accept_thread.start()

    def stop(self) -> None:
        if not self._running.is_set():
            return
        self._running.clear()
        listener, self._listener = self._listener, None
        if listener is not None:
            try:
                listener.close()
            except OSError:
                pass
        if self._accept_thread is not None:
            self._accept_thread.join(timeout=1.0)
        self._accept_thread = None
        with self._sessions_lock:
            sessions = tuple(self._sessions.values())
            self._sessions.clear()
            self._recent_disconnections.clear()
        for session in sessions:
            self._close_connection(session.connection)

    def session_rows(self) -> tuple[dict[str, object], ...]:
        """Return immutable-by-convention dashboard rows copied under the registry lock."""
        now = time.monotonic()
        with self._sessions_lock:
            rows = []
            for vehicle_id in sorted(self._sessions):
                session = self._sessions[vehicle_id]
                snapshot = session.latest_snapshot
                rows.append(
                    {
                        "vehicle_id": vehicle_id,
                        "session_id": session.session_id,
                        "address": session.address,
                        "connection_state": "connected",
                        "connected_at_monotonic": session.connected_at_monotonic,
                        "last_received_monotonic": session.last_received_monotonic,
                        "last_monitoring_monotonic": session.last_monitoring_monotonic,
                        "monitoring_rate_hz": _recent_rate(session.monitoring_received_monotonic, now),
                        "snapshot": snapshot.to_mapping() if snapshot is not None else None,
                        "last_command_result": dict(session.last_command_result or {}),
                        "last_error": session.last_error,
                    }
                )
            for vehicle_id in sorted(self._recent_disconnections):
                disconnected = self._recent_disconnections[vehicle_id]
                rows.append(
                    {
                        "vehicle_id": vehicle_id,
                        "session_id": "",
                        "address": disconnected.address,
                        "connection_state": "disconnected",
                        "connected_at_monotonic": 0.0,
                        "last_received_monotonic": disconnected.disconnected_at_monotonic,
                        "last_monitoring_monotonic": disconnected.disconnected_at_monotonic,
                        "monitoring_rate_hz": 0.0,
                        "snapshot": (
                            disconnected.latest_snapshot.to_mapping()
                            if disconnected.latest_snapshot is not None
                            else None
                        ),
                        "last_command_result": dict(disconnected.last_command_result),
                        "last_error": disconnected.last_error,
                    }
                )
        return tuple(rows)

    def send_command(self, vehicle_id: int, command: VehicleCommand) -> CommandDelivery:
        """Route one typed command to a registered vehicle; ack arrives asynchronously."""
        if not isinstance(vehicle_id, int) or isinstance(vehicle_id, bool) or vehicle_id < 0:
            return CommandDelivery(False, "vehicle_id must be a non-negative integer")
        if not isinstance(command, VehicleCommand):
            return CommandDelivery(False, "command must use VehicleCommand")
        if command.target_vehicle_id not in (None, vehicle_id):
            return CommandDelivery(False, "command target does not match requested vehicle")
        with self._sessions_lock:
            session = self._sessions.get(vehicle_id)
        if session is None:
            return CommandDelivery(False, "vehicle is not registered")
        routed = VehicleCommand(
            command_type=command.command_type,
            payload=command.payload,
            command_id=command.command_id,
            source=CommandSource.GROUND_STATION,
            issued_at_epoch_s=command.issued_at_epoch_s,
            target_vehicle_id=vehicle_id,
        )
        try:
            self._send(session, FrameType.COMMAND_REQUEST, routed.to_mapping())
        except OSError as exc:
            self._remove_session(session)
            return CommandDelivery(False, f"command delivery failed: {exc}")
        return CommandDelivery(True)

    def latest_lidar_scan(self, vehicle_id: int) -> LaserScanSample | None:
        """Return the newest bounded scan from one currently connected vehicle."""

        if not isinstance(vehicle_id, int) or isinstance(vehicle_id, bool) or vehicle_id < 0:
            raise ValueError("vehicle_id must be a non-negative integer")
        with self._sessions_lock:
            session = self._sessions.get(vehicle_id)
            return None if session is None else session.latest_lidar_scan

    def _accept_loop(self) -> None:
        while self._running.is_set():
            listener = self._listener
            if listener is None:
                return
            try:
                connection, address = listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            threading.Thread(
                target=self._serve_connection,
                args=(connection, address),
                name=f"GroundStationVehicle-{address[0]}:{address[1]}",
                daemon=True,
            ).start()

    def _serve_connection(self, connection: socket.socket, address: tuple[str, int]) -> None:
        connection.settimeout(0.5)
        decoder = FrameDecoder(max_frame_bytes=self._max_frame_bytes)
        session: VehicleSession | None = None
        try:
            while self._running.is_set():
                try:
                    data = connection.recv(65536)
                except socket.timeout:
                    continue
                if not data:
                    return
                for frame in decoder.feed(data):
                    if session is None:
                        if frame.frame_type != FrameType.REGISTER:
                            self._send_raw(connection, FrameType.ERROR, error_payload("registration_required", "REGISTER is required first"))
                            return
                        try:
                            session = self._register(connection, address, frame.payload)
                        except ProtocolError as exc:
                            self._send_raw(connection, FrameType.ERROR, error_payload("invalid_registration", str(exc)))
                            return
                        if session is None:
                            return
                        continue
                    self._process_frame(session, frame.frame_type, frame.payload)
        except ProtocolError as exc:
            if session is not None:
                try:
                    self._send(session, FrameType.ERROR, error_payload("protocol_error", str(exc)))
                except OSError:
                    pass
            self._logger.debug("Ground-station protocol error: %s", exc)
        except OSError as exc:
            self._logger.debug("Ground-station vehicle connection ended: %s", exc)
        finally:
            if session is not None:
                self._remove_session(session)
            self._close_connection(connection)

    def _register(
        self,
        connection: socket.socket,
        address: tuple[str, int],
        payload: Mapping[str, Any],
    ) -> VehicleSession | None:
        vehicle_id = _vehicle_id(payload.get("vehicle_id"))
        session_id = _nonempty_string(payload.get("session_id"), "session_id")
        capabilities = payload.get("capabilities", {})
        if not isinstance(capabilities, Mapping):
            raise ProtocolError("REGISTER capabilities must be a mapping")
        now = time.monotonic()
        session = VehicleSession(
            vehicle_id=vehicle_id,
            session_id=session_id,
            connection=connection,
            address=address,
            capabilities=dict(capabilities),
            connected_at_monotonic=now,
            last_received_monotonic=now,
        )
        with self._sessions_lock:
            if vehicle_id in self._sessions:
                self._send_raw(
                    connection,
                    FrameType.REGISTER_ACK,
                    {"accepted": False, "vehicle_id": vehicle_id, "reason": "duplicate_live_vehicle_id"},
                )
                return None
            self._sessions[vehicle_id] = session
            self._recent_disconnections.pop(vehicle_id, None)
        self._send(session, FrameType.REGISTER_ACK, {"accepted": True, "vehicle_id": vehicle_id})
        return session

    def _process_frame(self, session: VehicleSession, frame_type: FrameType, payload: Mapping[str, Any]) -> None:
        now = time.monotonic()
        if frame_type == FrameType.MONITORING_SNAPSHOT:
            snapshot = MonitoringSnapshot.from_mapping(payload)
            if snapshot.vehicle_id != session.vehicle_id:
                raise ProtocolError("snapshot vehicle_id does not match registered session")
            with self._sessions_lock:
                if self._sessions.get(session.vehicle_id) is session:
                    session.latest_snapshot = snapshot
                    session.last_received_monotonic = now
                    session.last_monitoring_monotonic = now
                    session.monitoring_received_monotonic.append(now)
            return
        if frame_type == FrameType.COMMAND_ACK:
            vehicle_id = _vehicle_id(payload.get("vehicle_id"))
            if vehicle_id != session.vehicle_id:
                raise ProtocolError("command acknowledgement vehicle_id does not match registered session")
            with self._sessions_lock:
                if self._sessions.get(session.vehicle_id) is session:
                    session.last_command_result = dict(payload)
                    session.last_received_monotonic = now
            return
        if frame_type == FrameType.LIDAR_SCAN:
            try:
                vehicle_id, scan = lidar_scan_from_mapping(payload)
            except (TypeError, ValueError) as error:
                raise ProtocolError(f"invalid LiDAR frame: {error}") from error
            if vehicle_id != session.vehicle_id:
                raise ProtocolError("LiDAR frame vehicle_id does not match registered session")
            with self._sessions_lock:
                if self._sessions.get(session.vehicle_id) is session:
                    session.latest_lidar_scan = scan
                    session.last_lidar_monotonic = now
                    session.last_received_monotonic = now
            return
        if frame_type == FrameType.ERROR:
            with self._sessions_lock:
                if self._sessions.get(session.vehicle_id) is session:
                    session.last_error = str(payload.get("message", "vehicle protocol error"))
                    session.last_received_monotonic = now
            return
        raise ProtocolError(f"unexpected vehicle frame: {frame_type.value}")

    def _send(self, session: VehicleSession, frame_type: FrameType, payload: Mapping[str, Any]) -> None:
        encoded = encode_frame(frame_type, payload, max_frame_bytes=self._max_frame_bytes)
        with session._send_lock:
            session.connection.sendall(encoded)

    def _send_raw(self, connection: socket.socket, frame_type: FrameType, payload: Mapping[str, Any]) -> None:
        connection.sendall(encode_frame(frame_type, payload, max_frame_bytes=self._max_frame_bytes))

    def _remove_session(self, session: VehicleSession) -> None:
        with self._sessions_lock:
            if self._sessions.get(session.vehicle_id) is session:
                del self._sessions[session.vehicle_id]
                self._recent_disconnections[session.vehicle_id] = DisconnectedVehicle(
                    vehicle_id=session.vehicle_id,
                    address=session.address,
                    disconnected_at_monotonic=time.monotonic(),
                    latest_snapshot=session.latest_snapshot,
                    last_command_result=dict(session.last_command_result or {}),
                    last_error=session.last_error,
                )
                while len(self._recent_disconnections) > self._MAX_RECENT_DISCONNECTIONS:
                    oldest_vehicle_id = min(
                        self._recent_disconnections,
                        key=lambda vehicle_id: self._recent_disconnections[vehicle_id].disconnected_at_monotonic,
                    )
                    del self._recent_disconnections[oldest_vehicle_id]

    @staticmethod
    def _close_connection(connection: socket.socket) -> None:
        try:
            connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            connection.close()
        except OSError:
            pass


def _vehicle_id(value: Any) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ProtocolError("vehicle_id must be a non-negative integer")
    return value


def _nonempty_string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ProtocolError(f"{name} must be a non-empty string")
    return value


def _recent_rate(timestamps, now_monotonic: float) -> float:
    return float(sum(1 for timestamp in timestamps if timestamp >= now_monotonic - 1.0))
