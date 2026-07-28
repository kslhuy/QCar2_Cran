"""Generic JSON-over-UDP transport for configured V2V peers."""

from __future__ import annotations

import json
import logging
import math
import queue
import socket
import threading
import time
from collections import deque
from typing import Any, Optional

from core.vehicle_types import V2VMessage
from utils.v2v.v2v_base import V2VBase


class V2VUdp(V2VBase):
    """Route generic envelopes; payload interpretation belongs outside V2V."""

    MAX_DATAGRAM_BYTES = 2048
    RECV_TIMEOUT_S = 0.05
    DEFAULT_SEND_BUFFER_BYTES = 16384
    DEFAULT_RECEIVE_BUFFER_BYTES = 32768

    def __init__(self, config: dict[str, Any], vehicle_id: Optional[int] = None, logger: logging.Logger | None = None) -> None:
        resolved_vehicle_id = int(config.get("vehicle_id", 0) if vehicle_id is None else vehicle_id)
        super().__init__(config, resolved_vehicle_id, logger)
        self._bind_ip = str(self._config.get("bind_ip", "0.0.0.0"))
        self._base_port = int(self._config.get("base_port", 8000))
        self.local_port = int(self._config.get("local_port", self._base_port + self._vehicle_id))
        self._peers = self._parse_peers(self._config.get("peers", []))
        self._send_buffer_bytes = self._positive_integer(
            self._config.get("send_buffer_bytes", self.DEFAULT_SEND_BUFFER_BYTES), "send_buffer_bytes"
        )
        self._receive_buffer_bytes = self._positive_integer(
            self._config.get("receive_buffer_bytes", self.DEFAULT_RECEIVE_BUFFER_BYTES), "receive_buffer_bytes"
        )
        self._message_rate_limits_hz = self._parse_rate_limits(self._config.get("message_rate_limits_hz", {}))
        self._recv_queue: queue.Queue[V2VMessage] = queue.Queue(maxsize=200)
        self._socket: socket.socket | None = None
        self._recv_thread: threading.Thread | None = None
        self._running = False
        self._active = False
        self._next_sequence = 0
        self._last_publish_ns_by_type: dict[str, int] = {}
        self._successful_publish_ns: deque[int] = deque()
        self._last_peer_sequence: dict[int, int] = {}
        self._peer_packets_lost: dict[int, int] = {}
        self._last_error = ""
        self._messages_sent = 0
        self._messages_received = 0
        self._packets_dropped = 0
        self._out_of_order_packets = 0
        self._stats_lock = threading.RLock()

    def start(self) -> None:
        if self._active:
            return
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self._send_buffer_bytes)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self._receive_buffer_bytes)
            sock.bind((self._bind_ip, self.local_port))
            sock.settimeout(self.RECV_TIMEOUT_S)
            self._socket = sock
            self._running = True
            self._active = True
            self._recv_thread = threading.Thread(target=self._receive_loop, name=f"UdpV2V-Recv-{self._vehicle_id}", daemon=True)
            self._recv_thread.start()
        except Exception as exc:
            self._set_last_error(str(exc))
            self.stop()
            raise

    def publish(self, message_type: str, payload: dict, target_vehicle_ids: list[int] | None = None) -> bool:
        if not self._active or self._socket is None:
            return False
        if not isinstance(message_type, str) or not message_type:
            raise ValueError("V2V message_type must be a non-empty string")
        if not isinstance(payload, dict):
            raise ValueError("V2V payload must be a mapping")
        sent_at_monotonic = time.monotonic()
        sent_at_perf_counter_ns = time.perf_counter_ns()
        if self._is_rate_limited(message_type, sent_at_perf_counter_ns):
            return False
        with self._stats_lock:
            sequence = self._next_sequence
        message = {
            "sender_id": self._vehicle_id,
            "message_type": message_type,
            "payload": payload,
            "sequence": sequence,
            "sent_at_monotonic": sent_at_monotonic,
            "sent_at_perf_counter_ns": sent_at_perf_counter_ns,
        }
        try:
            data = json.dumps(message, separators=(",", ":"), allow_nan=False).encode("utf-8")
        except (TypeError, ValueError) as exc:
            raise ValueError(f"V2V payload is not JSON serializable: {exc}") from exc

        if len(data) > self.MAX_DATAGRAM_BYTES:
            raise ValueError(f"V2V UDP datagram exceeds {self.MAX_DATAGRAM_BYTES} bytes")

        targets = self._resolve_targets(target_vehicle_ids)
        sent_count = 0
        for peer_id, address in targets.items():
            try:
                self._socket.sendto(data, address)
                sent_count += 1
            except OSError as exc:
                self._set_last_error(str(exc))
                self._logger.debug("V2V send failed to peer %s: %s", peer_id, exc)
        if sent_count:
            with self._stats_lock:
                self._messages_sent += sent_count
                self._next_sequence += 1
                self._last_publish_ns_by_type[message_type] = sent_at_perf_counter_ns
                self._successful_publish_ns.append(sent_at_perf_counter_ns)
            return True
        return False

    def drain_received(self) -> list[V2VMessage]:
        messages = []
        while True:
            try:
                message = self._recv_queue.get_nowait()
            except queue.Empty:
                break
            with self._stats_lock:
                previous_sequence = self._last_peer_sequence.get(message.sender_id)
                if previous_sequence is not None:
                    if message.sequence <= previous_sequence:
                        self._out_of_order_packets += 1
                        continue
                    self._peer_packets_lost[message.sender_id] = self._peer_packets_lost.get(message.sender_id, 0) + message.sequence - previous_sequence - 1
                self._last_peer_sequence[message.sender_id] = message.sequence
                self._messages_received += 1
            messages.append(message)
        return messages

    def get_status(self) -> dict:
        with self._stats_lock:
            now_ns = time.perf_counter_ns()
            one_second_ago_ns = now_ns - 1_000_000_000
            while self._successful_publish_ns and self._successful_publish_ns[0] < one_second_ago_ns:
                self._successful_publish_ns.popleft()
            peer_packet_loss = dict(self._peer_packets_lost)
            messages_sent = self._messages_sent
            messages_received = self._messages_received
            packets_dropped = self._packets_dropped
            out_of_order_packets = self._out_of_order_packets
            last_error = self._last_error
            publish_rate_hz = float(len(self._successful_publish_ns))
        return {
            "enabled": True,
            "active": self._active,
            "vehicle_id": self._vehicle_id,
            "local_port": self.local_port,
            "configured_peer_count": len(self._peers),
            "thread_alive": bool(self._recv_thread and self._recv_thread.is_alive()),
            "messages_sent": messages_sent,
            "messages_received": messages_received,
            "packets_dropped": packets_dropped,
            "estimated_packets_lost": sum(peer_packet_loss.values()),
            "peer_packet_loss": peer_packet_loss,
            "out_of_order_packets": out_of_order_packets,
            "publish_rate_hz": publish_rate_hz,
            "message_rate_limits_hz": dict(self._message_rate_limits_hz),
            "max_datagram_bytes": self.MAX_DATAGRAM_BYTES,
            "send_buffer_bytes": self._send_buffer_bytes,
            "receive_buffer_bytes": self._receive_buffer_bytes,
            "last_error": last_error,
        }

    def stop(self) -> None:
        self._running = False
        self._active = False
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None
        if self._recv_thread and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=1.0)

    def _receive_loop(self) -> None:
        while self._running:
            try:
                if self._socket is None:
                    break
                packet, _address = self._socket.recvfrom(self.MAX_DATAGRAM_BYTES + 1)
                if len(packet) > self.MAX_DATAGRAM_BYTES:
                    self._increment_dropped("V2V UDP datagram exceeds maximum size")
                    continue
                received_at_monotonic = time.monotonic()
                received_at_perf_counter_ns = time.perf_counter_ns()
                message = self._decode_message(packet, received_at_monotonic, received_at_perf_counter_ns)
                if message.sender_id == self._vehicle_id or message.sender_id not in self._peers:
                    continue
                self._recv_queue.put_nowait(message)
            except socket.timeout:
                continue
            except queue.Full:
                self._increment_dropped("V2V receive queue is full")
            except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as exc:
                self._increment_dropped(str(exc))
                self._logger.debug("Ignoring invalid V2V packet: %s", exc)
            except OSError:
                if self._running:
                    self._set_last_error("socket receive failed")
                break

    def _decode_message(self, packet: bytes, received_at_monotonic: float, received_at_perf_counter_ns: int) -> V2VMessage:
        raw = json.loads(packet.decode("utf-8"))
        if not isinstance(raw, dict):
            raise ValueError("V2V packet root must be an object")
        payload = raw.get("payload")
        if not isinstance(payload, dict):
            raise ValueError("V2V payload must be an object")
        message_type = raw.get("message_type")
        if not isinstance(message_type, str) or not message_type:
            raise ValueError("V2V message_type must be a non-empty string")
        return V2VMessage(
            sender_id=self._integer(raw.get("sender_id"), "sender_id"),
            message_type=message_type,
            payload=payload,
            sequence=self._integer(raw.get("sequence"), "sequence"),
            sent_at_monotonic=self._finite_number(raw.get("sent_at_monotonic"), "sent_at_monotonic"),
            sent_at_perf_counter_ns=self._integer(raw.get("sent_at_perf_counter_ns"), "sent_at_perf_counter_ns"),
            received_at_monotonic=received_at_monotonic,
            received_at_perf_counter_ns=received_at_perf_counter_ns,
        )

    def _parse_peers(self, peers: list[dict[str, Any]]) -> dict[int, tuple[str, int]]:
        if not isinstance(peers, list):
            raise ValueError("V2V peers must be a list")
        parsed = {}
        for peer in peers:
            if not isinstance(peer, dict):
                raise ValueError("Each V2V peer must be an object")
            peer_id = self._integer(peer.get("vehicle_id"), "peer.vehicle_id")
            if peer_id == self._vehicle_id:
                continue
            peer_ip = peer.get("ip", "127.0.0.1")
            if not isinstance(peer_ip, str) or not peer_ip:
                raise ValueError("peer.ip must be a non-empty string")
            peer_port = peer.get("port", self._base_port + peer_id)
            if not isinstance(peer_port, int) or isinstance(peer_port, bool) or not 1 <= peer_port <= 65535:
                raise ValueError("peer.port must be an integer in [1, 65535]")
            parsed[peer_id] = (peer_ip, peer_port)
        return parsed

    def _resolve_targets(self, target_vehicle_ids: list[int] | None) -> dict[int, tuple[str, int]]:
        if target_vehicle_ids is None:
            return self._peers
        if not isinstance(target_vehicle_ids, list):
            raise ValueError("target_vehicle_ids must be a list of configured vehicle IDs")
        targets = {}
        for peer_id in target_vehicle_ids:
            peer_id = self._integer(peer_id, "target_vehicle_id")
            if peer_id not in self._peers:
                raise ValueError(f"V2V target {peer_id} is not a configured peer")
            targets[peer_id] = self._peers[peer_id]
        return targets

    def _is_rate_limited(self, message_type: str, now_ns: int) -> bool:
        rate_hz = self._message_rate_limits_hz.get(message_type)
        if rate_hz is None:
            return False
        with self._stats_lock:
            last_publish_ns = self._last_publish_ns_by_type.get(message_type, 0)
        return now_ns - last_publish_ns < int(1_000_000_000 / rate_hz)

    def _increment_dropped(self, error: str) -> None:
        with self._stats_lock:
            self._packets_dropped += 1
            self._last_error = error

    def _set_last_error(self, error: str) -> None:
        with self._stats_lock:
            self._last_error = error

    @staticmethod
    def _parse_rate_limits(value: Any) -> dict[str, float]:
        if not isinstance(value, dict):
            raise ValueError("message_rate_limits_hz must be a mapping")
        parsed = {}
        for message_type, rate_hz in value.items():
            if not isinstance(message_type, str) or not message_type:
                raise ValueError("message rate-limit type must be a non-empty string")
            if not isinstance(rate_hz, (int, float)) or isinstance(rate_hz, bool) or not math.isfinite(rate_hz) or rate_hz <= 0:
                raise ValueError("message rate limit must be a positive finite number")
            parsed[message_type] = float(rate_hz)
        return parsed

    @staticmethod
    def _positive_integer(value: Any, name: str) -> int:
        if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
            raise ValueError(f"{name} must be a positive integer")
        return value

    @staticmethod
    def _integer(value: Any, name: str) -> int:
        if not isinstance(value, int) or isinstance(value, bool) or value < 0:
            raise ValueError(f"{name} must be a non-negative integer")
        return value

    @staticmethod
    def _finite_number(value: Any, name: str) -> float:
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
            raise ValueError(f"{name} must be a finite number")
        return float(value)
