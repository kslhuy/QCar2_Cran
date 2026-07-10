import json
import logging
import queue
import socket
import threading
import time
from typing import Any, Optional

from core.types import V2VState, VehicleStateEstimate
from utils.v2v.v2v_base import V2VBase


class V2VUdp(V2VBase):
    """Small JSON-over-UDP V2V adapter for local state broadcasts."""

    MESSAGE_TYPE_STATE = "STATE"
    MAX_PACKET_SIZE = 4096
    RECV_TIMEOUT_S = 0.05

    def __init__(
        self,
        config: dict[str, Any],
        vehicle_id: Optional[int] = None,
        logger: logging.Logger | None = None,
    ) -> None:
        self._logger = logger or logging.getLogger(self.__class__.__name__)
        self._vehicle_id = int(config.get("vehicle_id", 0) if vehicle_id is None else vehicle_id)
        self._bind_ip = str(config.get("bind_ip", "0.0.0.0"))
        self._base_port = int(config.get("base_port", 8000))
        self.local_port = int(config.get("local_port", self._base_port + self._vehicle_id))
        self.broadcast_rate_hz = float(config.get("broadcast_rate_hz", 20.0))
        self.peer_timeout_s = float(config.get("peer_timeout_s", 2.0))
        self._min_broadcast_interval_s = 1.0 / self.broadcast_rate_hz if self.broadcast_rate_hz > 0.0 else 0.0

        self._peers = self._parse_peers(config.get("peers", []))
        self._peer_states: dict[int, V2VState] = {}
        self._last_peer_seen: dict[int, float] = {}
        self._peer_lock = threading.Lock()

        self._recv_queue: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=200)
        self._socket: socket.socket | None = None
        self._recv_thread: threading.Thread | None = None
        self._running = False
        self._active = False
        self._last_broadcast_time = 0.0
        self._last_error = ""
        self._messages_sent = 0
        self._messages_received = 0
        self._packets_dropped = 0

    def start(self) -> None:
        if self._active:
            return
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self._bind_ip, self.local_port))
            sock.settimeout(self.RECV_TIMEOUT_S)
            self._socket = sock
            self._running = True
            self._active = True
            self._recv_thread = threading.Thread(
                target=self._receive_loop,
                name=f"UdpV2V-Recv-{self._vehicle_id}",
                daemon=True,
            )
            self._recv_thread.start()
        except Exception as exc:
            self._last_error = str(exc)
            self._logger.error("Failed to start UdpV2V: %s", exc)
            self.stop()
            raise

    def process_received_messages(self) -> None:
        """Drain received UDP packets and update the latest peer states."""
        now = time.time()
        while True:
            try:
                message = self._recv_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_message(message, now)

        self._expire_stale_peers(now)

    def broadcast_local_state(self, state: VehicleStateEstimate) -> bool:
        """Broadcast this vehicle's latest local state to configured peers."""
        if not self._active or self._socket is None:
            return False

        now = time.time()
        if now - self._last_broadcast_time < self._min_broadcast_interval_s:
            return False

        message = {
            "sender_id": self._vehicle_id,
            "timestamp": float(state.timestamp),
            "message_type": self.MESSAGE_TYPE_STATE,
            "payload": {
                "x": float(state.x),
                "y": float(state.y),
                "theta": float(state.theta),
                "velocity": float(state.velocity),
            },
        }
        data = json.dumps(message, separators=(",", ":")).encode("utf-8")

        sent_count = 0
        for peer_id, address in self._peers.items():
            try:
                self._socket.sendto(data, address)
                sent_count += 1
            except OSError as exc:
                self._last_error = str(exc)
                self._logger.debug("V2V send failed to peer %s: %s", peer_id, exc)

        if sent_count > 0:
            self._last_broadcast_time = now
            self._messages_sent += sent_count
            return True
        return False

    def get_peer_states(self) -> dict[int, V2VState]:
        with self._peer_lock:
            return dict(self._peer_states)

    def get_status(self) -> dict:
        thread_alive = bool(self._recv_thread and self._recv_thread.is_alive())
        with self._peer_lock:
            peer_ids = sorted(self._peer_states.keys())
        return {
            "enabled": True,
            "active": self._active,
            "vehicle_id": self._vehicle_id,
            "local_port": self.local_port,
            "configured_peer_count": len(self._peers),
            "peer_count": len(peer_ids),
            "peers": peer_ids,
            "thread_alive": thread_alive,
            "messages_sent": self._messages_sent,
            "messages_received": self._messages_received,
            "packets_dropped": self._packets_dropped,
            "last_error": self._last_error,
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
                packet, _addr = self._socket.recvfrom(self.MAX_PACKET_SIZE)
                message = json.loads(packet.decode("utf-8"))
                try:
                    self._recv_queue.put_nowait(message)
                except queue.Full:
                    self._packets_dropped += 1
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    self._last_error = "socket receive failed"
                break
            except (json.JSONDecodeError, UnicodeDecodeError) as exc:
                self._last_error = str(exc)
                self._logger.debug("Ignoring invalid V2V packet: %s", exc)
            except Exception as exc:
                self._last_error = str(exc)
                self._logger.debug("V2V receive loop error: %s", exc)

    def _handle_message(self, message: dict[str, Any], now: float) -> None:
        sender_id = int(message.get("sender_id", -1))
        if sender_id == self._vehicle_id:
            return
        if sender_id not in self._peers:
            return
        if message.get("message_type") != self.MESSAGE_TYPE_STATE:
            return

        payload = message.get("payload", {})
        state = V2VState(
            vehicle_id=sender_id,
            timestamp=float(message.get("timestamp", now)),
            x=float(payload.get("x", 0.0)),
            y=float(payload.get("y", 0.0)),
            theta=float(payload.get("theta", 0.0)),
            velocity=float(payload.get("velocity", 0.0)),
        )

        with self._peer_lock:
            self._peer_states[sender_id] = state
            self._last_peer_seen[sender_id] = now
        self._messages_received += 1

    def _expire_stale_peers(self, now: float) -> None:
        if self.peer_timeout_s <= 0.0:
            return
        with self._peer_lock:
            stale_ids = [
                peer_id
                for peer_id, last_seen in self._last_peer_seen.items()
                if now - last_seen > self.peer_timeout_s
            ]
            for peer_id in stale_ids:
                self._last_peer_seen.pop(peer_id, None)
                self._peer_states.pop(peer_id, None)

    def _parse_peers(self, peers: list[dict[str, Any]]) -> dict[int, tuple[str, int]]:
        parsed = {}
        for peer in peers:
            peer_id = int(peer["vehicle_id"])
            if peer_id == self._vehicle_id:
                continue
            peer_ip = str(peer.get("ip", "127.0.0.1"))
            peer_port = int(peer.get("port", self._base_port + peer_id))
            parsed[peer_id] = (peer_ip, peer_port)
        return parsed
