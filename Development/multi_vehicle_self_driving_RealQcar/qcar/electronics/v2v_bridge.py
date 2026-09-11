"""Bridge raw V2V datagrams through a compute target and radio model."""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any, Callable, Deque, Dict, Mapping, Optional, Tuple

from .board import ElectronicsDigitalTwin
from .buses import BusFaultProfile, TimedSerialBus
from .types import FrameStatus, MCUState
from .v2v_protocol import V2VEnvelopeError, decode_host_v2v, encode_host_v2v


Address = Tuple[str, int]
RawSend = Callable[[bytes, Address], bool]
RawDeliver = Callable[[bytes, Address], None]


class ElectronicsV2VBridge:
    """Adapter used by ``V2VCommunication`` at its raw MsgPack boundary.

    ``mirror`` preserves the legacy host path while the target observes packets.
    ``firmware`` makes TX/RX traverse the compute target and vehicle bus.
    """

    VALID_MODES = {"mirror", "firmware"}

    def __init__(
        self,
        twin: ElectronicsDigitalTwin,
        config: Optional[Mapping[str, Any]] = None,
        *,
        clock_ns: Callable[[], int] = time.time_ns,
    ) -> None:
        self.twin = twin
        self.config = dict(config or {})
        self.mode = self._normalize_mode(self.config.get("mode", "firmware"))
        self._clock_ns = clock_ns
        faults = BusFaultProfile(**{
            key: value
            for key, value in dict(self.config.get("faults", {})).items()
            if key in {
                "enabled", "fixed_delay_s", "jitter_s",
                "drop_probability", "bit_error_rate",
            }
        })
        self.radio = TimedSerialBus(
            "electronics_v2v_radio",
            bitrate=float(self.config.get("bitrate_bps", 6_000_000.0)),
            seed=int(self.config.get("seed", self.twin.seed + 41)),
            faults=faults,
            crc_rejects_corruption=True,
            max_payload_bytes=int(self.config.get("max_packet_size", 1500)),
        )
        self._lock = threading.RLock()
        self._incoming: Deque[Tuple[bytes, Address, RawDeliver]] = deque(
            maxlen=int(self.config.get("incoming_queue_size", 256))
        )
        self._pending_rx_callbacks: Deque[Tuple[Address, RawDeliver]] = deque()
        self._peer_addresses: Dict[int, Address] = {}
        self._raw_send: Optional[RawSend] = None
        self._stats = {
            "host_tx_accepted": 0,
            "firmware_tx_emitted": 0,
            "radio_tx_delivered": 0,
            "radio_rx_accepted": 0,
            "host_rx_delivered": 0,
            "dropped_board_unavailable": 0,
            "dropped_no_route": 0,
            "incoming_queue_overflow": 0,
        }

    def outbound(
        self,
        payload: bytes,
        target_id: int,
        address: Address,
        send_callback: RawSend,
    ) -> bool:
        """Accept one host datagram for a specific peer."""
        target = int(target_id)
        now_ns = int(self._clock_ns())
        with self._lock:
            if (
                self.mode == "firmware"
                and self.twin.compute_node.state != MCUState.RUNNING
            ):
                self._stats["dropped_board_unavailable"] += 1
                return False
            self._peer_addresses[target] = (str(address[0]), int(address[1]))
            self._raw_send = send_callback
            envelope = encode_host_v2v(
                payload, [target], mirror_only=self.mode == "mirror"
            )
            self.twin.inject_vehicle_payload(envelope)
            if self.mode == "mirror":
                self._queue_radio(payload, target, address, now_ns)
            self._stats["host_tx_accepted"] += 1
        return True

    def inbound(
        self,
        payload: bytes,
        address: Address,
        deliver_callback: RawDeliver,
    ) -> None:
        """Queue an arrived UDP datagram for COM/host processing on main loop."""
        with self._lock:
            if len(self._incoming) == self._incoming.maxlen:
                self._stats["incoming_queue_overflow"] += 1
            self._incoming.append(
                (bytes(payload), (str(address[0]), int(address[1])), deliver_callback)
            )
            self._stats["radio_rx_accepted"] += 1

    def pump(self, now_ns: Optional[int] = None) -> None:
        """Advance COM ingress/egress and deliver due radio frames."""
        now = int(self._clock_ns() if now_ns is None else now_ns)
        with self._lock:
            self._pump_incoming()
            self._pump_firmware_tx(now)
            self._pump_firmware_rx()
            for delivery in self.radio.advance(now):
                if delivery.status == FrameStatus.DROPPED:
                    continue
                callback = delivery.frame.metadata.get("send_callback", self._raw_send)
                address = delivery.frame.metadata.get("address")
                if callback is None or not isinstance(address, tuple):
                    self._stats["dropped_no_route"] += 1
                    continue
                if callback(delivery.payload, address):
                    self._stats["radio_tx_delivered"] += 1

    def configure_faults(self, **values: Any) -> None:
        with self._lock:
            self.radio.configure_faults(**values)

    def set_mode(self, mode: str) -> None:
        normalized = self._normalize_mode(mode)
        with self._lock:
            self.mode = normalized
            self._incoming.clear()
            self._pending_rx_callbacks.clear()
            self.twin.pop_v2v_transmissions()
            self.twin.pop_v2v_host_receptions()

    def reset_transport(self) -> None:
        with self._lock:
            self.radio.reset()
            self._incoming.clear()
            self._pending_rx_callbacks.clear()
            self.twin.pop_v2v_transmissions()
            self.twin.pop_v2v_host_receptions()
            for key in self._stats:
                self._stats[key] = 0

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            result = dict(self._stats)
            result.update(
                {
                    "enabled": True,
                    "mode": self.mode,
                    "peer_routes": len(self._peer_addresses),
                    "pending_radio_rx": len(self._incoming),
                    "pending_host_rx": len(self._pending_rx_callbacks),
                    "radio": self.radio.statistics(),
                }
            )
            return result

    def _pump_incoming(self) -> None:
        while self._incoming:
            payload, address, callback = self._incoming.popleft()
            if self.mode == "mirror":
                self.twin.inject_v2v_payload(payload)
                callback(payload, address)
                self._stats["host_rx_delivered"] += 1
                continue
            if self.twin.inject_v2v_payload(payload):
                self._pending_rx_callbacks.append((address, callback))
            else:
                self._stats["dropped_board_unavailable"] += 1

    def _pump_firmware_tx(self, now_ns: int) -> None:
        for envelope in self.twin.pop_v2v_transmissions():
            try:
                targets, payload, mirror_only = decode_host_v2v(envelope)
            except V2VEnvelopeError:
                targets = tuple(self._peer_addresses)
                payload = envelope
                mirror_only = False
            if mirror_only:
                continue
            self._stats["firmware_tx_emitted"] += 1
            for target_id in targets:
                address = self._peer_addresses.get(target_id)
                if address is None:
                    self._stats["dropped_no_route"] += 1
                    continue
                self._queue_radio(payload, target_id, address, now_ns)

    def _pump_firmware_rx(self) -> None:
        for payload in self.twin.pop_v2v_host_receptions():
            if not self._pending_rx_callbacks:
                self._stats["dropped_no_route"] += 1
                continue
            address, callback = self._pending_rx_callbacks.popleft()
            callback(payload, address)
            self._stats["host_rx_delivered"] += 1

    def _queue_radio(
        self, payload: bytes, target_id: int, address: Address, now_ns: int
    ) -> None:
        self.radio.transmit(
            payload,
            source=(
                f"{self.twin.hardware_manifest.compute_node.node_id}_"
                f"{self.twin.vehicle_id}"
            ),
            destination=f"vehicle_{target_id}",
            requested_at_ns=now_ns,
            metadata={
                "target_id": int(target_id),
                "address": address,
                "send_callback": self._raw_send,
            },
        )

    @classmethod
    def _normalize_mode(cls, mode: Any) -> str:
        normalized = str(mode).strip().lower()
        if normalized not in cls.VALID_MODES:
            raise ValueError(f"Unsupported electronics V2V mode: {mode}")
        return normalized
