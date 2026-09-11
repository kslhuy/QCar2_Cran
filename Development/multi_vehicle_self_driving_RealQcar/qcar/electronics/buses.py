"""Deterministic transaction-level UART, SPI and CAN bus models."""

from __future__ import annotations

import heapq
import math
import random
from dataclasses import asdict, dataclass
from typing import Any, Dict, List, Optional, Tuple

from .types import BusDelivery, BusFrame, FrameStatus


@dataclass
class BusFaultProfile:
    """Faults applied below the protocol/application layer."""

    enabled: bool = True
    fixed_delay_s: float = 0.0
    jitter_s: float = 0.0
    drop_probability: float = 0.0
    bit_error_rate: float = 0.0

    def validate(self) -> None:
        if self.fixed_delay_s < 0.0:
            raise ValueError("fixed_delay_s must be non-negative")
        if self.jitter_s < 0.0:
            raise ValueError("jitter_s must be non-negative")
        if not 0.0 <= self.drop_probability <= 1.0:
            raise ValueError("drop_probability must be in [0, 1]")
        if not 0.0 <= self.bit_error_rate <= 1.0:
            raise ValueError("bit_error_rate must be in [0, 1]")


class TimedSerialBus:
    """Shared event-queue implementation for a serialized digital bus.

    ``transmit`` only queues a request.  ``advance`` performs arbitration,
    serialization, fault injection and returns deliveries whose simulated
    arrival time has elapsed.  No wall-clock sleep is used.
    """

    def __init__(
        self,
        name: str,
        bitrate: float,
        *,
        seed: int = 0,
        faults: Optional[BusFaultProfile] = None,
        crc_rejects_corruption: bool = False,
        max_payload_bytes: Optional[int] = None,
    ) -> None:
        if bitrate <= 0.0:
            raise ValueError("bitrate must be positive")
        self.name = name
        self.bitrate = float(bitrate)
        self.faults = faults or BusFaultProfile()
        self.faults.validate()
        self.crc_rejects_corruption = bool(crc_rejects_corruption)
        self.max_payload_bytes = max_payload_bytes
        self._rng = random.Random(seed)
        self._request_heap: List[Tuple[int, int, int, BusFrame]] = []
        self._delivery_heap: List[Tuple[int, int, BusDelivery]] = []
        self._counter = 0
        self._bus_free_at_ns = 0
        self._stats: Dict[str, Any] = {
            "queued": 0,
            "delivered": 0,
            "dropped": 0,
            "corrupted": 0,
            "crc_rejected": 0,
            "payload_bytes": 0,
            "wire_bits": 0,
            "last_latency_ns": 0,
        }

    def configure_faults(self, **values: Any) -> None:
        for key, value in values.items():
            if not hasattr(self.faults, key):
                raise KeyError(f"Unknown bus fault setting: {key}")
            setattr(self.faults, key, value)
        self.faults.validate()

    def reset(self) -> None:
        self._request_heap.clear()
        self._delivery_heap.clear()
        self._counter = 0
        self._bus_free_at_ns = 0
        for key in self._stats:
            self._stats[key] = 0

    def transmit(
        self,
        payload: bytes,
        *,
        source: str,
        destination: str,
        requested_at_ns: int,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> int:
        if not isinstance(payload, (bytes, bytearray, memoryview)):
            raise TypeError("payload must be bytes-like")
        payload_bytes = bytes(payload)
        frame_id = self._counter
        self._counter += 1
        frame = BusFrame(
            frame_id=frame_id,
            requested_at_ns=int(requested_at_ns),
            source=str(source),
            destination=str(destination),
            payload=payload_bytes,
            metadata=dict(metadata or {}),
        )
        heapq.heappush(
            self._request_heap,
            (frame.requested_at_ns, self._priority(frame), frame_id, frame),
        )
        self._stats["queued"] += 1
        self._stats["payload_bytes"] += len(payload_bytes)
        return frame_id

    def advance(self, now_ns: int) -> List[BusDelivery]:
        now_ns = int(now_ns)
        due_requests: List[Tuple[int, int, int, BusFrame]] = []
        while self._request_heap and self._request_heap[0][0] <= now_ns:
            due_requests.append(heapq.heappop(self._request_heap))

        # Lowest CAN arbitration ID wins when requests share a timestamp.
        due_requests.sort(key=lambda item: (item[0], item[1], item[2]))
        for _, _, _, frame in due_requests:
            self._schedule(frame)

        deliveries: List[BusDelivery] = []
        while self._delivery_heap and self._delivery_heap[0][0] <= now_ns:
            _, _, delivery = heapq.heappop(self._delivery_heap)
            deliveries.append(delivery)
            if delivery.status == FrameStatus.OK:
                self._stats["delivered"] += 1
            elif delivery.status == FrameStatus.CORRUPTED:
                self._stats["delivered"] += 1
                self._stats["corrupted"] += 1
            else:
                self._stats["dropped"] += 1
                if delivery.reason == "crc_error":
                    self._stats["crc_rejected"] += 1
        return deliveries

    def statistics(self) -> Dict[str, Any]:
        result = dict(self._stats)
        result.update(
            {
                "name": self.name,
                "bitrate": self.bitrate,
                "pending_requests": len(self._request_heap),
                "pending_deliveries": len(self._delivery_heap),
                "faults": asdict(self.faults),
            }
        )
        return result

    def _schedule(self, frame: BusFrame) -> None:
        if self.max_payload_bytes is not None and len(frame.payload) > self.max_payload_bytes:
            self._push_delivery(
                BusDelivery(
                    frame=frame,
                    available_at_ns=frame.requested_at_ns,
                    status=FrameStatus.DROPPED,
                    payload=b"",
                    reason="payload_too_large",
                )
            )
            return

        if not self.faults.enabled or self._rng.random() < self.faults.drop_probability:
            self._push_delivery(
                BusDelivery(
                    frame=frame,
                    available_at_ns=frame.requested_at_ns,
                    status=FrameStatus.DROPPED,
                    payload=b"",
                    reason="bus_disabled" if not self.faults.enabled else "fault_drop",
                )
            )
            return

        start_ns = max(frame.requested_at_ns, self._bus_free_at_ns)
        wire_bits = self._frame_bits(frame)
        duration_ns = max(1, int(math.ceil((wire_bits / self.bitrate) * 1e9)))
        end_ns = start_ns + duration_ns
        self._bus_free_at_ns = end_ns
        self._stats["wire_bits"] += wire_bits

        jitter_ns = 0
        if self.faults.jitter_s > 0.0:
            jitter_ns = int(round(self._rng.gauss(0.0, self.faults.jitter_s) * 1e9))
        extra_delay_ns = int(round(self.faults.fixed_delay_s * 1e9))
        available_ns = max(end_ns, end_ns + extra_delay_ns + jitter_ns)

        payload, corrupted_bits = self._corrupt_payload(frame.payload)
        status = FrameStatus.OK
        reason = ""
        if corrupted_bits:
            if self.crc_rejects_corruption:
                status = FrameStatus.DROPPED
                payload = b""
                reason = "crc_error"
            else:
                status = FrameStatus.CORRUPTED
                reason = "bit_error"

        self._stats["last_latency_ns"] = available_ns - frame.requested_at_ns
        self._push_delivery(
            BusDelivery(
                frame=frame,
                available_at_ns=available_ns,
                status=status,
                payload=payload,
                corrupted_bits=corrupted_bits,
                reason=reason,
            )
        )

    def _push_delivery(self, delivery: BusDelivery) -> None:
        heapq.heappush(
            self._delivery_heap,
            (delivery.available_at_ns, delivery.frame.frame_id, delivery),
        )

    def _corrupt_payload(self, payload: bytes) -> Tuple[bytes, int]:
        ber = self.faults.bit_error_rate
        if ber <= 0.0 or not payload:
            return payload, 0
        mutable = bytearray(payload)
        corrupted_bits = 0
        for byte_index in range(len(mutable)):
            value = mutable[byte_index]
            for bit_index in range(8):
                if self._rng.random() < ber:
                    value ^= 1 << bit_index
                    corrupted_bits += 1
            mutable[byte_index] = value
        return bytes(mutable), corrupted_bits

    def _priority(self, frame: BusFrame) -> int:
        return 0

    def _frame_bits(self, frame: BusFrame) -> int:
        return max(1, len(frame.payload) * 8)


class UARTBus(TimedSerialBus):
    def __init__(
        self,
        name: str,
        baud_rate: float = 115_200.0,
        *,
        data_bits: int = 8,
        stop_bits: int = 1,
        parity_enabled: bool = False,
        seed: int = 0,
        faults: Optional[BusFaultProfile] = None,
    ) -> None:
        super().__init__(name, baud_rate, seed=seed, faults=faults)
        self.data_bits = int(data_bits)
        self.stop_bits = int(stop_bits)
        self.parity_enabled = bool(parity_enabled)

    def _frame_bits(self, frame: BusFrame) -> int:
        bits_per_byte = 1 + self.data_bits + self.stop_bits + int(self.parity_enabled)
        return max(1, len(frame.payload) * bits_per_byte)


class SPIBus(TimedSerialBus):
    def __init__(
        self,
        name: str,
        clock_hz: float = 6_000_000.0,
        *,
        chip_select_setup_s: float = 1e-6,
        seed: int = 0,
        faults: Optional[BusFaultProfile] = None,
    ) -> None:
        super().__init__(name, clock_hz, seed=seed, faults=faults)
        self.chip_select_setup_s = float(chip_select_setup_s)

    def _frame_bits(self, frame: BusFrame) -> int:
        setup_bits = int(math.ceil(self.chip_select_setup_s * self.bitrate))
        return max(1, len(frame.payload) * 8 + setup_bits)


class CANBus(TimedSerialBus):
    def __init__(
        self,
        name: str,
        bitrate: float = 1_000_000.0,
        *,
        can_fd: bool = True,
        seed: int = 0,
        faults: Optional[BusFaultProfile] = None,
    ) -> None:
        super().__init__(
            name,
            bitrate,
            seed=seed,
            faults=faults,
            crc_rejects_corruption=True,
            max_payload_bytes=64 if can_fd else 8,
        )
        self.can_fd = bool(can_fd)

    def _priority(self, frame: BusFrame) -> int:
        return int(frame.metadata.get("arbitration_id", 0x7FF))

    def _frame_bits(self, frame: BusFrame) -> int:
        # Transaction-level approximation including arbitration/control/CRC and
        # a conservative bit-stuffing factor.
        overhead = 67 if self.can_fd else 47
        return max(1, int(math.ceil((overhead + len(frame.payload) * 8) * 1.2)))

