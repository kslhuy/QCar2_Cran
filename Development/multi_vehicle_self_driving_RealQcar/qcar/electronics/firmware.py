"""Replaceable firmware-core boundary for Python, native C/C++ or HIL backends."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Protocol

from .types import BusDelivery, ElectronicsSensorFrame
from .v2v_protocol import (
    V2V_MIRROR_MAGIC,
    V2V_TX_MAGIC,
    encode_radio_v2v_rx,
)


@dataclass
class FirmwareStepResult:
    vehicle_payloads: List[bytes] = field(default_factory=list)
    v2v_payloads: List[bytes] = field(default_factory=list)
    status: Mapping[str, Any] = field(default_factory=dict)


class BoardFirmwareCore(Protocol):
    """Stable boundary implemented locally or by an external MCU/SoC."""

    def reset(self) -> None: ...

    def on_nav_frame(
        self, frame: ElectronicsSensorFrame, delivery: BusDelivery
    ) -> None: ...

    def on_vehicle_payload(self, payload: bytes, timestamp_ns: int) -> None: ...

    def on_v2v_payload(self, payload: bytes, timestamp_ns: int) -> None: ...

    def step(self, timestamp_ns: int) -> FirmwareStepResult: ...


class PythonReferenceFirmwareCore:
    """Minimal compute-node reference application.

    It proves the replaceable boundary and publishes electronics sensor data to
    the vehicle link.  Trust, distributed estimation and controller code can be
    moved behind this same API incrementally without changing the board model.
    """

    def __init__(self, *, vehicle_id: int, publish_rate_hz: float = 20.0) -> None:
        self.vehicle_id = int(vehicle_id)
        self.publish_period_ns = (
            max(1, int(round(1e9 / publish_rate_hz))) if publish_rate_hz > 0.0 else 0
        )
        self._last_nav_frame: Optional[ElectronicsSensorFrame] = None
        self._last_publish_ns = -1
        self._vehicle_rx_count = 0
        self._v2v_rx_count = 0
        self._nav_rx_count = 0
        self._pending_vehicle_payloads: List[bytes] = []
        self._pending_v2v_payloads: List[bytes] = []

    def reset(self) -> None:
        self._last_nav_frame = None
        self._last_publish_ns = -1
        self._vehicle_rx_count = 0
        self._v2v_rx_count = 0
        self._nav_rx_count = 0
        self._pending_vehicle_payloads.clear()
        self._pending_v2v_payloads.clear()

    def on_nav_frame(
        self, frame: ElectronicsSensorFrame, delivery: BusDelivery
    ) -> None:
        self._last_nav_frame = frame
        self._nav_rx_count += 1

    def on_vehicle_payload(self, payload: bytes, timestamp_ns: int) -> None:
        self._vehicle_rx_count += 1
        data = bytes(payload)
        if data.startswith(V2V_TX_MAGIC):
            self._pending_v2v_payloads.append(data)
        elif data.startswith(V2V_MIRROR_MAGIC):
            # Mirror mode is observation-only; host remains authoritative.
            pass

    def on_v2v_payload(self, payload: bytes, timestamp_ns: int) -> None:
        self._v2v_rx_count += 1
        self._pending_vehicle_payloads.append(encode_radio_v2v_rx(payload))

    def step(self, timestamp_ns: int) -> FirmwareStepResult:
        vehicle_payloads: List[bytes] = list(self._pending_vehicle_payloads)
        v2v_payloads: List[bytes] = list(self._pending_v2v_payloads)
        self._pending_vehicle_payloads.clear()
        self._pending_v2v_payloads.clear()
        due = (
            self.publish_period_ns > 0
            and self._last_nav_frame is not None
            and (
                self._last_publish_ns < 0
                or timestamp_ns - self._last_publish_ns >= self.publish_period_ns
            )
        )
        if due:
            message = {
                "type": "electronics_sensor",
                "vehicle_id": self.vehicle_id,
                "timestamp_ns": int(timestamp_ns),
                "sensor": self._last_nav_frame.to_dict(),
            }
            vehicle_payloads.append(
                json.dumps(message, separators=(",", ":"), allow_nan=False).encode("utf-8")
            )
            self._last_publish_ns = int(timestamp_ns)

        return FirmwareStepResult(
            vehicle_payloads=vehicle_payloads,
            v2v_payloads=v2v_payloads,
            status={
                "backend": "python_reference",
                "nav_frames_received": self._nav_rx_count,
                "vehicle_frames_received": self._vehicle_rx_count,
                "v2v_frames_received": self._v2v_rx_count,
            },
        )
