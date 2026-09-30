"""Vendor-neutral HIL wire protocol, transports and firmware backend."""

from __future__ import annotations

import json
import socket
import struct
import zlib
from collections import deque
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Callable, Deque, Dict, Iterable, List, Mapping, Optional, Protocol

from .codecs import NavSensorPacketCodecV2
from .firmware import BoardFirmwareCore, FirmwareStepResult, PythonReferenceFirmwareCore
from .hardware import HardwareTargetManifest, negotiate_capabilities
from .types import BusDelivery, BusFrame, ElectronicsSensorFrame, FrameStatus


HIL_MAGIC = b"CRHL"
HIL_PROTOCOL_VERSION = 1
_HEADER = struct.Struct("<4sBBHIQI")
_CRC = struct.Struct("<I")


class HILMessageType(IntEnum):
    HELLO = 1
    CAPABILITIES = 2
    NAV_SENSOR = 3
    VEHICLE_INPUT = 4
    V2V_INPUT = 5
    STEP = 6
    VEHICLE_OUTPUT = 7
    V2V_OUTPUT = 8
    STATUS = 9
    RESET = 10
    ERROR = 11


@dataclass(frozen=True)
class HILFrame:
    message_type: HILMessageType
    sequence: int
    timestamp_ns: int
    payload: bytes = b""
    flags: int = 0

    def encode(self) -> bytes:
        payload = bytes(self.payload)
        header = _HEADER.pack(
            HIL_MAGIC,
            HIL_PROTOCOL_VERSION,
            int(self.message_type),
            int(self.flags) & 0xFFFF,
            int(self.sequence) & 0xFFFFFFFF,
            int(self.timestamp_ns) & 0xFFFFFFFFFFFFFFFF,
            len(payload),
        )
        content = header + payload
        return content + _CRC.pack(zlib.crc32(content) & 0xFFFFFFFF)

    @classmethod
    def decode(cls, data: bytes) -> "HILFrame":
        raw = bytes(data)
        minimum = _HEADER.size + _CRC.size
        if len(raw) < minimum:
            raise ValueError("HIL frame is shorter than its header")
        magic, version, message_type, flags, sequence, timestamp_ns, length = (
            _HEADER.unpack_from(raw)
        )
        if magic != HIL_MAGIC:
            raise ValueError("Invalid HIL magic")
        if version != HIL_PROTOCOL_VERSION:
            raise ValueError(f"Unsupported HIL protocol version: {version}")
        expected_size = _HEADER.size + length + _CRC.size
        if len(raw) != expected_size:
            raise ValueError(
                f"Invalid HIL frame length: {len(raw)} != {expected_size}"
            )
        expected_crc = _CRC.unpack_from(raw, expected_size - _CRC.size)[0]
        actual_crc = zlib.crc32(raw[:-_CRC.size]) & 0xFFFFFFFF
        if actual_crc != expected_crc:
            raise ValueError("HIL frame CRC mismatch")
        return cls(
            message_type=HILMessageType(message_type),
            sequence=sequence,
            timestamp_ns=timestamp_ns,
            payload=raw[_HEADER.size : -_CRC.size],
            flags=flags,
        )


class HILTransport(Protocol):
    """Byte transport implemented by UDP, serial/CAN gateways or test loops."""

    def send(self, data: bytes) -> None: ...

    def receive(self, max_frames: int = 256) -> List[bytes]: ...

    def reset(self) -> None: ...

    def close(self) -> None: ...

    def get_status(self) -> Mapping[str, Any]: ...


class UDPHILTransport:
    """Non-blocking UDP transport suitable for MCU Ethernet and Linux SoCs."""

    def __init__(
        self,
        *,
        remote_host: str,
        remote_port: int,
        local_host: str = "0.0.0.0",
        local_port: int = 0,
        max_datagram_bytes: int = 65_507,
    ) -> None:
        self.remote_address = (str(remote_host), int(remote_port))
        self.max_datagram_bytes = max(256, int(max_datagram_bytes))
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.socket.bind((str(local_host), int(local_port)))
        self._sent = 0
        self._received = 0
        self._errors = 0

    def send(self, data: bytes) -> None:
        payload = bytes(data)
        if len(payload) > self.max_datagram_bytes:
            raise ValueError("HIL frame exceeds UDP transport capacity")
        self.socket.sendto(payload, self.remote_address)
        self._sent += 1

    def receive(self, max_frames: int = 256) -> List[bytes]:
        result = []
        for _ in range(max(0, int(max_frames))):
            try:
                payload, _ = self.socket.recvfrom(self.max_datagram_bytes)
            except BlockingIOError:
                break
            except OSError:
                self._errors += 1
                break
            result.append(bytes(payload))
            self._received += 1
        return result

    def reset(self) -> None:
        self.receive()

    def close(self) -> None:
        self.socket.close()

    def get_status(self) -> Mapping[str, Any]:
        return {
            "transport": "udp",
            "remote": f"{self.remote_address[0]}:{self.remote_address[1]}",
            "local": f"{self.socket.getsockname()[0]}:{self.socket.getsockname()[1]}",
            "sent": self._sent,
            "received": self._received,
            "errors": self._errors,
        }


class LoopbackHILTransport:
    """In-memory transport used by the cross-target compliance suite."""

    def __init__(self, endpoint: Callable[[bytes], Iterable[bytes]]) -> None:
        self.endpoint = endpoint
        self._incoming: Deque[bytes] = deque()
        self._sent = 0
        self._received = 0

    def send(self, data: bytes) -> None:
        self._sent += 1
        for response in self.endpoint(bytes(data)):
            self._incoming.append(bytes(response))

    def receive(self, max_frames: int = 256) -> List[bytes]:
        result = []
        while self._incoming and len(result) < max(0, int(max_frames)):
            result.append(self._incoming.popleft())
            self._received += 1
        return result

    def reset(self) -> None:
        self._incoming.clear()

    def close(self) -> None:
        self._incoming.clear()

    def get_status(self) -> Mapping[str, Any]:
        return {
            "transport": "loopback",
            "sent": self._sent,
            "received": self._received,
            "pending": len(self._incoming),
        }


def create_hil_transport(config: Mapping[str, Any]) -> HILTransport:
    values = dict(config or {})
    kind = str(values.get("transport", "udp")).strip().lower()
    if kind in {"udp", "ethernet_udp"}:
        return UDPHILTransport(
            remote_host=str(values.get("remote_host", "127.0.0.1")),
            remote_port=int(values.get("remote_port", 9100)),
            local_host=str(values.get("local_host", "0.0.0.0")),
            local_port=int(values.get("local_port", 0)),
            max_datagram_bytes=int(values.get("max_datagram_bytes", 65_507)),
        )
    raise ValueError(
        f"Unsupported HIL transport '{kind}'. Provide an adapter implementing "
        "HILTransport for UART, CAN, PCIe or another physical link."
    )


class HILFirmwareCore:
    """BoardFirmwareCore implementation backed by any external MCU or SoC."""

    def __init__(
        self,
        *,
        vehicle_id: int,
        local_manifest: HardwareTargetManifest,
        transport: HILTransport,
        strict_handshake: bool = True,
        required_features: Iterable[str] = ("firmware_core",),
        min_float_width_bits: int = 32,
        min_payload_bytes: int = 256,
        min_memory_bytes: int = 65_536,
        require_dynamic_allocation: bool = True,
        hello_retry_s: float = 1.0,
        link_timeout_s: float = 2.0,
    ) -> None:
        self.vehicle_id = int(vehicle_id)
        self.local_manifest = local_manifest
        self.transport = transport
        self.strict_handshake = bool(strict_handshake)
        self.required_features = tuple(str(item) for item in required_features)
        self.min_float_width_bits = int(min_float_width_bits)
        self.min_payload_bytes = int(min_payload_bytes)
        self.min_memory_bytes = int(min_memory_bytes)
        self.require_dynamic_allocation = bool(require_dynamic_allocation)
        self.hello_retry_ns = max(1, int(float(hello_retry_s) * 1e9))
        self.link_timeout_ns = max(1, int(float(link_timeout_s) * 1e9))
        self.remote_manifest: Optional[HardwareTargetManifest] = None
        self.negotiation: Dict[str, Any] = {
            "accepted": False,
            "blockers": ["capability handshake pending"],
        }
        self._sequence = 0
        self._frames_sent = 0
        self._frames_received = 0
        self._decode_errors = 0
        self._remote_errors = 0
        self._last_remote_status: Dict[str, Any] = {}
        self._last_hello_ns = -self.hello_retry_ns
        self._last_receive_ns = -1
        self._send_hello(0)

    @property
    def ready(self) -> bool:
        return bool(self.negotiation.get("accepted", False))

    def reset(self) -> None:
        self._send(HILMessageType.RESET, b"", 0)
        self.transport.reset()
        self.remote_manifest = None
        self.negotiation = {
            "accepted": False,
            "blockers": ["capability handshake pending"],
        }
        self._last_remote_status = {}
        self._last_hello_ns = -self.hello_retry_ns
        self._last_receive_ns = -1
        self._send_hello(0)

    def close(self) -> None:
        self.transport.close()

    def on_nav_frame(
        self, frame: ElectronicsSensorFrame, delivery: BusDelivery
    ) -> None:
        self._send(
            HILMessageType.NAV_SENSOR,
            NavSensorPacketCodecV2.encode(frame),
            frame.timestamp_ns,
        )

    def on_vehicle_payload(self, payload: bytes, timestamp_ns: int) -> None:
        self._send(HILMessageType.VEHICLE_INPUT, bytes(payload), timestamp_ns)

    def on_v2v_payload(self, payload: bytes, timestamp_ns: int) -> None:
        self._send(HILMessageType.V2V_INPUT, bytes(payload), timestamp_ns)

    def step(self, timestamp_ns: int) -> FirmwareStepResult:
        now_ns = int(timestamp_ns)
        if (
            self.ready
            and self._last_receive_ns >= 0
            and now_ns - self._last_receive_ns > self.link_timeout_ns
        ):
            self.negotiation = {
                **self.negotiation,
                "accepted": False,
                "blockers": ["HIL link timeout"],
            }
        if not self.ready and now_ns - self._last_hello_ns >= self.hello_retry_ns:
            self._send_hello(now_ns)
        self._send(HILMessageType.STEP, b"", timestamp_ns)
        vehicle_payloads: List[bytes] = []
        v2v_payloads: List[bytes] = []
        for raw in self.transport.receive():
            try:
                frame = HILFrame.decode(raw)
                self._frames_received += 1
                self._last_receive_ns = now_ns
                if frame.message_type == HILMessageType.CAPABILITIES:
                    values = json.loads(frame.payload.decode("utf-8"))
                    self.remote_manifest = HardwareTargetManifest.from_config(
                        values, vehicle_id=self.vehicle_id
                    )
                    self.negotiation = negotiate_capabilities(
                        self.remote_manifest,
                        required_features=self.required_features,
                        min_float_width_bits=self.min_float_width_bits,
                        min_payload_bytes=self.min_payload_bytes,
                        min_memory_bytes=self.min_memory_bytes,
                        require_dynamic_allocation=self.require_dynamic_allocation,
                    )
                elif frame.message_type == HILMessageType.VEHICLE_OUTPUT:
                    vehicle_payloads.append(frame.payload)
                elif frame.message_type == HILMessageType.V2V_OUTPUT:
                    v2v_payloads.append(frame.payload)
                elif frame.message_type == HILMessageType.STATUS:
                    self._last_remote_status = json.loads(
                        frame.payload.decode("utf-8")
                    )
                elif frame.message_type == HILMessageType.ERROR:
                    self._remote_errors += 1
                    self._last_remote_status["last_error"] = frame.payload.decode(
                        "utf-8", errors="replace"
                    )
            except Exception as exc:
                self._decode_errors += 1
                self._last_remote_status["last_decode_error"] = str(exc)

        if self.strict_handshake and not self.ready:
            vehicle_payloads.clear()
            v2v_payloads.clear()
        return FirmwareStepResult(
            vehicle_payloads=vehicle_payloads,
            v2v_payloads=v2v_payloads,
            status={
                "backend": "hil_external",
                "handshake_ready": self.ready,
                "capability_negotiation": dict(self.negotiation),
                "remote_target": None
                if self.remote_manifest is None
                else self.remote_manifest.to_dict(),
                "remote_status": dict(self._last_remote_status),
                "frames_sent": self._frames_sent,
                "frames_received": self._frames_received,
                "decode_errors": self._decode_errors,
                "remote_errors": self._remote_errors,
                "link_alive": self._last_receive_ns >= 0
                and now_ns - self._last_receive_ns <= self.link_timeout_ns,
                "last_receive_age_ns": None
                if self._last_receive_ns < 0
                else max(0, now_ns - self._last_receive_ns),
                "transport": dict(self.transport.get_status()),
            },
        )

    def _send(
        self, message_type: HILMessageType, payload: bytes, timestamp_ns: int
    ) -> None:
        frame = HILFrame(
            message_type=message_type,
            sequence=self._sequence,
            timestamp_ns=int(timestamp_ns),
            payload=bytes(payload),
        )
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        self.transport.send(frame.encode())
        self._frames_sent += 1

    def _send_hello(self, timestamp_ns: int) -> None:
        self._send(
            HILMessageType.HELLO,
            json.dumps(
                self.local_manifest.to_dict(), separators=(",", ":")
            ).encode(),
            timestamp_ns,
        )
        self._last_hello_ns = int(timestamp_ns)


class ReferenceHILDevice:
    """Protocol-compliant target emulator for tests and new-port bring-up."""

    def __init__(
        self,
        *,
        manifest: HardwareTargetManifest,
        vehicle_id: int,
        firmware_core: Optional[BoardFirmwareCore] = None,
        publish_rate_hz: float = 20.0,
    ) -> None:
        self.manifest = manifest
        self.core = firmware_core or PythonReferenceFirmwareCore(
            vehicle_id=vehicle_id, publish_rate_hz=publish_rate_hz
        )
        self._sequence = 0
        self._received = 0

    def handle(self, data: bytes) -> Iterable[bytes]:
        request = HILFrame.decode(data)
        self._received += 1
        responses: List[bytes] = []
        if request.message_type == HILMessageType.HELLO:
            responses.append(
                self._response(
                    HILMessageType.CAPABILITIES,
                    json.dumps(
                        self.manifest.to_dict(), separators=(",", ":")
                    ).encode(),
                    request.timestamp_ns,
                )
            )
        elif request.message_type == HILMessageType.RESET:
            self.core.reset()
        elif request.message_type == HILMessageType.NAV_SENSOR:
            sensor = NavSensorPacketCodecV2.decode(request.payload)
            frame = BusFrame(
                frame_id=request.sequence,
                requested_at_ns=request.timestamp_ns,
                source="sensor_node",
                destination="compute_node",
                payload=request.payload,
            )
            delivery = BusDelivery(
                frame=frame,
                available_at_ns=request.timestamp_ns,
                status=FrameStatus.OK,
                payload=request.payload,
            )
            self.core.on_nav_frame(sensor, delivery)
        elif request.message_type == HILMessageType.VEHICLE_INPUT:
            self.core.on_vehicle_payload(request.payload, request.timestamp_ns)
        elif request.message_type == HILMessageType.V2V_INPUT:
            self.core.on_v2v_payload(request.payload, request.timestamp_ns)
        elif request.message_type == HILMessageType.STEP:
            result = self.core.step(request.timestamp_ns)
            responses.extend(
                self._response(
                    HILMessageType.VEHICLE_OUTPUT,
                    payload,
                    request.timestamp_ns,
                )
                for payload in result.vehicle_payloads
            )
            responses.extend(
                self._response(
                    HILMessageType.V2V_OUTPUT,
                    payload,
                    request.timestamp_ns,
                )
                for payload in result.v2v_payloads
            )
            status = dict(result.status)
            status["hil_frames_received"] = self._received
            responses.append(
                self._response(
                    HILMessageType.STATUS,
                    json.dumps(status, separators=(",", ":"), default=str).encode(),
                    request.timestamp_ns,
                )
            )
        return responses

    def _response(
        self, message_type: HILMessageType, payload: bytes, timestamp_ns: int
    ) -> bytes:
        frame = HILFrame(
            message_type=message_type,
            sequence=self._sequence,
            timestamp_ns=timestamp_ns,
            payload=payload,
        )
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        return frame.encode()
