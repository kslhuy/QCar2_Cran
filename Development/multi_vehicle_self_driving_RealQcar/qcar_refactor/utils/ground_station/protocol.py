"""Versioned MessagePack framing for the ground-station TCP connection."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from struct import pack, unpack
from typing import Any, Mapping

import msgpack


PROTOCOL_VERSION = 1
DEFAULT_MAX_FRAME_BYTES = 64 * 1024


class ProtocolError(ValueError):
    """Raised for malformed, oversized, or unsupported TCP frames."""


class FrameType(str, Enum):
    REGISTER = "REGISTER"
    REGISTER_ACK = "REGISTER_ACK"
    MONITORING_SNAPSHOT = "MONITORING_SNAPSHOT"
    COMMAND_REQUEST = "COMMAND_REQUEST"
    COMMAND_ACK = "COMMAND_ACK"
    ERROR = "ERROR"


@dataclass(frozen=True)
class ProtocolFrame:
    """One decoded, versioned protocol message."""

    frame_type: FrameType
    payload: Mapping[str, Any]


def encode_frame(
    frame_type: FrameType,
    payload: Mapping[str, Any],
    *,
    max_frame_bytes: int = DEFAULT_MAX_FRAME_BYTES,
) -> bytes:
    """Serialize one checked frame with a four-byte network-order length."""
    if not isinstance(frame_type, FrameType):
        try:
            frame_type = FrameType(frame_type)
        except (TypeError, ValueError) as exc:
            raise ProtocolError("unsupported frame type") from exc
    if not isinstance(payload, Mapping):
        raise ProtocolError("frame payload must be a mapping")
    _validate_max_frame_bytes(max_frame_bytes)
    try:
        encoded = msgpack.packb(
            {
                "version": PROTOCOL_VERSION,
                "type": frame_type.value,
                "payload": dict(payload),
            },
            use_bin_type=True,
        )
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f"frame payload is not MessagePack serializable: {exc}") from exc
    if not encoded or len(encoded) > max_frame_bytes:
        raise ProtocolError("frame exceeds configured maximum size")
    return pack("!I", len(encoded)) + encoded


def decode_frame(encoded: bytes, *, max_frame_bytes: int = DEFAULT_MAX_FRAME_BYTES) -> ProtocolFrame:
    """Decode one MessagePack frame body after its length prefix is removed."""
    _validate_max_frame_bytes(max_frame_bytes)
    if not encoded or len(encoded) > max_frame_bytes:
        raise ProtocolError("frame exceeds configured maximum size")
    try:
        data = msgpack.unpackb(encoded, raw=False, strict_map_key=False)
    except (TypeError, ValueError, msgpack.ExtraData) as exc:
        raise ProtocolError("invalid MessagePack frame") from exc
    if not isinstance(data, Mapping):
        raise ProtocolError("frame root must be a mapping")
    if data.get("version") != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol version")
    try:
        frame_type = FrameType(data.get("type"))
    except (TypeError, ValueError) as exc:
        raise ProtocolError("unsupported frame type") from exc
    payload = data.get("payload")
    if not isinstance(payload, Mapping):
        raise ProtocolError("frame payload must be a mapping")
    return ProtocolFrame(frame_type, dict(payload))


class FrameDecoder:
    """Incrementally decode length-prefixed frames from arbitrary TCP reads."""

    def __init__(self, *, max_frame_bytes: int = DEFAULT_MAX_FRAME_BYTES) -> None:
        _validate_max_frame_bytes(max_frame_bytes)
        self._max_frame_bytes = max_frame_bytes
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[ProtocolFrame]:
        if not isinstance(data, (bytes, bytearray)):
            raise ProtocolError("TCP data must be bytes")
        self._buffer.extend(data)
        frames: list[ProtocolFrame] = []
        while len(self._buffer) >= 4:
            frame_size = unpack("!I", self._buffer[:4])[0]
            if frame_size <= 0 or frame_size > self._max_frame_bytes:
                self._buffer.clear()
                raise ProtocolError("invalid frame size")
            if len(self._buffer) < 4 + frame_size:
                break
            body = bytes(self._buffer[4 : 4 + frame_size])
            del self._buffer[: 4 + frame_size]
            frames.append(decode_frame(body, max_frame_bytes=self._max_frame_bytes))
        return frames


def error_payload(code: str, message: str) -> dict[str, str]:
    """Build a consistent protocol error body."""
    return {"code": str(code), "message": str(message)}


def _validate_max_frame_bytes(value: int) -> None:
    if not isinstance(value, int) or isinstance(value, bool) or value < 64:
        raise ProtocolError("max_frame_bytes must be an integer of at least 64")
