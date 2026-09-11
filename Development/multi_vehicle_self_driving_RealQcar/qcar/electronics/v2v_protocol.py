"""Small binary envelopes between the vehicle host and compute target."""

from __future__ import annotations

import struct
from typing import Iterable, Tuple


V2V_TX_MAGIC = b"EVT1"
V2V_RX_MAGIC = b"EVR1"
V2V_MIRROR_MAGIC = b"EVM1"
_COUNT = struct.Struct(">B")
_TARGET = struct.Struct(">H")


class V2VEnvelopeError(ValueError):
    pass


def encode_host_v2v(
    payload: bytes,
    target_ids: Iterable[int],
    *,
    mirror_only: bool = False,
) -> bytes:
    targets = tuple(int(item) for item in target_ids)
    if len(targets) > 255:
        raise ValueError("A V2V envelope supports at most 255 targets")
    if any(item < 0 or item > 0xFFFF for item in targets):
        raise ValueError("V2V target IDs must fit in uint16")
    magic = V2V_MIRROR_MAGIC if mirror_only else V2V_TX_MAGIC
    header = bytearray(magic)
    header.extend(_COUNT.pack(len(targets)))
    for target_id in targets:
        header.extend(_TARGET.pack(target_id))
    header.extend(bytes(payload))
    return bytes(header)


def decode_host_v2v(envelope: bytes) -> Tuple[Tuple[int, ...], bytes, bool]:
    data = bytes(envelope)
    if len(data) < 5 or data[:4] not in {V2V_TX_MAGIC, V2V_MIRROR_MAGIC}:
        raise V2VEnvelopeError("Not a host-to-COM V2V envelope")
    count = data[4]
    payload_offset = 5 + count * _TARGET.size
    if len(data) < payload_offset:
        raise V2VEnvelopeError("Truncated V2V target list")
    targets = tuple(
        _TARGET.unpack_from(data, 5 + index * _TARGET.size)[0]
        for index in range(count)
    )
    return targets, data[payload_offset:], data[:4] == V2V_MIRROR_MAGIC


def encode_radio_v2v_rx(payload: bytes) -> bytes:
    return V2V_RX_MAGIC + bytes(payload)


def decode_radio_v2v_rx(envelope: bytes) -> bytes:
    data = bytes(envelope)
    if not data.startswith(V2V_RX_MAGIC):
        raise V2VEnvelopeError("Not a COM-to-host V2V envelope")
    return data[len(V2V_RX_MAGIC) :]
