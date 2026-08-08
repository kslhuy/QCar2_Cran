# `utils/ground_station/protocol.py`

## 1. Introduction

Defines the bounded, versioned MessagePack wire format used by vehicle bridge
and operator server over a TCP byte stream.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `PROTOCOL_VERSION` / `DEFAULT_MAX_FRAME_BYTES` | No runtime inputs | Version `1` / 64 KiB default | Define compatible wire version and encoded-body maximum. |
| `ProtocolError(ValueError)` | Malformed frame condition | Protocol exception | Represents invalid, oversized, or unsupported frame data. |
| `FrameType(str, Enum)` | No runtime inputs | Typed frame discriminator | Enumerates registration, monitoring, command, acknowledgement, and error frames. |
| `ProtocolFrame(frame_type, payload)` | Validated `FrameType` and mapping payload | Frozen decoded frame | Represents one versioned protocol message. |
| `encode_frame(frame_type, payload, *, max_frame_bytes=...)` | Frame type, mapping payload, size limit | Length-prefixed `bytes` or `ProtocolError` | MessagePack-encodes version/type/payload and prepends network-order length. |
| `decode_frame(encoded, *, max_frame_bytes=...)` | One length-prefix-free body and limit | `ProtocolFrame` or `ProtocolError` | Validates size/version/type/payload and decodes MessagePack. |
| `FrameDecoder(max_frame_bytes=...)` | Maximum body size | Incremental decoder or `ProtocolError` | Owns partial TCP byte buffer. |
| `FrameDecoder.feed(data)` | Arbitrary TCP bytes | Complete frame list or `ProtocolError` | Buffers reads and emits every complete length-prefixed frame. |
| `error_payload(code, message)` | Error code and message | Two-string mapping | Builds common error payload. |
| `_validate_max_frame_bytes(value)` | Candidate maximum | `None` or `ProtocolError` | Requires an integer non-Boolean maximum of at least 64. |

## 3. Special data and cross-references

Every frame has a version, `FrameType`, mapping payload, and bounded encoded body.
`FrameDecoder` is deliberately stateful because a TCP read may contain part of a
frame or many frames; it does not decide command eligibility or monitoring
semantics.

## 4. Position in the project

Shared by [[bridge-tcp|GroundStationClientBridge]] and
[[server|GroundStationServer]]. It is serialization only: neither endpoint
may use it to bypass core command handling or runtime safe actuation.

## 5. Use and verification

`test/unit_test_ground_station.py` covers round trips, fragmented/multiple
reads, invalid lengths, oversize bodies, invalid MessagePack, and version/type
validation.
