# `utils/v2v/v2v_udp.py`

## 1. Introduction

`V2VUdp` implements [[v2v-base|V2VBase]] as bounded JSON-over-UDP transport.
It owns its socket and daemon receive thread, validates generic envelopes,
enforces per-message rate limits, and exposes link diagnostics.

## 2. Code structure

`V2VUdp(config, vehicle_id=None, logger=None)` resolves the local ID from its
argument or config, then initializes bind/peer/buffer/rate settings, bounded
receive queue, socket/thread state, sequence counters, and an RLock-protected
diagnostic state.

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `start()` | none | none or exception | Creates/configures/binds UDP socket and starts daemon receive thread; stops on startup failure. |
| `publish(message_type, payload, target_vehicle_ids)` | string, mapping, optional peer IDs | boolean | Validates/JSON-encodes bounded envelope, rate-limits by type, sends to targets, and advances sequence only after success. |
| `drain_received()` | none | ordered accepted messages | Drains queue and rejects non-increasing per-peer sequences while counting gaps/loss. |
| `get_status()` | none | diagnostics mapping | Reports active/thread state, ports/peers, counters, loss, publish rate, limits, buffers, and last error. |
| `stop()` | none | none | Clears running/active, closes owned socket, and joins receive thread up to one second. |
| `_receive_loop()` | socket packets | queue mutation | Receives bounded datagrams, timestamps/decodes valid configured-peer traffic, and counts malformed/overflow packets. |
| `_decode_message(packet, received_mono, received_perf_ns)` | bytes and local timestamps | `V2VMessage` | Validates JSON envelope fields and attaches local receive timestamps. |
| `_parse_peers(peers)` | peer list | `{vehicle_id: (ip, port)}` | Validates peer mappings, ignores self, and resolves default port. |
| `_resolve_targets(target_vehicle_ids)` | optional ID list | peer-address mapping | Returns all peers or validates a requested configured subset. |
| `_is_rate_limited(message_type, now_ns)` | kind/time | boolean | Compares last successful publish against configured period. |
| `_increment_dropped(error)` | error text | none | Increments dropped-packet counter and records error under lock. |
| `_set_last_error(error)` | error text | none | Records latest transport error under lock. |
| `_parse_rate_limits(value)` | mapping | `{type: positive Hz}` | Validates finite positive per-kind rate limits. |
| `_positive_integer(value, name)` | candidate value | positive integer | Validates buffer size. |
| `_integer(value, name)` | candidate value | non-negative integer | Validates IDs and sequences. |
| `_finite_number(value, name)` | candidate value | finite float | Validates sent monotonic timestamp. |

## 3. Special data and cross-references

The bounded queue holds decoded [[vehicle-types|V2VMessage]] envelopes.
`_next_sequence` is local send order; `_last_peer_sequence` and
`_peer_packets_lost` estimate per-peer packet gaps only when draining accepted
messages. `sent_at_*` comes from sender payload; `received_at_*` is local
receipt time. `MAX_DATAGRAM_BYTES=2048` bounds memory/network cost.

## 4. Position in the project

[[fleet-manager|FleetManager]] publishes and drains generic envelopes during
its cycle. This adapter does not decode fleet payloads, decide peer freshness,
or alter [[vehicle-runtime|VehicleRuntime]] lifecycle.

## 5. Use and verification

Configure unique local port, peer IDs/IP/ports, optional buffers, and optional
positive finite `message_rate_limits_hz`. Call `start()` before publish/drain
and `stop()` at shutdown. `test/unit_test_v2v.py` and multi-process V2V tests
cover framing, loss/order counters, rate limits, errors, and cleanup.
