# `utils/ground_station/bridge_tcp.py`

## 1. Introduction

Implements the reconnecting vehicle-side TCP client bridge. It transports
queued intent and observation only; its background thread never calls vehicle
runtime code.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `GroundStationClientBridge(config, vehicle_id=0, logger=None)` | Transport profile, vehicle ID, optional logger | Bridge instance or `ValueError` | Validates configuration, creates bounded queues, and allocates session ID. |
| `start()` / `stop()` | Lifecycle request | Daemon thread start / closed cleared connection | Start reconnect thread or close/join active transport. |
| `drain_commands(limit)` | Positive batch limit | Bounded `list[VehicleCommand]` or `ValueError` | Returns FIFO commands and then newest coalesced manual input if capacity remains. |
| `publish_snapshot(snapshot)` | `MonitoringSnapshot` | Latest retained sample or `TypeError` | Rate-limits and atomically replaces unsent snapshot. |
| `publish_ack(result)` | `CommandResult` | Queued `COMMAND_ACK` or `TypeError` | Serializes acknowledgement into bounded outbound control queue. |
| `get_status()` | Internal transport state | Diagnostics mapping | Reports connection, registration, errors, and counters. |
| `_run()` / `_run_connection()` | Running flag, socket/profile state | Reconnecting session lifecycle | Connect, register, then multiplex inbound frames and pending outbound data. |
| `_receive_available(connection, decoder)` / `_handle_frame(frame_type, payload)` | Readable socket and decoded frame | Queued command, registration state, or `ProtocolError` | Accept only valid frame types in their session phase. |
| `_queue_command(payload)` | Command-request payload | Queued command/coalesced manual input/rejection acknowledgement | Validates source and target before bounded command admission. |
| `_offer_control(...)` / `_send_queued_control(...)` / `_send_latest_snapshot(...)` | Typed payload, socket, queued state | Queued/sent control and newest snapshot | Bound acknowledgements and transmit control before latest observation. |
| `_send_direct(...)` / `_close_connection()` | Socket, frame, or socket state | Serialized frame / reset connection state | Send one frame and safely reset current connection. |
| `_required_string(...)` / `_port(...)` / `_positive_float(...)` / `_positive_int(...)` | Profile field values | Validated value or `ValueError` | Validate required host, port, and positive parameters. |

## 3. Special data and cross-references

Inbound `COMMAND_REQUEST` frames become [[commands|VehicleCommand]] only after
source/target validation. Normal commands are FIFO and bounded; repeated
`MANUAL_INPUT` replaces the retained latest input. Outbound acknowledgements are
bounded, while snapshots are rate-limited and latest-value-only.

## 4. Position in the project

[[runtime-facade|GroundStationRuntimeFacade]] is the only runtime-facing
consumer. This class owns socket/thread lifecycle and framed
[[protocol|protocol]] transport, but cannot invoke the state machine,
controller, or [[io-base|IOBase]].

## 5. Use and verification

`test/unit_test_ground_station.py` covers registration, rejection, bounded
queues, manual coalescing, monitoring rate limiting, reconnect, and shutdown.
`test/test_integration_ground_station.py` covers the live server/bridge path.
