# `extra/ground_station/core/server.py`

## 1. Introduction

Owns the operator TCP listener, registered vehicle sessions, and the most recent
operator-visible monitoring/acknowledgement data.

## 2. Code structure

| Definition                                                                               | Inputs                                                      | Output                                            | Algorithm or purpose                                                           |
| ---------------------------------------------------------------------------------------- | ----------------------------------------------------------- | ------------------------------------------------- | ------------------------------------------------------------------------------ |
| `VehicleSession(...)`                                                                    | Registered socket, identity, capabilities, and timing state | Mutable session record                            | Holds one live vehicle's monitoring, acknowledgement, and send-lock state.     |
| `DisconnectedVehicle(...)`                                                               | Identity, address, final monitoring/result/error data       | Frozen diagnostic record                          | Retains bounded recent-disconnect diagnostics.                                 |
| `CommandDelivery(accepted, reason='')`                                                   | Immediate transport acceptance and optional reason          | Frozen delivery result                            | Separates socket delivery from the later `COMMAND_ACK` outcome.                |
| `GroundStationServer(host='0.0.0.0', port=5000, *, max_frame_bytes=..., logger=None)`    | Listener and frame-limit configuration                      | Server instance or `ValueError`                   | Validates parameters and creates the guarded session registry.                 |
| `port`                                                                                   | Listener state                                              | Bound or configured `int` port                    | Returns OS-assigned port after start.                                          |
| `start()` / `stop()`                                                                     | Server lifecycle request                                    | Listener/thread start or closed/cleared state     | Starts acceptance or closes listener/sessions and clears retained diagnostics. |
| `session_rows()`                                                                         | Registry state                                              | Tuple of dashboard-safe mappings                  | Copies connected and retained-disconnect information under the registry lock.  |
| `send_command(vehicle_id, command)`                                                      | Target ID and `VehicleCommand`                              | `CommandDelivery`                                 | Validates/retargets ground-station command and sends a command-request frame.  |
| `_accept_loop()` / `_serve_connection(connection, address)`                              | Listener or one accepted socket                             | Per-connection servicing and cleanup              | Require first-frame `REGISTER`, decode later frames, and clean up on failure.  |
| `_register(connection, address, payload)`                                                | Registration payload                                        | `VehicleSession`, `None`, or protocol error frame | Validates registration, rejects duplicate live IDs, and sends `REGISTER_ACK`.  |
| `_process_frame(session, frame_type, payload)`                                           | Registered session and decoded frame                        | Updated session state or `ProtocolError`          | Stores monitoring/acknowledgement/error reports; rejects mismatches.           |
| `_send(...)` / `_send_raw(...)` / `_remove_session(session)`                             | Socket/session and typed payload                            | Serialized frame or retained disconnect record    | Send under the socket lock and retain bounded diagnostics.                     |
| `_vehicle_id(value)` / `_nonempty_string(value, name)` / `_recent_rate(timestamps, now)` | Protocol field or timestamp deque                           | Validated value or one-second rate                | Validate local fields and calculate monitoring rate.                           |

## 3. Special data and cross-references

The framed contract is [[protocol|ground-station protocol]]: registration,
`MonitoringSnapshot`, command requests, acknowledgements, and errors. The
server routes a typed [[commands|VehicleCommand]]; it does not infer a command
result from successful socket delivery.

## 4. Position in the project

External operator process. The corresponding vehicle boundary is
[[runtime-facade|GroundStationRuntimeFacade]]; that facade and core retain
eligibility, arbitration, and safe-zero decisions.

## 5. Use and verification

`test/unit_test_ground_station.py` covers framing, registration, duplicate
sessions, routing, acknowledgement/monitoring storage, and disconnect retention.
`test/test_integration_ground_station.py` covers a live listener/vehicle path.
