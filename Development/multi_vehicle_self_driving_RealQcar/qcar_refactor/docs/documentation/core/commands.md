# `core/commands.py`

## 1. Introduction

This file defines the transport-independent command and acknowledgement wire
contract. [[command-handler|VehicleCommandHandler]] decides whether a valid
command may be applied; this module only validates syntax, identity, and
payload shape.

## 2. Code structure

`VehicleCommand` is initialized with a command type, mapping payload, optional
ID/source/issue time/target. `CommandResult` is initialized with command ID,
vehicle ID, outcome, runtime state, and optional reason fields. Both are frozen
dataclasses after validation.

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `CommandError` | invalid external data | exception | Signals malformed or unsupported command/result data. |
| `CommandType` | string enum value | command enum | Defines lifecycle, path/velocity, SDCS, fleet, and manual command kinds. |
| `CommandSource` | string enum value | source enum | Records local, ground-station, simulator, or runtime origin. |
| `CommandOutcome` | string enum value | outcome enum | Distinguishes applied, deferred, rejected, and failed execution. |
| `VehicleCommand.__post_init__()` | all command fields | frozen valid instance | Coerces enums, validates ID/time/target/payload, copies payload, and freezes it with `MappingProxyType`. |
| `VehicleCommand.from_mapping(data)` | serializable command mapping | `VehicleCommand` | Decodes a transport payload, providing default ID/source/time when absent. |
| `VehicleCommand.to_mapping()` | command instance | serializable mapping | Emits enum values and a mutable payload copy for transport. |
| `CommandResult.__post_init__()` | all result fields | frozen valid instance | Validates command/vehicle identity, outcome, state, and reason strings. |
| `CommandResult.from_mapping(data)` | serializable result mapping | `CommandResult` | Strictly decodes an acknowledgement from a transport boundary. |
| `CommandResult.to_mapping()` | result instance | serializable mapping | Emits a transport-safe acknowledgement mapping. |
| `_validate_payload(command_type, payload)` | command kind and payload mapping | none or `CommandError` | Rejects unknown fields and validates velocity, path, SDCS nodes/loop, manual axes, and reason text. |
| `_required_string(data, key)` | mapping/key | non-empty string | Validates required result string fields. |
| `_optional_string(value)` | optional value | string | Converts `None` to empty string and rejects other non-strings. |
| `_vehicle_id(value)` | candidate ID | non-negative integer | Validates a result vehicle identity. |

## 3. Special data and cross-references

`VehicleCommand.payload` is immutable after construction. Its physical/control
fields are `velocity` in m/s, `path` as a source string, or SDCS `nodes` in
map-node IDs and `loop` (`0`, `1`, `2`, or `"inf"`); manual throttle/steering
are normalized requests. `issued_at_epoch_s` is audit time only, never a
cross-machine control or observer timestamp.

`CommandResult` identifies the original command and target vehicle, then records
the runtime state and machine-readable/human-readable rejection or failure
reason. See [[vehicle-types|cross-layer data]] for runtime control data, which
is intentionally separate from operator intent.

## 4. Position in the project

Ground-station, simulator, and V2V boundaries serialize these records.
[[vehicle-runtime|VehicleRuntime]] passes decoded commands to
[[command-handler|VehicleCommandHandler]]; this file cannot transition
[[vehicle-state-machine|StateMachine]] or write an actuator.

## 5. Use and verification

Create commands with `VehicleCommand(CommandType.START, {})`, or call
`VehicleCommand.from_mapping()` immediately after decoding a transport frame.
`test/unit_test_commands.py` verifies enum coercion, payload validation,
immutable copies, and command/result round trips.
