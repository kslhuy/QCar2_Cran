# Ground-Station Utilities

## 1. Introduction

This package is the vehicle-side command/monitoring transport boundary. The
operator server/CLI is documented under `extra/ground_station`.

## 2. File structure and variations

The [bridge base](bridge-base.md) defines the abstract/null lifecycle;
[bridge TCP](bridge-tcp.md) adds reconnecting sockets, bounded queues, and
manual-input coalescing; [protocol](protocol.md) defines bounded MessagePack
frames; [monitoring](monitoring.md) defines immutable observation data; and
[runtime facade](runtime-facade.md) is the only runtime-facing wrapper.

## 3. Shared data and cross-references

Typed [[commands|VehicleCommand]]/`CommandResult` frames and monitoring
snapshots cross this boundary. Transport queues are not permission to actuate;
the runtime validates and applies one command at a time.

## 4. Position in the project

[[vehicle-runtime|VehicleRuntime]] uses the facade, while extra ground-station
code operates the server. The package cannot change state or write IO directly.

## 5. Use and verification

Select a ground-station bridge profile in configuration. Verify framing,
queueing, facade behavior, and reconnection with
`test/unit_test_ground_station.py`; verify live vehicle/server exchange with
`test/test_integration_ground_station.py`.

## Conclusion

All bridge implementations share the framed command/monitoring contract; they
differ only in transport, while runtime retains lifecycle and safety authority.
