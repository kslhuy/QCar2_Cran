# `utils/ground_station/monitoring.py`

## 1. Introduction

Defines the immutable, validated operator monitoring snapshot independent of
the control loop and socket implementation.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `MonitoringSnapshot(...)` | Vehicle/runtime/fleet/estimate/health state, mappings, optional result | Frozen snapshot or `ValueError` | Defines the immutable operator observation record. |
| `__post_init__()` | Constructed field values | Validated snapshot with read-only copied mappings | Validates scalar/health/reference data and freezes nested mappings. |
| `from_mapping(cls, data)` | Decoded monitoring mapping | `MonitoringSnapshot` or `ValueError` | Validates wire data and reconstructs optional `CommandResult`. |
| `to_mapping()` | Snapshot fields | Wire-safe copied mapping | Serializes all fields and optional command result. |
| `_mapping(value, name)` / `_string(value, name)` / `_vehicle_id(value)` | Raw mapping/string/ID fields | Validated value or `ValueError` | Validate common structured identity fields. |
| `_number(value, name)` / `_bool(value, name)` | Raw scalar field | Finite `float` / strict `bool` or `ValueError` | Normalize numeric and Boolean fields. |
| `_control_reference(value)` | Reference mapping | Validated reference mapping or `ValueError` | Requires numeric target fields and Boolean completion. |

## 3. Special data and cross-references

The snapshot crosses the bridge as a `MONITORING_SNAPSHOT` frame. Its nested
V2V/fleet/reference mappings are copied at construction, preventing transport
or UI code from mutating live runtime data. It is observation data only.

## 4. Position in the project

[[runtime-facade|GroundStationRuntimeFacade]] creates it from public runtime,
fleet, V2V, estimate, and reference state after a step; bridge/server/dashboard
code only transports or renders it.

## 5. Use and verification

`test/unit_test_ground_station.py` covers construction, mapping round trips,
invalid scalar/reference values, and immutable nested status sections.
