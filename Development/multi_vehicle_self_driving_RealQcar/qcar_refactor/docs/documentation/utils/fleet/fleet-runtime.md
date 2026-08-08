# `utils/fleet/fleet_runtime.py`

## 1. Introduction

Creates one process-local `FleetManager` from serializable fleet selection.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `FleetRuntimeSpec(registry)` | Shared `FleetRegistry` | Frozen process-spec field | Carries formation registry into one vehicle process without creating runtime state. |
| `build_fleet_manager(spec, vehicle_id, logger=None)` | Runtime spec, local ID, optional logger | Process-local `FleetManager` | Builds manager from shared formation snapshot for exactly one vehicle. |

## 3. Special data and cross-references

The spec holds registry definition, not a mutable manager; each process receives its own manager/observer lifecycle.

## 4. Position in the project

[[vehicle-process|build_vehicle_process_runtime]] calls this before attaching module V2V transport.

## 5. Use and verification

`test/unit_test_fleet.py` and multi-process fleet tests verify construction and isolation.
