# `extra/simulator/virtual/scenario.py`

## 1. Introduction

Parses a virtual multi-vehicle scenario into validated immutable setup records.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `VirtualSetupError(ValueError)` | Invalid virtual scenario | Setup exception | Identifies platform-specific scenario validation failure. |
| `VirtualVehicleSetup(...)` | ID, waypoint route, speed, profiles, V2V, initial pose | Frozen vehicle setup | Holds one virtual vehicle's process-owned configuration. |
| `VirtualVehicleSetup.to_process_spec(simulation_profile='null', fleet_spec=None)` | Simulation profile and optional fleet spec | `VehicleProcessSpec` | Injects null IO/observer initial pose and selected profiles. |
| `VirtualSetup(simulation_profile, vehicles, fleet=None)` | Profile, vehicle tuple, optional fleet | Frozen scenario aggregate | Represents the setup consumed by virtual manager. |
| `parse_virtual_setup(argv=None)` | CLI argument list | `VirtualSetup` or `VirtualSetupError` | Parses `--setup-file` and delegates to loader. |
| `load_virtual_setup(path)` | YAML scenario path | `VirtualSetup` or `VirtualSetupError` | Requires null simulation implementation and parses vehicles/fleet. |
| `_parse_virtual_vehicle(data)` | One vehicle mapping | `VirtualVehicleSetup` or `VirtualSetupError` | Validates mission, V2V, profiles, and `[x, y, yaw]` pose. |
| `_validate_vehicle_ids(vehicles)` | Vehicle setup tuple | `None` or `VirtualSetupError` | Enforces unique IDs and explicit V2V local ports. |

## 3. Special data and cross-references

This integration module creates or coordinates [[vehicle-process|VehicleProcessSpec]] workers without bypassing [[vehicle-runtime|VehicleRuntime]].

Each vehicle record owns config, overrides, initial/mission data, and optional fleet/V2V settings; it has no shared runtime object.

## 4. Position in the project

Consumed by virtual process runner and shared scenario builder.

## 5. Use and verification

`test/unit_test_setup_virtual.py` covers malformed/valid profiles, missions,
V2V, and fleet input. `test/test_integration_multi_process_virtual.py` and its
fleet/control variants verify the produced worker setup.
