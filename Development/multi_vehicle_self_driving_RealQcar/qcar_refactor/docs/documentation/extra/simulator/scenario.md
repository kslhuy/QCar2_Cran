# `extra/simulator/scenario.py`

## 1. Introduction

Contains shared YAML scenario parsing, mission normalization, fleet parsing, and per-vehicle process-spec construction.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `FleetSetup(profile, registry)` | Fleet profile and registry | Frozen setup; formation / `FleetRuntimeSpec` | Wraps a scenario fleet and converts it to common runtime input. |
| `load_yaml_mapping(path, platform_name, error_type)` | YAML path and platform error class | YAML mapping or platform error | Loads one mapping and translates missing/malformed input. |
| `load_simulation_profile(profile, error_type)` | Simulation profile name | Profile mapping or platform error | Loads selected shared simulation profile. |
| `parse_fleet_setup(scenario, vehicles, error_type)` | Scenario mapping and parsed vehicles | `FleetSetup`, `None`, or platform error | Builds/validates fleet policy, members, and V2V routes. |
| `build_vehicle_process_spec(*, vehicle_id, vehicle_config_file, route, node_sequence=None, loop=0, target_velocity, module_overrides=None, selection_overrides=None, v2v=None, v2v_profile=None, fleet_spec=None)` | Parsed vehicle mission/configuration | `VehicleProcessSpec` or `ValueError` | Requires exactly one waypoint route or SDCS node sequence. |
| `require_mapping(...)` / `read_string(...)` | YAML value/source/key/error type | Validated mapping/string or platform error | Validate shared YAML structures. |
| `parse_mission_path(...)` / `parse_mission_node_sequence(...)` / `parse_mission_loop(...)` | Mission YAML values | Normalized route/nodes/loop or platform error | Accept only supported mission representations. |
| `validate_unique_vehicle_ids(...)` | Parsed vehicle tuple | `None` or platform error | Rejects duplicate process identities. |
| `parse_v2v_endpoint(...)` / `parse_v2v_profile(...)` / `validate_unique_v2v_local_ports(...)` | Endpoint/profile/vehicle values | Validated override/profile or platform error | Validate V2V settings and reject explicit port collisions. |
| `_v2v_peer_ids_by_vehicle(...)` / `_port(...)` | Vehicle entries or port value | Peer-ID map / validated port | Supply fleet route data and validate port ranges. |

## 3. Special data and cross-references

This integration module creates or coordinates [[vehicle-process|VehicleProcessSpec]] workers without bypassing [[vehicle-runtime|VehicleRuntime]].

Mission has either waypoint path or SDCS node sequence plus loop; vehicle IDs and local ports are unique process identities.

## 4. Position in the project

Platform scenario parsers call it before workers start; it does not spawn CARLA or drive a runtime.

## 5. Use and verification

`test/unit_test_setup_virtual.py` and `test/unit_test_setup_carla.py` exercise
shared scenario validation through their platform loaders. Multi-process virtual
and CARLA tests cover resulting process specifications and fleet routes.
