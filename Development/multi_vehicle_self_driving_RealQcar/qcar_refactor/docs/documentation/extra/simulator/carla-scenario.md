# `extra/simulator/carla/scenario.py`

## 1. Introduction

Parses CARLA scenario and actor spawn/session configuration into validated setup records.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `CarlaSetupError(ValueError)` | Invalid CARLA scenario | Setup exception | Identifies platform-specific scenario validation failure. |
| `CarlaVehicleSetup(...)` | Spawn, route/node, tick owner, V2V, controller profile | Frozen vehicle setup | Holds one CARLA vehicle's scenario configuration. |
| `CarlaVehicleSetup.to_process_spec(simulation_profile, fleet_spec=None)` | Profile and optional fleet spec | `VehicleProcessSpec` | Adds session overrides and converts CARLA Y/yaw to observer frame. |
| `CarlaSetup(simulation_profile, simulation_config, vehicles, fleet=None)` | Profile/configuration, vehicles, optional fleet | Frozen scenario; `host`/`port` | Represents the shared CARLA simulation setup. |
| `parse_carla_setup(argv=None)` / `load_carla_setup(path)` | CLI arguments or YAML path | `CarlaSetup` or `CarlaSetupError` | Require CARLA implementation and parse vehicles/fleet. |
| `_parse_vehicle_scenario(data)` | One vehicle mapping | `CarlaVehicleSetup` or `CarlaSetupError` | Requires exactly one mission type and validates tick/V2V/profile data. |
| `_validate_scenario_vehicles(vehicles, carla_port)` | Parsed vehicles and CARLA port | `None` or `CarlaSetupError` | Requires one tick owner and prevents ID/V2V/CARLA port collisions. |
| `_parse_spawn_transform(data)` | Spawn mapping | Numeric CARLA transform or `CarlaSetupError` | Requires `x`, `y`, `z`, and `yaw`. |

## 3. Special data and cross-references

This integration module creates or coordinates [[vehicle-process|VehicleProcessSpec]] workers without bypassing [[vehicle-runtime|VehicleRuntime]].

Spawn transform is CARLA-native; IOCarla mirrors Y/yaw only when publishing project-frame SensorData. CARLA port and vehicle IDs are unique resources.

## 4. Position in the project

Consumed by CARLA runner/session; it does not start a world itself.

## 5. Use and verification

`test/unit_test_setup_carla.py` covers profile, mission, spawn, tick-owner,
port, V2V, and fleet validation. CARLA multi-process tests verify the generated
worker setup.
