# `extra/simulator/carla/process_runner.py`

## 1. Introduction

Runs one CARLA vehicle worker through the common manager and its externally owned session.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `CarlaProcessManager(BaseSimulatorProcessManager)` | No additional constructor inputs | CARLA manager instance | Adapts common manager with `platform_name = 'CARLA'`. |
| `load_setup(setup_file)` | CARLA setup path | `CarlaSetup` | Loads setup without importing or starting a CARLA session. |
| `build_process_spec(setup, vehicle)` | Parsed CARLA setup and selected vehicle | `VehicleProcessSpec` | Converts one entry and optional fleet registry. |
| `main(argv=None) -> int` | Worker CLI argument list | Process status code and JSON telemetry | Runs one worker at configured CARLA fixed delta after the barrier. |
| Local `on_ready`, `on_step`, and `on_running` callbacks | Runtime/context/telemetry and barrier options | Ready file, retained rows, optional fleet command | Delay sole tick owner for followers, serialize rows, and optionally build fleet. |

## 3. Special data and cross-references

This integration module creates or coordinates [[vehicle-process|VehicleProcessSpec]] workers without bypassing [[vehicle-runtime|VehicleRuntime]].

Worker context holds one session/runtime pairing; session tick precedes IO cache refresh in the runtime loop.

## 4. Position in the project

Uses shared manager and CarlaSession while core retains vehicle safety.

## 5. Use and verification

`test/test_integration_multi_process_carla.py` and
`test/test_integration_multi_process_carla_fleet.py` verify coordinated workers,
tick ownership, telemetry, and fleet startup.
