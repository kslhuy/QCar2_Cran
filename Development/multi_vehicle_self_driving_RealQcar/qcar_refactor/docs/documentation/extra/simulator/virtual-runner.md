# `extra/simulator/virtual/process_runner.py`

## 1. Introduction

Runs one virtual scenario worker through the common process manager.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `VirtualProcessManager(BaseSimulatorProcessManager)` | No additional constructor inputs | Virtual manager instance | Adapts the common manager with `platform_name = 'virtual'`. |
| `load_setup(setup_file)` | Virtual setup path | `VirtualSetup` | Loads setup without starting a worker. |
| `build_process_spec(setup, vehicle)` | Parsed virtual setup and selected vehicle | `VehicleProcessSpec` | Converts one entry and optional fleet registry. |
| `run_virtual_vehicle(vehicle, cycles, dt_s, realtime=False)` | Vehicle setup, loop count, positive step, pacing flag | Telemetry list or `ValueError` | Direct deterministic helper with optional wall-clock pacing. |
| `main(argv=None) -> int` | Worker CLI argument list | Process status code and JSON telemetry | Waits for barrier, optionally builds fleet, runs worker, and prints rows. |
| Local `on_ready`, `on_step`, and `on_running` callbacks | Runtime/context/telemetry and barrier options | Ready file, retained rows, optional fleet command | Implement barrier, capture/pacing, and simulator-sourced `BUILD_FLEET`. |

## 3. Special data and cross-references

This integration module creates or coordinates [[vehicle-process|VehicleProcessSpec]] workers without bypassing [[vehicle-runtime|VehicleRuntime]].

Callbacks coordinate start and serialize telemetry without obtaining another actuator path.

## 4. Position in the project

Uses BaseSimulatorProcessManager and core vehicle process lifecycle.

## 5. Use and verification

`test/test_integration_multi_process_virtual.py`,
`test/test_integration_multi_process_virtual_control.py`, and
`test/test_integration_multi_process_virtual_fleet.py` verify worker/barrier,
telemetry, control, and fleet behavior.
