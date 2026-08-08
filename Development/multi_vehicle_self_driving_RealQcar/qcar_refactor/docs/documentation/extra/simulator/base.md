# `extra/simulator/base.py`

## 1. Introduction

Defines the shared scenario process-manager protocol and coordinated worker lifecycle.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `SimulatorSetup.vehicles` | Scenario-owned setup | Vehicle tuple | Protocol property required by every process manager. |
| `SimulatorProcessContext(setup, vehicle, spec)` | Parsed setup, selected vehicle, and `VehicleProcessSpec` | Frozen context record | Carries resolved inputs for exactly one worker. |
| `BaseSimulatorProcessManager(setup_file, vehicle_id)` | Scenario path and selected vehicle ID | Manager instance | Stores common worker selection. |
| `load_setup(setup_file)` / `build_process_spec(setup, vehicle)` | Backend setup path or selected entry | `SimulatorSetup` / `VehicleProcessSpec` | Abstract backend hooks for parsing and conversion. |
| `prepare()` | Stored setup path and ID | `SimulatorProcessContext` or `ValueError` | Selects the requested vehicle and builds its common spec. |
| `build_runtime(context)` | Prepared context | Core runtime | Builds runtime from `context.spec`. |
| `run(cycles, dt, on_ready=None, on_running=None, on_step=None, collect_telemetry=True, should_stop=None)` | Cycle/timing control and optional hooks | Context and telemetry list | Owns common runtime execution and invokes hooks only at lifecycle boundaries. |
| `wait_for_start_signal(path, timeout_s=30.0)` | Barrier file and timeout | Start release or `TimeoutError` | Waits for parent launcher to release an independent worker. |
| `serialize_telemetry(telemetry, fleet=None)` | Telemetry record and optional fleet | JSON-safe mapping | Builds observation row and adds fleet/V2V fields when applicable. |

## 3. Special data and cross-references

This integration module creates or coordinates [[vehicle-process|VehicleProcessSpec]] workers without bypassing [[vehicle-runtime|VehicleRuntime]].

Process context carries scenario, one selected vehicle, runtime spec, and synchronization artifacts; serialized telemetry is observation-only.

## 4. Position in the project

Virtual and CARLA runners inherit it; it calls core process helpers without accessing controllers/IO directly.

## 5. Use and verification

`test/test_integration_multi_process_virtual.py` and
`test/test_integration_multi_process_carla.py` verify worker barriers and
lifecycle; their fleet variants verify the optional fleet telemetry fields.
