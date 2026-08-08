# `extra/simulator/launcher.py`

## 1. Introduction

Selects a virtual or CARLA process manager from CLI arguments and runs one scenario.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `main(argv=None) -> int` | CLI argument list | Aggregate process status code | Launches every worker, releases barrier, and waits for completion. |
| `_vehicle_ids(platform, setup_file)` | Backend name and scenario path | Vehicle-ID tuple | Loads setup only to enumerate workers. |
| `_start_worker(command)` | One module command | `subprocess.Popen` worker | Starts a worker in a separate process/session group. |
| `_worker_commands(args, setup_file, vehicle_ids)` | Parsed CLI, setup path, IDs | Barrier-aware command-builder callable | Creates one runner command per vehicle. |
| `_InterruptFlag()` / `_handle(...)` / `restore()` | SIGINT lifecycle | Pollable flag / restored handler | Turns Ctrl-C into non-blocking launcher state. |
| `_wait_until_ready(paths, processes, timeout_s, interrupted)` | Ready files, workers, timeout, flag | Boolean or startup error | Waits for workers or surfaces early exit/timeout. |
| `_wait_for_workers(processes, interrupted)` | Workers and interrupt flag | Aggregate status code | Collects worker results and prints summaries/failures. |
| `_process_output(process)` / `_print_completion_summary(output)` | Completed process or captured output | Output text / printed summary | Consume output and emit final telemetry summary. |
| `_stop_remaining(processes, stop_file)` | Active workers and stop path | Cooperative/forced shutdown | Signals stop, then terminates/kills lingering workers. |

## 3. Special data and cross-references

This integration module creates or coordinates [[vehicle-process|VehicleProcessSpec]] workers without bypassing [[vehicle-runtime|VehicleRuntime]].

CLI data identifies a platform-wide setup, worker limit/timing, and optional
fleet-build request. Temporary ready/start/stop files are process-coordination
artifacts; no simulator session or actuator is hidden in this launcher.

## 4. Position in the project

Top-level scenario entry point; backend-specific work remains in virtual/CARLA modules.

## 5. Use and verification

`test/test_integration_multi_process_virtual.py` and
`test/test_integration_multi_process_carla.py` exercise launched workers,
barriers, output summaries, and coordinated shutdown.
