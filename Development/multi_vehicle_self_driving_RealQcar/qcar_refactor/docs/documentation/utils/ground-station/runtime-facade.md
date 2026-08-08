# `utils/ground_station/runtime_facade.py`

## 1. Introduction

Provides the runtime-safe facade between a bridge's asynchronous queues and the
single vehicle control-loop thread.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- |
| `GroundStationRuntimeFacade(bridge, command_batch_size=8)` | Bridge and positive batch limit | Facade instance or `ValueError` | Stores the runtime-facing bridge and bounds commands per control step. |
| `last_command_result` | Retained facade state | Latest `CommandResult` or `None` | Exposes most recently handled local or external result. |
| `start()` / `stop()` | Lifecycle request | Bridge start/stop side effect | Delegate transport lifecycle without changing vehicle state. |
| `process_pending(apply_command)` | Runtime command callback | Bounded command application and acknowledgements | Drains one FIFO batch and invokes callback on control-loop thread. |
| `record_command_result(command, result)` | Applied command and result | Retained `CommandResult`, optional acknowledgement | Acknowledges only non-manual ground-station commands. |
| `publish_monitoring(*, vehicle_id, runtime_state, estimate, control_reference, fleet=None, v2v=None, control_mode='auto', manual_input_age_s=None, io_healthy=True)` | Public runtime/fleet/V2V/estimate/reference state | Offered `MonitoringSnapshot` | Builds validated operator snapshot and supplies it to bridge. |
| `get_status()` | Bridge diagnostics | Status mapping | Exposes health without transport internals. |

## 3. Special data and cross-references

The bounded batch controls work per runtime step. `publish_monitoring` copies
fleet role/health/peer age, V2V diagnostics, estimate, and controller reference
into [[monitoring|MonitoringSnapshot]]; a failed V2V status query degrades the
snapshot to unhealthy observation rather than failing the control loop.

## 4. Position in the project

[[vehicle-runtime|VehicleRuntime]] owns the supplied command-application
callback and calls this facade on its control-loop thread. The facade owns no
command policy: it cannot call an actuator, choose a controller, or bypass the
state machine. [[bridge-base|GroundStationBridgeBase]] remains asynchronous.

## 5. Use and verification

`test/unit_test_ground_station.py` covers bounded command pumping, manual
acknowledgement suppression, result retention, monitoring fields, and bridge
status. `test/test_integration_ground_station.py` covers the end-to-end facade
and bridge/server path.
