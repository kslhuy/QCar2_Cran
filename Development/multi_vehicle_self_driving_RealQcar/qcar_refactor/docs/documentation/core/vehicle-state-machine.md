# `core/vehicle_state_machine.py`

## 1. Introduction

`StateMachine` is the global permission-to-drive policy used by
[[vehicle-runtime|VehicleRuntime]]. The implementation contains no IO,
controller, observer, or transport code.

## 2. Code structure

`StateMachine()` starts in `INITIALIZING` with an empty error reason.

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `State` | enum member | lifecycle state | Names INITIALIZING, CALIBRATING, READY, RUNNING, STOPPED, and ERROR. |
| `StateMachine.__init__()` | none | initialized machine | Sets INITIALIZING and empty error reason. |
| `state` | none | `State` | Exposes current lifecycle state. |
| `calibrating()` | none | boolean | Reports whether the state is CALIBRATING. |
| `should_drive()` | none | boolean | Allows non-zero actuation only in RUNNING. |
| `safe_command_required()` | none | boolean | Requires zero output in every non-RUNNING state. |
| `mark_ready()` | none | none | Changes INITIALIZING to READY after successful startup; otherwise leaves state unchanged. |
| `mark_error(reason)` | failure reason string | none | Stores reason and changes state to ERROR. |
| `handle_command(cmd)` | lifecycle `VehicleCommand` | accepted boolean | Applies START, STOP, EMERGENCY_STOP, or RESET only from allowed source states. |
| `get_status()` | none | `{state, error_reason}` mapping | Exposes monitoring representation. |

## 3. Special data and cross-references

`_error_reason` is diagnostic state, not a control request. The implemented
emergency transition sets STOPPED (not a separate EMERGENCY_STOP enum value)
unless already ERROR. [[commands|VehicleCommand]] supplies transition intent;
`safe_command_required()` is the invariant used to create zero
[[vehicle-types|ControlInput]].

## 4. Position in the project

[[command-handler|VehicleCommandHandler]] checks transition eligibility;
[[vehicle-runtime|VehicleRuntime]] acts on the resulting drive permission.
This class never sends an actuator command itself.

## 5. Use and verification

Create one state machine per runtime; call `mark_ready()` only after modules
start and pass validated lifecycle commands to `handle_command()`. Verify all
allowed/denied transitions and error reset with `test/unit_test_state_machine.py`.
