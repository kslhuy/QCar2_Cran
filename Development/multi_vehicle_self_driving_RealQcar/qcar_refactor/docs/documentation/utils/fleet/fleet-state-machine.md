# `utils/fleet/fleet_state_machine.py`

## 1. Introduction

`FleetStateMachine` owns one vehicle-local fleet phase without replacing global vehicle safety state.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `FleetPhase(str, Enum)` | No runtime inputs | Lifecycle phase value | Defines disabled, prepared, building, active, cancelling, and fault phases. |
| `FleetStateMachine(registry, vehicle_id)` | Formation registry and local ID | State machine or `FleetError` | Binds one valid local member to lifecycle policy. |
| `phase` / `status(peer_health=())` | Machine state and optional peer-health tuple | `FleetPhase` / `FleetStatus` | Expose current lifecycle and frozen local diagnostic. |
| `request_build(vehicle_running)` | Global vehicle-running eligibility | Accepted `bool` | Transitions disabled to prepared only while vehicle is running. |
| `begin_running()` / `activate()` | Current lifecycle state | Transition accepted `bool` | Progress prepared to building, then active. |
| `request_cancel(reason='')` / `complete_cancellation()` | Optional reason / cancelling state | Transition accepted `bool` | Enter cancelling and return to disabled when cleanup completes. |
| `fault(reason)` | Non-empty failure reason | Transition accepted `bool` or `ValueError` | Enters fault state with retained diagnostic reason. |

## 3. Special data and cross-references

PREPARED has no peer processing; BUILDING owns peer timeout; ACTIVE permits follower references. This local phase is constrained by [[vehicle-state-machine|StateMachine]] RUNNING state.

## 4. Position in the project

[[fleet-manager|FleetManager]] invokes transitions; runtime converts its results into global safe behavior.

## 5. Use and verification

`test/unit_test_fleet.py` covers all phase transitions, cancellation, and faults.
