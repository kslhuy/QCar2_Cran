# `utils/fleet/fleet_manager.py`

## 1. Introduction

`FleetManager` coordinates one vehicle's formation phase, peer cache, publications, follower reference, and advisory distributed estimate without owning socket or actuator output.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `FleetManager(registry, vehicle_id, distributed_observer=None, logger=None)` | Registry, local ID, optional advisory observer/logger | Vehicle-local manager | Owns fleet lifecycle, peer cache, following policy, publication, and advisory observer for one process. |
| `phase` / `status()` | Manager/state-machine state | `FleetPhase` / `FleetStatus` | Expose lifecycle and peer-health diagnostic. |
| `attach_transport(transport)` | V2V transport with required publish/receive contract | Attached transport or `TypeError` | Connects generic transport without owning its lifecycle. |
| `request_build(vehicle_running, now_monotonic=None)` / `start_for_vehicle(now_monotonic=None)` | Core eligibility and optional monotonic time | Accepted `bool` | Request fleet build then initialize local transport/observer when vehicle starts. |
| `is_follower()` / `_start_build(now_monotonic)` | Formation/state / time | `bool` / build side effect | Identify local role and activate building dependencies. |
| `handle_command(command, vehicle_running, now_monotonic=None)` | Core fleet command, eligibility, time | `FleetCommandResult` | Handles only build/cancel fleet command semantics. |
| `stop_for_vehicle(reason='')` / `abort(reason)` | Shutdown/fault reason | Cancellation/fault side effect | Stop vehicle-local fleet operation or force fault. |
| `request_cancel(reason='')` / `fault(reason)` / `complete_cancellation()` | Lifecycle reason/state | Transition accepted `bool` | Delegate cancellation/fault transitions and cleanup. |
| `process_ego_estimate(estimate)` | Local estimate | Fault reason or `None` | Validates local estimate for fleet use and faults invalid active operation. |
| `step(estimate, messages, now_monotonic, dt, measurements=None)` | Local estimate, inbound messages, time, step, optional measurements | `FleetStepResult` | Ingests/prunes peers, checks health, builds following reference/publication, and updates advisory observer. |
| `run_cycle(estimate, dt, now_monotonic=None, measurements=None)` / `process_received(messages, now_monotonic)` | Local estimate/timing or inbound messages/time | `FleetStepResult` / fault reason | Obtain transport messages then process a full cycle, or handle received data alone. |
| `build_publication(estimate, now_monotonic=None)` | Local estimate and time | `FleetPublication` or `None` | Encodes one active fleet state publication for outbound peer routes. |
| `snapshots()` / `predecessor_snapshot()` | Peer cache | Snapshot tuple / predecessor or `None` | Expose accepted remote peer state. |
| `predecessor_age_s(...)` / `peer_ages_s(...)` / `predecessor_gap_m(ego_estimate)` | Time or local estimate | Age / ID-age map / gap or `None` | Produce diagnostics for monitoring and following. |
| `counters()` / `distributed_estimate()` / `shutdown()` | Manager cache/lifecycle | Counter map / advisory estimate / cleanup | Expose diagnostics or stop observer and clear transport state. |

## 3. Special data and cross-references

`FleetStepResult` carries status, optional publication/reference/controller profile, fault/hold flag, and advisory estimate. Build deadline is local monotonic timeout; follower build holds zero while active follower receives fleet-owned reference.

## 4. Position in the project

Injected into [[vehicle-runtime|VehicleRuntime]]; runtime performs transport startup and all final safety actions.

## 5. Use and verification

`test/unit_test_fleet.py` plus virtual/CARLA fleet integration tests cover lifecycle, timeout, publication, follower hold/reference, and abort.
