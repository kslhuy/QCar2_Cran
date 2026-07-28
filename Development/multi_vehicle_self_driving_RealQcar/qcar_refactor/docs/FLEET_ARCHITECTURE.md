# Fleet Utilities Architecture

## Purpose

`utils/fleet/` provides the vehicle-local fleet subsystem. Each vehicle process
owns one `FleetManager`; managers coordinate through `utils/v2v/` but do not
share Python objects, simulator state, or actuator access.

The fleet subsystem turns validated peer state into an optional controller
reference. It never creates a second control or IO path.

## Runtime Architecture

```text
                         one vehicle process

 VehicleRuntime
      |
      +--> ego observer --------------------------> VehicleStateEstimate
      |                                                     |
      +--> V2V adapter <---- UDP messages -----------------+
      |       generic transport                              |
      |                                                     v
      +---------------------------------------------- FleetManager
                                                     |
               +-------------------------------------+------------------------------------+
               |                                     |                                    |
               v                                     v                                    v
    FleetStateMachine                         FleetPeerStore                  DistributedObserver
    lifecycle and FleetStatus                 validate/cache/expire            advisory collective estimate
                                              peer estimates
                                                       |
                                                       v
                                             FleetFollowingPolicy
                                                       |
                                        follower only: ControllerReference
                                                       |
                                                       v
                                             ControllerManager selects
                                             a configured fleet controller
                                                       |
                                                       v
                                             ControlInput -> IO adapter
```

The V2V adapter remains generic: it owns sockets, routing, message envelopes,
and receive metadata. `FleetManager` owns the fleet message schema and accepts
only validated `FleetPeerSnapshot` values for fleet decisions.

## Per-Cycle Behavior

```text
IO read -> ego observer update -> FleetManager.run_cycle()
    -> drain and validate inbound peer estimates
    -> update fleet lifecycle and peer freshness
    -> publish this vehicle's self estimate at the configured fleet rate
    -> update the advisory distributed observer
    -> return FleetStepResult

FleetStepResult
    leader:   normal planner reference remains selected
    follower: valid predecessor becomes a ControllerReference
    failure:  safe-stop intent is returned to VehicleRuntime
```

`VehicleRuntime` remains the safety owner: it decides whether the vehicle is
allowed to drive and performs the only IO write. `FleetManager` does not call a
controller, write actuators, or change the vehicle state machine directly.

## Roles And Lifecycle

```text
DISABLED --build--> PREPARED --vehicle START--> BUILDING --fresh peers--> ACTIVE
    ^                   |                         |                         |
    +--- cancel/fault --+-------------------------+---- peer fault ---------+
```

- The leader publishes its local estimate and continues with its normal planner
  and configured controller.
- A follower publishes its own local estimate, waits for a fresh predecessor,
  then uses the fleet controller selected by `FleetManager` through
  `ControllerManager`.
- `PREPARED` is intentionally inactive: no timeout, peer cache, publication,
  or distributed-observer update begins before the vehicle reaches `RUNNING`.
- Lost or stale required peer data returns a fault result; the runtime applies
  its normal safe-stop behavior.

## Main Components

| Component | Responsibility |
|---|---|
| `fleet_types.py` | Fleet contracts: member, formation, policy, status, peer snapshot, and step result. |
| `fleet_registry.py` | Revisioned formation membership; supports future dynamic membership. |
| `fleet_state_machine.py` | Per-vehicle fleet lifecycle independent of vehicle safety states. |
| `fleet_peer_store.py` and `fleet_message.py` | Decode, validate, cache, and expire fleet peer estimates. |
| `fleet_following_policy.py` | Converts a valid predecessor snapshot into a controller reference. |
| `fleet_utils/distributed_observer/` | Advisory collective-estimate algorithms and their factory. |
| `fleet_manager.py` | The sole runtime-facing fleet interface. |
| `fleet_runtime.py` | Builds the local manager from a scenario-derived `FleetRuntimeSpec`. |

## Composition Boundary

```text
scenario fleet configuration
    -> FleetRuntimeSpec
    -> VehicleProcessSpec
    -> core.vehicle_process builds FleetManager
    -> VehicleRuntime receives FleetManager as one injected utility
```

Simulators only parse and forward the specification. The same process contract
can be used by CARLA, virtual vehicles, or a physical vehicle launcher.
