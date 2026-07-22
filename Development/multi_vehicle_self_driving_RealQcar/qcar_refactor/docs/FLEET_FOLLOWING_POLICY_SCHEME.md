# Fleet Following Policy Scheme

This document defines the boundary between the fleet utility and the vehicle-control utility. It is a design scheme, not an implemented following algorithm.

## Goal

Allow a follower to maintain a longitudinal spacing objective relative to its direct predecessor without giving fleet code access to actuator commands, IO, UDP sockets, or simulator APIs.

The selected controller remains responsible for mapping its reference to throttle and steering. In fleet mode, that controller is a `ControllerFleetBase` implementation.

```text
ego observer + optional local path                  generic V2V
        |                                                |
        v                                                v
ControllerReference                         FleetPeerSnapshot
        |                                                |
        +---------------- FleetFollowingPolicy ----------+
                                |
                                v
                 ControllerFleetBase.compute(estimate, reference, dt)
                                |
                                v
        ControllerFleetLongitudinal or ControllerFleet2D
                                |
                                v
                         ControlInput -> IO
```

## Ownership

| Component | Owns | Must not own |
|---|---|---|
| `utils/v2v/` | Generic message envelope, UDP routing, receive metadata | Fleet state schema or actuator control |
| `utils/fleet/FleetPeerSnapshot` | Validated remote vehicle estimate and local freshness metadata | UDP sockets, IO, or fleet lifecycle ownership |
| `utils/fleet/FleetFollowingPolicy` | Fleet eligibility, predecessor selection, input validation, and front-reference composition | Following algorithm internals, actuator commands |
| `utils/control/controller/controller_fleet/ControllerFleetBase` | General fleet-controller contract; accepts the normal reference type and advertises fleet-reference support | V2V messages, fleet registry mutation, path parsing, IO |
| `utils/control/controller/controller_fleet/ControllerFleetLongitudinal` | First longitudinal following law | Fleet membership mutation, V2V messages, path parsing, IO |
| `utils/control/controller/controller_fleet/ControllerFleet2D` | 2D following law: virtual target, throttle, and steering | Fleet membership mutation, V2V messages, path parsing, IO |
| `utils/control/ControllerBase` | Normal-controller contract and parent class for fleet controllers | Fleet topology or network access |
| `core/vehicle_logic.py` | Ordered composition and existing safe-stop path | A concrete following-law implementation |

## Separate Estimate And Fleet-State Models

Do not use a single type called `FleetState`: it can ambiguously mean an individual vehicle estimate, operational fleet status, or a collective observer estimate. These are separate models.

| Model | Owner | Meaning | V2V representation |
|---|---|---|---|
| `VehicleStateEstimate` | `utils/control/observer` | One vehicle's locally estimated pose, velocity, and other control state | `VEHICLE_STATE_ESTIMATE` when this vehicle publishes its own estimate |
| `FleetPeerSnapshot` | `utils/fleet/` | Latest validated remote `VehicleStateEstimate` plus sender, sequence, local receive time, and freshness | Built after decoding `VEHICLE_STATE_ESTIMATE` |
| `FleetStatus` | `utils/fleet/` | Formation ID, membership revision, `source_vehicle_id`, member order/role, `FleetPhase`, and peer-health summary | Local by default; add a separate status message only when another feature requires it |
| `DistributedFleetEstimate` | `utils/fleet/` distributed observer | One vehicle's local, advisory estimate of collective fleet state | Local only; it is not a V2V message in the baseline |

The initial follower policy consumes the local `VehicleStateEstimate` and a predecessor `FleetPeerSnapshot`. It does not consume `FleetStatus` as a measurement. `DistributedObserverFake` already consumes direct measurements, local observer output, and peer snapshots to produce a `DistributedFleetEstimate` without calculation. `DistributedObserverLuenberger` is an advisory prototype with bounded prediction/correction gains. Neither output is automatically fused with the self estimate or used for fleet control.

Fleet message type names describe semantics, not versions. Every encoded fleet payload carries an explicit `schema_version`, which decoders validate before accepting its fields.

## Fleet Communication Profile

Fleet membership and order belong to the multi-vehicle scenario. Reusable fleet communication policy belongs in `config_fleet.yaml`; it applies only to fleet-owned messages and does not replace the V2V transport profile or scenario endpoint map.

```yaml
fleet_default:
  communication:
    topology: leader_follower  # leader-centred star
    edge_direction: bidirectional
    ego_estimate_rate_hz: 20
    peer_timeout_s: 0.50
```

The initial topology values are:

| Value | Graph | Intended use |
|---|---|---|
| `leader_follower` | Leader connected to every follower | Leader-reference policy, leader broadcasts, or supervisory messages |
| `predecessor_chain` | Each vehicle sends only to its direct successor | Direct-predecessor following without requiring leader feedback from the tail |
| `loop` | `leader -> follower_1 -> ... -> follower_last -> leader` | Distributed algorithms that require a connected ring; direction must be explicit |
| `vehicle_vehicle` | Every vehicle connected to every other vehicle | Small-fleet research, diagnostics, and algorithms requiring all peer estimates |

`edge_direction` must state whether each listed graph edge is one-way or both-way. UDP send routes are still resolved from the scenario's V2V endpoint map.

For the initial direct-predecessor following policy, each follower must receive its direct predecessor's estimate. A pure leader-centred star does not provide `follower_2` with `follower_1` data, so it is invalid for that policy unless the required predecessor links are added. The formation order and communication topology are related validation inputs, but they are not the same thing.

`ego_estimate_rate_hz` is the fleet application-message rate. The generic V2V adapter can impose an equal or higher transport rate limit, but it must not silently publish fleet estimates faster than the configured fleet rate.

## Proposed Fleet-Controller Interfaces

`ControllerFleetBase` keeps the existing controller-reference type and the same `ControllerBase.compute(estimate, reference, dt)` method. In fleet mode, that reference represents the direct front vehicle's current reference state rather than a path-planner waypoint. `FleetManager` validates fleet status before returning the reference; controllers do not receive fleet lifecycle state or V2V data.

```python
class ControllerFleetBase(ControllerBase):
    @property
    def supports_fleet_reference(self) -> bool:
        return True

    def compute(
        self,
        ego_estimate: VehicleStateEstimate,
        front_reference: ControllerReference,
        dt_s: float,
    ) -> ControlInput:
        """Return one safe command from a front-vehicle reference."""


class ControllerFleetLongitudinal(ControllerFleetBase):
    def compute(...):
        """Implement one bounded longitudinal following law."""


class ControllerFleet2D(ControllerFleetBase):
    def compute(...):
        """Track a virtual target behind the front vehicle with throttle and steering."""
```

`FleetFollowingPolicy` converts a valid predecessor `FleetPeerSnapshot` into a `ControllerReference` with the same data shape used elsewhere. It returns that reference only while fleet phase and predecessor freshness are valid; `VehicleRuntime` chooses whether its injected controller can consume it.

```python
def make_front_reference(
    predecessor: FleetPeerSnapshot,
) -> ControllerReference:
    # Validate phase, predecessor role, sequence, and freshness first.
    return ControllerReference(
        target_x=predecessor.estimate.x,
        target_y=predecessor.estimate.y,
        target_theta=predecessor.estimate.theta,
        target_velocity=predecessor.estimate.velocity,
        is_finished=False,
    )
```

The policy returns an explicit failure rather than a reference when the fleet phase is not active, the predecessor is stale, or geometry is invalid. `VehicleRuntime` then invokes its existing zero-command and safe-stop path. `ControllerFleetLongitudinal` uses the front position only to compute longitudinal gap. `ControllerFleet2D` derives a virtual target at the required offset behind the front vehicle, rather than steering directly towards the front vehicle centre.

## Initial Operating Rules

1. The leader passes its path-planner `ControllerReference` directly to its normal `ControllerBase` implementation.
2. A follower receives a `ControllerReference` representing the direct front vehicle. `ControllerFleetLongitudinal` uses the front position only to calculate longitudinal gap; it does not steer towards that position.
3. `ControllerFleetLongitudinal` is limited to longitudinal/straight-road evaluation. `ControllerFleet2D` is the separate 2D implementation that computes both throttle and steering from a virtual target behind the front vehicle; it requires independent geometry and closed-loop validation.
4. The selected normal or fleet controller is the sole producer of `ControlInput`; `IOBase` remains the sole actuator adapter.
5. Fleet control runs in the normal `VehicleRuntime` loop. Add a local controller rate limiter only when a selected algorithm requires a slower update rate; do not create a second control thread.
6. A stale, missing, malformed, out-of-order, or obsolete-membership predecessor snapshot cannot yield a command. It causes the configured fleet cancellation/fault path.

## Algorithm Selection Boundary

The first algorithm can be selected without changing fleet transport or path tracking. Suitable candidates to research are:

| Candidate | Inputs needed | Useful first evaluation |
|---|---|---|
| Constant-spacing PD | Gap and relative speed | Simple baseline, but can amplify disturbances |
| Constant-time-headway PD | Gap, ego speed, predecessor speed | Practical baseline with a speed-dependent desired gap |
| ACC/CACC | Above, optionally communicated predecessor acceleration | Requires clear acceleration validity and delay handling |
| MPC | A state model, constraints, and prediction horizon | Later benchmark after the baseline is stable |

The future distributed Luenberger observer is an estimator, not a following control law. It may later improve the estimate used by either fleet controller, but it must be evaluated separately before it influences follower control.

## Required Tests Before CARLA

1. Unit tests for every fleet-controller algorithm: bounds, invalid input, zero/negative gap, `dt` behavior, and 2D virtual-target geometry.
2. Policy/controller tests: leader pass-through, predecessor-to-reference mapping, stale-peer rejection, optional-status handling, and no fleet-registry mutation during command computation.
3. Two-process virtual tests: nominal spacing, speed changes by the leader, delayed/dropped V2V packets, cancellation, and clean shutdown.
4. Only after these gates pass, repeat the same fleet policy and algorithm in CARLA.
