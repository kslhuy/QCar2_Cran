# Fleet Utilities

## 1. Introduction

`utils/fleet` implements one vehicle-local view of formation lifecycle,
membership, peer freshness, following references, and optional distributed
estimates. It advises the runtime; it never owns global drive permission.

## 2. File structure and variations

| File | Role | Variation or shared boundary |
| --- | --- | --- |
| [fleet types](fleet-types.md) | Immutable formation, policy, message, and decision contracts | Provides data only; does not perform lifecycle or transport work. |
| [fleet base](fleet-base.md) | Formation builder and route validation | Converts policy/members into a transport-valid immutable formation. |
| [fleet registry](fleet-registry.md) | Revisioned formation membership store | Serializes joins/leaves/replacements into immutable snapshots. |
| [fleet state machine](fleet-state-machine.md) | Local fleet lifecycle policy | Governs disabled/prepared/building/active/cancelling/fault transitions. |
| [fleet message](fleet-message.md) | Fleet state codec | Encodes/decodes typed fleet payloads inside generic V2V envelopes. |
| [fleet peer store](fleet-peer-store.md) | Fresh inbound-peer cache | Enforces formation revision, sequence, expected-peer, and timeout rules. |
| [fleet following policy](fleet-following-policy.md) | Follower-reference construction | Returns fleet-owned predecessor reference only when active and healthy. |
| [fleet manager](fleet-manager.md) | Vehicle-local coordination | Combines lifecycle, transport, peer freshness, publication, following, and advisory estimation. |
| [fleet runtime](fleet-runtime.md) | Process-spec construction | Builds one local manager from the scenario-owned registry. |
| [distributed observer base](distributed-observer-base.md) | Advisory-estimation interface | All variants produce `DistributedFleetEstimate`, never an actuator command. |
| [distributed observer fake](distributed-observer-fake.md) | Deterministic advisory variant | Publishes measured local/fresh peer state without correction. |
| [distributed observer Luenberger](distributed-observer-luenberger.md) | Corrective advisory variant | Applies configured component gains to previous/measurement estimate. |
| [distributed observer factory](distributed-observer-factory.md) | Advisory implementation selection | Builds fake/Luenberger variant from policy without starting scenario execution. |

## 3. Shared data and cross-references

[[fleet-types|FleetFormation]] describes members, topology, policy, and
membership revision. Peer cache preserves `FleetPeerSnapshot` source/local time,
sequence, and validity; following produces a `ControllerReference`; distributed
observers produce advisory `DistributedFleetEstimate`. [[v2v-base|V2VBase]]
transports generic envelopes.

## 4. Position in the project

[[vehicle-runtime|VehicleRuntime]] calls its injected `FleetManager` during
the control loop. Followers receive a fleet-owned reference; command eligibility
and final safe output remain with core.

## 5. Use and verification

Build through [[vehicle-process|vehicle_process]] from `FleetRuntimeSpec`.
Use `test/unit_test_fleet.py`, `test/test_integration_multi_process_virtual_fleet.py`,
and `test/test_integration_multi_process_carla_fleet.py`; CARLA coverage
requires its external simulator environment.

## Conclusion

All fleet implementations share formation and freshness contracts; they differ
only in local policy/estimation detail, while [[vehicle-runtime|VehicleRuntime]]
retains lifecycle and safety authority.
