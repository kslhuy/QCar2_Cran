# `utils/fleet/fleet_types.py`

## 1. Introduction

This file defines fleet configuration, membership, status, peer, publication,
distributed-estimate, and runtime-decision contracts. They are frozen records:
formation/fleet utilities read or replace them, but none transmits or actuates.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `FleetError(ValueError)` | Invalid fleet value | Fleet exception | Common validation error for fleet contracts. |
| `FleetRole` / `FleetTopology` / `EdgeDirection` / `FollowingPolicy` | No runtime inputs | Closed enum values | Define member role, communication topology/direction, and following strategy. |
| `DistributedObserverConfig(implementation, parameters)` | Observer implementation and parameter mapping | Frozen config or `FleetError` | Freezes fake/Luenberger advisory selection; `from_mapping(data)` parses and validates gains. |
| `FleetCommunication(topology, direction, publish_rate_hz, peer_timeout_s)` | Topology/direction and positive transport timing | Frozen communication policy | `from_mapping(data)` parses/validates common V2V policy. |
| `FleetPolicy(following_policy, communication, follower_controller_profile, distributed_observer)` | Following, communication, controller profile, observer config | Frozen policy | `from_mapping(data)` parses all policy sections used by formation builder. |
| `FleetMember(vehicle_id, role, member_order)` | Non-negative ID, leader/follower role, order | Frozen membership item | `from_mapping(data)` validates one ordered member. |
| `FleetFormation(formation_id, membership_revision, members, policy)` | Identity, revision, ordered members, policy | Frozen formation snapshot | Owns ordered topology/policy; `leader` returns order-zero member. |
| `FleetFormation.member(vehicle_id)` / `predecessor(vehicle_id)` / `successors(vehicle_id)` | Member ID | Member, predecessor/tuple, or `None` | Perform ordered formation lookups. |
| `FleetFormation.outbound_peer_ids(vehicle_id)` / `inbound_peer_ids(vehicle_id)` | Local member ID | Peer-ID tuple | Filter topology edges into local V2V send/receive routes. |
| `FleetStatus(...)` | Formation/member/role/phase/peer-health/reason | Frozen lifecycle diagnostic | Records one vehicle-local view of formation state. |
| `FleetPeerSnapshot(...)` | Source metadata, estimate, sequence, source/local time, validity | Frozen remote state or `FleetError` | Represents one validated peer measurement with freshness metadata. |
| `FleetPublication(message_type, payload, target_vehicle_ids)` | V2V message type, payload, target IDs | Frozen publication request | Decouples fleet decision from generic V2V transport. |
| `DistributedEstimateSource(str, Enum)` | No runtime inputs | Measurement/local/V2V/distributed source | Identifies origin of an advisory state estimate. |
| `DistributedVehicleEstimate(vehicle_id, estimate, source)` / `DistributedFleetEstimate(observer_vehicle_id, membership_revision, estimates)` | Per-vehicle advisory data / collection | Frozen advisory estimates | `estimate_for(vehicle_id)` and `source_for(vehicle_id)` query collection entries. |
| `FleetStepResult(...)` | Status plus optional publication/reference/profile/fault/hold/advisory estimate | Frozen runtime fleet decision | Carries fleet output consumed by runtime without taking authority. |
| `FleetCommandResult(handled, accepted, stop_required, reason='')` | Fleet-command handling flags/reason | Frozen core-facing decision | Communicates accepted/rejected fleet command outcome. |
| `topology_edges(members, communication)` | Ordered members and communication policy | Directed `(source, target)` tuple | Produces valid topology routes for transport validation. |
| `_required_string(...)` / `_positive_number(...)` / `_validate_luenberger_parameters(...)` / `_valid_vehicle_id(...)` | Scalar/configuration values | Validated primitive or `FleetError` | Validate fleet configuration fields and observer gains. |

## 3. Special data and cross-references

`FleetFormation.members` is ordered leader then followers; `membership_revision`
invalidates all prior `FleetPeerSnapshot` data. Communication rate is Hz and
peer timeout is local-monotonic seconds. A peer snapshot preserves both remote
source timestamp/sequence and local receipt time. `FleetStepResult.target` is a
[[vehicle-types|ControllerReference]]; `hold_command` asks runtime to retain
safe zero while formation is building. `FleetStatus.peer_health` is
`(peer_id, fresh)` pairs.

## 4. Position in the project

[[fleet-manager|FleetManager]], peer store, state machine, formation builder,
distributed observers, and scenario parsers consume these contracts. They have
no V2V socket, controller, or actuator behavior.

## 5. Use and verification

Build records through their `from_mapping` constructors or a validated formation
builder, preserve immutability/revision semantics, and verify with
`test/unit_test_fleet.py`.
