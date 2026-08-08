# `utils/fleet/fleet_base.py`

## 1. Introduction

`FleetFormationBuilder` constructs validated immutable formation snapshots from scenario membership.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `FleetFormationBuilder.build(formation_id, members, policy, membership_revision=0, available_vehicle_ids=None)` | Formation ID, ordered members, policy, revision, optional scenario IDs | Validated `FleetFormation` or `FleetError` | Validates membership/policy routes and creates immutable formation snapshot. |
| `FleetFormationBuilder.validate_transport_routes(formation, peer_ids_by_vehicle)` | Formation and configured V2V peer IDs | Validation side effect or `FleetError` | Requires each topology route to be available in vehicle transport configuration. |
| `_validate_members(members, available_vehicle_ids)` | Ordered members and optional available IDs | Validation side effect or `FleetError` | Enforces leader/order/ID uniqueness and scenario membership consistency. |
| `_validate_policy_routes(formation)` | Formation topology/policy | Validation side effect or `FleetError` | Rejects topology/direction combinations without valid directed routes. |

## 3. Special data and cross-references

Formation members identify vehicle/role/order; policy topology turns them into required physical communication routes. See [[fleet-types|Fleet types]].

## 4. Position in the project

Scenario/fleet runtime construction calls this builder; it does not run every control cycle or own transport.

## 5. Use and verification

`test/unit_test_fleet.py` verifies membership/topology and missing V2V route rejection.
