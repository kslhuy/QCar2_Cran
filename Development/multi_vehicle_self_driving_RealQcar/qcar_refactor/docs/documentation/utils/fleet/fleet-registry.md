# `utils/fleet/fleet_registry.py`

## 1. Introduction

`FleetRegistry` owns lock-protected membership updates and exposes immutable formation revisions.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `FleetRegistry(formation)` | Validated immutable formation | Thread-safe registry | Retains formation snapshot and revisioned membership updates. |
| `snapshot()` | Registry state | Current `FleetFormation` | Returns immutable formation snapshot under lock. |
| `join(member)` / `leave(vehicle_id)` / `update_member(member)` | Membership addition/removal/replacement | Revision-incremented `FleetFormation` | Apply one validated membership change. |
| `_replace(members)` | Replacement member tuple | Revision-incremented `FleetFormation` | Builds/validates next snapshot through formation builder. |

## 3. Special data and cross-references

Membership revision invalidates stale peer data. Formation is immutable; mutation occurs only by building/replacing a new snapshot.

## 4. Position in the project

[[fleet-manager|FleetManager]] owns the registry. It creates neither V2V messages nor controller references.

## 5. Use and verification

`test/unit_test_fleet.py` verifies joins/leaves/updates and revision increments.
