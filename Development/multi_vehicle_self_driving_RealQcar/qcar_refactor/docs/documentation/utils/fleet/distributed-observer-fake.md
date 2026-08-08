# `utils/fleet/.../distributed_observer_fake.py`

## 1. Introduction

`DistributedObserverFake` provides deterministic pass-through distributed state for tests/development.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `DistributedObserverFake.update(local_estimate, peer_snapshots, membership_revision, measurements, dt)` | Local estimate, peer snapshots, revision; measurements/step ignored | `DistributedFleetEstimate` | Publishes the local estimate plus fresh peer measurements without correction; used for deterministic/test advisory behavior. |

## 3. Special data and cross-references

The selected `DistributedVehicleEstimate.source` records MEASUREMENT, LOCAL_OBSERVER, or V2V; `dt` is intentionally ignored.

## 4. Position in the project

Selected by fake profile; [[fleet-manager|FleetManager]] owns it and cannot use it to bypass safety.

## 5. Use and verification

`test/unit_test_fleet.py` verifies priority ordering and output.
