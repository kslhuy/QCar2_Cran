# distributed observer base and factory

## 1. Introduction

Defines the advisory distributed-observer lifecycle and profile construction.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `DistributedObserverBase(config, vehicle_id, logger=None)` | Observer parameter mapping, local ID, optional logger | Base observer state | Stores copied configuration and local identity. |
| `start()` | No inputs | Lifecycle side effect | Abstract advisory-observer startup hook. |
| `update(local_estimate, peer_snapshots, membership_revision, measurements, dt)` | Local/peer estimates, formation revision, optional measurements, step | `DistributedFleetEstimate` or `None` | Abstract per-cycle advisory estimate contract. |
| `get_latest()` | Observer state | Latest distributed estimate or `None` | Returns cached result without recomputation. |
| `stop()` | No inputs | Lifecycle side effect | Abstract shutdown hook. |

## 3. Special data and cross-references

`DistributedFleetEstimate` is advisory only and preserves selected source per vehicle; it must not bypass peer freshness or controller validation.

## 4. Position in the project

[[fleet-runtime|fleet_runtime]] constructs it and [[fleet-manager|FleetManager]] owns lifecycle; it does not publish or control.

## 5. Use and verification

`test/unit_test_fleet.py` verifies factory and lifecycle.
