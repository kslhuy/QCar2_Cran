# `utils/fleet/fleet_utils/distributed_observer/factory.py`

## 1. Introduction

Builds the process-local advisory distributed observer selected by fleet policy.
Its caller is [[fleet-manager|FleetManager]]; it selects an estimator only and
does not run a scenario, modify formation membership, or command a vehicle.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `build_distributed_observer(config, vehicle_id, logger=None)` | Validated `DistributedObserverConfig`, local vehicle ID, optional logger | `DistributedObserverFake`, `DistributedObserverLuenberger`, or `FleetError` | Selects the supported fake or Luenberger advisory implementation from policy without coupling it to runtime execution. |

## 3. Special data and cross-references

[[fleet-types|DistributedObserverConfig]] supplies the implementation name
and frozen parameter mapping. The returned observer produces an advisory
`DistributedFleetEstimate`; it is not a second localization or control path.

## 4. Position in the project

[[fleet-manager|FleetManager]] owns the returned observer lifecycle and calls
it inside fleet processing. [[vehicle-runtime|VehicleRuntime]] remains the
owner of local observation, command selection, and final safe output.

## 5. Use and verification

Fleet policy selects `fake` or `luenberger` under `distributed_observer`.
`test/unit_test_fleet.py` covers both selections and unsupported implementation
rejection; multi-process fleet tests require valid V2V routes and timing.
