# Utility Modules

## 1. Introduction

Utilities are injected backend or algorithm services used by
[[vehicle-runtime|VehicleRuntime]]. They never own global lifecycle changes
or create an alternate final actuation path.

## 2. File structure and variations

| Package | Base/template | Backend or algorithm variations |
| --- | --- | --- |
| [Control](control/README.md) | manager/interface contracts | observers, planners, manual/path/fleet controllers. |
| [Fleet](fleet/README.md) | formation/lifecycle contracts | following, peer cache, distributed estimators. |
| [Ground station](ground-station/README.md) | framed bridge contract | null/TCP bridge and monitoring facade. |
| [IO](io/README.md) | `IOBase` | null, virtual, CARLA, QCar, future ROS. |
| [V2V](v2v/README.md) | `V2VBase` | null and UDP transport. |

## 3. Shared data and cross-references

Utilities exchange [[vehicle-types|SensorData]], `VehicleStateEstimate`,
`ControllerReference`, `ControlInput`, `V2VMessage`, and typed commands through
the core-defined contracts.

## 4. Position in the project

[[module-factory|module_factory]] constructs utilities; core owns their
ordering, selection policy, lifecycle, and safe-zero decision.

## 5. Use and verification

Select current profiles through config and run the focused package tests listed
in each README. Test-only configurations remain under `config/scenarios/test/`.

## Conclusion

All utilities share injected-interface boundaries; each differs only in its
bounded service while core retains global vehicle authority.
