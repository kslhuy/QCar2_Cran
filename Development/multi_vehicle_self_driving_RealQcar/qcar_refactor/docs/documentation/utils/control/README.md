# Control Utilities

## 1. Introduction

`utils/control` supplies pluggable estimation, reference generation, and
control services to [[vehicle-runtime|VehicleRuntime]]. It owns algorithms,
not global lifecycle or final actuator authority.

## 2. File structure and variations

| File | Role | Variation or shared boundary |
| --- | --- | --- |
| [controller base](controller-base.md) | Controller interface and safe null controller | All controllers consume estimate/reference and return `ControlInput`; none reads sensors or writes IO. |
| [controller manual](controller-manual.md) | Fresh operator-input controller | Bounded latest axes expire to a safe zero command; planner completion is intentionally ignored. |
| [controller simple](controller-simple.md) | Normal path follower | Uses velocity PID-like throttle and target-heading steering. |
| [controller fleet base](controller-fleet-base.md) | Common follower gains/limits | Enables predecessor-derived references without owning formation policy. |
| [controller fleet 2D](controller-fleet-2d.md) | Planar predecessor follower | Tracks a virtual point behind the direct predecessor. |
| [controller fleet longitudinal](controller-fleet-longitudinal.md) | Straight-road follower | Computes speed from longitudinal gap and leaves steering zero. |
| [manager base](manager-base.md) | Configured/lazy profile selection | Reserves configured profile and validates any lazily built implementation. |
| [manager controller](manager-controller.md) | Controller selection/delegation | Resets on selection and gates fleet/manual-specific capabilities. |
| [manager observer](manager-observer.md) | Observer selection/delegation | Supplies one stable estimator lifecycle interface. |
| [manager path planner](manager-path-planner.md) | Planner selection/delegation | Gates SDCS-node routing on planner capability. |
| [observer base](observer-base.md) | Observer interface and safe null observer | All observers map `SensorData` and optional last command to an assessed estimate. |
| [observer EKF](observer-ekf.md) | Five-state kinematic EKF | Fuses tach, gyro, acceleration, optional GPS, and prior command. |
| [observer high gain](observer-high-gain.md) | Unsupported historical module | Byte-identical to EKF and defines `ObserverEKF`, not a high-gain class; factory does not expose it. |
| [observer Luenberger](observer-luenberger.md) | Unsupported historical module | Byte-identical to EKF and defines `ObserverEKF`, not a Luenberger class; factory does not expose it. |
| [planner base](path-planner-base.md) | Planner interface and safe null planner | All planners map estimate to `ControllerReference`, never to actuator output. |
| [planner static](path-planner-static.md) | Fixed waypoint planner | Uses monotonic closest/lookahead progress and final-distance completion. |
| [planner SDCS small map](path-planner-sdcs-small-map.md) | Directed 11-node map planner | Generates static route geometry with explicit `0`, `1`, `2`, and `inf` loop policies. |

## 3. Shared data and cross-references

Observers consume [[vehicle-types|SensorData]] and optional previous
`ControlInput`, then yield an assessed `VehicleStateEstimate`. Planners and
fleet services yield `ControllerReference`; controllers yield bounded
`ControlInput`. Manager profile names are selected only by core command/runtime
policy.

## 4. Position in the project

The runtime calls observer, planner/fleet, then controller in order. These
utilities do not read hardware, modify [[vehicle-state-machine|StateMachine]],
or bypass [[io-base|IOBase]] clipping.

## 5. Use and verification

Select profiles from configuration through [[module-factory|module_factory]].
Use `test/unit_test_controller.py`, `test/unit_test_vehicle_observer.py`, and
`test/unit_test_path_planner.py`. `test/test_integration_carla_sdcs_path.py`
requires a CARLA-capable integration environment; virtual/runtime tests cover
the non-CARLA control-loop path.

## Conclusion

All control implementations share typed estimate/reference/command contracts;
they differ only by algorithm or input source, while lifecycle and final safety
remain with [[vehicle-runtime|VehicleRuntime]].
