# CARLA Simulation And IO Reference

Implementation status and task ordering are centralized in `docs/REFACTOR_MASTER_TODO.md`, Step 5. This document defines the CARLA design constraints used by that checklist.

Reference points:

- CARLA session integration: `extra/simulation/carla_session.py`
- Vehicle adapter: `utils/io/io_carla.py`
- Common IO contract: `utils/io/io_base.py`
- QCar vehicle adapter: `utils/io/io_qcar2.py`
- Shared data: `core/types.py`
- CARLA examples: `refs/carla_test/examples/client_bounding_boxes.py` and `refs/carla_test/examples/generate_traffic.py`

## Ownership Boundary

| Component | Owns | Must not own |
|---|---|---|
| `CarlaSession` in `extra/simulation/carla/session.py` | Client connection, map/world settings, ego/sensor actors, callbacks, synchronous tick, and cleanup | Observer, planner, controller, state-machine decisions, or command clipping |
| `IOCarla` in `utils/io/io_carla.py` | Reading the current session snapshot and applying a `ControlCommand` to an existing ego actor | Client creation, world ticking, actor spawn/destruction, or world-setting restoration |
| Runtime | Session and IO construction, loop ordering, safety state, and final shutdown | CARLA API details inside controller/planner/observer code |

`CarlaSession` is the only owner of CARLA actors and original world settings. `IOCarla` is only a vehicle adapter.

## Standard Script Method

Project scripts must not create a CARLA client, change world settings, spawn actors, tick a world, or import CARLA at module import time. Use the standard composition path instead:

```python
config = load_config(vehicle_config_file="config_vehicle_carla.yaml")
modules = build_vehicle_modules(config)
runtime = VehicleRuntime(
    config, modules.io, modules.observer, modules.planner,
    modules.controller, modules.v2v, simulation=modules.simulation,
)
```

The script starts, steps, and shuts down `runtime` in `try`/`finally`. `CarlaSession` owns all CARLA operations; `IOCarla` only reads the session-owned actor/snapshot and applies controls.

## QLabs Boundary

Use the same `extra` boundary if QLabs becomes available again:

```text
extra/simulation/qlabs_session.py  owns QLabs startup and lifecycle
utils/io/io_qcar2.py               adapts an already-created QCar vehicle
```

The QLabs license is currently expired. Keep its code as a reference only and do not schedule live QLabs validation in the current milestone.

## File Layout

```text
utils/
    io/
        io_base.py
        io_carla.py
extra/
    simulation/
        carla_session.py

test/
    unit_test_carla_session.py
    unit_test_io_carla.py
    test_integration_carla.py
```

Keep `CarlaSensorSnapshot` as a small dataclass inside `carla_session.py` until it is shared by more than one simulation module. Do not add `carla_types.py` for one or two dataclasses.

## Configuration Ownership

| File | Owns |
|---|---|
| `config/config_vehicle.yaml` | Car ID and selected vehicle-owned module profiles |
| `config/config_io.yaml` | CARLA vehicle adapter read/write limits and rates |
| `config/config_simulation.yaml` | CARLA host, port, timeout, world, ego, sensors, and startup/warmup settings; `spawn_transform` is used for custom worlds without parsed map spawn points |

Example selection:

```yaml
vehicle:
  car_id: 0

modules:
  io: carla
  simulation: carla_sync
```

The config loader selects these module sections and passes only `config.module("io")` to `IOCarla` and only `config.module("simulation")` to `CarlaSession`. Although it is selected in a vehicle config, simulation is a platform integration rather than a reusable `utils` module.

## Synchronous Runtime Sequence

The MVP supports synchronous CARLA only. One runtime owns ticking.

```text
apply previous command
    -> CarlaSession.tick()
    -> session updates the latest frame snapshot
    -> IOCarla.read_to_cache()
    -> observer, planner, controller
    -> runtime stores next command
```

`IOCarla.read_to_cache()` overrides the normal `IOBase` rate check and refreshes once after each completed CARLA tick. It retains CARLA simulation timestamps in `SensorData`; no shared `IOBase` timing change is required for the MVP.

## Data Mapping

Use SI-facing defaults at the refactored module boundary. These mappings are working approximations, not a claim that QCar and CARLA sensors have identical physics.

| Shared field | CARLA source or mapping |
|---|---|
| `motor_tach` | Signed longitudinal velocity in m/s: velocity dot ego forward vector |
| `gyro_z` | IMU yaw rate converted to the project frame, in rad/s |
| `accelerometer` | IMU acceleration `[ax, ay, az]` in m/s^2, with frame conversion documented |
| `gps_position` | Ego transform converted to local `[x_m, y_m, yaw_rad]` for the MVP |
| timestamps | CARLA frame/simulation elapsed time |
| steering | Common radians converted to CARLA normalized `[-1, 1]`; with the current `project_y = -carla_y` conversion, project-positive steering uses a negative CARLA steering sign |
| negative throttle | Brake for the MVP; reverse is deferred |

Use one tested coordinate conversion helper. For a right-handed project XY frame, the initial candidate is:

```text
project_x = carla_x
project_y = -carla_y
project_yaw = wrap_radians(-radians(carla_yaw_degrees))
```

Validate the conversion with the selected CARLA map and path before accepting it.

## Session Lifecycle

`CarlaSession.start()` should connect, store original world settings, configure synchronous timing, spawn the ego actor and IMU, register callbacks, then warm up frames. Callbacks only update a thread-safe latest snapshot.

`CarlaSession.close()` must be idempotent and use this order:

1. Apply safe stop control.
2. Stop sensor listeners.
3. Destroy session-owned sensor actors.
4. Destroy the session-owned ego actor.
5. Disable Traffic Manager synchronization only when this session enabled it.
6. Restore original world settings only when this session changed them.

On partial startup failure, perform the same cleanup for every resource already created.

## Testing Boundary

`unit_test_carla_session.py` uses fake client/world/actor objects to verify settings, actor ownership, startup failures, tick ordering, and cleanup.

`unit_test_io_carla.py` uses fake actor and snapshot objects to verify sensor mapping, steering scaling, throttle/brake behavior, clipping, and safe stop. It must not require CARLA installation or a server.

`test_integration_carla.py` runs when executed directly against a local CARLA server; normal `unittest` discovery skips it. It verifies spawn, synchronous tick, coherent IO reads, bounded steering writes, safe stop, actor cleanup, and world-setting restoration. Its fixed 15-second maneuver writes `test/artifacts/carla_sensor_trace.csv` and `carla_sensor_trace.png` with command, sensor, pose, and rate traces.

`config_vehicle_carla.yaml` contains the canonical Tesla Model 3 route for its configured spawn transform. `test_integration_carla_control.py` loads that profile, then replaces the route in test code with a longer 45-second route based on the actual first `IOCarla` pose. The test route has an approximately 12 m minimum curvature radius, rather than tight QCar-style turns. It uses EKF, static planner, simple controller, and `IOCarla`; it verifies sustained route progress and safe cleanup rather than requiring the full long route to finish. It writes `integration_carla_control.csv` and `integration_carla_control.png`. The profile's Model 3 mass/inertia values are currently documentation values; the shared runtime consumes its `2.875 m` wheelbase for EKF bicycle prediction.

`test_integration_vehicle_main_carla.py` invokes `core.vehicle_main.main()` with the unchanged canonical CARLA config. It exercises the normal config loader, module factory, runtime lifecycle, CARLA session, and `IOCarla` sensor reads. The test patches only the sleep delay and captures runtime telemetry; it writes `vehicle_main_carla.csv` and `vehicle_main_carla.png` without injecting a test config or altering production startup behavior.

## Multi-Vehicle Process Setup

For a shared synchronous CARLA world, each vehicle may run in its own process, but only one configured process may call `world.tick()`. The other vehicle processes must consume completed frames; allowing every vehicle process to tick the same world causes races and incoherent sensor frames.

`extra/launch/carla_scenario.py` parses a JSON setup manifest before any CARLA import or process launch. The manifest provides shared host/port and, for each vehicle, a unique ID, selected vehicle config, spawn transform, XY route, and `tick_owner` flag. It validates that exactly one vehicle owns synchronous ticking. The process runner passes only each vehicle's own parsed values into its process; no Python runtime object is shared across process boundaries.

`extra/launch/carla_process_runner.py` is that launcher worker. It starts one manifest-selected vehicle runtime per process, then returns JSON telemetry to its parent process. The tick owner configures/ticks the world; followers use `wait_for_tick()`. `test_integration_carla_multi_process.py` owns test-only CSV/PNG generation: it starts two workers with distinct broad routes, waits for both ready markers, releases their shared start signal, and records the per-vehicle artifacts in `test/artifacts/carla_multi_process/`. Run it directly only with a ready local CARLA server.

`core/vehicle_process.py` is the platform-neutral base used by process runners. It builds and runs one vehicle from a config selection and overrides, with optional lifecycle hooks but no CARLA, ROS2, or CarSim dependency. `test_integration_multi_process_virtual.py` proves that the same base starts two independent mathematical `IOVirtual` vehicle processes. Future ROS2 and CarSim runners should adapt their setup values to this base, while their transport/session details remain outside `core/`.

## Deferred Features

Keep raw GNSS conversion, camera/lidar synchronization, Traffic Manager, background traffic, map changes, reverse gear, collision sensors, multiple ego vehicles, and asynchronous CARLA mode outside the MVP.
