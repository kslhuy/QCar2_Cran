# CARLA Startup And IO TODO

This plan adds CARLA as a backend for the refactored runtime while keeping simulation startup separate from the common vehicle IO adapter.

It depends on the contract, config, and runtime phases in `docs/MINIMAL_REFACTOR_PLAN.md`. Do not build CARLA-specific runtime branches before those shared boundaries exist.

Reference points:

- Current adapter stub: `utils/io/io_carla.py`
- Shared IO contract: `utils/io/io_base.py`
- Existing QCar adapter: `utils/io/io_qcar2.py`
- Shared data contracts: `core/types.py`
- Runtime plan: `docs/MINIMAL_REFACTOR_PLAN.md`
- Legacy QCar startup: `refs/qcar_origin/vehicle_main.py` and `refs/qcar_origin/vehicle_logic.py`
- CARLA examples: `refs/carla_test/examples/client_bounding_boxes.py` and `refs/carla_test/examples/generate_traffic.py`
- CARLA API notes: `refs/carla_test/docs/client.yml`, `world.yml`, `sensor.yml`, and `control.yml`

## Overview

CARLA support needs two different components:

| Component | Owns | Does not own |
|---|---|---|
| `CarlaSession` in `utils/io/carla_startup.py` | Client connection, world settings, ego/sensor actors, sensor callbacks, frame synchronization, ticking, and cleanup | Observer, planner, controller, state-machine decisions, or command clipping |
| `IOCarla` in `utils/io/io_carla.py` | Converting a session snapshot to `SensorData` and converting `ControlCommand` to ego vehicle control | Client creation, map loading, world ticking, actor destruction, or world setting restoration |

The runtime owns the session and the IO adapter. There must be one clear owner for CARLA actors and world settings: `CarlaSession`.

## Problems And Benefits

| Problem to solve | Plan change | Benefit | Main risk |
|---|---|---|---|
| `io_carla.py` imports CARLA at module import time | Lazy-load CARLA only when the CARLA backend is selected | Core and QCar tests work without the CARLA package | Backend errors must be reported clearly at startup |
| CARLA startup and IO could be placed in one class | Use a separate `CarlaSession` and inject it into `IOCarla` | Startup failure, cleanup, and IO mapping can be tested independently | Requires explicit runtime lifecycle wiring |
| CARLA steering is normalized while the common contract uses radians | Add a documented radians-to-normalized conversion and sign setting | Controllers remain backend independent | Wrong scale or sign can cause unsafe steering |
| CARLA throttle/brake/reverse are separate fields | Define an explicit negative-throttle policy | Commands have predictable behavior across backends | QCar motor command and CARLA pedal semantics are not identical |
| Raw GNSS is latitude/longitude but `gps_position` is local pose | Use actor transform for MVP local `[x, y, yaw]`; defer GNSS conversion | Observer receives the expected units immediately | Ground-truth pose is less realistic than simulated GNSS |
| CARLA and QCar use different coordinate conventions | Convert CARLA pose and vectors through one tested helper | Planner and observer see one project coordinate frame | Sign mistakes can mirror paths or yaw |
| Simulation timestamps differ from wall time | Separate IO poll clocks from sample timestamps | Deterministic simulation time can be preserved | Requires a shared `IOBase` change first |
| Sensor callbacks are asynchronous | Cache immutable frame snapshots under a lock and wait by frame ID | The control loop reads coherent data | Waiting logic needs timeouts to avoid deadlock |
| Synchronous worlds require one tick owner | Put `tick()` on `CarlaSession` and call it once from runtime | Multi-module code cannot accidentally advance multiple frames | Multiple external CARLA clients still need operational coordination |
| Cleanup may run after partial startup or more than once | Track owned actors/settings and make shutdown idempotent | Failed runs restore the simulator predictably | Every partial-start path needs a test |

## Required Data Decisions

Use these as working MVP defaults, not as a claim that QCar, CARLA, and ROS sensors have identical physics. The shared boundary uses SI-facing values where conversion is available, and each adapter documents approximations or missing validity:

- `motor_tach`: for CARLA, use signed longitudinal speed in m/s computed as velocity dot vehicle-forward-vector; retain the legacy field name until cross-platform measurements justify renaming it.
- `gyro_z`: yaw rate in rad/s in the project coordinate frame.
- `accelerometer`: `[ax, ay, az]` in m/s^2 with the frame documented.
- `gps_position`: local project pose `[x_m, y_m, yaw_rad]` for the MVP.
- `sensor_timestamp` and `gps_timestamp`: CARLA simulation elapsed seconds or another explicitly selected sample-time domain.
- Poll scheduling: internal monotonic clocks, never sample timestamps.
- Steering command: radians at the common boundary, mapped to CARLA `steer` in `[-1, 1]` with a configurable sign and full-scale steering angle.
- Negative throttle: one configured policy, initially `brake`; add reverse only after a dedicated test.

Keep the existing `ControlCommand` during the first integration. A future backend-specific inner controller may map desired speed and road-wheel angle to CARLA pedals/steering or QCar motor commands after both platforms have calibration data.

CARLA native coordinates should pass through one helper. If the project uses a right-handed local XY frame, the expected initial conversion is typically equivalent to:

```text
project_x = carla_x
project_y = -carla_y
project_yaw = wrap_radians(-radians(carla_yaw_degrees))
```

Verify this against the selected map and path data before accepting it as final.

## Proposed Files

```text
utils/io/
    io_base.py             shared IO contract and monotonic polling clocks
    io_carla.py            narrow CARLA sensor/control adapter
    carla_startup.py       session configuration, startup, tick, and shutdown
    carla_types.py         optional small dataclasses for handles/snapshots

test/
    unit_test_io_carla.py
    unit_test_carla_startup.py
    test_integration_carla.py
```

Keep `carla_types.py` separate only if it prevents `carla_startup.py` from becoming difficult to read. Do not create it for one or two trivial dataclasses.

## Suggested Configuration

Add a `carla` backend section to `config/config_io.yaml`. Keep runtime selection in `config/config.yaml`.

```yaml
vehicle:
  vehicle_type: carla

carla:
  write:
    max_throttle: 0.10
    max_steering: 0.48
  read:
    sensor_rate_hz: 100
    gps_rate_hz: 20
  connection:
    host: "127.0.0.1"
    port: 2000
    timeout_s: 10.0
  world:
    map: null
    synchronous_mode: true
    fixed_delta_seconds: 0.01
  ego:
    blueprint_filter: "vehicle.*"
    role_name: "hero"
    spawn_index: 0
  control:
    steering_full_scale_rad: 0.48
    steering_sign: 1.0
    negative_throttle_mode: "brake"
  sensors:
    imu: true
    camera: false
    gnss: false
  startup:
    warmup_frames: 5
    sensor_timeout_s: 2.0
```

Validation must reject zero/negative rates, invalid ports, non-positive fixed delta, empty blueprint results, invalid spawn indices, unsupported negative-throttle modes, and zero steering scale.

## Implementation TODO

A checked item means code and focused tests exist.

### Completed Review

- [x] Review the CARLA synchronous-client and traffic-generation examples.
- [x] Compare the CARLA work with `IOBase`, `IOQCar2`, and the legacy QCar startup path.
- [x] Decide that CARLA startup/session ownership stays separate from `IOCarla`.
- [x] Define a small MVP: ego actor, IMU, actor transform/velocity, control, deterministic tick, and cleanup.

### Phase 0: Finish Shared Prerequisites

- [ ] Record minimum SI-facing defaults, frame conversions, command semantics, timestamp domains, and backend approximations in the shared data contract.
- [ ] Change `IOBase` rate limiting to use separate monotonic poll clocks.
- [ ] Standardize idempotent `stop()` and `close()` behavior in the IO contract.
- [ ] Implement config composition and backend factories in `core/vehicle_config.py`.
- [ ] Implement the headless runtime lifecycle hook that can call a backend session `tick()`.
- [ ] Add `carla` to normalized vehicle types without importing CARLA during config parsing.

Acceptance gate: the null runtime and existing tests pass without CARLA installed.

### Phase 1: Isolate Imports And Add Config

- [ ] Remove the top-level `import carla` from `utils/io/io_carla.py`.
- [ ] Ensure importing `utils.io`, `IOBase`, or `IOQCar2` succeeds without CARLA installed.
- [ ] Add the validated `carla` section to `config/config_io.yaml`.
- [ ] Add a factory branch that imports CARLA modules only after `vehicle_type: carla` is selected.
- [ ] Raise one clear startup error containing the missing package or connection problem.
- [ ] Add tests for import-without-CARLA and unavailable-backend error handling.

Acceptance gate: non-CARLA backends have no CARLA package dependency.

### Phase 2: Define Session Data And Ownership

- [ ] Add a `CarlaStartupConfig` dataclass or validated immutable config object.
- [ ] Add a thread-safe `CarlaSensorSnapshot` containing frame ID, simulation timestamp, IMU values, local pose, signed speed, and validity flags.
- [ ] Add `CarlaHandles` only if a separate handle container improves testing.
- [ ] Implement `CarlaSession` with `start()`, `snapshot()`, `tick()`, `stop_vehicle()`, and `close()`.
- [ ] Track the original world settings and only the actors created by this session.
- [ ] Make `close()` safe before startup, after partial startup, and on repeated calls.
- [ ] Keep actor destruction and world-setting restoration out of `IOCarla`.

Acceptance gate: lifecycle behavior is testable with fake client/world/actor objects.

### Phase 3: Implement CARLA Startup

- [ ] Connect with `carla.Client(host, port)`, set timeout, and get or load the configured world.
- [ ] Copy and retain original world settings before applying changes.
- [ ] Apply synchronous mode and `fixed_delta_seconds` when configured.
- [ ] Do not initialize Traffic Manager in the MVP unless background traffic is enabled.
- [ ] Select an ego blueprint deterministically and set `role_name`.
- [ ] Validate spawn points and use `try_spawn_actor()` with a clear failure message.
- [ ] Spawn and attach the IMU sensor.
- [ ] Register callbacks that only convert and store sensor samples; callbacks must not run observer or control logic.
- [ ] Update actor transform and velocity in the session snapshot for each completed frame.
- [ ] Warm up the configured number of frames and wait for required sensor data by frame ID.
- [ ] On any startup exception, stop/destroy created sensors, destroy the ego actor, and restore changed world settings.

Acceptance gate: fake startup tests prove both successful construction and reverse-order cleanup after every injected failure point.

### Phase 4: Implement `IOCarla`

- [ ] Accept an already-started `CarlaSession` in the constructor and validate that an ego actor is available.
- [ ] Implement `_poll_sensors()` from one locked session snapshot.
  - [ ] Map signed longitudinal speed to `motor_tach` according to the shared contract.
  - [ ] Map converted IMU yaw rate and acceleration.
  - [ ] Preserve sample timestamp and frame validity.
- [ ] Implement `_poll_gps()` using converted actor transform for local `[x, y, yaw]`.
- [ ] Implement `_hardware_write()` without calling `world.tick()`.
  - [ ] Convert steering radians to normalized CARLA steering and clip to `[-1, 1]`.
  - [ ] Apply the configured steering sign.
  - [ ] Map positive throttle explicitly.
  - [ ] Map negative throttle to brake for the MVP; keep reverse disabled.
  - [ ] Clear stale brake/reverse fields when switching back to positive throttle.
- [ ] Implement `stop()` as full brake with zero throttle and neutral steering, according to the agreed safety policy.
- [ ] Implement `close()` without destroying session-owned actors or world settings.
- [ ] Log stale/missing sensor data without hiding repeated failures indefinitely.

Acceptance gate: fake-actor tests cover clipping, steering sign/scale, throttle-to-brake transitions, missing samples, stop behavior, and ownership boundaries.

### Phase 5: Integrate Deterministic Runtime Ticking

- [ ] Construct `CarlaSession` before `IOCarla` in the backend factory/runtime startup path.
- [ ] Keep exactly one tick owner.
- [ ] Use this synchronous loop order:

```text
apply the previous safe/control command
    -> CarlaSession.tick()
    -> wait for required sensor frame or timeout
    -> IOCarla.read_to_cache() and read()
    -> observer, planner, and controller
    -> store/apply the next command
```

- [ ] Decide and document whether the next command is applied immediately after computation or at the beginning of the next loop; tests must assume the same ordering.
- [ ] In asynchronous mode, use `wait_for_tick()` or callback snapshots without calling synchronous `world.tick()`.
- [ ] On runtime failure, call `vehicle_io.stop()` before `session.close()`.
- [ ] Restore the world only if this session changed its settings.
- [ ] Reject or clearly warn about multiple synchronous tick owners connected to the same world.

Acceptance gate: a fake runtime test proves one tick per loop, frame ordering, sensor timeout handling, zero command on error, and shutdown order.

### Phase 6: Verification

- [ ] Add `test/unit_test_carla_startup.py` using fake CARLA objects.
- [ ] Add `test/unit_test_io_carla.py` without requiring a running CARLA server.
- [ ] Keep the normal test suite runnable when the CARLA package is absent.
- [ ] Add an opt-in live test guarded by package/server availability.
- [ ] In the live test:
  - [ ] Connect to a local CARLA server.
  - [ ] Spawn one ego vehicle and IMU.
  - [ ] Tick until a coherent sample is available.
  - [ ] Verify forward motion produces positive signed speed.
  - [ ] Verify steering direction against the project coordinate convention.
  - [ ] Apply stop control and confirm safe shutdown.
  - [ ] Confirm actors are destroyed and original world settings are restored.
- [ ] Run a short closed-loop path test only after the zero/low-speed smoke test passes.

Acceptance gate: unit tests pass offline, and the opt-in live smoke test leaves no owned actors or modified world settings behind.

### Deferred Until After MVP

- [ ] Raw GNSS with latitude/longitude-to-local conversion.
- [ ] Camera and lidar ownership, queues, and frame synchronization.
- [ ] Traffic Manager and background traffic.
- [ ] Map loading and custom spawn transforms.
- [ ] Reverse gear behavior.
- [ ] Collision/lane-invasion sensors and safety events.
- [ ] Multiple ego vehicles in one CARLA process.
- [ ] Real-time-factor and performance telemetry.

Do not move a deferred item into the MVP unless it is required by a concrete runtime acceptance test.

## Shutdown Order

Use this order for both normal exit and exceptions:

1. Apply safe stop control to the ego vehicle.
2. Stop sensor listeners.
3. Destroy session-owned sensor actors.
4. Destroy the session-owned ego actor.
5. Disable Traffic Manager synchronous mode only if this session enabled it.
6. Restore original world settings only if this session changed them.
7. Mark the session closed so repeated cleanup is a no-op.

## Definition Of Done

CARLA support is complete for the first milestone when selecting `vehicle_type: carla` changes only backend construction/session wiring, the shared observer-planner-controller path runs unchanged, every exception path commands a safe stop, offline tests do not require CARLA, and a live smoke test restores the simulator to its original state.
