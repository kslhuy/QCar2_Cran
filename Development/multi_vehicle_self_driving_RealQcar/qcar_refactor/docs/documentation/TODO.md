# Documentation and Physical-Vehicle TODO

This is the working checklist for the project documentation vault.  It is
deliberately kept beside the reference pages so a design decision, its
implementation, and the remaining validation work can be followed together.

## Principles

- [x] `core` owns lifecycle, arbitration, and the final safe-zero decision;
  adapters never create a second control path.
- [x] Hardware, simulator, and ROS implementations share the
  [[io-base|IOBase]] contract: cached [[vehicle-types|SensorData]] in and
  bounded [[vehicle-types|ControlInput]] out.
- [x] Current/operator configuration remains in `config/scenarios/`; test-only
  configuration remains in `config/scenarios/test/` and is documented with its
  tests.
- [x] The SDCS route is a planner concern.  `loop=0`, `1`, `2`, and `inf` have
  explicit, testable route-completion semantics; fleet followers do not replace
  their fleet-owned reference.
- [ ] Every source page is independently checked against its implementation and
  follows the source-page template, recording classes,
  initialization dependencies, functions, data contracts, project role, and
  tests.
- [ ] Every package README is independently checked and follows the package
  template, explaining file
  variations rather than repeating each implementation page.
- [ ] Verify Obsidian links are meaningful ownership/data-contract relations;
  do not use links merely to decorate prose.

## Delivered behavior

- [x] Ground-station command handling exposes manual control and SDCS map
  enable/disable commands, including CLI help.
- [x] CARLA's current default selects the SDCS path planner and a default
  scenario route.
- [x] The map planner supports the documented loop modes and unit coverage.
- [x] Documentation scope includes active `extra/` integration and current
  configuration, while excluding test scenario detail from the operator guide.

## Remaining implementation work

- [ ] Implement a selectable ROS 2 QCar adapter rather than selecting the
  current `IOQCar2ROS2` stub: node/executor ownership, subscriptions, command
  publication, namespace and TF isolation, timestamp/freshness handling,
  shutdown zero command, factory profile, and tests.
- [ ] Add physical-vehicle bootstrap ownership for QCar, GPS, LiDAR, and
  cleanup.  [[io-qcar2|IOQCar2]] must continue to receive already-created
  resources rather than silently owning them.
- [ ] Make localization selectable by configuration.  Start with an
  open-source ROS localization pipeline (for example Cartographer plus a
  documented LiDAR-to-vehicle/GPS transform) and preserve the option to use
  Quanser's lidar2gps module behind the same contract.
- [ ] Establish and record the physical calibration procedure: sensor time
  synchronization, LiDAR-to-body extrinsic, GPS/vehicle frame relationship,
  map origin, TF validation, and repeatable acceptance drive.  A reference
  scan alone is not an extrinsic calibration.
- [ ] Add hardware-in-the-loop smoke tests for safe stop, manual command,
  GPS/IMU freshness, localization validity, and route completion before any
  autonomous physical run.

## Documentation conversion plan

- [x] Publish the general source-page and package-README templates.
- [x] Convert `utils/io` into the worked example, including the base contract
  and its hardware/simulator variations.
- [x] Audit and correct `core/` source pages definition by definition. Each
  source page now names its actual definitions, dependencies, data contracts,
  ownership boundary, and focused verification.
- [ ] Audit and correct `utils/control/`, `utils/fleet/`,
  `utils/ground_station/`, `utils/io/`, and `utils/v2v/` after `core` passes.
  - [x] `utils/v2v/`, `utils/fleet/`, `utils/ground_station/`, and
    `utils/control/` source pages have been checked against their implementation
    and focused unit tests.
    - Fleet re-audit: added the missing distributed-observer factory page; all
      13 fleet source pages now use the source-page table.
    - Control audit: all 17 pages now use the source-page table. The historical
      `observer_high_gain.py` and `observer_luenberger.py` are byte-identical
      copies of `observer_ekf.py`, define `ObserverEKF`, and remain unsupported
      by the current observer configuration/factory.
  - [ ] `utils/io/` remains to be checked definition by definition.
- [x] Audit and correct `extra/` package and source pages, including simulator.
  Each page now names its active definitions, data contracts, ownership
  boundary, and focused setup, ground-station, or multi-process coverage.
- [x] Establish the rule that each future test-only scenario page is linked from
  the corresponding test when that test and scenario are extended together.

## Definition of done

A source page is complete when it uses all five template sections, names the
actual class/function signatures, links its input/output contracts, states its
owner/caller, and points to its focused tests.  A package README is complete
when it uses the package template and explains what every implementation has
in common and what differs by backend or algorithm.
