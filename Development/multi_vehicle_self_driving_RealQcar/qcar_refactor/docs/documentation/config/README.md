# Current Configuration

## 1. Introduction

`config/` contains named module profiles, single-vehicle composition templates,
and current operator-facing scenarios. [[vehicle-config|vehicle_config]]
resolves this data; YAML never constructs or controls a runtime by itself.

## 2. File structure and variations

`config_model.yaml`, `config_io.yaml`, `config_observer.yaml`,
`config_planner.yaml`, `config_controller.yaml`, `config_v2v.yaml`,
`config_fleet.yaml`, `config_simulation.yaml`, and `config_ground_station.yaml`
define reusable implementation profiles. `config_vehicle*.yaml` files select a
profile set for one vehicle. Current operator scenarios live in `scenarios/`.

## 3. Shared data and cross-references

Selection names choose implementation identity; value overrides alter resolved
profile values. Mission data is waypoint `path` or SDCS `node_sequence` plus
`loop`. `config/scenarios/test/` is intentionally excluded: it is test-only
data documented together with the test that consumes it.

## 4. Position in the project

Configuration flows through [[vehicle-config|ConfigVehicle]] to
[[module-factory|module_factory]], then [[vehicle-runtime|VehicleRuntime]].
It has no direct hardware, network, or lifecycle authority.

## 5. Use and verification

Select a vehicle template or current scenario; resolve it with `load_config`.
Use `test/unit_test_vehicle_config.py` and scenario integration tests when
changing current configuration. Extend test scenarios only with their tests.

## Conclusion

All current configuration selects/parameterizes the same runtime contracts;
test-only configuration remains separate so operator defaults stay auditable.
