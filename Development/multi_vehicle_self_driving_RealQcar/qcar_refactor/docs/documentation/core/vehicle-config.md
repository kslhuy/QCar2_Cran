# `core/vehicle_config.py`

## 1. Introduction

This file resolves YAML selection into one immutable `ConfigVehicle`. It loads
profile data, injects mission-owned values into their owning modules, applies
overrides in a defined order, and validates the completed bundle; it never
creates hardware or connects a network.

## 2. Code structure

`ConfigVehicle(vehicle_id, runtime, mission, modules)` stores selected data;
`load_config(config_dir, vehicle_config_file, selection_overrides,
value_overrides)` is the normal runtime loader. `load_module_profile` is the
narrow launch-layer profile loader.

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `ConfigError` | malformed/unsafe configuration | exception | Identifies resolution or validation failure. |
| `ConfigVehicle` | ID, runtime mapping, mission mapping, module mappings | frozen config bundle | Holds one actor's resolved ownership boundary. |
| `ConfigVehicle.module(name)` | registered module name | selected module mapping | Returns the named module or raises `ConfigError`. |
| `load_config(...)` | config directory/file and optional overrides | `ConfigVehicle` | Resolves profile selections, injects mission/model timing values, applies value overrides, validates, and freezes bundle. |
| `load_module_profile(module_name, profile, config_dir)` | module/profile name | deep-copied profile mapping | Loads one reusable launch-layer profile without vehicle composition. |
| `_load_yaml(path)` | YAML path | root mapping | Loads ASCII YAML and normalizes file/YAML/root failures to `ConfigError`. |
| `_mapping(source, key)` | mapping/key | nested dict | Requires a mapping-valued configuration section. |
| `_string(source, key, default)` | mapping/key/default | non-empty string | Requires a profile/module name. |
| `_select(profiles, profile, kind)` | profiles/name/kind | deep-copied profile | Selects a named mapping or raises unknown-profile error. |
| `_module_config_path(directory, module_name)` | config directory/module name | YAML path | Validates module identifier then forms `config_<module>.yaml`. |
| `_apply_value_overrides(values, overrides)` | resolved bundle and nested overrides | in-place mutation | Deep-merges mapping values and rejects unknown top-level keys. |
| `_apply_selection_overrides(modules, selection)` | module selection and name mapping | in-place mutation | Replaces profile names before profile lookup. |
| `_merge(target, override)` | nested mappings | in-place mutation | Recursively deep-copies override leaves. |
| `_positive_number(value, name)` | candidate numeric value | none or error | Requires a positive non-boolean numeric configuration value. |
| `_positive_integer(value, name)` | candidate integer | none or error | Requires positive non-boolean integer. |
| `_port(value, name)` | candidate port | none or error | Requires integer port in `[1, 65535]`. |
| `_throttle_limit(value, name)` | candidate limit | none or error | Requires normalized positive throttle limit in `(0, 1]`. |
| `_validate_manual_controller(controller)` | controller mapping | none or error | Validates optional manual timeout/throttle/steering settings. |
| `_validate_planner(planner)` | planner mapping | none or error | Validates loop mode and runtime-profile mapping. |
| `_validate_v2v(v2v, vehicle_id)` | V2V profile/local ID | none or error | Validates endpoint, peers, buffers, and per-message rates. |
| `_validate_ground_station(ground_station)` | bridge profile | none or error | Validates supported bridge plus TCP host/port/rates/queue limits. |
| `_validate(values)` | completed bundle | none or error | Validates ID, loop rate, model, IO, controller, planner, V2V, and bridge sections. |

## 3. Special data and cross-references

`selection_overrides` changes profile identity before lookup; `value_overrides`
changes fields only after selected profiles are copied. Mission `path` or
`node_sequence` with `loop` is injected into planner only when that planner has
not set its own value. IO receives runtime loop rate under `io.timing`, and the
observer inherits model wheelbase when appropriate.

Current/operator scenarios are `config/scenarios/`; test-only scenarios are in
`config/scenarios/test/` and remain documented with their tests. See
[[module-factory|module_factory]] for the exact consumer of this bundle.

## 4. Position in the project

[[vehicle-process|VehicleProcessSpec]], single-vehicle main, and scenario
parsers call `load_config`; [[module-factory|module_factory]] consumes
`ConfigVehicle`. This file does not determine command eligibility, lifecycle,
or any physical resource ownership.

## 5. Use and verification

Use `load_config(vehicle_config_file=..., selection_overrides=...,
value_overrides=...)` once per actor process, then pass the result to the
factory. `test/unit_test_vehicle_config.py` verifies resolution order, mission
injection, deep overrides, invalid profiles, limits, and scenario-related
configuration validation.
