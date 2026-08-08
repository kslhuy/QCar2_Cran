# `core/vehicle_main.py`

## 1. Introduction

This is the command-line entry point for one configured vehicle actor. It is a
thin composition/lifecycle wrapper; multi-vehicle coordination belongs to
`extra.simulator.launcher`.

## 2. Code structure

| Definition | Inputs | Output | Purpose |
| --- | --- | --- | --- |
| `main(argv=None)` | optional argument sequence | integer process exit code | Parses CLI overrides, loads config, builds modules/runtime, paces steps at configured loop rate, and guarantees shutdown. |

The parser accepts `--vehicle-id`, `--cycles`, `--vehicle-config`, `--headless`,
`--ground-station`, `--ground-station-host`, and `--ground-station-port`.
Vehicle/bridge selection becomes `load_config` overrides; `--headless` selects
`config_vehicle_headless.yaml` when no explicit config is given.

## 3. Special data and cross-references

CLI data becomes `selection_overrides` (bridge profile) and `value_overrides`
(vehicle ID and nested bridge host/port) for [[vehicle-config|load_config]].
`period_s=1/loop_rate_hz` paces the outer loop; runtime computes its own
monotonic step period.

## 4. Position in the project

The entry point calls [[module-factory|build_vehicle_modules]] and creates
[[vehicle-runtime|VehicleRuntime]]. It owns no actor spawn, fleet manager,
or simulator session lifecycle beyond the injected runtime's normal shutdown.

## 5. Use and verification

Run the module with a current vehicle profile, optionally `--cycles N` for a
bounded smoke run. Keyboard interruption returns success after `shutdown()`.
Process/lifecycle behavior is covered by `test/unit_test_vehicle_process.py`
and runtime integration tests.
