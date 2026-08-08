# Core

## 1. Introduction

`core` owns vehicle-domain lifecycle, validated operator intent, composition,
and the ordered control loop. Its direct callers are entry points and simulator
workers; utilities are injected rather than imported by runtime logic.

## 2. File structure and variations

| File | Role |
| --- | --- |
| [commands](commands.md) | Typed command/result transport contract. |
| [command-handler](command-handler.md) | State, fleet, manual, and SDCS eligibility. |
| [vehicle-config](vehicle-config.md) | Profile resolution and validation. |
| [module-factory](module-factory.md) | Concrete dependency composition. |
| [vehicle-types](vehicle-types.md) | Cross-layer immutable records. |
| [vehicle-state-machine](vehicle-state-machine.md) | Drive-permission policy. |
| [vehicle-runtime](vehicle-runtime.md) | Ordered step loop and final safe write. |
| [vehicle-process](vehicle-process.md) | Process-level lifecycle wrapper. |
| [vehicle-main](vehicle-main.md) | Single-vehicle command-line entry point. |

## 3. Shared data and cross-references

[[commands|VehicleCommand]] and `CommandResult` carry intent and outcome;
[[vehicle-types|SensorData]], `VehicleStateEstimate`, `ControllerReference`,
and `ControlInput` carry control data. [[vehicle-config|ConfigVehicle]]
names the selected implementation profiles.

## 4. Position in the project

Core is the authority boundary: utilities implement bounded services, while
[[vehicle-runtime|VehicleRuntime]] alone coordinates them and enforces a
safe zero when driving is not permitted.

## 5. Use and verification

Use `VehicleProcessSpec` from an entry point or scenario worker. Focused tests
are `test/unit_test_commands.py`, `unit_test_command_handler.py`,
`unit_test_vehicle_config.py`, `unit_test_state_machine.py`,
`unit_test_vehicle_runtime.py`, and `unit_test_vehicle_process.py`.

## Conclusion

All core files share lifecycle and data contracts; each owns one policy or
composition boundary, while backend-specific behavior remains in `utils`.
