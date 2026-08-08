# System Mechanism

```text
Ground station / simulator command
        -> VehicleCommand -> VehicleCommandHandler
        -> VehicleRuntime (state + safe IO write)

IO.read -> Observer.update -> Planner.update -> Controller.compute -> IO.write
                         ^             |
                         |             +-- FleetManager may replace follower reference
                         +-- previous applied ControlInput
```

The runtime is the only component that decides whether a non-zero actuator
command may be written. `READY`, `STOPPED`, and `ERROR` produce zero output.
The command handler interprets typed intent but returns actions; the runtime
performs profile selection, controller reset, and safe-zero work.

## Main state flow

```text
INITIALIZING -> READY --START--> RUNNING --STOP/finish--> STOPPED
                     ^              |                       |
                     +--- RESET ----+--- ERROR/ESTOP --------+
```

Fleet and manual modes do not replace this state machine. They select bounded
references/controllers only while the runtime remains `RUNNING`.

## Configuration and launch layers

`core.vehicle_config` resolves named module profiles. `core.module_factory`
creates injected modules. `core.vehicle_process` builds one actor. The scenario
launchers in `extra/` select process-specific configuration; current scenarios
are in `config/scenarios/`, and tests use `config/scenarios/test/`.
