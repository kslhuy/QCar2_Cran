# Simulator Integration

## 1. Introduction

This extra package launches isolated vehicle processes for virtual and CARLA
scenarios. It coordinates startup/barriers/telemetry but delegates each vehicle
to the core process/runtime contract.

## 2. File structure and variations

[base](base.md), [scenario](scenario.md), and [launcher](launcher.md) are shared.
[Virtual scenario](virtual-scenario.md) and [runner](virtual-runner.md) use the
headless IO model. [CARLA scenario](carla-scenario.md), [runner](carla-runner.md),
and [session](carla-session.md) add CARLA resource ownership.

## 3. Shared data and cross-references

Scenario records produce [[vehicle-process|VehicleProcessSpec]] objects. Mission
path/node-sequence/loop is configuration data; telemetry is observation data.

## 4. Position in the project

Simulator code is an integration layer above [[vehicle-runtime|VehicleRuntime]].
It may coordinate processes and owned sessions but never bypasses runtime safety.

## 5. Use and verification

Run extra.simulator.launcher with a current scenario. Test-only scenario details
remain alongside their tests under config/scenarios/test.

## Conclusion

All simulator backends share process/runtime orchestration; they differ only in
world/session ownership, while control and safety remain in core.

