# Extra Integration

## 1. Introduction

`extra/` contains executable integration applications: deployment transport,
the operator-facing ground station, and platform launch infrastructure. It
composes core services but does not replace their authority.

## 2. File structure and variations

- [Deployment side program](deployment/README.md) contains bundle policy,
  Linux endpoint configuration, SSH/SFTP transport, and the release CLI.
- [Ground-station application](ground-station/README.md) contains operator
  configuration, a core listener/session layer, terminal presentation, and
  reusable logging/live-plotting support.
- Platform integration owns shared process setup and virtual/CARLA/QCar/ROS 2
  scenario, runner, session, or resource-context variations.

## 3. Shared data and cross-references

Ground station uses typed [[commands|VehicleCommand]] and monitoring frames.
Deployment sends only approved release files and lifecycle requests. Platform
scenarios produce [[vehicle-process|VehicleProcessSpec]] and observe runtime
telemetry. None is a second command or safety channel.

## 4. Position in the project

These are the top-level callers of [[vehicle-process|vehicle_process]] and
ground-station transport; [[vehicle-runtime|VehicleRuntime]] remains the
authority inside every vehicle process.

## 5. Use and verification

Run the extra entry points with current configuration/scenarios. Verify the
operator application with ground-station tests and scenario runners with
multi-process virtual/CARLA integration tests.

## Conclusion

All extra integrations share core contracts; they differ only by operator or
simulator boundary while per-vehicle lifecycle and safe actuation remain core-owned.
