# Deployment Side Program

## Purpose

`extra.deployment` packages a reviewed allowlist, verifies its manifest, and
uses pinned-host SSH/SFTP to stage/activate a release on one or more Linux
vehicle endpoints. It is not a vehicle runtime, ground-station TCP server, or
runtime fleet manager.

## Configuration

```text
extra/deployment/config/
  bundles/       versioned source-selection policies
  templates/     reviewed examples
  local/         ignored endpoint inventories, host keys, and output archives
```

One `deployment_target` separates logical identity, `ssh`, `release`, and
`preflight`. A `deployment_targets` inventory chooses its `bundle` and a list
of target-file references. The release command, not the deployment schema,
selects QCar, Limo, ROS 2, or another Linux platform.

## Entry Point

Use `python -m extra.deployment`. The current operator guide is maintained in
[`extra/deployment/README.md`](../../../../extra/deployment/README.md).
The shared bundle/target/result dataclasses are documented in
[deployment-type.md](deployment-type.md).
