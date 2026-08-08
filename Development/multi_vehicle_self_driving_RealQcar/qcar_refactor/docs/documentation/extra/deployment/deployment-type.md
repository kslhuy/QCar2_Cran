# `extra/deployment/deployment_type.py`

## Purpose

Owns every deployment-side data contract: `BundleSummary`, `DeploymentTarget`,
`CommandResult`, `PreflightReport`, `DeploymentTargetEntry`, and
`DeploymentTargets`. The bundle, configuration, SSH, and multi-target services
consume these contracts but do not define their own dataclasses.

## Verification

`test/unit_test_deployment.py` asserts that every deployment dataclass is
defined by this module.
