"""Shared executable platform integrations and process-lifecycle contracts."""

from .base import (
    BasePlatformProcessManager,
    PlatformProcessContext,
    PlatformSetup,
    ScenarioPlatformProcessManager,
    StartPolicy,
    run_platform_process,
)

__all__ = [
    "BasePlatformProcessManager",
    "PlatformProcessContext",
    "PlatformSetup",
    "ScenarioPlatformProcessManager",
    "StartPolicy",
    "run_platform_process",
]
