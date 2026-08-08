"""Quanser QCar platform resource ownership and physical process runner."""

from typing import Any

__all__ = [
    "QCarLidarFactory",
    "QCarProcessManager",
    "QCarSetup",
    "QCarVehicleSetup",
    "qcar_lidar_resource_context",
    "qcar_resource_context",
    "run_qcar_process",
]


def __getattr__(name: str) -> Any:
    """Keep package exports without importing the runner before ``python -m`` executes it."""

    if name in __all__:
        from . import process_runner

        return getattr(process_runner, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
