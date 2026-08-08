"""Vehicle IO interfaces with lazy optional-backend imports."""

from __future__ import annotations

from importlib import import_module

from .io_base import IOBase, IONull

__all__ = [
    "IOBase",
    "IONull",
    "IOCarla",
    "IOQCar2",
    "IOQCar2ROS2",
    "IOLimoROS2",
    "IOVirtual",
]

_LAZY_EXPORTS = {
    "IOCarla": (".io_carla", "IOCarla"),
    "IOQCar2": (".io_qcar2", "IOQCar2"),
    "IOQCar2ROS2": (".io_qcar2_ros2", "IOQCar2ROS2"),
    "IOLimoROS2": (".io_limo_ros2", "IOLimoROS2"),
    "IOVirtual": (".io_virtual", "IOVirtual"),
}


def __getattr__(name: str):
    """Import an optional adapter only when a caller requests it."""
    try:
        module_name, attribute = _LAZY_EXPORTS[name]
    except KeyError as exc:
        raise AttributeError(f"module '{__name__}' has no attribute '{name}'") from exc
    value = getattr(import_module(module_name, __name__), attribute)
    globals()[name] = value
    return value
