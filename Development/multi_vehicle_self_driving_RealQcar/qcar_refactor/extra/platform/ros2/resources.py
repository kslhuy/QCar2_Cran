"""Lifecycle ownership for a future ROS 2 vehicle-platform resource context.

This module intentionally does not implement ROS vehicle IO.  A future
``IORos2`` adapter will receive the yielded ``ros_node`` resource through the
same platform-runner contract used by QCar.  Keeping ``rclpy`` imports lazy
makes the rest of the project usable on machines without ROS 2 installed.
"""

from __future__ import annotations

from contextlib import contextmanager
from typing import Any, Callable, Iterator

from core.vehicle_config import ConfigError


@contextmanager
def ros2_resource_context(
    *,
    node_name: str,
    rclpy_module: Any | None = None,
    node_factory: Callable[[str], Any] | None = None,
) -> Iterator[dict[str, Any]]:
    """Initialise ROS 2, own one node, and release both deterministically."""

    if not isinstance(node_name, str) or not node_name.strip():
        raise ValueError("ROS 2 node_name must be a non-empty string")
    if rclpy_module is None:
        try:
            import rclpy as rclpy_module
        except ImportError as error:
            raise ConfigError(
                "ROS 2 support is unavailable. Install and source ROS 2 before "
                "using the ROS 2 platform resource context."
            ) from error

    rclpy_module.init()
    factory = node_factory or rclpy_module.create_node
    node = None
    try:
        node = factory(node_name.strip())
        yield {"ros_node": node}
    finally:
        if node is not None:
            destroy = getattr(node, "destroy_node", None)
            if callable(destroy):
                destroy()
        rclpy_module.shutdown()
