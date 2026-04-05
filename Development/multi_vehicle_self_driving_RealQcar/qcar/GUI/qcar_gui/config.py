"""
Configuration module for QCar Fleet Controller.

This module contains all configuration settings, constants, and dataclasses
for the application. Centralized configuration makes it easy to modify
settings without changing the core application code.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional
from enum import Enum, auto


class LogLevel(Enum):
    """Log level enumeration for consistent logging."""

    INFO = auto()
    SUCCESS = auto()
    WARNING = auto()
    ERROR = auto()
    DEBUG = auto()


@dataclass(frozen=True)
class NetworkConfig:
    """Network configuration settings."""

    host_ip: str = "0.0.0.0"
    base_port: int = 5000
    buffer_size: int = 4096
    connection_timeout: float = 30.0
    telemetry_buffer_size: int = 100


@dataclass(frozen=True)
class GUIConfig:
    """GUI configuration settings."""

    window_title: str = "🚗 QCar Fleet Controller"
    window_width: int = 1200
    window_height: int = 800
    update_rate_hz: float = 20.0
    max_cars: int = 8
    default_num_cars: int = 5


@dataclass(frozen=True)
class VehicleConfig:
    """Vehicle control configuration."""

    max_velocity: float = 2.0
    min_velocity: float = 0.0
    max_steering: float = 0.5
    steering_increment: float = 0.1
    steering_decay: float = 0.7
    default_throttle: float = 0.15
    default_following_distance: float = 2.0


@dataclass(frozen=True)
class ManualControlConfig:
    """Manual control configuration for keyboard and wheel input."""

    # Keyboard
    forward_key: str = "z"
    backward_key: str = "s"
    left_key: str = "q"
    right_key: str = "d"
    stop_key: str = "space"

    # Steering wheel (pygame axis indices)
    steering_axis: int = 0
    accelerator_axis: int = 5
    brake_axis: int = 4
    steering_scale: float = 0.5
    throttle_scale: float = 0.3
    deadzone: float = 0.05
    update_interval_ms: int = 50


@dataclass(frozen=True)
class VehicleDeploymentConfig:
    """Configuration for vehicle deployment (SSH connection and remote execution)."""

    # SSH credentials
    ssh_username: str = "nvidia"
    ssh_password: str = "nvidia"
    ssh_timeout: int = 10

    # Remote paths
    remote_path: str = "/home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/ros2test/multi_vehicle_RealCar"

    # Local scripts path (relative to GUI folder)
    scripts_path: str = "../../qcar"

    # Default vehicle settings
    default_ip_prefix: str = "192.168.137."
    default_vehicle_type: str = "Qcar"
    default_programme_type: str = "Ros"
    default_velocity: float = 0.6

    # Upload settings
    upload_folders: tuple = (
        "StateMachine",
        "Yolo",
        "Observer",
        "V2V",
        "Controller",
        "simulation",
        "Calibration",
        "PathPlanner",
        "Taxi",
    )


@dataclass
class AppConfig:
    """Main application configuration container."""

    network: NetworkConfig = field(default_factory=NetworkConfig)
    gui: GUIConfig = field(default_factory=GUIConfig)
    vehicle: VehicleConfig = field(default_factory=VehicleConfig)
    manual_control: ManualControlConfig = field(default_factory=ManualControlConfig)
    deployment: VehicleDeploymentConfig = field(default_factory=VehicleDeploymentConfig)

    @classmethod
    def default(cls) -> "AppConfig":
        """Create default application configuration."""
        return cls()

    @classmethod
    def from_dict(cls, config_dict: Dict) -> "AppConfig":
        """Create configuration from dictionary."""
        network_cfg = config_dict.get("network", {})
        gui_cfg = config_dict.get("gui", {})
        vehicle_cfg = config_dict.get("vehicle", {})
        manual_cfg = config_dict.get("manual_control", {})
        deployment_cfg = config_dict.get("deployment", {})

        return cls(
            network=NetworkConfig(**network_cfg) if network_cfg else NetworkConfig(),
            gui=GUIConfig(**gui_cfg) if gui_cfg else GUIConfig(),
            vehicle=VehicleConfig(**vehicle_cfg) if vehicle_cfg else VehicleConfig(),
            manual_control=ManualControlConfig(**manual_cfg)
            if manual_cfg
            else ManualControlConfig(),
            deployment=VehicleDeploymentConfig(**deployment_cfg)
            if deployment_cfg
            else VehicleDeploymentConfig(),
        )


# Telemetry field definitions for display
TELEMETRY_FIELDS = [
    ("position", "Pos:", "(0.0, 0.0)", 9),
    ("velocity", "Vel:", "0.00", 4),
    ("heading", "Heading:", "0.00", 4),
    ("state", "State:", "Unknown", 15),
    ("path_long_ctrl", "Path Long Ctrl:", "unk", 6),
    ("path_lat_ctrl", "Path Lat Ctrl:", "unk", 8),
    ("leader_long_ctrl", "Leader Long Ctrl:", "unk", 6),
    ("leader_lat_ctrl", "Leader Lat Ctrl:", "unk", 8),
    ("throttle", "Throttle:", "0.00", 4),
    ("steering", "Steering:", "0.00", 4),
    ("perception", "Perception:", "OFF", 3),
]


# Default path configurations per car
DEFAULT_PATHS = {0: "10, 2, 4, 6, 8, 10", "default": "10, 2, 4, 6, 8, 10"}
