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
    host_ip: str = '0.0.0.0'
    base_port: int = 5000
    buffer_size: int = 4096
    connection_timeout: float = 30.0
    telemetry_buffer_size: int = 100


@dataclass(frozen=True)
class GUIConfig:
    """GUI configuration settings."""
    window_title: str = "🚗 QCar Fleet Controller"
    window_width: int = 1400
    window_height: int = 900
    update_rate_hz: float = 20.0
    max_cars: int = 10
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
    forward_key: str = 'w'
    backward_key: str = 's'
    left_key: str = 'a'
    right_key: str = 'd'
    stop_key: str = 'space'
    
    # Steering wheel (pygame axis indices)
    steering_axis: int = 0
    accelerator_axis: int = 5
    brake_axis: int = 4
    steering_scale: float = 0.5
    throttle_scale: float = 0.3
    deadzone: float = 0.05


@dataclass
class AppConfig:
    """Main application configuration container."""
    network: NetworkConfig = field(default_factory=NetworkConfig)
    gui: GUIConfig = field(default_factory=GUIConfig)
    vehicle: VehicleConfig = field(default_factory=VehicleConfig)
    manual_control: ManualControlConfig = field(default_factory=ManualControlConfig)
    
    @classmethod
    def default(cls) -> 'AppConfig':
        """Create default application configuration."""
        return cls()
    
    @classmethod
    def from_dict(cls, config_dict: Dict) -> 'AppConfig':
        """Create configuration from dictionary."""
        network_cfg = config_dict.get('network', {})
        gui_cfg = config_dict.get('gui', {})
        vehicle_cfg = config_dict.get('vehicle', {})
        manual_cfg = config_dict.get('manual_control', {})
        
        return cls(
            network=NetworkConfig(**network_cfg) if network_cfg else NetworkConfig(),
            gui=GUIConfig(**gui_cfg) if gui_cfg else GUIConfig(),
            vehicle=VehicleConfig(**vehicle_cfg) if vehicle_cfg else VehicleConfig(),
            manual_control=ManualControlConfig(**manual_cfg) if manual_cfg else ManualControlConfig()
        )


# Telemetry field definitions for display
TELEMETRY_FIELDS = [
    ('position', 'Position (m):', '(0.00, 0.00)', 14),
    ('velocity', 'Velocity (m/s):', '0.00', 8),
    ('heading', 'Heading (rad):', '0.00', 8),
    ('state', 'Vehicle State:', 'Unknown', 15),
    ('longitudinal_ctrl_type', 'Long Ctrl:', 'Unknown', 10),
    ('lateral_ctrl_type', 'Lat Ctrl:', 'Unknown', 15),
    ('throttle', 'Throttle:', '0.00', 8),
    ('steering', 'Steering:', '0.00', 8),
]


# Default path configurations per car
DEFAULT_PATHS = {
    0: "10, 2, 4, 6, 8, 10",
    'default': "10, 2, 4, 6, 8, 10"
}
