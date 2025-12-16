"""
Configuration Management for QCar Vehicle Control System
"""
from dataclasses import dataclass, field
from typing import List, Optional
import json
import yaml
import numpy as np


@dataclass
class TimingConfig:
    """Timing and rate parameters"""
    tf: float = 6000  # Experiment duration in seconds
    start_delay: float = 1  # Delay before starting control
    controller_update_rate: int = 200  # Hz
    observer_rate: int = 200  # Hz
    telemetry_send_rate: int = 10  # Hz


@dataclass
class YOLODetectionConfig:
    """YOLO detection thresholds"""
    stop_sign_threshold: float = 0.6
    traffic_threshold: float = 1.7
    car_threshold: float = 0.3
    yield_threshold: float = 1.0
    person_threshold: float = 0.6
    pulse_length_multiplier: int = 3  # multiplied by controller update rate


@dataclass
class NetworkConfig:
    """Network communication parameters"""
    host_ip: Optional[str] = None
    base_port: int = 5000
    car_id: int = 0
    connection_timeout: int = 15  # Increased from 5 to 15 seconds
    max_reconnect_attempts: int = 5  # Reduced from 10 to 5
    reconnect_delay: float = 3.0  # Increased from 2.0 to 3.0 seconds
    telemetry_buffer_size: int = 100
    
    @property
    def is_remote_enabled(self) -> bool:
        return self.host_ip is not None
    
    # Auto compute port based on car_id
    @property
    def port(self) -> int:
        return self.base_port + self.car_id


@dataclass
class PathPlanningConfig:
    """Path planning parameters"""
    node_configuration: int = 0
    calibration_pose: List[float] = field(default_factory=lambda: [0, 2, -np.pi/2])
    calibrate: bool = False
    left_hand_traffic: bool = False
    
    @property
    def valid_nodes(self) -> List[int]:
        if self.node_configuration == 0:
            return [0, 2, 4, 6, 8, 10 , 2]
        else:
            return [0, 2, 4, 6, 8, 10 , 2]
        # if self.node_configuration == 0:
        #     return [10, 2, 4, 6, 8, 10]
        # else:
        #     return [10, 2, 4, 6, 8, 10]


@dataclass
class SafetyConfig:
    """Safety and monitoring parameters"""
    gps_timeout_max: int = 100
    max_loop_time_warning: float = 0.02  # seconds (20ms - allows for state transitions)
    emergency_stop_distance: float = 0.2  # meters
    watchdog_timeout: float = 1.0  # seconds


@dataclass
class LoggingConfig:
    """Logging configuration"""
    log_dir: str = "logs"
    data_log_dir: str = "data_logs"
    enable_telemetry_logging: bool = True
    enable_state_logging: bool = True
    log_level: str = "INFO"
    console_output: bool = True


@dataclass
class ObserverConfig:
    """
    Observer configuration block.
    This is optional in external YAML/JSON; defaults are provided here so that
    config.observer always exists and downstream code can safely read it.
    """
    # Update rates (Hz)
    observer_rate: int = 100
    fleet_observer_rate: int = 50

    # Selector for estimator implementations
    local_estimator_type: str = "ekf"  # ekf, luenberger, dead_reckoning
    fleet_estimator_type: str = "consensus"  # consensus, distributed_kalman, distributed_luenberger

    # Enable/disable distributed observation (used by VehicleObserverSimple)
    enable_distributed: bool = True

    # Gains can be scalar, vector, or matrix (parsed later into numpy arrays)
    consensus_gain: object = 0.3
    observer_gain: object = 0.1


@dataclass
class VehicleMainConfig:
    """Main configuration container"""
    timing: TimingConfig = field(default_factory=TimingConfig)
    yolo: YOLODetectionConfig = field(default_factory=YOLODetectionConfig)
    network: NetworkConfig = field(default_factory=NetworkConfig)
    path: PathPlanningConfig = field(default_factory=PathPlanningConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    observer: ObserverConfig = field(default_factory=ObserverConfig)
    
    @classmethod
    def from_dict(cls, config_dict: dict) -> 'VehicleMainConfig':
        """Create config from dictionary"""
        return cls(
            timing=TimingConfig(**config_dict.get('timing', {})),
            yolo=YOLODetectionConfig(**config_dict.get('yolo', {})),
            network=NetworkConfig(**config_dict.get('network', {})),
            path=PathPlanningConfig(**config_dict.get('path', {})),
            safety=SafetyConfig(**config_dict.get('safety', {})),
            logging=LoggingConfig(**config_dict.get('logging', {})),
            observer=ObserverConfig(**config_dict.get('observer', {}))
        )
    
    @classmethod
    def from_json(cls, filepath: str) -> 'VehicleMainConfig':
        """Load configuration from JSON file"""
        with open(filepath, 'r') as f:
            config_dict = json.load(f)
        return cls.from_dict(config_dict)
    
    @classmethod
    def from_yaml(cls, filepath: str) -> 'VehicleMainConfig':
        """Load configuration from YAML file"""
        with open(filepath, 'r') as f:
            config_dict = yaml.safe_load(f)
        return cls.from_dict(config_dict)
    
    def to_dict(self) -> dict:
        """Convert config to dictionary"""
        return {
            'timing': self.timing.__dict__,
            'yolo': self.yolo.__dict__,
            'network': self.network.__dict__,
            'path': self.path.__dict__,
            'safety': self.safety.__dict__,
            'logging': self.logging.__dict__,
            'observer': self.observer.__dict__,
        }
    
    def to_json(self, filepath: str):
        """Save configuration to JSON file"""
        with open(filepath, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)
    
    def to_yaml(self, filepath: str):
        """Save configuration to YAML file"""
        with open(filepath, 'w') as f:
            yaml.dump(self.to_dict(), f, default_flow_style=False)
    
    def update_from_args(self, args):
        """Update configuration from command line arguments"""
        if hasattr(args, 'calibrate'):
            self.path.calibrate = args.calibrate
        if hasattr(args, 'node_configuration'):
            self.path.node_configuration = args.node_configuration
        if hasattr(args, 'host') and args.host is not None:
            self.network.host_ip = args.host
        if hasattr(args, 'port'):
            self.network.base_port = args.port
        if hasattr(args, 'car_id'):
            self.network.car_id = args.car_id
