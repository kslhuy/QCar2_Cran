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

    path_number: int = 0
    calibrate: bool = False
    left_hand_traffic: bool = False

    # Node 0: Pose = [0.0, 0.13024, -1.5707963267948966]
    # Node 1: Pose = [0.26861999999999997, 0.0814, 1.5707963267948966]
    # Node 2: Pose = [1.12739, -1.084655, 0.0]
    # Node 3: Pose = [1.12739, -0.814, 3.141592653589793]
    # Node 4: Pose = [2.25478, 0.0814, 1.5707963267948966]
    # Node 5: Pose = [1.984125, 0.0814, -1.5707963267948966]
    # Node 6: Pose = [1.01343, 1.100935, 3.141592653589793]
    # Node 7: Pose = [1.235245, 0.83028, 0.0]
    # Node 8: Pose = [-0.74888, 1.100935, 3.141592653589793]
    # Node 9: Pose = [-0.74888, 0.83028, 0.0]
    # Node 10: Pose = [-1.28205, -0.45991, -0.7330382858376184]

    @property
    def valid_nodes(self) -> List[int]:
        if self.path_number == 0:
            return [10, 2, 4, 6, 8, 10]
        elif self.path_number == 1:
            return [0, 2, 4, 6, 8, 10, 2, 4, 6, 0]
        elif self.path_number == 2:
            return [0, 2, 4, 6, 8, 10, 1, 10, 2, 4, 6, 0]
        elif self.path_number == 3:
            return [0, 2, 4, 6, 13, 19, 17, 15, 6, 0]

    @property
    def calibration_pose(self) -> List[float]:
        """Get calibration pose based on the first valid node"""
        node_poses = {
            0: [0.0, 0.13024, -1.5707963267948966],
            1: [0.26861999999999997, 0.0814, 1.5707963267948966],
            2: [1.12739, -1.084655, 0.0],
            3: [1.12739, -0.814, 3.141592653589793],
            4: [2.25478, 0.0814, 1.5707963267948966],
            5: [1.984125, 0.0814, -1.5707963267948966],
            6: [1.01343, 1.100935, 3.141592653589793],
            7: [1.235245, 0.83028, 0.0],
            8: [-0.74888, 1.100935, 3.141592653589793],
            9: [-0.74888, 0.83028, 0.0],
            10: [-1.28205, -0.45991, -0.7330382858376184],
        }
        first_node = self.valid_nodes[0]
        return node_poses.get(first_node, [0, 0, 0])


@dataclass
class VehicleConfig:
    """Vehicle-specific configuration"""

    vehicle_type: str = "Qcar"  # "Qcar" or "Limo"
    probing: bool = False  # Enable YOLO perception system

    def __post_init__(self):
        """Validate vehicle type"""
        valid_types = ["Qcar", "Limo"]
        if self.vehicle_type not in valid_types:
            raise ValueError(
                f"Invalid vehicle_type: {self.vehicle_type}. Must be one of {valid_types}"
            )


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

    # Granular logging control
    enable_fleet_estimation_logging: bool = True
    enable_local_estimation_logging: bool = True
    enable_following_leader_logging: bool = True
    enable_trust_weight_logging: bool = True

    # File naming strategy
    use_timestamped_log_files: bool = (
        True  # If False, uses static filenames (single file/append mode)
    )


@dataclass
class ObserverConfig:
    """
    Observer configuration block.
    This is optional in external YAML/JSON; defaults are provided here so that
    config.observer always exists and downstream code can safely read it.
    """

    # Update rates (Hz)
    observer_rate: int = 100
    fleet_observer_rate: int = 30


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
    vehicle: VehicleConfig = field(default_factory=VehicleConfig)

    @classmethod
    def from_dict(cls, config_dict: dict) -> "VehicleMainConfig":
        """Create config from dictionary"""
        return cls(
            timing=TimingConfig(**config_dict.get("timing", {})),
            yolo=YOLODetectionConfig(**config_dict.get("yolo", {})),
            network=NetworkConfig(**config_dict.get("network", {})),
            path=PathPlanningConfig(**config_dict.get("path", {})),
            safety=SafetyConfig(**config_dict.get("safety", {})),
            logging=LoggingConfig(**config_dict.get("logging", {})),
            vehicle=VehicleConfig(**config_dict.get("vehicle", {})),
            observer=ObserverConfig(**config_dict.get("observer", {})),
        )

    @classmethod
    def from_json(cls, filepath: str) -> "VehicleMainConfig":
        """Load configuration from JSON file"""
        with open(filepath, "r") as f:
            config_dict = json.load(f)
        return cls.from_dict(config_dict)

    @classmethod
    def from_yaml(cls, filepath: str) -> "VehicleMainConfig":
        """Load configuration from YAML file"""
        with open(filepath, "r") as f:
            config_dict = yaml.safe_load(f)
        return cls.from_dict(config_dict)

    @classmethod
    def from_fleet_yaml(cls, filepath: str, car_id: int) -> "VehicleMainConfig":
        """
        Load configuration from fleet_config.yaml for a specific vehicle.
        Extracts per-vehicle settings (probing, vehicle_type, path_number, etc.)
        from the 'vehicles' array and merges with global settings.
        """
        with open(filepath, "r") as f:
            fleet_config = yaml.safe_load(f)

        # Start with global settings from the fleet config
        config_dict = {
            "timing": fleet_config.get("timing", {}),
            "yolo": fleet_config.get("yolo", {}),
            "safety": fleet_config.get("safety", {}),
            "logging": fleet_config.get("logging", {}),
            "observer": fleet_config.get("observer", {}),
        }

        # Find the vehicle with matching car_id
        vehicles = fleet_config.get("vehicles", [])
        vehicle_entry = None
        for v in vehicles:
            if v.get("car_id") == car_id:
                vehicle_entry = v
                break

        # Build network config
        ground_station = fleet_config.get("ground_station", {})
        config_dict["network"] = {
            "host_ip": ground_station.get("local_ip"),
            "base_port": ground_station.get("base_port", 5000),
            "car_id": car_id,
        }

        # Build vehicle config from per-vehicle entry
        if vehicle_entry:
            config_dict["vehicle"] = {
                "vehicle_type": vehicle_entry.get("vehicle_type", "Qcar"),
                "probing": vehicle_entry.get("probing", False),
            }
            config_dict["path"] = {
                "path_number": vehicle_entry.get("path_number", 0),
                "calibrate": vehicle_entry.get("calibrate", False),
                "left_hand_traffic": vehicle_entry.get("left_hand_traffic", False),
            }
        else:
            # Use defaults if vehicle not found
            config_dict["vehicle"] = {}
            config_dict["path"] = {}

        return cls.from_dict(config_dict)

    def to_dict(self) -> dict:
        """Convert config to dictionary"""
        return {
            "timing": self.timing.__dict__,
            "yolo": self.yolo.__dict__,
            "network": self.network.__dict__,
            "path": self.path.__dict__,
            "safety": self.safety.__dict__,
            "logging": self.logging.__dict__,
            "vehicle": self.vehicle.__dict__,
            "observer": self.observer.__dict__,
        }

    def to_json(self, filepath: str):
        """Save configuration to JSON file"""
        with open(filepath, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    def to_yaml(self, filepath: str):
        """Save configuration to YAML file"""
        with open(filepath, "w") as f:
            yaml.dump(self.to_dict(), f, default_flow_style=False)

    def update_from_args(self, args):
        """Update configuration from command line arguments"""
        if getattr(args, "calibrate", False):
            self.path.calibrate = True
        if getattr(args, "path_number", None) is not None:
            self.path.path_number = args.path_number
        if getattr(args, "left_hand_traffic", False):
            self.path.left_hand_traffic = True
        if getattr(args, "host", None) is not None:
            self.network.host_ip = args.host
        if getattr(args, "port", None) is not None:
            self.network.base_port = args.port
        if getattr(args, "car_id", None) is not None:
            self.network.car_id = args.car_id
        if getattr(args, "vehicle_type", None) is not None:
            self.vehicle.vehicle_type = args.vehicle_type
        if getattr(args, "probing", False):
            self.vehicle.probing = True
