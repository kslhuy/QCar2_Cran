"""
Simple Configuration Loader for QCar Fleet Framework
Loads configuration from YAML file and provides easy access to parameters.
"""

import yaml
import os
import numpy as np
from typing import Dict, Any, Optional


class SimpleFleetConfig:
    """
    Simple configuration loader that reads from YAML file.
    Much easier to modify than the complex FleetConfig class.
    """
    
    # Define preset configurations
    PRESETS = {
        'openroad_cacc': {
            'simulation': {
                'road_type': 'OpenRoad',
                'controller_type': 'CACC'
            },
            'control': {
                'max_velocity': 2.0,
                'lookahead_distance': 7.0,
                'enable_steering_control': False  # OpenRoad is straight, no steering needed
            },
            'path': {
                'node_sequence': [0, 1]
            },
            'controller_params': {
                'alpha': 1.0,
                'beta': 1.5,
                'v0': 1.0,
                'delta': 4,
                'T': 0.4,
                's0': 7,
                'ri': 7,
                'hi': 0.5,
                'K_gains': [1.0, 0.0, 0.0, 1.0]
            }
        },
        'studio_cacc': {
            'simulation': {
                'road_type': 'Studio',
                'controller_type': 'CACC'
            },
            'control': {
                'max_velocity': 0.5,
                'lookahead_distance': 0.5,
                'enable_steering_control': True  # Studio has curves, steering needed
            },
            'path': {
                'node_sequence': [10, 4, 20, 13, 10]
            },
            'controller_params': {
                'alpha': 1.2,
                'beta': 2.0,
                'v0': 0.5,
                'delta': 3,
                'T': 0.3,
                's0': 1,
                'ri': 1,
                'hi': 0.3,
                'K_gains': [1.1, 0.0, 0.0, 1.1]
            }
        }
    }
    
    def __init__(self, config_file: str = "config.yaml"):
        """
        Initialize configuration from YAML file.
        
        Args:
            config_file: Path to YAML configuration file
        """
        self.config_file = config_file
        self.config_data = {}
        self.load_config()
        self._apply_preset_if_specified()
    
    def load_config(self):
        """Load configuration from YAML file."""
        try:
            # Try to load from current directory first
            if os.path.exists(self.config_file):
                config_path = self.config_file
            else:
                # Try to load from the same directory as this script
                script_dir = os.path.dirname(__file__)
                config_path = os.path.join(script_dir, self.config_file)
            
            with open(config_path, 'r') as file:
                self.config_data = yaml.safe_load(file)
            
            print(f"Configuration loaded from: {config_path}")
            
        except FileNotFoundError:
            print(f"Warning: Configuration file '{self.config_file}' not found. Using default values.")
            self._load_default_config()
        except yaml.YAMLError as e:
            print(f"Error parsing YAML configuration: {e}")
            print("Using default configuration.")
            self._load_default_config()
    
    def _load_default_config(self):
        """Load default configuration if YAML file is not available."""
        self.config_data = {
            'simulation': {
                'time': 40,
                'road_type': 'Studio',
                'controller_type': 'CACC'
            },
            'fleet': {
                'num_vehicles': 2,
                'leader_index': 0,
                'distance_between_cars': 0.2
            },
            'control': {
                'max_velocity': 0.5,
                'max_steering': 0.6,
                'lookahead_distance': 0.5,
                'enable_steering_control': True,
                'update_rate': 100,
                'observer_rate': 100,
                'gps_update_rate': 50
            },
            'path': {
                'rebuild_path': True,
                'node_sequence': [10, 4, 20, 13, 10]
            },
            'controller_params': {
                'alpha': 1.2,
                'beta': 2.0,
                'v0': 0.5,
                'delta': 3,
                'T': 0.3,
                's0': 1,
                'ri': 1,
                'hi': 0.3,
                'K_gains': [1.1, 0.0, 0.0, 1.1]
            },
            'communication': {
                'use_communication': True,
                'communication_delay': 0.01,
                'gps_server_ip': '127.0.0.1',
                'gps_server_port': 8001
            },
            'physical_qcar': {
                'use_physical_qcar': False,
                'calibrate': False
            },
            'logging': {
                'enable_performance_monitoring': False,
                'log_level': 'INFO'
            },
            'enable_cam_lidar_fusion': False,
            'enable_visualization_camera': False,
            'enable_visualization_lidar': False
        }
    
    def _apply_preset_if_specified(self):
        """Apply preset configuration if 'active_preset' is specified in config."""
        active_preset = self.config_data.get('active_preset')
        
        if active_preset:
            if active_preset in self.PRESETS:
                print(f"Applying preset: {active_preset}")
                self._merge_preset(self.PRESETS[active_preset])
            else:
                available = ', '.join(self.PRESETS.keys())
                print(f"Warning: Unknown preset '{active_preset}'. Available presets: {available}")
    
    def _merge_preset(self, preset_data: Dict[str, Any]):
        """
        Merge preset configuration into current config.
        Preset values override existing values.
        """
        for section, values in preset_data.items():
            if section not in self.config_data:
                self.config_data[section] = {}
            
            if isinstance(values, dict):
                self.config_data[section].update(values)
            else:
                self.config_data[section] = values
    
    def apply_preset(self, preset_name: str):
        """
        Apply a preset configuration.
        
        Args:
            preset_name: Name of preset ('openroad_cacc', 'studio_cacc')
        """
        if preset_name in self.PRESETS:
            print(f"Applying preset: {preset_name}")
            self._merge_preset(self.PRESETS[preset_name])
            self.config_data['active_preset'] = preset_name
        else:
            available = ', '.join(self.PRESETS.keys())
            raise ValueError(f"Unknown preset '{preset_name}'. Available: {available}")
    
    def list_presets(self):
        """List all available presets."""
        print("Available Presets:")
        for preset_name, preset_data in self.PRESETS.items():
            road_type = preset_data.get('simulation', {}).get('road_type', 'N/A')
            controller = preset_data.get('simulation', {}).get('controller_type', 'N/A')
            print(f"  - {preset_name}: {road_type} with {controller}")
    
    def get(self, section: str, key: str, default: Any = None) -> Any:
        """
        Get a configuration value.
        
        Args:
            section: Configuration section (e.g., 'simulation', 'fleet')
            key: Configuration key within the section
            default: Default value if key not found
            
        Returns:
            Configuration value or default
        """
        try:
            return self.config_data[section][key]
        except KeyError:
            return default
    
    def get_section(self, section: str) -> Dict[str, Any]:
        """
        Get an entire configuration section.
        
        Args:
            section: Configuration section name
            
        Returns:
            Dictionary containing all keys in the section
        """
        return self.config_data.get(section, {})
    
    # =============================================================================
    # EASY ACCESS PROPERTIES - Compatible with existing FleetConfig interface
    # =============================================================================
    
    @property
    def simulation_time(self) -> int:
        return self.get('simulation', 'time', 40)
    
    @property
    def road_type(self) -> str:
        return self.get('simulation', 'road_type', 'Studio')
    
    @property
    def controller_type(self) -> str:
        return self.get('simulation', 'controller_type', 'CACC')
    
    @property
    def qcar_num(self) -> int:
        return self.get('fleet', 'num_vehicles', 2)
    
    @property
    def leader_index(self) -> int:
        return self.get('fleet', 'leader_index', 0)
    
    @property
    def distance_between_cars(self) -> float:
        return self.get('fleet', 'distance_between_cars', 0.2)
    
    @property
    def max_velocity(self) -> float:
        return self.get('control', 'max_velocity', 0.5)
    
    @property
    def max_steering(self) -> float:
        return self.get('control', 'max_steering', 0.6)
    
    @property
    def lookahead_distance(self) -> float:
        return self.get('control', 'lookahead_distance', 0.5)
    
    @property
    def enable_steering_control(self) -> bool:
        return self.get('control', 'enable_steering_control', True)
    
    @property
    def flag_path_rebuild(self) -> bool:
        return self.get('path', 'rebuild_path', True)
    
    @property
    def node_sequence(self) -> list:
        return self.get('path', 'node_sequence', [10, 4, 20, 13, 10])
    
    @property
    def update_rate(self) -> int:
        return self.get('control', 'general_update_rate', 100)
    
    @property
    def control_rate(self) -> int:
        return self.get('control', 'controller_rate', 100)

    @property
    def observer_rate(self) -> int:
        return self.get('control', 'observer_rate', 100)
    
    @property
    def gps_update_rate(self) -> int:
        return self.get('control', 'gps_update_rate', 10)
    
    # =============================================================================
    # PHYSICAL QCAR PARAMETERS
    # =============================================================================
    
    @property
    def use_physical_qcar(self) -> bool:
        return self.get('physical_qcar', 'use_physical_qcar', False)
    
    @property
    def use_control_observer_mode(self) -> bool:
        return self.get('physical_qcar', 'use_control_observer_mode', False)
    
    @property
    def enable_cam_lidar_fusion(self) -> bool:
        return self.config_data.get('enable_cam_lidar_fusion', True)
    
    @property
    def enable_visualization_camera(self) -> bool:
        return self.config_data.get('enable_visualization_camera', True)
    
    @property
    def enable_visualization_lidar(self) -> bool:
        return self.config_data.get('enable_visualization_lidar', True)
    
    # =============================================================================
    # CONTROLLER PARAMETERS
    # =============================================================================
    
    @property
    def dummy_controller_params(self) -> Dict[str, Any]:
        """Get dummy controller parameters with K matrix properly formatted."""
        params = self.get_section('controller_params')
        
        # Convert K_gains list to numpy array
        k_gains = params.get('K_gains', [1.1, 0.0, 0.0, 1.1])
        if len(k_gains) == 4:
            params['K'] = np.array([[k_gains[0], k_gains[1]], 
                                   [k_gains[2], k_gains[3]]])
        else:
            params['K'] = np.array([[1.0, 0.0], [0.0, 1.0]])
        
        return params
    
    @property
    def controller_specific_params(self) -> Dict[str, Any]:
        """Get controller-specific parameters."""
        comm_section = self.get_section('communication')
        return {
            'control_mode': self.controller_type,
            'use_communication': comm_section.get('use_communication', True),
            'communication_delay': comm_section.get('communication_delay', 0.01)
        }
    
    # =============================================================================
    # INTERFACE COMPATIBILITY METHODS
    # =============================================================================
    
    def get_dummy_controller_params(self, qcar_id: int) -> Dict[str, Any]:
        """Get DummyController parameters for a specific QCar (compatible with old interface)."""
        params = self.dummy_controller_params.copy()
        params['vehicle_id'] = qcar_id
        return params
    
    def get_node_sequence(self) -> list:
        """Get the node sequence for the current road type."""
        return self.node_sequence
    
    def get_road_type_name(self) -> str:
        """Get the road type as string."""
        return self.road_type
    
    def get_controller_type_name(self) -> str:
        """Get the controller type as string."""
        return self.controller_type
    
    def update_config(self, **kwargs):
        """
        Update configuration parameters dynamically.
        
        Args:
            **kwargs: Configuration parameters to update
                     Use dot notation for nested keys: 'simulation.time', 'fleet.num_vehicles'
        """
        for key, value in kwargs.items():
            if '.' in key:
                # Handle nested keys like 'simulation.time'
                section, param = key.split('.', 1)
                if section in self.config_data and isinstance(self.config_data[section], dict):
                    self.config_data[section][param] = value
                    print(f"Updated {section}.{param} = {value}")
                else:
                    print(f"Warning: Unknown configuration section: {section}")
            else:
                # Handle direct keys in controller_params
                if key in self.config_data.get('controller_params', {}):
                    self.config_data['controller_params'][key] = value
                    print(f"Updated controller_params.{key} = {value}")
                else:
                    print(f"Warning: Unknown configuration parameter: {key}")
    
    def print_config(self):
        """Print current configuration in a readable format."""
        print("=" * 60)
        print("FLEET CONFIGURATION")
        print("=" * 60)
        
        # Show active preset if any
        active_preset = self.config_data.get('active_preset')
        if active_preset:
            print(f"Active Preset: {active_preset.upper()}")
            print("-" * 60)
        
        print(f"Road Type: {self.road_type}")
        print(f"Controller Type: {self.controller_type}")
        print(f"Simulation Time: {self.simulation_time} seconds")
        print(f"Number of QCars: {self.qcar_num}")
        print(f"Leader Index: {self.leader_index}")
        print(f"Distance Between Cars: {self.distance_between_cars} m")
        
        print("\nCONTROL PARAMETERS:")
        print(f"  Max Velocity: {self.max_velocity} m/s")
        print(f"  Max Steering: {self.max_steering} rad")
        print(f"  Lookahead Distance: {self.lookahead_distance} m")
        print(f"  Steering Control: {self.enable_steering_control}")
        
        print("\nPATH SETTINGS:")
        print(f"  Node Sequence: {self.node_sequence}")
        print(f"  Rebuild Path: {self.flag_path_rebuild}")
        
        print("\nCONTROLLER PARAMETERS:")
        params = self.dummy_controller_params
        for key, value in params.items():
            if key != 'K':  # Skip K matrix for cleaner output
                print(f"  {key}: {value}")
        
        print("\nUPDATE RATES:")
        print(f"  Control: {self.update_rate} Hz")
        print(f"  Observer: {self.observer_rate} Hz")
        print(f"  GPS: {self.gps_update_rate} Hz")
        
        print("=" * 60)
    
    def save_config(self, filename: Optional[str] = None):
        """
        Save current configuration to YAML file.
        
        Args:
            filename: Optional filename to save to. If None, uses original config file.
        """
        save_path = filename or self.config_file
        
        try:
            with open(save_path, 'w') as file:
                yaml.dump(self.config_data, file, default_flow_style=False, indent=2)
            print(f"Configuration saved to: {save_path}")
        except Exception as e:
            print(f"Error saving configuration: {e}")


# =============================================================================
# EASY CONFIGURATION PRESETS
# =============================================================================

class ConfigPresets:
    """Easy-to-use configuration presets."""
    
    @staticmethod
    def load_from_file(config_file: str = "config.yaml") -> SimpleFleetConfig:
        """Load configuration from YAML file."""
        return SimpleFleetConfig(config_file)
    
    @staticmethod
    def create_openroad_cacc() -> SimpleFleetConfig:
        """Create OpenRoad CACC configuration."""
        config = SimpleFleetConfig()
        config.update_config(**{
            'simulation.road_type': 'OpenRoad',
            'simulation.controller_type': 'CACC',
            'control.max_velocity': 2.0,
            'control.lookahead_distance': 7.0,
            'path.node_sequence': [0, 1],
            'alpha': 1.0,
            'beta': 1.5,
            'v0': 1.0,
            'delta': 4,
            'T': 0.4,
            's0': 7,
            'ri': 7,
            'hi': 0.5
        })
        return config
    
    @staticmethod
    def create_studio_cacc() -> SimpleFleetConfig:
        """Create Studio CACC configuration."""
        config = SimpleFleetConfig()
        config.update_config(**{
            'simulation.road_type': 'Studio',
            'simulation.controller_type': 'CACC',
            'control.max_velocity': 0.5,
            'control.lookahead_distance': 0.5,
            'path.node_sequence': [10, 4, 20, 13, 10],
            'alpha': 1.2,
            'beta': 2.0,
            'v0': 0.5,
            'delta': 3,
            'T': 0.3,
            's0': 1,
            'ri': 1,
            'hi': 0.3
        })
        return config


# Example usage
if __name__ == "__main__":
    print("Testing Simple Fleet Configuration System")
    print("=" * 50)
    
    # Load from YAML file
    config = SimpleFleetConfig("config.yaml")
    config.print_config()
    
    print("\n" + "=" * 50)
    print("Testing configuration updates:")
    
    # Update some parameters
    config.update_config(**{
        'simulation.time': 60,
        'fleet.num_vehicles': 3,
        's0': 2.0,
        'ri': 2.0
    })
    
    config.print_config()
