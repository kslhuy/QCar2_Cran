import numpy as np
from enum import Enum

class RoadType(Enum):
    OpenRoad = "OpenRoad"
    Studio = "Studio"

class ControllerType(Enum):
    CACC = "CACC"
    IDM = "IDM"

class FleetConfig:
    """
    Configuration class for Fleet simulation parameters.
    Manages different configurations based on RoadType and ControllerType.
    """
    
    def __init__(self, road_type: RoadType = RoadType.OpenRoad, controller_type: ControllerType = ControllerType.CACC):
        self.road_type = road_type
        self.controller_type = controller_type
        self._load_config()
    
    def _load_config(self):
        """Load configuration parameters based on road type and controller type."""
        
        # Base configuration
        self.simulation_time = 40
        self.leader_index = 0
        self.qcar_num = 2
        self.distance_between_cars = 0.2
        self.enable_steering_control = True
        self.flag_path_rebuild = True
        
        # Road-specific configurations
        if self.road_type == RoadType.OpenRoad:
            self._configure_openroad()
        elif self.road_type == RoadType.Studio:
            self._configure_studio()
        
        # Controller-specific configurations
        self._configure_controller_params()
    
    def _configure_openroad(self):
        """Configure parameters for OpenRoad environment."""
        self.node_sequence = [0, 1]
        # Alternative complex sequence: [10, 4, 20, 13, 10]
        
        # DummyController parameters for OpenRoad
        self.dummy_controller_params = {
            'alpha': 1.0,
            'beta': 1.5,
            'v0': 1.0,
            'delta': 4,
            'T': 0.4,
            's0': 7,  # Larger spacing for OpenRoad
            'ri': 7,  # Larger spacing for CACC
            'hi': 0.5,
            'K': np.array([[1, 0.0], [0.0, 1]])
        }
        
        # OpenRoad specific parameters
        self.max_velocity = 2.0
        self.lookahead_distance = 7.0
        self.max_steering = 0.6
        
    def _configure_studio(self):
        """Configure parameters for Studio environment."""
        self.node_sequence = [10, 4, 20, 13, 10]  # Studio loop sequence
        # Alternative complex sequence: [10, 4, 20, 13, 10]

        
        # DummyController parameters for Studio (smaller space, tighter control)
        self.dummy_controller_params = {
            'alpha': 1.2,
            'beta': 2.0,
            'v0': 0.5,  # Lower speed for studio
            'delta': 3,
            'T': 0.3,   # Faster response
            's0': 1,    # Smaller spacing for Studio
            'ri': 1,    # Smaller spacing for CACC
            'hi': 0.3,  # Shorter time headway
            'K': np.array([[1.1, 0.0], [0.0, 1.1]])  # Higher gains
        }
        
        # Studio specific parameters
        self.max_velocity = 0.5
        self.lookahead_distance = 0.5
        self.max_steering = 0.6
    
    def _configure_controller_params(self):
        """Configure controller-specific parameters."""
        if self.controller_type == ControllerType.CACC:
            # CACC uses 'ri' parameter more heavily
            self.controller_specific_params = {
                'control_mode': 'CACC',
                'use_communication': True,
                'communication_delay': 0.01
            }
        elif self.controller_type == ControllerType.IDM:
            # IDM uses 's0' parameter more heavily
            self.controller_specific_params = {
                'control_mode': 'IDM',
                'use_communication': False,
                'aggressive_factor': 1.0
            }
    
    def get_dummy_controller_params(self, qcar_id):
        """Get DummyController parameters for a specific QCar."""
        params = self.dummy_controller_params.copy()
        params['vehicle_id'] = qcar_id
        return params
    
    def get_node_sequence(self):
        """Get the node sequence for the current road type."""
        return self.node_sequence
    
    def get_road_type_name(self):
        """Get the road type as string."""
        return self.road_type.value
    
    def get_controller_type_name(self):
        """Get the controller type as string."""
        return self.controller_type.value
    
    def to_dict(self):
        """Convert config to dictionary for serialization."""
        return {
            'road_type': self.road_type.value,
            'controller_type': self.controller_type.value,
            'simulation_time': self.simulation_time,
            'leader_index': self.leader_index,
            'qcar_num': self.qcar_num,
            'distance_between_cars': self.distance_between_cars,
            'enable_steering_control': self.enable_steering_control,
            'flag_path_rebuild': self.flag_path_rebuild,
            'node_sequence': self.node_sequence,
            'dummy_controller_params': self.dummy_controller_params,
            'max_velocity': self.max_velocity,
            'lookahead_distance': self.lookahead_distance,
            'max_steering': self.max_steering,
            'controller_specific_params': getattr(self, 'controller_specific_params', {})
        }
    
    def update_config(self, **kwargs):
        """Update configuration parameters dynamically."""
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
            elif key in self.dummy_controller_params:
                self.dummy_controller_params[key] = value
            else:
                print(f"Warning: Unknown configuration parameter: {key}")
    
    def print_config(self):
        """Print current configuration."""
        print(f"=== Fleet Configuration ===")
        print(f"Road Type: {self.road_type.value}")
        print(f"Controller Type: {self.controller_type.value}")
        print(f"Node Sequence: {self.node_sequence}")
        print(f"Simulation Time: {self.simulation_time}")
        print(f"Number of QCars: {self.qcar_num}")
        print(f"Distance Between Cars: {self.distance_between_cars}")
        print(f"Max Velocity: {self.max_velocity}")
        print(f"Lookahead Distance: {self.lookahead_distance}")
        print(f"Max Steering: {self.max_steering}")
        print(f"DummyController Params: {self.dummy_controller_params}")
        print(f"Controller Specific Params: {self.controller_specific_params}")


# Predefined configurations
class ConfigPresets:
    """Predefined configuration presets for common scenarios."""
    
    @staticmethod
    def get_openroad_cacc():
        """OpenRoad with CACC controller."""
        return FleetConfig(RoadType.OpenRoad, ControllerType.CACC)
    
    @staticmethod
    def get_openroad_idm():
        """OpenRoad with IDM controller."""
        return FleetConfig(RoadType.OpenRoad, ControllerType.IDM)
    
    @staticmethod
    def get_studio_cacc():
        """Studio with CACC controller."""
        return FleetConfig(RoadType.Studio, ControllerType.CACC)
    
    @staticmethod
    def get_studio_idm():
        """Studio with IDM controller."""
        return FleetConfig(RoadType.Studio, ControllerType.IDM)
    
    @staticmethod
    def get_custom_config(road_type_str: str, controller_type_str: str):
        """Get custom configuration from strings."""
        road_type = RoadType(road_type_str)
        controller_type = ControllerType(controller_type_str)
        return FleetConfig(road_type, controller_type)


# Example usage and testing
if __name__ == "__main__":
    # Test different configurations
    print("Testing OpenRoad CACC:")
    config1 = ConfigPresets.get_openroad_cacc()
    config1.print_config()
    
    print("\n" + "="*50 + "\n")
    
    print("Testing Studio IDM:")
    config2 = ConfigPresets.get_studio_idm()
    config2.print_config()
    
    print("\n" + "="*50 + "\n")
    
    # Test dynamic updates
    print("Testing dynamic configuration update:")
    config1.update_config(simulation_time=60, s0=5, ri=5)
    config1.print_config()
