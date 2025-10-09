import numpy as np

class DummyController:
    """
    Parameter container and interface adapter for CACC/IDM controllers.
    Provides the expected interface that the controllers need.
    """
    def __init__(self, qcar_id, config_params=None):
        """
        Initialize DummyController with configurable parameters.
        
        Args:
            qcar_id: ID of the QCar
            config_params: Dictionary of configuration parameters. If None, uses default values.
        """
        self.vehicle_id = qcar_id
        
        # Default parameters
        default_params = {
            'alpha': 1.0,
            'beta': 1.5,
            'v0': 1.0,
            'delta': 4,
            'T': 0.4,
            's0': 7,  # for IDM
            'ri': 7,  # for CACC (spacing policy)
            'hi': 0.5,
            'K': np.array([[1, 0.0], [0.0, 1]])  # for CACC gain matrix
        }
        
        if config_params is None:
            self.param_opt = default_params
        else:
            # Merge provided params with defaults (provided params override defaults)
            self.param_opt = {**default_params, **config_params}
        
        # Required by controllers but not used in our implementation
        self.param_sys = None
        self.goal = None
        self.straightlane = None
    
    def update_parameters(self, new_params):
        """Update controller parameters dynamically."""
        self.param_opt.update(new_params)
    
    def get_surrounding_vehicles(self, *args, **kwargs):
        """
        Required by IDM controller but overridden in VehicleFollowerController.
        Returns empty surrounding vehicles by default.
        """
        return None, [], None, None
    
class DummyVehicle:
     def __init__(self, state, vehicle_id=0):
            self.state = state
            self.vehicle_id = vehicle_id
