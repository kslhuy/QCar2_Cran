from omegaconf import DictConfig
from vehiclemodels.vehicle_parameters import setup_vehicle_parameters, VehicleParameters


def parameters_vehicle3() -> VehicleParameters:
    """
    Creates a VehicleParameters object holding all vehicle parameters for vehicle ID 3 (VW Vanagon)
    """
    return setup_vehicle_parameters(vehicle_id=3)


# Test parameters
if __name__ == "__main__":
    params = parameters_vehicle3()
