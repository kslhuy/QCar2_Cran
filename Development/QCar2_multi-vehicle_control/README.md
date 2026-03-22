


# [Simulation Environment and Vehicle Instance Initialization Layer](Development\QCar2_multi-vehicle_control)
  
## Connecting to and Resetting the QLabs/QCar2 Simulation Environment

By running the [initCar_new.py](Development/multi_vehicle_self_driving_RealQcar/initCar_new.py) script, users can connect to the QLabs simulation environment and reset QCar2 vehicle instances. This step ensures the correct initialization of the simulation environment and provides a foundation for subsequent vehicle control and autonomous driving logic. During startup, the script performs the following operations:
1. Connect to the QLabs simulation environment and establish the localhost communication channel;
2. Reset QCar2 vehicle instances to ensure all vehicles are in their initial state and ready to receive control commands;
3. Terminate all real-time models to avoid conflicts from lingering old models and scenes.

## Establishing or Switching the Simulation Traffic Environment
Construct simulation environments including walls, roads, and traffic signs, or switch to predefined simulation scenarios. This step provides vehicles with a realistic driving environment, enabling testing and validation under various traffic conditions. Users can select different scenarios to evaluate vehicle performance and adaptability.

## Traffic Light Sequence Control
A background thread periodically switches traffic light states to simulate real-world traffic light changes. This functionality enables vehicles to make decisions and execute control under different traffic light conditions, enhancing system practicality and robustness.

## Vehicle Spawning Configuration Management (Configuration File First, Hardcoded Fallback)
The system prioritizes reading the 'fleet_config.yaml' file to construct initial vehicle poses; if this fails, it falls back to default vehicle definitions.

## Human-Computer Interaction and Visualization Support
Provides free camera perspective, vehicle LED differentiation, and manual pose input commands (default/skip/quit/shut down models/toggle environment) for rapid alignment of experimental starting points.

## Multi-Vehicle System Initialization
The [initPlatoon.py](Development/multi_vehicle_self_driving_RealQcar/initPlatoon.py) script is responsible for initializing the multi-vehicle system. 