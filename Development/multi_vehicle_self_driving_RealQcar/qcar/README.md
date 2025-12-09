# QCar Vehicle Control System

Core vehicle control modules for autonomous multi-vehicle operation.

## Main Components

- **vehicle_main.py** - Main entry point for vehicle control with CLI arguments
- **vehicle_logic.py** - Core logic and state machine implementation
- **vehicle_control.py** - (exampale of Quanser)
- **command_handler.py** / **command_types.py** - Command processing for Ground Station (GUI)
- **config.py** - Configuration management to load *config_vehicle_main.py* for genral Vehicle System
- **safety.py** - Safety systems and collision avoidance
- **ground_station_client.py** - Communication with Ground Station (GUI)

## Subdirectories
**Controller/** - Control algorithms (PID, Stanley, IDM, platoon control)  
**StateMachine/** - Vehicle state management  
**Observer/** - State estimation  
**V2V/** - Vehicle-to-vehicle communication  
**GUI/** - Graphical interfaces
