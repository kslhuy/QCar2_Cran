# QCar Multi-Vehicle Management System



## Introduction

This project, based on the Quanser Self-Driving platform, developed a multi-vehicle control system management platform to achieve real-time control, monitoring, and communication of multiple autonomous QCar vehicles. The system supports seamless integration and expansion in simulation environments and real-world deployments, providing a comprehensive framework for managing and controlling a fleet of QCar vehicles. 

The project is structured into two main components: the Multi-Vehicle System and the Ground Station App. The Multi-Vehicle System focuses on the self-driving logic for real QCar hardware and simulation, while the Ground Station App provides a web-based monitoring dashboard. The more detailed about this two components will be described in the following sections.

## Architecture & Modules

### [Multi-Vehicle System](Development/multi_vehicle_self_driving_RealQcar)

[![Watch the demo video](https://img.youtube.com/vi/amur0EEShJE/hqdefault.jpg)](https://youtu.be/amur0EEShJE?si=yLor6u1dGCNiYwXE)

[![Watch the demo video](https://img.youtube.com/vi/n3M1J4P6dlQ/hqdefault.jpg)](https://youtu.be/n3M1J4P6dlQ?si=yLor6u1dGCNiYwXE)

Multi-Vehicle System is the core component of the project, responsible for implementing the self-driving logic for both real QCar hardware and simulation. It includes modules for vehicle control, state estimation, path planning, and communication between vehicles. The system is designed to be modular and scalable, allowing for easy integration of new algorithms and functionalities as needed.

The structure of the Multi-Vehicle System is organized as follows:

#### [Simulation Environment and Vehicle Instance Initialization Layer](Development\QCar2_multi-vehicle_control\README.md) 

#### [Vehicle Control Layer](Development/multi_vehicle_self_driving_RealQcar/qcar/README.md)

This layer is the runtime core of each vehicle. It integrates perception, control, state machine logic, V2V communication, Ground Station communication, safety checks, and asynchronous logging into one coherent execution pipeline.

Key entry points:
- [vehicle_main.py](Development/multi_vehicle_self_driving_RealQcar/qcar/vehicle_main.py): CLI entry point and system bootstrap.
- [vehicle_logic.py](Development/multi_vehicle_self_driving_RealQcar/qcar/vehicle_logic.py): Main orchestration loop for sensing, estimation, control, communication, and monitoring.

Main structure and responsibilities:

| Layer | Main files | Core function |
|---|---|---|
| Startup and Configuration | [vehicle_main.py](Development/multi_vehicle_self_driving_RealQcar/qcar/vehicle_main.py), [config_main.py](Development/multi_vehicle_self_driving_RealQcar/qcar/config_main.py), [config_vehicle_main.yaml](Development/multi_vehicle_self_driving_RealQcar/qcar/config_vehicle_main.yaml) | Parse arguments, load per-vehicle/fleet configuration, initialize runtime context |
| Orchestration Loop | [vehicle_logic.py](Development/multi_vehicle_self_driving_RealQcar/qcar/vehicle_logic.py) | Run periodic updates for sensors, observers, controllers, V2V, telemetry, and command handling |
| State Management | [StateMachine/vehicle_state_machine.py](Development/multi_vehicle_self_driving_RealQcar/qcar/StateMachine/vehicle_state_machine.py) | Manage transitions between initialization, waiting, path following, platoon following, manual mode, and stopped states |
| Controllers | [Controller/controller_manager.py](Development/multi_vehicle_self_driving_RealQcar/qcar/Controller/controller_manager.py), [Controller/README.md](Development/multi_vehicle_self_driving_RealQcar/qcar/Controller/README.md) | Longitudinal/lateral control and platoon control strategy selection |
| Observer and Estimation | [Observer/VehicleObserverSimple.py](Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/VehicleObserverSimple.py), [Observer/local_state_estimators.py](Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/local_state_estimators.py), [Observer/fleet_state_estimators.py](Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/fleet_state_estimators.py) | Local and fleet state estimation, including distributed observer support |
| Communication | [ground_station_client.py](Development/multi_vehicle_self_driving_RealQcar/qcar/ground_station_client.py), [command_handler.py](Development/multi_vehicle_self_driving_RealQcar/qcar/command_handler.py), [command_types.py](Development/multi_vehicle_self_driving_RealQcar/qcar/command_types.py), [V2V/v2v_manager.py](Development/multi_vehicle_self_driving_RealQcar/qcar/V2V/v2v_manager.py) | Ground Station TCP communication, command dispatching, and V2V broadcast/receive |
| Safety and Reliability | [safety.py](Development/multi_vehicle_self_driving_RealQcar/qcar/safety.py) | Control validation, sensor health checks, watchdog timeout protection |
| Logging and Offline Analysis | [logging_utils.py](Development/multi_vehicle_self_driving_RealQcar/qcar/logging_utils.py), [analyze_estimations.py](Development/multi_vehicle_self_driving_RealQcar/qcar/analyze_estimations.py), [ARCHITECTURE_DIAGRAM.md](Development/multi_vehicle_self_driving_RealQcar/qcar/ARCHITECTURE_DIAGRAM.md) | High-rate asynchronous data logging and post-run estimation/latency analysis |


### [Ground Station App](Development/ground_station_app)
[![Watch the demo video](https://img.youtube.com/vi/SraIErZ5QTg/hqdefault.jpg)](https://youtu.be/SraIErZ5QTg?si=rBwIPNAtXIAl1Any
)



## Pre-requisites

### Hardware Requirements

- Quanser QCar 2 or Limo Agilx (Can ez extend);
- Computer with at least 8GB RAM and a modern multi-core processor;
- High-speed internet connection for communication between vehicles and ground station.

### Software Requirements

- Python 3.13 or higher;
- Quanser QCar SDK;
- Quanser Interactive Labs (Qlabs) for simulation;

Note: For the more detailed software setup instructions, please refer to the [Installation Guide](https://qlabs.quanserdocs.com/en/latest/Get%20Started.html).

## Quick Start Guide

To get started with the QCar Multi-Vehicle Control System, please follow the instructions in the [For User Guide](Development/multi_vehicle_self_driving_RealQcar/ForUser.md) for both real QCar and simulation environments. This guide provides step-by-step instructions for setting up and running the system in different scenarios.

## Contributing

This project is supported  by the ANR agency under the project ArtISMo ANR-20-CE48-0015. 

We thanks for the following contributors for their valuable contributions to the project:

- [Huy Quang NGUYEN](https://github.com/kslhuy)
- [Shengya MENG](https://github.com/shengyameng)
- [Qi Li](https://github.com/shengyameng)
- [Shiyu Zhou](https://github.com)
- [Viet Cong Xuan](https://github.com)
- [Oussama Sadki](https://github.com)
- [Selsabil Bougherara](https://github.com)
- [Tanush Rathakumar](https://github.com)
- [Melissa Benabdelaziz](https://github.com)

- [Ali ZEMOUCHE](http://w3.cran.univ-lorraine.fr/ali.zemouche/?q=content/curriculum-vitae)


