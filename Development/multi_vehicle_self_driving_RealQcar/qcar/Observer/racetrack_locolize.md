# Localization and Path Following System Overview

Based on the research into the codebase, here is how the system obtains its global position and follows a track path.

## 1. Localization Techniques and Packages

The system primarily uses **SLAM (Simultaneous Localization and Mapping)** for global positioning, with an alternative **Particle Filter** implementation.

### Key Packages:
- **[Google Cartographer](https://github.com/cartographer-project/cartographer)**: This is the default SLAM package used for both mapping and localization. It uses 2D LiDAR data to provide a consistent global pose.
  - Configuration files can be found in `stack_master/config/NUC2/slam/` (e.g., `f110_2d_loc.lua`).
  - Standard ROS services like `/start_trajectory` and `/finish_trajectory` are used to manage localization sessions.
- **Particle Filter**: A secondary localization method is available, which can be toggled via the `localization:=pf2` argument in the `base_system.launch` file. 
  - Documentation in `state_estimation/README.md` suggests that the Particle Filter might be preferred in environments with slippery floors or low-feature areas (like long straights).

## 2. Sensor Fusion (EKF)

To ensure smooth and accurate state estimation (especially velocity), the system employs an **Extended Kalman Filter (EKF)** using the `robot_localization` package.

- **Inputs**: Fuses data from the **VESC** (odometry/RPM) and the **IMU** (linear acceleration and angular velocity).
- **Output**: Provides a refined estimate of the car's state, which is published to topics like `/car_state/odom` and `/car_state/pose`.

## 3. Global Position to Track Path (Frenet Frame)

The transition from a global map position to following a specific track path is handled via a **Frenet Frame** transformation.

- **Technique**: Instead of just using Cartesian coordinates ($x, y$), the system converts the car's position into $s$ (progress along the track) and $n$ (lateral distance from the centerline).
- **Implementation**:
  - The `frenet_odom_republisher` node converts the standard odometry into Frenet coordinates.
  - The **MPC (Model Predictive Control)** controller subscribes to `/car_state/odom_frenet` and follows the `global_waypoints` (raceline) using these coordinates.
  - This makes the path-following logic much simpler as it essentially becomes a 1D tracking problem with lateral error correction.

## Summary Table

| Component | Package/Technique | Purpose |
| :--- | :--- | :--- |
| **SLAM / Loc** | Google Cartographer | Provides global position ($x, y, \theta$) using LiDAR. |
| **Loc (Alt)** | Particle Filter | Alternative localization for specific track conditions. |
| **Sensor Fusion** | `robot_localization` (EKF) | Fuses IMU and VESC data for smooth velocity estimation. |
| **Path Tracking** | Frenet Frame + MPC | Converts global pose to track-relative coordinates for control. |

You can find the main launch configuration in [base_system.launch](file:///c:/Users/Quang%20Huy%20Nugyen/Desktop/PHD_paper/Simulation/HUY_ALL_TEST/race_stack-main/race_stack-main/stack_master/launch/base_system.launch) and the state estimation details in [state_estimation/README.md](file:///c:/Users/Quang%20Huy%20Nugyen/Desktop/PHD_paper/Simulation/HUY_ALL_TEST/race_stack-main/race_stack-main/state_estimation/README.md).
