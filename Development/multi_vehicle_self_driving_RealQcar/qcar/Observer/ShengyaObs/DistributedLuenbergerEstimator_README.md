# DistributedLuenbergerEstimator

## Overview

The `DistributedLuenbergerEstimator` is a Python class designed for **distributed state estimation** of a connected vehicle fleet. It is part of the `FleetStateEstimator` family in the QCar framework.

This estimator allows each vehicle in a fleet (specifically "followers") to estimate the longitudinal states (position, velocity, acceleration) of **all other vehicles** in the fleet, relative to a "Leader" vehicle (Vehicle 0). It uses a combination of:
1.  **Model-based prediction**: Using a physical model of vehicle platooning dynamics.
2.  **Measurement correction**: Using local sensors (e.g., radar/camera measuring distance to the car ahead).
3.  **Consensus**: Exchanging state estimates with neighboring vehicles via V2V communication to reach an agreement.

## Mathematical Foundation

The estimator relies on a relative state formulation. Instead of estimating absolute GPS coordinates directly, it estimates errors relative to a desired formation behind the Leader.

### 1. State Vector
For a fleet of $N$ followers (indexed $1$ to $N$) and 1 Leader (indexed $0$), the observer estimates a combined state vector. For each vehicle $i$, the internal state is:

$$
x_i = \begin{bmatrix} 
\tilde{p}_i \\
\tilde{v}_i \\
\tilde{a}_i
\end{bmatrix} = \begin{bmatrix} 
p_i - p_0 + d_{i0} \\
v_i - v_0 \\
a_i - a_0
\end{bmatrix}
$$

Where:
*   $p, v, a$ are Position, Velocity, and Acceleration.
*   $0$ denotes the Leader, $i$ denotes the Follower.
*   $d_{i0}$ is the **desired spacing** adjustment, calculated as:
    $$ d_{i0} = i \cdot d + h \cdot \sum_{k=1}^{i} v_k $$
    *   $d$: Desired stand-still distance.
    *   $h$: Time headway (time gap).

### 2. Observer Dynamics
The state update follows this structure:

$$
\dot{\hat{x}} = \underbrace{A_\delta \hat{x} + B_\delta u}_{\text{Prediction}} + \underbrace{L(y - C\hat{x})}_{\text{Correction}} - \underbrace{\gamma \sum_{j \in \mathcal{N}_i} a_{ij} (\hat{x}_i - \hat{x}_j)}_{\text{Consensus}}
$$

*   **Prediction**: Uses `A_delta` and `B_delta` matrices derived from the Constant Time Headway (CTH) spacing policy and longitudinal vehicle dynamics ($\tau$).
*   **Correction**: Compares the estimated relative distance ($p_i - p_{i-1}$) with actual sensor measurements locally available to the vehicle.
*   **Consensus**: Pushes the estimate $\hat{x}_i$ closer to the estimates received from communicating neighbors ($\hat{x}_j$).

## Key Components

### Initialization
```python
estimator = DistributedLuenbergerEstimator(
    vehicle_id=1, 
    fleet_size=3,
    config={
        'observer_gain': 0.1,    # L matrix gain
        'consensus_gain': 0.2,   # Consensus strength
        'adjacency_matrix': ...  # Optional custom topology
    }
)
```
*   **`vehicle_id`**: Identity of the car running this code.
*   **`fleet_size`**: Total cars including leader.
*   **`adjacency_matrix`**: Defines the communication network (who talks to whom). Defaults to a **Chain Topology** (1↔2↔3...).

### Main Methods

#### `update(local_state, dt, current_time_ns, control)`
The core loop called at every timestep.
1.  **Prepare**: Converts current absolute fleet states to the relative observer separation states (`_transfer_fleet_states_to_estimated_states`).
2.  **Estimate**: Runs the `_distributed_luenberger_observer_update`.
    *   Predicts next state using physics.
    *   Corrects using local measurements (distance to front car).
    *   Adjusts using Consensus (data from neighbors).
3.  **Output**: Converts the relative estimates back to absolute coordinates ($x, y, v, a$) for the rest of the software to use (`_transfer_estimated_states_to_fleet_states`).

#### `_transfer_estimated_states_to_fleet_states` & `_transfer_fleet_states_to_estimated_states`
These helper functions handle the complex transformation between:
*   **Absolute World Coordinates**: Used by the GUI, path planning, etc. ($x, y$).
*   **Relative String Coordinates**: Used by the Luenberger math ($p_i - p_0 + \text{offset}$).

#### `_get_latest_received_state`
Robustly retrieves data received from other vehicles, handling missing packets or communication delays by using buffered history.

## Configuration Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `observer_gain` | float | 0.1 | Strength of local sensor correction. |
| `consensus_gain` | float | 0.2 | Strength of agreement with neighbors. |
| `adjacency_matrix` | list | Chain | Matrix defining communication links. |

## Usage Context
This class is intended to run on **each vehicle** independently. Although it is "distributed" (no central computer), every vehicle runs an instance of this estimator to build its own "mental model" of where the entire fleet is, which is crucial for safe formation control without collisions.
