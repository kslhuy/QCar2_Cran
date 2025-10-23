# Trajectory Following Controller for Follower Vehicles

## Overview

The `VehicleFollowerController` now supports a new controller type called **"TRAJECTORY"** that allows follower vehicles to independently follow a predefined trajectory/path instead of following another vehicle.

## Features

- **Independent Path Following**: Follower vehicles can follow their own waypoint-based trajectories
- **Same Core Logic as Leader**: Uses the same proven path-following algorithms (SpeedController + SteeringController)
- **Flexible Configuration**: Each vehicle can have different trajectories, speeds, and control parameters
- **Multiple Controller Types**: Seamlessly integrates with existing CACC, IDM, and LOOKAHEAD controllers

## Controller Types Comparison

| Controller Type | Description | Requires Leader | Use Case |
|----------------|-------------|-----------------|----------|
| **TRAJECTORY** | Follow predefined waypoints | No | Independent path following |
| **CACC** | Cooperative Adaptive Cruise Control | Yes | Platoon following with vehicle communication |
| **IDM** | Intelligent Driver Model | Yes | Classic car-following behavior |
| **LOOKAHEAD** | Extended Look-Ahead Controller | Yes | Advanced platoon control with preview |

## Usage

### Basic Configuration

```python
# In your vehicle configuration
{
    'vehicle_id': 1,
    'role': 'follower',
    'controller_type': 'TRAJECTORY',  # Enable trajectory following
    'road_type': 'Studio',            # or 'OpenRoad'
    'node_sequence': [0, 1, 2, 3],   # Waypoint nodes to follow
    'enable_steering_control': True,
    'K_p': 0.1,                       # Speed control P gain
    'K_i': 0.08,                      # Speed control I gain
    'K_stanley': 0.8,                 # Stanley steering gain
    'max_steering': 0.55,             # Max steering angle (radians)
    'startDelay': 1                   # Start delay in seconds
}
```

### Example: Multi-Vehicle Scenario

```python
# Vehicle 0: Leader following trajectory
vehicle_0_config = {
    'controller_type': 'leader',
    'node_sequence': [0, 1, 2, 3]
}

# Vehicle 1: Follower independently following same trajectory
vehicle_1_config = {
    'controller_type': 'TRAJECTORY',
    'node_sequence': [0, 1, 2, 3]  # Same path as leader
}

# Vehicle 2: Follower using CACC to follow Vehicle 1
vehicle_2_config = {
    'controller_type': 'CACC',
    'leader_id': 1  # Follow vehicle 1
}
```

### Using the Controller

```python
from VehicleFollowerController import VehicleFollowerController

# Initialize controller
controller = VehicleFollowerController(
    vehicle_id=1,
    controller_type="TRAJECTORY",
    config=config,
    logger=logger
)

# Compute control commands (no leader information needed)
speed_cmd, steering_cmd = controller.compute_control(
    current_pos=[x, y, z],
    current_rot=[roll, pitch, yaw],
    current_velocity=velocity,
    dt=0.1
)

# For TRAJECTORY mode, leader parameters are optional (will be ignored)
```

## Configuration Parameters

### Required Parameters

- **controller_type**: Set to `"TRAJECTORY"`
- **road_type**: `"OpenRoad"` or `"Studio"`
- **node_sequence**: List of waypoint node IDs to follow (e.g., `[0, 1, 2, 3]`)

### Control Parameters

- **K_p** (default: 0.1): Proportional gain for speed controller
- **K_i** (default: 0.08): Integral gain for speed controller
- **K_stanley** (default: 0.8): Stanley steering controller gain
- **max_steering** (default: 0.55): Maximum steering angle in radians (~31.5°)
- **startDelay** (default: 1): Initial delay before starting control (seconds)

### Optional Parameters

- **enable_steering_control** (default: True): Enable/disable steering control
- **lookahead_distance** (inherited, not used for TRAJECTORY)

## Reference Velocity Profiles

The trajectory controller uses time-based reference velocity profiles:

### OpenRoad Profile
```python
t < 5s:   v_ref = 2.0 m/s
5s ≤ t < 10s: v_ref = 0.0 m/s
10s ≤ t < 15s: v_ref = -0.5 m/s
15s ≤ t < 20s: v_ref = 1.0 m/s
t ≥ 20s:  v_ref = 2.0 m/s
```

### Studio Profile
```python
v_ref = 0.3 m/s (constant)
```

You can customize these by modifying `_get_trajectory_vref()` method.

## Implementation Details

### Key Methods

1. **`_init_trajectory_controller()`**: Initializes waypoint sequence and control components
2. **`_generate_trajectory_path()`**: Generates path from road map and node sequence
3. **`_compute_trajectory_control()`**: Computes speed and steering commands
4. **`_get_trajectory_vref()`**: Returns reference velocity based on time

### Control Flow

```
Initialize → Generate Path → Start Timer → 
  ↓
Compute Control:
  - Get current time elapsed
  - Get reference velocity for current time
  - Compute speed error
  - Update speed controller (PI control)
  - Compute steering angle (Stanley controller)
  - Clamp outputs to safe ranges
```

## Advantages

1. **Flexibility**: Multiple vehicles can follow different paths simultaneously
2. **Independence**: No dependency on communication with other vehicles
3. **Proven Logic**: Uses the same reliable path-following code as leader vehicles
4. **Easy Testing**: Simplifies testing scenarios without complex multi-vehicle setups

## Use Cases

1. **Multi-Lane Scenarios**: Different vehicles following parallel paths
2. **Formation Control**: Vehicles maintaining specific geometric formations
3. **Testing & Validation**: Individual vehicle path-following tests
4. **Staged Platoons**: Mix of trajectory followers and vehicle followers
5. **Intersection Management**: Vehicles following conflict-free planned paths

## API Changes

### compute_control() Method

The `compute_control()` method signature now has optional leader parameters:

```python
def compute_control(
    self, 
    current_pos: list, 
    current_rot: list, 
    current_velocity: float,
    leader_pos: list = None,        # Optional for TRAJECTORY
    leader_rot: list = None,        # Optional for TRAJECTORY
    leader_velocity: float = None,  # Optional for TRAJECTORY
    leader_timestamp: float = None, # Optional for TRAJECTORY
    dt: float = 0.1
) -> Tuple[float, float]:
```

For TRAJECTORY mode, only the first 4 parameters are used.

## Error Handling

The controller includes robust error handling:

- **Missing Dependencies**: Checks if trajectory following components are available
- **Invalid Configuration**: Validates road type and node sequence
- **Runtime Errors**: Graceful fallback with logging

## Future Enhancements

Potential improvements:

1. Custom velocity profiles per vehicle
2. Dynamic waypoint updates during runtime
3. Obstacle avoidance integration
4. Path optimization algorithms
5. Adaptive control gains based on performance

## See Also

- `VehicleLeaderController_RTmodel.py` - Similar trajectory following for leader
- `example_trajectory_config.yaml` - Example configuration file
- `ControllerLeader.py` - Speed and steering controller implementations

## License

Same as parent project.
