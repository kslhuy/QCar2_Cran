# Enhanced State Machine Architecture

## Overview

The enhanced state machine provides a more structured approach to vehicle control by separating state logic into dedicated `enter`, `update`, and `exit` functions for each state. This design improves modularity, testability, and maintainability.

## Architecture

### Key Components

1. **StateHandler Base Class**: Provides the interface for all state handlers
2. **Specific State Handlers**: Implement the logic for each vehicle state
3. **EnhancedVehicleStateMachine**: Orchestrates state transitions and updates
4. **Sensor Data Structure**: Standardized data passing between components

### State Handler Interface

Each state handler implements three key methods:

```python
class StateHandler:
    def enter(self) -> bool:
        """Called when entering this state"""
        return True
    
    def update(self, dt: float, sensor_data: Dict[str, Any]) -> Tuple[float, float]:
        """Called every control loop iteration"""
        return 0.0, 0.0  # throttle, steering
    
    def exit(self):
        """Called when exiting this state"""
        pass
```

## FOLLOWING_PATH State Implementation

### Enter Function
- Resets speed controller integral to prevent windup
- Logs path information (waypoints, node sequence)
- Initializes state-specific variables (lap counting, timing)
- Returns `True` if initialization successful

### Update Function
- **Startup Delay**: Returns (0,0) during initial delay period
- **Speed Control**: Applies YOLO adjustments and computes throttle command
- **Steering Control**: Updates steering based on look-ahead point
- **Collision Monitoring**: Checks for emergency conditions
- **Progress Tracking**: Monitors waypoint progress and lap completion
- **Performance Logging**: Logs control errors periodically

### Exit Function
- Logs session statistics (duration, laps completed)
- Records final waypoint position
- Cleanup of state-specific resources

## Benefits

### 1. Separation of Concerns
- Each state encapsulates its own logic
- Clear boundaries between state behaviors
- Easier to understand and modify individual states

### 2. Improved Testability
- Each state can be unit tested independently
- Mock sensor data can be easily provided
- State transitions can be tested in isolation

### 3. Better Debugging
- State-specific logging and monitoring
- Clear entry/exit points for debugging
- Detailed performance metrics per state

### 4. Enhanced Modularity
- Easy to add new states without affecting existing ones
- State logic is self-contained
- Consistent interface across all states

## Usage Example

```python
# Create enhanced state machine
state_machine = EnhancedVehicleStateMachine(vehicle_logic, logger)

# Transition to a state (calls exit() on current state, enter() on new state)
state_machine.transition_to(VehicleState.FOLLOWING_PATH)

# In control loop (calls update() on current state)
sensor_data = {
    'x': current_x,
    'y': current_y, 
    'theta': current_theta,
    'velocity': current_velocity,
    # ... other sensor data
}

throttle, steering = state_machine.update(dt, sensor_data)
```

## Sensor Data Structure

The standardized sensor data dictionary includes:

```python
sensor_data = {
    # Position and motion
    'x': float,                    # X position (m)
    'y': float,                    # Y position (m) 
    'theta': float,                # Heading angle (rad)
    'velocity': float,             # Current velocity (m/s)
    
    # Raw sensor readings
    'motor_tach': float,           # Motor encoder reading
    'gyro_z': float,               # Z-axis gyroscope reading
    'state_valid': bool,           # EKF state validity
    
    # Perception data
    'yolo_data': {
        'stop_sign': bool,         # Stop sign detected
        'traffic_light': bool,     # Traffic light detected
        'cars': bool,              # Cars detected
        'yield_sign': bool,        # Yield sign detected
        'person': bool,            # Person detected
        'car_dist': float,         # Distance to nearest car (m)
        'person_dist': float       # Distance to nearest person (m)
    }
}
```

## Control Flow Separation

### Before (Monolithic)
```
_control_iteration():
  - Read sensors
  - Update EKF  
  - Process YOLO
  - Check state conditions
  - Compute all control logic
  - Handle all state transitions
  - Send commands
```

### After (Modular)
```
_control_iteration():
  - Read sensors
  - Update EKF (stays outside state machine)
  - Process YOLO
  - Prepare sensor data
  - Call state_machine.update() -> returns commands
  - Send commands

state_machine.update():
  - Calls current_state.update()
  - State handles its own control logic
  - Returns appropriate commands
```

## Future Extensions

The enhanced state machine can be easily extended with additional states:

1. **NavigatingToStartState**: Handle navigation to start position
2. **EmergencyStopState**: Emergency braking logic
3. **PlatoonStates**: Platoon formation and following logic
4. **StoppedState**: Handle stopped vehicle behavior

Each new state just needs to inherit from `StateHandler` and implement the three core methods.

## Migration Path

The enhanced state machine is designed for gradual migration:

1. **Phase 1**: Implement `FOLLOWING_PATH` state (current)
2. **Phase 2**: Add other operational states
3. **Phase 3**: Migrate platoon states 
4. **Phase 4**: Complete migration and remove legacy code

The original state machine logic is preserved as fallback methods to ensure compatibility during transition.