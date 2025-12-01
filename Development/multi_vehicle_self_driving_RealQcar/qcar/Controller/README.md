# Vehicle Controllers

Speed and steering control systems for QCar vehicles.

## Files

- `controllers.py` - Speed and steering controller implementations
- `__init__.py` - Package initialization

## Controllers

### SpeedController
PI controller with anti-windup for velocity tracking.

**Configuration:**
```yaml
speed:
  K_p: 0.1           # Proportional gain
  K_i: 1.0           # Integral gain
  max_throttle: 0.3  # Maximum throttle output
```

**Usage:**
```python
from Controller.controllers import SpeedController

speed_controller = SpeedController(config, logger)
throttle = speed_controller.update(current_velocity, target_velocity, dt)
```

### SteeringController
Stanley controller for path tracking.

**Configuration:**
```yaml
steering:
  K_stanley: 0.7                    # Stanley gain
  enable_steering_control: true     # Enable/disable steering
```

**Usage:**
```python
from Controller.controllers import SteeringController

steering_controller = SteeringController(config, logger)
steering = steering_controller.update(state, waypoints, dt)
```

## Features

### Speed Control
- PI controller with integral anti-windup
- Configurable gains and limits
- Thread-safe implementation
- Performance monitoring

### Steering Control
- Stanley controller for path following
- Look-ahead distance calculation
- Cross-track error minimization
- Smooth steering commands

## Tuning Guidelines

### Speed Controller
- **K_p too high**: Oscillations around target speed
- **K_p too low**: Slow response to speed changes
- **K_i too high**: Overshoot and instability
- **K_i too low**: Steady-state error

### Steering Controller
- **K_stanley too high**: Aggressive steering, oscillations
- **K_stanley too low**: Wide turns, cutting corners
- **Typical range**: 0.3 - 1.5

## Integration

Controllers are integrated into `vehicle_logic.py`:

```python
# Initialize controllers
self.speed_controller = SpeedController(self.config, self.logger)
self.steering_controller = SteeringController(self.config, self.logger)

# Use in control loop
throttle = self.speed_controller.update(v, v_ref, dt)
steering = self.steering_controller.update(state, waypoints, dt)
```

See `../Doc/REFACTORING_README.md` for complete system documentation.