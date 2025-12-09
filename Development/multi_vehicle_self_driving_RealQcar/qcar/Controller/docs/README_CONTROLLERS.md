# Modular Longitudinal Controller System

This system provides a modular way to switch between different longitudinal (throttle) controllers for vehicle following.

## Available Controllers

### 1. **CACC (Cooperative Adaptive Cruise Control)**
- Uses spacing error and velocity error to compute acceleration
- Best for platoon/convoy following with precise spacing
- Control law: `acc = K_spacing * spacing_error + K_velocity * velocity_error`

### 2. **PI Velocity Controller**
- Simple PI controller that tracks a target velocity
- Good for basic following without spacing control
- Control law: `throttle = Kp * error + Ki * integral(error)`

### 3. **Hybrid Controller**
- Automatically switches between CACC (when leader available) and PI (when not)
- Best for robust operation in various scenarios

## Quick Start

### Option 1: Change in Code

In `following_leader_state.py`, modify the `__init__` method:

```python
# Change this line to switch controllers
self.controller_type = 'cacc'  # Options: 'cacc', 'pi', 'hybrid'
```

### Option 2: Using Configuration (Recommended)

Add to your vehicle configuration:

```python
class VehicleConfig:
    # ... other config ...
    longitudinal_controller_type = 'cacc'  # Easy switch!
```

### Option 3: Runtime Switching

You can also add a method to switch controllers at runtime:

```python
def switch_controller(self, controller_type: str):
    """Switch to a different longitudinal controller"""
    self.controller_type = controller_type
    self._initialize_longitudinal_controller()
```

## Controller Parameters

### CACC Parameters

```python
cacc_params = {
    's0': 1.5,              # Minimum spacing (meters)
    'h': 0.5,               # Time headway (seconds)
    'K': [[0.2, 0.05]],     # [spacing_gain, velocity_gain]
    'acc_to_throttle_gain': 0.5,
    'max_throttle': 0.3
}
```

**Tuning Tips:**
- Increase `s0` for more conservative spacing
- Increase `h` for speed-dependent spacing
- Increase `K[0]` for faster spacing correction (may oscillate)
- Increase `K[1]` for better velocity tracking

### PI Parameters

```python
pi_params = {
    'kp': 0.1,              # Proportional gain
    'ki': 1.0,              # Integral gain
    'max_throttle': 0.3
}
```

**Tuning Tips:**
- Increase `kp` for faster response
- Increase `ki` for zero steady-state error
- Too high gains → oscillation

## Example Usage

```python
from Controller.longitudinal_controllers import ControllerFactory

# Create CACC controller
cacc = ControllerFactory.create('cacc', {
    's0': 1.5,
    'h': 0.5,
    'K': [[0.2, 0.05]],
    'max_throttle': 0.3
}, logger)

# Prepare states
follower_state = {
    'x': 0.0, 'y': 0.0, 'theta': 0.0, 'velocity': 0.5
}
leader_state = {
    'x': 2.0, 'y': 0.0, 'theta': 0.0, 'velocity': 0.6
}

# Compute throttle
dt = 0.005  # 200Hz
throttle = cacc.compute_throttle(follower_state, leader_state, dt)
```

## Adding Your Own Controller

1. Create a new class that inherits from `LongitudinalControllerBase`
2. Implement `compute_throttle()` and `reset()` methods
3. Add it to the `ControllerFactory.CONTROLLER_TYPES` dict

Example:

```python
class MyCustomController(LongitudinalControllerBase):
    def __init__(self, param1=1.0, param2=2.0, logger=None):
        self.param1 = param1
        self.param2 = param2
        self.logger = logger
    
    def compute_throttle(self, follower_state, leader_state, dt):
        # Your custom control logic here
        velocity = follower_state['velocity']
        target_velocity = follower_state.get('target_velocity', 0.0)
        
        # Simple example
        error = target_velocity - velocity
        throttle = self.param1 * error
        return np.clip(throttle, -0.3, 0.3)
    
    def reset(self):
        # Reset any internal state
        pass

# Register in factory
ControllerFactory.CONTROLLER_TYPES['custom'] = MyCustomController
```

## Benefits of This Modular Design

1. **Easy Switching**: Change one line to try different controllers
2. **Clean Code**: Each controller is isolated and testable
3. **Extensible**: Add new controllers without modifying existing code
4. **Configurable**: Parameters can be tuned per controller type
5. **Debugging**: Each controller has its own logging

## Testing Different Controllers

Quick test to compare controllers:

```python
# Test CACC
self.controller_type = 'cacc'
self._initialize_longitudinal_controller()
# Run and log performance

# Test PI
self.controller_type = 'pi'
self._initialize_longitudinal_controller()
# Run and log performance

# Compare results
```

## Common Issues

### Controller Not Working
- Check that `LONGITUDINAL_CONTROLLER_AVAILABLE = True`
- Verify import paths are correct
- Check logger output for initialization messages

### Oscillating Behavior
- Reduce controller gains (K_spacing, K_velocity, kp, ki)
- Increase filtering (alpha_filter)
- Check sensor data quality

### Poor Tracking
- Increase controller gains
- Verify leader state data is accurate
- Check throttle saturation limits

## Performance Tuning

For best performance:
1. Start with conservative gains
2. Gradually increase until you get desired response
3. Add filtering if oscillations occur
4. Test with different speeds and scenarios
5. Log and analyze spacing/velocity errors

---

**Need help?** Check the individual controller implementations in `longitudinal_controllers.py` for detailed documentation.
