# Vehicle Models Integration Guide

## Overview
The `MockQCar` class now uses proper vehicle dynamics from the `vehiclemodels` folder, which implements the CommonRoad vehicle models (Althoff & Würsching, 2020).

## Available Models

### 1. Kinematic Single-Track Model (Default)
- **File**: `vehiclemodels/vehicle_dynamics_ks.py`
- **States**: 5 (x, y, steering_angle, velocity, yaw_angle)
- **Pros**: 
  - Faster computation
  - Simpler, more stable
  - Good for path planning and testing
- **Cons**: 
  - No tire dynamics
  - Less realistic at high speeds
- **Use when**: Testing basic control, path following, multi-vehicle coordination

### 2. Single-Track Dynamic Model
- **File**: `vehiclemodels/vehicle_dynamics_st.py`
- **States**: 7 (x, y, steering_angle, velocity, yaw_angle, yaw_rate, slip_angle)
- **Pros**: 
  - More realistic vehicle behavior
  - Includes tire dynamics and slip
  - Better for high-speed scenarios
- **Cons**: 
  - Slower computation
  - May be unstable at very low speeds (switches to kinematic internally)
- **Use when**: Testing realistic vehicle behavior, tire grip limits, dynamic maneuvers

## Usage Examples

### Basic Usage (Kinematic Model - Default)
```bash
# Start car 0 with kinematic model
python fake_vehicle_real_logic.py 0

# Start multiple cars
python fake_vehicle_real_logic.py 0
python fake_vehicle_real_logic.py 1
python fake_vehicle_real_logic.py 2
```

### Using Dynamic Model
```bash
# Start car 0 with dynamic model
python fake_vehicle_real_logic.py 0 127.0.0.1 5000 true

# Or use 'dynamic' keyword
python fake_vehicle_real_logic.py 0 127.0.0.1 5000 dynamic

# Multiple cars with dynamic model
python fake_vehicle_real_logic.py 0 127.0.0.1 5000 true
python fake_vehicle_real_logic.py 1 127.0.0.1 5000 true
```

### Remote Ground Station
```bash
# Connect to remote ground station with dynamic model
python fake_vehicle_real_logic.py 0 192.168.1.100 5000 true
```

## Vehicle Parameters

The mock vehicles use **Vehicle ID 1 (Ford Escort)** parameters:
- **Mass**: ~1000 kg
- **Wheelbase**: ~2.5 m (a + b)
- **Max Velocity**: Defined in `parameters_vehicle1.yaml`
- **Max Steering**: Defined in steering parameters
- **Max Acceleration**: Defined in longitudinal parameters

### Changing Vehicle Parameters

To use different vehicle parameters, modify `MockQCar.__init__()`:

```python
# Currently uses:
self.params = parameters_vehicle1()  # Ford Escort

# Change to other vehicles:
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
self.params = parameters_vehicle2()  # BMW 320i

from vehiclemodels.parameters_vehicle3 import parameters_vehicle3
self.params = parameters_vehicle3()  # VW Vanagon

from vehiclemodels.parameters_vehicle4 import parameters_vehicle4
self.params = parameters_vehicle4()  # Articulated vehicle
```

## Other Available Models (Not Currently Integrated)

The `vehiclemodels` folder contains additional models that can be integrated:

1. **KST Model** (`vehicle_dynamics_kst.py`): Kinematic Single-Track with trailers
2. **MB Model** (`vehicle_dynamics_mb.py`): Multi-Body model (most complex)
3. **STD Model** (`vehicle_dynamics_std.py`): Single-Track Dynamic with additional states
4. **Linearized Model** (`vehicle_dynamics_linearized.py`): For control design

## Integration Details

### State Vector Mapping

**Kinematic Model** (5 states):
```python
state_ks[0] = x position (m)
state_ks[1] = y position (m)
state_ks[2] = steering angle (rad)
state_ks[3] = velocity (m/s)
state_ks[4] = yaw angle (rad)
```

**Dynamic Model** (7 states):
```python
state_st[0] = x position (m)
state_st[1] = y position (m)
state_st[2] = steering angle (rad)
state_st[3] = velocity (m/s)
state_st[4] = yaw angle (rad)
state_st[5] = yaw rate (rad/s)
state_st[6] = slip angle at vehicle center (rad)
```

### Control Inputs

Both models use the same control input format:
```python
u[0] = steering rate (rad/s)
u[1] = longitudinal acceleration (m/s^2)
```

The `MockQCar` class converts normalized throttle/steering commands (-1 to 1) to physical units using vehicle parameters.

## Physics Update Flow

1. **Command Reception**: `write()` or `read_write_std()` receives normalized commands
2. **Command Conversion**: Convert to physical units (rad, rad/s, m/s²)
3. **Dynamics Computation**: Call `vehicle_dynamics_ks()` or `vehicle_dynamics_st()`
4. **Integration**: Simple Euler integration with time step `dt`
5. **State Update**: Update internal state vector
6. **Sensor Update**: Update mock sensors (motorTach, gyroscope)

## Performance Considerations

- **Kinematic Model**: ~1000 updates per second on typical hardware
- **Dynamic Model**: ~500 updates per second on typical hardware
- **Controller Rate**: 200 Hz (5ms period) in VehicleLogic
- **Integration Method**: Simple Euler (sufficient for real-time simulation)

## Advanced: Adding Custom Models

To add a new vehicle model:

1. Create model file in `vehiclemodels/` following existing structure
2. Import in `fake_vehicle_real_logic.py`
3. Add state vector to `MockQCar.__init__()`
4. Add model selection logic in `MockQCar._update_physics()`
5. Update command-line argument parsing in `main()`

Example:
```python
# In MockQCar.__init__()
if self.use_multi_body_model:
    from vehiclemodels.vehicle_dynamics_mb import vehicle_dynamics_mb
    self.state_mb = np.array([...])  # MB model states

# In _update_physics()
if self.use_multi_body_model:
    derivatives = vehicle_dynamics_mb(self.state_mb, self.control_input, self.params)
    # ... integration logic
```

## Testing

Run a simple test:
```bash
# Terminal 1: Start ground station (GUI controller)
cd qcar\GUI
python enhanced_gui_controller.py

# Terminal 2: Start fake vehicle with kinematic model
python fake_vehicle_real_logic.py 0

# Terminal 3: Start fake vehicle with dynamic model
python fake_vehicle_real_logic.py 1 127.0.0.1 5000 true
```

Compare the behavior of both vehicles in the GUI to see the difference between kinematic and dynamic models.

## References

- Althoff, M. and Würsching, G. "CommonRoad: Vehicle Models", 2020
- Vehicle parameters based on real vehicle measurements
- Models validated against real vehicle data

## Troubleshooting

### Vehicle doesn't move
- Check throttle/steering commands are being received
- Verify control input conversion (may need tuning)
- Check vehicle parameters are loaded correctly

### Unstable behavior
- For dynamic model: Check if velocity is too low (model switches to kinematic)
- Reduce integration time step if needed
- Check steering/acceleration limits in parameters

### Poor performance
- Use kinematic model for multi-vehicle scenarios
- Reduce controller update rate
- Profile code to find bottlenecks
