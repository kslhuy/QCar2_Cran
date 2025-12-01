# Vehicle Observer System

Fleet state estimation and sensor data management for multi-vehicle coordination.

## Files

- `VehicleObserverSimple.py` - Main observer implementation
- `__init__.py` - Package initialization

## Overview

The VehicleObserver handles:
- Local vehicle state estimation
- Fleet state tracking and fusion
- Sensor data processing
- State synchronization between vehicles

## Features

### Local State Estimation
- Position tracking (x, y, heading)
- Velocity estimation
- Sensor fusion (GPS, encoders, IMU)
- State validation and filtering

### Fleet State Management
- Multi-vehicle state tracking
- State sharing via V2V communication
- Fleet formation monitoring
- Coordination data processing

## Usage

```python
from Observer.VehicleObserverSimple import VehicleObserver

# Initialize observer
observer = VehicleObserver(
    vehicle_id=0,
    fleet_size=2,
    config=config,
    logger=logger,
    initial_pose=[0, 0, 0],  # [x, y, theta]
    state_estimator=state_estimator
)

# Update with sensor data
observer.update_local_state(sensor_data)

# Get current state
local_state = observer.get_local_state()  # [x, y, theta, velocity]
fleet_states = observer.get_fleet_states()
```

## Configuration

```yaml
observer:
  state_timeout: 2.0        # Seconds before state is considered stale
  fusion_weights:           # Sensor fusion weights
    gps: 0.7
    encoder: 0.2
    imu: 0.1
  fleet_update_rate: 10.0   # Hz - Fleet state update frequency
```

## State Data Structure

### Local State
```python
state = {
    'x': float,         # Position X (m)
    'y': float,         # Position Y (m)
    'theta': float,     # Heading (rad)
    'velocity': float,  # Speed (m/s)
    'timestamp': float, # Unix timestamp
    'valid': bool       # State validity flag
}
```

### Fleet State
```python
fleet_state = {
    'vehicle_id': int,
    'state': state,     # Same format as local state
    'last_update': float,
    'communication_quality': float
}
```

## Integration

Observer is integrated into the main vehicle control:

```python
# In vehicle_logic.py
self.observer = VehicleObserver(
    vehicle_id=self.car_id,
    fleet_size=self.fleet_size,
    config=self.config,
    logger=self.logger,
    state_estimator=self.state_estimator
)

# Control loop
sensor_data = self.read_sensors()
self.observer.update_local_state(sensor_data)
current_state = self.observer.get_local_state()
```

## State Estimation Pipeline

1. **Sensor Reading**: Collect GPS, encoder, IMU data
2. **Data Validation**: Check sensor health and validity
3. **Fusion**: Combine sensor data using weighted fusion
4. **Filtering**: Apply Kalman filtering for smooth estimates
5. **Broadcasting**: Share state with other vehicles via V2V
6. **Fleet Update**: Receive and process other vehicles' states

## Performance

- **Update Rate**: 100 Hz local, 10 Hz fleet
- **Latency**: < 10 ms processing time
- **Accuracy**: Sub-meter positioning with good GPS
- **Memory**: Minimal footprint with circular buffers

## Troubleshooting

### GPS Timeout
- Check GPS hardware connection
- Verify GPS calibration
- Observer automatically falls back to dead reckoning

### State Divergence
- Check sensor calibration
- Verify fusion weights
- Monitor communication quality

### Fleet Desync
- Check V2V communication
- Verify network connectivity
- Monitor state timestamps

See `../Doc/REFACTORING_README.md` for complete system integration details.