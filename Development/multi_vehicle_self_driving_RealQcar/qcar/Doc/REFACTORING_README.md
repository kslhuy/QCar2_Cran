# Refactored Vehicle Control System

## Overview

This is a comprehensive refactoring of the QCar vehicle control system with improved architecture, error handling, safety systems, and monitoring capabilities.

## Key Improvements

### 1. **Configuration Management** (`config.py`)
- Centralized configuration with dataclasses
- Load from YAML/JSON files or command line
- Easy parameter tuning without code changes

### 2. **State Machine** (`state_machine.py`)
- Clear state transitions (INITIALIZING → FOLLOWING_PATH → etc.)
- State validation and callbacks
- Better error handling

### 3. **Enhanced Logging** (`logging_utils.py`)
- Structured logging with timestamps
- Performance monitoring
- CSV telemetry logging for post-analysis
- Separate log files per vehicle

### 4. **Network Communication** (`network_client.py`)
- Robust error handling with automatic reconnection
- Non-blocking operations
- Message framing and JSON serialization
- Network statistics tracking

### 5. **Safety Systems** (`safety.py`)
- Control command validation and clamping
- Sensor health monitoring (GPS timeout detection)
- Collision avoidance with emergency stop
- Watchdog timer for hang detection

### 6. **Controllers** (`controllers.py`)
- Thread-safe controller implementations
- Anti-windup for PI controller
- Improved Stanley steering controller
- State estimation wrapper

### 7. **Main Controller** (`vehicle_controller.py`)
- Modular architecture with clear separation of concerns
- Graceful initialization and shutdown
- Exception handling at all levels
- Performance monitoring

## File Structure

```
qcar/
├── config.py                      # Configuration management
├── logging_utils.py               # Logging and performance monitoring
├── state_machine.py               # State machine implementation
├── network_client.py              # Network communication
├── safety.py                      # Safety systems and validation
├── controllers.py                 # Speed and steering controllers
├── vehicle_controller.py          # Main controller class
├── vehicle_control_refactored.py  # Entry point
├── config_example.yaml            # Example configuration file
├── utils.py                       # YOLO utilities (existing)
└── vehicle_control_real.py        # Original implementation (backup)
```

## Usage

### Basic Usage

```bash
# Run with default settings
python vehicle_control_refactored.py

# Specify car ID
python vehicle_control_refactored.py --car-id 0

# Use custom configuration file
python vehicle_control_refactored.py --config my_config.yaml

# Enable remote control
python vehicle_control_refactored.py --host 192.168.1.100 --port 5000 --car-id 0
```

### Command Line Arguments

```
-c, --calibrate              Recalibrate vehicle before starting
-n, --node_configuration     Node configuration (0 or 1)
--host HOST                  Host PC IP for remote control
--port PORT                  Base port number (default: 5000)
--car-id ID                  Car ID (0, 1, 2, ...)
--config FILE                Configuration file (YAML or JSON)
--v-ref VELOCITY             Reference velocity in m/s
--no-steering                Disable steering control
```

### Configuration File

Create a YAML configuration file (see `config_example.yaml`):

```yaml
speed:
  v_ref: 0.75
  K_p: 0.1
  K_i: 1.0

steering:
  K_stanley: 0.7
  enable_steering_control: true

network:
  host_ip: "192.168.1.100"
  car_id: 0
```

Then run:

```bash
python vehicle_control_refactored.py --config my_config.yaml
```

## Architecture

### Component Interaction

```
VehicleController (Main)
├── VehicleLogger (Logging)
├── PerformanceMonitor (Metrics)
├── VehicleStateMachine (States)
├── NetworkClient (Communication)
├── SpeedController (Control)
├── SteeringController (Control)
├── StateEstimator (State)
├── ControlValidator (Safety)
├── SensorHealthMonitor (Safety)
├── CollisionAvoidance (Safety)
└── WatchdogTimer (Safety)
```

### State Machine Flow

```
INITIALIZING
    ↓
WAITING_FOR_CONNECTION (if remote enabled)
    ↓
WAITING_FOR_START
    ↓
NAVIGATING_TO_START (if not at start)
    ↓
FOLLOWING_PATH
    ↓
STOPPED / EMERGENCY_STOP
    ↓
SHUTTING_DOWN
```

## Safety Features

### 1. **Control Validation**
- Throttle and steering commands are validated and clamped
- State values checked for NaN/Inf
- Velocity bounds enforced

### 2. **Sensor Health Monitoring**
- GPS timeout detection
- Automatic fallback to dead reckoning
- Health status reporting

### 3. **Collision Avoidance**
- Emergency stop on imminent collision
- Warning system for close objects
- Configurable safety distances

### 4. **Watchdog Timer**
- Detects system hangs
- Automatic error logging
- Timeout statistics

## Logging

### Log Files

The system generates several log files:

```
logs/
├── vehicle_0_20251008_143000.log   # Main log file
data_logs/
└── run_20251008_143000/
    └── telemetry_vehicle_0.csv      # Telemetry data
```

### Log Levels

- **DEBUG**: Detailed control loop information
- **INFO**: Important events and state transitions
- **WARNING**: Recoverable errors and anomalies
- **ERROR**: Critical errors requiring attention

### Performance Monitoring

The system logs performance metrics every 10 seconds:
- Average loop time and frequency
- Network latency statistics
- Control computation time

## Network Protocol

### Telemetry Message (Vehicle → Host)

```json
{
  "car_id": 0,
  "timestamp": 1234567890.123,
  "data": {
    "x": 1.23,
    "y": 4.56,
    "theta": 0.78,
    "velocity": 0.75,
    "throttle": 0.1,
    "steering": 0.05,
    "state": "FOLLOWING_PATH"
  }
}
```

### Command Message (Host → Vehicle)

```json
{
  "v_ref": 0.8,
  "command": "resume"
}
```

Available commands:
- `stop`: Stop vehicle
- `resume`: Resume path following
- `shutdown`: Shutdown system

## Testing

### Unit Tests (TODO)

```bash
# Run all tests
python -m pytest tests/

# Run specific test
python -m pytest tests/test_controllers.py
```

### Integration Tests (TODO)

Test the full system with simulated sensors.

## Migration from Original Code

To migrate from `vehicle_control_real.py`:

1. **No code changes needed** - The refactored version maintains API compatibility
2. **Optional**: Create a configuration file for your parameters
3. **Run**: Use `vehicle_control_refactored.py` instead

### Comparison

| Feature | Original | Refactored |
|---------|----------|------------|
| Configuration | Hardcoded | File-based |
| Logging | Print statements | Structured logging |
| Error Handling | Basic | Comprehensive |
| State Management | Flags | State machine |
| Network | Basic | Robust with retry |
| Safety | Minimal | Multi-layered |
| Testing | None | Ready for tests |
| Performance Monitoring | None | Built-in |

## Troubleshooting

### GPS Not Working

Check logs for GPS timeout warnings. The system will automatically fall back to dead reckoning.

### Network Connection Issues

- Verify host IP and port
- Check firewall settings
- Review network logs for connection attempts

### Performance Issues

- Check performance statistics in logs
- Reduce controller update rate if needed
- Monitor loop time warnings

### YOLO Server Not Starting

- Ensure YOLO server is running before starting vehicle control
- Check YOLO server logs
- Verify network connectivity

## Future Enhancements

- [ ] Add unit tests for all components
- [ ] Implement EKF state estimation
- [ ] Add visualization dashboard
- [ ] Support for multiple YOLO streams
- [ ] Path planning optimization
- [ ] Fleet coordination protocols
- [ ] Real-time parameter tuning
- [ ] Automated calibration

## License

Same as the original QCar project.

## Authors

Original implementation: QCar Team
Refactoring: GitHub Copilot (October 2025)

## Contributing

When adding features:
1. Follow the modular architecture
2. Add appropriate logging
3. Implement error handling
4. Update configuration if needed
5. Add tests
6. Update documentation
