# Enhanced Vehicle Logging System

## Overview
The enhanced logging system provides comprehensive data logging for vehicle operations, including dedicated CSV files for tracking received estimations from other vehicles in the fleet.

## New Logging Features

### 1. Received Fleet Estimations Log
**File**: `data_logs/run_<timestamp>/received_fleet_estimations_vehicle_<car_id>.csv`

Logs all fleet state estimations received from other vehicles. Each row represents one vehicle's state within a fleet estimation message.

**Columns**:
- `timestamp`: Original timestamp from sender
- `receive_time`: When this vehicle received the message
- `sender_id`: Vehicle ID that sent the fleet estimation
- `source`: Source of the estimation (e.g., 'fleet_consensus', 'local_observer')
- `vehicle_id`: ID of the vehicle this state describes
- `x`: X position (meters)
- `y`: Y position (meters)
- `theta`: Heading angle (radians)
- `v`: Velocity (m/s)
- `confidence`: Confidence level (0.0-1.0)

**Use Cases**:
- Analyze consensus algorithm convergence
- Track how each vehicle estimates the fleet state
- Measure fleet-wide state estimation accuracy
- Debug cooperative localization issues

### 2. Received Local Estimations Log
**File**: `data_logs/run_<timestamp>/received_local_estimations_vehicle_<car_id>.csv`

Logs all local state estimations received from individual vehicles (self-reported states).

**Columns**:
- `timestamp`: Original timestamp from sender
- `receive_time`: When this vehicle received the message
- `sender_id`: Vehicle ID that sent the local estimation
- `source`: Source of the estimation (e.g., 'gps', 'ekf', 'observer', 'local_sensors')
- `x`: X position (meters)
- `y`: Y position (meters)
- `theta`: Heading angle (radians)
- `v`: Velocity (m/s)
- `confidence`: Confidence level (0.0-1.0)

**Use Cases**:
- Track V2V communication latency (receive_time - timestamp)
- Compare self-reported states vs. fleet consensus
- Monitor network reliability and message reception
- Validate state estimation quality across vehicles

### 3. Existing Telemetry Log
**File**: `data_logs/run_<timestamp>/telemetry_vehicle_<car_id>.csv`

Continues to log the vehicle's own operational telemetry including control inputs, waypoint tracking, and platoon status.

## Architecture

### Non-Blocking Async Logging
All logging uses dedicated background threads with buffered queues to prevent blocking the main control loop:

```
Main Control Loop (100Hz+)
    ↓
VehicleLogger.log_fleet_estimation()
    ↓
Non-blocking queue.put_nowait()
    ↓
Background Thread Worker
    ↓
CSV File Write + Periodic Flush
```

**Performance Characteristics**:
- Queue size: 1000 entries per logger
- Flush interval: Every 100 entries or when queue empty
- Zero main loop blocking
- Automatic data loss prevention with fsync

### Integration Points

#### 1. Initialization
When telemetry logging is setup, the fleet and local estimation loggers are automatically initialized:

```python
# In vehicle_logic.py or similar
run_dir = self.logger.setup_telemetry_logging(data_log_dir="data_logs")
# Automatically creates:
# - received_fleet_estimations_vehicle_<car_id>.csv
# - received_local_estimations_vehicle_<car_id>.csv
```

#### 2. V2V Message Handling
The `V2VManager` automatically logs received estimations:

```python
# In v2v_manager.py - _handle_local_state_message()
self.logger.log_local_estimation(
    sender_id=sender_id,
    state={'x': ..., 'y': ..., 'theta': ..., 'v': ..., 'confidence': ...},
    source='local_sensors',
    timestamp=timestamp
)

# In v2v_manager.py - _handle_fleet_state_message()
self.logger.log_fleet_estimation(
    sender_id=sender_id,
    fleet_states={'vehicle_0': {...}, 'vehicle_1': {...}},
    source='fleet_consensus',
    timestamp=timestamp
)
```

## Data Analysis Examples

### Example 1: Fleet Consensus Convergence
```python
import pandas as pd
import matplotlib.pyplot as plt

# Load fleet estimations received by vehicle 1
df = pd.read_csv('data_logs/run_20250101_120000/received_fleet_estimations_vehicle_1.csv')

# Filter for estimates of vehicle 0 from different senders
vehicle_0_estimates = df[df['vehicle_id'] == 0]

# Plot how different vehicles estimate vehicle 0's position
plt.figure(figsize=(12, 6))
for sender in vehicle_0_estimates['sender_id'].unique():
    sender_data = vehicle_0_estimates[vehicle_0_estimates['sender_id'] == sender]
    plt.plot(sender_data['timestamp'], sender_data['x'], label=f'From Vehicle {sender}')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('X Position (m)')
plt.title('Fleet Consensus: Vehicle 0 X Position Estimates')
plt.show()
```

### Example 2: V2V Communication Latency
```python
import pandas as pd
import numpy as np

# Load local estimations
df = pd.read_csv('data_logs/run_20250101_120000/received_local_estimations_vehicle_1.csv')

# Calculate latency
df['latency_ms'] = (df['receive_time'] - df['timestamp']) * 1000

# Statistics per sender
for sender in df['sender_id'].unique():
    sender_data = df[df['sender_id'] == sender]
    print(f"\nVehicle {sender} -> Vehicle 1 Latency:")
    print(f"  Mean: {sender_data['latency_ms'].mean():.2f} ms")
    print(f"  Max: {sender_data['latency_ms'].max():.2f} ms")
    print(f"  Std: {sender_data['latency_ms'].std():.2f} ms")
```

### Example 3: Self-Report vs Fleet Consensus
```python
import pandas as pd

# Load both logs for vehicle 1
local_df = pd.read_csv('.../received_local_estimations_vehicle_1.csv')
fleet_df = pd.read_csv('.../received_fleet_estimations_vehicle_1.csv')

# Filter self-reports from vehicle 0
self_reports = local_df[local_df['sender_id'] == 0]

# Filter fleet consensus estimates of vehicle 0 from vehicle 2
consensus = fleet_df[(fleet_df['sender_id'] == 2) & (fleet_df['vehicle_id'] == 0)]

# Compare positions over time
# ... merge on timestamp and plot differences
```

## File Structure

```
data_logs/
└── run_20250110_143000/
    ├── telemetry_vehicle_0.csv                    # Vehicle 0's own telemetry
    ├── telemetry_vehicle_1.csv                    # Vehicle 1's own telemetry
    ├── received_fleet_estimations_vehicle_0.csv   # Fleet states received by Vehicle 0
    ├── received_fleet_estimations_vehicle_1.csv   # Fleet states received by Vehicle 1
    ├── received_local_estimations_vehicle_0.csv   # Local states received by Vehicle 0
    └── received_local_estimations_vehicle_1.csv   # Local states received by Vehicle 1
```

## Performance Notes

### Memory Usage
- Each logger uses ~8KB file buffer + 1000 entry queue
- Total additional memory per vehicle: ~100-200KB
- Negligible impact on system performance

### Disk I/O
- Buffered writes reduce disk I/O frequency
- fsync() ensures data integrity
- Typical write frequency: 10-20 times per second per file
- Estimated disk usage: ~1-5 MB per minute per vehicle (depends on fleet size and update rates)

### CPU Usage
- Background threads run at low priority
- Main loop blocking: 0% (non-blocking queue operations)
- CPU overhead: <1% per logging thread

## Troubleshooting

### Missing Logs
If estimation log files are not created:
1. Check that `setup_telemetry_logging()` is called
2. Verify V2V communication is active
3. Ensure other vehicles are broadcasting states

### Queue Full Warnings
If you see queue full messages:
- Reduce broadcast frequencies in `V2VBroadcastConfig`
- Increase queue size (current: 1000)
- Check disk write performance

### Data Integrity
All logs use:
- `fsync()` for guaranteed disk writes
- Buffered I/O for performance
- Thread-safe queue operations
- Graceful shutdown with flush

## API Reference

### VehicleLogger Methods

#### `log_fleet_estimation(sender_id, fleet_states, source, timestamp)`
Logs a received fleet state estimation.

**Parameters**:
- `sender_id` (int): Vehicle that sent the estimation
- `fleet_states` (dict): Fleet states dict with vehicle states
- `source` (str): Estimation source identifier
- `timestamp` (float): Original timestamp from sender

#### `log_local_estimation(sender_id, state, source, timestamp)`
Logs a received local state estimation.

**Parameters**:
- `sender_id` (int): Vehicle that sent the estimation
- `state` (dict): State dict with x, y, theta, v, confidence
- `source` (str): Estimation source identifier
- `timestamp` (float): Original timestamp from sender

#### `close()`
Properly shuts down all logging threads and flushes data to disk. Called automatically in context manager or destructor.

## Future Enhancements

Potential additions for future versions:
- Real-time plotting dashboard
- Automatic data analysis reports
- Anomaly detection in estimations
- Network topology visualization
- Estimation accuracy metrics
