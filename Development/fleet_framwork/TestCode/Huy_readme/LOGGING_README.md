# Fleet Vehicle Logging System Documentation

## Overview

The enhanced logging system for fleet vehicles provides comprehensive logging capabilities with separate log files for different types of events. This allows for better organization, debugging, and analysis of vehicle fleet operations.

## Log File Structure

The system creates the following log files in the `logs/` directory:

### 1. Individual Vehicle Logs
- **Files**: `vehicle_1.log`, `vehicle_2.log`, etc.
- **Purpose**: Dedicated log file for each vehicle's general operations
- **Content**: Vehicle-specific events, state changes, and general operations

### 2. Individual Communication Data Logs
- **Files**: `communication_vehicle_1.log`, `communication_vehicle_2.log`, etc.
- **Purpose**: Separate communication log for each vehicle
- **Content**: Each vehicle's message sending/receiving, ACK operations, communication errors

### 3. Individual GPS Synchronization Logs  
- **Files**: `gps_vehicle_1.log`, `gps_vehicle_2.log`, etc.
- **Purpose**: Separate GPS time synchronization log for each vehicle
- **Content**: Each vehicle's time synchronization, offset calculations, GPS server communication

### 4. Individual Vehicle Control Logs
- **Files**: `control_vehicle_1.log`, `control_vehicle_2.log`, etc. 
- **Purpose**: Separate control operations log for each vehicle
- **Content**: Each vehicle's control commands, steering/speed adjustments, control errors

### 5. Fleet Operations Log
- **File**: `fleet_operations.log`
- **Purpose**: General fleet coordination and management
- **Content**: Fleet-level events, coordination messages, general operations

## Usage Examples

### Basic Usage in Vehicle Code

```python
from md_logging_config import (
    get_individual_vehicle_logger,
    get_communication_logger,
    get_gps_logger,
    get_control_logger
)

class Vehicle:
    def __init__(self, vehicle_id):
        # Individual vehicle logger (creates vehicle_N.log)
        self.logger = get_individual_vehicle_logger(vehicle_id)
        
        # Specialized loggers
        self.comm_logger = get_communication_logger(vehicle_id)
        self.gps_logger = get_gps_logger(vehicle_id)
        self.control_logger = get_control_logger(vehicle_id)
    
    def send_message(self, message):
        # Log communication events
        self.comm_logger.info(f"SENT: {message}")
    
    def update_control(self, speed, steering):
        # Log control operations
        self.control_logger.info(f"Control update - Speed: {speed}, Steering: {steering}")
    
    def sync_gps(self, offset):
        # Log GPS events
        self.gps_logger.info(f"GPS sync completed, offset: {offset}")
```

### Communication Logging Example

```python
# In CommHandler class
class CommHandler:
    def __init__(self, vehicle_id, ...):
        self.comm_logger = get_communication_logger(vehicle_id)
    
    def send_state_message(self, data):
        self.comm_logger.info(f"SENT: Seq: {data['seq']}, Pos: {data['pos']}, V: {data['v']}")
    
    def receive_ack(self, ack_data):
        self.comm_logger.info(f"ACK received for seq: {ack_data['ack_seq']}")
    
    def handle_communication_error(self, error):
        self.comm_logger.error(f"Communication error: {error}")
```

### GPS Logging Example

```python
# In GPSSync class
class GPSSync:
    def __init__(self, vehicle_id, ...):
        self.gps_logger = get_gps_logger(vehicle_id)
    
    def sync_with_gps(self):
        try:
            offset = self.calculate_offset()
            self.gps_logger.info(f"GPS sync successful, offset: {offset:.3f} sec")
        except Exception as e:
            self.gps_logger.error(f"GPS sync failed: {e}")
```

### Control Logging Example

```python
# In VehicleController classes
class VehicleFollowerController:
    def __init__(self, vehicle_id, ...):
        self.control_logger = get_control_logger(vehicle_id)
    
    def compute_control(self, current_pos, leader_data):
        speed, steering = self.calculate_commands(current_pos, leader_data)
        self.control_logger.info(f"Control commands - Speed: {speed:.3f}, Steering: {steering:.3f}")
        return speed, steering
    
    def handle_control_error(self, error):
        self.control_logger.error(f"Control computation error: {error}")
```

## Configuration Options

### Enable Console Output

```python
from md_logging_config import enable_console_logging

# Enable console output for all loggers (useful for debugging)
enable_console_logging(True)

# Disable console output (default - logs only to files)
enable_console_logging(False)
```

### Custom Log Directory and Rotation

```python
from md_logging_config import FleetLoggingConfig

# Custom configuration
fleet_logging = FleetLoggingConfig(
    log_dir="custom_logs",        # Custom directory
    max_bytes=10*1024*1024,       # 10MB per file
    backup_count=5                # Keep 5 backup files
)

# Get loggers from custom configuration
vehicle_logger = fleet_logging.get_individual_vehicle_logger(1)
```

## Log Format

All log entries follow this format:
```
YYYY-MM-DD HH:MM:SS,mmm [LEVEL] VehicleID [SUBSYSTEM]: Message
```

Examples:
```
2025-08-13 14:30:25,123 [INFO] V1: Vehicle 1 initialized and starting operations
2025-08-13 14:30:25,124 [INFO] V1 [COMM]: SENT: Seq: 1, Pos: [10.5, 20.3, 0.0], V: 15.2
2025-08-13 14:30:25,125 [INFO] V1 [GPS]: GPS sync completed, offset: +0.023 sec
2025-08-13 14:30:25,126 [INFO] V1 [CTRL]: Control commands - Speed: 15.200, Steering: 0.150
```

## Log Rotation

- **File Size Limit**: 5MB per log file (configurable)
- **Backup Files**: 3 backup files kept per log type (configurable)  
- **Rotation Behavior**: When a log file reaches the size limit, it's renamed with a `.1` suffix, and previous backups are shifted (`.1` → `.2`, `.2` → `.3`, etc.)

## Benefits

1. **Organized Debugging**: Separate files for different types of events make it easier to focus on specific issues
2. **Scalability**: Each vehicle gets its own log file, preventing log mixing in multi-vehicle scenarios
3. **Performance Analysis**: Dedicated communication and control logs help analyze system performance
4. **GPS Analysis**: Separate GPS logs help debug time synchronization issues
5. **Log Management**: Automatic rotation prevents disk space issues

## Migration from Old System

The new system maintains backward compatibility:

```python
# Old way (still works)
from md_logging_config import logger

# New way (recommended)
from md_logging_config import get_individual_vehicle_logger
vehicle_logger = get_individual_vehicle_logger(vehicle_id)
```

## Testing the System

Run the example script to test the logging system:

```bash
python logging_example.py
```

This will create sample log files demonstrating all logging capabilities.

## Updated Log File Structure

**NEW**: Each vehicle now gets its own dedicated log files for different subsystems:

### Per Vehicle Files:
- `vehicle_N.log` - General vehicle operations and status
- `communication_vehicle_N.log` - Communication events (send/receive messages)
- `gps_vehicle_N.log` - GPS synchronization events  
- `control_vehicle_N.log` - Control operations and commands

### Shared Files:
- `fleet_operations.log` - General fleet coordination

### Example for 2 vehicles:
```
logs/
├── vehicle_1.log                    # Vehicle 1 general operations
├── communication_vehicle_1.log      # Vehicle 1 communication
├── gps_vehicle_1.log               # Vehicle 1 GPS sync
├── control_vehicle_1.log           # Vehicle 1 control
├── vehicle_2.log                    # Vehicle 2 general operations  
├── communication_vehicle_2.log      # Vehicle 2 communication
├── gps_vehicle_2.log               # Vehicle 2 GPS sync
├── control_vehicle_2.log           # Vehicle 2 control
└── fleet_operations.log            # Shared fleet coordination
```

This structure makes it much easier to debug individual vehicle issues by having all data for each vehicle separated into dedicated files.
