# Vehicle-to-Vehicle (V2V) Communication

Real-time communication system for multi-vehicle coordination and fleet management.

## Files

- `v2v_manager.py` - High-level V2V management and message routing
- `v2v_communication.py` - Low-level communication protocol
- `__init__.py` - Package initialization

## Overview

The V2V system enables:
- Real-time state sharing between vehicles
- Fleet coordination messages
- Warning and alert broadcasting
- Platoon formation communication

## Architecture

### V2VManager (High-level)
- Message routing and processing
- Broadcasting logic
- Data type management
- Queue handling

### V2VCommunication (Low-level)
- Network protocol implementation
- Message serialization
- Connection management
- Error handling and retry logic

## Message Types

### State Messages
```python
# Local vehicle state
{
    "type": "local_state",
    "vehicle_id": 0,
    "data": {
        "x": 1.23,
        "y": 4.56,
        "theta": 0.78,
        "velocity": 0.85,
        "timestamp": 1234567890.123
    }
}
```

### Fleet Coordination
```python
# Fleet formation command
{
    "type": "fleet_command",
    "leader_id": 0,
    "formation": "line",
    "spacing": 2.0
}
```

### Warnings
```python
# Emergency warning
{
    "type": "warning",
    "severity": "high",
    "message": "obstacle_detected",
    "location": {"x": 10.5, "y": 5.2}
}
```

## Usage

```python
from V2V.v2v_manager import V2VManager
from V2V.v2v_communication import V2VCommunication

# Initialize communication
v2v_comm = V2VCommunication(vehicle_id=0, base_port=6000)
v2v_manager = V2VManager(
    vehicle_id=0,
    v2v_communication=v2v_comm,
    vehicle_observer=observer,
    logger=logger
)

# Start communication
v2v_manager.start()

# Send state update
v2v_manager.broadcast_local_state(state_data)

# Receive messages
messages = v2v_manager.get_received_messages()
```

## Configuration

```yaml
v2v:
  base_port: 6000               # Base port for V2V communication
  broadcast_rate: 20.0          # Hz - Local state broadcast rate
  fleet_update_rate: 5.0        # Hz - Fleet coordination rate
  heartbeat_rate: 1.0           # Hz - Heartbeat frequency
  message_timeout: 2.0          # Seconds - Message validity timeout
  max_queue_size: 100           # Maximum message queue size
  retry_attempts: 3             # Network retry attempts
  retry_delay: 0.1              # Seconds between retries
```

## Communication Protocols

### Port Assignment
- Vehicle 0: Port 6000 (send), 6001 (receive)
- Vehicle 1: Port 6002 (send), 6003 (receive)
- Vehicle N: Port 6000+(N*2) (send), 6001+(N*2) (receive)

### Message Format
```python
class V2VMessage:
    vehicle_id: int
    message_type: MessageType
    timestamp: float
    data: dict
    sequence_number: int
```

### Broadcasting Strategy
- **High Frequency**: Local state updates (20 Hz)
- **Medium Frequency**: Fleet coordination (5 Hz)
- **Low Frequency**: Heartbeats (1 Hz)

## Features

### Reliable Communication
- Automatic retry on failure
- Message acknowledgment
- Timeout handling
- Connection health monitoring

### Message Prioritization
- Emergency messages: Immediate
- State updates: High priority
- Fleet commands: Medium priority
- Heartbeats: Low priority

### Data Management
- Circular buffers for efficiency
- Automatic queue cleanup
- Message deduplication
- Timestamp validation

## Integration

V2V is integrated into the main vehicle system:

```python
# In vehicle_logic.py
self.v2v_manager = V2VManager(
    vehicle_id=self.car_id,
    v2v_communication=self.v2v_comm,
    vehicle_observer=self.observer,
    logger=self.logger
)

# Control loop
self.v2v_manager.broadcast_local_state(current_state)
fleet_updates = self.v2v_manager.get_fleet_updates()
```

## Platoon Communication

### Formation Commands
- Leader broadcasts formation parameters
- Followers acknowledge and adjust
- Continuous spacing updates

### Emergency Protocols
- Immediate stop broadcasts
- Collision warnings
- System failure notifications

## Performance

- **Latency**: < 50 ms end-to-end
- **Throughput**: 100+ messages/second
- **Reliability**: > 99% message delivery
- **Range**: 100+ meters (depends on network)

## Troubleshooting

### Connection Issues
```powershell
# Check port availability
netstat -an | findstr "6000"

# Test network connectivity
ping <target_vehicle_ip>
```

### Message Loss
- Check network quality
- Verify port configuration
- Monitor retry statistics
- Increase timeout values

### High Latency
- Check network congestion
- Reduce broadcast rates
- Optimize message size
- Use priority queues

## Network Setup

Ensure proper network configuration:
- All vehicles on same subnet
- Multicast enabled (if used)
- Firewall allows V2V ports
- Stable WiFi connection

See `../Doc/REFACTORING_README.md` for complete system integration details.