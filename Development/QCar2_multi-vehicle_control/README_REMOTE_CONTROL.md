# Remote Control Setup for Multiple Physical QCars

This guide explains how to control multiple physical QCars from a centralized Host PC over a network.

## Architecture

```
┌─────────────────┐
│    Host PC      │  ← Your control station
│  (Controller)   │
└────────┬────────┘
         │ Network (WiFi/Ethernet)
    ┌────┴────┬─────────┐
    │         │         │
┌───▼───┐ ┌──▼────┐ ┌──▼────┐
│ QCar 0│ │QCar 1 │ │QCar 2 │
│Physical│ │Physical│ │Physical│
└───────┘ └───────┘ └───────┘
```

## Setup Steps

### 1. Network Configuration

#### On Host PC:
1. Connect to your local network (WiFi or Ethernet)
2. Find your IP address:
   - Windows: `ipconfig` (look for IPv4 Address)
   - Linux/Mac: `ifconfig` or `ip addr`
   - Example: `192.168.1.100`

#### On Each Physical QCar:
1. Ensure each QCar is connected to the same network
2. Each QCar should be able to ping your Host PC

### 2. File Setup

#### On Host PC:
- Use: `remote_controller.py`
- This runs the central controller

#### On Each Physical QCar:
- Use: `vehicle_control_remote.py`
- **IMPORTANT**: Edit the file and change:
  ```python
  HOST_PC_IP = '192.168.1.100'  # Change to your Host PC's IP
  CAR_ID = 0  # Change to 0, 1, 2, etc. for each car
  ```

### 3. Port Configuration

The system uses sequential ports:
- Car 0: Port 5000
- Car 1: Port 5001
- Car 2: Port 5002
- etc.

**Make sure these ports are not blocked by firewall!**

#### Windows Firewall Configuration:
```powershell
# Run PowerShell as Administrator
New-NetFirewallRule -DisplayName "QCar Remote Control" -Direction Inbound -LocalPort 5000-5010 -Protocol TCP -Action Allow
```

### 4. Running the System

#### Step 1: Start the Host Controller
On your Host PC:
```bash
python remote_controller.py
```

You should see:
```
[Host PC] Listening for Car 0 on port 5000
[Host PC] Listening for Car 1 on port 5001
Waiting for 2 cars to connect...
```

#### Step 2: Start Each QCar
On each physical QCar:
```bash
# On Car 0
python vehicle_control_remote.py

# On Car 1 (after editing CAR_ID to 1)
python vehicle_control_remote.py
```

You should see on each car:
```
[Car 0] Connecting to Host PC at 192.168.1.100:5000...
[Car 0] Connected to Host PC!
```

And on Host PC:
```
[Host PC] Car 0 connected from ('192.168.1.50', 54321)
[Host PC] Car 1 connected from ('192.168.1.51', 54322)
```

## Using the Controller

### Available Commands

From the Host PC terminal:

```bash
# Start a car
Command> start 0        # Start Car 0
Command> start 1        # Start Car 1

# Stop a car
Command> stop 0         # Stop Car 0
Command> stop 1         # Stop Car 1

# Set velocity (m/s)
Command> velocity 0 0.9    # Set Car 0 velocity to 0.9 m/s
Command> velocity 1 0.5    # Set Car 1 velocity to 0.5 m/s

# Change path (node sequence)
Command> path 0 10 4 20 10     # Set Car 0 to follow nodes [10, 4, 20, 10]
Command> path 1 4 13 9 4       # Set Car 1 to follow nodes [4, 13, 9, 4]

# Check status
Command> status
# Shows position, velocity, heading for all cars

# Emergency stop all cars
Command> emergency

# Exit
Command> quit
```

### Example Session

```bash
# Wait for both cars to connect
Command> status
Car 0: connected
  Position: (1.23, 2.45)
  Velocity: 0.00 m/s
  Heading: -1.57 rad
Car 1: connected
  Position: (1.23, 2.45)
  Velocity: 0.00 m/s
  Heading: -1.57 rad

# Start both cars with different velocities
Command> velocity 0 0.9
[Host PC] Sent to Car 0: {'type': 'set_params', 'v_ref': 0.9}

Command> velocity 1 0.6
[Host PC] Sent to Car 1: {'type': 'set_params', 'v_ref': 0.6}

Command> start 0
Command> start 1

# Monitor progress
Command> status

# Stop one car
Command> stop 1

# Emergency stop if needed
Command> emergency
```

## Advanced: Creating a GUI Controller

You can create a GUI controller using the `QCarRemoteController` class:

```python
from remote_controller import QCarRemoteController
import tkinter as tk

# Create GUI with buttons for each car
# Use controller.send_command() to control cars
```

## Troubleshooting

### Connection Issues

1. **"Connection refused"**
   - Check if Host PC firewall allows the ports
   - Verify IP address is correct
   - Ensure both devices are on same network

2. **"Connection timeout"**
   - Check network connectivity: `ping <HOST_PC_IP>`
   - Verify Host PC is running the controller first

3. **Cars disconnect randomly**
   - Check network stability
   - Ensure QCars have good WiFi signal
   - Consider using wired Ethernet if available

### Performance Issues

1. **Laggy response**
   - Check network latency
   - Reduce telemetry send rate if needed
   - Use wired connection instead of WiFi

2. **Commands not executing**
   - Check command format in terminal
   - Verify car ID is correct
   - Check `status` to confirm connection

## Safety Notes

1. **Always test in a safe environment first**
2. **Keep emergency stop ready** - Type `emergency` or press Ctrl+C
3. **Monitor network connection** - Disconnection means loss of control
4. **Have physical access** to cars in case of network failure
5. **Start with low velocities** when testing

## Network Requirements

- **Latency**: < 50ms recommended
- **Bandwidth**: Minimal (< 1 Mbps per car)
- **Reliability**: Stable connection required
- **Type**: WiFi 802.11n or better, or Ethernet

## Files Overview

- `remote_controller.py` - Runs on Host PC, controls multiple cars
- `vehicle_control_remote.py` - Runs on each physical QCar
- `vehicle_control.py` - Original script (for reference, without Qt)
- `vehicle_control2.py` - Original script for Car 1 (with Qt)

## Next Steps

1. Test with one car first
2. Add second car once comfortable
3. Consider implementing:
   - Formation control
   - Coordinated path planning
   - Collision avoidance
   - Fleet management dashboard

## Support

For issues specific to:
- **Network setup**: Check your router/network documentation
- **QCar hardware**: Refer to Quanser documentation
- **This remote control system**: Check logs on both Host PC and QCar
