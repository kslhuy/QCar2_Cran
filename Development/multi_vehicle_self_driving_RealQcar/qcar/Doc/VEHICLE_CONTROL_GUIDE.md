# QCar Vehicle Control - Quick Start Guide

## Overview
The vehicle control system now automatically loads `config_example.yaml` by default.

## Configuration

### Default Configuration File
The system will automatically use `config_example.yaml` from the script directory if no config file is specified.

### Important Settings to Check

#### 1. Network Configuration (Remote Control)
```yaml
network:
  host_ip: 192.168.2.200  # IP address of the GUI controller PC
  base_port: 5000          # Base port (Car 0 uses 5000, Car 1 uses 5001, etc.)
  car_id: 0                # This car's ID
```

**⚠️ IMPORTANT:** 
- Set `host_ip` to the IP address where your GUI controller is running
- If `host_ip` is set, remote control is **ENABLED**
- To disable remote control, set `host_ip: null` or don't specify it

#### 2. Speed Settings
```yaml
speed:
  v_ref: 0.75  # Reference velocity in m/s
```

#### 3. Path Planning
```yaml
path:
  node_configuration: 0  # 0 or 1 for different paths
  calibrate: false       # Set to true to recalibrate GPS
```

## Running the Vehicle

### Method 1: Use Default Config (Recommended)
```bash
cd qcar
python vehicle_control_refactored.py
```
This will automatically load `config_example.yaml`

### Method 2: Override Config File
```bash
python vehicle_control_refactored.py --config my_custom_config.yaml
```

### Method 3: Override Specific Parameters
```bash
# Override network settings
python vehicle_control_refactored.py --host 192.168.2.200 --car-id 0

# Override velocity
python vehicle_control_refactored.py --v-ref 0.9

# Multiple overrides
python vehicle_control_refactored.py --host 192.168.2.200 --car-id 1 --v-ref 0.8
```

### Common Command Line Options

| Option | Description | Default |
|--------|-------------|---------|
| `--host IP` | Host PC IP for remote control | From config |
| `--port PORT` | Base port number | 5000 |
| `--car-id ID` | Car ID (0, 1, 2, ...) | 0 |
| `--v-ref SPEED` | Reference velocity (m/s) | From config |
| `--config FILE` | Custom config file | config_example.yaml |
| `-c`, `--calibrate` | Recalibrate GPS | false |
| `-n`, `--node_configuration` | Path config (0 or 1) | 0 |
| `--no-steering` | Disable steering control | false |

## Testing Configuration

To verify your configuration loads correctly:

```bash
cd qcar
python test_config.py
```

This will display all loaded configuration values.

## Multi-Vehicle Setup

### Car 0 (First Vehicle)
```bash
python vehicle_control_refactored.py --host 192.168.2.200 --car-id 0
```
- Connects to port 5000

### Car 1 (Second Vehicle)
```bash
python vehicle_control_refactored.py --host 192.168.2.200 --car-id 1
```
- Connects to port 5001

### Car 2 (Third Vehicle)
```bash
python vehicle_control_refactored.py --host 192.168.2.200 --car-id 2
```
- Connects to port 5002

## Troubleshooting

### Configuration Not Loading
1. Verify `config_example.yaml` exists in the `qcar` directory
2. Run `python test_config.py` to check for errors
3. Check YAML syntax is correct (no tabs, proper indentation)

### Vehicle Not Connecting to GUI
1. Check `host_ip` in `config_example.yaml` matches GUI PC's IP
2. Verify GUI is running and shows "Listening on port..."
3. Check firewall allows connections on port 5000+
4. Test network: `ping 192.168.2.200` (replace with your GUI IP)
5. Ensure both PC and vehicle are on same network

### Remote Control Disabled
- If you see "Remote control disabled" in logs:
  - Check `host_ip` in config is not `null`
  - Or pass `--host IP` on command line

### GPS/Calibration Issues
- If vehicle doesn't start at correct position:
  - Set `calibrate: true` in config, OR
  - Use `--calibrate` flag
  - Adjust `calibration_pose` values

## Configuration Priority

Command line arguments override config file values:

1. **Highest Priority:** Command line arguments (e.g., `--host`, `--v-ref`)
2. **Medium Priority:** Custom config file (if `--config` specified)
3. **Lowest Priority:** Default `config_example.yaml`

## Examples

### Autonomous Mode (No Remote Control)
```yaml
# In config_example.yaml
network:
  host_ip: null  # or comment out
```

### Remote Control Mode
```yaml
# In config_example.yaml
network:
  host_ip: 192.168.2.200  # GUI controller IP
  car_id: 0
```

### Multiple Cars with Different Configs
Create `car0_config.yaml`, `car1_config.yaml`, etc., then:
```bash
# Terminal 1 (Car 0)
python vehicle_control_refactored.py --config car0_config.yaml

# Terminal 2 (Car 1)
python vehicle_control_refactored.py --config car1_config.yaml
```

## Next Steps

1. **Test Config:** `python test_config.py`
2. **Start GUI:** (on host PC) `python gui_controller.py`
3. **Start Vehicle:** `python vehicle_control_refactored.py`
4. Watch logs for connection status
