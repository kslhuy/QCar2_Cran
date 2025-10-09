# Multi-Vehicle Startup System (Refactored)

## Overview

Automated startup and shutdown scripts for the refactored multi-vehicle control system. These scripts read configuration from `config.txt` and automatically:
- Detect the number of vehicles
- Upload all necessary files (Python, YAML, TXT, MD)
- Start vehicle control with proper car IDs and ports
- Start YOLO servers
- Provide clean shutdown

## Configuration File

Edit `config.txt` in the parent directory:

```txt
REMOTE_PATH=/home/nvidia/Documents/multi_vehicle_RealCar
QCAR_IPS=[192.168.2.108 , 192.168.2.30]
CALIBRATE_IP=192.168.2.108
PROBING_IP=192.168.2.108
LOCAL_IP=192.168.2.200
WIDTH=320
HEIGHT=200
```

### Configuration Parameters

- **REMOTE_PATH**: Path on QCar where scripts are stored
- **QCAR_IPS**: Comma-separated list of QCar IP addresses (in brackets)
- **CALIBRATE_IP**: IP of QCar used for calibration
- **PROBING_IP**: IP of QCar that sends images to observer
- **LOCAL_IP**: Your host PC IP address
- **WIDTH/HEIGHT**: Observer display resolution

## Files

### Python Scripts

1. **`start_refactored.py`** - Start all vehicles
   - Reads config.txt automatically
   - Uploads all files to QCars
   - Starts vehicle_control_refactored.py with proper parameters
   - Starts YOLO servers
   - Starts observer on probing QCar

2. **`stop_refactored.py`** - Stop all vehicles
   - Cleanly stops all processes
   - Kills vehicle_control and yolo_server
   - Verifies shutdown

### Batch Files (Windows)

1. **`start_refactored.bat`** - Double-click to start
2. **`stop_refactored.bat`** - Double-click to stop

## Usage

### Method 1: Using Batch Files (Easiest)

```powershell
# Start all vehicles
.\start_refactored.bat

# Stop all vehicles
.\stop_refactored.bat
```

### Method 2: Using Python Directly

```powershell
# Start with default config.txt
python start_refactored.py

# Start with custom config
python start_refactored.py --config /path/to/config.txt

# Start without re-uploading files (faster restart)
python start_refactored.py --skip-upload

# Stop all vehicles
python stop_refactored.py

# Stop with custom config
python stop_refactored.py --config /path/to/config.txt
```

## How It Works

### Startup Sequence

For each QCar in `QCAR_IPS`:

1. **Connect** via SSH to QCar
2. **Upload Files** (unless `--skip-upload`):
   - All `.py` files (including refactored code)
   - All `.yaml` configuration files
   - All `.txt` files (requirements, etc.)
   - All `.md` documentation files
3. **Stop** any existing processes
4. **Start vehicle_control_refactored.py** with:
   - `--host <LOCAL_IP>` (host PC IP)
   - `--port <BASE_PORT>` (5000)
   - `--car-id <index>` (0, 1, 2, ...)
5. **Start yolo_server.py** with:
   - `-p True/False` (probing flag)
   - `-i <LOCAL_IP>` (host PC IP)
   - `-w <WIDTH>` (observer width)
   - `-ht <HEIGHT>` (observer height)
6. **Start observer.py** locally (only for probing QCar)

### Car ID Assignment

Car IDs are automatically assigned based on position in `QCAR_IPS`:
- First IP → Car ID 0, Port 5000
- Second IP → Car ID 1, Port 5001
- Third IP → Car ID 2, Port 5002
- etc.

### Port Assignment

- Base port: 5000 (configurable in script)
- Each car gets: BASE_PORT + CAR_ID
- Example:
  - Car 0: Port 5000
  - Car 1: Port 5001
  - Car 2: Port 5002

## Example Workflow

### 1. Initial Setup

Edit `config.txt`:
```txt
QCAR_IPS=[192.168.2.108 , 192.168.2.30 , 192.168.2.50]
LOCAL_IP=192.168.2.200
```

### 2. Start All Vehicles

```powershell
.\start_refactored.bat
```

Output:
```
======================================================================
 Multi-Vehicle Refactored Control System - Startup
======================================================================

Configuration:
  Number of QCars: 3
  QCar IPs: 192.168.2.108, 192.168.2.30, 192.168.2.50
  Host PC IP: 192.168.2.200
  Base Port: 5000
...

======================================================================
 Starting QCar 0: 192.168.2.108 (Port: 5000)
======================================================================
  [✓] Connected
  [✓] Uploaded 15 Python files
  [✓] Uploaded 2 YAML configuration files
  [✓] Uploaded 1 text files
  [✓] Observer started
  [✓] Vehicle control started (Car ID: 0)
  [✓] YOLO server started (Probing: True)
...
```

### 3. Monitor

View logs on QCar:
```bash
ssh nvidia@192.168.2.108
cd /home/nvidia/Documents/multi_vehicle_RealCar
tail -f vehicle_0.log
```

### 4. Stop All Vehicles

```powershell
.\stop_refactored.bat
```

## Advanced Usage

### Quick Restart (Skip Upload)

```powershell
# Stop
python stop_refactored.py

# Restart without re-uploading files
python start_refactored.py --skip-upload
```

This is much faster for testing parameter changes in YAML files that are already on the QCars.

### Custom Configuration

```powershell
# Create custom config
cp ../config.txt test_config.txt
# Edit test_config.txt...

# Start with custom config
python start_refactored.py --config test_config.txt
```

### Single Vehicle Testing

Edit `config.txt` to include only one IP:
```txt
QCAR_IPS=[192.168.2.108]
```

### Different Network Setup

For different subnets, just update IPs in `config.txt`:
```txt
QCAR_IPS=[10.0.0.101 , 10.0.0.102]
LOCAL_IP=10.0.0.1
```

## Troubleshooting

### Connection Timeout

**Error**: `SSH connection timeout`

**Solutions**:
- Verify QCar is powered on
- Check network connectivity: `ping 192.168.2.108`
- Verify IP addresses in config.txt
- Check firewall settings

### File Upload Failed

**Error**: `SCP transfer failed`

**Solutions**:
- Verify REMOTE_PATH exists on QCar
- Check disk space on QCar: `df -h`
- Verify SSH credentials (nvidia/nvidia)

### Process Not Starting

**Error**: `Vehicle control not starting`

**Solutions**:
- SSH to QCar and check logs
- Verify Python environment: `which python`
- Check for missing dependencies
- View error logs: `tail -f vehicle_0.log`

### Observer Not Showing Images

**Error**: `Observer window empty`

**Solutions**:
- Verify PROBING_IP is correct
- Check if observer.py exists in python directory
- Verify LOCAL_IP is reachable from QCar
- Check WIDTH and HEIGHT settings

### Port Already in Use

**Error**: `Address already in use`

**Solutions**:
- Stop existing processes: `python stop_refactored.py`
- Change BASE_PORT in start_refactored.py
- Check for zombie processes: `netstat -an | findstr 5000`

## Integration with Refactored System

This startup system is designed for the refactored vehicle control:

### Files Uploaded

- `vehicle_control_refactored.py` - Main entry point
- `vehicle_controller.py` - Controller class
- `config.py` - Configuration management
- `controllers.py` - Speed/steering controllers
- `state_machine.py` - State management
- `network_client.py` - Network communication
- `safety.py` - Safety systems
- `logging_utils.py` - Logging utilities
- `config_example.yaml` - Example configuration
- `*.yaml` - Configuration files
- `*.txt` - Requirements and documentation

### Command Line Arguments

The startup script passes these arguments to `vehicle_control_refactored.py`:

```bash
python vehicle_control_refactored.py \
  --host 192.168.2.200 \
  --port 5000 \
  --car-id 0
```

You can add more arguments by editing `start_refactored.py`.

## Comparison: Original vs Refactored Startup

| Feature | Original | Refactored |
|---------|----------|------------|
| Config file | Manual arguments | config.txt |
| Car ID | Manual | Automatic |
| Port assignment | Manual | Automatic |
| File types | .py only | .py, .yaml, .txt, .md |
| Error handling | Basic | Comprehensive |
| Logs | Console only | Per-vehicle logs |
| Cleanup | Manual kill | Clean shutdown |
| Observer | Manual start | Automatic |

## Logging

### On QCar

Each vehicle generates logs:
- `vehicle_<car_id>.log` - Vehicle control system log
- `yolo_<car_id>.log` - YOLO server log

View in real-time:
```bash
ssh nvidia@192.168.2.108
cd /home/nvidia/Documents/multi_vehicle_RealCar
tail -f vehicle_0.log
```

### On Host PC

The refactored system creates:
- `logs/vehicle_<car_id>_<timestamp>.log` - Detailed logs
- `data_logs/run_<timestamp>/telemetry_vehicle_<car_id>.csv` - Telemetry

## Tips & Best Practices

1. **Always stop before starting**: Run `stop_refactored.bat` before `start_refactored.bat`

2. **Use --skip-upload for quick tests**: After initial upload, use `--skip-upload` flag

3. **Check config.txt first**: Verify IPs and paths before starting

4. **Monitor first vehicle**: Watch vehicle_0.log to catch early errors

5. **Test with one vehicle first**: Start with single QCar, then add more

6. **Keep config.txt in version control**: Track different setups

7. **Use descriptive config names**: e.g., `config_lab.txt`, `config_outdoor.txt`

## Next Steps

After starting vehicles:

1. **Verify all started**: Check that all QCars respond
2. **Monitor telemetry**: View CSV logs for data
3. **Tune parameters**: Edit YAML configs and restart
4. **Test coordination**: Verify multi-vehicle behavior
5. **Collect data**: Analyze telemetry logs after runs

## Support

See also:
- `../qcar/REFACTORING_README.md` - Full system documentation
- `../qcar/QUICKSTART.md` - Quick start guide
- `../qcar/IMPLEMENTATION_SUMMARY.md` - Implementation details
