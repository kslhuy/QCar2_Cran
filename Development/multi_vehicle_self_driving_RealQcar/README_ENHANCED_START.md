# Enhanced Multi-Vehicle Control System

## Overview

The enhanced start script provides a flexible, YAML-based configuration system for managing multiple QCar vehicles with individual settings. Each vehicle can have its own path, speed, and operational parameters.

## Features

- **YAML Configuration**: Easy-to-read configuration file with per-vehicle settings
- **Individual Vehicle Settings**: Each vehicle can have different:
  - Path number (route selection)
  - Initial velocity reference
  - Probing mode (YOLO object detection)
  - Calibration settings
  - Traffic mode (left/right hand traffic)
- **Path Management**: Pre-defined paths with node sequences
- **Flexible Deployment**: Skip uploads for quick restarts, enable/disable logging
- **Visual Feedback**: Clear startup progress and status reporting

## Configuration File

The main configuration file is `fleet_config.yaml`. It contains:

### Ground Station Configuration
```yaml
ground_station:
  local_ip: 192.168.2.200
  base_port: 5000
```

### Vehicle Configuration
```yaml
vehicles:
  - car_id: 0
    ip: 192.168.2.137
    enabled: true
    probing: true
    calibrate: false
    path_number: 0
    left_hand_traffic: false
    initial_v_ref: 0.75
    description: "Leader vehicle with probing"
```

### Path Definitions
```yaml
paths:
  0:
    name: "Main Circuit"
    nodes: [10, 2, 4, 6, 8, 10]
    description: "Default circuit starting from node 10"
```

## Usage

### Basic Start
```bash
start_enhanced.bat
```

Or directly with Python:
```bash
python python/start_enhanced.py
```

### Advanced Options

#### Use custom config file
```bash
python python/start_enhanced.py --config custom_fleet.yaml
```

#### Skip file upload (quick restart)
```bash
python python/start_enhanced.py --skip-upload
```

#### Disable remote logging
```bash
python python/start_enhanced.py --no-logs
```

#### Auto-confirm (no prompt)
```bash
python python/start_enhanced.py --yes
```

#### Combined options
```bash
python python/start_enhanced.py --skip-upload --no-logs --yes
```

## Vehicle Configuration Options

### Required Fields
- `car_id`: Unique identifier (0, 1, 2, ...)
- `ip`: Vehicle IP address
- `enabled`: Whether to start this vehicle (true/false)

### Optional Fields
- `probing`: Enable YOLO object detection (default: false)
- `calibrate`: Start in calibration mode (default: false)
- `path_number`: Path selection 0-2 (default: 0)
- `left_hand_traffic`: Traffic mode (default: false)
- `initial_v_ref`: Initial velocity in m/s (default: 0.75)
- `description`: Human-readable description

## Path Numbers

### Path 0: Main Circuit
- Nodes: [10, 2, 4, 6, 8, 10]
- Description: Standard circuit starting from node 10

### Path 1: Extended Loop
- Nodes: [0, 2, 4, 6, 8, 10, 2, 4, 6, 0]
- Description: Longer path with multiple loops

### Path 2: Inner Circuit
- Nodes: [2, 4, 6, 8, 10, 2]
- Description: Inner circuit without node 0

## Adding New Vehicles

1. Open `fleet_config.yaml`
2. Add a new vehicle entry under `vehicles:`
```yaml
  - car_id: 2
    ip: 192.168.2.XXX
    enabled: true
    probing: false
    path_number: 0
    initial_v_ref: 0.75
    description: "Second follower"
```
3. Save and run `start_enhanced.bat`

## Adding New Paths

1. Open `fleet_config.yaml`
2. Add a new path under `paths:`
```yaml
  3:
    name: "Custom Path"
    nodes: [10, 4, 6, 8, 10]
    description: "Custom route description"
```
3. Reference in vehicle config using `path_number: 3`

## Node Reference

The system uses 11 waypoint nodes (0-10) positioned on the track:

- **Node 0**: [0.0, 0.13024, -1.5707963267948966]
- **Node 2**: [1.12739, -1.084655, 0.0]
- **Node 4**: [2.25478, 0.0814, 1.5707963267948966]
- **Node 6**: [1.01343, 1.100935, 3.141592653589793]
- **Node 8**: [-0.74888, 1.100935, 3.141592653589793]
- **Node 10**: [-1.28205, -0.45991, -0.7330382858376184] (Default start)

(See `fleet_config.yaml` for complete node list with descriptions)

## Startup Sequence

1. **Load Configuration**: Read and validate `fleet_config.yaml`
2. **Display Summary**: Show all vehicles and their settings
3. **Confirm Start**: Prompt user (unless `--yes` flag used)
4. **Start Probing**: Launch multi-probing for vehicles with `probing: true`
5. **Start Vehicles**: For each enabled vehicle:
   - Connect via SSH
   - Upload files (unless `--skip-upload`)
   - Kill existing processes
   - Start `vehicle_main.py` with custom parameters
   - Start `yolo_server.py`
   - Wait 3 seconds before next vehicle
6. **Display Summary**: Show startup results and next steps

## Monitoring

### View Logs on QCar
```bash
ssh nvidia@192.168.2.XXX
cd /home/nvidia/Documents/multi_vehicle_RealCar
tail -f vehicle_0.log
tail -f yolo_0.log
```

### Stop All Vehicles
```bash
python python/stop_refactored.py --config fleet_config.yaml
```

## Troubleshooting

### Vehicle Won't Start
- Check IP address is correct in config
- Verify SSH connection: `ssh nvidia@192.168.2.XXX`
- Check if vehicle is already running: `ssh nvidia@192.168.2.XXX "pgrep -f vehicle_main"`

### File Upload Fails
- Verify remote path exists: `ssh nvidia@192.168.2.XXX "ls /home/nvidia/Documents/multi_vehicle_RealCar"`
- Check disk space: `ssh nvidia@192.168.2.XXX "df -h"`

### Probing Not Working
- Ensure `probing: true` in config
- Check if multi_probing.py exists in `python/` folder
- Verify YOLO server logs: `tail -f yolo_0.log`

## Example Configurations

### Two-Vehicle Platoon (Leader + Follower)
```yaml
vehicles:
  - car_id: 0
    ip: 192.168.2.137
    enabled: true
    probing: true
    path_number: 0
    initial_v_ref: 0.75
    
  - car_id: 1
    ip: 192.168.2.108
    enabled: true
    probing: false
    path_number: 0
    initial_v_ref: 0.75
```

### Three-Vehicle Different Paths
```yaml
vehicles:
  - car_id: 0
    ip: 192.168.2.137
    path_number: 0
    initial_v_ref: 0.70
    
  - car_id: 1
    ip: 192.168.2.108
    path_number: 1
    initial_v_ref: 0.80
    
  - car_id: 2
    ip: 192.168.2.109
    path_number: 2
    initial_v_ref: 0.75
```

### Testing Single Vehicle
```yaml
vehicles:
  - car_id: 0
    ip: 192.168.2.137
    enabled: true
    probing: true
    calibrate: true
    path_number: 0
    
  - car_id: 1
    ip: 192.168.2.108
    enabled: false  # Disabled
```

## Benefits Over Old System

1. **Flexibility**: Each vehicle has independent configuration
2. **Maintainability**: Single YAML file instead of multiple config files
3. **Scalability**: Easy to add/remove vehicles
4. **Documentation**: Self-documenting configuration with descriptions
5. **Path Management**: Centralized path definitions
6. **Quick Testing**: Enable/disable vehicles without editing code
7. **Version Control**: YAML format is git-friendly

## Migration from Old System

Old `config.txt`:
```
QCAR_IPS=[192.168.2.137 , 192.168.2.108]
LOCAL_IP=192.168.2.200
```

New `fleet_config.yaml`:
```yaml
ground_station:
  local_ip: 192.168.2.200
  
vehicles:
  - car_id: 0
    ip: 192.168.2.137
  - car_id: 1
    ip: 192.168.2.108
```

The new system is backward compatible - just run `start_enhanced.bat` instead of `start_refactored.bat`.
