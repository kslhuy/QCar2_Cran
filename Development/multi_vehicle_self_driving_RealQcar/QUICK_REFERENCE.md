# Enhanced Start System - Quick Reference

## Quick Start Commands

```bash
# Start all vehicles with default config
start_enhanced.bat

# Quick restart (skip upload)
python python/start_enhanced.py --skip-upload --yes

# Use custom config
python python/start_enhanced.py --config my_fleet.yaml

# Disable logging
python python/start_enhanced.py --no-logs
```

## Config File Quick Edit

Edit `fleet_config.yaml`:

```yaml
vehicles:
  - car_id: 0           # Vehicle ID
    ip: 192.168.2.137   # IP address
    enabled: true       # Start this vehicle?
    probing: true       # Enable object detection?
    path_number: 0      # Which path? (0, 1, or 2)
    initial_v_ref: 0.75 # Speed in m/s
```

## Common Tasks

### Change Vehicle Speed
```yaml
initial_v_ref: 0.80  # Change from 0.75 to 0.80 m/s
```

### Change Path
```yaml
path_number: 1  # Change from 0 to 1 (different route)
```

### Disable Vehicle
```yaml
enabled: false  # Vehicle won't start
```

### Enable Object Detection
```yaml
probing: true  # Enable YOLO detection
```

## Path Quick Reference

- **Path 0**: [10→2→4→6→8→10] - Main circuit
- **Path 1**: [0→2→4→6→8→10→2→4→6→0] - Extended loop  
- **Path 2**: [2→4→6→8→10→2] - Inner circuit

## Command Line Flags

| Flag | Description |
|------|-------------|
| `--config FILE` | Use custom config file |
| `--skip-upload` | Don't upload files (faster restart) |
| `--no-logs` | Disable remote logging |
| `--yes` | Skip confirmation prompt |

## Troubleshooting One-Liners

```bash
# Check if vehicle is running
ssh nvidia@192.168.2.137 "pgrep -f vehicle_main"

# View vehicle logs
ssh nvidia@192.168.2.137 "tail -f /home/nvidia/Documents/multi_vehicle_RealCar/vehicle_0.log"

# Kill stuck process
ssh nvidia@192.168.2.137 "pkill -f vehicle_main"

# Check disk space
ssh nvidia@192.168.2.137 "df -h"
```

## Example Configs

### Single Vehicle Testing
```yaml
vehicles:
  - car_id: 0
    ip: 192.168.2.137
    enabled: true
    probing: true
    path_number: 0
    initial_v_ref: 0.75
```

### Two-Vehicle Platoon
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

### Different Speeds
```yaml
vehicles:
  - car_id: 0
    initial_v_ref: 0.70  # Slower leader
    
  - car_id: 1
    initial_v_ref: 0.80  # Faster follower
```

## File Locations

- Config: `fleet_config.yaml`
- Start script: `python/start_enhanced.py`
- Batch file: `start_enhanced.bat`
- Vehicle code: `qcar/vehicle_main.py`
- Full docs: `README_ENHANCED_START.md`
