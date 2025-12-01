# QCar Multi-Vehicle Self-Driving System - Refactored

## 🚀 Quick Start

Refactored multi-vehicle autonomous driving system for QCar platforms with improved architecture and automated deployment.

### ⚡ Run the System

```powershell
# Start all vehicles
.\start_refactored.bat

# Stop all vehicles
.\stop_enhanced.bat
```

## 📁 Project Structure

```
multi_vehicle_self_driving_RealQcar/
├── config.txt                    # Main configuration
├── start_refactored.bat         # Start all vehicles
├── stop_enhanced.bat          # Stop all vehicles
├── python/                      # Startup scripts → See python/STARTUP_README.md
├── qcar/                        # Core system
│   ├── vehicle_main.py         # Main entry point
│   ├── vehicle_logic.py        # Core controller
│   ├── StateMachine/           # State management → See StateMachine/README.md
│   ├── Controller/             # Speed/steering control → See Controller/README.md
│   ├── Observer/               # Vehicle observer → See Observer/README.md
│   ├── V2V/                    # Vehicle communication → See V2V/README.md
│   ├── Yolo/                   # Object detection → See Yolo/README.md
│   ├── GUI/                    # Testing tools
│   └── Doc/                    # Detailed documentation
└── reference_scan/             # Calibration data
```

## 🔧 Configuration

Edit `config.txt` for your network:

```txt
QCAR_IPS=[192.168.137.102 , 192.168.137.208]
LOCAL_IP=192.168.137.1
REMOTE_PATH=/home/nvidia/Documents/multi_vehicle_RealCar
```

## 🎯 Usage Examples

### Single Vehicle Test
```powershell
cd qcar
python vehicle_main.py --car-id 0
```

### Multi-Vehicle with Custom Config
```powershell
python vehicle_main.py --config my_config.yaml --car-id 0
```

### Quick Restart (Skip Upload)
```powershell
python python/start_refactored.py --skip-upload
```

## 📊 Monitoring

### View Logs
```powershell
# Real-time logs
Get-Content qcar\logs\vehicle_0_*.log -Wait -Tail 20

# Telemetry data
$latest = Get-ChildItem qcar\data_logs | Sort-Object LastWriteTime -Descending | Select-Object -First 1
Import-Csv "$($latest.FullName)\telemetry_vehicle_0.csv" | Format-Table
```

## 🛠️ System Architecture

### Refactored Improvements
- ✅ **Configuration**: YAML/JSON files instead of hardcoded values
- ✅ **State Machine**: Proper state management vs boolean flags
- ✅ **Safety**: Multi-layered safety systems
- ✅ **Logging**: Structured logging + telemetry
- ✅ **Network**: Robust communication with retry logic
- ✅ **Deployment**: Automated startup/shutdown

### State Flow
```
INITIALIZING → WAITING_FOR_START → FOLLOWING_PATH ⇄ FOLLOWING_LEADER
                      ↑                   ↓               ↓
                      ←──────────── STOPPED ←─────────────┘
```

## 🔧 Troubleshooting

### Connection Issues
```powershell
# Test connectivity
ping 192.168.137.102

# Check SSH
ssh nvidia@192.168.137.102
```

### YOLO Server Problems
```powershell
# Check dependencies on QCar
ssh nvidia@192.168.137.102 "python -c 'import torch, cv2; print(\"OK\")'"

# Manual YOLO start for debugging
ssh nvidia@192.168.137.102
cd /home/nvidia/Documents/multi_vehicle_RealCar
python yolo_server.py -i 192.168.137.1 -p True
```

### Stop All Processes
```powershell
# Emergency stop
.\stop_refactored.bat

# Manual cleanup
taskkill /F /IM python.exe
```

## 📚 Documentation

- **Startup System**: `python/STARTUP_README.md`
- **Complete Guide**: `qcar/Doc/REFACTORING_README.md`
- **Quick Start**: `qcar/Doc/QUICKSTART.md`
- **State Machine**: `qcar/StateMachine/README.md`

## 🚀 Next Steps

1. ✅ Edit `config.txt` with your IPs
2. ✅ Run `.\start_refactored.bat`
3. ✅ Monitor logs for errors
4. ✅ Tune parameters in YAML configs
5. ✅ Scale to multiple vehicles

---

**Version**: Refactored System (December 2024)  
**Platform**: Windows + QCar Hardware

## 🔧 Configuration

### Main Configuration File (`config.txt`)

Edit this file to configure your system:

```txt
REMOTE_PATH=/home/nvidia/Documents/multi_vehicle_RealCar
QCAR_IPS=[192.168.137.102 , 192.168.137.208]
CALIBRATE_IP=192.168.137.102
PROBING_IP=192.168.137.102 , 192.168.137.208
LOCAL_IP=192.168.137.1
WIDTH=320
HEIGHT=200
```

**Configuration Parameters:**
- `QCAR_IPS`: List of QCar IP addresses
- `LOCAL_IP`: Your host PC IP address
- `CALIBRATE_IP`: IP of QCar used for calibration
- `PROBING_IP`: IPs of QCars that send images to observer
- `REMOTE_PATH`: Path on QCars where scripts are stored
- `WIDTH/HEIGHT`: Observer display resolution

### Vehicle Configuration (`qcar/config_example.yaml`)

Create custom vehicle configurations:

```yaml
speed:
  v_ref: 0.75              # Reference velocity (m/s)
  K_p: 0.1                 # Proportional gain
  K_i: 1.0                 # Integral gain
  max_throttle: 0.3        # Maximum throttle

steering:
  K_stanley: 0.7           # Stanley controller gain
  enable_steering_control: true

safety:
  emergency_stop_distance: 0.2  # Emergency stop distance (m)
  gps_timeout_max: 100          # GPS timeout threshold

logging:
  log_level: "INFO"        # Logging level
  console_output: true     # Console output
```

## 🎯 Running the System

### Method 1: Automated Multi-Vehicle (Recommended)

**Start all vehicles:**
```powershell
.\start_refactored.bat
```

**Stop all vehicles:**
```powershell
.\stop_refactored.bat
```

### Method 2: Manual Control

**Single vehicle:**
```powershell
cd qcar
python vehicle_main.py --car-id 0
```

**Multiple vehicles:**
```powershell
# Terminal 1 - Car 0
python vehicle_main.py --car-id 0 --port 5000

# Terminal 2 - Car 1  
python vehicle_main.py --car-id 1 --port 5001
```

**With custom configuration:**
```powershell
python vehicle_main.py --config my_config.yaml --car-id 0
```

### Method 3: Advanced Options

```powershell
# Skip file upload (faster restart)
python python/start_refactored.py --skip-upload

# Use custom configuration file
python python/start_refactored.py --config test_config.txt

# Run with specific parameters
python qcar/vehicle_main.py --host 192.168.1.100 --port 5000 --car-id 0 --v-ref 0.8
```

## 📊 Monitoring and Logs

### Real-time Monitoring



### Log Files Generated

```
qcar/
├── logs/
│   └── vehicle_0_20251201_143000.log    # Main system logs
└── data_logs/
    └── run_20251201_143000/
        └── telemetry_vehicle_0.csv       # Telemetry data for analysis
```

## 🛠️ System Architecture (Refactored)

### Key Improvements Over Original System

| Feature | Original System | Refactored System |
|---------|----------------|-------------------|
| Configuration | Hardcoded values | YAML/JSON config files |
| Error Handling | Basic try-catch | Comprehensive safety systems |
| State Management | Boolean flags | Proper state machine |
| Logging | Print statements | Structured logging + telemetry |
| Network Communication | Simple sockets | Robust client with retry logic |
| Safety Features | Minimal | Multi-layered safety systems |
| Performance Monitoring | None | Built-in performance metrics |
| Startup Process | Manual commands | Automated deployment |

### State Machine

The refactored system uses a clear state machine:

```
INITIALIZING → WAITING_FOR_START → FOLLOWING_PATH ⇄ FOLLOWING_LEADER
                      ↑                   ↓               ↓
                      ←──────────── STOPPED ←─────────────┘
```

**States:**
- `INITIALIZING`: System startup and component verification
- `WAITING_FOR_START`: Ready state, waiting for commands
- `FOLLOWING_PATH`: Autonomous path following using waypoints
- `FOLLOWING_LEADER`: Following another vehicle (platoon mode)
- `STOPPED`: Vehicle stopped (manual, safety, or emergency)



## 🧪 Testing and Development

### Run Tests
```powershell
cd qcar
python -m pytest test_components.py -v
```

### Development Mode
```powershell
# Run with debug logging
python vehicle_main.py --config debug_config.yaml

# Test with fake vehicles (no hardware)
cd qcar/GUI
python fake_vehicle_real_logic.py 0
```

### GUI Testing Tools
```powershell
cd qcar/GUI
python enhanced_gui_controller.py    # GUI controller for testing
python enhanced_remote_controller.py # Remote controller interface
```

## 🔄 Migration from Original System

If you're migrating from the original `vehicle_control_real.py` system:

1. **Backup your current setup:**
   ```powershell
   cp qcar/vehicle_control_real.py qcar/vehicle_control_real_backup.py
   ```

2. **Extract your parameters to a config file:**
   ```yaml
   speed:
     v_ref: 0.75  # Your current v_ref value
     K_p: 0.1     # Your current K_p value
     K_i: 1.0     # Your current K_i value
   ```

3. **Test the new system:**
   ```powershell
   python qcar/vehicle_main.py --config my_config.yaml
   ```

### Software
- Python 3.7+
- Windows PowerShell (for .bat files)
- SSH access to QCars
- Required Python packages (see `qcar/requirements_refactored.txt`)

### Network
- Stable network connection between host PC and QCars
- Proper IP configuration as specified in `config.txt`
- SSH access enabled on QCars

## 🏁 Next Steps


---

**Version:** Refactored System (December 2024)
**Compatibility:** Windows PowerShell, QCar Hardware Platforms
**License:** Same as original QCar project