# QCar Multi-Vehicle Self-Driving System - Refactored

## 🚀 Quick Start

Refactored multi-vehicle autonomous driving system for QCar platforms with improved architecture and automated deployment.

### ⚡ Run the System

```powershell

Edit `config.txt` with your IPs , and the real car

# Start GUI 
python .\enhanced_gui_controller.py

# Start all vehicles (in another terminal)
.\start_refactored.bat

# Stop all vehicles (in same terminal of the start)
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




### Development Mode
```powershell
# Run with debug logging
python vehicle_main.py --config debug_config.yaml

# Test with fake vehicles (no hardware)
cd qcar/GUI
python fake_vehicle_real_logic.py 0
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



