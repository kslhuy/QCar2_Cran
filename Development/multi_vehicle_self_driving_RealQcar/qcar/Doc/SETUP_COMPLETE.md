# ✅ SETUP COMPLETE - Vehicle Control Configuration

## What Changed

### 1. **Automatic Config Loading**
`vehicle_control_refactored.py` now **automatically loads `config_example.yaml`** from the script directory.

**Before:**
```bash
python vehicle_control_refactored.py  # Used hardcoded defaults
```

**After:**
```bash
python vehicle_control_refactored.py  # Automatically loads config_example.yaml
```

### 2. **GUI Controller Fixed**
`gui_controller.py` now listens on `0.0.0.0` (all network interfaces) instead of a specific IP.

**Before:**
```python
HOST_IP = '192.168.2.200'  # Only listened on specific interface
```

**After:**
```python
HOST_IP = '0.0.0.0'  # Listens on ALL interfaces - vehicles can connect!
```

## 🚀 Quick Start Guide

### Step 1: Configure Your Vehicle
Edit `config_example.yaml`:
```yaml
network:
  host_ip: 192.168.2.200  # ← Change this to YOUR GUI PC's IP address
  car_id: 0               # ← Car 0, 1, 2, etc.
```

### Step 2: Start GUI Controller (On Host PC)
```bash
cd Development/QCar2_multi-vehicle_control
python gui_controller.py
```

You should see:
```
[Host PC] Listening for Car 0 on port 5000
[Host PC] Listening for Car 1 on port 5001
```

### Step 3: Start Vehicle (On QCar)
```bash
cd Development/multi_vehicle_self_driving_RealQcar/qcar
python vehicle_control_refactored.py
```

Or use the quick start script:
```bash
start_vehicle.bat 0 192.168.2.200
```

### Step 4: Verify Connection
**On GUI:** You should see:
```
[Host PC] Car 0 connected from <vehicle-ip>
⚫ Disconnected → 🟢 Connected
```

**On Vehicle:** You should see:
```
[INIT] Loading configuration from: config_example.yaml
  Remote control: Enabled
    Host: 192.168.2.200:5000
[READY] Network connection established
```

## 📝 Configuration Summary

Your `config_example.yaml` currently has:
- **Host IP:** `192.168.2.200` (GUI controller location)
- **Base Port:** `5000` (Car 0 = 5000, Car 1 = 5001, etc.)
- **Car ID:** `0` (can be overridden with `--car-id`)
- **Reference Velocity:** `0.75 m/s`
- **Remote Control:** **ENABLED** ✅

## 🔧 Testing Commands

### Test Configuration Loading
```bash
cd qcar
python test_config.py
```

### Start Vehicle with Default Config
```bash
python vehicle_control_refactored.py
```

### Start Vehicle with Custom Car ID
```bash
python vehicle_control_refactored.py --car-id 1
```

### Start Vehicle with Different Host
```bash
python vehicle_control_refactored.py --host 192.168.1.100 --car-id 0
```

### Start Vehicle with Custom Velocity
```bash
python vehicle_control_refactored.py --v-ref 0.9
```

## 🐛 Troubleshooting

### Problem: "Remote control disabled"
**Solution:** Check `config_example.yaml` has `host_ip` set:
```yaml
network:
  host_ip: 192.168.2.200  # Must be set, not null
```

### Problem: "Connection refused"
**Solutions:**
1. Verify GUI is running first
2. Check IP address is correct: `ping 192.168.2.200`
3. Check firewall allows port 5000
4. Verify both on same network

### Problem: GUI shows "Disconnected"
**Solutions:**
1. Make sure GUI is using `HOST_IP = '0.0.0.0'` ✅ (already fixed)
2. Check vehicle is connecting to correct IP
3. Check port numbers match (5000, 5001, etc.)

### Problem: Config not loading
**Solution:** Verify file exists:
```bash
cd qcar
dir config_example.yaml
```

## 📂 File Locations

```
qcar2/Development/
├── QCar2_multi-vehicle_control/
│   └── gui_controller.py          ← GUI (Host PC)
│
└── multi_vehicle_self_driving_RealQcar/qcar/
    ├── vehicle_control_refactored.py  ← Main vehicle script
    ├── config_example.yaml            ← Configuration (auto-loaded)
    ├── test_config.py                 ← Test config loading
    ├── start_vehicle.bat              ← Quick start script
    └── VEHICLE_CONTROL_GUIDE.md       ← Full documentation
```

## ✅ Checklist

Before running your vehicle:

- [ ] `config_example.yaml` exists in `qcar` directory
- [ ] `host_ip` in config matches your GUI PC's IP address
- [ ] GUI controller is running and shows "Listening on port..."
- [ ] Both GUI PC and vehicle are on same network
- [ ] Firewall allows port 5000 (and 5001, 5002 for multiple cars)
- [ ] Run `python test_config.py` to verify config loads

## 🎯 Next Steps

1. **Test config:** `python test_config.py`
2. **Start GUI** (on host PC)
3. **Start vehicle:** `python vehicle_control_refactored.py`
4. **Check logs** for "Connected" status
5. **Control from GUI** - Start/Stop buttons should work!

## 📖 Full Documentation

See `VEHICLE_CONTROL_GUIDE.md` for complete documentation including:
- All command line options
- Multi-vehicle setup
- Advanced configuration
- Detailed troubleshooting

---
**Date:** October 8, 2025
**Status:** ✅ Ready to use
