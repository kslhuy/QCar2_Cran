# GUI Controller for QCar Fleet

A beautiful, modern tkinter-based GUI for controlling multiple physical QCars from your Host PC.

## 🎨 Features

### Visual Interface
- **Dark modern theme** - Easy on the eyes for long sessions
- **Real-time telemetry display** - Position, velocity, heading, throttle
- **Connection status indicators** - See which cars are connected at a glance
- **Activity log** - Track all commands and events
- **Color-coded feedback** - Success (green), warnings (yellow), errors (red)

### Control Features
- ✅ **Individual car control** - Start, stop, set velocity, change path for each car
- ✅ **Fleet control** - Start/stop all cars at once
- ✅ **Emergency stop** - Large red button with confirmation
- ✅ **Quick velocity presets** - Buttons for common speeds (0.3, 0.5, 0.7, 0.9 m/s)
- ✅ **Custom velocity** - Type any value between 0-2.0 m/s
- ✅ **Path configuration** - Change waypoint sequences on the fly
- ✅ **Live telemetry** - Updates at 10 Hz

## 📦 Installation

No additional packages needed! The GUI uses tkinter which comes with Python.

```bash
# Verify tkinter is available
python -c "import tkinter; print('Tkinter ready!')"
```

## 🚀 Usage

### Step 1: Start the GUI on Host PC

```bash
python gui_controller.py
```

You'll see a window like this:

```
┌─────────────────────────────────────────────────────────────┐
│          🚗 QCar Fleet Controller                           │
├──────────────────────────────┬──────────────────────────────┤
│                              │  Connection Status           │
│  Car 0                       │  Connected: 0/2              │
│  ⚫ Disconnected              │  Total Cars: 2               │
│  ┌─────────────────────────┐ │  Ports: 5000-5001           │
│  │ Telemetry               │ │                              │
│  │ Position: N/A           │ ├──────────────────────────────┤
│  │ Velocity: N/A           │ │  Activity Log                │
│  │ Heading:  N/A           │ │                              │
│  │ Throttle: N/A           │ │  [12:34:56] INFO: Started    │
│  └─────────────────────────┘ │  [12:34:57] INFO: Waiting... │
│  [▶ START]  [⬛ STOP]        │                              │
│  ...                         │                              │
└──────────────────────────────┴──────────────────────────────┘
```

### Step 2: Configure and Start QCars

On each physical QCar, edit `vehicle_control_remote.py`:

```python
HOST_PC_IP = '192.168.1.100'  # Your Host PC's IP
CAR_ID = 0  # 0 for first car, 1 for second, etc.
```

Then run:
```bash
python vehicle_control_remote.py
```

### Step 3: Control from GUI

Once cars are connected (green indicators), you can:

#### Individual Car Control
1. **Start/Stop**: Click the green START or red STOP button
2. **Set Velocity**: 
   - Type a value and click "Set"
   - Or click quick presets (0.3, 0.5, 0.7, 0.9)
3. **Change Path**: Enter node numbers (e.g., "10 4 20 10") and click "Set Path"
4. **Monitor**: Watch real-time telemetry update

#### Fleet Control
- **START ALL**: Start all connected cars at once
- **STOP ALL**: Stop all cars
- **🚨 EMERGENCY STOP 🚨**: Immediate stop with confirmation dialog

## 🎮 GUI Layout

### Car Control Panel (Each Car)
```
┌────────────────────────────────┐
│ Car 0              🟢 Connected │
├────────────────────────────────┤
│ Telemetry                      │
│ Position:  (1.23, 2.45) m      │
│ Velocity:  0.87 / 0.90 m/s     │
│ Heading:   -1.57 rad           │
│ Throttle:  0.12                │
├────────────────────────────────┤
│ [▶ START]      [⬛ STOP]       │
├────────────────────────────────┤
│ Velocity Control               │
│ Target: [0.9] [Set]            │
│ [0.3] [0.5] [0.7] [0.9]        │
├────────────────────────────────┤
│ Path Control                   │
│ Nodes: [10 4 20 10] [Set Path]│
└────────────────────────────────┘
```

### Right Panel
- **Connection Status**: Shows how many cars are connected
- **Activity Log**: Scrolling log with timestamps and color coding
  - 🔵 INFO: General information
  - 🟢 SUCCESS: Successful operations
  - 🟡 WARNING: Warnings
  - 🔴 ERROR: Errors

## ⚙️ Configuration

Edit the top of `gui_controller.py`:

```python
# Configuration
HOST_IP = '0.0.0.0'  # Listen on all interfaces
BASE_PORT = 5000     # First car uses 5000, second uses 5001, etc.
NUM_CARS = 2         # Number of cars in your fleet
```

## 🎨 Color Scheme

- **Background**: Dark gray (#2b2b2b, #3c3c3c)
- **Text**: White (#ffffff)
- **Success/Start**: Green (#4caf50)
- **Stop/Error**: Red (#f44336)
- **Command**: Blue (#2196f3)
- **Warning**: Yellow (#ffc107)
- **Connected**: Green indicator (🟢)
- **Disconnected**: Gray indicator (⚫)

## 🔒 Safety Features

1. **Emergency Stop Confirmation**: Requires confirmation before emergency stop
2. **Connection Monitoring**: Always shows connection status
3. **Activity Logging**: All actions are logged with timestamps
4. **Safe Close**: Asks to stop all cars before closing window
5. **Input Validation**: 
   - Velocity limited to 0-2.0 m/s
   - Path must have at least 2 nodes
   - Invalid inputs show warning messages

## 📊 Telemetry Updates

The GUI updates telemetry at **10 Hz** (every 100ms):
- Position (x, y) in meters
- Velocity (current / target) in m/s
- Heading in radians
- Throttle command

## 🐛 Troubleshooting

### GUI doesn't start
```bash
# Check if tkinter is installed
python -c "import tkinter"

# If error, install tkinter (Ubuntu/Debian)
sudo apt-get install python3-tk
```

### Cars not connecting
1. Check IP address in `vehicle_control_remote.py`
2. Verify firewall allows ports 5000-5001
3. Ensure all devices on same network
4. Check connection status in GUI

### Telemetry not updating
1. Verify car shows "Connected" (green indicator)
2. Check activity log for errors
3. Restart the car connection

### GUI freezes
- GUI is designed to be thread-safe and shouldn't freeze
- If it does, check your network connection
- Try restarting both GUI and car scripts

## 🎯 Keyboard Shortcuts

The GUI doesn't currently have keyboard shortcuts, but you can add them by editing `gui_controller.py`:

```python
# In __init__ method, add:
self.root.bind('<space>', lambda e: self.emergency_stop())
self.root.bind('<Escape>', lambda e: self.stop_all_cars())
```

## 📝 Tips

1. **Test with one car first** before adding more
2. **Keep velocity low** (< 0.5 m/s) when testing indoors
3. **Monitor the log** for any warning messages
4. **Use quick velocity buttons** for common speeds
5. **Save common paths** by noting down working node sequences
6. **Emergency stop is your friend** - don't hesitate to use it

## 🔧 Customization

### Change number of cars
```python
NUM_CARS = 3  # For 3 cars
```

### Change window size
```python
self.root.geometry("1400x900")  # Wider window
```

### Add more quick velocity buttons
In `create_car_panel` method:
```python
for vel in [0.3, 0.5, 0.7, 0.9, 1.2]:  # Added 1.2
    # ... button creation code ...
```

### Change update rate
```python
time.sleep(0.1)  # 10 Hz (change to 0.05 for 20 Hz)
```

## 🆚 GUI vs Command Line

| Feature | GUI | Command Line |
|---------|-----|--------------|
| Ease of use | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| Visual feedback | ⭐⭐⭐⭐⭐ | ⭐⭐ |
| Quick actions | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| Telemetry display | ⭐⭐⭐⭐⭐ | ⭐⭐ |
| Scripting | ⭐⭐ | ⭐⭐⭐⭐⭐ |
| Resource usage | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |

**Recommendation**: Use GUI for manual control, use command line interface for automation/scripting.

## 📸 Screenshots

The GUI features:
- Clean, modern dark theme
- Large, easy-to-click buttons
- Real-time updating displays
- Color-coded status indicators
- Professional layout

## 🚀 Next Steps

After mastering the basic GUI, consider:
1. Adding a 2D visualization map
2. Implementing formation control
3. Adding data recording/playback
4. Creating custom mission presets
5. Adding joystick/gamepad support

## 📄 License

Same as the main project.

## 🤝 Support

For issues:
1. Check the Activity Log in the GUI
2. Review this README
3. Check main README_REMOTE_CONTROL.md
4. Verify network connectivity

Happy controlling! 🚗💨
