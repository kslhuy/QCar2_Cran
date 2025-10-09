# Easy Fleet Configuration Guide

## Overview
This new configuration system makes it super easy to modify your QCar fleet simulation parameters without touching any Python code. Just edit the `config.txt` file!

## Quick Start

### 1. Basic Configuration
Edit `config.txt` to change simulation parameters:

```ini
[simulation]
time = 40                          # How long to run (seconds)
road_type = "Studio"               # "OpenRoad" or "Studio"  
controller_type = "CACC"           # "CACC" or "IDM"

[fleet]
num_vehicles = 2                   # Number of cars (2-5)
leader_index = 0                   # Which car leads (0-based)
distance_between_cars = 0.2        # Space between cars (meters)
```

### 2. Control Parameters
Adjust vehicle behavior:

```ini
[control]
max_velocity = 0.5                 # Top speed (m/s)
max_steering = 0.6                 # Max steering angle (radians)
lookahead_distance = 0.5           # Path following distance (meters)
```

### 3. Following Behavior
Fine-tune how cars follow each other:

```ini
[controller_params]
alpha = 1.2                        # How quickly cars accelerate
beta = 2.0                         # How quickly cars brake
v0 = 0.5                           # Desired speed (m/s)
T = 0.3                            # Time gap (seconds)
s0 = 1                             # Minimum distance (meters)
ri = 1                             # CACC spacing (meters)
```

## Easy Preset Switching

### Studio Environment (Small, Indoor)
```ini
[simulation]
road_type = "Studio"
[control]
max_velocity = 0.5
lookahead_distance = 0.5
[path]
node_sequence = [10, 4, 20, 13, 10]
[controller_params]
s0 = 1
ri = 1
```

### OpenRoad Environment (Large, Outdoor)
```ini
[simulation]
road_type = "OpenRoad"
[control]
max_velocity = 2.0
lookahead_distance = 7.0
[path]
node_sequence = [0, 1]
[controller_params]
s0 = 7
ri = 7
```

## Common Modifications

### More Vehicles
```ini
[fleet]
num_vehicles = 3                   # Add more cars
```

### Faster Simulation
```ini
[control]
max_velocity = 1.0                 # Increase speed
[controller_params]
v0 = 1.0                           # Match desired speed
```

### Tighter Following
```ini
[controller_params]
s0 = 0.5                           # Smaller minimum distance
ri = 0.5                           # Smaller CACC spacing
T = 0.2                            # Shorter time gap
```

### Longer Simulation
```ini
[simulation]
time = 120                         # Run for 2 minutes
```

## File Format Rules

1. **Sections**: Use `[section_name]` to group related settings
2. **Comments**: Use `#` for comments
3. **Values**: 
   - Numbers: `42` or `3.14`
   - Text: `"Studio"` (with quotes)
   - Lists: `[1, 2, 3, 4]`
   - Boolean: `true` or `false`

## Troubleshooting

### Config File Not Found
- Make sure `config.txt` is in the same folder as `main.py`
- The system will use default values if the file is missing

### Invalid Values
- Check for typos in section names `[simulation]`, `[fleet]`, etc.
- Make sure boolean values are `true` or `false` (lowercase)
- Lists need square brackets: `[1, 2, 3]`

### Simulation Crashes
- Try reducing `max_velocity` if cars are unstable
- Increase `s0` and `ri` if cars are colliding
- Check that `num_vehicles` is between 2 and 5

## Advanced Usage

### Programmatic Updates
You can still update parameters in code:

```python
config.update_config(**{
    'simulation.time': 60,
    'fleet.num_vehicles': 3,
    's0': 2.0,
    'ri': 2.0
})
```

### Save Current Config
```python
config.save_config("my_custom_config.txt")
```

## Benefits of New System

✅ **Easy to modify** - Just edit a text file  
✅ **No Python knowledge needed** - Simple key=value format  
✅ **Quick preset switching** - Uncomment different sections  
✅ **Version control friendly** - Text files work great with git  
✅ **No external dependencies** - Works out of the box  
✅ **Backward compatible** - Old FleetConfig still works as fallback  

## File Structure
```
fleet_framework/
├── main.py                       # Main simulation script
├── config.txt                    # Easy configuration file (EDIT THIS!)
├── simple_config_txt.py          # Configuration loader
├── config.yaml                   # Alternative YAML format (optional)
├── simple_config.py              # YAML configuration loader (optional)
├── FleetConfig.py                # Original complex config (fallback)
└── README_CONFIG.md              # This guide
```

That's it! Just edit `config.txt` and run your simulation. Much easier than navigating complex Python configuration classes!
