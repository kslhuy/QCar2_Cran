# Vehicle Type Feature Documentation

## Overview
This document describes the new `vehicle_type` field added to the fleet configuration system, which allows specifying different vehicle types in the multi-vehicle control system.

## Supported Vehicle Types
- **Qcar**: Standard QCar vehicle (default)
- **Limo**: Limo vehicle platform

## Modified Files

### 1. `fleet_config.yaml`
**Changes:**
- Added `vehicle_type` field to each vehicle configuration
- Supported values: `"Qcar"` or `"Limo"`
- Defaults to `"Qcar"` if not specified

**Example:**
```yaml
vehicles:
  - car_id: 0
    ip: 192.168.2.137
    vehicle_type: "Qcar"   # NEW FIELD
    enabled: true
    probing: true
    calibrate: false
    path_number: 0
    left_hand_traffic: false
    initial_v_ref: 0.75
    description: "Leader vehicle with probing"
    
  - car_id: 1
    ip: 192.168.2.108
    vehicle_type: "Limo"   # NEW FIELD - Can be different type
    enabled: true
    probing: true
    calibrate: false
    path_number: 1
    left_hand_traffic: false
    initial_v_ref: 0.75
    description: "Follower vehicle"
```

### 2. `python/start_enhanced.py`
**Changes:**
- Updated `display_configuration()` to show vehicle type for each vehicle
- Updated `start_vehicle()` to:
  - Extract vehicle type from configuration
  - Display vehicle type in startup messages
  - Pass vehicle type as command-line argument to `vehicle_main.py`
- Updated `display_summary()` to show vehicle type in final summary

**New Command-Line Argument Passed:**
```bash
--vehicle-type Qcar  # or Limo
```

### 3. `qcar/vehicle_main.py`
**Changes:**
- Added new command-line argument `--vehicle-type` with choices `['Qcar', 'Limo']`
- Default value: `'Qcar'`
- Updated configuration display to show vehicle type
- Vehicle type is now stored in the configuration and can be accessed by vehicle control logic

**New Argument:**
```python
parser.add_argument(
    '--vehicle-type',
    type=str,
    default='Qcar',
    choices=['Qcar', 'Limo'],
    help='Vehicle type: Qcar or Limo (default: Qcar)'
)
```

### 4. `qcar/config_main.py`
**Changes:**
- Added new `VehicleConfig` dataclass to store vehicle-specific properties
- Added `vehicle_type` field with validation
- Updated `VehicleMainConfig` to include `VehicleConfig`
- Updated serialization/deserialization methods (`from_dict`, `to_dict`)
- Updated `update_from_args()` to handle vehicle type from command-line arguments

**New Classes:**
```python
@dataclass
class VehicleConfig:
    """Vehicle-specific configuration"""
    vehicle_type: str = "Qcar"  # "Qcar" or "Limo"
    
    def __post_init__(self):
        """Validate vehicle type"""
        valid_types = ["Qcar", "Limo"]
        if self.vehicle_type not in valid_types:
            raise ValueError(f"Invalid vehicle_type: {self.vehicle_type}. Must be one of {valid_types}")
```

## Usage

### Starting the Fleet
When you run `start_enhanced.bat` or `python start_enhanced.py`, the system will:

1. Load `fleet_config.yaml`
2. Display vehicle type for each vehicle in the configuration summary
3. Start each vehicle with its specified type
4. Pass the vehicle type to each vehicle's control system

### Display Example
```
Vehicle Fleet (2 vehicles):
----------------------------------------------------------------------
  • Car 0: 192.168.2.137 [Qcar]
    Description: Leader vehicle with probing
    Vehicle Type: Qcar
    Path: 0 - Main Circuit
    Nodes: [10, 2, 4, 6, 8, 10]
    Probing: Yes
    Calibration: No
    Initial Speed: 0.75 m/s
    Port: 5000

  • Car 1: 192.168.2.108 [Limo]
    Description: Follower vehicle
    Vehicle Type: Limo
    Path: 1 - Extended Loop
    Nodes: [0, 2, 4, 6, 8, 10, 2, 4, 6, 0]
    Probing: Yes
    Calibration: No
    Initial Speed: 0.75 m/s
    Port: 5001
```

### Accessing Vehicle Type in Code
In your vehicle control logic, you can access the vehicle type through the configuration:

```python
# In vehicle_logic.py or other modules
vehicle_type = config.vehicle.vehicle_type

if vehicle_type == "Qcar":
    # QCar-specific behavior
    pass
elif vehicle_type == "Limo":
    # Limo-specific behavior
    pass
```

## Migration Guide

### Existing Configurations
All existing configurations will default to `"Qcar"` if the `vehicle_type` field is not specified. No breaking changes for existing setups.

### Adding Vehicle Type to Existing Vehicles
Simply add the `vehicle_type` field to each vehicle in your `fleet_config.yaml`:

```yaml
vehicles:
  - car_id: 0
    ip: 192.168.2.137
    vehicle_type: "Qcar"  # Add this line
    # ... rest of configuration
```

## Future Enhancements

### Potential Extensions
1. **Vehicle-Specific Parameters**: Add type-specific configurations (e.g., different control gains for Qcar vs Limo)
2. **Dynamic Behavior**: Implement vehicle-type-specific control strategies
3. **Hardware Abstraction**: Create abstraction layers for different vehicle hardware
4. **Additional Types**: Support for more vehicle types as needed

### Example Future Configuration
```yaml
vehicle_parameters:
  Qcar:
    max_speed: 2.0
    wheelbase: 0.33
    turning_radius: 0.4
  Limo:
    max_speed: 1.5
    wheelbase: 0.45
    turning_radius: 0.5
```

## Validation

The system includes validation to ensure only valid vehicle types are used:
- Valid types: `"Qcar"`, `"Limo"`
- Invalid types will raise a `ValueError` with a descriptive message
- Validation occurs both at configuration load and during runtime

## Testing

To test the new feature:

1. **Update Configuration:**
   ```yaml
   vehicles:
     - car_id: 0
       vehicle_type: "Qcar"
       # ... other settings
     - car_id: 1
       vehicle_type: "Limo"
       # ... other settings
   ```

2. **Run Startup Script:**
   ```bash
   python python/start_enhanced.py --config fleet_config.yaml
   ```

3. **Verify Output:**
   - Check that vehicle types are displayed correctly in the configuration summary
   - Verify that each vehicle shows its type in the startup messages
   - Confirm that the vehicle type appears in the final summary

## Notes

- The vehicle type is purely informational at this stage
- Future implementations can use this field to customize behavior per vehicle type
- The default value ensures backward compatibility with existing configurations
