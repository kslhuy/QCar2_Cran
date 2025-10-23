# Acceleration Logging and GPS Plotting Updates

## Summary
This document describes the updates made to add acceleration logging and improve GPS plotting in the fleet framework.

## Changes Made

### 1. Added Acceleration Logging to DataLogger

**File: `DataLogger.py`**

#### Changes:
- **CSV Header**: Added `actual_acceleration` column to the local state CSV file
- **Method Signature**: Updated `log_local_state()` to accept `actual_acceleration` parameter (default: 0.0)
- **Data Storage**: Now stores actual acceleration in both memory and CSV file

#### Modified Functions:
```python
def log_local_state(self, timestamp: float, local_state: np.ndarray, 
                   measured_state: Optional[np.ndarray], control_input: np.ndarray,
                   gps_available: bool, dt: float, actual_acceleration: float = 0.0):
```

The CSV now includes:
- `actual_acceleration`: The measured/actual acceleration value
- `control_acceleration`: The commanded acceleration (already existed in control_input)

### 2. Enhanced State Comparison Plot with Acceleration

**File: `DataLogger.py` - `plot_state_comparison()` method**

#### Changes:
- **Dynamic Layout**: Plot layout now adjusts based on available data
  - 2x2 grid if no acceleration data
  - 3x2 grid if acceleration data is available
- **New Plots Added**:
  - **Acceleration Plot** (axes[2,0]): Shows actual acceleration vs control acceleration
  - **Steering Control Plot** (axes[2,1]): Shows steering angle over time

#### Features:
- Green line: Actual measured acceleration
- Orange dashed line: Control (commanded) acceleration
- Purple line: Steering control
- All plots have proper labels, legends, and grid

### 3. Updated VehicleObserver to Pass Acceleration

**File: `VehicleObserver.py` - `update_local_state()` method**

#### Changes:
- Extracts actual acceleration from the `acceleration` parameter
- Passes it to `data_logger.log_local_state()` with proper error handling
- Handles cases where acceleration parameter might be None or empty

```python
# Extract actual acceleration from the acceleration parameter if provided
actual_accel = acceleration[1] if acceleration is not None and len(acceleration) > 1 else 0.0

self.data_logger.log_local_state(
    timestamp=timestamp,
    local_state=self.local_state,
    measured_state=measured_state,
    control_input=control_input,
    gps_available=self.gps_available,
    dt=dt,
    actual_acceleration=actual_accel
)
```

### 4. GPS Plotting - Fixed to Use gps_available Flag

**Status: ✅ Fixed!**

**Problem**: When GPS is not available, the CSV still contains the last GPS reading (not NaN), causing duplicate GPS points to be plotted even when GPS is unavailable.

**Solution**: Changed plotting to filter based on the `gps_available` column instead of just checking for NaN values.

**Changes Made**:
- `plot_vehicle_trajectory()`: Now uses `data[data['gps_available'] == True]` to filter GPS points
- `plot_state_comparison()`: All GPS subplots now filter by `gps_available` flag

**Before**:
```python
# Old code - would plot duplicate last GPS reading
gps_data = data.dropna(subset=['gps_x', 'gps_y'])
```

**After**:
```python
# New code - only plots when GPS is actually available
gps_data = data[data['gps_available'] == True].copy()
```

This ensures GPS measurements are only shown when they are fresh and valid, not stale values from when GPS was last available.

## Usage

### Running the Observer
The observer will now automatically log acceleration data:

```python
observer.update_local_state(
    measured_state=gps_state,      # [x, y, theta, v] or None
    control_input=control,          # [steering, acceleration]
    timestamp=current_time,
    motor_tach=tach_reading,
    gyroscope_z=gyro_reading,
    acceleration=accel_data         # Should contain actual acceleration
)
```

### Plotting the Data
When you plot existing data, acceleration plots will automatically appear if the data is available:

```python
# From plot_existing_data.py
visualizer = FleetDataVisualizer(data_dir)
visualizer.plot_state_comparison(vehicle_id=0)  # Will show acceleration if available
```

## CSV Format

The local state CSV now has this structure:
```csv
timestamp,vehicle_id,x,y,theta,velocity,gps_available,dt,gps_x,gps_y,gps_theta,gps_velocity,control_steering,control_acceleration,actual_acceleration
```

## Benefits

1. **Better Analysis**: Can now compare commanded vs actual acceleration
2. **Debug Support**: Helps identify control delays or actuator limitations
3. **Performance Metrics**: Enables acceleration-based performance analysis
4. **Clean GPS Plotting**: GPS only shown when valid measurements exist (no duplicates)
5. **Backward Compatible**: Old data without acceleration will still plot correctly (just without acceleration subplot)

## Testing

To verify the changes work correctly:

1. **Run a simulation** with acceleration data
2. **Check CSV file** - should have `actual_acceleration` column
3. **Plot the data** using `plot_existing_data.py`
4. **Verify plots**:
   - State comparison should show 3x2 grid with acceleration plots
   - GPS points should only appear when available (no NaN plotting)
   - Acceleration vs control acceleration comparison visible

## Notes

- Default acceleration value is 0.0 if not provided (backward compatibility)
- Acceleration plots only appear when data is available
- GPS filtering was already correct, just documented here
- All changes maintain backward compatibility with existing code
