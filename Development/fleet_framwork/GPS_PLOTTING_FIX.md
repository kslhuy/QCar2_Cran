# GPS Plotting Fix - Removing Duplicate GPS Points

## Problem Description

When GPS is not available, the system keeps the last GPS reading in the CSV file (not NaN). This caused the plotting functions to show **duplicate GPS points** at the last known GPS location, even when GPS was unavailable.

### Example Scenario:
1. GPS is available at t=0.0s → GPS point plotted at (x=1.0, y=2.0) ✅
2. GPS is available at t=0.1s → GPS point plotted at (x=1.1, y=2.1) ✅
3. GPS becomes unavailable at t=0.2s → CSV still shows (x=1.1, y=2.1) but `gps_available=False`
4. **Old behavior**: GPS point still plotted at (x=1.1, y=2.1) ❌ **DUPLICATE!**
5. **New behavior**: No GPS point plotted ✅ **CORRECT!**

## Root Cause

The old filtering logic used:
```python
gps_data = data.dropna(subset=['gps_x', 'gps_y'])
```

This only filters out NaN values, but when GPS is unavailable, the system retains the last GPS reading (not NaN), so it gets plotted repeatedly.

## Solution

Changed all GPS plotting to filter based on the `gps_available` boolean flag:
```python
# New filtering logic
gps_data = data[data['gps_available'] == True].copy()
```

This ensures GPS points are **only plotted when they are fresh and valid**, not when they are stale values from the last time GPS was available.

## Files Modified

### 1. DataLogger.py - plot_vehicle_trajectory()
**Line ~425**
```python
# OLD CODE
if show_gps and 'gps_x' in data.columns:
    gps_data = data.dropna(subset=['gps_x', 'gps_y'])
    if not gps_data.empty:
        plt.scatter(gps_data['gps_x'], gps_data['gps_y'], ...)

# NEW CODE
if show_gps and 'gps_x' in data.columns and 'gps_available' in data.columns:
    gps_data = data[data['gps_available'] == True].copy()
    if not gps_data.empty:
        plt.scatter(gps_data['gps_x'], gps_data['gps_y'], ...)
```

### 2. DataLogger.py - plot_state_comparison()
**Lines ~629, ~641, ~654, ~667**

Updated all four state comparison subplots (X, Y, Theta, Velocity):
```python
# OLD CODE
if 'gps_x' in data.columns:
    gps_data = data.dropna(subset=['gps_x'])
    ...

# NEW CODE  
if 'gps_x' in data.columns and 'gps_available' in data.columns:
    gps_data = data[data['gps_available'] == True].copy()
    ...
```

## Benefits

1. **No Duplicate GPS Points**: GPS markers only appear when measurements are fresh
2. **Accurate Visualization**: Clear distinction between estimated trajectory and actual GPS measurements
3. **Better Analysis**: Easy to see when GPS was available vs when observer used prediction
4. **Correct Statistics**: GPS error analysis won't be skewed by duplicate points

## Testing

To verify the fix works:

1. **Run simulation** with intermittent GPS (e.g., GPS dropout periods)
2. **Check CSV file**:
   - Look for periods where `gps_available=False`
   - Verify GPS values remain constant during these periods
3. **Plot the data**:
   - GPS scatter points should have **gaps** during unavailable periods
   - No clustering of GPS points at the same location
4. **Compare**:
   - Estimated trajectory (blue line) should be continuous
   - GPS measurements (red points) should be sparse and show gaps

## Visual Example

**Before Fix** (Wrong):
```
Time:  0.0s  0.1s  0.2s  0.3s  0.4s  0.5s
GPS:   •     •     •     •     •     •      ← All plotted (duplicates!)
Avail: True  True  False False False False
```

**After Fix** (Correct):
```
Time:  0.0s  0.1s  0.2s  0.3s  0.4s  0.5s
GPS:   •     •                              ← Only when available!
Avail: True  True  False False False False
```

## Data Structure Reference

The CSV file has these columns:
- `gps_available`: Boolean flag (True/False)
- `gps_x`, `gps_y`, `gps_theta`, `gps_velocity`: GPS measurements (may be stale when unavailable)

The observer logic in VehicleObserver.py:
```python
# When GPS is unavailable, last GPS values are kept in CSV
gps_values = measured_state if measured_state is not None else [np.nan, np.nan, np.nan, np.nan]
```

But the plotting now correctly uses:
```python
# Only plot when gps_available is True
gps_data = data[data['gps_available'] == True].copy()
```

## Related Changes

This fix is part of a larger update that also includes:
- Acceleration logging and plotting
- Enhanced state comparison plots
- See `ACCELERATION_LOGGING_UPDATE.md` for full details
