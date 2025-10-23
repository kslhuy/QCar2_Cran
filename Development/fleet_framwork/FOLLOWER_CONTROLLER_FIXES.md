# Follower Controller Fixes

## Summary
Fixed multiple issues in the `VehicleFollowerController.py` that were preventing the follower vehicle from properly tracking the leader vehicle.

## Problems Identified

### 1. **Lookahead Controller Issues**
   - **Problem**: Incorrect curvature calculation using heading instead of yaw rate
   - **Problem**: Sign errors in extension vector calculations
   - **Problem**: Noisy yaw rate calculations without filtering
   - **Fix**: 
     - Added low-pass filter for yaw rate
     - Corrected extension vector to use `(cos, sin)` instead of `(sin, -cos)`
     - Fixed error calculations to properly compute position and velocity errors
     - Improved steering angle conversion using bicycle model

### 2. **Pure Pursuit Lateral Control Issues**
   - **Problem**: Fixed lookahead distance regardless of vehicle distance
   - **Problem**: Incorrect sign in steering command (negative gain)
   - **Problem**: No consideration for backward motion
   - **Fix**:
     - Implemented adaptive lookahead based on distance to leader
     - Corrected steering command sign (removed negative)
     - Added motion direction detection based on leader velocity
     - Prevents overshooting when close to leader

### 3. **CACC Spacing Calculation**
   - **Problem**: Used leader's heading for spacing projection instead of actual distance
   - **Fix**: 
     - Changed to use Euclidean distance for more accurate spacing
     - Improved comments explaining spacing calculation
     - Added alternative projection method (commented out)

### 4. **Configuration Parameters**
   - **Problem**: Spacing parameters too large (7m) for close vehicle following
   - **Problem**: Control gains too conservative
   - **Problem**: Lookahead distance too large (7m)
   - **Problem**: Steering control was disabled
   - **Fix**:
     - Reduced `s0` and `ri` from 7m to 0.5m for closer following
     - Increased `hi` from 0.5s to 0.6s for smoother response
     - Updated K_gains to `[0.8, 0.5, 0.0, 1.2]` for better tracking
     - Reduced lookahead distance to 0.4m
     - Added `k_steering: 2.5` for responsive steering
     - **Enabled steering control** (was set to false)

## Changes Made

### VehicleFollowerController.py

1. **Added state variable for filtering**:
   ```python
   self.prev_yaw_rate_lead = 0.0  # previous yaw rate for filtering
   ```

2. **Improved `_compute_lookahead_control()`**:
   - Added yaw rate filtering with alpha = 0.5
   - Fixed extension vector calculation
   - Corrected position error calculations
   - Improved steering angle conversion using wheelbase

3. **Improved `_compute_lateral_control()`**:
   - Added adaptive lookahead distance
   - Fixed steering command sign
   - Added motion direction detection
   - Prevents overshooting when close to leader

### src/Controller/CACC.py

1. **Improved `compute_cacc_acceleration()`**:
   - Changed spacing calculation to Euclidean distance
   - Added clear comments explaining the algorithm
   - Improved error calculation logic

### config.yaml

1. **Updated controller parameters**:
   - `s0: 0.5` (was 7) - minimum spacing
   - `ri: 0.5` (was 7) - CACC spacing parameter
   - `hi: 0.6` (was 0.5) - time headway
   - `K_gains: [0.8, 0.5, 0.0, 1.2]` (was [1.0, 0.0, 0.0, 1.0])

2. **Updated control parameters**:
   - `lookahead_distance: 0.4` (was 7.0)
   - `k_steering: 2.5` (new parameter)
   - `enable_steering_control: true` (was false)

## Testing Recommendations

1. **Test with 2 vehicles first**:
   - Set `num_vehicles: 2` in config
   - Verify follower tracks leader smoothly

2. **Gradually increase complexity**:
   - Test with straight paths first (OpenRoad)
   - Then test with curved paths (Studio)
   - Finally test with multiple vehicles

3. **Monitor key metrics**:
   - Spacing error (should converge to near zero)
   - Velocity tracking (follower should match leader speed)
   - Steering smoothness (no oscillations)

4. **Tune if needed**:
   - If too aggressive: reduce `K_gains` values
   - If too slow to respond: increase `K_gains` values
   - If oscillating: reduce `k_steering` gain
   - If not following closely: reduce `s0` and `ri`

## Configuration Presets

The config now supports two presets:
- `active_preset: "openroad_cacc"` - For outdoor, straight roads
- `active_preset: "studio_cacc"` - For indoor, curved paths

Make sure to set the appropriate preset for your testing environment.

## Known Limitations

1. The LOOKAHEAD controller is still experimental and may need tuning
2. Backward motion handling could be improved
3. Multi-vehicle platoon behavior needs additional testing

## Next Steps

1. Test the controller with the current configuration
2. Monitor logs for any errors or warnings
3. Adjust gains if the response is too aggressive or too conservative
4. Consider adding velocity-dependent gain scheduling for better performance
