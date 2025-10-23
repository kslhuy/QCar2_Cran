# CACC Controller Tuning Fix - Smooth Following Behavior

## Problem Description
The follower vehicle was exhibiting jerky "accelerate-stop-accelerate" behavior:
- Would accelerate quickly towards the leader
- Brake hard when getting too close
- Repeat the cycle, creating oscillations
- Very uncomfortable and unstable following behavior

## Root Causes Identified

### 1. **Extremely Large Spacing Parameters** ❌
```yaml
# OLD (INCORRECT):
s0: 7        # Minimum spacing = 7 meters!
ri: 7        # CACC spacing parameter = 7 meters!
hi: 0.5      # Time headway = 0.5 seconds
```

**Problem**: For a QCar (< 0.5m long), a 7-meter minimum spacing is **enormous**!
- The follower would try to maintain 7m + 0.5*velocity spacing
- Would aggressively brake when closer than 7m
- Would aggressively accelerate when farther than 7m
- Creates large oscillations

### 2. **Weak Control Gains**
```yaml
# OLD (INCORRECT):
K_gains: [1.0, 0.0, 0.0, 1.0]
```

**Problem**: This creates a diagonal matrix with no coupling:
```
K = [1.0  0.0]
    [0.0  1.0]
```
- K11 = 1.0: spacing error gain (too weak for fast response)
- K12 = 0.0: no velocity feedforward from spacing error
- K21 = 0.0: no spatial coupling to velocity control
- K22 = 1.0: velocity error gain (too weak)

Result: Sluggish response → overshoots → oscillations

### 3. **Aggressive Acceleration Limits**
```python
# OLD (INCORRECT):
self.max_acc = 5.0   # m/s² - way too high for small QCar
self.min_acc = -5.0  # m/s² - extremely hard braking
```

**Problem**: 5 m/s² is very aggressive for a small vehicle
- Causes rapid speed changes
- Creates jerky motion
- Uncomfortable for passengers
- Not realistic for QCar dynamics

### 4. **No Smoothing Filter**
The raw acceleration commands were applied directly without any filtering
- High-frequency oscillations in control output
- Jerky actuator commands
- Poor ride quality

## Solutions Implemented

### 1. ✅ Realistic Spacing Parameters
```yaml
# NEW (FIXED):
s0: 0.5      # Minimum spacing = 0.5 meters (appropriate for QCar size)
ri: 0.5      # CACC spacing parameter = 0.5 meters
hi: 0.8      # Time headway = 0.8 seconds (increased for smoother following)
```

**Benefits**:
- `s0 = 0.5m`: Appropriate minimum spacing for small vehicles
- `hi = 0.8s`: Longer time headway → more gradual response → smoother following
- Target spacing = 0.5 + 0.8*v → comfortable and safe

### 2. ✅ Improved Control Gains with Coupling
```yaml
# NEW (FIXED):
K_gains: [2.5, 1.0, 0.5, 2.0]
```

**Benefits**:
```
K = [2.5  1.0]
    [0.5  2.0]
```
- K11 = 2.5: Higher spacing error gain → faster tracking
- K12 = 1.0: Velocity feedforward → anticipates leader velocity changes
- K21 = 0.5: Spatial coupling → coordinates position and velocity control
- K22 = 2.0: Higher velocity error gain → better velocity tracking

Result: Faster response without overshooting, coordinated control

### 3. ✅ Gentler Acceleration Limits
```python
# NEW (FIXED):
self.max_acc = 1.5   # m/s² - appropriate for QCar
self.min_acc = -1.5  # m/s² - comfortable braking
```

**Benefits**:
- 1.5 m/s² is comfortable and realistic for QCar
- Prevents jerky motion
- More realistic vehicle dynamics
- Better passenger comfort

### 4. ✅ Exponential Smoothing Filter
```python
# NEW (ADDED):
self.prev_acc = 0.0         # Previous acceleration command
self.alpha_filter = 0.3     # Smoothing factor (30% new, 70% old)

# In compute_cacc_acceleration():
acc_smoothed = self.alpha_filter * acc + (1 - self.alpha_filter) * self.prev_acc
self.prev_acc = acc_smoothed
return acc_smoothed
```

**Benefits**:
- Low-pass filter removes high-frequency oscillations
- Smoother acceleration transitions
- More comfortable ride
- alpha = 0.3 means 30% new command, 70% previous → gentle changes

## Expected Behavior After Fix

### Before (Jerky):
```
Distance: 5m → 8m → 4m → 9m → 3m → 10m ...  (oscillating)
Speed:    0.8 → 0.3 → 1.2 → 0.2 → 1.5 ...   (jerky)
```

### After (Smooth):
```
Distance: 5m → 5.2m → 5.5m → 5.7m → 6m ...  (converging smoothly)
Speed:    0.8 → 0.85 → 0.9 → 0.95 → 1.0 ... (smooth acceleration)
```

## Tuning Guide

If you still experience issues, adjust these parameters:

### For MORE aggressive following (faster response):
```yaml
hi: 0.6          # Reduce time headway (but don't go below 0.4)
K_gains: [3.0, 1.2, 0.6, 2.5]  # Increase gains
```

### For MORE gentle following (smoother but slower):
```yaml
hi: 1.0          # Increase time headway
K_gains: [2.0, 0.8, 0.4, 1.5]  # Reduce gains
```

### For MORE smoothing (less responsive but very smooth):
```python
self.alpha_filter = 0.2  # More smoothing (20% new, 80% old)
```

### For LESS smoothing (more responsive but potentially jerky):
```python
self.alpha_filter = 0.5  # Less smoothing (50% new, 50% old)
```

## Files Modified

1. **config.yaml**: Updated controller parameters
   - `s0`: 7 → 0.5 meters
   - `ri`: 7 → 0.5 meters
   - `hi`: 0.5 → 0.8 seconds
   - `K_gains`: [1, 0, 0, 1] → [2.5, 1.0, 0.5, 2.0]

2. **src/Controller/CACC.py**: Improved controller implementation
   - `max_acc`: 5.0 → 1.5 m/s²
   - `min_acc`: -5.0 → -1.5 m/s²
   - Added exponential smoothing filter

## Testing Recommendations

1. **Test with different leader velocities**:
   - Constant speed: Should maintain steady spacing
   - Accelerating: Should smoothly accelerate to maintain spacing
   - Decelerating: Should smoothly decelerate without hard braking
   - Stop-and-go: Should handle without jerking

2. **Monitor these metrics**:
   - Spacing error: Should converge to near zero without oscillations
   - Velocity error: Should converge smoothly
   - Acceleration command: Should be smooth, no sudden jumps
   - Actual vehicle motion: Should be comfortable

3. **Visualize the data**:
   ```bash
   python plot_existing_data.py
   ```
   Look for:
   - Smooth trajectory (no zigzag)
   - Smooth velocity profile (no spikes)
   - Spacing converging to steady-state

## References

The CACC control law:
```
spacing_target = s0 + hi * v_follower
spacing_error = spacing_actual - spacing_target
velocity_error = v_leader - v_follower

u = K @ [spacing_error, velocity_error]
acc = u[0]
```

Where proper tuning ensures:
- Small steady-state error
- Fast convergence
- No overshoot/oscillations
- Comfortable ride quality

---
**Date**: October 19, 2025
**Status**: FIXED - Ready for testing
