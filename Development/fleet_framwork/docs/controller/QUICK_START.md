# Quick Start Guide - Controller Configuration

## TL;DR (Too Long; Didn't Read)

1. **Change follower mode** → Edit `controller_config.yaml`, set `follower_mode`
2. **Fine-tune parameters** → Edit the appropriate section in `controller_config.yaml`
3. **Per-vehicle config** → Add to `vehicle_mode_overrides` dict in `controller_config.yaml`
4. **Run** → `python main.py`

That's it! No other files to modify. ✨

---

## Step-by-Step Examples

### Example 1: Switch All Vehicles to Follow a Path
```yaml
# controller_config.yaml
follower_mode: "trajectory"
```
Save and run: `python main.py`

Result: All vehicles follow predefined paths independently.

---

### Example 2: Make Vehicle 1 Use Different Mode
```yaml
# controller_config.yaml
follower_mode: "vehicle_following"  # Default

vehicle_mode_overrides:
  1: "trajectory"  # Vehicle 1 uses trajectory instead
```
Save and run: `python main.py`

Result: Vehicle 1 follows a path, others follow Vehicle 0 (leader).

---

### Example 3: Use Smart Hybrid Mode
```yaml
# controller_config.yaml
follower_mode: "hybrid"

hybrid:
  priority: "vehicle_following"
  distance_threshold: 2.0
  leader_data_timeout: 1.0
```
Save and run: `python main.py`

Result: Vehicles automatically switch between following leader and path.

---

### Example 4: Make Vehicles Follow Tighter
```yaml
# controller_config.yaml
follower_mode: "vehicle_following"

vehicle_following:
  s0: 0.5        # Tighter minimum spacing (was 1.0)
  hi: 0.2        # Shorter time headway (was 0.3)
  v0: 0.5        # Faster desired velocity (was 0.4)
  K_gains: [1.5, 1.0, 0.5, 2.5]  # More aggressive gains
```
Save and run: `python main.py`

Result: Tighter fleet formation with more aggressive control.

---

### Example 5: Make Vehicles Follow More Gently
```yaml
# controller_config.yaml
follower_mode: "vehicle_following"

vehicle_following:
  s0: 2.0        # Larger minimum spacing (was 1.0)
  hi: 0.4        # Longer time headway (was 0.3)
  v0: 0.3        # Slower desired velocity (was 0.4)
  alpha: 0.8     # Less aggressive acceleration (was 1.2)
  K_gains: [0.8, 0.8, 0.5, 1.5]  # Smoother gains
```
Save and run: `python main.py`

Result: Smooth, gentle fleet formation with larger spacing.

---

## Configuration File Structure

```yaml
# MODE SELECTION
follower_mode: "vehicle_following"

# PER-VEHICLE OVERRIDES (optional)
vehicle_mode_overrides:
  # vehicle_id: "mode_name"
  # 1: "trajectory"

# VEHICLE-FOLLOWING CONFIG (for vehicle_following and hybrid modes)
vehicle_following:
  # Parameters for following leader
  alpha: 1.2
  beta: 1.5
  v0: 0.4       # Desired velocity
  s0: 1         # Minimum spacing
  hi: 0.3       # Time headway
  # ... more parameters

# TRAJECTORY CONFIG (for trajectory and hybrid modes)
trajectory_following:
  # Parameters for following path
  K_p: 0.1      # Speed controller
  K_stanley: 0.8  # Steering controller
  lookahead_distance: 0.5
  # ... more parameters

# HYBRID CONFIG (only for hybrid mode)
hybrid:
  # Smart switching settings
  priority: "vehicle_following"
  distance_threshold: 2.0
  leader_data_timeout: 1.0
  hysteresis_offset: 0.5
```

---

## Common Tasks

### Change All Vehicles' Desired Speed
```yaml
vehicle_following:
  v0: 0.5  # Change this (was 0.4)
```

### Increase Minimum Spacing
```yaml
vehicle_following:
  s0: 1.5  # Change this (was 1.0)
```

### Enable Steering Control
```yaml
trajectory_following:
  enable_steering_control: true  # Change this (was false)
```

### Change Hybrid Mode Priority
```yaml
hybrid:
  priority: "trajectory"  # Change to prefer trajectory (was "vehicle_following")
```

### Make Vehicle 2 Follow a Path
```yaml
vehicle_mode_overrides:
  2: "trajectory"  # Add this line
```

### Make Vehicle 3 Use Hybrid
```yaml
vehicle_mode_overrides:
  3: "hybrid"  # Add this line
```

---

## Testing Your Changes

1. **Edit `controller_config.yaml`**
2. **Run the simulation:**
   ```bash
   python main.py
   ```
3. **Check the logs** for messages like:
   ```
   Vehicle 0: Using follower mode 'vehicle_following'
   Vehicle 0: Loaded vehicle-following config
   Vehicle 1: Using mode override 'trajectory'
   Vehicle 1: Loaded trajectory-following config
   ```
4. **Observe the vehicle behavior**
5. **Adjust and repeat**

---

## Modes Explained

### `vehicle_following`
Vehicles follow the leader using CACC/IDM control.
- **When to use:** Formation control, platoons, connected vehicles
- **Requires:** Leader data communication
- **Parameters:** `alpha`, `beta`, `v0`, `s0`, `hi`, etc.

### `trajectory`
Vehicles follow predefined paths independently.
- **When to use:** Multi-agent systems, independent navigation
- **Requires:** Path information (from main config.yaml)
- **Parameters:** `K_p`, `K_stanley`, `lookahead_distance`, etc.

### `hybrid`
Vehicles smartly switch between following leader and following path.
- **When to use:** Robust systems, handling communication failures
- **Fallback:** If leader unavailable or too far, switch to trajectory
- **Parameters:** `priority`, `distance_threshold`, `leader_data_timeout`, etc.

---

## Parameter Guide

| Parameter | Effect | Range | Default |
|-----------|--------|-------|---------|
| `v0` | Desired velocity | 0.1 - 1.0 | 0.4 |
| `s0` | Minimum spacing | 0.5 - 3.0 | 1.0 |
| `hi` | Time headway | 0.1 - 0.5 | 0.3 |
| `alpha` | Acceleration gain | 0.5 - 2.0 | 1.2 |
| `beta` | Deceleration gain | 0.5 - 2.0 | 1.5 |
| `K_stanley` | Steering gain | 0.1 - 2.0 | 0.8 |
| `lookahead_distance` | Path look-ahead | 0.2 - 1.0 | 0.5 |

---

## Troubleshooting

**Problem:** Vehicles crashing
- **Solution 1:** Increase `s0` (minimum spacing)
- **Solution 2:** Decrease `alpha` (less aggressive)
- **Solution 3:** Increase `hi` (longer time headway)

**Problem:** Vehicles moving too slowly
- **Solution:** Increase `v0` (desired velocity)

**Problem:** Vehicles oscillating/jerky movement
- **Solution 1:** Decrease `K_gains` values
- **Solution 2:** Increase `hi` (longer time headway)
- **Solution 3:** Decrease `alpha` (less aggressive)

**Problem:** Vehicles not responding to path
- **Solution 1:** Increase `K_stanley` (steering gain)
- **Solution 2:** Enable `enable_steering_control: true`
- **Solution 3:** Check path configuration in main config.yaml

**Problem:** Vehicles not following leader
- **Solution 1:** Check `follower_mode: "vehicle_following"`
- **Solution 2:** Check communication is enabled in main config.yaml
- **Solution 3:** Increase `K_gains` values

---

## Next Steps

1. ✅ Open `controller_config.yaml`
2. ✅ Choose an example from above
3. ✅ Copy the configuration
4. ✅ Save the file
5. ✅ Run: `python main.py`
6. ✅ Watch it work! 🎉

---

## Files You Don't Need to Edit Anymore

- ❌ `QcarFleet.py` - Configuration passing removed
- ❌ `VehicleProcess.py` - No follower_mode passing needed
- ❌ `config.yaml` - Controller parameters moved to `controller_config.yaml`

**Just edit `controller_config.yaml` and you're done!** ✨
