# Controller Config Examples

## Example 1: Simple Vehicle Following (Default)
Just follow the leader with CACC controller.

**controller_config.yaml:**
```yaml
follower_mode: "vehicle_following"

vehicle_following:
  alpha: 1.2
  beta: 1.5
  v0: 0.4
  s0: 1
  hi: 0.3
  K_gains: [1.0, 1.0, 0.5, 2.0]
```

**Result:** All vehicles follow the leader

---

## Example 2: All Trajectories (No Communication)
Each vehicle follows its own path, no leader following needed.

**controller_config.yaml:**
```yaml
follower_mode: "trajectory"

trajectory_following:
  K_p: 0.1
  K_stanley: 0.8
  lookahead_distance: 0.5
  enable_steering_control: false
```

**Result:** Each vehicle operates independently

---

## Example 3: Mixed Fleet
- Vehicle 0 (leader): Follows path
- Vehicle 1: Follows leader (vehicle-following)
- Vehicle 2: Follows path independently (trajectory)
- Vehicles 3+: Follows leader (default)

**controller_config.yaml:**
```yaml
follower_mode: "vehicle_following"  # Default for all

vehicle_mode_overrides:
  2: "trajectory"  # Vehicle 2 uses trajectory mode
```

**Result:**
- Vehicle 1 follows Vehicle 0 (vehicle-following)
- Vehicle 2 ignores Vehicle 1, follows its own path (trajectory)
- Vehicle 3+ follows Vehicle 2 (vehicle-following)

---

## Example 4: Smart Hybrid Mode
Switch automatically between trajectory and vehicle-following based on distance and data freshness.

**controller_config.yaml:**
```yaml
follower_mode: "hybrid"

hybrid:
  priority: "vehicle_following"    # Prefer following leader when possible
  distance_threshold: 2.0          # If leader > 2m away, switch to trajectory
  leader_data_timeout: 1.0         # If leader data > 1s old, switch to trajectory
  hysteresis_offset: 0.5           # Switch back at 2.5m (hysteresis)

vehicle_following:
  v0: 0.4
  s0: 1
  hi: 0.3

trajectory_following:
  K_p: 0.1
  K_stanley: 0.8
  lookahead_distance: 0.5
```

**Result:** Robust fleet that continues on trajectory if leader communication fails

---

## Example 5: Tighter Vehicle Following
Lower spacing, shorter time headway, more aggressive acceleration.

**controller_config.yaml:**
```yaml
follower_mode: "vehicle_following"

vehicle_following:
  v0: 0.5              # Higher desired velocity
  s0: 0.5              # Tighter minimum spacing (0.5m instead of 1m)
  T: 0.2               # Shorter time headway (0.2s instead of 0.3s)
  hi: 0.2              # Shorter CACC time headway
  alpha: 1.5           # More aggressive acceleration
  K_gains: [1.5, 1.0, 0.5, 2.5]  # Higher gains for tighter response
```

**Result:** Tighter platoon formation with less spacing

---

## Example 6: Relaxed Vehicle Following
Larger spacing, gentle acceleration - good for smooth, stable following.

**controller_config.yaml:**
```yaml
follower_mode: "vehicle_following"

vehicle_following:
  v0: 0.3              # Lower desired velocity
  s0: 2.0              # Larger minimum spacing (2m)
  T: 0.5               # Longer time headway (0.5s)
  hi: 0.4              # Longer CACC time headway
  alpha: 0.8           # Less aggressive acceleration
  beta: 1.0            # Gentler deceleration
  K_gains: [0.8, 0.8, 0.5, 1.5]  # Lower gains for smoother response
```

**Result:** Smooth, stable fleet with generous spacing

---

## Example 7: Trajectory with Steering Control
Follow path with steering and speed control enabled.

**controller_config.yaml:**
```yaml
follower_mode: "trajectory"

trajectory_following:
  K_p: 0.15            # Stronger speed control
  K_i: 0.1             # More integral action
  K_stanley: 1.2       # Stronger steering control
  enable_steering_control: true
  lookahead_distance: 0.7  # Longer look-ahead for curves
  startDelay: 0.5      # Start quickly
```

**Result:** Smooth trajectory tracking with steering corrections

---

## Example 8: Per-Vehicle Configuration
Different vehicles with different behaviors.

**controller_config.yaml:**
```yaml
follower_mode: "vehicle_following"  # Default

vehicle_mode_overrides:
  1: "trajectory"      # Vehicle 1: Independent trajectory
  2: "hybrid"          # Vehicle 2: Smart switching
  # Vehicle 0, 3, 4: Default vehicle_following

# Adjust parameters for vehicle-following mode
vehicle_following:
  v0: 0.4
  s0: 1.0
  hi: 0.3

# Adjust parameters for trajectory mode (used by Vehicle 1)
trajectory_following:
  K_p: 0.1
  K_stanley: 0.8

# Configure hybrid for Vehicle 2
hybrid:
  priority: "vehicle_following"
  distance_threshold: 1.5
  leader_data_timeout: 0.8
```

**Result:**
- Vehicle 0: Follows Vehicle -1 (leader)
- Vehicle 1: Follows preset path independently
- Vehicle 2: Hybrid mode - switches between following and trajectory
- Vehicle 3: Follows Vehicle 2
- Vehicle 4: Follows Vehicle 3

---

## How to Use These Examples

1. **Copy one of the sections above** into your `controller_config.yaml`
2. **Keep the basic structure** - Don't remove other required fields
3. **Test it** - Run `python main.py` and watch the logs
4. **Adjust as needed** - Change parameters and test again

---

## Common Adjustments

### Speed Control
- `v0` - Desired velocity (increase = faster)
- `alpha` - Acceleration response (increase = more aggressive)
- `beta` - Deceleration response (increase = more aggressive)

### Spacing Control
- `s0` - Minimum spacing when stopped (increase = more space)
- `T` - Time headway (increase = more space at speed)
- `hi` - CACC time headway (increase = more space)

### Steering Control
- `K_stanley` - Steering response (increase = more aggressive steering)
- `lookahead_distance` - How far ahead to look (increase = smoother curves)
- `enable_steering_control` - Enable/disable steering (true/false)

### Hybrid Mode Switching
- `priority` - Which mode to prefer ("vehicle_following" or "trajectory")
- `distance_threshold` - Distance to leader before switching (in meters)
- `leader_data_timeout` - How old data can be before switching (in seconds)

---

## Next Steps

1. **Choose an example** that matches your use case
2. **Edit `controller_config.yaml`** with the example configuration
3. **Run the simulation** - `python main.py`
4. **Check the logs** for mode information
5. **Adjust parameters** based on how it behaves

That's it! Simple and straightforward. 🎉
