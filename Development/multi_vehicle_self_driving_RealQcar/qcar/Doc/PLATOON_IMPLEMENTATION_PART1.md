# Platoon Mode Implementation - Part 1: State Machine & Spacing Controller

## Overview
This is the foundation for multi-vehicle platoon control. We've implemented:
1. **Extended State Machine** with platoon states
2. **Platoon Controller** with spacing control logic

## New State Machine States

### Platoon States Added:
```python
PLATOON_LEADER_FORMING      # Leader moving slowly, waiting for followers
PLATOON_FOLLOWER_SEARCHING  # Follower looking for leader via YOLO
PLATOON_FOLLOWER_FORMING    # Follower adjusting spacing to leader
PLATOON_ACTIVE              # Platoon formed, coordinated motion
PLATOON_LOST                # Lost connection to platoon
```

### State Transitions:
```
Normal Operation:
FOLLOWING_PATH → PLATOON_LEADER_FORMING (when designated as leader)
FOLLOWING_PATH → PLATOON_FOLLOWER_SEARCHING (when designated as follower)

Leader Flow:
PLATOON_LEADER_FORMING → PLATOON_ACTIVE (when all followers ready)

Follower Flow:
PLATOON_FOLLOWER_SEARCHING → PLATOON_FOLLOWER_FORMING (leader detected)
PLATOON_FOLLOWER_FORMING → PLATOON_ACTIVE (spacing stable)

Error Handling:
PLATOON_FOLLOWER_* → PLATOON_LOST (leader lost)
PLATOON_LOST → PLATOON_FOLLOWER_SEARCHING (retry)
Any Platoon State → FOLLOWING_PATH (disable platoon mode)
```

## Platoon Controller Features

### 1. **Configuration (PlatoonConfig)**
```python
formation_speed: 0.3 m/s        # Slow speed during formation
active_speed: 0.75 m/s          # Normal platoon speed
target_spacing: 1.5 m           # Desired inter-vehicle distance
spacing_tolerance: 0.3 m        # Acceptable error
min_safe_spacing: 0.8 m         # Emergency threshold
max_spacing: 3.0 m              # Lost leader threshold
```

### 2. **Leader Tracking (for Followers)**
- Uses **YOLO car detection** to detect leader ahead
- Estimates distance from YOLO `carDist`
- Receives leader velocity via network telemetry
- Tracks last time leader was seen (timeout detection)

### 3. **Spacing Controller (PI Controller)**
The follower spacing control uses a **PI (Proportional-Integral) controller**:

```python
spacing_error = leader_distance - target_spacing

# PI control
velocity_adjustment = Kp * spacing_error + Ki * integral(spacing_error)

# Apply to velocity
target_velocity = leader_velocity + velocity_adjustment
```

**Control Gains:**
- `Kp = 0.3`: Quick response to spacing errors
- `Ki = 0.05`: Eliminates steady-state error
- `Filter = 0.2`: Smooths velocity changes (low-pass filter)

**Safety Features:**
- Anti-windup: Limits integral term to ±0.5 m/s
- Emergency slowdown: If distance < 0.8m, reduce to 50% velocity
- Velocity limits: Clipped to [0, 1.5 × base_velocity]

### 4. **Formation Detection**

**For Followers (Spacing Stability):**
```python
is_spacing_stable():
    - Check: |distance - target| < tolerance
    - Must be stable for 2 seconds continuously
    - Returns True → Follower broadcasts "READY"
```

**For Leader (All Followers Ready):**
```python
are_all_followers_ready():
    - Tracks which followers sent "READY" signal
    - Returns True when all expected followers ready
    - Triggers transition to PLATOON_ACTIVE
```

### 5. **Timeout & Error Handling**

```python
has_lost_leader():
    - No YOLO detection for > 10 seconds
    - Returns True → Transition to PLATOON_LOST

has_formation_timeout():
    - Formation taking > 20 seconds
    - Returns True → Give up, return to normal mode
```

## How Spacing Control Works

### Physics-Based Approach:
```
Follower velocity adjustment maintains constant spacing

If too close (error < 0):
    → Reduce velocity (slow down)
    
If too far (error > 0):
    → Increase velocity (speed up)
    
If just right (error ≈ 0):
    → Match leader velocity
```

### Example Scenario:
```
Initial state:
- Leader: 1.5m ahead, moving at 0.3 m/s
- Follower: Moving at 0.3 m/s
- Distance: 2.0m (too far!)

Controller response:
- Error = 2.0 - 1.5 = 0.5m (positive)
- Adjustment = 0.3 × 0.5 = 0.15 m/s (speed up)
- Target velocity = 0.3 + 0.15 = 0.45 m/s

After 3 seconds:
- Follower catches up, distance → 1.5m
- Error = 0m
- Adjustment = 0m
- Both vehicles at 0.3 m/s → STABLE!
```

## Integration Points

### In vehicle_controller.py (next step):
```python
# 1. Initialize platoon controller
self.platoon_controller = PlatoonController(config, logger)

# 2. In control iteration:
if self.state_machine.is_in_platoon():
    # Update leader detection from YOLO
    self.platoon_controller.update_leader_info(
        detected=self.yolo.cars,
        distance=self.yolo_drive.carDist,
        velocity=leader_velocity_from_network
    )
    
    # Compute platoon velocity
    if self.state_machine.is_platoon_follower():
        v_ref = self.platoon_controller.compute_follower_velocity(
            current_velocity=velocity,
            base_velocity=active_speed
        )
```

### In network telemetry (next step):
```python
telemetry = {
    # ... existing fields ...
    **self.platoon_controller.get_telemetry()
}
```

## Advantages of This Design

### ✅ **No GPS Required Between Vehicles**
- Uses YOLO for relative detection
- Network only for velocity coordination
- Same path ensures follower is always behind leader

### ✅ **Robust to Sensor Noise**
- PI controller smooths out YOLO distance noise
- Low-pass filter prevents jerky motion
- Integral term eliminates steady-state error

### ✅ **Safe**
- Emergency braking if too close
- Timeout if formation fails
- Graceful degradation to normal mode

### ✅ **Scalable**
- Leader tracks N followers
- Each follower only tracks one leader
- O(N) communication complexity

## Next Steps

1. **Integrate into vehicle_controller.py**:
   - Add platoon controller initialization
   - Add platoon state logic in control loop
   - Handle platoon velocity commands

2. **Update network protocol**:
   - Add platoon telemetry fields
   - Add platoon command types
   - Implement follower status updates

3. **Create GUI controls**:
   - Enable/disable platoon mode
   - Select leader
   - Monitor formation status
   - Display spacing & readiness

Would you like me to proceed with vehicle_controller.py integration next?
