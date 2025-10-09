# Platoon Mode Implementation - Complete Guide

## Overview
The platoon mode enables multiple QCars to coordinate their movement in a leader-follower formation. The system uses YOLO-based vision detection for leader tracking and PI control for maintaining precise spacing.

## Architecture

### 1. State Machine (`state_machine.py`)
Added 5 new states for platoon operation:

- **PLATOON_LEADER_FORMING**: Leader is waiting for followers to join
- **PLATOON_FOLLOWER_SEARCHING**: Follower is looking for leader via YOLO
- **PLATOON_FOLLOWER_FORMING**: Follower detected leader, establishing formation
- **PLATOON_ACTIVE**: Platoon is formed and operating
- **PLATOON_LOST**: Follower lost sight of leader

**Helper Methods:**
```python
is_in_platoon()          # Check if vehicle is in any platoon state
is_platoon_leader()      # Check if vehicle is platoon leader
is_platoon_follower()    # Check if vehicle is platoon follower
```

### 2. Platoon Controller (`platoon_controller.py`)
New file implementing the platoon coordination logic.

**Key Components:**

#### PlatoonConfig
Configuration parameters for platoon behavior:
```python
target_spacing: float = 1.5      # Target distance between vehicles (m)
spacing_tolerance: float = 0.3   # Formation tolerance (±m)
formation_speed: float = 0.3     # Speed during formation (m/s)
active_speed: float = 0.75       # Speed when platoon is active (m/s)
max_spacing: float = 3.0         # Max distance before losing leader (m)
min_safe_spacing: float = 0.8    # Emergency brake distance (m)

# PI Controller Gains
kp_spacing: float = 0.3          # Proportional gain
ki_spacing: float = 0.05         # Integral gain
integral_filter: float = 0.2     # Anti-windup filter
```

#### PlatoonController Class
Main methods:
- `enable_as_leader()` / `enable_as_follower()` / `disable()`: Mode control
- `update_leader_info(detected, distance, velocity)`: Process leader detection
- `compute_follower_velocity(current_velocity)`: PI-based spacing control
- `is_spacing_stable()`: Check if formation is ready (2 seconds stable)
- `get_telemetry()`: Export platoon status for network/GUI

**Safety Features:**
- Emergency braking when too close (< 0.8m)
- Anti-windup integral clamping
- Velocity limits (0.1 - 2.0 m/s)
- Timeout detection for lost leader

### 3. Vehicle Controller Integration (`vehicle_controller.py`)
Enhanced main controller with platoon logic.

#### New Methods

**_handle_platoon_states(x, y, velocity)**
Implements the state machine logic:
- Leader: Waits for follower readiness → transitions to ACTIVE
- Follower: Searches → Forms → Active → Lost detection

**_compute_platoon_velocity(current_velocity)**
Returns appropriate velocity for each state:
- LEADER_FORMING: formation_speed (0.3 m/s)
- FOLLOWER_SEARCHING: formation_speed (0.3 m/s)
- FOLLOWER_FORMING: PI controller output
- ACTIVE: PI controller output
- LOST: Slow deceleration (0.5 m/s)

**_get_leader_velocity_from_network()**
Placeholder for receiving leader velocity via network (future enhancement).

#### Control Loop Integration
```python
# In _control_iteration():
if self.state_machine.is_in_platoon():
    self._handle_platoon_states(x, y, velocity)

# In _compute_control():
if self.state_machine.is_in_platoon():
    v_ref_adjusted = self._compute_platoon_velocity(velocity)
else:
    v_ref_adjusted = self.v_ref * self.yolo_gain
```

#### Command Processing
Added two new command types:
```python
# Enable platoon
{
    'type': 'enable_platoon',
    'role': 'leader' or 'follower',
    'leader_id': <car_id>,  # For followers
    'node_sequence': [...]  # Optional path sync
}

# Disable platoon
{
    'type': 'disable_platoon'
}
```

#### Telemetry Enhancement
Platoon data now included in telemetry:
```python
telemetry.update(self.platoon_controller.get_telemetry())
# Adds: platoon_role, platoon_active, leader_detected, 
#       leader_distance, spacing_error, desired_speed, formation_ready
```

### 4. GUI Controller (`gui_controller.py`)
Enhanced GUI with platoon controls.

#### New UI Components
**Platoon Control Panel** (per vehicle):
- Role selection: Leader / Follower radio buttons
- Enable Platoon button (orange)
- Disable button (gray)
- Platoon status indicator with real-time updates

#### Status Display
Dynamic platoon status with color coding:
- 🟢 Leader: Active leader
- 🟢 Active (X.Xm): Platoon formed, showing spacing
- 🟡 Forming (X.Xm): Formation in progress
- 🔴 Searching: Looking for leader
- ⚫ Inactive: Platoon disabled

#### New Methods
```python
enable_platoon(car_id, role)   # Send enable command
disable_platoon(car_id)        # Send disable command
```

#### Telemetry Update
Enhanced update loop displays platoon metrics:
- Platoon active/inactive status
- Current role (Leader/Follower)
- Leader detection status
- Real-time spacing distance
- Formation readiness

## Usage Guide

### Starting a 2-Vehicle Platoon

#### Step 1: Start the GUI
```bash
cd Development\QCar2_multi-vehicle_control
python gui_controller.py
```

#### Step 2: Start QCar Controllers
On each QCar, run:
```bash
cd Development\multi_vehicle_self_driving_RealQcar
python main_vehicle_control.py --car-id 0  # or 1, 2, etc.
```

#### Step 3: Enable Platoon Mode

**On Leader (Car 0):**
1. Select "Leader" radio button
2. Click "Enable Platoon"
3. Leader state: PLATOON_LEADER_FORMING
4. Leader drives at formation_speed (0.3 m/s)

**On Follower (Car 1):**
1. Select "Follower" radio button
2. Click "Enable Platoon"
3. Follower states:
   - SEARCHING → looking for car via YOLO
   - FORMING → adjusting spacing
   - ACTIVE → formation complete

#### Step 4: Monitor Status
Watch the GUI platoon status indicators:
- Leader: 🟢 Platoon: Leader
- Follower: 🟡 Forming (X.Xm) → 🟢 Active (X.Xm)

#### Step 5: Disable Platoon
Click "Disable" button on either vehicle to exit platoon mode.

## Key Parameters to Tune

### Spacing Control
```python
target_spacing = 1.5 m      # Increase for more distance
spacing_tolerance = 0.3 m   # Tighten for precise formation
```

### PI Controller Gains
```python
kp_spacing = 0.3   # Increase for faster response
ki_spacing = 0.05  # Increase to eliminate steady-state error
```

### Speed Profiles
```python
formation_speed = 0.3 m/s   # Slow speed for joining
active_speed = 0.75 m/s     # Cruising speed
```

### Safety Margins
```python
min_safe_spacing = 0.8 m    # Emergency brake threshold
max_spacing = 3.0 m         # Lost leader threshold
```

## Testing Checklist

### Formation Testing
- [ ] Leader transitions to LEADER_FORMING
- [ ] Follower detects leader via YOLO
- [ ] Follower approaches at formation_speed
- [ ] Spacing stabilizes within tolerance
- [ ] Formation becomes ready after 2 seconds
- [ ] Both vehicles transition to ACTIVE

### Active Operation
- [ ] Follower maintains spacing during straight driving
- [ ] Follower tracks leader through turns
- [ ] PI controller eliminates spacing errors
- [ ] Speed adjusts smoothly

### Error Recovery
- [ ] Follower detects leader loss
- [ ] Transitions to PLATOON_LOST state
- [ ] Vehicle decelerates safely
- [ ] Can re-form platoon after recovery

### Safety
- [ ] Emergency braking at min_safe_spacing
- [ ] Integral anti-windup prevents overshoot
- [ ] Velocity limits respected
- [ ] Collision avoidance overrides platoon control

## Network Protocol

### Telemetry Data (QCar → GUI)
```json
{
  "timestamp": 1234567890.123,
  "x": 1.23,
  "y": 2.34,
  "v": 0.75,
  "platoon_role": "Leader" | "Follower" | "None",
  "platoon_active": true,
  "leader_detected": true,
  "leader_distance": 1.52,
  "spacing_error": 0.02,
  "desired_speed": 0.73,
  "formation_ready": true
}
```

### Command Data (GUI → QCar)
```json
// Enable platoon
{
  "type": "enable_platoon",
  "role": "leader" | "follower",
  "leader_id": 0,
  "node_sequence": [10, 4, 20, 10]
}

// Disable platoon
{
  "type": "disable_platoon"
}
```

## Future Enhancements

### Network-Based Velocity Sharing
Currently, followers use YOLO distance only. Future: leader broadcasts velocity via network for better tracking.

**Implementation:**
```python
# In leader telemetry
telemetry['leader_velocity'] = velocity

# In follower _get_leader_velocity_from_network()
return self._leader_velocity_from_network  # From received telemetry
```

### Multi-Follower Support
Extend to 3+ vehicles in formation.

**Changes needed:**
- Track multiple followers in leader
- Each follower follows vehicle ahead
- Propagate spacing errors through chain

### Adaptive Spacing
Adjust spacing based on velocity:
```python
target_spacing = base_spacing + velocity * time_gap
```

### Path Synchronization
Ensure all platoon members follow same waypoint sequence:
```python
if 'node_sequence' in command:
    self.node_sequence = command['node_sequence']
    self.steering_controller.update_waypoints(...)
```

## Troubleshooting

### Follower Can't Detect Leader
- Check YOLO camera is working: `self.yolo.cars`
- Verify leader is within camera view (1-3m ahead)
- Check lighting conditions for YOLO performance

### Spacing Oscillates
- Reduce `kp_spacing` (try 0.2)
- Increase `integral_filter` (try 0.3)
- Check dt accuracy in control loop

### Formation Never Stabilizes
- Increase `spacing_tolerance` (try 0.5)
- Reduce required stability time in `is_spacing_stable()`
- Check YOLO distance measurement noise

### Vehicle Stops in Platoon Mode
- Check for min_safe_spacing trigger
- Verify velocity limits aren't too restrictive
- Look for collision avoidance override

## Files Modified

1. **state_machine.py**: Added 5 platoon states and helper methods
2. **platoon_controller.py**: New file with PI spacing controller
3. **vehicle_controller.py**: Integrated platoon logic into control loop
4. **gui_controller.py**: Added platoon UI controls and status display
5. **remote_controller.py**: No changes needed (handles new commands automatically)

## Summary

The platoon implementation is **complete and ready for testing**. The system provides:

✅ Leader-follower formation control
✅ YOLO-based leader detection
✅ PI spacing controller with safety features
✅ State machine for robust coordination
✅ GUI controls for easy operation
✅ Real-time telemetry and status display
✅ Safety features (emergency braking, anti-windup, timeouts)

**Next step:** Test with physical QCars to tune parameters and validate performance!
