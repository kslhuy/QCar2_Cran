# Architecture: Simple Controller Configuration

## Overview

```
Before (Complex):
┌──────────────┐
│ config.yaml  │
└──────┬───────┘
       │ passes to
       ▼
┌─────────────────┐
│ QcarFleet.py    │
└──────┬──────────┘
       │ passes to
       ▼
┌──────────────────────┐
│ VehicleProcess.py    │
└──────┬───────────────┘
       │ passes to
       ▼
┌────────────────────────────────┐
│ VehicleFollowerController.py    │
└────────────────────────────────┘

Problem: Configuration passed through 4 files! 😞
```

```
After (Simple):
┌──────────────────────────────────┐
│ controller_config.yaml           │
└──────┬───────────────────────────┘
       │ reads directly
       ▼
┌────────────────────────────────┐
│ VehicleFollowerController.py    │
└────────────────────────────────┘

Solution: Controller reads its own config! ✨
```

---

## File Responsibilities

### `controller_config.yaml` (NEW)
**Purpose:** Centralized configuration for all follower controller behavior

**Contains:**
- Follower mode selection (`vehicle_following`, `trajectory`, `hybrid`)
- Per-vehicle mode overrides
- Vehicle-following parameters (CACC/IDM/LOOKAHEAD)
- Trajectory-following parameters
- Hybrid mode settings

**Usage:** Read directly by `VehicleFollowerController.__init__()`

```python
self.controller_config = self._load_controller_config()
# Done! No passing through other files needed
```

---

### `VehicleFollowerController.py` (MODIFIED)
**Purpose:** Handle all follower-specific control logic

**New Methods:**
- `_load_controller_config()` - Load YAML file
- `_get_follower_mode()` - Get mode with per-vehicle overrides
- `_load_vehicle_following_config()` - Load vehicle-following parameters
- `_load_trajectory_config()` - Load trajectory parameters
- `_load_hybrid_config()` - Load hybrid parameters

**Key Change:** Constructor now reads `controller_config.yaml` directly

```python
def __init__(self, vehicle_id, controller_type="CACC", config=None, logger=None):
    # Load and apply controller_config.yaml
    self.controller_config = self._load_controller_config()
    self.follower_mode = self._get_follower_mode()
    
    if self.follower_mode == "vehicle_following":
        self._load_vehicle_following_config()
    elif self.follower_mode == "trajectory":
        self._load_trajectory_config()
    elif self.follower_mode == "hybrid":
        self._load_hybrid_config()
```

**Result:** Clean, self-contained initialization

---

### `config.yaml` (UNCHANGED)
**Purpose:** Main simulation configuration (fleet size, paths, rates, etc.)

**Does NOT contain:** Follower controller parameters (moved to `controller_config.yaml`)

**Usage:** By `main.py`, `QcarFleet.py`, `VehicleLeaderController.py`

---

### `QcarFleet.py` (UNCHANGED)
**Purpose:** Build and spawn fleet

**Does NOT need to:**
- Pass follower_mode to vehicles
- Pass controller parameters
- Handle configuration routing

**Result:** Simpler code, no config passing

---

### `VehicleProcess.py` (UNCHANGED)
**Purpose:** Vehicle process loop

**Does NOT need to:**
- Receive follower_mode
- Pass controller parameters
- Modify controller configuration

**Result:** Simpler code, no config passing

---

## Configuration Flow

### Current (Proposed):
```
User edits controller_config.yaml
        │
        │ VehicleFollowerController reads directly
        ▼
   Follower mode determined
        │
        ├─ Load vehicle_following config
        ├─ Load trajectory config
        └─ Load hybrid config
```

### Parameter Loading:
```
_load_vehicle_following_config()
├─ alpha: 1.2
├─ beta: 1.5
├─ v0: 0.4
├─ s0: 1
├─ hi: 0.3
├─ K_gains: [1.0, 1.0, 0.5, 2.0]
└─ ... more parameters

_load_trajectory_config()
├─ K_p: 0.1
├─ K_stanley: 0.8
├─ lookahead_distance: 0.5
└─ ... more parameters

_load_hybrid_config()
├─ priority: "vehicle_following"
├─ distance_threshold: 2.0
├─ leader_data_timeout: 1.0
└─ ... hybrid settings
```

---

## Per-Vehicle Overrides

```yaml
# Default for all vehicles
follower_mode: "vehicle_following"

# Specific vehicles can override
vehicle_mode_overrides:
  1: "trajectory"        # Vehicle 1 uses trajectory
  2: "hybrid"            # Vehicle 2 uses hybrid
  # Vehicle 0, 3, 4 use default (vehicle_following)
```

**How it works:**
```python
def _get_follower_mode(self):
    overrides = self.controller_config.get('vehicle_mode_overrides', {})
    
    if self.vehicle_id in overrides:
        return overrides[self.vehicle_id]  # Use override
    
    return self.controller_config.get('follower_mode', 'vehicle_following')  # Use default
```

---

## Mode-Specific Behavior

### Vehicle-Following Mode
```python
if self.follower_mode == "vehicle_following":
    self._load_vehicle_following_config()
    # Controller will compute acceleration based on leader data
```

**Parameters used:**
- `alpha`, `beta` - Acceleration sensitivity
- `v0` - Desired velocity
- `s0`, `hi`, `ri` - Spacing parameters
- `K_gains` - Control gains

---

### Trajectory Mode
```python
elif self.follower_mode == "trajectory":
    self._load_trajectory_config()
    self.init_trajectory_controller()
    # Controller will follow predefined waypoints
```

**Parameters used:**
- `K_p`, `K_i` - Speed controller gains
- `K_stanley` - Steering controller gain
- `lookahead_distance` - Path following distance
- `enable_steering_control` - Enable/disable steering

---

### Hybrid Mode
```python
elif self.follower_mode == "hybrid":
    self._load_hybrid_config()
    # Load both vehicle-following and trajectory configs
```

**Parameters used:**
- All vehicle-following parameters
- All trajectory parameters
- Hybrid-specific settings:
  - `priority` - Preferred mode
  - `distance_threshold` - Switch distance
  - `leader_data_timeout` - Data freshness threshold
  - `hysteresis_offset` - Hysteresis for switching

**Switching logic:**
```
if leader_is_close AND leader_data_is_fresh:
    use vehicle_following
else:
    use trajectory
```

---

## Example: Changing Modes

### To switch all vehicles to trajectory mode:
1. Edit `controller_config.yaml`
2. Change: `follower_mode: "trajectory"`
3. Run: `python main.py`
4. Done!

### To use different mode for Vehicle 1:
1. Edit `controller_config.yaml`
2. Add:
   ```yaml
   vehicle_mode_overrides:
     1: "trajectory"
   ```
3. Run: `python main.py`
4. Done!

### To fine-tune vehicle-following:
1. Edit `controller_config.yaml`
2. Adjust parameters in `vehicle_following` section:
   ```yaml
   vehicle_following:
     v0: 0.5        # Increase speed
     s0: 0.5        # Tighter spacing
     K_gains: [1.5, 1.0, 0.5, 2.0]  # More aggressive
   ```
3. Run: `python main.py`
4. Done!

---

## Benefits

| Aspect | Before | After |
|--------|--------|-------|
| **Config Location** | Multiple files | Single file |
| **Passing Complexity** | QcarFleet → VehicleProcess → Controller | Controller reads directly |
| **Per-Vehicle Config** | Complex dict manipulation | Simple dict in YAML |
| **Modification** | Edit multiple files | Edit one YAML file |
| **Code Clarity** | Mixed concerns | Clear separation |
| **Dependency Chain** | 4+ files involved | 1 file involved |
| **Error Prone** | High | Low |
| **Maintainability** | Difficult | Easy |

---

## Summary

✨ **Simple approach:**
- Controller reads its own config file
- No complex passing through QcarFleet or VehicleProcess
- Clean, maintainable, easy to modify
- Per-vehicle overrides built-in

🎉 **Result:** Flexible, simple, maintainable configuration system!
