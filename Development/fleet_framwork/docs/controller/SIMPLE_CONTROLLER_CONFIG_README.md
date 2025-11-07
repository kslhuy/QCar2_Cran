# Simple Controller Configuration - No Complexity!

## The Problem (Before)
The previous approach required passing complex configuration through multiple files:
- `config.yaml` → `QcarFleet.py` → `VehicleProcess.py` → `VehicleFollowerController.py`

This was complicated and error-prone.

## The Solution (Now)
**The controller reads `controller_config.yaml` directly!** Simple and clean! ✨

```
┌─────────────────────────────────────────┐
│  VehicleFollowerController.py            │
│  ↓                                       │
│  Reads controller_config.yaml directly  │
└─────────────────────────────────────────┘
```

No need to pass through QcarFleet or VehicleProcess!

---

## Quick Start

### 1. Change Follower Mode
Edit `controller_config.yaml`:

```yaml
follower_mode: "vehicle_following"  # Change this
```

**Available modes:**
- `"vehicle_following"` - Follow leader using CACC/IDM/LOOKAHEAD
- `"trajectory"` - Follow predefined waypoints independently  
- `"hybrid"` - Smart switching between both modes

### 2. Make Vehicle 1 Use Different Mode
Edit `controller_config.yaml`:

```yaml
vehicle_mode_overrides:
  1: "trajectory"   # Vehicle 1 uses trajectory mode
  # Vehicle 0, 2, 3, etc. use the default mode
```

### 3. Adjust Control Parameters
Edit the appropriate section in `controller_config.yaml`:

**For vehicle-following mode:**
```yaml
vehicle_following:
  alpha: 1.2
  beta: 1.5
  v0: 0.4          # Desired velocity
  s0: 1            # Minimum spacing
  hi: 0.3          # Time headway
  K_gains: [1.0, 1.0, 0.5, 2.0]
```

**For trajectory mode:**
```yaml
trajectory_following:
  K_p: 0.1
  K_stanley: 0.8
  lookahead_distance: 0.5
```

**For hybrid mode:**
```yaml
hybrid:
  priority: "vehicle_following"     # Which mode to prefer
  distance_threshold: 2.0           # Switch if leader > 2m away
  leader_data_timeout: 1.0          # Switch if data older than 1s
```

---

## How It Works

### VehicleFollowerController.__init__()

```python
def __init__(self, vehicle_id, controller_type="CACC", config=None, logger=None):
    # Load controller_config.yaml directly
    self.controller_config = self._load_controller_config()
    
    # Get follower mode (with per-vehicle override support)
    self.follower_mode = self._get_follower_mode()
    
    # Load mode-specific config
    if self.follower_mode == "vehicle_following":
        self._load_vehicle_following_config()
    elif self.follower_mode == "trajectory":
        self._load_trajectory_config()
    elif self.follower_mode == "hybrid":
        self._load_hybrid_config()
```

That's it! No complex config passing needed.

---

## File Structure

```
fleet_framwork/
├── config.yaml                    # Main simulation config (unchanged)
├── controller_config.yaml         # NEW: Controller-specific config
├── VehicleFollowerController.py   # UPDATED: Reads controller_config.yaml directly
└── VehicleProcess.py              # UNCHANGED: No need to pass follower_mode
```

---

## Configuration Examples

### Example 1: All Vehicles Follow Leader
```yaml
follower_mode: "vehicle_following"
vehicle_mode_overrides: {}  # Empty - all use default
```

### Example 2: Mix of Modes
```yaml
follower_mode: "trajectory"  # Default
vehicle_mode_overrides:
  1: "vehicle_following"     # Vehicle 1 follows leader
  2: "hybrid"                # Vehicle 2 uses smart switching
  # Vehicles 0, 3, etc. use trajectory
```

### Example 3: All Vehicles Use Hybrid Mode
```yaml
follower_mode: "hybrid"
hybrid:
  priority: "vehicle_following"
  distance_threshold: 2.0
  leader_data_timeout: 1.0
```

### Example 4: Fine-Tune Vehicle Following
```yaml
follower_mode: "vehicle_following"
vehicle_following:
  v0: 0.5          # Desired velocity
  s0: 0.5          # Tighter spacing
  hi: 0.2          # Shorter time headway
  K_gains: [1.5, 1.0, 0.5, 2.0]  # Higher acceleration gain
```

---

## What Didn't Change?

These files remain **unchanged**:
- `QcarFleet.py` - No need to pass follower_mode anymore
- `VehicleProcess.py` - Works as before
- `config.yaml` - Main simulation config

The controller reads its own config file. Much simpler! ✨

---

## Testing

1. **Edit `controller_config.yaml`** - Change `follower_mode` to what you want
2. **Run simulation** - `python main.py`
3. **Check logs** - Look for messages like:
   ```
   Vehicle 0: Using follower mode 'vehicle_following'
   Vehicle 0: Loaded vehicle-following config
   ```

---

## Summary

| Feature | Before | After |
|---------|--------|-------|
| Config Location | Spread across multiple files | Single file: `controller_config.yaml` |
| Easy to Change | Need to edit multiple places | Edit one YAML file |
| Per-Vehicle Config | Complex | Simple: `vehicle_mode_overrides` dict |
| Code Complexity | High | Low |
| Dependencies | QcarFleet → VehicleProcess → Controller | Controller reads directly |

**Result:** Simple, clean, maintainable! 🎉
