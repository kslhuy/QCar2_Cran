# Visual Diagrams - Simple Controller Configuration

## 1. System Architecture Comparison

### OLD (Complex - ❌ Not Used Anymore)
```
┌──────────────────────────────────────────────────────────────┐
│                      main.py                                  │
│                   Initialization                              │
└─────────────┬──────────────────────────────────────────────────┘
              │
              │ load_config_file("config.yaml")
              ▼
┌──────────────────────────────────────────────────────────────┐
│                      config.yaml                              │
│  ├─ simulation settings                                       │
│  ├─ fleet settings                                            │
│  ├─ controller_params (nested dict)                          │
│  └─ other parameters                                         │
└─────────────┬──────────────────────────────────────────────────┘
              │
              │ Create QcarFleet(config)
              ▼
┌──────────────────────────────────────────────────────────────┐
│                     QcarFleet.py                              │
│  ├─ Extract config values                                     │
│  ├─ Build vehicle_config dict                                │
│  │  ├─ Add follower_mode                                      │
│  │  ├─ Add controller_params                                  │
│  │  └─ Add other parameters                                   │
│  └─ Create processes with vehicle_config                    │
└─────────────┬──────────────────────────────────────────────────┘
              │
              │ Pass vehicle_config to process
              ▼
┌──────────────────────────────────────────────────────────────┐
│                  VehicleProcess.py                            │
│  ├─ Extract vehicle_config parameters                         │
│  ├─ follower_mode = vehicle_config['follower_mode']         │
│  ├─ controller_params = vehicle_config['controller_params']  │
│  └─ Pass to VehicleFollowerController                        │
└─────────────┬──────────────────────────────────────────────────┘
              │
              │ Create VehicleFollowerController(
              │   follower_mode=follower_mode,
              │   controller_params=controller_params
              │ )
              ▼
┌──────────────────────────────────────────────────────────────┐
│           VehicleFollowerController.py                        │
│  ├─ Receive follower_mode from VehicleProcess                │
│  ├─ Receive controller_params dict                           │
│  ├─ Extract alpha, beta, v0, s0, ... from dict             │
│  └─ Use for control                                          │
└──────────────────────────────────────────────────────────────┘

Problems:
  ❌ Configuration passed through 4 files
  ❌ Multiple dict manipulations and extractions
  ❌ Error-prone nested dict operations
  ❌ Difficult to track parameter flow
  ❌ Hard to maintain and debug
```

### NEW (Simple - ✨ Currently Used)
```
┌──────────────────────────────────────────────────────────────┐
│           VehicleFollowerController.py                        │
│                                                               │
│  def __init__(self, vehicle_id, ...):                        │
│                                                               │
│    1. Load YAML file:                                         │
│       self.controller_config = _load_controller_config()     │
│                                                               │
│    2. Get mode (with per-vehicle override):                  │
│       self.follower_mode = _get_follower_mode()             │
│                                                               │
│    3. Load mode-specific config:                             │
│       if mode == "vehicle_following":                        │
│         _load_vehicle_following_config()                     │
│       elif mode == "trajectory":                             │
│         _load_trajectory_config()                            │
│       elif mode == "hybrid":                                 │
│         _load_hybrid_config()                                │
│                                                               │
│    4. Use parameters for control ✅                          │
└────────────────────────┬─────────────────────────────────────┘
                         │
                         │ Read from file
                         ▼
         ┌───────────────────────────────────────┐
         │   controller_config.yaml              │
         │  ├─ follower_mode                     │
         │  ├─ vehicle_following: {...}          │
         │  ├─ trajectory_following: {...}       │
         │  ├─ hybrid: {...}                     │
         │  └─ vehicle_mode_overrides: {...}     │
         └───────────────────────────────────────┘

Benefits:
  ✅ Direct file reading
  ✅ No config passing
  ✅ Simple, clean code
  ✅ Self-contained initialization
  ✅ Easy to understand and maintain
  ✅ Single point of configuration
```

---

## 2. Configuration File Structure

### controller_config.yaml Organization
```
controller_config.yaml
│
├── follower_mode: "vehicle_following"          # Global mode setting
│
├── vehicle_mode_overrides:                     # Per-vehicle overrides
│   ├── 1: "trajectory"
│   └── 2: "hybrid"
│
├── vehicle_following:                         # VF mode parameters
│   ├── Control parameters
│   │   ├── alpha, beta, v0, ...
│   │   └── K_gains: [...]
│   ├── Physical parameters
│   │   ├── l_r, l_f, mass, ...
│   │   └── k1, k2 (controller gains)
│   └── Steering parameters
│       ├── max_steering
│       └── k_steering
│
├── trajectory_following:                      # Trajectory mode parameters
│   ├── Speed controller
│   │   ├── K_p, K_i
│   │   └── startDelay
│   ├── Steering controller
│   │   ├── K_stanley
│   │   ├── enable_steering_control
│   │   └── lookahead_distance
│   └── Path settings
│       └── use_main_config_path
│
└── hybrid:                                     # Hybrid mode parameters
    ├── priority: "vehicle_following"
    ├── distance_threshold: 2.0
    ├── leader_data_timeout: 1.0
    └── hysteresis_offset: 0.5
```

---

## 3. Follower Mode Selection Flow

### How Mode is Determined
```
Start: Vehicle created

  │
  ├─► Check vehicle_mode_overrides dict
  │   │
  │   └─► Vehicle ID in overrides?
  │       │
  │       YES ─► Use override mode
  │       │
  │       NO  ─► Continue
  │
  ├─► Check default follower_mode
  │   │
  │   └─► Get mode from "follower_mode" key
  │
  ├─► Use selected mode
  │   │
  │   ├─ "vehicle_following" ─► Load VF config
  │   ├─ "trajectory"        ─► Load trajectory config
  │   ├─ "hybrid"            ─► Load both + hybrid config
  │   └─ other              ─► Default to "vehicle_following"
  │
  └─► Ready for control ✅
```

### Example Decision Tree
```
Vehicle 0 initialized:
  No override for vehicle 0
  Use default mode: "vehicle_following"
  Load vehicle_following config
  ✅ Vehicle 0 in vehicle_following mode

Vehicle 1 initialized:
  Check override: vehicle_mode_overrides[1] = "trajectory"
  Use override: "trajectory"
  Load trajectory config
  ✅ Vehicle 1 in trajectory mode

Vehicle 2 initialized:
  Check override: vehicle_mode_overrides[2] = "hybrid"
  Use override: "hybrid"
  Load vehicle_following + trajectory + hybrid config
  ✅ Vehicle 2 in hybrid mode

Vehicle 3 initialized:
  No override for vehicle 3
  Use default mode: "vehicle_following"
  Load vehicle_following config
  ✅ Vehicle 3 in vehicle_following mode
```

---

## 4. Hybrid Mode Switching Logic

### Smart Mode Switching Decision Tree
```
During control loop:

Check: Is leader data available and fresh?
  │
  ├─ YES: Leader data is recent (< leader_data_timeout)
  │       │
  │       └─► Check: Distance to leader?
  │           │
  │           ├─ CLOSE (< distance_threshold)
  │           │  └─► USE: vehicle_following mode
  │           │      (Actively follow leader)
  │           │
  │           └─ FAR (>= distance_threshold)
  │              └─► USE: trajectory mode
  │                  (Leader too far, follow path)
  │
  └─ NO: Leader data stale or missing (> leader_data_timeout)
         │
         └─► USE: trajectory mode
             (No fresh leader data, fall back to path)

Example thresholds:
  distance_threshold: 2.0 m
  leader_data_timeout: 1.0 s
  
With these:
  - If leader within 2m AND data fresh: vehicle_following
  - If leader beyond 2m OR data stale: trajectory
```

### Visual: Distance-Based Switching
```
Vehicle following mode ─────────────────┐
                                        │
                         leader_close   │
        │────────────────────────────│  │
        │ distance < threshold       │  │
        │ (2.0 m default)            │  │
        │                            │  │
    ┌───┴────────────────────────────┴──┐
    │   ACTIVE LEADER FOLLOWING         │
    │   (vehicle_following mode)         │
    │                                    │
    │   • Compute acceleration from      │
    │     leader error                   │
    │   • Track leader spacing           │
    │   • Responsive to leader motion    │
    └──────────────┬─────────────────────┘
                   │
                   │ leader_far
                   │ OR data_stale
                   │
    ┌──────────────┴─────────────────────┐
    │   FALLBACK TO PATH FOLLOWING       │
    │   (trajectory mode)                │
    │                                    │
    │   • Follow predefined path         │
    │   • Independent navigation         │
    │   • Robust to comms failure        │
    └────────────────────────────────────┘

Trajectory mode ──────────────────────────
```

---

## 5. Data Flow Comparison

### Before: Complex Multi-File Data Flow
```
                         ┌─────────────┐
                         │config.yaml  │
                         └──────┬──────┘
                                │
              ┌─────────────────┼─────────────────┐
              │ Read: follower_mode, controller_params, ...
              │
              ▼
        ┌──────────────┐
        │ QcarFleet    │
        │ ├─ Extract   │
        │ ├─ Build     │
        │ │ vehicle_   │
        │ │ config     │
        │ │ dict       │
        │ └─ Create    │
        │   processes  │
        └────┬─────────┘
             │
    Pass vehicle_config dict
             │
             ▼
        ┌─────────────────┐
        │ VehicleProcess  │
        │ ├─ Extract      │
        │ │ follower_mode │
        │ ├─ Extract      │
        │ │ controller_   │
        │ │ params        │
        │ └─ Pass to      │
        │   controller    │
        └────┬────────────┘
             │
    Pass follower_mode, controller_params
             │
             ▼
        ┌──────────────────┐
        │ VehicleFollower  │
        │ Controller       │
        │ ├─ Receive       │
        │ │ follower_mode  │
        │ ├─ Receive       │
        │ │ controller_    │
        │ │ params dict    │
        │ ├─ Extract:     │
        │ │  alpha, beta,  │
        │ │  v0, s0, ...   │
        │ └─ Use for      │
        │   control       │
        └──────────────────┘

Complexity: 4 files involved, multiple dict operations
```

### After: Simple Direct Reading
```
                      ┌──────────────────────┐
                      │ controller_config.   │
                      │ yaml                 │
                      │ ├─ follower_mode     │
                      │ ├─ vehicle_following │
                      │ ├─ trajectory_       │
                      │ │ following          │
                      │ ├─ hybrid            │
                      │ └─ vehicle_mode_     │
                      │   overrides          │
                      └──────────┬───────────┘
                                 │
                    Direct file reading
                                 │
                      ┌──────────▼───────────┐
                      │ VehicleFollower      │
                      │ Controller           │
                      │ ├─ Load YAML file    │
                      │ ├─ Get follower_mode │
                      │ ├─ Load mode-specific│
                      │ │ parameters         │
                      │ └─ Use for control   │
                      └──────────────────────┘

Complexity: 1 file involved, direct reading
```

---

## 6. Parameter Reference Card

### Vehicle-Following Mode Parameters
```
┌────────────────────────────────┐
│   VEHICLE-FOLLOWING CONTROL    │
├────────────────────────────────┤
│ Alpha:        1.2  (accel gain)│
│ Beta:         1.5  (decel gain)│
│ V0:           0.4  (desired v) │
│ S0:           1.0  (min space) │
│ Hi:           0.3  (time hdwy) │
│ Ri:           1.0  (CACC dist) │
│ K_gains: [1.0, 1.0, 0.5, 2.0] │
└────────────────────────────────┘
```

### Trajectory Mode Parameters
```
┌─────────────────────────────────┐
│   TRAJECTORY FOLLOWING CONTROL  │
├─────────────────────────────────┤
│ K_p:                    0.1     │
│ K_i:                    0.08    │
│ K_stanley:              0.8     │
│ lookahead_distance:     0.5     │
│ enable_steering:        false   │
│ startDelay:             1       │
└─────────────────────────────────┘
```

### Hybrid Mode Parameters
```
┌──────────────────────────────────┐
│      HYBRID MODE SETTINGS        │
├──────────────────────────────────┤
│ Priority: vehicle_following      │
│ Distance threshold:     2.0 m    │
│ Leader data timeout:    1.0 s    │
│ Hysteresis offset:      0.5 m    │
└──────────────────────────────────┘
```

---

## 7. Configuration Modification Map

### How Changing Parameters Affects Behavior
```
INCREASE v0 (desired velocity)
  ↓
Vehicles accelerate to higher speed
  ↓
Faster fleet movement


DECREASE s0 (minimum spacing)
  ↓
Tighter vehicle-to-vehicle spacing
  ↓
Closer, tighter formation


INCREASE alpha (acceleration gain)
  ↓
More aggressive response to spacing errors
  ↓
Faster acceleration and braking


INCREASE K_stanley (steering gain)
  ↓
More responsive steering
  ↓
Better path tracking but less smooth


INCREASE distance_threshold (hybrid)
  ↓
Prefer leader following for longer distances
  ↓
Less fallback to trajectory mode
```

---

## 8. System State Diagram

### Vehicle Lifecycle
```
        ┌──────────────┐
        │   Created    │
        └──────┬───────┘
               │
               │ Load controller_config.yaml
               ▼
        ┌──────────────────────────┐
        │ Determine follower_mode  │
        │ (with per-vehicle check) │
        └──────┬───────────────────┘
               │
       ┌───────┴───────┬──────────┐
       │               │          │
       ▼               ▼          ▼
   ┌────────────┐ ┌────────────┐ ┌──────────┐
   │vehicle_    │ │trajectory  │ │  hybrid  │
   │following   │ │            │ │          │
   └────┬───────┘ └────┬───────┘ └────┬─────┘
        │              │               │
        │ Load VF cfg  │ Load Traj cfg │ Load all
        ▼              ▼               ▼
   ┌────────────────────────────────────────┐
   │    READY FOR CONTROL                   │
   │  Parameters loaded and initialized     │
   └────────┬─────────────────────────────┘
            │
            │ Main control loop
            ▼
   ┌────────────────────────────────────────┐
   │    RUNNING                             │
   │  Compute control commands continuously│
   └────────┬─────────────────────────────┘
            │
            │ Stop event
            ▼
   ┌────────────────────────────────────────┐
   │    CLEANUP & SHUTDOWN                  │
   │  Stop controllers, close connections   │
   └────────────────────────────────────────┘
```

---

## Summary

These diagrams show:
1. ✅ **Old complex architecture** (no longer used)
2. ✅ **New simple architecture** (currently used)
3. ✅ **Configuration file structure** (what to edit)
4. ✅ **Mode selection logic** (how modes are chosen)
5. ✅ **Hybrid switching** (smart mode transitions)
6. ✅ **Data flow comparison** (before vs after)
7. ✅ **Parameter reference** (quick lookup)
8. ✅ **System lifecycle** (vehicle states)

**The new architecture is simpler, cleaner, and easier to maintain!** ✨
