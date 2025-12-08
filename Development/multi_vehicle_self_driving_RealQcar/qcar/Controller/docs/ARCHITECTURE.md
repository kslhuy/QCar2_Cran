# Modular Longitudinal Controller Architecture

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    FollowingLeaderState                         │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  Configuration: controller_type = 'cacc' / 'pi' / 'hybrid'│ │
│  └──────────────────────────────────────────────────────────┘  │
│                           │                                     │
│                           ▼                                     │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │           ControllerFactory.create()                     │  │
│  │  (Returns appropriate controller instance)               │  │
│  └──────────────────────────────────────────────────────────┘  │
│                           │                                     │
│         ┌─────────────────┴─────────────────┐                  │
│         ▼                 ▼                 ▼                   │
│  ┌──────────┐      ┌──────────┐      ┌──────────┐             │
│  │   CACC   │      │    PI    │      │  Hybrid  │             │
│  │Controller│      │Controller│      │Controller│             │
│  └──────────┘      └──────────┘      └──────────┘             │
│         │                 │                 │                   │
│         └─────────────────┴─────────────────┘                  │
│                           │                                     │
│                           ▼                                     │
│                   compute_throttle()                            │
│                           │                                     │
│                           ▼                                     │
│                    Throttle Command                             │
│                         (u)                                     │
└─────────────────────────────────────────────────────────────────┘
```

## Data Flow

```
┌──────────────┐
│  Sensor Data │
│  (x, y, θ, v)│
└──────┬───────┘
       │
       ▼
┌──────────────────────┐
│  V2V Communication   │
│  (Leader x, y, θ, v) │
└──────┬───────────────┘
       │
       ▼
┌───────────────────────────────────────────────────────┐
│  Prepare States                                       │
│  • follower_state = {x, y, theta, velocity, ...}      │
│  • leader_state = {x, y, theta, velocity}             │
└──────┬────────────────────────────────────────────────┘
       │
       ▼
┌───────────────────────────────────────────────────────┐
│  Longitudinal Controller                              │
│  (Selected type: CACC/PI/Hybrid)                      │
│                                                       │
│  Input:  follower_state, leader_state, dt             │
│  Output: throttle command (u)                         │
└──────┬────────────────────────────────────────────────┘
       │
       ▼
┌──────────────────┐
│  Vehicle Actuator│
│  (Apply throttle)│
└──────────────────┘
```

## Controller Comparison

```
┌─────────────┬──────────────────┬────────────────┬─────────────┐
│ Feature     │ CACC             │ PI             │ Hybrid      │
├─────────────┼──────────────────┼────────────────┼─────────────┤
│ Inputs      │ • Spacing error  │ • Velocity err │ Both        │
│             │ • Velocity error │                │             │
├─────────────┼──────────────────┼────────────────┼─────────────┤
│ Best For    │ Platoon/convoy   │ Simple follow  │ Mixed       │
│             │ Precise spacing  │ Basic tracking │ conditions  │
├─────────────┼──────────────────┼────────────────┼─────────────┤
│ Complexity  │ Medium           │ Low            │ High        │
├─────────────┼──────────────────┼────────────────┼─────────────┤
│ Parameters  │ s0, h, K[2]      │ kp, ki         │ Both sets   │
├─────────────┼──────────────────┼────────────────┼─────────────┤
│ Requires    │ Leader position  │ Target vel     │ Auto-detect │
│             │ and velocity     │                │             │
└─────────────┴──────────────────┴────────────────┴─────────────┘
```

## CACC Control Law Detail

```
┌────────────────────────────────────────────────────────────┐
│  CACC (Cooperative Adaptive Cruise Control)                │
├────────────────────────────────────────────────────────────┤
│                                                            │
│  Step 1: Calculate Spacing                                 │
│    spacing = √[(x_leader - x_follower)² + (y_leader - y_f)²]
│                                                            │
│  Step 2: Calculate Desired Spacing (CTH Policy)            │
│    spacing_desired = s0 + h × v_follower                   │
│                      └─┘   └┘                              │
│                       │     └─ time headway                │
│                       └─ minimum spacing                   │
│                                                            │
│  Step 3: Calculate Errors                                  │
│    e_spacing = spacing - spacing_desired                   │
│    e_velocity = v_leader - v_follower                      │
│                                                            │
│  Step 4: CACC Control Law                                  │
│    acceleration = K[0] × e_spacing + K[1] × e_velocity     │
│                   └──┘             └───┘                   │
│                    │                 └─ velocity gain      │
│                    └─ spacing gain                         │
│                                                            │
│  Step 5: Convert to Throttle                               │
│    throttle = acc_to_throttle_gain × acceleration          │
│    throttle = clip(throttle, -max, +max)                   │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

## How to Switch Controllers

```
┌─────────────────────────────────────────────────────────────┐
│  OPTION 1: Quick Code Change                                │
│  ───────────────────────────                                │
│  In following_leader_state.py:                              │
│                                                             │
│    self.controller_type = 'cacc'  # ← Change this line!     │
│                                                             │
│  That's it! Restart and it uses the new controller.         │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  OPTION 2: Configuration File                               │
│  ────────────────────────                                   │
│  In config.yaml or similar:                                 │
│                                                             │
│    longitudinal_controller_type: 'cacc'                     │
│                                                             │
│  Then read in code:                                         │
│    self.controller_type = config.longitudinal_controller_type│
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  OPTION 3: Runtime Switching                                │
│  ──────────────────────                                     │
│  Add method to state class:                                 │
│                                                             │
│    def switch_controller(self, new_type):                   │
│        self.controller_type = new_type                      │
│        self._initialize_longitudinal_controller()           │
│                                                             │
│  Usage:                                                     │
│    state.switch_controller('pi')                            │
└─────────────────────────────────────────────────────────────┘
```

## File Structure

```
qcar/
├── Controller/
│   ├── CACC.py                      # Original CACC implementation
│   ├── longitudinal_controllers.py  # ✨ NEW: Modular controllers
│   │   ├── LongitudinalControllerBase (abstract)
│   │   ├── PIVelocityController
│   │   ├── CACCLongitudinalController
│   │   ├── HybridController
│   │   └── ControllerFactory
│   ├── controllers.py               # Existing speed/steering controllers
│   ├── platoon_controller.py        # Platoon coordination logic
│   ├── README_CONTROLLERS.md        # ✨ NEW: Detailed documentation
│   ├── QUICK_SWITCH_GUIDE.py        # ✨ NEW: Copy-paste examples
│   ├── test_controllers.py          # ✨ NEW: Test script
│   └── controller_config_example.yaml # ✨ NEW: Config template
│
└── StateMachine/
    └── following_leader_state.py    # ✨ UPDATED: Uses modular controllers
```

## Integration Points

```
┌──────────────────────────────────────────────────────────────┐
│  Vehicle Logic                                               │
│                                                              │
│  ┌────────────────────────────────────────────────────────┐ │
│  │ State Machine                                          │ │
│  │  ┌──────────────────────────────────────────────────┐  │ │
│  │  │ FOLLOWING_LEADER State                           │  │ │
│  │  │  • Reads sensor data                             │  │ │
│  │  │  • Gets V2V leader data                          │  │ │
│  │  │  • Calls longitudinal_controller.compute_throttle│  │ │
│  │  │  • Returns (throttle, steering)                  │  │ │
│  │  └──────────────────────────────────────────────────┘  │ │
│  └────────────────────────────────────────────────────────┘ │
│                                                              │
│  ┌────────────────────────────────────────────────────────┐ │
│  │ Platoon Controller                                     │ │
│  │  • Manages formation                                   │ │
│  │  • Tracks leader                                       │ │
│  │  • Provides V2V data                                   │ │
│  │  • Computes steering (pure pursuit)                    │ │
│  └────────────────────────────────────────────────────────┘ │
│                                                              │
│  ┌────────────────────────────────────────────────────────┐ │
│  │ Longitudinal Controller ← NEW MODULAR COMPONENT        │ │
│  │  • Computes throttle command                           │ │
│  │  • Implements control law (CACC/PI/Hybrid)             │ │
│  │  • Maintains internal state                            │ │
│  └────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────┘
```

## Quick Reference

**To use CACC:**
```python
self.controller_type = 'cacc'
```

**To use PI:**
```python
self.controller_type = 'pi'
```

**To use Hybrid:**
```python
self.controller_type = 'hybrid'
```

**To add your own controller:**
1. Inherit from `LongitudinalControllerBase`
2. Implement `compute_throttle()` and `reset()`
3. Add to `ControllerFactory.CONTROLLER_TYPES`

**That's it! Enjoy easy controller switching! 🚗💨**
