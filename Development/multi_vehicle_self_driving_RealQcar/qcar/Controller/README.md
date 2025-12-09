# Vehicle Controllers

Modular control systems for QCar vehicles supporting waypoint tracking and platoon formation.

## Core Files

- `controllers.py` - Basic speed (PI) and steering (Stanley) controllers
- `lateral_controllers.py` - Lateral control strategies (Pure Pursuit, Stanley, LQR)
- `longitudinal_controllers.py` - Longitudinal control strategies (PI, CACC, IDM)
- `platoon_controller.py` - Platoon formation and coordination logic
- `idm_control.py` - Intelligent Driver Model implementation
- `CACC.py` - Cooperative Adaptive Cruise Control implementation

## Controller Types

### Lateral Controllers
Control steering for path following:
- **Pure Pursuit** - Tracks a lookahead point ahead of the leader
- **Stanley** - Path tracking using cross-track and heading error
- **LQR** - Optimal control for smooth trajectory tracking

### Longitudinal Controllers
Control throttle for speed/spacing:
- **PI Velocity** - Simple speed tracking with PI feedback
- **CACC** - Cooperative spacing control for platoons (spacing + velocity error)
- **IDM** - Intelligent Driver Model for natural car-following behavior
- **Hybrid** - Auto-switches between CACC (with leader) and PI (without leader)

### Platoon Controller
Manages multi-vehicle formation:
- Formation states (IDLE, SEARCHING, FORMING, ACTIVE, LOST)
- Automatic spacing control between vehicles
- Leader detection and following
- Safe distance maintenance

## Quick Start

**Switch Controllers:**
```python
# In your vehicle configuration
longitudinal_controller_type = 'cacc'  # Options: 'pi', 'cacc', 'idm', 'hybrid'
lateral_controller_type = 'pure_pursuit'  # Options: 'pure_pursuit', 'stanley', 'lqr'
```

**Basic Usage:**
```python
from Controller.longitudinal_controllers import ControllerFactory
from Controller.lateral_controllers import PurePursuitController

# Create controllers
throttle_ctrl = ControllerFactory.create('cacc', params, logger)
steering_ctrl = PurePursuitController(lookahead_distance=1.0)

# Compute control commands
throttle = throttle_ctrl.compute_throttle(follower_state, leader_state, dt)
steering = steering_ctrl.compute_steering(follower_state, leader_state, dt)
```

## Controller Parameters

**CACC (Platoon Following):**
```python
s0 = 1.5              # Minimum spacing (m)
h = 0.5               # Time headway (s)
K = [[0.2, 0.05]]     # [spacing_gain, velocity_gain]
max_throttle = 0.3
```

**Pure Pursuit (Lateral):**
```python
lookahead_distance = 1.0  # Lookahead distance (m)
k_steering = 1.0          # Steering gain
max_steering = 0.55       # Max steering angle (rad)
```

**PI Velocity:**
```python
kp = 0.1              # Proportional gain
ki = 1.0              # Integral gain
max_throttle = 0.3
```

## Documentation

- `docs/ARCHITECTURE.md` - System architecture and data flow
- `docs/README_CONTROLLERS.md` - Detailed controller documentation
- `docs/QUICK_START.txt` - Quick start guide
- `docs/IMPLEMENTATION_SUMMARY.md` - Implementation details