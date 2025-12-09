# Controller Configuration System

## Overview

The controller configuration system provides a **centralized YAML-based configuration** for all longitudinal and lateral controllers. You can now change all controller parameters in **ONE place** instead of hardcoding them throughout the codebase.

## Quick Start

1. **Configuration File**: `controller_config.yaml`
   - Located in: `qcar/Controller/controller_config.yaml`
   - Template available: `controller_config_vehicle_main.yaml`

2. **Change Controller Type**: Edit the YAML file
   ```yaml
   longitudinal_controller_type: 'cacc'  # or 'pi', 'hybrid'
   lateral_controller_type: 'pure_pursuit'  # or 'stanley', 'lookahead', 'hybrid'
   ```

3. **Adjust Parameters**: Modify parameters under each controller section
   ```yaml
   cacc:
     s0: 1.5                    # Minimum spacing
     h: 0.5                     # Time headway
     K_spacing: 0.2             # Spacing gain
     # ... etc
   ```

4. **Run Your Vehicle**: The system automatically loads the config!

## How It Works

### Architecture

```
controller_config.yaml
        ↓
config_loader.py (ControllerConfig class)
        ↓
following_leader_state.py
        ↓
Creates controllers with YAML parameters
```

### Files

1. **`controller_config.yaml`** - Your configuration file
   - Defines which controllers to use
   - Sets all parameters for each controller
   - Includes quick preset configurations

2. **`config_loader.py`** - Configuration loader utility
   - Reads YAML file
   - Validates parameters
   - Provides clean interface to access config

3. **`following_leader_state.py`** - State machine (updated)
   - Loads config on initialization
   - Creates controllers using config parameters
   - Falls back to defaults if config unavailable

## Available Controllers

### Longitudinal (Speed Control)

| Type | Description | Use Case |
|------|-------------|----------|
| `cacc` | Cooperative Adaptive Cruise Control | Platoon following with spacing control |
| `pi` | PI Velocity Controller | Simple velocity tracking |
| `hybrid` | Hybrid (CACC + PI) | Switches based on leader availability |

### Lateral (Steering Control)

| Type | Description | Use Case |
|------|-------------|----------|
| `pure_pursuit` | Pure Pursuit | Simple, smooth following |
| `stanley` | Stanley Controller | Heading + cross-track error |
| `lookahead` | Extended Lookahead | Advanced curvature-aware control |
| `hybrid` | Hybrid Lateral | Switches based on distance |
| `path` | Path Following | Follow predefined waypoints |

## Configuration Examples

### Example 1: Aggressive Following
```yaml
longitudinal_controller_type: 'cacc'
lateral_controller_type: 'pure_pursuit'

cacc:
  s0: 1.0          # Tight spacing
  h: 0.3           # Short headway
  K_spacing: 0.3   # High spacing gain
  K_velocity: 0.08

pure_pursuit:
  lookahead_distance: 0.8
  k_steering: 1.2
```

### Example 2: Conservative/Safe
```yaml
longitudinal_controller_type: 'cacc'
lateral_controller_type: 'stanley'

cacc:
  s0: 2.0          # Large spacing
  h: 0.7           # Long headway
  K_spacing: 0.15  # Lower gain
  K_velocity: 0.03
```

### Example 3: Hybrid Adaptive
```yaml
longitudinal_controller_type: 'hybrid'
lateral_controller_type: 'hybrid'
# Uses params from sub-controller sections
```

## Usage in Code

### Automatic (Recommended)
The `following_leader_state.py` automatically loads the config. Just edit the YAML file!

### Manual (Advanced)
```python
from Controller.config_loader import get_controller_config
from Controller.longitudinal_controllers import ControllerFactory

# Load config
config = get_controller_config()

# Get controller type and params
controller_type = config.get_longitudinal_controller_type()
params = config.get_longitudinal_params()

# Create controller
controller = ControllerFactory.create(controller_type, params, logger)
```

## Parameter Reference

### Longitudinal - CACC
- `s0`: Minimum spacing (meters)
- `h`: Time headway (seconds)
- `K_spacing`: Spacing error gain
- `K_velocity`: Velocity error gain
- `acc_to_throttle_gain`: Acceleration to throttle conversion
- `max_throttle`: Maximum throttle output
- `alpha_filter`: Low-pass filter coefficient (0-1)
- `ki_velocity`: Velocity integral gain

### Longitudinal - PI
- `kp`: Proportional gain
- `ki`: Integral gain
- `max_throttle`: Maximum throttle output
- `ei_max`: Integral anti-windup limit

### Lateral - Pure Pursuit
- `lookahead_distance`: Base lookahead distance (meters)
- `k_steering`: Proportional steering gain
- `max_steering`: Maximum steering angle (radians)
- `adaptive_lookahead`: Enable adaptive lookahead (true/false)

### Lateral - Stanley
- `k_e`: Cross-track error gain
- `k_soft`: Softening gain (prevents division by zero)
- `max_steering`: Maximum steering angle (radians)

### Lateral - Lookahead
- `ri`: Desired inter-vehicle spacing (meters)
- `hi`: Time headway (seconds)
- `l_r`: Distance from CG to rear axle (meters)
- `l_f`: Distance from CG to front axle (meters)
- `k1`: Lateral control gain
- `k2`: Yaw rate control gain
- `max_steering`: Maximum steering angle (radians)

## Dependencies

The config system requires PyYAML:
```bash
pip install pyyaml
```

## Troubleshooting

### Config file not found
- Ensure `controller_config.yaml` exists in `qcar/Controller/`
- Copy from `controller_config_vehicle_main.yaml` if needed

### Invalid parameter values
- Check YAML syntax (proper indentation)
- Verify parameter names match exactly
- Check data types (numbers vs strings)

### Controller not switching
- Verify `longitudinal_controller_type` and `lateral_controller_type` are set
- Check spelling (case-sensitive)
- Restart your vehicle process after config changes

## Benefits

✅ **Single Source of Truth** - All parameters in one file  
✅ **Easy Experimentation** - Quickly test different controllers  
✅ **No Code Changes** - Adjust behavior without editing Python  
✅ **Version Control Friendly** - Track parameter changes in git  
✅ **Quick Presets** - Commented examples for common scenarios  
✅ **Type Safety** - Config loader validates parameters  

## Migration from Hardcoded Parameters

If you have old code with hardcoded parameters:

**Before (Old):**
```python
params = {
    's0': 1.5,
    'h': 0.5,
    # ... hardcoded values
}
controller = ControllerFactory.create('cacc', params)
```

**After (New):**
```python
config = get_controller_config()
params = config.get_longitudinal_params()
controller = ControllerFactory.create(config.get_longitudinal_controller_type(), params)
```

Or even simpler - just edit the YAML file, the state machine handles everything!
