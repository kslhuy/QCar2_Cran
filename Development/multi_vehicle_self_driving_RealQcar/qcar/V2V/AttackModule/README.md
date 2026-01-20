# V2V Attack Module System

**Fault Injection Framework for Vehicle-to-Vehicle Communication Security Research**

---

## Overview

The V2V Attack Module System provides a comprehensive framework for simulating communication attacks within autonomous vehicle fleet networks. It integrates directly with the V2V communication system (`V2VManager` and `V2VCommunication`) to inject faults into vehicle-to-vehicle broadcasts for security testing and research.

### Key Capabilities

- **Transparent V2V Integration**: Drop-in middleware that wraps `V2VManager`
- **Multiple Attack Types**: Bogus, DoS, Position, Velocity, Acceleration, Collusion
- **7 Modification Types**: Scaling, Bias, Linear drift, Sinusoidal, Faulty/Noise, Zero, Constant
- **Time-Based Scheduling**: Precise attack timing with start/end timestamps
- **Dual Data Targeting**: Attacks can target local broadcasts, fleet estimates, or both
- **YAML Configuration**: Define scenarios without code changes
- **Detailed Logging**: Comprehensive attack event logging for analysis

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Vehicle Process                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌─────────────┐    ┌─────────────────┐    ┌────────────────┐   │
│  │   Vehicle   │───▶│  V2VAttack      │───▶│ V2VManager     │   │
│  │  Observer   │    │  Injector       │    │                │   │
│  └─────────────┘    │  (middleware)   │    │ ┌────────────┐ │   │
│         │           │                 │    │ │ V2VComm    │ │   │
│         │           │ ┌─────────────┐ │    │ │            │ │   │
│         │           │ │ Attack      │ │    │ └────────────┘ │   │
│         ▼           │ │ Module      │ │    └───────┬────────┘   │
│  get_local_state    │ └─────────────┘ │            │            │
│  for_broadcast()    └─────────────────┘            │            │
│                              │                     ▼            │
│                              │               UDP Broadcast      │
│                              ▼               to Peer Vehicles   │
│                    ┌─────────────────┐                          │
│                    │ attack_config   │                          │
│                    │     .yaml       │                          │
│                    └─────────────────┘                          │
└─────────────────────────────────────────────────────────────────┘
```

---

## Quick Start

### 1. Basic Integration

Replace `V2VManager` with `V2VAttackInjector` in your vehicle initialization:

```python
from V2V.v2v_manager import V2VManager, V2VBroadcastConfig
from V2V.AttackModule import V2VAttackInjector

# Create standard V2VManager
v2v_manager = V2VManager(
    vehicle_id=vehicle_id,
    vehicle_logger=logger,
    config=V2VBroadcastConfig(),
    vehicle_observer=observer
)

# Wrap with attack injector (loads scenarios from config)
attack_injector = V2VAttackInjector(
    v2v_manager=v2v_manager,
    attack_config_path="V2V/AttackModule/attack_config.yaml",
    enabled=True
)

# Use attack_injector instead of v2v_manager
attack_injector.activate(peer_vehicles, peer_ips)

# In main loop
while running:
    attack_injector.update_broadcast()  # Attacks injected automatically
```

### 2. Programmatic Attack Creation

```python
from V2V.AttackModule import AttackModule
from V2V.AttackModule.AttackScenarios import atk_scenarios, make_scenario

# Create attack module
attack_module = AttackModule(vehicle_id=1, logger=my_logger)

# Add predefined scenario (MATLAB-compatible API)
atk_scenarios(
    attack_module,
    attack_type="Bogus",
    data_type="local", 
    case_number=2,      # Velocity scaling by 0.5
    t_start=10.0,
    t_end=20.0,
    attacker_id=1,
    victim_id=-1        # All vehicles
)

# Or create custom scenario
custom_scenario = make_scenario(
    attacker_id=1,
    victim_ids=[-1],
    t_start=15.0,
    t_end=25.0,
    modification_type='sinusoidal',
    intensity={'amplitude': 3.0, 'frequency': 0.5, 'phase': 0.0},
    data_type='local',
    target_fields=['x', 'y'],
    attack_type='Bogus',
    scenario_name='PositionOscillation'
)
attack_module.add_scenario(custom_scenario)
```

---

## Module Components

### 1. AttackModule.py

Core attack engine that manages scenarios and applies modifications.

**Key Classes:**
- `AttackType`: Enum for attack types (Bogus, DoS, POS, VEL, ACC, Collusion)
- `ModificationType`: Enum for modifications (scaling, bias, linear, sinusoidal, faulty, zero, constant)
- `DataType`: Enum for target data (local, fleet, both, heartbeat)
- `AttackScenario`: Dataclass defining an attack with timing, targets, and parameters
- `AttackModule`: Main class for managing and applying attacks

**Key Methods:**
```python
attack_module.update(elapsed_time)           # Update attack timing
attack_module.should_attack_local_data()     # Check if local attack active
attack_module.should_attack_fleet_data()     # Check if fleet attack active
attack_module.apply_attack_to_local_state(state)   # Apply attack to local broadcast
attack_module.apply_attack_to_fleet_state(state)   # Apply attack to fleet broadcast
attack_module.get_attack_status()            # Get current status and stats
```

### 2. V2VAttackInjector.py

Middleware that wraps V2VManager for transparent attack injection.

**Features:**
- Full V2VManager API compatibility (drop-in replacement)
- Automatic attack timing based on elapsed time
- Intercepts and modifies broadcasts before sending
- Detailed statistics tracking

**Key Methods:**
```python
injector.update_broadcast()      # Main method (replaces V2VManager's)
injector.enable_attacks()        # Enable attack injection
injector.disable_attacks()       # Disable attack injection
injector.is_attack_active()      # Check if attack currently active
injector.get_attack_status()     # Get comprehensive status
injector.add_attack_scenario(scenario)  # Add new scenario at runtime
```

### 3. AttackScenarios.py

Factory functions for creating attack scenarios.

**Predefined Scenario Functions:**
| Function | Cases | Description |
|----------|-------|-------------|
| `create_bogus_scenarios(attack_module, case)` | 1-10 | Bogus message attacks |
| `create_dos_scenarios(attack_module, case)` | 1-3 | Denial of Service |
| `create_position_attack(attack_module, case)` | 1-3 | GPS spoofing simulation |
| `create_velocity_attack(attack_module, case)` | 1-3 | Speedometer tampering |
| `create_acceleration_attack(attack_module, case)` | 1-3 | IMU manipulation |
| `create_heading_attack(attack_module, case)` | 1-3 | Compass spoofing |

**Helper Functions:**
```python
make_scenario(...)                    # Create custom scenario
atk_scenarios(attack_module, ...)     # MATLAB-compatible dispatcher
load_scenarios_from_config(attack_module, path)  # Load from YAML
create_multi_vehicle_attack(...)      # Create collusion attack
```

### 4. attack_config.yaml

YAML configuration file for defining attack scenarios.

---

## Attack Types Reference

### Bogus Attacks (10 Cases)

| Case | Modification | Target | Intensity | Description |
|------|-------------|--------|-----------|-------------|
| 1 | Scaling | velocity | -0.5 | Reverse and reduce velocity |
| 2 | Scaling | velocity | 0.5 | Halve velocity |
| 3 | Bias | x | -15.0 | Shift X position -15m |
| 4 | Bias | x | 33.0 | Shift X position +33m |
| 5 | Linear | velocity | 0.1 | Velocity drifts 0.1 m/s² |
| 6 | Sinusoidal | x | 5m@0.5Hz | Position oscillates |
| 7 | Faulty | x | σ=10m | Position noise injection |
| 8 | Faulty | velocity | σ=2.5m/s | Velocity noise injection |
| 9 | Faulty | acceleration | σ=0.2m/s² | Acceleration noise |
| 10 | Bias | velocity | +2.0 | Add 2 m/s to velocity |

### DoS Attacks (3 Cases)

| Case | Target | Description |
|------|--------|-------------|
| 1 | velocity | Set velocity to zero |
| 2 | x, y | Set position to zero |
| 3 | x, y, velocity | Set all to zero |

### Modification Types

| Type | Formula | Example |
|------|---------|---------|
| `scaling` | `value × factor` | velocity × 0.5 |
| `bias` | `value + offset` | x + 10.0 |
| `linear` | `value + rate × time` | velocity + 0.1 × t |
| `sinusoidal` | `value + A × sin(2πft + φ)` | x + 5×sin(πt) |
| `faulty` | `value + N(0, σ)` | x + noise |
| `zero` | `0` | Set to zero |
| `constant` | `C` | Fixed value |
| `random` | `uniform(min, max)` | Random in range |

---

## Configuration File Format

```yaml
attack_settings:
  enabled: true           # Master switch
  dt: 0.01                # Timestep
  log_level: "WARNING"    # Log verbosity
  log_interval: 1.0       # Log throttle interval

scenarios:
  - name: "Scenario_Name"
    enabled: true/false
    attack_type: "Bogus"              # Bogus, DoS, POS, VEL, ACC, Collusion
    modification_type: "scaling"       # scaling, bias, linear, sinusoidal, faulty, zero, constant
    data_type: "local"                 # local, fleet, both
    t_start: 10.0
    t_end: 20.0
    attacker_id: 1
    victim_ids: [-1]                   # -1 = all vehicles
    intensity: 0.5                     # or dict for sinusoidal
    target_fields: ["velocity"]
    description: "Human-readable description"
```

### Sinusoidal Intensity Format

```yaml
intensity:
  amplitude: 5.0     # Oscillation amplitude
  frequency: 0.5     # Frequency in Hz
  phase: 0.0         # Phase offset in radians
```

---

## Data Structures

### Local State (Input to `apply_attack_to_local_state`)

```python
{
    'vehicle_id': 1,
    'x': 5.234,           # Position X (meters)
    'y': 3.125,           # Position Y (meters)
    'theta': 1.57,        # Heading (radians)
    'velocity': 2.5,      # Velocity (m/s)
    'acceleration': 0.1,  # Acceleration (m/s²) - optional
    'source': 'local_sensors'
}
```

### Fleet State (Input to `apply_attack_to_fleet_state`)

```python
{
    'sender_id': 1,
    'fleet_states': {
        0: {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'velocity': 2.0, 'confidence': 0.95},
        1: {'x': 5.2, 'y': 3.1, 'theta': 1.57, 'velocity': 2.5, 'confidence': 0.90},
        2: {'x': 10.4, 'y': 6.2, 'theta': 1.57, 'velocity': 2.3, 'confidence': 0.85},
    },
    'source': 'fleet_consensus'
}
```

---

## Logging

Attack events are logged with the following format:

```
======================================================================
🔴 LOCAL STATE ATTACK - Vehicle 1 at t=10.25s
======================================================================
  📍 Position X: 5.2340 → 5.2340 (Δ +0.0000)
  📍 Position Y: 3.1250 → 3.1250 (Δ +0.0000)
  🏎️  Velocity: 2.5000 → 1.2500 (Δ -1.2500)
  Scenario: Bogus_Case2_VelScalingPos [scaling]
======================================================================
```

Logs are written to the vehicle's logger at WARNING level.

---

## Example: Complete Integration

```python
# vehicle_logic.py

from V2V.v2v_manager import V2VManager, V2VBroadcastConfig
from V2V.AttackModule.V2VAttackInjector import V2VAttackInjector
from V2V.AttackModule.AttackModule import AttackModule
from V2V.AttackModule.AttackScenarios import atk_scenarios

class VehicleLogic:
    def __init__(self, vehicle_id, logger, observer):
        self.vehicle_id = vehicle_id
        
        # Create V2VManager
        self.v2v_manager = V2VManager(
            vehicle_id=vehicle_id,
            vehicle_logger=logger,
            config=V2VBroadcastConfig(),
            vehicle_observer=observer
        )
        
        # Determine if this vehicle is an attacker
        if vehicle_id == 1:  # Vehicle 1 is the attacker
            # Create attack injector with config
            self.v2v = V2VAttackInjector(
                v2v_manager=self.v2v_manager,
                attack_config_path="V2V/AttackModule/attack_config.yaml",
                enabled=True
            )
        else:
            # Non-attackers use standard V2VManager
            self.v2v = self.v2v_manager
    
    def start_communication(self, peers, peer_ips):
        self.v2v.activate(peers, peer_ips)
        self.start_time = time.time()
    
    def update(self):
        # This will inject attacks if this is the attacker
        self.v2v.update_broadcast()
        
        # Check attack status
        if hasattr(self.v2v, 'is_attack_active'):
            if self.v2v.is_attack_active():
                print("⚠️ Attack currently active!")
```

---

## Testing

### Unit Test Example

```python
import unittest
from V2V.AttackModule.AttackModule import AttackModule
from V2V.AttackModule.AttackScenarios import create_bogus_scenarios

class TestAttackModule(unittest.TestCase):
    
    def test_velocity_scaling_attack(self):
        attack_module = AttackModule(vehicle_id=1)
        create_bogus_scenarios(attack_module, case_number=2,
                              t_start=10.0, t_end=20.0)
        
        # Simulate time at 15 seconds (attack active)
        attack_module.update(15.0)
        
        self.assertTrue(attack_module.is_attack_active())
        self.assertTrue(attack_module.should_attack_local_data())
        
        # Apply attack
        original = {'x': 0, 'y': 0, 'theta': 0, 'velocity': 4.0}
        modified = attack_module.apply_attack_to_local_state(original)
        
        # Velocity should be halved
        self.assertEqual(modified['velocity'], 2.0)
```

---

## Security Notice

> ⚠️ **WARNING**: This module is designed for **RESEARCH AND TESTING PURPOSES ONLY**.
> 
> - Always disable attacks in production (`enabled: false`)
> - Never deploy attack capabilities on public roads
> - Use only in controlled simulation environments
> - Follow ethical guidelines for security research

---

## File Structure

```
V2V/AttackModule/
├── __init__.py           # Package initializer
├── AttackModule.py       # Core attack engine
├── AttackScenarios.py    # Scenario factory functions
├── V2VAttackInjector.py  # V2VManager middleware
├── attack_config.yaml    # Attack scenario configuration
├── README.md             # This documentation
└── olds/                 # Previous implementation (reference)
    ├── AttackModule.py
    ├── AttackScenarios.py
    └── attack_config.yaml
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 2.0.0 | January 2026 | Complete rewrite for V2V integration |
| 1.0.0 | Original | MATLAB-based implementation |

---

*Fleet Framework Security Research - January 2026*
