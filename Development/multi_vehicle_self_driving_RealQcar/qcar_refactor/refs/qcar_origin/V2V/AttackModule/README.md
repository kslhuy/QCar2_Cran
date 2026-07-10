# V2V Attack Module
# Overview

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


This module injects false data into the existing V2V broadcast path.

In this project, the attack is applied here:

`VehicleObserver -> get_local_state_for_broadcast() / get_fleet_state_for_broadcast() -> V2VAttackInjector -> V2VManager -> UDP send`

The practical use is simple:

- `data_type: local` attacks the vehicle's own broadcasted local state.
- `data_type: fleet` attacks the broadcasted fleet/global state estimate.
- `data_type: both` attacks both messages.


## What "global state estimation" means here

In this codebase, global state estimation is the fleet state built by the observer and returned by:

- `Observer/VehicleObserverSimple.py -> get_fleet_state_for_broadcast()`

That message contains `fleet_states[vehicle_id]` for all vehicles.  
A fleet attack modifies that structure before it is sent to the other cars.


## How to apply the attack to the current system

By default, [`vehicle_logic.py`](/c:/Users/Quang%20Huy%20Nugyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar/vehicle_logic.py) creates a normal `V2VManager`.  
To enable attacks, wrap it with `V2VAttackInjector`.

### 1. Import the injector

Add:

```python
from V2V.AttackModule.V2VAttackInjector import V2VAttackInjector
```

### 2. Wrap `self.v2v_manager`

After the normal `V2VManager(...)` is created, replace it with:

```python
self.v2v_manager = V2VAttackInjector(
    v2v_manager=self.v2v_manager,
    attack_config_path=os.path.join(
        os.path.dirname(__file__),
        "V2V",
        "AttackModule",
        "attack_config.yaml",
    ),
    enabled=(self.vehicle_id == 1),  # example: vehicle 1 is the attacker
)
```

This works with the current code because:

- `_broadcast_v2v_state()` already calls `self.v2v_manager.update_broadcast()`
- `V2VAttackInjector` exposes the same main methods as `V2VManager`
- `self.v2v_manager.update_vehicle_observer(self.vehicle_observer)` still works through passthrough

### 3. Enable one scenario in `attack_config.yaml`

File:

- [`attack_config.yaml`](/c:/Users/Quang%20Huy%20Nugyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar/V2V/AttackModule/attack_config.yaml)

Keep only the scenario you want as `enabled: true` for a clean test.


## Minimal YAML fields you need

```yaml
attack_settings:
  enabled: true

scenarios:
  - name: "My_Attack"
    enabled: true
    attack_type: "Bogus"
    modification_type: "bias"
    data_type: "local"      # local, fleet, both
    t_start: 10.0
    t_end: 20.0
    attacker_id: 1
    victim_ids: [-1]
    intensity: 2.0
    target_fields: ["velocity"]
    description: "Example attack"
```

Important:

- `attacker_id` is the vehicle that sends the corrupted message.
- For `fleet` attacks, `victim_ids` refers to the vehicle entries inside `fleet_states`.
- `-1` means all vehicles.


## Example: attack on global state estimation

This is the main example for your current system.

Goal:

- Vehicle `1` is malicious.
- From `10s` to `20s`, it sends a corrupted fleet/global estimate.
- In that fleet message, the estimate of vehicle `2` is shifted by `+5 m` on `x` and `y`.
- Other vehicles then receive a wrong global/fleet state.

Use this scenario:

```yaml
attack_settings:
  enabled: true

scenarios:
  - name: "Global_State_Position_Bias_On_V2"
    enabled: true
    attack_type: "Bogus"
    modification_type: "bias"
    data_type: "fleet"
    t_start: 10.0
    t_end: 20.0
    attacker_id: 1
    victim_ids: [2]
    intensity: 5.0
    target_fields: ["x", "y"]
    description: "Vehicle 1 sends a biased fleet estimate for vehicle 2"
```

How it behaves in code:

- `VehicleObserver.get_fleet_state_for_broadcast()` builds the normal fleet estimate.
- `V2VAttackInjector._broadcast_fleet_state_with_attack()` intercepts it.
- `AttackModule.apply_attack_to_fleet_state()` modifies `fleet_states[2]["x"]` and `fleet_states[2]["y"]`.
- The modified fleet packet is sent as `message_type="fleet_state"`.

This is the correct way to test a global-state-estimation attack in this repository.


## Example: attack everyone in the global estimate

If you want vehicle `1` to corrupt the whole fleet message:

```yaml
- name: "Global_State_All_Vehicles_Velocity_Scaling"
  enabled: true
  attack_type: "Bogus"
  modification_type: "scaling"
  data_type: "fleet"
  t_start: 10.0
  t_end: 20.0
  attacker_id: 1
  victim_ids: [-1]
  intensity: 0.6
  target_fields: ["velocity"]
  description: "Vehicle 1 reduces all fleet velocities by 40 percent"
```


## Fields you can attack

For `local` data, the useful fields are:

- `x`
- `y`
- `theta`
- `velocity`
- `acceleration`

For `fleet` data, the implemented fields are:

- `x`
- `y`
- `theta`
- `velocity`
- `confidence`


## Common attack modes

- `scaling`: multiply the value
- `bias`: add a constant offset
- `linear`: drift over time
- `sinusoidal`: oscillating corruption
- `faulty`: Gaussian noise
- `zero`: set to zero
- `constant`: force one constant value


## Quick verification

When the attack is active, the logger prints messages such as:

- `LOCAL STATE ATTACK`
- `FLEET STATE ATTACK`

For a fleet/global attack, check that:

- the attacker vehicle is the one with `attacker_id`
- the active time is inside `[t_start, t_end]`
- the modified values appear in the fleet attack log


## Practical notes

- The simplest setup is to wrap only the attacker vehicle with `V2VAttackInjector`.
- You can also wrap all vehicles and keep one `attacker_id` in YAML.
- For repeatable experiments, enable one scenario at a time.
- This module changes outgoing messages only. It does not alter incoming-message handling.


## Main files

- [`AttackModule.py`](/c:/Users/Quang%20Huy%20Nugyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar/V2V/AttackModule/AttackModule.py)
- [`AttackScenarios.py`](/c:/Users/Quang%20Huy%20Nugyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar/V2V/AttackModule/AttackScenarios.py)
- [`V2VAttackInjector.py`](/c:/Users/Quang%20Huy%20Nugyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar/V2V/AttackModule/V2VAttackInjector.py)
- [`attack_config.yaml`](/c:/Users/Quang%20Huy%20Nugyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar/V2V/AttackModule/attack_config.yaml)
