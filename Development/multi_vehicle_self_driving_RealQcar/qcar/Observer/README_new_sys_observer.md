<p align="center">
  <h1 align="center"> Vehicle Observer System</h1>
  <p align="center">
    <strong>A modular, pluggable state estimation framework for multi-vehicle autonomous coordination</strong>
  </p>
  <p align="center">
    <a href="#key-features">Features</a> •
    <a href="#architecture">Architecture</a> •
    <a href="#quickstart">Quickstart</a> •
    <a href="#configuration">Configuration</a> •
    <a href="#api-reference">API</a>
  </p>
</p>

---

## Introduction

The **Vehicle Observer** is a sophisticated state estimation system designed for real-time multi-vehicle coordination. It provides a unified interface for both **local state estimation** (single vehicle) and **distributed fleet estimation** (multi-vehicle consensus).

Built with a **factory pattern** architecture, the system allows seamless switching between estimation algorithms without changing application code—from Extended Kalman Filters to Distributed Consensus estimators.

```
┌─────────────────────────────────────────────────────────────────┐
│                     Vehicle Observer System                      │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐        ┌─────────────────────────────────┐ │
│  │ Local Estimator │        │    Fleet Estimator (Distributed)│ │
│  │   • EKF         │        │      • Consensus                │ │
│  │   • Luenberger  │   ⟷    │      • Distributed Kalman       │ │
│  │   • Dead Reckon │        │      • Distributed Luenberger   │ │
│  └─────────────────┘        └─────────────────────────────────┘ │
│            ▲                              ▲                      │
│            │    Unified API               │                      │
│            └──────────┬───────────────────┘                      │
│                       ▼                                          │
│              ┌────────────────┐                                  │
│              │ VehicleObserver│  ← Manager & Coordinator         │
│              │    (Manager)   │                                  │
│              └────────────────┘                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Key Features

### 🔌 Pluggable Architecture
Swap between estimation algorithms at runtime using factory-based instantiation. No code changes required when switching from EKF to Luenberger or from Consensus to Distributed Kalman.

### 🚗 Local State Estimation
High-frequency (100 Hz) local state estimation using:
- **Extended Kalman Filter (EKF)** — Optimal fusion of GPS, IMU, and odometry
- **Luenberger Observer** — Deterministic observer with tunable gains
- **Dead Reckoning** — Fallback when GPS is unavailable

### 🚛 Distributed Fleet Estimation
Lower-frequency (50 Hz) consensus-based estimation across the vehicle fleet:
- **Consensus Estimator** — Simple, robust averaging with direct measurement fusion
- **Distributed Kalman** — Model-based prediction with consensus correction
- **Distributed Luenberger** — Longitudinal platoon estimation with vehicle dynamics

### 📡 V2V Integration
Seamless integration with Vehicle-to-Vehicle communication:
- Automatic fleet size discovery and expansion
- Timestamped state broadcasts (local + fleet)
- Age-based validity checking for received data

### 🧵 Thread-Safe Design
Full thread safety with `RLock` protection for all shared state access.

---


---

## State Vector Definition

The system uses a **5-dimensional state vector**:

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0 | `x` | Position X | meters |
| 1 | `y` | Position Y | meters |
| 2 | `θ` | Heading angle | radians |
| 3 | `v` | Velocity | m/s |
| 4 | `a` | Acceleration | m/s² |

> [!NOTE]
> Some estimators (legacy) use a 4D state without acceleration. The manager automatically handles dimension conversion.

---

## Configuration

### Local Estimator Configuration

**File:** `config_local_estimators.yaml`

### Fleet Estimator Configuration

**File:** `config_fleet_estimators.yaml`

---



## API Reference

### VehicleObserver (Manager Class)

| Method | Description | Returns |
|--------|-------------|---------|
| `update_sensor_data(qcar)` | Read all sensors from QCar | `bool` |
| `update_observer(dt, steering, throttle)` | Run estimation cycle | `dict` |
| `get_local_state()` | Get local state as numpy array | `np.ndarray [5]` |
| `get_fleet_states()` | Get all fleet states | `np.ndarray [5×N]` |
| `get_vehicle_state(id)` | Get specific vehicle state | `np.ndarray [5]` |
| `get_local_state_for_broadcast()` | Format for V2V local broadcast | `dict` |
| `get_fleet_state_for_broadcast()` | Format for V2V fleet broadcast | `dict` |
| `add_received_local_state(...)` | Process incoming local state | `bool` |
| `add_received_fleet_state(...)` | Process incoming fleet state | `bool` |
| `reinitialize_fleet_estimation(...)` | Reconfigure when V2V activates | `None` |
| `reset_observer(initial_pose)` | Reset all state | `None` |

---

## Performance Characteristics

| Metric | Local Estimator | Fleet Estimator |
|--------|-----------------|-----------------|
| **Update Rate** | 100 Hz | 50 Hz |
| **Latency** | < 1 ms | < 5 ms |
| **Memory** | ~1 KB per vehicle | ~5 KB per fleet |
| **State Age Validity** | — | 1.0 second |

### Auto-Expansion

The fleet estimator automatically expands capacity when new vehicles are discovered:

```python
# When vehicle_id >= current_fleet_size:
#   1. Expand fleet_states array
#   2. Update weight arrays (for Distributed Kalman)
#   3. Log the expansion
```

---


---

## Related Documentation

- [Architecture Diagram](../ARCHITECTURE_DIAGRAM.md) — Full system overview
- [V2V Communication](../V2V/README.md) — Vehicle-to-Vehicle integration
- [Controller](../Controller/README.md) — Control system using observer data

---

<p align="center">
  <sub>Built with ❤️ for the QCar Multi-Vehicle Research Project</sub>
</p>
