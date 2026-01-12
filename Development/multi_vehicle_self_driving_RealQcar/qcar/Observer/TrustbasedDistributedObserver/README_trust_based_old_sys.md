# 🚗 Fleet Framework: Distributed Observer with Trust-Based Adaptive Weights

> **A robust framework for cooperative vehicle state estimation in adversarial environments**

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://python.org)
[![NumPy](https://img.shields.io/badge/NumPy-Required-orange.svg)](https://numpy.org)
[![License](https://img.shields.io/badge/License-Research-green.svg)](LICENSE)

---

## 📋 Table of Contents

- [Introduction](#-introduction)
- [Key Features](#-key-features)
- [Overall Architecture](#-overall-architecture)
- [Module Documentation](#-module-documentation)
  - [Observer Module](#-observer-module)
  - [Trust Module](#-trust-module)
  - [Weight Module](#-weight-module)
- [Data Flow](#-data-flow)
- [Configuration](#-configuration)
- [Getting Started](#-getting-started)

---

## 🎯 Introduction

The **Fleet Framework** is a sophisticated distributed state estimation system designed for cooperative vehicle platoons operating in potentially adversarial environments. It implements a multi-layered approach combining:

1. **Distributed Observers** - For resilient state estimation across the vehicle fleet
2. **Trust Evaluation** - Using the TriP (Trust-based Intelligent Platoon) model
3. **Adaptive Weight Calculation** - Dynamically adjusting influence based on trust scores

This framework enables vehicles in a platoon to maintain accurate situational awareness even when some vehicles may be compromised, malfunctioning, or experiencing communication issues.

### 🎓 Research Context

This implementation supports research in:
- Cooperative Adaptive Cruise Control (CACC)
- Attack-resilient vehicle platooning
- Distributed consensus algorithms
- Trust management in vehicular networks

---

## ✨ Key Features

| Feature | Description |
|---------|-------------|
| **Distributed State Estimation** | Each vehicle estimates states of all other vehicles using consensus-based observers |
| **Multi-Component Trust Evaluation** | Velocity, distance, acceleration, heading, and communication quality assessment |
| **Adaptive Weight Distribution** | Trust-aware weight allocation with influence capping |
| **Attack Detection** | Anomaly monitoring with sliding window analysis |
| **Kalman Filter Support** | Optional Kalman filtering for local state estimation |
| **EMA Smoothing** | Temporal stability through exponential moving average |
| **Virtual Graph Topology** | Enhanced connectivity modeling for weight calculation |

---

## 🏗 Overall Architecture

The framework follows a modular architecture with three core components working in synergy:

```mermaid
graph TB
    subgraph "Vehicle i"
        subgraph "State Estimation"
            LO[Local Observer<br/>Kalman/Custom]
            DO[Distributed Observer<br/>Consensus-Based]
        end
        
        subgraph "Trust Evaluation"
            TE[TriPTrustModel]
            VS[Velocity Score]
            DS[Distance Score]
            AS[Acceleration Score]
            HS[Heading Score]
            CQ[Comm Quality]
        end
        
        subgraph "Weight Calculation"
            WM[WeightTrustModule]
            VG[Virtual Graph]
            AW[Adaptive Weights]
        end
        
        LO --> DO
        TE --> WM
        WM --> DO
        VS --> TE
        DS --> TE
        AS --> TE
        HS --> TE
        CQ --> TE
        VG --> WM
    end
    
    subgraph "Communication"
        CC[Center Communication]
        BC[Beacon Messages]
    end
    
    CC <--> DO
    BC --> TE
    
    style TE fill:#e1f5fe
    style WM fill:#fff3e0
    style DO fill:#e8f5e9
```

### Architecture Overview

```mermaid
graph LR
    subgraph "Input Layer"
        S1[Sensor Data]
        V2V[V2V Messages]
        GPS[GPS/Localization]
    end
    
    subgraph "Processing Layer"
        OBS[Observer Module]
        TRUST[Trust Module]
        WEIGHT[Weight Module]
    end
    
    subgraph "Output Layer"
        EST[State Estimates]
        CTRL[Controller Input]
        LOG[Data Logging]
    end
    
    S1 --> OBS
    V2V --> TRUST
    GPS --> OBS
    
    OBS <--> TRUST
    TRUST <--> WEIGHT
    WEIGHT --> OBS
    
    OBS --> EST
    EST --> CTRL
    OBS --> LOG
    
    style OBS fill:#4caf50,color:#fff
    style TRUST fill:#2196f3,color:#fff
    style WEIGHT fill:#ff9800,color:#fff
```

---

## 📦 Module Documentation

### 🔍 Observer Module

**Location:** `src/Observer/`

The Observer module implements distributed state estimation allowing each vehicle to estimate the states of all vehicles in the platoon.

#### Files

| File | Purpose |
|------|---------|
| [`observer.py`](src/Observer/observer.py) | Core distributed observer implementation |
| [`idm_control.py`](src/Observer/idm_control.py) | IDM-based control prediction |

#### Observer Class

The main `Observer` class provides:

```python
class Observer:
    def __init__(self, vehicle, veh_param, initial_global_state, initial_local_state)
    def local_observer(self, mesure_state, instant_index)
    def distributed_observer(self, instant_index, weights)
    def kalman_filter(self, mesure_state)
```

#### Core Concepts

##### Local Observer

Estimates the host vehicle's own state using either:
- **Kalman Filter** - Standard prediction-correction cycle
- **Custom Observer** - Gain-based estimation with configurable L-gain

```mermaid
flowchart LR
    M[Measurement] --> |Input| LO[Local Observer]
    LO --> |Estimate| LS[Local State]
    
    subgraph "Local Observer Types"
        KF[Kalman Filter]
        CO[Custom Observer]
        DI[Direct Measurement]
    end
    
    LO -.-> KF
    LO -.-> CO
    LO -.-> DI
```

##### Distributed Observer

Each vehicle maintains estimates of all vehicles using a consensus protocol:

$$\hat{x}_{i,j}^{+} = A \cdot \left( \hat{x}_{i,j} + \sum_{l \in \mathcal{N}_i} w_{il} (\hat{x}_{l,j} - \hat{x}_{i,j}) + w_{i0} (\bar{x}_j - \hat{x}_{i,j}) \right) + B \cdot u_j$$

Where:
- $\hat{x}_{i,j}$ = Vehicle i's estimate of vehicle j's state
- $w_{il}$ = Weight assigned to neighbor l's estimate
- $\bar{x}_j$ = Local measurement of vehicle j
- $A, B$ = System dynamics matrices

```mermaid
sequenceDiagram
    participant V1 as Vehicle 1
    participant V2 as Vehicle 2 (Host)
    participant V3 as Vehicle 3
    
    V1->>V2: Global estimate x̂₁
    V3->>V2: Global estimate x̂₃
    V2->>V2: Local estimate x̄₂
    
    Note over V2: Consensus Update
    V2->>V2: x̂₂,j = f(x̂₁,j, x̂₂,j, x̂₃,j, x̄ⱼ, weights)
```

---

### 🛡 Trust Module

**Location:** `src/Trust/`

The Trust module implements the **TriP (Trust-based Intelligent Platoon)** model for evaluating the trustworthiness of neighboring vehicles.

#### Files

| File | Purpose |
|------|---------|
| [`TriPTrustModel.py`](src/Trust/TriPTrustModel.py) | Main trust evaluation model |
| [`test_trust_model.py`](src/Trust/test_trust_model.py) | Unit tests for trust model |

#### Trust Evaluation Components

The trust score is computed from multiple independent evaluators:

```mermaid
graph TB
    subgraph "Trust Components"
        V[Velocity Score<br/>wv = 12.0]
        D[Distance Score<br/>wd = 2.0]
        A[Acceleration Score<br/>wa = 0.3]
        H[Heading Score<br/>wh = 1.0]
        B[Beacon Score<br/>Binary 0/1]
        Q[Quality Factor<br/>Age, Drop Rate,<br/>Covariance]
    end
    
    subgraph "Aggregation"
        LS[Local Trust Sample]
        GS[Global Trust Sample]
    end
    
    subgraph "Output"
        FS[Final Trust Score]
        FL[Attack Flags]
    end
    
    V --> LS
    D --> LS
    A --> LS
    H --> LS
    B --> LS
    Q --> LS
    
    LS --> FS
    GS --> FS
    
    FS --> FL
    
    style V fill:#e3f2fd
    style D fill:#e3f2fd
    style A fill:#e3f2fd
    style H fill:#e3f2fd
    style Q fill:#fff3e0
```

#### Trust Score Formulas

| Component | Formula | Description |
|-----------|---------|-------------|
| **Velocity** | $v_{score} = \max(1 - \frac{\|v_y - v_{ref}\|}{v_{ref}}, 0)^{w_v}$ | Compares reported velocity to reference |
| **Distance** | $d_{score} = \max(1 - \frac{\|d_y - d_{measured}\|}{d_{measured}}, 0)^{w_d}$ | Validates distance consistency |
| **Acceleration** | $a_{score} = \max(1 - \|d_{add} \cdot \Delta a\|, 0)^{w_a}$ | Checks acceleration dynamics |
| **Heading** | $h_{score} = \max(1 - \frac{\theta_{diff}}{\theta_{max}}, 0)$ | Validates heading from trajectory |
| **Quality** | $q = e^{-2 \cdot age} \cdot (1 - drop\_rate) \cdot \frac{1}{1 + 0.2 \cdot cov}$ | Communication quality penalty |

#### Trust Score Levels

The model maintains a 5-level trust rating vector with Dirichlet-based updates:

```mermaid
graph LR
    L1[Level 1<br/>Untrusted<br/>0.0-0.2] --> L2[Level 2<br/>Suspicious<br/>0.2-0.4]
    L2 --> L3[Level 3<br/>Neutral<br/>0.4-0.6]
    L3 --> L4[Level 4<br/>Trusted<br/>0.6-0.8]
    L4 --> L5[Level 5<br/>Highly Trusted<br/>0.8-1.0]
    
    style L1 fill:#f44336,color:#fff
    style L2 fill:#ff9800,color:#fff
    style L3 fill:#ffeb3b,color:#000
    style L4 fill:#8bc34a,color:#fff
    style L5 fill:#4caf50,color:#fff
```

#### Attack Detection Flags

The trust model sets diagnostic flags for anomaly detection:

| Flag | Condition | Interpretation |
|------|-----------|----------------|
| `flag_target_attk` | γ_local > 0.5 AND γ_cross < 0.5 | Target may be under attack |
| `flag_glob_est_check` | γ_local < 0.5 AND γ_cross > 0.5 | Global estimate needs verification |
| `flag_local_est_check` | local_trust_sample < 0.5 | Local measurements unreliable |

---

### ⚖️ Weight Module

**Location:** `src/Weight/`

The Weight module calculates adaptive consensus weights based on trust scores and network topology.

#### Files

| File | Purpose |
|------|---------|
| [`Weight_Trust_module.py`](src/Weight/Weight_Trust_module.py) | Trust-based weight calculation |

#### WeightTrustModule Class

```python
class WeightTrustModule:
    def __init__(self, graph, trust_threshold, kappa, vehicle_id)
    def get_trusted_neighbors(self, car_idx, trust_scores)
    def calculate_weights_trust_v2(self, vehicle_index, trust_scores, config)
    def generate_virtual_graph(self, graph, vehicle_index)
```

#### Weight Calculation Algorithm (v2)

The simplified trust-based weight algorithm:

```mermaid
flowchart TD
    A[Start] --> B[Get Trusted Neighbors<br/>trust > threshold]
    B --> C[Set Fixed Weights<br/>w0 = 0.3, w_self = 0.2]
    C --> D[Calculate Neighbor Budget<br/>1.0 - w0 - w_self = 0.5]
    D --> E{Trusted<br/>Neighbors?}
    E -->|Yes| F[Distribute Budget<br/>Proportional to Trust]
    E -->|No| G[All Budget to Self]
    F --> H[Apply Influence Cap<br/>max 40% per neighbor]
    G --> I[Normalize Weights<br/>Sum = 1.0]
    H --> I
    I --> J[Apply EMA Smoothing<br/>η = 0.15]
    J --> K[Return Weight Array]
    
    style A fill:#4caf50,color:#fff
    style K fill:#4caf50,color:#fff
```

#### Default Configuration

```python
DEFAULT_WEIGHT_CONFIG = {
    'w0_fixed': 0.3,          # Virtual node weight (local measurement)
    'w_self_base': 0.2,       # Self weight
    'w_cap': 0.4,             # Maximum weight per neighbor (40%)
    'kappa': 5,               # Maximum neighbors to consider
    'eta': 0.15,              # EMA smoothing factor
    'enable_smoothing': True  # Enable temporal smoothing
}
```

#### Virtual Graph Construction

The module creates a virtual graph by adding an extra node (node 0) representing the local measurement:

```mermaid
graph TD
    subgraph "Virtual Graph"
        V0[Node 0<br/>Virtual/Local]
        V1[Vehicle 1]
        V2[Vehicle 2<br/>Host]
        V3[Vehicle 3]
        V4[Vehicle 4]
    end
    
    V0 <--> V2
    V0 <--> V1
    V0 <--> V3
    V1 <--> V2
    V2 <--> V3
    V3 <--> V4
    
    style V0 fill:#e1f5fe,stroke:#0288d1
    style V2 fill:#c8e6c9,stroke:#388e3c
```

---

## 🔄 Data Flow

Complete data flow through the framework:

```mermaid
sequenceDiagram
    participant S as Sensors
    participant LO as Local Observer
    participant TE as Trust Evaluator
    participant WM as Weight Module
    participant DO as Distributed Observer
    participant C as Controller
    
    loop Every Timestep
        S->>LO: Raw measurements
        LO->>LO: Kalman/Observer update
        LO-->>TE: Local state estimate
        
        Note over TE: Evaluate all neighbors
        TE->>TE: Calculate v, d, a, h scores
        TE->>TE: Compute trust samples
        TE-->>WM: Trust scores array
        
        WM->>WM: Get trusted neighbors
        WM->>WM: Calculate adaptive weights
        WM-->>DO: Weight matrix
        
        DO->>DO: Consensus update
        DO->>DO: Predict-correct all states
        DO-->>C: Global state estimates
    end
```

---

## ⚙️ Configuration

### Observer Configuration

```yaml
# In scenarios_config
Local_observer_type: "kalman"  # Options: "kalman", "observer", "direct"
Is_noise_measurement: true
Use_predict_observer: true
predict_controller_type: "self"  # Options: "self", "true_other", "predict_other"
```

### Trust Configuration

```yaml
# Trust thresholds
trust_threshold: 0.5

# Monitoring options
Monitor_sudden_change: true
Dirichlet_type: "Dual"  # Options: "Single", "Dual"
```

### Weight Configuration

```python
config = {
    'w0_fixed': 0.3,
    'w_self_base': 0.2,
    'w_cap': 0.4,
    'kappa': 5,
    'eta': 0.15,
    'enable_smoothing': True
}
```

---

## 🚀 Getting Started

### Prerequisites

```bash
pip install numpy scipy
```

### Basic Usage

```python
from src.Observer.observer import Observer
from src.Trust.TriPTrustModel import TriPTrustModel
from src.Weight.Weight_Trust_module import WeightTrustModule

# Initialize components
trust_model = TriPTrustModel()
weight_module = WeightTrustModule(
    graph=adjacency_matrix,
    trust_threshold=0.5,
    kappa=5,
    vehicle_id=vehicle_id
)

# In main loop
for t in range(num_timesteps):
    # Update local observer
    observer.local_observer(measurement, t)
    
    # Evaluate trust for each neighbor
    trust_scores = []
    for neighbor in neighbors:
        score, *_ = trust_model.calculate_trust(
            host_vehicle, neighbor, leader,
            neighbors, is_nearby, t
        )
        trust_scores.append(score)
    
    # Calculate adaptive weights
    result = weight_module.calculate_weights_trust_v2(
        vehicle_index, trust_scores
    )
    weights = result['weights']
    
    # Update distributed observer
    observer.distributed_observer(t, weights)
```

---

## 📚 References

- [TriP Trust Model Paper](docs/)
- [Distributed Observer Design](docs/)
- [Cooperative Vehicle Platooning](docs/)

---

## 📝 License

This project is part of academic research. Please contact the authors for licensing information.

---

*Last updated: January 2026*
