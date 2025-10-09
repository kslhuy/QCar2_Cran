# Visual Summary: Trust-Based Weight Design

**Quick visual guide to understand the complete system**

---

## System Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                     VEHICLE i (Host)                            │
├────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │  INPUTS: Received Data                                   │  │
│  ├─────────────────────────────────────────────────────────┤  │
│  │  From Vehicle j (target):                                │  │
│  │    • Local state: [x, y, θ, v]                          │  │
│  │    • Covariance: trace(P_j)                             │  │
│  │    • Timestamp                                           │  │
│  │                                                           │  │
│  │  From Vehicle k (neighbor):                              │  │
│  │    • Fleet estimate of j: x̂_k→j                        │  │
│  │    • Timestamp                                           │  │
│  │                                                           │  │
│  │  From Trust Evaluator:                                   │  │
│  │    • trust_local[j]                                      │  │
│  │    • trust_global[j]                                     │  │
│  └─────────────────────────────────────────────────────────┘  │
│                           ↓                                     │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │  QUALITY TRACKING                                        │  │
│  ├─────────────────────────────────────────────────────────┤  │
│  │  • Message age: Δt = now - timestamp                    │  │
│  │  • Innovation: ε = ||received - predicted||              │  │
│  │  • Drop rate: drops / expected_msgs                      │  │
│  │  • Distance: d = ||pos_i - pos_j||                       │  │
│  └─────────────────────────────────────────────────────────┘  │
│                           ↓                                     │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │  WEIGHT CALCULATION (3 stages)                          │  │
│  ├─────────────────────────────────────────────────────────┤  │
│  │  Stage 1: GATE                                           │  │
│  │    Hard: Age > 0.5s? Innovation > 5m? → reject          │  │
│  │    Soft: Hysteresis taper for trust near threshold      │  │
│  │                                                           │  │
│  │  Stage 2: SCORE                                          │  │
│  │    s[j] = trust^p * age_decay * reliability * ...       │  │
│  │                                                           │  │
│  │  Stage 3: NORMALIZE                                      │  │
│  │    w_0 = adaptive (connectivity + quality)               │  │
│  │    w_self = confidence-based                             │  │
│  │    w_neighbors = proportional to scores                  │  │
│  │    Ensure: Σ w = 1, all w ≥ 0                           │  │
│  └─────────────────────────────────────────────────────────┘  │
│                           ↓                                     │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │  OUTPUT: Weight Vector W                                │  │
│  ├─────────────────────────────────────────────────────────┤  │
│  │  W = [w_0, w_1, ..., w_N]                               │  │
│  │                                                           │  │
│  │  w_0: Virtual (local observer estimate)                  │  │
│  │  w_{i+1}: Self (our previous estimate)                   │  │
│  │  w_{k+1}: Neighbor k's estimate                          │  │
│  └─────────────────────────────────────────────────────────┘  │
│                           ↓                                     │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │  DISTRIBUTED OBSERVER UPDATE                             │  │
│  ├─────────────────────────────────────────────────────────┤  │
│  │  x̂_i→j(t+1) = w_0·x_local + w_self·x̂_i→j(t) +          │  │
│  │                 Σ_k w_k·x̂_k→j                           │  │
│  └─────────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────┘
```

---

## Virtual Node: What It Really Means

```
┌────────────────────────────────────────────────────────────────┐
│  Vehicle i estimating Vehicle j                                 │
└────────────────────────────────────────────────────────────────┘

Case 1: j = i (Self)
─────────────────────
     ┌───────┐
     │ GPS   │ ──┐
     │ IMU   │   ├──→ w_0 (Virtual) → High (0.7)
     │ Tach  │ ──┘
     └───────┘
  (True sensors)

Case 2: j is connected neighbor (Graph[i,j] = 1)
────────────────────────────────────────────────
     ┌──────────────────┐
     │ Network          │
     │ Receives j's     │ ──→ w_0 (Virtual) → Medium (0.3-0.7)
     │ local state      │     (depends on quality)
     └──────────────────┘
  (j broadcasts position)

Case 3: j is NOT connected (Graph[i,j] = 0)
────────────────────────────────────────────
     ┌──────────────────┐
     │ No direct data   │
     │ from j           │ ──→ w_0 (Virtual) → Low (0.1)
     └──────────────────┘
  (Must rely on neighbors)

KEY INSIGHT:
  Virtual node (w_0) = "Local processing" vs "Fleet consensus"
                    ≠ "Camera/LiDAR sensor"
```

---

## Weight Allocation Example

```
Scenario: 3-vehicle fleet, V1 estimating V0

Fleet Graph:      V0 ←→ V1 ←→ V2
                  (V0-V1 connected, V1-V2 connected, V0-V2 NOT connected)

V1's view of V0:
  • V0 is directly connected ✓
  • Receives V0's local state broadcasts
  • trust_local[V0] = 0.8 (high)
  • trust_global[V0] = 0.7
  • message_age = 0.05s (fresh)
  • drop_rate = 0.02 (very low)
  • self_covariance = 0.5 (moderate uncertainty)

Weight Calculation:
─────────────────────

Stage 1: Gate
  ✓ Age < 0.5s
  ✓ Innovation < 5m
  ✓ Trust > tau_low
  → V0 passes all gates

Stage 2: Score
  s[V0] = 0.8^1.5 * 0.7^1.0 * exp(-2*0.05) * (1/(1+0.02)) * ...
        ≈ 0.72 * 0.7 * 0.90 * 0.98
        ≈ 0.44

  s[V2] = 0.6^1.5 * 0.5^1.0 * exp(-2*0.15) * (1/(1+0.10)) * ...
        ≈ 0.46 * 0.5 * 0.74 * 0.91
        ≈ 0.15

Stage 3: Normalize

  Step 1: Virtual weight
    Connected to V0 ✓, high quality
    w0_base = 0.3 + 0.4 * (0.8 * 0.90) = 0.59
    Adjust for self uncertainty:
    w0 = 0.59 / (1 + 0.2*0.5) = 0.54

  Step 2: Self weight
    w_self = 0.1 + 0.3 / (1 + 0.5*0.5) = 0.34
    Clamp: w_self ≤ 1 - 0.54 = 0.46 ✓

  Step 3: Neighbor mass
    neighbor_mass = 1 - 0.54 - 0.34 = 0.12

  Step 4: Distribute to neighbors
    S_total = s[V0] + s[V2] = 0.44 + 0.15 = 0.59
    w[V0] = 0.12 * (0.44 / 0.59) = 0.09
    w[V2] = 0.12 * (0.15 / 0.59) = 0.03

Final Weights:
──────────────
  W = [0.54, 0.09, 0.34, 0.03]
       │     │     │     │
       │     │     │     └─ V2 estimate (low - indirect)
       │     │     └─────── Self (previous estimate)
       │     └───────────── V0 estimate (from neighbor)
       └─────────────────── Virtual (local processing of V0's data)

Verification:
  Sum = 0.54 + 0.09 + 0.34 + 0.03 = 1.00 ✓
  All ≥ 0 ✓
```

---

## Hysteresis State Machine

```
Trust Score Timeline:
─────────────────────

Time:  t0    t1    t2    t3    t4    t5    t6
Trust: 0.7   0.55  0.45  0.42  0.50  0.63  0.68
       │     │     │     │     │     │     │
State: TRUSTED     │     UNTRUSTED   TRANSITION TRUSTED
                   │                         │
                   └→ Drops below τ_high    └→ Rises above τ_high
                      (transition)              (back to trusted)

Without Hysteresis (BAD):
  t0: trust > 0.5 → w = 0.3 ✓
  t1: trust > 0.5 → w = 0.3 ✓
  t2: trust < 0.5 → w = 0.0 ✗ (dropped)
  t3: trust < 0.5 → w = 0.0 ✗
  t4: trust > 0.5 → w = 0.3 ✓ (back!)
  → OSCILLATION!

With Hysteresis (GOOD):
  t0: trust > 0.6 → TRUSTED, α_taper = 1.0, w ≈ 0.3
  t1: trust > 0.6 → TRUSTED (stays), α_taper = 1.0, w ≈ 0.3
  t2: trust < 0.6 → TRANSITION, α_taper = 0.25, w ≈ 0.075
  t3: trust < 0.4 → UNTRUSTED, α_taper = 0.1, w ≈ 0.03
  t4: 0.4 < trust < 0.6 → TRANSITION, α_taper = 0.5, w ≈ 0.15
  t5: trust > 0.6 → TRUSTED, α_taper = 1.0, w ≈ 0.3
  → SMOOTH TRANSITION!

Hysteresis Gap: τ_high - τ_low = 0.6 - 0.4 = 0.2
```

---

## Row-Stochastic Guarantee

```
Why Σ w_j = 1 is GUARANTEED:

┌──────────────────────────────────────────────────────────┐
│ Stage 3: Normalization (Critical Constraints)             │
├──────────────────────────────────────────────────────────┤
│                                                            │
│ Step 1: Compute w_0                                       │
│   w_0 ∈ [0.1, 0.7]  (clamped)                            │
│                                                            │
│ Step 2: Compute w_self with CRITICAL constraint           │
│   w_self ≤ 1 - w_0   ← THIS IS KEY!                      │
│                                                            │
│   Why? Ensures w_0 + w_self ≤ 1                          │
│                                                            │
│ Step 3: Remaining mass for neighbors                      │
│   neighbor_mass = 1 - w_0 - w_self                        │
│                                                            │
│   Since w_0 + w_self ≤ 1:                                │
│     neighbor_mass = 1 - (w_0 + w_self) ≥ 0  ✓            │
│                                                            │
│ Step 4: Distribute neighbor_mass proportionally           │
│   For each neighbor k:                                     │
│     w_k = neighbor_mass * (s[k] / Σs)                     │
│                                                            │
│   Sum of neighbor weights:                                 │
│     Σ w_k = neighbor_mass * Σ(s[k]/Σs)                   │
│           = neighbor_mass * (Σs / Σs)                     │
│           = neighbor_mass * 1                              │
│           = neighbor_mass                                  │
│                                                            │
│ Step 5: Total sum                                          │
│   Σ ALL weights = w_0 + w_self + Σ w_k                   │
│                 = w_0 + w_self + neighbor_mass             │
│                 = w_0 + w_self + (1 - w_0 - w_self)       │
│                 = 1  ✓✓✓                                  │
│                                                            │
└──────────────────────────────────────────────────────────┘

Mathematical guarantee by construction!
No need for post-hoc normalization.
```

---

## Data Flow: Covariance & Innovation

```
┌────────────────────────────────────────────────────────────────┐
│ SENDER (Vehicle j)                                              │
├────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Observer Update:                                                │
│    local_state = [x, y, θ, v]                                  │
│    P_local = covariance matrix (4×4)                            │
│                                                                  │
│  Broadcast Message:                                              │
│    {                                                             │
│      'vehicle_id': j,                                            │
│      'x': local_state[0],                                        │
│      'y': local_state[1],                                        │
│      'theta': local_state[2],                                    │
│      'v': local_state[3],                                        │
│      'covariance_trace': trace(P_local),  ← NEW                 │
│      'timestamp': GPS_time                                       │
│    }                                                             │
│                                                                  │
└────────────────────────────────────────────────────────────────┘
                            ↓ (network)
┌────────────────────────────────────────────────────────────────┐
│ RECEIVER (Vehicle i)                                            │
├────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Step 1: Receive state from j                                   │
│    received_state = parse_message()                             │
│                                                                  │
│  Step 2: Predict j's state using previous data                  │
│    last_state = state_queue.get_latest(j)                       │
│    dt = now - last_state['timestamp']                           │
│    predicted = {                                                 │
│      'x': last_x + last_v * cos(last_θ) * dt,                  │
│      'y': last_y + last_v * sin(last_θ) * dt,                  │
│      'theta': last_θ,                                           │
│      'v': last_v                                                 │
│    }                                                             │
│                                                                  │
│  Step 3: Compute innovation                                     │
│    position_error = ||[recv_x, recv_y] - [pred_x, pred_y]||    │
│    velocity_error = |recv_v - pred_v|                           │
│    innovation = position_error + 0.5 * velocity_error           │
│                                                                  │
│  Step 4: Store metrics                                          │
│    neighbor_quality[j] = {                                       │
│      'covariance': received_state['covariance_trace'],          │
│      'recent_innovations': deque([..., innovation]),            │
│      'last_recv_time': now,                                     │
│      'drop_rate': compute_drop_rate(j)                          │
│    }                                                             │
│                                                                  │
│  Step 5: Use in weight calculation                              │
│    weights = calculate_weights_trust_v2(                        │
│      neighbor_covariances=[quality[k]['covariance']],           │
│      innovations=[mean(quality[k]['recent_innovations'])],      │
│      ...                                                         │
│    )                                                             │
│                                                                  │
└────────────────────────────────────────────────────────────────┘
```

---

## Stability Guarantee

```
┌────────────────────────────────────────────────────────────────┐
│  Why Row-Stochastic → Stability                                │
├────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Distributed Observer Update:                                   │
│    x̂(t+1) = W · x̂(t)                                          │
│                                                                  │
│  Where W is row-stochastic (Σ w_j = 1, w_j ≥ 0)               │
│                                                                  │
│  Lemma (Boundedness):                                           │
│    ||x̂(t+1)||∞ ≤ ||x̂(t)||∞                                   │
│                                                                  │
│  Proof:                                                         │
│    x̂_i(t+1) = Σ w_j · x̂_j(t)                                 │
│                                                                  │
│    Since W is row-stochastic, this is a CONVEX COMBINATION:    │
│      min_j x̂_j(t) ≤ x̂_i(t+1) ≤ max_j x̂_j(t)                │
│                                                                  │
│    Therefore:                                                   │
│      |x̂_i(t+1)| ≤ max_j |x̂_j(t)| = ||x̂(t)||∞               │
│                                                                  │
│    Taking max over all i:                                       │
│      ||x̂(t+1)||∞ ≤ ||x̂(t)||∞  ✓                             │
│                                                                  │
│  Result: State estimates NEVER EXPLODE!                         │
│                                                                  │
│  Additional guarantee under connectivity:                       │
│    lim_{t→∞} ||x̂_i(t) - x̂_j(t)|| = 0  (Consensus)           │
│                                                                  │
└────────────────────────────────────────────────────────────────┘
```

---

## Quick Decision Tree

```
┌─────────────────────────────────────────────────────────┐
│  "How should I set w_0 for target j?"                   │
└─────────────────────────────────────────────────────────┘
                        ↓
          ┌─────────────┴─────────────┐
          │  Is j = self (i)?          │
          └─────────────┬─────────────┘
                Yes ↓         ↓ No
           ┌──────────┐   ┌─────────────────────────────┐
           │ w_0 HIGH │   │ Is j connected to i?         │
           │ (0.7)    │   │ (Graph[i,j] = 1)            │
           └──────────┘   └──────────┬──────────────────┘
                                 Yes ↓    ↓ No
                        ┌──────────────┐  ┌──────────────┐
                        │ Check quality │  │ w_0 LOW      │
                        │ of comm       │  │ (0.1)        │
                        └──────┬────────┘  └──────────────┘
                               ↓
                    ┌──────────┴──────────┐
                    │ High trust + fresh?  │
                    └──────────┬───────────┘
                          Yes ↓    ↓ No
                    ┌──────────┐  ┌──────────┐
                    │ w_0 MED  │  │ w_0 LOW- │
                    │ (0.5)    │  │ MED (0.3)│
                    └──────────┘  └──────────┘
```

---

**For complete details, see the full documentation suite in `docs/`**
