# Clarifications: Adaptive Trust-Based Weighting Design

**Date:** October 5, 2025  
**Status:** Critical Implementation Details  
**Related:** AdaptiveTrustWeightDesign.md

---

## 1. Virtual Node Clarification: What Data Does It Actually Use?

### 1.1 The Problem with "Local Measurement"

**Original claim (INCORRECT for distant targets):**
> "Virtual node uses direct sensor measurements (camera/LiDAR)"

**Reality:** This only works for:
- ✅ **Self-estimation** (target = host vehicle): We DO have GPS, IMU, tachometer
- ✅ **Adjacent vehicles** (1-2m away): Camera/LiDAR can measure relative position
- ❌ **Distant vehicles** (>10m): No direct sensor measurement available!

### 1.2 Corrected Virtual Node Interpretation

The virtual node (w₀) in distributed observer represents **our LOCAL OBSERVER's estimate**, NOT raw sensor measurement.

```
For vehicle i estimating vehicle j:

┌─────────────────────────────────────────────────────────────┐
│ Virtual Node (w_0): LOCAL OBSERVER estimate of target j     │
├─────────────────────────────────────────────────────────────┤
│ Data source depends on target:                               │
│                                                               │
│ Case 1: target j = self (i = j)                             │
│   → Use GPS/IMU/tachometer (true local measurement)         │
│   → Highest confidence (direct sensors)                      │
│                                                               │
│ Case 2: target j is DIRECTLY communicating with i           │
│   → Use RECEIVED local state from j                          │
│   → Data: j broadcasts its GPS position via network         │
│   → Trust: depends on communication quality + trust_local    │
│                                                               │
│ Case 3: target j is NOT directly connected to i             │
│   → Use previous distributed estimate (fallback)             │
│   → OR: interpolate from connected neighbors' fleet states  │
│                                                               │
│ Key point: Virtual node is NOT about sensor modality,       │
│            it's about LOCAL vs DISTRIBUTED consensus!        │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Distributed Nodes (w_k): FLEET OBSERVER estimates           │
├─────────────────────────────────────────────────────────────┤
│ Data source:                                                 │
│   → Neighbor k's distributed observer estimate of j         │
│   → Aggregates k's view of the entire fleet                 │
│   → Trust: depends on trust_global (cross-consistency)      │
└─────────────────────────────────────────────────────────────┘
```

### 1.3 Correct Weight Interpretation

**w₀ (virtual/local weight):** 
- High w₀ → **Trust our own local processing** of received data from j
- Low w₀ → **Trust fleet consensus** from multiple observers

**Physical meaning:**
- When **j is directly connected** to i and sending reliable local states:
  - High w₀ = "I trust j's self-report more than others' estimates of j"
  - This makes sense if: trust_local[j] is high, messages are fresh, low packet loss
  
- When **j is NOT directly connected** to i:
  - w₀ should be LOW (we don't have direct data from j)
  - Rely on w_k from neighbors who ARE connected to j

### 1.4 Revised Virtual Weight Formula

```python
def compute_virtual_weight(self, target_id, host_id, trust_scores, quality_metrics):
    """
    Compute adaptive virtual node weight.
    
    Virtual weight depends on:
    1. Whether target is directly connected (adjacency)
    2. Quality of direct communication if connected
    3. Self-confidence in local processing
    """
    
    # Check if target is directly connected
    is_directly_connected = self.graph[host_id, target_id] == 1
    
    if target_id == host_id:
        # Case 1: Self-estimation (always trust local sensors)
        base_w0 = 0.7  # High confidence in own sensors
        
    elif is_directly_connected:
        # Case 2: Target is neighbor - we receive their local state
        # Adjust w0 based on communication quality and local trust
        
        local_trust = trust_scores.get('local', 0.5)
        message_age = quality_metrics.get('age', 0.5)
        drop_rate = quality_metrics.get('drop_rate', 0.0)
        
        # High quality communication → trust direct data (high w0)
        # Poor quality → rely on fleet consensus (low w0)
        quality_score = local_trust * np.exp(-2.0 * message_age) * (1 - drop_rate)
        
        base_w0 = 0.3 + 0.4 * quality_score  # Range: [0.3, 0.7]
        
    else:
        # Case 3: Target is NOT directly connected
        # We don't have direct data from target, must rely on fleet
        base_w0 = 0.1  # Low - rely on distributed consensus
    
    # Further adjust by self-confidence (covariance)
    if hasattr(self, 'P_local'):
        self_uncertainty = np.trace(self.P_local)
        # Higher uncertainty → reduce w0 further
        confidence_factor = 1.0 / (1.0 + 0.2 * self_uncertainty)
    else:
        confidence_factor = 1.0
    
    w0 = base_w0 * confidence_factor
    w0 = np.clip(w0, 0.05, 0.75)  # Safety bounds
    
    return w0
```

### 1.5 Example Scenarios

**Scenario A: 3-vehicle chain [V0 → V1 → V2]**

```
Graph: V0-V1, V1-V2 (no V0-V2 connection)

V1 estimating V0:
  - V0 is directly connected ✓
  - V1 receives V0's local state broadcasts
  - w0 = 0.5 (moderate, depends on communication quality)
  - w1 (neighbors) = use V2's estimate of V0 (if V2-V0 connected)

V1 estimating V2:
  - V2 is directly connected ✓
  - w0 = 0.5 (trust direct data from V2)
  
V0 estimating V2:
  - V2 is NOT directly connected ✗
  - V0 doesn't receive V2's local state
  - w0 = 0.1 (low - no direct data)
  - w_neighbors = rely heavily on V1's estimate of V2
```

---

## 2. How to Get Covariance and Innovation?

### 2.1 Covariance (Uncertainty Metric)

**What is it?**
Covariance matrix P represents estimation uncertainty. Trace(P) is a scalar summary.

**Where to get it?**

#### Option A: Broadcast in Messages (Recommended)

Modify state broadcast to include covariance:

```python
# In VehicleProcess.broadcast_own_state()
def broadcast_own_state(self):
    # Get local state from observer
    local_state = self.observer.get_local_state()  # [x, y, theta, v]
    
    # Get covariance from observer
    if hasattr(self.observer, 'P_local'):
        cov_trace = float(np.trace(self.observer.P_local))
    else:
        cov_trace = 1.0  # Default uncertainty
    
    state_msg = {
        'vehicle_id': self.vehicle_id,
        'timestamp': self.gps_sync.get_synced_time(),
        'x': float(local_state[0]),
        'y': float(local_state[1]),
        'theta': float(local_state[2]),
        'v': float(local_state[3]),
        'covariance_trace': cov_trace,  # NEW: uncertainty metric
        'control': self.current_control_input.tolist()
    }
    
    self.comm.send_state(state_msg)
```

**Receiving side:**
```python
# In VehicleProcess._handle_vehicle_state_direct()
def _handle_vehicle_state_direct(self, received_state: dict):
    sender_id = received_state['vehicle_id']
    
    # Extract covariance if available
    neighbor_cov = received_state.get('covariance_trace', None)
    
    # Store for weight calculation
    if neighbor_cov is not None:
        self.neighbor_quality[sender_id]['last_covariance'] = neighbor_cov
```

#### Option B: Estimate from Innovation Variance (Fallback)

If covariance not broadcast, proxy with recent innovation variance:

```python
def estimate_neighbor_uncertainty(self, neighbor_id):
    """
    Estimate neighbor's uncertainty from our innovation history.
    High innovation variance → neighbor is uncertain or unreliable.
    """
    innovations = self.neighbor_quality[neighbor_id]['recent_innovations']
    
    if len(innovations) < 3:
        return 1.0  # Default
    
    # Use variance of innovations as proxy for uncertainty
    innov_variance = np.var(innovations)
    
    # Map variance to covariance trace (heuristic)
    # Typical position variance: 0.01-1.0 m²
    estimated_cov_trace = np.sqrt(innov_variance) * 2.0
    
    return estimated_cov_trace
```

### 2.2 Innovation (Prediction Residual)

**What is it?**
Innovation = (received state) - (predicted state)

Measures how much the received data deviates from our expectation.

**How to compute:**

```python
# In VehicleProcess._handle_vehicle_state_direct()
def _handle_vehicle_state_direct(self, received_state: dict):
    sender_id = received_state['vehicle_id']
    current_time = time.time()
    
    # Step 1: Predict where sender should be now
    predicted_state = self._predict_neighbor_state(sender_id, current_time)
    
    # Step 2: Compute innovation (prediction error)
    received_pos = np.array([received_state['x'], received_state['y']])
    predicted_pos = np.array([predicted_state['x'], predicted_state['y']])
    
    position_innovation = np.linalg.norm(received_pos - predicted_pos)
    
    # Also compute velocity innovation if available
    if 'v' in received_state and 'v' in predicted_state:
        velocity_innovation = abs(received_state['v'] - predicted_state['v'])
    else:
        velocity_innovation = 0.0
    
    # Combined innovation (weighted by state dimensions)
    total_innovation = position_innovation + 0.5 * velocity_innovation
    
    # Store for weight calculation
    self.neighbor_quality[sender_id]['recent_innovations'].append(total_innovation)
    
    # Log large innovations (potential anomaly)
    if position_innovation > 2.0:  # 2m threshold
        self.comm_logger.warning(
            f"Large innovation from V{sender_id}: {position_innovation:.2f}m. "
            f"Received: ({received_pos[0]:.2f}, {received_pos[1]:.2f}), "
            f"Predicted: ({predicted_pos[0]:.2f}, {predicted_pos[1]:.2f})"
        )
```

**Prediction method:**

```python
def _predict_neighbor_state(self, neighbor_id, target_time):
    """
    Predict neighbor's state at target_time using previous received state
    and constant velocity model.
    """
    # Get last received state
    if neighbor_id not in self.neighbor_quality:
        return {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'v': 0.0}
    
    last_state = self.state_queue.get_latest_state(neighbor_id)
    if last_state is None:
        return {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'v': 0.0}
    
    # Time since last state
    dt = target_time - last_state['timestamp']
    dt = max(0.0, min(dt, 1.0))  # Clamp to [0, 1] second
    
    # Constant velocity prediction
    x_pred = last_state['x'] + last_state['v'] * np.cos(last_state['theta']) * dt
    y_pred = last_state['y'] + last_state['v'] * np.sin(last_state['theta']) * dt
    theta_pred = last_state['theta']  # Assume constant heading
    v_pred = last_state['v']  # Assume constant velocity
    
    return {
        'x': x_pred,
        'y': y_pred,
        'theta': theta_pred,
        'v': v_pred,
        'timestamp': target_time
    }
```

### 2.3 Complete Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│ SENDER (Vehicle k)                                           │
├─────────────────────────────────────────────────────────────┤
│ 1. Observer computes: local_state, P_local                  │
│ 2. Broadcast: {x, y, θ, v, trace(P_local), timestamp}      │
└─────────────────────────────────────────────────────────────┘
                            ↓ (network)
┌─────────────────────────────────────────────────────────────┐
│ RECEIVER (Vehicle i)                                         │
├─────────────────────────────────────────────────────────────┤
│ 3. Receive state from k                                      │
│ 4. Predict k's state using previous data                     │
│ 5. Compute innovation = received - predicted                 │
│ 6. Store:                                                     │
│    - covariance_trace[k] = trace(P_k)                       │
│    - innovation_history[k].append(innovation)                │
│    - message_age[k] = now - timestamp                        │
│    - drop_rate[k] = update running average                   │
│ 7. Use in weight calculation every observer cycle           │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. Row-Stochastic Property & Stability Guarantees

### 3.1 Row-Stochastic Definition

A matrix W is **row-stochastic** if:

```math
∀i: Σ_j w_{ij} = 1  AND  ∀i,j: w_{ij} ≥ 0
```

For our weight vector W_i ∈ ℝ^(1×(N+1)):

```math
w_0 + w_{i+1} + Σ_{k≠i} w_{k+1} = 1
w_j ≥ 0  ∀j
```

**Why it matters:**
- Ensures weights are a **convex combination** (weighted average)
- Critical for consensus convergence proofs
- Prevents unbounded state growth

### 3.2 How Our Design Preserves Row-Stochastic Property

**Stage 3 Normalization (from design doc):**

```python
# Step 1: Compute virtual weight (bounded)
w0 = clamp(alpha_local, w0_min, w0_max)  # e.g., [0.1, 0.7]

# Step 2: Compute self weight (bounded)
w_self = clamp(w_self_base, w_self_min, 1 - w0)  # Ensures w0 + w_self ≤ 1

# Step 3: Allocate remaining mass to neighbors
neighbor_mass = 1 - w0 - w_self  # By construction: ≥ 0

# Step 4: Distribute proportionally
S_total = Σ_k s[k]  # Sum of neighbor scores
for k in neighbors:
    w_k = neighbor_mass * (s[k] / S_total)  # Proportional split

# Result: Σ w_j = w0 + w_self + Σ w_k 
#               = w0 + w_self + neighbor_mass * Σ(s[k]/S_total)
#               = w0 + w_self + neighbor_mass * 1
#               = w0 + w_self + (1 - w0 - w_self)
#               = 1  ✓
```

**Key invariant:** 
```
neighbor_mass = 1 - w0 - w_self
```
This ensures the sum is always 1, regardless of how we split neighbor_mass.

### 3.3 Stability Analysis

#### A. Consensus Convergence (Formal)

**Standard Distributed Observer Theorem:**

Given a connected graph with row-stochastic weights satisfying:
1. **Aperiodicity:** ∃i: w_{ii} > 0 (self-weight > 0)
2. **Connectivity:** Graph is strongly connected
3. **Bounded influence:** max_k w_{ik} ≤ α < 1

Then the distributed observer converges to consensus:

```math
lim_{t→∞} ||x̂_i(t) - x̂_j(t)|| = 0  ∀i,j
```

**Our design satisfies:**
1. ✓ Aperiodicity: w_self ≥ w_self_min > 0
2. ✓ Connectivity: Assumed from fleet graph
3. ✓ Bounded influence: w_cap parameter limits max neighbor weight

#### B. Convergence Rate

The convergence rate depends on the **second-largest eigenvalue** of the weight matrix W.

**Faster convergence when:**
- Balanced weights (avoid extreme values like 0.9 or 0.01)
- Higher connectivity (more neighbors)
- Moderate self-weight (not too high, not too low)

**Our adaptive weights optimize convergence by:**
- Reducing weights of low-trust neighbors (focus on reliable sources)
- Increasing w0 when local data is high quality (reduce network dependency)
- Smoothing prevents rapid weight changes (stability)

#### C. Bounded Estimation Error Under Attacks

**Resilient Consensus Theorem (Informal):**

If:
- At most f neighbors are Byzantine (malicious)
- Number of trusted neighbors |N_i^t| ≥ 2f + 1
- Virtual weight w0 ≥ w_min > 0 (local anchor)

Then:
```math
||x̂_i(∞) - x_true|| ≤ ε(f, w0, trust_scores)
```

Where ε decreases with:
- Higher w0 (stronger local anchor)
- Higher trust scores (better neighbor selection)
- Lower f (fewer attackers)

**Physical interpretation:**
- Even if half the fleet is compromised, each vehicle can maintain bounded error
- The virtual node w0 acts as a "safety anchor" preventing divergence
- Trust-based gating removes Byzantine influence

### 3.4 Hysteresis for Stability

**Problem:** Trust scores oscillate near threshold → weights oscillate → instability

**Solution:** Hysteresis state machine

```
State transitions per neighbor k:

  TRUSTED (trust > τ_high)
     ↓ (trust drops below τ_high)
  TRANSITION (τ_low ≤ trust ≤ τ_high)
     ↓ (trust drops below τ_low)
  UNTRUSTED (trust < τ_low)
     ↑ (trust rises above τ_high)  ← Note: requires crossing τ_high to re-enter
```

**Gap between thresholds:**
```
τ_high - τ_low = 0.6 - 0.4 = 0.2
```

This prevents rapid on/off switching when trust hovers around a single threshold.

**Weight taper in transition zone:**
```python
if state == TRANSITION:
    alpha_taper = (trust - tau_low) / (tau_high - tau_low)
    # Smoothly varies from 0.1 to 1.0 as trust increases
else:
    alpha_taper = 1.0 if state == TRUSTED else 0.1
```

### 3.5 EMA Smoothing for Temporal Stability

**Exponential Moving Average:**
```python
W(t) = (1 - η) * W(t-1) + η * W_computed(t)
```

**Effect:**
- η = 0.2 → 80% old weight, 20% new weight
- Dampens rapid changes
- Trade-off: slower response to actual changes

**When to use:**
- High η (0.3-0.5): Fast response, less smooth (aggressive mode)
- Low η (0.1-0.2): Slow response, very smooth (conservative mode)

**Convergence time:**
After a step change in trust, weight reaches 95% of final value in:
```
t_95% = -ln(0.05) / η
For η = 0.2: t_95% ≈ 15 iterations
At 100 Hz: ≈ 0.15 seconds
```

### 3.6 Proof Sketch: Why Row-Stochastic + Non-negative Ensures Stability

**Lemma:** If W is row-stochastic and W_ij ≥ 0, then for any state vector x(t):

```math
||x(t+1)||∞ ≤ ||x(t)||∞
```

**Proof:**
```math
x_i(t+1) = Σ_j w_{ij} x_j(t)
         = Σ_j w_{ij} x_j(t)  where Σ_j w_{ij} = 1

By convexity of weighted average:
min_j x_j(t) ≤ x_i(t+1) ≤ max_j x_j(t)

Therefore:
||x(t+1)||∞ = max_i |x_i(t+1)| ≤ max_j |x_j(t)| = ||x(t)||∞
```

**Implication:** State estimates remain bounded (no explosion).

---

## 4. Implementation Checklist

### 4.1 Phase 1: Basic Infrastructure

- [ ] **Broadcast covariance in state messages**
  - Modify `broadcast_own_state()` to include `trace(P_local)`
  - Test: verify covariance is received and stored

- [ ] **Implement innovation computation**
  - Add `_predict_neighbor_state()` method
  - Compute innovation in `_handle_vehicle_state_direct()`
  - Store in `recent_innovations` deque

- [ ] **Quality metrics aggregation**
  - Implement `_get_neighbor_quality_metrics()` in VehicleProcess
  - Return dict with ages, drop_rates, covariances, innovations

### 4.2 Phase 2: Weight Calculation v2

- [ ] **Implement `calculate_weights_trust_v2()`**
  - Stage 1: Gating (hard + soft with hysteresis)
  - Stage 2: Multi-factor scoring
  - Stage 3: Normalization with adaptive w0
  - Verify row-stochastic property (unit test)

- [ ] **Revise virtual weight logic**
  - Check graph connectivity: `graph[host_id, target_id]`
  - Different w0 for self / connected / disconnected targets
  - Adjust by communication quality

- [ ] **Add hysteresis state machine**
  - Track per-neighbor trust state (TRUSTED/TRANSITION/UNTRUSTED)
  - Persist state across weight calculations
  - Log state transitions

### 4.3 Phase 3: Integration

- [ ] **VehicleObserver integration**
  - Modify `_get_distributed_weights()` to call v2
  - Pass `vehicle_process` reference for quality metrics
  - Cache previous weights for smoothing

- [ ] **VehicleProcess integration**
  - Initialize `neighbor_quality` dict in `__init__`
  - Update quality metrics in communication handlers
  - Call observer with quality data

- [ ] **Configuration management**
  - Add `WEIGHT_CONFIG` to config file
  - Support conservative/aggressive presets
  - Allow per-vehicle config overrides

### 4.4 Phase 4: Testing & Validation

- [ ] **Unit tests**
  - Row-stochastic property (sum = 1, all ≥ 0)
  - Hard gate rejection (stale, large innovation)
  - Hysteresis stability (no oscillation)
  - Smoothing convergence time

- [ ] **Integration tests**
  - 3-vehicle chain scenario
  - Single Byzantine attacker
  - GPS denial (high self covariance)
  - Network packet loss

- [ ] **Performance benchmarks**
  - Estimation RMSE vs. baseline
  - Convergence time
  - Weight change frequency
  - False positive/negative gating rates

---

## 5. Key Takeaways

### Virtual Node Reality Check ✓
- For **distant targets**: virtual node uses **received local state**, NOT sensors
- For **self**: virtual node uses true sensor measurements
- Virtual weight adapts based on **connectivity** and **communication quality**

### Covariance & Innovation ✓
- **Covariance**: Broadcast `trace(P)` in state messages OR estimate from innovation variance
- **Innovation**: Compute as `||received - predicted||` using constant velocity model

### Stability Guarantees ✓
- **Row-stochastic**: Preserved by careful normalization (w0 + w_self + Σw_k = 1)
- **Convergence**: Standard consensus theory applies with bounded weights
- **Hysteresis**: Prevents oscillation near trust thresholds
- **Smoothing**: EMA dampens rapid weight changes

### Critical Implementation Points ✓
1. Check graph connectivity before setting high w0
2. Always normalize weights to sum to 1
3. Apply hysteresis to trust-based gating
4. Smooth weights over time for stability
5. Log diagnostics for weight changes and gating events

---

**Next Steps:**
1. Review and approve these clarifications
2. Update `AdaptiveTrustWeightDesign.md` with corrected virtual node interpretation
3. Begin Phase 1 implementation (covariance broadcast + innovation)
4. Create unit test suite for weight calculation
5. Run scenario-based validation

**Questions for Discussion:**
- Should we support dimension-wise weighting (different weights for x, y, v)?
- What's the acceptable convergence time for your application?
- Do you want automatic parameter tuning or manual configuration?
