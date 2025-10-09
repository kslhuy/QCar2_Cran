# Quick Reference: Adaptive Trust-Based Weights

**Date:** October 5, 2025  
**Purpose:** Quick lookup for implementation team

---

## 1. Critical Clarifications

### Virtual Node Reality
```
Virtual node (w_0) is NOT about sensors, it's about LOCAL vs DISTRIBUTED:

┌─────────────────────────────────────────────────────────────┐
│ Estimating Target j                                          │
├─────────────────────────────────────────────────────────────┤
│ Case 1: j = self                                             │
│   w_0 → uses GPS/IMU/tachometer (true sensors)              │
│                                                               │
│ Case 2: j is connected neighbor                              │
│   w_0 → uses RECEIVED local state from j                     │
│   (j broadcasts its position via network)                    │
│                                                               │
│ Case 3: j is NOT connected                                   │
│   w_0 → LOW (we have no direct data from j)                  │
│   rely on w_k from neighbors connected to j                  │
└─────────────────────────────────────────────────────────────┘
```

### Data Sources for Metrics

| Metric | Source | How to Get |
|--------|--------|------------|
| **Covariance** | Observer P matrix | `trace(observer.P_local)` - broadcast in state msg |
| **Innovation** | Prediction residual | `norm(received_state - predicted_state)` |
| **Message Age** | Timestamp diff | `current_time - msg_timestamp` |
| **Drop Rate** | Communication stats | `drop_count / expected_msgs` (running average) |
| **Distance** | Sensor/State | CamLidar fusion OR `norm(pos_i - pos_j)` |

### Row-Stochastic Guarantee

```python
# Critical constraint in normalization:
w_self = clamp(w_self_base, w_self_min, 1 - w0)  # Ensures w0 + w_self ≤ 1

neighbor_mass = 1 - w0 - w_self  # Always ≥ 0

# Distribute neighbor_mass proportionally:
for k in neighbors:
    w_k = neighbor_mass * (score[k] / sum_scores)

# Result: sum(weights) = w0 + w_self + neighbor_mass = 1 ✓
```

---

## 2. Implementation Sequence

### Phase 1: Data Collection (Week 1)

```python
# 1. Broadcast covariance
def broadcast_own_state(self):
    state_msg = {
        'vehicle_id': self.vehicle_id,
        'x': float(local_state[0]),
        'y': float(local_state[1]),
        'theta': float(local_state[2]),
        'v': float(local_state[3]),
        'covariance_trace': float(np.trace(self.observer.P_local)),  # NEW
        'timestamp': self.gps_sync.get_synced_time()
    }
    self.comm.send_state(state_msg)

# 2. Compute innovation
def _handle_vehicle_state_direct(self, received_state):
    predicted = self._predict_neighbor_state(sender_id, current_time)
    innovation = np.linalg.norm([
        received_state['x'] - predicted['x'],
        received_state['y'] - predicted['y']
    ])
    self.neighbor_quality[sender_id]['recent_innovations'].append(innovation)

# 3. Track message age and drops
self.neighbor_quality[sender_id]['last_recv_time'] = current_time
self.neighbor_quality[sender_id]['recv_count'] += 1
```

### Phase 2: Weight Calculator (Week 2)

```python
# In Weight_Trust_module.py
def calculate_weights_trust_v2(
    self,
    vehicle_index: int,
    trust_local: np.ndarray,      # From GraphBasedTrustEvaluator
    trust_global: np.ndarray,
    message_ages: np.ndarray = None,
    drop_rates: np.ndarray = None,
    neighbor_covariances: np.ndarray = None,
    distances: np.ndarray = None,
    self_covariance: float = None,
    innovations: np.ndarray = None,
    prev_weights: np.ndarray = None,
    config: dict = None
) -> dict:
    """
    Returns: {
        'weights': (1, N+1) array,
        'used_neighbors': list,
        'gated_neighbors': list,
        'scores': array,
        'virtual_weight': float,
        'self_weight': float
    }
    """
    # Stage 1: Gating
    gated = []
    for k in range(N):
        if message_ages[k] > 0.5:  # Hard gate: stale
            scores[k] = 0
            gated.append(k)
        elif innovations[k] > 5.0:  # Hard gate: large innovation
            scores[k] = 0
            gated.append(k)
        else:
            # Soft gate: hysteresis
            alpha_taper = apply_hysteresis(trust_local[k], trust_global[k])
            
            # Stage 2: Multi-factor scoring
            scores[k] = alpha_taper * \
                        (trust_local[k]**1.5 * trust_global[k]**1.0) * \
                        np.exp(-2.0 * message_ages[k]) * \
                        (1 / (1 + drop_rates[k])) * \
                        (1 / (1 + 0.1 * neighbor_covariances[k])) * \
                        (1 / (1 + 0.05 * distances[k]))
    
    # Stage 3: Normalization
    # Check connectivity
    is_connected = self.graph[vehicle_index, k] == 1
    
    if is_connected:
        w0_base = 0.3 + 0.4 * quality_score(trust_local, message_ages, drop_rates)
    else:
        w0_base = 0.1  # Low - no direct data
    
    # Adjust by self-confidence
    w0 = w0_base / (1 + 0.2 * self_covariance)
    w0 = np.clip(w0, 0.1, 0.7)
    
    w_self = 0.1 + 0.3 / (1 + 0.5 * self_covariance)
    w_self = np.clip(w_self, 0.1, 1 - w0)
    
    neighbor_mass = 1 - w0 - w_self
    weights_neighbors = neighbor_mass * (scores / np.sum(scores))
    
    # Optional: EMA smoothing
    if prev_weights is not None:
        weights = 0.8 * prev_weights + 0.2 * weights_new
    
    return {'weights': weights, ...}
```

### Phase 3: Integration (Week 3)

```python
# In VehicleObserver._get_distributed_weights()
def _get_distributed_weights(self, vehicle_process=None):
    if vehicle_process is None:
        return self.distributed_weights  # Fallback
    
    # Get trust scores
    trust_dict = vehicle_process.get_connected_vehicle_trust_scores()
    
    # Get quality metrics
    quality = vehicle_process._get_neighbor_quality_metrics()
    
    # Call v2 calculator
    result = self.weight_module.calculate_weights_trust_v2(
        vehicle_index=self.vehicle_id,
        trust_local=trust_dict['local'],
        trust_global=trust_dict['global'],
        message_ages=quality['ages'],
        drop_rates=quality['drop_rates'],
        neighbor_covariances=quality['covariances'],
        distances=quality['distances'],
        self_covariance=np.trace(self.P_local),
        innovations=quality['innovations'],
        prev_weights=self._prev_weights
    )
    
    self._prev_weights = result['weights']
    
    return result['weights']
```

---

## 3. Configuration Presets

### Conservative (Safety-Critical)
```python
CONSERVATIVE_CONFIG = {
    'tau_low': 0.5,           # High threshold
    'tau_high': 0.7,
    'w0_min': 0.3,            # Strong local anchor
    'w0_max': 0.7,
    'trust_exp_local': 1.0,   # Linear trust
    'trust_exp_global': 1.0,
    'lambda_age': 1.0,        # Slow age decay
    'eta': 0.3,               # More smoothing
    'innovation_max': 3.0     # Strict gate
}
```

### Aggressive (Performance)
```python
AGGRESSIVE_CONFIG = {
    'tau_low': 0.3,           # Low threshold
    'tau_high': 0.5,
    'w0_min': 0.1,            # Trust fleet more
    'w0_max': 0.5,
    'trust_exp_local': 2.0,   # Emphasize high trust
    'trust_exp_global': 1.5,
    'lambda_age': 3.0,        # Fast age decay
    'eta': 0.1,               # Less smoothing
    'innovation_max': 7.0     # Relaxed gate
}
```

---

## 4. Testing Checklist

### Unit Tests
- [ ] `test_row_stochastic()`: Verify sum(weights) = 1, all ≥ 0
- [ ] `test_hard_gate_stale()`: Age > 0.5s → w_k = 0
- [ ] `test_hard_gate_innovation()`: Large error → w_k = 0
- [ ] `test_hysteresis_no_oscillation()`: Trust bounces 0.48-0.52 → stable weights
- [ ] `test_smoothing_convergence()`: After step change, reaches 95% in ~15 iter
- [ ] `test_connectivity_check()`: Disconnected target → low w0

### Integration Tests
- [ ] 3-vehicle chain [V0-V1-V2]: V0 estimates V2 (not connected)
- [ ] Single Byzantine: 1 attacker, verify bounded error
- [ ] GPS denial: High P_self → low w0, high w_neighbors
- [ ] Packet loss: 30% drop rate → reduced neighbor weights
- [ ] Trust oscillation: Trust near threshold → smooth weight transition

### Performance Benchmarks
- [ ] RMSE vs. baseline (expect -25% improvement)
- [ ] Convergence time (expect -40%)
- [ ] Weight stability (std dev < 0.05)
- [ ] False positive rate (< 5%)

---

## 5. Common Pitfalls

### ❌ DON'T: Assume virtual node uses camera/LiDAR for distant targets
```python
# WRONG: Will fail for distant vehicles!
if self.cam_lidar_fusion.has_measurement(target_id):
    w0 = 0.7  # High
```

### ✅ DO: Check graph connectivity first
```python
# CORRECT
if self.graph[host_id, target_id] == 1:  # Connected
    w0 = compute_based_on_quality()
else:  # Not connected
    w0 = 0.1  # Low - no direct data
```

### ❌ DON'T: Forget to normalize weights
```python
# WRONG: Weights may not sum to 1
w0 = 0.5
w_self = 0.3
w_neighbors = scores / sum(scores)  # Missing scaling!
```

### ✅ DO: Ensure neighbor mass sums correctly
```python
# CORRECT
neighbor_mass = 1 - w0 - w_self
w_neighbors = neighbor_mass * (scores / sum(scores))
```

### ❌ DON'T: Use hard threshold without hysteresis
```python
# WRONG: Causes oscillation
if trust > 0.5:
    weight = 1.0 / n
else:
    weight = 0.0
```

### ✅ DO: Apply hysteresis for stability
```python
# CORRECT
if trust > tau_high:
    state = TRUSTED
elif trust < tau_low:
    state = UNTRUSTED
# state persists in between
```

---

## 6. Debug Diagnostics

### Log Weight Changes
```python
if np.linalg.norm(weights - prev_weights) > 0.1:
    logger.info(
        f"Large weight shift: Δ={delta:.3f}, "
        f"w0={w0:.2f}, w_self={w_self:.2f}, "
        f"gated={gated_neighbors}"
    )
```

### Monitor Trust-Weight Correlation
```python
logger.debug(
    f"V{neighbor_id}: trust_loc={t_loc:.2f}, trust_glob={t_glob:.2f}, "
    f"age={age:.3f}s, drop={drop:.2%}, weight={w:.3f}"
)
```

### Track Consensus Convergence
```python
# Compute inter-vehicle estimate variance
estimates = [vehicle.observer.fleet_states[:, target_id] for vehicle in fleet]
variance = np.var(estimates, axis=0)
logger.info(f"Fleet consensus variance: {np.mean(variance):.4f}")
```

---

## 7. Mathematical Quick Reference

| Property | Formula | Meaning |
|----------|---------|---------|
| Row-stochastic | $\sum_j w_j = 1, w_j \geq 0$ | Weights are convex combination |
| Innovation | $\varepsilon = \|\|x_{\text{recv}} - x_{\text{pred}}\|\|$ | Prediction error |
| Covariance trace | $\text{tr}(P) = \sum_i P_{ii}$ | Scalar uncertainty measure |
| Trust blend | $s_{\text{trust}} = t_{\text{loc}}^p \cdot t_{\text{glob}}^q$ | Multiplicative trust |
| Age decay | $s_{\text{age}} = e^{-\lambda \Delta t}$ | Exponential freshness |
| Virtual weight | $w_0 \propto \text{quality} / (1 + c \cdot \text{tr}(P_{\text{self}}))$ | Adaptive local anchor |
| EMA smoothing | $W(t) = (1-\eta)W(t-1) + \eta W^*(t)$ | Temporal stability |

---

**For full details, see:**
- `AdaptiveTrustWeightDesign.md` - Complete design specification
- `AdaptiveTrustWeight_Clarifications.md` - Detailed clarifications on virtual node, covariance, stability

**Questions?** Review the clarifications document or contact the fleet framework team.
