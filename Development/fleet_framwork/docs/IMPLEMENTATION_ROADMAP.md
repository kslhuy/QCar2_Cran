# Implementation Roadmap: Trust-Based Adaptive Weights

**Start Date:** October 5, 2025  
**Target:** 4-week implementation  
**Priority:** High - Critical for trust-resilient distributed observer

---

## 🎯 Implementation Strategy

### Incremental Approach (Recommended)
We'll build in 4 phases, testing each before moving to the next:

1. **Week 1:** Data infrastructure (covariance, innovation, quality tracking)
2. **Week 2:** Weight calculator v2 (multi-factor scoring)
3. **Week 3:** Integration with observer and vehicle process
4. **Week 4:** Testing, tuning, and validation

---

## Week 1: Data Collection Infrastructure

### Goal
Get all the metrics needed for weight calculation flowing through the system.

---

### ✅ Task 1.1: Broadcast Covariance in State Messages

**File:** `VehicleProcess.py`  
**Method:** `broadcast_own_state()`  
**Priority:** HIGH (Required for uncertainty-based weighting)

**What to do:**

1. Find the `broadcast_own_state()` method (around line 1260)
2. Add covariance trace to the state message

**Code to add:**
```python
def broadcast_own_state(self):
    """Broadcast vehicle's own state to fleet."""
    if self.observer is None:
        return
    
    # Get local state from observer
    local_state = self.observer.get_local_state()  # [x, y, theta, v]
    current_time = self.gps_sync.get_synced_time()
    
    # NEW: Get covariance trace from observer
    try:
        if hasattr(self.observer, 'P_local') and self.observer.P_local is not None:
            cov_trace = float(np.trace(self.observer.P_local))
        else:
            cov_trace = 1.0  # Default uncertainty if covariance not available
    except Exception as e:
        self.comm_logger.warning(f"Vehicle {self.vehicle_id}: Failed to get covariance: {e}")
        cov_trace = 1.0
    
    # Build state message
    state_msg = {
        'vehicle_id': self.vehicle_id,
        'timestamp': current_time,
        'x': float(local_state[0]),
        'y': float(local_state[1]),
        'theta': float(local_state[2]),
        'v': float(local_state[3]),
        'covariance_trace': cov_trace,  # NEW FIELD
        'control': self.current_control_input.tolist() if hasattr(self, 'current_control_input') else [0.0, 0.0]
    }
    
    # Send via communication handler
    if self.comm is not None:
        self.comm.send_state(state_msg)
```

**Test:** Run simulation and verify `covariance_trace` appears in received messages.

---

### ✅ Task 1.2: Initialize Quality Tracking Dictionary

**File:** `VehicleProcess.py`  
**Method:** `__init__()`  
**Priority:** HIGH

**What to do:**

1. Find the trust initialization section (around line 470)
2. Add quality tracking dictionary

**Code to add:**
```python
# After trust_evaluator initialization, add:

# Initialize quality tracking for each connected vehicle
self.neighbor_quality = {}
for neighbor_id in self.connected_vehicles:
    if neighbor_id == self.vehicle_id:
        continue  # Skip self
    
    self.neighbor_quality[neighbor_id] = {
        'last_recv_time': 0.0,              # When last message received
        'recv_count': 0,                     # Total messages received
        'expected_count': 0,                 # Expected messages (for drop rate)
        'drop_count': 0,                     # Dropped/missed messages
        'recent_innovations': deque(maxlen=10),  # Last 10 innovation values
        'last_covariance': None,             # Last received covariance trace
        'last_distance': None,               # Last computed distance
        'last_state': None                   # Last received state (for prediction)
    }

self.comm_start_time = time.time()  # Track when communication started

self.logger.info(f"Vehicle {self.vehicle_id}: Initialized quality tracking for {len(self.neighbor_quality)} neighbors")
```

**Test:** Check that `neighbor_quality` dictionary is created on startup.

---

### ✅ Task 1.3: Implement State Prediction Method

**File:** `VehicleProcess.py`  
**New Method:** `_predict_neighbor_state()`  
**Priority:** HIGH (Required for innovation calculation)

**What to do:**

Add this new method to `VehicleProcess` class (around line 1600, in Helper Methods region):

```python
def _predict_neighbor_state(self, neighbor_id: int, target_time: float) -> Optional[dict]:
    """
    Predict neighbor's state at target_time using constant velocity model.
    
    Args:
        neighbor_id: ID of neighbor vehicle
        target_time: Time to predict state at
        
    Returns:
        Predicted state dict or None if no previous state available
    """
    if neighbor_id not in self.neighbor_quality:
        return None
    
    last_state = self.neighbor_quality[neighbor_id].get('last_state')
    if last_state is None:
        # No previous state, try to get from state queue
        last_state = self.state_queue.get_latest_state(neighbor_id)
        if last_state is None:
            return None
    
    # Time since last state
    dt = target_time - last_state.get('timestamp', target_time)
    dt = max(0.0, min(dt, 1.0))  # Clamp to [0, 1] second for safety
    
    # Extract state components
    x = last_state.get('x', 0.0)
    y = last_state.get('y', 0.0)
    theta = last_state.get('theta', 0.0)
    v = last_state.get('v', 0.0)
    
    # Constant velocity prediction (simple kinematic model)
    x_pred = x + v * np.cos(theta) * dt
    y_pred = y + v * np.sin(theta) * dt
    theta_pred = theta  # Assume constant heading
    v_pred = v  # Assume constant velocity
    
    return {
        'x': x_pred,
        'y': y_pred,
        'theta': theta_pred,
        'v': v_pred,
        'timestamp': target_time
    }
```

**Test:** Call this method manually and verify predictions are reasonable.

---

### ✅ Task 1.4: Compute Innovation in Message Handler

**File:** `VehicleProcess.py`  
**Method:** `_handle_vehicle_state_direct()`  
**Priority:** HIGH

**What to do:**

1. Find `_handle_vehicle_state_direct()` method (around line 1400)
2. Add innovation computation and quality tracking updates

**Code to add** (after state is validated, before adding to queue):

```python
def _handle_vehicle_state_direct(self, received_state: dict):
    """Handle received vehicle state and compute quality metrics."""
    sender_id = received_state.get('vehicle_id')
    current_time = time.time()
    
    # ... existing validation code ...
    
    # NEW: Update quality tracking
    if sender_id in self.neighbor_quality:
        q = self.neighbor_quality[sender_id]
        
        # Update receive timing
        q['last_recv_time'] = current_time
        q['recv_count'] += 1
        
        # Store received state for next prediction
        q['last_state'] = received_state.copy()
        
        # Extract covariance if available
        if 'covariance_trace' in received_state:
            q['last_covariance'] = received_state['covariance_trace']
        
        # Compute innovation (prediction error)
        try:
            predicted_state = self._predict_neighbor_state(sender_id, current_time)
            
            if predicted_state is not None:
                # Position innovation
                recv_pos = np.array([received_state['x'], received_state['y']])
                pred_pos = np.array([predicted_state['x'], predicted_state['y']])
                position_innovation = np.linalg.norm(recv_pos - pred_pos)
                
                # Velocity innovation (if available)
                if 'v' in received_state and 'v' in predicted_state:
                    velocity_innovation = abs(received_state['v'] - predicted_state['v'])
                else:
                    velocity_innovation = 0.0
                
                # Combined innovation (position weighted more heavily)
                total_innovation = position_innovation + 0.5 * velocity_innovation
                
                # Store in recent history
                q['recent_innovations'].append(total_innovation)
                
                # Log large innovations (potential anomaly)
                if position_innovation > 2.0:  # 2m threshold
                    self.comm_logger.warning(
                        f"Vehicle {self.vehicle_id}: Large innovation from V{sender_id}: "
                        f"{position_innovation:.3f}m. Received: ({recv_pos[0]:.2f}, {recv_pos[1]:.2f}), "
                        f"Predicted: ({pred_pos[0]:.2f}, {pred_pos[1]:.2f})"
                    )
                else:
                    self.comm_logger.debug(
                        f"Vehicle {self.vehicle_id}: Innovation from V{sender_id}: {position_innovation:.3f}m"
                    )
        
        except Exception as e:
            self.comm_logger.debug(f"Vehicle {self.vehicle_id}: Failed to compute innovation for V{sender_id}: {e}")
    
    # ... rest of existing code (add to queue, trust buffering, etc.) ...
```

**Test:** Check logs for innovation values. Should be small (<0.5m) during normal operation.

---

### ✅ Task 1.5: Implement Quality Metrics Aggregation

**File:** `VehicleProcess.py`  
**New Method:** `_get_neighbor_quality_metrics()`  
**Priority:** HIGH

**What to do:**

Add this method to aggregate all quality metrics for weight calculation:

```python
def _get_neighbor_quality_metrics(self) -> Dict[str, np.ndarray]:
    """
    Aggregate quality metrics for all connected neighbors.
    Used by weight calculation.
    
    Returns:
        Dictionary with arrays of metrics aligned with sorted connected_vehicles
    """
    current_time = time.time()
    
    metrics = {
        'ages': [],
        'drop_rates': [],
        'covariances': [],
        'distances': [],
        'innovations': []
    }
    
    # Process each connected vehicle in sorted order
    for neighbor_id in sorted(self.connected_vehicles):
        if neighbor_id == self.vehicle_id:
            # Skip self - weight calculator handles this
            continue
        
        if neighbor_id not in self.neighbor_quality:
            # No quality data yet, use defaults
            metrics['ages'].append(1.0)  # Old
            metrics['drop_rates'].append(0.5)  # Moderate loss
            metrics['covariances'].append(1.0)  # Default uncertainty
            metrics['distances'].append(10.0)  # Default distance
            metrics['innovations'].append(5.0)  # High innovation
            continue
        
        q = self.neighbor_quality[neighbor_id]
        
        # 1. Message Age
        age = current_time - q['last_recv_time']
        metrics['ages'].append(age)
        
        # 2. Drop Rate (simple moving window)
        elapsed_time = current_time - self.comm_start_time
        if elapsed_time > 1.0:  # After 1 second of operation
            expected_msgs = elapsed_time * (self.update_rate / 10)  # Approximate broadcast rate
            actual_msgs = q['recv_count']
            drop_rate = max(0.0, (expected_msgs - actual_msgs) / max(expected_msgs, 1))
            drop_rate = min(1.0, drop_rate)  # Clamp to [0, 1]
        else:
            drop_rate = 0.0
        metrics['drop_rates'].append(drop_rate)
        
        # 3. Covariance
        cov = q['last_covariance'] if q['last_covariance'] is not None else 1.0
        metrics['covariances'].append(cov)
        
        # 4. Distance (from state difference or sensor fusion)
        try:
            if hasattr(self, 'cam_lidar_fusion') and self.cam_lidar_fusion is not None:
                # Try to get from sensor fusion first
                dist = self.cam_lidar_fusion.get_distance_to_vehicle(neighbor_id)
                if dist is not None and dist > 0:
                    metrics['distances'].append(dist)
                    q['last_distance'] = dist
                else:
                    # Fallback: compute from position difference
                    dist = self._compute_distance_from_states(neighbor_id)
                    metrics['distances'].append(dist)
            else:
                # No sensor fusion, use state-based distance
                dist = self._compute_distance_from_states(neighbor_id)
                metrics['distances'].append(dist)
        except Exception as e:
            self.comm_logger.debug(f"Failed to compute distance to V{neighbor_id}: {e}")
            metrics['distances'].append(10.0)  # Default
        
        # 5. Innovation (mean of recent values)
        if len(q['recent_innovations']) > 0:
            mean_innovation = np.mean(q['recent_innovations'])
        else:
            mean_innovation = 0.0
        metrics['innovations'].append(mean_innovation)
    
    # Convert to numpy arrays
    return {k: np.array(v) for k, v in metrics.items()}


def _compute_distance_from_states(self, neighbor_id: int) -> float:
    """Compute distance to neighbor from position states."""
    try:
        # Get our position
        our_state = self.observer.get_local_state() if self.observer else None
        if our_state is None:
            return 10.0
        
        our_pos = our_state[:2]  # [x, y]
        
        # Get neighbor position (from queue or observer)
        neighbor_state = self.state_queue.get_latest_state(neighbor_id)
        if neighbor_state is None:
            # Try observer's fleet estimate
            if self.observer:
                fleet_states = self.observer.get_fleet_states()
                if fleet_states is not None and neighbor_id < fleet_states.shape[1]:
                    neighbor_pos = fleet_states[:2, neighbor_id]
                else:
                    return 10.0
            else:
                return 10.0
        else:
            neighbor_pos = np.array([neighbor_state['x'], neighbor_state['y']])
        
        # Euclidean distance
        distance = np.linalg.norm(our_pos - neighbor_pos)
        return float(distance)
    
    except Exception as e:
        self.comm_logger.debug(f"Error computing distance to V{neighbor_id}: {e}")
        return 10.0
```

**Test:** Call this method and verify all metrics are reasonable values.

---

### ✅ Task 1.6: Week 1 Testing Checklist

**Before moving to Week 2, verify:**

- [ ] State messages include `covariance_trace` field
- [ ] `neighbor_quality` dictionary is populated
- [ ] Innovation values are logged and reasonable (<1m typically)
- [ ] `_get_neighbor_quality_metrics()` returns valid arrays
- [ ] All arrays have same length (number of neighbors)
- [ ] No crashes or exceptions during normal operation

**Test command:**
```bash
python main.py
# Check logs for innovation values and quality metrics
```

---

## Week 2: Weight Calculator v2 (Simplified)

### Goal
Implement simplified trust-based weight calculation (trust values already normalized 0-1 from Week 1).

**Key Simplifications:**
- ✅ **No gating stage** - Trust evaluation already handles quality gating
- ✅ **No multi-factor scoring** - Trust scores already incorporate all factors
- ✅ **No self-covariance** - Simplified virtual weight calculation
- ✅ **Keep geometry component** - Distance-based weight adjustment

---

### ✅ Task 2.1: Create Simplified Weight Configuration

**File:** `Weight_Trust_module.py`  
**Location:** Top of file, after imports  
**Priority:** HIGH

**What to do:**

Add simplified configuration constants:

```python
# Simplified configuration for trust-based weight calculation
DEFAULT_WEIGHT_CONFIG = {
    # Geometry (distance-based adjustment)
    'c_d': 0.05,                # Distance penalty coefficient
    'max_distance': 50.0,       # Maximum distance to consider (meters)
    
    # Virtual node weight (fixed - no covariance adaptation)
    'w0_fixed': 0.3,            # Fixed virtual node weight
    
    # Self weight  
    'w_self_base': 0.2,         # Base self weight (fixed)
    
    # Safety limits
    'w_cap': 0.4,               # Maximum weight per neighbor (40%)
    'kappa': 5,                 # Maximum neighbor influence count
    
    # Smoothing
    'eta': 0.15,                # EMA smoothing factor (15% new, 85% old)
    'enable_smoothing': True    # Enable temporal smoothing
}
```

---

### ✅ Task 2.2: Add EMA Smoothing Support

**File:** `Weight_Trust_module.py`  
**Class:** `WeightTrustModule`  
**Priority:** LOW

**What to do:**

Add previous weights tracking to `__init__` for EMA smoothing:

```python
class WeightTrustModule:
    def __init__(self, graph, trust_threshold, kappa):
        # ... existing initialization ...
        
        # NEW: Previous weights for EMA smoothing
        # Key: vehicle_index, Value: previous weight vector
        self.prev_weights = {}
```

**Note:** Hysteresis is NOT needed - trust evaluation already handles state transitions smoothly.

---

### ✅ Task 2.3: Implement Simplified `calculate_weights_trust_v2()`

**File:** `Weight_Trust_module.py`  
**Class:** `WeightTrustModule`  
**Priority:** CRITICAL (Core algorithm)

**What to do:**

Add the simplified weight calculation method:

```python
def calculate_weights_trust_v2(
    self,
    vehicle_index: int,
    trust_scores: np.ndarray,     # Combined trust (already 0-1)
    distances: Optional[np.ndarray] = None,
    config: Optional[Dict] = None
) -> Dict[str, Any]:
    """
    SIMPLIFIED trust-based weight calculation.
    
    Trust scores already incorporate:
    - Local trust (velocity, distance, acceleration)
    - Global trust (fleet estimate consistency)
    - Communication quality (age, drop rate, innovation)
    
    This function only adds:
    - Geometry adjustment (distance penalty)
    - Weight normalization (virtual node + self + neighbors)
    - Optional EMA smoothing
    
    Args:
        vehicle_index: This vehicle's index (0-based)
        trust_scores: Trust scores for all vehicles (0-1 range)
        distances: Optional distances to neighbors (meters)
        config: Optional configuration overrides
        
    Returns:
        Dictionary with:
            - weights: (1, N+1) row-stochastic weight vector
            - used_neighbors: List of neighbor indices with non-zero weight
            - scores: Final scores per neighbor
            - virtual_weight: w_0 value
            - self_weight: w_self value
    """
    # Use default config if not provided
    cfg = DEFAULT_WEIGHT_CONFIG.copy()
    if config is not None:
        cfg.update(config)
    
    N = len(trust_scores)  # Number of vehicles
    scores = np.copy(trust_scores)  # Start with trust scores
    
    # Apply geometry adjustment (only additional factor)
    if distances is not None:
        for k in range(N):
            if k != vehicle_index and distances[k] < cfg['max_distance']:
                # Distance penalty: closer = higher weight
                s_geom = 1.0 / (1.0 + cfg['c_d'] * distances[k])
                scores[k] = scores[k] * s_geom
            elif k != vehicle_index:
                # Too far - zero weight
                scores[k] = 0.0
    
    # Set self score to 1.0 (maximum)
    scores[vehicle_index] = 1.0
    
    # WEIGHT NORMALIZATION
    
    # Step 1: Fixed virtual weight (no covariance adaptation)
    w0 = cfg['w0_fixed']
    
    # Step 2: Fixed self weight
    w_self = cfg['w_self_base']
    
    # Ensure w0 + w_self <= 1
    if w0 + w_self > 1.0:
        # Scale down proportionally
        scale = 1.0 / (w0 + w_self)
        w0 *= scale
        w_self *= scale
    
    # Step 3: Neighbor mass
    neighbor_mass = 1.0 - w0 - w_self
    
    # Step 4: Distribute to neighbors proportionally by score
    neighbor_scores = scores.copy()
    neighbor_scores[vehicle_index] = 0.0  # Exclude self from neighbor distribution
    
    S_total = np.sum(neighbor_scores)
    
    if S_total > 1e-9:  # Have valid neighbors
        weights_neighbors = neighbor_mass * (neighbor_scores / S_total)
    else:
        # No valid neighbors - redistribute to w0 and w_self
        weights_neighbors = np.zeros(N)
        w0 += neighbor_mass * 0.5
        w_self += neighbor_mass * 0.5
    
    # Step 5: Apply influence capping
    if cfg['w_cap'] < 1.0:
        for k in range(N):
            if k != vehicle_index and weights_neighbors[k] > cfg['w_cap']:
                excess = weights_neighbors[k] - cfg['w_cap']
                weights_neighbors[k] = cfg['w_cap']
                # Redistribute excess to self
                w_self += excess
    
    # Build final weight vector [w0, w1, ..., wN]
    # Index 0: virtual node
    # Index 1 to N: vehicles (index i+1 is vehicle i)
    weights = np.zeros((1, N + 1))
    weights[0, 0] = w0
    
    for k in range(N):
        if k == vehicle_index:
            weights[0, k + 1] = w_self
        else:
            weights[0, k + 1] = weights_neighbors[k]
    
    # Optional: EMA smoothing
    if cfg['enable_smoothing'] and vehicle_index in self.prev_weights:
        eta = cfg['eta']
        prev_w = self.prev_weights[vehicle_index]
        weights = (1.0 - eta) * prev_w + eta * weights
    
    # Store for next iteration
    self.prev_weights[vehicle_index] = weights.copy()
    
    # Final normalization (numerical stability)
    weight_sum = np.sum(weights)
    if abs(weight_sum - 1.0) > 1e-6:
        weights = weights / weight_sum
    
    # Identify used neighbors
    used_neighbors = [k for k in range(N) if k != vehicle_index and weights[0, k+1] > 1e-6]
    
    return {
        'weights': weights,
        'used_neighbors': used_neighbors,
        'scores': scores,
        'virtual_weight': float(w0),
        'self_weight': float(w_self)
    }
```

**Key Changes from Original:**
- ✅ **Removed gating** - trust already handles quality checks
- ✅ **Removed multi-factor scoring** - trust already incorporates all factors
- ✅ **Removed self-covariance** - use fixed weights
- ✅ **Kept geometry** - distance penalty still useful
- ✅ **Simplified normalization** - fixed w0 and w_self

**Test:** Create unit test to verify row-stochastic property.

---

### ✅ Task 2.4: Simplified Unit Tests

**File:** `test_weight_trust_simple.py` (new file)  
**Priority:** HIGH

**What to do:**

Create simplified unit tests:

```python
import numpy as np
import sys
sys.path.append('..')

from src.Weight.Weight_Trust_module import WeightTrustModule

def test_row_stochastic_simple():
    """Test that weights sum to 1 and are non-negative."""
    # Create simple 3-vehicle graph
    graph = np.array([
        [0, 1, 1],
        [1, 0, 1],
        [1, 1, 0]
    ])
    
    module = WeightTrustModule(graph, trust_threshold=0.5, kappa=3)
    
    # Simplified test case - only trust scores and distances
    result = module.calculate_weights_trust_v2(
        vehicle_index=0,
        trust_scores=np.array([1.0, 0.8, 0.6]),  # Self=1.0, high trust neighbor, medium trust neighbor
        distances=np.array([0.0, 5.0, 10.0])      # Self=0, close neighbor, far neighbor
    )
    
    weights = result['weights']
    
    # Check row-stochastic property
    assert np.isclose(np.sum(weights), 1.0), f"Sum = {np.sum(weights)}, expected 1.0"
    assert np.all(weights >= 0), "All weights must be non-negative"
    
    # Check vehicle 1 (closer + higher trust) gets more weight than vehicle 2
    assert weights[0, 2] > weights[0, 3], "Closer/higher trust neighbor should get more weight"
    
    print(f"✓ Row-stochastic test passed")
    print(f"  Weights: {weights}")
    print(f"  w0 (virtual): {result['virtual_weight']:.3f}")
    print(f"  w_self: {result['self_weight']:.3f}")
    print(f"  w1 (V1): {weights[0, 2]:.3f}")
    print(f"  w2 (V2): {weights[0, 3]:.3f}")
    print(f"  Used neighbors: {result['used_neighbors']}")


def test_low_trust_zero_weight():
    """Test that zero trust results in zero weight."""
    graph = np.array([
        [0, 1, 1],
        [1, 0, 1],
        [1, 1, 0]
    ])
    
    module = WeightTrustModule(graph, 0.5, 3)
    
    # Vehicle 2 has zero trust (malicious)
    result = module.calculate_weights_trust_v2(
        vehicle_index=0,
        trust_scores=np.array([1.0, 0.9, 0.0]),  # V2 has 0 trust
        distances=np.array([0.0, 5.0, 5.0])
    )
    
    weights = result['weights']
    
    # Vehicle 2 should have near-zero weight
    assert weights[0, 3] < 0.01, f"Zero trust should result in ~zero weight, got {weights[0, 3]}"
    
    # Weight should be redistributed
    assert np.isclose(np.sum(weights), 1.0), "Weights must still sum to 1"
    
    print(f"✓ Zero trust test passed")
    print(f"  V2 weight with 0 trust: {weights[0, 3]:.6f}")
    print(f"  Weights redistributed to: w0={result['virtual_weight']:.3f}, "
          f"w_self={result['self_weight']:.3f}, w1={weights[0, 2]:.3f}")


def test_distance_penalty():
    """Test that far neighbors get less weight."""
    graph = np.ones((3, 3)) - np.eye(3)  # Fully connected
    module = WeightTrustModule(graph, 0.5, 3)
    
    # Same trust, different distances
    result = module.calculate_weights_trust_v2(
        vehicle_index=0,
        trust_scores=np.array([1.0, 0.8, 0.8]),  # Same trust for V1 and V2
        distances=np.array([0.0, 5.0, 20.0])     # V2 is 4x farther
    )
    
    weights = result['weights']
    
    # V1 (closer) should get more weight than V2 (farther)
    assert weights[0, 2] > weights[0, 3], "Closer neighbor should get more weight"
    
    print(f"✓ Distance penalty test passed")
    print(f"  V1 (5m):  {weights[0, 2]:.4f}")
    print(f"  V2 (20m): {weights[0, 3]:.4f}")
    print(f"  Ratio: {weights[0, 2] / weights[0, 3]:.2f}x")


if __name__ == "__main__":
    print("=" * 60)
    print("SIMPLIFIED TRUST-BASED WEIGHT TESTS")
    print("=" * 60)
    print()
    
    test_row_stochastic_simple()
    print()
    
    test_low_trust_zero_weight()
    print()
    
    test_distance_penalty()
    print()
    
    print("=" * 60)
    print("✓ ALL TESTS PASSED!")
    print("=" * 60)
```

**Run:** `python test_weight_trust_simple.py`

---

### ✅ Task 2.5: Week 2 Testing Checklist

**Before moving to Week 3, verify:**

- [ ] `DEFAULT_WEIGHT_CONFIG` added to `Weight_Trust_module.py`
- [ ] `self.prev_weights = {}` added to `__init__`
- [ ] `calculate_weights_trust_v2()` compiles without errors
- [ ] Unit test: weights sum to 1.0 (row-stochastic)
- [ ] Unit test: all weights are non-negative
- [ ] Unit test: zero trust → near-zero weight
- [ ] Unit test: closer neighbors get higher weights
- [ ] No import errors or crashes

**Test command:**
```bash
cd src/Weight
python test_weight_trust_simple.py
```

---

### 📝 Week 2 Summary: Simplified Approach

**What we removed (already in trust evaluation):**
- ❌ Hard gating (age, innovation) → Trust already gates bad messages
- ❌ Multi-factor scoring (freshness, reliability, uncertainty) → Trust includes these
- ❌ Hysteresis state machine → Trust transitions are smooth
- ❌ Self-covariance adaptation → Simplified to fixed weights

**What we kept:**
- ✅ Trust scores (0-1 from Week 1)
- ✅ Geometry adjustment (distance penalty)
- ✅ Weight normalization (w0 + w_self + neighbors = 1)
- ✅ Influence capping (max 40% per neighbor)
- ✅ EMA smoothing (temporal stability)

**Benefits:**
- 🎯 **Simpler**: ~100 lines vs ~200 lines
- 🚀 **Faster**: No redundant computations
- 🔧 **Maintainable**: Clear separation of concerns
- ✅ **Effective**: Trust does the heavy lifting

---

## Week 3: Integration

*Will provide detailed tasks after Week 1-2 are complete.*

**Preview:**
- Modify `VehicleObserver._get_distributed_weights()` to call v2
- Pass `vehicle_process` reference to observer
- Update observer config to enable adaptive weights

---

## Week 4: Testing & Validation

*Will provide detailed scenarios after integration is complete.*

**Preview:**
- Byzantine attack scenario
- GPS denial scenario
- High packet loss scenario
- Performance benchmarking

---

## 📋 Implementation Checklist Summary

### Week 1: Start Here 👇

```
□ Task 1.1: Add covariance to broadcast         [VehicleProcess.py]
□ Task 1.2: Initialize quality tracking          [VehicleProcess.py __init__]
□ Task 1.3: Implement state prediction           [VehicleProcess.py new method]
□ Task 1.4: Compute innovation in handler        [VehicleProcess.py _handle_vehicle_state_direct]
□ Task 1.5: Aggregate quality metrics            [VehicleProcess.py new method]
□ Task 1.6: Test Week 1 deliverables
```

---

## 🎯 Quick Start: Next 30 Minutes

**Do these 3 things right now:**

1. **Read** `AdaptiveTrustWeight_Clarifications.md` - Section 1 (Virtual Node)
2. **Start** Task 1.1: Add covariance to broadcast
3. **Test** Task 1.1: Run simulation and check logs for `covariance_trace`

---

## 📞 Questions?

If you get stuck:
- **Virtual node confusion?** → Re-read Clarifications Section 1
- **Innovation not working?** → Check prediction method is called
- **Weights not summing to 1?** → Verify `w_self ≤ 1 - w0` constraint

---

**Ready to start?** Begin with Task 1.1! 🚀
