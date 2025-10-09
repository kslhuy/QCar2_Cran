# Revised Integration Plan: Quality Metrics into Trust System

**Date:** October 5, 2025  
**Insight:** You're right - we should enhance EXISTING trust calculation, not create separate weight system!

---

## ✅ Your Correct Understanding

1. **Fleet topology is known**: V0 (leader) → V1 → V2 → V3 (chain)
2. **Trust calculation exists**: `GraphBasedTrustEvaluator` + `TriPTrustModel`
3. **Quality metrics should ENHANCE trust**: Age, covariance, drop_rate, innovation → feed into trust
4. **Distance calculation redundant**: Fleet position known from vehicle IDs

---

## 🎯 Revised Strategy

### OLD (Week 1-4 roadmap) ❌
- Create separate `WeightTrustModule` 
- Calculate weights independently from trust
- Complex integration with observer

### NEW (Simplified) ✅
- **Enhance `TriPTrustModel`** with quality factor
- **Feed quality metrics** to `GraphBasedTrustEvaluator`
- **Use enhanced trust directly** in observer weights

---

## 📋 Simplified Tasks (Only 3 Steps!)

### ✅ Task 1: Add Quality Metrics Collection (KEEP from Week 1)

**What we already did (GOOD!):**
- ✅ Broadcast covariance in `broadcast_own_state()`
- ✅ Initialize `neighbor_quality` dict in `__init__()`
- ✅ Compute innovation in `_handle_vehicle_state_direct()`
- ✅ Track message age, drop rate

**These are correct - we need this data!**

---

### 🆕 Task 2: Enhance TriPTrustModel with Quality Factor

**File:** `TriPTrustModel.py`  
**Add new method:**

```python
def evaluate_communication_quality(self, message_age: float, drop_rate: float, 
                                   covariance: float, innovation: float) -> float:
    """
    Evaluate communication quality factor for trust calculation.
    
    This factor reduces trust when:
    - Messages are stale (high age)
    - Packet loss is high (high drop_rate)
    - Uncertainty is high (high covariance)
    - Prediction error is high (high innovation)
    
    Args:
        message_age: Time since last message (seconds)
        drop_rate: Packet loss rate [0, 1]
        covariance: Covariance trace (uncertainty)
        innovation: Prediction error (meters)
        
    Returns:
        Quality factor [0, 1] where 1 = perfect quality
    """
    # Age penalty (exponential decay)
    age_factor = np.exp(-2.0 * message_age)  # 2.0 = decay rate
    
    # Drop rate penalty (linear)
    reliability_factor = 1.0 - drop_rate
    
    # Covariance penalty (inverse relationship)
    uncertainty_factor = 1.0 / (1.0 + 0.2 * covariance)  # 0.2 = penalty weight
    
    # Innovation penalty (prediction accuracy)
    consistency_factor = np.exp(-0.5 * innovation)  # 0.5 = sensitivity
    
    # Combine all factors (multiplicative - any bad factor reduces trust)
    quality_factor = age_factor * reliability_factor * uncertainty_factor * consistency_factor
    
    # Log components for debugging
    self.age_factor_log.append(age_factor)
    self.reliability_factor_log.append(reliability_factor)
    self.uncertainty_factor_log.append(uncertainty_factor)
    self.consistency_factor_log.append(consistency_factor)
    
    return quality_factor
```

**Add logging arrays in `__init__`:**

```python
# Quality factor logs
self.age_factor_log = []
self.reliability_factor_log = []
self.uncertainty_factor_log = []
self.consistency_factor_log = []
self.quality_factor_log = []
```

---

### 🆕 Task 3: Integrate Quality into GraphBasedTrust

**File:** `GraphBasedTrust.py`  
**Modify:** `_evaluate_local_trust()` method

**Current calculation (line ~360):**
```python
# Calculate final local trust sample
local_trust_sample = trust_model.calculate_trust_sample(
    v_score, d_score, a_score, beacon_score, h_score, is_nearby
)
```

**NEW calculation (enhanced with quality):**
```python
# NEW: Get quality metrics from VehicleProcess
quality_metrics = self._get_quality_metrics_for_vehicle(target_id)

if quality_metrics is not None:
    # Evaluate communication quality factor
    quality_factor = trust_model.evaluate_communication_quality(
        message_age=quality_metrics.get('age', 0.0),
        drop_rate=quality_metrics.get('drop_rate', 0.0),
        covariance=quality_metrics.get('covariance', 1.0),
        innovation=quality_metrics.get('innovation', 0.0)
    )
    
    # Log quality factor
    if logger:
        logger.debug(f"V{self.vehicle_id}→V{target_id}: Quality factor = {quality_factor:.3f} "
                    f"(age={quality_metrics['age']:.2f}s, drop={quality_metrics['drop_rate']:.2f}, "
                    f"cov={quality_metrics['covariance']:.3f}, innov={quality_metrics['innovation']:.3f}m)")
else:
    quality_factor = 1.0  # Default if no quality data

# Calculate final local trust sample WITH quality
local_trust_sample = trust_model.calculate_trust_sample(
    v_score, d_score, a_score, beacon_score, h_score, is_nearby
)

# Apply quality factor to trust sample
local_trust_sample = local_trust_sample * quality_factor
```

**Add helper method to GraphBasedTrustEvaluator:**

```python
def set_vehicle_process_reference(self, vehicle_process):
    """
    Set reference to VehicleProcess for accessing quality metrics.
    Call this after creating GraphBasedTrustEvaluator.
    """
    self.vehicle_process = vehicle_process

def _get_quality_metrics_for_vehicle(self, target_id: int) -> Optional[Dict]:
    """Get quality metrics from VehicleProcess."""
    if not hasattr(self, 'vehicle_process') or self.vehicle_process is None:
        return None
    
    if not hasattr(self.vehicle_process, 'neighbor_quality'):
        return None
    
    if target_id not in self.vehicle_process.neighbor_quality:
        return None
    
    q = self.vehicle_process.neighbor_quality[target_id]
    current_time = time.time()
    
    # Aggregate metrics
    age = current_time - q['last_recv_time'] if q['last_recv_time'] > 0 else 1.0
    
    # Drop rate calculation
    elapsed = current_time - self.vehicle_process.comm_start_time
    if elapsed > 1.0:
        expected = elapsed * (self.vehicle_process.update_rate / 10)
        actual = q['recv_count']
        drop_rate = max(0.0, min(1.0, (expected - actual) / max(expected, 1)))
    else:
        drop_rate = 0.0
    
    covariance = q.get('last_covariance', 1.0)
    
    innovation = np.mean(q['recent_innovations']) if len(q['recent_innovations']) > 0 else 0.0
    
    return {
        'age': age,
        'drop_rate': drop_rate,
        'covariance': covariance,
        'innovation': innovation
    }
```

---

### 🆕 Task 4: Connect in VehicleProcess

**File:** `VehicleProcess.py`  
**In `__init__` after creating trust_evaluator:**

```python
# After trust_evaluator initialization (around line 495):
if self.trust_evaluator is not None:
    # Set reference for quality metrics access
    self.trust_evaluator.set_vehicle_process_reference(self)
    self.logger.info(f"Vehicle {self.vehicle_id}: Connected quality metrics to trust evaluator")
```

---

## 🔧 What to REMOVE/SIMPLIFY

### Remove from Week 1 tasks:
- ❌ `_compute_distance_from_states()` - Not needed (fleet topology known)
- ❌ Distance calculation in `_get_neighbor_quality_metrics()` - Remove distance field
- ⚠️ Keep age, drop_rate, covariance, innovation - These are essential!

### Simplify `_get_neighbor_quality_metrics()`:

**OLD (complex):**
```python
def _get_neighbor_quality_metrics(self) -> Dict[str, np.ndarray]:
    # Returns arrays for ALL neighbors with distances...
```

**NEW (simple - just for single vehicle):**
```python
def get_quality_metrics_for_vehicle(self, vehicle_id: int) -> Optional[Dict]:
    """
    Get quality metrics for a specific vehicle.
    Used by trust evaluator.
    """
    if not hasattr(self, 'neighbor_quality') or vehicle_id not in self.neighbor_quality:
        return None
    
    q = self.neighbor_quality[vehicle_id]
    current_time = time.time()
    
    return {
        'age': current_time - q['last_recv_time'] if q['last_recv_time'] > 0 else 1.0,
        'drop_rate': self._compute_drop_rate(vehicle_id),
        'covariance': q.get('last_covariance', 1.0),
        'innovation': np.mean(q['recent_innovations']) if len(q['recent_innovations']) > 0 else 0.0
    }

def _compute_drop_rate(self, vehicle_id: int) -> float:
    """Compute packet drop rate for a vehicle."""
    if vehicle_id not in self.neighbor_quality:
        return 0.5
    
    q = self.neighbor_quality[vehicle_id]
    elapsed = time.time() - self.comm_start_time
    
    if elapsed > 1.0:
        expected = elapsed * (self.update_rate / 10)
        actual = q['recv_count']
        return max(0.0, min(1.0, (expected - actual) / max(expected, 1)))
    
    return 0.0
```

---

## 📊 How It Works (Complete Flow)

```
1. Vehicle receives message → VehicleProcess._handle_vehicle_state_direct()
   ├─ Update neighbor_quality (age, recv_count, innovation, covariance)
   └─ Buffer state → trust_evaluator.buffer_local_state()

2. Periodic trust update → trust_evaluator.update_all_trust_scores()
   ├─ For each vehicle:
   │   └─ _evaluate_local_trust()
   │       ├─ Get v_score, d_score, a_score (existing TriPTrust)
   │       ├─ NEW: Get quality_factor from quality metrics
   │       └─ local_trust = trust_sample * quality_factor
   └─ Store trust scores

3. Observer uses trust → VehicleObserver._get_distributed_weights()
   ├─ Get trust scores from trust_evaluator
   └─ Use as weights directly (already normalized)
```

---

## ✅ Final Implementation Steps (Simplified)

### Step 1: Enhance TriPTrustModel ⏱️ 15 min
```bash
# Add evaluate_communication_quality() method
# Add logging arrays
```

### Step 2: Modify GraphBasedTrust ⏱️ 20 min
```bash
# Add set_vehicle_process_reference()
# Add _get_quality_metrics_for_vehicle()
# Modify _evaluate_local_trust() to use quality factor
```

### Step 3: Connect in VehicleProcess ⏱️ 5 min
```bash
# Call trust_evaluator.set_vehicle_process_reference(self)
# Already have quality tracking from Week 1 ✅
```

### Step 4: Remove unnecessary code ⏱️ 10 min
```bash
# Remove _compute_distance_from_states()
# Simplify _get_neighbor_quality_metrics() or remove
```

### Step 5: Test ⏱️ 30 min
```bash
python main.py
# Check logs for quality_factor values
# Verify trust scores change with packet loss/delays
```

---

## 🎯 Summary

**Key Insight:** Don't create separate weight system - enhance existing trust!

**What we keep from Week 1:**
- ✅ Covariance broadcasting
- ✅ Quality tracking (age, drop_rate, innovation)
- ✅ Innovation computation

**What we simplify:**
- ❌ Remove distance calculation (fleet topology known)
- ❌ No separate WeightTrustModule needed
- ✅ Quality → Trust → Weights (direct pipeline)

**Benefits:**
- 🚀 Simpler (3 steps vs 4 weeks)
- 🎯 Uses existing trust framework
- 💪 Less code, fewer bugs
- ✅ Trust scores already normalized for observer weights

---

**Ready to implement?** Let's start with Step 1 (TriPTrustModel enhancement).
