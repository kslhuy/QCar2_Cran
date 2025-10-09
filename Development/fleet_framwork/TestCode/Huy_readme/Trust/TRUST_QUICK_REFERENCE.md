# Quick Reference: Hybrid Buffered Trust Evaluation

## 🎯 What Changed?

### Before (Immediate Evaluation)
```python
# Every time a state was received:
_handle_vehicle_state_direct()
  └─> _evaluate_trust_for_received_state()
      └─> trust_model.evaluate_velocity()  # Expensive
      └─> trust_model.calculate_trust()     # Every message!
```

### After (Buffered Evaluation)
```python
# When state received (fast):
_handle_vehicle_state_direct()
  └─> buffer_local_state()  # O(1) append to deque

# Every 0.5 seconds (batch):
run() main loop
  └─> update_all_trust_scores()
      └─> For each vehicle: _evaluate_trust_from_buffers()
```

---

## 📦 What to Import

```python
from GraphBasedTrust import GraphBasedTrustEvaluator, FleetStateSnapshot
from src.Trust.TriPTrustModel import TriPTrustModel
```

---

## 🔧 Configuration

Add to your vehicle config file:
```yaml
enable_trust_evaluation: true
trust_update_interval: 0.5    # Seconds between trust calculations
trust_buffer_size: 20         # Max samples per vehicle
```

Or in Python dict:
```python
vehicle_config = {
    'enable_trust_evaluation': True,
    'trust_update_interval': 0.5,
    'trust_buffer_size': 20,
    # ... other config
}
```

---

## 🚀 Quick Start

### 1. Initialization (Already done in VehicleProcess.__init__)
```python
self.trust_evaluator = GraphBasedTrustEvaluator(
    vehicle_id=self.vehicle_id,
    connected_vehicles=[0, 1, 3],
    trust_models={0: TriPTrustModel(), 1: TriPTrustModel(), 3: TriPTrustModel()},
    trust_update_interval=0.5,
    max_buffer_size=20
)
```

### 2. Buffer States (Already integrated)
```python
# Automatically happens when states are received
# No manual intervention needed
```

### 3. Get Trust Scores
```python
# Get current trust score for a vehicle
trust_score = self.trust_evaluator.get_trust_score(vehicle_id)

# Check if vehicle is trusted
is_trusted = self.trust_evaluator.is_vehicle_trusted(vehicle_id, threshold=0.7)

# Get all trust scores
all_scores = self.trust_evaluator.get_all_trust_scores()
# Returns: {0: 0.95, 1: 0.87, 3: 0.92}
```

---

## 🎭 Advanced: Immediate Evaluation

For critical situations where you need instant trust feedback:

```python
# Emergency detection scenario
if suspicious_behavior_detected(received_state):
    trust_score = self.trust_evaluator.evaluate_trust_immediate(
        sender_id=sender_id,
        received_state=received_state,
        our_state=self.get_best_available_state(),
        logger=self.trust_logger,
        relative_distance=sensor_distance
    )
    
    if trust_score < 0.3:
        # Take immediate action
        emergency_stop()
        broadcast_warning()
```

---

## 📊 Monitoring Trust

### Check Buffer Status
```python
# Get buffer info for debugging
local_buffer = self.trust_evaluator.local_state_buffer[vehicle_id]
fleet_buffer = self.trust_evaluator.fleet_state_buffer[vehicle_id]

print(f"Local buffer size: {len(local_buffer)}/{local_buffer.maxlen}")
print(f"Fleet buffer size: {len(fleet_buffer)}/{fleet_buffer.maxlen}")
```

### Trust Statistics
```python
stats = self.trust_evaluator.get_trust_statistics()
print(f"Trust stats: count={stats['count']}, "
      f"mean={stats['mean']:.3f}, "
      f"min={stats['min']:.3f}, "
      f"max={stats['max']:.3f}")
```

### Log Trust Summary
```python
# Manually trigger trust summary log
self.trust_evaluator.log_trust_summary(self.trust_logger)
# Output: "TRUST_SUMMARY: Connected vehicles trust scores: V0=0.950, V1=0.870, V3=0.920"
```

---

## ⚙️ Tuning Parameters

### Adjust Update Interval (Globally)
```python
# In vehicle config
'trust_update_interval': 0.2  # Faster (more CPU, quicker response)
'trust_update_interval': 1.0  # Slower (less CPU, delayed response)
```

### Adjust Buffer Size
```python
# In vehicle config
'trust_buffer_size': 10   # Less smoothing, faster adaptation
'trust_buffer_size': 50   # More smoothing, slower adaptation
```

### Runtime Adjustment (Advanced)
```python
# Change update interval at runtime
self.trust_evaluator.trust_update_interval = 0.2  # Switch to fast mode

# Change buffer size (requires reinit)
self.trust_evaluator.max_buffer_size = 30
self.trust_evaluator.local_state_buffer[vid] = deque(maxlen=30)
```

---

## 🐛 Common Issues

### Issue 1: Trust scores not updating
```python
# Check if trust evaluation is enabled
print(f"Trust enabled: {self.trust_enabled}")

# Check if evaluator exists
print(f"Evaluator: {hasattr(self, 'trust_evaluator')}")

# Check if interval has passed
print(f"Should update: {self.trust_evaluator.should_update_trust()}")

# Check buffer contents
for vid in self.connected_vehicles:
    local_buf = self.trust_evaluator.local_state_buffer.get(vid, deque())
    print(f"V{vid} local buffer: {len(local_buf)} samples")
```

### Issue 2: Trust always 1.0
```python
# Check if states are being buffered
if len(self.trust_evaluator.local_state_buffer[vehicle_id]) == 0:
    print("ERROR: No states buffered!")
    # Verify _handle_vehicle_state_direct is called
    # Verify buffer_local_state is called
```

### Issue 3: High latency in trust updates
```python
# Reduce update interval
self.trust_evaluator.trust_update_interval = 0.1  # Update every 100ms

# Or use immediate evaluation
trust = self.trust_evaluator.evaluate_trust_immediate(...)
```

---

## 📝 Logging Examples

### Trust Update Log
```
TRUST_UPDATE: Vehicle 1 -> 0.870 (v=0.950, d=0.920, dist_recv=7.8m, dist_meas=7.9m [sensor], source=local, buffers=[local=10,fleet=2])
```

**Interpretation:**
- Vehicle 1 trust score: 0.870
- Velocity score: 0.950 (good agreement)
- Distance score: 0.920 (good agreement)
- Received distance: 7.8m (from V1's message)
- Measured distance: 7.9m (from our sensors)
- Source: local state (not fleet estimate)
- 10 local samples + 2 fleet samples buffered

### Trust Summary Log
```
TRUST_SUMMARY: Connected vehicles trust scores: V0=0.950, V1=0.870, V3=0.920
```

### Buffer Log
```
TRUST: No buffered data for vehicle 2
```
Means: Vehicle 2 hasn't sent any states yet.

---

## 🎯 Best Practices

### 1. Use Buffered by Default
```python
# ✅ Good: Let the system handle it automatically
# States are buffered automatically when received
# Trust calculated at optimal intervals
```

### 2. Use Immediate Only When Needed
```python
# ❌ Bad: Immediate evaluation for every message
for state in received_states:
    trust = evaluate_trust_immediate(...)  # Expensive!

# ✅ Good: Immediate only for critical checks
if is_emergency_situation:
    trust = evaluate_trust_immediate(...)
```

### 3. Monitor Trust Trends
```python
# Keep track of trust over time
self.trust_history[vehicle_id].append((time.time(), trust_score))

# Detect degrading trust
if trust_score < last_trust_score - 0.2:
    logger.warning(f"Trust dropping for V{vehicle_id}!")
```

### 4. Trust-Based Control
```python
# Adjust behavior based on trust
trust = self.trust_evaluator.get_trust_score(self.following_target)

if trust < 0.5:
    following_distance *= 1.5  # Increase distance
    fusion_weight *= 0.5        # Reduce weight in fusion
elif trust > 0.9:
    following_distance *= 0.9  # Can get closer
```

---

## 🔗 Related Files

- **Implementation**: `GraphBasedTrust.py`
- **Integration**: `VehicleProcess.py`
- **Trust Model**: `src/Trust/TriPTrustModel.py`
- **Documentation**: `TRUST_IMPLEMENTATION_SUMMARY.md`

---

## 💡 Quick Tips

1. **Normal operation**: Default settings work well (0.5s interval, 20 buffer)
2. **High security**: Use 0.2s interval, 30 buffer, lower threshold
3. **Low CPU**: Use 1.0s interval, 15 buffer
4. **Debugging**: Enable trust logs, check buffer sizes
5. **Testing**: Artificially inject false data to verify detection

---

## 📞 Support

Check logs in:
- `logs/trust_vehicle_X.log` - Trust evaluation logs
- `logs/communication_vehicle_X.log` - State reception logs
- `logs/control_vehicle_X.log` - Control adjustments

---

**Last Updated**: October 2, 2025  
**Quick Reference Version**: 1.0
