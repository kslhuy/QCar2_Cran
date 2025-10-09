# 🐛 Debug Guide: Local State Buffer Empty Issue

**Error:** `buffers=[local=0,fleet=20]`  
**Problem:** Local states not being buffered for trust evaluation  
**Impact:** Quality metrics cannot be computed, trust falls back to base calculation

---

## 🔍 Diagnosis

### Symptoms:
```
16:57:35.502 [ERROR] V1 [TRUST]: LOCAL_TRUST_EVAL: Error for vehicle 0: unsupported operand type(s) for *: 'float' and 'NoneType'
16:57:35.502 [INFO] V1 [TRUST]: TRUST_UPDATE: Vehicle 0 -> 0.956 (local=N/A, global=0.956, source=global_only, buffers=[local=0,fleet=20])
```

### What This Means:
- ✅ **Fleet estimates** from V0 ARE being received (20 buffers)
- ❌ **Local states** from V0 are NOT being buffered (0 buffers)
- ⚠️ **Quality factor** was `None` causing multiplication error (NOW FIXED)

---

## ✅ Fixes Applied

### 1. Fixed NoneType Error
**File:** `GraphBasedTrust.py`  
**Change:** Initialize `quality_factor = 1.0` BEFORE trying to get quality metrics

```python
# BEFORE (buggy):
quality_metrics = self._get_quality_metrics_for_vehicle(target_id)
if quality_metrics is not None:
    quality_factor = trust_model.evaluate_communication_quality(...)
else:
    quality_factor = 1.0  # Too late if exception happens before!

# AFTER (fixed):
quality_factor = 1.0  # Initialize first!
quality_metrics = self._get_quality_metrics_for_vehicle(target_id)
if quality_metrics is not None:
    quality_factor = trust_model.evaluate_communication_quality(...)
```

### 2. Added Debug Logging
**File:** `VehicleProcess.py`  
**Added logs to trace:**
- State queue add success/failure
- Trust buffering success
- Trust evaluator availability

---

## 🧪 Debug Steps

### Step 1: Run and Check Logs

```bash
python main.py
```

**Look for these specific messages from V1 (follower):**

#### A. State Reception:
```
[INFO] V1 [COMM]: STATE_RECV From=V0 Seq=X T=... Pos=(...) ...
```
✅ If you see this → V0 IS broadcasting states

#### B. Queue Addition:
```
[DEBUG] V1 [COMM]: State from V0 queue_add=True
```
- ✅ `True` → State accepted by queue
- ❌ `False` → State rejected (timing/validation issue)

#### C. Trust Buffering:
```
[DEBUG] V1 [COMM]: Buffered local state from V0 for trust eval
```
- ✅ If you see this → Buffering is working
- ❌ If you DON'T see this → Check conditions below

#### D. Trust Evaluator Status:
```
[DEBUG] V1 [COMM]: Cannot buffer - trust_evaluator not available
```
- If you see this → Trust evaluator not initialized properly

---

## 🔧 Possible Root Causes

### Cause 1: States Rejected by Queue ❌
**Symptom:** `queue_add=False`  
**Reason:** States might be "too old" or fail validation

**Solution:** Check `StateQueue` validation logic:
```python
# In StateQueue.add_state():
# - Check if GPS time sync is working
# - Check if timestamp validation is too strict
# - Check if sequence numbers are correct
```

### Cause 2: Trust Not Enabled for V1 ⚙️
**Symptom:** No buffering attempts at all  
**Check:** Vehicle 1 configuration

```python
# In config/vehicle_configs or main.py:
vehicles = [
    {'id': 0, 'trust_enabled': True, ...},  # Leader
    {'id': 1, 'trust_enabled': True, ...},  # Follower <- CHECK THIS!
]
```

### Cause 3: Connected Vehicles List Issue 📋
**Symptom:** Trust evaluator exists but V0 not in connected list  
**Check:** `connected_vehicles` parameter

```python
# In VehicleProcess.__init__:
self.connected_vehicles = vehicle_config.get('connected_vehicles', [])
# Should include 0 for V1

# In trust_evaluator.__init__:
# V0 should be in connected_vehicles list
```

### Cause 4: Buffering Happens Before Queue Success ⏱️
**Current Flow:**
```
receive_state
  → add_state (queue)
  → if success:
      → buffer_local_state (trust)  ← Only if queue accepts!
```

**If queue rejects:** Trust never sees the state!

**Solution:** Consider buffering BEFORE queue validation:
```python
# Buffer for trust BEFORE queue (trust has its own validation)
if self.trust_enabled and sender_id != self.vehicle_id:
    self.trust_evaluator.buffer_local_state(...)

# Then add to queue
success = self.state_queue.add_state(...)
```

---

## 🎯 Quick Diagnostic Commands

### Check if V0 broadcasts are being sent:
```bash
# Filter logs for V0 broadcasts
grep "Vehicle 0: Broadcasted state" logs/communication_vehicle_0.log
```

### Check if V1 receives V0 states:
```bash
# Filter logs for V1 receiving from V0
grep "STATE_RECV From=V0" logs/communication_vehicle_1.log
```

### Check trust buffer status:
```bash
# Filter trust evaluation logs
grep "buffers=" logs/trust_vehicle_1.log
```

---

## 🚀 Recommended Next Steps

### Option A: Move Buffering Before Queue (Recommended)

**Rationale:** Trust evaluation should see ALL received states, even if queue rejects them for timing reasons.

```python
# In _handle_vehicle_state_direct(), move this UP:

# Extract state data (existing code)
sender_id = received_state.get('vehicle_id', ...)
# ... extract pos, rot, vel, etc ...

# BUFFER FOR TRUST FIRST (before queue validation)
if self.trust_enabled and sender_id != self.vehicle_id:
    if hasattr(self, 'trust_evaluator') and self.trust_evaluator is not None:
        self.trust_evaluator.buffer_local_state(
            sender_id=sender_id,
            local_state=received_state,
            relative_distance=None,
            timestamp=timestamp
        )

# THEN add to queue (existing code)
success = self.state_queue.add_state(received_state, self.gps_sync)
```

### Option B: Debug Queue Rejection

Check why states are being rejected:

```python
# In StateQueue.add_state(), add logging:
if not self._validate_state(state):
    logger.debug(f"State rejected - validation failed: {reason}")
    return False
```

---

## 📊 Expected Behavior After Fix

### Before Fix:
```
buffers=[local=0,fleet=20]
LOCAL_TRUST: N/A (no data)
trust_score = global_only
```

### After Fix:
```
buffers=[local=20,fleet=20]
QUALITY_FACTOR: V0 = 0.850 (age=0.10s, drop=0.00, cov=0.450, innov=0.15m)
LOCAL_TRUST: V0 -> 0.920 (v=0.980, d=0.990, q=0.850, ...)
trust_score = 0.920 (combined local+global)
```

---

## 💡 Summary

**Immediate Issue:** ✅ FIXED - `NoneType` multiplication error  
**Root Cause:** ⏳ INVESTIGATING - Local states not being buffered  
**Most Likely:** States rejected by queue OR buffering happens in wrong order  
**Recommended Fix:** Move trust buffering BEFORE queue validation  

---

**Next Action:** Run simulation with new debug logs and check which scenario matches! 🔍
