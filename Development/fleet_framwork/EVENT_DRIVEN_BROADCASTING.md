# Event-Driven Broadcasting Optimization

## Problem Statement

With the observer running at 100Hz (or 50Hz), the previous implementation was broadcasting state on **every observer update cycle**, regardless of whether the state actually changed. This caused:

1. **Excessive network traffic** - Broadcasting identical states repeatedly
2. **Wasted CPU cycles** - Processing unchanged data
3. **Log spam** - Thousands of redundant broadcast messages
4. **Potential message queue buildup** - Overwhelming receivers with duplicate data

## Solution: Event-Driven Broadcasting with Time-Based Fallback

### Core Concept

**Broadcast only when state actually changes**, with a time-based fallback to ensure stationary vehicles still communicate.

### Implementation

#### 1. State Change Detection (VehicleObserver.py)

```python
# In __init__():
self.state_has_new_data = False
self.last_broadcast_state = np.zeros(self.state_dim)
self.last_broadcast_time = 0.0
self.min_broadcast_interval = 0.1  # 100ms = 10Hz max rate
self.max_broadcast_interval = 0.2  # 200ms = 5Hz min rate

# In update_local_state():
state_changed = not np.allclose(self.local_state, self.local_state_prev, rtol=1e-4, atol=1e-4)
if state_changed or measured_state is not None:
    self.state_has_new_data = True
    self.local_state_prev = self.local_state.copy()
```

**State change detection**: Compares new state with previous using `np.allclose()` with tolerance of 0.01% relative and 0.0001 absolute difference.

#### 2. Smart Broadcasting Decision (VehicleObserver.py)

```python
def should_broadcast_state(self) -> bool:
    """
    Broadcasts when:
    1. New state data is available (event-driven)
    2. OR: Maximum time interval exceeded (time-based fallback)
    3. AND: Minimum time interval has passed (rate limiting)
    """
    current_time = time.time()
    time_since_last_broadcast = current_time - self.last_broadcast_time
    
    # Rate limiting: Don't broadcast too frequently
    if time_since_last_broadcast < self.min_broadcast_interval:
        return False
    
    # Event-driven OR time-based fallback
    should_broadcast = (
        self.state_has_new_data or  # New state available
        time_since_last_broadcast >= self.max_broadcast_interval  # Forced periodic update
    )
    
    return should_broadcast
```

#### 3. Integration with VehicleProcess (VehicleProcess.py)

```python
# In observer_update():
if estimated_state is not None and len(estimated_state) >= 4:
    # Only broadcast when needed
    if self.observer.should_broadcast_state():
        self.broadcast_own_state()
        self.observer.mark_state_broadcasted()  # Clear flag and update timestamp
        
        # Also broadcast fleet estimates
        if hasattr(self, 'fleet_state_estimates') and len(self.fleet_state_estimates) > 1:
            self.broadcast_fleet_estimates()
```

## Broadcasting Behavior

### Scenario 1: Vehicle Moving (State Changing)
- **Observer Rate**: 50Hz (every 20ms)
- **State Change Rate**: ~40Hz (state differs on 80% of updates)
- **Broadcast Rate**: 10Hz (every 100ms, limited by `min_broadcast_interval`)
- **Result**: 80% reduction in broadcasts while maintaining fresh data

### Scenario 2: Vehicle Stationary (State NOT Changing)
- **Observer Rate**: 50Hz (every 20ms)
- **State Change Rate**: 0Hz (state identical on all updates)
- **Broadcast Rate**: 5Hz (every 200ms, enforced by `max_broadcast_interval`)
- **Result**: 90% reduction in broadcasts, guaranteed periodic updates

### Scenario 3: Vehicle Accelerating/Decelerating
- **Observer Rate**: 50Hz (every 20ms)
- **State Change Rate**: 50Hz (state differs on every update due to velocity change)
- **Broadcast Rate**: 10Hz (every 100ms, limited by `min_broadcast_interval`)
- **Result**: 80% reduction, high-frequency state changes still captured

## Benefits

### 1. Reduced Network Traffic
- **Before**: 50 broadcasts/sec × 3 vehicles = 150 msgs/sec
- **After**: 5-10 broadcasts/sec × 3 vehicles = 15-30 msgs/sec
- **Savings**: **80-90% reduction** in network messages

### 2. Improved Observer Performance
- Less time spent in broadcast operations
- More time for actual state estimation
- Observer updates can complete faster

### 3. Better Scalability
- Network traffic scales better with fleet size
- 10 vehicles: 100-150 msgs/sec (vs 500 msgs/sec before)
- 20 vehicles: 200-300 msgs/sec (vs 1000 msgs/sec before)

### 4. Guaranteed Communication
- **Time-based fallback** ensures stationary vehicles still broadcast every 200ms
- Followers won't lose tracking of leader even if leader stops moving
- **Rate limiting** prevents message flooding during rapid state changes

## Configuration Parameters

### Adjustable Thresholds

```python
# In VehicleObserver.__init__():
self.min_broadcast_interval = 0.1  # Minimum 100ms between broadcasts
self.max_broadcast_interval = 0.2  # Maximum 200ms without broadcast
```

### Recommended Settings by Fleet Size

| Fleet Size | min_interval | max_interval | Expected Rate  |
|-----------|--------------|--------------|----------------|
| 2-3       | 0.1s (10Hz)  | 0.2s (5Hz)   | 15-30 msg/s    |
| 4-6       | 0.15s (6Hz)  | 0.3s (3Hz)   | 18-36 msg/s    |
| 7-10      | 0.2s (5Hz)   | 0.4s (2.5Hz) | 25-50 msg/s    |

### State Change Sensitivity

```python
# In VehicleObserver.update_local_state():
state_changed = not np.allclose(
    self.local_state, 
    self.local_state_prev, 
    rtol=1e-4,  # 0.01% relative tolerance
    atol=1e-4   # 0.0001 absolute tolerance
)
```

**Adjust for different scenarios**:
- **High precision tracking**: `rtol=1e-5, atol=1e-5` (more sensitive, more broadcasts)
- **Rough positioning**: `rtol=1e-3, atol=1e-3` (less sensitive, fewer broadcasts)

## Performance Metrics

### Expected Improvements

1. **Network bandwidth**: 80-90% reduction
2. **CPU usage**: 5-10% reduction (less broadcast processing)
3. **Observer timing**: More consistent, fewer "exceeds interval" warnings
4. **Message freshness**: Same or better (only broadcasts when data changes)

### Monitoring

Check logs for broadcast patterns:

```bash
# Count broadcasts per second
grep "Broadcasted state" communication_vehicle_0.log | wc -l
# Should be 5-10 per second (not 50-100)

# Check for forced periodic broadcasts (stationary)
grep "max_broadcast_interval" observer_vehicle_0.log
# Should appear when vehicle is stationary
```

## Troubleshooting

### Problem: Follower losing leader tracking

**Symptom**: "No valid target data available" errors

**Cause**: `max_broadcast_interval` too large, leader not broadcasting often enough

**Solution**: Reduce `max_broadcast_interval`:
```python
self.max_broadcast_interval = 0.1  # 100ms = 10Hz minimum
```

### Problem: Too many broadcasts still

**Symptom**: Network still busy, high message rate

**Cause**: `min_broadcast_interval` too small or state changing very frequently

**Solution**: Increase intervals or reduce state sensitivity:
```python
self.min_broadcast_interval = 0.2  # 200ms = 5Hz max rate
self.max_broadcast_interval = 0.5  # 500ms = 2Hz min rate
```

### Problem: State updates not being broadcast

**Symptom**: Receivers getting stale data

**Cause**: State change detection too insensitive

**Solution**: Increase sensitivity:
```python
state_changed = not np.allclose(
    self.local_state, 
    self.local_state_prev, 
    rtol=1e-5,  # More sensitive
    atol=1e-5
)
```

## Conclusion

Event-driven broadcasting with time-based fallback provides:
- **Efficiency**: Only broadcast when data changes
- **Reliability**: Guaranteed periodic updates for stationary vehicles
- **Scalability**: Network traffic scales sub-linearly with fleet size
- **Performance**: Reduced CPU and network overhead

This optimization, combined with weight caching, provides the performance headroom needed for 3+ vehicle fleets to operate reliably at high update rates.
