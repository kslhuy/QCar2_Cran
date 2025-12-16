# Multi-Probing System - Technical Documentation

## Overview

The multi-probing system allows multiple QCar vehicles to stream their YOLO object detection results to separate observer windows on the ground station PC. Each vehicle that has `probing: true` in the configuration will have its own dedicated window.

## Architecture

### Components

1. **multi_probing.py** - Ground station observer manager
   - Creates separate observer windows for each probing vehicle
   - Each window has a unique Display ID matching the car_id
   - Runs in separate threads for concurrent streaming

2. **yolo_server.py** - Vehicle-side YOLO processor (runs on each QCar)
   - Performs object detection using YOLOv8
   - Sends detection results to ground station
   - Only runs when `probing: true` for that vehicle

3. **start_enhanced.py** - Fleet startup script
   - Automatically launches multi_probing for all probing vehicles
   - Passes correct car IDs and image configuration

## How It Works

### 1. Configuration
In `fleet_config.yaml`:
```yaml
image_size:
  width: 320
  height: 200

vehicles:
  - car_id: 0
    ip: 192.168.2.137
    probing: true      # This vehicle will send YOLO stream
    
  - car_id: 1
    ip: 192.168.2.108
    probing: false     # This vehicle won't send YOLO stream
```

### 2. Startup Sequence

When you run `start_enhanced.bat`:

1. **Start each vehicle** with `vehicle_main.py`
2. **Start YOLO servers** on each vehicle
   - If `probing: true` → yolo_server sends stream to ground station
   - If `probing: false` → yolo_server runs locally only (no streaming)
3. **Start multi_probing.py** on ground station
   - Creates one observer window per probing vehicle
   - Each window listens for its specific car_id

### 3. Display ID Mapping

The key to multi-vehicle probing is the Display ID system:

```python
# In multi_probing.py
observer.numDisplays = car_id  # Car 0 → Display 0, Car 1 → Display 1, etc.
```

Each QCar's yolo_server uses its car_id to send to the correct display:
- Car 0 → Display 0 → Window "QCar 0 YOLO"
- Car 1 → Display 1 → Window "QCar 1 YOLO"  
- Car 2 → Display 2 → Window "QCar 2 YOLO"

## Command Examples

### Manual Testing

Test with single vehicle:
```bash
cd python
python multi_probing.py --cars 0 --width 320 --height 200
```

Test with multiple vehicles:
```bash
python multi_probing.py --cars 0 1 2 --width 320 --height 200
```

### Integration with Start Script

The start script automatically generates the correct command:

```python
# In start_enhanced.py
probing_car_ids = [str(v['car_id']) for v in probing_vehicles]
cmd = ["python", "multi_probing.py", "--cars"] + probing_car_ids + ["--width", "320", "--height", "200"]
```

For config with Car 0 and Car 2 probing, this generates:
```bash
python multi_probing.py --cars 0 2 --width 320 --height 200
```

## Multi-Vehicle Scenarios

### Scenario 1: Single Probing Vehicle
```yaml
vehicles:
  - car_id: 0
    probing: true   # Only Car 0 sends YOLO stream
  - car_id: 1
    probing: false
```
**Result**: 1 observer window for Car 0

### Scenario 2: Multiple Probing Vehicles
```yaml
vehicles:
  - car_id: 0
    probing: true   # Car 0 sends YOLO stream
  - car_id: 1
    probing: true   # Car 1 sends YOLO stream
  - car_id: 2
    probing: false  # Car 2 doesn't send stream
```
**Result**: 2 observer windows (Car 0 and Car 1)

### Scenario 3: All Probing
```yaml
vehicles:
  - car_id: 0
    probing: true
  - car_id: 1
    probing: true
  - car_id: 2
    probing: true
```
**Result**: 3 observer windows (all vehicles)

## Threading Model

Each observer window runs in its own thread:

```python
thread = threading.Thread(
    target=launch_observer,
    args=(observer, car_id),
    daemon=True,
    name=f"Observer-Car{car_id}"
)
```

Benefits:
- Non-blocking: Each window operates independently
- Concurrent streaming: Multiple vehicles can send data simultaneously
- Fault tolerance: If one window fails, others continue

## Troubleshooting

### No Observer Windows Appear

**Check 1**: Verify probing configuration
```bash
python python/test_multiprobing_config.py
```

**Check 2**: Ensure vehicles have `probing: true`
```yaml
vehicles:
  - car_id: 0
    probing: true  # Must be true!
```

**Check 3**: Check if multi_probing.py is running
```bash
# Windows
tasklist | findstr python

# Look for process running multi_probing.py
```

### Wrong Vehicle in Wrong Window

**Symptom**: Car 1's video appears in Car 0's window

**Cause**: Display ID mismatch

**Fix**: Verify that car_id in config matches actual vehicle ID

### Observer Window Freezes

**Cause**: Network issues or vehicle stopped sending

**Fix**: 
1. Check vehicle is still running: `ssh nvidia@<ip> "pgrep -f yolo_server"`
2. Restart multi_probing: Kill process and rerun start_enhanced.bat

### Multiple Vehicles Show Same Stream

**Cause**: Vehicles have same car_id

**Fix**: Ensure each vehicle has unique car_id in fleet_config.yaml

## Performance Considerations

### Image Size
- Default: 320x200 (balanced)
- Smaller (160x120): Less bandwidth, faster but lower quality
- Larger (640x480): Higher quality but more bandwidth

```yaml
image_size:
  width: 640   # Increase for better quality
  height: 480
```

### Number of Probing Vehicles
- 1 vehicle: ~5-10 Mbps bandwidth
- 2 vehicles: ~10-20 Mbps bandwidth  
- 3+ vehicles: Consider network capacity

**Recommendation**: Only enable probing on leader vehicles or vehicles that need obstacle detection.

## Testing Checklist

- [ ] Configuration test passes: `python test_multiprobing_config.py`
- [ ] Multi-probing starts without errors
- [ ] Correct number of observer windows appear
- [ ] Each window shows correct vehicle ID in title
- [ ] YOLO detections appear in correct windows
- [ ] All windows update smoothly (no freezing)
- [ ] Stopping (Ctrl+C) cleanly terminates all windows

## Code References

### Key Files
- `python/multi_probing.py` - Observer manager
- `python/start_enhanced.py` - Fleet startup (calls multi_probing)
- `qcar/Yolo/yolo_server.py` - Vehicle-side YOLO processor
- `fleet_config.yaml` - Configuration file

### Key Parameters
- `observer.numDisplays = car_id` - Maps vehicle to display
- `--cars` - List of car IDs to observe
- `--width`, `--height` - Image resolution
- `probing: true/false` - Enable/disable per vehicle

## Future Enhancements

1. **Dynamic Vehicle Addition**: Add vehicles while multi_probing is running
2. **Window Layout**: Automatic tiling of multiple windows
3. **Recording**: Save YOLO streams to video files
4. **Performance Metrics**: Display FPS and latency per vehicle
5. **Filtered Views**: Show only specific object classes (cars, people, etc.)
