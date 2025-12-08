# YOLO Object Detection System

Real-time object detection and perception system for autonomous vehicle navigation.

## Files

- `yolo_server.py` - YOLO detection server and image processing
- `YoLo.py` - YOLO publisher and communication utilities
- `__init__.py` - Package initialization

## Overview

The YOLO system provides:
- Real-time object detection (cars, stop signs, pedestrians)
- Distance estimation using depth cameras
- Multi-vehicle perception sharing
- Visual obstacle avoidance

## Features

### Object Detection
- **Cars**: Other vehicle detection and tracking
- **Stop Signs**: Traffic sign recognition
- **Pedestrians**: Human detection for safety
- **Lane Markers**: Road boundary detection
- **General Obstacles**: Unknown object detection

### Distance Estimation
- RGB-D camera fusion
- Real-world distance calculation
- 3D object localization
- Depth map processing

### Multi-Vehicle Perception
- YOLO data sharing via network
- Distributed perception fusion
- Collaborative object tracking
- Redundant detection validation

## Usage

### Starting YOLO Server

```powershell
# Basic startup
python yolo_server.py

# With specific configuration run in Qcar
python yolo_server.py -i 192.168.1.100 -p True -w 320 -ht 200 -idx 0

# In PC Ground station 
python multi_probing.py --cars [0,1] --width WIDTH --height HEIGHT


```

### Command Line Arguments

- `-i, --ip_host`: Host PC IP address (default: 192.168.2.10)
- `-p, --probing`: Enable image probing to observer (True/False)
- `-w, --width`: Display width for observer (default: 320)
- `-ht, --height`: Display height for observer (default: 200)
- `-idx, --caridx`: Car ID for port assignment (default: 0)

### Integration Example

```python
from Yolo.YoLo import YOLOPublisher

# Initialize YOLO publisher
yolo_server = YOLOPublisher(port='18666')

# Process detection data
yolo_data = yolo_server.get_detection_data()
if yolo_data:
    cars_detected = yolo_data.get('cars', False)
    car_distance = yolo_data.get('car_dist', float('inf'))
    stop_sign = yolo_data.get('stop_sign', False)
```

## Detection Data Format

```python
yolo_data = {
    'cars': bool,           # Car detected
    'car_dist': float,      # Distance to nearest car (m)
    'stop_sign': bool,      # Stop sign detected
    'pedestrian': bool,     # Pedestrian detected
    'ped_dist': float,      # Distance to pedestrian (m)
    'lane_left': bool,      # Left lane marker
    'lane_right': bool,     # Right lane marker
    'obstacle': bool,       # General obstacle
    'obs_dist': float,      # Distance to obstacle (m)
    'timestamp': float      # Detection timestamp
}
```

## Configuration

### Model Configuration
```python
# YOLOv8 model setup
imageWidth = 640
imageHeight = 480
myYolo = YOLOv8(
    # modelPath = 'path/to/custom/model',
    imageHeight=imageHeight,
    imageWidth=imageWidth,
)
```

### Camera Configuration
```python
# Depth/RGB alignment
QCarImg = QCar2DepthAligned(port='18777')

# YOLO publisher
YOLOserver = YOLOPublisher(port='18666')
```

### Network Ports

Per-vehicle port assignment:
- **Camera Stream**: `1877X` (Car 0: 18770, Car 1: 18771)
- **YOLO Data**: `1866X` (Car 0: 18660, Car 1: 18661)  
- **Probe Display**: `1880X` (Car 0: 18800, Car 1: 18801)

## Integration with Vehicle Control

### State Machine Integration
YOLO data influences vehicle behavior:

```python
# In following_path_state.py
def update(self, dt, sensor_data):
    yolo_data = sensor_data.get('yolo_data', {})
    
    # Stop for cars ahead
    if yolo_data.get('cars') and yolo_data.get('car_dist', float('inf')) < 1.5:
        return 0.0, steering, None  # Stop
    
    # Slow for stop signs
    if yolo_data.get('stop_sign'):
        v_ref *= 0.5  # Reduce speed
```

### Safety Integration
```python
# Emergency stop for pedestrians
if yolo_data.get('pedestrian') and yolo_data.get('ped_dist', float('inf')) < 2.0:
    return 0.0, 0.0, (VehicleState.EMERGENCY_STOP, "pedestrian_detected")
```

## Performance

### Detection Performance
- **Frame Rate**: 20-30 FPS (depends on model)
- **Detection Accuracy**: >90% for trained objects
- **Processing Latency**: <50 ms per frame
- **Detection Range**: 2-20 meters (depends on object size)

### Network Performance
- **Data Rate**: ~10 KB/s per detection stream
- **Update Rate**: 20 Hz detection data
- **Latency**: <100 ms end-to-end

## Troubleshooting

### YOLO Server Not Starting
```powershell
# Check camera connection
ls /dev/video*  # Linux
# or check device manager on Windows

# Verify dependencies
python -c "import torch, cv2; print('Dependencies OK')"

# Check ports
netstat -an | findstr "18666"
```

### Poor Detection Performance
- Check camera calibration
- Verify lighting conditions
- Update YOLO model
- Adjust detection thresholds

### Network Issues
- Verify IP configuration
- Check firewall settings
- Test network connectivity
- Monitor bandwidth usage

### Image Display Issues
```powershell
# Check probe connection
python -c "from pal.utilities.probe import Probe; print('Probe OK')"

# Verify display ports
netstat -an | findstr "18800"
```

## Model Training

For custom object detection:

1. **Collect Data**: Gather images from QCar cameras
2. **Label Data**: Annotate objects in YOLO format
3. **Train Model**: Use YOLOv8 training pipeline
4. **Validate**: Test on real vehicle scenarios
5. **Deploy**: Update model path in configuration

## Multi-Vehicle Coordination

### Shared Perception
- Each vehicle runs its own YOLO server
- Detection data shared via V2V communication
- Fusion of multiple viewpoints
- Improved detection reliability

### Collaborative Tracking
- Object tracking across multiple cameras
- Handoff between vehicle sensors
- Distributed object database
- Consistent object IDs

## Integration Points

YOLO integrates with:
- **State Machine**: Detection-based state transitions
- **Observer**: Perception data for state estimation
- **V2V**: Shared detection data
- **Safety**: Emergency detection responses
- **GUI**: Real-time detection visualization

See `../Doc/REFACTORING_README.md` for complete system integration details.