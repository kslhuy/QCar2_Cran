# YOLO Object Detection System

Real-time object detection and perception system for autonomous vehicle navigation.

## Files

- `yolo_server.py` - YOLO detection server and image processing
- `yolo_server_virtual`
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



## Usage

### Starting YOLO Server

```powershell
# Basic startup
python yolo_server.py

# With specific configuration run in Qcar
python yolo_server.py -i 192.168.1.100 -p True -w 320 -ht 200 -idx 0

# In PC Ground station 

```
for limo 
```bash
ros2 run limo_nav_huy_test yolo_ros2_server -p True --ros-args -p probing:=True
```

### Depth Alignment

QCar2_Depth2RBG_align.py for tuning  

QCar2DepthAlignedCamera for use 

## Test Scripts


# Terminal 1: Start observer for car 0
python multi_probing.py --cars 0
<!-- python multi_probing.py --cars 0 --width 320 --height 200 -->


# Terminal 2: Start YOLO server
python yolo_server_virtual.py -idx 0 -p True

# Another things
python test_yolo_with_images.py --car-id 0


### 3. `visual_yolo_monitor.py` - Visual Monitor with Images ⭐ NEW
Real-time visual monitor with OpenCV showing:
- Detection statistics with bar charts
- Live camera feed with YOLO annotations
- FPS and performance metrics
- Screenshot capture capability

### 4. `test_yolo_with_images.py` - Complete Visual Test ⭐ NEW
Comprehensive test that runs local YOLO processing with full visualization:
- Live camera feed with real-time YOLO detections
- Detection counts and distances
- Server comparison (if connected)
- Performance statistics
