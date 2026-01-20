# Lane Fusion Module

A modular lane detection and fusion system for improving path following with any lane detection algorithm.


Lane detection algorithms are now **centralized in this module** and imported by `yolo_server_virtual.py`:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      Controller/LaneFusion/                                  │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │ lane_detectors.py         │ lane_detection_interface.py                │ │
│  │  - HSVLaneDetector        │  - LaneDetectorBase (abstract)             │ │
│  │  - BEVLaneDetector        │  - LaneDetectionResult (dataclass)         │ │
│  │  - LaneNetDetector        │  - YOLOLaneDetector (wrapper)              │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
│                                    │                                         │
│                    create_lane_detector('hsv', config)                       │
└────────────────────────────────────┼────────────────────────────────────────┘
                                     │ imports
                                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  yolo_server_virtual.py (lightweight - no duplicate lane detection code!)   │
│                                                                              │
│  1. Camera capture (QCar2DepthAlignedCamera)                                │
│  2. YOLO object detection (YOLOv8)                                          │
│  3. Lane detection via: lane_detector.detect(rgb_image)  ◄── from module   │
│  4. Send results via UDP (YOLOPublisher)                                    │
│                                                                              │
│  laneBuffer = [confidence, steering, curvature, lateral_offset, 0, 0, 0]    │
└─────────────────────────────────────┼───────────────────────────────────────┘
                                      │ UDP
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  following_path_state.py                                                     │
│                                                                              │
│  1. Receive yolo_data dict                                                  │
│  2. LaneFusion combines waypoint + lane steering                            │
│  3. Output final_steering to vehicle control                                │
└─────────────────────────────────────────────────────────────────────────────┘
```



## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    FollowingPathState                        │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────────┐    ┌──────────────────────────────┐   │
│  │ StanleyController │    │         LaneFusion           │   │
│  │  (Waypoints)      │───►│  ┌─────────────────────────┐ │   │
│  └──────────────────┘    │  │    Fusion Strategy      │ │   │
│                           │  │  (adaptive/cascaded/...) │ │   │
│  ┌──────────────────┐    │  └─────────────────────────┘ │   │
│  │  LaneDetector    │───►│                              │   │
│  │  (any algorithm) │    │  Output: final_steering      │   │
│  └──────────────────┘    └──────────────────────────────┘   │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```
