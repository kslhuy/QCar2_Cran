"""
Lane Detection Interface - Abstract Base Class for All Lane Detection Algorithms

This module provides a common interface for different lane detection algorithms.
Any lane detection algorithm can implement this interface to be used with the
LaneFusion system for improved path following.

Supported algorithms can include:
- HSV-based lane detection (simple, fast)
- Bird's Eye View (BEV) with sliding window
- LaneNet (deep learning based)
- Custom algorithms

Author: QCar2 Team
"""

import numpy as np
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, Tuple, Dict, Any
from enum import Enum


class LaneDetectionMethod(Enum):
    """Enumeration of available lane detection methods"""

    HSV_THRESHOLD = "hsv_threshold"  # Simple HSV color thresholding
    BEV_SLIDING = "bev_sliding"  # Bird's Eye View with sliding window
    LANENET = "lanenet"  # LaneNet deep learning
    ULTRAFAST = "ultrafast"  # UltraFast-v2 Lane Detection
    YOLO_LANE = "yolo_lane"  # Lane data from YOLO server
    CUSTOM = "custom"  # Custom implementation


@dataclass
class LaneDetectionResult:
    """
    Standardized output from any lane detection algorithm.

    This dataclass provides a common format for lane detection results,
    allowing different algorithms to be swapped without changing downstream code.

    Attributes:
        is_valid: Whether lane detection was successful
        confidence: Detection confidence [0.0, 1.0]
        steering_correction: Suggested steering correction [-1.0, 1.0],
                            positive = steer right, negative = steer left
        lateral_offset: Offset from lane center in meters (positive = right of center)
        curvature: Lane curvature (1/radius), positive = curves right
        lane_width: Detected lane width in meters (if available)
        left_lane_detected: Whether left lane boundary was detected
        right_lane_detected: Whether right lane boundary was detected
        raw_data: Optional dictionary for algorithm-specific data
        timestamp: Detection timestamp (for synchronization)
    """

    is_valid: bool = False
    confidence: float = 0.0
    steering_correction: float = 0.0
    lateral_offset: float = 0.0
    curvature: float = 0.0
    lane_width: float = 0.0
    left_lane_detected: bool = False
    right_lane_detected: bool = False
    raw_data: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = 0.0

    def is_reliable(self, min_confidence: float = 0.3) -> bool:
        """Check if detection is reliable enough for use"""
        return self.is_valid and self.confidence >= min_confidence

    def get_steering_gain(self, max_gain: float = 0.5) -> float:
        """
        Calculate adaptive steering gain based on confidence.
        Higher confidence = higher gain (more influence from lane detection)
        """
        if not self.is_valid:
            return 0.0
        return min(max_gain * self.confidence, max_gain)


class LaneDetectorBase(ABC):
    """
    Abstract base class for all lane detection algorithms.

    Implement this interface to create a new lane detection algorithm
    that can be used with the LaneFusion system.

    Example usage:
        class MyCustomLaneDetector(LaneDetectorBase):
            def detect(self, image, vehicle_state):
                # Your algorithm here
                return LaneDetectionResult(...)
    """

    def __init__(self, config: Dict[str, Any] = None, logger=None):
        """
        Initialize lane detector.

        Args:
            config: Algorithm-specific configuration parameters
            logger: Optional logger instance
        """
        self.config = config or {}
        self.logger = logger
        self._is_initialized = False
        self._last_result = LaneDetectionResult()

    @abstractmethod
    def detect(
        self, image: np.ndarray, vehicle_state: Optional[Dict[str, float]] = None
    ) -> LaneDetectionResult:
        """
        Perform lane detection on input image.

        Args:
            image: RGB or BGR image array (H, W, 3)
            vehicle_state: Optional vehicle state dict with keys:
                          'x', 'y', 'theta', 'velocity'

        Returns:
            LaneDetectionResult with detection data
        """
        pass

    @abstractmethod
    def initialize(self) -> bool:
        """
        Initialize the lane detector (load models, calibration, etc.)

        Returns:
            True if initialization successful
        """
        pass

    def reset(self):
        """Reset detector state (clear history, filters, etc.)"""
        self._last_result = LaneDetectionResult()

    def terminate(self):
        """Clean up resources when done"""
        pass

    def get_last_result(self) -> LaneDetectionResult:
        """Get the most recent detection result"""
        return self._last_result

    @property
    def is_initialized(self) -> bool:
        """Check if detector is initialized and ready"""
        return self._is_initialized

    @property
    @abstractmethod
    def method(self) -> LaneDetectionMethod:
        """Return the detection method type"""
        pass

    def get_config(self) -> Dict[str, Any]:
        """Get current configuration"""
        return self.config.copy()

    def update_config(self, new_config: Dict[str, Any]):
        """Update configuration parameters"""
        self.config.update(new_config)


def convert_yolo_data_to_lane_result(
    yolo_data: Dict[str, Any],
) -> Optional[LaneDetectionResult]:
    """
    Convert YOLO server data dict to LaneDetectionResult.

    This is a simple helper function that replaces the YOLOLaneDetector wrapper class.

    Args:
        yolo_data: Dictionary with lane data from YOLO receiver
                  Expected keys: 'lane_confidence', 'lane_steering',
                                'lane_slope', 'lane_intercept'

    Returns:
        LaneDetectionResult or None if yolo_data is None/invalid
    """
    import time

    if yolo_data is None:
        return None

    confidence = yolo_data.get("lane_confidence", 0.0)
    steering = yolo_data.get("lane_steering", 0.0)
    slope = yolo_data.get("lane_slope", 0.0)
    intercept = yolo_data.get("lane_intercept", 0.0)

    # Calculate derived values
    is_valid = confidence > 0.05  # Minimal threshold for validity

    return LaneDetectionResult(
        is_valid=is_valid,
        confidence=min(confidence * 5.0, 1.0),  # Scale confidence to [0, 1]
        steering_correction=steering,
        lateral_offset=0.0,  # Not directly available from simple HSV
        curvature=slope,  # Use slope as curvature proxy
        lane_width=0.0,
        left_lane_detected=is_valid,
        right_lane_detected=False,
        raw_data={"slope": slope, "intercept": intercept, "raw_confidence": confidence},
        timestamp=time.time(),
    )
