"""
LaneFollow Module - Lane Detection for YOLO Server

This module provides lane detection algorithms for use with the YOLO server.
It runs on the camera side (yolo_server_virtual.py) and publishes lane data.

Main Components:
- LaneDetectorBase: Abstract interface for lane detection algorithms
- LaneDetectionResult: Standardized result from any detector
- HSVLaneDetector, BEVLaneDetector, LaneNetDetector: Implementations
- create_lane_detector: Factory function to create detectors from config

Usage:
    from LaneFollow import create_lane_detector_from_config, LaneDetectionResult
    
    detector = create_lane_detector_from_config()
    if detector:
        result = detector.detect(rgb_image)
        steering = result.steering_correction
"""

from .lane_detection_interface import (
    LaneDetectorBase,
    LaneDetectionResult,
    LaneDetectionMethod,
    convert_yolo_data_to_lane_result
)

from .lane_detectors import (
    HSVLaneDetector,
    BEVLaneDetector,
    LaneNetDetector
)

from .config_loader_lane_detection import (
    get_lane_detection_config,
    LaneDetectionConfigLoader,
    HSVDetectorSettings,
    BEVDetectorSettings,
    LaneNetDetectorSettings
)


def create_lane_detector(algorithm: str = 'hsv', config: dict = None) -> 'LaneDetectorBase':
    """
    Factory function to create a lane detector based on algorithm name.
    
    Args:
        algorithm: 'hsv', 'bev', or 'lanenet'
        config: Optional configuration dict for the detector
        
    Returns:
        Initialized lane detector instance
    """
    config = config or {}
    
    if algorithm == 'hsv':
        detector = HSVLaneDetector(config=config)
        detector.initialize()
        return detector
        
    elif algorithm == 'bev':
        detector = BEVLaneDetector(config=config)
        detector.initialize()
        return detector
        
    elif algorithm == 'lanenet':
        detector = LaneNetDetector(config=config)
        if not detector.initialize():
            print("[LANE] LaneNet failed to initialize, using HSV fallback")
            return create_lane_detector('hsv', config)
        return detector
        
    else:
        print(f"[LANE] Unknown algorithm '{algorithm}', using HSV")
        return create_lane_detector('hsv', config)


def create_lane_detector_from_config(config_loader: 'LaneDetectionConfigLoader' = None):
    """
    Create lane detector directly from config loader.
    
    This is the recommended way to create a lane detector with proper configuration.
    
    Args:
        config_loader: Optional config loader instance. If None, uses default.
        
    Returns:
        Initialized lane detector, or None if disabled
    """
    if config_loader is None:
        config_loader = get_lane_detection_config()
    
    if not config_loader.is_lane_detection_enabled():
        return None
    
    algorithm = config_loader.get_detection_algorithm()
    detector_config = config_loader.get_detector_config_dict(algorithm)
    
    detector = create_lane_detector(algorithm, detector_config)
    print(f"[LANE] Created {algorithm.upper()} detector from config")
    return detector


__all__ = [
    # Interface
    'LaneDetectorBase',
    'LaneDetectionResult',
    'LaneDetectionMethod',
    'convert_yolo_data_to_lane_result',
    
    # Detector implementations
    'HSVLaneDetector',
    'BEVLaneDetector',
    'LaneNetDetector',
    
    # Config
    'get_lane_detection_config',
    'LaneDetectionConfigLoader',
    'HSVDetectorSettings',
    'BEVDetectorSettings',
    'LaneNetDetectorSettings',
    
    # Factory functions
    'create_lane_detector',
    'create_lane_detector_from_config',
]
