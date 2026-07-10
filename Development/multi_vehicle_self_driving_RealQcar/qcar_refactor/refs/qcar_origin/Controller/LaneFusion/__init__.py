"""
LaneFusion Module - Fusion System for Combining Waypoint and Lane-based Steering

This module provides the fusion system that combines waypoint-based steering
with lane detection corrections for improved path following.

Note: Lane detection implementations have been moved to Yolo/LaneFollow module.
      This module focuses only on the fusion logic.

Main Components:
- LaneFusion: Fusion system that combines waypoint + lane steering
- LaneFusionConfig: Configuration for fusion behavior
- FusionStrategy: Available fusion strategies

Usage:
    from Controller.LaneFusion import LaneFusion, LaneFusionConfig, FusionStrategy
    
    # Create fusion with adaptive strategy
    fusion = LaneFusion(LaneFusionConfig(strategy=FusionStrategy.ADAPTIVE))
    
    # In control loop:
    result = fusion.compute_steering(
        waypoint_steering=stanley_output,
        yolo_data=yolo_data_dict
    )
    final_steering = result.final_steering

Author: QCar2 Team
"""

from .lane_fusion import (
    LaneFusion,
    LaneFusionConfig,
    FusionStrategy,
    FusionResult,
    create_lane_fusion
)

# Import config loader for fusion settings
try:
    from .config_loader_lane_fusion import (
        get_lane_fusion_config,
        LaneFusionConfigLoader,
        LaneFusionSettings
    )
    _CONFIG_AVAILABLE = True
except ImportError:
    _CONFIG_AVAILABLE = False
    get_lane_fusion_config = None
    LaneFusionConfigLoader = None
    LaneFusionSettings = None

# Import lane detection types for type hints and compatibility
# These are now in Yolo/LaneFollow but we re-export for backward compatibility
try:
    import sys
    import os
    # Add Yolo folder to path for importing LaneFollow
    yolo_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'Yolo')
    if yolo_dir not in sys.path:
        sys.path.insert(0, yolo_dir)
    
    from LaneFollow import (
        LaneDetectorBase,
        LaneDetectionResult,
        LaneDetectionMethod,
        convert_yolo_data_to_lane_result
    )
    _LANE_TYPES_AVAILABLE = True
except ImportError:
    _LANE_TYPES_AVAILABLE = False
    LaneDetectorBase = None
    LaneDetectionResult = None
    LaneDetectionMethod = None
    convert_yolo_data_to_lane_result = None


__all__ = [
    # Core fusion classes
    'LaneFusion',
    'LaneFusionConfig',
    'FusionStrategy',
    'FusionResult',
    
    # Config
    'get_lane_fusion_config',
    'LaneFusionConfigLoader',
    'LaneFusionSettings',
    
    # Factory function
    'create_lane_fusion',
    
    # Lane detection types (re-exported for compatibility)
    'LaneDetectorBase',
    'LaneDetectionResult',
    'LaneDetectionMethod',
    'convert_yolo_data_to_lane_result',
]
