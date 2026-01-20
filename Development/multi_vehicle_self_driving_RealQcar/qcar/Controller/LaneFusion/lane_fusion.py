"""
Lane Fusion Module - Combines Waypoint Steering with Lane-Based Correction

This module provides a flexible fusion system that combines:
1. Global path following (waypoint-based steering from Stanley/Pure Pursuit)
2. Local lane keeping (lane detection-based corrections)

The fusion strategy uses configurable weights and adaptive gains based on
lane detection confidence to provide robust path following even with poor GPS.

Fusion Strategies:
- WEIGHTED: Simple weighted average of waypoint and lane steering
- CASCADED: Lane correction applied as offset to waypoint steering
- ADAPTIVE: Confidence-based adaptive weighting
- SWITCH: Hard switch based on confidence threshold

Author: QCar2 Team
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, List, Tuple
from enum import Enum
import time
import sys
import os

# Add Yolo folder to path for importing LaneFollow
_yolo_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'Yolo')
if _yolo_dir not in sys.path:
    sys.path.insert(0, _yolo_dir)

# Import lane detection types from LaneFollow module (moved from here)
from LaneFollow.lane_detection_interface import (
    LaneDetectionResult, 
    LaneDetectionMethod,
    convert_yolo_data_to_lane_result
)


class FusionStrategy(Enum):
    """Available fusion strategies"""
    WEIGHTED = "weighted"       # Weighted average based on fixed ratio
    CASCADED = "cascaded"       # Lane as correction to waypoint
    ADAPTIVE = "adaptive"       # Confidence-adaptive weighting
    SWITCH = "switch"           # Switch based on confidence threshold
    LANE_PRIORITY = "lane_priority"  # Prioritize lane when available


@dataclass
class LaneFusionConfig:
    """
    Configuration for lane fusion system.
    
    Attributes:
        strategy: Fusion strategy to use
        min_confidence: Minimum lane confidence to use lane data
        max_lane_weight: Maximum weight for lane steering (0-1)
        lane_gain: Gain applied to lane steering correction
        waypoint_weight: Base weight for waypoint steering (strategy-specific)
        smoothing_factor: Low-pass filter factor for output (0-1, lower = smoother)
        enable_curvature_compensation: Use lane curvature in steering calc
        max_steering: Maximum absolute steering value
        deadband: Ignore lane corrections smaller than this
        switch_threshold: Confidence threshold for SWITCH strategy
        debug_logging: Enable detailed logging
    """
    strategy: FusionStrategy = FusionStrategy.ADAPTIVE
    min_confidence: float = 0.2
    max_lane_weight: float = 0.4
    lane_gain: float = 1.0
    waypoint_weight: float = 0.7
    smoothing_factor: float = 0.3
    enable_curvature_compensation: bool = False
    max_steering: float = 0.5
    deadband: float = 0.01
    switch_threshold: float = 0.6
    debug_logging: bool = False
    
    @classmethod
    def from_dict(cls, config_dict: Dict[str, Any]) -> 'LaneFusionConfig':
        """Create config from dictionary"""
        return cls(
            strategy=FusionStrategy(config_dict.get('strategy', 'adaptive')),
            min_confidence=config_dict.get('min_confidence', 0.2),
            max_lane_weight=config_dict.get('max_lane_weight', 0.4),
            lane_gain=config_dict.get('lane_gain', 1.0),
            waypoint_weight=config_dict.get('waypoint_weight', 0.7),
            smoothing_factor=config_dict.get('smoothing_factor', 0.3),
            enable_curvature_compensation=config_dict.get('enable_curvature_compensation', False),
            max_steering=config_dict.get('max_steering', 0.5),
            deadband=config_dict.get('deadband', 0.01),
            switch_threshold=config_dict.get('switch_threshold', 0.6),
            debug_logging=config_dict.get('debug_logging', False)
        )
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert config to dictionary"""
        return {
            'strategy': self.strategy.value,
            'min_confidence': self.min_confidence,
            'max_lane_weight': self.max_lane_weight,
            'lane_gain': self.lane_gain,
            'waypoint_weight': self.waypoint_weight,
            'smoothing_factor': self.smoothing_factor,
            'enable_curvature_compensation': self.enable_curvature_compensation,
            'max_steering': self.max_steering,
            'deadband': self.deadband,
            'switch_threshold': self.switch_threshold,
            'debug_logging': self.debug_logging
        }


@dataclass 
class FusionResult:
    """
    Result of steering fusion computation.
    
    Provides detailed breakdown for debugging and telemetry.
    """
    final_steering: float = 0.0
    waypoint_steering: float = 0.0
    lane_steering: float = 0.0
    lane_weight_used: float = 0.0
    lane_confidence: float = 0.0
    strategy_used: FusionStrategy = FusionStrategy.ADAPTIVE
    lane_valid: bool = False
    curvature_compensation: float = 0.0
    timestamp: float = 0.0
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for logging"""
        return {
            'final_steering': self.final_steering,
            'waypoint_steering': self.waypoint_steering,
            'lane_steering': self.lane_steering,
            'lane_weight_used': self.lane_weight_used,
            'lane_confidence': self.lane_confidence,
            'strategy_used': self.strategy_used.value,
            'lane_valid': self.lane_valid,
            'curvature_compensation': self.curvature_compensation,
            'timestamp': self.timestamp
        }


class LaneFusion:
    """
    Main lane fusion class that combines waypoint and lane-based steering.
    
    This class provides a plug-and-play system for improving path following
    by combining waypoint steering with lane detection data from YOLO server.
    
    Usage:
        # Create fusion system with default config
        fusion = LaneFusion()
        
        # Or with custom config
        config = LaneFusionConfig(strategy=FusionStrategy.ADAPTIVE, max_lane_weight=0.5)
        fusion = LaneFusion(config=config)
        
        # In control loop (lane data received from YOLO server):
        result = fusion.compute_steering(
            waypoint_steering=stanley_steering,
            yolo_data=yolo_data_dict  # Lane data from YOLO server
        )
        final_steering = result.final_steering
    """
    
    def __init__(self, config: LaneFusionConfig = None, logger=None):
        """
        Initialize lane fusion system.
        
        Args:
            config: Fusion configuration (uses defaults if None)
            logger: Optional logger instance
        """
        self.config = config or LaneFusionConfig()
        self.logger = logger
        
        # State for smoothing
        self._last_output = 0.0
        self._last_lane_result: Optional[LaneDetectionResult] = None
        
        # Statistics for debugging
        self._fusion_count = 0
        self._lane_used_count = 0
        self._last_fusion_result: Optional[FusionResult] = None
        
    def set_config(self, config: LaneFusionConfig):
        """Update fusion configuration"""
        self.config = config
        
    def update_config_param(self, param: str, value: Any):
        """Update a single config parameter"""
        if hasattr(self.config, param):
            setattr(self.config, param, value)
    
    def compute_steering(self,
                         waypoint_steering: float,
                         lane_detection: Optional[LaneDetectionResult] = None,
                         yolo_data: Optional[Dict[str, Any]] = None,
                         velocity: float = 0.5,
                         dt: float = 0.01) -> FusionResult:
        """
        Compute fused steering command.
        
        Args:
            waypoint_steering: Steering from waypoint controller (Stanley, etc.)
            lane_detection: Optional LaneDetectionResult (takes priority)
            yolo_data: Optional YOLO data dict (used if lane_detection is None)
            velocity: Current vehicle velocity (for adaptive gains)
            dt: Time step for filtering
            
        Returns:
            FusionResult with final steering and debug info
        """
        self._fusion_count += 1
        
        # Get lane detection result
        if lane_detection is None and yolo_data is not None:
            # Convert YOLO data to LaneDetectionResult using helper function
            lane_detection = convert_yolo_data_to_lane_result(yolo_data)
        
        # Check if lane data is valid and reliable
        lane_valid = (lane_detection is not None and 
                      lane_detection.is_reliable(self.config.min_confidence))
        
        # Get lane steering correction
        lane_steering = 0.0
        lane_confidence = 0.0
        lane_weight = 0.0
        
        if lane_valid:
            lane_steering = lane_detection.steering_correction * self.config.lane_gain
            lane_confidence = lane_detection.confidence
            
            # Apply deadband
            if abs(lane_steering) < self.config.deadband:
                lane_steering = 0.0
        
        # Compute fusion based on strategy
        final_steering, lane_weight = self._apply_fusion_strategy(
            waypoint_steering, lane_steering, lane_confidence, velocity
        )
        
        # Apply curvature compensation if enabled
        curvature_comp = 0.0
        if self.config.enable_curvature_compensation and lane_valid:
            curvature_comp = self._compute_curvature_compensation(
                lane_detection.curvature, velocity
            )
            final_steering += curvature_comp
        
        # Apply smoothing
        final_steering = self._apply_smoothing(final_steering)
        
        # Clip to limits
        final_steering = np.clip(final_steering, 
                                  -self.config.max_steering, 
                                  self.config.max_steering)
        
        # Update statistics
        if lane_valid and lane_weight > 0:
            self._lane_used_count += 1
        
        # Create result
        result = FusionResult(
            final_steering=final_steering,
            waypoint_steering=waypoint_steering,
            lane_steering=lane_steering if lane_valid else 0.0,
            lane_weight_used=lane_weight,
            lane_confidence=lane_confidence,
            strategy_used=self.config.strategy,
            lane_valid=lane_valid,
            curvature_compensation=curvature_comp,
            timestamp=time.time()
        )
        
        self._last_fusion_result = result
        self._last_lane_result = lane_detection
        
        # Debug logging
        if self.config.debug_logging and self.logger and self._fusion_count % 50 == 0:
            self.logger.logger.debug(
                f"[FUSION] wp={waypoint_steering:.3f}, lane={lane_steering:.3f}, "
                f"weight={lane_weight:.2f}, final={final_steering:.3f}, "
                f"conf={lane_confidence:.2f}"
            )
        
        return result
    
    def _apply_fusion_strategy(self, 
                                waypoint_steering: float,
                                lane_steering: float,
                                lane_confidence: float,
                                velocity: float) -> Tuple[float, float]:
        """
        Apply the configured fusion strategy.
        
        Returns:
            (final_steering, lane_weight_used)
        """
        strategy = self.config.strategy
        
        if strategy == FusionStrategy.WEIGHTED:
            # Fixed weighted average
            lane_weight = self.config.max_lane_weight
            wp_weight = 1.0 - lane_weight
            final = wp_weight * waypoint_steering + lane_weight * lane_steering
            return final, lane_weight
        
        elif strategy == FusionStrategy.CASCADED:
            # Lane steering as correction/offset to waypoint steering
            lane_weight = min(lane_confidence, self.config.max_lane_weight)
            final = waypoint_steering + lane_weight * lane_steering
            return final, lane_weight
        
        elif strategy == FusionStrategy.ADAPTIVE:
            # Confidence-adaptive weighting
            # Higher confidence = more lane influence
            lane_weight = min(lane_confidence * self.config.max_lane_weight, 
                              self.config.max_lane_weight)
            wp_weight = 1.0 - lane_weight
            final = wp_weight * waypoint_steering + lane_weight * lane_steering
            return final, lane_weight
        
        elif strategy == FusionStrategy.SWITCH:
            # Hard switch based on confidence threshold
            if lane_confidence >= self.config.switch_threshold:
                # Use lane steering (with some waypoint influence)
                lane_weight = 0.8
                wp_weight = 0.2
            else:
                # Use waypoint steering only
                lane_weight = 0.0
                wp_weight = 1.0
            final = wp_weight * waypoint_steering + lane_weight * lane_steering
            return final, lane_weight
        
        elif strategy == FusionStrategy.LANE_PRIORITY:
            # Prioritize lane steering when available
            if lane_confidence >= self.config.min_confidence:
                # Blend but prioritize lane
                lane_weight = 0.7
                wp_weight = 0.3
                final = wp_weight * waypoint_steering + lane_weight * lane_steering
            else:
                # Fall back to waypoint
                final = waypoint_steering
                lane_weight = 0.0
            return final, lane_weight
        
        else:
            # Default to waypoint only
            return waypoint_steering, 0.0
    
    def _compute_curvature_compensation(self, curvature: float, velocity: float) -> float:
        """
        Compute feedforward steering compensation based on lane curvature.
        
        Args:
            curvature: Lane curvature (1/radius)
            velocity: Current velocity
            
        Returns:
            Steering compensation value
        """
        # Simple feedforward: steering = L * curvature * some_gain
        # where L is wheelbase (approximately 0.3m for QCar2)
        wheelbase = 0.3  # meters
        gain = 0.5
        
        compensation = wheelbase * curvature * gain
        return np.clip(compensation, -0.1, 0.1)
    
    def _apply_smoothing(self, steering: float) -> float:
        """Apply low-pass filtering to smooth output"""
        alpha = self.config.smoothing_factor
        smoothed = alpha * steering + (1 - alpha) * self._last_output
        self._last_output = smoothed
        return smoothed
    
    def reset(self):
        """Reset fusion state"""
        self._last_output = 0.0
        self._last_lane_result = None
        self._fusion_count = 0
        self._lane_used_count = 0
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get fusion statistics for monitoring"""
        lane_usage = 0.0
        if self._fusion_count > 0:
            lane_usage = self._lane_used_count / self._fusion_count * 100
            
        return {
            'fusion_count': self._fusion_count,
            'lane_used_count': self._lane_used_count,
            'lane_usage_percent': lane_usage,
            'current_strategy': self.config.strategy.value,
            'last_result': self._last_fusion_result.to_dict() if self._last_fusion_result else None
        }
    
    def get_last_result(self) -> Optional[FusionResult]:
        """Get the last fusion result"""
        return self._last_fusion_result


# Convenience function for quick setup
def create_lane_fusion(strategy: str = 'adaptive',
                       max_lane_weight: float = 0.4,
                       min_confidence: float = 0.2,
                       logger=None) -> LaneFusion:
    """
    Create a LaneFusion instance with common settings.
    
    Args:
        strategy: Fusion strategy ('weighted', 'cascaded', 'adaptive', 'switch', 'lane_priority')
        max_lane_weight: Maximum influence of lane steering (0-1)
        min_confidence: Minimum confidence to use lane data
        logger: Optional logger
        
    Returns:
        Configured LaneFusion instance
    """
    config = LaneFusionConfig(
        strategy=FusionStrategy(strategy),
        max_lane_weight=max_lane_weight,
        min_confidence=min_confidence
    )
    return LaneFusion(config=config, logger=logger)
