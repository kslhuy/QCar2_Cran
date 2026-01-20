"""
Lane Detection Config Loader

Loads lane detection configuration from YAML file.
"""

import os
import yaml
from typing import Dict, Any, Optional
from dataclasses import dataclass, asdict

# Get config file path (same directory as this file)
CONFIG_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG_FILE = os.path.join(CONFIG_DIR, 'config_lane_detection.yaml')


@dataclass
class LaneFusionSettings:
    """Lane fusion settings loaded from config"""
    enabled: bool = True
    strategy: str = 'adaptive'
    max_lane_weight: float = 0.4
    min_confidence: float = 0.2
    lane_gain: float = 1.0
    smoothing_factor: float = 0.3
    max_steering: float = 0.5
    deadband: float = 0.01
    switch_threshold: float = 0.6
    enable_curvature_compensation: bool = False
    debug_logging: bool = False


@dataclass
class HSVDetectorSettings:
    """HSV detector settings"""
    crop_ratio: float = 0.4
    hsv_lower: list = None
    hsv_upper: list = None
    target_slope: float = 0.3419
    slope_gain: float = 1.5
    intercept_gain: float = 0.00667
    intercept_offset: float = 5.0
    
    def __post_init__(self):
        if self.hsv_lower is None:
            self.hsv_lower = [10, 50, 100]
        if self.hsv_upper is None:
            self.hsv_upper = [45, 255, 255]
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for factory function"""
        return asdict(self)


@dataclass
class BEVDetectorSettings:
    """BEV detector settings"""
    img_width: int = 640
    img_height: int = 480
    warp_w_top: int = 150
    warp_w_bot: int = 640
    warp_h: int = 180
    warp_y_offset: int = 0
    nwindows: int = 9
    margin: int = 100
    minpix: int = 50
    camera_offset_m: float = 0.032
    lane_width_ref_m: float = 0.5
    single_lane_offset_px: int = 250
    assumed_lane_width_px: int = 500
    steer_gain: float = 1.0
    curve_gain: float = 200.0
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for factory function"""
        return asdict(self)


@dataclass
class LaneNetDetectorSettings:
    """LaneNet detector settings"""
    image_width: int = 640
    image_height: int = 480
    row_upper_bound: int = 200
    use_clustering: bool = False
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for factory function"""
        return asdict(self)


class LaneDetectionConfigLoader:
    """
    Loads and manages lane detection configuration from YAML file.
    
    Usage:
        config = LaneDetectionConfigLoader()
        hsv_settings = config.get_hsv_settings()
        bev_settings = config.get_bev_settings()
    """
    
    def __init__(self, config_file: str = None):
        """
        Initialize config loader.
        
        Args:
            config_file: Path to YAML config file (uses default if None)
        """
        self.config_file = config_file or DEFAULT_CONFIG_FILE
        self._config = {}
        self._load_config()
    
    def _load_config(self):
        """Load configuration from YAML file"""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    self._config = yaml.safe_load(f) or {}
                print(f"[LaneDetection] Loaded config from {self.config_file}")
            except Exception as e:
                print(f"[LaneDetection] Error loading config: {e}")
                self._config = {}
        else:
            print(f"[LaneDetection] Config file not found: {self.config_file}, using defaults")
            self._config = {}
    
    def reload(self):
        """Reload configuration from file"""
        self._load_config()
    
    def get_fusion_settings(self) -> LaneFusionSettings:
        """Get lane fusion settings"""
        cfg = self._config.get('lane_fusion', {})
        return LaneFusionSettings(
            enabled=cfg.get('enabled', True),
            strategy=cfg.get('strategy', 'adaptive'),
            max_lane_weight=cfg.get('max_lane_weight', 0.4),
            min_confidence=cfg.get('min_confidence', 0.2),
            lane_gain=cfg.get('lane_gain', 1.0),
            smoothing_factor=cfg.get('smoothing_factor', 0.3),
            max_steering=cfg.get('max_steering', 0.5),
            deadband=cfg.get('deadband', 0.01),
            switch_threshold=cfg.get('switch_threshold', 0.6),
            enable_curvature_compensation=cfg.get('enable_curvature_compensation', False),
            debug_logging=cfg.get('debug_logging', False)
        )
    
    def get_detection_algorithm(self) -> str:
        """Get lane detection algorithm ('hsv', 'bev', 'lanenet')"""
        return self._config.get('lane_detection', {}).get('algorithm', 'hsv')
    
    def is_lane_detection_enabled(self) -> bool:
        """Check if lane detection is enabled in yolo_server"""
        return self._config.get('lane_detection', {}).get('enabled', True)
    
    def get_hsv_settings(self) -> HSVDetectorSettings:
        """Get HSV detector settings"""
        cfg = self._config.get('hsv_detector', {})
        return HSVDetectorSettings(
            crop_ratio=cfg.get('crop_ratio', 0.4),
            hsv_lower=cfg.get('hsv_lower', [10, 50, 100]),
            hsv_upper=cfg.get('hsv_upper', [45, 255, 255]),
            target_slope=cfg.get('target_slope', 0.3419),
            slope_gain=cfg.get('slope_gain', 1.5),
            intercept_gain=cfg.get('intercept_gain', 0.00667),
            intercept_offset=cfg.get('intercept_offset', 5.0)
        )
    
    def get_bev_settings(self) -> BEVDetectorSettings:
        """Get BEV detector settings"""
        cfg = self._config.get('bev_detector', {})
        return BEVDetectorSettings(
            img_width=cfg.get('img_width', 640),
            img_height=cfg.get('img_height', 480),
            warp_w_top=cfg.get('warp_w_top', 150),
            warp_w_bot=cfg.get('warp_w_bot', 640),
            warp_h=cfg.get('warp_h', 180),
            warp_y_offset=cfg.get('warp_y_offset', 0),
            nwindows=cfg.get('nwindows', 9),
            margin=cfg.get('margin', 100),
            minpix=cfg.get('minpix', 50),
            camera_offset_m=cfg.get('camera_offset_m', 0.032),
            lane_width_ref_m=cfg.get('lane_width_ref_m', 0.5),
            single_lane_offset_px=cfg.get('single_lane_offset_px', 250),
            assumed_lane_width_px=cfg.get('assumed_lane_width_px', 500),
            steer_gain=cfg.get('steer_gain', 1.0),
            curve_gain=cfg.get('curve_gain', 200.0)
        )
    
    def get_lanenet_settings(self) -> LaneNetDetectorSettings:
        """Get LaneNet detector settings"""
        cfg = self._config.get('lanenet_detector', {})
        return LaneNetDetectorSettings(
            image_width=cfg.get('image_width', 640),
            image_height=cfg.get('image_height', 480),
            row_upper_bound=cfg.get('row_upper_bound', 200),
            use_clustering=cfg.get('use_clustering', False)
        )
    
    def get_raw_config(self) -> Dict[str, Any]:
        """Get raw config dictionary"""
        return self._config.copy()
    
    def get_detector_config_dict(self, algorithm: str = None) -> Dict[str, Any]:
        """
        Get detector configuration as dictionary for factory function.
        
        This is a convenience method that returns settings as a dict,
        suitable for passing to create_lane_detector() factory.
        
        Args:
            algorithm: Override algorithm name. If None, uses config value.
            
        Returns:
            Dictionary suitable for create_lane_detector() factory
        """
        algo = algorithm or self.get_detection_algorithm()
        if algo == 'hsv':
            return self.get_hsv_settings().to_dict()
        elif algo == 'bev':
            return self.get_bev_settings().to_dict()
        elif algo == 'lanenet':
            return self.get_lanenet_settings().to_dict()
        return {}


# Global config instance (lazy loaded)
_config_instance: Optional[LaneDetectionConfigLoader] = None


def get_lane_detection_config(config_file: str = None) -> LaneDetectionConfigLoader:
    """
    Get the lane detection config loader instance.
    
    Args:
        config_file: Optional path to config file
        
    Returns:
        LaneDetectionConfigLoader instance
    """
    global _config_instance
    if _config_instance is None or config_file is not None:
        _config_instance = LaneDetectionConfigLoader(config_file)
    return _config_instance
