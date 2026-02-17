"""
Example Lane Detector Implementations

This module provides example implementations of lane detectors using
different algorithms. These can be used with the LaneFusion system.

Available Detectors:
- HSVLaneDetector: Simple HSV color thresholding (fast, ~100+ FPS)
- BEVLaneDetector: Bird's Eye View with sliding window (accurate, ~20-30 FPS)
- LaneNetDetector: Deep learning based (most accurate, ~10-20 FPS)

Each detector implements the LaneDetectorBase interface and returns
standardized LaneDetectionResult objects.

Usage:
    from Controller.LaneFusion.lane_detectors import HSVLaneDetector
    
    detector = HSVLaneDetector()
    detector.initialize()
    
    result = detector.detect(rgb_image, vehicle_state)
    print(f"Steering correction: {result.steering_correction}")

Author: QCar2 Team
"""

import numpy as np
import cv2
import time
import json
import os
from typing import Optional, Dict, Any, Tuple
from collections import deque
from enum import Enum

# Import to detect if running on physical QCar or simulation
try:
    from pal.products.qcar import IS_PHYSICAL_QCAR
except ImportError:
    IS_PHYSICAL_QCAR = False  # Default to simulation if import fails

from .lane_detection_interface import (
    LaneDetectorBase,
    LaneDetectionResult,
    LaneDetectionMethod
)


def _draw_text_outline(
    image: np.ndarray,
    text: str,
    org: Tuple[int, int],
    color: Tuple[int, int, int] = (255, 255, 255),
    scale: float = 0.5,
    thickness: int = 1,
):
    """Draw readable text regardless of background using black outline."""
    cv2.putText(image, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 3, cv2.LINE_AA)
    cv2.putText(image, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


class LanePosition(Enum):
    """Enum representing the vehicle's lane position relative to yellow divider."""
    UNKNOWN = "unknown"
    LEFT_LANE = "left_lane"    # Yellow line detected on RIGHT side
    RIGHT_LANE = "right_lane"  # Yellow line detected on LEFT side


class HSVLaneDetector(LaneDetectorBase):
    """
    Simple HSV-based lane detector using color thresholding with lane position detection.
    
    This is a fast detector suitable for yellow/white lane markings.
    Best for well-lit conditions with consistent lane colors.
    
    Features:
        - Detects yellow lane markings using HSV thresholding
        - Determines vehicle lane position (left or right lane) by detecting
          which side of the image contains the yellow dividing line
        - Uses different crop ratios for simulation vs physical QCar
    
    Config options:
        crop_ratio_sim: Bottom portion for simulation (default: 0.4 = bottom 40%)
        crop_ratio_real: Bottom portion for physical QCar (default: 0.3)
        hsv_lower: Lower HSV bounds [H, S, V] (default: [10, 50, 100] for yellow)
        hsv_upper: Upper HSV bounds [H, S, V] (default: [45, 255, 255])
        target_slope: Expected lane slope when centered (default: 0.3419)
        slope_gain: Gain for slope-based steering (default: 1.5)
        intercept_gain: Gain for intercept-based steering (default: 1/150)
        lane_detection_threshold: Min pixel ratio to confirm lane presence (default: 0.01)
    """
    
    def __init__(self, config: Dict[str, Any] = None, logger=None):
        super().__init__(config, logger)
        
        # Crop ratio depends on simulation vs physical QCar
        if IS_PHYSICAL_QCAR:
            self.crop_ratio = self.config.get('crop_ratio_real', 0.0)
        else:
            self.crop_ratio = self.config.get('crop_ratio_sim', 0.4)
        
        # HSV thresholds for yellow lane detection
        self.hsv_lower = np.array(self.config.get('hsv_lower', [10, 50, 100]))
        self.hsv_upper = np.array(self.config.get('hsv_upper', [45, 255, 255]))
        
        # Steering calculation parameters
        self.target_slope = self.config.get('target_slope', 0.3419)
        self.slope_gain = self.config.get('slope_gain', 1.5)
        self.intercept_gain = self.config.get('intercept_gain', 1/150)
        self.intercept_offset = self.config.get('intercept_offset', 5)
        
        # Lane position detection parameters
        self.lane_detection_threshold = self.config.get('lane_detection_threshold', 0.01)
        self._last_lane_position = LanePosition.UNKNOWN
        
    @property
    def method(self) -> LaneDetectionMethod:
        return LaneDetectionMethod.HSV_THRESHOLD
    
    def initialize(self) -> bool:
        self._is_initialized = True
        return True
    
    def detect(self, image: np.ndarray,
               vehicle_state: Optional[Dict[str, float]] = None) -> LaneDetectionResult:
        """
        Detect lane using HSV thresholding and determine lane position.
        
        The lane position is determined by checking which half of the image
        contains the yellow dividing line:
        - Yellow line on LEFT half  -> Vehicle is in RIGHT lane
        - Yellow line on RIGHT half -> Vehicle is in LEFT lane
        
        Args:
            image: BGR image (H, W, 3)
            vehicle_state: Optional vehicle state (not used)
            
        Returns:
            LaneDetectionResult with steering correction and lane_position
        """
        try:
            height, width = image.shape[:2]
            
            # Crop bottom portion for lane detection
            crop_start = int(height * (1.0 - self.crop_ratio))
            cropped = image[crop_start:, :, :]
            
            # Convert to HSV and threshold for yellow lane
            hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

            # Collect mask statistics for debugging/tuning
            mid_x = mask.shape[1] // 2
            left_pixels = int(np.sum(mask[:, :mid_x] > 0))
            right_pixels = int(np.sum(mask[:, mid_x:] > 0))
            left_ratio = left_pixels / max(mask[:, :mid_x].size, 1)
            right_ratio = right_pixels / max(mask[:, mid_x:].size, 1)
            mask_pixels = int(np.sum(mask > 0))
            mask_ratio = mask_pixels / max(mask.size, 1)
            
            # Determine lane position by checking left/right halves
            lane_position = self._detect_lane_position(mask)
            self._last_lane_position = lane_position
            
            # Calculate slope and intercept using linear regression
            slope, intercept = self._find_slope_intercept(mask)
            
            if np.isnan(slope) or np.isnan(intercept):
                self._last_result = LaneDetectionResult(
                    timestamp=time.time(),
                    raw_data={
                        'lane_position': lane_position.value,
                        'slope': np.nan,
                        'intercept': np.nan,
                        'crop_start_px': crop_start,
                        'left_ratio': left_ratio,
                        'right_ratio': right_ratio,
                        'mask_ratio': mask_ratio,
                        'mask_pixels': mask_pixels,
                    }
                )
                return self._last_result
            
            # Calculate steering correction
            steering = self.slope_gain * (slope - self.target_slope)
            steering += self.intercept_gain * (intercept + self.intercept_offset)
            steering = np.clip(steering, -0.5, 0.5)
            
            # Confidence based on detected lane pixels
            confidence = mask_ratio
            confidence = min(confidence * 10, 1.0)  # Scale up
            
            # Determine which lanes are detected based on position
            left_lane_detected = (lane_position == LanePosition.RIGHT_LANE)  # Yellow on left
            right_lane_detected = (lane_position == LanePosition.LEFT_LANE)  # Yellow on right
            
            # Create debug image
            self._debug_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            
            # Draw the detected line on debug image
            if not np.isnan(slope) and not np.isnan(intercept):
                rows, cols = self._debug_image.shape[:2]
                y1 = 0
                x1 = int((y1 - intercept) / slope) if abs(slope) > 1e-4 else 0
                y2 = rows
                x2 = int((y2 - intercept) / slope) if abs(slope) > 1e-4 else 0
                cv2.line(self._debug_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
            
            # Draw readable metrics on debug image
            _draw_text_outline(self._debug_image, f"Pos: {lane_position.value}", (10, 30), (0, 255, 0), 0.6, 2)
            _draw_text_outline(self._debug_image, f"Slope: {slope:.4f}  Int: {intercept:.1f}", (10, 55), (220, 220, 220), 0.5, 1)
            _draw_text_outline(self._debug_image, f"Mask: {mask_ratio:.3f}  L/R: {left_ratio:.3f}/{right_ratio:.3f}", (10, 78), (220, 220, 220), 0.48, 1)

            self._last_result = LaneDetectionResult(
                is_valid=True,
                confidence=confidence,
                steering_correction=steering,
                curvature=slope,
                left_lane_detected=left_lane_detected,
                right_lane_detected=right_lane_detected,
                raw_data={
                    'slope': slope,
                    'intercept': intercept,
                    'lane_position': lane_position.value,
                    'crop_start_px': crop_start,
                    'left_ratio': left_ratio,
                    'right_ratio': right_ratio,
                    'mask_ratio': mask_ratio,
                    'mask_pixels': mask_pixels,
                },
                timestamp=time.time()
            )
            
        except Exception as e:
            if self.logger:
                self.logger.logger.debug(f"HSV lane detection error: {e}")
            self._last_result = LaneDetectionResult(timestamp=time.time())
            self._debug_image = None
        
        return self._last_result

    def get_debug_image(self) -> Optional[np.ndarray]:
        """Get the debug visualization (mask + line)"""
        return getattr(self, '_debug_image', None)
    
    def _detect_lane_position(self, mask: np.ndarray) -> LanePosition:
        """
        Detect which lane the vehicle is in by checking yellow line position.
        
        In a double-lane environment, the yellow line separates the lanes:
        - If yellow line is detected on the LEFT side -> We are in the RIGHT lane
        - If yellow line is detected on the RIGHT side -> We are in the LEFT lane
        
        Args:
            mask: Binary mask of yellow lane pixels
            
        Returns:
            LanePosition enum indicating detected lane
        """
        height, width = mask.shape
        mid_x = width // 2
        
        # Split mask into left and right halves
        left_half = mask[:, :mid_x]
        right_half = mask[:, mid_x:]
        
        # Count yellow pixels in each half
        left_pixels = np.sum(left_half > 0)
        right_pixels = np.sum(right_half > 0)
        
        # Calculate pixel ratios
        left_ratio = left_pixels / left_half.size if left_half.size > 0 else 0
        right_ratio = right_pixels / right_half.size if right_half.size > 0 else 0
        
        # Determine lane position based on where yellow line is detected
        if left_ratio > self.lane_detection_threshold and left_ratio > right_ratio * 1.5:
            # Yellow line predominantly on LEFT -> Vehicle in RIGHT lane
            return LanePosition.RIGHT_LANE
        elif right_ratio > self.lane_detection_threshold and right_ratio > left_ratio * 1.5:
            # Yellow line predominantly on RIGHT -> Vehicle in LEFT lane
            return LanePosition.LEFT_LANE
        elif left_ratio > self.lane_detection_threshold or right_ratio > self.lane_detection_threshold:
            # Yellow detected but not clearly on one side - use previous position
            return self._last_lane_position
        else:
            return LanePosition.UNKNOWN
    
    def get_lane_position(self) -> LanePosition:
        """Get the last detected lane position."""
        return self._last_lane_position
    
    def is_in_left_lane(self) -> bool:
        """Check if vehicle is in the left lane."""
        return self._last_lane_position == LanePosition.LEFT_LANE
    
    def is_in_right_lane(self) -> bool:
        """Check if vehicle is in the right lane."""
        return self._last_lane_position == LanePosition.RIGHT_LANE
    
    def _find_slope_intercept(self, binary_image: np.ndarray) -> Tuple[float, float]:
        """Find slope and intercept from binary lane mask using regression"""
        try:
            # Find white pixels
            y_indices, x_indices = np.nonzero(binary_image)
            
            if len(x_indices) < 10:
                return np.nan, np.nan
            
            # Linear regression: y = slope * x + intercept
            # Use numpy polyfit for speed
            coeffs = np.polyfit(x_indices, y_indices, 1)
            slope = coeffs[0]
            intercept = coeffs[1]
            
            return slope, intercept
            
        except Exception:
            return np.nan, np.nan


class BEVLaneDetector(LaneDetectorBase):
    """
    Bird's Eye View lane detector with sliding window.
    
    Provides accurate lane detection for curved roads by transforming
    the image to a top-down view and using sliding window detection.
    
    Config options:
        img_width: Image width (default: 640)
        img_height: Image height (default: 480)
        warp_w_top: Top width of perspective trapezoid
        warp_w_bot: Bottom width of perspective trapezoid
        warp_h: Height of trapezoid region
        nwindows: Number of sliding windows (default: 9)
        margin: Sliding window margin (default: 100)
        camera_offset_m: Camera lateral offset in meters
    """
    
    def __init__(self, config: Dict[str, Any] = None, logger=None):
        super().__init__(config, logger)
        
        self.img_width = self.config.get('img_width', 640)
        self.img_height = self.config.get('img_height', 480)
        
        # Warp parameters
        self.warp_w_top = self.config.get('warp_w_top', 150)
        self.warp_w_bot = self.config.get('warp_w_bot', 640)
        self.warp_h = self.config.get('warp_h', 180)
        self.warp_y_offset = self.config.get('warp_y_offset', 0)
        
        # Sliding window parameters
        self.nwindows = self.config.get('nwindows', 9)
        self.margin = self.config.get('margin', 100)
        self.minpix = self.config.get('minpix', 50)
        
        # Calibration
        self.camera_offset_m = self.config.get('camera_offset_m', 0.032)
        self.lane_width_ref_m = self.config.get('lane_width_ref_m', 0.5)
        self.single_lane_offset_px = self.config.get('single_lane_offset_px', 250)
        self.assumed_lane_width_px = self.config.get('assumed_lane_width_px', 500)
        
        # Transform matrices
        self.M = None
        self.Minv = None
        
        # Lane width validation (from QCar2_lane_following_new.py)
        self.min_lane_width_px = self.config.get('min_lane_width_px', 100)
        self.max_lane_width_px = self.config.get('max_lane_width_px', 600)
        
        # Fit history for smoothing
        self.left_fit_history = deque(maxlen=5)
        self.right_fit_history = deque(maxlen=5)
        self.best_left_fit = None
        self.best_right_fit = None
        
        # Debug visualization
        self._debug_image = None
        self._enable_debug = self.config.get('enable_debug', True)
        self._last_debug_data = {}
        
    @property
    def method(self) -> LaneDetectionMethod:
        return LaneDetectionMethod.BEV_SLIDING
    
    def initialize(self) -> bool:
        """Initialize perspective transform matrices"""
        self._update_transform_matrix()
        self._is_initialized = True
        return True
    
    def reset(self):
        """Reset detector state"""
        super().reset()
        self.left_fit_history.clear()
        self.right_fit_history.clear()
        self.best_left_fit = None
        self.best_right_fit = None
    
    def load_settings(self, filepath: str = None) -> bool:
        """
        Load BEV parameters from JSON file.
        
        Args:
            filepath: Path to JSON settings file. If None, uses default location.
            
        Returns:
            True if settings were loaded successfully
        """
        if filepath is None:
            # Default: look in LaneFollow folder relative to this file
            base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            filepath = os.path.join(base_dir, '..', 'Yolo', 'LaneFollow', 'lane_params_bev.json')
        
        try:
            if os.path.exists(filepath):
                with open(filepath, 'r') as f:
                    params = json.load(f)
                
                # Update warp parameters
                self.warp_w_top = params.get('warp_w_top', self.warp_w_top)
                self.warp_w_bot = params.get('warp_w_bot', self.warp_w_bot)
                self.warp_h = params.get('warp_h', self.warp_h)
                self.warp_y_offset = params.get('warp_y_offset', self.warp_y_offset)
                
                # Update gains
                if 'steer_gain' in params:
                    self.config['steer_gain'] = params['steer_gain']
                if 'curve_gain' in params:
                    self.config['curve_gain'] = params['curve_gain']
                
                # Recalculate transform matrix
                self._update_transform_matrix()
                
                if self.logger:
                    self.logger.logger.info(f"[BEV] Loaded settings from {filepath}")
                return True
                
        except Exception as e:
            if self.logger:
                self.logger.logger.warning(f"[BEV] Failed to load settings: {e}")
        
        return False
    
    def save_settings(self, filepath: str = None):
        """
        Save current BEV parameters to JSON file.
        
        Args:
            filepath: Path to save JSON settings. If None, uses default location.
        """
        if filepath is None:
            base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            filepath = os.path.join(base_dir, '..', 'Yolo', 'LaneFollow', 'lane_params_bev.json')
        
        params = {
            'warp_w_top': self.warp_w_top,
            'warp_w_bot': self.warp_w_bot,
            'warp_h': self.warp_h,
            'warp_y_offset': self.warp_y_offset,
            'steer_gain': self.config.get('steer_gain', 1.0),
            'curve_gain': self.config.get('curve_gain', 200.0),
        }
        
        try:
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            with open(filepath, 'w') as f:
                json.dump(params, f, indent=4)
            
            if self.logger:
                self.logger.logger.info(f"[BEV] Saved settings to {filepath}")
                
        except Exception as e:
            if self.logger:
                self.logger.logger.warning(f"[BEV] Failed to save settings: {e}")
    
    def get_debug_image(self) -> Optional[np.ndarray]:
        """Get the last debug visualization image (warped view with lane overlay)."""
        return self._debug_image
    
    def _update_transform_matrix(self):
        """Calculate perspective transform matrix"""
        cx = self.img_width // 2
        
        # Source trapezoid
        src = np.float32([
            [cx - self.warp_w_top // 2, self.img_height - self.warp_h - self.warp_y_offset],
            [cx + self.warp_w_top // 2, self.img_height - self.warp_h - self.warp_y_offset],
            [cx + self.warp_w_bot // 2, self.img_height - self.warp_y_offset],
            [cx - self.warp_w_bot // 2, self.img_height - self.warp_y_offset]
        ])
        
        # Destination rectangle
        dst_margin = self.img_width * 0.2
        dst = np.float32([
            [dst_margin, 0],
            [self.img_width - dst_margin, 0],
            [self.img_width - dst_margin, self.img_height],
            [dst_margin, self.img_height]
        ])
        
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src)
    
    def detect(self, image: np.ndarray,
               vehicle_state: Optional[Dict[str, float]] = None) -> LaneDetectionResult:
        """
        Detect lanes using BEV transformation and sliding window.
        
        Args:
            image: BGR image (H, W, 3)
            vehicle_state: Optional vehicle state
            
        Returns:
            LaneDetectionResult with steering correction and curvature
        """
        try:
            if self.M is None:
                self._update_transform_matrix()
            
            # Preprocess and warp
            binary = self._preprocess(image)
            warped = cv2.warpPerspective(binary, self.M, 
                                         (self.img_width, self.img_height))
            
            # Find lanes with sliding window
            left_fit, right_fit, left_conf, right_conf, out_img, debug_data = self._find_lanes(warped)
            self._last_debug_data = debug_data
            if self._enable_debug:
                self._debug_image = out_img

            
            # Smooth fits
            left_fit = self._smooth_fit(left_fit, self.left_fit_history)
            right_fit = self._smooth_fit(right_fit, self.right_fit_history)
            
            self.best_left_fit = left_fit
            self.best_right_fit = right_fit
            
            # Calculate offset and curvature
            offset, curvature = self._calculate_offset_curvature(left_fit, right_fit)
            
            # Convert offset to steering correction
            steer_gain = self.config.get('steer_gain', 1.0)
            curve_gain = self.config.get('curve_gain', 200.0)
            
            steering = steer_gain * offset + curve_gain * curvature
            steering = np.clip(steering, -0.5, 0.5)
            
            # Calculate confidence
            confidence = (left_conf + right_conf) / 2.0
            is_valid = left_fit is not None or right_fit is not None
            
            self._last_result = LaneDetectionResult(
                is_valid=is_valid,
                confidence=confidence,
                steering_correction=steering,
                lateral_offset=offset,
                curvature=curvature,
                left_lane_detected=left_fit is not None,
                right_lane_detected=right_fit is not None,
                raw_data={
                    'left_fit': left_fit.tolist() if left_fit is not None else None,
                    'right_fit': right_fit.tolist() if right_fit is not None else None,
                    'left_confidence': left_conf,
                    'right_confidence': right_conf,
                    'offset': offset,
                    'curvature': curvature,
                    **debug_data,
                },
                timestamp=time.time()
            )
            
        except Exception as e:
            if self.logger:
                self.logger.logger.debug(f"BEV lane detection error: {e}")
            self._last_result = LaneDetectionResult(timestamp=time.time())
        
        return self._last_result
    
    def _preprocess(self, image: np.ndarray) -> np.ndarray:
        """Convert image to binary lane mask"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # White lanes
        white_lower = np.array([0, 0, 180])
        white_upper = np.array([180, 60, 255])
        white_mask = cv2.inRange(hsv, white_lower, white_upper)
        
        # Yellow lanes
        yellow_lower = np.array([15, 80, 100])
        yellow_upper = np.array([35, 255, 255])
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        
        return cv2.bitwise_or(white_mask, yellow_mask)
    
    def _find_lanes(self, binary_warped: np.ndarray) -> Tuple:
        """Find lane lines using sliding window and return visualization + debug metrics."""
        # Create an output image to draw on and visualize the result
        out_img = None
        if self._enable_debug:
            out_img = np.zeros((binary_warped.shape[0], binary_warped.shape[1], 3), dtype=np.uint8)
            out_img[binary_warped > 0] = (70, 70, 70)  # lane-mask pixels as neutral gray

        # Histogram of bottom half
        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:, :], axis=0)
        midpoint = len(histogram) // 2

        left_hist_peak = float(np.max(histogram[:midpoint])) if midpoint > 0 else 0.0
        right_hist_peak = float(np.max(histogram[midpoint:])) if midpoint < len(histogram) else 0.0

        leftx_base = int(np.argmax(histogram[:midpoint])) if midpoint > 0 else 0
        rightx_base = int(np.argmax(histogram[midpoint:]) + midpoint) if midpoint < len(histogram) else 0

        # Sliding window
        window_height = binary_warped.shape[0] // self.nwindows
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx_current = leftx_base
        rightx_current = rightx_base

        left_lane_inds = []
        right_lane_inds = []

        for window in range(self.nwindows):
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height

            win_xleft_low = leftx_current - self.margin
            win_xleft_high = leftx_current + self.margin
            win_xright_low = rightx_current - self.margin
            win_xright_high = rightx_current + self.margin

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > self.minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > self.minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

            # Visualization
            if out_img is not None:
                cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (120, 120, 120), 2)
                cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (160, 160, 160), 2)

        # Concatenate indices
        left_lane_inds = np.concatenate(left_lane_inds) if left_lane_inds else np.array([])
        right_lane_inds = np.concatenate(right_lane_inds) if right_lane_inds else np.array([])

        # Fit polynomials
        left_fit = None
        right_fit = None
        left_conf = 0.0
        right_conf = 0.0
        lane_width = np.nan
        lane_width_rejected = False

        if len(left_lane_inds) > 50:
            leftx = nonzerox[left_lane_inds]
            lefty = nonzeroy[left_lane_inds]
            left_fit = np.polyfit(lefty, leftx, 2)
            left_conf = min(len(left_lane_inds) / 1000.0, 1.0)

        if len(right_lane_inds) > 50:
            rightx = nonzerox[right_lane_inds]
            righty = nonzeroy[right_lane_inds]
            right_fit = np.polyfit(righty, rightx, 2)
            right_conf = min(len(right_lane_inds) / 1000.0, 1.0)

        # Lane width sanity check
        if left_fit is not None and right_fit is not None:
            y_eval = binary_warped.shape[0]
            left_x = left_fit[0]*y_eval**2 + left_fit[1]*y_eval + left_fit[2]
            right_x = right_fit[0]*y_eval**2 + right_fit[1]*y_eval + right_fit[2]
            lane_width = abs(right_x - left_x)

            # Reject if lane width is unrealistic
            if lane_width < self.min_lane_width_px or lane_width > self.max_lane_width_px:
                lane_width_rejected = True
                if len(left_lane_inds) < len(right_lane_inds):
                    left_fit = None
                    left_conf = 0.0
                else:
                    right_fit = None
                    right_conf = 0.0

        # Visualize polynomial fits
        if out_img is not None:
            out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [210, 210, 210]
            out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [240, 240, 240]

            ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])

            if left_fit is not None:
                try:
                    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
                    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))]).astype(np.int32)
                    cv2.polylines(out_img, pts_left, False, (245, 245, 245), 3)
                except Exception:
                    pass

            if right_fit is not None:
                try:
                    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
                    pts_right = np.array([np.transpose(np.vstack([right_fitx, ploty]))]).astype(np.int32)
                    cv2.polylines(out_img, pts_right, False, (245, 245, 245), 3)
                except Exception:
                    pass

            _draw_text_outline(out_img, "Left/Right lane points + fit (grayscale)", (10, 22), (230, 230, 230), 0.5, 1)
            _draw_text_outline(out_img, f"L/R conf: {left_conf:.2f}/{right_conf:.2f}", (10, 44), (255, 255, 255), 0.5, 1)
            _draw_text_outline(out_img, f"L/R pts: {len(left_lane_inds)}/{len(right_lane_inds)}", (10, 66), (220, 220, 220), 0.48, 1)
            _draw_text_outline(out_img, f"Lane width px: {lane_width:.1f}" if not np.isnan(lane_width) else "Lane width px: n/a", (10, 88), (220, 220, 220), 0.48, 1)
            if lane_width_rejected:
                _draw_text_outline(out_img, "Rejected lane width out of range", (10, 110), (245, 245, 245), 0.5, 2)

        debug_data = {
            'left_points': int(len(left_lane_inds)),
            'right_points': int(len(right_lane_inds)),
            'left_hist_peak': left_hist_peak,
            'right_hist_peak': right_hist_peak,
            'left_base_x': leftx_base,
            'right_base_x': rightx_base,
            'lane_width_px': None if np.isnan(lane_width) else float(lane_width),
            'lane_width_rejected': lane_width_rejected,
        }

        return left_fit, right_fit, left_conf, right_conf, out_img, debug_data
    
    def _smooth_fit(self, new_fit, history: deque):
        """Smooth polynomial fit using history"""
        if new_fit is None:
            history.clear()
            return None
        history.append(new_fit)
        return np.mean(history, axis=0)
    
    def _calculate_offset_curvature(self, left_fit, right_fit) -> Tuple[float, float]:
        """Calculate lane center offset and curvature"""
        y_eval = self.img_height
        
        left_x = 0
        right_x = self.img_width
        lane_width = self.assumed_lane_width_px
        
        if left_fit is not None:
            left_x = left_fit[0]*y_eval**2 + left_fit[1]*y_eval + left_fit[2]
        if right_fit is not None:
            right_x = right_fit[0]*y_eval**2 + right_fit[1]*y_eval + right_fit[2]
        
        if left_fit is not None and right_fit is not None:
            lane_center = (left_x + right_x) / 2
            lane_width = right_x - left_x
        elif left_fit is not None:
            lane_center = left_x + self.single_lane_offset_px
        elif right_fit is not None:
            lane_center = right_x - self.single_lane_offset_px
        else:
            return 0.0, 0.0
        
        # Camera offset
        cam_offset_px = (self.camera_offset_m / self.lane_width_ref_m) * lane_width
        car_pos = (self.img_width / 2) - cam_offset_px
        offset = (car_pos - lane_center) / (lane_width / 2.0)
        offset = np.clip(offset, -1.0, 1.0)
        
        # Curvature proxy
        curvature = 0.0
        if left_fit is not None and right_fit is not None:
            curvature = (left_fit[0] + right_fit[0]) / 2
        elif left_fit is not None:
            curvature = left_fit[0]
        elif right_fit is not None:
            curvature = right_fit[0]
        
        return offset, curvature


class LaneNetDetector(LaneDetectorBase):
    """
    LaneNet-based lane detector (deep learning).
    
    Provides high accuracy lane detection using a neural network.
    Requires the pit.LaneNet module to be available.
    
    Config options:
        image_width: Input image width (default: 640)
        image_height: Input image height (default: 480)
        row_upper_bound: Upper row bound for detection (default: 200)
        use_clustering: Whether to use DBSCAN clustering (slower but separates lanes)
    """
    
    def __init__(self, config: Dict[str, Any] = None, logger=None):
        super().__init__(config, logger)
        
        self.image_width = self.config.get('image_width', 640)
        self.image_height = self.config.get('image_height', 480)
        self.row_upper_bound = self.config.get('row_upper_bound', 200)
        self.use_clustering = self.config.get('use_clustering', False)
        
        self.lanenet = None
        
    @property
    def method(self) -> LaneDetectionMethod:
        return LaneDetectionMethod.LANENET
    
    def initialize(self) -> bool:
        """Initialize LaneNet model"""
        try:
            from pit.LaneNet.nets import LaneNet
            
            self.lanenet = LaneNet(
                imageHeight=self.image_height,
                imageWidth=self.image_width,
                rowUpperBound=self.row_upper_bound
            )
            self._is_initialized = True
            
            if self.logger:
                self.logger.logger.info("LaneNet detector initialized")
            return True
            
        except ImportError as e:
            if self.logger:
                self.logger.logger.warning(f"LaneNet not available: {e}")
            return False
        except Exception as e:
            if self.logger:
                self.logger.logger.error(f"Failed to initialize LaneNet: {e}")
            return False
    
    def detect(self, image: np.ndarray,
               vehicle_state: Optional[Dict[str, float]] = None) -> LaneDetectionResult:
        """
        Detect lanes using LaneNet.
        
        Args:
            image: BGR image
            vehicle_state: Optional vehicle state
            
        Returns:
            LaneDetectionResult
        """
        if self.lanenet is None:
            self._last_result = LaneDetectionResult(timestamp=time.time())
            return self._last_result
        
        try:
            # Preprocess
            processed = self.lanenet.pre_process(image)
            
            # Predict
            binary_pred, instance_pred = self.lanenet.predict(processed)
            
            # Calculate lane metrics from prediction
            # Binary prediction gives lane mask
            if binary_pred is not None:
                lane_pixels = np.sum(binary_pred > 0.5)
                total_pixels = binary_pred.size
                confidence = min(lane_pixels / (total_pixels * 0.1), 1.0)
                
                # Calculate steering from lane position
                steering = self._calculate_steering_from_mask(binary_pred)
                
                self._last_result = LaneDetectionResult(
                    is_valid=confidence > 0.1,
                    confidence=confidence,
                    steering_correction=steering,
                    left_lane_detected=True,
                    right_lane_detected=True,
                    timestamp=time.time()
                )
            else:
                self._last_result = LaneDetectionResult(timestamp=time.time())
                
        except Exception as e:
            if self.logger:
                self.logger.logger.debug(f"LaneNet detection error: {e}")
            self._last_result = LaneDetectionResult(timestamp=time.time())
        
        return self._last_result
    
    def _calculate_steering_from_mask(self, mask: np.ndarray) -> float:
        """Calculate steering from lane mask"""
        try:
            # Find lane center at bottom of image
            height = mask.shape[0]
            bottom_region = mask[int(height*0.7):, :]
            
            # Calculate center of mass
            y_indices, x_indices = np.nonzero(bottom_region > 0.5)
            if len(x_indices) < 10:
                return 0.0
            
            lane_center = np.mean(x_indices)
            image_center = mask.shape[1] / 2
            
            # Normalize offset to steering
            offset = (lane_center - image_center) / (mask.shape[1] / 2)
            steering = -offset * 0.5  # Negative because if lane is right, steer left
            
            return np.clip(steering, -0.3, 0.3)
            
        except Exception:
            return 0.0
    
    def terminate(self):
        """Clean up LaneNet resources"""
        self.lanenet = None
        self._is_initialized = False
