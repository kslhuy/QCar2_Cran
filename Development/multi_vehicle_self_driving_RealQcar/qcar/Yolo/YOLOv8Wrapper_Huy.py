import cv2
import numpy as np
import torch
from pit.YOLO.nets import YOLOv8, MASK_COLORS_RGB
from pit.YOLO.utils import TrafficLight, Obstacle


from dataclasses import dataclass, field
from typing import List, Optional

@dataclass
class DetectionBuffers:
    """Pre-allocated detection buffers for efficient packet building."""
    
    BUFFER_SIZE = 7
    CLASS_NAMES = ['stop_sign', 'traffic', 'car', 'yield', 'person', 'lane']
    
    stop_sign: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))
    traffic: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))
    car: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))
    yield_sign: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))
    person: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))
    lane: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))
    
    def reset(self):
        """Reset all buffers to zero."""
        self.stop_sign.fill(0)
        self.traffic.fill(0)
        self.car.fill(0)
        self.yield_sign.fill(0)
        self.person.fill(0)
        self.lane.fill(0)
    
    def fill_from_results(self, results: list):
        """Fill buffers from YOLO detection results."""
        counts = {'car': 0, 'stop_sign': 0, 'traffic': 0, 'yield': 0, 'person': 0}
        
        for det in results:
            name = det.name
            if 'car' in name and counts['car'] < 6:
                counts['car'] += 1
                self.car[counts['car']] = det.distance
            elif 'stop sign' in name and counts['stop_sign'] < 6:
                counts['stop_sign'] += 1
                self.stop_sign[counts['stop_sign']] = det.distance
            elif 'red' in name and counts['traffic'] < 6:
                counts['traffic'] += 1
                self.traffic[counts['traffic']] = det.distance
            elif 'yield' in name and counts['yield'] < 6:
                counts['yield'] += 1
                self.yield_sign[counts['yield']] = det.distance
            elif 'person' in name and counts['person'] < 6:
                counts['person'] += 1
                self.person[counts['person']] = det.distance
        
        # Set counts in first element
        self.car[0] = counts['car']
        self.traffic[0] = counts['traffic']
        self.stop_sign[0] = counts['stop_sign']
        self.yield_sign[0] = counts['yield']
        self.person[0] = counts['person']
    
    def fill_lane(self, lane_result):
        """Fill lane buffer from lane detection result."""
        if lane_result is not None and lane_result.is_valid:
            self.lane[0] = lane_result.confidence
            self.lane[1] = lane_result.steering_correction
            self.lane[2] = lane_result.curvature if lane_result.curvature else 0.0
            self.lane[3] = lane_result.lateral_offset if lane_result.lateral_offset else 0.0
            self.lane[4] = 1.0 if lane_result.left_lane_detected else 0.0
            self.lane[5] = 1.0 if lane_result.right_lane_detected else 0.0
            self.lane[6] = 0.0  # Reserved
    
    def to_packet(self) -> np.ndarray:
        """Create send packet from all buffers."""
        return np.vstack((
            self.stop_sign, self.traffic, self.car,
            self.yield_sign, self.person, self.lane
        ))


class YOLOv8Wrapper_Huy(YOLOv8):
    """
    Enhanced YOLOv8 wrapper with unified rendering for both object detection and lane detection.
    
    Features:
    - Original YOLO post_processing and rendering
    - Integrated lane detection overlay
    - Unified post_process_render that combines both YOLO and lane visualization
    - Support for custom lane detector instances
    """
    
    def __init__(self, imageWidth=640, imageHeight=480, modelPath=None):
        """Initialize the enhanced YOLOv8 wrapper"""
        super().__init__(imageWidth, imageHeight, modelPath)
        self.lane_result = None
        self.lane_detector = None
        print('YOLOv8Wrapper_Huy initialized with unified rendering support')
    
    def set_lane_detector(self, lane_detector):
        """Set the lane detector instance for integrated rendering"""
        self.lane_detector = lane_detector
    
    def set_lane_result(self, lane_result):
        """Set the current lane detection result for rendering"""
        self.lane_result = lane_result
    
    def post_process_render(self, showFPS=True, bbox_thickness=4, 
                           lane_result=None, lane_detector=None, 
                           show_lane_overlay=True):
        """
        Unified rendering method that combines YOLO detections and lane detection.
        
        Args:
            showFPS (bool): Display FPS in the rendered image
            bbox_thickness (int): Thickness of bounding box outlines
            lane_result: LaneDetectionResult object (uses self.lane_result if None)
            lane_detector: Lane detector instance (uses self.lane_detector if None)
            show_lane_overlay (bool): Whether to show lane detection overlay
        
        Returns:
            ndarray: Annotated image with both YOLO detections and lane overlay
        """
        # Use provided or stored lane detection info
        lane_result = lane_result or self.lane_result
        lane_detector = lane_detector or self.lane_detector
        
        # First, render YOLO detections (original implementation)
        if not self.processedResults:
            # No YOLO detections, just resize input image
            if self.img.shape[:2] != self.inputShape:
                output_img = cv2.resize(self.img, (self.inputShape[1], self.inputShape[0]))
            else:
                output_img = self.img.copy()
            
            # Apply lane overlay even without YOLO detections
            if show_lane_overlay and lane_result is not None:
                if lane_detector is not None and hasattr(lane_detector, 'render_lane_overlay'):
                    output_img = lane_detector.render_lane_overlay(output_img, lane_result)
                else:
                    output_img = self._render_basic_lane_overlay(output_img, lane_result)
            
            return output_img
        
        # Render YOLO detections with masks
        colors = []
        masks = self.predictions[0].masks.data.cuda()
        boxes = self.predictions[0].boxes.xyxy.cpu().numpy().astype(int)
        imgClone = self.img.copy()
        
        for i in range(len(self.objectsDetected)):
            colors.append(MASK_COLORS_RGB[self.objectsDetected[i].astype(int)])
            name = self.processedResults[i].name
            x = self.processedResults[i].x
            y = self.processedResults[i].y
            distance = self.processedResults[i].distance
            
            cv2.rectangle(imgClone, (boxes[i, :2]), (boxes[i, 2:4]), colors[i], bbox_thickness)
            cv2.putText(imgClone, name, (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors[i], 2)
            
            if self._calc_distence:
                cv2.putText(imgClone, str(distance) + " m", (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors[i], 2)
        
        if showFPS:
            cv2.putText(imgClone, 'Inference FPS: ' + str(round(self.FPS)), 
                       (480, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Apply mask coloring
        imgTensor = torch.from_numpy(imgClone).to("cuda:0")
        imgMask = self.mask_color(masks, imgTensor, colors)
        
        if imgMask.shape[:2] != self.inputShape:
            imgMask = cv2.resize(imgMask, (self.inputShape[1], self.inputShape[0]))
        
        # Apply lane detection overlay on top of YOLO rendering
        if show_lane_overlay and lane_result is not None:
            if lane_detector is not None and hasattr(lane_detector, 'render_lane_overlay'):
                imgMask = lane_detector.render_lane_overlay(imgMask, lane_result)
            else:
                imgMask = self._render_basic_lane_overlay(imgMask, lane_result)
        
        return imgMask
    
    def _render_basic_lane_overlay(self, image: np.ndarray, lane_result) -> np.ndarray:
        """
        Render basic lane detection overlay when no detector instance is available.
        
        Args:
            image: Input image to overlay lane detection on
            lane_result: LaneDetectionResult object
        
        Returns:
            Image with lane detection overlay
        """
        overlay = image.copy()
        
        if not lane_result.is_valid:
            return overlay
        
        # Choose color based on confidence
        color = (0, 255, 0) if lane_result.confidence > 0.5 else (0, 255, 255)
        
        # Draw steering indicator arrow at bottom of image
        center_x = image.shape[1] // 2
        start_y = image.shape[0] - 30
        end_x = int(center_x + lane_result.steering_correction * 200)
        cv2.arrowedLine(overlay, (center_x, start_y), (end_x, start_y), 
                       color, 3, tipLength=0.3)
        
        # Draw lane detection information
        lane_info = f"Lane: Steer={lane_result.steering_correction:.3f} Conf={lane_result.confidence:.2f}"
        cv2.putText(overlay, lane_info, (10, image.shape[0] - 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Draw lateral offset if available
        if lane_result.lateral_offset is not None:
            offset_text = f"Offset: {lane_result.lateral_offset:.3f}"
            cv2.putText(overlay, offset_text, (10, image.shape[0] - 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Draw lane detection indicators
        indicator_y = image.shape[0] - 100
        if lane_result.left_lane_detected:
            cv2.circle(overlay, (20, indicator_y), 8, (0, 255, 0), -1)
            cv2.putText(overlay, "L", (15, indicator_y + 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        if lane_result.right_lane_detected:
            cv2.circle(overlay, (50, indicator_y), 8, (0, 255, 0), -1)
            cv2.putText(overlay, "R", (45, indicator_y + 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return overlay
    
    def render_combined(self, showFPS=True, show_yolo=True, show_lane=True):
        """
        Convenience method for rendering with flexible control over what to show.
        
        Args:
            showFPS: Show FPS counter
            show_yolo: Show YOLO detection overlays
            show_lane: Show lane detection overlays
        
        Returns:
            Rendered image
        """
        if show_yolo:
            return self.post_process_render(showFPS=showFPS, show_lane_overlay=show_lane)
        else:
            # Just show raw image with lane overlay
            img = self.img.copy()
            if show_lane and self.lane_result is not None:
                if self.lane_detector is not None and hasattr(self.lane_detector, 'render_lane_overlay'):
                    img = self.lane_detector.render_lane_overlay(img, self.lane_result)
                else:
                    img = self._render_basic_lane_overlay(img, self.lane_result)
            return img