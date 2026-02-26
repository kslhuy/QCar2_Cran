"""
YOLO Server Virtual - Clean modular implementation
Provides YOLO object detection and lane detection for virtual QCar.
"""
import numpy as np
import time
import cv2
import os
import sys
import argparse
from dataclasses import dataclass, field
from typing import Optional, Any

from DepthAlignment.QCar2DepthAlignedCamera import QCar2DepthAlignedCamera
from YOLOv8Wrapper_Huy import YOLOv8Wrapper_Huy, DetectionBuffers
from qvl.multi_agent import readRobots
from YoLo import YOLOPublisher, YOLOVideoPublisher


# Add Controller path for LaneFusion import
_qcar_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _qcar_dir not in sys.path:
    sys.path.insert(0, _qcar_dir)

# Lane detection module (optional)
try:
    from LaneFollow import create_lane_detector_from_config, LaneDetectionResult
    LANE_MODULE_AVAILABLE = True
except ImportError as e:
    print(f"[LANE] WARNING: LaneFollow module not available: {e}")
    LANE_MODULE_AVAILABLE = False
    create_lane_detector_from_config = None
    LaneDetectionResult = None


# =============================================================================
# Configuration
# =============================================================================
@dataclass
class ServerConfig:
    """Configuration for YOLO server."""
    car_id: int = 0
    show_image: bool = False
    show_lane_debug: bool = False
    image_width: int = 640
    image_height: int = 480
    probing: bool = False
    
    @classmethod
    def from_args(cls) -> 'ServerConfig':
        """Create config from command line arguments."""
        parser = argparse.ArgumentParser(prog='YOLO Server Virtual')
        parser.add_argument('-idx', '--caridx', type=int, default=0)
        parser.add_argument('-s', '--show-image', action='store_true')
        parser.add_argument('-p', '--probing', default="False")
        parser.add_argument('--show-lane-debug', action='store_true')
        args = parser.parse_args()
        
        return cls(
            car_id=args.caridx,
            show_image=args.show_image,
            show_lane_debug=args.show_lane_debug,
            probing=(args.probing == "True")
        )


# =============================================================================
# Detection Buffers - Pre-allocated for performance
# =============================================================================
# DetectionBuffers imported from YOLOv8Wrapper_Huy


# =============================================================================
# Main YOLO Server Class
# =============================================================================
class YOLOServerVirtual:
    """Main YOLO server handling detection and streaming."""
    
    def __init__(self, config: ServerConfig):
        self.config = config
        self.running = False
        
        # Initialize components
        self._init_camera()
        self._init_yolo()
        self._init_lane_detector()
        self._init_publisher()
        
        # Pre-allocated buffers
        self.buffers = DetectionBuffers()
        
        print(f"[SERVER] YOLOServerVirtual initialized for Car {config.car_id}")
    
    def _init_camera(self):
        """Initialize depth-aligned camera."""
        # robots = readRobots()
        # name = f"QC2_{self.config.car_id}"
        # car_config = robots[name]
        
        self.camera = QCar2DepthAlignedCamera(
            imageWidth=self.config.image_width,
            imageHeight=self.config.image_height,
            use_intrinsics=True,
            clipping_distance=10.0,
            # video3dPort=car_config['video3dPort'],
            load_settings=True,
            use_fast_alignment=True
        )
    
    def _init_yolo(self):
        """Initialize YOLO detector."""
        self.yolo = YOLOv8Wrapper_Huy(
            imageHeight=self.config.image_height,
            imageWidth=self.config.image_width,
            convert_tensorrt=False
        )
    
    def _init_lane_detector(self):
        """Initialize lane detector if available."""
        self.lane_detector = None
        self.lane_enabled = False
        
        if not LANE_MODULE_AVAILABLE:
            print("[LANE] Module not available")
            return
        
        try:
            self.lane_detector = create_lane_detector_from_config()
            if self.lane_detector:
                self.lane_enabled = True
                self.yolo.set_lane_detector(self.lane_detector)
                print("[LANE] Lane detector initialized and integrated")
        except Exception as e:
            print(f"[LANE] Initialization failed: {e}")
    
    def _init_publisher(self):
        """Initialize YOLO data and video publishers."""
        # Data port: 1866x
        data_port = f'1866{self.config.car_id}'
        self.publisher = YOLOPublisher(port=data_port)
        print(f"[SERVER] Data stream on {data_port}")

        self.video_publisher = None
        if self.config.probing:
            # Video port: 1876x
            video_port = f'1876{self.config.car_id}'
            self.video_publisher = YOLOVideoPublisher(port=video_port)
            print(f"[SERVER] Video stream enabled on {video_port}")
    
    def run(self):
        """Main processing loop."""
        self.running = True
        print(f"[SERVER] Starting main loop for Car {self.config.car_id}")
        
        try:
            while self.running:
                self._process_frame()
        except KeyboardInterrupt:
            print("[SERVER] User interrupted")
        finally:
            self.terminate()
    
    def _process_frame(self):
        """Process a single frame."""
        # Reset buffers
        self.buffers.reset()
        
        # Get aligned RGB and depth
        self.camera.read()
        
        # Crop bottom and resize
        cropped_rgb = self.camera.rgb[:-40, :, :]
        cropped_depth = self.camera.depth[:-40, :]
        rgb = cv2.resize(cropped_rgb, (self.config.image_width, self.config.image_height))
        depth = cv2.resize(cropped_depth, (self.config.image_width, self.config.image_height))
        
        # YOLO detection
        processed = self.yolo.pre_process(rgb)
        self.yolo.predict(
            inputImg=processed,
            classes=[0, 2, 9, 11, 33],
            confidence=0.4,
            half=True,
            verbose=False
        )
        results = self.yolo.post_processing(alignedDepth=depth, clippingDistance=10)
        
        # Lane detection
        lane_result = None
        if self.lane_enabled and self.lane_detector:
            lane_result = self.lane_detector.detect(rgb)
            self.yolo.set_lane_result(lane_result)
        
        # Render annotated image
        annotated = self.yolo.post_process_render(showFPS=True, show_lane_overlay=True)
        
        # Send video over ZMQ if probing is enabled
        if self.video_publisher:
            self.video_publisher.send(annotated)
        
        # Show image locally if enabled
        if self.config.show_image:
            cv2.imshow('YOLO Server', annotated)
            if cv2.waitKey(1) == ord('q'):
                self.running = False
                return
        
        # Show lane debug if enabled
        if self.config.show_lane_debug and self.lane_detector:
            if hasattr(self.lane_detector, 'get_debug_image'):
                debug_img = self.lane_detector.get_debug_image()
                if debug_img is not None:
                    cv2.imshow('Lane Debug', debug_img)
        
        # Build and send detection packet
        self.buffers.fill_from_results(results)
        self.buffers.fill_lane(lane_result)
        self.publisher.send(self.buffers.to_packet())
    
    def terminate(self):
        """Clean shutdown of all components."""
        print("[SERVER] Terminating...")
        self.running = False
        
        if hasattr(self, 'camera'):
            self.camera.terminate()
        
        if self.lane_detector:
            self.lane_detector.terminate()
            print("[LANE] Lane detector terminated")
            
        if hasattr(self, 'video_publisher'):
            self.video_publisher.terminate()
            
        if hasattr(self, 'publisher'):
            self.publisher.terminate()
        
        cv2.destroyAllWindows()
        print("[SERVER] Shutdown complete")
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.terminate()
        return False


# =============================================================================
# Entry Point
# =============================================================================
def main():
    config = ServerConfig.from_args()
    with YOLOServerVirtual(config) as server:
        server.run()


if __name__ == '__main__':
    main()
