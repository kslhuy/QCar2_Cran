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
from pal.utilities.probe import Probe
from YOLOv8Wrapper_Huy import YOLOv8Wrapper_Huy
from qvl.multi_agent import readRobots
from YoLo import YOLOPublisher

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
    ip_host: str = 'localhost'
    width: int = 320
    height: int = 200
    car_id: int = 0
    probing: bool = False
    show_image: bool = False
    show_lane_debug: bool = False
    image_width: int = 640
    image_height: int = 480
    
    @classmethod
    def from_args(cls) -> 'ServerConfig':
        """Create config from command line arguments."""
        parser = argparse.ArgumentParser(prog='YOLO Server Virtual')
        parser.add_argument('-i', '--ip_host', default='localhost')
        parser.add_argument('-p', '--probing', default="False")
        parser.add_argument('-w', '--width', type=int, default=320)
        parser.add_argument('-ht', '--height', type=int, default=200)
        parser.add_argument('-idx', '--caridx', type=int, default=0)
        parser.add_argument('-s', '--show-image', action='store_true')
        parser.add_argument('--show-lane-debug', action='store_true')
        args = parser.parse_args()
        
        return cls(
            ip_host=args.ip_host,
            width=args.width,
            height=args.height,
            car_id=args.caridx,
            probing=(args.probing == "True"),
            show_image=args.show_image,
            show_lane_debug=args.show_lane_debug
        )


# =============================================================================
# Probe Manager - Handles observer connection and streaming
# =============================================================================
class ProbeManager:
    """Manages probe connection with auto-reconnect and adaptive frame skipping."""
    
    DEAD_THRESHOLD = 5  # Consecutive failures before reconnection
    RECONNECT_COOLDOWN = 2.0  # Seconds between reconnection attempts
    
    def __init__(self, ip_host: str, car_id: int, height: int, width: int):
        self.ip_host = ip_host
        self.car_id = car_id
        self.height = height
        self.width = width
        
        # Connection state
        self.probe: Optional[Probe] = None
        self.frame_count = 0
        self.frame_skip = 4
        self.consecutive_failures = 0
        self.total_failures = 0
        self.last_success_time = time.time()
        self.last_reconnect_time = 0.0
        
        self._create_probe()
    
    def _create_probe(self):
        """Create and initialize probe instance."""
        self.probe = Probe(ip=self.ip_host)
        self.probe.numDisplays = self.car_id + 50
        self.probe.add_display(
            imageSize=[self.height, self.width, 3],
            name=f'YOLO Car {self.car_id}',
            # scalingFactor=1
        )
        print(f"[PROBE] Initialized for Car {self.car_id} on port {18851 + self.car_id}")
    
    def send_frame(self, image: np.ndarray) -> bool:
        """Send frame with adaptive skip and auto-reconnect. Returns True if sent."""
        self.frame_count += 1
        current_time = time.time()
        
        # Check if connection is dead
        if self._is_connection_dead(current_time):
            self._attempt_reconnect(current_time)
        
        # Adaptive frame skipping
        self._adjust_frame_skip(current_time)
        
        # Skip frames to reduce load
        if self.frame_count % self.frame_skip != 0:
            return False
        
        # Try to send
        self.probe.check_connection()
        if not self.probe.connected:
            if self.frame_count % 200 == 0:
                print(f"[PROBE] Car {self.car_id} waiting for observer...")
            return False
        
        try:
            # Compress and resize
            _, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            compressed = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
            resized = cv2.resize(compressed, (self.width, self.height))
            
            success = self.probe.send(name=f'YOLO Car {self.car_id}', imageData=resized)
            
            if success is True:
                self.last_success_time = current_time
                self.consecutive_failures = 0
                self.total_failures = max(0, self.total_failures - 1)
                return True
            else:
                self._record_failure()
                return False
        except Exception as e:
            self._record_failure()
            if self.frame_count % 100 == 0:
                print(f"[PROBE] Send error Car {self.car_id}: {e}")
            return False
    
    def _is_connection_dead(self, current_time: float) -> bool:
        """Check if connection appears dead."""
        return (
            self.consecutive_failures >= self.DEAD_THRESHOLD or
            (current_time - self.last_success_time > 5.0 and self.consecutive_failures > 0)
        )
    
    def _attempt_reconnect(self, current_time: float):
        """Attempt to reconnect if cooldown has passed."""
        if current_time - self.last_reconnect_time < self.RECONNECT_COOLDOWN:
            return
        
        self.last_reconnect_time = current_time
        print(f"[PROBE] Reconnecting Car {self.car_id} (failures: {self.consecutive_failures})...")
        
        try:
            if self.probe:
                try:
                    self.probe.terminate()
                except:
                    pass
            time.sleep(0.3)
            self._create_probe()
            self.consecutive_failures = 0
            self.total_failures = 0
        except Exception as e:
            print(f"[PROBE] Reconnection failed: {e}")
    
    def _adjust_frame_skip(self, current_time: float):
        """Dynamically adjust frame skip based on success rate."""
        if self.total_failures > 10 and self.frame_skip < 8:
            self.frame_skip = min(8, self.frame_skip + 1)
            self.total_failures = 0
        elif current_time - self.last_success_time < 1.0 and self.frame_skip > 3:
            self.frame_skip = max(3, self.frame_skip - 1)
    
    def _record_failure(self):
        """Record a send failure."""
        self.consecutive_failures += 1
        self.total_failures += 1
    
    def terminate(self):
        """Clean shutdown."""
        if self.probe:
            try:
                self.probe.terminate()
            except:
                pass


# =============================================================================
# Detection Buffers - Pre-allocated for performance
# =============================================================================
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
        self._init_probe()
        
        # Pre-allocated buffers
        self.buffers = DetectionBuffers()
        
        print(f"[SERVER] YOLOServerVirtual initialized for Car {config.car_id}")
    
    def _init_camera(self):
        """Initialize depth-aligned camera."""
        robots = readRobots()
        name = f"QC2_{self.config.car_id}"
        car_config = robots[name]
        
        self.camera = QCar2DepthAlignedCamera(
            imageWidth=self.config.image_width,
            imageHeight=self.config.image_height,
            use_intrinsics=True,
            clipping_distance=10.0,
            video3dPort=car_config['video3dPort'],
            load_settings=True,
            use_fast_alignment=True
        )
    
    def _init_yolo(self):
        """Initialize YOLO detector."""
        self.yolo = YOLOv8Wrapper_Huy(
            imageHeight=self.config.image_height,
            imageWidth=self.config.image_width
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
        """Initialize YOLO data publisher."""
        port = f'1866{self.config.car_id}'
        self.publisher = YOLOPublisher(port=port)
    
    def _init_probe(self):
        """Initialize probe manager if enabled."""
        self.probe_manager = None
        if self.config.probing:
            self.probe_manager = ProbeManager(
                self.config.ip_host,
                self.config.car_id,
                self.config.height,
                self.config.width
            )
    
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
        
        # Show image if enabled
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
        
        # Send to probe if enabled
        if self.probe_manager:
            self.probe_manager.send_frame(annotated)
        
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
        
        if self.probe_manager:
            self.probe_manager.terminate()
        
        if self.lane_detector:
            self.lane_detector.terminate()
            print("[LANE] Lane detector terminated")
        
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
