"""
YOLO Server Physical - Refactored for Robustness
matches architecture of yolo_server_virtual.py but for physical QCar.
"""
import numpy as np
import time
import cv2
import os
import sys
import argparse
from dataclasses import dataclass, field
from typing import Optional, Any

# Retaining original camera import as requested
from pit.YOLO.utils import QCar2DepthAligned
from pal.utilities.probe import Probe
# Using enhanced wrapper for consistency with virtual server (better rendering)
from YOLOv8Wrapper_Huy import YOLOv8Wrapper_Huy
from Yolo.YoLo import YOLOPublisher

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
    image_width: int = 640
    image_height: int = 480
    
    @classmethod
    def from_args(cls) -> 'ServerConfig':
        """Create config from command line arguments."""
        parser = argparse.ArgumentParser(prog='YOLO Server Physical')
        parser.add_argument('-i', '--ip_host', default='localhost')
        parser.add_argument('-p', '--probing', default="False")
        parser.add_argument('-w', '--width', type=int, default=320)
        parser.add_argument('-ht', '--height', type=int, default=200)
        parser.add_argument('-idx', '--caridx', type=int, default=0)
        # Note: --car-id was used in original script, mapping both just in case
        parser.add_argument('--car-id', type=int, dest='caridx_alt', default=None)
        
        args = parser.parse_args()
        
        # Handle car_id / idx ambiguity
        cid = args.caridx
        if args.caridx_alt is not None:
            cid = args.caridx_alt
            
        return cls(
            ip_host=args.ip_host,
            width=args.width,
            height=args.height,
            car_id=cid,
            probing=(args.probing == "True")
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
        try:
            self.probe = Probe(ip=self.ip_host)
            # Use car_id directly as numDisplays for physical cars (as per original script)
            self.probe.numDisplays = self.car_id 
            self.probe.add_display(
                imageSize=[self.height, self.width, 3],
                name=f'YOLO Car {self.car_id}',
                scalingFactor=1
            )
            print(f"[PROBE] Initialized for Car {self.car_id}")
        except Exception as e:
            print(f"[PROBE] Init failed: {e}")
            self.probe = None
    
    def send_frame(self, image: np.ndarray) -> bool:
        """Send frame with adaptive skip and auto-reconnect. Returns True if sent."""
        if self.probe is None:
            self._attempt_reconnect(time.time())
            return False
            
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
            return False
        
        try:
            # Resize for transmission
            resized = cv2.resize(image, (self.width, self.height))
            
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
                print(f"[PROBE] Send error: {e}")
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
        print(f"[PROBE] Reconnecting... (failures: {self.consecutive_failures})")
        
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
    
    # 5 rows for objects, +1 for lane data compatibility (even if empty)
    # The original server sent 5 rows. virtual sends 6.
    # To be safe and compatible with updated YOLOReceiver, we can send 6.
    # But if physical car receiver expects 5, we stick to 5. 
    # Checking state_base.py -> YoLo.py -> YOLOReceiver:
    # it expects 6 rows: receiveBuffer=np.zeros((6,7) ...
    # So we MUST send 6 rows now.
    
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
        
    def to_packet(self) -> np.ndarray:
        """Create send packet from all buffers."""
        # Stack 6 arrays to match YOLOReceiver expectation (6, 7)
        return np.vstack((
            self.stop_sign, self.traffic, self.car,
            self.yield_sign, self.person, self.lane
        ))


# =============================================================================
# Main YOLO Server Class for Physical Car
# =============================================================================
class YOLOServerPhysical:
    """
    Main YOLO server for Physical QCar.
    - Uses hardcoded port 18666
    - Uses QCar2DepthAligned (pit.YOLO.utils)
    - No local display (Probe only)
    """
    
    def __init__(self, config: ServerConfig):
        self.config = config
        self.running = False
        
        # Initialize components
        self._init_camera()
        self._init_yolo()
        self._init_publisher()
        self._init_probe()
        
        # Pre-allocated buffers
        self.buffers = DetectionBuffers()
        
        print(f"[SERVER] YOLOServerPhysical initialized for Car {config.car_id}")
    
    def _init_camera(self):
        """Initialize camera using the specified QCar2DepthAligned class."""
        # NOTE: Using hardcoded port 18777 as in original script for physical car
        # Just in case multiple cars run on same hardware (?) no, 
        # usually 1 car = 1 OS instance. 
        # Original script had: QCarImg = QCar2DepthAligned(port='18777')
        try:
            self.camera = QCar2DepthAligned(port='18777')
            print("[SERVER] Camera initialized (QCar2DepthAligned)")
        except Exception as e:
            print(f"[SERVER] Data capture init failed: {e}")
            raise e
    
    def _init_yolo(self):
        """Initialize YOLO detector."""
        # Using enhanced wrapper for better visualization in Probe
        self.yolo = YOLOv8Wrapper_Huy(
            imageHeight=self.config.image_height,
            imageWidth=self.config.image_width
        )
        print("[SERVER] YOLOv8 initialized")
    
    def _init_publisher(self):
        """Initialize YOLO data publisher."""
        # HARDCODED PORT 18666 as requested
        self.publisher = YOLOPublisher(port='18666')
        print("[SERVER] Publisher initialized on port 18666")
    
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
        
        # Original logic: just read .rgb and .depth from camera object
        # QCar2DepthAligned puts them in .rgb and .depth attributes
        raw_rgb = self.camera.rgb
        raw_depth = self.camera.depth
        
        # YOLO detection
        # Note: YOLOv8Wrapper_Huy might expect 640x480. 
        # If camera gives 640x480, we are good.
        processed = self.yolo.pre_process(raw_rgb)
        
        self.yolo.predict(
            inputImg=processed,
            classes=[0, 2, 9, 11, 33],
            confidence=0.4,
            half=True,
            verbose=False
        )
        
        results = self.yolo.post_processing(alignedDepth=raw_depth, clippingDistance=10)
        
        # Render annotated image
        # Even without Lane Detection, this renders bounding boxes nicer
        annotated = self.yolo.post_process_render(showFPS=True, show_lane_overlay=True)
        
        # Send to probe if enabled
        if self.probe_manager:
            self.probe_manager.send_frame(annotated)
        
        # Build and send detection packet
        self.buffers.fill_from_results(results)
        # No lane data for physical car yet, buffer stays 0
        
        self.publisher.send(self.buffers.to_packet())
    
    def terminate(self):
        """Clean shutdown of all components."""
        print("[SERVER] Terminating...")
        self.running = False
        
        if hasattr(self, 'camera'):
            try:
                self.camera.terminate()
            except:
                pass
        
        if hasattr(self, 'publisher'):
            try:
                self.publisher.terminate()
            except:
                pass

        if self.probe_manager:
            self.probe_manager.terminate()
        
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
    try:
        with YOLOServerPhysical(config) as server:
            server.run()
    except Exception as e:
        print(f"[FATAL] Server crashed: {e}")
        # Try to print traceback
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
