"""
YOLO Server Physical - Refactored for Robustness
matches architecture of yolo_server_virtual.py but for physical QCar.
"""

import os
import sys
import cv2

import argparse
from dataclasses import dataclass

current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

# Retaining original camera import as requested
from pit.YOLO.utils import QCar2DepthAligned

# Using enhanced wrapper for consistency with virtual server (better rendering)
from YOLOv8Wrapper_Huy import YOLOv8Wrapper_Huy, DetectionBuffers
from YoLo import YOLOPublisher, YOLOVideoPublisher
from yolo_config import DEFAULT_CONFIG_PATH, YoloServerConfig, parse_bool_string

# Import Lane Detection and Interface
try:
    from LaneFollow import create_lane_detector
    from LaneFollow.lane_detection_interface import LaneDetectionResult

    LANE_MODULE_AVAILABLE = True
except ImportError as e:
    print(f"[SERVER] WARNING: LaneFollow module not available: {e}")
    LANE_MODULE_AVAILABLE = False


# =============================================================================
# Configuration
# =============================================================================
@dataclass
class ServerConfig:
    """Configuration for YOLO server."""

    ip_host: str = "localhost"
    width: int = 320
    height: int = 200
    car_id: int = 0
    probing: bool = False
    show_obstacle_box: bool = False
    image_width: int = 640
    image_height: int = 480
    config_path: str = DEFAULT_CONFIG_PATH
    distance_offset_m: float = 0.0
    video_port: int = (
        18760  # Base port; actual port = 18760 + car_id (set via --video-port)
    )

    @classmethod
    def from_args(cls) -> "ServerConfig":
        """Create config from command line arguments."""
        parser = argparse.ArgumentParser(prog="YOLO Server Physical")
        parser.add_argument("-i", "--ip_host", default="localhost")
        parser.add_argument("-p", "--probing", default="False")
        parser.add_argument("-w", "--width", type=int, default=320)
        parser.add_argument("-ht", "--height", type=int, default=200)
        parser.add_argument("-idx", "--caridx", type=int, default=0)
        parser.add_argument(
            "--config",
            type=str,
            default=DEFAULT_CONFIG_PATH,
            help="Path to yolo_config.yaml",
        )
        parser.add_argument("--video-port", type=int, default=18766)
        parser.add_argument(
            "--obsbox",
            action="store_true",
            help="Draw the center obstacle detection bounding box",
        )
        # Note: --car-id was used in original script, mapping both just in case
        parser.add_argument("--car-id", type=int, dest="caridx_alt", default=None)

        args = parser.parse_args()

        # Handle car_id / idx ambiguity
        cid = args.caridx
        if args.caridx_alt is not None:
            cid = args.caridx_alt
        probing = parse_bool_string(args.probing)
        if probing is None:
            probing = False

        yaml_config = YoloServerConfig.from_yaml(
            config_path=args.config,
            overrides={"car_id": cid, "probing": probing},
        )

        return cls(
            ip_host=args.ip_host,
            width=args.width,
            height=args.height,
            car_id=cid,
            probing=probing,
            show_obstacle_box=True,  # Always show the obstacle box
            config_path=args.config,
            distance_offset_m=yaml_config.yolo.postprocess.distance_offset_m,
            video_port=args.video_port,
        )


# =============================================================================
# Probe Manager - Handles observer connection and streaming
# =============================================================================


# =============================================================================
# Detection Buffers - Pre-allocated for performance
# =============================================================================
# DetectionBuffers imported from YOLOv8Wrapper_Huy


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
        self._init_lane_detector()
        self._init_publisher()
        self._init_video_publisher()

        # Pre-allocated buffers
        self.buffers = DetectionBuffers(
            image_width=config.image_width,
            image_height=config.image_height,
            distance_offset_m=config.distance_offset_m,
        )

        print(f"[SERVER] YOLOServerPhysical initialized for Car {config.car_id}")
        if abs(float(config.distance_offset_m)) > 1e-9:
            print(
                "[SERVER] YOLO distance offset: "
                f"published_distance = measured_distance - {config.distance_offset_m:.2f} m"
            )

    def _init_camera(self):
        """Initialize camera using the specified QCar2DepthAligned class."""
        # NOTE: Using hardcoded port 18777 as in original script for physical car
        # Just in case multiple cars run on same hardware (?) no,
        # usually 1 car = 1 OS instance.
        # Original script had: QCarImg = QCar2DepthAligned(port='18777')
        try:
            self.camera = QCar2DepthAligned(port="18777")
            print("[SERVER] Camera initialized (QCar2DepthAligned)")
        except Exception as e:
            print(f"[SERVER] Data capture init failed: {e}")
            raise e

    def _init_yolo(self):
        """Initialize YOLO detector."""
        # Using enhanced wrapper for better visualization in Probe
        self.yolo = YOLOv8Wrapper_Huy(
            imageHeight=self.config.image_height, imageWidth=self.config.image_width
        )
        print("[SERVER] YOLOv8 initialized")

    def _init_lane_detector(self):
        """Initialize UltraFast lane detector for physical car."""
        self.lane_enabled = False
        self.lane_detector = None

        if not LANE_MODULE_AVAILABLE:
            print("[SERVER] Lane module not available.")
            return

        try:
            # Create UltraFast lane detector using the modular factory
            self.lane_detector = create_lane_detector("ultrafast")
            if self.lane_detector and self.lane_detector.is_initialized:
                self.yolo.set_lane_detector(self.lane_detector)
                self.lane_enabled = True
                print("[SERVER] UltraFast Lane detector initialized")
            else:
                self.lane_detector = None
                print(
                    "[SERVER] UltraFast Lane detection initialization failed or fallback activated."
                )
        except Exception as e:
            print(f"[SERVER] Failed to init lane detector: {e}")

    def _init_publisher(self):
        """Initialize YOLO data publisher."""
        # HARDCODED PORT 18666 as requested
        self.publisher = YOLOPublisher(port="18666")
        print("[SERVER] Publisher initialized on port 18666")

    def _init_video_publisher(self):
        """Initialize video publisher if probing is enabled."""
        self.video_publisher = None
        if self.config.probing:
            # Use configured video port and IP
            video_port = str(self.config.video_port)
            # Always bind to all interfaces ('*') so the Ground Station can connect
            # from a different machine.  ip_host is intentionally not used here.
            self.video_publisher = YOLOVideoPublisher(ip="*", port=video_port)
            print(
                f"[SERVER] Video streaming enabled on *:{video_port} (all interfaces)"
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
            verbose=False,
        )

        results = self.yolo.post_processing(alignedDepth=raw_depth, clippingDistance=10)

        # Lane Detection
        lane_result = None
        if self.lane_enabled and self.lane_detector:
            lane_result = self.lane_detector.detect(raw_rgb)
            self.yolo.set_lane_result(lane_result)

        bboxes = None
        if hasattr(self.yolo, "predictions") and self.yolo.predictions is not None and len(self.yolo.predictions) > 0:
            bboxes = self.yolo.predictions[0].boxes.xyxy.cpu().numpy()
        else:
            bboxes = getattr(self.yolo, "bounding", None)
            
        # Build and send detection packet first for obstacle state
        self.buffers.fill_from_results(results, bounding_boxes=bboxes)
        self.buffers.fill_lane(lane_result)

        # Render after buffer fill so the distance text uses corrected distances.
        annotated = self.yolo.post_process_render(showFPS=True, show_lane_overlay=True)

        # Draw obstacle box if config says so
        if getattr(self.config, "show_obstacle_box", False):
            margin_x = (1.0 - self.buffers.center_box_width_ratio) / 2
            x_min = int(self.config.image_width * margin_x)
            x_max = int(self.config.image_width * (1.0 - margin_x))
            y_min = int(
                self.config.image_height * (1.0 - self.buffers.center_box_height_ratio)
            )
            y_max = int(self.config.image_height)

            if self.buffers.obstacle[0] > 0:
                color = (0, 0, 255)  # Red - obstacle
                text = "Obstacle Detected"
            else:
                color = (0, 255, 0)  # Green - clear
                text = "Clear Path"

            cv2.rectangle(annotated, (x_min, y_min), (x_max, y_max), color, 2)
            cv2.putText(
                annotated,
                text,
                (x_min + 5, y_min + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )

        # Send video if enabled
        if self.video_publisher:
            self.video_publisher.send(annotated)

        self.publisher.send(self.buffers.to_packet())

    def terminate(self):
        """Clean shutdown of all components."""
        print("[SERVER] Terminating...")
        self.running = False

        if hasattr(self, "camera"):
            try:
                self.camera.terminate()
            except:
                pass

        if hasattr(self, "publisher"):
            try:
                self.publisher.terminate()
            except:
                pass

        if self.video_publisher:
            self.video_publisher.terminate()

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


if __name__ == "__main__":
    main()
