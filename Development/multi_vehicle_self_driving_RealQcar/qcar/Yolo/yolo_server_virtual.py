"""
YOLO Server Virtual - Clean modular implementation
Provides YOLO object detection and lane detection for virtual QCar.
"""

# import numpy as np
# import time
import cv2
import os
import sys
import argparse
from dataclasses import dataclass, field
from typing import List

from DepthAlignment.QCar2DepthAlignedCamera import QCar2DepthAlignedCamera
from YOLOv8Wrapper_Huy import YOLOv8Wrapper_Huy, DetectionBuffers
from qvl.multi_agent import readRobots
from YoLo import YOLOPublisher, YOLOVideoPublisher
from pit.YOLO.utils import QCar2DepthAligned


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
LEGACY_CLASS_IDS = [0, 2, 9, 11, 33]
TRAFFIC_PLUS_CLASS_IDS = [0, 1, 2, 3, 5, 7, 9, 10, 11, 33]


def _parse_class_ids(raw: str) -> List[int]:
    """Parse comma-separated class ids and return unique, ordered ints."""
    ids = []
    for token in raw.split(","):
        token = token.strip()
        if not token:
            continue
        ids.append(int(token))
    # Keep deterministic order while removing duplicates
    return list(dict.fromkeys(ids))


@dataclass
class ServerConfig:
    """Configuration for YOLO server."""

    car_id: int = 0
    show_image: bool = False
    show_lane_debug: bool = False
    show_obstacle_box: bool = True
    image_width: int = 640
    image_height: int = 480
    probing: bool = False
    use_tracking: bool = True  # Use BoT-SORT tracking instead of per-frame detect
    model_path: str = ""
    class_ids: List[int] = field(default_factory=lambda: TRAFFIC_PLUS_CLASS_IDS.copy())
    confidence: float = 0.4

    @classmethod
    def from_args(cls) -> "ServerConfig":
        """Create config from command line arguments."""
        parser = argparse.ArgumentParser(prog="YOLO Server Virtual")
        parser.add_argument("-idx", "--caridx", type=int, default=0)
        parser.add_argument("-s", "--show-image", action="store_true")
        parser.add_argument("-p", "--probing", default="False")
        parser.add_argument("--show-lane-debug", action="store_true")
        parser.add_argument(
            "--no-tracking",
            action="store_true",
            help="Disable BoT-SORT tracking, use per-frame detection",
        )
        parser.add_argument(
            "--obsbox",
            action="store_true",
            help="Draw the center obstacle detection bounding box",
        )
        parser.add_argument(
            "--classes-preset",
            choices=["legacy", "traffic_plus"],
            default="traffic_plus",
            help="Detection class preset for off-the-shelf models",
        )
        parser.add_argument(
            "--classes",
            type=str,
            default="",
            help="Comma-separated class ids (overrides preset), e.g. 0,1,2,3,5,7,9,10,11,33",
        )
        parser.add_argument(
            "--confidence",
            type=float,
            default=0.4,
            help="YOLO confidence threshold (0.0-1.0)",
        )
        parser.add_argument(
            "--model-path",
            type=str,
            default="",
            help="Optional YOLO model path (off-the-shelf weights)",
        )
        args = parser.parse_args()

        class_ids = (
            LEGACY_CLASS_IDS.copy()
            if args.classes_preset == "legacy"
            else TRAFFIC_PLUS_CLASS_IDS.copy()
        )
        if args.classes:
            class_ids = _parse_class_ids(args.classes)

        return cls(
            car_id=args.caridx,
            show_image=args.show_image,
            show_lane_debug=args.show_lane_debug,
            show_obstacle_box=True,  # Always show the obstacle box
            probing=(args.probing == "True"),
            use_tracking=not args.no_tracking,
            model_path=args.model_path,
            class_ids=class_ids,
            confidence=float(max(0.0, min(1.0, args.confidence))),
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

        # Pre-allocated buffers (set image_width for lane-side filtering)
        self.buffers = DetectionBuffers(
            image_width=config.image_width,
            image_height=config.image_height,
        )

        tracking_str = (
            "tracking (BoT-SORT)" if config.use_tracking else "per-frame detection"
        )
        print(
            f"[SERVER] YOLOServerVirtual initialized for Car {config.car_id} [{tracking_str}]"
        )
        print(
            f"[SERVER] YOLO classes={self.config.class_ids}, conf={self.config.confidence:.2f}"
        )
        if self.config.model_path:
            print(f"[SERVER] YOLO model override: {self.config.model_path}")

    def _init_camera(self):
        """Initialize depth-aligned camera."""
        robots = readRobots()
        name = f"QC2_{self.config.car_id}"
        car_config = robots[name]
        # try:
        #     self.camera = QCar2DepthAligned(video3dPort=str(car_config["video3dPort"]))
        #     print("[SERVER] Camera initialized (QCar2DepthAligned)")
        # except Exception as e:
        #     print(f"[SERVER] Data capture init failed: {e}")
        #     raise e

        self.camera = QCar2DepthAlignedCamera(
            imageWidth=self.config.image_width,
            imageHeight=self.config.image_height,
            use_intrinsics=False,
            clipping_distance=5.0,
            video3dPort=car_config["video3dPort"],
            load_settings=True,
            use_fast_alignment=True,
        )

    def _init_yolo(self):
        """Initialize YOLO detector."""
        model_path = self.config.model_path if self.config.model_path else None
        self.yolo = YOLOv8Wrapper_Huy(
            imageHeight=self.config.image_height,
            imageWidth=self.config.image_width,
            modelPath=model_path,
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
        data_port = f"1866{self.config.car_id}"
        self.publisher = YOLOPublisher(port=data_port)
        print(f"[SERVER] Data stream on {data_port}")

        self.video_publisher = None
        if self.config.probing:
            # Video port: 1876x
            video_port = f"1876{self.config.car_id}"
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
        rgb = cv2.resize(
            cropped_rgb, (self.config.image_width, self.config.image_height)
        )
        depth = cv2.resize(
            cropped_depth, (self.config.image_width, self.config.image_height)
        )

        # YOLO detection (tracking or per-frame)
        processed = self.yolo.pre_process(rgb)
        if self.config.use_tracking:
            self.yolo.track(
                inputImg=processed,
                classes=self.config.class_ids,
                confidence=self.config.confidence,
                half=True,
                verbose=False,
            )
        else:
            self.yolo.predict(
                inputImg=processed,
                classes=self.config.class_ids,
                confidence=self.config.confidence,
                half=True,
                verbose=False,
            )
        results = self.yolo.post_processing(alignedDepth=depth, clippingDistance=10)

        # Lane detection
        lane_result = None
        if self.lane_enabled and self.lane_detector:
            lane_result = self.lane_detector.detect(rgb)
            self.yolo.set_lane_result(lane_result)

        # Render annotated image
        annotated = self.yolo.post_process_render(showFPS=True, show_lane_overlay=True)

        # Build detection packet to update obstacle logic from results BEFORE rendering box
        bboxes = None
        if hasattr(self.yolo, "predictions") and self.yolo.predictions is not None and len(self.yolo.predictions) > 0:
            bboxes = self.yolo.predictions[0].boxes.xyxy.cpu().numpy()
        else:
            bboxes = getattr(self.yolo, "bounding", None)
            
        self.buffers.fill_from_results(results, bounding_boxes=bboxes)
        # self.buffers.fill_lane(lane_result)

        # Draw obstacle box if config says so
        if getattr(self.config, "show_obstacle_box", True):
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

        # Send video over ZMQ if probing is enabled
        if self.video_publisher:
            self.video_publisher.send(annotated)

        # Show image locally if enabled
        if self.config.show_image:
            cv2.imshow("YOLO Server", annotated)
            if cv2.waitKey(1) == ord("q"):
                self.running = False
                return

        # Show lane debug if enabled
        if self.config.show_lane_debug and self.lane_detector:
            if hasattr(self.lane_detector, "get_debug_image"):
                debug_img = self.lane_detector.get_debug_image()
                if debug_img is not None:
                    cv2.imshow("Lane Debug", debug_img)

        # Publish the already-filled payload
        self.publisher.send(self.buffers.to_packet())

    def terminate(self):
        """Clean shutdown of all components."""
        print("[SERVER] Terminating...")
        self.running = False

        if hasattr(self, "camera"):
            self.camera.terminate()

        if self.lane_detector:
            self.lane_detector.terminate()
            print("[LANE] Lane detector terminated")

        if hasattr(self, "video_publisher"):
            self.video_publisher.terminate()

        if hasattr(self, "publisher"):
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


if __name__ == "__main__":
    main()
