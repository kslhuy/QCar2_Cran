"""
YOLO Server Virtual - Clean modular implementation
Provides YOLO object detection and lane detection for virtual QCar.
Optional LiDAR-camera fusion for improved obstacle distance estimation.
"""

import numpy as np
import cv2
import os
import sys
import argparse
from typing import Optional

from DepthAlignment.QCar2DepthAlignedCamera import QCar2DepthAlignedCamera
from YOLOv8Wrapper_Huy import YOLOv8Wrapper_Huy, DetectionBuffers
from qvl.multi_agent import readRobots
from YoLo import YOLOPublisher, YOLOVideoPublisher
from yolo_config import DEFAULT_CONFIG_PATH, YoloServerConfig, parse_bool_string

# LiDAR Fusion imports (optional, behind --lidar-fusion flag)
try:
    from pal.products.qcar import QCarLidar
    # _lidar_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'LidarFusion')
    # if _lidar_dir not in sys.path:
    #     sys.path.insert(0, _lidar_dir)
    from LidarFusion.RobotGeometryManager import RobotGeometryManager
    from LidarFusion.qcar_utils_HUY import *
    LIDAR_FUSION_AVAILABLE = True
except ImportError as e:
    print(f"[LIDAR] WARNING: LiDAR fusion modules not available: {e}")
    LIDAR_FUSION_AVAILABLE = False
    EnhancedObstacle = None

if LIDAR_FUSION_AVAILABLE:
    try:
        from LidarFusion.YOLOv8Wrapper_Huy_lidar import EnhancedObstacle
    except Exception:
        EnhancedObstacle = None


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


def _parse_cli_args():
    """Minimal CLI surface. Tunables live in yolo_config.yaml."""
    parser = argparse.ArgumentParser(prog="YOLO Server Virtual")
    parser.add_argument(
        "--config",
        type=str,
        default=DEFAULT_CONFIG_PATH,
        help="Path to yolo_config.yaml",
    )
    parser.add_argument(
        "-idx",
        "--caridx",
        "--id",
        type=int,
        default=None,
        help="Override server.car_id from yaml",
    )
    parser.add_argument(
        "-p",
        "--probing",
        "--probe",
        type=str,
        default=None,
        help="Override server.probing (True/False)",
    )
    parser.add_argument(
        "-s",
        "--show-image",
        "--show",
        action="store_true",
        help="Override server.show_image to True",
    )
    return parser.parse_args()


# =============================================================================
# Detection Buffers - Pre-allocated for performance
# =============================================================================
# DetectionBuffers imported from YOLOv8Wrapper_Huy


# =============================================================================
# Main YOLO Server Class
# =============================================================================
class YOLOServerVirtual:
    """Main YOLO server handling detection and streaming."""

    def __init__(self, config: YoloServerConfig):
        self.config = config
        self.running = False
        self._terminated = False
        self._last_lidar_angles_body = np.empty((0,), dtype=np.float32)
        self._last_lidar_distances = np.empty((0,), dtype=np.float32)
        self._last_fused_lidar_points = np.empty((0, 3), dtype=np.float32)
        self._last_fused_obstacles_lidar = []
        self._lidar_bev_size = int(self.config.lidar_bev_size_px)
        self._lidar_bev_scale = float(self.config.lidar_bev_pixels_per_meter)
        self._lidar_bev_max_range = float(self.config.lidar_bev_max_range_m)

        robots = readRobots()
        name = f"QC2_{self.config.car_id}"
        self.car_config = robots[name]


        # Initialize components
        self._init_camera()
        self._init_yolo()
        self._init_lane_detector()
        self._init_lidar_fusion()
        self._init_publisher()

        # Pre-allocated buffers (set image_width for lane-side filtering)
        self.buffers = DetectionBuffers.from_config(
            image_width=config.image_width,
            image_height=config.image_height,
            offset_sign=(-1.0 if config.invert_lr else 1.0),
            sign_filter_side=config.sign_filter_side,
            config=config.to_buffer_runtime_config(),
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
        print(
            f"[SERVER] Offset sign={'inverted' if self.config.invert_lr else 'normal'}, sign-side={self.config.sign_filter_side}"
        )
        lane_status = "enabled" if self.config.enable_lane_detection else "disabled"
        lane_overlay_status = "enabled" if self.config.show_lane_overlay else "disabled"
        print(
            f"[SERVER] Lane detection: {lane_status}, lane overlay: {lane_overlay_status}"
        )
        if self.config.model_path:
            print(f"[SERVER] YOLO model override: {self.config.model_path}")
        if self.config.enable_lidar_fusion:
            lidar_status = "active" if self.lidar is not None else "FAILED"
            print(f"[SERVER] LiDAR fusion: {lidar_status}")
            if self.config.show_lidar_bev and self.lidar is not None:
                print("[LIDAR] BEV visualization enabled")
            print(
                "[LIDAR] Fusion gates: "
                f"{self.config.lidar_min_distance_m:.2f}m..{self.config.lidar_max_distance_m:.2f}m, "
                f"obstacle<= {self.config.obstacle_distance_threshold_m:.2f}m"
            )

    def _init_camera(self):
        """Initialize depth-aligned camera."""

        # try:
        #     self.camera = QCar2DepthAligned(video3dPort=str(car_config["video3dPort"]))
        #     print("[SERVER] Camera initialized (QCar2DepthAligned)")
        # except Exception as e:
        #     print(f"[SERVER] Data capture init failed: {e}")
        #     raise e

        self.camera = QCar2DepthAlignedCamera(
            imageWidth=self.config.image_width,
            imageHeight=self.config.image_height,
            use_intrinsics=self.config.camera.use_intrinsics,
            clipping_distance=self.config.camera.clipping_distance_m,
            video3dPort=self.car_config["video3dPort"],
            load_settings=self.config.camera.load_settings,
            use_fast_alignment=self.config.camera.use_fast_alignment,
        )

    def _init_yolo(self):
        """Initialize YOLO detector."""
        model_path = self.config.model_path if self.config.model_path else None
        self.yolo = YOLOv8Wrapper_Huy(
            imageHeight=self.config.image_height,
            imageWidth=self.config.image_width,
            modelPath=model_path,
            runtime_config=self.config.to_yolo_runtime_config(),
        )

    def _init_lane_detector(self):
        """Initialize lane detector if available."""
        self.lane_detector = None
        self.lane_enabled = False

        if not self.config.enable_lane_detection:
            print("[LANE] Disabled by config")
            self.yolo.set_lane_detector(None)
            self.yolo.set_lane_result(None)
            return

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

    def _init_lidar_fusion(self):
        """Initialize LiDAR fusion components if enabled."""
        self.lidar = None
        self.geometry_manager = None

        if not self.config.enable_lidar_fusion:
            return

        if not LIDAR_FUSION_AVAILABLE:
            print("[LIDAR] Fusion requested but modules not available")
            return

        try:
            # Initialize geometry manager from lidar_fusion.geometry config.
            self.geometry_manager = RobotGeometryManager(
                geometry_config=self.config.to_lidar_geometry_runtime_config()
            )
            # Align projection intrinsics with the actual aligned RGB stream.
            if hasattr(self, "camera"):
                fx = getattr(self.camera, "fx_rgb", None)
                fy = getattr(self.camera, "fy_rgb", None)
                cx = getattr(self.camera, "cx_rgb", None)
                cy = getattr(self.camera, "cy_rgb", None)
                if all(v is not None for v in (fx, fy, cx, cy)):
                    self.geometry_manager.fx = float(fx)
                    self.geometry_manager.fy = float(fy)
                    self.geometry_manager.cx = float(cx)
                    self.geometry_manager.cy = float(cy)
                    # Keep FOV values consistent with updated intrinsics.
                    self.geometry_manager.camera_fov_h = 2.0 * np.arctan(
                        self.config.image_width / (2.0 * self.geometry_manager.fx)
                    )
                    self.geometry_manager.camera_fov_v = 2.0 * np.arctan(
                        self.config.image_height / (2.0 * self.geometry_manager.fy)
                    )
                    print(
                        "[LIDAR] Using camera RGB intrinsics "
                        f"(fx={self.geometry_manager.fx:.1f}, fy={self.geometry_manager.fy:.1f}, "
                        f"cx={self.geometry_manager.cx:.1f}, cy={self.geometry_manager.cy:.1f})"
                    )

            # Initialize QCarLidar (same config as QCar2_lidar_point_cloud.py)
            self.lidar = QCarLidar(
                numMeasurements=self.config.lidar.sensor.num_measurements,
                rangingDistanceMode=self.config.lidar.sensor.ranging_distance_mode,
                interpolationMode=self.config.lidar.sensor.interpolation_mode,
                lidarPort=self.car_config["lidarPort"],
            )
            print("[LIDAR] LiDAR fusion initialized (QCarLidar + RobotGeometryManager)")
        except Exception as e:
            print(f"[LIDAR] LiDAR fusion initialization failed: {e}")
            self.lidar = None
            self.geometry_manager = None

    def _init_publisher(self):
        """Initialize YOLO data and video publishers."""
        # Data port comes from config ports.data_port_base + car_id.
        data_port = self.config.data_port
        self.publisher = YOLOPublisher(port=data_port)
        print(f"[SERVER] Data stream on {data_port}")

        self.video_publisher = None
        if self.config.probing:
            video_port = self.config.video_port
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

        # Keep full frame geometry for LiDAR-camera projection consistency.
        # Cropping + re-scaling changes effective intrinsics and causes vertical
        # projection drift (LiDAR points appear above/below detections).
        rgb = self.camera.rgb
        depth = self.camera.depth

        # Resize only when input shape differs from configured processing shape.
        if rgb.shape[1] != self.config.image_width or rgb.shape[0] != self.config.image_height:
            rgb = cv2.resize(rgb, (self.config.image_width, self.config.image_height))
        if depth.shape[1] != self.config.image_width or depth.shape[0] != self.config.image_height:
            depth = cv2.resize(depth, (self.config.image_width, self.config.image_height))

        # YOLO detection pipeline
        processed = self.yolo.pre_process(rgb)
        self.yolo.run_inference(inputImg=processed)
        results = self.yolo.post_process_detections(alignedDepth=depth)

        # =====================================================================
        # LiDAR Fusion (optional, obstacles only)
        # =====================================================================
        lidar_projected = None
        lidar_3d_cam = None
        fused_lidar_points = None
        fused_obstacles_lidar = None
        if self.config.enable_lidar_fusion and self.lidar is not None:
            (
                lidar_projected,
                lidar_3d_cam,
                fused_lidar_points,
                fused_obstacles_lidar,
            ) = self._lidar_fuse_obstacles(results)

        # Lane detection
        self.yolo.set_lane_result(None)
        lane_result = None
        if self.lane_enabled and self.lane_detector:
            lane_result = self.lane_detector.detect(rgb)
            self.yolo.set_lane_result(lane_result)

        # Build detection packet from current-frame results
        bboxes = None
        if (
            hasattr(self.yolo, "predictions")
            and self.yolo.predictions is not None
            and len(self.yolo.predictions) > 0
        ):
            bboxes = self.yolo.predictions[0].boxes.xyxy.cpu().numpy()
        else:
            bboxes = getattr(self.yolo, "bounding", None)

        self.buffers.fill_from_results(results, bounding_boxes=bboxes)
        self._apply_obstacle_distance_gate()
        # self.buffers.fill_lane(lane_result)

        # Rendering masks/overlays is expensive; skip it when not needed.
        need_render = (
            self.video_publisher is not None
            or self.config.show_image
            or bool(getattr(self.config, "show_obstacle_box", False))
            or self.config.show_lidar_overlay
        )
        annotated = None
        if need_render:
            annotated = self.yolo.post_process_render(
                showFPS=True,
                show_lane_overlay=(self.config.show_lane_overlay and self.lane_enabled),
            )

        # Draw LiDAR overlay on annotated image
        if (annotated is not None and self.config.show_lidar_overlay
                and lidar_projected is not None and len(lidar_projected) > 0):
            annotated = self.geometry_manager.draw_lidar_on_camera_image_bbox(
                annotated,
                lidar_projected,
                lidar_3d_cam,
                results,
                shrink_factor=self.config.lidar_overlay_bbox_shrink_factor,
            )

        # Draw obstacle box if config says so
        if annotated is not None and getattr(self.config, "show_obstacle_box", False):
            margin_x = (1.0 - self.buffers.center_box_width_ratio) / 2
            x_min = int(self.config.image_width * margin_x)
            x_max = int(self.config.image_width * (1.0 - margin_x))
            bottom_ignore_px = int(max(0, getattr(self.buffers, "bottom_ignore_px", 0)))
            center_box_raise_px = int(
                max(0, getattr(self.buffers, "center_box_raise_px", 0))
            )
            y_min = int(
                self.config.image_height * (1.0 - self.buffers.center_box_height_ratio)
            ) - center_box_raise_px
            y_max = int(self.config.image_height - bottom_ignore_px) - center_box_raise_px
            y_min = max(0, y_min)
            y_max = min(int(self.config.image_height), y_max)
            if y_max <= y_min:
                y_min = int(self.config.image_height * 0.5)
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

        # Final output crop for displayed/streamed annotated image.
        final_annotated = annotated
        crop_px = 50
        if annotated is not None and annotated.shape[0] > crop_px:
            final_annotated = annotated[:-crop_px, :]

        # Send video over ZMQ if probing is enabled
        if self.video_publisher and final_annotated is not None:
            self.video_publisher.send(final_annotated)

        has_local_window = False

        # Show image locally if enabled
        if self.config.show_image and final_annotated is not None:
            cv2.imshow("YOLO Server", final_annotated)
            has_local_window = True

        # Show lane debug if enabled
        if self.config.show_lane_debug and self.lane_enabled and self.lane_detector:
            if hasattr(self.lane_detector, "get_debug_image"):
                debug_img = self.lane_detector.get_debug_image()
                if debug_img is not None:
                    cv2.imshow("Lane Debug", debug_img)
                    has_local_window = True

        # Show LiDAR BEV if enabled
        if (
            self.config.show_lidar_bev
            and self.config.enable_lidar_fusion
            and self.lidar is not None
        ):
            bev = self._render_lidar_bev(
                fused_points_lidar=fused_lidar_points,
                fused_objects_lidar=fused_obstacles_lidar,
            )
            if bev is not None:
                cv2.imshow("LiDAR BEV Fusion", bev)
                has_local_window = True

        if has_local_window:
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                self.running = False
                return

        # Publish the already-filled payload
        self.publisher.send(self.buffers.to_packet())

    # =========================================================================
    # LiDAR Fusion helpers
    # =========================================================================
    # Only 3 obstacle categories are used for fusion and BEV labels.
    _PERSON_CLASS_IDS = {0}
    _CAR_CLASS_IDS = {1, 2, 3, 5, 7}
    _CONE_CLASS_IDS = {33}
    _PERSON_KEYWORDS = ("person", "pedestrian")
    _CAR_KEYWORDS = (
        "car",
        "qcar",
        "automobile",
        "sedan",
        "suv",
        "pickup",
        "vehicle",
        "truck",
        "bus",
        "van",
        "motorcycle",
        "bicycle",
    )
    _CONE_KEYWORDS = ("cone", "traffic cone", "pylon", "bollard", "fire hydrant")
    _NON_OBSTACLE_KEYWORDS = {
        "stop sign",
        "red",
        "green",
        "yellow",
        "idle",
        "yield",
        "traffic light",
    }

    def _get_detection_class_id(self, det) -> Optional[int]:
        """Extract class id from a detection object when present."""
        for attr in ("class_id", "cls", "classId"):
            if hasattr(det, attr):
                raw = getattr(det, attr)
                if raw is None:
                    continue
                try:
                    return int(raw)
                except (TypeError, ValueError):
                    continue
        return None

    def _get_obstacle_type(self, det) -> Optional[str]:
        """Map a detection to one of {car, person, cone} or None."""
        class_id = self._get_detection_class_id(det)
        if class_id is not None:
            if class_id in self._PERSON_CLASS_IDS:
                return "person"
            if class_id in self._CAR_CLASS_IDS:
                return "car"
            if class_id in self._CONE_CLASS_IDS:
                return "cone"
            # Explicit non-obstacle classes used in this stack.
            if class_id in {9, 10, 11}:
                return None

        name_l = str(getattr(det, "name", "")).lower()
        if any(kw in name_l for kw in self._NON_OBSTACLE_KEYWORDS):
            return None
        if any(kw in name_l for kw in self._PERSON_KEYWORDS):
            return "person"
        if any(kw in name_l for kw in self._CONE_KEYWORDS):
            return "cone"
        if any(kw in name_l for kw in self._CAR_KEYWORDS):
            return "car"
        return None

    def _is_obstacle_detection(self, det) -> bool:
        """Return True only for the 3 fusion obstacle categories."""
        return self._get_obstacle_type(det) is not None

    def _to_enhanced_obstacle(self, det, obstacle_type: str):
        """Convert a detection to EnhancedObstacle when available."""
        if EnhancedObstacle is None:
            det.name = obstacle_type
            det.obstacle_type = obstacle_type
            return det

        try:
            enhanced = EnhancedObstacle(
                name=obstacle_type,
                distance=float(getattr(det, "distance", 0.0)),
                conf=float(getattr(det, "conf", 0.0)),
                x=float(getattr(det, "x", 0.0)),
                y=float(getattr(det, "y", 0.0)),
            )
            for attr in (
                "bbox",
                "center_x",
                "center_y",
                "width",
                "height",
                "area",
                "class_id",
                "mask",
                "mask_area",
                "lidar_distance",
                "lidar_points_count",
            ):
                if hasattr(det, attr):
                    setattr(enhanced, attr, getattr(det, attr))
            enhanced.name = obstacle_type
            enhanced.obstacle_type = obstacle_type
            return enhanced
        except Exception:
            det.name = obstacle_type
            det.obstacle_type = obstacle_type
            return det

    def _get_bbox_xyxy(self, det):
        """Return detection bbox as (x1, y1, x2, y2) or None."""
        bbox = getattr(det, "bbox", None)
        if bbox is None or len(bbox) < 4:
            return None
        x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
        if x2 <= x1 or y2 <= y1:
            return None
        return x1, y1, x2, y2

    def _is_point_in_lower_bbox_band(self, det, u: float, v: float, lower_start_ratio: Optional[float] = None) -> bool:
        """Keep LiDAR association near lower object region (where LiDAR hits are expected)."""
        bbox = self._get_bbox_xyxy(det)
        if bbox is None:
            return True
        if lower_start_ratio is None:
            lower_start_ratio = float(self.config.lidar_lower_bbox_start_ratio)
        _, y1, _, y2 = bbox
        y_start = y1 + lower_start_ratio * (y2 - y1)
        return float(v) >= float(y_start)

    def _is_point_in_bottom_ignored_strip(self, v: float) -> bool:
        """Return True for points inside the configured bottom ignore strip."""
        bottom_ignore_px = int(max(0, getattr(self.buffers, "bottom_ignore_px", 0)))
        if bottom_ignore_px <= 0:
            return False
        return float(v) >= float(self.config.image_height - bottom_ignore_px)

    def _is_detection_in_bottom_ignored_strip(
        self, det, obstacle_type: Optional[str] = None
    ) -> bool:
        """Ignore hood/capo false positives near image bottom (car type only)."""
        if obstacle_type not in (None, "car"):
            return False
        bbox = self._get_bbox_xyxy(det)
        if bbox is None:
            return False
        _, y1, _, y2 = bbox
        cy = 0.5 * (y1 + y2)
        return self._is_point_in_bottom_ignored_strip(cy)

    def _is_valid_fusion_distance(self, dist) -> bool:
        """Check whether a distance is valid inside configured fusion range."""
        try:
            d = float(dist)
        except (TypeError, ValueError):
            return False
        return self.config.lidar_min_distance_m <= d <= self.config.lidar_max_distance_m

    def _apply_obstacle_distance_gate(self):
        """Only keep obstacle state when closest obstacle is within alert threshold."""
        if self.buffers.obstacle[0] <= 0:
            return
        try:
            closest_dist = float(self.buffers.obstacle[1])
        except (TypeError, ValueError):
            closest_dist = 0.0

        if (
            closest_dist <= 0.0
            or closest_dist > float(self.config.obstacle_distance_threshold_m)
        ):
            # Clear obstacle flag for far/invalid detections.
            self.buffers.obstacle.fill(0.0)

    def _extract_detection_mask(
        self, det_idx: int, apply_erosion: bool = True
    ) -> Optional[np.ndarray]:
        """Get one binary segmentation mask and optionally erode it."""
        if not hasattr(self.yolo, "predictions") or not self.yolo.predictions:
            return None

        pred = self.yolo.predictions[0]
        masks = getattr(pred, "masks", None)
        if masks is None or getattr(masks, "data", None) is None:
            return None
        if det_idx >= len(masks.data):
            return None

        mask_tensor = masks.data[det_idx]
        if hasattr(mask_tensor, "cpu"):
            mask_np = mask_tensor.cpu().numpy()
        elif hasattr(mask_tensor, "numpy"):
            mask_np = mask_tensor.numpy()
        else:
            mask_np = np.array(mask_tensor)

        if mask_np.ndim > 2:
            mask_np = np.squeeze(mask_np)

        target_shape = (self.config.image_height, self.config.image_width)
        if mask_np.shape != target_shape:
            mask_np = cv2.resize(
                mask_np.astype(np.float32),
                (self.config.image_width, self.config.image_height),
                interpolation=cv2.INTER_NEAREST,
            )

        mask_bin = (mask_np > 0.5).astype(np.uint8)
        bottom_ignore_px = int(max(0, getattr(self.buffers, "bottom_ignore_px", 0)))
        if bottom_ignore_px > 0 and bottom_ignore_px < mask_bin.shape[0]:
            mask_bin[-bottom_ignore_px:, :] = 0
        if not apply_erosion:
            return mask_bin

        kernel_size = int(max(1, self.config.lidar_mask_erode_kernel_size))
        iterations = int(max(1, self.config.lidar_mask_erode_iterations))
        if kernel_size > 1:
            for _ in range(iterations):
                mask_bin = erode_mask(mask_bin, kernel_size=kernel_size)
        return mask_bin

    def _enrich_detection_geometry(self, results):
        """Populate bbox/center/size fields using YOLO xyxy boxes."""
        bounding = None
        class_ids = None

        # Preferred source: current predictions (works for both predict/track).
        if (
            hasattr(self.yolo, "predictions")
            and self.yolo.predictions is not None
            and len(self.yolo.predictions) > 0
        ):
            pred_boxes = getattr(self.yolo.predictions[0], "boxes", None)
            if pred_boxes is not None and getattr(pred_boxes, "xyxy", None) is not None:
                try:
                    bounding = pred_boxes.xyxy.cpu().numpy()
                    if getattr(pred_boxes, "cls", None) is not None:
                        class_ids = pred_boxes.cls.cpu().numpy()
                except Exception:
                    bounding = None
                    class_ids = None

        # Fallback source used by some wrapper modes.
        if bounding is None:
            bounding = getattr(self.yolo, "bounding", None)

        if bounding is None:
            return

        for idx, det in enumerate(results):
            if idx >= len(bounding):
                break
            bbox = bounding[idx]
            if bbox is None or len(bbox) < 4:
                continue

            x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
            det.bbox = [x1, y1, x2, y2]
            det.center_x = 0.5 * (x1 + x2)
            det.center_y = 0.5 * (y1 + y2)
            det.width = max(0.0, x2 - x1)
            det.height = max(0.0, y2 - y1)
            if class_ids is not None and idx < len(class_ids):
                try:
                    det.class_id = int(class_ids[idx])
                except (TypeError, ValueError):
                    pass

    def _lidar_fuse_obstacles_legacy(self, results):
        """Read LiDAR, project to camera, match to obstacle detections, fuse distances.

        Returns:
            projected: Nx3 array of (u, v, depth) — projected LiDAR points
            lidar_3d_cam: Nx3 array of (x, y, z) in camera frame
        """
        try:
            self.lidar.read()
        except Exception as e:
            print(f"[LIDAR] Read failed: {e}")
            return None, None

        # Convert angles to body frame (same as QCar2_lidar_point_cloud.py)
        angles_body = self.lidar.angles * -1 + np.pi

        # Project LiDAR points to camera image coordinates
        projection_outputs = self.geometry_manager.lidar_to_camera_projection(
            angles_body,
            self.lidar.distances,
            self.config.image_width,
            self.config.image_height,
        )
        if len(projection_outputs) >= 3:
            projected, lidar_3d_cam, _ = projection_outputs
        else:
            projected, lidar_3d_cam = projection_outputs

        if len(projected) == 0:
            return projected, lidar_3d_cam

        # Ensure every result has a bbox attribute (base Obstacle objects may lack it)
        bounding = getattr(self.yolo, 'bounding', None)
        if bounding is not None:
            for idx, det in enumerate(results):
                if not hasattr(det, 'bbox') or det.bbox is None:
                    if idx < len(bounding):
                        det.bbox = bounding[idx]  # [x1, y1, x2, y2]

        # Filter to obstacle detections only
        obstacle_dets = [det for det in results if self._is_obstacle_detection(det)]
        if not obstacle_dets:
            return projected, lidar_3d_cam

        # Match LiDAR points to each obstacle bbox and fuse distances
        for det in obstacle_dets:
            matched_depths = []
            for u, v, depth in projected:
                if rect_contains_point(
                    det, (u, v),
                    self.config.image_width, self.config.image_height,
                    shrink_factor=float(self.config.lidar_overlay_bbox_shrink_factor),
                ):
                    matched_depths.append(depth)

            if matched_depths:
                lidar_dist = get_best_distance(
                    matched_depths,
                    technique=str(self.config.lidar_distance_selection_technique),
                )
                det.distance = fuse_distance_measurements(
                    lidar_distance=lidar_dist,
                    depth_distance=det.distance,
                )

        return projected, lidar_3d_cam

    def _lidar_fuse_obstacles(self, results):
        """Read LiDAR, project to camera, match to obstacle detections, fuse distances.

        Returns:
            projected: Nx3 array of (u, v, depth) projected LiDAR points
            lidar_3d_cam: Nx3 array of (x, y, z) in camera frame
            fused_lidar_points: Mx3 array of matched LiDAR points in LiDAR frame
            fused_objects_lidar: list of YOLO-fused objects in LiDAR coordinates
        """
        try:
            self.lidar.read()
        except Exception as e:
            print(f"[LIDAR] Read failed: {e}")
            return None, None, None, None

        angles_body = self.lidar.angles * -1 + np.pi
        self._last_lidar_angles_body = np.array(angles_body, copy=True)
        self._last_lidar_distances = np.array(self.lidar.distances, copy=True)
        self._last_fused_lidar_points = np.empty((0, 3), dtype=np.float32)
        self._last_fused_obstacles_lidar = []

        projection_outputs = self.geometry_manager.lidar_to_camera_projection(
            angles_body,
            self.lidar.distances,
            self.config.image_width,
            self.config.image_height,
        )
        if len(projection_outputs) >= 3:
            projected, lidar_3d_cam, lidar_3d_lidar = projection_outputs
        else:
            projected, lidar_3d_cam = projection_outputs
            lidar_3d_lidar = np.empty((0, 3), dtype=np.float32)

        if len(projected) == 0:
            return (
                projected,
                lidar_3d_cam,
                self._last_fused_lidar_points,
                self._last_fused_obstacles_lidar,
            )

        self._enrich_detection_geometry(results)

        obstacle_dets = []
        for idx, det in enumerate(results):
            obstacle_type = self._get_obstacle_type(det)
            if obstacle_type is not None:
                if self._is_detection_in_bottom_ignored_strip(det, obstacle_type):
                    continue
                obstacle_dets.append((idx, det, obstacle_type))
        if not obstacle_dets:
            return (
                projected,
                lidar_3d_cam,
                self._last_fused_lidar_points,
                self._last_fused_obstacles_lidar,
            )

        matched_point_indices = set()
        fused_obstacles = []
        min_d = float(self.config.lidar_min_distance_m)
        max_d = float(self.config.lidar_max_distance_m)

        for det_idx, det, obstacle_type in obstacle_dets:
            matched_depths = []
            local_point_indices = []

            # Segmentation-first association (preferred): eroded mask for robustness.
            det_mask = self._extract_detection_mask(det_idx, apply_erosion=True)
            has_mask_available = det_mask is not None
            has_segmentation_mask = has_mask_available and bool(np.any(det_mask))
            if has_segmentation_mask:
                det.mask = det_mask
                det.mask_area = int(np.count_nonzero(det_mask))
                for point_idx, (u, v, depth) in enumerate(projected):
                    if depth < min_d or depth > max_d:
                        continue
                    if self._is_point_in_bottom_ignored_strip(v):
                        continue
                    pu, pv = int(round(u)), int(round(v))
                    if 0 <= pv < det_mask.shape[0] and 0 <= pu < det_mask.shape[1]:
                        if det_mask[pv, pu] > 0:
                            matched_depths.append(float(depth))
                            local_point_indices.append(point_idx)

            # Retry without erosion if erosion removed too much mask area.
            if has_mask_available and not matched_depths:
                det_mask_raw = self._extract_detection_mask(det_idx, apply_erosion=False)
                if det_mask_raw is not None and bool(np.any(det_mask_raw)):
                    det.mask = det_mask_raw
                    det.mask_area = int(np.count_nonzero(det_mask_raw))
                    for point_idx, (u, v, depth) in enumerate(projected):
                        if depth < min_d or depth > max_d:
                            continue
                        if self._is_point_in_bottom_ignored_strip(v):
                            continue
                        pu, pv = int(round(u)), int(round(v))
                        if 0 <= pv < det_mask_raw.shape[0] and 0 <= pu < det_mask_raw.shape[1]:
                            if det_mask_raw[pv, pu] > 0:
                                matched_depths.append(float(depth))
                                local_point_indices.append(point_idx)

            # Only use bbox fallback when segmentation is unavailable.
            if not has_mask_available and not matched_depths:
                for point_idx, (u, v, depth) in enumerate(projected):
                    if depth < min_d or depth > max_d:
                        continue
                    if self._is_point_in_bottom_ignored_strip(v):
                        continue
                    if not self._is_point_in_lower_bbox_band(det, u, v):
                        continue
                    if rect_contains_point(
                        det,
                        (u, v),
                        self.config.image_width,
                        self.config.image_height,
                        shrink_factor=float(self.config.lidar_bbox_fallback_shrink_factor),
                        only_height=False,
                    ):
                        matched_depths.append(float(depth))
                        local_point_indices.append(point_idx)

            # Recovery path: if lower-band gating is too strict for this frame,
            # retry full bbox association.
            if (
                not has_mask_available
                and not matched_depths
                and bool(self.config.lidar_allow_upper_bbox_fallback)
            ):
                for point_idx, (u, v, depth) in enumerate(projected):
                    if depth < min_d or depth > max_d:
                        continue
                    if self._is_point_in_bottom_ignored_strip(v):
                        continue
                    if rect_contains_point(
                        det,
                        (u, v),
                        self.config.image_width,
                        self.config.image_height,
                        shrink_factor=float(self.config.lidar_bbox_fallback_shrink_factor),
                        only_height=False,
                    ):
                        matched_depths.append(float(depth))
                        local_point_indices.append(point_idx)

            if matched_depths:
                filtered_depths = filter_outlier_distances(
                    matched_depths,
                    method=str(self.config.lidar_outlier_filter_method),
                )
                if len(filtered_depths) == 0:
                    filtered_depths = np.array(matched_depths, dtype=np.float32)

                lidar_dist = get_best_distance(
                    filtered_depths,
                    technique=str(self.config.lidar_distance_selection_technique),
                )
                if lidar_dist is None:
                    continue
                lidar_dist = float(np.clip(float(lidar_dist), min_d, max_d))
                depth_dist = None
                try:
                    depth_candidate = float(getattr(det, "distance", 0.0))
                    if self._is_valid_fusion_distance(depth_candidate):
                        depth_dist = depth_candidate
                except (TypeError, ValueError):
                    depth_dist = None

                # Reject depth when it disagrees too much with LiDAR.
                if depth_dist is not None:
                    disagreement = abs(depth_dist - lidar_dist) / max(lidar_dist, 0.05)
                    if disagreement > float(self.config.max_depth_lidar_disagreement_ratio):
                        depth_dist = None

                # Robust rule:
                # - enough LiDAR points: trust LiDAR, optional light blend with depth
                # - too few LiDAR points: still prefer LiDAR but allow modest depth help
                if len(filtered_depths) >= int(self.config.lidar_min_points_for_trust):
                    lidar_w, depth_w = self.config.lidar_blend_weights_trust
                    final_dist = (
                        lidar_dist
                        if depth_dist is None
                        else (lidar_w * lidar_dist + depth_w * depth_dist)
                    )
                else:
                    lidar_w, depth_w = self.config.lidar_blend_weights_sparse
                    final_dist = (
                        lidar_dist
                        if depth_dist is None
                        else (lidar_w * lidar_dist + depth_w * depth_dist)
                    )

                det.distance = float(np.clip(float(final_dist), min_d, max_d))
                det.lidar_distance = float(lidar_dist) if lidar_dist is not None else 0.0
                det.lidar_points_count = int(len(filtered_depths))
                det.obstacle_type = obstacle_type
                fused_obstacles.append(
                    self._to_enhanced_obstacle(det, obstacle_type=obstacle_type)
                )
                matched_point_indices.update(local_point_indices)

        if matched_point_indices and len(lidar_3d_lidar) > 0:
            matched_indices = np.array(sorted(matched_point_indices), dtype=np.int32)
            self._last_fused_lidar_points = lidar_3d_lidar[matched_indices]

        if fused_obstacles:
            try:
                self._last_fused_obstacles_lidar = (
                    self.geometry_manager.project_semantic_objects_to_lidar(
                        fused_obstacles,
                        self.config.image_width,
                        self.config.image_height,
                    )
                )
            except Exception as e:
                print(f"[LIDAR] Failed to project fused objects to LiDAR frame: {e}")
                self._last_fused_obstacles_lidar = []
        else:
            self._last_fused_obstacles_lidar = []

        return (
            projected,
            lidar_3d_cam,
            self._last_fused_lidar_points,
            self._last_fused_obstacles_lidar,
        )

    def _render_lidar_bev(self, fused_points_lidar=None, fused_objects_lidar=None):
        """Render top-down LiDAR BEV highlighting YOLO-fused obstacle points."""
        if self._last_lidar_angles_body.size == 0 or self._last_lidar_distances.size == 0:
            return None

        if fused_points_lidar is not None:
            self._last_fused_lidar_points = fused_points_lidar
        if fused_objects_lidar is not None:
            self._last_fused_obstacles_lidar = fused_objects_lidar

        bev = np.zeros((self._lidar_bev_size, self._lidar_bev_size, 3), dtype=np.uint8)
        bev[:] = (20, 20, 20)
        center_x = self._lidar_bev_size // 2
        center_y = self._lidar_bev_size // 2

        for r in range(2, int(self._lidar_bev_max_range) + 1, 2):
            radius_px = int(r * self._lidar_bev_scale)
            if radius_px < center_x:
                cv2.circle(bev, (center_x, center_y), radius_px, (55, 55, 55), 1)
        cv2.line(bev, (center_x, 0), (center_x, self._lidar_bev_size), (45, 45, 45), 1)
        cv2.line(bev, (0, center_y), (self._lidar_bev_size, center_y), (45, 45, 45), 1)

        valid = (
            (self._last_lidar_distances > max(0.01, float(self.config.lidar_min_distance_m)))
            & (self._last_lidar_distances <= self._lidar_bev_max_range)
        )
        if np.any(valid):
            angles = self._last_lidar_angles_body[valid]
            dists = self._last_lidar_distances[valid]
            lidar_x = -dists * np.cos(angles)
            lidar_y = dists * np.sin(angles)

            px = np.round(center_x + lidar_y * self._lidar_bev_scale).astype(np.int32)
            py = np.round(center_y - (-lidar_x) * self._lidar_bev_scale).astype(np.int32)
            in_bounds = (
                (px >= 0)
                & (px < self._lidar_bev_size)
                & (py >= 0)
                & (py < self._lidar_bev_size)
            )
            bev[py[in_bounds], px[in_bounds]] = (95, 95, 95)

        if self._last_fused_lidar_points is not None and len(self._last_fused_lidar_points) > 0:
            for point in self._last_fused_lidar_points:
                lidar_x, lidar_y = float(point[0]), float(point[1])
                px = int(round(center_x + lidar_y * self._lidar_bev_scale))
                py = int(round(center_y - (-lidar_x) * self._lidar_bev_scale))
                if 0 <= px < self._lidar_bev_size and 0 <= py < self._lidar_bev_size:
                    cv2.circle(bev, (px, py), 2, (0, 220, 255), -1)

        if self._last_fused_obstacles_lidar:
            for det in self._last_fused_obstacles_lidar:
                lidar_x = det.get("lidar_x")
                lidar_y = det.get("lidar_y")
                if lidar_x is None or lidar_y is None:
                    continue

                px = int(round(center_x + float(lidar_y) * self._lidar_bev_scale))
                py = int(round(center_y - (-float(lidar_x)) * self._lidar_bev_scale))
                if 0 <= px < self._lidar_bev_size and 0 <= py < self._lidar_bev_size:
                    cv2.rectangle(bev, (px - 4, py - 4), (px + 4, py + 4), (0, 255, 0), -1)
                    label = f"{det.get('class_name', 'obj')} {det.get('lidar_dist', 0.0):.1f}m"
                    cv2.putText(
                        bev,
                        label,
                        (px + 6, max(15, py - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.35,
                        (0, 255, 0),
                        1,
                    )

        cv2.circle(bev, (center_x, center_y), 4, (255, 0, 0), -1)
        cv2.arrowedLine(
            bev,
            (center_x, center_y),
            (center_x, center_y - 20),
            (255, 0, 0),
            2,
            tipLength=0.3,
        )
        cv2.putText(
            bev,
            "LiDAR BEV (gray=all points, yellow=fused points, green=fused objects)",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (230, 230, 230),
            1,
        )
        cv2.putText(
            bev,
            f"fused_points={len(self._last_fused_lidar_points)} fused_objects={len(self._last_fused_obstacles_lidar)}",
            (10, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (230, 230, 230),
            1,
        )
        return bev

    def terminate(self):
        """Clean shutdown of all components."""
        if self._terminated:
            return
        self._terminated = True
        print("[SERVER] Terminating...")
        self.running = False

        if hasattr(self, "camera"):
            self.camera.terminate()

        if getattr(self, "lidar", None) is not None:
            try:
                self.lidar.terminate()
                print("[LIDAR] LiDAR terminated")
            except Exception as e:
                print(f"[LIDAR] Terminate error: {e}")

        if self.lane_detector:
            self.lane_detector.terminate()
            print("[LANE] Lane detector terminated")

        if hasattr(self, "video_publisher") and self.video_publisher is not None:
            self.video_publisher.terminate()

        if hasattr(self, "publisher") :
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
    args = _parse_cli_args()
    overrides = {}
    if args.caridx is not None:
        overrides["car_id"] = int(args.caridx)
    if args.probing is not None:
        probing_override = parse_bool_string(args.probing)
        if probing_override is not None:
            overrides["probing"] = probing_override
        else:
            print(
                f"[CONFIG] Invalid probing override '{args.probing}'. "
                "Expected True/False. Ignoring override."
            )
    if args.show_image:
        overrides["show_image"] = True

    config = YoloServerConfig.from_yaml(config_path=args.config, overrides=overrides)
    with YOLOServerVirtual(config) as server:
        server.run()


if __name__ == "__main__":
    main()
